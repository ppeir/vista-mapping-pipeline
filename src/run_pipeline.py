#!/usr/bin/env python3
"""
run_pipeline.py – End-to-end pipeline: SLAM → video → poses → 2D map → ZIP.

Orchestrates existing scripts without modifying them:
  1. process_svo.py   (subprocess)   → rtabmap.db, map.pgm/yaml, cloud.ply
  2. svo_export.py    (subprocess)   → <name>.mp4
  3. convert_poses.py (import+patch) → positions.json
  4. project_ply.py   (subprocess)   → map_manual.pgm / map_manual.yaml
  5. Interactive map choice
  6. ZIP assembly

Usage:
    python3 src/run_pipeline.py \
        --svo ./data/raw/outdoor_08.svo2 \
        --output ./data/outputs/outdoor_08 \
        --min-z 0.4 --max-z 1.2 \
        --render cloud --superpoint \
        --Grid/MapFrameProjection true \
        --Grid/NormalsSegmentation false \
        --Grid/MaxGroundHeight 0.4 \
        --Grid/MaxObstacleHeight 1.2

All unrecognised arguments are forwarded verbatim to process_svo.py.
"""

import argparse
import importlib.util
import subprocess
import sys
import threading
import time
import zipfile
from pathlib import Path

from rich.console import Group
from rich.live import Live
from rich.text import Text

SRC_DIR = Path(__file__).resolve().parent


def run(cmd: list[str], description: str) -> None:
    print(f"\n{'='*60}\n  {description}\n{'='*60}")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        print(f"\n[ERROR] Failed (exit {result.returncode}): {description}", file=sys.stderr)
        sys.exit(result.returncode)


def run_parallel(jobs: list[tuple[list[str], str]]) -> None:
    """Launch subprocesses in parallel; update output in-place using rich."""
    n = len(jobs)
    procs: list[subprocess.Popen] = []
    status_state: dict[int, str] = {i: "starting..." for i in range(n)}
    labels = [desc.split("→")[-1].strip()[:18] for _, desc in jobs]

    for cmd, description in jobs:
        print(f"\n{'='*60}\n  {description} [parallel]\n{'='*60}")
        procs.append(subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
        ))

    def reader(idx: int, pipe) -> None:
        buffer = bytearray()
        try:
            while True:
                char = pipe.read(1)
                if not char:
                    if buffer:
                        status_state[idx] = buffer.decode('utf-8', errors='replace').strip()
                    break
                if char in (b'\r', b'\n'):
                    if buffer:
                        status_state[idx] = buffer.decode('utf-8', errors='replace').strip()
                        buffer.clear()
                else:
                    buffer.extend(char)
        finally:
            pipe.close()

    threads = [
        threading.Thread(target=reader, args=(i, p.stdout), daemon=True)
        for i, p in enumerate(procs)
    ]
    for t in threads:
        t.start()

    def render_status() -> Group:
        return Group(*[
            Text(f"  [{labels[i]:<18}]  {status_state[i]}")
            for i in range(n)
        ])

    print()
    with Live(render_status(), refresh_per_second=10) as live:
        while any(p.poll() is None for p in procs):
            live.update(render_status())
            time.sleep(0.1)
        live.update(render_status())

    for t in threads:
        t.join()

    for i in range(n):
        rc = procs[i].returncode
        tag = "OK" if rc == 0 else f"FAILED exit={rc}"
        print(f"  [{labels[i]:<18}]  [{tag}]")

    failed = [
        (procs[i].returncode, desc)
        for i, (_, desc) in enumerate(jobs)
        if procs[i].returncode != 0
    ]
    if failed:
        for code, description in failed:
            print(f"\n[ERROR] Failed (exit {code}): {description}", file=sys.stderr)
        sys.exit(failed[0][0])


def load_convert_poses():
    """Import convert_poses.py as a module, patching INPUT_FILE/OUTPUT_FILE before calling process_poses()."""
    spec = importlib.util.spec_from_file_location("convert_poses", SRC_DIR / "convert_poses.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def ask_map_choice(rtabmap_pgm: Path, manual_pgm: Path) -> tuple[Path, Path]:
    rtabmap_ok = rtabmap_pgm.is_file()
    manual_ok = manual_pgm.is_file()
    print(f"\n{'='*60}\n  Map selection\n{'='*60}")
    print(f"  1) RTAB-Map built-in  (map.pgm)        {'[available]' if rtabmap_ok else '[MISSING]'}")
    print(f"  2) Manual projection  (map_manual.pgm) {'[available]' if manual_ok else '[MISSING]'}")
    while True:
        choice = input("  Choose [1/2]: ").strip()
        if choice == "1" and rtabmap_ok:
            return rtabmap_pgm, rtabmap_pgm.with_suffix(".yaml")
        if choice == "2" and manual_ok:
            return manual_pgm, manual_pgm.with_suffix(".yaml")
        print("  Invalid choice or file not available, try again.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Full pipeline: SLAM → video → poses → 2D map → ZIP.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--svo", required=True, help="Path to input SVO file.")
    parser.add_argument("--output", required=True,
                        help="Output directory (its name becomes the ZIP/video filename).")
    parser.add_argument("--min-z", type=float, required=True, dest="min_z",
                        help="Min Z (m) for manual 2D projection slice.")
    parser.add_argument("--max-z", type=float, required=True, dest="max_z",
                        help="Max Z (m) for manual 2D projection slice.")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="Pixel size (m) for manual projection.")
    parser.add_argument("--side", choices=["left", "right"], default="right",
                        help="Camera side for video export.")
    parser.add_argument("--skip-slam", action="store_true",
                        help="Skip SLAM step (rtabmap.db + cloud must already exist).")
    parser.add_argument("--skip-video", action="store_true",
                        help="Skip video export step (MP4 must already exist).")

    args, extra = parser.parse_known_args()  # extra → forwarded to process_svo.py

    svo_path = Path(args.svo).resolve()
    output_path = Path(args.output).resolve()
    output_name = output_path.name
    output_path.mkdir(parents=True, exist_ok=True)

    # Steps 1+2: SLAM and video export in parallel (both read SVO, write to different files)
    mp4_path = output_path / f"{output_name}.mp4"
    parallel_jobs = []
    if not args.skip_slam:
        parallel_jobs.append((
            [sys.executable, str(SRC_DIR / "process_svo.py"),
             "--svo", str(svo_path), "--output", str(output_path)] + extra,
            "Step 1/5 – SLAM  →  process_svo.py",
        ))
    else:
        print("\n[SKIP] SLAM step.")

    if not args.skip_video:
        parallel_jobs.append((
            [sys.executable, str(SRC_DIR / "svo_export.py"),
             "--mode", "0",
             "--input_svo_file", str(svo_path),
             "--output_file", str(mp4_path),
             "--side", args.side],
            "Step 2/5 – Video export  →  svo_export.py",
        ))
    else:
        print("\n[SKIP] Video export step.")

    if parallel_jobs:
        run_parallel(parallel_jobs)

    # Step 3: Convert poses
    print(f"\n{'='*60}\n  Step 3/5 – Pose conversion  →  convert_poses.py\n{'='*60}")
    localisation_json = output_path / "positions.json"
    cp = load_convert_poses()
    cp.INPUT_FILE = str(output_path / "rtabmap_poses.txt")
    cp.OUTPUT_FILE = str(localisation_json)
    cp.process_poses()
    print(f"[OK] {localisation_json}")

    # Step 4: Manual 2D projection
    manual_pgm = output_path / "map_manual.pgm"
    run(
        [sys.executable, str(SRC_DIR / "project_ply.py"),
         "--input", str(output_path / "rtabmap_cloud.ply"),
         "--output", str(manual_pgm),
         "--min_z", str(args.min_z),
         "--max_z", str(args.max_z),
         "--resolution", str(args.resolution)],
        "Step 4/5 – Manual 2D projection  →  project_ply.py",
    )

    # Step 5: Map choice + ZIP
    final_pgm, final_yaml = ask_map_choice(output_path / "map.pgm", manual_pgm)

    zip_path = output_path / f"{output_name}.zip"
    print(f"\n{'='*60}\n  Step 5/5 – Creating ZIP\n{'='*60}")
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.write(localisation_json, "positions.json")
        if mp4_path.is_file():
            zf.write(mp4_path, f"{output_name}.mp4")
        else:
            print(f"[WARN] MP4 not found, skipped: {mp4_path}", file=sys.stderr)
        zf.write(final_pgm, "map.pgm")
        zf.write(final_yaml, "map.yaml")

    print(f"\n[DONE] {zip_path}")
    print(f"       positions.json | {output_name}.mp4 | map.pgm | map.yaml")


if __name__ == "__main__":
    main()