#!/usr/bin/env python3
"""
run_pipeline.py – End-to-end pipeline: SLAM → video → poses → 2D map → ZIP.

Orchestrates existing scripts without modifying them:
  1. process_svo.py   (subprocess)   → rtabmap.db, map.pgm/yaml, cloud.ply
  2. svo_export.py    (subprocess)   → <name>.mp4
  3. convert_poses.py (import+patch) → localisation.json
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
import zipfile
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parent


def run(cmd: list[str], description: str) -> None:
    print(f"\n{'='*60}\n  {description}\n{'='*60}")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        print(f"\n[ERROR] Failed (exit {result.returncode}): {description}", file=sys.stderr)
        sys.exit(result.returncode)


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

    # Step 1: SLAM
    if not args.skip_slam:
        run(
            [sys.executable, str(SRC_DIR / "process_svo.py"),
             "--svo", str(svo_path), "--output", str(output_path)] + extra,
            "Step 1/5 – SLAM  →  process_svo.py",
        )
    else:
        print("\n[SKIP] SLAM step.")

    # Step 2: Video export
    mp4_path = output_path / f"{output_name}.mp4"
    if not args.skip_video:
        run(
            [sys.executable, str(SRC_DIR / "svo_export.py"),
             "--mode", "0",
             "--input_svo_file", str(svo_path),
             "--output_file", str(mp4_path),
             "--side", args.side],
            "Step 2/5 – Video export  →  svo_export.py",
        )
    else:
        print("\n[SKIP] Video export step.")

    # Step 3: Convert poses
    print(f"\n{'='*60}\n  Step 3/5 – Pose conversion  →  convert_poses.py\n{'='*60}")
    localisation_json = output_path / "localisation.json"
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
        zf.write(localisation_json, "localisation.json")
        if mp4_path.is_file():
            zf.write(mp4_path, f"{output_name}.mp4")
        else:
            print(f"[WARN] MP4 not found, skipped: {mp4_path}", file=sys.stderr)
        zf.write(final_pgm, "map.pgm")
        zf.write(final_yaml, "map.yaml")

    print(f"\n[DONE] {zip_path}")
    print(f"       localisation.json | {output_name}.mp4 | map.pgm | map.yaml")


if __name__ == "__main__":
    main()