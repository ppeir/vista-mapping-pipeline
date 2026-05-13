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


try:
    import yaml as _yaml
except ImportError:
    _yaml = None

SRC_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = REPO_ROOT / "config"


def run(cmd: list[str], description: str) -> None:
    print(f"\n{'='*60}\n  {description}\n{'='*60}")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        print(f"\n[ERROR] Failed (exit {result.returncode}): {description}", file=sys.stderr)
        sys.exit(result.returncode)


def run_parallel(jobs: list[tuple[list[str], str]]) -> None:
    """Launch subprocesses in parallel; stream each output line prefixed with [label]."""
    labels = [desc.split("→")[-1].strip() for _, desc in jobs]
    procs: list[subprocess.Popen] = []

    for cmd, description in jobs:
        print(f"\n{'='*60}\n  {description} [parallel]\n{'='*60}", flush=True)
        procs.append(subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
        ))

    def reader(label: str, pipe) -> None:
        buf = bytearray()
        try:
            while True:
                char = pipe.read(1)
                if not char:
                    if buf:
                        text = buf.decode("utf-8", errors="replace").strip()
                        if text:
                            print(f"[{label}] {text}", flush=True)
                    break
                if char in (b"\n", b"\r"):
                    text = buf.decode("utf-8", errors="replace").strip()
                    if text:
                        print(f"[{label}] {text}", flush=True)
                    buf.clear()
                else:
                    buf.extend(char)
        finally:
            pipe.close()

    threads = [
        threading.Thread(target=reader, args=(labels[i], procs[i].stdout), daemon=True)
        for i in range(len(procs))
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    for i, (_, desc) in enumerate(jobs):
        rc = procs[i].returncode
        tag = "OK" if rc == 0 else f"FAILED exit={rc}"
        print(f"  [{labels[i]}]  [{tag}]", flush=True)

    failed = [
        (procs[i].returncode, desc)
        for i, (_, desc) in enumerate(jobs)
        if procs[i].returncode != 0
    ]
    if failed:
        for code, description in failed:
            print(f"\n[ERROR] Failed (exit {code}): {description}", file=sys.stderr, flush=True)
        sys.exit(failed[0][0])


def load_convert_poses():
    """Import convert_poses.py as a module, patching INPUT_FILE/OUTPUT_FILE before calling process_poses()."""
    spec = importlib.util.spec_from_file_location("convert_poses", SRC_DIR / "convert_poses.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod



# ── Config / YAML helpers ──────────────────────────────────────────────────

# Maps YAML key → (CLI flag, is_boolean_flag)
# These are process_svo.py args forwarded as CLI extras (not parsed by run_pipeline).
_EXTRA_PARAM_MAP: dict[str, tuple[str, bool]] = {
    "render":           ("--render",           False),
    "superpoint":       ("--superpoint",       True),
    "superpoint_model": ("--superpoint-model", False),
    "quality":          ("--quality",          False),
    "trim_start":       ("--trim-start",       False),
    "trim_end":         ("--trim-end",         False),
    "regen_grid":       ("--regen-grid",       True),
}

_PIPELINE_KEYS = {
    "svo", "output", "min_z", "max_z", "resolution", "side",
    "depth", "depth_scale", "depth_compression", "skip_slam", "skip_video",
}


def _require_yaml() -> None:
    if _yaml is None:
        print("[ERROR] PyYAML not installed. Run: pip install pyyaml", file=sys.stderr)
        sys.exit(1)


def load_config(config_path: Path) -> tuple[dict, dict, dict]:
    """Load a scene YAML and optional preset, return merged (pipeline, rtabmap, extra_params)."""
    _require_yaml()
    with open(config_path) as f:
        scene: dict = _yaml.safe_load(f) or {}

    base: dict = {}
    if "preset" in scene:
        preset_path = CONFIG_DIR / "presets" / f"{scene['preset']}.yaml"
        if not preset_path.is_file():
            print(f"[ERROR] Preset not found: {preset_path}", file=sys.stderr)
            sys.exit(1)
        with open(preset_path) as f:
            base = _yaml.safe_load(f) or {}

    # Merge: preset ← scene (scene overrides, except the 'preset' key itself)
    merged = {**base, **{k: v for k, v in scene.items() if k != "preset"}}
    # Merge rtabmap sub-dicts independently so scene only overrides its own keys
    rtabmap: dict = {**base.get("rtabmap", {}), **scene.get("rtabmap", {})}

    pipeline = {k: v for k, v in merged.items() if k in _PIPELINE_KEYS}
    extra_params = {k: v for k, v in merged.items() if k in _EXTRA_PARAM_MAP}
    return pipeline, rtabmap, extra_params


def _rtabmap_str(v) -> str:
    """Normalise a YAML value for RTAB-Map (Python booleans → lowercase string)."""
    if isinstance(v, bool):
        return "true" if v else "false"
    return str(v)


def yaml_extras_to_list(extra_params: dict, rtabmap: dict) -> list[str]:
    """Convert YAML extra_params + rtabmap dicts to a flat CLI arg list."""
    result: list[str] = []
    for yaml_key, (cli_flag, is_bool) in _EXTRA_PARAM_MAP.items():
        if yaml_key not in extra_params:
            continue
        val = extra_params[yaml_key]
        if is_bool:
            if val:
                result.append(cli_flag)
        else:
            result.extend([cli_flag, str(val)])
    for key, val in rtabmap.items():
        result.extend([f"--{key}", _rtabmap_str(val)])
    return result


def _parse_extra_to_dict(extra: list[str]) -> dict[str, str | None]:
    """Parse ['--Foo/Bar', '1.0', '--flag'] → {'Foo/Bar': '1.0', 'flag': None}."""
    d: dict[str, str | None] = {}
    i = 0
    while i < len(extra):
        key = extra[i].lstrip("-")
        i += 1
        if i < len(extra) and not extra[i].startswith("-"):
            d[key] = extra[i]
            i += 1
        else:
            d[key] = None
    return d


def merge_extras(yaml_extra: list[str], cli_extra: list[str]) -> list[str]:
    """Merge two extra-arg lists; CLI entries take priority per key."""
    merged = {**_parse_extra_to_dict(yaml_extra), **_parse_extra_to_dict(cli_extra)}
    result: list[str] = []
    for k, v in merged.items():
        result.append(f"--{k}")
        if v is not None:
            result.append(v)
    return result


# ── Interactive map selection ──────────────────────────────────────────────

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
    parser.add_argument("--config", metavar="YAML",
                        help="Scene YAML config file (can reference a preset). "
                             "CLI args override values from the config.")
    parser.add_argument("--svo", default=None, help="Path to input SVO file.")
    parser.add_argument("--output", default=None,
                        help="Output directory (its name becomes the ZIP/video filename).")
    parser.add_argument("--min-z", type=float, default=None, dest="min_z",
                        help="Min Z (m) for manual 2D projection slice.")
    parser.add_argument("--max-z", type=float, default=None, dest="max_z",
                        help="Max Z (m) for manual 2D projection slice.")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="Pixel size (m) for manual projection.")
    parser.add_argument("--side", choices=["left", "right"], default="right",
                        help="Camera side for video export.")
    parser.add_argument("--skip-slam", action="store_true",
                        help="Skip SLAM step (rtabmap.db + cloud must already exist).")
    parser.add_argument("--skip-video", action="store_true",
                        help="Skip video export step (MP4 must already exist).")
    parser.add_argument("--depth", action="store_true", default=True,
                        help="Export depth PNG sequence alongside video (mode 5, single SVO pass). (default: enabled)")
    parser.add_argument("--no-depth", action="store_false", dest="depth",
                        help="Disable depth PNG export.")
    parser.add_argument("--depth-scale", type=float, default=1.0, dest="depth_scale",
                        help="Scale factor for depth images (e.g. 0.5 = half resolution).")
    parser.add_argument("--depth-compression", type=int, default=5, dest="depth_compression",
                        metavar="[0-9]",
                        help="PNG compression level for depth images (0=none, 9=max, default: 5).")
    parser.add_argument("--map-choice", type=int, choices=[1, 2], default=None, dest="map_choice",
                        help="Non-interactive map selection: 1=RTAB-Map built-in, 2=manual projection. "
                             "If omitted, an interactive prompt is shown.")

    args, cli_extra = parser.parse_known_args()  # cli_extra → forwarded to process_svo.py

    # ── Load YAML config (if provided) and merge with CLI ─────────────────
    yaml_extra: list[str] = []
    if args.config:
        config_path = Path(args.config)
        if not config_path.is_file():
            parser.error(f"Config file not found: {config_path}")
        pipeline_defaults, rtabmap_cfg, extra_params_cfg = load_config(config_path)
        # Inject YAML values as argparse defaults (CLI args still take priority)
        parser.set_defaults(**pipeline_defaults)
        args, cli_extra = parser.parse_known_args()
        yaml_extra = yaml_extras_to_list(extra_params_cfg, rtabmap_cfg)

    # CLI extras override YAML extras per key
    extra = merge_extras(yaml_extra, cli_extra)

    # ── Validate required fields ───────────────────────────────────────────
    if args.svo is None:
        parser.error("--svo is required (pass it via CLI or --config)")
    if args.output is None:
        parser.error("--output is required (pass it via CLI or --config)")
    if args.min_z is None:
        parser.error("--min-z is required (pass it via CLI or --config)")
    if args.max_z is None:
        parser.error("--max-z is required (pass it via CLI or --config)")

    svo_path = Path(args.svo).resolve()
    output_path = Path(args.output).resolve()
    output_name = output_path.name
    output_path.mkdir(parents=True, exist_ok=True)

    # Steps 1+2: SLAM and video export in parallel (both read SVO, write to different files)
    mp4_path = output_path / f"{output_name}.mp4"
    # Step 1/6: Camera intrinsics (fast, sequential, before parallel jobs)
    run(
        [
            sys.executable, str(SRC_DIR / "zed_camera_info.py"),
            "--svo", str(svo_path),
            "--output-dir", str(output_path),
            "--depth-scale", str(args.depth_scale),
        ],
        "Step 1/6 – Camera intrinsics  →  zed_camera_info.py",
    )

    # Steps 1+2: SLAM and video export in parallel (both read SVO, write to different files)
    parallel_jobs = []
    if not args.skip_slam:
        parallel_jobs.append((
            [sys.executable, str(SRC_DIR / "process_svo.py"),
             "--svo", str(svo_path), "--output", str(output_path)] + extra,
            "Step 2/6 – SLAM  →  process_svo.py",
        ))
    else:
        print("\n[SKIP] SLAM step.")

    if not args.skip_video:
        depth_dir = output_path / "depth" if args.depth else None
        svo_export_cmd = [
            sys.executable, str(SRC_DIR / "svo_export.py"),
            "--input_svo_file", str(svo_path),
            "--output_file", str(mp4_path),
            "--side", args.side,
        ]
        if args.depth and depth_dir is not None:
            svo_export_cmd += [
                "--mode", "5",
                "--output_path_dir", str(depth_dir),
                "--depth-scale", str(args.depth_scale),
                "--depth-compression", str(args.depth_compression),
            ]
        else:
            svo_export_cmd += ["--mode", "0"]
        parallel_jobs.append((
            svo_export_cmd,
            "Step 3/6 – Video export  →  svo_export.py",
        ))
    else:
        depth_dir = None
        print("\n[SKIP] Video export step.")

    if parallel_jobs:
        run_parallel(parallel_jobs)

    # Step 3: Convert poses
    print(f"\n{'='*60}\n  Step 4/6 – Pose conversion  →  convert_poses.py\n{'='*60}")
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
        "Step 5/6 – Manual 2D projection  →  project_ply.py",
    )

    # Step 5: Map choice + ZIP
    if args.map_choice is not None:
        if args.map_choice == 1:
            final_pgm = output_path / "map.pgm"
            if not final_pgm.is_file():
                print(f"[ERROR] map.pgm not found: {final_pgm}", file=sys.stderr)
                sys.exit(1)
            final_yaml = final_pgm.with_suffix(".yaml")
        else:
            if not manual_pgm.is_file():
                print(f"[ERROR] map_manual.pgm not found: {manual_pgm}", file=sys.stderr)
                sys.exit(1)
            final_pgm = manual_pgm
            final_yaml = manual_pgm.with_suffix(".yaml")
    else:
        final_pgm, final_yaml = ask_map_choice(output_path / "map.pgm", manual_pgm)

    zip_path = output_path / f"{output_name}.zip"
    print(f"\n{'='*60}\n  Step 6/6 – Creating ZIP\n{'='*60}")
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.write(localisation_json, "positions.json")
        if mp4_path.is_file():
            zf.write(mp4_path, f"{output_name}.mp4")
        else:
            print(f"[WARN] MP4 not found, skipped: {mp4_path}", file=sys.stderr)
        for json_name in ("camera_info.json", "depth_camera_info.json"):
            json_path = output_path / json_name
            if json_path.is_file():
                zf.write(json_path, json_name)
            else:
                print(f"[WARN] {json_name} not found, skipped.", file=sys.stderr)
        if depth_dir is not None and depth_dir.is_dir():
            depth_files = sorted(depth_dir.iterdir())
            for f in depth_files:
                zf.write(f, f"depth/{f.name}")
            print(f"  depth/  ({len(depth_files)} files)")
        zf.write(final_pgm, "map.pgm")
        zf.write(final_yaml, "map.yaml")

    print(f"\n[DONE] {zip_path}")
    print(f"       positions.json | {output_name}.mp4 | map.pgm | map.yaml | camera_info.json | depth_camera_info.json | depth/*.png")


if __name__ == "__main__":
    main()