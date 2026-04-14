#!/usr/bin/env python3
"""
process_svo.py – Offline SLAM orchestrator (Global Bundle Adjustment)
======================================================================
Mounts a directory containing a ZED SVO file into the rtabmap-standalone
Docker container and runs:
  1. rtabmap-zed_svo → opens SVO via ZED SDK, runs RGBD-SLAM → rtabmap.db
  2. rtabmap-export  → extracts map.pgm + map.yaml from the database

No ROS dependency.  Requires:
  - Docker Engine with NVIDIA Container Toolkit (sudo docker run --gpus all)
  - The Docker image built from Dockerfile.rtabmap_standalone

Usage:
    python3 process_svo.py [OPTIONS]

    python3 process_svo.py \
        --svo    /path/to/data/bureau_complet.svo \
        --output /path/to/output \
        --image  rtabmap_standalone:latest
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration defaults
# ---------------------------------------------------------------------------

DEFAULT_IMAGE = "rtabmap_standalone:latest"

# RTAB-Map core parameters for offline SVO processing
# Key references:
#   https://github.com/introlab/rtabmap/wiki/Appendix
#   https://github.com/introlab/rtabmap/issues (ZED standalone examples)
#
# Parameter rationale:
#   Odom/Strategy 0              → Frame-to-Map (F2M) visual odometry
#   RGBD/CreateOccupancyGrid true → build 2-D occupancy grid during mapping
#   Grid/3D false                → project 3-D obstacles into 2-D (saves memory)
#   Grid/RayTracing true         → cleaner free-space estimation
#   Rtabmap/TimeThr 0            → no time limit per iteration (offline batch)
#   Mem/STMSize 30               → Short-term memory size (nodes in RAM)
#   Mem/InitWMWithAllNodes true  → load all nodes before graph optimisation
#   Optimizer/Strategy 0         → TORO (g2o unavailable on ARM64)
#   Rtabmap/DetectionRate 1      → attempt loop closure every frame
RTABMAP_PARAMS = [
    "--Odom/Strategy", "0",
    "--RGBD/CreateOccupancyGrid", "true",
    "--Grid/3D", "false",
    "--Grid/RayTracing", "true",
    "--Grid/CellSize", "0.05",
    "--Grid/ClusterRadius", "0.1",
    "--Rtabmap/TimeThr", "0",
    "--Rtabmap/DetectionRate", "1",
    "--Mem/STMSize", "30",
    "--Mem/InitWMWithAllNodes", "true",
    "--Optimizer/Strategy", "0",    # 0=TORO (built-in); g2o unavailable on Ubuntu 22.04 ARM64
    "--OdomF2M/MaxSize", "1000",
]

# rtabmap-export render modes
RENDER_MODES = {
    "cloud": [],                                          # point cloud PLY
    "mesh":  ["--mesh"],                                   # triangulated mesh PLY
    "texture": ["--texture", "--texture_size", "4096"],    # textured mesh OBJ
}
DEFAULT_RENDER = "mesh"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def run(cmd: list[str], description: str) -> None:
    """Run a shell command and raise on failure."""
    print(f"\n{'='*60}")
    print(f"  {description}")
    print(f"{'='*60}")
    print("  CMD:", " ".join(cmd))
    print()
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        print(f"\n[ERROR] Command failed (exit code {result.returncode}): {description}",
              file=sys.stderr)
        sys.exit(result.returncode)


def check_docker() -> None:
    """Verify that Docker is accessible."""
    result = subprocess.run(["docker", "info"], capture_output=True)
    if result.returncode != 0:
        print("[ERROR] Docker is not running or not accessible.", file=sys.stderr)
        print("        Try:  sudo systemctl start docker", file=sys.stderr)
        sys.exit(1)


def image_exists(image: str) -> bool:
    """Return True if the Docker image is present locally."""
    result = subprocess.run(
        ["docker", "image", "inspect", image],
        capture_output=True,
    )
    return result.returncode == 0


# ---------------------------------------------------------------------------
# Main logic
# ---------------------------------------------------------------------------

def build_rtabmap_cmd(svo_filename: str, output_dir_in_container: str,
                      trim_start: float = 0.0, trim_end: float = 0.0) -> list[str]:
    """
    Build the rtabmap-zed_svo CLI command for offline RGBD-SLAM.

    rtabmap-zed_svo uses CameraStereoZed to open the SVO natively
    through the ZED SDK, then runs F2M visual odometry + SLAM.
    Output: <output_dir>/rtabmap.db
    """
    cmd = (
        ["rtabmap-zed_svo"]
        + ["--output", output_dir_in_container]
        + ["--output_name", "rtabmap"]
        + ["--quality", "1"]     # 0=NONE, 1=PERFORMANCE, 2=QUALITY, 3=NEURAL
        + RTABMAP_PARAMS
    )
    if trim_start > 0:
        cmd += ["--trim-start", str(trim_start)]
    if trim_end > 0:
        cmd += ["--trim-end", str(trim_end)]
    cmd += [f"/data/{svo_filename}"]
    return cmd


def build_export_cmd(output_dir_in_container: str, render: str) -> list[str]:
    """Build the rtabmap-export command with the chosen render mode."""
    db_path = f"{output_dir_in_container}/rtabmap.db"
    return (
        ["rtabmap-export"]
        + RENDER_MODES[render]
        + ["--output-dir", output_dir_in_container]
        + [db_path]
    )


def run_step(
    image: str,
    data_host: str,
    output_host: str,
    cmd_in_container: list[str],
    description: str,
    extra_docker_args: list[str] | None = None,
) -> None:
    """
    Start a Docker container, run one command inside it, then remove it.

    Volume mounts:
      /data   ← read-only  host data directory (contains the SVO file)
      /output ← read-write host output directory (receives rtabmap.db / map files)
    """
    docker_cmd = [
        "docker", "run",
        "--rm",                      # auto-remove container after exit
        "--runtime=nvidia",          # L4T/Tegra: NVIDIA runtime (required for ZED daemon low-level access)
        "--gpus", "all",             # NVIDIA GPU access (required by ZED SDK)
        "--privileged",              # USB3/ZED hardware access (needed even for SVO playback)
        "-e", "QT_QPA_PLATFORM=offscreen",  # headless: no X display needed
        "-e", "ZED_SDK_VERBOSE=1",           # show ZED SDK init / TensorRT progress
        "-v", f"{data_host}:/data:ro",
        "-v", f"{output_host}:/output:rw",
        # ZED SDK performs a licence/connectivity check at init; --network none
        # causes a silent 2-minute timeout, so we allow host networking.
        # Mount the host ZED resources directory so the container reuses the
        # TensorRT engines already compiled on the Jetson (avoids ~10 min recompile).
        # The host path /usr/local/zed/resources contains the pre-optimised engines.
        "-v", "/usr/local/zed/resources:/usr/local/zed/resources:rw",
    ]

    if extra_docker_args:
        docker_cmd.extend(extra_docker_args)

    docker_cmd.append(image)
    docker_cmd.extend(cmd_in_container)

    run(docker_cmd, description)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Offline SLAM: process a ZED SVO file with RTAB-Map standalone.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--svo",
        required=True,
        help="Absolute path to the input SVO file (e.g. /mnt/data/bureau_complet.svo).",
    )
    parser.add_argument(
        "--output",
        default=str(Path.cwd() / "rtabmap_output"),
        help="Host directory where rtabmap.db, map.pgm and map.yaml will be written.",
    )
    parser.add_argument(
        "--image",
        default=DEFAULT_IMAGE,
        help="Docker image name:tag to use.",
    )
    parser.add_argument(
        "--render",
        choices=list(RENDER_MODES.keys()),
        default=DEFAULT_RENDER,
        help="3-D export type: cloud (point cloud PLY), mesh (triangulated PLY), texture (textured OBJ).",
    )
    parser.add_argument(
        "--skip-slam",
        action="store_true",
        help="Skip the SLAM step and only run the map export (rtabmap.db must exist).",
    )
    parser.add_argument(
        "--skip-export",
        action="store_true",
        help="Skip the map export step (only produce rtabmap.db).",
    )
    parser.add_argument(
        "--trim-start",
        type=float,
        default=0.0,
        help="Trim the first N seconds from the SVO before SLAM (default: 0).",
    )
    parser.add_argument(
        "--trim-end",
        type=float,
        default=0.0,
        help="Trim the last N seconds from the SVO before SLAM (default: 0).",
    )
    args = parser.parse_args()

    # ---- Validate inputs -----------------------------------------------
    svo_path = Path(args.svo).resolve()
    if not svo_path.is_file():
        print(f"[ERROR] SVO file not found: {svo_path}", file=sys.stderr)
        sys.exit(1)

    data_host = str(svo_path.parent)
    svo_filename = svo_path.name

    output_host = str(Path(args.output).resolve())
    os.makedirs(output_host, exist_ok=True)

    # ---- Pre-flight checks ---------------------------------------------
    check_docker()

    if not image_exists(args.image):
        print(f"[ERROR] Docker image '{args.image}' not found locally.", file=sys.stderr)
        print(f"        Build it first:\n"
              f"          docker build -f Dockerfile.rtabmap_standalone "
              f"-t {args.image} .",
              file=sys.stderr)
        sys.exit(1)

    print(f"\n[INFO] SVO input  : {svo_path}")
    print(f"[INFO] Output dir : {output_host}")
    print(f"[INFO] Render mode: {args.render}")
    print(f"[INFO] Docker image: {args.image}")
    if args.trim_start > 0 or args.trim_end > 0:
        print(f"[INFO] Trim start : {args.trim_start}s")
        print(f"[INFO] Trim end   : {args.trim_end}s")

    # ---- Step 1: SLAM --------------------------------------------------
    if not args.skip_slam:
        rtabmap_cmd = build_rtabmap_cmd(svo_filename, "/output",
                                         args.trim_start, args.trim_end)
        run_step(
            image=args.image,
            data_host=data_host,
            output_host=output_host,
            cmd_in_container=rtabmap_cmd,
            description="Step 1/2 – rtabmap-zed_svo: RGBD-SLAM (ZED SDK + F2M odom)",
        )
        db_path = Path(output_host) / "rtabmap.db"
        if not db_path.is_file():
            print(f"\n[ERROR] rtabmap.db was not produced in {output_host}.",
                  file=sys.stderr)
            sys.exit(1)
        print(f"\n[OK] rtabmap.db created: {db_path}")
        for fname in ("map.pgm", "map.yaml"):
            p = Path(output_host) / fname
            if p.is_file():
                print(f"[OK] {p}")
            else:
                print(f"[WARN] {fname} not produced – grid data may be missing.",
                      file=sys.stderr)
    else:
        print("\n[SKIP] Skipping SLAM step (--skip-slam).")

    # ---- Step 2: Export 2-D occupancy map ------------------------------
    if not args.skip_export:
        export_cmd = build_export_cmd("/output", args.render)
        run_step(
            image=args.image,
            data_host=data_host,
            output_host=output_host,
            cmd_in_container=export_cmd,
            description=f"Step 2/2 – rtabmap-export: {args.render} export",
        )
    else:
        print("\n[SKIP] Skipping export step (--skip-export).")

    print("\n[DONE] Offline SLAM pipeline complete.")
    print(f"       Results in: {output_host}")


if __name__ == "__main__":
    main()
