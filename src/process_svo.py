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
#   Rtabmap/DetectionRate 1      → attempt loop closure 1 time per second
RTABMAP_PARAMS = [
    "--Odom/Strategy", "1",
    "--RGBD/CreateOccupancyGrid", "true",
    "--Grid/3D", "false",
    "--Grid/RayTracing", "false",       # OctoMap not compiled; true triggers a warning
    "--Grid/CellSize", "0.05",
    "--Grid/ClusterRadius", "0.1",
    "--Grid/MaxObstacleHeight", "2.0",  # ignore points above 2 m (ceiling / high reflections)
    "--Grid/MinClusterSize", "15",      # discard isolated noise clusters < n cells
    "--Rtabmap/TimeThr", "0",
    "--Rtabmap/DetectionRate", "1",
    "--Mem/STMSize", "30",
    "--Mem/InitWMWithAllNodes", "true",
    "--Optimizer/Strategy", "2",        # 2=GTSAM; override with 0=TORO if GTSAM unavailable
    "--Optimizer/GravitySigma", "0.3",  # gravity constraints for VIO odometry (set 0 to disable)
    "--OdomF2M/MaxSize", "1000",
    "--Vis/MaxDepth", "4.0",            # ignore features beyond 4 m
]

# SuperPoint extractor + SuperGlue matcher (enabled with --superpoint)
# Parameter names verified against RTAB-Map 0.21.4 Parameters.h:
#   SuperPoint = Kp/DetectorStrategy 11, Vis/FeatureType 11
#   SuperPoint model path = SuperPoint/ModelPath (NOT Kp/DictionaryPath)
#   SuperGlue matcher = Vis/CorNNType 6 (NOT Vis/CorType 4)
#   SuperGlue variant = PyMatcher/Model indoor|outdoor (NOT Vis/SuperGluePath)
SUPERPOINT_PARAMS = [
    "--Kp/DetectorStrategy", "11",
    "--Vis/FeatureType", "11",
    "--SuperPoint/ModelPath", "/models/superpoint_v1.pt",
    "--Vis/CorNNType", "6",
    "--PyMatcher/Path", "/opt/SuperGluePretrainedNetwork/rtabmap_superglue.py",
    "--Vis/MinInliers", "15",
]

# rtabmap-export render modes
# Cloud density is controlled by two orthogonal parameters:
#   --cloud_decimation N : keep 1 pixel out of N from each depth image (default: 4)
#                          1 = full resolution, 2 = half, 4 = quarter (default)
#   --cloud_voxel_size M : voxel-grid downsampling in metres after assembly (default: 0.01)
#                          0 = disabled (keep all points)
# Tradeoff: decimation=1 + voxel=0 → densest cloud, largest file, longest export
RENDER_MODES = {
    "cloud": ["--cloud",
              "--cloud_decimation", "1",   # full pixel resolution (default=4)
              "--cloud_voxel_size", "0.0", # no voxel downsampling (default=0.01)
              ],
    "mesh":  ["--mesh"],                                   # triangulated mesh PLY
    "texture": ["--texture", "--texture_size", "4096"],    # textured mesh OBJ
}
DEFAULT_RENDER = "cloud"


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
                      trim_start: float = 0.0, trim_end: float = 0.0,
                      quality: int = 5, superpoint: bool = False,
                      superpoint_model: str = "indoor",
                      extra_params: list[str] | None = None) -> list[str]:
    """
    Build the rtabmap-zed_svo CLI command for offline RGBD-SLAM.

    rtabmap-zed_svo uses CameraStereoZed to open the SVO natively
    through the ZED SDK, then runs F2F (Frame-to-Frame) visual odometry + SLAM.
    (Odom/Strategy 1 = F2F; override with 0 for F2M via extra_params)
    Output: <output_dir>/rtabmap.db

    extra_params override RTABMAP_PARAMS defaults (last --Param wins).
    """
    sp_params = SUPERPOINT_PARAMS + ["--PyMatcher/Model", superpoint_model] if superpoint else []
    cmd = (
        ["rtabmap-zed_svo"]
        + ["--output", output_dir_in_container]
        + ["--output_name", "rtabmap"]
        + ["--quality", str(quality)]   # 0=NONE, 1=PERFORMANCE, 2=QUALITY, 3=ULTRA(deprecated), 4=NEURAL_LIGHT, 5=NEURAL, 6=NEURAL_PLUS
        + RTABMAP_PARAMS
        + sp_params
        + (extra_params or [])
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


def build_regen_grid_cmd(output_dir_in_container: str,
                         extra_params: list[str] | None = None) -> list[str]:
    """Rebuild map.pgm/map.yaml from an existing DB with current Grid/* params."""
    return (
        ["rtabmap-zed_svo", "--regen-grid"]
        + ["--output", output_dir_in_container]
        + ["--output_name", "rtabmap"]
        + RTABMAP_PARAMS
        + (extra_params or [])
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
        default=None,
        help="Absolute path to the input SVO file. Required unless --regen-grid is set.",
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
        "--regen-grid",
        action="store_true",
        help="Rebuild map.pgm/map.yaml from existing rtabmap.db with current Grid/* params. "
             "Implies --skip-slam and --skip-export; --svo is not required.",
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
    parser.add_argument(
        "--quality",
        type=int,
        default=5,
        choices=[0, 1, 2, 3, 4, 5, 6],
        help="ZED depth quality: 0=NONE, 1=PERFORMANCE, 2=QUALITY, 3=ULTRA(deprecated), 4=NEURAL_LIGHT, 5=NEURAL, 6=NEURAL_PLUS (default: 5).",
    )
    parser.add_argument(
        "--superpoint",
        action="store_true",
        default=False,
        help="Use SuperPoint extractor + SuperGlue matcher for loop closure (requires models/ and Torch-enabled image).",
    )
    parser.add_argument(
        "--superpoint-model",
        choices=["indoor", "outdoor"],
        default="indoor",
        dest="superpoint_model",
        help="SuperGlue model variant. Only used with --superpoint (default: indoor).",
    )
    args, extra = parser.parse_known_args()

    # --regen-grid implies --skip-slam and --skip-export: only the grid is rebuilt
    if args.regen_grid:
        args.skip_slam = True
        args.skip_export = True

    # Extra arguments are forwarded as RTAB-Map --Param value pairs
    # Usage: python3 process_svo.py --svo ... --Vis/MaxDepth 5.0 --Grid/CellSize 0.03
    rtabmap_extra_params = extra  # e.g. ["--Vis/MaxDepth", "5.0"]

    # ---- Validate inputs -----------------------------------------------
    if args.svo is None:
        if not args.skip_slam:
            parser.error("--svo is required unless --regen-grid (or --skip-slam) is set.")
        svo_filename = None
        data_host = None
    else:
        svo_path = Path(args.svo).resolve()
        if not svo_path.is_file():
            print(f"[ERROR] SVO file not found: {svo_path}", file=sys.stderr)
            sys.exit(1)
        svo_filename = svo_path.name
        data_host = str(svo_path.parent)

    output_host = str(Path(args.output).resolve())
    os.makedirs(output_host, exist_ok=True)
    if data_host is None:
        data_host = output_host  # regen-grid: no SVO dir, mount output as /data

    # ---- Pre-flight checks ---------------------------------------------
    check_docker()

    if not image_exists(args.image):
        print(f"[ERROR] Docker image '{args.image}' not found locally.", file=sys.stderr)
        print(f"        Build it first:\n"
              f"          docker build -f Dockerfile.rtabmap_standalone "
              f"-t {args.image} .",
              file=sys.stderr)
        sys.exit(1)

    if args.svo:
        print(f"[INFO] SVO input  : {svo_path}")
    print(f"[INFO] Output dir : {output_host}")
    print(f"[INFO] Render mode: {args.render}")
    print(f"[INFO] Docker image: {args.image}")
    if args.trim_start > 0 or args.trim_end > 0:
        print(f"[INFO] Trim start : {args.trim_start}s")
        print(f"[INFO] Trim end   : {args.trim_end}s")

    # Local binary override: mount host-compiled binary into the container
    # to avoid rebuilding Docker for every code change.
    local_bin = Path(__file__).resolve().parent.parent / "tools/ZedSvo/build/zed_svo"
    local_bin_volume = ([("-v"), f"{local_bin}:/usr/local/bin/rtabmap-zed_svo:ro"]
                        if local_bin.is_file() else [])
    if local_bin.is_file():
        print(f"[INFO] Using local binary: {local_bin}")

    # ---- Step 1: SLAM --------------------------------------------------
    if not args.skip_slam:
        rtabmap_cmd = build_rtabmap_cmd(svo_filename, "/output",
                                         args.trim_start, args.trim_end,
                                         args.quality, args.superpoint,
                                         args.superpoint_model,
                                         rtabmap_extra_params)
        slam_extra = list(local_bin_volume)
        if args.superpoint:
            slam_extra += ["-v", f"{Path.cwd()}/models:/models:ro"]
            slam_extra += ["-v", f"{Path.cwd()}/models/superglue_{args.superpoint_model}.pt:/opt/SuperGluePretrainedNetwork/models/weights/superglue_{args.superpoint_model}.pth:ro"]
        run_step(
            image=args.image,
            data_host=data_host,
            output_host=output_host,
            cmd_in_container=rtabmap_cmd,
            description="Step 1/2 – rtabmap-zed_svo: RGBD-SLAM (ZED SDK VIO)",
            extra_docker_args=slam_extra,
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

    # ---- Optional: Regen 2D grid from existing DB --------------------
    if args.regen_grid:
        db_path = Path(output_host) / "rtabmap.db"
        if not db_path.is_file():
            print(f"[ERROR] rtabmap.db not found in {output_host}.", file=sys.stderr)
            sys.exit(1)
        regen_extra = list(local_bin_volume)
        run_step(
            image=args.image,
            data_host=data_host,
            output_host=output_host,
            cmd_in_container=build_regen_grid_cmd("/output", rtabmap_extra_params),
            description="regen-grid – Rebuild 2D map from existing DB",
            extra_docker_args=regen_extra,
        )

    # ---- Step 2: Export 2-D occupancy map ------------------------------
    if not args.skip_export:
        export_cmd = build_export_cmd("/output", args.render)
        export_extra = []
        if args.superpoint:
            export_extra += ["-v", f"{Path.cwd()}/models:/models:ro"]
            export_extra += ["-v", f"{Path.cwd()}/models/superglue_{args.superpoint_model}.pt:/opt/SuperGluePretrainedNetwork/models/weights/superglue_{args.superpoint_model}.pth:ro"]
        run_step(
            image=args.image,
            data_host=data_host,
            output_host=output_host,
            cmd_in_container=export_cmd,
            description=f"Step 2/2 – rtabmap-export: {args.render} export",
            extra_docker_args=export_extra,
        )
    else:
        print("\n[SKIP] Skipping export step (--skip-export).")

    print("\n[DONE] Offline SLAM pipeline complete.")
    print(f"       Results in: {output_host}")


if __name__ == "__main__":
    main()
