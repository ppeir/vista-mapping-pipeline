# RTAB-Map Standalone – Offline SLAM from ZED SVO files

> **No ROS.  No live camera.** This module builds RTAB-Map as a native binary  
> (with ZED SDK support) inside Docker and runs it against a pre-recorded SVO file  
> to produce a globally-optimised map.

---

## Architecture

```
rtab-map/
├── Dockerfile.rtabmap_standalone   # ARM64 image: ZED SDK + RTAB-Map (no ROS)
├── tools_patch/ZedSvo/             # Custom C++ tool: rtabmap-zed_svo
│   ├── main.cpp                    #   Offline RGBD-SLAM from ZED SVO via ZED SDK
│   └── CMakeLists.txt
├── process_svo.py                  # Python orchestrator – runs the two-step pipeline
└── README.md                       # This file
```

**Pipeline:**

```
bureau_complet.svo
        │
        ▼
┌─────────────────────────────────┐
│  rtabmap-zed_svo (Step 1)       │  ← Opens SVO via ZED SDK (CameraStereoZed)
│                                 │    F2M visual odometry + RGBD-SLAM
│  Input  : /data/<file>.svo      │    Loop closure + TORO graph optimisation
│  Output : /output/rtabmap.db    │
└─────────────────────────────────┘
        │
        ▼
┌─────────────────────────────────┐
│  rtabmap-export (Step 2)        │  ← Extracts 2-D occupancy grid
│                                 │    from the optimised poses
│  Input  : /output/rtabmap.db    │
│  Output : /output/map.pgm       │
│           /output/map.yaml      │
└─────────────────────────────────┘
```

---

## Host Requirements

| Requirement | Details |
|---|---|
| Hardware | Jetson Orin (any variant) |
| JetPack | 6.1 (L4T 36.4) |
| Docker | Engine ≥ 24 + **NVIDIA Container Toolkit** |
| Disk | ~15 GB free (ZED base image ~8 GB + build ~4 GB) |
| Python | 3.10+ (orchestrator only – no extra packages needed) |

Verify NVIDIA Container Toolkit is installed:
```bash
sudo docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi
```

---

## Step 0 – Verify the ZED base image tag

The Dockerfile uses the Stereolabs image for ZED SDK 5.2 on Jetson / L4T 36.4:

```
FROM stereolabs/zed:5.2-tools-devel-l4t-r36.4
```

> The `tools-devel` L4T variant is the correct Jetson-native dev image.  
> It includes the full ZED SDK 5.2 with NEURAL depth model support and is built  
> against L4T r36.4 (JetPack 6.1) for Jetson Orin.  
> (The `gl-devel` variant only exists for x86 desktop CUDA images, not for L4T.)

Alternative equivalent tag:
```bash
FROM stereolabs/zed:5.2-tools-devel-jetson-jp6.1.0
```

Check available tags:
```bash
# Browse tags at https://hub.docker.com/r/stereolabs/zed/tags
# Filter for: tools-devel-l4t
```
Update the `FROM` line in `Dockerfile.rtabmap_standalone` accordingly.

---

## Step 1 – Build the Docker image

Run from the `rtab-map/` directory:

```bash
cd vista-project-suite/rtab-map/

docker build \
    --file Dockerfile.rtabmap_standalone \
    --tag  rtabmap_standalone:latest \
    .
```

> The build compiles RTAB-Map 0.21.4 from source. Expect **20–40 minutes** on  
> Jetson Orin depending on the number of CPU cores allocated.

To build a different RTAB-Map version:
```bash
docker build \
    --build-arg RTABMAP_VERSION=0.21.5 \
    --file Dockerfile.rtabmap_standalone \
    --tag  rtabmap_standalone:0.21.5 \
    .
```

---

## Step 2 – Run the offline SLAM pipeline

```bash
python3 process_svo.py \
    --svo    /absolute/path/to/bureau_complet.svo \
    --output /absolute/path/to/output_folder \
    --image  rtabmap_standalone:latest
```

The script will:
1. Validate inputs and check for the Docker image.
2. Run `rtabmap-zed_svo` inside the container → `output_folder/rtabmap.db`
3. Run `rtabmap-export` → `output_folder/map.pgm` + `output_folder/map.yaml`

### Common options

| Flag | Default | Description |
|---|---|---|
| `--svo` | *(required)* | Absolute path to `.svo` file |
| `--output` | `./rtabmap_output` | Host directory for all outputs |
| `--image` | `rtabmap_standalone:latest` | Docker image name:tag |
| `--skip-slam` | false | Skip SLAM (re-export only) |
| `--skip-export` | false | Skip map export (keep only `.db`) |

### Example – re-export a map from an existing database

```bash
python3 process_svo.py \
    --svo    /data/bureau_complet.svo \
    --output /data/output \
    --skip-slam
```

---

## Step 3 – Inspect the outputs

| File | Description |
|---|---|
| `rtabmap.db` | Full RTAB-Map database with poses, point clouds, loop closures |
| `map.pgm` | 2-D occupancy grid (PGM greyscale: 0=occupied 205=unknown 254=free) |
| `map.yaml` | ROS-compatible occupancy-map metadata (resolution, origin, etc.) |

Open the database with the RTAB-Map database viewer (optional, needs display):
```bash
docker run --rm \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /data/output:/output \
    rtabmap_standalone:latest \
    rtabmap-databaseViewer /output/rtabmap.db
```

---

## Key RTAB-Map parameters (tuning guide)

The parameters passed to `rtabmap` are defined in `process_svo.py` inside the  
`RTABMAP_PARAMS` list. Adjust them to suit your environment:

| Parameter | Default | Effect |
|---|---|---|
| `Grid/CellSize` | `0.05` | Map resolution in metres (smaller = finer map, more RAM) |
| `Odom/Strategy` | `0` (F2M) | `0`=Frame-to-Map, `1`=FOVIS, `2`=ORB – change if odometry drifts |
| `Rtabmap/DetectionRate` | `1` | Loop closure checks per second; `0`=every frame |
| `Mem/STMSize` | `30` | Short-term memory window; increase for large environments |
| `Optimizer/Strategy` | `0` (TORO) | `0`=TORO (built-in, works on ARM64), `1`=g2o, `2`=GTSAM |
| `Grid/RayTracing` | `true` | Better free-space estimation; slower |

Full parameter reference: <https://github.com/introlab/rtabmap/wiki/Appendix>

---

## Troubleshooting

**`image not found` error**  
→ Build the image first (Step 1).

**ZED SDK initialisation fails on SVO file**  
→ Ensure the SVO was recorded with a ZED 2i (or compatible model) and ZED SDK ≥ 4.0.  
→ Verify the SVO is not corrupted: `file bureau_complet.svo` should show a binary file.

**rtabmap exits immediately with `No input image`**  
→ The SVO filename path inside the container must be `/data/<filename>`.  
   Check that `--svo` points to the actual `.svo` file (not the directory).

**Out of memory during RTAB-Map build**  
→ Reduce parallel jobs: edit the Dockerfile and change `make -j$(nproc)` to `make -j2`.

**`docker: Error response from daemon: could not select device driver "nvidia"`**  
→ NVIDIA Container Toolkit is not installed or not configured.  
   Follow: <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html>

---

## Related modules

- [`vista-alpha/webapp/zed2i_node/`](../vista-alpha/webapp/zed2i_node/) – ROS 2 ZED node for live operation
- [`video-analysis-pipeline/`](../video-analysis-pipeline/) – Video analytics pipeline
