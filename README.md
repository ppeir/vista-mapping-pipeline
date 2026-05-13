# Vista Capture App

> A headless web application for field mapping with the ZED 2i camera on NVIDIA Jetson Orin.  
> Control capture and SLAM processing from any smartphone — no screen, no keyboard, no SSH required.

---

## Overview

**Vista Capture App** is a local web interface that runs natively on a Jetson Orin. On the field, the operator connects to the Jetson's Wi-Fi hotspot and opens the app in a mobile browser to:

- **Record** stereo video from the ZED 2i camera into `.svo2` files.
- **Process** recordings through a modular 6-step SLAM pipeline (RTAB-Map) and download the results as a ZIP archive.

All pipeline logs stream in real time to the browser via Server-Sent Events (SSE). The entire UI fits comfortably on a smartphone screen.

---

## Architecture & Technologies

### Backend — runs natively on the Jetson host OS

The backend is a **FastAPI** application executed directly on the host (not inside Docker). This is intentional: the ZED SDK requires direct access to USB/CSI hardware and GPU drivers that are unavailable inside a standard container.

| Component | Details |
|---|---|
| Runtime | Python 3.10, Uvicorn |
| Framework | FastAPI |
| ZED SDK | Accessed natively via `pyzed` |
| Capture | `src/svo_recording.py` — SIGINT-safe SVO2 recording |
| Pipeline orchestration | `src/run_pipeline.py` — 6-step sequential/parallel runner |

### SLAM — runs inside Docker

RTAB-Map is built from source inside a custom ARM64 Docker image (`rtabmap_standalone`) that bundles ZED SDK + RTAB-Map 0.21.4. Pipeline scripts launch and monitor this container programmatically, streaming its stdout line-by-line to the UI.

```
src/process_svo.py  →  docker run rtabmap_standalone  →  rtabmap.db / map.pgm / cloud.ply
```

The Docker image is based on the Stereolabs Jetson image:
```dockerfile
FROM stereolabs/zed:5.2-tools-devel-l4t-r36.4
```

> The `tools-devel` L4T variant is the correct Jetson-native dev image for JetPack 6.1 (L4T r36.4).  
> Check available tags at [hub.docker.com/r/stereolabs/zed/tags](https://hub.docker.com/r/stereolabs/zed/tags).

### Frontend — static SPA served by FastAPI

| Component | Details |
|---|---|
| Framework | React 18 + TypeScript |
| Build tool | Vite |
| Styling | Tailwind CSS |
| Notifications | react-hot-toast |
| Deployment | Built to `backend/static/`, served directly by FastAPI as a SPA |

---

## Installation & Build

### Prerequisites

| Requirement | Details |
|---|---|
| Hardware | Jetson Orin (any variant) |
| OS | Ubuntu 22.04 ARM64 (JetPack 6.x, L4T r36.4) |
| Docker | Engine ≥ 24 + **NVIDIA Container Toolkit** |
| ZED SDK | 5.x installed on the host |
| Python | 3.10 |
| Node.js | ≥ 18 + npm |
| Disk | ~15 GB free (ZED base image ~8 GB + RTAB-Map build ~4 GB) |

Verify NVIDIA Container Toolkit:
```bash
sudo docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi
```

### 1. Clone the repository

```bash
git clone git@github-ppeir:ppeir/vista-mapping-pipeline.git
cd vista-mapping-pipeline
```

### 2. Build the SLAM Docker image

```bash
docker build \
    --file Dockerfile.rtabmap_standalone \
    --tag  rtabmap_standalone:latest \
    .
```

> Compiles RTAB-Map 0.21.4 from source. Expect **20–40 minutes** on Jetson Orin.

To build a specific RTAB-Map version:
```bash
docker build \
    --build-arg RTABMAP_VERSION=0.21.5 \
    --file Dockerfile.rtabmap_standalone \
    --tag  rtabmap_standalone:0.21.5 \
    .
```

### 3. Set up the Python virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 4. Build the frontend

```bash
cd frontend
npm install
npm run build      # outputs to ../backend/static/
cd ..
```

### 5. Start the server

```bash
source .venv/bin/activate
uvicorn backend.main:app --host 0.0.0.0 --port 8080
```

The app is now available at `http://<jetson-ip>:8080`.

---

## Field Usage — Network Access

The typical field setup does **not** require a router. Two options:

### Option A — Jetson Wi-Fi Hotspot (recommended)

Enable the Jetson's built-in hotspot (Settings → Wi-Fi → Hotspot).  
Connect the smartphone to the Jetson's hotspot network, then open:

```
http://10.42.0.1:8080
```

or if mDNS resolves: `http://jetson.local:8080`

### Option B — Smartphone USB tethering / mobile hotspot

Share the phone's connection to the Jetson. Both devices end up on the same LAN.  
The Jetson's mDNS name resolves automatically on most modern phones:

```
http://jetson.local:8080
```

> **Tip:** bookmark the URL on the smartphone home screen for one-tap access.

---

## Pipeline

### Steps

The pipeline is split into 6 independent steps. Each step can be individually skipped from the UI (generating the corresponding `--skip-xxx` flag passed to `run_pipeline.py`).

| # | Step | Script | Skip flag |
|---|---|---|---|
| 1 | Camera intrinsics | `zed_camera_info.py` | `--skip-camera-info` |
| 2 | SLAM (RTAB-Map) | `process_svo.py` → Docker | `--skip-slam` |
| 3 | SVO export (MP4 + depth PNGs) | `svo_export.py` | `--skip-video` / `--skip-depth` |
| 4 | Pose conversion | `convert_poses.py` | `--skip-poses` |
| 5 | 2D projection | `project_ply.py` | `--skip-projection` |
| 6 | ZIP assembly | *(inline in run_pipeline.py)* | `--skip-zip` |

Steps 2 and 3 run **in parallel** (both read the SVO file independently, writing to different outputs).

The final ZIP contains: `positions.json`, `<name>.mp4`, `depth/*.png`, `map.pgm`, `map.yaml`, `camera_info.json`, `depth_camera_info.json`.

### SVO export modes

Step 3 uses `svo_export.py` which selects the export mode automatically based on which sub-options are enabled:

| Enabled | Mode | Description |
|---|---|---|
| Video + Depth | `5` | Single SVO pass: MP4 + depth PNG sequence |
| Video only | `0` | MP4 only |
| Depth only | `4` | Depth PNG sequence only |

### Preset YAML configuration

Each pipeline run is driven by a YAML preset (`config/presets/`). Key parameters:

| Parameter | Description |
|---|---|
| `min_z` / `max_z` | Height slice for 2D projection (metres) |
| `resolution` | Map cell size (metres/pixel) |
| `side` | Camera side for video export (`left` / `right`) |
| `depth_scale` | Scale factor for depth image resolution (default: `0.75`) |
| `depth_compression` | PNG compression level for depth images (0–9, default: `5`) |
| `trim_start` / `trim_end` | Seconds to skip at start/end of the SVO |
| `render` | 3D export mode: `cloud` / `mesh` / `texture` |
| `superpoint` | Enable SuperPoint+SuperGlue features for loop closure |
| `quality` | ZED depth quality (0–6) |

RTAB-Map parameters can also be overridden per-preset under the `rtabmap:` key.

### Key RTAB-Map parameters (tuning guide)

| Parameter | Default | Effect |
|---|---|---|
| `Grid/CellSize` | `0.05` | Map resolution in metres (smaller = finer, more RAM) |
| `Odom/Strategy` | `0` (F2M) | `0`=Frame-to-Map, `1`=FOVIS, `2`=ORB |
| `Rtabmap/DetectionRate` | `1` | Loop closure checks per second; `0`=every frame |
| `Mem/STMSize` | `30` | Short-term memory window; increase for large environments |
| `Optimizer/Strategy` | `0` (TORO) | `0`=TORO (ARM64 built-in), `1`=g2o, `2`=GTSAM |
| `Grid/RayTracing` | `true` | Better free-space estimation; slower |

Full RTAB-Map parameter reference: <https://github.com/introlab/rtabmap/wiki/Appendix>

---

## Outputs

| File | Description |
|---|---|
| `rtabmap.db` | Full RTAB-Map database: poses, point clouds, loop closures |
| `rtabmap_cloud.ply` | Voxel-filtered coloured point cloud |
| `map.pgm` | 2D occupancy grid (0=occupied, 205=unknown, 254=free) |
| `map.yaml` | ROS-compatible map metadata (resolution, origin) |
| `map_manual.pgm` | Alternative 2D map from manual PLY projection (Step 5) |
| `rtabmap_poses.txt` | Raw RTAB-Map pose trajectory |
| `positions.json` | Converted pose trajectory (Step 4 output) |
| `camera_info.json` | Colour camera intrinsic matrix |
| `depth_camera_info.json` | Depth camera intrinsic matrix (scaled by `depth_scale`) |
| `<name>.mp4` | H.264 re-encoded video (browser-compatible) |
| `depth/*.png` | 16-bit depth PNG sequence (millimetres) |
| `<name>.zip` | Final archive with all of the above |

---

## Project Structure

```
vista-mapping-pipeline/
│
├── backend/                        # FastAPI application
│   ├── main.py                     # Entry point, static file serving
│   ├── config.py                   # Shared paths (REPO_ROOT)
│   ├── routers/
│   │   ├── pipeline.py             # Pipeline endpoints (start, logs, download)
│   │   └── capture.py              # Recording endpoints (start, stop, logs)
│   ├── utils/
│   │   └── process_manager.py      # Subprocess management + SSE streaming
│   └── static/                     # Built frontend (generated by npm run build)
│
├── frontend/                       # React / TypeScript SPA
│   ├── src/App.tsx                 # Single-page application (all UI logic)
│   ├── vite.config.ts
│   └── package.json
│
├── src/                            # Python pipeline scripts
│   ├── run_pipeline.py             # Main orchestrator (6 steps)
│   ├── process_svo.py              # SLAM via Docker (rtabmap_standalone)
│   ├── svo_export.py               # MP4 + depth PNG export from SVO2
│   ├── zed_camera_info.py          # ZED camera intrinsics extractor
│   ├── convert_poses.py            # RTAB-Map poses → positions.json
│   ├── project_ply.py              # PLY point cloud → 2D occupancy grid
│   └── svo_recording.py            # ZED SVO2 capture script
│
├── config/
│   └── presets/                    # YAML pipeline presets
│       ├── indoor.yaml
│       ├── outdoor.yaml
│       └── garage.yaml
│
├── tools_patch/ZedSvo/             # Custom C++ SLAM tool source
│   ├── main.cpp                    # CameraStereoZed + F2M odom + OccupancyGrid
│   └── CMakeLists.txt
│
├── data/
│   ├── raw/                        # Input SVO2 files
│   ├── outputs/                    # Pipeline results (one folder per session)
│   └── logs/                       # Pipeline run logs
│
├── models/                         # SuperPoint / SuperGlue weights
├── tools/                          # Compiled native binaries (zed_svo)
├── Dockerfile.rtabmap_standalone   # ARM64 Docker image (RTAB-Map + ZED SDK)
└── README.md
```

---

## Troubleshooting

**`image not found` when starting the pipeline**  
→ Build the Docker image first (Step 2 above).

**ZED SDK initialisation fails on SVO file**  
→ Ensure the SVO was recorded with a ZED 2i and ZED SDK ≥ 5.0.  
→ Check the file is not corrupted: `file <recording>.svo2` should show a binary file.

**RTAB-Map exits immediately with `No input image`**  
→ The SVO path inside the container must be `/data/<filename>`. Check that `--svo` points to the actual `.svo2` file, not a directory.

**Pipeline reports `[FAILED exit=None]`**  
→ The subprocess stdout was not fully drained before checking the return code. This is a known issue fixed in `run_pipeline.py` by calling `proc.wait()` after joining reader threads.

**Out of memory during Docker build**  
→ Reduce parallel jobs: in `Dockerfile.rtabmap_standalone`, change `make -j$(nproc)` to `make -j2`.

**`docker: Error response from daemon: could not select device driver "nvidia"`**  
→ NVIDIA Container Toolkit is not installed or not configured.  
   Follow: <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html>

**Qt platform error (`QXcbConnection: Could not connect to display`)**  
→ Already handled: the pipeline sets `QT_QPA_PLATFORM=offscreen` inside the container.

---

## Platform

| | |
|---|---|
| Hardware | NVIDIA Jetson Orin |
| OS | Ubuntu 22.04 ARM64 |
| JetPack | 6.1 (L4T r36.4) |
| CUDA | 12.6 |
| ZED SDK | 5.2 |
| RTAB-Map | 0.21.4 (inside Docker) |
| Docker base image | `stereolabs/zed:5.2-tools-devel-l4t-r36.4` |

---

## Related modules

- [`vista-alpha/webapp/zed2i_node/`](../vista-alpha/webapp/zed2i_node/) — ROS 2 ZED node for live operation
- [`video-analysis-pipeline/`](../video-analysis-pipeline/) — Video analytics pipeline
