"""
capture.py – Endpoints for SVO recording (start / stop / SSE log stream).

POST /api/record/start  – launch svo_recording.py
POST /api/record/stop   – send SIGINT (clean shutdown via ZED SDK handler)
GET  /api/logs/record   – Server-Sent Events log stream
"""

import asyncio
import logging
import sys

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field

from ..config import REPO_ROOT
from ..utils.process_manager import process_manager

logger = logging.getLogger(__name__)
router = APIRouter()


# ── Schemas ───────────────────────────────────────────────────────────────

class RecordStartRequest(BaseModel):
    session_name: str = Field(..., min_length=1, description="Base name for the .svo2 file")
    fps: int = Field(default=15, ge=5, le=100, description="Recording frame rate")
    imu_warmup: float = Field(default=2.0, ge=0.0, description="IMU warm-up seconds before recording")


# ── Endpoints ─────────────────────────────────────────────────────────────

@router.post("/record/start", summary="Start SVO recording")
async def start_recording(body: RecordStartRequest):
    if process_manager.is_running("record"):
        raise HTTPException(409, "Recording already in progress")

    svo_path = REPO_ROOT / "data" / "raw" / f"{body.session_name}.svo2"
    (REPO_ROOT / "data" / "raw").mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable, "-u",
        str(REPO_ROOT / "src" / "svo_recording.py"),
        "--output_svo_file", str(svo_path),
        "--fps", str(body.fps),
        "--imu-warmup", str(body.imu_warmup),
    ]
    loop = asyncio.get_running_loop()
    process_manager.start("record", cmd, str(REPO_ROOT), loop)
    logger.info("Recording started → %s", svo_path)
    return {"status": "started", "svo_path": str(svo_path)}


@router.post("/record/stop", summary="Stop SVO recording (SIGINT)")
async def stop_recording():
    if not process_manager.is_running("record"):
        raise HTTPException(409, "No recording in progress")
    process_manager.stop("record")
    return {"status": "stopping"}


@router.get("/logs/record", summary="SSE log stream for recording")
async def stream_record_logs():
    mp = process_manager.get("record")

    async def generator():
        if mp is None:
            yield "data: [INFO] No recording process found.\n\n"
            return
        try:
            while True:
                try:
                    msg: str = await asyncio.wait_for(mp.log_queue.get(), timeout=15.0)
                    if msg.startswith("__EXIT__"):
                        rc = int(msg[8:])
                        yield f"data: [DONE] exit={rc}\n\n"
                        break
                    yield f"data: {msg}\n\n"
                except asyncio.TimeoutError:
                    yield ": keepalive\n\n"   # SSE comment – keeps connection alive
        except asyncio.CancelledError:
            pass  # client disconnected

    return StreamingResponse(
        generator(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",   # disable nginx proxy buffering
            "Connection": "keep-alive",
        },
    )
