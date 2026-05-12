"""
main.py – FastAPI application entry point.

Start the server (from repo root):

    uvicorn backend.main:app --host 0.0.0.0 --port 8000 --reload

In development, run the React dev server separately (cd frontend && npm run dev),
which proxies /api to localhost:8000.

For production, build the frontend first (cd frontend && npm run build), which
writes static assets to backend/static/.  The app will then serve the SPA
from that directory.
"""

import logging
import sys
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from .routers import capture, pipeline

# ── Logging ───────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger(__name__)

# ── App ───────────────────────────────────────────────────────────────────

app = FastAPI(
    title="Vista Capture App",
    version="1.0.0",
    description="Web interface for controlling ZED2i SVO recording and RTAB-Map pipeline.",
)

# API routers (must be included before static-file fallback mount)
app.include_router(capture.router, prefix="/api", tags=["capture"])
app.include_router(pipeline.router, prefix="/api", tags=["pipeline"])

# ── Static files (built React SPA) ────────────────────────────────────────

_static_dir = Path(__file__).parent / "static"

if _static_dir.is_dir():
    app.mount("/", StaticFiles(directory=str(_static_dir), html=True), name="static")
    logger.info("Serving frontend from: %s", _static_dir)
else:
    # Minimal placeholder so the server is still usable before a frontend build
    logger.warning(
        "Frontend static dir not found (%s). Run: cd frontend && npm run build",
        _static_dir,
    )

    @app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def _index():
        return (
            "<html><body>"
            "<h2>Vista Capture App – backend running</h2>"
            "<p>Build the frontend first: <code>cd frontend &amp;&amp; npm run build</code></p>"
            "<p>API docs: <a href='/docs'>/docs</a></p>"
            "</body></html>"
        )
