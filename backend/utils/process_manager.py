"""
process_manager.py – Manages long-running subprocesses for the FastAPI backend.

Each process is identified by a string key (e.g. "record", "pipeline").
Stdout/stderr of each process is read in a dedicated daemon thread and forwarded
to an asyncio.Queue, which SSE endpoints can drain asynchronously.

Carriage-return (\\r) lines (progress bars) are throttled to ≤ 2/sec so the SSE
stream is not flooded. ANSI escape codes are stripped before forwarding.
"""

import asyncio
import logging
import os
import re
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

logger = logging.getLogger(__name__)

_ANSI_RE = re.compile(r"\x1b\[[0-9;]*[mKHJABCDEFsu]")


class ProcessState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    DONE = "done"
    ERROR = "error"


@dataclass
class ManagedProcess:
    proc: subprocess.Popen
    log_queue: asyncio.Queue = field(default_factory=asyncio.Queue)
    state: ProcessState = ProcessState.RUNNING
    _loop: Optional[asyncio.AbstractEventLoop] = field(default=None, repr=False)
    _thread: Optional[threading.Thread] = field(default=None, repr=False)

    # ── Internal helpers ──────────────────────────────────────────

    def _clean(self, buf: bytearray) -> str:
        text = buf.decode("utf-8", errors="replace").strip()
        return _ANSI_RE.sub("", text)

    def _put(self, msg: str) -> None:
        """Thread-safe enqueue into the asyncio queue."""
        assert self._loop is not None
        asyncio.run_coroutine_threadsafe(self.log_queue.put(msg), self._loop)

    def _read(self) -> None:
        """Reader thread: drain stdout, handle \\r progress bars, forward clean lines."""
        buf = bytearray()
        last_r_emit = 0.0
        try:
            while True:
                char: bytes = self.proc.stdout.read(1)  # type: ignore[union-attr]
                if not char:
                    break
                if char == b"\n":
                    line = self._clean(buf)
                    if line:
                        self._put(line)
                    buf.clear()
                elif char == b"\r":
                    # Throttle \\r-overwrite lines to 2 per second
                    now = time.monotonic()
                    if now - last_r_emit >= 0.5:
                        line = self._clean(buf)
                        if line:
                            self._put(line)
                        last_r_emit = now
                    buf.clear()
                else:
                    buf.extend(char)
        except Exception as exc:  # noqa: BLE001
            logger.error("stdout reader error [pid=%s]: %s", self.proc.pid, exc)
        finally:
            # Flush remaining buffer
            if buf:
                line = self._clean(buf)
                if line:
                    self._put(line)
            rc = self.proc.wait()
            self.state = ProcessState.DONE if rc == 0 else ProcessState.ERROR
            logger.info("Process [pid=%s] exited with code %d", self.proc.pid, rc)
            self._put(f"__EXIT__{rc}")

    # ── Public API ────────────────────────────────────────────────

    def start_reader(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()


class ProcessManager:
    """Singleton process manager – one record slot, one pipeline slot."""

    def __init__(self) -> None:
        self._procs: dict[str, ManagedProcess] = {}

    def is_running(self, key: str) -> bool:
        mp = self._procs.get(key)
        return mp is not None and mp.state == ProcessState.RUNNING

    def status(self, key: str) -> ProcessState:
        mp = self._procs.get(key)
        return mp.state if mp is not None else ProcessState.IDLE

    def get(self, key: str) -> Optional[ManagedProcess]:
        return self._procs.get(key)

    def start(
        self,
        key: str,
        cmd: list[str],
        cwd: str,
        loop: asyncio.AbstractEventLoop,
    ) -> ManagedProcess:
        if self.is_running(key):
            raise RuntimeError(f"Process '{key}' is already running")

        logger.info("Starting [%s]: %s", key, " ".join(cmd))
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            bufsize=0,            # unbuffered – important for real-time output
            cwd=cwd,
        )
        mp = ManagedProcess(proc=proc, log_queue=asyncio.Queue())
        mp.start_reader(loop)
        self._procs[key] = mp
        return mp

    def stop(self, key: str) -> None:
        mp = self._procs.get(key)
        if mp is None or mp.state != ProcessState.RUNNING:
            raise RuntimeError(f"Process '{key}' is not running")
        logger.info("Sending SIGINT to [%s] pid=%d", key, mp.proc.pid)
        os.kill(mp.proc.pid, signal.SIGINT)


# Module-level singleton
process_manager = ProcessManager()
