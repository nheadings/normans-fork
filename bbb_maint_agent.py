#!/usr/bin/env python3
"""Sidecar PTY maintenance agent for RotorSync BBB boxes.

The BLE GATT process forwards signed JSON frames to this local TCP service.
The service owns the PTY lifecycle so dashboard.py and the safety loop never
run remote shell work.
"""

from __future__ import annotations

import asyncio
import fcntl
import json
import os
import pwd
import pty
import signal
import struct
import subprocess
import termios
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from src.maintenance_protocol import (
    IDLE_TIMEOUT_SECONDS,
    MAX_SESSION_SECONDS,
    MaintenanceProtocolError,
    ReplayWindow,
    chunk_text,
    decode_frame,
    encode_frame,
    new_nonce,
    sign_frame,
    verify_frame,
)


HOST = "127.0.0.1"
PORT = int(os.environ.get("BBB_MAINT_AGENT_PORT", "18991"))
SHELL = os.environ.get("BBB_MAINT_SHELL", "/bin/bash")
PI_USER = os.environ.get("BBB_MAINT_USER", "pi")
MAX_FILE_BYTES = 64 * 1024


@dataclass
class ShellSession:
    session_id: str
    nonce: str
    created_at: float
    last_activity: float
    replay: ReplayWindow = field(default_factory=ReplayWindow)
    master_fd: int | None = None
    process: subprocess.Popen[bytes] | None = None
    stdout_sequence: int = 0

    @property
    def is_open(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def touch(self) -> None:
        self.last_activity = time.time()


class MaintenanceAgent:
    def __init__(self) -> None:
        self.session: ShellSession | None = None
        self.clients: set[asyncio.StreamWriter] = set()

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.clients.add(writer)
        try:
            while not reader.at_eof():
                raw = await reader.readline()
                if not raw:
                    break
                try:
                    frame = decode_frame(raw.strip())
                    await self.handle_frame(frame)
                except Exception as exc:
                    await self.broadcast({
                        "type": "error",
                        "message": str(exc),
                        "timestamp": time.time(),
                    })
        finally:
            self.clients.discard(writer)
            writer.close()
            await writer.wait_closed()

    async def handle_frame(self, frame: dict[str, Any]) -> None:
        command = str(frame.get("type", ""))
        if command == "hello":
            await self.broadcast({"type": "hello", "agent": "bbb-maint-agent", "timestamp": time.time()})
            return

        verify_frame(frame)
        sequence = int(frame.get("seq", 0))
        if self.session and command != "open":
            self.session.replay.accept(sequence)

        if command == "open":
            await self.open_session(frame)
        elif command == "stdin":
            await self.write_stdin(frame)
        elif command == "resize":
            await self.resize(frame)
        elif command == "heartbeat":
            await self.heartbeat(frame)
        elif command == "file_get":
            await self.file_get(frame)
        elif command == "file_put":
            await self.file_put(frame)
        elif command == "close":
            await self.close_session("closed by remote", exit_status=None)
        else:
            raise MaintenanceProtocolError(f"unknown maintenance frame type {command!r}")

    async def open_session(self, frame: dict[str, Any]) -> None:
        if self.session and self.session.is_open:
            await self.close_session("replaced by new session", exit_status=None)

        now = time.time()
        expires_at = min(float(frame.get("expires_at", now + MAX_SESSION_SECONDS)), now + MAX_SESSION_SECONDS)
        nonce = str(frame.get("nonce") or new_nonce())
        session_id = str(frame.get("session_id") or nonce)
        session = ShellSession(session_id=session_id, nonce=nonce, created_at=now, last_activity=now)
        session.replay.accept(int(frame.get("seq", 0)))

        master_fd, slave_fd = pty.openpty()
        self._set_winsize(master_fd, int(frame.get("rows", 24)), int(frame.get("cols", 80)))
        process = self._spawn_shell(slave_fd)
        os.close(slave_fd)
        session.master_fd = master_fd
        session.process = process
        self.session = session

        asyncio.create_task(self._pump_stdout(session))
        asyncio.create_task(self._watch_session(session, expires_at))
        await self.broadcast({"type": "opened", "session_id": session_id, "nonce": nonce, "timestamp": now})

    def _spawn_shell(self, slave_fd: int) -> subprocess.Popen[bytes]:
        env = os.environ.copy()
        env.update({"TERM": "xterm-256color", "SHELL": SHELL})

        preexec_fn = None
        if os.geteuid() == 0:
            try:
                pi = pwd.getpwnam(PI_USER)
            except KeyError:
                pi = None

            if pi is not None:
                def drop_privileges() -> None:
                    os.setgid(pi.pw_gid)
                    os.setuid(pi.pw_uid)
                    os.chdir(pi.pw_dir)
                preexec_fn = drop_privileges

        return subprocess.Popen(
            [SHELL, "-l"],
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            close_fds=True,
            env=env,
            preexec_fn=preexec_fn,
        )

    async def _pump_stdout(self, session: ShellSession) -> None:
        assert session.master_fd is not None
        loop = asyncio.get_running_loop()
        while session.is_open:
            try:
                data = await loop.run_in_executor(None, os.read, session.master_fd, 1024)
            except OSError:
                break
            if not data:
                break
            session.touch()
            text = data.decode("utf-8", errors="replace")
            for chunk in chunk_text(text):
                session.stdout_sequence += 1
                await self.broadcast({
                    "type": "stdout",
                    "session_id": session.session_id,
                    "seq": session.stdout_sequence,
                    "data": chunk,
                    "timestamp": time.time(),
                })
        status = session.process.poll() if session.process else None
        if self.session is session:
            await self.close_session("shell exited", exit_status=status)

    async def _watch_session(self, session: ShellSession, expires_at: float) -> None:
        while self.session is session and session.is_open:
            now = time.time()
            if now >= expires_at:
                await self.close_session("session expired", exit_status=None)
                return
            if now - session.last_activity >= IDLE_TIMEOUT_SECONDS:
                await self.close_session("session idle timeout", exit_status=None)
                return
            await asyncio.sleep(5)

    async def write_stdin(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        data = str(frame.get("data", "")).encode("utf-8")
        if session.master_fd is not None:
            os.write(session.master_fd, data)
            session.touch()

    async def resize(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        if session.master_fd is not None:
            self._set_winsize(session.master_fd, int(frame.get("rows", 24)), int(frame.get("cols", 80)))
            session.touch()

    async def heartbeat(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        session.touch()
        await self.broadcast({"type": "heartbeat", "session_id": session.session_id, "timestamp": time.time()})

    async def file_get(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        path = self._safe_user_path(str(frame.get("path", "")))
        data = path.read_bytes()
        if len(data) > MAX_FILE_BYTES:
            raise MaintenanceProtocolError("file_get exceeds 64 KiB limit")
        await self.broadcast({
            "type": "file_data",
            "session_id": session.session_id,
            "path": str(path),
            "data": data.decode("utf-8", errors="replace"),
            "timestamp": time.time(),
        })

    async def file_put(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        path = self._safe_user_path(str(frame.get("path", "")))
        data = str(frame.get("data", "")).encode("utf-8")
        if len(data) > MAX_FILE_BYTES:
            raise MaintenanceProtocolError("file_put exceeds 64 KiB limit")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        session.touch()
        await self.broadcast({"type": "file_saved", "session_id": session.session_id, "path": str(path), "bytes": len(data)})

    def _safe_user_path(self, raw_path: str) -> Path:
        if not raw_path:
            raise MaintenanceProtocolError("missing file path")
        pi_home = Path(pwd.getpwnam(PI_USER).pw_dir if PI_USER else "/home/pi").resolve()
        path = Path(raw_path)
        if not path.is_absolute():
            path = pi_home / path
        resolved = path.resolve()
        allowed_roots = [pi_home, Path("/opt").resolve(), Path("/tmp").resolve()]
        if not any(str(resolved).startswith(str(root)) for root in allowed_roots):
            raise MaintenanceProtocolError("file path outside allowed maintenance roots")
        return resolved

    def _require_session(self, frame: dict[str, Any]) -> ShellSession:
        if not self.session or not self.session.is_open:
            raise MaintenanceProtocolError("no open maintenance session")
        if frame.get("session_id") != self.session.session_id:
            raise MaintenanceProtocolError("session id mismatch")
        return self.session

    async def close_session(self, reason: str, exit_status: int | None) -> None:
        session = self.session
        if not session:
            return
        self.session = None
        if session.process and session.process.poll() is None:
            session.process.send_signal(signal.SIGHUP)
            try:
                await asyncio.wait_for(asyncio.to_thread(session.process.wait), timeout=2)
            except asyncio.TimeoutError:
                session.process.kill()
        if session.master_fd is not None:
            try:
                os.close(session.master_fd)
            except OSError:
                pass
        await self.broadcast({
            "type": "closed",
            "session_id": session.session_id,
            "reason": reason,
            "exit_status": exit_status,
            "timestamp": time.time(),
        })

    def _set_winsize(self, fd: int, rows: int, cols: int) -> None:
        rows = max(10, min(rows, 80))
        cols = max(20, min(cols, 240))
        fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))

    async def broadcast(self, frame: dict[str, Any]) -> None:
        frame.setdefault("sig", sign_frame(frame))
        data = encode_frame(frame) + b"\n"
        stale: list[asyncio.StreamWriter] = []
        for writer in list(self.clients):
            try:
                writer.write(data)
                await writer.drain()
            except Exception:
                stale.append(writer)
        for writer in stale:
            self.clients.discard(writer)


async def main() -> None:
    agent = MaintenanceAgent()
    server = await asyncio.start_server(agent.handle_client, HOST, PORT)
    sockets = ", ".join(str(sock.getsockname()) for sock in (server.sockets or []))
    print(f"bbb-maint-agent listening on {sockets}", flush=True)
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(main())
