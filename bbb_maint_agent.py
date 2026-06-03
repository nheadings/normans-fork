#!/usr/bin/env python3
"""Sidecar PTY maintenance agent for RotorSync BBB boxes.

The BLE GATT process forwards signed JSON frames to this local TCP service.
The service owns the PTY lifecycle so dashboard.py and the safety loop never
run remote shell work.
"""

from __future__ import annotations

import asyncio
import base64
import fcntl
import hashlib
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
MAX_UPDATE_BYTES = int(os.environ.get("BBB_MAINT_MAX_UPDATE_BYTES", str(40 * 1024 * 1024)))
UPDATE_ROOT = Path(os.environ.get("BBB_MAINT_UPDATE_ROOT", "/home/pi/.rotorsync-maintenance-updates"))


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


@dataclass
class UpdateTransfer:
    update_id: str
    expected_size: int
    expected_sha256: str
    part_path: Path
    final_path: Path
    manifest_path: Path
    received: int = 0
    started_at: float = field(default_factory=time.time)


class MaintenanceAgent:
    def __init__(self) -> None:
        self.session: ShellSession | None = None
        self.clients: set[asyncio.StreamWriter] = set()
        self.update_root = UPDATE_ROOT
        self.active_update: UpdateTransfer | None = None

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
        elif command == "update_begin":
            await self.update_begin(frame)
        elif command == "update_chunk":
            await self.update_chunk(frame)
        elif command == "update_finalize":
            await self.update_finalize(frame)
        elif command == "update_status":
            await self.update_status(frame)
        elif command == "update_abort":
            await self.update_abort(frame)
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

    async def update_begin(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        update_id = self._safe_update_id(str(frame.get("update_id") or frame.get("updateId") or ""))
        expected_size = self._expected_update_size(frame.get("size"))
        expected_sha256 = self._expected_sha256(str(frame.get("sha256") or ""))

        update_dir = self.update_root / update_id
        update_dir.mkdir(parents=True, exist_ok=True)
        part_path = update_dir / "artifact.bin.part"
        final_path = update_dir / "artifact.bin"
        manifest_path = update_dir / "manifest.json"

        for path in (part_path, final_path, manifest_path):
            try:
                path.unlink()
            except FileNotFoundError:
                pass

        self.active_update = UpdateTransfer(
            update_id=update_id,
            expected_size=expected_size,
            expected_sha256=expected_sha256,
            part_path=part_path,
            final_path=final_path,
            manifest_path=manifest_path,
        )
        session.touch()
        await self.broadcast({
            "type": "update_started",
            "session_id": session.session_id,
            "update_id": update_id,
            "size": expected_size,
            "sha256": expected_sha256,
            "timestamp": time.time(),
        })

    async def update_chunk(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        transfer = self._require_update(str(frame.get("update_id") or frame.get("updateId") or ""))
        offset = self._non_negative_int(frame.get("offset"), "invalid update chunk offset")
        if offset != transfer.received:
            raise MaintenanceProtocolError("update chunk offset mismatch")

        data_b64 = str(frame.get("data_b64") or frame.get("dataBase64") or "")
        try:
            data = base64.b64decode(data_b64, validate=True)
        except Exception as exc:
            raise MaintenanceProtocolError("invalid update chunk data") from exc
        if not data:
            raise MaintenanceProtocolError("empty update chunk")
        if transfer.received + len(data) > transfer.expected_size:
            raise MaintenanceProtocolError("update chunk exceeds expected size")

        with transfer.part_path.open("ab") as handle:
            handle.write(data)
        transfer.received += len(data)
        session.touch()
        await self.broadcast({
            "type": "update_chunk_ack",
            "session_id": session.session_id,
            "update_id": transfer.update_id,
            "offset": transfer.received,
            "received": transfer.received,
            "size": transfer.expected_size,
            "timestamp": time.time(),
        })

    async def update_finalize(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        transfer = self._require_update(str(frame.get("update_id") or frame.get("updateId") or ""))
        if transfer.received != transfer.expected_size:
            raise MaintenanceProtocolError("update transfer incomplete")

        digest = hashlib.sha256()
        with transfer.part_path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        actual_sha256 = digest.hexdigest()
        if actual_sha256 != transfer.expected_sha256:
            raise MaintenanceProtocolError("update sha256 mismatch")

        transfer.part_path.replace(transfer.final_path)
        manifest = {
            "update_id": transfer.update_id,
            "size": transfer.expected_size,
            "sha256": actual_sha256,
            "verified": True,
            "verified_at": time.time(),
            "artifact": str(transfer.final_path),
        }
        transfer.manifest_path.write_text(json.dumps(manifest, separators=(",", ":"), sort_keys=True))
        self.active_update = None
        session.touch()
        await self.broadcast({
            "type": "update_verified",
            "session_id": session.session_id,
            "update_id": transfer.update_id,
            "size": transfer.expected_size,
            "sha256": actual_sha256,
            "artifact": str(transfer.final_path),
            "timestamp": time.time(),
        })

    async def update_status(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        update_id = str(frame.get("update_id") or frame.get("updateId") or "").strip()
        if self.active_update and (not update_id or update_id == self.active_update.update_id):
            transfer = self.active_update
            payload = {
                "type": "update_status",
                "session_id": session.session_id,
                "update_id": transfer.update_id,
                "state": "receiving",
                "received": transfer.received,
                "size": transfer.expected_size,
                "sha256": transfer.expected_sha256,
                "timestamp": time.time(),
            }
        elif update_id:
            manifest_path = self.update_root / self._safe_update_id(update_id) / "manifest.json"
            payload = {
                "type": "update_status",
                "session_id": session.session_id,
                "update_id": update_id,
                "state": "verified" if manifest_path.exists() else "missing",
                "timestamp": time.time(),
            }
            if manifest_path.exists():
                try:
                    payload.update(json.loads(manifest_path.read_text()))
                except Exception:
                    payload["state"] = "manifest_error"
        else:
            payload = {
                "type": "update_status",
                "session_id": session.session_id,
                "state": "idle",
                "timestamp": time.time(),
            }
        session.touch()
        await self.broadcast(payload)

    async def update_abort(self, frame: dict[str, Any]) -> None:
        session = self._require_session(frame)
        transfer = self._require_update(str(frame.get("update_id") or frame.get("updateId") or ""))
        try:
            transfer.part_path.unlink()
        except FileNotFoundError:
            pass
        update_id = transfer.update_id
        self.active_update = None
        session.touch()
        await self.broadcast({
            "type": "update_aborted",
            "session_id": session.session_id,
            "update_id": update_id,
            "timestamp": time.time(),
        })

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

    def _safe_update_id(self, raw_update_id: str) -> str:
        update_id = raw_update_id.strip()
        if not update_id:
            raise MaintenanceProtocolError("missing update id")
        if len(update_id) > 96 or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-" for ch in update_id):
            raise MaintenanceProtocolError("invalid update id")
        return update_id

    def _expected_update_size(self, raw_size: Any) -> int:
        size = self._non_negative_int(raw_size, "invalid update size")
        if size <= 0:
            raise MaintenanceProtocolError("update size must be positive")
        if size > MAX_UPDATE_BYTES:
            raise MaintenanceProtocolError("update exceeds maximum allowed size")
        return size

    def _expected_sha256(self, value: str) -> str:
        digest = value.strip().lower()
        if len(digest) != 64 or any(ch not in "0123456789abcdef" for ch in digest):
            raise MaintenanceProtocolError("invalid update sha256")
        return digest

    def _non_negative_int(self, value: Any, message: str) -> int:
        try:
            parsed = int(value)
        except (TypeError, ValueError) as exc:
            raise MaintenanceProtocolError(message) from exc
        if parsed < 0:
            raise MaintenanceProtocolError(message)
        return parsed

    def _require_update(self, update_id: str) -> UpdateTransfer:
        if not self.active_update:
            raise MaintenanceProtocolError("no active update transfer")
        safe_update_id = self._safe_update_id(update_id)
        if safe_update_id != self.active_update.update_id:
            raise MaintenanceProtocolError("update id mismatch")
        return self.active_update

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
