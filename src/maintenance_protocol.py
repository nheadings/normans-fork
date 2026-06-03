"""Framing and authorization helpers for the BBB maintenance bridge."""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import os
import secrets
import time
from dataclasses import dataclass
from typing import Any


MAX_FRAME_BYTES = 2048
MAX_SESSION_SECONDS = 30 * 60
IDLE_TIMEOUT_SECONDS = 10 * 60
DEFAULT_CHUNK_BYTES = 160


class MaintenanceProtocolError(ValueError):
    """Raised when a maintenance frame is malformed or unauthorized."""


@dataclass
class ReplayWindow:
    highest_sequence: int = -1

    def accept(self, sequence: int) -> None:
        if sequence <= self.highest_sequence:
            raise MaintenanceProtocolError("replayed maintenance frame")
        self.highest_sequence = sequence


def maintenance_secret() -> bytes:
    """Return the shared maintenance secret from the environment or local file."""
    env_secret = os.environ.get("BBB_MAINTENANCE_SECRET", "").strip()
    if env_secret:
        return env_secret.encode("utf-8")

    for path in ("/etc/rotorsync/maintenance.secret", "/home/pi/.rotorsync-maintenance-secret"):
        try:
            with open(path, "rb") as handle:
                value = handle.read().strip()
            if value:
                return value
        except OSError:
            continue

    return b"rotorsync-development-maintenance-secret"


def new_nonce() -> str:
    return secrets.token_urlsafe(24)


def canonical_payload(frame: dict[str, Any]) -> bytes:
    unsigned = {key: value for key, value in frame.items() if key != "sig"}
    return json.dumps(unsigned, separators=(",", ":"), sort_keys=True).encode("utf-8")


def sign_frame(frame: dict[str, Any], secret: bytes | None = None) -> str:
    digest = hmac.new(secret or maintenance_secret(), canonical_payload(frame), hashlib.sha256).digest()
    return base64.urlsafe_b64encode(digest).decode("ascii").rstrip("=")


def verify_frame(frame: dict[str, Any], secret: bytes | None = None, now: float | None = None) -> None:
    if not isinstance(frame, dict):
        raise MaintenanceProtocolError("frame must be a JSON object")

    signature = frame.get("sig")
    if not isinstance(signature, str) or not signature:
        raise MaintenanceProtocolError("missing frame signature")

    expected = sign_frame(frame, secret)
    if not hmac.compare_digest(signature, expected):
        raise MaintenanceProtocolError("invalid frame signature")

    expires_at = frame.get("expires_at")
    if expires_at is not None:
        try:
            expires_at_value = float(expires_at)
        except (TypeError, ValueError) as exc:
            raise MaintenanceProtocolError("invalid frame expiry") from exc
        if (now or time.time()) > expires_at_value:
            raise MaintenanceProtocolError("expired maintenance frame")


def encode_frame(frame: dict[str, Any]) -> bytes:
    data = json.dumps(frame, separators=(",", ":"), sort_keys=True).encode("utf-8")
    if len(data) > MAX_FRAME_BYTES:
        raise MaintenanceProtocolError(f"maintenance frame exceeds {MAX_FRAME_BYTES} bytes")
    return data


def decode_frame(data: bytes | str) -> dict[str, Any]:
    if isinstance(data, bytes):
        if len(data) > MAX_FRAME_BYTES:
            raise MaintenanceProtocolError(f"maintenance frame exceeds {MAX_FRAME_BYTES} bytes")
        data = data.decode("utf-8")

    try:
        frame = json.loads(data)
    except json.JSONDecodeError as exc:
        raise MaintenanceProtocolError("invalid maintenance JSON frame") from exc

    if not isinstance(frame, dict):
        raise MaintenanceProtocolError("maintenance frame must be an object")
    return frame


def chunk_text(text: str, chunk_bytes: int = DEFAULT_CHUNK_BYTES) -> list[str]:
    encoded = text.encode("utf-8")
    chunks: list[str] = []
    for start in range(0, len(encoded), chunk_bytes):
        chunks.append(encoded[start:start + chunk_bytes].decode("utf-8", errors="replace"))
    return chunks or [""]
