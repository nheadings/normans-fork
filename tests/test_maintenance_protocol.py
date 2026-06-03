import time

import pytest

from src.maintenance_protocol import (
    MaintenanceProtocolError,
    ReplayWindow,
    decode_frame,
    encode_frame,
    sign_frame,
    verify_frame,
)


def signed_frame(**extra):
    frame = {
        "type": "heartbeat",
        "session_id": "session-1",
        "seq": 1,
        "expires_at": time.time() + 30,
        **extra,
    }
    frame["sig"] = sign_frame(frame, b"test-secret")
    return frame


def test_signed_frame_verifies():
    frame = signed_frame()
    verify_frame(frame, b"test-secret")


def test_tampered_frame_rejected():
    frame = signed_frame()
    frame["seq"] = 2
    with pytest.raises(MaintenanceProtocolError):
        verify_frame(frame, b"test-secret")


def test_expired_frame_rejected():
    frame = signed_frame(expires_at=time.time() - 1)
    frame["sig"] = sign_frame(frame, b"test-secret")
    with pytest.raises(MaintenanceProtocolError):
        verify_frame(frame, b"test-secret")


def test_replay_window_rejects_old_sequence():
    replay = ReplayWindow()
    replay.accept(1)
    replay.accept(2)
    with pytest.raises(MaintenanceProtocolError):
        replay.accept(2)


def test_frame_round_trip():
    frame = signed_frame()
    assert decode_frame(encode_frame(frame)) == frame
