from src.dashboard_polling import (
    STATUS_POLL_INTERVAL_ACTIVE,
    STATUS_POLL_INTERVAL_IDLE,
    dashboard_is_active,
    dashboard_poll_interval,
)


def test_idle_dashboard_uses_relaxed_poll_interval():
    state = {
        "mode": "fill",
        "flow_gpm": 0.0,
        "fill_pending": False,
        "can_confirm_fill": False,
    }

    assert dashboard_is_active(state) is False
    assert dashboard_poll_interval(state) == STATUS_POLL_INTERVAL_IDLE


def test_flowing_dashboard_uses_active_poll_interval():
    state = {
        "mode": "fill",
        "flow_gpm": 0.4,
        "fill_pending": False,
        "can_confirm_fill": False,
    }

    assert dashboard_is_active(state) is True
    assert dashboard_poll_interval(state) == STATUS_POLL_INTERVAL_ACTIVE


def test_pending_dashboard_uses_active_poll_interval():
    assert dashboard_is_active({"fill_pending": True}) is True
    assert dashboard_is_active({"can_confirm_fill": True}) is True
