"""Dashboard polling cadence for the BLE bridge."""

STATUS_POLL_INTERVAL_IDLE = 2.0
STATUS_POLL_INTERVAL_ACTIVE = 0.5
HISTORY_POLL_INTERVAL = 10.0


def dashboard_is_active(state):
    """Return true when live state needs faster foreground updates."""
    if not isinstance(state, dict):
        return False

    if state.get("fill_pending") or state.get("can_confirm_fill"):
        return True

    try:
        if abs(float(state.get("flow_gpm") or 0.0)) > 0.05:
            return True
    except (TypeError, ValueError):
        pass

    mode = str(state.get("mode") or "").strip().lower()
    return mode in ("active", "fill_active", "filling", "flowing", "pump", "pumping")


def dashboard_poll_interval(state):
    """Preserve fill responsiveness without idle log churn."""
    return STATUS_POLL_INTERVAL_ACTIVE if dashboard_is_active(state) else STATUS_POLL_INTERVAL_IDLE
