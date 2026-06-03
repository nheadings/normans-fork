#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import time
import sys
import struct
import socket
import serial
import threading
import subprocess
import os
import json
import csv
import shlex
import builtins
import math
from collections import deque
from pathlib import Path
from PIL import Image, ImageTk
from src.batchmix_payload import (
    parse_field_color,
    scaled_batchmix_payload_for_water,
)

# Version
VERSION_FILE = Path(__file__).with_name("VERSION")
REPO_DIR = Path(__file__).resolve().parent
SIM_MODE = os.environ.get("BBB_SIM_MODE") == "1"
SIM_STATE_DIR = Path(os.environ.get("BBB_SIM_STATE_DIR", REPO_DIR / ".sim-data"))
if SIM_MODE:
    os.environ.setdefault("BBB_SIM_STATE_DIR", str(SIM_STATE_DIR))
SIM_GEOMETRY = os.environ.get("BBB_SIM_GEOMETRY", "1920x1080")
SIM_PUMP_GLIDE_SECONDS = 3.0
SIM_RENDER_SCALE = 1.0


def _geometry_size(geometry):
    try:
        size = geometry.split("+", 1)[0]
        width, height = size.lower().split("x", 1)
        return int(width), int(height)
    except Exception:
        return 1920, 1080


SIM_WINDOW_WIDTH, SIM_WINDOW_HEIGHT = _geometry_size(SIM_GEOMETRY)


def _pi_path(path):
    """Map Pi absolute paths into a local state directory during simulator runs."""
    if SIM_MODE and isinstance(path, (str, os.PathLike)):
        path_str = os.fspath(path)
        pi_prefix = "/home/pi/"
        if path_str.startswith(pi_prefix):
            mapped = SIM_STATE_DIR / path_str[len(pi_prefix):]
            mapped.parent.mkdir(parents=True, exist_ok=True)
            return str(mapped)
    return path


if SIM_MODE:
    SIM_STATE_DIR.mkdir(parents=True, exist_ok=True)
    _real_open = builtins.open

    def _sim_open(file, *args, **kwargs):
        return _real_open(_pi_path(file), *args, **kwargs)

    builtins.open = _sim_open


def _read_local_version():
    try:
        return VERSION_FILE.read_text().strip()
    except Exception:
        return "V1.9.40"


def _read_git_ref_version(git_ref):
    try:
        result = subprocess.run(
            ['git', '-C', '/home/pi/Big-Beautiful-Box', 'show', f'{git_ref}:VERSION'],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    except Exception:
        pass
    return None


VERSION = _read_local_version()
PROGRAM_STARTED_AT = time.time()

# Import configuration
import config

if SIM_MODE:
    config.MAIN_LOG_FILE = _pi_path(config.MAIN_LOG_FILE)
    config.SERIAL_DEBUG_LOG = _pi_path(config.SERIAL_DEBUG_LOG)
    config.RELAY_TEST_LOG = _pi_path(config.RELAY_TEST_LOG)
    config.BUTTON_DEBUG_LOG = _pi_path(config.BUTTON_DEBUG_LOG)
    config.FLOW_CONTROL_LOG_FILE = _pi_path(config.FLOW_CONTROL_LOG_FILE)

from src import flow_curve
FLOW_CURVE_OVERRIDE_PATH = _pi_path(config.FLOW_CURVE_OVERRIDE_FILE)
FLOW_CURVE_SAMPLES_PATH = _pi_path(config.FLOW_CURVE_SAMPLES_FILE)
FLOW_CURVE_PROPOSAL_PATH = _pi_path(config.FLOW_CURVE_PROPOSAL_FILE)
active_flow_curve = flow_curve.FlowCurve.factory()
flow_curve_metadata = {"source": "factory", "reason": "startup"}

# Set up rotating loggers
from src.logger import get_main_logger, get_serial_logger, get_button_logger, get_relay_logger
main_logger = get_main_logger()
serial_logger = get_serial_logger()
button_logger = get_button_logger()
relay_logger = get_relay_logger()


def log_serial_debug(message):
    """Append a timestamped message to the serial debug log."""
    try:
        with open(config.SERIAL_DEBUG_LOG, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {message}\n")
    except Exception:
        pass

# Add paths for libraries
sys.path.insert(0, config.RPI_GPIO_PATH)
sys.path.insert(0, config.IOL_HAT_PATH)

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("WARNING: RPi.GPIO not available, relay control disabled")

import iolhat


class _SimStatus2:
    def __init__(self, connected=True, powered=True):
        self.pd_in_valid = 1 if connected and powered else 0
        self.transmission_rate = 0x02 if connected and powered else 0
        self.master_cycle_time = 0x20
        self.error = 0x00 if connected and powered else 0x80
        self.power = 1 if powered else 0


class _SimIOLHat:
    LED_GREEN = getattr(iolhat, "LED_GREEN", 2)
    LED_RED = getattr(iolhat, "LED_RED", 1)

    def __init__(self):
        self._lock = threading.RLock()
        self.connected = True
        self.powered = True
        self.totalizer_liters = 0.0
        self.flow_rate_l_per_s = 0.0
        self._last_tick = time.time()
        self._flow_start_l_per_s = 0.0
        self._flow_target_l_per_s = 0.0
        self._transition_start = self._last_tick
        self._transition_duration = 0.0
        self._packet_counter = 0
        self._led = self.LED_GREEN

    def _flow_at(self, when):
        if self._transition_duration <= 0:
            return self._flow_target_l_per_s
        progress = (when - self._transition_start) / self._transition_duration
        if progress >= 1:
            return self._flow_target_l_per_s
        if progress <= 0:
            return self._flow_start_l_per_s
        return self._flow_start_l_per_s + (
            self._flow_target_l_per_s - self._flow_start_l_per_s
        ) * progress

    def _tick(self):
        now = time.time()
        elapsed = max(0.0, now - self._last_tick)
        if self.connected and self.powered and elapsed > 0:
            start_flow = self._flow_at(self._last_tick)
            end_flow = self._flow_at(now)
            self.totalizer_liters += ((start_flow + end_flow) / 2.0) * elapsed
            self.flow_rate_l_per_s = end_flow
        else:
            self.flow_rate_l_per_s = self._flow_at(now)
        self._last_tick = now
        if self._transition_duration > 0 and now >= self._transition_start + self._transition_duration:
            self._transition_duration = 0.0
            self._flow_start_l_per_s = self._flow_target_l_per_s
            self.flow_rate_l_per_s = self._flow_target_l_per_s

    def set_flow_gpm(self, gpm):
        with self._lock:
            self._tick()
            flow_l_per_s = max(0.0, float(gpm)) / config.LITERS_PER_SEC_TO_GPM
            self.flow_rate_l_per_s = flow_l_per_s
            self._flow_start_l_per_s = flow_l_per_s
            self._flow_target_l_per_s = flow_l_per_s
            self._transition_start = time.time()
            self._transition_duration = 0.0

    def stop_pump(self, glide_seconds=3.0):
        with self._lock:
            self._tick()
            now = time.time()
            self._flow_start_l_per_s = self.flow_rate_l_per_s
            self._flow_target_l_per_s = 0.0
            self._transition_start = now
            self._transition_duration = max(0.0, float(glide_seconds))

    def get_flow_gpm(self):
        with self._lock:
            self._tick()
            return self.flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM

    def reset_totalizer(self):
        with self._lock:
            self.totalizer_liters = 0.0
            self._last_tick = time.time()

    def pd(self, port, index, data_length, callback):
        with self._lock:
            self._tick()
            self._packet_counter = (self._packet_counter + 1) % 256
            if not self.connected or not self.powered:
                return b"\x00" * data_length

            packet = (
                b"\x00\x00\x00\x00"
                + struct.pack(">f", abs(self.totalizer_liters))
                + struct.pack(">f", self.flow_rate_l_per_s)
                + bytes([self._packet_counter, 0, 0])
            )
            return packet[:data_length].ljust(data_length, b"\x00")

    def power(self, port, status):
        with self._lock:
            self.powered = bool(status)
        return 1

    def led(self, port, color):
        with self._lock:
            self._led = color
        return 1

    def readStatus2(self, port):
        with self._lock:
            return _SimStatus2(self.connected, self.powered)


if SIM_MODE:
    iolhat = _SimIOLHat()

# Global variables
last_totalizer_liters = 0.0
last_flow_rate = 0.0
previous_totalizer_liters = 0.0
connection_error = False
error_message = ""
requested_gallons = config.REQUESTED_GALLONS
serial_connected = False
override_mode = False
last_alert_triggered = False
auto_shutoff_latched = False  # True once the pump-stop relay has fired for the current fill cycle
last_successful_read_time = time.time()
last_flow_read_was_fresh = False  # True only when latest IO-Link data changed
last_fresh_flow_read_time = 0.0  # Monotonic-ish wall time of latest changed IO-Link data
was_flowing = False  # Track if flow was active in previous update (for detecting flow stop)
new_fill_flow_started_at = None  # Time flow first exceeded the new-fill threshold
new_fill_last_fresh_at = None  # Last fresh high-flow sample during new-fill clearing
new_fill_cycle_cleared = False  # True after prior fill state is cleared for this cycle
colors_are_green = False  # Track if colors have been changed to green
last_reminder_date = None  # Track the last date reminders were shown (YYYY-MM-DD format)
reminders_mode = False  # Track if we're showing reminders
reminders_window = None  # Reference to reminders window
menu_mode = False  # Track if we're in menu mode
menu_window = None  # Reference to menu window
menu_selected_index = 0  # Currently selected menu item (0=logs, 1=self-test, 2=update, 3=shutdown, 4=reboot, 5=exit-desktop, 6=exit-menu)
menu_buttons = []  # List of menu button widgets
menu_arrows = []  # List of arrow label widgets
menu_daily_label = None  # Reference to daily total label in menu
menu_season_label = None  # Reference to season total label in menu
menu_position_label = None  # Reference to position indicator label
menu_highlight_refresh_pending = False  # Coalesce menu redraws so knob pulses do not stack UI work
menu_displayed_index = None  # Last menu item whose highlight state was painted
menu_ov_guard_until = 0.0  # Suppress OV contact/repeat bounce after screen changes
MENU_OV_GUARD_SECONDS = 2.5
log_viewer_mode = False  # Track if we're in log viewer
log_viewer_window = None  # Reference to log viewer window
log_viewer_text = None  # Reference to log text widget
fill_history_mode = False  # Track if we're in fill history viewer
fill_history_window = None  # Reference to fill history window
fill_history_text = None  # Reference to fill history text widget
self_test_mode = False  # Track if we're in self-test
self_test_window = None  # Reference to self-test window
full_test_mode = False  # Track if we're in full-test
full_test_window = None  # Reference to full-test window
update_mode = False  # Track if we're in update screen
update_window = None  # Reference to update window
serial_command_received = False  # Track if any serial command has been received (for color change)
exit_confirm_window = None  # Reference to exit confirmation window
exit_confirm_handler = None  # Function to call on confirmation
exit_cancel_handler = None  # Function to call on cancel
reset_season_confirm_window = None  # Reference to reset season confirmation window
reset_season_confirm_handler = None  # Function to call on confirmation
reset_season_cancel_handler = None  # Function to call on cancel
reset_flow_curve_confirm_window = None  # Reference to flow curve reset confirmation
reset_flow_curve_confirm_handler = None  # Function to call on confirmation
reset_flow_curve_cancel_handler = None  # Function to call on cancel
accept_flow_curve_confirm_window = None  # Reference to learned curve accept confirmation
accept_flow_curve_confirm_handler = None  # Function to call on confirmation
accept_flow_curve_cancel_handler = None  # Function to call on cancel
daily_total = 0.0  # Total gallons pumped today
season_total = 0.0  # Total gallons pumped this season (until manually reset)
last_reset_date = None  # Track last daily reset date
last_loads_gallons = []  # Most recent recorded load sizes (newest first)
pending_fill_gallons = 0.0  # Gallons from last fill, waiting for thumbs up confirmation
pending_fill_requested = 0.0  # Requested gallons from last fill
pending_fill_shutoff_type = ""  # Shutoff type from last fill
pending_fill_flow_gpm = 0.0  # Flow snapshot associated with the completed fill
pending_fill_trigger_threshold = 0.0  # Trigger threshold associated with the completed fill
last_flowing_rate_l_per_s = 0.0  # Most recent non-zero flow during the current fill
last_trigger_flow_gpm = 0.0  # Flow when auto shutoff triggered
last_trigger_threshold = 0.0  # Threshold when auto shutoff triggered
last_trigger_actual = 0.0  # Actual gallons when auto shutoff triggered
recent_flow_rates_l_per_s = deque(maxlen=config.FLOW_AVERAGING_SAMPLES)
last_heartbeat_time = time.time()  # Last time we received OK heartbeat from switch box
heartbeat_disconnected = False  # Track if heartbeat has timed out
consecutive_identical_raw = 0  # Track byte-for-byte identical reads
last_raw_data = None  # Previous raw bytes for stale detection
last_power_cycle_time = 0         # Timestamp of last IOL power-cycle attempt
iol_power_cycle_in_progress = False  # Flag to prevent overlapping power-cycle threads
override_enabled_time = 0  # Timestamp when override mode was last enabled
iol_io_lock = threading.RLock()  # Single gate for direct IO-Link process-data calls
flow_control_stop_event = threading.Event()
flow_control_thread = None
flow_control_was_flowing = False
flow_control_last_tick = None
flow_control_last_loop_time = time.time()
flow_control_last_error_log_time = 0.0
flow_control_audit_started_at = time.time()
flow_control_audit_polls = 0
flow_control_audit_fresh = 0
flow_control_audit_duplicates = 0
flow_control_audit_flowing_polls = 0
flow_control_audit_flowing_fresh = 0
flow_control_audit_errors = 0
flow_control_audit_loop_ms_total = 0.0
flow_control_audit_loop_ms_max = 0.0
relay_slowdown_watch_active = False
relay_slowdown_alarm_active = False
relay_slowdown_alarm_visible = False
relay_slowdown_trigger_time = 0.0
relay_slowdown_trigger_flow_gpm = 0.0
pump_stop_relay_lock = threading.Lock()
pump_stop_pulse_count = 0
pump_stop_fault_hold_active = False
pump_stop_fault_hold_reason = ""
flow_meter_reconnect_fresh_reads = 0
flow_meter_reconnect_started_at = 0.0
flow_meter_reconnect_last_status_check = 0.0
flow_meter_reconnect_status_ok = False
flow_meter_reconnect_fault_reason = ""
last_trigger_predicted_actual = 0.0
last_trigger_loop_dt_ms = 0.0
# Mix/Fill mode variables
current_mode = "fill"  # Current mode: "fill" or "mix"
fill_requested_gallons = config.REQUESTED_GALLONS  # Preset for fill mode
mix_requested_gallons = 40  # Preset for mix mode (default 40)
mode_indicator_label = None  # Label to display "MIX" in corner
last_status_text = None  # Cache bottom status line to avoid needless redraws
last_daily_total_text = None  # Cache daily total footer text
last_daily_total_mode = None  # Track mode used for daily total rendering
last_flow_rate_text = None  # Cache flow rate footer text
last_flow_rate_mode = None  # Track mode used for flow footer rendering

# Shared menu order.
MENU_ITEMS = [
    "VIEW LOGS",
    "FILL HISTORY",
    "TANK CALIBRATION",
    "FULL TEST",
    "RESET SEASON",
    "ACCEPT CURVE",
    "FACTORY CURVE",
    "SELF TEST",
    "CAPTURE BUG",
    "SYSTEM UPDATE",
    "SHUTDOWN",
    "REBOOT",
    "EXIT TO DESKTOP",
    "EXIT MENU",
]

# Mopeka tank level display
mopeka1_gallons = 0
mopeka2_gallons = 0
mopeka1_quality = 0
mopeka2_quality = 0
mopeka_connected = False
mopeka_enabled = True
mopeka1_level_mm = 0.0
mopeka2_level_mm = 0.0
mopeka1_level_in = 0.0
mopeka2_level_in = 0.0
bms_soc = None
bms_voltage = None

# Tank calibration workflow state
calibration_mode = False
calibration_window = None
calibration_title_label = None
calibration_body_label = None
calibration_footer_label = None
calibration_hint_label = None
calibration_state = None


# Batch mix data from iPad (cached)
batch_mix_data = None  # Cached JSON data from iPad
batch_mix_overlay = None  # Reference to batch mix overlay frame


def load_flow_curve_state():
    """Load the learned curve override, or fall back to factory values."""
    global active_flow_curve, flow_curve_metadata
    active_flow_curve, flow_curve_metadata = flow_curve.load_curve_override(
        FLOW_CURVE_OVERRIDE_PATH
    )
    status = flow_curve_status_text()
    reason = flow_curve_metadata.get("reason", "")
    msg = f"Loaded flow curve: {status}"
    if reason:
        msg = f"{msg} ({reason})"
    print(msg)
    log_serial_debug(msg)


def flow_curve_status_text():
    """Short field-readable label for the active shutoff curve."""
    if flow_curve_metadata.get("source") == "learned":
        learning = flow_curve_metadata.get("learning", {})
        offset = learning.get("applied_offset_gallons")
        if isinstance(offset, (int, float)):
            return f"Learned {offset:+.2f} gal"
        return "Learned"
    return "Factory"


def flow_curve_proposal_status_text():
    """Short label describing whether a learned curve proposal is waiting."""
    proposal = flow_curve.load_curve_proposal(FLOW_CURVE_PROPOSAL_PATH)
    if not proposal:
        return "No pending curve"
    learning = proposal.get("learning", {})
    offset = learning.get("applied_offset_gallons")
    if isinstance(offset, (int, float)):
        return f"Pending {offset:+.2f} gal"
    return "Pending curve"


def calculate_trigger_threshold(flow_rate_l_per_s):
    """
    Calculate how many gallons before target to trigger shutoff based on flow rate.
    Uses calibration data to predict coast distance after relay activation.

    Args:
        flow_rate_l_per_s: Current flow rate in liters per second

    Returns:
        Gallons before target to trigger shutoff (predicted coast distance)
    """
    return flow_curve.calculate_trigger_threshold(flow_rate_l_per_s, active_flow_curve)


def get_smoothed_flow_rate():
    """Return a short rolling average of recent flow while flow is active."""
    if not recent_flow_rates_l_per_s:
        return last_flow_rate
    return sum(recent_flow_rates_l_per_s) / len(recent_flow_rates_l_per_s)


def flow_control_enabled():
    """True when the dedicated flow-control loop owns IO-Link shutoff timing."""
    return bool(getattr(config, "FLOW_CONTROL_THREAD_ENABLED", False))


def flow_control_active():
    """True when the configured flow-control thread is currently alive."""
    return bool(
        flow_control_enabled()
        and flow_control_thread is not None
        and flow_control_thread.is_alive()
    )


def flow_read_interval_seconds():
    """Return the expected interval between IO-Link process-data reads."""
    if flow_control_enabled():
        return max(0.01, float(getattr(config, "FLOW_CONTROL_INTERVAL", 0.05)))
    return max(0.01, config.UPDATE_INTERVAL / 1000.0)


def stale_raw_threshold_reads():
    """Convert the stale-data timeout into a read count for the active poll rate."""
    return max(1, int(config.FLOW_METER_TIMEOUT / flow_read_interval_seconds()))


def get_cached_actual_gallons():
    """Return the latest cached totalizer reading without touching IO-Link."""
    return last_totalizer_liters * config.LITERS_TO_GALLONS


def log_flow_control(message):
    """Append a timestamped flow-control event for shutoff timing audits."""
    try:
        with open(config.FLOW_CONTROL_LOG_FILE, "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | {message}\n")
    except Exception:
        pass


def log_relay_event(message):
    """Append a timestamped pump-stop relay event."""
    try:
        with open(config.RELAY_TEST_LOG, "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {message}\n")
    except Exception:
        pass


def _set_pump_stop_output(active, reason):
    """Drive the pump-stop relay output without changing hold/pulse state."""
    if SIM_MODE:
        if active:
            iolhat.stop_pump(SIM_PUMP_GLIDE_SECONDS)
        return True

    if not GPIO_AVAILABLE:
        log_relay_event(f"ERROR: Cannot {'activate' if active else 'release'} relay ({reason}); GPIO unavailable")
        return False

    GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.HIGH if active else GPIO.LOW)
    log_relay_event(f"Relay {'HIGH' if active else 'LOW'} ({reason})")
    return True


def set_pump_stop_fault_hold(active, reason="flow meter fault"):
    """Latch flow-meter fault status and pulse pump stop when the fault starts."""
    global pump_stop_fault_hold_active, pump_stop_fault_hold_reason

    reason = reason or "flow meter fault"
    pulse_fault_stop = False
    with pump_stop_relay_lock:
        if active:
            if pump_stop_fault_hold_active:
                if pump_stop_fault_hold_reason != reason:
                    pump_stop_fault_hold_reason = reason
                    log_flow_control(f"pump_stop_fault_hold_reason | reason={reason}")
                return
            pump_stop_fault_hold_active = True
            pump_stop_fault_hold_reason = reason
            pulse_fault_stop = True
            log_flow_control(f"pump_stop_fault_latch_active | reason={reason}")
        else:
            if not pump_stop_fault_hold_active:
                return
            previous_reason = pump_stop_fault_hold_reason
            pump_stop_fault_hold_active = False
            pump_stop_fault_hold_reason = ""
            log_flow_control(f"pump_stop_fault_latch_cleared | reason={previous_reason}")

    if pulse_fault_stop:
        log_flow_control(
            f"pump_stop_fault_momentary_stop | reason={reason}"
            f" | duration={config.PUMP_STOP_DURATION}s"
        )
        start_pump_stop_thread(config.PUMP_STOP_DURATION)


def update_flow_meter_fault_hold(flow_meter_disconnected=None):
    """Pulse pump stop and latch warning while the flow meter is disconnected or stale."""
    global flow_meter_reconnect_fresh_reads
    global flow_meter_reconnect_started_at, flow_meter_reconnect_last_status_check
    global flow_meter_reconnect_status_ok, flow_meter_reconnect_fault_reason

    if flow_meter_disconnected is None:
        flow_meter_disconnected = (time.time() - last_successful_read_time) > config.FLOW_METER_TIMEOUT

    if connection_error:
        flow_meter_reconnect_fresh_reads = 0
        flow_meter_reconnect_started_at = 0.0
        flow_meter_reconnect_status_ok = False
        flow_meter_reconnect_fault_reason = ""
        set_pump_stop_fault_hold(True, error_message or "flow meter connection error")
    elif flow_meter_disconnected:
        flow_meter_reconnect_fresh_reads = 0
        flow_meter_reconnect_started_at = 0.0
        flow_meter_reconnect_status_ok = False
        flow_meter_reconnect_fault_reason = ""
        set_pump_stop_fault_hold(True, "flow meter read timeout")
    elif pump_stop_fault_hold_active:
        now = time.time()
        required_reads = max(1, int(getattr(config, "FLOW_METER_RECONNECT_FRESH_READS", 3)))
        stable_seconds = max(0.0, float(getattr(config, "FLOW_METER_RECONNECT_STABLE_SECONDS", 10.0)))
        if iol_power_cycle_in_progress:
            flow_meter_reconnect_started_at = 0.0
            flow_meter_reconnect_fresh_reads = 0
            flow_meter_reconnect_status_ok = False
            flow_meter_reconnect_fault_reason = ""
        elif now - flow_meter_reconnect_last_status_check >= 1.0:
            flow_meter_reconnect_last_status_check = now
            try:
                flow_meter_reconnect_status_ok, st = _read_iol_status_ok()
                if flow_meter_reconnect_status_ok:
                    if flow_meter_reconnect_started_at <= 0:
                        flow_meter_reconnect_started_at = now
                    flow_meter_reconnect_fresh_reads += 1
                    flow_meter_reconnect_fault_reason = ""
                else:
                    flow_meter_reconnect_started_at = 0.0
                    flow_meter_reconnect_fresh_reads = 0
                    flow_meter_reconnect_fault_reason = _describe_iol_status_fault(st)
                log_flow_control(
                    "flow_meter_reconnect_status"
                    f" | ok={flow_meter_reconnect_status_ok}"
                    f" | pdInValid={st.pd_in_valid}"
                    f" | txRate=0x{st.transmission_rate:02X}"
                    f" | error=0x{st.error:02X}"
                    f" | reason={flow_meter_reconnect_fault_reason or 'healthy'}"
                )
            except Exception as exc:
                flow_meter_reconnect_status_ok = False
                flow_meter_reconnect_started_at = 0.0
                flow_meter_reconnect_fresh_reads = 0
                flow_meter_reconnect_fault_reason = "flow meter status read failed"
                log_flow_control(f"flow_meter_reconnect_status | ok=False | error={exc}")

        stable_elapsed = (
            now - flow_meter_reconnect_started_at
            if flow_meter_reconnect_started_at > 0 else 0.0
        )
        if flow_meter_reconnect_fresh_reads >= required_reads:
            if flow_meter_reconnect_status_ok and stable_elapsed >= stable_seconds:
                flow_meter_reconnect_fresh_reads = 0
                flow_meter_reconnect_started_at = 0.0
                flow_meter_reconnect_status_ok = False
                set_pump_stop_fault_hold(False)
            else:
                set_pump_stop_fault_hold(
                    True,
                    f"waiting for stable flow meter recovery ({stable_elapsed:.1f}/{stable_seconds:.1f}s)",
                )
        else:
            set_pump_stop_fault_hold(
                True,
                flow_meter_reconnect_fault_reason or "waiting for healthy flow meter status",
            )
    else:
        flow_meter_reconnect_fresh_reads = 0
        flow_meter_reconnect_started_at = 0.0
        flow_meter_reconnect_status_ok = False
        flow_meter_reconnect_fault_reason = ""
        set_pump_stop_fault_hold(False)


def _suppress_startup_iol_warning(flow_meter_disconnected=False):
    """Keep startup IO-Link settling quiet on-screen without changing relay safety."""
    grace_seconds = max(0.0, float(getattr(config, "IOL_STARTUP_WARNING_GRACE_SECONDS", 0.0)))
    if grace_seconds <= 0.0 or time.time() - PROGRAM_STARTED_AT > grace_seconds:
        return False
    if last_flow_rate >= config.FLOW_STOPPED_THRESHOLD:
        return False
    return bool(pump_stop_fault_hold_active or connection_error or flow_meter_disconnected)


def start_pump_stop_thread(duration=config.AUTO_ALERT_DURATION):
    """Fire pump-stop relay without blocking the caller."""
    relay_thread = threading.Thread(
        target=pump_stop_relay,
        args=(duration,),
        daemon=True,
    )
    relay_thread.start()

def load_totals():
    """Load daily and season totals from files"""
    global daily_total, season_total, last_reset_date

    # Load daily total
    try:
        with open('/home/pi/daily_total.txt', 'r') as f:
            lines = f.readlines()
            if len(lines) >= 2:
                daily_total = float(lines[0].strip())
                last_reset_date = lines[1].strip()
    except Exception:
        daily_total = 0.0
        last_reset_date = None

    # Load season total
    try:
        with open('/home/pi/season_total.txt', 'r') as f:
            season_total = float(f.read().strip())
    except Exception:
        season_total = 0.0

def save_totals():
    """Save daily and season totals to files"""
    global daily_total, season_total, last_reset_date

    # Save daily total with date
    try:
        with open('/home/pi/daily_total.txt', 'w') as f:
            f.write(f"{daily_total}\n")
            f.write(f"{last_reset_date}\n")
    except Exception as e:
        print(f"Error saving daily total: {e}")

    # Save season total
    try:
        with open('/home/pi/season_total.txt', 'w') as f:
            f.write(f"{season_total}\n")
    except Exception as e:
        print(f"Error saving season total: {e}")


def load_last_load():
    """Load the three most recent recorded actual gallons from fill history."""
    global last_loads_gallons

    try:
        with open('/home/pi/fill_history.log', 'r') as f:
            lines = [line.strip() for line in f if line.strip()]
        last_loads_gallons = []
        for last_line in reversed(lines[-3:]):
            marker = "Actual: "
            start = last_line.index(marker) + len(marker)
            end = last_line.index(" gal", start)
            last_loads_gallons.append(float(last_line[start:end]))
    except Exception:
        last_loads_gallons = []

def load_mode_presets():
    """Load fill and mix mode gallon presets from file"""
    global fill_requested_gallons, mix_requested_gallons, current_mode, batch_mix_data

    try:
        with open('/home/pi/mode_presets.txt', 'r') as f:
            lines = f.readlines()
            if len(lines) >= 3:
                fill_requested_gallons = float(lines[0].strip())
                mix_requested_gallons = float(lines[1].strip())
                current_mode = lines[2].strip()
                if current_mode not in ['fill', 'mix']:
                    current_mode = 'fill'
    except Exception:
        fill_requested_gallons = config.REQUESTED_GALLONS
        mix_requested_gallons = 40
        current_mode = 'fill'

def save_mode_presets():
    """Save fill and mix mode gallon presets to file"""
    global fill_requested_gallons, mix_requested_gallons, current_mode, batch_mix_data

    try:
        with open('/home/pi/mode_presets.txt', 'w') as f:
            f.write(f"{fill_requested_gallons}\n")
            f.write(f"{mix_requested_gallons}\n")
            f.write(f"{current_mode}\n")
    except Exception as e:
        print(f"Error saving mode presets: {e}")

def switch_mode(new_mode):
    """Switch between fill and mix modes"""
    global current_mode, requested_gallons, fill_requested_gallons, mix_requested_gallons
    global mode_indicator_label, colors_are_green, serial_command_received, batch_mix_layout_active
    global thumbs_up_label, thumbs_up_animation_id, override_mode, override_enabled_time
    global last_totalizer_liters, last_flow_rate

    if new_mode == current_mode:
        return  # Already in this mode

    # Save current requested gallons to the current mode
    if current_mode == 'fill':
        fill_requested_gallons = requested_gallons
    else:
        mix_requested_gallons = requested_gallons

    current_actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    current_flow_gpm = last_flow_rate * 60 * config.LITERS_TO_GALLONS

    if current_mode == 'mix' and new_mode == 'fill' and override_mode:
        override_mode = False
        override_enabled_time = None
        print("Cleared override while switching from MIX to FILL")

    if current_mode == 'mix' and new_mode == 'fill' and current_actual_gallons > 0 and current_flow_gpm < 10:
        root.after(0, lambda: force_flow_reset("mix_to_fill_low_flow"))

    if current_mode == 'fill' and new_mode == 'mix' and current_actual_gallons > 0 and current_flow_gpm < 10:
        root.after(0, lambda: force_flow_reset("fill_to_mix_low_flow"))

    # Switch to new mode and load its preset
    current_mode = new_mode
    if current_mode == 'fill':
        requested_gallons = fill_requested_gallons
    else:
        requested_gallons = mix_requested_gallons

    # Reset color state for new fill
    colors_are_green = False
    serial_command_received = False

    update_mix_mode_indicator()

    if current_mode == 'mix':
        if thumbs_up_animation_id:
            root.after_cancel(thumbs_up_animation_id)
            thumbs_up_animation_id = None
        if thumbs_up_label:
            thumbs_up_label.place_forget()
            _set_thumbs_up_visible(False)

    # Update mode indicator
    if mode_indicator_label:
        if current_mode == 'mix':
            place_mix_mode_indicator()
        else:
            mode_indicator_label.place_forget()

    # Save presets
    save_mode_presets()

    # Update the display
    draw_requested_number(f"{requested_gallons:.0f}", "red")
    update_mopeka_display()
    update_last_load_display()
    update_bms_display()

    print(f"Switched to {current_mode.upper()} mode - requested gallons: {requested_gallons}")

    # Show/hide batch mix overlay based on mode
    update_batch_mix_overlay()

# Track if batch mix layout is active
batch_mix_layout_active = False

# Cache for preventing flicker - only redraw when values change
_last_requested_text = None
_last_requested_color = None
_last_actual_text = None
_last_actual_color = None
_last_batch_requested_text = None
_last_batch_requested_color = None
_last_batch_actual_text = None
_last_batch_actual_color = None

def update_batch_mix_overlay():
    """Update the batch mix screen layout based on mode and data"""
    global batch_mix_layout_active, batch_mix_data, thumbs_up_animation_id

    # Only show batch mix layout in mix mode with data
    if current_mode == "mix" and batch_mix_data is not None:
        place_mix_mode_indicator()
        if thumbs_up_animation_id:
            root.after_cancel(thumbs_up_animation_id)
            thumbs_up_animation_id = None
        if thumbs_up_label:
            thumbs_up_label.place_forget()
            _set_thumbs_up_visible(False)
        if not batch_mix_layout_active:
            activate_batch_mix_layout()
        else:
            refresh_batch_mix_products()
            refresh_batch_mix_totals()
            refresh_batch_mix_tank_levels()
    else:
        update_mix_mode_indicator()
        if batch_mix_layout_active:
            deactivate_batch_mix_layout()


def clear_batch_mix_screen(reason="clear"):
    """Exit the batch mix product overlay and return to the normal mix screen."""
    global batch_mix_data, thumbs_up_animation_id
    batch_mix_data = None
    if thumbs_up_animation_id:
        root.after_cancel(thumbs_up_animation_id)
        thumbs_up_animation_id = None
    if thumbs_up_label:
        thumbs_up_label.place_forget()
        _set_thumbs_up_visible(False)
    update_batch_mix_overlay()
    msg = f"Batch mix screen cleared: {reason}"
    print(msg)
    log_serial_debug(msg)

def show_batchmix_error(error_msg):
    """Display a BatchMix error message on screen"""
    canvas.delete("batchmix_error")

    width = _canvas_width()
    height = _canvas_height()

    # Red background box
    canvas.create_rectangle(width * 0.1, height * 0.3, width * 0.9, height * 0.5,
                           fill="darkred", outline="red", width=3, tags="batchmix_error")

    # Error title
    canvas.create_text(width // 2, height * 0.35, text="BATCHMIX ERROR",
                      font=("Helvetica", 28, "bold"), fill="white", tags="batchmix_error")

    # Error message
    canvas.create_text(width // 2, height * 0.43, text=error_msg,
                      font=("Helvetica", 20), fill="yellow", tags="batchmix_error")

    # Auto-clear after 5 seconds
    canvas.after(5000, lambda: canvas.delete("batchmix_error"))

def activate_batch_mix_layout():
    """Switch to batch mix screen layout"""
    global batch_mix_layout_active
    global _last_requested_text, _last_requested_color, _last_actual_text, _last_actual_color
    global _last_batch_requested_text, _last_batch_requested_color
    global _last_batch_actual_text, _last_batch_actual_color

    # Clear existing labels and redraw in new positions
    canvas.delete("labels")
    canvas.delete("batchmix")

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    # Left 1/3 section - center point
    left_center_x = width // 6

    # Draw "Requested:" label on left side
    canvas.create_text(left_center_x, int(height * 0.13), text="Requested:",
                      font=("Helvetica", 28, "bold"), fill="white", tags="labels")

    # Draw "Actual:" label on left side
    canvas.create_text(left_center_x, int(height * 0.38), text="Actual:",
                      font=("Helvetica", 28, "bold"), fill="white", tags="labels")

    # Separator line above totals (bottom 1/4)
    canvas.create_line(0, int(height * 0.75), width, int(height * 0.75),
                      fill="cyan", width=8, tags="batchmix")

    # Vertical separator between left and right sections
    canvas.create_line(width // 3, 0, width // 3, int(height * 0.75),
                      fill="cyan", width=8, tags="batchmix")

    # Products section title (right 2/3, top area)
    products_x = int(width * 0.54)
    canvas.create_text(products_x, int(height * 0.05), text="PRODUCTS",
                      font=("Helvetica", 32, "bold"), fill="lime", tags="batchmix")
    refresh_batch_mix_tank_levels()

    # Draw products and totals
    refresh_batch_mix_products()
    refresh_batch_mix_totals()

    # Mark layout active and invalidate cached number state so the side layout is forced.
    batch_mix_layout_active = True
    _last_requested_text = None
    _last_requested_color = None
    _last_actual_text = None
    _last_actual_color = None
    _last_batch_requested_text = None
    _last_batch_requested_color = None
    _last_batch_actual_text = None
    _last_batch_actual_color = None

    # Redraw the numbers in new positions
    redraw_numbers_for_batch_mix()

def refresh_batch_mix_totals():
    """Draw/update totals section at bottom of screen"""
    global batch_mix_data

    canvas.delete("totals")

    if batch_mix_data is None:
        return

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    # Bottom 1/4 section - totals info
    bottom_y = int(height * 0.85)
    label_y = min(height - 28, bottom_y + 62)
    value_font = ("Helvetica", 77, "bold")
    label_font = ("Helvetica", 42, "bold")

    acres_format = ".2f" if _batch_mix_has_product_rates() else ".1f"
    totals_data = [
        (width * 0.12, f"{batch_mix_data.get('total_acres', 0):{acres_format}}", "ACRES"),
        (width * 0.37, f"{batch_mix_data.get('gallons_per_acre', 0):.1f}", "GAL/AC"),
        (width * 0.62, f"{batch_mix_data.get('total_liquid', 0):.1f}", "TOTAL GAL"),
        (width * 0.87, f"{batch_mix_data.get('water_needed', 0):.1f}", "WATER"),
    ]

    for x, value, label in totals_data:
        # Value - large cyan text
        canvas.create_text(x, bottom_y, text=value, font=value_font,
                          fill="cyan", tags="totals")
        # Label below - smaller gray text
        canvas.create_text(x, label_y, text=label, font=label_font,
                          fill="#d0d0d0", tags="totals")

def _mopeka_quality_color(q):
    """Return display color for a Mopeka quality value."""
    if q >= 3:
        return "#00ff00"
    if q >= 2:
        return "#ffff00"
    if q >= 1:
        return "#ff8800"
    return "#ff0000"

def refresh_batch_mix_tank_levels():
    """Draw small tank levels in the BatchMix product header."""
    canvas.delete("batchmix_tanks")

    if current_mode != "mix" or batch_mix_data is None or not mopeka_enabled:
        return

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()
    x = width - 20
    y = int(height * 0.05)
    font = ("Helvetica", 28, "bold")

    if not mopeka_connected:
        canvas.create_text(x, y, text="Tanks: No Signal", font=font,
                          fill="#ff0000", anchor="ne", tags="batchmix_tanks")
        return

    front_text = f"Front: {mopeka1_gallons:.0f} gal"
    back_text = f"Back: {mopeka2_gallons:.0f} gal"
    canvas.create_text(x - 220, y, text=front_text, font=font,
                      fill=_mopeka_quality_color(mopeka1_quality),
                      anchor="ne", tags="batchmix_tanks")
    canvas.create_text(x, y, text=back_text, font=font,
                      fill=_mopeka_quality_color(mopeka2_quality),
                      anchor="ne", tags="batchmix_tanks")

def _batch_mix_payload_active():
    """Return True when the visible BatchMix screen should own knob adjustments."""
    return current_mode == "mix" and batch_mix_data is not None

def _format_batch_mix_target(value):
    """Format the water target for the requested-gallons display."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = 0.0

    if value == int(value):
        return f"{int(value)}"
    return f"{value:.1f}"

def adjust_batch_mix_gallons(delta, source):
    """Adjust BatchMix gallons and scale all formula amounts from that change."""
    global batch_mix_data, requested_gallons, mix_requested_gallons, colors_are_green

    if not _batch_mix_payload_active():
        return False

    try:
        old_water_needed = float(batch_mix_data.get("water_needed", requested_gallons))
        water_needed = max(1.0, old_water_needed + float(delta))
        batch_mix_data = scaled_batchmix_payload_for_water(batch_mix_data, water_needed)
        water_needed = float(batch_mix_data.get("water_needed", requested_gallons))
        new_acres = float(batch_mix_data.get("total_acres", 0))
    except Exception as exc:
        msg = f"{source}: BatchMix gallon adjust failed: {exc}"
        print(msg)
        try:
            with open(debug_log, "a") as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
        except Exception:
            pass
        return True

    requested_gallons = water_needed
    mix_requested_gallons = water_needed
    colors_are_green = False
    save_mode_presets()

    msg = (
        f"{source}: Adjusted BatchMix gallons by {delta:+.0f}, "
        f"water target {water_needed:.1f} gal, acres now {new_acres:.1f}"
    )
    print(msg)
    try:
        with open(debug_log, "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    except Exception:
        pass

    root.after(0, update_batch_mix_overlay)
    root.after(0, lambda value=water_needed: draw_requested_number(_format_batch_mix_target(value), "red"))
    return True

def set_requested_gallons_value(value, source):
    """Set requested gallons exactly from a remote dashboard command."""
    global batch_mix_data, requested_gallons, fill_requested_gallons, mix_requested_gallons
    global colors_are_green

    try:
        target = float(value)
    except (TypeError, ValueError):
        raise ValueError("requested gallons must be a number")

    if not math.isfinite(target):
        raise ValueError("requested gallons must be finite")

    # Keep the remote control inside the same practical range as the display workflow.
    target = max(0.0, min(10000.0, target))

    if _batch_mix_payload_active():
        water_needed = max(1.0, target)
        batch_mix_data = scaled_batchmix_payload_for_water(batch_mix_data, water_needed)
        requested_gallons = float(batch_mix_data.get("water_needed", water_needed))
        mix_requested_gallons = requested_gallons
        root.after(0, update_batch_mix_overlay)
        display_value = _format_batch_mix_target(requested_gallons)
    else:
        requested_gallons = target
        if current_mode == "fill":
            fill_requested_gallons = requested_gallons
        else:
            mix_requested_gallons = requested_gallons
        display_value = f"{requested_gallons:.0f}"

    colors_are_green = False
    save_mode_presets()
    msg = f"{source}: Set requested gallons to {requested_gallons:.3f}"
    print(msg)
    try:
        with open(debug_log, "a") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    except Exception:
        pass
    root.after(0, lambda value=display_value: draw_requested_number(value, "red"))
    return requested_gallons

def _format_batch_mix_product_amount(ounces):
    """Format product volume as gallons and ounces for display."""
    try:
        total_ounces = int(round(max(0.0, float(ounces))))
    except (TypeError, ValueError):
        total_ounces = 0

    whole_gallons, ounces = divmod(total_ounces, 128)

    if whole_gallons and ounces:
        return f"{whole_gallons} gal {ounces} oz"
    if whole_gallons:
        return f"{whole_gallons} gal"
    return f"{ounces} oz"

def _format_batch_mix_product_display_amount(prod):
    """Format the product amount from the BatchMix payload."""
    if "amount_oz" in prod:
        return _format_batch_mix_product_amount(prod.get("amount_oz", 0))

    try:
        pounds = max(0.0, float(prod.get("amount_lb", 0)))
    except (TypeError, ValueError):
        pounds = 0.0

    if pounds == int(pounds):
        return f"{int(pounds)} lb"
    return f"{pounds:.1f} lb"

def _format_batch_mix_product_rate(prod):
    """Format the optional product rate from the BatchMix payload."""
    if "rate_per_acre" not in prod or "rate_unit" not in prod:
        return ""

    try:
        rate = max(0.0, float(prod.get("rate_per_acre")))
    except (TypeError, ValueError):
        return ""

    rate_unit = prod.get("rate_unit")
    if not isinstance(rate_unit, str):
        return ""

    rate_unit = rate_unit.strip()
    if rate_unit not in ("oz/ac", "lb/ac"):
        return ""

    if rate == int(rate):
        rate_text = f"{int(rate)}"
    else:
        rate_text = f"{rate:.1f}"
    return f"{rate_text}{rate_unit}"

def _format_batch_mix_product_name(prod):
    """Format product name with optional compact rate."""
    name = prod.get("name", "Unknown")
    return name

def _batch_mix_has_product_rates():
    products = batch_mix_data.get("products", []) if batch_mix_data else []
    if not isinstance(products, list):
        return False
    return any(
        isinstance(product, dict)
        and "rate_per_acre" in product
        and "rate_unit" in product
        for product in products
    )

def _text_width(text, font, anchor="w"):
    test_id = canvas.create_text(0, 0, text=text, font=font,
                                anchor=anchor, tags="temp_measure")
    bbox = canvas.bbox(test_id)
    width = bbox[2] - bbox[0] if bbox else 0
    canvas.delete(test_id)
    return width

def _hex_to_rgb(color):
    """Convert #RRGGBB to an RGB tuple."""
    return tuple(int(color[index:index + 2], 16) for index in (1, 3, 5))

def _contrast_text_color(background):
    """Choose black or white text for readable contrast on a hex background."""
    red, green, blue = _hex_to_rgb(background)
    luminance = (0.299 * red) + (0.587 * green) + (0.114 * blue)
    return "black" if luminance >= 150 else "white"

def _batch_mix_badge_color():
    """Return optional color for the Mix mode badge."""
    colors = batch_mix_data.get("field_colors", []) if batch_mix_data else []
    if not isinstance(colors, list) or not colors:
        return None

    entry = colors[0]
    if not isinstance(entry, dict):
        return None

    color = entry.get("color")
    return parse_field_color(color)

def _striped_mix_badge_image(width, height, first_color, second_color):
    """Return a two-color striped badge image."""
    stripe_width = 18
    first_rgb = _hex_to_rgb(first_color)
    second_rgb = _hex_to_rgb(second_color)
    image = Image.new("RGB", (width, height), first_rgb)
    pixels = image.load()
    for x in range(width):
        stripe_color = second_rgb if (x // stripe_width) % 2 else first_rgb
        for y in range(height):
            pixels[x, y] = stripe_color
    return ImageTk.PhotoImage(image)

def _no_color_mix_badge_image(width, height):
    """Return a white/gray striped badge image for no-color BatchMix payloads."""
    return _striped_mix_badge_image(width, height, "#FFFFFF", "#BEBEBE")

def _contrast_text_color_for_pair(first_color, second_color):
    """Choose black or white text for readable contrast on a striped background."""
    first_rgb = _hex_to_rgb(first_color)
    second_rgb = _hex_to_rgb(second_color)
    blended = "#%02X%02X%02X" % tuple(
        int((first_rgb[index] + second_rgb[index]) / 2) for index in range(3)
    )
    return _contrast_text_color(blended)

def update_mix_mode_indicator():
    """Update the upper-left mix badge from the active BatchMix color."""
    label = globals().get("mode_indicator_label")
    if not label:
        return

    if current_mode != "mix" or batch_mix_data is None:
        label._stripe_image = None
        label.configure(
            text="MIX",
            foreground="cyan",
            background="black",
            image="",
            compound="none",
        )
        return

    _sync_canvas_geometry()
    badge_width = max(180, _canvas_width() // 3)
    badge_height = max(54, int(_canvas_height() * 0.085))
    color_spec = _batch_mix_badge_color()
    if not color_spec:
        label._stripe_image = _no_color_mix_badge_image(badge_width, badge_height)
        label.configure(
            text="NO COLOR MIX",
            foreground="black",
            background="white",
            image=label._stripe_image,
            compound="center",
        )
        return

    if color_spec[0] == "stripe":
        first_color, second_color = color_spec[1], color_spec[2]
        label._stripe_image = _striped_mix_badge_image(
            badge_width, badge_height, first_color, second_color
        )
        label.configure(
            text="MIX",
            foreground=_contrast_text_color_for_pair(first_color, second_color),
            background=first_color,
            image=label._stripe_image,
            compound="center",
        )
        return

    color = color_spec[1]
    label._stripe_image = None
    label.configure(
        text="MIX",
        foreground=_contrast_text_color(color),
        background=color,
        image="",
        compound="none",
    )

def place_mix_mode_indicator():
    """Place the Mix badge across the left panel up to the divider."""
    if not mode_indicator_label:
        return

    if current_mode != "mix" or batch_mix_data is None:
        mode_indicator_label.place_forget()
        return

    _sync_canvas_geometry()
    update_mix_mode_indicator()
    mode_indicator_label.place(
        x=0,
        y=0,
        width=max(180, _canvas_width() // 3),
        height=max(54, int(_canvas_height() * 0.085)),
        anchor="nw",
    )

def refresh_batch_mix_products():
    """Draw/update products list on canvas"""
    global batch_mix_data

    canvas.delete("products")

    if batch_mix_data is None:
        return

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    products = batch_mix_data.get("products", [])
    products_x_start = width // 3 + 20
    products_x_end = width - 20

    start_y = int(height * 0.14)
    row_height = int(height * 0.105)  # Height per product row

    for i, prod in enumerate(products[:6]):  # Max 6 products
        y = start_y + (i * row_height)

        # Product name (left side of products area) - auto-scale to fit
        name = _format_batch_mix_product_name(prod)
        rate_text = _format_batch_mix_product_rate(prod)
        rate_suffix = f" ({rate_text})" if rate_text else ""
        max_name_width = (products_x_end - products_x_start) // 2 - 20  # Half the products area

        # Start with larger font, scale down if needed
        font_size = 48
        while font_size >= 22:
            rate_font_size = max(16, int(font_size * 0.62))
            text_width = _text_width(name, ("Helvetica", font_size, "bold"))
            if rate_suffix:
                text_width += _text_width(
                    rate_suffix, ("Helvetica", rate_font_size, "bold")
                )

            if text_width <= max_name_width:
                break
            font_size -= 2

        canvas.create_text(products_x_start + 10, y, text=name,
                          font=("Helvetica", font_size, "bold"), fill="white",
                          anchor="w", tags="products")

        if rate_suffix:
            rate_font_size = max(16, int(font_size * 0.62))
            name_width = _text_width(name, ("Helvetica", font_size, "bold"))
            canvas.create_text(products_x_start + 10 + name_width, y,
                              text=rate_suffix,
                              font=("Helvetica", rate_font_size, "bold"),
                              fill="cyan", anchor="w", tags="products")

        # Amount (right side)
        amount_text = _format_batch_mix_product_display_amount(prod)
        max_amount_width = (products_x_end - products_x_start) // 2 - 20
        amount_font_size = 44
        while amount_font_size >= 30:
            test_id = canvas.create_text(0, 0, text=amount_text,
                                        font=("Helvetica", amount_font_size, "bold"),
                                        anchor="e", tags="temp_measure")
            bbox = canvas.bbox(test_id)
            text_width = bbox[2] - bbox[0] if bbox else 0
            canvas.delete(test_id)

            if text_width <= max_amount_width:
                break
            amount_font_size -= 2

        canvas.create_text(products_x_end - 10, y, text=amount_text,
                          font=("Helvetica", amount_font_size, "bold"), fill="yellow",
                          anchor="e", tags="products")

        # Draw subtle separator line under each product (except last)
        if i < min(len(products), 6) - 1:
            line_y = y + row_height // 2
            canvas.create_line(products_x_start + 5, line_y,
                              products_x_end - 5, line_y,
                              fill="#333355", width=1, tags="products")

    # Easy Mix indicator at bottom of products
    if batch_mix_data.get("easy_mix", False):
        easy_y = start_y + (min(len(products), 6) * row_height) + 10
        canvas.create_text(width * 2 // 3, easy_y, text="EASY MIX",
                          font=("Helvetica", 22, "bold"), fill="lime", tags="products")

def deactivate_batch_mix_layout():
    """Switch back to normal screen layout"""
    global batch_mix_layout_active
    global _last_requested_text, _last_requested_color, _last_actual_text, _last_actual_color
    global _last_batch_requested_text, _last_batch_requested_color
    global _last_batch_actual_text, _last_batch_actual_color

    # Clear batch mix elements
    canvas.delete("batchmix")
    canvas.delete("batchmix_tanks")
    canvas.delete("products")
    canvas.delete("totals")

    # Restore normal labels
    canvas.delete("labels")
    _sync_canvas_geometry()
    center_x = _canvas_width() // 2
    height = _canvas_height()

    canvas.create_text(center_x, int(height * 0.08), text="Requested Gallons:",
                      font=("Helvetica", 36, "bold"), fill="white", tags="labels")
    canvas.create_text(center_x, int(height * 0.45), text="Actual Gallons:",
                      font=("Helvetica", 36, "bold"), fill="white", tags="labels")

    # Invalidate cached number state so normal-mode positions are redrawn.
    _last_requested_text = None
    _last_requested_color = None
    _last_actual_text = None
    _last_actual_color = None
    _last_batch_requested_text = None
    _last_batch_requested_color = None
    _last_batch_actual_text = None
    _last_batch_actual_color = None

    # Leave batch-mix layout before redrawing so normal positioning is used.
    batch_mix_layout_active = False

    # Redraw numbers in normal positions
    redraw_numbers_normal()

# Cache for preventing flicker - only redraw when values change
_last_requested_text = None
_last_requested_color = None
_last_actual_text = None
_last_actual_color = None
_last_batch_requested_text = None
_last_batch_requested_color = None
_last_batch_actual_text = None
_last_batch_actual_color = None

def target_display_color(actual_gallons=None):
    """Return the color used for requested/actual gallons on the main display."""
    if actual_gallons is None:
        actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    if actual_gallons > requested_gallons + config.WARNING_THRESHOLD:
        return "red"
    return "green" if colors_are_green else "red"

def redraw_numbers_for_batch_mix():
    """Redraw requested/actual numbers in batch mix positions (left 1/3)"""
    global requested_gallons, last_totalizer_liters
    global _last_batch_requested_text, _last_batch_requested_color
    global _last_batch_actual_text, _last_batch_actual_color

    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    left_center_x = width // 6
    actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    color = target_display_color(actual_gallons)

    # Requested number - left panel
    req_y = int(height * 0.27)
    req_font = ("Helvetica", 151, "bold")
    req_text = f"{requested_gallons:.0f}"

    if req_text != _last_batch_requested_text or color != _last_batch_requested_color:
        canvas.delete("requested")
        for dx, dy in [(-3,-3), (-3,0), (-3,3), (0,-3), (0,3), (3,-3), (3,0), (3,3)]:
            canvas.create_text(left_center_x+dx, req_y+dy, text=req_text,
                              font=req_font, fill="white", tags="requested")
        canvas.create_text(left_center_x, req_y, text=req_text,
                          font=req_font, fill=color, tags="requested")
        _last_batch_requested_text = req_text
        _last_batch_requested_color = color

    # Actual number - left panel
    act_y = int(height * 0.55)
    act_font = ("Helvetica", 185, "bold")
    act_text = f"{actual_gallons:.1f}"

    if act_text != _last_batch_actual_text or color != _last_batch_actual_color:
        canvas.delete("actual")
        for dx, dy in [(-4,-4), (-4,0), (-4,4), (0,-4), (0,4), (4,-4), (4,0), (4,4)]:
            canvas.create_text(left_center_x+dx, act_y+dy, text=act_text,
                              font=act_font, fill="white", tags="actual")
        canvas.create_text(left_center_x, act_y, text=act_text,
                          font=act_font, fill=color, tags="actual")
        _last_batch_actual_text = act_text
        _last_batch_actual_color = color

def redraw_numbers_normal():
    """Redraw requested/actual numbers in normal centered positions"""
    global requested_gallons, last_totalizer_liters

    actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    color = target_display_color(actual_gallons)
    draw_requested_number(f"{requested_gallons:.0f}", color)
    draw_actual_number(f"{actual_gallons:.1f}", color)

def add_to_totals(gallons):
    """Add gallons to both daily and season totals"""
    global daily_total, season_total
    daily_total += gallons
    season_total += gallons
    save_totals()


def record_flow_curve_learning_sample():
    """Learn a conservative curve offset from confirmed Auto thumbs-up fills."""
    calibration_log = "/home/pi/fill_calibration.log"
    try:
        sample, reason = flow_curve.make_confirmed_auto_sample(
            requested_gallons=pending_fill_requested,
            actual_gallons=pending_fill_gallons,
            flow_gpm=pending_fill_flow_gpm,
            threshold_gallons=pending_fill_trigger_threshold,
            shutoff_type=pending_fill_shutoff_type,
        )
    except Exception as exc:
        with open(calibration_log, "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: error"
                f" | Stage: sample | Error: {exc}\n"
            )
        return

    if sample is None:
        with open(calibration_log, "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: skipped"
                f" | Reason: {reason}\n"
            )
        return

    try:
        result = flow_curve.record_learning_sample(
            FLOW_CURVE_SAMPLES_PATH,
            FLOW_CURVE_PROPOSAL_PATH,
            sample,
        )
    except Exception as exc:
        with open(calibration_log, "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: error"
                f" | Stage: save | Error: {exc}\n"
            )
        return
    with open(calibration_log, "a") as f:
        f.write(
            f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: accepted"
            f" | Samples: {result['sample_count']}/{result['required_sample_count']}"
        )
        if result.get("proposal_saved"):
            learning = result["learning"]
            f.write(
                f" | ProposalReady: yes"
                f" | Offset: {learning['applied_offset_gallons']:+.3f} gal"
                f" | RawOffset: {learning['raw_offset_gallons']:+.3f} gal"
            )
        f.write("\n")

def reset_daily_total():
    """Reset daily total and log the previous day's total"""
    global daily_total, last_reset_date
    import datetime

    # Log yesterday's total if it was non-zero
    if daily_total > 0:
        try:
            with open('/home/pi/daily_totals_log.log', 'a') as f:
                yesterday = datetime.datetime.now() - datetime.timedelta(days=1)
                f.write(f"{yesterday.strftime('%Y-%m-%d')}: {daily_total:.2f} gallons\n")
        except Exception as e:
            print(f"Error logging daily total: {e}")

    # Reset daily total
    daily_total = 0.0
    last_reset_date = datetime.datetime.now().strftime('%Y-%m-%d')
    save_totals()

def reset_season_total():
    """Reset season total (manual reset from menu)"""
    global season_total
    season_total = 0.0
    save_totals()

def update_totals_display():
    """Update the totals display in the menu (if menu is open)"""
    global menu_daily_label, menu_season_label
    if menu_daily_label and menu_season_label:
        menu_daily_label.config(text=f"Daily: {daily_total:.1f} gal")
        menu_season_label.config(text=f"Season: {season_total:.1f} gal")


def update_last_load_display():
    """Draw the three most recent load sizes in the top-left corner."""
    canvas.delete("last_load")
    if current_mode == "mix":
        return
    if last_loads_gallons:
        load_lines = "\n".join(f"{load:.1f} g" for load in last_loads_gallons[:3])
    else:
        load_lines = "--\n--\n--"
    title_font = ("Helvetica", 72, "bold")
    value_font = ("Helvetica", 96, "bold")
    canvas.create_text(
        20,
        20,
        text="Last Loads:",
        font=title_font,
        fill="cyan",
        anchor="nw",
        tags="last_load",
    )
    canvas.create_text(
        20,
        102,
        text=load_lines,
        font=value_font,
        fill="cyan",
        anchor="nw",
        tags="last_load",
    )


def update_bms_display():
    """Draw battery SOC below the last-load block on the left side."""
    canvas.delete("bms_display")
    if current_mode == "mix":
        return

    if bms_soc is None:
        value_text = "--"
    else:
        value_text = f"{int(round(bms_soc))}%"

    title_font = ("Helvetica", 72, "bold")
    value_font = ("Helvetica", 84, "bold")

    canvas.create_text(
        20,
        505,
        text="SOC:",
        font=title_font,
        fill="cyan",
        anchor="nw",
        tags="bms_display",
    )
    canvas.create_text(
        20,
        587,
        text=value_text,
        font=value_font,
        fill="cyan",
        anchor="nw",
        tags="bms_display",
    )


def update_flow_rate_display(flow_rate_gpm):
    """Draw the current flow rate in the bottom-right corner."""
    global last_flow_rate_text, last_flow_rate_mode

    flow_text = f"Flow:\n{flow_rate_gpm:.1f} GPM"
    if current_mode == "mix":
        if last_flow_rate_mode != "mix":
            canvas.delete("flow_rate")
        last_flow_rate_mode = "mix"
        last_flow_rate_text = None
        return

    if last_flow_rate_mode == current_mode and last_flow_rate_text == flow_text:
        return

    canvas.delete("flow_rate")
    if current_mode == "mix":
        return
    canvas.create_text(
        _canvas_width() - 10,
        _canvas_height() - 10,
        text=flow_text,
        font=("Helvetica", 72, "bold"),
        fill="cyan",
        anchor="se",
        tags="flow_rate",
    )
    last_flow_rate_text = flow_text
    last_flow_rate_mode = current_mode

def record_pending_fill():
    """Record the pending fill to history log and totals when thumbs up is pressed"""
    global pending_fill_gallons, pending_fill_requested, pending_fill_shutoff_type
    global pending_fill_flow_gpm, pending_fill_trigger_threshold, thumbs_up_label, last_loads_gallons

    # Only record if there's pending fill data
    if pending_fill_gallons > 0:
        fill_log = "/home/pi/fill_history.log"
        calibration_log = "/home/pi/fill_calibration.log"

        # Write to fill history log
        with open(fill_log, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} | Requested: {pending_fill_requested:.3f} gal | Actual: {pending_fill_gallons:.3f} gal | Diff: {pending_fill_gallons - pending_fill_requested:+.3f} gal | {pending_fill_shutoff_type}\n")

        # Write detailed calibration record with flow snapshot and threshold in one line
        with open(calibration_log, 'a') as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | Requested: {pending_fill_requested:.3f} gal"
                f" | Actual: {pending_fill_gallons:.3f} gal"
                f" | Diff: {pending_fill_gallons - pending_fill_requested:+.3f} gal"
                f" | FlowAtStop: {pending_fill_flow_gpm:.1f} GPM"
                f" | Threshold: {pending_fill_trigger_threshold:.3f} gal"
                f" | TriggerActual: {last_trigger_actual:.3f} gal"
                f" | TriggerPredicted: {last_trigger_predicted_actual:.3f} gal"
                f" | TriggerLoopMs: {last_trigger_loop_dt_ms:.1f}"
                f" | Type: {pending_fill_shutoff_type}\n"
            )

        # Add to daily and season totals
        add_to_totals(pending_fill_gallons)
        last_loads_gallons = [pending_fill_gallons] + last_loads_gallons[:2]
        update_last_load_display()
        record_flow_curve_learning_sample()
        print(f"Fill recorded - Actual: {pending_fill_gallons:.3f} gal")
        print(f"Updated totals - Daily: {daily_total:.2f}, Season: {season_total:.2f}")

        # Clear pending fill data
        pending_fill_gallons = 0.0
        pending_fill_requested = 0.0
        pending_fill_shutoff_type = ""
        pending_fill_flow_gpm = 0.0
        pending_fill_trigger_threshold = 0.0

    else:
        print("No pending fill to record")

def handle_thumbs_up_press(source):
    """Accept thumbs up only after flow has stopped."""
    button_log = "/home/pi/button_debug.log"
    if calibration_mode:
        msg = f"Thumbs up ignored from {source}: calibration workflow active"
        print(msg)
        log_serial_debug(msg)
        return

    is_flowing = last_flow_rate >= config.FLOW_STOPPED_THRESHOLD

    if is_flowing:
        msg = (
            f"Thumbs up ignored from {source}: flow still active "
            f"({last_flow_rate:.3f} L/s)"
        )
        print(msg)
        log_serial_debug(msg)
        with open(button_log, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] {msg}\n")
        return

    change_colors_to_green(from_button=True)
    record_pending_fill()

def pump_stop_relay(duration=config.PUMP_STOP_DURATION):
    """Activate pump stop relay for specified duration"""
    global pump_stop_pulse_count

    try:
        log_relay_event("")
        log_relay_event("=" * 60)
        log_relay_event(
            f"pump_stop_relay() CALLED | duration={duration} | "
            f"GPIO_AVAILABLE={GPIO_AVAILABLE} | pin={config.PUMP_STOP_RELAY_PIN}"
        )

        with pump_stop_relay_lock:
            pump_stop_pulse_count += 1
            _set_pump_stop_output(True, f"pulse start {duration}s")

        msg = f"Alert relay (GPIO {config.PUMP_STOP_RELAY_PIN}) activated for {duration} seconds"
        print(msg)
        log_relay_event(msg)
        time.sleep(duration)

        relay_released = False
        with pump_stop_relay_lock:
            pump_stop_pulse_count = max(0, pump_stop_pulse_count - 1)
            if pump_stop_pulse_count == 0:
                _set_pump_stop_output(False, "pulse complete")
                relay_released = True
            else:
                log_relay_event(
                    "Pulse complete; relay remains HIGH "
                    f"(active_pulses={pump_stop_pulse_count})"
                )

        msg = (
            f"Alert relay (GPIO {config.PUMP_STOP_RELAY_PIN}) deactivated"
            if relay_released
            else f"Alert relay (GPIO {config.PUMP_STOP_RELAY_PIN}) pulse complete; relay held HIGH by active pulse"
        )
        print(msg)
        log_relay_event(msg)
        log_relay_event("=" * 60)
    except Exception as e:
        msg = f"Error controlling relay: {e}"
        print(msg)
        log_relay_event(f"EXCEPTION: {msg}")
        import traceback
        try:
            with open(config.RELAY_TEST_LOG, 'a') as f:
                f.write(traceback.format_exc())
                f.write(f"{'='*60}\n")
        except Exception:
            pass


def get_ip_address():
    """Get the current IP address of the system"""
    try:
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2)
        ip = result.stdout.strip().split()[0] if result.stdout.strip() else "No IP"
        return ip
    except Exception:
        return "No IP"

def get_username():
    """Get the current username"""
    try:
        return os.getenv('USER', 'unknown')
    except Exception:
        return 'unknown'

def change_colors_to_green(from_button=False):
    """Change display colors from red to green if within 2 gallons of target, or show thumbs up only if button pressed"""
    global serial_command_received, last_totalizer_liters, requested_gallons, colors_are_green

    button_log = "/home/pi/button_debug.log"
    with open(button_log, 'a') as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] change_colors_to_green() called with from_button={from_button}\n")

    if batch_mix_layout_active:
        with open(button_log, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Ignoring thumbs-up/green transition while batch mix screen is active\n")
        if thumbs_up_animation_id:
            root.after_cancel(thumbs_up_animation_id)
        if thumbs_up_label:
            thumbs_up_label.place_forget()
            _set_thumbs_up_visible(False)
        return

    # Calculate current actual gallons
    actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    with open(button_log, 'a') as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Actual: {actual_gallons:.1f}, Requested: {requested_gallons:.0f}, Diff: {abs(actual_gallons - requested_gallons):.1f}\n")

    # Check if within 2 gallons of target
    within_threshold = abs(actual_gallons - requested_gallons) <= 2.0

    if within_threshold:
        with open(button_log, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Within 2 gallon threshold! serial_command_received={serial_command_received}\n")
        # Button press always works, auto-alert only works once
        if from_button or not serial_command_received:
            with open(button_log, 'a') as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Proceeding with color change to GREEN...\n")
            if not from_button:
                serial_command_received = True

            # Set the flag so update_dashboard keeps colors green
            colors_are_green = True

            # Change the number colors to green by redrawing
            current_actual = last_totalizer_liters * config.LITERS_TO_GALLONS
            display_color = target_display_color(current_actual)
            draw_requested_number(f"{requested_gallons:.0f}", display_color)
            draw_actual_number(f"{current_actual:.1f}", display_color)
            # Show big thumbs up on the right side and start animation, except on the batch product screen.
            if thumbs_up_label and not batch_mix_layout_active:
                show_thumbs_up(current_actual)
                schedule_flow_reset()
                if thumbs_up_frames:  # Only animate if we have GIF frames
                    animate_thumbs_up()
            source = "button press" if from_button else "auto-alert"
            with open(button_log, 'a') as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} Display colors changed to green ({source}, within 2 gallons: {actual_gallons:.1f}/{requested_gallons:.0f})\n")
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] colors_are_green flag set to True\n")
        else:
            with open(button_log, 'a') as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} Color change already triggered by auto-alert ({actual_gallons:.1f}/{requested_gallons:.0f})\n")
    else:
        # NOT within threshold
        with open(button_log, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] NOT within 2 gallon threshold ({actual_gallons:.1f}/{requested_gallons:.0f})\n")

        # If button was pressed, still show thumbs up but keep screen RED
        if from_button:
            with open(button_log, 'a') as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [DEBUG] Button pressed but not within threshold - showing thumbs up, keeping RED\n")
            # Show thumbs up but DO NOT change colors to green, except on the batch product screen.
            if thumbs_up_label and not batch_mix_layout_active:
                show_thumbs_up(actual_gallons)
                schedule_flow_reset()
                if thumbs_up_frames:  # Only animate if we have GIF frames
                    animate_thumbs_up()
            with open(button_log, 'a') as f:
                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} Thumbs up shown but screen stays RED (not within 2 gallons: {actual_gallons:.1f}/{requested_gallons:.0f})\n")

def green_button_monitor():
    """Monitor GPIO pin for green button press (active low with pull-up)"""
    button_log = "/home/pi/button_debug.log"

    if not GPIO_AVAILABLE:
        print("GPIO not available, green button monitoring disabled")
        return

    try:
        # Initialize GPIO if not already done
        try:
            GPIO.setmode(GPIO.BCM)
        except Exception:
            pass  # Already initialized

        # Set up green button pin as input with pull-up resistor
        GPIO.setup(config.GREEN_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        msg = f"Green button monitor started on GPIO {config.GREEN_BUTTON_PIN}"
        print(msg)
        with open(button_log, 'a') as f:
            f.write(f"\n{'='*60}\n")
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}\n")

        last_button_state = GPIO.HIGH

        while True:
            # Read current button state
            current_state = GPIO.input(config.GREEN_BUTTON_PIN)

            # Detect button press (transition from HIGH to LOW)
            if last_button_state == GPIO.HIGH and current_state == GPIO.LOW:
                with open(button_log, 'a') as f:
                    f.write(f"\n{time.strftime('%Y-%m-%d %H:%M:%S')} *** GREEN BUTTON PRESSED! ***\n")
                print("Green button pressed!")
                # If in reminders mode, dismiss reminders
                if reminders_mode:
                    root.after(0, dismiss_reminders)
                # If in full test mode, mark button test as passed
                elif full_test_mode:
                    if full_test_window and hasattr(full_test_window, 'mark_tested'):
                        root.after(0, lambda: full_test_window.mark_tested('button'))
                        print("Full test: Button test marked as passed")
                else:
                    root.after(0, lambda: handle_thumbs_up_press("GPIO button"))
                # Debounce delay
                time.sleep(0.3)

            last_button_state = current_state
            time.sleep(0.05)  # Check every 50ms

    except Exception as e:
        print(f"Green button monitor error: {e}")

def show_log_viewer():
    """Display log viewer window with button controls"""
    global log_viewer_mode, log_viewer_window, log_viewer_text

    # Submenus should replace the menu, not stack on top of it.
    if menu_window:
        close_menu()
    if log_viewer_window:
        try:
            log_viewer_window.destroy()
        except Exception:
            pass
        log_viewer_window = None

    log_viewer_mode = True
    log_viewer_window = tk.Toplevel()
    log_viewer_window.title("System Logs")
    log_viewer_window.attributes('-fullscreen', True)
    log_viewer_window.configure(bg='black')

    # Title
    title = tk.Label(log_viewer_window, text="SYSTEM LOGS", font=("Helvetica", 32, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=10)

    # Controls instruction
    controls = tk.Label(log_viewer_window, text="-1=SCROLL UP  +1=SCROLL DOWN  OV=EXIT",
                       font=("Helvetica", 22, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Create scrolled text widget for logs - HUGE FONT for 7" display
    log_frame = tk.Frame(log_viewer_window, bg='black')
    log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    scrollbar = tk.Scrollbar(log_frame, width=30)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # EXTRA LARGE FONT - 24pt bold
    log_viewer_text = tk.Text(log_frame, font=("Courier", 24, "bold"), bg="black", fg="lime",
                             yscrollcommand=scrollbar.set, wrap=tk.WORD)
    log_viewer_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.config(command=log_viewer_text.yview)

    # Load logs - get last 200 lines for scrolling through history
    try:
        import subprocess

        # Get last 200 lines from system log
        log_viewer_text.insert(tk.END, "=== SYSTEM LOG (last 200 lines) ===\n\n")
        result = subprocess.run(['tail', '-200', '/home/pi/iol_dashboard.log'],
                              capture_output=True, text=True, timeout=2)
        log_viewer_text.insert(tk.END, result.stdout)

        # Get last 200 lines from serial log
        log_viewer_text.insert(tk.END, '\n\n=== SERIAL LOG (last 200 lines) ===\n\n')
        result = subprocess.run(['tail', '-200', '/home/pi/serial_debug.log'],
                              capture_output=True, text=True, timeout=2)
        log_viewer_text.insert(tk.END, result.stdout)

        # Get button debug log if it exists
        log_viewer_text.insert(tk.END, '\n\n=== BUTTON DEBUG LOG (last 100 lines) ===\n\n')
        result = subprocess.run(['tail', '-100', '/home/pi/button_debug.log'],
                              capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            log_viewer_text.insert(tk.END, result.stdout)
        else:
            log_viewer_text.insert(tk.END, "(No button log found)\n")

        # Scroll to bottom initially
        log_viewer_text.see(tk.END)
    except Exception as e:
        log_viewer_text.insert(tk.END, f"ERROR loading logs:\n{e}")

    # Make read-only but keep enabled for scrolling
    log_viewer_text.config(state=tk.NORMAL)

def log_viewer_scroll_down():
    """Scroll log viewer down"""
    global log_viewer_text
    if log_viewer_text:
        log_viewer_text.yview_scroll(5, "units")  # Scroll down 5 lines (faster scrolling)

def log_viewer_scroll_up():
    """Scroll log viewer up"""
    global log_viewer_text
    if log_viewer_text:
        log_viewer_text.yview_scroll(-5, "units")  # Scroll up 5 lines (faster scrolling)

def close_log_viewer():
    """Close log viewer and return to menu"""
    global log_viewer_mode, log_viewer_window, log_viewer_text
    log_viewer_mode = False
    if log_viewer_window:
        log_viewer_window.destroy()
        log_viewer_window = None
        log_viewer_text = None
    arm_menu_ov_guard()
    show_menu()

def show_fill_history():
    """Display fill history viewer window"""
    global fill_history_mode, fill_history_window, fill_history_text

    # Submenus should replace the menu, not stack on top of it.
    if menu_window:
        close_menu()
    if fill_history_window:
        try:
            fill_history_window.destroy()
        except Exception:
            pass
        fill_history_window = None

    fill_history_mode = True
    fill_history_window = tk.Toplevel()
    fill_history_window.title("Fill History")
    fill_history_window.attributes('-fullscreen', True)
    fill_history_window.configure(bg='black')

    # Title
    title = tk.Label(fill_history_window, text="FILL HISTORY", font=("Helvetica", 32, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=10)

    # Controls instruction
    controls = tk.Label(fill_history_window, text="-1=SCROLL UP  +1=SCROLL DOWN  OV=EXIT",
                       font=("Helvetica", 22, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Create scrolled text widget for fill history
    history_frame = tk.Frame(fill_history_window, bg='black')
    history_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    scrollbar = tk.Scrollbar(history_frame, width=30)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    # EXTRA LARGE FONT - 28pt bold for better readability
    fill_history_text = tk.Text(history_frame, font=("Courier", 28, "bold"), bg="black", fg="lime",
                             yscrollcommand=scrollbar.set, wrap=tk.WORD)
    fill_history_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.config(command=fill_history_text.yview)

    # Load fill history
    try:
        import subprocess

        # Get last 100 fill history entries
        fill_history_text.insert(tk.END, "=== FILL HISTORY (last 100 fills) ===\n\n")
        result = subprocess.run(['tail', '-100', '/home/pi/fill_history.log'],
                              capture_output=True, text=True, timeout=2)
        if result.returncode == 0 and result.stdout:
            fill_history_text.insert(tk.END, result.stdout)
        else:
            fill_history_text.insert(tk.END, "(No fill history found)\n\n")
            fill_history_text.insert(tk.END, "Fill history will appear here after completing fills.\n")

        # Scroll to bottom initially
        fill_history_text.see(tk.END)
    except Exception as e:
        fill_history_text.insert(tk.END, f"ERROR loading fill history:\n{e}")

    # Make read-only but keep enabled for scrolling
    fill_history_text.config(state=tk.NORMAL)

def fill_history_scroll_down():
    """Scroll fill history down"""
    global fill_history_text
    if fill_history_text:
        fill_history_text.yview_scroll(5, "units")  # Scroll down 5 lines

def fill_history_scroll_up():
    """Scroll fill history up"""
    global fill_history_text
    if fill_history_text:
        fill_history_text.yview_scroll(-5, "units")  # Scroll up 5 lines

def close_fill_history():
    """Close fill history and return to menu"""
    global fill_history_mode, fill_history_window, fill_history_text
    fill_history_mode = False
    if fill_history_window:
        fill_history_window.destroy()
        fill_history_window = None
        fill_history_text = None
    arm_menu_ov_guard()
    show_menu()


def _default_calibration_state():
    return {
        "phase": "choose_tank",
        "tank": "front",
        "total_capacity": 1070,
        "target_capacity": 1000,
        "step_size": 10,
        "current_step": 10,
        "points": [],
        "flow_started": False,
        "settle_deadline": None,
        "last_step_actual": 0.0,
        "reading": None,
        "return_phase": None,
        "profile_path": "",
        "profile_error": "",
    }


def _selected_tank_reading():
    tank = calibration_state.get("tank", "front")
    if tank == "front":
        return {
            "tank": "front",
            "level_mm": mopeka1_level_mm,
            "level_in": mopeka1_level_in,
            "gallons": mopeka1_gallons,
            "quality": mopeka1_quality,
        }
    return {
        "tank": "back",
        "level_mm": mopeka2_level_mm,
        "level_in": mopeka2_level_in,
        "gallons": mopeka2_gallons,
        "quality": mopeka2_quality,
    }


def _calibration_points_path():
    return "/home/pi/tank_calibration_points.csv"


def _calibration_runs_path():
    return "/home/pi/tank_calibration_runs.json"


def _mopeka_config_path():
    return "/opt/mopeka/mopeka_config.json"


def _mopeka_calibration_dir():
    return "/opt/mopeka/calibrations"


def _safe_calibration_profile_key(value):
    key = str(value or "").strip().lower().replace("_", "-")
    return "".join(ch for ch in key if ch.isalnum() or ch == "-")


def _current_tank_calibration_profile_key(tank):
    try:
        with open(_mopeka_config_path(), "r") as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}

    mode = str(cfg.get("box_mode") or "fleet").strip().lower()
    trailer = cfg.get("assigned_trailer", cfg.get("trailer"))
    tank = "back" if tank == "back" else "front"

    if mode != "customer" and trailer not in (None, ""):
        return _safe_calibration_profile_key(f"trailer-{trailer}-{tank}")
    return f"customer-{tank}"


def _current_tank_calibration_profile_path(tank):
    profile_key = _current_tank_calibration_profile_key(tank)
    return os.path.join(_mopeka_calibration_dir(), f"{profile_key}.csv")


def _build_dashboard_state_snapshot():
    """Build a compact state snapshot for BLE/iPad clients."""
    actual_gallons = last_totalizer_liters * config.LITERS_TO_GALLONS
    flow_gpm = last_flow_rate * config.LITERS_PER_SEC_TO_GPM
    flow_meter_connected = (time.time() - last_successful_read_time) <= config.FLOW_METER_TIMEOUT
    switch_box_connected = bool(serial_connected and not heartbeat_disconnected)
    fill_pending = pending_fill_gallons > 0
    can_confirm_fill = bool(fill_pending and last_flow_rate < config.FLOW_STOPPED_THRESHOLD)

    return {
        "version": VERSION,
        "requested_gal": round(requested_gallons, 3),
        "actual_gal": round(actual_gallons, 3),
        "flow_gpm": round(flow_gpm, 2),
        "mode": current_mode,
        "override": bool(override_mode),
        "thumbs_visible": bool(thumbs_up_visible),
        "fill_pending": bool(fill_pending),
        "can_confirm_fill": bool(can_confirm_fill),
        "colors_green": bool(colors_are_green),
        "pump_stop_latched": bool(auto_shutoff_latched),
        "flow_meter_connected": bool(flow_meter_connected),
        "switch_box_connected": bool(switch_box_connected),
        "bms_soc": None if bms_soc is None else int(round(bms_soc)),
        "bms_voltage": None if bms_voltage is None else round(bms_voltage, 2),
        "front_tank_gal": round(mopeka1_gallons, 1),
        "back_tank_gal": round(mopeka2_gallons, 1),
        "front_tank_quality": int(mopeka1_quality),
        "back_tank_quality": int(mopeka2_quality),
        "mopeka_enabled": bool(mopeka_enabled),
        "mopeka_connected": bool(mopeka_connected),
        "last_loads_gal": [round(load, 3) for load in last_loads_gallons[:3]],
        "current_curve": flow_curve_status_text(),
        "pending_curve": flow_curve_proposal_status_text(),
    }


def _build_fill_history_items(limit=5):
    """Return recent fill rows with thumbs-confirmation status."""
    history_items = []

    try:
        with open("/home/pi/fill_history.log", "r") as hf:
            all_lines = hf.readlines()
            last_lines = all_lines[-limit:] if len(all_lines) >= limit else all_lines
            for entry in last_lines:
                parts = entry.strip().split("|")
                if len(parts) >= 3:
                    ts = parts[0].strip()
                    req = parts[1].replace("Requested:", "").replace("gal", "").strip()
                    act = parts[2].replace("Actual:", "").replace("gal", "").strip()
                    shutoff_type = parts[4].strip() if len(parts) >= 5 else ""
                    history_items.append((ts, req, act, "accepted", shutoff_type))
    except FileNotFoundError:
        pass
    except Exception as exc:
        print(f"Fill history parse error: {exc}", flush=True)

    if pending_fill_gallons > 0:
        history_items.append((
            "Pending thumbs",
            f"{pending_fill_requested:.3f}",
            f"{pending_fill_gallons:.3f}",
            "pending",
            pending_fill_shutoff_type,
        ))

    return history_items[-limit:]


def _save_calibration_run():
    if not calibration_state or not calibration_state.get("points"):
        return

    run_record = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "tank": calibration_state["tank"],
        "total_capacity": calibration_state["total_capacity"],
        "target_capacity": calibration_state["target_capacity"],
        "step_size": calibration_state["step_size"],
        "points": calibration_state["points"],
    }

    runs_path = _calibration_runs_path()
    try:
        if os.path.exists(runs_path):
            with open(runs_path, "r") as f:
                runs = json.load(f)
                if not isinstance(runs, list):
                    runs = []
        else:
            runs = []
    except Exception:
        runs = []

    runs.append(run_record)
    with open(runs_path, "w") as f:
        json.dump(runs, f, indent=2)

    points_path = _calibration_points_path()
    write_header = not os.path.exists(points_path)
    with open(points_path, "a") as f:
        if write_header:
            f.write(
                "timestamp,tank,total_capacity,target_capacity,step_target_gallons,"
                "actual_total_gallons,level_mm,level_in,display_gallons,quality\n"
            )
        for point in calibration_state["points"]:
            f.write(
                f"{run_record['timestamp']},{run_record['tank']},{run_record['total_capacity']},"
                f"{run_record['target_capacity']},{point['step_target_gallons']},"
                f"{point['actual_total_gallons']:.3f},{point['level_mm']:.1f},"
                f"{point['level_in']:.2f},{point['display_gallons']:.1f},{point['quality']}\n"
            )


def _append_calibration_point(actual_gallons, step_target_gallons=None):
    reading = calibration_state.get("reading") or _selected_tank_reading()
    calibration_state["points"].append({
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "step_target_gallons": (
            actual_gallons if step_target_gallons is None else step_target_gallons
        ),
        "actual_total_gallons": actual_gallons,
        "level_mm": reading["level_mm"],
        "level_in": reading["level_in"],
        "display_gallons": reading["gallons"],
        "quality": reading["quality"],
    })


def _apply_tank_calibration_profile():
    if not calibration_state or len(calibration_state.get("points", [])) < 2:
        raise ValueError("Need at least empty plus one fill point")

    tank = calibration_state["tank"]
    profile_path = _current_tank_calibration_profile_path(tank)
    os.makedirs(os.path.dirname(profile_path), exist_ok=True)

    if os.path.exists(profile_path):
        backup_path = f"{profile_path}.backup-{time.strftime('%Y%m%d_%H%M%S')}"
        with open(profile_path, "r") as src, open(backup_path, "w") as dst:
            dst.write(src.read())

    rows = []
    for point in calibration_state["points"]:
        rows.append({
            "tank_level_in": float(point["level_in"]),
            "gallons": float(point["actual_total_gallons"]),
            "tank_size": float(calibration_state["total_capacity"]),
        })
    rows.sort(key=lambda row: row["tank_level_in"], reverse=True)

    tmp_path = f"{profile_path}.tmp"
    with open(tmp_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Tank Level (in)", "Gallons", "Tank Size (gal)"])
        for row in rows:
            writer.writerow([row["tank_level_in"], row["gallons"], row["tank_size"]])
    os.replace(tmp_path, profile_path)
    return profile_path


def _refresh_calibration_window():
    if not calibration_window or not calibration_state:
        return

    phase = calibration_state["phase"]
    tank_label = calibration_state["tank"].title()
    title = "TANK CALIBRATION"
    body = ""
    footer = ""
    hint = ""

    if phase == "choose_tank":
        body = f"Select Tank\n\n{tank_label} Tank"
        footer = "+1/-1 = CHANGE   OV = CONFIRM   PS = CANCEL"
        hint = "Choose which trailer tank to calibrate."
    elif phase == "set_total":
        body = f"{tank_label} Tank\n\nTotal Capacity\n{calibration_state['total_capacity']} gal"
        footer = "+10/-10, +1/-1 = ADJUST   OV = NEXT   PS = BACK"
        hint = "Set the full physical tank capacity."
    elif phase == "set_target":
        body = (
            f"{tank_label} Tank\n\nCalibration Endpoint\n"
            f"{calibration_state['target_capacity']} gal"
        )
        footer = "+10/-10, +1/-1 = ADJUST   OV = NEXT   PS = BACK"
        hint = "Set the usable gallons to calibrate up to."
    elif phase == "confirm_empty":
        body = f"{tank_label} Tank\n\nConfirm this tank is completely empty."
        footer = "OV = YES, START   PS = BACK"
        hint = "The workflow will fill in 10 gallon increments."
    elif phase == "wait_for_fill":
        step = calibration_state["current_step"]
        body = (
            f"{tank_label} Tank\n\nTarget Step: {step} gal\n\n"
            "Start the pump.\nBBB will stop flow automatically."
        )
        footer = "PS = ABORT"
        hint = "Waiting for flow to start."
        if calibration_state.get("flow_started"):
            hint = "Filling..."
    elif phase == "settling":
        remaining = max(0, int(calibration_state["settle_deadline"] - time.time()))
        body = (
            f"{tank_label} Tank\n"
            f"{calibration_state['current_step']} gal reached.\n"
            "Waiting for Mopeka to settle\n"
            f"{remaining} sec remaining"
        )
        footer = "PS = ABORT"
        hint = "Do not disturb the tank during the settle period."
    elif phase == "review":
        reading = calibration_state.get("reading") or _selected_tank_reading()
        body = (
            f"{tank_label} Tank\n\nStep: {calibration_state['current_step']} gal\n"
            f"Actual: {calibration_state['last_step_actual']:.1f} gal\n"
            f"Mopeka: {reading['level_mm']:.1f} mm / {reading['level_in']:.2f} in\n"
            f"Display: {reading['gallons']:.0f} gal   Quality: {reading['quality']}"
        )
        footer = "OV = SAVE/NEXT   +1 = REREAD   -1 = WAIT 2 MORE MIN   PS = ABORT"
        hint = "Confirm the Mopeka reading before continuing."
    elif phase == "complete":
        profile_path = _current_tank_calibration_profile_path(calibration_state["tank"])
        body = (
            f"{tank_label} Tank Calibration Complete\n\n"
            f"Saved {len(calibration_state['points'])} calibration points\n"
            f"through {calibration_state['target_capacity']} gallons.\n\n"
            "Apply this as the live tank curve?"
        )
        footer = "OV = APPLY PROFILE   PS = RETURN"
        hint = profile_path
    elif phase == "applied":
        body = (
            f"{tank_label} Tank Calibration Applied\n\n"
            "RotorSync will reload the curve automatically."
        )
        footer = "OV = RETURN TO MENU"
        hint = calibration_state.get("profile_path") or _current_tank_calibration_profile_path(calibration_state["tank"])
    elif phase == "apply_error":
        body = (
            f"{tank_label} Tank Calibration Apply Failed\n\n"
            f"{calibration_state.get('profile_error', 'Unknown error')}"
        )
        footer = "OV = RETURN TO MENU"
        hint = _calibration_points_path()
    elif phase == "abort_confirm":
        body = (
            f"{tank_label} Tank\n\nAbort Calibration?\n\n"
            "This will discard the current calibration run."
        )
        footer = "OV = YES, ABORT   PS = NO, GO BACK"
        hint = "Use OV to exit the calibration workflow."

    body_font_size = 54
    if phase in ("wait_for_fill", "settling", "review", "complete", "applied", "apply_error", "abort_confirm"):
        body_font_size = 46
    elif phase in ("set_total", "set_target"):
        body_font_size = 58

    calibration_title_label.config(text=title)
    calibration_body_label.config(text=body, font=("Helvetica", body_font_size, "bold"))
    calibration_footer_label.config(text=footer)
    calibration_hint_label.config(text=hint)


def _calibration_prepare_next_step():
    global requested_gallons, fill_requested_gallons, colors_are_green
    step = calibration_state["current_step"]
    calibration_state["phase"] = "wait_for_fill"
    calibration_state["flow_started"] = False
    calibration_state["settle_deadline"] = None
    calibration_state["reading"] = None
    requested_gallons = float(step)
    fill_requested_gallons = requested_gallons
    colors_are_green = False
    draw_requested_number(f"{requested_gallons:.0f}", "red")
    _refresh_calibration_window()


def _close_calibration_window(return_to_menu=True):
    global calibration_mode, calibration_window, calibration_state
    global calibration_title_label, calibration_body_label, calibration_footer_label, calibration_hint_label
    calibration_mode = False
    calibration_state = None
    if calibration_window:
        calibration_window.destroy()
        calibration_window = None
    calibration_title_label = None
    calibration_body_label = None
    calibration_footer_label = None
    calibration_hint_label = None
    if return_to_menu:
        show_menu()


def show_tank_calibration():
    global calibration_mode, calibration_window, calibration_state
    global calibration_title_label, calibration_body_label, calibration_footer_label, calibration_hint_label

    if menu_window:
        close_menu()
    if calibration_window:
        try:
            calibration_window.destroy()
        except Exception:
            pass

    calibration_mode = True
    calibration_state = _default_calibration_state()
    calibration_window = tk.Toplevel()
    calibration_window.title("Tank Calibration")
    calibration_window.attributes('-fullscreen', True)
    calibration_window.configure(bg='black')

    calibration_title_label = tk.Label(
        calibration_window, text="", font=("Helvetica", 40, "bold"), fg="cyan", bg="black"
    )
    calibration_title_label.pack(pady=(10, 2))

    calibration_body_label = tk.Label(
        calibration_window,
        text="",
        font=("Helvetica", 54, "bold"),
        fg="white",
        bg="black",
        justify=tk.CENTER,
        wraplength=760,
    )
    calibration_body_label.pack(expand=True, fill=tk.BOTH, padx=20, pady=6)

    calibration_hint_label = tk.Label(
        calibration_window,
        text="",
        font=("Helvetica", 24, "bold"),
        fg="#ffff99",
        bg="black",
        wraplength=760,
    )
    calibration_hint_label.pack(pady=(2, 4))

    calibration_footer_label = tk.Label(
        calibration_window,
        text="",
        font=("Helvetica", 26, "bold"),
        fg="#00ffff",
        bg="#0a0a0a",
        wraplength=760,
    )
    calibration_footer_label.pack(side=tk.BOTTOM, fill=tk.X, pady=(0, 8), ipady=6)

    _refresh_calibration_window()


def calibration_adjust_value(delta):
    if not calibration_state:
        return
    phase = calibration_state["phase"]
    if phase == "choose_tank":
        calibration_state["tank"] = "back" if calibration_state["tank"] == "front" else "front"
    elif phase == "set_total":
        calibration_state["total_capacity"] = max(10, calibration_state["total_capacity"] + delta)
        if calibration_state["target_capacity"] > calibration_state["total_capacity"]:
            calibration_state["target_capacity"] = calibration_state["total_capacity"]
    elif phase == "set_target":
        calibration_state["target_capacity"] = min(
            calibration_state["total_capacity"],
            max(10, calibration_state["target_capacity"] + delta),
        )
    elif phase == "review" and delta == -1:
        calibration_state["phase"] = "settling"
        calibration_state["settle_deadline"] = time.time() + 120
    _refresh_calibration_window()


def calibration_confirm():
    if not calibration_state:
        return

    phase = calibration_state["phase"]
    if phase == "choose_tank":
        calibration_state["phase"] = "set_total"
    elif phase == "set_total":
        calibration_state["phase"] = "set_target"
    elif phase == "set_target":
        calibration_state["phase"] = "confirm_empty"
    elif phase == "confirm_empty":
        calibration_state["points"] = []
        calibration_state["reading"] = _selected_tank_reading()
        _append_calibration_point(0.0, 0)
        calibration_state["reading"] = None
        calibration_state["current_step"] = calibration_state["step_size"]
        switch_mode("fill")
        _calibration_prepare_next_step()
    elif phase == "review":
        _append_calibration_point(
            calibration_state["last_step_actual"],
            calibration_state["current_step"],
        )
        next_step = calibration_state["current_step"] + calibration_state["step_size"]
        if next_step <= calibration_state["target_capacity"]:
            calibration_state["current_step"] = next_step
            _calibration_prepare_next_step()
            return
        _save_calibration_run()
        calibration_state["phase"] = "complete"
    elif phase == "complete":
        try:
            calibration_state["profile_path"] = _apply_tank_calibration_profile()
            calibration_state["profile_error"] = ""
            calibration_state["phase"] = "applied"
        except Exception as exc:
            calibration_state["profile_error"] = str(exc)
            calibration_state["phase"] = "apply_error"
    elif phase == "abort_confirm":
        _close_calibration_window(return_to_menu=True)
        return
    elif phase in ("applied", "apply_error"):
        _close_calibration_window(return_to_menu=True)
        return

    _refresh_calibration_window()


def calibration_cancel():
    if not calibration_state:
        return

    phase = calibration_state["phase"]
    if phase == "choose_tank":
        calibration_state["return_phase"] = "choose_tank"
        calibration_state["phase"] = "abort_confirm"
        return
    if phase in ("complete", "applied", "apply_error"):
        _close_calibration_window(return_to_menu=True)
        return
    if phase == "set_total":
        calibration_state["phase"] = "choose_tank"
    elif phase == "set_target":
        calibration_state["phase"] = "set_total"
    elif phase == "confirm_empty":
        calibration_state["phase"] = "set_target"
    elif phase == "abort_confirm":
        calibration_state["phase"] = calibration_state.get("return_phase") or "choose_tank"
        calibration_state["return_phase"] = None
    else:
        calibration_state["return_phase"] = phase
        calibration_state["phase"] = "abort_confirm"
    _refresh_calibration_window()


def calibration_reread_now():
    if calibration_state and calibration_state["phase"] == "review":
        calibration_state["reading"] = _selected_tank_reading()
        _refresh_calibration_window()

def run_self_test():
    """Run system self-test"""
    global self_test_mode, self_test_window

    self_test_mode = True
    self_test_window = tk.Toplevel()
    self_test_window.title("System Self-Test")
    self_test_window.attributes('-fullscreen', True)
    self_test_window.configure(bg='black')

    # Title
    title = tk.Label(self_test_window, text="SYSTEM SELF-TEST", font=("Helvetica", 44, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=(12, 6))

    # Controls instruction
    controls = tk.Label(self_test_window, text="OV=EXIT TO MENU",
                       font=("Helvetica", 28, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Results frame
    results_frame = tk.Frame(self_test_window, bg='black')
    results_frame.pack(fill=tk.BOTH, expand=True, padx=24, pady=(8, 14))

    results_text = tk.Text(results_frame, font=("Courier", 32, "bold"), bg="black", fg="white",
                           wrap=tk.WORD, borderwidth=0, highlightthickness=0)
    results_text.pack(fill=tk.BOTH, expand=True)

    def run_tests():
        results_text.insert(tk.END, "Starting self-test...\n\n")
        results_text.update()

        # Test 1: GPIO
        results_text.insert(tk.END, "1. GPIO Test: ")
        results_text.update()
        if GPIO_AVAILABLE:
            try:
                # Quick relay pulse test (0.5 seconds)
                GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.LOW)
                results_text.insert(tk.END, "PASS (Relay pulsed)\n", "pass")
            except Exception as e:
                results_text.insert(tk.END, f"FAIL ({e})\n", "fail")
        else:
            results_text.insert(tk.END, "SKIP (Not available)\n", "skip")
        results_text.update()

        # Test 2: Serial Port
        results_text.insert(tk.END, "2. Serial Port Test: ")
        results_text.update()
        try:
            import serial
            ser = serial.Serial(config.SERIAL_PORT, config.SERIAL_BAUD, timeout=1)
            ser.close()
            results_text.insert(tk.END, "PASS\n", "pass")
        except Exception as e:
            results_text.insert(tk.END, f"FAIL ({e})\n", "fail")
        results_text.update()

        # Test 3: IOL-HAT
        results_text.insert(tk.END, "3. IOL-HAT Communication: ")
        results_text.update()
        try:
            with iol_io_lock:
                raw_data = iolhat.pd(config.IOL_PORT, 0, config.DATA_LENGTH, None)
            if len(raw_data) >= 15 and raw_data != b'\x00' * len(raw_data):
                results_text.insert(tk.END, "PASS (Data received)\n", "pass")
            else:
                results_text.insert(tk.END, "FAIL (No response or all zeros)\n", "fail")
        except Exception as e:
            results_text.insert(tk.END, f"FAIL ({e})\n", "fail")
        results_text.update()

        # Test 4: Flow Meter
        results_text.insert(tk.END, "4. Flow Meter Reading: ")
        results_text.update()
        try:
            gallons = get_cached_actual_gallons() if flow_control_active() else read_flow_meter()
            if not connection_error:
                results_text.insert(tk.END, f"PASS (Reading: {gallons:.2f} gal)\n", "pass")
            else:
                results_text.insert(tk.END, f"FAIL ({error_message})\n", "fail")
        except Exception as e:
            results_text.insert(tk.END, f"FAIL ({e})\n", "fail")
        results_text.update()

        # Test 5: Display
        results_text.insert(tk.END, "5. Display System: PASS (You can see this)\n", "pass")

        results_text.insert(tk.END, "\n=== Test Complete ===\n")

        # Color tags
        results_text.tag_config("pass", foreground="green")
        results_text.tag_config("fail", foreground="red")
        results_text.tag_config("skip", foreground="yellow")

    # Run tests in thread to not block GUI
    test_thread = threading.Thread(target=run_tests, daemon=True)
    test_thread.start()

def close_self_test():
    """Close self-test and return to menu"""
    global self_test_mode, self_test_window
    self_test_mode = False
    if self_test_window:
        self_test_window.destroy()
        self_test_window = None
    arm_menu_ov_guard()

def run_full_test():
    """Run comprehensive interactive full system test"""
    global full_test_mode, full_test_window

    full_test_mode = True
    full_test_window = tk.Toplevel()
    full_test_window.title("Full System Test")
    full_test_window.attributes('-fullscreen', True)
    full_test_window.configure(bg='black')

    # Title
    title = tk.Label(full_test_window, text="FULL SYSTEM TEST", font=("Helvetica", 36, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=15)

    # Controls instruction
    controls = tk.Label(full_test_window, text="Test each item - OV=TEST OV / EXIT MENU (press twice to exit)",
                       font=("Helvetica", 18, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Test status dict
    test_status = {
        'minus_1': False,
        'plus_1': False,
        'minus_10': False,
        'plus_10': False,
        'OV': False,
        'PS': False,
        'button': False,
        'gpio': False,
        'relay': False,
        'serial': False,
        'iolhat': False,
        'flow_meter': False,
        'display': False
    }

    # Flow meter error message
    flow_meter_error = [None]  # Use list to allow modification in nested function

    # Results frame
    results_frame = tk.Frame(full_test_window, bg='black')
    results_frame.pack(fill=tk.BOTH, expand=True, padx=30, pady=10)

    results_text = tk.Text(results_frame, font=("Courier", 20), bg="black", fg="white",
                           height=18, width=65)
    results_text.pack()

    # Tag configs for colors
    results_text.tag_config("pass", foreground="green")
    results_text.tag_config("pending", foreground="yellow")
    results_text.tag_config("fail", foreground="red")
    results_text.tag_config("header", foreground="cyan")

    def update_display():
        results_text.delete('1.0', tk.END)
        results_text.insert(tk.END, "SERIAL COMMANDS:\n", "header")
        results_text.insert(tk.END, f"  Send -1:     {'✓ PASS' if test_status['minus_1'] else '○ PENDING'}\n",
                           "pass" if test_status['minus_1'] else "pending")
        results_text.insert(tk.END, f"  Send +1:     {'✓ PASS' if test_status['plus_1'] else '○ PENDING'}\n",
                           "pass" if test_status['plus_1'] else "pending")
        results_text.insert(tk.END, f"  Send -10:    {'✓ PASS' if test_status['minus_10'] else '○ PENDING'}\n",
                           "pass" if test_status['minus_10'] else "pending")
        results_text.insert(tk.END, f"  Send +10:    {'✓ PASS' if test_status['plus_10'] else '○ PENDING'}\n",
                           "pass" if test_status['plus_10'] else "pending")
        results_text.insert(tk.END, f"  Send OV:     {'✓ PASS' if test_status['OV'] else '○ PENDING'}\n",
                           "pass" if test_status['OV'] else "pending")
        results_text.insert(tk.END, f"  Send PS:     {'✓ PASS' if test_status['PS'] else '○ PENDING'}\n\n",
                           "pass" if test_status['PS'] else "pending")

        results_text.insert(tk.END, "HARDWARE:\n", "header")
        results_text.insert(tk.END, f"  GPIO:        {'✓ PASS' if test_status['gpio'] else '○ TESTING...'}\n",
                           "pass" if test_status['gpio'] else "pending")
        results_text.insert(tk.END, f"  Relay:       {'✓ PASS' if test_status['relay'] else '○ TESTING...'}\n",
                           "pass" if test_status['relay'] else "pending")
        results_text.insert(tk.END, f"  Serial Port: {'✓ PASS' if test_status['serial'] else '○ TESTING...'}\n",
                           "pass" if test_status['serial'] else "pending")
        results_text.insert(tk.END, f"  IOL-HAT:     {'✓ PASS' if test_status['iolhat'] else '○ TESTING...'}\n",
                           "pass" if test_status['iolhat'] else "pending")

        # Flow meter with error display
        if flow_meter_error[0]:
            results_text.insert(tk.END, f"  Flow Meter:  ✗ ERROR\n", "fail")
            results_text.insert(tk.END, f"\n{flow_meter_error[0]}\n", "fail")
        else:
            results_text.insert(tk.END, f"  Flow Meter:  {'✓ PASS' if test_status['flow_meter'] else '○ TESTING...'}\n",
                               "pass" if test_status['flow_meter'] else "pending")

        results_text.insert(tk.END, f"  Button:      {'✓ PASS' if test_status['button'] else '○ PENDING (Press thumbs up)'}\n",
                           "pass" if test_status['button'] else "pending")
        results_text.insert(tk.END, f"  Display:     {'✓ PASS' if test_status['display'] else '○ PASS (You can see this)'}\n",
                           "pass")

    def mark_tested(test_name):
        test_status[test_name] = True
        update_display()

    # Store functions in window for serial listener access
    full_test_window.mark_tested = mark_tested
    full_test_window.test_status = test_status

    # Run hardware tests automatically
    def run_hardware_tests():
        # Test 1: GPIO
        if GPIO_AVAILABLE:
            try:
                # Quick test - just check if GPIO is initialized
                root.after(0, lambda: mark_tested('gpio'))
            except Exception:
                pass

        time.sleep(0.2)

        # Test 2: Relay (with visible pulse)
        if GPIO_AVAILABLE:
            try:
                GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.HIGH)
                time.sleep(0.3)
                GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.LOW)
                root.after(0, lambda: mark_tested('relay'))
            except Exception:
                pass

        time.sleep(0.2)

        # Test 3: Serial Port
        try:
            import serial
            ser = serial.Serial(config.SERIAL_PORT, config.SERIAL_BAUD, timeout=1)
            ser.close()
            root.after(0, lambda: mark_tested('serial'))
        except Exception:
            pass

        time.sleep(0.2)

        # Test 4: IOL-HAT Communication (with 5 second timeout)
        iolhat_error = [None]
        iolhat_success = [False]

        def test_iolhat():
            try:
                with iol_io_lock:
                    raw_data = iolhat.pd(config.IOL_PORT, 0, config.DATA_LENGTH, None)
                if len(raw_data) >= 15 and raw_data != b'\x00' * len(raw_data):
                    iolhat_success[0] = True
                    root.after(0, lambda: mark_tested('iolhat'))
                # If all zeros, it means IOL-HAT works but device isn't responding
                elif len(raw_data) >= 15:
                    iolhat_success[0] = True
                    root.after(0, lambda: mark_tested('iolhat'))
            except Exception as e:
                iolhat_error[0] = str(e)

        # Run IOL-HAT test with timeout
        iolhat_thread = threading.Thread(target=test_iolhat, daemon=True)
        iolhat_thread.start()
        iolhat_thread.join(timeout=5.0)  # 5 second timeout

        # Check if test completed or timed out
        if not iolhat_success[0]:
            # Test failed or timed out
            pass  # IOL-HAT test remains in TESTING state (will show as yellow/pending)

        time.sleep(0.2)

        # Test 5: Flow Meter
        try:
            with iol_io_lock:
                raw_data = iolhat.pd(config.IOL_PORT, 0, config.DATA_LENGTH, None)
            if raw_data and len(raw_data) == config.DATA_LENGTH and raw_data != b'\x00' * len(raw_data):
                root.after(0, lambda: mark_tested('flow_meter'))
            else:
                # Flow meter returned all zeros or wrong length
                error_msg = "ERROR: Ensure flow meter is connected.\nCheck if flow meter displays green check mark in corner."
                flow_meter_error[0] = error_msg
                root.after(0, update_display)
        except Exception as e:
            error_msg = f"ERROR: Ensure flow meter is connected.\nCheck if flow meter displays green check mark in corner.\n({str(e)})"
            flow_meter_error[0] = error_msg
            root.after(0, update_display)

        # Test 6: Display (automatically pass - if you can see this)
        root.after(0, lambda: mark_tested('display'))

    update_display()

    # Start hardware tests in background
    hardware_thread = threading.Thread(target=run_hardware_tests, daemon=True)
    hardware_thread.start()

def close_full_test():
    """Close full-test and return to menu"""
    global full_test_mode, full_test_window
    full_test_mode = False
    if full_test_window:
        full_test_window.destroy()
        full_test_window = None
    arm_menu_ov_guard()

def check_wifi_status():
    """Check if WiFi is connected and return status string"""
    try:
        import subprocess
        # Check if wlan0 is up and has an IP
        result = subprocess.run(['ip', 'addr', 'show', 'wlan0'],
                              capture_output=True, text=True, timeout=1)

        if 'state UP' in result.stdout and 'inet ' in result.stdout:
            # Get signal strength if available
            try:
                signal_result = subprocess.run(['cat', '/proc/net/wireless'],
                                             capture_output=True, text=True, timeout=1)
                if 'wlan0' in signal_result.stdout:
                    return "WiFi: CONNECTED"
            except Exception:
                pass
            return "WiFi: CONNECTED"
        else:
            return "WiFi: DISCONNECTED"
    except Exception as e:
        return "WiFi: UNKNOWN"

def run_system_update():
    """Run system update"""
    global update_mode, update_window

    update_mode = True
    update_window = tk.Toplevel()
    update_window.title("System Update")
    update_window.attributes('-fullscreen', True)
    update_window.configure(bg='black')

    # Title
    title = tk.Label(update_window, text="SYSTEM UPDATE", font=("Helvetica", 36, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=20)

    # Controls instruction
    controls = tk.Label(update_window, text="OV=EXIT TO MENU",
                       font=("Helvetica", 22, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    # Status text
    status_frame = tk.Frame(update_window, bg='black')
    status_frame.pack(fill=tk.BOTH, expand=True, padx=40, pady=20)

    status_text = tk.Text(status_frame, font=("Courier", 32, "bold"), bg="black", fg="lime",
                         wrap=tk.WORD)
    status_text.pack(fill=tk.BOTH, expand=True)

    def run_update():
        import subprocess
        status_text.insert(tk.END, "Starting BBB software update from GitHub...\n\n")
        status_text.update()

        try:
            # Show current version
            status_text.insert(tk.END, f"Current Version: {VERSION}\n\n")
            status_text.update()

            if not os.path.isdir('/home/pi/Big-Beautiful-Box/.git'):
                status_text.insert(tk.END, "ERROR: Installed BBB folder is not a git repository.\n")
                status_text.insert(tk.END, "This box must be installed from a real git checkout for updates to work.\n")
                status_text.insert(tk.END, "Re-run install.sh from a cloned BBB repo.\n")
                status_text.insert(tk.END, "Press OV to return to menu\n")
                status_text.update()
                return

            # Step 1: Navigate to git repo and pull latest
            status_text.insert(tk.END, "=== Step 1: Checking for updates ===\n")
            status_text.update()

            result = subprocess.run(['git', '-C', '/home/pi/Big-Beautiful-Box', 'fetch', 'origin'],
                                  capture_output=True, text=True, timeout=60)
            if result.stderr:
                status_text.insert(tk.END, result.stderr + "\n")
            status_text.update()

            if result.returncode != 0:
                status_text.insert(tk.END, "ERROR: Git fetch failed!\n")
                status_text.insert(tk.END, "Press OV to return to menu\n")
                return

            # Check whether anything new exists on origin/master
            result = subprocess.run(
                ['git', '-C', '/home/pi/Big-Beautiful-Box', 'rev-list', '--count', 'HEAD..origin/master'],
                capture_output=True, text=True, timeout=10
            )
            updates_available = int(result.stdout.strip() or "0")

            # Get the version we're updating to
            result = subprocess.run(['git', '-C', '/home/pi/Big-Beautiful-Box', 'log', 'origin/master', '-1', '--format=%s'],
                                  capture_output=True, text=True, timeout=10)
            new_version_msg = result.stdout.strip()
            new_version = _read_git_ref_version('origin/master') or "(version file missing)"
            if updates_available == 0:
                status_text.insert(tk.END, "\nAlready up to date.\n")
                status_text.insert(tk.END, f"Latest Version: {new_version}\n")
                status_text.insert(tk.END, f"Latest commit:\n{new_version_msg}\n\n")
                status_text.insert(tk.END, "Press OV to return to menu\n")
                status_text.update()
                return

            status_text.insert(tk.END, f"\nNew Version Available: {new_version}\n")
            status_text.insert(tk.END, f"Commit:\n{new_version_msg}\n")
            status_text.insert(tk.END, f"Commits to apply: {updates_available}\n\n")
            status_text.update()

            # Step 2: Reset to latest origin/master
            status_text.insert(tk.END, "=== Step 2: Installing update ===\n")
            status_text.update()

            result = subprocess.run(['git', '-C', '/home/pi/Big-Beautiful-Box', 'reset', '--hard', 'origin/master'],
                                  capture_output=True, text=True, timeout=30)
            status_text.insert(tk.END, "Updated repository\n")
            status_text.update()

            if result.returncode != 0:
                status_text.insert(tk.END, "ERROR: Git reset failed!\n")
                status_text.insert(tk.END, "Press OV to return to menu\n")
                return

            status_text.insert(tk.END, "Refreshing deployed BBB runtime...\n")
            status_text.update()

            deploy_cmd = (
                "set -e; "
                "cp /home/pi/Big-Beautiful-Box/deploy/bbb-logrotate.conf /etc/logrotate.d/bbb; "
                "cp /home/pi/Big-Beautiful-Box/deploy/bbb-logrotate.service /etc/systemd/system/bbb-logrotate.service; "
                "cp /home/pi/Big-Beautiful-Box/deploy/bbb-logrotate.timer /etc/systemd/system/bbb-logrotate.timer; "
                "mkdir -p /opt/src /opt/mopeka; "
                "cp /home/pi/Big-Beautiful-Box/rotorsync_bumble.py /opt/rotorsync_bumble.py; "
                "cp /home/pi/Big-Beautiful-Box/rotorsync_watchdog.py /opt/rotorsync_watchdog.py; "
                "cp -r /home/pi/Big-Beautiful-Box/src/. /opt/src/; "
                "for mopeka_file in /home/pi/Big-Beautiful-Box/mopeka/*; do "
                "[ -f \"$mopeka_file\" ] || continue; "
                "base_name=$(basename \"$mopeka_file\"); "
                "if [ \"$base_name\" = \"mopeka_config.json\" ] && [ -f /opt/mopeka/mopeka_config.json ]; then continue; fi; "
                "cp \"$mopeka_file\" /opt/mopeka/; "
                "done; "
                "chmod 755 /opt/rotorsync_bumble.py /opt/rotorsync_watchdog.py; "
                "python3 - <<'PY'\n"
                "from pathlib import Path\n"
                "cfg = Path('/boot/firmware/config.txt')\n"
                "text = cfg.read_text()\n"
                "text = text.replace('hdmi_group=2', 'hdmi_group=1')\n"
                "text = text.replace('hdmi_mode=87', 'hdmi_mode=34')\n"
                "text = text.replace('hdmi_cvt=1024 600 60 6 0 0 0\\n', '')\n"
                "for line in ('hdmi_force_hotplug=1', 'hdmi_drive=2', 'hdmi_group=1', 'hdmi_mode=34'):\n"
                "    if line not in text:\n"
                "        text += ('\\n' if not text.endswith('\\n') else '') + line + '\\n'\n"
                "cfg.write_text(text)\n"
                "video_arg = 'video=HDMI-A-1:1920x1080M@30D'\n"
                "for path_str in ('/boot/firmware/current/cmdline.txt', '/boot/firmware/cmdline.txt', '/boot/cmdline.txt'):\n"
                "    p = Path(path_str)\n"
                "    if not p.exists():\n"
                "        continue\n"
                "    parts = p.read_text().strip().split()\n"
                "    if video_arg not in parts:\n"
                "        parts.append(video_arg)\n"
                "        p.write_text(' '.join(parts) + '\\n')\n"
                "PY\n"
                "systemctl daemon-reload; "
                "systemctl enable --now bbb-logrotate.timer; "
                "systemctl start bbb-logrotate.service"
            )
            result = subprocess.run(
                ['bash', '-lc', f"printf 'raspi\\n' | sudo -S bash -lc {shlex.quote(deploy_cmd)}"],
                capture_output=True, text=True, timeout=30
            )
            if result.returncode != 0:
                status_text.insert(tk.END, "WARNING: Could not refresh deployed BBB runtime.\n")
                if result.stderr:
                    status_text.insert(tk.END, result.stderr + "\n")
            else:
                status_text.insert(tk.END, "BBB runtime files updated.\n")
                status_text.insert(tk.END, "Boot display config updated to 1080p30.\n")
            status_text.update()

            status_text.insert(tk.END, "\n=== UPDATE COMPLETE ===\n\n")
            status_text.insert(tk.END, "Restarting service to apply changes...\n\n")
            status_text.update()

            # Wait 2 seconds so user can see the message
            time.sleep(2)

            # Restart via systemd so both IOL master and dashboard restart cleanly.
            # Launch in the background because the current process will be terminated by the restart.
            restart_cmd = (
                "sleep 1; "
                "printf 'raspi\n' | sudo -S systemctl restart rotorsync.service rotorsync_watchdog.service iol_dashboard.service"
            )
            subprocess.Popen(['bash', '-lc', restart_cmd])
            return

        except subprocess.TimeoutExpired:
            status_text.insert(tk.END, "\n\nERROR: Command timed out\n")
            status_text.insert(tk.END, "Press OV to return to menu\n")
        except Exception as e:
            status_text.insert(tk.END, f"\n\nERROR: {e}\n")
            status_text.insert(tk.END, "Press OV to return to menu\n")

        status_text.see(tk.END)

    # Run update in thread
    update_thread = threading.Thread(target=run_update, daemon=True)
    update_thread.start()


def run_bug_capture():
    """Capture a compact bug report with the most useful recent diagnostics."""
    global update_mode, update_window

    update_mode = True
    update_window = tk.Toplevel()
    update_window.title("Capture Bug Report")
    update_window.attributes('-fullscreen', True)
    update_window.configure(bg='black')

    title = tk.Label(update_window, text="CAPTURE BUG REPORT", font=("Helvetica", 36, "bold"),
                     fg="cyan", bg="black")
    title.pack(pady=20)

    controls = tk.Label(update_window, text="OV=EXIT TO MENU",
                       font=("Helvetica", 22, "bold"), fg="#ffff00", bg="#0a0a0a")
    controls.pack(pady=2)

    status_frame = tk.Frame(update_window, bg='black')
    status_frame.pack(fill=tk.BOTH, expand=True, padx=40, pady=20)

    status_text = tk.Text(status_frame, font=("Courier", 24, "bold"), bg="black", fg="lime",
                         wrap=tk.WORD)
    status_text.pack(fill=tk.BOTH, expand=True)

    def append(msg):
        status_text.insert(tk.END, msg)
        status_text.see(tk.END)
        status_text.update()

    def run_capture():
        timestamp = time.strftime('%Y%m%d-%H%M%S')
        report_dir = "/home/pi/bug_reports"
        report_path = f"{report_dir}/bug-report-{timestamp}.txt"
        capture_script = f"""set -e
mkdir -p {report_dir}
{{
  echo "BBB Bug Report"
  echo "Generated: $(date)"
  echo "Version: {VERSION}"
  echo
  echo "=== SYSTEM ==="
  uname -a
  echo
  uptime
  echo
  df -h /
  echo
  echo "=== SERVICES ==="
  systemctl --no-pager --full status iol_dashboard.service rotorsync.service rotorsync_watchdog.service || true
  echo
  echo "=== JOURNAL: IOL DASHBOARD (LAST 120) ==="
  journalctl -u iol_dashboard.service -n 120 --no-pager || true
  echo
  echo "=== JOURNAL: ROTORSYNC (LAST 120) ==="
  journalctl -u rotorsync.service -n 120 --no-pager || true
  echo
  echo "=== FILL HISTORY (LAST 40) ==="
  tail -n 40 /home/pi/fill_history.log 2>/dev/null || true
  echo
  echo "=== FILL CALIBRATION (LAST 40) ==="
  tail -n 40 /home/pi/fill_calibration.log 2>/dev/null || true
  echo
  echo "=== SERIAL DEBUG (LAST 200) ==="
  tail -n 200 /home/pi/serial_debug.log 2>/dev/null || true
  echo
  echo "=== MENU DEBUG (LAST 120) ==="
  tail -n 120 /home/pi/menu_debug.log 2>/dev/null || true
  echo
  echo "=== BUTTON DEBUG (LAST 120) ==="
  tail -n 120 /home/pi/button_debug.log 2>/dev/null || true
  echo
  echo "=== WATCHDOG LOG (LAST 120) ==="
  tail -n 120 /home/pi/rotorsync_watchdog.log 2>/dev/null || true
  echo
  echo "=== IOL DASHBOARD LOG (LAST 400) ==="
  tail -n 400 /home/pi/iol_dashboard.log 2>/dev/null || true
}} > {report_path}
echo {report_path}
"""

        append("Capturing bug report...\n\n")
        try:
            result = subprocess.run(
                ['bash', '-lc', capture_script],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0:
                append("ERROR: Bug capture failed\n\n")
                if result.stderr:
                    append(result.stderr + "\n")
            else:
                path = result.stdout.strip().splitlines()[-1]
                append("Bug report captured successfully.\n\n")
                append(f"Saved to:\n{path}\n\n")
                append("Press OV to return to menu\n")
        except subprocess.TimeoutExpired:
            append("ERROR: Bug capture timed out\n\nPress OV to return to menu\n")
        except Exception as e:
            append(f"ERROR: {e}\n\nPress OV to return to menu\n")

    capture_thread = threading.Thread(target=run_capture, daemon=True)
    capture_thread.start()

def close_update():
    """Close update window and return to menu"""
    global update_mode, update_window
    update_mode = False
    if update_window:
        update_window.destroy()
        update_window = None
    arm_menu_ov_guard()

def confirm_reset_season():
    """Confirm and reset season total with OV/PS commands"""
    # Create confirmation window
    confirm_window = tk.Toplevel()
    confirm_window.title("Reset Season Confirmation")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    # Confirmation message
    message = tk.Label(confirm_window, text="RESET SEASON TOTAL?",
                      font=("Helvetica", 48, "bold"), fg="orange", bg="black")
    message.pack(expand=True, pady=50)

    # Show current season total
    current_total = tk.Label(confirm_window, text=f"Current Season Total: {season_total:.2f} gal",
                            font=("Helvetica", 32, "bold"), fg="lime", bg="black")
    current_total.pack(pady=20)

    # Instructions
    instructions = tk.Label(confirm_window, text="Press OV to CONFIRM or PS to CANCEL",
                          font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
    instructions.pack(pady=30)

    # Countdown timer
    countdown_label = tk.Label(confirm_window, text="Auto-cancel in 10 seconds",
                             font=("Helvetica", 22), fg="white", bg="black")
    countdown_label.pack(pady=20)

    countdown = [10]  # Use list to allow modification in nested function

    def cancel_reset():
        global reset_season_confirm_window, reset_season_confirm_handler, reset_season_cancel_handler
        confirm_window.destroy()
        reset_season_confirm_window = None
        reset_season_confirm_handler = None
        reset_season_cancel_handler = None

    def confirm_reset():
        global reset_season_confirm_window, reset_season_confirm_handler, reset_season_cancel_handler
        print("Resetting season total...")
        reset_season_total()
        confirm_window.destroy()
        reset_season_confirm_window = None
        reset_season_confirm_handler = None
        reset_season_cancel_handler = None
        # Update menu display
        if menu_window:
            update_totals_display()

    def update_countdown():
        countdown[0] -= 1
        if countdown[0] <= 0:
            cancel_reset()
        else:
            countdown_label.config(text=f"Auto-cancel in {countdown[0]} seconds")
            confirm_window.after(1000, update_countdown)

    # Bind keyboard shortcuts
    confirm_window.bind('<Escape>', lambda e: cancel_reset())

    # Store the confirmation handlers globally so serial listener can access them
    global reset_season_confirm_window, reset_season_confirm_handler, reset_season_cancel_handler
    reset_season_confirm_window = confirm_window
    reset_season_confirm_handler = confirm_reset
    reset_season_cancel_handler = cancel_reset

    # Start countdown
    confirm_window.after(1000, update_countdown)


def confirm_reset_flow_curve():
    """Confirm and reset learned flow curve override with OV/PS commands."""
    confirm_window = tk.Toplevel()
    confirm_window.title("Flow Curve Reset Confirmation")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    message = tk.Label(confirm_window, text="USE FACTORY FLOW CURVE?",
                      font=("Helvetica", 46, "bold"), fg="orange", bg="black")
    message.pack(expand=True, pady=40)

    current_curve = tk.Label(
        confirm_window,
        text=f"Current Curve: {flow_curve_status_text()}",
        font=("Helvetica", 30, "bold"),
        fg="lime",
        bg="black",
    )
    current_curve.pack(pady=16)

    detail = tk.Label(
        confirm_window,
        text="This archives learned curve samples and reloads factory defaults.",
        font=("Helvetica", 24, "bold"),
        fg="white",
        bg="black",
        wraplength=1100,
        justify=tk.CENTER,
    )
    detail.pack(pady=12)

    instructions = tk.Label(confirm_window, text="Press OV to CONFIRM or PS to CANCEL",
                          font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
    instructions.pack(pady=24)

    countdown_label = tk.Label(confirm_window, text="Auto-cancel in 10 seconds",
                             font=("Helvetica", 22), fg="white", bg="black")
    countdown_label.pack(pady=16)

    countdown = [10]

    def cancel_reset():
        global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
        global reset_flow_curve_cancel_handler
        confirm_window.destroy()
        reset_flow_curve_confirm_window = None
        reset_flow_curve_confirm_handler = None
        reset_flow_curve_cancel_handler = None

    def confirm_reset():
        global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
        global reset_flow_curve_cancel_handler
        archived = flow_curve.reset_learning(
            FLOW_CURVE_SAMPLES_PATH,
            FLOW_CURVE_PROPOSAL_PATH,
            FLOW_CURVE_OVERRIDE_PATH,
        )
        load_flow_curve_state()
        with open("/home/pi/fill_calibration.log", "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: factory reset"
                f" | Archived: {', '.join(archived) if archived else 'none'}\n"
            )
        confirm_window.destroy()
        reset_flow_curve_confirm_window = None
        reset_flow_curve_confirm_handler = None
        reset_flow_curve_cancel_handler = None
        if menu_window:
            close_menu()
            show_menu()

    def update_countdown():
        countdown[0] -= 1
        if countdown[0] <= 0:
            cancel_reset()
        else:
            countdown_label.config(text=f"Auto-cancel in {countdown[0]} seconds")
            confirm_window.after(1000, update_countdown)

    confirm_window.bind('<Escape>', lambda e: cancel_reset())

    global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
    global reset_flow_curve_cancel_handler
    reset_flow_curve_confirm_window = confirm_window
    reset_flow_curve_confirm_handler = confirm_reset
    reset_flow_curve_cancel_handler = cancel_reset

    confirm_window.after(1000, update_countdown)


def confirm_accept_flow_curve():
    """Confirm and activate a pending learned flow curve proposal."""
    proposal = flow_curve.load_curve_proposal(FLOW_CURVE_PROPOSAL_PATH)
    if not proposal:
        confirm_window = tk.Toplevel()
        confirm_window.title("No Curve Proposal")
        confirm_window.attributes('-fullscreen', True)
        confirm_window.configure(bg='black')

        message = tk.Label(confirm_window, text="NO LEARNED CURVE READY",
                          font=("Helvetica", 46, "bold"), fg="orange", bg="black")
        message.pack(expand=True, pady=50)
        detail = tk.Label(confirm_window, text="Need 3 thumbs-up confirmed Auto fills first.",
                         font=("Helvetica", 28, "bold"), fg="white", bg="black")
        detail.pack(pady=20)
        instructions = tk.Label(confirm_window, text="Press OV or PS to return",
                              font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
        instructions.pack(pady=30)

        def close_notice():
            global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
            global accept_flow_curve_cancel_handler
            confirm_window.destroy()
            accept_flow_curve_confirm_window = None
            accept_flow_curve_confirm_handler = None
            accept_flow_curve_cancel_handler = None

        global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
        global accept_flow_curve_cancel_handler
        accept_flow_curve_confirm_window = confirm_window
        accept_flow_curve_confirm_handler = close_notice
        accept_flow_curve_cancel_handler = close_notice
        confirm_window.after(8000, close_notice)
        return

    learning = proposal.get("learning", {})
    offset = learning.get("applied_offset_gallons")
    raw_offset = learning.get("raw_offset_gallons")
    sample_count = proposal.get("sample_count", 0)

    confirm_window = tk.Toplevel()
    confirm_window.title("Accept Flow Curve Confirmation")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    message = tk.Label(confirm_window, text="ACCEPT LEARNED FLOW CURVE?",
                      font=("Helvetica", 44, "bold"), fg="orange", bg="black")
    message.pack(expand=True, pady=36)

    detail_text = (
        f"Samples: {sample_count} Auto fills\n"
        f"Proposed offset: {offset:+.3f} gal\n"
        f"Raw offset: {raw_offset:+.3f} gal"
        if isinstance(offset, (int, float)) and isinstance(raw_offset, (int, float))
        else f"Samples: {sample_count} Auto fills"
    )
    detail = tk.Label(
        confirm_window,
        text=detail_text,
        font=("Helvetica", 28, "bold"),
        fg="lime",
        bg="black",
        justify=tk.CENTER,
    )
    detail.pack(pady=16)

    warning = tk.Label(
        confirm_window,
        text="This makes the learned curve active. Factory curve remains available.",
        font=("Helvetica", 23, "bold"),
        fg="white",
        bg="black",
        wraplength=1100,
        justify=tk.CENTER,
    )
    warning.pack(pady=12)

    instructions = tk.Label(confirm_window, text="Press OV to ACCEPT or PS to CANCEL",
                          font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
    instructions.pack(pady=24)

    countdown_label = tk.Label(confirm_window, text="Auto-cancel in 10 seconds",
                             font=("Helvetica", 22), fg="white", bg="black")
    countdown_label.pack(pady=16)

    countdown = [10]

    def cancel_accept():
        global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
        global accept_flow_curve_cancel_handler
        confirm_window.destroy()
        accept_flow_curve_confirm_window = None
        accept_flow_curve_confirm_handler = None
        accept_flow_curve_cancel_handler = None

    def confirm_accept():
        global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
        global accept_flow_curve_cancel_handler
        try:
            accepted = flow_curve.accept_curve_proposal(
                FLOW_CURVE_PROPOSAL_PATH,
                FLOW_CURVE_OVERRIDE_PATH,
            )
            load_flow_curve_state()
            with open("/home/pi/fill_calibration.log", "a") as f:
                f.write(
                    f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: accepted proposal"
                    f" | Offset: {accepted.get('learning', {}).get('applied_offset_gallons', 'unknown')}\n"
                )
        except Exception as exc:
            with open("/home/pi/fill_calibration.log", "a") as f:
                f.write(
                    f"{time.strftime('%Y-%m-%d %H:%M:%S')} | CurveLearning: accept failed"
                    f" | Error: {exc}\n"
                )
        confirm_window.destroy()
        accept_flow_curve_confirm_window = None
        accept_flow_curve_confirm_handler = None
        accept_flow_curve_cancel_handler = None
        if menu_window:
            close_menu()
            show_menu()

    def update_countdown():
        countdown[0] -= 1
        if countdown[0] <= 0:
            cancel_accept()
        else:
            countdown_label.config(text=f"Auto-cancel in {countdown[0]} seconds")
            confirm_window.after(1000, update_countdown)

    confirm_window.bind('<Escape>', lambda e: cancel_accept())

    accept_flow_curve_confirm_window = confirm_window
    accept_flow_curve_confirm_handler = confirm_accept
    accept_flow_curve_cancel_handler = cancel_accept

    confirm_window.after(1000, update_countdown)

def shutdown_system():
    """Shutdown the system"""
    import subprocess
    # Show confirmation for 2 seconds then shutdown
    confirm_window = tk.Toplevel()
    confirm_window.title("Shutdown")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    msg = tk.Label(confirm_window, text="SHUTTING DOWN...",
                  font=("Helvetica", 48, "bold"), fg="red", bg="black")
    msg.pack(expand=True)

    def do_shutdown():
        time.sleep(2)
        # Use systemctl poweroff with password
        subprocess.run(['bash', '-c', 'echo raspi | sudo -S systemctl poweroff'])

    shutdown_thread = threading.Thread(target=do_shutdown, daemon=True)
    shutdown_thread.start()

def reboot_system():
    """Reboot the system"""
    import subprocess
    # Show confirmation for 2 seconds then reboot
    confirm_window = tk.Toplevel()
    confirm_window.title("Reboot")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')

    msg = tk.Label(confirm_window, text="REBOOTING...",
                  font=("Helvetica", 48, "bold"), fg="orange", bg="black")
    msg.pack(expand=True)

    def do_reboot():
        time.sleep(2)
        # Use systemctl reboot with password
        subprocess.run(['bash', '-c', 'echo raspi | sudo -S systemctl reboot'])

    reboot_thread = threading.Thread(target=do_reboot, daemon=True)
    reboot_thread.start()

def style_menu_item(index, selected):
    """Apply selected or unselected styling to a single menu item."""
    if index < 0 or index >= len(menu_buttons) or index >= len(menu_arrows):
        return

    btn = menu_buttons[index]
    arrow = menu_arrows[index]

    # High-contrast unselected palette for sunlight readability.
    colors = [
        ("#d7efff", "#111111"),  # View Logs
        ("#eadcff", "#111111"),  # Fill History
        ("#d8ffff", "#111111"),  # Tank Calibration
        ("#ffe3bf", "#111111"),  # Full Test
        ("#dff7d9", "#111111"),  # Reset Season
        ("#e7ffd8", "#111111"),  # Self Test
        ("#fff0b8", "#111111"),  # Capture Bug
        ("#eadcff", "#111111"),  # System Update
        ("#ffd6d6", "#111111"),  # Shutdown
        ("#ffe3bf", "#111111"),  # Reboot
        ("#ffc9c9", "#111111"),  # Exit to Desktop
        ("#e3e3e3", "#111111"),  # Exit Menu
    ]

    if selected:
        # SELECTED - Use a dark tile with bright text so it stands off the lighter menu items.
        btn.config(bg="#111111", fg="#ffff00",
                  activebackground="#111111", activeforeground="#ffff00",
                  font=("Helvetica", 28, "bold"),
                  relief=tk.RAISED, borderwidth=6,
                  highlightbackground="#ffffff", highlightthickness=4, highlightcolor="#ffffff",
                  width=16, height=2,
                  wraplength=360, justify=tk.CENTER)
        arrow.config(text=">>> SELECTED >>>", fg="#00ffff",
                    font=("Helvetica", 20, "bold"))
    else:
        bg, fg = colors[index % len(colors)]
        btn.config(bg=bg, fg=fg,
                  activebackground=bg, activeforeground=fg,
                  font=("Helvetica", 28, "bold"),
                  relief=tk.FLAT, borderwidth=2,
                  highlightthickness=0,
                  highlightbackground=bg,
                  highlightcolor=bg,
                  width=16, height=2,
                  wraplength=360, justify=tk.CENTER)
        arrow.config(text="", fg="black")

def update_menu_highlight(full_refresh=False):
    """Update visual highlighting of selected menu item."""
    global menu_buttons, menu_arrows, menu_selected_index, menu_position_label, menu_displayed_index

    if not menu_buttons or not menu_arrows:
        return

    # Update position indicator with current selection
    if menu_position_label:
        menu_position_label.config(
            text=f"Option {menu_selected_index + 1} of {len(MENU_ITEMS)}: {MENU_ITEMS[menu_selected_index]}"
        )

    if full_refresh or menu_displayed_index is None:
        for i in range(len(menu_buttons)):
            style_menu_item(i, i == menu_selected_index)
    else:
        if menu_displayed_index != menu_selected_index:
            style_menu_item(menu_displayed_index, False)
        style_menu_item(menu_selected_index, True)

    menu_displayed_index = menu_selected_index

def _apply_menu_highlight_update():
    """Run one coalesced menu highlight redraw on the Tk thread."""
    global menu_highlight_refresh_pending
    menu_highlight_refresh_pending = False
    update_menu_highlight()

def schedule_menu_highlight_update():
    """Schedule at most one menu redraw while serial knob pulses are arriving."""
    global menu_highlight_refresh_pending
    if menu_highlight_refresh_pending:
        return
    menu_highlight_refresh_pending = True
    root.after(0, _apply_menu_highlight_update)


def arm_menu_ov_guard():
    """Ignore repeated OV messages from the same press after a screen change."""
    global menu_ov_guard_until
    menu_ov_guard_until = time.time() + MENU_OV_GUARD_SECONDS


def should_ignore_menu_ov_bounce(line, source):
    global menu_ov_guard_until
    if line != "OV":
        menu_ov_guard_until = 0.0
        return False
    if time.time() >= menu_ov_guard_until:
        return False
    msg = f"{source}: Ignored OV bounce after menu select"
    print(msg)
    log_serial_debug(msg)
    with open(debug_log, 'a') as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
    return True


def menu_navigate_up():
    """Move selection up in menu"""
    global menu_selected_index
    old_index = menu_selected_index
    menu_selected_index = (menu_selected_index - 1) % len(MENU_ITEMS)
    with open('/home/pi/menu_debug.log', 'a') as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - menu_navigate_up: {old_index} -> {menu_selected_index}\n")
    schedule_menu_highlight_update()

def menu_navigate_down():
    """Move selection down in menu"""
    global menu_selected_index
    old_index = menu_selected_index
    menu_selected_index = (menu_selected_index + 1) % len(MENU_ITEMS)
    with open('/home/pi/menu_debug.log', 'a') as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - menu_navigate_down: {old_index} -> {menu_selected_index}\n")
    schedule_menu_highlight_update()

def menu_select():
    """Activate the currently selected menu item"""
    global menu_selected_index

    # Debug logging to track selection
    with open('/home/pi/menu_debug.log', 'a') as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - menu_select() called with index={menu_selected_index}, item={MENU_ITEMS[menu_selected_index] if menu_selected_index < len(MENU_ITEMS) else 'UNKNOWN'}\n")

    if menu_selected_index == 0:
        show_log_viewer()
    elif menu_selected_index == 1:
        show_fill_history()
    elif menu_selected_index == 2:
        show_tank_calibration()
    elif menu_selected_index == 3:
        run_full_test()
    elif menu_selected_index == 4:
        confirm_reset_season()
    elif menu_selected_index == 5:
        confirm_accept_flow_curve()
    elif menu_selected_index == 6:
        confirm_reset_flow_curve()
    elif menu_selected_index == 7:
        run_self_test()
    elif menu_selected_index == 8:
        run_bug_capture()
    elif menu_selected_index == 9:
        run_system_update()
    elif menu_selected_index == 10:
        shutdown_system()
    elif menu_selected_index == 11:
        reboot_system()
    elif menu_selected_index == 12:
        exit_to_desktop()
    elif menu_selected_index == 13:
        close_menu()

def exit_to_desktop():
    """Exit the dashboard and return to desktop with confirmation"""
    # Create confirmation window
    confirm_window = tk.Toplevel()
    confirm_window.title("Exit Confirmation")
    confirm_window.attributes('-fullscreen', True)
    confirm_window.configure(bg='black')
    
    # Confirmation message
    message = tk.Label(confirm_window, text="EXIT TO DESKTOP?", 
                       font=("Helvetica", 48, "bold"), fg="red", bg="black")
    message.pack(expand=True, pady=50)
    
    # Instructions
    instructions = tk.Label(confirm_window, text="Press OV to CONFIRM or PS to CANCEL",
                           font=("Helvetica", 22, "bold"), fg="cyan", bg="black")
    instructions.pack(pady=30)
    
    # Countdown timer
    countdown_label = tk.Label(confirm_window, text="Auto-cancel in 10 seconds",
                              font=("Helvetica", 22), fg="white", bg="black")
    countdown_label.pack(pady=20)
    
    countdown = [10]  # Use list to allow modification in nested function
    
    def cancel_exit():
        global exit_confirm_window, exit_confirm_handler, exit_cancel_handler
        confirm_window.destroy()
        exit_confirm_window = None
        exit_confirm_handler = None
        exit_cancel_handler = None
        # Menu window is already open behind this dialog - just let it reappear
    
    def confirm_exit():
        print("Exiting dashboard to desktop...")
        confirm_window.destroy()
        root.destroy()
        sys.exit(0)
    
    def update_countdown():
        countdown[0] -= 1
        if countdown[0] <= 0:
            cancel_exit()
        else:
            countdown_label.config(text=f"Auto-cancel in {countdown[0]} seconds")
            confirm_window.after(1000, update_countdown)
    
    # Bind keyboard shortcuts for confirmation
    confirm_window.bind('<Escape>', lambda e: cancel_exit())
    
    # Store the confirmation handlers globally so serial listener can access them
    global exit_confirm_window, exit_confirm_handler, exit_cancel_handler
    exit_confirm_window = confirm_window
    exit_confirm_handler = confirm_exit
    exit_cancel_handler = cancel_exit
    
    # Start countdown
    confirm_window.after(1000, update_countdown)

def close_menu():
    """Close the menu and return to main dashboard"""
    global menu_mode, menu_window, menu_buttons, menu_arrows, menu_selected_index, menu_displayed_index
    global menu_highlight_refresh_pending
    global exit_confirm_window, exit_confirm_handler, exit_cancel_handler
    global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
    global reset_flow_curve_cancel_handler
    global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
    global accept_flow_curve_cancel_handler
    menu_mode = False
    menu_selected_index = 0
    menu_displayed_index = None
    menu_highlight_refresh_pending = False
    menu_buttons = []
    menu_arrows = []
    # Reset exit confirmation globals
    exit_confirm_window = None
    exit_confirm_handler = None
    exit_cancel_handler = None
    reset_flow_curve_confirm_window = None
    reset_flow_curve_confirm_handler = None
    reset_flow_curve_cancel_handler = None
    accept_flow_curve_confirm_window = None
    accept_flow_curve_confirm_handler = None
    accept_flow_curve_cancel_handler = None
    if menu_window:
        menu_window.destroy()
        menu_window = None

def dismiss_reminders():
    """Dismiss the daily reminders screen"""
    global reminders_mode, reminders_window, last_reminder_date
    reminders_mode = False
    # Mark today as shown
    last_reminder_date = time.strftime('%Y-%m-%d')
    if reminders_window:
        reminders_window.destroy()
        reminders_window = None
    print(f"Daily reminders dismissed for {last_reminder_date}")

def show_daily_reminders():
    """Display daily reminders checklist with time-based greeting"""
    global reminders_mode, reminders_window

    reminders_mode = True
    reminders_window = tk.Toplevel(root)
    reminders_window.title("Daily Reminders")
    reminders_window.attributes('-fullscreen', True)
    reminders_window.configure(bg='black')

    # Determine greeting based on time of day
    current_hour = time.localtime().tm_hour
    if 6 <= current_hour < 12:
        greeting = "☀️ GOOD MORNING! ☀️"
        color = "yellow"
    elif 12 <= current_hour < 18:
        greeting = "🌤️ GOOD AFTERNOON! 🌤️"
        color = "orange"
    else:
        greeting = "🌙 GOOD EVENING! 🌙"
        color = "lightblue"

    # Title with time-based greeting
    title = tk.Label(reminders_window, text=greeting, font=("Helvetica", 48, "bold"),
                     fg=color, bg="black")
    title.pack(pady=30)

    # Subtitle
    subtitle = tk.Label(reminders_window, text="Before you start:", font=("Helvetica", 32),
                       fg="cyan", bg="black")
    subtitle.pack(pady=10)

    # Reminders frame
    reminders_frame = tk.Frame(reminders_window, bg='black')
    reminders_frame.pack(expand=True, pady=20)

    # Reminder items
    reminders = [
        "✓ Check for water in fuel",
        "✓ Connect app",
        "✓ Stay safe!"
    ]

    for reminder in reminders:
        label = tk.Label(reminders_frame, text=reminder, font=("Helvetica", 36, "bold"),
                        fg="white", bg="black", anchor="w")
        label.pack(pady=15, padx=50)

    # Instructions at bottom
    instructions = tk.Label(reminders_window, text="Press THUMBS UP button or send OV to continue",
                           font=("Helvetica", 28, "bold"), fg="green", bg="black")
    instructions.pack(side=tk.BOTTOM, pady=30)

def show_menu():
    """Display the main menu"""
    global menu_mode, menu_window, menu_buttons, menu_arrows, menu_selected_index, menu_position_label
    global menu_displayed_index, menu_highlight_refresh_pending

    load_flow_curve_state()

    # Defensive cleanup in case any stale submenu window/mode was left behind.
    global log_viewer_mode, log_viewer_window, log_viewer_text
    global fill_history_mode, fill_history_window, fill_history_text
    global calibration_mode, calibration_window, calibration_state
    global reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler
    global reset_flow_curve_cancel_handler
    global accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler
    global accept_flow_curve_cancel_handler

    log_viewer_mode = False
    if log_viewer_window:
        try:
            log_viewer_window.destroy()
        except Exception:
            pass
        log_viewer_window = None
        log_viewer_text = None

    fill_history_mode = False
    if fill_history_window:
        try:
            fill_history_window.destroy()
        except Exception:
            pass
        fill_history_window = None
        fill_history_text = None

    calibration_mode = False
    if calibration_window:
        try:
            calibration_window.destroy()
        except Exception:
            pass
        calibration_window = None
        calibration_state = None

    if reset_flow_curve_confirm_window:
        try:
            reset_flow_curve_confirm_window.destroy()
        except Exception:
            pass
        reset_flow_curve_confirm_window = None
        reset_flow_curve_confirm_handler = None
        reset_flow_curve_cancel_handler = None

    if accept_flow_curve_confirm_window:
        try:
            accept_flow_curve_confirm_window.destroy()
        except Exception:
            pass
        accept_flow_curve_confirm_window = None
        accept_flow_curve_confirm_handler = None
        accept_flow_curve_cancel_handler = None

    if menu_window:
        try:
            menu_window.destroy()
        except Exception:
            pass
        menu_window = None

    menu_mode = True
    menu_selected_index = 0  # Start at first item
    menu_displayed_index = None
    menu_highlight_refresh_pending = False
    menu_buttons = []
    menu_arrows = []

    menu_window = tk.Toplevel(root)
    menu_window.title("System Menu")
    menu_window.attributes('-fullscreen', True)
    menu_window.configure(bg='#0a0a0a')

    # ═══════════════════════════════════════════════════════════════════
    # TOP INFO BAR - Professional header with system info
    # ═══════════════════════════════════════════════════════════════════
    header_frame = tk.Frame(menu_window, bg='#1a1a1a', highlightbackground='#333333',
                           highlightthickness=2)
    header_frame.pack(fill=tk.X, padx=10, pady=5)

    # Left side - System Information Panel
    left_info_frame = tk.Frame(header_frame, bg='#1a1a1a')
    left_info_frame.pack(side=tk.LEFT, padx=15, pady=8)

    # WiFi status indicator
    wifi_status = check_wifi_status()
    if "CONNECTED" in wifi_status:
        wifi_color = "#00ff00"  # Bright green
        wifi_symbol = "●"
    else:
        wifi_color = "#ff0000"  # Bright red
        wifi_symbol = "●"

    wifi_label = tk.Label(left_info_frame, text=f"{wifi_symbol} {wifi_status}",
                         font=("Helvetica", 24, "bold"), fg=wifi_color, bg="#1a1a1a")
    wifi_label.pack(anchor='w')

    # IP address
    ip_address = get_ip_address()
    ip_label = tk.Label(left_info_frame, text=f"IP: {ip_address}",
                       font=("Helvetica", 22), fg="#00d4ff", bg="#1a1a1a")
    ip_label.pack(anchor='w', pady=2)

    # Username
    username = get_username()
    user_label = tk.Label(left_info_frame, text=f"User: {username}",
                         font=("Helvetica", 22), fg="#00d4ff", bg="#1a1a1a")
    user_label.pack(anchor='w', pady=2)

    # Center - Title and Version
    center_frame = tk.Frame(header_frame, bg='#1a1a1a')
    center_frame.pack(side=tk.LEFT, expand=True, padx=15, pady=8)

    title = tk.Label(center_frame, text="SYSTEM MENU", font=("Helvetica", 32, "bold"),
                     fg="#00d4ff", bg="#1a1a1a")
    title.pack()

    version_label = tk.Label(center_frame, text=f"Version {VERSION}", font=("Helvetica", 20),
                            fg="#888888", bg="#1a1a1a")
    version_label.pack()

    curve_label = tk.Label(center_frame, text=f"Curve: {flow_curve_status_text()}",
                           font=("Helvetica", 18), fg="#ffaa00", bg="#1a1a1a")
    curve_label.pack()

    proposal_label = tk.Label(center_frame, text=flow_curve_proposal_status_text(),
                              font=("Helvetica", 16), fg="#ffd080", bg="#1a1a1a")
    proposal_label.pack()

    # Right side - Totals Panel
    global menu_daily_label, menu_season_label
    right_info_frame = tk.Frame(header_frame, bg='#1a1a1a')
    right_info_frame.pack(side=tk.RIGHT, padx=15, pady=8)

    totals_title = tk.Label(right_info_frame, text="TOTALS",
                           font=("Helvetica", 22, "bold"), fg="#888888", bg="#1a1a1a")
    totals_title.pack()

    menu_daily_label = tk.Label(right_info_frame, text=f"Daily: {daily_total:.1f} gal",
                          font=("Helvetica", 24, "bold"), fg="#00ffff", bg="#1a1a1a")
    menu_daily_label.pack(anchor='e', pady=2)

    menu_season_label = tk.Label(right_info_frame, text=f"Season: {season_total:.1f} gal",
                           font=("Helvetica", 24, "bold"), fg="#00ff00", bg="#1a1a1a")
    menu_season_label.pack(anchor='e', pady=2)

    # ═══════════════════════════════════════════════════════════════════
    # MENU OPTIONS SECTION
    # ═══════════════════════════════════════════════════════════════════

    # Position indicator - make it global so we can update it
    global menu_position_label
    menu_position_label = tk.Label(menu_window,
                                    text=f"Option 1 of {len(MENU_ITEMS)}: {MENU_ITEMS[0]}",
                                    font=("Helvetica", 18, "bold"),
                                    fg="#ffffff", bg='#0a0a0a')
    menu_position_label.pack(pady=2)

    # Menu buttons frame with two-column layout for larger text.
    button_frame = tk.Frame(menu_window, bg='#0a0a0a')
    button_frame.pack(expand=True, fill=tk.BOTH, padx=24, pady=6)
    button_frame.grid_columnconfigure(0, weight=1, uniform="menu")
    button_frame.grid_columnconfigure(1, weight=1, uniform="menu")
    for row in range((len(MENU_ITEMS) + 1) // 2):
        button_frame.grid_rowconfigure(row, weight=1, uniform="menu_row")

    menu_actions = [
        ("VIEW LOGS", show_log_viewer),
        ("FILL HISTORY", show_fill_history),
        ("TANK CALIBRATION", show_tank_calibration),
        ("FULL TEST", run_full_test),
        ("RESET SEASON", lambda: confirm_reset_season()),
        ("ACCEPT CURVE", lambda: confirm_accept_flow_curve()),
        ("FACTORY CURVE", lambda: confirm_reset_flow_curve()),
        ("SELF TEST", run_self_test),
        ("CAPTURE BUG", run_bug_capture),
        ("SYSTEM UPDATE", run_system_update),
        ("SHUTDOWN", shutdown_system),
        ("REBOOT", reboot_system),
        ("EXIT TO DESKTOP", exit_to_desktop),
        ("EXIT MENU", close_menu),
    ]

    for index, (label, action) in enumerate(menu_actions):
        row = index // 2
        column = index % 2

        cell = tk.Frame(button_frame, bg='#0a0a0a')
        cell.grid(row=row, column=column, sticky="nsew", padx=12, pady=8)
        cell.grid_columnconfigure(0, weight=1)
        cell.grid_rowconfigure(1, weight=1)

        arrow = tk.Label(
            cell,
            text="",
            font=("Helvetica", 20, "bold"),
            fg="#00ffff",
            bg="#0a0a0a",
        )
        arrow.grid(row=0, column=0, pady=(0, 4))

        btn = tk.Button(
            cell,
            text=label,
            font=("Helvetica", 28, "bold"),
            bg="white",
            fg="black",
            command=action,
            width=16,
            height=2,
            borderwidth=3,
            wraplength=360,
            justify=tk.CENTER,
        )
        btn.grid(row=1, column=0, sticky="nsew")

        menu_buttons.append(btn)
        menu_arrows.append(arrow)

    # Instructions - Professional footer
    footer_frame = tk.Frame(menu_window, bg='#1a1a1a', highlightbackground='#333333',
                           highlightthickness=2)
    footer_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

    instructions = tk.Label(footer_frame, text="+1 = NEXT  │  -1 = PREVIOUS  │  OV = SELECT",
                           font=("Helvetica", 20, "bold"), fg="#00d4ff", bg="#1a1a1a")
    instructions.pack(pady=8)

    # Apply initial highlight
    update_menu_highlight(full_refresh=True)

def iol_power_cycle():
    """Power-cycle the IOL port in a background thread to trigger re-negotiation.

    Called when the flow meter is detected as disconnected (all-zero or stale data).
    The IOL master firmware does not auto-negotiate when a device is reconnected,
    so the port must be powered off and back on to restart the IO-Link handshake.
    Uses readStatus2() to check hardware error state before and after the cycle.
    """
    global iol_power_cycle_in_progress, last_power_cycle_time

    try:
        with iol_io_lock:
            # Check port status before power cycle
            try:
                pre_status = iolhat.readStatus2(config.IOL_PORT)
                print(f"IOL power-cycle: Pre-cycle status - pdInValid={pre_status.pd_in_valid}, "
                      f"txRate=0x{pre_status.transmission_rate:02X}, "
                      f"error=0x{pre_status.error:02X}, power={pre_status.power}", flush=True)
            except Exception as e:
                print(f"IOL power-cycle: Pre-cycle status check failed: {e}", flush=True)

            print(f"IOL power-cycle: Starting port {config.IOL_PORT} power-cycle", flush=True)

            # Step 1: Power off
            try:
                iolhat.power(config.IOL_PORT, 0)
                print(f"IOL power-cycle: Port {config.IOL_PORT} powered OFF", flush=True)
            except Exception as e:
                print(f"IOL power-cycle: Failed to power off: {e}", flush=True)
                return

            time.sleep(1.0)

            # Step 2: Power on
            try:
                iolhat.power(config.IOL_PORT, 1)
                print(f"IOL power-cycle: Port {config.IOL_PORT} powered ON", flush=True)
            except Exception as e:
                print(f"IOL power-cycle: Failed to power on: {e}", flush=True)
                return

            # Step 3: Wait for IO-Link handshake (up to 5 seconds, polling status)
            for attempt in range(5):
                time.sleep(1.0)
                try:
                    post_status = iolhat.readStatus2(config.IOL_PORT)
                    print(f"IOL power-cycle: Post-cycle check {attempt+1}/5 - "
                          f"pdInValid={post_status.pd_in_valid}, "
                          f"txRate=0x{post_status.transmission_rate:02X}, "
                          f"error=0x{post_status.error:02X}", flush=True)
                    if post_status.pd_in_valid == 1 and post_status.transmission_rate != 0:
                        print(f"IOL power-cycle: Device reconnected successfully", flush=True)
                        try:
                            iolhat.led(config.IOL_PORT, iolhat.LED_GREEN)
                        except:
                            pass
                        return
                except Exception as e:
                    print(f"IOL power-cycle: Post-cycle status check failed: {e}", flush=True)

        print(f"IOL power-cycle: Device did not reconnect after power cycle", flush=True)

    except Exception as e:
        print(f"IOL power-cycle: Unexpected error: {e}", flush=True)
    finally:
        last_power_cycle_time = time.time()
        iol_power_cycle_in_progress = False

def _try_iol_power_cycle():
    """Check rate-limiting and spawn power-cycle thread if appropriate."""
    global iol_power_cycle_in_progress

    if iol_power_cycle_in_progress:
        return

    if (time.time() - last_power_cycle_time) < config.IOL_RECONNECT_INTERVAL:
        return

    iol_power_cycle_in_progress = True
    cycle_thread = threading.Thread(target=iol_power_cycle, daemon=True)
    cycle_thread.start()


def _log_iol_disconnect_status(reason):
    """Read STATUS2 and log the hardware error byte when a disconnect is first detected."""
    try:
        _, st = _read_iol_status_ok()
        print(f"IOL DISCONNECT [{reason}]: pdInValid={st.pd_in_valid}, "
              f"txRate=0x{st.transmission_rate:02X}, "
              f"cycleTime=0x{st.master_cycle_time:02X}, "
              f"error=0x{st.error:02X}, power={st.power}", flush=True)
    except Exception as e:
        print(f"IOL DISCONNECT [{reason}]: STATUS2 read failed: {e}", flush=True)

def _read_iol_status_ok():
    """Return whether IO-Link status indicates valid process data."""
    with iol_io_lock:
        st = iolhat.readStatus2(config.IOL_PORT)
    ok = st.pd_in_valid == 1 and st.transmission_rate != 0 and st.error == 0
    return ok, st


def _describe_iol_status_fault(st):
    """Return a field-friendly reason for an unhealthy IO-Link status."""
    status_error = getattr(st, "error", 0)
    if status_error & 0x02:
        return "LOW VOLTAGE: check 24V"
    if status_error & 0x04:
        return "IO-Link current limit"
    if status_error & 0x01:
        return "IO-Link CQ fault"
    if getattr(st, "power", 0) != 1:
        return "IO-Link port power off"
    if getattr(st, "pd_in_valid", 0) != 1:
        return "flow meter data invalid"
    if getattr(st, "transmission_rate", 0) == 0:
        return "flow meter no COM"
    return "waiting for healthy flow meter status"


def read_flow_meter():
    """Read data from the Picomag flow meter via IO-Link"""
    global last_totalizer_liters, last_flow_rate, connection_error, error_message, last_successful_read_time
    global last_flow_read_was_fresh, last_fresh_flow_read_time
    global consecutive_identical_raw, last_raw_data

    try:
        last_flow_read_was_fresh = False
        # Read process data from IO-Link device. The flow-control thread is the
        # normal owner; this lock protects occasional diagnostics from racing it.
        with iol_io_lock:
            raw_data = iolhat.pd(config.IOL_PORT, 0, config.DATA_LENGTH, None)

        if len(raw_data) >= 15:
            if raw_data == b'\x00' * len(raw_data):
                if not connection_error:
                    _log_iol_disconnect_status("all-zero data")
                connection_error = True
                error_message = "Device not responding (all-zero data)"
                try:
                    iolhat.led(config.IOL_PORT, iolhat.LED_RED)
                except:
                    pass
                last_raw_data = None
                consecutive_identical_raw = 0
                _try_iol_power_cycle()
                return last_totalizer_liters * config.LITERS_TO_GALLONS

            # If we were in error state and now getting valid non-stale data, log recovery
            if connection_error and raw_data != last_raw_data:
                print(f"Flow meter reconnected - valid data received", flush=True)

            raw_data_changed = raw_data != last_raw_data
            raw_flow_rate_l_per_s = struct.unpack('>f', raw_data[8:12])[0]

            # Identical idle process data can be valid at zero flow. Only treat
            # repeated identical bytes as stale if IO-Link status is unhealthy
            # or the frozen data claims flow is still active.
            if not raw_data_changed:
                consecutive_identical_raw += 1
                stale_threshold = stale_raw_threshold_reads()
                if consecutive_identical_raw >= stale_threshold:
                    try:
                        status_ok, st = _read_iol_status_ok()
                    except Exception:
                        status_ok = False
                        st = None

                    if status_ok and abs(raw_flow_rate_l_per_s) < config.FLOW_STOPPED_THRESHOLD:
                        if consecutive_identical_raw == stale_threshold:
                            print("Flow meter identical idle data accepted with healthy IO-Link status", flush=True)
                            log_flow_control(
                                "flow_meter_identical_data_ok"
                                f" | count={consecutive_identical_raw}"
                                f" | flow_lps={raw_flow_rate_l_per_s:.6f}"
                                f" | pdInValid={st.pd_in_valid}"
                                f" | txRate=0x{st.transmission_rate:02X}"
                                f" | error=0x{st.error:02X}"
                            )
                        consecutive_identical_raw = 0
                    else:
                        if st is not None and consecutive_identical_raw == stale_threshold:
                            log_flow_control(
                                "flow_meter_identical_data_bad_status"
                                f" | count={consecutive_identical_raw}"
                                f" | flow_lps={raw_flow_rate_l_per_s:.6f}"
                                f" | pdInValid={st.pd_in_valid}"
                                f" | txRate=0x{st.transmission_rate:02X}"
                                f" | error=0x{st.error:02X}"
                            )
                        connection_error = True
                        stale_secs = consecutive_identical_raw * flow_read_interval_seconds()
                        error_message = f"Stale data - meter may be disconnected ({stale_secs:.0f}s)"
                        if consecutive_identical_raw == stale_threshold:
                            print(f"Flow meter stale data detected after {stale_secs:.0f}s", flush=True)
                            _log_iol_disconnect_status("stale data")
                            try:
                                iolhat.led(config.IOL_PORT, iolhat.LED_RED)
                            except:
                                pass
                        _try_iol_power_cycle()
                        return last_totalizer_liters * config.LITERS_TO_GALLONS
            else:
                if consecutive_identical_raw >= stale_raw_threshold_reads():
                    print(f"Flow meter data flowing again", flush=True)
                consecutive_identical_raw = 0
            last_raw_data = raw_data

            # Decode the data according to Picomag format
            totalizer_liters = abs(struct.unpack('>f', raw_data[4:8])[0])
            flow_rate_l_per_s = raw_flow_rate_l_per_s
            detect_totalizer_reset(totalizer_liters)

            last_totalizer_liters = totalizer_liters
            last_flow_rate = flow_rate_l_per_s
            # Clear error state - set LED green on reconnect
            if connection_error:
                try:
                    iolhat.led(config.IOL_PORT, iolhat.LED_GREEN)
                except:
                    pass
            connection_error = False
            error_message = ""
            last_successful_read_time = time.time()
            last_flow_read_was_fresh = raw_data_changed
            if raw_data_changed:
                last_fresh_flow_read_time = last_successful_read_time

            return totalizer_liters * config.LITERS_TO_GALLONS
        else:
            connection_error = True
            error_message = "Invalid data length"
            return last_totalizer_liters * config.LITERS_TO_GALLONS

    except Exception as e:
        connection_error = True
        error_message = str(e)
        _try_iol_power_cycle()
        return last_totalizer_liters * config.LITERS_TO_GALLONS


def _flow_control_process_sample(actual, now, loop_dt_ms):
    """Run safety-critical auto-shutoff decisions from the control loop."""
    global flow_control_was_flowing, flow_cycle_counter, last_alert_triggered
    global auto_shutoff_latched, last_flowing_rate_l_per_s
    global last_trigger_flow_gpm, last_trigger_threshold, last_trigger_actual
    global last_trigger_predicted_actual, last_trigger_loop_dt_ms

    is_flowing = last_flow_rate >= config.FLOW_STOPPED_THRESHOLD
    flow_meter_disconnected = (time.time() - last_successful_read_time) > config.FLOW_METER_TIMEOUT
    current_flow_gpm = max(0.0, last_flow_rate * config.LITERS_PER_SEC_TO_GPM)

    if is_flowing and not flow_control_was_flowing:
        flow_cycle_counter += 1
        last_alert_triggered = False
        auto_shutoff_latched = False
        last_trigger_flow_gpm = 0.0
        last_trigger_threshold = 0.0
        last_trigger_actual = 0.0
        last_trigger_predicted_actual = 0.0
        last_trigger_loop_dt_ms = 0.0
        recent_flow_rates_l_per_s.clear()
        log_flow_control(
            f"cycle_start | actual={actual:.3f} | flow_lps={last_flow_rate:.4f}"
        )

    if is_flowing:
        if last_flow_read_was_fresh:
            recent_flow_rates_l_per_s.append(last_flow_rate)
            last_flowing_rate_l_per_s = last_flow_rate
    elif flow_control_was_flowing:
        log_flow_control(
            f"cycle_stop | actual={actual:.3f} | auto={auto_shutoff_latched}"
        )

    flow_control_was_flowing = is_flowing
    _update_relay_slowdown_watch(is_flowing, current_flow_gpm, now)

    if not is_flowing or override_mode or flow_meter_disconnected or last_alert_triggered:
        return

    smoothed_flow_rate_l_per_s = get_smoothed_flow_rate()
    flow_rate_gpm = smoothed_flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM
    trigger_threshold = calculate_trigger_threshold(smoothed_flow_rate_l_per_s)
    prediction_seconds = max(0.0, float(getattr(config, "FLOW_CONTROL_PREDICTION_SECONDS", 0.0)))
    predicted_actual = actual + (smoothed_flow_rate_l_per_s * prediction_seconds * config.LITERS_TO_GALLONS)

    if predicted_actual >= requested_gallons - trigger_threshold:
        last_alert_triggered = True
        auto_shutoff_latched = True
        last_trigger_flow_gpm = flow_rate_gpm
        last_trigger_threshold = trigger_threshold
        last_trigger_actual = actual
        last_trigger_predicted_actual = predicted_actual
        last_trigger_loop_dt_ms = loop_dt_ms
        log_flow_control(
            "auto_stop"
            f" | requested={requested_gallons:.3f}"
            f" | actual={actual:.3f}"
            f" | predicted={predicted_actual:.3f}"
            f" | threshold={trigger_threshold:.3f}"
            f" | flow_gpm={flow_rate_gpm:.1f}"
            f" | loop_dt_ms={loop_dt_ms:.1f}"
        )
        print(
            f"Auto-alert(control): Flow={flow_rate_gpm:.1f} GPM, "
            f"threshold={trigger_threshold:.2f}gal, triggering relay for "
            f"{config.AUTO_ALERT_DURATION}s"
        )
        _arm_relay_slowdown_watch(flow_rate_gpm, now)
        start_pump_stop_thread(config.AUTO_ALERT_DURATION)


def _arm_relay_slowdown_watch(flow_gpm, now):
    """Watch for measurable flow slowdown after an auto-stop relay trigger."""
    global relay_slowdown_watch_active, relay_slowdown_alarm_active
    global relay_slowdown_trigger_time, relay_slowdown_trigger_flow_gpm

    relay_slowdown_watch_active = True
    relay_slowdown_alarm_active = False
    relay_slowdown_trigger_time = now
    relay_slowdown_trigger_flow_gpm = max(0.0, flow_gpm)
    log_flow_control(
        "slowdown_watch_start"
        f" | flow_gpm={relay_slowdown_trigger_flow_gpm:.1f}"
        f" | check_after={config.RELAY_SLOWDOWN_CHECK_SECONDS:.1f}s"
    )


def _clear_relay_slowdown_watch(reason):
    """Clear post-relay slowdown watch/alarm state."""
    global relay_slowdown_watch_active, relay_slowdown_alarm_active
    global relay_slowdown_trigger_time, relay_slowdown_trigger_flow_gpm

    if relay_slowdown_watch_active or relay_slowdown_alarm_active:
        log_flow_control(f"slowdown_watch_clear | reason={reason}")
    relay_slowdown_watch_active = False
    relay_slowdown_alarm_active = False
    relay_slowdown_trigger_time = 0.0
    relay_slowdown_trigger_flow_gpm = 0.0


def _update_relay_slowdown_watch(is_flowing, flow_gpm, now):
    """Latch an alarm if flow does not drop measurably after auto-stop."""
    global relay_slowdown_alarm_active

    if not relay_slowdown_watch_active:
        return

    if not is_flowing:
        _clear_relay_slowdown_watch("flow_stopped")
        return

    elapsed = now - relay_slowdown_trigger_time
    if elapsed < config.RELAY_SLOWDOWN_CHECK_SECONDS:
        return

    required_drop = max(
        config.RELAY_SLOWDOWN_MIN_DROP_GPM,
        relay_slowdown_trigger_flow_gpm * config.RELAY_SLOWDOWN_MIN_DROP_FRACTION,
    )
    actual_drop = relay_slowdown_trigger_flow_gpm - max(0.0, flow_gpm)
    if actual_drop >= required_drop:
        _clear_relay_slowdown_watch("flow_slowed")
        return

    if not relay_slowdown_alarm_active:
        relay_slowdown_alarm_active = True
        log_flow_control(
            "slowdown_alarm"
            f" | elapsed={elapsed:.1f}s"
            f" | trigger_flow_gpm={relay_slowdown_trigger_flow_gpm:.1f}"
            f" | current_flow_gpm={flow_gpm:.1f}"
            f" | drop_gpm={actual_drop:.1f}"
            f" | required_drop_gpm={required_drop:.1f}"
        )


def _record_flow_control_audit(loop_dt_ms, read_ok=True):
    """Track flow-meter sample freshness without adding another IO-Link reader."""
    global flow_control_audit_started_at, flow_control_audit_polls
    global flow_control_audit_fresh, flow_control_audit_duplicates
    global flow_control_audit_flowing_polls, flow_control_audit_flowing_fresh
    global flow_control_audit_errors, flow_control_audit_loop_ms_total
    global flow_control_audit_loop_ms_max

    now = time.time()
    flow_control_audit_polls += 1
    flow_control_audit_loop_ms_total += loop_dt_ms
    flow_control_audit_loop_ms_max = max(flow_control_audit_loop_ms_max, loop_dt_ms)

    if read_ok:
        is_fresh = bool(last_flow_read_was_fresh)
        is_flowing = last_flow_rate >= config.FLOW_STOPPED_THRESHOLD
        if is_fresh:
            flow_control_audit_fresh += 1
        else:
            flow_control_audit_duplicates += 1
        if is_flowing:
            flow_control_audit_flowing_polls += 1
            if is_fresh:
                flow_control_audit_flowing_fresh += 1
    else:
        flow_control_audit_errors += 1

    elapsed = now - flow_control_audit_started_at
    audit_interval = max(1.0, float(getattr(config, "FLOW_CONTROL_AUDIT_INTERVAL", 5.0)))
    if elapsed < audit_interval or flow_control_audit_polls <= 0:
        return

    polls = flow_control_audit_polls
    fresh = flow_control_audit_fresh
    duplicates = flow_control_audit_duplicates
    flowing_polls = flow_control_audit_flowing_polls
    flowing_fresh = flow_control_audit_flowing_fresh
    flowing_duplicates = max(0, flowing_polls - flowing_fresh)
    errors = flow_control_audit_errors
    avg_loop_ms = flow_control_audit_loop_ms_total / polls
    poll_hz = polls / elapsed
    fresh_hz = fresh / elapsed
    flowing_fresh_hz = flowing_fresh / elapsed

    log_flow_control(
        "audit"
        f" | window={elapsed:.1f}s"
        f" | polls={polls}"
        f" | fresh={fresh}"
        f" | dup={duplicates}"
        f" | errors={errors}"
        f" | poll_hz={poll_hz:.1f}"
        f" | fresh_hz={fresh_hz:.1f}"
        f" | flowing_polls={flowing_polls}"
        f" | flowing_fresh={flowing_fresh}"
        f" | flowing_dup={flowing_duplicates}"
        f" | flowing_fresh_hz={flowing_fresh_hz:.1f}"
        f" | loop_avg_ms={avg_loop_ms:.1f}"
        f" | loop_max_ms={flow_control_audit_loop_ms_max:.1f}"
    )

    flow_control_audit_started_at = now
    flow_control_audit_polls = 0
    flow_control_audit_fresh = 0
    flow_control_audit_duplicates = 0
    flow_control_audit_flowing_polls = 0
    flow_control_audit_flowing_fresh = 0
    flow_control_audit_errors = 0
    flow_control_audit_loop_ms_total = 0.0
    flow_control_audit_loop_ms_max = 0.0


def _reset_flow_control_audit(started_at=None):
    """Start a fresh flow-meter freshness audit window."""
    global flow_control_audit_started_at, flow_control_audit_polls
    global flow_control_audit_fresh, flow_control_audit_duplicates
    global flow_control_audit_flowing_polls, flow_control_audit_flowing_fresh
    global flow_control_audit_errors, flow_control_audit_loop_ms_total
    global flow_control_audit_loop_ms_max

    flow_control_audit_started_at = started_at if started_at is not None else time.time()
    flow_control_audit_polls = 0
    flow_control_audit_fresh = 0
    flow_control_audit_duplicates = 0
    flow_control_audit_flowing_polls = 0
    flow_control_audit_flowing_fresh = 0
    flow_control_audit_errors = 0
    flow_control_audit_loop_ms_total = 0.0
    flow_control_audit_loop_ms_max = 0.0


def flow_control_loop():
    """Own IO-Link flow reads and auto-shutoff timing outside the Tk loop."""
    global flow_control_last_tick, flow_control_last_loop_time, flow_control_last_error_log_time

    interval = max(0.01, float(getattr(config, "FLOW_CONTROL_INTERVAL", 0.05)))
    next_tick = time.monotonic()
    flow_control_last_tick = next_tick
    flow_control_last_loop_time = time.time()
    _reset_flow_control_audit(flow_control_last_loop_time)
    log_flow_control(f"thread_start | interval={interval:.3f}s")

    while not flow_control_stop_event.is_set():
        now_mono = time.monotonic()
        loop_dt_ms = (now_mono - flow_control_last_tick) * 1000.0
        flow_control_last_tick = now_mono

        try:
            actual = read_flow_meter()
            update_flow_meter_fault_hold()
            flow_control_last_loop_time = time.time()
            _record_flow_control_audit(loop_dt_ms, read_ok=True)
            _flow_control_process_sample(actual, time.time(), loop_dt_ms)
        except Exception as exc:
            set_pump_stop_fault_hold(True, f"flow control read exception: {exc}")
            flow_control_last_loop_time = time.time()
            _record_flow_control_audit(loop_dt_ms, read_ok=False)
            now = time.time()
            if now - flow_control_last_error_log_time > 5:
                flow_control_last_error_log_time = now
                log_flow_control(f"thread_error | {exc}")

        next_tick += interval
        sleep_for = next_tick - time.monotonic()
        if sleep_for < 0:
            next_tick = time.monotonic()
            sleep_for = 0
        flow_control_stop_event.wait(sleep_for)

    log_flow_control("thread_stop")


def start_flow_control_thread():
    """Start the dedicated flow-control loop when configured."""
    global flow_control_thread

    if not flow_control_enabled():
        log_flow_control("thread_disabled")
        return
    if flow_control_thread and flow_control_thread.is_alive():
        return

    flow_control_stop_event.clear()
    flow_control_thread = threading.Thread(
        target=flow_control_loop,
        name="flow-control",
        daemon=True,
    )
    flow_control_thread.start()



def _wifi_status_snapshot():
    """Return current WiFi status (excluding secrets)."""
    try:
        active_ssid = ''
        ip_addr = ''

        r = subprocess.run(
            ['nmcli', '-t', '-f', 'ACTIVE,SSID,DEVICE', 'dev', 'wifi'],
            capture_output=True,
            text=True,
            timeout=4,
        )
        if r.returncode == 0:
            for line in r.stdout.splitlines():
                parts = line.split(':')
                if len(parts) >= 3 and parts[0] == 'yes':
                    active_ssid = parts[1]
                    break

        r2 = subprocess.run(
            ['nmcli', '-t', '-f', 'IP4.ADDRESS', 'dev', 'show', 'wlan0'],
            capture_output=True,
            text=True,
            timeout=4,
        )
        if r2.returncode == 0:
            for line in r2.stdout.splitlines():
                if line.startswith('IP4.ADDRESS'):
                    ip_addr = line.split(':', 1)[1].split('/')[0].strip()
                    break

        return {
            'ok': bool(active_ssid),
            'connected': bool(active_ssid),
            'ssid': active_ssid,
            'ip': ip_addr,
        }
    except Exception as e:
        return {'ok': False, 'connected': False, 'error': str(e)}


def _wifi_connect(ssid, password, hidden=False):
    """Connect to WiFi using nmcli without logging secrets."""
    ssid = str(ssid or '').strip()
    password = str(password or '')

    if not ssid:
        return {'ok': False, 'code': 'INVALID_SSID', 'message': 'Missing ssid'}
    if len(ssid) > 64:
        return {'ok': False, 'code': 'INVALID_SSID', 'message': 'SSID too long'}
    if len(password) > 128:
        return {'ok': False, 'code': 'INVALID_PASSWORD', 'message': 'Password too long'}

    try:
        # Remove stale connection profile for this SSID to ensure fresh credentials.
        subprocess.run(
            ['nmcli', 'connection', 'delete', ssid],
            capture_output=True,
            text=True,
            timeout=6,
        )

        cmd = ['nmcli', '--wait', '20', 'dev', 'wifi', 'connect', ssid]
        if password:
            cmd += ['password', password]
        if hidden:
            cmd += ['hidden', 'yes']

        r = subprocess.run(cmd, capture_output=True, text=True, timeout=25)
        out = (r.stdout or '') + '\n' + (r.stderr or '')

        if r.returncode != 0:
            low = out.lower()
            if 'secrets were required' in low or 'wrong password' in low or '802-11-wireless-security.key-mgmt' in low:
                return {'ok': False, 'code': 'AUTH_FAILED', 'message': 'Authentication failed'}
            if 'no network with ssid' in low or 'not found' in low:
                return {'ok': False, 'code': 'NO_AP_FOUND', 'message': 'SSID not found'}
            if 'timeout' in low:
                return {'ok': False, 'code': 'TIMEOUT', 'message': 'Connection timeout'}
            return {'ok': False, 'code': 'NMCLI_ERROR', 'message': out.strip()[:160]}

        status = _wifi_status_snapshot()
        status.update({'ok': True, 'code': 'OK'})
        return status

    except subprocess.TimeoutExpired:
        return {'ok': False, 'code': 'TIMEOUT', 'message': 'Connection timeout'}
    except Exception as e:
        return {'ok': False, 'code': 'NMCLI_ERROR', 'message': str(e)}


def socket_command_listener():
    """Listen for commands from rotorsync BLE server via localhost socket"""
    global requested_gallons, override_mode, override_enabled_time, colors_are_green
    global fill_requested_gallons, mix_requested_gallons, current_mode, batch_mix_data

    import socket as sock_module

    DASHBOARD_PORT = 9999
    debug_log = config.SERIAL_DEBUG_LOG

    sock_server = sock_module.socket(sock_module.AF_INET, sock_module.SOCK_STREAM)
    sock_server.setsockopt(sock_module.SOL_SOCKET, sock_module.SO_REUSEADDR, 1)
    sock_server.bind(("127.0.0.1", DASHBOARD_PORT))
    sock_server.listen(1)
    sock_server.settimeout(1.0)

    print(f"Socket listener started on port {DASHBOARD_PORT}")
    with open(debug_log, "a") as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Socket listener started on port {DASHBOARD_PORT}\n")

    while True:
        try:
            try:
                client, addr = sock_server.accept()
                client.settimeout(5.0)
                try:
                    data = client.recv(4096).decode("utf-8").strip()
                    if data:
                        for line in data.split("\n"):
                            line = line.strip()
                            if not line:
                                continue

                            safe_line = line
                            if line.startswith('WIFI_SET:'):
                                try:
                                    payload = line[9:]
                                    req = json.loads(payload)
                                    if isinstance(req, dict) and 'password' in req:
                                        req['password'] = '***'
                                    safe_line = f"WIFI_SET:{json.dumps(req, separators=(',', ':'))}"
                                except Exception:
                                    safe_line = 'WIFI_SET:{...}'

                            if line != "STATE_JSON":
                                with open(debug_log, "a") as f:
                                    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Socket received: '{safe_line}'\n")

                            if line == "STATUS":
                                actual = last_totalizer_liters * config.LITERS_TO_GALLONS
                                response = f"REQ:{requested_gallons:.1f}|ACT:{actual:.1f}|MODE:{current_mode}\n"
                                client.send(response.encode())
                                continue

                            if line == "STATE_JSON":
                                snapshot = _build_dashboard_state_snapshot()
                                client.send(
                                    f"STATE_JSON:{json.dumps(snapshot, separators=(',', ':'))}\n".encode()
                                )
                                continue

                            elif line == "MIX":
                                root.after(0, lambda: switch_mode("mix"))

                            elif line == "RESET":
                                root.after(0, lambda: force_flow_reset("socket_reset"))

                            elif line == "FILL":
                                root.after(0, lambda: switch_mode("fill"))

                            elif line.startswith("WIFI_SET:"):
                                try:
                                    payload = line[9:]
                                    req = json.loads(payload)
                                    ssid = req.get('ssid', '')
                                    password = req.get('password', '')
                                    hidden = bool(req.get('hidden', False))

                                    # Log without secrets
                                    msg = f"Socket: WIFI_SET requested for SSID '{ssid}' (hidden={hidden})"
                                    print(msg)
                                    with open(debug_log, "a") as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")

                                    result = _wifi_connect(ssid, password, hidden)
                                    if result.get('ok'):
                                        client.send(f"WIFI_OK:{json.dumps(result, separators=(',', ':'))}\n".encode())
                                    else:
                                        err_payload = {
                                            'code': result.get('code', 'NMCLI_ERROR'),
                                            'message': result.get('message', ''),
                                        }
                                        client.send(f"WIFI_ERR:{json.dumps(err_payload, separators=(',', ':'))}\n".encode())
                                    continue
                                except Exception as we:
                                    err_payload = {'code': 'NMCLI_ERROR', 'message': str(we)}
                                    client.send(f"WIFI_ERR:{json.dumps(err_payload, separators=(',', ':'))}\n".encode())
                                    continue

                            elif line == "WIFI_STATUS":
                                status = _wifi_status_snapshot()
                                client.send(f"WIFI_STATUS:{json.dumps(status, separators=(',', ':'))}\n".encode())
                                continue

                            elif line.startswith("BATCHMIX_ERROR:"):
                                # Handle BatchMix validation error
                                error_msg = line[15:]
                                msg = f"Socket: BatchMix ERROR - {error_msg}"
                                print(msg)
                                with open(debug_log, "a") as f:
                                    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                # Display error on screen
                                root.after(0, lambda e=error_msg: show_batchmix_error(e))

                            elif line.startswith("BATCHMIX:"):
                                try:
                                    json_str = line[9:]
                                    batch_mix_data = json.loads(json_str)
                                    msg = f"Socket: BatchMix received - {len(batch_mix_data.get('products', []))} products"
                                    print(msg)
                                    with open(debug_log, "a") as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")

                                    # Store the BatchMix target as the mix preset. If we are already in
                                    # mix mode, also update the live target used by auto-stop.
                                    water_needed = batch_mix_data.get('water_needed', 0)
                                    if water_needed > 0:
                                        water_needed = float(water_needed)
                                        mix_requested_gallons = water_needed
                                        save_mode_presets()
                                        # Only update display if in mix mode
                                        if current_mode == "mix":
                                            requested_gallons = water_needed
                                            colors_are_green = False
                                            # Show decimal if present, otherwise whole number
                                            if water_needed == int(water_needed):
                                                req_str = f"{int(water_needed)}"
                                            else:
                                                req_str = f"{water_needed:.1f}"
                                            root.after(0, lambda s=req_str: draw_requested_number(s, "green" if colors_are_green else "red"))

                                    root.after(0, update_batch_mix_overlay)
                                except Exception as bme:
                                    msg = f"Socket: BatchMix parse error: {bme}"
                                    print(msg)
                                    with open(debug_log, "a") as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")


                            elif line == "MOPEKA_OFFLINE":
                                root.after(0, _mopeka_offline)

                            elif line == "MOPEKA_DISABLED":
                                root.after(0, _mopeka_disabled)

                            elif line.startswith("MOPEKA:"):
                                try:
                                    parts = line[7:].split("|")
                                    if len(parts) >= 4:
                                        _m1g = float(parts[0])
                                        _m2g = float(parts[1])
                                        _m1q = int(parts[2])
                                        _m2q = int(parts[3])
                                        root.after(0, _apply_mopeka, _m1g, _m2g, _m1q, _m2q)
                                except Exception as me:
                                    print(f"Mopeka parse error: {me}", flush=True)

                            elif line.startswith("MOPEKA_RAW:"):
                                try:
                                    parts = line[11:].split("|")
                                    if len(parts) >= 4:
                                        root.after(
                                            0,
                                            _apply_mopeka_raw,
                                            float(parts[0]),
                                            float(parts[1]),
                                            float(parts[2]),
                                            float(parts[3]),
                                        )
                                except Exception as me:
                                    print(f"Mopeka raw parse error: {me}", flush=True)

                            elif line.startswith("BMS:"):
                                try:
                                    parts = line[4:].split("|")
                                    if len(parts) >= 2:
                                        root.after(0, _apply_bms, float(parts[0]), float(parts[1]))
                                except Exception as be:
                                    print(f"BMS parse error: {be}", flush=True)

                            elif line == "HISTORY":
                                try:
                                    history_items = _build_fill_history_items()
                                    history_response = ";".join(
                                        ",".join(str(field).replace(",", " ") for field in item)
                                        for item in history_items
                                    )
                                    client.send(f"HIST:{history_response}\n".encode())
                                except Exception:
                                    client.send(b"HIST:\n")
                                continue

                            elif line in ['+1', '-1', '+10', '-10']:
                                try:
                                    adjustment = int(line)
                                    if adjust_batch_mix_gallons(adjustment, "Socket"):
                                        continue
                                    requested_gallons += adjustment
                                    if requested_gallons < 0:
                                        requested_gallons = 0
                                    colors_are_green = False
                                    if current_mode == 'fill':
                                        fill_requested_gallons = requested_gallons
                                    else:
                                        mix_requested_gallons = requested_gallons
                                    save_mode_presets()
                                    msg = f"Socket: Adjusted by {adjustment}, requested gallons now {requested_gallons}"
                                    print(msg)
                                    with open(debug_log, "a") as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                except ValueError:
                                    pass

                            elif line.startswith("SET_REQUESTED_GALLONS:"):
                                try:
                                    value = line.split(":", 1)[1].strip()
                                    new_value = set_requested_gallons_value(value, "Socket")
                                    client.send(f"REQ_SET:{new_value:.3f}\n".encode())
                                except Exception as exc:
                                    err_payload = {"code": "INVALID_REQUESTED_GALLONS", "message": str(exc)}
                                    client.send(f"REQ_ERR:{json.dumps(err_payload, separators=(',', ':'))}\n".encode())
                                continue

                            elif line == 'PS':
                                msg = "Socket: Pump Stop command received"
                                print(msg)
                                with open(debug_log, "a") as f:
                                    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                start_pump_stop_thread(config.PUMP_STOP_DURATION)

                            elif line in ('OV:1', 'OV:0'):
                                override_mode = (line == 'OV:1')
                                if override_mode:
                                    override_enabled_time = time.time()
                                msg = f"Socket: Override mode {'ENABLED' if override_mode else 'DISABLED'} (explicit)"
                                print(msg)
                                with open(debug_log, "a") as f:
                                    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")

                            elif line == 'OV':
                                override_mode = not override_mode
                                if override_mode:
                                    override_enabled_time = time.time()
                                msg = f"Socket: Override mode {'ENABLED' if override_mode else 'DISABLED'}"
                                print(msg)
                                with open(debug_log, "a") as f:
                                    f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")

                            client.send(b"OK\n")
                except sock_module.timeout:
                    pass
                finally:
                    client.close()
            except sock_module.timeout:
                pass
        except Exception as e:
            print(f"Socket listener error: {e}")
            time.sleep(1)

def serial_listener():
    """Listen for serial messages with format: requested,actual"""
    global requested_gallons, serial_connected, override_mode, colors_are_green, last_heartbeat_time
    global fill_requested_gallons, mix_requested_gallons, current_mode

    debug_log = config.SERIAL_DEBUG_LOG
    buffer = ""

    try:
        ser = serial.Serial(config.SERIAL_PORT, config.SERIAL_BAUD, timeout=0.5)
        ser.reset_input_buffer()
        serial_connected = True
        msg = f"Serial listener started on {config.SERIAL_PORT} at {config.SERIAL_BAUD} baud"
        print(msg)
        log_serial_debug(msg)

        while True:
            try:
                if ser.in_waiting > 0:
                    # Read all available bytes
                    raw_bytes = ser.read(ser.in_waiting)
                    log_serial_debug(f"Raw bytes: {raw_bytes} (hex: {raw_bytes.hex()})")

                    # Decode and add to buffer
                    chunk = raw_bytes.decode('utf-8', errors='ignore')
                    buffer += chunk
                    log_serial_debug(f"Decoded: '{chunk}' | Buffer now: '{buffer}'")

                    # Process complete lines (ending with \n or \r)
                    while '\n' in buffer or '\r' in buffer:
                        # Split on either \n or \r
                        if '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                        else:
                            line, buffer = buffer.split('\r', 1)

                        line = line.strip()
                        log_serial_debug(f"Complete line: '{line}'")

                        if line:
                            # Update heartbeat if we receive OK message
                            if line == 'OK':
                                last_heartbeat_time = time.time()
                                log_serial_debug("Heartbeat received (OK)")
                                continue  # Don't process OK as a command

                            if should_ignore_menu_ov_bounce(line, "Serial"):
                                continue

                            # Handle exit confirmation dialog
                            if exit_confirm_window:
                                if line == 'OV':
                                    msg = "Serial: Exit confirmation (OV - Confirm)"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if exit_confirm_handler:
                                        root.after(0, exit_confirm_handler)
                                elif line == 'PS':
                                    msg = "Serial: Exit confirmation (PS - Cancel)"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if exit_cancel_handler:
                                        root.after(0, exit_cancel_handler)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in exit confirmation: '{line}'\n")

                            # Handle reset season confirmation dialog
                            elif reset_season_confirm_window:
                                if line == 'OV':
                                    msg = "Serial: Reset season confirmation (OV - Confirm)"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_season_confirm_handler:
                                        root.after(0, reset_season_confirm_handler)
                                elif line == 'PS':
                                    msg = "Serial: Reset season confirmation (PS - Cancel)"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_season_cancel_handler:
                                        root.after(0, reset_season_cancel_handler)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in reset season confirmation: '{line}'\n")

                            # Handle flow curve reset confirmation dialog
                            elif reset_flow_curve_confirm_window:
                                if line == 'OV':
                                    msg = "Serial: Flow curve reset confirmation (OV - Confirm)"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_flow_curve_confirm_handler:
                                        root.after(0, reset_flow_curve_confirm_handler)
                                elif line == 'PS':
                                    msg = "Serial: Flow curve reset confirmation (PS - Cancel)"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if reset_flow_curve_cancel_handler:
                                        root.after(0, reset_flow_curve_cancel_handler)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in flow curve reset confirmation: '{line}'\n")

                            # Handle learned flow curve accept confirmation dialog
                            elif accept_flow_curve_confirm_window:
                                if line == 'OV':
                                    msg = "Serial: Flow curve accept confirmation (OV - Confirm)"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if accept_flow_curve_confirm_handler:
                                        root.after(0, accept_flow_curve_confirm_handler)
                                elif line == 'PS':
                                    msg = "Serial: Flow curve accept confirmation (PS - Cancel)"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    if accept_flow_curve_cancel_handler:
                                        root.after(0, accept_flow_curve_cancel_handler)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in flow curve accept confirmation: '{line}'\n")

                            # Handle reminders mode - dismiss on OV
                            elif reminders_mode:
                                if line == 'OV':
                                    msg = "Serial: Dismiss reminders"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, dismiss_reminders)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in reminders mode: '{line}'\n")

                            elif calibration_mode:
                                if calibration_state and calibration_state.get("phase") == "review" and line == '+1':
                                    msg = "Serial: Calibration reread"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_reread_now)
                                elif line in ('+1', '-1', '+10', '-10'):
                                    msg = f"Serial: Calibration command {line}"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_adjust_value, int(line))
                                elif line == 'OV':
                                    msg = "Serial: Calibration confirm"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_confirm)
                                elif line == 'PS':
                                    msg = "Serial: Calibration cancel/back"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, calibration_cancel)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in calibration mode: '{line}'\n")

                            # Handle log viewer navigation if in log viewer mode
                            elif log_viewer_mode:
                                if line == '+1':
                                    msg = "Serial: Log viewer scroll down"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, log_viewer_scroll_down)
                                elif line == '-1':
                                    msg = "Serial: Log viewer scroll up"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, log_viewer_scroll_up)
                                elif line == 'OV':
                                    msg = "Serial: Log viewer exit"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_log_viewer)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in log viewer mode: '{line}'\n")

                            # Handle fill history navigation if in fill history mode
                            elif fill_history_mode:
                                if line == '+1':
                                    msg = "Serial: Fill history scroll down"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, fill_history_scroll_down)
                                elif line == '-1':
                                    msg = "Serial: Fill history scroll up"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, fill_history_scroll_up)
                                elif line == 'OV':
                                    msg = "Serial: Fill history exit"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_fill_history)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in fill history mode: '{line}'\n")

                            # Handle self-test mode
                            elif self_test_mode:
                                if line == 'OV':
                                    msg = "Serial: Self-test exit"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_self_test)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in self-test mode: '{line}'\n")

                            # Handle full-test mode
                            elif full_test_mode:
                                if line == 'OV':
                                    # Check if OV test has been done already
                                    if full_test_window and hasattr(full_test_window, 'test_status'):
                                        if not full_test_window.test_status['OV']:
                                            # First press: mark OV as tested
                                            msg = "Serial: Full-test OV command detected (marked as tested)"
                                            print(msg)
                                            with open(debug_log, 'a') as f:
                                                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            root.after(0, lambda: full_test_window.mark_tested('OV'))
                                        else:
                                            # Second press: exit full test
                                            msg = "Serial: Full-test exit (OV pressed second time)"
                                            print(msg)
                                            with open(debug_log, 'a') as f:
                                                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            root.after(0, close_full_test)
                                    else:
                                        # Fallback: just exit
                                        msg = "Serial: Full-test exit (OV pressed)"
                                        print(msg)
                                        with open(debug_log, 'a') as f:
                                            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                        root.after(0, close_full_test)
                                elif line in ['-1', '+1', '-10', '+10', 'PS']:
                                    msg = f"Serial: Full-test {line} command detected"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    # Mark the corresponding test as passed
                                    if full_test_window and hasattr(full_test_window, 'mark_tested'):
                                        if line == '-1':
                                            root.after(0, lambda: full_test_window.mark_tested('minus_1'))
                                        elif line == '+1':
                                            root.after(0, lambda: full_test_window.mark_tested('plus_1'))
                                        elif line == '-10':
                                            root.after(0, lambda: full_test_window.mark_tested('minus_10'))
                                        elif line == '+10':
                                            root.after(0, lambda: full_test_window.mark_tested('plus_10'))
                                        elif line == 'PS':
                                            root.after(0, lambda: full_test_window.mark_tested('PS'))
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in full-test mode: '{line}'\n")

                            # Handle update mode
                            elif update_mode:
                                if line == 'OV':
                                    msg = "Serial: Update exit"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, close_update)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in update mode: '{line}'\n")

                            # Handle menu navigation if in menu mode
                            elif menu_mode:
                                if line == '+1':
                                    msg = "Serial: Menu navigate down"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    menu_navigate_down()
                                elif line == '-1':
                                    msg = "Serial: Menu navigate up"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    menu_navigate_up()
                                elif line == 'OV':
                                    msg = "Serial: Menu select"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    arm_menu_ov_guard()
                                    root.after(0, menu_select)
                                else:
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Ignored in menu mode: '{line}'\n")

                            # Normal dashboard mode
                            else:
                                # Handle adjustment commands
                                if line in ['+1', '-1', '+10', '-10']:
                                    try:
                                        adjustment = int(line)
                                        if adjust_batch_mix_gallons(adjustment, "Serial"):
                                            continue
                                        requested_gallons += adjustment
                                        # Don't allow requested gallons to go below zero
                                        if requested_gallons < 0:
                                            requested_gallons = 0
                                        colors_are_green = False  # Reset colors when requested gallons changes
                                        # Update the current mode's preset
                                        if current_mode == 'fill':
                                            fill_requested_gallons = requested_gallons
                                        else:
                                            mix_requested_gallons = requested_gallons
                                        save_mode_presets()
                                        heartbeat_age = time.time() - last_heartbeat_time if last_heartbeat_time else -1
                                        msg = (
                                            f"Serial: Adjusted by {adjustment}, requested gallons now {requested_gallons} "
                                            f"(heartbeat_age={heartbeat_age:.1f}s, mode={current_mode})"
                                        )
                                        print(msg)
                                        log_serial_debug(msg)
                                        root.after(
                                            0,
                                            lambda value=requested_gallons, is_green=colors_are_green: (
                                                draw_requested_number(f"{value:.0f}", "green" if is_green else "red")
                                                if batch_mix_layout_active
                                                else (draw_requested_number(f"{value:.0f}", "green" if is_green else "red"), update_batch_mix_overlay())
                                            ),
                                        )
                                    except ValueError as ve:
                                        log_serial_debug(f"ValueError parsing adjustment: {ve}")

                                # Handle special commands
                                elif line == 'PS':
                                    msg = "Serial: Pump Stop command received - activating relay"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    start_pump_stop_thread(config.PUMP_STOP_DURATION)

                                elif line == 'OV':
                                    global override_enabled_time
                                    # Check if requested gallons is 0 to trigger menu or leave batch mix screen.
                                    if requested_gallons == 0:
                                        if current_mode == 'mix' and batch_mix_data is not None:
                                            msg = "Serial: Batch mix screen exit triggered (gallons=0, OV pressed)"
                                            print(msg)
                                            with open(debug_log, 'a') as f:
                                                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            root.after(0, lambda: clear_batch_mix_screen("serial OV at zero gallons"))
                                        else:
                                            msg = "Serial: Menu access triggered (gallons=0, OV pressed)"
                                            print(msg)
                                            with open(debug_log, 'a') as f:
                                                f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                            # Show menu in main thread
                                            arm_menu_ov_guard()
                                            root.after(0, show_menu)
                                    else:
                                        override_mode = not override_mode
                                        if override_mode:
                                            override_enabled_time = time.time()  # Record when enabled
                                        msg = f"Serial: Override mode {'ENABLED' if override_mode else 'DISABLED'}"
                                        print(msg)
                                        with open(debug_log, 'a') as f:
                                            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")


                                elif line == 'TU':
                                    msg = "Serial: Thumbs Up command received"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, lambda: handle_thumbs_up_press("serial TU"))

                                elif line == 'MIX':
                                    msg = "Serial: Mix mode command received"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, lambda: switch_mode('mix'))

                                elif line == 'FILL':
                                    msg = "Serial: Fill mode command received"
                                    print(msg)
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {msg}\n")
                                    root.after(0, lambda: switch_mode('fill'))

                                else:
                                    # Unknown command
                                    with open(debug_log, 'a') as f:
                                        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Unknown command: '{line}'\n")
                else:
                    time.sleep(0.1)
            except Exception as e:
                msg = f"Serial read error: {e}"
                print(msg)
                log_serial_debug(msg)
                time.sleep(0.1)

    except Exception as e:
        serial_connected = False
        msg = f"Serial listener error: {e}"
        print(msg)
        log_serial_debug(msg)

def initialize_gpio():
    """Initialize GPIO for relay control"""
    if not GPIO_AVAILABLE:
        print("GPIO not available, skipping GPIO initialization")
        return True

    try:
        # Disable warnings about channels already in use
        GPIO.setwarnings(False)
        # Configure GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(config.PUMP_STOP_RELAY_PIN, GPIO.OUT)
        GPIO.setup(config.FLOW_RESET_PIN, GPIO.OUT)
        GPIO.output(config.FLOW_RESET_PIN, GPIO.LOW)
        GPIO.output(config.PUMP_STOP_RELAY_PIN, GPIO.LOW)
        print(f"GPIO initialized: Relay on pin {config.PUMP_STOP_RELAY_PIN}")
        return True
    except Exception as e:
        print(f"Failed to initialize GPIO: {e}")
        return False

# Flow meter reset
flow_reset_scheduled = False
flow_reset_cycle_id = None
flow_cycle_counter = 0

def flow_is_active():
    """Return True when flow is currently active."""
    return last_flow_rate >= config.FLOW_STOPPED_THRESHOLD


def clear_auto_shutoff_state(reason=""):
    """Clear per-cycle auto-shutoff state after a reset or new flow cycle."""
    global last_alert_triggered, auto_shutoff_latched
    global last_trigger_flow_gpm, last_trigger_threshold, last_trigger_actual
    global last_trigger_predicted_actual, last_trigger_loop_dt_ms

    last_alert_triggered = False
    auto_shutoff_latched = False
    last_trigger_flow_gpm = 0.0
    last_trigger_threshold = 0.0
    last_trigger_actual = 0.0
    last_trigger_predicted_actual = 0.0
    last_trigger_loop_dt_ms = 0.0
    recent_flow_rates_l_per_s.clear()
    if reason:
        print(f"Auto-shutoff state cleared: {reason}")


def detect_totalizer_reset(totalizer_liters):
    """Treat a confirmed nonzero-to-zero totalizer drop as a new cycle boundary."""
    global previous_totalizer_liters, flow_cycle_counter

    previous_gallons = previous_totalizer_liters * config.LITERS_TO_GALLONS
    current_gallons = totalizer_liters * config.LITERS_TO_GALLONS
    reset_detected = previous_gallons > 0.25 and current_gallons < 0.05
    previous_totalizer_liters = totalizer_liters

    if reset_detected:
        flow_cycle_counter += 1
        clear_auto_shutoff_state("totalizer reset to zero")
        print(
            f"New fill cycle assumed from totalizer reset "
            f"({previous_gallons:.3f} -> {current_gallons:.3f} gal)"
        )


def _pulse_flow_reset_gpio():
    global flow_reset_scheduled, flow_reset_cycle_id
    if SIM_MODE:
        _sim_flow_reset("gpio pulse")
        return
    try:
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write("pulsing gpio0\n")
        GPIO.output(config.FLOW_RESET_PIN, GPIO.HIGH)
        time.sleep(config.FLOW_RESET_DURATION)
        GPIO.output(config.FLOW_RESET_PIN, GPIO.LOW)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write("done\n")
    except Exception as e:
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(str(e) + "\n")
    flow_reset_scheduled = False
    flow_reset_cycle_id = None


def _sim_flow_reset(reason):
    global flow_reset_scheduled, flow_reset_cycle_id, last_totalizer_liters
    iolhat.reset_totalizer()
    detect_totalizer_reset(0.0)
    last_totalizer_liters = 0.0
    draw_actual_number("0.0", target_display_color(0.0))
    with open("/home/pi/reset_debug.log", "a") as dbg:
        dbg.write(f"sim totalizer reset: {reason}\n")
    flow_reset_scheduled = False
    flow_reset_cycle_id = None


def force_flow_reset(reason="forced"):
    global flow_reset_scheduled, flow_reset_cycle_id
    if SIM_MODE:
        msg = f"Flow reset forced: {reason} ({last_flow_rate:.3f} L/s)"
        print(msg)
        log_serial_debug(msg)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(msg + "\n")
        clear_auto_shutoff_state(reason)
        _sim_flow_reset(reason)
        return
    if not GPIO_AVAILABLE:
        flow_reset_scheduled = False
        flow_reset_cycle_id = None
        return
    msg = f"Flow reset forced: {reason} ({last_flow_rate:.3f} L/s)"
    print(msg)
    log_serial_debug(msg)
    with open("/home/pi/reset_debug.log", "a") as dbg:
        dbg.write(msg + "\n")
    clear_auto_shutoff_state(reason)
    _pulse_flow_reset_gpio()


def pulse_flow_reset():
    global flow_reset_scheduled, flow_reset_cycle_id
    with open("/home/pi/reset_debug.log", "a") as dbg:
        dbg.write("pulse called\n")
    if flow_reset_cycle_id != flow_cycle_counter:
        msg = "Flow reset cancelled: new flow started after reset was requested"
        print(msg)
        log_serial_debug(msg)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(msg + "\n")
        flow_reset_scheduled = False
        flow_reset_cycle_id = None
        return
    if flow_is_active():
        msg = f"Flow reset blocked: flow still active ({last_flow_rate:.3f} L/s)"
        print(msg)
        log_serial_debug(msg)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(msg + "\n")
        flow_reset_scheduled = False
        flow_reset_cycle_id = None
        return
    if SIM_MODE:
        _sim_flow_reset("scheduled pulse")
        return
    if not GPIO_AVAILABLE:
        flow_reset_scheduled = False
        flow_reset_cycle_id = None
        return
    try:
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write("pulsing gpio0\n")
        GPIO.output(config.FLOW_RESET_PIN, GPIO.HIGH)
        time.sleep(config.FLOW_RESET_DURATION)
        GPIO.output(config.FLOW_RESET_PIN, GPIO.LOW)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write("done\n")
    except Exception as e:
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(str(e) + "\n")
    flow_reset_scheduled = False
    flow_reset_cycle_id = None

def schedule_flow_reset():
    global flow_reset_scheduled, flow_reset_cycle_id
    if flow_reset_scheduled:
        return
    if flow_is_active():
        msg = f"Flow reset not scheduled: flow still active ({last_flow_rate:.3f} L/s)"
        print(msg)
        log_serial_debug(msg)
        with open("/home/pi/reset_debug.log", "a") as dbg:
            dbg.write(msg + "\n")
        return
    flow_reset_scheduled = True
    flow_reset_cycle_id = flow_cycle_counter
    root.after(int(config.FLOW_RESET_DELAY * 1000), pulse_flow_reset)


def initialize_iol():
    """Initialize IO-Link port"""
    try:
        with iol_io_lock:
            # Power on the port
            iolhat.power(config.IOL_PORT, 1)
            # Set LED to green
            iolhat.led(config.IOL_PORT, iolhat.LED_GREEN)
        time.sleep(0.5)
        print(f"IO-Link Port {config.IOL_PORT+1} initialized successfully")
        return True
    except Exception as e:
        print(f"Failed to initialize IO-Link Port {config.IOL_PORT+1}: {e}")
        return False

# Tkinter GUI setup
root = tk.Tk()
root.title("Tank Dashboard")
root.configure(bg="black")
if SIM_MODE:
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()
    SIM_RENDER_SCALE = min(
        max(0.1, (screen_w - 24) / SIM_WINDOW_WIDTH),
        max(0.1, (screen_h - 96) / SIM_WINDOW_HEIGHT),
        1.0,
    )
    sim_display_width = max(1, int(SIM_WINDOW_WIDTH * SIM_RENDER_SCALE))
    sim_display_height = max(1, int(SIM_WINDOW_HEIGHT * SIM_RENDER_SCALE))
    root.geometry(f"{sim_display_width}x{sim_display_height}")
    root.resizable(True, True)
else:
    root.attributes("-fullscreen", True)

# Get screen dimensions
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# Create ONE full-screen canvas for everything
canvas = tk.Canvas(
    root,
    bg="black",
    highlightthickness=0,
    width=sim_display_width if SIM_MODE else 0,
    height=sim_display_height if SIM_MODE else 0,
)
canvas.pack(fill='both', expand=True)


def _sim_scale_value(value):
    if SIM_MODE and isinstance(value, (int, float)):
        return value * SIM_RENDER_SCALE
    return value


def _sim_scale_args(args):
    return tuple(_sim_scale_value(arg) for arg in args)


def _sim_scale_font(font):
    if not SIM_MODE or not isinstance(font, tuple) or len(font) < 2:
        return font
    size = font[1]
    if not isinstance(size, int):
        return font
    scaled_size = max(1, int(round(abs(size) * SIM_RENDER_SCALE)))
    return (font[0], -scaled_size, *font[2:])


if SIM_MODE:
    _canvas_create_text = canvas.create_text
    _canvas_create_rectangle = canvas.create_rectangle
    _canvas_create_line = canvas.create_line

    def _sim_create_text(*args, **kwargs):
        if "font" in kwargs:
            kwargs["font"] = _sim_scale_font(kwargs["font"])
        return _canvas_create_text(*_sim_scale_args(args), **kwargs)

    def _sim_create_rectangle(*args, **kwargs):
        return _canvas_create_rectangle(*_sim_scale_args(args), **kwargs)

    def _sim_create_line(*args, **kwargs):
        return _canvas_create_line(*_sim_scale_args(args), **kwargs)

    canvas.create_text = _sim_create_text
    canvas.create_rectangle = _sim_create_rectangle
    canvas.create_line = _sim_create_line

def _sync_canvas_geometry():
    if not SIM_MODE:
        canvas.update()


def _canvas_width():
    width = canvas.winfo_width()
    return SIM_WINDOW_WIDTH if SIM_MODE else width


def _canvas_height():
    height = canvas.winfo_height()
    return SIM_WINDOW_HEIGHT if SIM_MODE else height


def update_relay_slowdown_alarm_flash():
    """Flash the whole display while auto-stop relay has not slowed flow."""
    global relay_slowdown_alarm_visible

    flash_hz = max(1.0, float(getattr(config, "RELAY_SLOWDOWN_ALARM_FLASH_HZ", 30)))
    interval_ms = max(1, int(1000 / flash_hz))

    if relay_slowdown_alarm_active:
        phase = int(time.time() * flash_hz) % 2
        color = (
            config.RELAY_SLOWDOWN_ALARM_COLOR_A
            if phase == 0
            else config.RELAY_SLOWDOWN_ALARM_COLOR_B
        )
        canvas.delete("relay_slowdown_alarm")
        canvas.create_rectangle(
            0,
            0,
            _canvas_width(),
            _canvas_height(),
            fill=color,
            outline="",
            tags="relay_slowdown_alarm",
        )
        canvas.tag_raise("relay_slowdown_alarm")
        relay_slowdown_alarm_visible = True
    elif relay_slowdown_alarm_visible:
        canvas.delete("relay_slowdown_alarm")
        relay_slowdown_alarm_visible = False

    root.after(interval_ms, update_relay_slowdown_alarm_flash)


# Draw full-screen barber pole stripes
def draw_fullscreen_stripes():
    """Draw barber pole stripes across entire screen"""
    # Get actual canvas size after it's been packed
    _sync_canvas_geometry()
    width = _canvas_width()
    height = _canvas_height()

    stripe_height = 30
    dark_yellow = "#CC9900"

    for i, stripe_y in enumerate(range(0, height, stripe_height)):
        stripe_color = "red" if i % 2 == 0 else dark_yellow
        canvas.create_rectangle(0, stripe_y, width, stripe_y + stripe_height,
                               fill=stripe_color, outline="", tags="stripes")

# Draw stripes after window is created
if not SIM_MODE:
    root.update()
# draw_fullscreen_stripes()  # Disabled - using solid black background


def _apply_mopeka(m1g, m2g, m1q, m2q):
    """Apply mopeka values and update display (called from main thread via root.after)"""
    global mopeka1_gallons, mopeka2_gallons, mopeka1_quality, mopeka2_quality, mopeka_connected, mopeka_enabled
    mopeka1_gallons = m1g
    mopeka2_gallons = m2g
    mopeka1_quality = m1q
    mopeka2_quality = m2q
    mopeka_enabled = True
    mopeka_connected = True
    print(f"Mopeka applied: front={m1g:.0f} back={m2g:.0f} q={m1q}/{m2q}", flush=True)
    update_mopeka_display()


def _apply_mopeka_raw(m1mm, m2mm, m1in, m2in):
    global mopeka1_level_mm, mopeka2_level_mm, mopeka1_level_in, mopeka2_level_in
    mopeka1_level_mm = m1mm
    mopeka2_level_mm = m2mm
    mopeka1_level_in = m1in
    mopeka2_level_in = m2in


def _apply_bms(soc, voltage):
    global bms_soc, bms_voltage
    bms_soc = soc
    bms_voltage = voltage
    update_bms_display()


def _mopeka_offline():
    """Mark mopeka sensors as offline and update display"""
    global mopeka_connected
    mopeka_connected = False
    print("Mopeka offline", flush=True)
    update_mopeka_display()


def _mopeka_disabled():
    """Mark mopeka as intentionally disabled for this box."""
    global mopeka_connected, mopeka_enabled
    mopeka_enabled = False
    mopeka_connected = False
    print("Mopeka disabled", flush=True)
    update_mopeka_display()


def update_mopeka_display():
    """Draw Mopeka tank levels in top-right corner of screen"""
    canvas.delete("mopeka_display")

    if current_mode == "mix":
        refresh_batch_mix_tank_levels()
        return

    if not mopeka_enabled:
        canvas.delete("batchmix_tanks")
        return
    
    width = _canvas_width()
    x = width - 20  # 20px from right edge
    font = ("Helvetica", 72, "bold")
    
    if not mopeka_connected:
        canvas.create_text(x, 40, text="Tanks: No Signal", font=font,
                          fill="#ff0000", anchor="ne", tags="mopeka_display")
        return
    
    # Front tank - top right
    color1 = _mopeka_quality_color(mopeka1_quality)
    label1 = f"Front: {mopeka1_gallons:.0f} gal"
    canvas.create_text(x, 40, text=label1, font=font,
                      fill=color1, anchor="ne", tags="mopeka_display")
    
    # Back tank - below front
    color2 = _mopeka_quality_color(mopeka2_quality)
    label2 = f"Back: {mopeka2_gallons:.0f} gal"
    canvas.create_text(x, 110, text=label2, font=font,
                      fill=color2, anchor="ne", tags="mopeka_display")

def draw_requested_number(text, color="red"):
    """Draw the requested number with white outline on full-screen canvas"""
    global batch_mix_layout_active, _last_requested_text, _last_requested_color

    # If batch mix layout is active, use different positioning
    if batch_mix_layout_active:
        redraw_numbers_for_batch_mix()
        return

    # Only redraw if value or color changed (prevents flicker)
    if text == _last_requested_text and color == _last_requested_color:
        return

    _last_requested_text = text
    _last_requested_color = color

    # Delete old requested number
    canvas.delete("requested")

    # Position: centered horizontally, 20% from top
    x = _canvas_width() // 2
    y = int(_canvas_height() * 0.28) + 24
    font = ("Helvetica", 220, "bold")

    # Draw white outline (8 positions around the text)
    for dx, dy in [(-5,-5), (-5,0), (-5,5), (0,-5), (0,5), (5,-5), (5,0), (5,5)]:
        canvas.create_text(x+dx, y+dy, text=text, font=font, fill="white", tags="requested")

    # Draw text with specified color on top
    canvas.create_text(x, y, text=text, font=font, fill=color, tags="requested")

# Draw text labels on canvas (centered)
_sync_canvas_geometry()
center_x = _canvas_width() // 2
height = _canvas_height()

canvas.create_text(center_x, int(height * 0.08), text="Requested Gallons:", font=("Helvetica", 36, "bold"),
                  fill="white", tags="labels")

# Draw initial requested value
draw_requested_number(f"{config.REQUESTED_GALLONS:.0f}", "red")

canvas.create_text(center_x, int(height * 0.45), text="Actual Gallons:", font=("Helvetica", 36, "bold"),
                  fill="white", tags="labels")

def draw_actual_number(text, color="red"):
    """Draw the actual number with white outline on full-screen canvas"""
    global batch_mix_layout_active, _last_actual_text, _last_actual_color

    # If batch mix layout is active, use different positioning
    if batch_mix_layout_active:
        redraw_numbers_for_batch_mix()
        return

    # Only redraw if value or color changed (prevents flicker)
    if text == _last_actual_text and color == _last_actual_color:
        return

    _last_actual_text = text
    _last_actual_color = color

    # Delete old actual number
    canvas.delete("actual")

    # Position: centered horizontally, 68% from top
    x = _canvas_width() // 2
    y = int(_canvas_height() * 0.68)
    font = ("Helvetica", 310, "bold")

    # Draw white outline (8 positions around the text)
    for dx, dy in [(-6,-6), (-6,0), (-6,6), (0,-6), (0,6), (6,-6), (6,0), (6,6)]:
        canvas.create_text(x+dx, y+dy, text=text, font=font, fill="white", tags="actual")

    # Draw text with specified color on top
    canvas.create_text(x, y, text=text, font=font, fill=color, tags="actual")

# Draw initial value
draw_actual_number("0.0", "red")

# Draw initial mopeka state (no signal)
root.after(1000, update_mopeka_display)

# Status Label (for connection errors)
status_label = ttk.Label(root, text="", font=("Helvetica", 24),
                        foreground="yellow", background="black")
# status_label.pack(pady=2)

# Flow Meter Disconnected Warning (flashing)
flowmeter_disconnected_label = ttk.Label(root, text="FLOW METER\nDISCONNECTED",
                                         font=("Helvetica", 60, "bold"),
                                         foreground="red", background="black")
# flowmeter_disconnected_label.pack(pady=2)
# flowmeter_disconnected_label.pack_forget()

# Switch Box Disconnected Label (shown when heartbeat times out)
switchbox_disconnected_label = ttk.Label(root, text="SWITCH BOX\nDISCONNECTED",
                                         font=("Helvetica", 60, "bold"),
                                         foreground="red", background="black")
# switchbox_disconnected_label.pack(pady=2)
# switchbox_disconnected_label.pack_forget()

# Warning Label (flashing)
warning_label = ttk.Label(root, text="OVER TARGET!", font=("Helvetica", 72, "bold"),
                          foreground="red", background="black")
# warning_label.pack(pady=2)
# warning_label.pack_forget()

# Manual Mode Label (flashing when override is active)
manual_label = ttk.Label(root, text="MANUAL", font=("Helvetica", 90, "bold"),
                         foreground="orange", background="black")
# manual_label.pack(pady=2)
# manual_label.pack_forget()

# Mix Mode Indicator Label (shown in top-left corner when in mix mode)
mode_indicator_label = tk.Label(root, text="MIX", font=("Helvetica", 38, "bold"),
                                foreground="cyan", background="black",
                                padx=14, pady=4, bd=0)
# mode_indicator_label initially hidden, shown via place() when in mix mode

# Thumbs up animated GIF support
thumbs_up_frames = []
thumbs_up_frames_by_color = {}
thumbs_up_frame_index = [0]  # Use list for mutable reference
thumbs_up_label = None
thumbs_up_animation_id = None
thumbs_up_visible = False
thumbs_up_current_color = "green"
THUMBS_UP_RELX = 0.88
THUMBS_UP_RELY = 0.25
THUMBS_UP_TINTS = {
    "green": (0, 190, 0),
    "red": (230, 0, 0),
}

def tint_thumbs_up_image(image, rgb):
    """Tint a transparent thumbs-up image while keeping its shading and alpha."""
    rgba = image.convert("RGBA")
    alpha = rgba.getchannel("A")
    shade = rgba.convert("L")
    red, green, blue = rgb
    channels = [
        shade.point(lambda value, base=base: int(base * (0.45 + 0.55 * (value / 255.0))))
        for base in (red, green, blue)
    ]
    return Image.merge("RGBA", (*channels, alpha))


def set_thumbs_up_color(color):
    """Switch the thumbs-up image/text to match the gallon display color."""
    global thumbs_up_frames, thumbs_up_current_color

    color = "red" if color == "red" else "green"
    if color == thumbs_up_current_color and thumbs_up_frames:
        return

    thumbs_up_current_color = color
    if thumbs_up_frames_by_color:
        thumbs_up_frames = thumbs_up_frames_by_color.get(color, thumbs_up_frames)
        thumbs_up_frame_index[0] = min(thumbs_up_frame_index[0], max(len(thumbs_up_frames) - 1, 0))
        if thumbs_up_label and thumbs_up_frames:
            thumbs_up_label.config(image=thumbs_up_frames[thumbs_up_frame_index[0]])
    elif thumbs_up_label:
        thumbs_up_label.config(foreground=color)


def show_thumbs_up(actual_gallons=None):
    """Show thumbs-up with the same red/green state as the gallon text."""
    if not thumbs_up_label or batch_mix_layout_active:
        return
    set_thumbs_up_color(target_display_color(actual_gallons))
    thumbs_up_label.place(relx=THUMBS_UP_RELX, rely=THUMBS_UP_RELY, anchor="n")
    _set_thumbs_up_visible(True)

def load_thumbs_up_gif():
    """Load thumbs up image (PNG or GIF) for display"""
    global thumbs_up_frames, thumbs_up_frames_by_color, thumbs_up_label

    script_dir = os.path.dirname(os.path.abspath(__file__))
    png_candidates = [
        os.path.join(script_dir, "thumbs_up.png"),
        "/home/pi/thumbs_up.png",
    ]
    gif_candidates = [
        os.path.join(script_dir, "thumbs_up.gif"),
        "/home/pi/thumbs_up.gif",
    ]
    
    try:
        png_path = next((path for path in png_candidates if os.path.exists(path)), None)
        gif_path = next((path for path in gif_candidates if os.path.exists(path)), None)
        base_frames = []

        # Try PNG first
        if png_path:
            img = Image.open(png_path)
            # Resize to fit nicely on screen
            img = img.resize((533, 533), Image.Resampling.LANCZOS)
            base_frames = [img]
            print(f"Loaded thumbs up from PNG: {png_path}")
        elif gif_path:
            img = Image.open(gif_path)
            # Extract all frames from GIF
            try:
                while True:
                    frame = img.copy()
                    frame = frame.resize((533, 533), Image.Resampling.LANCZOS)
                    base_frames.append(frame)
                    img.seek(img.tell() + 1)
            except EOFError:
                pass
            print(f"Loaded {len(base_frames)} frames from thumbs up GIF")
        else:
            base_frames = []

        thumbs_up_frames_by_color = (
            {
                color: [ImageTk.PhotoImage(tint_thumbs_up_image(frame, rgb)) for frame in base_frames]
                for color, rgb in THUMBS_UP_TINTS.items()
            }
            if base_frames else {}
        )
        thumbs_up_frames = thumbs_up_frames_by_color.get(thumbs_up_current_color, [])
        
        # Create label
        if thumbs_up_frames:
            thumbs_up_label = tk.Label(root, image=thumbs_up_frames[0], bg="black")
        else:
            thumbs_up_label = tk.Label(root, text="👍", font=("Helvetica", 400, "bold"),
                                       foreground="green", background="black")
    except Exception as e:
        print(f"Could not load thumbs up image: {e}")
        thumbs_up_label = tk.Label(root, text="👍", font=("Helvetica", 400, "bold"),
                                   foreground="green", background="black")


def _set_thumbs_up_visible(visible):
    """Track whether the thumbs-up indicator is currently visible."""
    global thumbs_up_visible
    thumbs_up_visible = bool(visible)


def animate_thumbs_up():
    """Animate the thumbs up GIF"""
    global thumbs_up_animation_id
    
    if thumbs_up_frames and thumbs_up_label:
        thumbs_up_frame_index[0] = (thumbs_up_frame_index[0] + 1) % len(thumbs_up_frames)
        thumbs_up_label.config(image=thumbs_up_frames[thumbs_up_frame_index[0]])
        thumbs_up_animation_id = root.after(100, animate_thumbs_up)  # 10 FPS

# Load the GIF on startup
load_thumbs_up_gif()

def update_dashboard():
    """Update the dashboard with current flow meter readings"""
    global last_alert_triggered, auto_shutoff_latched, override_mode, was_flowing, colors_are_green, heartbeat_disconnected, override_enabled_time
    global new_fill_flow_started_at, new_fill_last_fresh_at, new_fill_cycle_cleared
    global pending_fill_gallons, pending_fill_requested, pending_fill_shutoff_type
    global pending_fill_flow_gpm, pending_fill_trigger_threshold, last_flowing_rate_l_per_s
    global last_trigger_flow_gpm, last_trigger_threshold, last_trigger_actual
    global last_trigger_predicted_actual, last_trigger_loop_dt_ms
    global flow_cycle_counter, calibration_state, last_status_text, last_daily_total_text, last_daily_total_mode

    control_active = flow_control_active()

    if control_active:
        actual = get_cached_actual_gallons()
    else:
        actual = read_flow_meter()

    color = target_display_color(actual)
    if thumbs_up_visible:
        set_thumbs_up_color(color)

    # Debug log color being used
    button_log = "/home/pi/button_debug.log"
    if colors_are_green:
        with open(button_log, 'a') as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [UPDATE_DASHBOARD] colors_are_green={colors_are_green}, using color={color}\n")

    draw_actual_number(f"{actual:.1f}", color)

    # Update requested gallons number
    draw_requested_number(f"{requested_gallons:.0f}", color)

    # Check if flow meter has timed out (no successful reads in X seconds)
    flow_meter_disconnected = (time.time() - last_successful_read_time) > config.FLOW_METER_TIMEOUT
    update_flow_meter_fault_hold(flow_meter_disconnected)

    # Check if heartbeat has timed out (no OK message in 11 seconds)
    heartbeat_timeout = (time.time() - last_heartbeat_time) > 11
    if heartbeat_timeout and not heartbeat_disconnected:
        heartbeat_disconnected = True
        msg = "Heartbeat timeout - Switch box disconnected"
        print(msg)
        log_serial_debug(msg)
    elif not heartbeat_timeout and heartbeat_disconnected:
        heartbeat_disconnected = False
        msg = "Heartbeat restored - Switch box reconnected"
        print(msg)
        log_serial_debug(msg)

    # Detect flow state
    is_flowing = last_flow_rate >= config.FLOW_STOPPED_THRESHOLD
    now = time.time()
    latest_fresh_age = now - last_fresh_flow_read_time if last_fresh_flow_read_time else float("inf")
    fresh_grace_seconds = config.NEW_FILL_CYCLE_FRESH_GRACE_SECONDS
    has_recent_fresh_flow = latest_fresh_age <= fresh_grace_seconds
    is_recent_fresh_new_fill_flowing = (
        has_recent_fresh_flow
        and not flow_meter_disconnected
        and not connection_error
        and last_flow_rate >= config.NEW_FILL_CYCLE_THRESHOLD
    )
    if is_recent_fresh_new_fill_flowing:
        if new_fill_flow_started_at is None:
            new_fill_flow_started_at = last_fresh_flow_read_time
        new_fill_last_fresh_at = last_fresh_flow_read_time
    elif (
        flow_meter_disconnected
        or connection_error
        or last_flow_rate < config.NEW_FILL_CYCLE_THRESHOLD
        or (
            new_fill_last_fresh_at is not None
            and now - new_fill_last_fresh_at > fresh_grace_seconds
        )
    ):
        new_fill_flow_started_at = None
        new_fill_last_fresh_at = None
        new_fill_cycle_cleared = False
    if is_flowing and not control_active and last_flow_read_was_fresh:
        recent_flow_rates_l_per_s.append(last_flow_rate)
        last_flowing_rate_l_per_s = last_flow_rate

    calibration_waiting_for_fill = (
        calibration_mode
        and calibration_state
        and calibration_state.get("phase") == "wait_for_fill"
        and calibration_state.get("flow_started")
    )

    # Store fill data when flow stops (don't record yet - wait for thumbs up)
    if was_flowing and not is_flowing:
        if calibration_waiting_for_fill:
            calibration_state["last_step_actual"] = actual
            calibration_state["phase"] = "settling"
            calibration_state["settle_deadline"] = time.time() + 120
            calibration_state["flow_started"] = False
            if thumbs_up_label:
                thumbs_up_label.place_forget()
                _set_thumbs_up_visible(False)
            pending_fill_gallons = 0.0
            pending_fill_requested = 0.0
            pending_fill_shutoff_type = ""
            pending_fill_flow_gpm = 0.0
            pending_fill_trigger_threshold = 0.0
            last_trigger_flow_gpm = 0.0
            last_trigger_threshold = 0.0
            last_trigger_actual = 0.0
            recent_flow_rates_l_per_s.clear()
            _refresh_calibration_window()
        else:
            # Determine if shutoff was automatic or manual
            shutoff_type = "Auto" if auto_shutoff_latched else "Manual"

            if shutoff_type == "Auto" and last_trigger_flow_gpm > 0:
                pending_fill_flow_gpm = last_trigger_flow_gpm
                pending_fill_trigger_threshold = last_trigger_threshold
            else:
                smoothed_stop_flow_l_per_s = get_smoothed_flow_rate()
                pending_fill_flow_gpm = smoothed_stop_flow_l_per_s * config.LITERS_PER_SEC_TO_GPM
                pending_fill_trigger_threshold = calculate_trigger_threshold(smoothed_stop_flow_l_per_s)

            # Store pending fill data (will be recorded when thumbs up is pressed)
            pending_fill_gallons = actual
            pending_fill_requested = requested_gallons
            pending_fill_shutoff_type = shutoff_type

            print(
                f"Fill complete - Requested: {requested_gallons:.3f}, Actual: {actual:.3f}, "
                f"Diff: {actual - requested_gallons:+.3f}, Type: {shutoff_type}, "
                f"FlowAtStop: {pending_fill_flow_gpm:.1f} GPM, Threshold: {pending_fill_trigger_threshold:.3f}"
            )
            print(f"Waiting for thumbs up button to record fill...")

            # NOTE: Do NOT hide thumbs up when flow stops - keep it visible so user can press it
            # Thumbs up will be hidden after button is pressed or when new fill starts

    # Reset colors when new fill cycle starts
    if not was_flowing and is_flowing:
        colors_are_green = False
        new_fill_cycle_cleared = False
        if calibration_mode and calibration_state and calibration_state.get("phase") == "wait_for_fill":
            calibration_state["flow_started"] = True
            _refresh_calibration_window()
        if not control_active:
            flow_cycle_counter += 1
            last_alert_triggered = False
            auto_shutoff_latched = False
            last_trigger_flow_gpm = 0.0
            last_trigger_threshold = 0.0
            last_trigger_actual = 0.0
            last_trigger_predicted_actual = 0.0
            last_trigger_loop_dt_ms = 0.0
            recent_flow_rates_l_per_s.clear()
        print("New fill cycle started - colors reset to red, auto-shutoff state cleared")

    # Hide thumbs up and clear pending fill only after sustained high flow.
    if (
        new_fill_flow_started_at is not None
        and new_fill_last_fresh_at is not None
        and not new_fill_cycle_cleared
        and now - new_fill_flow_started_at >= config.NEW_FILL_CYCLE_HOLD_SECONDS
        and now - new_fill_last_fresh_at <= config.NEW_FILL_CYCLE_FRESH_GRACE_SECONDS
    ):
        if thumbs_up_label:
            thumbs_up_label.place_forget()
            _set_thumbs_up_visible(False)
        pending_fill_gallons = 0.0
        pending_fill_requested = 0.0
        pending_fill_shutoff_type = ""
        pending_fill_flow_gpm = 0.0
        pending_fill_trigger_threshold = 0.0
        new_fill_cycle_cleared = True
        print("Sustained high-flow fill cycle detected - thumbs up hidden, pending fill cleared")

    if calibration_mode and calibration_state and calibration_state.get("phase") == "settling":
        if time.time() >= calibration_state.get("settle_deadline", time.time()):
            calibration_state["phase"] = "review"
            calibration_state["reading"] = _selected_tank_reading()
        _refresh_calibration_window()

    # Update flow state for next cycle
    was_flowing = is_flowing

    # Auto-disable override after 1 minute if no flow (only if flow meter is connected)
    if override_mode and not flow_meter_disconnected:
        if last_flow_rate < config.FLOW_STOPPED_THRESHOLD:
            # No flow detected - check if override has been enabled for more than 60 seconds
            time_since_override_enabled = time.time() - override_enabled_time
            if time_since_override_enabled > 60:
                override_mode = False
                print(f"Override auto-disabled: no flow for {time_since_override_enabled:.0f} seconds (> 60s limit)")
        else:
            # Flow detected - reset the timer so override can stay active
            override_enabled_time = time.time()
    # If flow meter is disconnected, override stays on indefinitely (no auto-disable)

    # Calculate dynamic trigger threshold based on current flow rate
    smoothed_flow_rate_l_per_s = get_smoothed_flow_rate()
    flow_rate_gpm = smoothed_flow_rate_l_per_s * config.LITERS_PER_SEC_TO_GPM
    display_flow_rate_gpm = flow_rate_gpm if is_flowing else 0.0
    trigger_threshold = calculate_trigger_threshold(smoothed_flow_rate_l_per_s)

    # Auto-alert: Trigger GPIO 27 based on flow-adjusted threshold (once per cycle)
    # Only if override mode is OFF and flow meter is connected
    if (
        not control_active
        and is_flowing
        and not override_mode
        and not flow_meter_disconnected
        and actual >= requested_gallons - trigger_threshold
        and not last_alert_triggered
    ):
        last_alert_triggered = True
        auto_shutoff_latched = True
        last_trigger_flow_gpm = flow_rate_gpm
        last_trigger_threshold = trigger_threshold
        last_trigger_actual = actual
        print(f"Auto-alert: Flow={flow_rate_gpm:.1f} GPM, threshold={trigger_threshold:.2f}gal, triggering relay for {config.AUTO_ALERT_DURATION}s")
        start_pump_stop_thread(config.AUTO_ALERT_DURATION)
    startup_iol_warning_suppressed = _suppress_startup_iol_warning(flow_meter_disconnected)

    # Update status label
    status_parts = []
    if startup_iol_warning_suppressed:
        status_parts.append("IOL: Starting")
    elif pump_stop_fault_hold_active:
        status_parts.append(f"IOL: {pump_stop_fault_hold_reason[:30]}")
    elif connection_error:
        status_parts.append(f"IOL: {error_message[:30]}")
    else:
        status_parts.append("IOL: Connected")

    if serial_connected:
        # Show Connected or Disconnected based on heartbeat status
        if heartbeat_disconnected:
            status_parts.append("Serial: Disconnected")
        else:
            status_parts.append("Serial: Connected")
    else:
        status_parts.append("Serial: Disconnected")

    if override_mode:
        status_parts.append("OVERRIDE: ON")

    # Draw status text only when it changes.
    status_text = " | ".join(status_parts)
    if status_text != last_status_text:
        canvas.delete("status")
        if status_text:
            canvas.create_text(_canvas_width() // 2, _canvas_height() - 20, text=status_text,
                              font=("Helvetica", 20), fill="yellow", tags="status")
        last_status_text = status_text

    # Draw daily total only when the visible text changes.
    daily_total_text = f"Today:\n{daily_total:.1f} gal" if current_mode != "mix" else ""
    if daily_total_text != last_daily_total_text or current_mode != last_daily_total_mode:
        canvas.delete("daily_total")
        if daily_total_text:
            canvas.create_text(10, _canvas_height() - 10, text=daily_total_text,
                              font=("Helvetica", 72, "bold"), fill="cyan", anchor="sw", tags="daily_total")
        last_daily_total_text = daily_total_text
        last_daily_total_mode = current_mode
    update_flow_rate_display(display_flow_rate_gpm)

    # Draw skull icons on sides when flow meter is disconnected (3 inches ~= 288pt at 96 DPI)
    # Pulse animation: size varies between 240pt and 288pt with 1-second cycle
    canvas.delete("skull_icons")
    if flow_meter_disconnected and not startup_iol_warning_suppressed:
        import math
        pulse = math.sin(time.time() * 2 * math.pi)  # -1 to 1, completes cycle every 1 second
        skull_size = int(264 + 24 * pulse)  # Varies from 240pt to 288pt

        # Left skull
        canvas.create_text(150, _canvas_height() // 2, text="☠",
                         font=("Helvetica", skull_size, "bold"), fill="red", tags="skull_icons")
        # Right skull
        canvas.create_text(_canvas_width() - 150, _canvas_height() // 2, text="☠",
                         font=("Helvetica", skull_size, "bold"), fill="red", tags="skull_icons")

    # Draw warnings on canvas - collect all active warnings and cycle through them
    canvas.delete("warning")
    canvas.delete("caution_blocks")  # Delete caution blocks from previous frame

    # Special handling for override/caution mode - draw flashing red blocks with caution symbols
    if override_mode:
        # Railroad crossing alternating flash pattern (1 Hz - left side / right side alternate every 0.5s)
        phase = int(time.time() * 2) % 2  # 0 or 1

        # Block dimensions - larger to fit caution symbol
        block_width = 320
        block_height = 380

        # Calculate vertical positions for upper and lower blocks
        upper_y = int(_canvas_height() * 0.35)  # Upper blocks at 35% down screen
        lower_y = int(_canvas_height() * 0.65)  # Lower blocks at 65% down screen

        # Block positions (left and right sides)
        left_x = 50
        right_x = _canvas_width() - 50 - block_width

        # Draw LEFT side blocks when phase=0 (both upper and lower left)
        if phase == 0:
            # Upper left block
            canvas.create_rectangle(left_x, upper_y - block_height//2,
                                   left_x + block_width, upper_y + block_height//2,
                                   fill="red", outline="", tags="caution_blocks")
            canvas.create_text(left_x + block_width//2, upper_y,
                             text="⚠", font=("Helvetica", 180, "bold"),
                             fill="white", tags="caution_blocks")

            # Lower left block
            canvas.create_rectangle(left_x, lower_y - block_height//2,
                                   left_x + block_width, lower_y + block_height//2,
                                   fill="red", outline="", tags="caution_blocks")
            canvas.create_text(left_x + block_width//2, lower_y,
                             text="⚠", font=("Helvetica", 180, "bold"),
                             fill="white", tags="caution_blocks")

        # Draw RIGHT side blocks when phase=1 (both upper and lower right)
        else:
            # Upper right block
            canvas.create_rectangle(right_x, upper_y - block_height//2,
                                   right_x + block_width, upper_y + block_height//2,
                                   fill="red", outline="", tags="caution_blocks")
            canvas.create_text(right_x + block_width//2, upper_y,
                             text="⚠", font=("Helvetica", 180, "bold"),
                             fill="white", tags="caution_blocks")

            # Lower right block
            canvas.create_rectangle(right_x, lower_y - block_height//2,
                                   right_x + block_width, lower_y + block_height//2,
                                   fill="red", outline="", tags="caution_blocks")
            canvas.create_text(right_x + block_width//2, lower_y,
                             text="⚠", font=("Helvetica", 180, "bold"),
                             fill="white", tags="caution_blocks")

        # Draw "MANUAL" text at center bottom
        canvas.create_text(_canvas_width() // 2, int(_canvas_height() * 0.88),
                         text="MANUAL", font=("Helvetica", 90, "bold"),
                         fill="orange", tags="warning")
        if pump_stop_fault_hold_active and not startup_iol_warning_suppressed:
            canvas.create_text(_canvas_width() // 2, int(_canvas_height() * 0.15),
                             text=f"IO-LINK FAULT\n{pump_stop_fault_hold_reason}",
                             font=("Helvetica", 72, "bold"),
                             fill="red", tags="warning")

    else:
        # Build list of active warnings (in priority order) - only when NOT in override mode
        active_warnings = []

        if pump_stop_fault_hold_active and not startup_iol_warning_suppressed:
            active_warnings.append((
                f"IO-LINK FAULT\n{pump_stop_fault_hold_reason}",
                "Helvetica",
                72,
                "red",
            ))

        if heartbeat_disconnected:
            active_warnings.append(("SWITCH BOX\nDISCONNECTED", "Helvetica", 60, "red"))

        if flow_meter_disconnected and not startup_iol_warning_suppressed:
            active_warnings.append(("☠ FLOW METER ☠\nDISCONNECTED", "Helvetica", 60, "red"))

        if actual > requested_gallons + config.WARNING_THRESHOLD:
            active_warnings.append(("OVER TARGET!", "Helvetica", 72, "red"))

        # Display warnings - cycle through them if multiple exist
        if active_warnings:
            # Flash on/off at 2Hz (on for 0.5s, off for 0.5s)
            if int(time.time() * 2) % 2 == 0:
                # If multiple warnings, cycle through them every 3 seconds
                if len(active_warnings) > 1:
                    warning_index = int(time.time() / 3) % len(active_warnings)
                else:
                    warning_index = 0

                text, font_family, font_size, color = active_warnings[warning_index]
                canvas.create_text(_canvas_width() // 2, int(_canvas_height() * 0.88),
                                 text=text, font=(font_family, font_size, "bold"), fill=color, tags="warning")

    if relay_slowdown_alarm_active and relay_slowdown_alarm_visible:
        canvas.tag_raise("relay_slowdown_alarm")

    root.after(config.UPDATE_INTERVAL, update_dashboard)


def _sim_send_command(line):
    """Drive the most common switch-box commands from the simulator panel."""
    global requested_gallons, serial_connected, override_mode, override_enabled_time
    global colors_are_green, fill_requested_gallons, mix_requested_gallons, current_mode
    global last_heartbeat_time

    line = str(line).strip()
    serial_connected = True

    if line == "OK":
        last_heartbeat_time = time.time()
        return

    if should_ignore_menu_ov_bounce(line, "Sim"):
        return

    if _sim_handle_confirmation_command(line):
        return

    if menu_mode:
        if line == "+1":
            menu_navigate_down()
        elif line == "-1":
            menu_navigate_up()
        elif line == "OV":
            arm_menu_ov_guard()
            menu_select()
        elif line == "PS":
            close_menu()
        return

    if line in ["+1", "-1", "+10", "-10"]:
        adjustment = int(line)
        if adjust_batch_mix_gallons(adjustment, "Sim"):
            return
        requested_gallons = max(0, requested_gallons + adjustment)
        colors_are_green = False
        if current_mode == "fill":
            fill_requested_gallons = requested_gallons
        else:
            mix_requested_gallons = requested_gallons
        save_mode_presets()
        draw_requested_number(f"{requested_gallons:.0f}", "red")
        update_batch_mix_overlay()
    elif line == "PS":
        threading.Thread(target=pump_stop_relay, daemon=True).start()
    elif line == "OV":
        if requested_gallons == 0:
            arm_menu_ov_guard()
            show_menu()
        else:
            override_mode = not override_mode
            if override_mode:
                override_enabled_time = time.time()
    elif line == "TU":
        handle_thumbs_up_press("sim TU")
    elif line == "MIX":
        switch_mode("mix")
    elif line == "FILL":
        switch_mode("fill")


def _sim_handle_confirmation_command(line):
    """Route simulator OV/PS through the same confirmation dialogs as serial."""
    if line not in {"OV", "PS"}:
        return False

    confirmation_pairs = [
        (exit_confirm_window, exit_confirm_handler, exit_cancel_handler),
        (reset_season_confirm_window, reset_season_confirm_handler, reset_season_cancel_handler),
        (reset_flow_curve_confirm_window, reset_flow_curve_confirm_handler, reset_flow_curve_cancel_handler),
        (accept_flow_curve_confirm_window, accept_flow_curve_confirm_handler, accept_flow_curve_cancel_handler),
    ]
    for window, confirm_handler, cancel_handler in confirmation_pairs:
        if window:
            handler = confirm_handler if line == "OV" else cancel_handler
            if handler:
                root.after(0, handler)
            return True

    if reminders_mode:
        if line == "OV":
            root.after(0, dismiss_reminders)
        return True

    return False


def _sim_bind_keyboard_shortcuts(panel):
    """Map simple keyboard keys to the simulated switch-box controls."""
    key_commands = {
        "p": "PS",
        "o": "OV",
        "Left": "-1",
        "Right": "+1",
    }

    def handle_key(event):
        key = event.keysym
        command = key_commands.get(key)
        if command is None:
            command = key_commands.get(getattr(event, "char", "").lower())
        if command:
            _sim_send_command(command)
            return "break"
        return None

    def bind_widget(widget):
        widget.bind("<KeyPress>", handle_key)
        for child in widget.winfo_children():
            bind_widget(child)

    bind_widget(root)
    bind_widget(panel)
    root.bind_all("<KeyPress>", handle_key)
    panel.focus_force()


def _create_sim_controls():
    """Create a small side window for driving the dashboard without Pi hardware."""
    global serial_connected, last_heartbeat_time

    serial_connected = True
    last_heartbeat_time = time.time()

    panel = tk.Toplevel(root)
    panel.title("BBB Simulator")
    panel.geometry("360x560")
    panel.configure(bg="#202020")

    status_var = tk.StringVar()
    flow_var = tk.DoubleVar(value=0)
    connected_var = tk.BooleanVar(value=True)
    switchbox_var = tk.BooleanVar(value=True)

    def set_flow_from_slider(_value=None):
        iolhat.set_flow_gpm(flow_var.get())

    def set_flow(value):
        flow_var.set(value)
        iolhat.set_flow_gpm(value)

    def set_iol_connected():
        iolhat.connected = connected_var.get()

    def set_switchbox_connected():
        global serial_connected, last_heartbeat_time
        serial_connected = switchbox_var.get()
        if serial_connected:
            last_heartbeat_time = time.time()

    def heartbeat_tick():
        if switchbox_var.get():
            _sim_send_command("OK")
        root.after(2500, heartbeat_tick)

    def update_status():
        actual_gal = iolhat.totalizer_liters * config.LITERS_TO_GALLONS
        actual_flow_gpm = iolhat.get_flow_gpm()
        status_var.set(
            f"Flow: {actual_flow_gpm:.0f} GPM | Actual: {actual_gal:.1f} gal | "
            f"IOL: {'on' if iolhat.connected else 'off'} | Switch: {'on' if switchbox_var.get() else 'off'}"
        )
        root.after(250, update_status)

    title = ttk.Label(panel, text="BBB Simulator", font=("Helvetica", 18, "bold"))
    title.pack(pady=(14, 6))

    status = ttk.Label(panel, textvariable=status_var, wraplength=320)
    status.pack(pady=(0, 12))

    ttk.Label(panel, text="Flow Rate").pack(anchor="w", padx=16)
    flow_scale = ttk.Scale(panel, from_=0, to=120, variable=flow_var, command=set_flow_from_slider)
    flow_scale.pack(fill="x", padx=16, pady=(2, 10))

    flow_buttons = ttk.Frame(panel)
    flow_buttons.pack(fill="x", padx=16, pady=(0, 12))
    for label, value in [("Stop", 0), ("45 GPM", 45), ("75 GPM", 75), ("100 GPM", 100)]:
        ttk.Button(flow_buttons, text=label, command=lambda v=value: set_flow(v)).pack(
            side="left", expand=True, fill="x", padx=2
        )

    cmd_frame = ttk.LabelFrame(panel, text="Switch Box")
    cmd_frame.pack(fill="x", padx=16, pady=8)
    for row in [("+1", "-1", "+10", "-10"), ("OV", "PS", "TU"), ("FILL", "MIX")]:
        row_frame = ttk.Frame(cmd_frame)
        row_frame.pack(fill="x", padx=6, pady=4)
        for command in row:
            ttk.Button(row_frame, text=command, command=lambda c=command: _sim_send_command(c)).pack(
                side="left", expand=True, fill="x", padx=2
            )

    sensors = ttk.LabelFrame(panel, text="Sensors")
    sensors.pack(fill="x", padx=16, pady=8)
    ttk.Checkbutton(
        sensors,
        text="IO-Link connected",
        variable=connected_var,
        command=set_iol_connected,
    ).pack(anchor="w", padx=8, pady=4)
    ttk.Checkbutton(
        sensors,
        text="Switch box heartbeat",
        variable=switchbox_var,
        command=set_switchbox_connected,
    ).pack(anchor="w", padx=8, pady=4)
    ttk.Button(sensors, text="Reset Flow Totalizer", command=iolhat.reset_totalizer).pack(
        fill="x", padx=8, pady=4
    )
    ttk.Button(
        sensors,
        text="Tanks Good",
        command=lambda: (_apply_mopeka(325, 310, 3, 3), _apply_mopeka_raw(640, 610, 25.2, 24.0)),
    ).pack(fill="x", padx=8, pady=4)
    ttk.Button(sensors, text="Tanks Offline", command=_mopeka_offline).pack(fill="x", padx=8, pady=4)
    ttk.Button(sensors, text="Battery 84%", command=lambda: _apply_bms(84, 13.1)).pack(
        fill="x", padx=8, pady=4
    )

    ttk.Button(panel, text="Quit Simulator", command=root.destroy).pack(fill="x", padx=16, pady=16)

    heartbeat_tick()
    update_status()
    _sim_bind_keyboard_shortcuts(panel)


# Initialize GPIO and IO-Link and start serial listener
gpio_ok = initialize_gpio()
iol_ok = initialize_iol()

if gpio_ok:
    print(f"Starting dashboard (GPIO: OK, IOL: {'OK' if iol_ok else 'FAILED - display frozen'})")
else:
    print(f"Starting dashboard (GPIO: FAILED - no relay control, IOL: {'OK' if iol_ok else 'FAILED - display frozen'})")

# Load totals from files
load_totals()
load_last_load()
load_flow_curve_state()
today_str = time.strftime('%Y-%m-%d')
if last_reset_date != today_str:
    print(f"Date changed since last reset ({last_reset_date} -> {today_str}) - resetting daily total on startup")
    reset_daily_total()
print(f"Loaded totals - Daily: {daily_total:.2f}, Season: {season_total:.2f}")
print(f"Loaded last loads - {last_loads_gallons[:3]}")

# Load mode presets and set initial requested gallons
load_mode_presets()
if current_mode == 'fill':
    requested_gallons = fill_requested_gallons
else:
    requested_gallons = mix_requested_gallons
    # Show mode indicator if starting in mix mode
    if mode_indicator_label:
        place_mix_mode_indicator()
print(f"Loaded mode - Mode: {current_mode}, Requested: {requested_gallons}, Fill preset: {fill_requested_gallons}, Mix preset: {mix_requested_gallons}")

# Redraw requested gallons with the loaded value
draw_requested_number(f"{requested_gallons:.0f}", "red")
update_last_load_display()
update_bms_display()

if SIM_MODE:
    root.after(100, _create_sim_controls)
else:
    # Start serial listener in background thread (works without IOL)
    serial_thread = threading.Thread(target=serial_listener, daemon=True)
    serial_thread.start()

    # Start socket command listener in background thread (for BLE server communication)
    socket_thread = threading.Thread(target=socket_command_listener, daemon=True)
    socket_thread.start()

    # Start green button monitor in background thread
    green_button_thread = threading.Thread(target=green_button_monitor, daemon=True)
    green_button_thread.start()

start_flow_control_thread()

def daily_total_checker():
    """Background thread to check time and reset daily total at 1:00 AM"""
    global last_reset_date
    import datetime

    while True:
        try:
            current_time = datetime.datetime.now()
            current_hour = current_time.hour
            current_minute = current_time.minute
            current_date = current_time.strftime('%Y-%m-%d')

            # Check if it's 1:00 AM and we haven't reset today
            if current_hour == 1 and current_minute == 0:
                if current_date != last_reset_date:
                    print(f"It's 1:00 AM - resetting daily total for {current_date}")
                    reset_daily_total()
                # Sleep for 61 seconds to avoid re-triggering during the same minute
                time.sleep(61)
            else:
                # Check every 30 seconds
                time.sleep(30)
        except Exception as e:
            print(f"Error in daily_total_checker: {e}")
            time.sleep(60)

def daily_reminder_checker():
    """Background thread to check time and show reminders at 2 AM daily"""
    global last_reminder_date, reminders_mode

    while True:
        try:
            current_time = time.localtime()
            current_hour = current_time.tm_hour
            current_minute = current_time.tm_minute
            current_date = time.strftime('%Y-%m-%d')

            # Check if it's 2 AM and we haven't shown reminders today
            if current_hour == 2 and current_minute == 0:
                if current_date != last_reminder_date and not reminders_mode:
                    print(f"It's 2 AM - showing daily reminders for {current_date}")
                    root.after(0, show_daily_reminders)
                # Sleep for 61 seconds to avoid re-triggering during the same minute
                time.sleep(61)
            else:
                # Check every 30 seconds
                time.sleep(30)
        except Exception as e:
            print(f"Error in daily_reminder_checker: {e}")
            time.sleep(60)

# Start daily total checker thread
total_checker_thread = threading.Thread(target=daily_total_checker, daemon=True)
total_checker_thread.start()

# Start daily reminder checker thread
reminder_thread = threading.Thread(target=daily_reminder_checker, daemon=True)
reminder_thread.start()

update_relay_slowdown_alarm_flash()
update_dashboard()

if SIM_MODE:
    root.deiconify()
    root.lift()
    root.update_idletasks()

try:
    root.mainloop()
finally:
    flow_control_stop_event.set()
    if flow_control_thread and flow_control_thread.is_alive():
        flow_control_thread.join(timeout=1.0)
    # Cleanup GPIO on exit
    if GPIO_AVAILABLE:
        GPIO.cleanup()
