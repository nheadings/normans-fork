#!/usr/bin/env python3
"""
Rotorsync BLE GATT Server using Bumble (bypasses BlueZ)
With live sensor reading via BlueZ (separate adapter)
Sends commands to dashboard via localhost socket
Auto-recovery for sensor connections
Exposes requested/actual gallons from dashboard

"""
import asyncio
import base64
import csv
import json
import logging
import os
from collections import deque
from pathlib import Path
import subprocess
import socket
import time

logging.basicConfig(level=logging.INFO)

from bumble import hci
from bumble.device import Device, Peer
from bumble.host import Host
from bumble.transport.hci_socket import open_hci_socket_transport
from bumble.gatt import Service, Characteristic, CharacteristicValue
from bumble.core import UUID, AdvertisingData
# Mopeka gallon conversion
from src.mopeka_converter import mm_to_gallons, init as mopeka_init, reload as mopeka_reload
from src.bluetooth_adapter_selection import list_bluetooth_adapters, select_adapters
from src.batchmix_payload import batchmix_validation_error
from src.dashboard_polling import (
    HISTORY_POLL_INTERVAL,
    STATUS_POLL_INTERVAL_IDLE,
    dashboard_poll_interval,
)

# Configuration - Use MAC addresses to find adapters dynamically
GATT_ADAPTER_MAC = 'E8:EA:6A:BD:E7:4F'  # USB adapter used for RotorSync GATT server
SENSOR_ADAPTER_MAC = 'BC:FC:E7:2D:86:7B'  # USB adapter reserved for BMS/Mopeka sensor scanning

# Socket connection to dashboard
DASHBOARD_HOST = '127.0.0.1'
DASHBOARD_PORT = 9999

BMS_MAC = 'A5:C2:37:31:77:C0'
BMS_NAME = 'TR2-BMS'
BMS_NOTIFY_UUID = UUID('0000ff01-0000-1000-8000-00805f9b34fb')
BMS_WRITE_UUID = UUID('0000ff02-0000-1000-8000-00805f9b34fb')
MOPEKA1_MAC_SUFFIX = ''  # Set by trailer selection (defa) or restored from mopeka_config.json
MOPEKA2_MAC_SUFFIX = ''
BMS_ENABLED = True
# Timing configuration
SCAN_TIMEOUT = 5
BMS_TIMEOUT = 8
SCAN_INTERVAL = 15
BMS_READ_INTERVAL = 20  # 20 * 15s scan interval = 5 minutes
STARTUP_DASHBOARD_WAIT_SECONDS = 30
STARTUP_DASHBOARD_RETRY_INTERVAL = 0.5

# Recovery settings
MAX_CONSECUTIVE_FAILURES = 5
ADAPTER_RESET_COOLDOWN = 30
GATT_ADAPTER_CHECK_INTERVAL = 5
SENSOR_LOOP_HEARTBEAT_TIMEOUT = 120
SENSOR_ADAPTER_OPEN_TIMEOUT = 10

# UUIDs
SERVICE_UUID = UUID('12345678-1234-5678-1234-56789abcdef0')
BMS_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef1')
MOPEKA1_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef2')
MOPEKA2_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef3')
PUMP_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef4')
GALLONS_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef5')
REQUESTED_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef6')  # Requested gallons
ACTUAL_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef7')     # Actual gallons
HISTORY_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef8')    # Last 5 fills
BATCHMIX_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdef9')  # Batch mix data from iPad
TRAILER_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdefa')   # Trailer selection
CONFIG_CMD_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdefb') # Config command write
CONFIG_DATA_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdefc') # Config data read
STATE_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdefd')      # Live iOS-friendly dashboard state
COMMAND_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdefe')    # JSON command channel for iOS app
CONFIG_NOTIFY_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdeff')  # Config response notify/read
MAINT_CONTROL_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdf00')  # Maintenance shell control/session
MAINT_STDIN_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdf01')    # Maintenance shell stdin chunks
MAINT_STDOUT_CHAR_UUID = UUID('12345678-1234-5678-1234-56789abcdf02')   # Maintenance shell stdout notify/read

# File paths for mopeka data
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MOPEKA_DIR = os.path.join(SCRIPT_DIR, 'mopeka')
SENSOR_CSV_PATH = os.path.join(MOPEKA_DIR, 'mopeka-sensor-details.csv')
CALIBRATION_CSV_PATH = os.path.join(MOPEKA_DIR, 'calibration-points-1070gal-tank.csv')
CALIBRATION_PROFILE_DIR = os.path.join(MOPEKA_DIR, 'calibrations')
MOPEKA_CONFIG_PATH = os.path.join(MOPEKA_DIR, 'mopeka_config.json')
WATCHDOG_LOG_PATH = '/home/pi/rotorsync_watchdog.log'
GATT_DEVICE_PATH_FILE = '/home/pi/rotorsync_gatt_device_path'
DEFAULT_FLEET_BLE_NAME = 'TrailerSync-TR2'
DEFAULT_CUSTOMER_BLE_NAME = 'TrailerSync-Customer'

# Sensor data with timestamps
sensor_data = {
    'bms': {'voltage': 0, 'soc': 0, 'last_update': 0},
    'mopeka1': {'level_mm': 0, 'quality': 0, 'last_update': 0},
    'mopeka2': {'level_mm': 0, 'quality': 0, 'last_update': 0}
}

# Dashboard status
dashboard_status = {
    'requested': 0.0,
    'actual': 0.0,
    'mode': 'fill',
    'history': '',
    'state': {},
    'state_json': '{}',
    'last_update': 0
}

sensor_loop_heartbeat = 0.0
calibration_mtime_snapshot = None
last_calibration_reload_check = 0.0


def _encode_ble_state_payload(state):
    """Encode the dashboard snapshot into a compact BLE/iOS-friendly JSON payload."""
    compact = {
        'ver': state.get('version'),
        'req': state.get('requested_gal'),
        'act': state.get('actual_gal'),
        'flow': state.get('flow_gpm'),
        'mode': state.get('mode'),
        'ov': state.get('override'),
        'thumb': state.get('thumbs_visible'),
        'pend': state.get('fill_pending'),
        'confirm': state.get('can_confirm_fill'),
        'green': state.get('colors_green'),
        'latch': state.get('pump_stop_latched'),
        'fm_ok': state.get('flow_meter_connected'),
        'sb_ok': state.get('switch_box_connected'),
    }
    return json.dumps(compact, separators=(',', ':'))

# Config command state
config_response = '{"ok":false,"error":"No command issued"}'
config_response_pages = []  # Pre-computed pages for paginated responses
config_notify_char = None
maintenance_stdout_char = None
maintenance_response = '{"ok":false,"error":"No maintenance command issued"}'
maintenance_read_chunks = deque()
maintenance_active_sessions = set()
maintenance_notify_pending = False
ble_device = None
dashboard_ready = False
last_dashboard_error_log = 0.0
last_dashboard_error_message = None

# Chunked config command buffer
config_cmd_chunks = {}
config_cmd_chunk_timeout = 30
maintenance_write_buffers = {}
maintenance_chunk_buffers = {}
maintenance_chunk_timeout = 30
maintenance_stdout_notify_spacing = 0.30
maintenance_last_stdout_notify_time = 0.0

def _redact_dashboard_command(cmd):
    """Redact sensitive fields (like WiFi password) from command logs."""
    try:
        if cmd.startswith('WIFI_SET:'):
            payload = cmd.split(':', 1)[1]
            data = json.loads(payload)
            if isinstance(data, dict) and 'password' in data:
                data['password'] = '***'
            return f"WIFI_SET:{json.dumps(data, separators=(',', ':'))}"
    except Exception:
        pass
    return cmd


def send_dashboard_command(cmd):
    """Send command to dashboard via socket"""
    global dashboard_ready, last_dashboard_error_log, last_dashboard_error_message
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((DASHBOARD_HOST, DASHBOARD_PORT))
            s.send(f'{cmd}\n'.encode())
            response = s.recv(4096).decode().strip()
            dashboard_ready = True
            print(f"Dashboard command: {_redact_dashboard_command(cmd)} -> {response}", flush=True)
            return response
    except Exception as e:
        dashboard_ready = False
        message = f'Dashboard command error: {e}'
        now = time.time()
        if message != last_dashboard_error_message or now - last_dashboard_error_log >= 10:
            print(message, flush=True)
            last_dashboard_error_log = now
            last_dashboard_error_message = message
        return None


async def wait_for_dashboard_ready(timeout=STARTUP_DASHBOARD_WAIT_SECONDS):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if query_dashboard_status():
            return True
        await asyncio.sleep(STARTUP_DASHBOARD_RETRY_INTERVAL)
    return False

def query_fill_history():
    """Query dashboard for last 5 fills"""
    global dashboard_status
    response = send_dashboard_command('HISTORY')
    if response and response.startswith('HIST:'):
        dashboard_status['history'] = response[5:]
        return True
    return False

def query_dashboard_status():
    """Query dashboard for current state snapshot, with legacy STATUS fallback."""
    global dashboard_status
    response = send_dashboard_command('STATE_JSON')
    if response and response.startswith('STATE_JSON:'):
        try:
            payload = response.split(':', 1)[1]
            state = json.loads(payload)
            dashboard_status['state'] = state
            dashboard_status['state_json'] = _encode_ble_state_payload(state)
            dashboard_status['requested'] = float(state.get('requested_gal', 0.0))
            dashboard_status['actual'] = float(state.get('actual_gal', 0.0))
            dashboard_status['mode'] = str(state.get('mode', 'fill'))
            dashboard_status['last_update'] = time.time()
            return True
        except Exception as e:
            print(f'State JSON parse error: {e}', flush=True)

    response = send_dashboard_command('STATUS')
    if response and response.startswith('REQ:'):
        try:
            parts = response.split('|')
            for part in parts:
                if part.startswith('REQ:'):
                    dashboard_status['requested'] = float(part[4:])
                elif part.startswith('ACT:'):
                    dashboard_status['actual'] = float(part[4:])
                elif part.startswith('MODE:'):
                    dashboard_status['mode'] = part[5:]
            dashboard_status['state'] = {
                'requested_gal': dashboard_status['requested'],
                'actual_gal': dashboard_status['actual'],
                'mode': dashboard_status['mode'],
            }
            dashboard_status['state_json'] = _encode_ble_state_payload(
                dashboard_status['state']
            )
            dashboard_status['last_update'] = time.time()
            return True
        except Exception as e:
            print(f'Status parse error: {e}', flush=True)
    return False


def _extract_jbd_frame(buffer, expected_function=None):
    """Extract the first complete JBD frame from a notification buffer."""
    start = buffer.find(b'\xDD')
    while start != -1:
        if len(buffer) - start < 7:
            return None

        function = buffer[start + 1]
        data_len = buffer[start + 3]
        frame_len = 4 + data_len + 3
        end = start + frame_len
        if len(buffer) < end:
            return None

        frame = bytes(buffer[start:end])
        del buffer[:end]
        if frame[-1] != 0x77:
            start = buffer.find(b'\xDD')
            continue
        if expected_function is not None and function != expected_function:
            start = buffer.find(b'\xDD')
            continue
        return frame

    return None

def find_adapter_by_mac(mac):
    """Find hci index by MAC address"""
    mac = mac.upper()
    result = subprocess.run(['hciconfig', '-a'], capture_output=True, text=True)
    current_hci = None
    for line in result.stdout.split('\n'):
        if line.startswith('hci'):
            current_hci = line.split(':')[0]
        if mac in line.upper():
            return current_hci

    return None


def _format_adapter(adapter):
    if not adapter:
        return 'none'
    usb_id = f"{adapter.get('vendor_id', '')}:{adapter.get('product_id', '')}"
    desc = ' '.join(
        part for part in (adapter.get('manufacturer'), adapter.get('product'))
        if part
    )
    return f"{adapter.get('hci')} {adapter.get('mac')} {usb_id} {desc}".strip()


def select_runtime_adapters():
    """Resolve GATT and sensor HCI adapters by known USB chip role."""
    global GATT_ADAPTER_MAC, SENSOR_ADAPTER_MAC

    adapters = list_bluetooth_adapters()
    if adapters:
        print('Detected Bluetooth adapters:', flush=True)
        for adapter in adapters:
            print(f'  {_format_adapter(adapter)}', flush=True)

    gatt_adapter, sensor_adapter, used_usb_role = select_adapters(
        adapters,
        GATT_ADAPTER_MAC,
        SENSOR_ADAPTER_MAC,
    )

    if used_usb_role:
        print('Bluetooth adapter roles selected by USB chip identity', flush=True)

    if gatt_adapter and gatt_adapter.get('mac'):
        GATT_ADAPTER_MAC = gatt_adapter['mac']
    if sensor_adapter and sensor_adapter.get('mac'):
        SENSOR_ADAPTER_MAC = sensor_adapter['mac']

    return (
        gatt_adapter['hci'] if gatt_adapter else None,
        sensor_adapter['hci'] if sensor_adapter else None,
    )


def get_adapter_device_path(adapter):
    """Return the resolved sysfs device path for an adapter."""
    try:
        return Path('/sys/class/bluetooth', adapter, 'device').resolve()
    except FileNotFoundError:
        return None


def find_adapter_by_device_path(expected_device_path):
    """Find the current hci index for a physical Bluetooth USB interface."""
    if not expected_device_path:
        return None

    bluetooth_root = Path('/sys/class/bluetooth')
    for adapter_path in sorted(bluetooth_root.glob('hci*')):
        try:
            if (adapter_path / 'device').resolve() == expected_device_path:
                return adapter_path.name
        except FileNotFoundError:
            continue
        except Exception:
            continue

    return None

def reset_adapter(adapter):
    """Reset Bluetooth adapter"""
    print(f'Resetting adapter {adapter}...', flush=True)
    try:
        subprocess.run(['hciconfig', adapter, 'down'], capture_output=True, timeout=5)
        time.sleep(1)
        subprocess.run(['hciconfig', adapter, 'up'], capture_output=True, timeout=5)
        time.sleep(2)
        print(f'Adapter {adapter} reset complete', flush=True)
        return True
    except Exception as e:
        print(f'Adapter reset error: {e}', flush=True)
        return False

def adapter_exists(adapter):
    """Return True if the named HCI adapter still exists."""
    return Path('/sys/class/bluetooth', adapter).exists()

def log_watchdog_event(reason):
    """Log watchdog-triggered restarts in one dedicated file."""
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    count = 1

    try:
        with open(WATCHDOG_LOG_PATH, 'r', encoding='utf-8') as f:
            count += sum(1 for _ in f)
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f'Watchdog log read error: {e}', flush=True)

    line = f'{timestamp} - event {count}: {reason}'
    print(line, flush=True)

    try:
        with open(WATCHDOG_LOG_PATH, 'a', encoding='utf-8') as f:
            f.write(line + '\n')
    except Exception as e:
        print(f'Watchdog log write error: {e}', flush=True)


def persist_gatt_device_path(device_path):
    """Persist the current GATT adapter sysfs path for the watchdog."""
    try:
        path_str = str(device_path) if device_path else ''
        with open(GATT_DEVICE_PATH_FILE, 'w', encoding='utf-8') as f:
            f.write(path_str + '\n')
    except Exception as e:
        print(f'Failed to persist GATT device path: {e}', flush=True)

async def monitor_gatt_adapter(expected_adapter, expected_device_path):
    """Exit so systemd can restart if the GATT adapter is re-enumerated."""
    while True:
        await asyncio.sleep(GATT_ADAPTER_CHECK_INTERVAL)
        current_adapter = find_adapter_by_device_path(expected_device_path)

        if not current_adapter:
            log_watchdog_event(
                f'GATT adapter path {expected_device_path} disappeared; exiting for restart'
            )
            os._exit(1)

        if current_adapter != expected_adapter:
            log_watchdog_event(
                f'GATT adapter moved from {expected_adapter} to {current_adapter}; exiting for restart'
            )
            os._exit(1)

        if not adapter_exists(expected_adapter):
            log_watchdog_event(
                f'GATT adapter {expected_adapter} no longer exists; exiting for restart'
            )
            os._exit(1)

def make_read_handler(data_key):
    def read_value(connection):
        data = sensor_data[data_key].copy()
        data.pop('last_update', None)
        value = json.dumps(data)
        print(f'ReadValue {data_key}: {value}', flush=True)
        return bytes(value, 'utf-8')
    return read_value

def make_history_read_handler():
    def read_value(connection):
        value = dashboard_status['history']
        print(f'ReadValue history: {value[:50]}...', flush=True)
        return bytes(value, 'utf-8')
    return read_value

def make_dashboard_read_handler(field):
    def read_value(connection):
        value = str(dashboard_status[field])
        print(f'ReadValue {field}: {value}', flush=True)
        return bytes(value, 'utf-8')
    return read_value


def make_state_read_handler():
    def read_value(connection):
        value = dashboard_status.get('state_json', '{}')
        print(f'ReadValue state: {value[:120]}', flush=True)
        return bytes(value, 'utf-8')
    return read_value


def make_config_notify_read_handler():
    def read_value(connection):
        value = config_response
        print(f'ReadValue config_notify: {value[:120]}', flush=True)
        return bytes(value, 'utf-8')
    return read_value


def make_maintenance_stdout_read_handler():
    def read_value(connection):
        global maintenance_response
        if maintenance_read_chunks:
            value = maintenance_read_chunks.popleft()
            maintenance_response = value
            _schedule_maintenance_output_notify()
        else:
            value = maintenance_response
        print(f'ReadValue maintenance_stdout: {value[:120]}', flush=True)
        return bytes(value, 'utf-8')
    return read_value


def _connection_key(connection):
    for attr in ('peer_address', 'address', 'handle'):
        value = getattr(connection, attr, None)
        if value is not None:
            return str(value)
    return 'default'


def _notify_config_response():
    if not ble_device or not config_notify_char:
        return

    async def _notify():
        try:
            await ble_device.notify_subscribers(config_notify_char, config_response.encode('utf-8'))
        except Exception as e:
            print(f'Config notify error: {e}', flush=True)

    try:
        asyncio.get_running_loop().create_task(_notify())
    except RuntimeError:
        pass


def _notify_maintenance_response():
    if not ble_device or not maintenance_stdout_char:
        return

    async def _notify():
        try:
            await ble_device.notify_subscribers(maintenance_stdout_char, maintenance_response.encode('utf-8'))
            print(f'Notified maintenance_stdout: {maintenance_response[:120]}', flush=True)
        except Exception as e:
            print(f'Maintenance notify error: {e}', flush=True)

    try:
        asyncio.get_running_loop().create_task(_notify())
    except RuntimeError:
        pass


def _schedule_maintenance_output_notify(delay=0.0):
    global maintenance_notify_pending
    if maintenance_notify_pending:
        return
    maintenance_notify_pending = True

    async def _notify_next():
        global maintenance_notify_pending, maintenance_response, maintenance_last_stdout_notify_time
        requested_delay = max(delay, 0.0)
        try:
            while maintenance_read_chunks and ble_device and maintenance_stdout_char:
                elapsed = time.monotonic() - maintenance_last_stdout_notify_time
                notify_delay = max(requested_delay, maintenance_stdout_notify_spacing - elapsed)
                if notify_delay > 0:
                    await asyncio.sleep(notify_delay)
                requested_delay = 0.0
                maintenance_response = maintenance_read_chunks.popleft()
                await ble_device.notify_subscribers(maintenance_stdout_char, maintenance_response.encode('utf-8'))
                maintenance_last_stdout_notify_time = time.monotonic()
                print(f'Notified maintenance_stdout queued: {maintenance_response[:120]}', flush=True)
        except Exception as e:
            print(f'Maintenance notify error: {e}', flush=True)
        finally:
            maintenance_notify_pending = False
            if maintenance_read_chunks:
                _schedule_maintenance_output_notify(maintenance_stdout_notify_spacing)

    try:
        asyncio.get_running_loop().create_task(_notify_next())
    except RuntimeError:
        maintenance_notify_pending = False


def _queue_maintenance_response_text(text):
    global maintenance_response
    maintenance_read_chunks.append(text)
    while len(maintenance_read_chunks) > 300:
        maintenance_read_chunks.popleft()
    if len(maintenance_read_chunks) == 1:
        maintenance_response = text
    _schedule_maintenance_output_notify()


def _set_maintenance_response_obj(obj):
    global maintenance_response
    maintenance_response = json.dumps(obj, separators=(',', ':'))
    _notify_maintenance_response()


async def maintenance_agent_bridge_loop():
    """Keep a local TCP connection to bbb-maint-agent and notify BLE output."""
    global maintenance_writer
    while True:
        try:
            print('Connecting to bbb-maint-agent on 127.0.0.1:18991...', flush=True)
            reader, writer = await asyncio.open_connection('127.0.0.1', 18991)
            maintenance_writer = writer
            _set_maintenance_response_obj({'type': 'bridge_status', 'status': 'connected', 'timestamp': time.time()})
            while True:
                line = await reader.readline()
                if not line:
                    break
                try:
                    text = line.decode('utf-8').strip()
                except UnicodeDecodeError:
                    text = json.dumps({'type': 'error', 'message': 'maintenance agent sent non-utf8 output'})
                print(f'Maintenance agent output: {text[:160]}', flush=True)
                _queue_maintenance_response_text(text)
        except Exception as e:
            _set_maintenance_response_obj({'type': 'bridge_status', 'status': 'disconnected', 'error': str(e), 'timestamp': time.time()})
            await asyncio.sleep(2)
        finally:
            maintenance_writer = None


def _schedule_maintenance_agent_frame(frame_text):
    async def _send():
        global maintenance_writer
        if maintenance_writer is None:
            _set_maintenance_response_obj({'type': 'error', 'message': 'bbb-maint-agent is not connected', 'timestamp': time.time()})
            return
        try:
            maintenance_writer.write(frame_text.encode('utf-8') + b'\n')
            await maintenance_writer.drain()
        except Exception as e:
            _set_maintenance_response_obj({'type': 'error', 'message': f'maintenance bridge write failed: {e}', 'timestamp': time.time()})

    try:
        asyncio.get_running_loop().create_task(_send())
    except RuntimeError:
        pass


def _handle_maintenance_agent_write(connection, value, channel):
    key = f'{_connection_key(connection)}:{channel}'
    try:
        chunk = value.decode('utf-8')
        if chunk.startswith('MCHUNK:'):
            parts = chunk.split(':', 3)
            if len(parts) != 4:
                print(f'Maintenance {channel} invalid chunk header', flush=True)
                return
            _, message_id, chunk_info, chunk_data = parts
            try:
                chunk_num_str, total_str = chunk_info.split('/', 1)
                chunk_num = int(chunk_num_str)
                total_chunks = int(total_str)
            except ValueError:
                print(f'Maintenance {channel} invalid chunk info {chunk_info!r}', flush=True)
                return
            if chunk_num < 1 or total_chunks < 1 or chunk_num > total_chunks or total_chunks > 200:
                print(f'Maintenance {channel} invalid chunk range {chunk_info!r}', flush=True)
                return

            chunk_key = f'{key}:{message_id}'
            now = time.time()
            stale_keys = [
                existing_key for existing_key, existing in maintenance_chunk_buffers.items()
                if now - existing.get('timestamp', now) > maintenance_chunk_timeout
            ]
            for stale_key in stale_keys:
                maintenance_chunk_buffers.pop(stale_key, None)

            buffer = maintenance_chunk_buffers.get(chunk_key)
            if not buffer or buffer.get('total') != total_chunks:
                buffer = {'total': total_chunks, 'chunks': {}, 'timestamp': now}
                maintenance_chunk_buffers[chunk_key] = buffer
            buffer['chunks'][chunk_num] = chunk_data
            buffer['timestamp'] = now
            print(f'Maintenance {channel} chunk {chunk_num}/{total_chunks} id={message_id}', flush=True)

            if len(buffer['chunks']) != total_chunks:
                return

            encoded = ''.join(buffer['chunks'][index] for index in range(1, total_chunks + 1))
            maintenance_chunk_buffers.pop(chunk_key, None)
            try:
                data_str = base64.b64decode(encoded.encode('ascii')).decode('utf-8')
            except Exception as e:
                print(f'Maintenance {channel} chunk decode error: {e}', flush=True)
                _set_maintenance_response_obj({
                    'type': 'error',
                    'message': f'maintenance {channel} chunk decode failed',
                    'timestamp': time.time(),
                })
                return
        else:
            data_str = (maintenance_write_buffers.get(key, '') + chunk).strip()
        try:
            json.loads(data_str)
        except json.JSONDecodeError:
            if len(data_str) > 8192:
                maintenance_write_buffers.pop(key, None)
                print(f'Maintenance {channel} error: partial JSON exceeded 8192 bytes', flush=True)
                _set_maintenance_response_obj({
                    'type': 'error',
                    'message': f'maintenance {channel} frame exceeded chunk buffer',
                    'timestamp': time.time(),
                })
            else:
                maintenance_write_buffers[key] = data_str
                print(f'Maintenance {channel} chunk buffered: {len(data_str)} bytes', flush=True)
            return

        maintenance_write_buffers.pop(key, None)
        print(f'Maintenance {channel} write: {data_str[:160]}', flush=True)
        _schedule_maintenance_agent_frame(data_str)
    except Exception as e:
        maintenance_write_buffers.pop(key, None)
        print(f'Maintenance {channel} error: {e}', flush=True)


def maintenance_control_write_handler(connection, value):
    _handle_maintenance_agent_write(connection, value, 'control')


def maintenance_stdin_write_handler(connection, value):
    _handle_maintenance_agent_write(connection, value, 'stdin')


def _set_config_response_obj(obj):
    global config_response
    config_response = json.dumps(obj, separators=(',', ':'))
    _notify_config_response()


def _schedule_identity_restart(reason, delay=1.0):
    async def _restart():
        await asyncio.sleep(delay)
        log_watchdog_event(reason)
        os._exit(0)

    try:
        asyncio.get_running_loop().create_task(_restart())
    except RuntimeError:
        pass


def _set_paginated_config_response(items, *, request_id=None, op=None, page_size_bytes=450):
    global config_response_pages
    pages = paginate_response(items, page_size_bytes=page_size_bytes)
    enriched_pages = []
    for page_json in pages:
        page_obj = json.loads(page_json)
        if request_id is not None:
            page_obj['request_id'] = request_id
        if op:
            page_obj['op'] = op
        enriched_pages.append(json.dumps(page_obj, separators=(',', ':')))
    config_response_pages = enriched_pages
    if enriched_pages:
        _set_config_response_obj(json.loads(enriched_pages[0]))
    else:
        _set_config_response_obj({
            'ok': True,
            'op': op,
            'request_id': request_id,
            'page': 1,
            'total_pages': 1,
            'total_items': 0,
            'items': [],
        })

def pump_write_handler(connection, value):
    """Handle pump control writes. Write '1' or 'PS' to stop pump."""
    cmd = value.decode('utf-8').strip().upper()
    print(f'Pump write: {cmd}', flush=True)
    if cmd in ['1', 'PS', 'STOP']:
        send_dashboard_command('PS')
    return


def command_write_handler(connection, value):
    """Handle compact JSON commands from the iOS app."""
    try:
        payload = value.decode('utf-8').strip()
        print(f'Command write: {payload[:200]}', flush=True)
        cmd = json.loads(payload)
        if not isinstance(cmd, dict):
            raise ValueError('command payload must be a JSON object')
    except Exception as e:
        print(f'Command write parse error: {e}', flush=True)
        return

    command = str(cmd.get('cmd', '')).strip().lower()
    if not command:
        print('Command write ignored: missing cmd', flush=True)
        return

    if command == 'pump_stop':
        send_dashboard_command('PS')
        query_dashboard_status()
        return

    if command == 'confirm_fill':
        send_dashboard_command('TU')
        query_dashboard_status()
        return

    if command == 'set_mode':
        mode = str(cmd.get('mode', '')).strip().lower()
        if mode == 'mix':
            send_dashboard_command('MIX')
            query_dashboard_status()
        elif mode == 'fill':
            send_dashboard_command('FILL')
            query_dashboard_status()
        else:
            print(f'Command write ignored: invalid mode {mode!r}', flush=True)
        return

    if command == 'adjust':
        delta = cmd.get('delta')
        allowed = {
            1: '+1',
            -1: '-1',
            10: '+10',
            -10: '-10',
        }
        try:
            delta = int(delta)
        except Exception:
            delta = None
        if delta in allowed:
            send_dashboard_command(allowed[delta])
            query_dashboard_status()
        else:
            print(f'Command write ignored: invalid delta {delta!r}', flush=True)
        return

    if command == 'set_override':
        enabled = cmd.get('enabled')
        if isinstance(enabled, bool):
            desired = enabled
        elif isinstance(enabled, int) and enabled in (0, 1):
            desired = bool(enabled)
        elif isinstance(enabled, str):
            normalized = enabled.strip().lower()
            if normalized in ('true', '1', 'yes', 'on'):
                desired = True
            elif normalized in ('false', '0', 'no', 'off'):
                desired = False
            else:
                print(f'Command write ignored: invalid override value {enabled!r}', flush=True)
                return
        else:
            print(f'Command write ignored: invalid override value {enabled!r}', flush=True)
            return
        send_dashboard_command('OV:1' if desired else 'OV:0')
        query_dashboard_status()
        return

    if command == 'set_batchmix':
        data = cmd.get('data')
        if isinstance(data, dict):
            _send_validated_batchmix(json.dumps(data, separators=(",", ":")))
        else:
            print('Command write ignored: set_batchmix missing object data', flush=True)
        return

    print(f'Command write ignored: unsupported cmd {command!r}', flush=True)


def gallons_write_handler(connection, value):
    """Handle gallons adjustment. Write '+1', '-1', '+10', '-10'."""
    cmd = value.decode('utf-8').strip()
    print(f'Gallons write: {cmd}', flush=True)

    if cmd in ['+1', '-1', '+10', '-10']:
        send_dashboard_command(cmd)
        # Update local status after change
        asyncio.get_event_loop().call_soon(query_dashboard_status)
    return

# Chunked data buffer for batch mix (handles large payloads)
batchmix_chunks = {}
batchmix_chunk_timeout = 30  # Seconds before incomplete chunks expire

def _send_validated_batchmix(compact_json):
    """Validate and forward a compact BatchMix JSON payload to the dashboard."""
    try:
        data = json.loads(compact_json)
    except json.JSONDecodeError as je:
        error_msg = f'Invalid JSON: {je}'
        print(f'BatchMix ERROR: {error_msg}', flush=True)
        send_dashboard_command(f'BATCHMIX_ERROR:{error_msg}')
        return False

    error_msg = batchmix_validation_error(data)
    if error_msg:
        print(f'BatchMix ERROR: {error_msg}', flush=True)
        send_dashboard_command(f'BATCHMIX_ERROR:{error_msg}')
        return False

    print(f'BatchMix validated: {len(data.get("products", []))} products', flush=True)
    send_dashboard_command(f'BATCHMIX:{compact_json}')
    return True

def batchmix_write_handler(connection, value):
    """Handle batch mix data from iPad. Supports chunked or single-write JSON.

    Chunked format: CHUNK:X/Y:data
      - X = chunk number (1-based)
      - Y = total chunks
      - data = partial JSON

    Single write: raw JSON (if data fits in one BLE write)
    """
    global batchmix_chunks

    try:
        data_str = value.decode('utf-8').strip()

        # Check if this is chunked data
        if data_str.startswith('CHUNK:'):
            # Parse chunk header: CHUNK:X/Y:data
            parts = data_str.split(':', 2)
            if len(parts) >= 3:
                chunk_info = parts[1]  # "X/Y"
                chunk_data = parts[2]  # The actual data

                chunk_num, total_chunks = map(int, chunk_info.split('/'))

                print(f'BatchMix chunk {chunk_num}/{total_chunks} ({len(chunk_data)} bytes)', flush=True)

                # Use single global buffer (simplified - one sender at a time)
                # Reset buffer if total_chunks changed (new transmission)
                if 'total' not in batchmix_chunks or batchmix_chunks['total'] != total_chunks:
                    batchmix_chunks = {
                        'chunks': {},
                        'total': total_chunks,
                        'timestamp': time.time()
                    }

                # Store this chunk
                batchmix_chunks['chunks'][chunk_num] = chunk_data
                batchmix_chunks['timestamp'] = time.time()

                print(f'BatchMix: have {len(batchmix_chunks["chunks"])}/{total_chunks} chunks', flush=True)

                # Check if we have all chunks
                if len(batchmix_chunks['chunks']) == total_chunks:
                    # Assemble complete JSON
                    assembled = ''
                    for i in range(1, total_chunks + 1):
                        if i in batchmix_chunks['chunks']:
                            assembled += batchmix_chunks['chunks'][i]
                        else:
                            print(f'BatchMix: missing chunk {i}!', flush=True)
                            return

                    print(f'BatchMix complete: {len(assembled)} bytes from {total_chunks} chunks', flush=True)

                    # Strip newlines for socket transmission
                    compact_json = assembled.replace('\n', '').replace('\r', '')

                    _send_validated_batchmix(compact_json)

                    # Clear buffer
                    batchmix_chunks = {}

        else:
            # Single write (no chunking) - data fits in one BLE write
            print(f'BatchMix single write: {len(data_str)} bytes', flush=True)
            # Strip newlines for socket transmission
            compact_json = data_str.replace('\n', '').replace('\r', '')

            _send_validated_batchmix(compact_json)

    except Exception as e:
        print(f'BatchMix error: {e}', flush=True)
    return


# =============================================================================
# CSV + Config Helper Functions
# =============================================================================

SENSOR_CSV_HEADER = ['Man', 'Trailer', 'Tank', 'Center Sump?', 'Height Offset',
                     'Mopeka Name in app', 'Mopeka ID', 'MQTT Topic for app', 'Added to app']

def load_sensor_csv():
    """Parse sensor CSV. 4 blank preamble rows, header on row 5.
    Man column only on Front rows - carry forward for Back rows."""
    sensors = []
    try:
        with open(SENSOR_CSV_PATH, 'r', newline='') as f:
            reader = csv.reader(f)
            for _ in range(4):
                next(reader)
            header = next(reader)
            current_man = ''
            for row in reader:
                if not row or len(row) < 2 or not row[1].strip():
                    continue
                d = {}
                for i, h in enumerate(header):
                    d[h.strip()] = row[i].strip() if i < len(row) else ''
                if d.get('Man'):
                    current_man = d['Man']
                else:
                    d['Man'] = current_man
                sensors.append(d)
    except FileNotFoundError:
        print(f'Sensor CSV not found: {SENSOR_CSV_PATH}', flush=True)
    return sensors


def save_sensor_csv(sensors):
    """Write sensors back to CSV preserving format (4 blank rows, header, data)."""
    with open(SENSOR_CSV_PATH, 'w', newline='') as f:
        writer = csv.writer(f)
        for _ in range(4):
            writer.writerow([''] * len(SENSOR_CSV_HEADER))
        writer.writerow(SENSOR_CSV_HEADER)
        last_man = ''
        for s in sensors:
            row = []
            for h in SENSOR_CSV_HEADER:
                val = s.get(h, '')
                if h == 'Man':
                    if val == last_man:
                        row.append('')
                    else:
                        row.append(val)
                        last_man = val
                else:
                    row.append(val)
            writer.writerow(row)


def _safe_calibration_profile_key(value):
    key = str(value or '').strip().lower().replace('_', '-')
    return ''.join(ch for ch in key if ch.isalnum() or ch == '-')


def _calibration_profile_key_for_box_tank(tank):
    cfg = load_config()
    mode = _normalize_box_mode(cfg.get('box_mode'))
    trailer = _get_assigned_trailer(cfg)
    tank = str(tank or '').strip().lower()
    tank = 'back' if tank.startswith('back') else 'front'

    if mode == 'fleet' and trailer not in (None, ''):
        return _safe_calibration_profile_key(f'trailer-{trailer}-{tank}')
    return f'customer-{tank}'


def _calibration_csv_path_for_cmd(cmd=None):
    cmd = cmd or {}
    profile = cmd.get('profile')
    if not profile and cmd.get('tank'):
        profile = _calibration_profile_key_for_box_tank(cmd.get('tank'))

    profile = _safe_calibration_profile_key(profile)
    if profile:
        os.makedirs(CALIBRATION_PROFILE_DIR, exist_ok=True)
        return os.path.join(CALIBRATION_PROFILE_DIR, f'{profile}.csv'), profile

    return CALIBRATION_CSV_PATH, ''


def load_calibration_csv(path=CALIBRATION_CSV_PATH):
    """Load calibration CSV. Standard header, no preamble."""
    points = []
    try:
        with open(path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                points.append({
                    'tank_level_in': float(row['Tank Level (in)']),
                    'gallons': float(row['Gallons']),
                    'tank_size': float(row['Tank Size (gal)'])
                })
    except FileNotFoundError:
        print(f'Calibration CSV not found: {path}', flush=True)
    return points


def save_calibration_csv(points, path=CALIBRATION_CSV_PATH):
    """Write calibration points back to CSV, sorted descending by tank level."""
    points.sort(key=lambda p: p['tank_level_in'], reverse=True)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Tank Level (in)', 'Gallons', 'Tank Size (gal)'])
        for p in points:
            writer.writerow([p['tank_level_in'], p['gallons'], p['tank_size']])


def _calibration_file_mtimes():
    paths = [CALIBRATION_CSV_PATH]
    if os.path.isdir(CALIBRATION_PROFILE_DIR):
        for name in sorted(os.listdir(CALIBRATION_PROFILE_DIR)):
            if name.lower().endswith('.csv'):
                paths.append(os.path.join(CALIBRATION_PROFILE_DIR, name))

    mtimes = {}
    for path in paths:
        try:
            mtimes[path] = os.path.getmtime(path)
        except FileNotFoundError:
            mtimes[path] = None
    return mtimes


def reload_converter_if_calibration_changed(force=False):
    """Reload Mopeka conversion when dashboard-applied profiles change."""
    global calibration_mtime_snapshot, last_calibration_reload_check

    now = time.time()
    if not force and now - last_calibration_reload_check < 5.0:
        return

    last_calibration_reload_check = now
    current = _calibration_file_mtimes()
    if calibration_mtime_snapshot is None:
        calibration_mtime_snapshot = current
        return

    if current != calibration_mtime_snapshot:
        calibration_mtime_snapshot = current
        print('Calibration files changed; reloading Mopeka converter', flush=True)
        _reload_converter()


def load_config():
    """Load mopeka_config.json."""
    try:
        with open(MOPEKA_CONFIG_PATH, 'r') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {
            'box_mode': 'fleet',
            'assigned_trailer': None,
            'trailer': None,
        }


def save_config(cfg):
    """Write mopeka_config.json."""
    os.makedirs(os.path.dirname(MOPEKA_CONFIG_PATH), exist_ok=True)
    with open(MOPEKA_CONFIG_PATH, 'w') as f:
        json.dump(cfg, f, indent=2)


def _normalize_ble_mac(value):
    mac = str(value or '').strip().upper().replace('-', ':')
    parts = mac.split(':')
    if len(parts) != 6 or any(len(part) != 2 for part in parts):
        return None
    if any(any(ch not in '0123456789ABCDEF' for ch in part) for part in parts):
        return None
    return ':'.join(parts)


def _normalize_box_mode(value):
    mode = str(value or 'fleet').strip().lower()
    return 'customer' if mode == 'customer' else 'fleet'


def _get_assigned_trailer(cfg=None):
    cfg = cfg or load_config()
    return cfg.get('assigned_trailer', cfg.get('trailer'))


def _compute_bms_name(cfg=None):
    cfg = cfg or load_config()
    explicit_name = str(cfg.get('bms_name') or '').strip()
    if explicit_name:
        return explicit_name

    trailer = _get_assigned_trailer(cfg)
    if trailer not in (None, ''):
        return f'TR{trailer}-BMS'

    return BMS_NAME


def _box_mode_uses_trailer_list(cfg=None):
    cfg = cfg or load_config()
    return _normalize_box_mode(cfg.get('box_mode')) == 'fleet'


def _mopeka_enabled(cfg=None):
    cfg = cfg or load_config()
    return bool(cfg.get('mopeka_enabled', True))


def _compute_ble_name(cfg=None):
    cfg = cfg or load_config()
    mode = _normalize_box_mode(cfg.get('box_mode'))
    display_name = str(cfg.get('display_name') or '').strip()
    trailer = _get_assigned_trailer(cfg)

    if mode == 'customer':
        return display_name or DEFAULT_CUSTOMER_BLE_NAME

    if trailer not in (None, ''):
        return f'TrailerSync-TR{trailer}'

    return display_name or DEFAULT_FLEET_BLE_NAME


def _current_box_config():
    cfg = load_config()
    mode = _normalize_box_mode(cfg.get('box_mode'))
    return {
        'box_mode': mode,
        'assigned_trailer': _get_assigned_trailer(cfg),
        'display_name': str(cfg.get('display_name') or '').strip(),
        'ble_name': _compute_ble_name(cfg),
        'mopeka_enabled': _mopeka_enabled(cfg),
        'trailer_list_enabled': mode == 'fleet',
    }


def enforce_bumble_only_stack():
    """Best-effort block of BlueZ so Bumble can keep exclusive HCI ownership."""
    commands = [
        ['systemctl', 'disable', '--now', 'bluetooth.service'],
        ['systemctl', 'mask', 'bluetooth.service'],
        ['systemctl', 'stop', 'bluetooth.service'],
    ]
    for cmd in commands:
        try:
            subprocess.run(cmd, capture_output=True)
        except Exception:
            pass


# =============================================================================
# Trailer Selection Logic
# =============================================================================

def apply_trailer(trailer_num):
    """Look up trailer in sensor CSV, set MOPEKA1/2_MAC_SUFFIX globals,
    save config, reload mopeka_converter, return trailer info dict."""
    global MOPEKA1_MAC_SUFFIX, MOPEKA2_MAC_SUFFIX, BMS_NAME

    sensors = load_sensor_csv()
    trailer_sensors = [s for s in sensors if str(s.get('Trailer')) == str(trailer_num)]

    if not trailer_sensors:
        return None

    front = next((s for s in trailer_sensors if s.get('Tank') == 'Front'), None)
    back = next((s for s in trailer_sensors if s.get('Tank') == 'Back'), None)
    man = trailer_sensors[0].get('Man', '')

    front_id = front['Mopeka ID'] if front else '---------------'
    back_id = back['Mopeka ID'] if back else '---------------'

    def parse_offset(sensor):
        if sensor and sensor.get('Height Offset'):
            try:
                return float(sensor['Height Offset'])
            except ValueError:
                pass
        return 0.0

    front_offset = parse_offset(front)
    back_offset = parse_offset(back)

    # Update globals for the scanner
    if front_id != '---------------':
        MOPEKA1_MAC_SUFFIX = front_id
    if back_id != '---------------':
        MOPEKA2_MAC_SUFFIX = back_id

    # Persist to config
    cfg = load_config()
    cfg.update({
        'box_mode': _normalize_box_mode(cfg.get('box_mode')),
        'assigned_trailer': trailer_num,
        'trailer': trailer_num,
        'front_id': front_id,
        'back_id': back_id,
        'display_name': f'TrailerSync-TR{trailer_num}',
    })
    save_config(cfg)
    BMS_NAME = _compute_bms_name(cfg)

    # Reload mopeka_converter so offsets match new sensors
    try:
        mopeka_reload()
    except Exception as e:
        print(f'mopeka_converter reload: {e}', flush=True)

    info = {
        'trailer': trailer_num,
        'man': man,
        'front': {'id': front_id, 'offset': front_offset},
        'back': {'id': back_id, 'offset': back_offset}
    }

    print(f'Applied trailer {trailer_num}: front={front_id} back={back_id}', flush=True)
    return info


def restore_trailer_config():
    """Restore trailer config on startup from mopeka_config.json."""
    global MOPEKA1_MAC_SUFFIX, MOPEKA2_MAC_SUFFIX
    cfg = load_config()
    front_id = str(cfg.get('front_id') or '').strip().upper()
    back_id = str(cfg.get('back_id') or '').strip().upper()
    trailer_num = _get_assigned_trailer(cfg)

    # If no trailer is assigned, or the box is in customer mode, fall back to
    # the explicitly stored sensor IDs. This keeps manually configured boxes
    # working across restarts and prevents a blank trailer assignment from
    # discarding otherwise valid Mopeka sensor settings.
    if not _box_mode_uses_trailer_list(cfg) or trailer_num is None:
        restored = False

        if front_id and front_id != '---------------':
            MOPEKA1_MAC_SUFFIX = front_id
            restored = True

        if back_id and back_id != '---------------':
            MOPEKA2_MAC_SUFFIX = back_id
            restored = True

        if restored:
            print(
                f'Restored manual Mopeka IDs from config: '
                f'front={MOPEKA1_MAC_SUFFIX or "-"} back={MOPEKA2_MAC_SUFFIX or "-"}',
                flush=True,
            )
        elif not _box_mode_uses_trailer_list(cfg):
            print('Customer mode active; no manual Mopeka IDs configured', flush=True)
        else:
            print('No trailer assigned and no manual Mopeka IDs configured', flush=True)
        return

    if trailer_num is not None:
        result = apply_trailer(trailer_num)
        if result:
            print(f'Restored trailer {trailer_num} from config', flush=True)
        else:
            print(f'Failed to restore trailer {trailer_num}', flush=True)


def restore_bms_config():
    """Restore BMS config from mopeka_config.json."""
    global BMS_MAC, BMS_NAME
    cfg = load_config()
    saved_mac = _normalize_ble_mac(cfg.get('bms_mac'))
    if saved_mac:
        BMS_MAC = saved_mac
        print(f'Restored BMS MAC from config: {BMS_MAC}', flush=True)
    BMS_NAME = _compute_bms_name(cfg)
    print(f'Restored BMS name from config: {BMS_NAME}', flush=True)


def restore_adapter_config():
    """Restore per-box Bluetooth adapter MACs from mopeka_config.json."""
    global GATT_ADAPTER_MAC, SENSOR_ADAPTER_MAC
    cfg = load_config()

    saved_gatt = _normalize_ble_mac(cfg.get('gatt_adapter_mac'))
    if saved_gatt:
        GATT_ADAPTER_MAC = saved_gatt
        print(f'Restored GATT adapter MAC from config: {GATT_ADAPTER_MAC}', flush=True)

    saved_sensor = _normalize_ble_mac(cfg.get('sensor_adapter_mac'))
    if saved_sensor:
        SENSOR_ADAPTER_MAC = saved_sensor
        print(f'Restored sensor adapter MAC from config: {SENSOR_ADAPTER_MAC}', flush=True)


def persist_adapter_config():
    """Persist the active per-box Bluetooth adapter MACs to mopeka_config.json."""
    cfg = load_config()
    changed = False

    if GATT_ADAPTER_MAC and _normalize_ble_mac(cfg.get('gatt_adapter_mac')) != GATT_ADAPTER_MAC:
        cfg['gatt_adapter_mac'] = GATT_ADAPTER_MAC
        changed = True

    if SENSOR_ADAPTER_MAC and _normalize_ble_mac(cfg.get('sensor_adapter_mac')) != SENSOR_ADAPTER_MAC:
        cfg['sensor_adapter_mac'] = SENSOR_ADAPTER_MAC
        changed = True

    if changed:
        save_config(cfg)
        print(
            f'Persisted adapter config: gatt={GATT_ADAPTER_MAC} sensor={SENSOR_ADAPTER_MAC}',
            flush=True,
        )


# =============================================================================
# Pagination
# =============================================================================

def paginate_response(items, page_size_bytes=450):
    """Split items into pages that fit in BLE reads (~512 byte limit).
    Returns list of page JSON strings."""
    if not items:
        return [json.dumps({'page': 1, 'total_pages': 1, 'total_items': 0, 'items': []})]

    pages_items = []
    current_page = []
    overhead = 60
    current_size = overhead

    for item in items:
        item_json = json.dumps(item, separators=(',', ':'))
        item_size = len(item_json) + 1
        if current_size + item_size > page_size_bytes and current_page:
            pages_items.append(current_page)
            current_page = []
            current_size = overhead
        current_page.append(item)
        current_size += item_size

    if current_page:
        pages_items.append(current_page)

    total_pages = len(pages_items)
    total_items = len(items)

    result = []
    for i, page_items in enumerate(pages_items):
        result.append(json.dumps({
            'page': i + 1,
            'total_pages': total_pages,
            'total_items': total_items,
            'items': page_items
        }, separators=(',', ':')))

    return result


# =============================================================================
# Command Processor
# =============================================================================



def _wifi_code_from_response(resp):
    if not resp:
        return 'NO_RESPONSE'
    if resp.startswith('WIFI_OK:'):
        return 'OK'
    if resp.startswith('WIFI_ERR:'):
        try:
            payload = resp.split(':', 1)[1]
            data = json.loads(payload)
            return data.get('code', 'UNKNOWN')
        except Exception:
            return 'UNKNOWN'
    return 'BAD_RESPONSE'


def _wifi_set_from_ble(cmd):
    """Handle WIFI_SET operation from config command channel and forward to dashboard."""
    global config_response, config_response_pages
    request_id = cmd.get('request_id')

    ssid = str(cmd.get('ssid', '')).strip()
    password = str(cmd.get('password', ''))
    hidden = bool(cmd.get('hidden', False))

    if not ssid:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'WIFI_SET', 'request_id': request_id, 'error': 'Missing ssid'})
        return

    if len(ssid) > 64:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'WIFI_SET', 'request_id': request_id, 'error': 'SSID too long'})
        return

    if len(password) > 128:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'WIFI_SET', 'request_id': request_id, 'error': 'Password too long'})
        return

    payload = {
        'ssid': ssid,
        'password': password,
        'hidden': hidden,
    }

    resp = send_dashboard_command(f"WIFI_SET:{json.dumps(payload, separators=(',', ':'))}")
    code = _wifi_code_from_response(resp)

    if resp and resp.startswith('WIFI_OK:'):
        try:
            data = json.loads(resp.split(':', 1)[1])
        except Exception:
            data = {'ssid': ssid}
        data['ok'] = True
        data['op'] = 'WIFI_SET'
        if request_id is not None:
            data['request_id'] = request_id
        _set_config_response_obj(data)
    else:
        _set_config_response_obj({'ok': False, 'op': 'WIFI_SET', 'request_id': request_id, 'error': code})

    config_response_pages = []


def _wifi_status_from_ble(cmd=None):
    """Query current WiFi status from dashboard."""
    global config_response, config_response_pages
    request_id = cmd.get('request_id') if isinstance(cmd, dict) else None

    resp = send_dashboard_command('WIFI_STATUS')
    if resp and resp.startswith('WIFI_STATUS:'):
        payload = resp.split(':', 1)[1]
        try:
            data = json.loads(payload)
        except Exception:
            data = {'ok': False, 'error': 'PARSE_ERROR'}
        data['op'] = 'WIFI_STATUS'
        if request_id is not None:
            data['request_id'] = request_id
        _set_config_response_obj(data)
    else:
        _set_config_response_obj({'ok': False, 'op': 'WIFI_STATUS', 'request_id': request_id, 'error': 'NO_RESPONSE'})
    config_response_pages = []


def process_config_command(cmd_str):
    """Parse JSON command, dispatch by op field. Sets config_response."""
    global config_response, config_response_pages

    try:
        cmd = json.loads(cmd_str)
    except json.JSONDecodeError as e:
        config_response = json.dumps({'ok': False, 'error': f'Invalid JSON: {e}'})
        config_response_pages = []
        return

    op = cmd.get('op', '')
    request_id = cmd.get('request_id')
    print(f'Config command: {op}', flush=True)

    try:
        if op == 'WIFI_SET':
            _wifi_set_from_ble(cmd)
        elif op == 'WIFI_STATUS':
            _wifi_status_from_ble(cmd)
        elif op == 'GET_BOX_CONFIG':
            _cmd_get_box_config(request_id=request_id)
        elif op == 'SET_BOX_MODE':
            _cmd_set_box_mode(cmd, request_id=request_id)
        elif op == 'SET_DISPLAY_NAME':
            _cmd_set_display_name(cmd, request_id=request_id)
        elif op == 'SET_MOPEKA_ENABLED':
            _cmd_set_mopeka_enabled(cmd, request_id=request_id)
        elif op == 'GET_BMS':
            _cmd_get_bms(request_id=request_id)
        elif op == 'SET_BMS_MAC':
            _cmd_set_bms_mac(cmd, request_id=request_id)
        elif op == 'GET_TRAILER':
            _cmd_get_trailer(request_id=request_id)
        elif op == 'SELECT_TRAILER':
            _cmd_select_trailer(cmd, request_id=request_id)
        elif op == 'LIST_TRAILERS':
            _cmd_list_trailers(request_id=request_id)
        elif op == 'LIST_SENSORS':
            _cmd_list_sensors(cmd, request_id=request_id)
        elif op == 'ADD_SENSOR':
            _cmd_add_sensor(cmd, request_id=request_id)
        elif op == 'UPDATE_SENSOR':
            _cmd_update_sensor(cmd, request_id=request_id)
        elif op == 'DELETE_SENSOR':
            _cmd_delete_sensor(cmd, request_id=request_id)
        elif op == 'LIST_CALIBRATION':
            _cmd_list_calibration(cmd, request_id=request_id)
        elif op == 'ADD_CALIBRATION':
            _cmd_add_calibration(cmd, request_id=request_id)
        elif op == 'UPDATE_CALIBRATION':
            _cmd_update_calibration(cmd, request_id=request_id)
        elif op == 'DELETE_CALIBRATION':
            _cmd_delete_calibration(cmd, request_id=request_id)
        elif op == 'PAGE':
            _cmd_page(cmd, request_id=request_id)
        else:
            _set_config_response_obj({'ok': False, 'error': f'Unknown op: {op}', 'request_id': request_id, 'op': op})
            config_response_pages = []
    except Exception as e:
        print(f'Config command error: {e}', flush=True)
        _set_config_response_obj({'ok': False, 'error': str(e), 'request_id': request_id, 'op': op})
        config_response_pages = []


def _current_trailer_info():
    cfg = load_config()
    mode = _normalize_box_mode(cfg.get('box_mode'))
    trailer_num = _get_assigned_trailer(cfg)
    if mode != 'fleet':
        return {'box_mode': mode, 'trailer': None, 'enabled': False}
    if trailer_num is None:
        return {'box_mode': mode, 'trailer': None, 'enabled': True}

    sensors = load_sensor_csv()
    trailer_sensors = [s for s in sensors if str(s.get('Trailer')) == str(trailer_num)]
    front = next((s for s in trailer_sensors if s.get('Tank') == 'Front'), None)
    back = next((s for s in trailer_sensors if s.get('Tank') == 'Back'), None)
    man = trailer_sensors[0].get('Man', '') if trailer_sensors else ''

    def get_offset(sensor):
        if sensor and sensor.get('Height Offset'):
            try:
                return float(sensor['Height Offset'])
            except ValueError:
                pass
        return 0.0

    return {
        'box_mode': mode,
        'enabled': True,
        'trailer': trailer_num,
        'man': man,
        'front': {
            'id': front['Mopeka ID'] if front else '---------------',
            'offset': get_offset(front)
        },
        'back': {
            'id': back['Mopeka ID'] if back else '---------------',
            'offset': get_offset(back)
        }
    }


def _cmd_get_trailer(*, request_id=None):
    global config_response_pages
    config_response_pages = []
    payload = {'ok': True, 'op': 'GET_TRAILER', 'current': _current_trailer_info()}
    if request_id is not None:
        payload['request_id'] = request_id
    _set_config_response_obj(payload)


def _cmd_get_box_config(*, request_id=None):
    global config_response_pages
    config_response_pages = []
    payload = {'ok': True, 'op': 'GET_BOX_CONFIG', 'box': _current_box_config()}
    if request_id is not None:
        payload['request_id'] = request_id
    _set_config_response_obj(payload)


def _cmd_set_box_mode(cmd, *, request_id=None):
    global config_response_pages
    mode = _normalize_box_mode(cmd.get('mode'))
    cfg = load_config()
    cfg['box_mode'] = mode
    if mode == 'customer':
        cfg['assigned_trailer'] = None
        cfg['trailer'] = None
        if not str(cfg.get('display_name') or '').strip():
            cfg['display_name'] = DEFAULT_CUSTOMER_BLE_NAME
    save_config(cfg)
    config_response_pages = []
    _set_config_response_obj({
        'ok': True,
        'op': 'SET_BOX_MODE',
        'request_id': request_id,
        'box': _current_box_config(),
    })
    _schedule_identity_restart(f'Box mode changed to {mode}; restarting to refresh BLE identity')


def _cmd_set_display_name(cmd, *, request_id=None):
    global config_response_pages
    display_name = str(cmd.get('display_name') or '').strip()
    config_response_pages = []
    if not display_name:
        _set_config_response_obj({
            'ok': False,
            'op': 'SET_DISPLAY_NAME',
            'request_id': request_id,
            'error': 'display_name required',
        })
        return

    cfg = load_config()
    cfg['display_name'] = display_name
    save_config(cfg)
    _set_config_response_obj({
        'ok': True,
        'op': 'SET_DISPLAY_NAME',
        'request_id': request_id,
        'box': _current_box_config(),
    })
    _schedule_identity_restart('Display name changed; restarting to refresh BLE identity')


def _cmd_set_mopeka_enabled(cmd, *, request_id=None):
    global config_response_pages
    cfg = load_config()
    cfg['mopeka_enabled'] = bool(cmd.get('enabled'))
    save_config(cfg)
    config_response_pages = []
    _set_config_response_obj({
        'ok': True,
        'op': 'SET_MOPEKA_ENABLED',
        'request_id': request_id,
        'box': _current_box_config(),
    })


def _cmd_get_bms(*, request_id=None):
    global config_response_pages
    config_response_pages = []
    payload = {
        'ok': True,
        'op': 'GET_BMS',
        'bms': {
            'mac': BMS_MAC,
            'name': _compute_bms_name(),
            'enabled': bool(BMS_ENABLED),
        },
    }
    if request_id is not None:
        payload['request_id'] = request_id
    _set_config_response_obj(payload)


def _cmd_set_bms_mac(cmd, *, request_id=None):
    global config_response_pages, BMS_MAC
    raw_mac = cmd.get('mac')
    mac = _normalize_ble_mac(raw_mac)
    config_response_pages = []

    if not mac:
        _set_config_response_obj({
            'ok': False,
            'op': 'SET_BMS_MAC',
            'request_id': request_id,
            'error': 'Invalid MAC address',
        })
        return

    BMS_MAC = mac
    cfg = load_config()
    cfg['bms_mac'] = mac
    save_config(cfg)
    _set_config_response_obj({
        'ok': True,
        'op': 'SET_BMS_MAC',
        'request_id': request_id,
        'bms': {
            'mac': BMS_MAC,
            'name': _compute_bms_name(),
            'enabled': bool(BMS_ENABLED),
        },
    })


def _cmd_select_trailer(cmd, *, request_id=None):
    global config_response_pages
    if not _box_mode_uses_trailer_list():
        config_response_pages = []
        _set_config_response_obj({
            'ok': False,
            'op': 'SELECT_TRAILER',
            'request_id': request_id,
            'error': 'Trailer selection disabled in customer mode',
        })
        return

    trailer_num = cmd.get('trailer')
    try:
        trailer_num = int(trailer_num)
    except Exception:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'SELECT_TRAILER', 'request_id': request_id, 'error': 'Invalid trailer'})
        return

    result = apply_trailer(trailer_num)
    config_response_pages = []
    if result is None:
        _set_config_response_obj({'ok': False, 'op': 'SELECT_TRAILER', 'request_id': request_id, 'error': f'Trailer {trailer_num} not found'})
        return

    _set_config_response_obj({'ok': True, 'op': 'SELECT_TRAILER', 'request_id': request_id, 'current': result})
    _schedule_identity_restart(f'Trailer changed to {trailer_num}; restarting to refresh BLE identity')


def _cmd_list_trailers(*, request_id=None):
    global config_response, config_response_pages
    if not _box_mode_uses_trailer_list():
        config_response_pages = []
        _set_config_response_obj({
            'ok': False,
            'op': 'LIST_TRAILERS',
            'request_id': request_id,
            'error': 'Trailer list disabled in customer mode',
        })
        return

    sensors = load_sensor_csv()

    trailers = {}
    for s in sensors:
        t = s.get('Trailer', '')
        if t not in trailers:
            trailers[t] = {'trailer': int(t) if t.isdigit() else t, 'man': s.get('Man', '')}
        tank = s.get('Tank', '')
        mid = s.get('Mopeka ID', '')
        if tank == 'Front':
            trailers[t]['front'] = mid
        elif tank == 'Back':
            trailers[t]['back'] = mid

    items = sorted(trailers.values(), key=lambda x: x.get('trailer', 0))
    _set_paginated_config_response(items, request_id=request_id, op='LIST_TRAILERS')


def _cmd_list_sensors(cmd, *, request_id=None):
    global config_response, config_response_pages
    sensors = load_sensor_csv()

    trailer_filter = cmd.get('trailer')
    if trailer_filter is not None:
        sensors = [s for s in sensors if str(s.get('Trailer')) == str(trailer_filter)]

    items = []
    for s in sensors:
        item = {
            'man': s.get('Man', ''),
            'trailer': int(s['Trailer']) if s.get('Trailer', '').isdigit() else s.get('Trailer', ''),
            'tank': s.get('Tank', ''),
            'id': s.get('Mopeka ID', ''),
            'offset': s.get('Height Offset', ''),
            'name': s.get('Mopeka Name in app', ''),
        }
        items.append(item)

    _set_paginated_config_response(items, request_id=request_id, op='LIST_SENSORS')


def _cmd_add_sensor(cmd, *, request_id=None):
    global config_response, config_response_pages
    data = cmd.get('data')
    if not data:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'ADD_SENSOR', 'request_id': request_id, 'error': 'Missing data field'})
        return

    sensors = load_sensor_csv()

    new_sensor = {}
    field_map = {
        'man': 'Man', 'trailer': 'Trailer', 'tank': 'Tank',
        'center_sump': 'Center Sump?', 'height_offset': 'Height Offset',
        'name': 'Mopeka Name in app', 'id': 'Mopeka ID',
        'mqtt_topic': 'MQTT Topic for app', 'added_to_app': 'Added to app'
    }
    for json_key, csv_key in field_map.items():
        if json_key in data:
            new_sensor[csv_key] = str(data[json_key])

    if 'Mopeka ID' not in new_sensor or 'Trailer' not in new_sensor or 'Tank' not in new_sensor:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'ADD_SENSOR', 'request_id': request_id, 'error': 'Required: id, trailer, tank'})
        return

    sensors.append(new_sensor)
    tank_order = {'Front': 0, 'Back': 1}
    sensors.sort(key=lambda s: (int(s['Trailer']) if s.get('Trailer', '').isdigit() else 999,
                                tank_order.get(s.get('Tank', ''), 2)))
    save_sensor_csv(sensors)
    _reload_converter()

    config_response_pages = []
    _set_config_response_obj({'ok': True, 'op': 'ADD_SENSOR', 'request_id': request_id, 'id': new_sensor.get('Mopeka ID', '')})


def _cmd_update_sensor(cmd, *, request_id=None):
    global config_response, config_response_pages
    sensor_id = cmd.get('id')
    data = cmd.get('data')
    if not sensor_id or not data:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'UPDATE_SENSOR', 'request_id': request_id, 'error': 'Required: id, data'})
        return

    sensors = load_sensor_csv()
    found = False
    field_map = {
        'man': 'Man', 'trailer': 'Trailer', 'tank': 'Tank',
        'center_sump': 'Center Sump?', 'height_offset': 'Height Offset',
        'name': 'Mopeka Name in app', 'id': 'Mopeka ID',
        'mqtt_topic': 'MQTT Topic for app', 'added_to_app': 'Added to app'
    }
    for s in sensors:
        if s.get('Mopeka ID') == sensor_id:
            for json_key, csv_key in field_map.items():
                if json_key in data:
                    s[csv_key] = str(data[json_key])
            found = True
            break

    if not found:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'UPDATE_SENSOR', 'request_id': request_id, 'error': f'Sensor {sensor_id} not found'})
        return

    save_sensor_csv(sensors)
    _reload_converter()

    config_response_pages = []
    _set_config_response_obj({'ok': True, 'op': 'UPDATE_SENSOR', 'request_id': request_id, 'id': sensor_id})


def _cmd_delete_sensor(cmd, *, request_id=None):
    global config_response, config_response_pages
    sensor_id = cmd.get('id')
    if not sensor_id:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'DELETE_SENSOR', 'request_id': request_id, 'error': 'Required: id'})
        return

    sensors = load_sensor_csv()
    original_len = len(sensors)
    sensors = [s for s in sensors if s.get('Mopeka ID') != sensor_id]

    if len(sensors) == original_len:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'DELETE_SENSOR', 'request_id': request_id, 'error': f'Sensor {sensor_id} not found'})
        return

    save_sensor_csv(sensors)
    _reload_converter()

    config_response_pages = []
    _set_config_response_obj({'ok': True, 'op': 'DELETE_SENSOR', 'request_id': request_id, 'id': sensor_id})


def _cmd_list_calibration(cmd=None, *, request_id=None):
    global config_response, config_response_pages
    path, profile = _calibration_csv_path_for_cmd(cmd or {})
    points = load_calibration_csv(path)

    items = []
    for i, p in enumerate(points):
        items.append({
            'index': i,
            'tank_level_in': p['tank_level_in'],
            'gallons': p['gallons'],
            'tank_size': p['tank_size']
        })

    _set_paginated_config_response(items, request_id=request_id, op='LIST_CALIBRATION')


def _cmd_add_calibration(cmd, *, request_id=None):
    global config_response, config_response_pages
    data = cmd.get('data')
    if not data:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'ADD_CALIBRATION', 'request_id': request_id, 'error': 'Missing data field'})
        return

    if 'tank_level_in' not in data or 'gallons' not in data:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'ADD_CALIBRATION', 'request_id': request_id, 'error': 'Required: tank_level_in, gallons'})
        return

    path, profile = _calibration_csv_path_for_cmd(cmd)
    points = load_calibration_csv(path)
    new_point = {
        'tank_level_in': float(data['tank_level_in']),
        'gallons': float(data['gallons']),
        'tank_size': float(data.get('tank_size', 1070.0))
    }
    points.append(new_point)
    save_calibration_csv(points, path)
    _reload_converter()

    config_response_pages = []
    _set_config_response_obj({
        'ok': True,
        'op': 'ADD_CALIBRATION',
        'request_id': request_id,
        'profile': profile or None,
    })


def _cmd_update_calibration(cmd, *, request_id=None):
    global config_response, config_response_pages
    index = cmd.get('index')
    data = cmd.get('data')
    if index is None or not data:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'UPDATE_CALIBRATION', 'request_id': request_id, 'error': 'Required: index, data'})
        return

    path, profile = _calibration_csv_path_for_cmd(cmd)
    points = load_calibration_csv(path)
    if index < 0 or index >= len(points):
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'UPDATE_CALIBRATION', 'request_id': request_id, 'error': f'Index {index} out of range (0-{len(points)-1})'})
        return

    for key in ('tank_level_in', 'gallons', 'tank_size'):
        if key in data:
            points[index][key] = float(data[key])

    save_calibration_csv(points, path)
    _reload_converter()

    config_response_pages = []
    _set_config_response_obj({
        'ok': True,
        'op': 'UPDATE_CALIBRATION',
        'request_id': request_id,
        'index': index,
        'profile': profile or None,
    })


def _cmd_delete_calibration(cmd, *, request_id=None):
    global config_response, config_response_pages
    index = cmd.get('index')
    if index is None:
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'DELETE_CALIBRATION', 'request_id': request_id, 'error': 'Required: index'})
        return

    path, profile = _calibration_csv_path_for_cmd(cmd)
    points = load_calibration_csv(path)
    if index < 0 or index >= len(points):
        config_response_pages = []
        _set_config_response_obj({'ok': False, 'op': 'DELETE_CALIBRATION', 'request_id': request_id, 'error': f'Index {index} out of range (0-{len(points)-1})'})
        return

    points.pop(index)
    save_calibration_csv(points, path)
    _reload_converter()

    config_response_pages = []
    _set_config_response_obj({
        'ok': True,
        'op': 'DELETE_CALIBRATION',
        'request_id': request_id,
        'index': index,
        'profile': profile or None,
    })


def _cmd_page(cmd, *, request_id=None):
    global config_response
    page = cmd.get('page', 1)
    if not config_response_pages:
        _set_config_response_obj({'ok': False, 'op': 'PAGE', 'request_id': request_id, 'error': 'No paginated data available'})
        return

    if page < 1 or page > len(config_response_pages):
        _set_config_response_obj({'ok': False, 'op': 'PAGE', 'request_id': request_id, 'error': f'Page {page} out of range (1-{len(config_response_pages)})'})
        return

    page_obj = json.loads(config_response_pages[page - 1])
    if request_id is not None:
        page_obj['request_id'] = request_id
    _set_config_response_obj(page_obj)


def _reload_converter():
    """Reload mopeka_converter after CSV changes."""
    try:
        mopeka_reload()
    except Exception as e:
        print(f'mopeka_converter reload: {e}', flush=True)


# =============================================================================
# BLE Characteristic Handlers for Trailer Config
# =============================================================================

def trailer_read_handler(connection):
    """Read current trailer config as JSON."""
    info = _current_trailer_info()
    value = json.dumps(info, separators=(',', ':'))
    print(f'ReadValue trailer: {value}', flush=True)
    return value.encode('utf-8')


def trailer_write_handler(connection, value):
    """Write trailer number to select and configure sensors for that trailer."""
    if not _box_mode_uses_trailer_list():
        print('Trailer write rejected: customer mode', flush=True)
        return

    trailer_str = value.decode('utf-8').strip()
    print(f'Trailer write: {trailer_str}', flush=True)
    try:
        trailer_num = int(trailer_str)
    except ValueError:
        print(f'Invalid trailer number: {trailer_str}', flush=True)
        return

    result = apply_trailer(trailer_num)
    if result is None:
        print(f'Trailer {trailer_num} not found in sensor CSV', flush=True)
        return

    _schedule_identity_restart(f'Trailer write changed to {trailer_num}; restarting to refresh BLE identity')
    return


def config_cmd_write_handler(connection, value):
    """Handle config command writes. Supports chunked writes via CHUNK:X/Y:data pattern."""
    global config_cmd_chunks

    try:
        data_str = value.decode('utf-8').strip()
        connection_key = _connection_key(connection)

        if data_str.startswith('CHUNK:'):
            parts = data_str.split(':', 2)
            if len(parts) >= 3:
                chunk_info = parts[1]
                chunk_data = parts[2]
                chunk_num, total_chunks = map(int, chunk_info.split('/'))

                print(f'ConfigCmd chunk {chunk_num}/{total_chunks} ({len(chunk_data)} bytes)', flush=True)

                buffer = config_cmd_chunks.get(connection_key)
                if not buffer or buffer.get('total') != total_chunks:
                    buffer = {
                        'chunks': {},
                        'total': total_chunks,
                        'timestamp': time.time()
                    }
                    config_cmd_chunks[connection_key] = buffer

                buffer['chunks'][chunk_num] = chunk_data
                buffer['timestamp'] = time.time()

                if len(buffer['chunks']) == total_chunks:
                    assembled = ''
                    for i in range(1, total_chunks + 1):
                        if i in buffer['chunks']:
                            assembled += buffer['chunks'][i]
                        else:
                            print(f'ConfigCmd: missing chunk {i}!', flush=True)
                            return
                    config_cmd_chunks.pop(connection_key, None)
                    process_config_command(assembled)
        else:
            process_config_command(data_str)

    except Exception as e:
        print(f'ConfigCmd error: {e}', flush=True)
    return


def config_data_read_handler(connection):
    """Read the response from the last config command."""
    value = config_response
    print(f'ReadValue config_data: {value[:80]}...', flush=True)
    return value.encode('utf-8')


def decode_mopeka(data):
    temp_raw = data[2] & 0x7F
    tank_raw = data[3] | ((data[4] & 0x3F) << 8)
    quality = (data[4] >> 6) & 0x03
    # Use the air coefficients from the reference parser so empty spray tanks
    # decode to the physical tank height instead of propane-liquid depth.
    level_mm = tank_raw * (0.153096 + 0.000327 * temp_raw - 0.000000294 * temp_raw * temp_raw)
    return {'level_mm': round(level_mm, 1), 'quality': quality}

async def scan_mopeka(sensor_device, current_time):
    mopeka_found = False

    def on_advertisement(advertisement):
        nonlocal mopeka_found

        try:
            addr = str(advertisement.address).upper()
            manufacturer_data = advertisement.data.get_all(
                AdvertisingData.MANUFACTURER_SPECIFIC_DATA
            )

            for company_id, data in manufacturer_data:
                if company_id != 89:
                    continue

                decoded = decode_mopeka(data)
                decoded['last_update'] = current_time

                if MOPEKA1_MAC_SUFFIX and MOPEKA1_MAC_SUFFIX in addr:
                    conversion = mm_to_gallons(decoded["level_mm"], MOPEKA1_MAC_SUFFIX)
                    decoded.update(conversion)
                    sensor_data['mopeka1'] = decoded
                    print(f'Mopeka1: {decoded}', flush=True)
                    mopeka_found = True
                elif MOPEKA2_MAC_SUFFIX and MOPEKA2_MAC_SUFFIX in addr:
                    conversion = mm_to_gallons(decoded["level_mm"], MOPEKA2_MAC_SUFFIX)
                    decoded.update(conversion)
                    sensor_data['mopeka2'] = decoded
                    print(f'Mopeka2: {decoded}', flush=True)
                    mopeka_found = True
        except Exception as e:
            print(f'Mopeka advertisement parse error: {e}', flush=True)

    sensor_device.on('advertisement', on_advertisement)
    try:
        await sensor_device.start_scanning(active=False)
        await asyncio.sleep(SCAN_TIMEOUT)
        await sensor_device.stop_scanning()
    finally:
        sensor_device.remove_listener('advertisement', on_advertisement)

    return mopeka_found

async def read_bms(sensor_device, current_time):
    response_buffer = bytearray()
    notification_received = asyncio.Event()
    connection = None
    peer = None
    notify_char = None
    write_char = None
    subscriber = None

    try:
        connect_errors = []
        for target in (BMS_MAC, BMS_NAME):
            try:
                connection = await sensor_device.connect(
                    target,
                    own_address_type=hci.OwnAddressType.RANDOM,
                    timeout=BMS_TIMEOUT,
                )
                print(f'BMS connected via {target}', flush=True)
                break
            except Exception as e:
                connect_errors.append(f'{target}: {type(e).__name__} {e!r}')

        if connection is None:
            raise TimeoutError('; '.join(connect_errors))

        peer = Peer(connection)
        await peer.discover_services()
        notify_chars = await peer.discover_characteristics(uuids=[BMS_NOTIFY_UUID])
        write_chars = await peer.discover_characteristics(uuids=[BMS_WRITE_UUID])

        if not notify_chars or not write_chars:
            raise RuntimeError('BMS characteristics not found')

        notify_char = notify_chars[0]
        write_char = write_chars[0]

        def on_bms_notification(value):
            response_buffer.extend(value)
            notification_received.set()

        subscriber = on_bms_notification
        await peer.subscribe(notify_char, subscriber)
        # Match the older working flow: wake/prime with cell-info, then request hw-info.
        await peer.write_value(write_char, jbd_cmd(0x04), with_response=False)
        await asyncio.sleep(1.0)
        notification_received.clear()
        response_buffer.clear()
        await peer.write_value(write_char, jbd_cmd(0x03), with_response=False)

        hwinfo_frame = None
        deadline = time.time() + 5.0
        while time.time() < deadline:
            await asyncio.wait_for(notification_received.wait(), timeout=max(0.1, deadline - time.time()))
            notification_received.clear()
            hwinfo_frame = _extract_jbd_frame(response_buffer, expected_function=0x03)
            if hwinfo_frame is not None:
                break

        if hwinfo_frame is None:
            raise RuntimeError('Timed out waiting for complete BMS hardware-info frame')

        payload_len = hwinfo_frame[3]
        d = hwinfo_frame[4:4 + payload_len]
        if len(d) < 23:
            raise RuntimeError(f'BMS hardware-info payload too short ({len(d)} bytes)')

        sensor_data['bms'] = {
            'voltage': round(int.from_bytes(d[0:2], 'big') * 0.01, 2),
            'soc': d[19],
            'last_update': current_time
        }
        print(f'BMS: {sensor_data["bms"]}', flush=True)
        return True
    finally:
        if peer and notify_char and subscriber:
            try:
                await peer.unsubscribe(notify_char, subscriber)
            except Exception:
                pass
        if connection:
            try:
                await connection.disconnect()
            except Exception:
                pass

def jbd_cmd(func):
    frame = [0xDD, 0xA5, func, 0x00]
    crc = sum([-b for b in frame[2:4]]) & 0xFFFF
    return bytes(frame + [crc >> 8, crc & 0xFF, 0x77])

async def poll_dashboard_status(device, state_char):
    """Periodically poll dashboard state and notify subscribers on change."""
    last_history_poll = 0.0
    last_notified_state_json = dashboard_status.get('state_json', '{}')
    while True:
        sleep_interval = STATUS_POLL_INTERVAL_IDLE
        try:
            updated = query_dashboard_status()
            current_state_json = dashboard_status.get('state_json', '{}')
            sleep_interval = dashboard_poll_interval(dashboard_status.get('state'))
            if updated and current_state_json != last_notified_state_json:
                await device.notify_subscribers(
                    state_char,
                    bytes(current_state_json, 'utf-8'),
                )
                last_notified_state_json = current_state_json

            now = time.time()
            if now - last_history_poll >= HISTORY_POLL_INTERVAL:
                query_fill_history()
                last_history_poll = now
        except Exception as e:
            print(f'Status poll error: {e}', flush=True)
        await asyncio.sleep(sleep_interval)

async def read_sensors(sensor_device, sensor_adapter):
    global sensor_data, sensor_loop_heartbeat

    cfg = load_config()
    mopeka_enabled = _mopeka_enabled(cfg)

    print(f"Sensor reader started on {sensor_adapter}", flush=True)
    if BMS_ENABLED:
        print(f"Scan interval: {SCAN_INTERVAL}s, BMS every {BMS_READ_INTERVAL} cycles", flush=True)
    else:
        print(f"Scan interval: {SCAN_INTERVAL}s, BMS polling disabled", flush=True)
    print(f"Mopeka polling {'enabled' if mopeka_enabled else 'disabled'}", flush=True)
    print(f"Auto-recovery after {MAX_CONSECUTIVE_FAILURES} failures", flush=True)

    cycle_count = 0
    mopeka_failures = 0
    bms_failures = 0
    last_adapter_reset = 0
    mopeka_disabled_announced = False

    while True:
        sensor_loop_heartbeat = time.time()
        cycle_count += 1
        current_time = time.time()
        reload_converter_if_calibration_changed()

        if ((mopeka_enabled and mopeka_failures >= MAX_CONSECUTIVE_FAILURES) or bms_failures >= MAX_CONSECUTIVE_FAILURES):
            if current_time - last_adapter_reset > ADAPTER_RESET_COOLDOWN:
                print('Too many sensor failures, exiting for service restart', flush=True)
                os._exit(1)

        if mopeka_enabled:
            try:
                mopeka_found = await scan_mopeka(sensor_device, current_time)

                if mopeka_found:
                    mopeka_failures = 0
                    m1 = sensor_data["mopeka1"]
                    m2 = sensor_data["mopeka2"]
                    m1_gal = m1.get("gallons", 0)
                    m2_gal = m2.get("gallons", 0)
                    m1_q = m1.get("quality", 0)
                    m2_q = m2.get("quality", 0)
                    m1_mm = m1.get("level_mm", 0)
                    m2_mm = m2.get("level_mm", 0)
                    m1_in = m1.get("level_in", 0)
                    m2_in = m2.get("level_in", 0)
                    send_dashboard_command(f"MOPEKA:{m1_gal:.0f}|{m2_gal:.0f}|{m1_q}|{m2_q}")
                    send_dashboard_command(f"MOPEKA_RAW:{m1_mm:.1f}|{m2_mm:.1f}|{m1_in:.2f}|{m2_in:.2f}")
                else:
                    mopeka_failures += 1
                    if mopeka_failures > 1:
                        print(f'Mopeka not found ({mopeka_failures}/{MAX_CONSECUTIVE_FAILURES})', flush=True)
                        send_dashboard_command('MOPEKA_OFFLINE')

            except Exception as e:
                mopeka_failures += 1
                print(f'Mopeka error ({mopeka_failures}/{MAX_CONSECUTIVE_FAILURES}): {e}', flush=True)
        else:
            mopeka_failures = 0
            if not mopeka_disabled_announced:
                send_dashboard_command('MOPEKA_DISABLED')
                mopeka_disabled_announced = True

        if BMS_ENABLED and (cycle_count == 1 or cycle_count % BMS_READ_INTERVAL == 0):
            try:
                if await read_bms(sensor_device, current_time):
                    bms_failures = 0
                    bms = sensor_data["bms"]
                    send_dashboard_command(f"BMS:{bms.get('soc', 0)}|{bms.get('voltage', 0):.2f}")
            except Exception as e:
                bms_failures += 1
                print(
                    f'BMS error ({bms_failures}/{MAX_CONSECUTIVE_FAILURES}) '
                    f'{type(e).__name__}: {e!r}',
                    flush=True,
                )

        await asyncio.sleep(SCAN_INTERVAL)


def _handle_sensor_task_done(task):
    try:
        exc = task.exception()
    except asyncio.CancelledError:
        log_watchdog_event('Sensor task was cancelled; exiting for restart')
        os._exit(1)
    except Exception as e:
        log_watchdog_event(f'Failed to inspect sensor task completion: {type(e).__name__}: {e}')
        os._exit(1)

    if exc is not None:
        log_watchdog_event(f'Sensor task crashed: {type(exc).__name__}: {exc}')
        os._exit(1)

    log_watchdog_event('Sensor task exited unexpectedly; exiting for restart')
    os._exit(1)


async def monitor_sensor_health(sensor_task):
    global sensor_loop_heartbeat

    while True:
        await asyncio.sleep(10)

        if sensor_task.done():
            _handle_sensor_task_done(sensor_task)
            return

        if sensor_loop_heartbeat == 0:
            continue

        stale_for = time.time() - sensor_loop_heartbeat
        if stale_for > SENSOR_LOOP_HEARTBEAT_TIMEOUT:
            log_watchdog_event(
                f'Sensor loop heartbeat stale for {stale_for:.0f}s; exiting for restart'
            )
            os._exit(1)

async def main():
    global ble_device, config_notify_char, maintenance_stdout_char
    print('Starting Rotorsync GATT server (Bumble)...', flush=True)
    # Initialize Mopeka gallon converter
    mopeka_init()
    reload_converter_if_calibration_changed(force=True)

    # Restore adapter config from last session
    restore_adapter_config()

    # Restore BMS config from last session
    restore_bms_config()

    # Restore trailer config from last session
    restore_trailer_config()

    # Find adapters by known USB chip role first, then saved MAC fallback.
    gatt_adapter, sensor_adapter = select_runtime_adapters()

    if not gatt_adapter:
        print(f'ERROR: GATT adapter {GATT_ADAPTER_MAC} not found!', flush=True)
        return
    if not sensor_adapter:
        print(f'WARNING: Sensor adapter {SENSOR_ADAPTER_MAC} not found - sensors disabled', flush=True)

    gatt_adapter_index = int(gatt_adapter.replace('hci', ''))
    sensor_adapter_index = int(sensor_adapter.replace('hci', '')) if sensor_adapter else None
    persist_adapter_config()
    gatt_device_path = get_adapter_device_path(gatt_adapter)
    persist_gatt_device_path(gatt_device_path)
    print(f'GATT adapter: {gatt_adapter} ({GATT_ADAPTER_MAC})', flush=True)
    if sensor_adapter:
        print(f'Sensor adapter: {sensor_adapter} ({SENSOR_ADAPTER_MAC})', flush=True)

    enforce_bumble_only_stack()
    await asyncio.sleep(0.5)

    sensor_device = None
    if sensor_adapter and sensor_adapter_index is not None:
        print(f'Opening HCI socket for {sensor_adapter}...', flush=True)
        try:
            sensor_transport = await asyncio.wait_for(
                open_hci_socket_transport(sensor_adapter_index),
                timeout=SENSOR_ADAPTER_OPEN_TIMEOUT,
            )
            sensor_host = Host(sensor_transport.source, sensor_transport.sink)
            sensor_device = Device(name='TrailerSync-Sensors', host=sensor_host)
            await sensor_device.power_on()
        except Exception as e:
            print(
                f'WARNING: Sensor adapter {sensor_adapter} open failed; sensors disabled: {e!r}',
                flush=True,
            )
            sensor_adapter = None

    if sensor_adapter:
        sensor_task = asyncio.create_task(read_sensors(sensor_device, sensor_adapter))
        sensor_task.add_done_callback(_handle_sensor_task_done)
        asyncio.create_task(monitor_sensor_health(sensor_task))
    gatt_monitor_task = asyncio.create_task(
        monitor_gatt_adapter(gatt_adapter, gatt_device_path)
    )

    print(f'Opening HCI socket for {gatt_adapter}...', flush=True)
    hci_transport = await open_hci_socket_transport(gatt_adapter_index)

    host = Host(hci_transport.source, hci_transport.sink)
    ble_name = _compute_ble_name()
    device = Device(name=ble_name, host=host)
    ble_device = device

    # Create characteristics - READ for sensors
    bms_char = Characteristic(BMS_CHAR_UUID, Characteristic.Properties.READ,
        Characteristic.READABLE, CharacteristicValue(read=make_read_handler('bms')))
    mopeka1_char = Characteristic(MOPEKA1_CHAR_UUID, Characteristic.Properties.READ,
        Characteristic.READABLE, CharacteristicValue(read=make_read_handler('mopeka1')))
    mopeka2_char = Characteristic(MOPEKA2_CHAR_UUID, Characteristic.Properties.READ,
        Characteristic.READABLE, CharacteristicValue(read=make_read_handler('mopeka2')))

    # Create characteristics - WRITE for controls
    pump_char = Characteristic(PUMP_CHAR_UUID,
        Characteristic.Properties.WRITE | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
        Characteristic.WRITEABLE, CharacteristicValue(write=pump_write_handler))
    gallons_char = Characteristic(GALLONS_CHAR_UUID,
        Characteristic.Properties.WRITE | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
        Characteristic.WRITEABLE, CharacteristicValue(write=gallons_write_handler))

    # Create characteristics - READ for dashboard status
    requested_char = Characteristic(REQUESTED_CHAR_UUID, Characteristic.Properties.READ,
        Characteristic.READABLE, CharacteristicValue(read=make_dashboard_read_handler('requested')))
    actual_char = Characteristic(ACTUAL_CHAR_UUID, Characteristic.Properties.READ,
        Characteristic.READABLE, CharacteristicValue(read=make_dashboard_read_handler('actual')))
    state_char = Characteristic(
        STATE_CHAR_UUID,
        Characteristic.Properties.READ | Characteristic.Properties.NOTIFY,
        Characteristic.READABLE,
        CharacteristicValue(read=make_state_read_handler()),
    )
    command_char = Characteristic(
        COMMAND_CHAR_UUID,
        Characteristic.Properties.WRITE | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
        Characteristic.WRITEABLE,
        CharacteristicValue(write=command_write_handler),
    )

    history_char = Characteristic(HISTORY_CHAR_UUID, Characteristic.Properties.READ,
        Characteristic.READABLE, CharacteristicValue(read=make_history_read_handler()))

    # Create characteristic - WRITE for batch mix data from iPad
    batchmix_char = Characteristic(BATCHMIX_CHAR_UUID,
        Characteristic.Properties.WRITE | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
        Characteristic.WRITEABLE, CharacteristicValue(write=batchmix_write_handler))

    # Create characteristics - Trailer config
    trailer_char = Characteristic(TRAILER_CHAR_UUID,
        Characteristic.Properties.READ | Characteristic.Properties.WRITE | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
        Characteristic.READABLE | Characteristic.WRITEABLE,
        CharacteristicValue(read=trailer_read_handler, write=trailer_write_handler))

    config_cmd_char = Characteristic(CONFIG_CMD_CHAR_UUID,
        Characteristic.Properties.WRITE | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
        Characteristic.WRITEABLE, CharacteristicValue(write=config_cmd_write_handler))

    config_data_char = Characteristic(CONFIG_DATA_CHAR_UUID,
        Characteristic.Properties.READ,
        Characteristic.READABLE, CharacteristicValue(read=config_data_read_handler))
    config_notify_char = Characteristic(
        CONFIG_NOTIFY_CHAR_UUID,
        Characteristic.Properties.READ | Characteristic.Properties.NOTIFY,
        Characteristic.READABLE,
        CharacteristicValue(read=make_config_notify_read_handler()),
    )
    maint_control_char = Characteristic(
        MAINT_CONTROL_CHAR_UUID,
        Characteristic.Properties.WRITE | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
        Characteristic.WRITEABLE,
        CharacteristicValue(write=maintenance_control_write_handler),
    )
    maint_stdin_char = Characteristic(
        MAINT_STDIN_CHAR_UUID,
        Characteristic.Properties.WRITE | Characteristic.Properties.WRITE_WITHOUT_RESPONSE,
        Characteristic.WRITEABLE,
        CharacteristicValue(write=maintenance_stdin_write_handler),
    )
    maintenance_stdout_char = Characteristic(
        MAINT_STDOUT_CHAR_UUID,
        Characteristic.Properties.READ | Characteristic.Properties.NOTIFY,
        Characteristic.READABLE,
        CharacteristicValue(read=make_maintenance_stdout_read_handler()),
    )

    service = Service(
        SERVICE_UUID,
        [
            bms_char,
            mopeka1_char,
            mopeka2_char,
            pump_char,
            gallons_char,
            requested_char,
            actual_char,
            state_char,
            command_char,
            history_char,
            batchmix_char,
            trailer_char,
            config_cmd_char,
            config_data_char,
            config_notify_char,
            maint_control_char,
            maint_stdin_char,
            maintenance_stdout_char,
        ],
    )
    device.add_service(service)
    asyncio.create_task(maintenance_agent_bridge_loop())
    if await wait_for_dashboard_ready():
        print('Dashboard socket is ready', flush=True)
    else:
        print(
            f'Dashboard socket not ready after {STARTUP_DASHBOARD_WAIT_SECONDS}s; '
            'continuing and relying on retry loop',
            flush=True,
        )
    status_task = asyncio.create_task(poll_dashboard_status(device, state_char))

    await device.power_on()
    print(f'Device address: {device.public_address}', flush=True)
    print(f'BLE name: {ble_name}', flush=True)

    adv_data = AdvertisingData([
        (AdvertisingData.COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, bytes(SERVICE_UUID)),
    ])
    scan_response = AdvertisingData([
        (AdvertisingData.COMPLETE_LOCAL_NAME, ble_name.encode('utf-8')),
    ])

    await device.start_advertising(
        advertising_data=bytes(adv_data),
        scan_response_data=bytes(scan_response),
        auto_restart=True,
    )
    print('=== Rotorsync GATT Server Running ===', flush=True)
    print('Characteristics:', flush=True)
    print('  def1: BMS (read)        - {"voltage": x, "soc": y}', flush=True)
    print('  def2: Mopeka1 (read)    - {"level_mm": x, "quality": y, "gallons": z}', flush=True)
    print('  def3: Mopeka2 (read)    - {"level_mm": x, "quality": y, "gallons": z}', flush=True)
    print('  def4: Pump (write)      - "PS" to stop pump', flush=True)
    print('  def5: Gallons (write)   - "+1", "-1", "+10", "-10"', flush=True)
    print('  def6: Requested (read)  - requested gallons', flush=True)
    print('  def7: Actual (read)     - actual gallons', flush=True)
    print('  defd: State (r/n)       - live dashboard JSON snapshot', flush=True)
    print('  defe: Command (write)   - JSON command channel for iPad app', flush=True)
    print('  def8: History (read)    - last 5 fills', flush=True)
    print('  def9: BatchMix (write)  - JSON batch mix data from iPad', flush=True)
    print('  defa: Trailer (r/w)     - fleet mode trailer selection/current trailer', flush=True)
    print('  defb: ConfigCmd (write) - JSON commands for sensor/calibration CRUD', flush=True)
    print('  defc: ConfigData (read) - response from last config command', flush=True)
    print('  deff: ConfigNotify(r/n) - last config response with request_id echo', flush=True)

    await asyncio.Event().wait()

if __name__ == '__main__':
    asyncio.run(main())
