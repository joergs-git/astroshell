#!/usr/bin/env python3
"""
==============================================================================
AstroShell Tick Logger & Event Server (v4.0)
==============================================================================
Receives motor runtime data and event notifications from Arduino dome
controller. Logs tick data to CSV, sends Pushover alerts for safety events.

Target Hardware: Cloudwatcher Solo (Raspberry Pi 3, armv6l)
Port: 88 (requires root)

v4.0 Changes:
- New /event endpoint for dome safety event notifications
- Pushover integration for frozen dome, sensor failures, signal conflicts
- Updated CSV format: now includes Arduino-reported temp and ToF distance
- Arduino sends temp and ToF in all /log and /interrupt requests

Arduino sends:
  GET /log?m=<motor>&d=<direction>&t=<ticks>&temp=<C>&tof=<cm>
  GET /interrupt?m=<motor>&d=<direction>&t=<ticks>&temp=<C>&tof=<cm>
  GET /event?type=<type>&detail=<detail>&temp=<C>&tof=<cm>

==============================================================================
FILE LOCATIONS
==============================================================================
Script (persistent):    /usr/local/bin/astroshell_ticklogger.py
Service (persistent):   /etc/systemd/system/astroshell_ticklogger.service
CSV output (tmpfs):     /home/aagsolo/motor_ticks.csv
Weather JSON (tmpfs):   /home/aagsolo/aag_json.dat (created by Solo)

Note: The Solo has a read-only root filesystem to protect the SD card.
      /home/aagsolo is a tmpfs (RAM disk) - data is lost on reboot.
      Periodic backup to Synology NAS via SCP preserves data.

Synology Backup (one-time SSH key setup on Solo as root):
      1. ssh-keygen -t rsa -N "" -f ~/.ssh/id_rsa
      2. ssh-copy-id solo@192.168.1.113
      (Step 1 MUST run before step 2, otherwise "No identities found" error)

==============================================================================
INSTALLATION (one-time setup)
==============================================================================
# 1. SSH to the Solo
ssh root@192.168.1.151

# 2. Make root filesystem writable (temporarily)
mount -o remount,rw /

# 3. Create the Python script
vi /usr/local/bin/astroshell_ticklogger.py
# (paste this entire file, save with :wq)
chmod +x /usr/local/bin/astroshell_ticklogger.py

# 4. Create the systemd service file
vi /etc/systemd/system/astroshell_ticklogger.service
# (paste the following content):
#
# [Unit]
# Description=AstroShell Tick Logger - Motor Runtime Data Collector
# After=network.target
#
# [Service]
# Type=simple
# User=root
# WorkingDirectory=/home/aagsolo
# ExecStart=/usr/bin/python3 /usr/local/bin/astroshell_ticklogger.py
# Restart=always
# RestartSec=10
# MemoryMax=50M
# Nice=10
#
# [Install]
# WantedBy=multi-user.target

# 5. Register and enable the service
systemctl daemon-reload
systemctl enable astroshell_ticklogger

# 6. Protect root filesystem again
mount -o remount,ro /

# 7. Start the service
systemctl start astroshell_ticklogger

==============================================================================
SERVICE MANAGEMENT
==============================================================================
Start:      systemctl start astroshell_ticklogger
Stop:       systemctl stop astroshell_ticklogger
Restart:    systemctl restart astroshell_ticklogger
Status:     systemctl status astroshell_ticklogger
Logs:       journalctl -u astroshell_ticklogger -f
Disable:    mount -o remount,rw / && systemctl disable astroshell_ticklogger && mount -o remount,ro /

==============================================================================
TESTING
==============================================================================
# Check if service is running
curl http://localhost:88/status

# Simulate Arduino tick data (v4.0 format with temp and tof)
curl "http://localhost:88/log?m=1&d=1&t=5234&temp=12.5&tof=7.3"

# Simulate event notification
curl "http://localhost:88/event?type=frozen_dome&detail=S1+attempt+1/3&temp=0.5&tof=7.3"

# View CSV output
cat /home/aagsolo/motor_ticks.csv

# Get temperature and coefficient (for Arduino /env endpoint)
curl http://localhost:88/env

==============================================================================
HTTP ENDPOINTS
==============================================================================
GET /log?m=<motor>&d=<direction>&t=<ticks>&temp=<C>&tof=<cm>
    Logs tick data to CSV with timestamp and temperature.
    m: Motor number (1 or 2)
    d: Direction (1=closing/open->close, 2=opening/close->open)
    t: Tick count from Arduino ISR (~61 ticks/second)
    temp: Arduino DS18B20 temperature in C (optional, -999 = unavailable)
    tof: Arduino VL53L0X distance in cm (optional, -1 = unavailable)
    Returns: "OK" or "ERROR"

GET /interrupt?m=<motor>&d=<direction>&t=<ticks>&temp=<C>&tof=<cm>
    Logs interrupted stop (motor stopped before reaching target limit).
    Same parameters as /log, logs to same CSV with "INTERRUPTED" marker.

GET /event?type=<type>&detail=<detail>&temp=<C>&tof=<cm>
    Receives event notification from Arduino, sends Pushover alert.
    type: Event type (frozen_dome, frozen_lockout, frozen_clear, sensor_fail, conflict)
    detail: Additional detail string
    temp: Current temperature in C
    tof: Current ToF distance in cm
    Returns: "OK"

GET /env
    Returns current temperature and coefficient for Arduino.
    Format: "<temperature>,<coefficient>" (e.g. "18.5,1.0")

GET /status
    Health check endpoint.
    Returns: "Tick Logger Running v4.0"

==============================================================================
CSV FORMAT (v4.0 — extended with Arduino temp and ToF)
==============================================================================
timestamp_utc,motor,direction,ticks,temperature,arduino_temp,tof_cm
2026-02-03T18:30:45Z,1,closing,5234,18.5,18.3,7.3
2026-02-03T18:35:12Z,1,opening,5456,-0.9,-1.0,7.2

==============================================================================
EVENT TYPES (Pushover notifications)
==============================================================================
frozen_dome     — Frozen dome detected, auto-reversing
frozen_lockout  — Dome locked out after 3 failed attempts (HIGH PRIORITY)
frozen_clear    — Dome opened successfully after previous frozen detection
sensor_fail     — DS18B20 or VL53L0X failure (10+ consecutive failures)
conflict        — Limit switch vs ToF disagreement

==============================================================================
"""

import os
import json
import subprocess
import threading
import urllib.request
import urllib.parse
from datetime import datetime, timezone
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# --- Configuration ---
SERVER_PORT = 88
CSV_FILE = "/home/aagsolo/motor_ticks.csv"
AAG_JSON_FILE = "/home/aagsolo/aag_json.dat"  # Cloudwatcher Solo weather data (read-only)

# --- Synology Backup Configuration ---
SYNOLOGY_ENABLED = True
SYNOLOGY_HOST = "192.168.1.113"
SYNOLOGY_USER = "solo"
SYNOLOGY_PATH = "/volume1/homes/solo/"

# --- Pushover Configuration ---
# Replace JOHNDOE placeholders with real credentials on the Solo device
PUSHOVER_ENABLED = True
PUSHOVER_API_TOKEN = "JOHNDOE_API_TOKEN"    # Your Pushover app API token
PUSHOVER_USER_KEY = "JOHNDOE_USER_KEY"      # Your Pushover user key
PUSHOVER_API_URL = "https://api.pushover.net/1/messages.json"

# Priority mapping for event types
# -2=lowest, -1=low, 0=normal, 1=high (bypass quiet hours), 2=emergency (requires ack)
PUSHOVER_PRIORITIES = {
    "frozen_dome":    0,   # Normal priority — informational, retrying
    "frozen_lockout": 1,   # High priority — dome is locked, needs attention!
    "frozen_clear":   -1,  # Low priority — good news, resolved
    "sensor_fail":    0,   # Normal priority — degraded but functioning
    "conflict":       0,   # Normal priority — informational
}

# Sound mapping for event types (Pushover sound names)
PUSHOVER_SOUNDS = {
    "frozen_lockout": "siren",  # Urgent attention needed
}

def get_solo_temperature():
    """
    Reads ambient temperature from Cloudwatcher Solo's aag_json.dat file.
    Returns temperature in Celsius or -999 if unavailable.

    JSON structure:
    {
        "temp" : -0.970000,
        "clouds" : 5.350000,
        ...
    }
    """
    try:
        with open(AAG_JSON_FILE, 'r') as f:
            data = json.load(f)
        temp = data.get('temp')
        if temp is not None:
            return round(float(temp), 1)
    except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError) as e:
        print(f"Temperature read error: {e}")
    return -999  # Fallback value indicates "not available"

def ensure_csv_header():
    """Creates CSV file with header if it doesn't exist (v4.0 format)."""
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w') as f:
            f.write("timestamp_utc,motor,direction,ticks,temperature,arduino_temp,tof_cm\n")
        print(f"Created new CSV file: {CSV_FILE}")

def log_tick_data(motor, direction, ticks, arduino_temp=None, tof_cm=None):
    """
    Logs a single tick measurement to CSV.

    Args:
        motor: 1 or 2
        direction: 1 (closing/open->close) or 2 (opening/close->open)
        ticks: ISR tick count (~61 ticks/second)
        arduino_temp: Temperature from Arduino DS18B20 (string, optional)
        tof_cm: ToF distance from Arduino VL53L0X in cm (string, optional)
    """
    timestamp = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    temperature = get_solo_temperature()

    # Direction as readable string for CSV
    dir_str = "closing" if direction == "1" else "opening"

    # Arduino-reported values (use empty string if not provided)
    a_temp = arduino_temp if arduino_temp else ""
    a_tof = tof_cm if tof_cm else ""

    line = f"{timestamp},{motor},{dir_str},{ticks},{temperature},{a_temp},{a_tof}\n"

    try:
        with open(CSV_FILE, 'a') as f:
            f.write(line)
        print(f"Logged: M{motor} {dir_str} {ticks} ticks @ {temperature}C (Arduino: {a_temp}C, ToF: {a_tof}cm)")
        backup_to_synology()  # Sync to NAS after each record
        return True
    except IOError as e:
        print(f"CSV write error: {e}")
        return False

def log_interrupt_data(motor, direction, ticks, arduino_temp=None, tof_cm=None):
    """
    Logs an interrupted stop to CSV (motor stopped before reaching target).

    Args:
        motor: 1 or 2
        direction: 1 (was closing) or 2 (was opening)
        ticks: ISR tick count at interruption
        arduino_temp: Temperature from Arduino DS18B20 (string, optional)
        tof_cm: ToF distance from Arduino VL53L0X in cm (string, optional)
    """
    timestamp = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    temperature = get_solo_temperature()

    # Direction with INTERRUPTED marker
    dir_str = "INTERRUPTED-closing" if direction == "1" else "INTERRUPTED-opening"

    a_temp = arduino_temp if arduino_temp else ""
    a_tof = tof_cm if tof_cm else ""

    line = f"{timestamp},{motor},{dir_str},{ticks},{temperature},{a_temp},{a_tof}\n"

    try:
        with open(CSV_FILE, 'a') as f:
            f.write(line)
        print(f"INTERRUPT: M{motor} {dir_str} at {ticks} ticks @ {temperature}C (Arduino: {a_temp}C, ToF: {a_tof}cm)")
        backup_to_synology()  # Sync to NAS after each record
        return True
    except IOError as e:
        print(f"CSV write error: {e}")
        return False

def backup_to_synology():
    """
    Copies CSV file to Synology NAS via SCP with UTC timestamp in filename.
    Runs in background thread so HTTP handler is never blocked.
    Errors are logged but don't affect main operation.
    """
    if not SYNOLOGY_ENABLED:
        return

    if not os.path.exists(CSV_FILE):
        return

    def _scp_worker():
        timestamp = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H-%M-%SZ')
        remote_filename = f"motor_ticks_{timestamp}.csv"
        remote_path = f"{SYNOLOGY_USER}@{SYNOLOGY_HOST}:{SYNOLOGY_PATH}{remote_filename}"

        try:
            result = subprocess.run(
                ['scp', '-q', '-o', 'StrictHostKeyChecking=no', '-o', 'ConnectTimeout=10',
                 CSV_FILE, remote_path],
                capture_output=True,
                timeout=30
            )
            if result.returncode == 0:
                print(f"Backup: {remote_filename}")
            else:
                print(f"Backup: SCP failed - {result.stderr.decode().strip()}")
        except subprocess.TimeoutExpired:
            print("Backup: SCP timeout")
        except Exception as e:
            print(f"Backup: Error - {e}")

    thread = threading.Thread(target=_scp_worker, daemon=True)
    thread.start()

def send_pushover_event(event_type, detail, temp=None, tof=None):
    """
    Sends a Pushover notification for dome safety events.
    Runs in background thread so HTTP handler returns immediately.

    Args:
        event_type: Event type string (e.g., "frozen_dome", "sensor_fail")
        detail: Additional detail string from Arduino
        temp: Current temperature in C (string from Arduino)
        tof: Current ToF distance in cm (string from Arduino)
    """
    if not PUSHOVER_ENABLED:
        return

    # Skip if using placeholder credentials
    if "JOHNDOE" in PUSHOVER_API_TOKEN or "JOHNDOE" in PUSHOVER_USER_KEY:
        print(f"Pushover: Skipped (placeholder credentials) - {event_type}: {detail}")
        return

    def _pushover_worker():
        # Build human-readable message
        title_map = {
            "frozen_dome":    "Frozen Dome Detected",
            "frozen_lockout": "DOME LOCKED - Frozen!",
            "frozen_clear":   "Dome Unfrozen",
            "sensor_fail":    "Sensor Failure",
            "conflict":       "Signal Conflict",
        }
        title = f"AstroShell: {title_map.get(event_type, event_type)}"

        # Build message body with sensor data
        lines = [detail]
        if temp and temp != "-999":
            lines.append(f"Temp: {temp}C")
        if tof and tof != "-1":
            lines.append(f"ToF: {tof}cm")
        message = "\n".join(lines)

        # Pushover API parameters
        data = urllib.parse.urlencode({
            "token": PUSHOVER_API_TOKEN,
            "user": PUSHOVER_USER_KEY,
            "title": title,
            "message": message,
            "priority": PUSHOVER_PRIORITIES.get(event_type, 0),
            "sound": PUSHOVER_SOUNDS.get(event_type, "pushover"),
        }).encode('utf-8')

        # High priority events need retry/expire parameters
        if PUSHOVER_PRIORITIES.get(event_type, 0) >= 2:
            extra = urllib.parse.urlencode({
                "retry": 60,    # Retry every 60 seconds
                "expire": 3600  # Stop retrying after 1 hour
            }).encode('utf-8')
            data = data + b"&" + extra

        try:
            req = urllib.request.Request(PUSHOVER_API_URL, data=data)
            with urllib.request.urlopen(req, timeout=10) as resp:
                if resp.status == 200:
                    print(f"Pushover: Sent [{event_type}] {detail}")
                else:
                    print(f"Pushover: HTTP {resp.status} for [{event_type}]")
        except Exception as e:
            print(f"Pushover: Error - {e}")

    thread = threading.Thread(target=_pushover_worker, daemon=True)
    thread.start()

class TickLoggerHandler(BaseHTTPRequestHandler):
    """HTTP request handler for tick logging and event notifications."""

    def log_message(self, format, *args):
        """Override to customize logging."""
        print(f"{self.address_string()} - {format % args}")

    def do_GET(self):
        """Handle GET requests from Arduino."""
        parsed = urlparse(self.path)

        if parsed.path == '/log':
            # Parse query parameters: ?m=1&d=1&t=5234&temp=12.5&tof=7.3
            params = parse_qs(parsed.query)

            motor = params.get('m', [None])[0]
            direction = params.get('d', [None])[0]
            ticks = params.get('t', [None])[0]
            arduino_temp = params.get('temp', [None])[0]
            tof_cm = params.get('tof', [None])[0]

            if motor and direction and ticks:
                success = log_tick_data(motor, direction, ticks, arduino_temp, tof_cm)
                self.send_response(200 if success else 500)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'OK' if success else b'ERROR')
            else:
                self.send_response(400)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'Missing parameters (m, d, t required)')

        elif parsed.path == '/interrupt':
            # Parse query parameters for interrupted stop
            params = parse_qs(parsed.query)

            motor = params.get('m', [None])[0]
            direction = params.get('d', [None])[0]
            ticks = params.get('t', [None])[0]
            arduino_temp = params.get('temp', [None])[0]
            tof_cm = params.get('tof', [None])[0]

            if motor and direction and ticks:
                success = log_interrupt_data(motor, direction, ticks, arduino_temp, tof_cm)
                self.send_response(200 if success else 500)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'OK' if success else b'ERROR')
            else:
                self.send_response(400)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'Missing parameters (m, d, t required)')

        elif parsed.path == '/event':
            # Event notification from Arduino — send Pushover alert
            params = parse_qs(parsed.query)

            event_type = params.get('type', [None])[0]
            detail = params.get('detail', [''])[0]
            temp = params.get('temp', [None])[0]
            tof = params.get('tof', [None])[0]

            if event_type:
                print(f"EVENT: [{event_type}] {detail} (temp={temp}, tof={tof})")
                send_pushover_event(event_type, detail, temp, tof)

                self.send_response(200)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'OK')
            else:
                self.send_response(400)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'Missing parameter: type')

        elif parsed.path == '/env':
            # Return current temperature and coefficient for Arduino
            temp = get_solo_temperature()
            coeff = 1.0  # Fixed for now, later from analysis

            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(f"{temp},{coeff}".encode())

        elif parsed.path == '/status':
            # Simple status check endpoint
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Tick Logger Running v4.0')

        else:
            self.send_response(404)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Not Found')

def main():
    """Main entry point."""
    print(f"=" * 50)
    print(f"AstroShell Tick Logger & Event Server v4.0")
    print(f"=" * 50)
    print(f"Port: {SERVER_PORT}")
    print(f"CSV:  {CSV_FILE}")
    print(f"=" * 50)

    ensure_csv_header()

    # Test temperature fetch on startup
    temp = get_solo_temperature()
    print(f"Current temperature: {temp}C")

    # Log Synology backup status
    if SYNOLOGY_ENABLED:
        print(f"Backup: Syncing to {SYNOLOGY_USER}@{SYNOLOGY_HOST} after each record")

    # Log Pushover status
    if PUSHOVER_ENABLED:
        if "JOHNDOE" in PUSHOVER_API_TOKEN:
            print("Pushover: DISABLED (placeholder credentials - replace JOHNDOE values)")
        else:
            print("Pushover: ENABLED")

    server = HTTPServer(('0.0.0.0', SERVER_PORT), TickLoggerHandler)
    print(f"Server listening on port {SERVER_PORT}...")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()

if __name__ == '__main__':
    main()
