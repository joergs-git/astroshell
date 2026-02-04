#!/usr/bin/env python3
"""
==============================================================================
AstroShell Tick Logger Server
==============================================================================
Receives motor runtime data from Arduino dome controller and logs to CSV
with ambient temperature for later analysis of temperature-dependent
motor timing coefficients.

Target Hardware: Cloudwatcher Solo (Raspberry Pi 3, armv6l)
Port: 88 (requires root)
Arduino sends: GET /log?m=<motor>&d=<direction>&t=<ticks>

==============================================================================
FILE LOCATIONS
==============================================================================
Script (persistent):    /usr/local/bin/astroshell_ticklogger.py
Service (persistent):   /etc/systemd/system/astroshell_ticklogger.service
CSV output (tmpfs):     /home/aagsolo/motor_ticks.csv
Weather JSON (tmpfs):   /home/aagsolo/aag_json.dat (created by Solo)

Note: The Solo has a read-only root filesystem to protect the SD card.
      /home/aagsolo is a tmpfs (RAM disk) - data is lost on reboot.
      For long-term storage, implement Supabase push later.

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

# Simulate Arduino tick data
curl "http://localhost:88/log?m=1&d=1&t=5234"

# View CSV output
cat /home/aagsolo/motor_ticks.csv

# Get temperature and coefficient (for Arduino /env endpoint)
curl http://localhost:88/env

==============================================================================
HTTP ENDPOINTS
==============================================================================
GET /log?m=<motor>&d=<direction>&t=<ticks>
    Logs tick data to CSV with timestamp and temperature.
    m: Motor number (1 or 2)
    d: Direction (1=closing/open->close, 2=opening/close->open)
    t: Tick count from Arduino ISR (~61 ticks/second)
    Returns: "OK" or "ERROR"

GET /interrupt?m=<motor>&d=<direction>&t=<ticks>
    Logs interrupted stop (motor stopped before reaching target limit).
    Same parameters as /log, logs to same CSV with "INTERRUPTED" marker.

GET /env
    Returns current temperature and coefficient for Arduino.
    Format: "<temperature>,<coefficient>" (e.g. "18.5,1.0")

GET /status
    Health check endpoint.
    Returns: "Tick Logger Running"

==============================================================================
CSV FORMAT
==============================================================================
timestamp_utc,motor,direction,ticks,temperature
2026-02-03T18:30:45Z,1,closing,5234,18.5
2026-02-03T18:35:12Z,1,opening,5456,-0.9

==============================================================================
"""

import os
import json
from datetime import datetime, timezone
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# --- Configuration ---
SERVER_PORT = 88
CSV_FILE = "/home/aagsolo/motor_ticks.csv"
AAG_JSON_FILE = "/home/aagsolo/aag_json.dat"  # Cloudwatcher Solo weather data (read-only)

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
    """Creates CSV file with header if it doesn't exist."""
    if not os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'w') as f:
            f.write("timestamp_utc,motor,direction,ticks,temperature\n")
        print(f"Created new CSV file: {CSV_FILE}")

def log_tick_data(motor, direction, ticks):
    """
    Logs a single tick measurement to CSV.

    Args:
        motor: 1 or 2
        direction: 1 (closing/open->close) or 2 (opening/close->open)
        ticks: ISR tick count (~61 ticks/second)
    """
    timestamp = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    temperature = get_solo_temperature()

    # Direction as readable string for CSV
    dir_str = "closing" if direction == "1" else "opening"

    line = f"{timestamp},{motor},{dir_str},{ticks},{temperature}\n"

    try:
        with open(CSV_FILE, 'a') as f:
            f.write(line)
        print(f"Logged: M{motor} {dir_str} {ticks} ticks @ {temperature}C")
        return True
    except IOError as e:
        print(f"CSV write error: {e}")
        return False

def log_interrupt_data(motor, direction, ticks):
    """
    Logs an interrupted stop to CSV (motor stopped before reaching target).

    Args:
        motor: 1 or 2
        direction: 1 (was closing) or 2 (was opening)
        ticks: ISR tick count at interruption
    """
    timestamp = datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    temperature = get_solo_temperature()

    # Direction with INTERRUPTED marker
    dir_str = "INTERRUPTED-closing" if direction == "1" else "INTERRUPTED-opening"

    line = f"{timestamp},{motor},{dir_str},{ticks},{temperature}\n"

    try:
        with open(CSV_FILE, 'a') as f:
            f.write(line)
        print(f"INTERRUPT: M{motor} {dir_str} at {ticks} ticks @ {temperature}C")
        return True
    except IOError as e:
        print(f"CSV write error: {e}")
        return False

class TickLoggerHandler(BaseHTTPRequestHandler):
    """HTTP request handler for tick logging."""

    def log_message(self, format, *args):
        """Override to customize logging."""
        print(f"{self.address_string()} - {format % args}")

    def do_GET(self):
        """Handle GET requests from Arduino."""
        parsed = urlparse(self.path)

        if parsed.path == '/log':
            # Parse query parameters: ?m=1&d=1&t=5234
            params = parse_qs(parsed.query)

            motor = params.get('m', [None])[0]
            direction = params.get('d', [None])[0]
            ticks = params.get('t', [None])[0]

            if motor and direction and ticks:
                success = log_tick_data(motor, direction, ticks)
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

            if motor and direction and ticks:
                success = log_interrupt_data(motor, direction, ticks)
                self.send_response(200 if success else 500)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'OK' if success else b'ERROR')
            else:
                self.send_response(400)
                self.send_header('Content-Type', 'text/plain')
                self.end_headers()
                self.wfile.write(b'Missing parameters (m, d, t required)')

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
            self.wfile.write(b'Tick Logger Running')

        else:
            self.send_response(404)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'Not Found')

def main():
    """Main entry point."""
    print(f"=" * 50)
    print(f"AstroShell Tick Logger Server")
    print(f"=" * 50)
    print(f"Port: {SERVER_PORT}")
    print(f"CSV:  {CSV_FILE}")
    print(f"=" * 50)

    ensure_csv_header()

    # Test temperature fetch on startup
    temp = get_solo_temperature()
    print(f"Current temperature: {temp}C")

    server = HTTPServer(('0.0.0.0', SERVER_PORT), TickLoggerHandler)
    print(f"Server listening on port {SERVER_PORT}...")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()

if __name__ == '__main__':
    main()
