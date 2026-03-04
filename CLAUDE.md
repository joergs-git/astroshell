# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Important Rules

- **NO co-authoring lines in commits** - Do not add "Co-Authored-By" lines to commit messages
- **Read before editing** - Always read a file before making changes
- **Preserve code style** - Match existing formatting, indentation, and naming conventions
- **Comments-only commits** - When adding comments/analysis without code changes, state this clearly in the commit message
- **Use different MAC and IP for testing before git commit and push** - During development and testing ALWAYS use IP `192.168.1.179` and MAC `{0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEA}` to avoid conflicts with the live production dome at .177/:ED. Before the final commit and push, ALWAYS change back to IP `192.168.1.177` and MAC `{0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}`. This is a mandatory pre-commit check — never push test IP/MAC to the repository.


## Project Overview

AstroShell Dome Controller - Arduino-based control system for a two-shutter astronomical telescope dome with automatic rain protection via Lunatico Cloudwatcher Solo integration, temperature-based dynamic motor timeout, and frozen dome detection.

### Repository Files

| File | Purpose |
|------|---------|
| `domecontrol_JK3.ino` | **Main controller** - Arduino MEGA v4.0 with safety sensors, dynamic timeout, frozen dome detection |
| `astroshell_ticklogger.py` | **Pi server** - Receives tick data + events from Arduino, logs to CSV, sends Pushover alerts |
| `astroshell_ticklogger.service` | Systemd service file for tick logger autostart |
| `controller_wcapacitor_2025_germany.ino` | Alternative controller with capacitor backup (has known bugs, see header) |
| `nocapacitor2023.ino` | Original AstroShell reference code (unmodified) |
| `cloudwatcher-raincheckerV3.sh` | **Rain checker** - Auto-closes dome on rain, Pushover alerts |
| `cloudwatcher-rainchecker.service` | Systemd service file for rain checker autostart |
| `open_astroshell.bat` | **NINA script** - Opens dome via sequence instruction, Pushover notification |
| `close_astroshell.bat` | **NINA script** - Closes dome via sequence instruction, Pushover notification |
| `motortick-statistic-batchrun.sh` | **Batch runner** - Automated open/close cycles for motor tick statistics |
| `README.md` | User documentation with wiring notes |

## Hardware Configuration

- **Microcontroller**: Arduino MEGA 2560 with Ethernet Shield (W5100/W5500)
- **Dome IP**: 192.168.1.177 (static)
- **Cloudwatcher IP**: 192.168.1.151 (monitored for connectivity)
- **Motors**: Two DC motors with PWM soft-start (pins 3, 5, 6, 9)
- **Limit switches**: Pins 0, 1, 2, 7 (pins 0/1 conflict with Serial - debugging disabled)
- **Buttons**: A2-A5 (shutter controls), pin 8 (emergency STOP)
- **DS18B20**: Pin 22 (MEGA-exclusive, 4.7k pullup to 5V, external Vcc power)
- **VL53L0X**: I2C pins 20 (SDA), 21 (SCL) (MEGA-exclusive)
- **Watchdog**: 8-second hardware watchdog via avr/wdt.h

### Pin Policy
NO existing pins/wiring can be changed (hardwired on the board). Only MEGA-exclusive pins (14+, 20/21) are used for new sensors, guaranteeing no conflicts with existing hardwired connections.

## Architecture - domecontrol_JK3.ino (Main)

### Timer2 ISR (~61 Hz)
All real-time operations run in the interrupt service routine:
- Motor PWM control with soft-start (SMOOTH=30)
- Button debouncing (cnt > 6 ticks = ~100ms)
- Limit switch monitoring
- Motor timeout safety (dynamic or static 6527 ticks)
- Tick counting for motor runtime measurement
- Frozen dome tick counter (~4 clock cycles overhead when active)

### Sensors (v4.0)
- **DS18B20**: Non-blocking async read every 5 seconds, used for dynamic timeout
- **VL53L0X**: Continuous mode, read every 200ms, used for frozen dome detection
- Both sensors degrade gracefully: 10 consecutive failures → feature disabled

### Dynamic Motor Timeout (v4.0)
- Linear regression model from 253 cycle analysis (-1.6C to 22.6C)
- Four separate coefficients for M1/M2 closing/opening
- Integer-only math (no floats): `timeout = base - (slope100 * temp_x10) / 1000 + margin`
- Clamped to [2000, 6527] ticks
- Falls back to static 6527 on temperature sensor failure

### Frozen Dome Detection (v4.0)
- VL53L0X ToF measures gap between dome halves across mounting point
- EEPROM-calibrated baseline: `/?$C` stores current distance as "closed"
- After ~4 sec of opening: checks if gap increased by >30mm (tolerance)
- If frozen: STOP motor → 20s gravity wait → re-check → retry or reverse
- 3 attempts maximum, then LOCKOUT (all open commands blocked)
- `/?$U` unlocks dome from lockout
- Disabled when ToF not calibrated or disconnected

### Network Monitoring
- Checks Cloudwatcher IP every **90 seconds** (`connectCheckInterval = 90000`)
- After **10 failures within 15 minutes**, dome auto-closes
- Cable removal triggers **immediate** auto-close
- Uses `netClient.setTimeout(3000)` to prevent watchdog timeout
- No Pushover for IP auto-close (network is down when it triggers)

### Event Notifications (v4.0)
- Arduino pushes events to Solo:88/event via HTTP GET
- Solo server sends Pushover alerts (background thread)
- Rate-limited: 10s minimum between events, 5min for conflict events
- Events: frozen_dome, frozen_lockout, frozen_clear, sensor_fail, conflict

### Web Server (Port 80)
- Auto-refresh every 10 seconds
- Commands via URL parameters

## Web API Commands

| Command | Physical Action |
|---------|-----------------|
| `/?$1` | CLOSE Shutter 1 (East) |
| `/?$2` | OPEN Shutter 1 (East) — blocked during frozen lockout |
| `/?$3` | CLOSE Shutter 2 (West) |
| `/?$4` | OPEN Shutter 2 (West) — blocked during frozen lockout |
| `/?$5` | Emergency STOP all motors |
| `/?$S` | Get status: "OPEN" or "CLOSE" |
| `/?$R` | Reset EEPROM counters |
| `/?$L` | Toggle tick logging on/off (default: ON after reboot) |
| `/?$U` | Unlock dome from frozen lockout (v4.0) |
| `/?$C` | Calibrate ToF baseline — dome must be fully closed (v4.0) |

**Note:** Button labels in code are inverted due to hardware wiring swap. The web interface shows physical reality.

## Motor Stop Reason Codes

Displayed on web interface under "Stop Reason":

| Code | Meaning | Web Display |
|------|---------|-------------|
| 0 | Limit switch reached | "Limit 'Phys. Open/Closed'" |
| 1 | Physical button or SWSTOP | "Button/SWSTOP" |
| 2 | Web command ($5 or direction change) | "Web STOP" |
| 3 | IP failure auto-close | "IP Fail Auto-Close" |
| 4 | VCC failure auto-close | "VCC Fail Auto-Close" |
| 5 | Motor timeout reached | "TIMEOUT at X ticks" |
| 6 | Frozen dome auto-reverse (v4.0) | "FROZEN DOME Auto-Reverse" |

## Critical Code Quirk - Inverted Limit Switches

Due to installation wiring error, limit switch names are **inverted** in code:
```
lim1open (pin 7)   → Actually detects PHYSICALLY CLOSED
lim1closed (pin 2) → Actually detects PHYSICALLY OPEN
lim2open (pin 1)   → Actually detects PHYSICALLY CLOSED
lim2closed (pin 0) → Actually detects PHYSICALLY OPEN
```

The **motor control logic is unchanged** from original. Only the **web display** was inverted to show physical reality. See README.md for details if adapting for correctly-wired installations.

## EEPROM Memory Map

| Offset | Size | Content |
|--------|------|---------|
| 0 | 1 byte | Magic byte (0x42) |
| 1-2 | 2 bytes | Total IP failures (uint16) |
| 3-4 | 2 bytes | Auto-close events (uint16) |
| 5 | 1 byte | Uptime day counter |
| 6-7 | 2 bytes | ToF baseline distance in mm (v4.0) |
| 8 | 1 byte | ToF calibration magic (0xA5) (v4.0) |

## Key Configuration Constants

### domecontrol_JK3.ino
```cpp
#define SMOOTH 30                           // Soft-start smoothness
#define MAX_MOT1_OPEN  6527                 // Motor timeout fallback (107 sec at 61Hz)

// Dynamic timeout regression coefficients (v4.0)
#define DYN_M1_CLOSE_BASE    6096           // M1 closing: base ticks at 0C
#define DYN_M1_CLOSE_SLOPE   1945           // M1 closing: slope * 100
#define DYN_M1_CLOSE_MARGIN  601            // M1 closing: safety margin ticks

// Frozen dome detection (v4.0)
#define TOF_OPEN_TOLERANCE     30           // mm: gap increase to detect opening
#define TOF_CHECK_TICKS       244           // ~4 sec of motor before first check
#define FROZEN_GRAVITY_WAIT  20000          // ms: wait with motor off
#define FROZEN_MAX_RETRIES       3          // Attempts before lockout

const unsigned long connectCheckInterval = 90000UL;   // IP check every 90 sec
const byte maxConnectFails = 10;            // Failures to trigger close
const unsigned long maxFailTimeWindow = 900000UL;     // 15-min failure window
```

### cloudwatcher-raincheckerV3.sh
```bash
RAIN_THRESHOLD=2900         # Rain detected when value < this
DRY_COOLDOWN_MINUTES=30     # Minutes before "dry" notification
DOME_IP="192.168.1.177"     # Dome controller IP
CHECK_INTERVAL=10           # Seconds between checks
RAIN_ACTION_COOLDOWN=300    # Seconds after dome close (5 min)
PING_FAIL_THRESHOLD=3       # Consecutive ping failures before alarm
SCHEDULED_CHECK_HOURS="8 13 16"  # Hours to check if dome is open
CLOSE_VERIFY_DELAY=180      # Seconds before verifying close (3 min)
```

## Motor Runtime Tick Logging

Measures motor runtime in ISR ticks for temperature correlation analysis.

### How It Works
1. Arduino counts ticks during motor full-runs (start at one limit, stop at opposite)
2. Valid full-runs logged to `/log` endpoint, interrupted stops logged to `/interrupt`
3. When toggle enabled (`$L`), Arduino pushes data to Solo:88 via HTTP GET
4. All requests include temperature (DS18B20) and ToF distance (VL53L0X) (v4.0)
5. Pi server logs to CSV with timestamp, Solo temperature, Arduino temp, and ToF
6. CSV is backed up to Synology NAS via SCP after each new record

### Tick Logger & Event Server (Cloudwatcher Solo / Pi3)

**Location on Pi (read-only root filesystem):**
- Script: `/usr/local/bin/astroshell_ticklogger.py`
- Service: `/etc/systemd/system/astroshell_ticklogger.service`
- CSV output: `/home/aagsolo/motor_ticks.csv` (tmpfs, backed up to Synology)
- Weather data: `/home/aagsolo/aag_json.dat`
- Synology backup: `solo@192.168.1.113:/volume1/homes/solo/motor_ticks_<UTC-timestamp>.csv`

**Server Endpoints (port 88):**
| Endpoint | Purpose |
|----------|---------|
| `GET /log?m=1&d=1&t=5234&temp=12.5&tof=7.3` | Log valid full-run with sensor data |
| `GET /interrupt?m=1&d=1&t=5234&temp=12.5&tof=7.3` | Log interrupted stop with sensor data |
| `GET /event?type=frozen_dome&detail=S1+attempt+1/3&temp=0.5&tof=7.3` | Event notification → Pushover (v4.0) |
| `GET /env` | Get temperature and coefficient for Arduino |
| `GET /status` | Health check |

**CSV Format (v4.0):**
```
timestamp_utc,motor,direction,ticks,temperature,arduino_temp,tof_cm
2026-02-04T09:33:28Z,1,closing,5800,3.0,3.1,7.3     # Valid full-run
2026-02-04T09:35:00Z,2,INTERRUPTED-closing,6527,3.1,3.2,7.2  # Timeout/manual stop
```

**Event Types (Pushover):**
| Type | Priority | When |
|------|----------|------|
| `frozen_dome` | Normal | Frozen detected, auto-reversing (includes attempt #) |
| `frozen_lockout` | High + siren | Dome locked after 3 failed attempts |
| `frozen_clear` | Low | Dome opened successfully after frozen detection |
| `sensor_fail` | Normal | DS18B20 or VL53L0X failure (10+ consecutive) |
| `conflict` | Normal | Limit switch vs ToF disagreement (rate-limited 5min) |

**Service Management:**
```bash
systemctl start astroshell_ticklogger
systemctl stop astroshell_ticklogger
systemctl status astroshell_ticklogger
journalctl -u astroshell_ticklogger -f
```

**Synology Backup Setup (one-time on Solo as root):**
```bash
ssh-keygen -t rsa -N "" -f ~/.ssh/id_rsa
ssh-copy-id solo@192.168.1.113
```

**Modifying files on read-only Solo:**
```bash
mount -o remount,rw /
# make changes
mount -o remount,ro /
```

## Development Notes

### Arduino IDE Setup
Upload `domecontrol_JK3.ino` to Arduino MEGA. Required libraries:
- SPI.h, Ethernet.h, EEPROM.h, avr/wdt.h (built-in)
- OneWire.h, DallasTemperature.h (install via Library Manager) (v4.0)
- Wire.h (built-in), VL53L0X.h (Pololu, install via Library Manager) (v4.0)

### Debug Modes (USE WITH CAUTION)
Serial debugging conflicts with limit switch pins 0/1. Only enable with limit switches disconnected:
```cpp
// #define SERIAL_DEBUG_GENERAL
// #define SERIAL_DEBUG_IP
// #define SERIAL_DEBUG_BUTTONS
// #define SERIAL_DEBUG_LIMITS
// #define SERIAL_DEBUG_EEPROM
```

## Rain Checker (Cloudwatcher Solo / Pi3)

Monitors rain sensor and automatically closes dome. Sends Pushover notifications.

### Functionality
1. Reads rain value from `aag_json.dat` every 10 seconds
2. If rain detected (< 2900): Closes dome ($3 West, $1 East), sends Pushover alert
3. 30-minute cooldown before "dry" notification (prevents spam during showers)
4. Ping monitoring (rain AND dry): Alerts after 3 consecutive failures (configurable)
5. Scheduled status checks (8:00, 13:00, 16:00): Alerts if dome is open
6. Close verification: 3 min after rain close, sends success/failure notification

### Location on Pi (read-only root filesystem)
- Script: `/usr/local/bin/cloudwatcher-rainchecker.sh`
- Service: `/etc/systemd/system/cloudwatcher-rainchecker.service`
- Log file: `/home/aagsolo/rainchecker.log` (tmpfs)
- Status flags: `/home/aagsolo/RAINTRIGGERED`, `DRYTRIGGERED`, `PINGALARM`, `PINGFAILCOUNT`, `LASTSCHEDULEDCHECK`, `CLOSEVERIFYTIME`, `CLOSEVERIFYALERTED` (tmpfs)

### Service Management
```bash
systemctl start cloudwatcher-rainchecker
systemctl stop cloudwatcher-rainchecker
systemctl status cloudwatcher-rainchecker
journalctl -u cloudwatcher-rainchecker -f
tail -f /home/aagsolo/rainchecker.log
```

### Installation
```bash
mount -o remount,rw /
cp cloudwatcher-raincheckerV3.sh /usr/local/bin/cloudwatcher-rainchecker.sh
chmod +x /usr/local/bin/cloudwatcher-rainchecker.sh
cp cloudwatcher-rainchecker.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable cloudwatcher-rainchecker
mount -o remount,ro /
systemctl start cloudwatcher-rainchecker
```

Requires `jq` for JSON parsing (falls back to grep/sed).

## Motor Tick Statistic Batch Runner (Cloudwatcher Solo / Pi3)

Automated dome open/close cycles to collect motor tick runtime data. Designed for manual daytime execution on the Solo Pi. One "run" = dome CLOSED -> OPEN -> CLOSED.

### Purpose

Motor runtime (ISR ticks) varies with temperature due to motor oil viscosity. This script repeatedly cycles the dome with randomized rest periods (5-30 min) to collect data across varying oil temperatures:
- Short rest (5 min): motor oil still warm from previous run
- Long rest (30 min): motor oil cools to ambient temperature
- Random distribution provides data points across the cooling curve

### Behavior

1. Enables tick logging on Arduino (`$L`) if not already enabled
2. Verifies dome at CLOSED endpoints (both shutters at limit switches)
3. Checks weather - only opens when dry
4. Opens dome (West `$4`, East `$2`), waits 200s for motors
5. Verifies BOTH shutters at OPEN endpoints (parses web page HTML)
6. Rests random 5-30 min (rain monitored during open rest)
7. Checks dome state - detects if closed externally during rest
8. Closes dome (West `$3`, East `$1`), waits 200s - no weather check needed
9. Verifies BOTH shutters at CLOSED endpoints
10. Run complete. Rests random 5-30 min, then repeats
11. After MAX_RUNS (default 20), sends summary and exits

### Endpoint Verification

After each motor run, the script parses the Arduino web page to confirm both shutters reached their target limit switch. Checks for "Physically OPEN" / "Physically CLOSED" state strings. Any "Intermediate" state triggers an INTERRUPT alert via Pushover, and the run does not count.

### Weather Strategy

- Rain is only checked **before opening** the dome. Closing is always safe.
- During open rest periods, rain is checked every 60 seconds.
- If rain detected while open: dome closes immediately, run does not count.
- If rain checker or manual closes dome during rest: detected and handled gracefully.

### Pushover Notifications

Every message includes "Run X/Y" (current/total). Sent for:
- Script start / complete / stopped (Ctrl+C)
- Every dome open and close (start + endpoint confirmation)
- Interrupted motor runs (shutter not at expected endpoint)
- Rain detected / cleared
- Run completion with next rest time

### Configuration

```bash
MAX_RUNS=20                 # Number of full open/close cycles
MOTOR_WAIT_SECONDS=200      # Wait for motors to reach endpoints
MIN_REST_MINUTES=5          # Minimum rest between direction changes
MAX_REST_MINUTES=30         # Maximum rest (randomized)
RAIN_THRESHOLD=2900         # Same as rain checker
DOME_IP="192.168.1.177"     # Dome controller
```

### Location on Pi

- Script: `/usr/local/bin/motortick-statistic-batchrun.sh`
- Log file: `/home/aagsolo/batchrun.log` (tmpfs)

### Usage

```bash
ssh root@192.168.1.151
/usr/local/bin/motortick-statistic-batchrun.sh
# Stop with Ctrl+C (dome will be closed safely)
```

### Installation

```bash
mount -o remount,rw /
cp motortick-statistic-batchrun.sh /usr/local/bin/
chmod +x /usr/local/bin/motortick-statistic-batchrun.sh
mount -o remount,ro /
```

### Prerequisites

- `astroshell_ticklogger.py` running on Solo port 88
- `aag_json.dat` updated by Cloudwatcher software
- Rain checker may remain running for safety (script cooperates)

## Known Issues in controller_wcapacitor_2025_germany.ino

This file has documented bugs (see file header for full list):
1. **FIXED**: Cloud sensor timer reset bug
2. Serial.println() without Serial.begin()
3. Pins 0/1 conflict with Serial
4. Pin 20 invalid on Arduino UNO
5. vibrotimer uses excessive memory
6. No hardware watchdog
7. millis() overflow after ~50 days

## Testing Safety

**Always test with dome in intermediate position** (not fully open or closed) so manual intervention is possible if motor moves unexpectedly.

### Sensor Testing (v4.0)

1. **DS18B20**: Verify temperature on web UI. Unplug → confirm "Not connected" and static 6527 timeout fallback
2. **VL53L0X**: Verify distance on web UI. Move hand → see distance change. Unplug → frozen detection shows "Disabled (sensor disconnected)"
3. **ToF calibration**: Close dome → click "Calibrate ToF Baseline" → verify baseline stored → reboot → verify persists
4. **Frozen dome simulation**: Block ToF sensor during open → verify stop + reverse + 3 retries + lockout
5. **Unlock**: After lockout → click "UNLOCK DOME" → verify open commands work again
6. **Graceful degradation**: Disconnect both sensors → verify system behaves identically to v3.3
7. **Lab testing without limit switches**: Connect pin 2 (lim1closed) to GND to allow M1 open commands. Pin 7 stays HIGH (pullup) simulating closed position. Close phase is instant in this setup.

### Important Implementation Details (v4.0)

These patterns were discovered during lab testing and are critical for future modifications:

- **ISR vs loop() race condition**: The frozen dome state machine runs in `loop()` and uses its own `fd_prev_mot1dir`/`fd_prev_mot2dir` static locals to detect motor start transitions. Do NOT use the ISR's `m1_prev_dir`/`m2_prev_dir` — the ISR updates them at 61 Hz before `loop()` runs, so `loop()` never sees the 0→CLOSE transition.
- **Sensor hot-plug re-detection**: Both DS18B20 and VL53L0X have 30-second periodic re-detection loops that re-scan the bus after 10 consecutive failures. Without this, unplugging and reconnecting a sensor requires a reboot.
- **DS18B20 startup delay**: A `delay(100)` before `ds18b20.begin()` in `setupDS18B20()` is required — the sensor needs time to boot after power-on. Without it, detection fails at startup and takes ~80 seconds via the re-detection path.
- **DS18B20 pullup resistor**: 4.7kΩ required (5.1kΩ also works). Without pullup, OneWire open-drain bus cannot communicate.
- **URL encoding**: Event detail strings sent to the Solo must be URL-encoded (spaces → `%20`). Python's `BaseHTTPRequestHandler` rejects URLs with literal spaces (HTTP 400). Encoding is done in a local buffer before `print()`, not char-by-char via `write()` (which caused TCP issues on W5100/W5500).
- **Python stdout buffering**: The ticklogger service file must use `python3 -u` (unbuffered) or `print()` output won't appear in `journalctl`. The `-u` flag is set in `astroshell_ticklogger.service`.
