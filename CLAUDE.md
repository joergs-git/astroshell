# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Important Rules

- **NO co-authoring lines in commits** - Do not add "Co-Authored-By" lines to commit messages
- **Read before editing** - Always read a file before making changes
- **Preserve code style** - Match existing formatting, indentation, and naming conventions
- **Comments-only commits** - When adding comments/analysis without code changes, state this clearly in the commit message
- **Use different MAC and IP for testing before git commit and push** - During development use always a different IP e.g. 192.168.1.178 instead of .177 and a different MAC Adresse e.g. {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEA}. When committing change this back to .177 and ED.


## Project Overview

AstroShell Dome Controller - Arduino-based control system for a two-shutter astronomical telescope dome with automatic rain protection via Lunatico Cloudwatcher Solo integration.

### Repository Files

| File | Purpose |
|------|---------|
| `domecontrol_JK3.ino` | **Main controller** - Arduino MEGA v3.3 with network monitoring and tick logging |
| `astroshell_ticklogger.py` | **Pi server** - Receives tick data from Arduino, logs to CSV with temperature |
| `astroshell_ticklogger.service` | Systemd service file for tick logger autostart |
| `controller_wcapacitor_2025_germany.ino` | Alternative controller with capacitor backup (has known bugs, see header) |
| `nocapacitor2023.ino` | Original AstroShell reference code (unmodified) |
| `cloudwatcher-raincheckerV3.sh` | **Rain checker** - Auto-closes dome on rain, Pushover alerts |
| `cloudwatcher-rainchecker.service` | Systemd service file for rain checker autostart |
| `README.md` | User documentation with wiring notes |

## Hardware Configuration

- **Microcontroller**: Arduino MEGA 2560 with Ethernet Shield (W5100/W5500)
- **Dome IP**: 192.168.1.177 (static)
- **Cloudwatcher IP**: 192.168.1.151 (monitored for connectivity)
- **Motors**: Two DC motors with PWM soft-start (pins 3, 5, 6, 9)
- **Limit switches**: Pins 0, 1, 2, 7 (pins 0/1 conflict with Serial - debugging disabled)
- **Buttons**: A2-A5 (shutter controls), pin 8 (emergency STOP)
- **Watchdog**: 8-second hardware watchdog via avr/wdt.h

## Architecture - domecontrol_JK3.ino (Main)

### Timer2 ISR (~61 Hz)
All real-time operations run in the interrupt service routine:
- Motor PWM control with soft-start (SMOOTH=30)
- Button debouncing (cnt > 6 ticks = ~100ms)
- Limit switch monitoring
- Motor timeout safety (6527 ticks = 107 seconds)
- Tick counting for motor runtime measurement (v3.3)

### Network Monitoring
- Checks Cloudwatcher IP every **1 minute** (`connectCheckInterval = 60000`)
- After **5 failures within 5 minutes**, dome auto-closes
- Cable removal triggers **immediate** auto-close
- Uses `netClient.setTimeout(3000)` to prevent watchdog timeout

### Web Server (Port 80)
- Auto-refresh every 10 seconds
- Commands via URL parameters

## Web API Commands

| Command | Physical Action |
|---------|-----------------|
| `/?$1` | CLOSE Shutter 1 (East) |
| `/?$2` | OPEN Shutter 1 (East) |
| `/?$3` | CLOSE Shutter 2 (West) |
| `/?$4` | OPEN Shutter 2 (West) |
| `/?$5` | Emergency STOP all motors |
| `/?$S` | Get status: "OPEN" or "CLOSE" |
| `/?$R` | Reset EEPROM counters |
| `/?$L` | Toggle tick logging on/off (default: off after reboot) |

**Note:** Button labels in code are inverted due to hardware wiring swap. The web interface shows physical reality.

## Critical Code Quirk - Inverted Limit Switches

Due to installation wiring error, limit switch names are **inverted** in code:
```
lim1open (pin 7)   → Actually detects PHYSICALLY CLOSED
lim1closed (pin 2) → Actually detects PHYSICALLY OPEN
lim2open (pin 1)   → Actually detects PHYSICALLY CLOSED
lim2closed (pin 0) → Actually detects PHYSICALLY OPEN
```

The **motor control logic is unchanged** from original. Only the **web display** was inverted to show physical reality. See README.md for details if adapting for correctly-wired installations.

## Key Configuration Constants

### domecontrol_JK3.ino
```cpp
#define SMOOTH 30                           // Soft-start smoothness
#define MAX_MOT1_OPEN  6527                 // Motor timeout (107 sec at 61Hz)
const unsigned long connectCheckInterval = 60000UL;   // IP check every 1 min
const byte maxConnectFails = 5;             // Failures to trigger close
const unsigned long maxFailTimeWindow = 300000UL;     // 5-min failure window
```

### cloudwatcher-raincheckerV3.sh
```bash
RAIN_THRESHOLD=2900         # Rain detected when value < this
DRY_COOLDOWN_MINUTES=30     # Minutes before "dry" notification
DOME_IP="192.168.1.177"     # Dome controller IP
CHECK_INTERVAL=10           # Seconds between checks
RAIN_ACTION_COOLDOWN=300    # Seconds after dome close (5 min)
PING_FAIL_THRESHOLD=3       # Consecutive ping failures before alarm
```

## Motor Runtime Tick Logging (v3.3)

Measures motor runtime in ISR ticks for temperature correlation analysis. Purpose: determine temperature-dependent motor timeout coefficients.

### How It Works
1. Arduino counts ticks during motor full-runs (start at one limit, stop at opposite)
2. Only valid measurements are recorded (manual stops, timeouts discarded)
3. When toggle enabled (`$L`), Arduino pushes data to Solo:88 via HTTP GET
4. Pi server logs to CSV with timestamp and ambient temperature

### Tick Logger Server (Cloudwatcher Solo / Pi3)

**Location on Pi (read-only root filesystem):**
- Script: `/usr/local/bin/astroshell_ticklogger.py`
- Service: `/etc/systemd/system/astroshell_ticklogger.service`
- CSV output: `/home/aagsolo/motor_ticks.csv` (tmpfs, lost on reboot)
- Weather data: `/home/aagsolo/aag_json.dat`

**Server Endpoints (port 88):**
| Endpoint | Purpose |
|----------|---------|
| `GET /log?m=1&d=1&t=5234` | Log tick data (m=motor, d=direction, t=ticks) |
| `GET /env` | Get temperature and coefficient for Arduino |
| `GET /status` | Health check |

**Service Management:**
```bash
systemctl start astroshell_ticklogger
systemctl stop astroshell_ticklogger
systemctl status astroshell_ticklogger
journalctl -u astroshell_ticklogger -f
```

**Modifying files on read-only Solo:**
```bash
mount -o remount,rw /
# make changes
mount -o remount,ro /
```

## Development Notes

### Arduino IDE Setup
Upload `domecontrol_JK3.ino` to Arduino MEGA. Built-in libraries only:
- SPI.h, Ethernet.h, EEPROM.h, avr/wdt.h

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

### Location on Pi (read-only root filesystem)
- Script: `/usr/local/bin/cloudwatcher-rainchecker.sh`
- Service: `/etc/systemd/system/cloudwatcher-rainchecker.service`
- Log file: `/home/aagsolo/rainchecker.log` (tmpfs)
- Status flags: `/home/aagsolo/RAINTRIGGERED`, `DRYTRIGGERED`, `PINGALARM`, `PINGFAILCOUNT` (tmpfs)

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
