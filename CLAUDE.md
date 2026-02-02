# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AstroShell Dome Controller - Arduino-based control system for a two-shutter astronomical telescope dome with automatic rain protection via Lunatico Cloudwatcher Solo integration.

## Hardware Configuration

- **Microcontroller**: Arduino MEGA with Ethernet Shield (W5100/W5500)
- **Network**: Static IP 192.168.1.177, monitors Cloudwatcher at 192.168.1.151
- **Motors**: Two DC motors with PWM control (pins 3, 5, 6, 9)
- **Limit switches**: Pins 0, 1, 2, 7 (note: pins 0/1 conflict with Serial, so serial debugging is disabled)
- **Control buttons**: A2-A5 (shutter controls), pin 8 (emergency STOP)

## Architecture

### Arduino Sketch (domecontrol_JK3.ino)

The main firmware runs on Arduino MEGA with these key components:

- **Timer2 ISR**: Handles all real-time motor control, button reading, and limit switch monitoring. Motor PWM with soft-start (SMOOTH=30).
- **IP Auto-Close**: Monitors Cloudwatcher IP every 3 minutes. After 5 failed connections within 8 minutes, dome auto-closes for safety.
- **Web Server**: HTTP interface on port 80 with 10-second auto-refresh. Commands via URL parameters (`$1`-`$5`, `$S`, `$R`).
- **EEPROM Persistence**: Stores IP failure counts and auto-close events across reboots.

**Important quirk**: The limit switch naming is inverted in code due to installation swap. `lim1open` pin actually detects "physically closed" state.

### Rain Checker Script (cloudwatcher-raincheckerV3.sh)

Bash script running on Cloudwatcher Solo device:
- Reads rain sensor JSON from `/home/aagsolo/aag_json.dat`
- Rain detected when value < 2900 threshold
- Sends Pushover notifications and calls dome HTTP API to close shutters
- Uses flag files (`RAINTRIGGERED`, `DRYTRIGGERED`, `PINGALARM`) for state management

## Web API Commands

| Command | Action |
|---------|--------|
| `/?$1` | Close Shutter 1 (East) - physically |
| `/?$2` | Open Shutter 1 (East) - physically |
| `/?$3` | Close Shutter 2 (West) - physically |
| `/?$4` | Open Shutter 2 (West) - physically |
| `/?$5` | Emergency STOP all motors |
| `/?$S` | Get plain text status: "OPEN" or "CLOSED" |
| `/?$R` | Reset persistent counters |

## Development

### Arduino IDE

Upload `domecontrol_JK3.ino` to Arduino MEGA. Required libraries:
- SPI.h (built-in)
- Ethernet.h (built-in)
- EEPROM.h (built-in)

### Debug Modes

Uncomment defines in the Arduino sketch to enable serial debugging (conflicts with limit switch pins 0/1):
```cpp
// #define SERIAL_DEBUG_GENERAL
// #define SERIAL_DEBUG_IP
// #define SERIAL_DEBUG_BUTTONS
// #define SERIAL_DEBUG_LIMITS
// #define SERIAL_DEBUG_EEPROM
```

### Cloudwatcher Script

Deploy to Lunatico Cloudwatcher Solo and run:
```bash
nohup /home/aagsolo/rainchecker.sh > /dev/null 2>&1 &
```

Requires `jq` for JSON parsing (falls back to grep/sed if unavailable).

## Key Configuration Constants

In `domecontrol_JK3.ino`:
- `MAX_MOT1_OPEN/CLOSE`, `MAX_MOT2_OPEN/CLOSE`: Motor timeout values (calibrated for gear oil temperature)
- `connectCheckInterval`: 180000ms (3 min) between IP checks
- `maxConnectFails`: 5 failures needed to trigger auto-close
- `maxFailTimeWindow`: 500000ms (8 min) window for counting failures

In `cloudwatcher-raincheckerV3.sh`:
- `RAIN_THRESHOLD`: 2900 (rain detected when below this value)
- `MAX_LOG_LINES`: 5000 (log rotation limit)
