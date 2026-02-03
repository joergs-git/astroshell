# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Important Rules

- **NO co-authoring lines in commits** - Do not add "Co-Authored-By" lines to commit messages
- **Read before editing** - Always read a file before making changes
- **Preserve code style** - Match existing formatting, indentation, and naming conventions
- **Comments-only commits** - When adding comments/analysis without code changes, state this clearly in the commit message

## Project Overview

AstroShell Dome Controller - Arduino-based control system for a two-shutter astronomical telescope dome with automatic rain protection via Lunatico Cloudwatcher Solo integration.

### Repository Files

| File | Purpose |
|------|---------|
| `domecontrol_JK3.ino` | **Main controller** - Arduino MEGA, enhanced version with network monitoring |
| `controller_wcapacitor_2025_germany.ino` | Alternative controller with capacitor backup (has known bugs, see header) |
| `nocapacitor2023.ino` | Original AstroShell reference code (unmodified) |
| `cloudwatcher-raincheckerV3.sh` | Bash script for Cloudwatcher Solo rain detection |
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
RAIN_THRESHOLD=2900     # Rain detected when value < this
DOME_IP="192.168.1.177" # Dome controller IP
MAX_LOG_LINES=5000      # Log rotation limit
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

### Cloudwatcher Script Deployment
```bash
nohup /home/aagsolo/rainchecker.sh > /dev/null 2>&1 &
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
