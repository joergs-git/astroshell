# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Important Rules

- **NO co-authoring lines in commits** - Do not add "Co-Authored-By" lines to commit messages
- **Read before editing** - Always read a file before making changes
- **Preserve code style** - Match existing formatting, indentation, and naming conventions
- **Comments-only commits** - When adding comments/analysis without code changes, state this clearly in the commit message
- **Use different MAC and IP for testing before git commit and push** - During development and testing ALWAYS use IP `192.168.1.179` (`IP_ADR3 179`) and MAC `{0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEA}` to avoid conflicts with the live production dome at .177/:ED. Before the final commit and push, ALWAYS change back to `IP_ADR3 177` and MAC `{0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}`. The third octet comes from the site profile since v4.2.2 (Germany 1, Portugal 0) and is not a test value. This is a mandatory pre-commit check — never push test IP/MAC to the repository.


## Project Overview

AstroShell Dome Controller - Arduino-based control system for a two-shutter astronomical telescope dome with automatic rain protection via Lunatico Cloudwatcher Solo integration, temperature-based dynamic motor timeout, and frozen dome detection.

### Repository Files

| File | Purpose |
|------|---------|
| `domecontrol_JK3.ino` | **Main controller** - Arduino MEGA v4.3.3, dual-site (Germany mirrored / Portugal standard), safety sensors, dynamic timeout, frozen dome detection, limit fault detection, emergency override |
| `astroshell_ticklogger.py` | **Pi server** - Receives tick data + events from Arduino, logs to CSV, sends Pushover alerts |
| `astroshell_ticklogger.service` | Systemd service file for tick logger autostart |
| `controller_wcapacitor_2025_germany.ino` | Alternative controller with capacitor backup (has known bugs, see header) |
| `nocapacitor2023.ino` | Original AstroShell reference code (unmodified) |
| `cloudwatcher-raincheckerV3.sh` | **Rain checker** - Auto-closes dome on rain, Pushover alerts |
| `cloudwatcher-rainchecker.service` | Systemd service file for rain checker autostart |
| `open_astroshell.bat` | **NINA script** - Opens dome via sequence instruction, Pushover notification |
| `close_astroshell.bat` | **NINA script** - Closes dome via sequence instruction, Pushover notification |
| `motortick-statistic-batchrun.sh` | **Batch runner** - Automated open/close cycles for motor tick statistics |
| `ascom-driver/` | **ASCOM COM driver** - Native IDomeV2 driver for NINA dome control (v1.1.0) |
| `test/host/` | **Logic tests** - Arduino mocks + dome physics; `make -C test/host test` runs 77 scenarios per site profile (87 with both sensor layers built in) (v4.3.3) |
| `.github/workflows/firmware-ci.yml` | **Firmware CI** - logic tests + avr-gcc build of both site profiles (v4.2.0) |
| `README.md` | User documentation with wiring notes |

## Hardware Configuration

- **Microcontroller**: Arduino MEGA 2560 with Ethernet Shield (W5100/W5500)
- **Dome IP**: static, per site profile (v4.2.2): Germany 192.168.1.177, Portugal 192.168.0.177 (`IP_ADR2`)
- **Cloudwatcher IP**: x.x.`IP_ADR2`.`SOLO_IP_ADR3` per site (v4.3.0): Germany 192.168.1.151, Portugal 192.168.0.185 ("astromonte"). Monitored for connectivity and used as tick-logger target; the check is switchable at runtime with `$W`, see Network Monitoring
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
- Limit switch sampling: a level counts only after `LIM_CONFIRM_TICKS` (4) identical samples; every reader uses `limRead()` (v4.3.0). The far end stop of a full run is ignored for its first `LIM_MIN_RUN_TICKS_CLOSE` (4220) / `LIM_MIN_RUN_TICKS_OPEN` (3800) ticks by physical direction, reversals exempt (v4.3.1, per direction since v4.3.3)
- Start-limit release check (v4.3.3): a full run whose start end switch is still active after `LIM_RELEASE_TICKS` (915) loses its full-run status (`m*_run_start_dir = 0`); the check never stops a motor
- Emergency override (v4.3.3): while `m*ForceClose` is set the end stop of the physical CLOSE direction is ignored (`LIM_CLOSED_IGNORED`); the ISR clears the flag as soon as the motor is not closing
- Stale far-end level (v4.3.3): a far-end level that became active while implausible and never dropped (`m*_stale_dir`) is no arrival after the window either. A physical CLOSE from a PROVEN start (`m*_runon_ok`) ignores it to the end of the run and the motor timeout ends it ("run-on", `limRunOnEvent`). Every other run ends at the window as before, but the level that ended it stays doubted (`m*_stale_stop`, `limStaleStopEvent`) until the motor runs with it LOW; the same when the motor is stopped or turned while the level is ignored, when the release check ends a stale phase, and after an override that did not use its whole timeout. Never a valid run
- Known position (v4.3.3): `m*_known_stop` = direction whose end stop the shutter rests at, set by every ISR limit stop (not by a stale one, `LIM_STOP_BOOKKEEPING`), at boot (exactly one end switch active), for a level that TURNS active within `2 * LIM_CONFIRM_TICKS` after the motor stopped on its way there, and by a completed run-on; cleared when the motor runs and that level reads LOW. A full run needs it: a level that appears at rest is no start limit. `m*_known_proven` (the stop was reached by a proven full run of at least `LIM_MIN_RUN_TICKS_CLOSE` ticks; voided by a failed release check) is what run-on needs in addition
- Motor timeout safety (dynamic in Germany, otherwise the static site values `STATIC_TIMEOUT_M*_CLOSE/OPEN`)
- Tick counting for motor runtime measurement
- Frozen dome tick counter (~4 clock cycles overhead when active)

### Sensors (v4.0)
- **DS18B20**: Non-blocking async read every 5 seconds, used for dynamic timeout. **Per site since v4.2.1** (`ENABLE_DS18B20_DYNAMIC_TIMEOUT`, Germany 1, Portugal 0): compiled out, no OneWire/DallasTemperature libraries, static site timeouts, "Temperature: Not installed". **v4.3.0:** the regression itself is gated by `HAVE_TIMEOUT_REGRESSION` (Germany 1, Portugal 0); a probe on Portugal would only show the temperature.
- **VL53L0X**: Continuous mode, read every 200ms, used for frozen dome detection. **Compiled out by default since v4.2.0** (`ENABLE_TOF_FROZEN_DETECTION 0`, no sensor mounted on either dome): no Wire/VL53L0X libraries, no I2C traffic; `tof_connected` stays false so the frozen state machine, `$C`/`$U` and the conflict check are inert. The host test suite runs each site with and without the layer.
- Both sensors degrade gracefully: 10 consecutive failures → feature disabled

### Dynamic Motor Timeout (v4.0)
- Linear regression model from 253 cycle analysis (-1.6C to 22.6C)
- Four separate coefficients for M1/M2 closing/opening
- Integer-only math (no floats): `timeout = base - (slope100 * temp_x10) / 1000 + margin`
- Clamped to [2000, 6527] ticks
- Falls back to the static site values (`STATIC_TIMEOUT_M*`) on temperature sensor failure or where `HAVE_TIMEOUT_REGRESSION` is 0

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
- After **10 failures within 30 minutes**, dome auto-closes
- Fail counters reset after successful Ethernet stack rebuild (prevents carryover)
- First IP check skipped after stack reset (W5100/W5500 stabilization)
- Cable removal triggers **immediate** auto-close
- Uses `netClient.setTimeout(3000)` to prevent watchdog timeout
- No Pushover for IP auto-close (network is down when it triggers)
- **`$W` (v4.2.0)**: the whole block is gated by `remoteCheckEnabled` (EEPROM byte 9, with the site id in bits 1-2). OFF = no polling, no fail counting, no IP or cable-removal auto-close, no tick/interrupt/event pushes; a running auto-close is left alone. `$W1`/`$W0` are idempotent (the character after the W is parsed, so `$W1` is not followed by a `$1` CLOSE command); only a real change resets the failure window; the first real check comes 90 s (+ one skipped check after an Ethernet rebuild) after enabling.

### Event Notifications (v4.0)
- Arduino pushes events to Solo:88/event via HTTP GET
- Solo server sends Pushover alerts (background thread)
- Rate-limited: 10s minimum between events, 5min for conflict events
- Events: frozen_dome, frozen_lockout, frozen_clear, sensor_fail, conflict, limit_glitch (v4.3.2), limit_fault, limit_stuck, force_close (v4.3.3)

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
| `/?$S` | Get status: "OPEN" or "CLOSED"; CLOSED only if both shutters are at the closed stop with their open end switch inactive (a limit fault answers OPEN, v4.3.3) |
| `/?$A` | ASCOM status: `S1_STATE\|S1_MOTOR\|S2_STATE\|S2_MOTOR` (pipe-delimited, v4.0.3); a shutter in limit fault reports INTERMEDIATE (v4.3.3) |
| `/?$R` | Reset EEPROM counters |
| `/?$L` | Toggle tick logging on/off (default: ON after reboot) |
| `/?$U` | Unlock dome from frozen lockout (v4.0) |
| `/?$C` | Calibrate ToF baseline — dome must be fully closed (v4.0) |
| `/?$W` | Toggle Cloudwatcher link check; `/?$W1` = on, `/?$W0` = off (idempotent; the UI button uses these) — persisted in EEPROM byte 9 with the site id; OFF also disables cable-removal auto-close and all pushes to the Solo (v4.2.0) |

| `/?$F1` + `/?$G1`, `/?$F3` + `/?$G3` | Emergency override (v4.3.3): close Shutter 1 (East) / Shutter 2 (West) with its "closed" end switch ignored for that one run; `$F` arms, `$G` for the same shutter within 30 s (`FORCE_ARM_MS`) starts, STOP or OPEN disarm; ends at STOP, an OPEN command or the close timeout. Supervised use only, never sent by any script |

**Note:** The web API, `$S`, `$A` and the web page are physical on both sites; the site profile resolves the wiring (see Site Profile and Physical Semantics).

### Limit Fault and Emergency Override (v4.3.3)
- The limit switches pull their inputs LOW while the shutter is NOT at the stop (INPUT_PULLUP, active HIGH): a broken wire, loose terminal or dead switch reads as "end stop reached". Both end switches of one shutter active is therefore a **limit fault** (`shutter1State()` / `shutter2State()` return `SH_FAULT`): page "LIMIT FAULT", `$S` OPEN, `$A` INTERMEDIATE, `limit_fault` event after 1 s (`reportLimitFaults()`: per shutter, once per episode, kept queued if the episode ends first, at most one per 5 min). Normal commands, panel buttons and the auto-close do not move a shutter in that state (their guards are the raw end switch levels).
- `$F1` / `$F3` arm (`forceCloseArm()`, `forceArmShutter`, one shutter at a time), `$G1` / `$G3` start (`forceCloseConfirm()` -> `forceCloseShutter()`) if that shutter was armed at most `FORCE_ARM_MS` (30 s) ago: the way to close when a "closed" line reads active at rest. The page shows CONFIRM NOW (the `$G` link) while armed; `$5`, an OPEN for that shutter and the panel STOP (in the ISR's STOP branch, so a short press counts while `loop()` is blocked) disarm, `forceArmExpire()` forgets an unconfirmed arming, non-GET requests are ignored for both, `$G` does nothing while the panel STOP is held. The two requests differ on purpose: any repetition of the arming request (double click, a client that retries a GET, a request that waited while `loop()` was blocked) only arms again. A first version confirmed with a second `$F` and was started by two GETs sent 48 ms apart that the firmware processed 1.5 s apart. `forceCloseShutter()` sets `m*ForceClose` together with the motor start in one critical section; the run is never a measured run (no tick log), a close that is already running keeps its remaining timeout. The flag is cleared by the ISR when the motor is not closing any more and by `startMotor*()` / `fdStartMotor()` on every new run. While it is set, while a level is stale (`m*_stale_dir`) and after a stale level ended or outlived a run (`m*_stale_stop`, until it drops), `shutter*State()` does not believe that level: such a shutter is reported INTERMEDIATE, never CLOSED. An override that used its whole timeout is taken as closed (`m*_known_stop`); one that was cut short leaves the "closed" level doubted, so after STOP the page shows the limit fault and `$S` answers OPEN until the switch has let go of a moving shutter. The character after the F is consumed like `$W1` (`forceClosePending`), so `$F1` is not followed by a normal `$1`.
- A "closed" line that opens DURING a close (the Portugal pattern): after a valid full opening the close runs on to its timeout and the Solo gets `limit_fault` "S2 closed switch stuck, closed on timeout". Without that proven start (first close after a boot with the dome open, after a short run, after any run that did not end as a valid full run) the close ends at the window, the page shows "LIMIT FAULT ('closed' end switch not trusted)" with the override button, `$S` answers OPEN and the Solo gets `limit_fault` "S2 closed switch not trusted, close stopped". Reason for the gate: with a FALSE start position a TRUE early "closed" level would be run past, up to the whole timeout against the stop (review round 2: 127 s).
- Not detectable from two switch levels: only the "closed" line going HIGH with the shutter mid-travel and the motor off, with no ignored level and no override before, reads as "Physically CLOSED". The override covers it (the operator sees the open shutter).
- Not done, by decision of scope (2026-11-21, proposed to the user): a persistent "closed switch not trusted" lock. Today an OPEN is still accepted with a "closed" switch that the firmware has just seen stuck (run-on, override, `limit_stuck`); the dome then ends in a limit fault at the open stop and only the override closes it.
- `reportLimitFaults()` sends its events urgent and spaces them itself (10 s after a delivered one, 60 s doubling to 16 min after a failed one; anything new in the queue is tried at once and resets the pause); order: both-switches fault, doubted close, run-on, `force_close`, `limit_stuck`.
- Bench without limit switches: unconnected limit inputs are HIGH, i.e. a limit fault on that shutter and `$S` = OPEN.

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

## Site Profile and Physical Semantics (v4.2.0)

Two domes run this source. Select the site at the top of `domecontrol_JK3.ino` (`#define SITE SITE_GERMANY`, the default) or override with `-DSITE=2` on the command line (CI builds both).

| | `SITE_GERMANY` (mirrored install) | `SITE_PORTUGAL` (original AstroShell wiring) |
|---|---|---|
| `LIMIT_WIRING_SWAPPED` | 1 | 0 |
| Direction `OPEN` (PWM motor*b, ends at `lim*open`) is physically | closing | opening |
| `lim1open` (pin 7) / `lim2open` (pin 1) HIGH | physically CLOSED | physically OPEN |
| Panel button "up" (SW*up) | closes | opens |
| `DEFAULT_REMOTE_CHECK` (Cloudwatcher check until `$W` is used) | 1 | 1 (since v4.3.0: Solo on site) |
| `IP_ADR2` dome IP / `SOLO_IP_ADR3` Solo IP (v4.3.0) | 1 → 192.168.1.177 / .1.151 | 0 → 192.168.0.177 / .0.185 |
| `ENABLE_DS18B20_DYNAMIC_TIMEOUT` (temperature probe, v4.2.1) | 1 | 0 (static site timeouts) |
| `HAVE_TIMEOUT_REGRESSION` (v4.0 coefficients apply, v4.3.0) | 1 | 0 (a probe would only show the temperature) |
| `STATIC_TIMEOUT_M1_CLOSE/OPEN`, `M2_CLOSE/OPEN` (ticks, v4.3.0) | 6527 / 6527, 6527 / 6527 | 7320 / 6527, 7930 / 6527 (120 s / 107 s, 130 s / 107 s) |
| `ENABLE_TOF_FROZEN_DETECTION` (VL53L0X, global default) | 0 | 0 |

Rules:
- The hardware pairing never changes: the ISR stops direction `OPEN` at `lim*open` and `CLOSE` at `lim*closed`; the PWM pin mapping is fixed.
- Limit switches are read only through `limRead()` (level confirmed over `LIM_CONFIRM_TICKS` = 4 ISR ticks, v4.3.0). The raw pins are read only in `limSample()` (ISR) and once in `setup()`. Never add a `digitalRead()` of a limit pin anywhere else. The ISR limit stops also apply `LIM_FAR_END_IMPLAUSIBLE()` (v4.3.1): a full run cannot reach its far end within `LIM_MIN_RUN_TICKS_CLOSE` / `LIM_MIN_RUN_TICKS_OPEN` (v4.3.3: 4220 / 3800 = 80 % of the fastest full close / open measured on either dome, 5276 / 4753 ticks); keep them below the fastest full run of their direction, a shutter that arrives earlier pushes against its stop until the value is reached.
- Everything in `loop()` that asks "is this shutter open / closed" (status answers, web page, calibration gate, conflict check) uses `shutter1State()` / `shutter2State()` (v4.3.3: `SH_INTERMEDIATE`, `SH_OPEN`, `SH_CLOSED`, `SH_FAULT`, one snapshot of both levels). The raw `limRead(LIM*_PHYS_*)` level stays the right test for "may a motor start towards this stop" (commands, auto-close, frozen cycle) and in the network gates.
- **Everything that means "physically open/closed" must use** `DIR_PHYS_CLOSE` / `DIR_PHYS_OPEN` / `LIM1_PHYS_CLOSED` / `LIM1_PHYS_OPEN` / `LIM2_PHYS_CLOSED` / `LIM2_PHYS_OPEN` (plus the `*_NAME` strings and `WIRING_LABEL` for UI text). Raw `OPEN`/`CLOSE`/`lim*` are allowed only in the defines, the PHYSICAL SEMANTICS block, `pinMode`, the ISR limit stops, the ISR PWM output, the ISR stop-reason housekeeping and the ISR panel buttons (which use `BUTTON_MAY_START(dir)` and `TIMEOUT_M*_FOR(dir)`).
- The web API is physical on both sites: `$1`/`$3` close, `$2`/`$4` open; `$S`, `$A` and the web page report physical state. The ASCOM driver needs no site setting ("Swap Open/Close" stays unchecked on both domes).
- Before v4.2.0 the mirrored interpretation was hard-coded in 109 places. Flashed on a standard-wired dome, the IP-fail auto-close drove the shutters to the physically OPEN end stop. Never add a raw limit/direction test outside the allowed places; `make -C test/host test` runs every scenario for both sites and catches it.

## EEPROM Memory Map

| Offset | Size | Content |
|--------|------|---------|
| 0 | 1 byte | Magic byte (0x42) |
| 1-2 | 2 bytes | Total IP failures (uint16) |
| 3-4 | 2 bytes | Auto-close events (uint16) |
| 5 | 1 byte | Uptime day counter |
| 6-7 | 2 bytes | ToF baseline distance in mm (v4.0) |
| 8 | 1 byte | ToF calibration magic (0xA5) (v4.0) |
| 9 | 1 byte | Config flags (v4.2.0): bit 0 = Cloudwatcher check enabled; bits 1-2 = SITE that wrote the byte (another SITE re-applies its default); bit 7 set (0xFF, never written) = apply site default and persist |

## Key Configuration Constants

### domecontrol_JK3.ino
```cpp
#define SITE SITE_GERMANY                   // Site profile: SITE_GERMANY (mirrored) or SITE_PORTUGAL (standard), v4.2.0
#define ENABLE_TOF_FROZEN_DETECTION 0       // v4.2.0: ToF/I2C layer compiled out until a VL53L0X is mounted
#define ENABLE_DS18B20_DYNAMIC_TIMEOUT 1    // v4.2.1: per site (Germany 1, Portugal 0); 0 = OneWire layer out, static site timeouts
#define HAVE_TIMEOUT_REGRESSION 1           // v4.3.0: per site (Germany 1, Portugal 0); 0 = probe shows temperature only
#define STATIC_TIMEOUT_M1_CLOSE 6527        // v4.3.0: per site and physical direction; Portugal 7320 / 6527 / 7930 / 6527
#define LIM_CONFIRM_TICKS 4                 // v4.3.0: identical ISR samples before a limit level is accepted (~66 ms)
#define LIM_MIN_RUN_TICKS_CLOSE 4220        // v4.3.3 (one value 2440 in v4.3.1): far end stop of a full CLOSE ignored before this (~69 s), reversals exempt; ignored episodes counted (limGlitch*, v4.3.2)
#define LIM_MIN_RUN_TICKS_OPEN  3800        // v4.3.3: the same for a full OPEN (~62 s)
#define LIM_RELEASE_TICKS 915               // v4.3.3: the start end switch of a full run must have released by then (15 s), else no proven full run (limStuck*)
#define FORCE_ARM_MS 30000UL                // v4.3.3: $G1 / $G3 must follow the arming $F1 / $F3 within this time
#define LIMIT_FAULT_CONFIRM_MS 1000UL       // v4.3.3: both end switches active for this long before limit_fault is queued
#define SMOOTH 30                           // Soft-start smoothness

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
const unsigned long maxFailTimeWindow = 1800000UL;    // 30-min failure window
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
| `limit_glitch` | Normal | A far-end limit level ignored by the plausibility rule (v4.3.2): "M2 at 212 ticks (3 since boot)"; also shown as "Ignored limit levels" on the web page |
| `limit_fault` | High + siren | Both end switches of a shutter active for 1 s (v4.3.3): "S2 both end switches active"; position unknown, normal commands refused, the dome may be open. Also "S2 closed switch stuck, closed on timeout" (a close ran on past a stale "closed" level to its timeout) and "S2 closed switch not trusted, close stopped" (a close ended at the window on such a level: the shutter may be open) |
| `limit_stuck` | Normal | The end switch a full run started from did not release within 15 s (v4.3.3): "M2 start limit not released (1 since boot)"; stuck switch line or shutter not moving; web row "Start limit not released" |
| `force_close` | Normal | Emergency override started (v4.3.3): "S2 forced close started" |

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
- OneWire.h, DallasTemperature.h (install via Library Manager) — only when `ENABLE_DS18B20_DYNAMIC_TIMEOUT` is 1 (Germany profile; Portugal compiles the layer out since v4.2.1)
- Wire.h (built-in), VL53L0X.h (Pololu, install via Library Manager) — only when `ENABLE_TOF_FROZEN_DETECTION` is 1 (default 0 since v4.2.0, then not needed)

### Host Logic Tests (v4.2.0)
`make -C test/host test` compiles the unchanged `.ino` against the mocks in `test/host/mock/` and runs the scenario suite for each site profile in three sensor configurations: default (as flashed), both sensor layers built in, both compiled out (six binaries; one forked process per scenario; a physics model decides what is physically open/closed per site, independently of the firmware). Run it before every firmware commit. CI (`.github/workflows/firmware-ci.yml`) runs it plus an avr-gcc build of both profiles with `-DSITE=1/2`. `./test/host/build/test_site2 frozen` runs only scenarios whose name contains "frozen"; `make -C test/host test FW=/path/old.ino` tests another firmware revision.
Harness facts: the ISR debounce needs more than 6 ticks between panel-button presses (`pressButton()` waits 8); at a limit switch the ISR clears every stop reason except 3/4 on the next tick, so never assert stop reason 6 at an end stop; the first IP check after boot is skipped, so the 10th failure arrives about 16.5 minutes after the dome left the closed position. Since v4.3.0 a limit level is confirmed `LIM_CONFIRM_TICKS` (4) ticks after the pin changes: after `setPos()` on a booted firmware run `runTicks(LIM_CONFIRM_TICKS)` before asserting a state, a motor stops up to 4 ticks after the physical end stop (logged tick counts are up to 4 higher than `g_travel`), and `S[i].closedGlitchTicks` / `openGlitchTicks` inject spurious HIGH samples on a limit input. The test file carries fallback defines so `FW=<pre-4.3.0 .ino>` still compiles; the glitch and site-timeout scenarios then fail on purpose. Since v4.3.3 `S[i].closedStuck` / `openStuck` hold a limit input HIGH wherever the shutter is (open-circuit line); the 15 limit-fault / override / window / release scenarios all fail against v4.3.2 (`PRE_V433` guards the lines that need new symbols). The mock `Arduino.h` maps `PSTR`, `strcpy_P` and `snprintf_P` to their RAM versions. `forceClose(n)` sends `$F` and `$G` for that shutter (arm + start). A level set with `setPos()` or a `*Stuck` flag AFTER boot is a level that appears at rest: it is no start limit (drive the shutter there, or boot there, for a full run). The events of `reportLimitFaults()` are urgent and spaced 10 s apart by the function itself, so the second one of a run arrives about 10 s after the first. A run-on needs a proven start: let the shutter make a full valid run to the stop first (boot at the stop is only "known"). Two more checks belong to every change of the limit handling: 33 of the 35 v4.3.3 scenarios must fail against v4.3.2 (`make -B -C test/host test FW=<v4.3.2.ino>`; `-B` because make does not see the FW change), and a random command sequence without switch faults must give the same trace on the old and the new firmware (differential harness of the 2026-11-21 review, three modes including commands timed around arrivals: 2400 runs tick-exact identical; randomized stress with a model of the arming rules: 1200 runs).

### Toolchain Note (Apple Silicon Mac)
The Arduino IDE's bundled avr-gcc is x86_64. Without Rosetta neither the IDE nor `arduino-cli` can compile here ("bad CPU type in executable"); install Rosetta (`softwareupdate --install-rosetta --agree-to-license`) or rely on the CI build. Bundled CLI: `/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli`. Compile from a folder named like the sketch (`domecontrol_JK3/domecontrol_JK3.ino`), never from the repo root (it holds three `.ino` files).

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

1. **DS18B20**: Verify temperature on web UI. Unplug → confirm "Not connected" and the static site timeouts ("Timeout Mode: Static (site profile)")
2. **VL53L0X**: Verify distance on web UI. Move hand → see distance change. Unplug → frozen detection shows "Disabled (sensor disconnected)"
3. **ToF calibration**: Close dome → click "Calibrate ToF Baseline" → verify baseline stored → reboot → verify persists
4. **Frozen dome simulation**: Block ToF sensor during open → verify stop + reverse + 3 retries + lockout
5. **Unlock**: After lockout → click "UNLOCK DOME" → verify open commands work again
6. **Graceful degradation**: Disconnect both sensors → verify system behaves identically to v3.3
7. **Lab testing without limit switches**: Connect pin 2 (lim1closed) to GND to allow M1 open commands. Pin 7 stays HIGH (pullup) simulating closed position. Close phase is instant in this setup. (Germany profile; in the Portugal profile the roles of pins 2 and 7 swap.) Since v4.3.3 a shutter with both limit inputs unconnected shows LIMIT FAULT and `$S` answers OPEN; tie one input of each shutter to GND for a "closed" bench state.
8. **Limit fault and override (v4.3.3)**: dome mid-travel, pull the "closed" end switch wire of one shutter and drive it to the open stop: page must show LIMIT FAULT, `$S` OPEN, CLOSE refused. `$F1`/`$F3` must only arm, however often it is sent (page: "Override ARMED", button CONFIRM NOW); `$G1`/`$G3` within 30 s must start the close ("FORCED, end switch ignored"); press STOP when the shutter is closed: the page then shows "LIMIT FAULT ('closed' end switch not trusted)" until the switch works again. Never leave a forced run unattended.

### Portugal Commissioning Checklist (v4.2.0)
1. IDE board = Arduino Mega or Mega 2560 (an "Arduino Uno" selection fails with "Sketch too big", 32256 bytes). Build with `#define SITE SITE_PORTUGAL` (or `-DSITE=2`), flash, open the web UI: System Status must show "Limit Wiring: Standard (original AstroShell)", IP Monitoring must show the site default "Cloudwatcher Check: ENABLED" (v4.3.0; a `$W` choice stored by an earlier build of the SAME site wins, `$W1`/`$W0` set it), Sensors must show "Temperature: Not installed".
2. Dome in intermediate position: "OPEN S1" must physically open S1, the page must say "Opening (physically)", `$A` must say `INTERMEDIATE|OPENING|...`. Then "CLOSE S1".
3. Run one shutter to each end stop: state line "Physically OPEN/CLOSED" and the pin rows (S1 phys. closed = pin 2, phys. open = pin 7) must match.
4. ASCOM driver: "Swap Open/Close" unchecked; NINA must show the real shutter state.
5. Keep "Cloudwatcher Check" DISABLED until a Solo exists at the site's `SOLO_IP_ADR3` (Portugal: "astromonte" 192.168.0.185 since 2026-11-02, default ON); enabled without one, the dome closes about 16 minutes after every opening.
6. Run one full open and close per shutter and read "Last Valid Full-Runs": every value must stay well below the site's `STATIC_TIMEOUT_M*` for that direction (Portugal, 2026-11-02: East close 5877-5943, West close > 6527 ticks after a night open).

### Important Implementation Details (v4.0)

These patterns were discovered during lab testing and are critical for future modifications:

- **ISR vs loop() race condition**: The frozen dome state machine runs in `loop()` and uses its own `fd_prev_mot1dir`/`fd_prev_mot2dir` static locals to detect motor start transitions. Do NOT use the ISR's `m1_prev_dir`/`m2_prev_dir` — the ISR updates them at 61 Hz before `loop()` runs, so `loop()` never sees the 0→CLOSE transition.
- **Sensor hot-plug re-detection**: Both DS18B20 and VL53L0X have 30-second periodic re-detection loops that re-scan the bus after 10 consecutive failures. Without this, unplugging and reconnecting a sensor requires a reboot.
- **DS18B20 startup delay**: A `delay(100)` before `ds18b20.begin()` in `setupDS18B20()` is required — the sensor needs time to boot after power-on. Without it, detection fails at startup and takes ~80 seconds via the re-detection path.
- **DS18B20 pullup resistor**: 4.7kΩ required (5.1kΩ also works). Without pullup, OneWire open-drain bus cannot communicate.
- **URL encoding**: Event detail strings sent to the Solo must be URL-encoded (spaces → `%20`). Python's `BaseHTTPRequestHandler` rejects URLs with literal spaces (HTTP 400). Encoding is done in a local buffer before `print()`, not char-by-char via `write()` (which caused TCP issues on W5100/W5500).
- **Python stdout buffering**: The ticklogger service file must use `python3 -u` (unbuffered) or `print()` output won't appear in `journalctl`. The `-u` flag is set in `astroshell_ticklogger.service`.
- **Never fake a non-zero timestamp with `| 1`** (v4.3.0 review): `linkOffSince = millis() | 1` is 1 ms in the future on even millis, `millis() - linkOffSince` wraps and the 200 ms LinkOFF filter was bypassed half of the time. Use `if (stamp == 0) stamp = 1` or a separate flag.
- **Confirmed limit levels lag a motor start by LIM_CONFIRM_TICKS** (v4.3.0 review): a decision like "close everything that is not closed" must treat a motor that is already moving away from the stop explicitly (`mot*dir == DIR_PHYS_OPEN || (mot*dir == 0 && !closed)`), otherwise it skips a shutter that started opening 66 ms ago. The reversal itself needs `m*LimitHold` (ISR limit stop suppressed for `LIM_CONFIRM_TICKS`), or the stale level ends it at once with the shutter a millimetre open.
- **Rate-limiter timestamps start at 0** (v4.2.0): a plain `now - last < interval` test with `last == 0` suppresses the first event after boot (events for 10 s, conflict events for 5 min). Treat 0 as "never sent".
- **Stop reasons at limit switches**: once a motor is off at a limit, the ISR clears every stop reason except 3 (with the auto-close flag) and 4 on the next tick. Reason 6 (frozen) is therefore only visible mid-travel.
- **Counted refactors** (v4.2.0): the 109 wiring-sensitive sites were changed by a script that asserted the exact number of occurrences of every replaced snippet per function and refused to write on any mismatch. Use the same approach for future sweeps; then run `make -C test/host test`. (v4.3.0: the 62 limit reads were swept the same way.)
- **Limit confirmation (v4.3.0) and far-end plausibility (v4.3.1)**: Portugal, 2026-11-02: three West closes from fully open, two stopped at 252 and 212 ticks (4.1 s / 3.5 s) by a HIGH on pin 0 (`lim2closed`, shares RX0) that the firmware logged as *valid* closes; the second one happened with the 4-tick confirmation already active, so the level lasted > 66 ms, and the operator saw it coincide with the release of the OPEN switch (pin 1). The clean run took 5485 ticks (89.9 s). The German 253-cycle CSV has no such run. USB is never connected to the Portuguese MEGA in operation; suspect the West limit wiring (shared/loose line between the two switches). The night failure of 2026-11-02 05:58 may have been this glitch as well (NINA's state poll adds up to ~6 s of latency), not necessarily the timeout.
- **Portugal 2026-11-20, West shutter could not be closed (v4.3.3)**: the page showed pin 0 and pin 1 both HIGH, "Physically CLOSED" next to stop reason "Limit 'Phys. Open'" (the state line tested "closed" first, the stop reason "open" first), "Last Valid Full-Runs" West close 3268 ticks and 8 ignored limit levels, the last at 2191 ticks. The close guard `!limRead(LIM2_PHYS_CLOSED)` refused every close, `$S` answered CLOSED. The fault is in the pin 0 line (West "closed" switch, wire, terminal or the common return); pin 0 also hangs on the USB-serial chip's TX through 1 kOhm, so a few hundred ohms of contact resistance already read as HIGH there (assumption from the MEGA schematic, to be measured on site: pin 0 to GND must be below 0.5 V with the shutter away from the closed stop).
- **A state derived from two inputs needs its "both active" case spelled out** (v4.3.3): four independent `if (closed) ... else if (open)` chains disagreed about the same pair of levels. Use `shutter*State()`.
- **One-run flags must be cleared where runs start, not only where they end** (v4.3.3): `/?$5$1` stops and restarts a motor inside one request, between two ISR ticks; the ISR never sees `mot*dir == 0`. `m*ForceClose` is therefore cleared in `startMotor*()` / `fdStartMotor()` as well as in the ISR. Start motors from `loop()` only through these helpers.
- **A try-once event is lost when another event went out less than 10 s before** (v4.3.3): `limit_stuck` follows the `limit_glitch` of the same run by about 2 s. Events that must arrive are retried with their own interval (`reportLimitFaults()`), and only `sendEventNotification()` returning true marks them as sent; never retry on every `loop()` pass (a dead Solo costs 1.5 s per connect).
- **The end of a filter window is not an event** (v4.3.3 review): the far-end rule ignored a false "closed" level for 4220 ticks and then took the very same, still active level as the arrival: the close ended at the window with the shutter a quarter open, reported CLOSED and logged valid (v4.3.1 did the same at 2440). A level that was already active while it could not be true stays false until it has dropped once (`m*_stale_dir`). For the direction that must complete (CLOSE) the run goes on to its timeout.
- **A rule that trusts a start condition needs that condition proven** (v4.3.3 review): "started at the opposite end stop" was read from the momentary level; a level that appeared at rest mid-travel and dropped within the release time made the next run ignore its real end stop for the whole window (up to 58 s against the stop, and the larger windows had doubled that). `m*_known_stop` proves it by how the level came about.
- **An "ignore this input" rule multiplies the cost of a false premise** (v4.3.3 review, round 2): run-on from a merely "known" position turned a TRUE early "closed" level into up to 127 s against the stop when the position was false (a line open at boot, or one that had stopped an opening half way). Grade the evidence: `m*_known_stop` gets the window, only `m*_known_proven` (a proven full run ended there) gets the run-on, and whatever ends a run on an ignored level leaves that level doubted (`m*_stale_stop`), so the state never flips to CLOSED on it.
- **A confirmation must be a different request than the one it confirms** (v4.3.3 review, round 3): "the same GET a second time, 1 to 30 s later" was started by a duplicate (two GETs 48 ms apart, processed 1.5 s apart behind a blocked `loop()`), and HTTP clients repeat GETs on their own. `$F` arms, `$G` starts.
- **Doubt must outlive what created it** (v4.3.3 review, round 3): the doubted level was cleared by a 6-tick dip of the line at rest, and a failed release check left `m*_known_proven` standing. State that expresses distrust falls only on positive evidence (the motor runs and the switch lets go), never on the absence of the symptom.
- **A test model must use the time the firmware sees** (v4.3.3 review, round 2): the stress model judged the 1 s / 30 s arming window by the time a request was issued and reported violations; the firmware had processed that request 1.5 s later, after a blocked `loop()` pass. Evaluate such models after the call returns, and treat the edges of a window as undecided.
- **Compare a safety change with the previous release on random healthy sequences** (v4.3.3 review): a first version of the known-position rule changed healthy behaviour in 68 of 480 random runs (a STOP in the 4 ticks in which the confirmed level was still catching up at arrival); the scenario suite did not see it. The differential harness did, and showed 0 of 1200 after the fix. A command that can push a motor against a stop also needs a second step: GET links are fetched by preloaders and previews.
- **Event texts belong in flash** (v4.3.3): a plain string literal passed as `const char*` lives in RAM on AVR (the first draft of the three new events cost 216 B); build the detail with `strcpy_P` / `snprintf_P` and `PSTR`.
- **Portugal night close (2026-11-02 05:58 local)**: NINA's AscomDome.CloseShutter waits 3 s and then polls once per second while the state is ShutterClosing, without an own timeout; the wait ended 107.24 s after the call, i.e. at the firmware's 6527-tick timeout (106.9 s), with East closed (5943 ticks) and West intermediate. The ASCOM driver reports ShutterError only when both motors are stopped and the shutters are not both closed or both open, so a NINA ShutterError always means a motor stopped early.
