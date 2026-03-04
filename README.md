# AstroShell Dome Controller

**Enhanced Arduino fail-safe firmware for AstroShell telescope domes with automatic rain protection, temperature-based dynamic motor timeout, and frozen dome detection**

[![Platform](https://img.shields.io/badge/Platform-Arduino%20MEGA-blue)]()
[![Version](https://img.shields.io/badge/Version-4.0-green)]()
[![License](https://img.shields.io/badge/License-Free%20to%20Use-green)]()

> Also check out the [Wiki](https://github.com/joergs-git/astroshell/wiki) for additional documentation.

---

## Key Features

| Feature | Description |
|---------|-------------|
| **Automatic Rain Protection** | Closes dome automatically when rain detected via Cloudwatcher Solo |
| **Network Failsafe** | Auto-closes if Cloudwatcher becomes unreachable (10 failures in 15 min) |
| **Cable Removal Detection** | Instant dome closure if Ethernet cable is disconnected |
| **Dynamic Motor Timeout** | Temperature-based timeout from 253-cycle regression analysis (v4.0) |
| **Frozen Dome Detection** | VL53L0X ToF sensor detects frozen halves, auto-retries with lockout (v4.0) |
| **Event Notifications** | Pushover alerts for frozen dome, sensor failures, signal conflicts (v4.0) |
| **Mobile-Friendly Web UI** | Responsive design works on phones, tablets, and desktops |
| **Hardware Watchdog** | 8-second watchdog ensures automatic recovery from hangs |
| **Motor Runtime Logging** | Tick-based logging with temperature and ToF data for analysis |

---

## Safety Features at a Glance

```
Telescope Protection Priority:

  Rain Detected          Cloudwatcher        Ethernet Cable       Dome Halves
       via                Unreachable           Removed            Frozen (v4.0)
   Cloudwatcher           (15 min)            (Instant)           (Detected by ToF)
       |                     |                    |                    |
       v                     v                    v                    v
   +-----------------------------------------------+   +--------------------+
   |           AUTOMATIC DOME CLOSURE              |   | 3x Retry + Lockout |
   +-----------------------------------------------+   +--------------------+
                         |                                       |
                         v                                       v
              Pushover Notification                   Pushover Notification
                    to Owner                               to Owner
```

---

## What's New in v4.0

| Feature | Description |
|---------|-------------|
| **DS18B20 Temperature Probe** | Ambient temperature on pin 22 for dynamic timeout computation |
| **Dynamic Motor Timeout** | Linear regression model replaces static 107s timeout — adapts to temperature |
| **VL53L0X ToF Sensor** | Measures gap between dome halves via I2C (pins 20/21) |
| **Frozen Dome Detection** | 3 retry attempts with 20s gravity wait, then lockout |
| **ToF Calibration** | `/?$C` stores current closed distance in EEPROM |
| **Dome Unlock** | `/?$U` clears frozen lockout |
| **Event Notifications** | Arduino pushes events to Solo:88, Solo sends Pushover alerts |
| **Conflicting Signals** | Detects limit switch vs ToF disagreement (rate-limited alerts) |
| **Graceful Degradation** | All new features auto-disable if sensors disconnected |
| **Sensor Hot-Plug** | DS18B20 and VL53L0X auto-reconnect within 30 seconds after unplug/replug |
| **Tick Logging Default ON** | Motor runtime logging enabled by default after reboot |

---

## Quick Start

1. **Install Libraries** in Arduino IDE Library Manager:
   - `OneWire` by Jim Studt
   - `DallasTemperature` by Miles Burton
   - `VL53L0X` by Pololu
2. **Upload** `domecontrol_JK3.ino` to Arduino MEGA
3. **Configure** IP addresses in the code (dome: 192.168.1.177, Cloudwatcher: 192.168.1.151)
4. **Wire sensors** (see Hardware Setup below)
5. **Calibrate ToF** — close dome, navigate to web UI, click "Calibrate ToF Baseline"
6. **Deploy** tick logger + event server to Cloudwatcher Solo (optional but recommended)
7. **Access** web interface at `http://192.168.1.177`

8. **Addons:** If you want to have it perfect I recommend you also use the batch files to control dome from NINA and also install the cloudwatcher-rainchecker and astroshell-ticklogger services on your cloudwatcher solo device. After that you're really super set. Also I recommend you registering a Pushover API account for messaging services if you don't already have one. Just have a look into the different files where you can see further explanations and howtos.

---

## Hardware Setup

### Required Components

- **Controller:** Arduino MEGA 2560 — **use original Arduino only** (see warning below)
- **Network:** W5500 Ethernet Shield
- **Weather:** Lunatico Cloudwatcher Solo (optional but recommended)
- **Temperature:** DS18B20 waterproof probe (v4.0)
- **Distance:** VL53L0X Time-of-Flight sensor breakout (v4.0)

### Important: Arduino Board Compatibility

The original AstroShell kit ships with an **Arduino UNO R3**. This project upgraded to the **Arduino MEGA 2560** because the UNO's 2KB SRAM and 32KB flash are insufficient for the Ethernet web server, HTML UI, and v4.0 sensor features (DS18B20, VL53L0X). The MEGA also provides dedicated I2C pins (20/21) and additional GPIO (pin 22+) that don't conflict with existing wiring.

> **Warning: Always use an original Arduino MEGA 2560 R3.** Clone boards from manufacturers like AZ-Delivery (CH340 chip) or Keyestudio (CP2102 chip) can cause unpredictable failures. The Shutter 2 (West) limit switches use pins 0 and 1, which are shared with the USB-serial chip. The original Arduino MEGA has an ATmega16U2 with a series resistor that allows limit switches to function correctly on these pins. Clone boards often drive pins 0/1 directly from their USB-serial chip, overpowering the limit switch signals. This causes symptoms like: West shutter refusing to open (code thinks it's already at the open limit) while closing still works, or both shutters behaving erratically. This is a hardware-level issue — no code fix is possible.

### Sensor Wiring (v4.0)

#### DS18B20 Temperature Probe (Pin 22)
```
Arduino MEGA Pin 22 ──── DS18B20 Data (yellow/white wire)
                    │
                    ├── 4.7kΩ pullup to 5V
                    │
Arduino 5V ──────── DS18B20 Vcc (red wire)
Arduino GND ─────── DS18B20 GND (black wire)
```
- Use external Vcc power (not parasitic mode) for reliability
- **4.7kΩ pullup resistor required** between Data and 5V (5.1kΩ also works)
- Without the pullup, the OneWire open-drain bus cannot communicate
- Pin 22 is MEGA-exclusive — guaranteed no conflict with existing wiring

#### VL53L0X Time-of-Flight Sensor (I2C)
```
Arduino MEGA Pin 20 (SDA) ──── VL53L0X SDA
Arduino MEGA Pin 21 (SCL) ──── VL53L0X SCL
Arduino 3.3V ────────────────── VL53L0X VIN
Arduino GND ─────────────────── VL53L0X GND
```
- Mount across the gap between dome halves at a point where movement is visible
- Closed distance will be calibrated via web UI — no need to hardcode

### ToF Sensor Mounting

The VL53L0X should be mounted so it measures the distance across the gap between the two dome halves. When the dome opens, the gap increases. The exact mounting position doesn't matter — calibration stores whatever "closed" distance your installation produces.

**Tips:**
- Mount on the fixed part of the dome frame, pointing at the moving half
- Protect from rain/condensation with a small shield or enclosure
- Test by manually moving the dome — distance should increase clearly when opening

---

## Repository Files

| File | Description |
|------|-------------|
| `domecontrol_JK3.ino` | Main Arduino firmware (v4.0) with safety sensors, dynamic timeout, frozen dome detection |
| `cloudwatcher-raincheckerV3.sh` | Rain checker script for Cloudwatcher Solo - auto-closes dome on rain |
| `cloudwatcher-rainchecker.service` | Systemd service for rain checker autostart |
| `astroshell_ticklogger.py` | Motor runtime & event server - receives tick data + events, logs CSV, sends Pushover |
| `astroshell_ticklogger.service` | Systemd service for tick logger autostart |
| `open_astroshell.bat` | Windows batch script for NINA - opens dome with Pushover notification |
| `close_astroshell.bat` | Windows batch script for NINA - closes dome with Pushover notification |
| `motortick-statistic-batchrun.sh` | Automated dome open/close cycles for motor tick statistics collection |
| `controller_wcapacitor_2025_germany.ino` | Alternative firmware with capacitor backup (has known bugs) |
| `nocapacitor2023.ino` | Original unmodified AstroShell reference code |

---

## Web API Commands

| Command | Action |
|---------|--------|
| `/?$1` | CLOSE Shutter 1 (East) |
| `/?$2` | OPEN Shutter 1 (East) — blocked during frozen lockout |
| `/?$3` | CLOSE Shutter 2 (West) |
| `/?$4` | OPEN Shutter 2 (West) — blocked during frozen lockout |
| `/?$5` | Emergency STOP all motors |
| `/?$S` | Get plain text status: "OPEN" or "CLOSED" |
| `/?$R` | Reset EEPROM counters |
| `/?$L` | Toggle tick logging on/off (default: ON) |
| `/?$U` | Unlock dome from frozen lockout (v4.0) |
| `/?$C` | Calibrate ToF baseline — dome must be closed (v4.0) |

---

## Motor Tick Statistic Batch Runner

Automated script to collect motor tick runtime data by repeatedly opening and closing the dome. Runs manually on the Cloudwatcher Solo (Pi3) during daytime.

### What It Does

One **run** = dome CLOSED → OPEN → CLOSED. Default: **20 runs** per session.

Motor runtime varies with temperature (oil viscosity). Rest periods between direction changes are **randomized between 5 and 30 minutes** to collect data at varying motor temperatures — short rests keep the oil warm, long rests let it cool to ambient.

### How It Works

1. Enables tick logging on Arduino (`$L`)
2. Verifies dome at closed endpoints (both shutters at limit switches)
3. Checks weather — only opens when dry
4. Opens dome, waits 200s, verifies both shutters at open endpoints
5. Rests 5-30 min (random), monitors rain while open
6. Checks dome state — detects if closed externally during rest
7. Closes dome (no weather check needed), waits 200s, verifies closed endpoints
8. Run complete — repeats until all runs finished

**Weather strategy:** Rain is only checked before **opening**. Closing is always safe. If rain starts while open, the dome closes immediately and the run does not count.

**Endpoint verification:** After each motor run, the script parses the Arduino web page to confirm both shutters reached their target limit switches. Any "Intermediate" state triggers a Pushover alert.

**Pushover:** Every notification includes "Run X/Y". Messages sent for: script start/complete/stop, every open and close, interrupted runs, rain events, and rest periods with planned wait time.

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

### Configuration (edit script header)

| Variable | Default | Description |
|----------|---------|-------------|
| `MAX_RUNS` | 20 | Number of full open/close cycles |
| `MOTOR_WAIT_SECONDS` | 200 | Wait for motors to reach endpoints |
| `MIN_REST_MINUTES` | 5 | Minimum rest between direction changes |
| `MAX_REST_MINUTES` | 30 | Maximum rest (randomized) |
| `RAIN_THRESHOLD` | 2900 | Rain detection threshold (same as rain checker) |

### Prerequisites

- `astroshell_ticklogger.py` running on Solo port 88
- `aag_json.dat` updated by Cloudwatcher software
- Rain checker may remain running for safety (script cooperates)

---

## Dynamic Motor Timeout (v4.0)

Motor runtime varies with ambient temperature due to motor oil viscosity. The v4.0 firmware uses a linear regression model derived from 253 measured motor cycles (-1.6C to 22.6C) to compute temperature-appropriate timeouts.

### How It Works

- DS18B20 reads temperature every 5 seconds (non-blocking async)
- Four separate regression formulas for M1/M2 closing/opening
- Integer-only computation in loop() — no float math
- ISR reads precomputed timeout values atomically (16-bit AVR)
- Falls back to static 6527 ticks (107s) if sensor fails

### Temperature Effect

| Temperature | Approx M1 Close Timeout | Approx M1 Open Timeout |
|-------------|------------------------|----------------------|
| -5C | ~6800 ticks (limited to 6527) | ~6260 ticks |
| 0C | ~6700 ticks (limited to 6527) | ~6190 ticks |
| 10C | ~6500 ticks | ~6040 ticks |
| 20C | ~6310 ticks | ~5900 ticks |

---

## Frozen Dome Detection (v4.0)

In winter, dome halves can freeze together at the top. The motor opens the bottom but the top stays stuck, creating a crash risk when frost releases later. The VL53L0X ToF sensor detects this by measuring the gap between dome halves.

### How It Works

1. When an OPEN command starts, frozen detection monitors the ToF distance
2. After ~4 seconds of motor running, checks if gap increased by >30mm
3. If gap hasn't increased: STOP motor → wait 20s for gravity → re-check
4. If gravity separated halves: resume opening
5. If still stuck: reverse motor to close → retry (up to 3 attempts)
6. After 3 failed attempts: LOCKOUT — all open commands blocked
7. Unlock via web UI (`/?$U`) or physical button

### Calibration

Close the dome fully, then navigate to the web UI and click "Calibrate ToF Baseline". This stores the current ToF distance as the reference for "closed". The calibration persists in EEPROM across reboots.

---

## Tested & Verified (v4.0)

All v4.0 features were lab-tested on real hardware (Arduino MEGA + DS18B20 + VL53L0X) with a 13-step structured test workflow:

| Test | Result |
|------|--------|
| DS18B20 temperature reading | Passed |
| Dynamic timeout at -7.7°C and 19°C | Passed — all 8 values match regression model |
| DS18B20 graceful degradation + hot-plug | Passed |
| VL53L0X distance reading | Passed |
| ToF calibration + EEPROM persistence | Passed |
| VL53L0X graceful degradation + hot-plug | Passed |
| Frozen dome simulation (full 3-retry cycle) | Passed |
| Dome unlock from lockout | Passed |
| Frozen dome success path (not frozen) | Passed |
| Event notifications (Arduino → Solo → Pushover) | Passed |
| Conflicting signal detection | Passed |
| Tick data format (both temps + ToF in CSV) | Passed |
| Watchdog stability (no resets during testing) | Passed |

### Bugs Found and Fixed During Testing

5 bugs were discovered and fixed during lab testing:

1. **Sensor hot-plug failure** — Sensors stayed "Not connected" after unplug/replug. Fixed: 30-second periodic bus re-scan.
2. **DS18B20 slow startup** — Sensor not detected at boot (~80s delay). Fixed: 100ms power-on delay before bus init.
3. **Frozen detection display** — Web UI showed "OK" when ToF disconnected instead of "Disabled". Fixed: added sensor connection check.
4. **ISR race condition** — Frozen dome state machine never detected motor starts. Fixed: independent direction tracking variables (not shared with ISR).
5. **URL encoding** — Event detail strings with spaces caused HTTP 400 on Solo. Fixed: buffer-based URL encoding before send.

---

## Deploying to Cloudwatcher Solo

The Solo Pi3 has a **read-only root filesystem**. To deploy updated files:

```bash
ssh root@192.168.1.151
mount -o remount,rw /

# Update tick logger + event server
cp astroshell_ticklogger.py /usr/local/bin/
chmod +x /usr/local/bin/astroshell_ticklogger.py

# Update service file (important: -u flag for unbuffered stdout)
cp astroshell_ticklogger.service /etc/systemd/system/
systemctl daemon-reload

mount -o remount,ro /
systemctl restart astroshell_ticklogger
```

**Important:** The service file must use `python3 -u` (unbuffered output) or `print()` output won't appear in `journalctl`. This is already set in the provided service file.

**Pushover setup:** Replace `JOHNDOE_API_TOKEN` and `JOHNDOE_USER_KEY` in the deployed `/usr/local/bin/astroshell_ticklogger.py` with your real Pushover credentials. Never commit real credentials to the git repo.

---

## Important: Limit Switch Wiring

My dome had limit switches accidentally swapped during installation. The code compensates for this in the web display only. **If your switches are wired correctly**, see the section below for how to revert the display logic.

---

*Feel free to use this code at your own risk. Always test with the dome in an intermediate position!*

*Cheers, Joerg*

## If Your Limit Switches Are Wired Correctly

My code has **inverted web display logic** because my limit switches were accidentally swapped during installation. The underlying motor control logic and pin definitions are **identical to the original AstroShell code** - only the web interface displays the inverted physical state.

### What Was Changed (My Swapped Wiring)

The pin definitions and motor constants are **unchanged** from the original. Only the **web display** was modified to show the correct physical state despite the swapped wiring:

| Element | Original Display | My Modified Display |
|---------|------------------|---------------------|
| `lim1open` HIGH | "Opened" | "Physically CLOSED" |
| `lim1closed` HIGH | "Closed" | "Physically OPEN" |
| `mot1dir==OPEN` | "Opening" | "Closing (physically)" |
| `mot1dir==CLOSE` | "Closing" | "Opening (physically)" |
| Button $1 | "Open" | "CLOSE S1" |
| Button $2 | "Close" | "OPEN S1" |

### If Your Wiring Is Correct

If your AstroShell dome has **correctly wired limit switches**, you need to revert the web display logic in `domecontrol_JK3.ino`:

#### 1. Update Status Display

**Current (for my swapped wiring):**
```cpp
bool s1_is_physically_closed_state = digitalRead(lim1open);
bool s1_is_physically_open_state = digitalRead(lim1closed);
```

**Change to (correct wiring):**
```cpp
bool s1_is_physically_closed_state = digitalRead(lim1closed);
bool s1_is_physically_open_state = digitalRead(lim1open);
```

#### 2. Update Movement Display

**Current (for my swapped wiring):**
```cpp
if (mot1dir == OPEN) { client.print(F("Closing (physically)")); ... }
else if (mot1dir == CLOSE) { client.print(F("Opening (physically)")); }
```

**Change to (correct wiring):**
```cpp
if (mot1dir == OPEN) { client.print(F("Opening (physically)")); ... }
else if (mot1dir == CLOSE) { client.print(F("Closing (physically)")); }
```

#### 3. Update Button Labels

**Current (for my swapped wiring):**
```cpp
client.print(F("<a href='/?$2' class='button b-open'>OPEN S1</a>"));
client.print(F("<a href='/?$1' class='button b-close'>CLOSE S1</a>"));
```

**Change to (correct wiring):**
```cpp
client.print(F("<a href='/?$1' class='button b-open'>OPEN S1</a>"));
client.print(F("<a href='/?$2' class='button b-close'>CLOSE S1</a>"));
```

#### 4. Update Comments

Search for comments mentioning "inverted", "swapped", or "physically" and update them to reflect your correct wiring.

### Testing

After making changes, test carefully with the dome in an intermediate position so you can intervene if something moves in the wrong direction!

## Network Monitoring Behavior - Test Cases

The dome controller has smart network monitoring that adapts to different scenarios. Below are all possible combinations and expected behaviors:

### Startup Scenarios

| Ethernet Cable | Result | IP Monitoring |
|----------------|--------|---------------|
| Not connected at startup | Standalone mode, buttons work immediately | Disabled |
| Connected at startup | Full network init (~5s), web interface active | Enabled |
| Inserted after startup (standalone) | Detected within 1 minute, network initializes | Becomes enabled |

### Cable Removal Scenarios (when monitoring is enabled)

| Dome State | Cable Removed | Action | Timing |
|------------|---------------|--------|--------|
| Dome open | Cable physically removed | Immediate auto-close | Instant |
| Dome partially open | Cable physically removed | Immediate auto-close | Instant |
| Dome closed | Cable physically removed | No action needed | - |
| Dome opening | Cable physically removed | Stops opening, starts closing | Instant |

### Cloudwatcher IP Unreachable (cable connected but target not responding)

| Condition | Action | Timing |
|-----------|--------|--------|
| Target IP unreachable | Count failures | Every 90 seconds |
| 10 failures within 15 minutes | Auto-close dome | After 10th failure |
| Target responds again | Reset fail counter | Immediate |
| Connection restored during auto-close | Stop closing motors | Immediate |

### User Intervention During Auto-Close

| User Action | Result | Network State |
|-------------|--------|---------------|
| Press STOP button | Motors stop immediately | Monitoring paused until cable reconnected |
| Press direction button | Motors stop (toggle behavior) | Monitoring paused until cable reconnected |
| Manually open dome after STOP | Dome opens, system won't interfere | Manual control mode |
| Manually close dome | Works normally | Manual control mode |
| Reconnect cable | Reset trigger flag, resume monitoring | Monitoring active again |

### Edge Cases

| Scenario | Behavior |
|----------|----------|
| Cable removed, auto-close, user STOPs, walks away | Dome stays stopped (user took control) |
| Cable reconnected after user STOP | Monitoring resumes, will auto-close on next removal |
| Rapid cable connect/disconnect | Only triggers once per removal (debounced) |
| No cable ever connected | Pure standalone mode, no network checks |

### Debug Output (when SERIAL_DEBUG_IP enabled)

Every 10 seconds status line:
```
IP Status: Link=1 EthInit=1 NetMon=1 Fails=0/10 S1closed=1 S2closed=0
```
- Link: 0=cable disconnected, 1=cable connected
- EthInit: 0=Ethernet not initialized, 1=initialized
- NetMon: 0=monitoring disabled, 1=monitoring enabled
- Fails: current fail count / max before auto-close
- S1closed/S2closed: 0=shutter open/moving, 1=shutter at closed position
