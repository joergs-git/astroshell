# AstroShell Dome Controller

**Enhanced Arduino fail-safe firmware for AstroShell telescope domes with automatic rain protection with lunaticoastro's cloudwatcher solo**

[![Platform](https://img.shields.io/badge/Platform-Arduino%20MEGA-blue)]()
[![License](https://img.shields.io/badge/License-Free%20to%20Use-green)]()

> Also check out the [Wiki](https://github.com/joergs-git/astroshell/wiki) for additional documentation.

---

## Key Features

| Feature | Description |
|---------|-------------|
| **Automatic Rain Protection** | Closes dome automatically when rain detected via Cloudwatcher Solo |
| **Network Failsafe** | Auto-closes if Cloudwatcher becomes unreachable (5 failures in 5 min) |
| **Cable Removal Detection** | Instant dome closure if Ethernet cable is disconnected |
| **Mobile-Friendly Web UI** | Responsive design works on phones, tablets, and desktops |
| **Hardware Watchdog** | 8-second watchdog ensures automatic recovery from hangs |
| **Motor Runtime Logging** | Tick-based logging for temperature correlation analysis |
| **Pushover Notifications** | Real-time alerts for rain, connectivity issues, and dome status |

---

## Safety Features at a Glance

```
Telescope Protection Priority:

  Rain Detected          Cloudwatcher        Ethernet Cable
       via                Unreachable           Removed
   Cloudwatcher           (5 min)             (Instant)
       |                     |                    |
       v                     v                    v
   +-----------------------------------------------+
   |           AUTOMATIC DOME CLOSURE              |
   +-----------------------------------------------+
                         |
                         v
              Pushover Notification
                    to Owner
```

---

## Quick Start

1. **Upload** `domecontrol_JK3.ino` to Arduino MEGA
2. **Configure** IP addresses in the code (dome: 192.168.1.177, Cloudwatcher: 192.168.1.151)
3. **Deploy** rain checker script to Cloudwatcher Solo (optional but recommended)
4. **Access** web interface at `http://192.168.1.177`

5. **Addons:** If you want to have it perfect I recommend you also use the batch files to control dome from NINA and also install the cloudwatcher-rainchecker and astroshell-ticklogger services on your cloudwatcher solo device. After that you're really super set. Also I recommend you registering a Pushover API account for messaging services if you don't already have one. Just have a look into the different files where you can see further explanations and howtos.
---

## What's Different from Original AstroShell Code

This firmware is enhanced from the original AstroShell code with these improvements:

| Area | Enhancement |
|------|-------------|
| **Web Interface** | Modern responsive UI for mobile devices |
| **Network Monitoring** | IP failsafe auto-close (original had none) |
| **Rain Protection** | Integrated Cloudwatcher Solo rain checker script |
| **Stability** | Hardware watchdog, memory optimization, removed unused features |
| **API** | Added `$S` status command, `$L` logging toggle, `$R` counter reset |
| **Diagnostics** | Motor tick logging, timeout tracking, detailed stop reasons |

**Note:** The motor control logic is unchanged from the original - only safety features and UI were enhanced.

---

## Hardware

- **Controller:** Arduino MEGA 2560 (upgraded from UNO for more memory)
- **Network:** W5500 Ethernet Shield
- **Weather:** Lunatico Cloudwatcher Solo (optional but recommended)

---

## Repository Files

| File | Description |
|------|-------------|
| `domecontrol_JK3.ino` | Main Arduino firmware (v3.3) with network monitoring, tick logging, web UI |
| `cloudwatcher-raincheckerV3.sh` | Rain checker script for Cloudwatcher Solo - auto-closes dome on rain |
| `cloudwatcher-rainchecker.service` | Systemd service for rain checker autostart |
| `astroshell_ticklogger.py` | Motor runtime logger server - receives tick data, logs to CSV |
| `astroshell_ticklogger.service` | Systemd service for tick logger autostart |
| `open_astroshell.bat` | Windows batch script for NINA - opens dome with Pushover notification |
| `close_astroshell.bat` | Windows batch script for NINA - closes dome with Pushover notification |
| `motortick-statistic-batchrun.sh` | Automated dome open/close cycles for motor tick statistics collection |
| `controller_wcapacitor_2025_germany.ino` | Alternative firmware with capacitor backup (has known bugs) |
| `nocapacitor2023.ino` | Original unmodified AstroShell reference code |

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

#### 1. Update Status Display (around line 1041-1046 and 1075-1080)

**Current (for my swapped wiring):**
```cpp
bool s1_is_physically_closed_state = digitalRead(lim1open);
bool s1_is_physically_open_state = digitalRead(lim1closed);
if (s1_is_physically_closed_state) client.print(F("Physically CLOSED"));
else if (s1_is_physically_open_state) client.print(F("Physically OPEN"));
```

**Change to (correct wiring):**
```cpp
bool s1_is_physically_closed_state = digitalRead(lim1closed);
bool s1_is_physically_open_state = digitalRead(lim1open);
if (s1_is_physically_closed_state) client.print(F("Physically CLOSED"));
else if (s1_is_physically_open_state) client.print(F("Physically OPEN"));
```

#### 2. Update Movement Display (around line 1050-1052 and 1084-1086)

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

#### 3. Update Button Labels (around line 1036-1037 and 1070-1071)

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
| Target IP unreachable | Count failures | Every 1 minute |
| 5 failures within 5 minutes | Auto-close dome | After 5th failure |
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
IP Status: Link=1 EthInit=1 NetMon=1 Fails=0/5 S1closed=1 S2closed=0
```
- Link: 0=cable disconnected, 1=cable connected
- EthInit: 0=Ethernet not initialized, 1=initialized
- NetMon: 0=monitoring disabled, 1=monitoring enabled
- Fails: current fail count / max before auto-close
- S1closed/S2closed: 0=shutter open/moving, 1=shutter at closed position 
