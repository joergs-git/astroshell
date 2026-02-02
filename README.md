# Astroshell Driver and Lunatico Cloudwatcher Solo Integration / Enhancements
Optimized Arduino Code for controlling of my AstroShell Dome
(Also have a look at the litte wiki)

This arduino code is kind of branched from the producer of the AstroShell Telescope Domes.
As I was not supersatisfied with some aspects I enhanced it to my likings. 

So feel free to use it too. 
One important hint: When my dome was installed the technician team accidentally swapped 
the limit-endpoint-switches so my dome reported open while it was actually closed and vice versa.
Because I didn't wanted to re-solder all the cablings I simply asked AstroShell for the code to make the necessary logical changes there
and thus avoided re-cabling.

While I initially just planned to solve my little problem I ran into some further ideas which I implemented into this code now too:

1. A enhanced UI webfront-end which works better for iphones as well as tablets (imho)
2. Disabled the cloudsensor evaluation because it simply doesn't make sense at all (imho)
3. Disabled the "vibration motor option" (for whatever it was for). No clue.
4. Added hardcoded ethernet pinging of my lunatic cloudsensor to automatically close the dome in case the clouddetector is not up and running and thus might not be able to detect rain. Dome will be closed after 3 failed pings within 3 minutes.
5. Added a new command URL Command $S to get a quick response if the Dome is open or closed (to use it in other scripts or tools e.g. lunatic cloudwatcher solo). I place my little Bash-Shell-Script running on my Lunatico Cloudwatcher Solo device also into this repository so you get an idea of what I established for my self.
(To start it and keep it running on the solo you have to run it with nohup. e.g. "_nohup /home/aagsolo/rainchecker.sh > /dev/null 2>&1 &_" 

6. optimized memory usage of arduino
7. disabled Serial debugging Outputs as it's conflicting with Data Input Ports 0,1 from the arduino which are used for the astroshell end-point limit switches.
8. Some stabilty tweaks and checks
9. Showing some further switch informations and results of pinging the cloudwatcher
10. created a linux bash shell script running on Cloudwatcher Solo from Lunatico to check for raindrops and close it automatically. Also if the PC is not running/working.
11. Overall mostly failure proof safety now. (not 100% though of course).

The logic of the motor itself was not touched at all.

If you like to use it, feel free doing so. But on your own risk of course.
It's always a good idea to try out things while the dome is half open (not fully closed or fully open) so you have a chance you can react in case of....

Cheers
Joerg

Addons:
BTW I meanwhile replaced the UNO of the Astroshell into a Arduino MEGA which has a bit more memory so not running into certain issues.

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
