# Lessons Learned

## [2026-02-28] — Sensor hot-plug re-detection
- **Mistake:** After unplugging and reconnecting DS18B20/VL53L0X, the sensor stayed "Not connected" until reboot
- **Root cause:** Once `failCount >= THRESHOLD` and `connected == false`, the reading function returned early and never re-scanned the bus
- **Rule:** Any sensor with a failure threshold must have a periodic re-detection loop (e.g., every 30 seconds) that re-initializes the bus and checks for the device
- **Applies to:** All hot-pluggable sensors on Arduino

## [2026-02-28] — DS18B20 needs startup delay
- **Mistake:** DS18B20 not detected at boot, took ~80 seconds to appear via re-detection
- **Root cause:** `setupDS18B20()` ran too early after power-on. The sensor hadn't finished its own boot, so `getDeviceCount()` returned 0
- **Rule:** Add `delay(100)` before `ds18b20.begin()` to allow sensor power-on initialization. Always consider hardware boot timing for sensors initialized in `setup()`
- **Applies to:** DS18B20, potentially other sensors with power-on delay requirements

## [2026-02-28] — ISR vs loop() race condition on direction tracking
- **Mistake:** Frozen dome state machine used ISR's `m1_prev_dir` to detect motor start transitions — never worked because ISR always updated the variable before `loop()` ran
- **Root cause:** ISR runs at 61 Hz and updates `m1_prev_dir = mot1dir` every tick. By the time `frozenDomeStateMachine()` runs in `loop()`, the transition (0 → CLOSE) has already been overwritten
- **Rule:** When `loop()` code needs to detect state transitions in ISR-controlled variables, it MUST use its own independent tracking variables (static locals). Never share transition-detection variables between ISR and `loop()` contexts
- **Applies to:** Any Arduino code where loop() needs to detect transitions in ISR-modified variables

## [2026-02-28] — URL encoding required for HTTP parameters with spaces
- **Mistake:** Event detail strings like "S1 attempt 1/3" sent with literal spaces caused HTTP 400 on Python server
- **Root cause:** Python's `BaseHTTPRequestHandler` splits the HTTP request line by spaces to parse method/path/version. Unencoded spaces in the URL break parsing
- **Rule:** Always URL-encode dynamic strings in HTTP query parameters (spaces → `%20`). On Arduino, encode into a local buffer first, then send with a single `print()` — do NOT use character-by-character `write()` to EthernetClient (causes TCP buffering issues on W5100/W5500)
- **Applies to:** All Arduino HTTP client code sending dynamic strings

## [2026-02-28] — Python stdout buffered under systemd
- **Mistake:** Event handler `print()` output invisible in `journalctl` despite events processing correctly (HTTP 200 returned)
- **Root cause:** Python buffers stdout when not connected to a TTY (standard behavior when running as a systemd service)
- **Rule:** Always use `python3 -u` (unbuffered) in systemd service ExecStart, or set `Environment=PYTHONUNBUFFERED=1`. This applies to ALL Python services that use `print()` for logging under systemd
- **Applies to:** Any Python script running as a systemd service that uses print() for logging

## [2026-10-24] — Wiring convention was hard-coded far beyond the display
- **Mistake:** README claimed the mirrored German wiring only affected the web display ("revert three lines for correct wiring"). In v4.1.0 the convention was baked into 109 places, including the IP/cable auto-close direction, frozen-dome reverse, tick-logging direction and timeout selection
- **Root cause:** Each v4.x feature re-derived "physically closed" from the raw `lim*open` / `OPEN` names instead of one semantic layer
- **Rule:** Every physical open/closed decision goes through `DIR_PHYS_*` / `LIM*_PHYS_*`; raw `OPEN`/`CLOSE`/`lim*` only in the ISR pairing, PWM map and panel buttons. Any firmware change runs `make -C test/host test` (both sites) first
- **Applies to:** domecontrol_JK3.ino, any new feature that moves or reports a shutter

## [2026-10-24] — Rate-limiter timestamps initialised to 0 swallowed the first event after boot
- **Mistake:** `now - lastEventSentTime < 10 s` with `lastEventSentTime == 0` dropped events for 10 s after every reset; the conflict limiter dropped them for 5 minutes
- **Root cause:** "never sent" and "sent at millis()==0" were not distinguished
- **Rule:** Treat 0 as "never sent" (`last == 0 || now - last >= interval`) in every `millis()`-based limiter
- **Applies to:** all Arduino rate limiters and periodic timers

## [2026-10-24] — Test expectations at limit switches (made the same mistake three times)
- **Mistake:** Scenarios asserted stop reason 5 or 6 after a motor stopped at an end stop; the checks failed although the firmware was right
- **Root cause:** The ISR stop-reason housekeeping clears every reason except 3 (with the auto-close flag) and 4 on the next tick once a motor is off at a limit
- **Rule:** At an end stop assert the state machine or `m*_interrupt_ticks`, never reason 5/6; panel-button tests need more than 6 ticks between presses (ISR debounce)
- **Applies to:** test/host/test_dome.cpp

## [2026-10-24] — Multi-file edit script applied partially
- **Mistake:** One replacement script edited the firmware, then hit a stale search string in the test file and stopped; firmware and tests were out of step for one run
- **Root cause:** Each file was written as soon as its own replacements matched instead of validating all files first
- **Rule:** Validate every replacement in every file, then write all files, or none. Keep the count-asserting approach (refuse on mismatch); it caught two miscounted sites in the 109-site sweep
- **Applies to:** any scripted refactor across several files

## [2026-10-24] — Shell working directory drifted between parallel tool calls
- **Mistake:** A workflow file was written relative to the wrong directory and silently missed the repo
- **Root cause:** `cd` in one Bash call persisted into a parallel call
- **Rule:** Always write files with absolute paths; never rely on the cwd in a parallel batch
- **Applies to:** every Claude Code session in this repo

## [2026-10-24] — Persisted safety flag not tied to the build that wrote it
- **Mistake:** First `$W` design stored only the on/off bit; a board flashed with the other site profile would have inherited the wrong default (Portugal with rain-check ON -> auto-close every 16 min; Germany with it OFF)
- **Root cause:** EEPROM outlives re-flashing, the semantics of the flag did not
- **Rule:** Persist the writer's site id next to any site-dependent flag and re-apply the default when it differs; make web toggles idempotent (`$W1`/`$W0`) so reloads or double clicks cannot flip safety settings. Found by an independent review, not by me: request one for every safety-relevant change
- **Applies to:** EEPROM config bytes, web commands that change persistent state

## [2026-10-25] — Portugal commissioning pitfalls
- **Mistake:** Two false alarms: "Sketch too big" (32256 B) and "no web response"
- **Root cause:** IDE board set to Arduino Uno instead of Mega 2560; firmware IP is static 192.168.1.177 while the browser was on another subnet
- **Rule:** Commissioning step 1 = board Mega 2560; step 2 = Arduino IP must be inside the local subnet (`IP_ADR2`), ideally carried by the site profile
- **Applies to:** flashing any dome controller on a new site

## [2026-11-02] — Incident analysed for the wrong dome
- **Mistake:** A "dome did not close completely" report was analysed against the German profile (dynamic timeout, DS18B20) until the user pointed out that 192.168.0.177 is the Portugal dome
- **Root cause:** The report only said "IP address 177"; both domes end in .177 and differ in timeout mode (Germany dynamic, Portugal static 6527), sensors and Cloudwatcher check
- **Rule:** For any incident report first establish the site (full IP / subnet, or ask), then read the live status page of THAT dome before reasoning about firmware paths. Note that the Portugal board can run a locally adapted build (its page showed push target .185 while main says .151)
- **Applies to:** every dual-site incident analysis in this repo
