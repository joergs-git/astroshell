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

## [2026-11-02] — Independent review found two pre-existing bugs behind a safety change
- **Mistake:** My v4.3.0 diff was correct in itself, but (a) the auto-close skipped a shutter that had just started opening from the closed stop while its confirmed level still said "closed", and (b) the 200 ms LinkOFF filter stamped `millis() | 1`, which wraps the subtraction on even millis
- **Root cause:** (a) a "close whatever is not closed" decision read a now-debounced level without treating "motor already moving away from the stop" explicitly; (b) a 0-means-never sentinel was faked with `| 1`
- **Rule:** Any decision on a debounced/confirmed level must handle the motor direction first. Never fake a non-zero timestamp with `| 1`; use `if (stamp == 0) stamp = 1` or a flag. Request the independent review for every safety-relevant change (this is the second time it found a real bug I did not)
- **Applies to:** domecontrol_JK3.ino auto-close and any millis()-based filter

## [2026-11-02] — A debounce alone did not cover the field fault
- **Mistake:** v4.3.0 assumed the spurious limit level was a single sample; the next close on the real dome showed a HIGH that outlasted the 4-tick confirmation (212-tick stop, logged as a valid run)
- **Root cause:** The confirmation window was sized from one observation; the fault is a sustained level coupled to the release of the other switch, not a one-sample glitch
- **Rule:** For safety inputs combine a short debounce with a physical plausibility check (a full run cannot reach the far end before LIM_MIN_RUN_TICKS); never size a filter from a single event. Watch the first real runs after flashing with the logger before declaring a fix done
- **Applies to:** domecontrol_JK3.ino limit handling, any sensor filter sized from field data

## [2026-11-20] — Two limit inputs, four readers, no "both active" case
- **Mistake:** With both West end switches HIGH the page said "Physically CLOSED", `$S` said CLOSED, `$A` said OPEN and the stop reason said "Limit 'Phys. Open'"; the close guard refused every close while the shutter stood open. The rain checker would have taken the dome for closed
- **Root cause:** Every reader had its own `if (closed) ... else if (open)` chain over the same two levels, each with a different order, and none knew the impossible combination. The switches read HIGH when their line is open, so the fault direction is exactly "end stop reached"
- **Rule:** Derive a multi-input state in ONE function with every combination named (`shutter*State()` with `SH_FAULT`); a status answer that scripts rely on (`$S`) must fail towards "not closed". For an input whose failure mode is "reads active", plan the manual override together with the detection (`$F1`/`$F3`), never an automatic one
- **Applies to:** domecontrol_JK3.ino status and command paths, any safety state built from several sensors

## [2026-11-20] — A plausibility window sized for one fault moved the damage elsewhere
- **Mistake:** v4.3.1 ignored the far end for 2440 ticks because the first glitches came 3.5 s into a close; the next ones came at 2191 and 3268 ticks, and the 3268 one ended a close with the shutter 40 % open, logged as valid. The same window also made an OPEN from a falsely "closed" position push against the open stop for up to 40 s
- **Root cause:** The window was sized from the first observations, and its premise ("the run started at the opposite end stop") was taken from a level that can itself be stuck
- **Rule:** Size a plausibility window from the physical limit (fastest full run per direction, with margin), not from where the fault showed up first, and check the premise of a rule with its own evidence (the start switch must release, `LIM_RELEASE_TICKS`). A run counts as valid only if it also satisfies the rule that protected it
- **Applies to:** domecontrol_JK3.ino limit handling, any filter that trusts one sensor to qualify another

## [2026-11-20] — New events silently dropped and 216 B of RAM for three messages
- **Mistake:** The first draft pushed `limit_stuck` once; it follows the `limit_glitch` of the same run by 2 s and was dropped by the 10 s event spacing in exactly its main use case. The event texts were plain literals: +216 B RAM
- **Root cause:** `sendEventNotification()` returned nothing, so callers could not tell a delivered event from a dropped one; string literals passed as `const char*` live in RAM on AVR
- **Rule:** An event that must arrive is retried with its own interval and marked sent only on a true return (never on every loop pass: a dead Solo costs 1.5 s per connect). Event texts go through `PSTR` / `strcpy_P` / `snprintf_P`. Found by a test that asserted the event, and by comparing avr-size before and after: do both for every firmware change
- **Applies to:** every `sendEventNotification()` caller

## [2026-11-21] — The independent review found what my own scenarios could not
- **Mistake:** My first v4.3.3 moved the plausibility window from 2440 to 4220 ticks and called the 3268-tick field case fixed. A level that stays active simply ended the close at tick 4220 instead, a quarter open, reported CLOSED, logged valid. My scenarios only injected 30-tick levels. The larger window also doubled the time a motor pushes against a stop when a false start level drops in time, and the new override flipped the page to "Physically CLOSED" the moment the open switch released
- **Root cause:** I tested the fault shape I had in mind (a short glitch) and not the one the field showed last (a line that stays open); I sized a filter without asking what happens at its end; I added a command that changes what a level means without re-reading every reader of that level
- **Rule:** For every filter ask "what if the input is still wrong when the filter ends" and test a sustained fault next to the short one. After a safety change run the previous release and the new one on the same random healthy sequences and demand identical traces. Request the independent review before calling it done (third time it found real defects), and give the reviewer the code, not the conclusion
- **Applies to:** domecontrol_JK3.ino limit handling, every filter or override on a safety input

## [2026-11-21] — My fix for the review finding created a worse case than the finding
- **Mistake:** After round 1 I let a close run on to its timeout past a stale "closed" level whenever the start position was "known". Round 2 showed that with a false start position (a line open at boot, or one that had stopped an opening half way) the "stale" level is the TRUE one, and a normal close pushed against the closed stop for up to 127 s (58 s before my fix, 28 s in v4.3.2). My grace rule for arrivals also accepted a level that had been active and ignored all along
- **Root cause:** I reasoned from the case I wanted to fix (false level, true position) and did not turn it round (true level, false position); "known" was one flag for evidence of very different strength
- **Rule:** For every rule that ignores a safety input, write down the opposite failure (the input was right, the premise wrong) and bound its cost before shipping. Grade position evidence (boot or short run = known, valid full run = proven) and give the strong privilege only to the strong evidence. When a rule ends a run on an input it did not believe, keep not believing it afterwards. Plan a second review round for the fixes of the first
- **Applies to:** domecontrol_JK3.ino limit handling, any override or run-on logic
