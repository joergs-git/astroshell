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
