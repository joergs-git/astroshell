# AstroShell — Task Tracking

## Completed (2026-10-24) — Dual-site firmware v4.2.0, PR #3 merged into main (c2c0350)

Decisions taken with the user (2026-10-24):
- D1 wiring switch: compile-time site profile (`SITE_GERMANY` default / `SITE_PORTUGAL`)
- D2 `$W` OFF also disables the cable-removal auto-close: yes
- D3 `$W` proposed and accepted; tick logging `$L` unchanged (log server in Portugal comes later)
- D4 IP stays 192.168.1.177 / MAC :ED for both sites (separate LANs)
- D5 CI compile matrix; locally Rosetta recommended (IDE toolchain is x86_64)

Done:
- [x] Site profile block + PHYSICAL SEMANTICS macros (DIR_PHYS_*, LIM*_PHYS_*, *_NAME, WIRING_LABEL, TIMEOUT_M*_FOR, BUTTON_MAY_START)
- [x] All 109 wiring-sensitive sites converted with a counted replacement script (refused to write on any count mismatch)
- [x] EEPROM byte 9 config flags, `$W` toggle, gating of IP check / cable auto-close / tick / interrupt / event pushes
- [x] Rate-limiter fix (timestamp 0 = never sent) for events and conflict events
- [x] Web UI: Cloudwatcher Check row + toggle button, Limit Wiring row (German UI text otherwise unchanged)
- [x] Host logic test harness test/host (mocks + dome physics), 35 scenarios x 2 sites, all green
- [x] Baseline run: v4.1.0 with Germany physics 24/26 (2 known rate-limiter warts), with Portugal physics 20/26 FAIL (hazard documented)
- [x] GitHub Actions firmware-ci.yml: logic tests + avr-gcc build matrix SITE=1/2
- [x] README, CLAUDE.md, ascom-driver/README updated; obsolete "revert three display lines" README section removed
- [x] Independent review of the firmware diff requested (subagent)

- [x] Review findings incorporated: EEPROM byte 9 carries the SITE id (other-site value -> site default), `$W1`/`$W0` idempotent forms (UI button uses them), stale auto-close flag cleared while OFF, ISR comment corrected
- [x] Test gaps from the review closed: S2 frozen cycle + lockout, closing during lockout (web + panel), cable and IP auto-close during the frozen cycle, M2 tick push, stop-reason display after auto-close, cable auto-close reversing an opening shutter -> 41 scenarios x 2 sites green
- [x] Rosetta installed by the user; local arduino-cli build with the IDE toolchain: SITE=1 50514 B flash / 1181 B RAM, SITE=2 50514 B / 1180 B, no warnings
- [x] User: no VL53L0X exists on either dome -> `ENABLE_TOF_FROZEN_DETECTION 0` (default, overridable). I2C/ToF layer compiled out, frozen logic inert; host suite now 4 binaries (site x ToF): 32 + 41 scenarios per site, all green. Sizes without ToF layer: 43650 B flash / 870 B RAM (SITE=1), 43652 B / 871 B (SITE=2); with ToF 50514 B / 1181 B

- [x] PR #3 merged into main by the user's instruction (merge commit c2c0350), branch deleted

## Completed (2026-10-25) — v4.2.1: DS18B20 layer per site
- [x] User: no temperature probe in Portugal -> `ENABLE_DS18B20_DYNAMIC_TIMEOUT` per site (Germany 1, Portugal 0); OneWire/Dallas compiled out, static 6527 timeout, "Temperature: Not installed"
- [x] Compile error in Portugal was the IDE board selection (Uno instead of Mega 2560); documented in README/CLAUDE.md commissioning steps
- [x] Tick logger: no Solo/Synology in Portugal; with the Cloudwatcher check off no pushes happen, web page now says "pushes suspended"
- [x] Host suite: 6 binaries (site x default/sensors/bare), 32/41/32 scenarios per site, green; CI Arduino matrix site x ToF x DS18B20 (8 builds)
- [x] IDE-toolchain sizes, default profiles: Germany 43724 B flash / 870 B RAM, Portugal 39608 B / 727 B

## Completed (2026-10-25) — Portugal commissioning
- [x] v4.2.1 flashed with `SITE_PORTUGAL` (board Mega 2560). Web page reachable after fixing the IDE board selection and the LAN subnet; open / stop / end positions verified via web UI and via the panel buttons (user report)

## Completed (2026-11-01) — v4.2.2: IP per site, Portugal fully verified
- [x] Portugal LAN is 192.168.0.x: `IP_ADR2` moved into the site profile (Germany 1 -> 192.168.1.177, Portugal 0 -> 192.168.0.177), Solo address derived from the same subnet. PR #5 merged (b56d5a9); host suite 6 builds green, IDE builds Germany 43724 B / 870 B, Portugal 39606 B / 727 B
- [x] Portugal: ASCOM dome driver verified by the user (open / close, end detection), "Swap Open/Close" unchecked

## Planned (2026-11-02) — v4.3.0: limit-switch debounce + per-site static timeouts (Portugal)
Findings (Portugal dome 192.168.0.177, Solo/tick logger 192.168.0.185):
- Night close 2026-11-02 via NINA: West hit the static 6527-tick timeout. NINA's AscomDome wait loop has no own timeout
  (polls while ShutterClosing, 1 s); it ended 107.24 s after the close call = firmware timeout at 106.9 s. East closed in 5943 ticks.
- Measurement cycle 2026-11-02 08:28 UTC (21 C, dry): West close stopped after 252 ticks by a spurious HIGH on pin 0
  (lim2closed = West "phys. closed" in the Portugal profile, shares RX0 with the USB-serial chip); the firmware logged it as a
  VALID full run of 252 ticks. Re-sent $3 closed the rest in 87.4 s -> warm West close about 91.5 s (~5585 ticks).
  East close 5877, East open 5410, West open 5201 ticks. Germany's 253-cycle CSV shows no such short run.
- The ISR stops a motor on a single raw limit sample (no debounce); 55 digitalRead(lim*/LIM*) sites in 7 functions.
Tasks:
- [x] ISR samples the four limit pins every tick; a level counts only after LIM_CONFIRM_TICKS = 4 identical samples (66 ms,
      ~1.5 mm of travel); `limRead()` replaced all 62 raw reads via a counted replacement script; raw digitalRead only in
      `limSample()` and once in setup()
- [x] STATIC_TIMEOUT_M*_CLOSE/OPEN in the site profile (physical direction); Portugal: West close 7930 (130 s), East close
      7320 (120 s), opens 6527; Germany 6527 x4 (dynamic cap unchanged). HAVE_TIMEOUT_REGRESSION gates the German regression
      (Portugal 0: a probe only shows the temperature). SOLO_IP_ADR3 per site (.151 / .185), Portugal remote-check default ON
- [x] Host tests: physics can inject limit glitches; 7 new scenarios (glitch ignored, threshold level counts, real end stop
      within the window, site timeouts via web and panel buttons, one LinkOFF read ignored, auto-close reversal right after
      an open from the stop) + probe-without-regression for the Portugal sensors build; fallback defines keep `FW=<old.ino>`
      compiling. New firmware: 39/48/39 per site green; v4.2.2 against the new suite fails the glitch, LinkOFF and reversal ones
- [x] Independent review (subagent, 2026-11-02): no blockers; fixed its two should-fix code findings (auto-close skipped a
      shutter that had just left the closed stop; `millis() | 1` stamp bypassed the 200 ms LinkOFF filter), the DYN cap guard,
      the probe-without-regression periodic path, the panel-button timeout checks and the commissioning text. Residual noted:
      a glitch of >= LIM_CONFIRM_TICKS samples is logged as a valid run (logger plausibility check = follow-up)
- [x] IDE toolchain builds clean (after the review fixes): Germany 44040 B flash / 877 B RAM, Portugal 39942 B / 733 B (+316 B / +7 B vs v4.2.2)
- [x] README (v4.3.0 section, site table, commissioning step 6), CLAUDE.md, firmware header v4.3.0; MAC :ED / IP_ADR3 177 unchanged
- [x] Pushed as PR #6 (feature/limit-debounce-site-timeouts), CI run 41416587060 green (6 host builds + 8 Arduino builds)
- [ ] PR #6 merge on the user's word
- [ ] User flashes Portugal on site with SITE_PORTUGAL (profile now carries IP .0.177, Solo .185, check ON, DS18B20 0, the
      timeouts); then one logged open/close cycle and a check of "Last Valid Full-Runs"; tighten the Portugal timeouts from
      logger data after a few nights. USB is never connected in operation (user), so the pin-0 glitch is line noise

## Next
- [ ] Germany: flash v4.2.2 with the default profile when convenient (no EEPROM step; Cloudwatcher check stays ON)
- [ ] Portugal, later: Solo at 192.168.0.151 -> `$W1`; sensors -> flip `ENABLE_DS18B20_DYNAMIC_TIMEOUT` / `ENABLE_TOF_FROZEN_DETECTION` in the profile, calibrate ToF with `$C`
- [ ] Germany: flash v4.2.0 (default profile) at a convenient moment; no EEPROM step needed, Cloudwatcher check stays ON
- [ ] When a VL53L0X is mounted: `ENABLE_TOF_FROZEN_DETECTION 1` (per site if only one dome), calibrate with `$C`

## Results (2026-10-24)
- One firmware source for both domes; Germany behaviour byte-identical except the intended `$W` additions and the rate-limiter fix (verified by an independent review that substituted the Germany macros and diffed against main).
- Baseline evidence: v4.1.0 under Portugal physics fails 20/26 scenarios incl. "Cloudwatcher unreachable closes the dome" (it would have opened the dome).
- Cost: +864 B flash, +2 B RAM vs v4.1.0 with the ToF layer built in; the default build without the ToF layer is 6 KB flash and 309 B RAM SMALLER than v4.1.0.
- Side observation (pre-existing, not changed): the frozen check compares one ToF distance against the baseline; if the other half is already open and the sensor sees that gap, a frozen second half is not detected. Depends on the sensor geometry.

## Completed

### v4.0 Safety Sensor Enhancement (2026-02-28)
All phases implemented and merged via PR #1.
- DS18B20 temperature probe + dynamic timeout
- VL53L0X frozen dome detection
- Event notification system (Arduino + Python)
- Web UI enhancements
- Conflicting signal detection
- Documentation updates

### IP Auto-Close Parameter Relaxation (2026-03-04)
- connectCheckInterval: 60s → 90s
- maxConnectFails: 5 → 10
- maxFailTimeWindow: 5min → 15min
