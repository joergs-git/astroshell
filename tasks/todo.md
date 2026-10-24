# AstroShell — Task Tracking

## In Progress (2026-10-24) — Dual-site firmware v4.2.0, branch feature/dual-site-wiring

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

Open:
- [ ] Merge PR #3 after user approval (CI green, PR marked ready)
- [ ] Portugal commissioning with the dome in intermediate position (checklist in README / CLAUDE.md)

## Results (2026-10-24)
- One firmware source for both domes; Germany behaviour byte-identical except the intended `$W` additions and the rate-limiter fix (verified by an independent review that substituted the Germany macros and diffed against main).
- Baseline evidence: v4.1.0 under Portugal physics fails 20/26 scenarios incl. "Cloudwatcher unreachable closes the dome" (it would have opened the dome).
- Cost: +864 B flash, +2 B RAM vs v4.1.0 on a 256 KB / 8 KB MEGA.
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
