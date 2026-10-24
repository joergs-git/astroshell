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

Open:
- [ ] Incorporate review findings, push, CI green
- [ ] Rosetta on the Mac (user) -> local arduino-cli compile of both profiles for the size table
- [ ] Merge PR after user approval
- [ ] Portugal commissioning with the dome in intermediate position (checklist in README / CLAUDE.md)

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
