# AstroShell v4.0 — Safety Sensor Enhancement

## Phases

- [x] Phase 1: DS18B20 temperature probe + dynamic timeout
- [x] Phase 2: VL53L0X frozen dome detection
- [x] Phase 3: Event notification system (Arduino + Python)
- [x] Phase 4: Web UI enhancements
- [x] Phase 5: Conflicting signal detection
- [x] Phase 6: Documentation updates (CLAUDE.md, README.md)
- [x] Include temp/ToF in existing tick/interrupt HTTP posts

## Results

All phases implemented in `feature/safety-sensors-v4` branch.

### Files Modified
- `domecontrol_JK3.ino` — All Arduino changes (sensors, dynamic timeout, frozen dome, events, web UI)
- `astroshell_ticklogger.py` — New /event endpoint, Pushover integration, updated CSV format
- `CLAUDE.md` — Full v4.0 documentation
- `README.md` — Updated with new features, wiring, sensor setup
