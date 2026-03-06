# AstroShell — Task Tracking

## Active

### ASCOM COM Driver (v1.0.0) — feature/ascom-driver
- [x] Create branch `feature/ascom-driver`
- [x] ArduinoClient.cs — HTTP client using $A endpoint
- [x] Dome.cs — IDomeV2 implementation
- [x] SetupDialogForm — WinForms setup dialog with test connection
- [x] AssemblyInfo.cs — COM visibility, GUID
- [x] AstroShellDome.csproj + .sln — .NET Framework 4.8
- [x] install.bat / uninstall.bat — regasm registration scripts
- [x] GitHub Actions CI — build-ascom-driver.yml
- [x] Arduino firmware: add $A ASCOM status endpoint (v4.0.3)
- [x] Update CLAUDE.md, README.md, ascom-driver/README.md
- [ ] Verify GitHub Actions build succeeds
- [ ] Test on Windows with NINA

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
