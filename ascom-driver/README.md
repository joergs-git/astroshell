# AstroShell Dome — ASCOM Driver

Native ASCOM COM driver for the AstroShell two-shutter dome controller. Communicates with the Arduino dome controller over HTTP using the existing web API.

## Requirements

- Windows 10/11 (64-bit)
- [ASCOM Platform 7](https://github.com/ASCOMInitiative/ASCOMPlatform/releases) (or 6.6+)
- .NET Framework 4.8 (included with Windows 10 1903+)
- AstroShell Arduino dome controller on the network

## Installation

### From GitHub Actions (recommended)

1. Go to the [Actions tab](../../actions) and download the latest `AstroShellDome-ASCOM-Driver` artifact
2. Extract the ZIP file to a permanent location (e.g. `C:\ASCOM\AstroShellDome\`)
3. Right-click `install.bat` → **Run as administrator**
4. Done — the driver is now registered with ASCOM

### From source

```bash
cd ascom-driver
dotnet restore AstroShellDome.sln
dotnet build AstroShellDome.sln --configuration Release
```

Then register with:
```
C:\Windows\Microsoft.NET\Framework64\v4.0.30319\RegAsm.exe /codebase AstroShellDome\bin\Release\ASCOM.AstroShellDome.dll
```

## NINA Setup

1. Open NINA → **Equipment** → **Dome**
2. Select **AstroShell Dome** from the driver dropdown
3. Click the **Setup** (wrench) icon
4. Enter your Arduino's IP address (default: `192.168.1.177`) and port (default: `80`)
5. Click **Test Connection** to verify
6. Click **OK**, then **Connect**

## Configuration

| Setting | Default | Description |
|---------|---------|-------------|
| Arduino IP | `192.168.1.177` | IP address of the dome controller |
| Port | `80` | HTTP port |
| Swap Open/Close | unchecked | Reverses command mapping for domes wired opposite |

### Swap Open/Close

The AstroShell dome has inverted limit switch wiring (documented in main README). The default command mapping matches the standard AstroShell configuration:

- **Unchecked (default):** `OpenShutter()` sends `$4` (West) + `$2` (East) = physically opens
- **Checked:** `OpenShutter()` sends `$3` (West) + `$1` (East) = reversed for opposite wiring

## How It Works

- **ShutterStatus**: Uses the Arduino `$A` endpoint for per-shutter state and motor direction
- **OpenShutter()**: Sends open commands to both shutters with a 3-second delay between them (West first, then East)
- **CloseShutter()**: Sends close commands to both shutters with a 3-second delay
- **AbortSlew()**: Sends `$5` (emergency STOP all motors), cancels any timed open
- **Slewing**: Returns `true` while any motor is running
- Status is cached for 1 second to prevent hammering the Arduino

## Custom Actions (v1.1.0)

The driver supports custom Actions for individual shutter control and timed opening. These can be called from NINA's Advanced Sequencer or any ASCOM client.

### Individual Shutter Control

| Action | Description |
|--------|-------------|
| `Action("OpenEast", "")` | Open Shutter 1 (East) only |
| `Action("CloseEast", "")` | Close Shutter 1 (East) only |
| `Action("OpenWest", "")` | Open Shutter 2 (West) only |
| `Action("CloseWest", "")` | Close Shutter 2 (West) only |
| `Action("Stop", "")` | Emergency stop all motors |

### Timed Opening

Opens shutter(s) for a specified number of seconds (1-120), then automatically sends STOP. Useful for partial ventilation or cooling without fully opening the dome.

| Action | Description |
|--------|-------------|
| `Action("TimedOpen", "30")` | Open both shutters for 30 seconds |
| `Action("TimedOpenEast", "30")` | Open East shutter for 30 seconds |
| `Action("TimedOpenWest", "30")` | Open West shutter for 30 seconds |

The Arduino's own safety timeout (107 seconds) still applies as a backstop — if the driver's stop command fails to reach the Arduino, the motor will stop on its own.

**Note:** Timed opening does NOT provide precise position control. The dome has no position sensors, so the actual opening amount depends on motor speed (which varies with temperature). This is useful for "crack the dome open for ventilation" scenarios, not for precise positioning.

## ASCOM Capabilities

| Capability | Supported |
|-----------|-----------|
| CanSetShutter | Yes |
| CanFindHome | No |
| CanPark | No |
| CanSetAzimuth | No |
| CanSetAltitude | No |
| CanSlave | No (dome does not rotate) |

## Uninstall

1. Right-click `uninstall.bat` → **Run as administrator**
2. Delete the driver folder

## Version History

| Version | Changes |
|---------|---------|
| 1.1.0 | Individual shutter Actions, timed open command (1-120s auto-stop) |
| 1.0.0 | Initial release — shutter control, status monitoring, setup dialog |
