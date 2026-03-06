using System;
using System.Collections;
using System.Runtime.InteropServices;
using System.Threading;
// Must use global:: because our namespace (ASCOM.AstroShellDome) nests under ASCOM,
// causing C# to search parent namespaces before using directives
using ASCOM.DeviceInterface;
using ASCOM.Utilities;
using ShutterState = global::ASCOM.DeviceInterface.ShutterState;

namespace ASCOM.AstroShellDome
{
    // v1.1.0 — Native ASCOM COM driver for AstroShell two-shutter dome controller
    // Added: individual shutter Actions, timed open command
    [ComVisible(true)]
    [Guid("a7e3d1c5-4b29-4f6a-8c0e-3d1a5b7f9e2c")]
    [ClassInterface(ClassInterfaceType.None)]
    [ProgId(Dome.DriverId)]
    public class Dome : IDomeV2
    {
        public const string DriverId = "ASCOM.AstroShellDome.Dome";
        private const string DriverDescription = "AstroShell Dome";
        private const string ProfileName = "AstroShell Dome";

        // Default settings
        private const string DefaultIP = "192.168.1.177";
        private const int DefaultPort = 80;
        private const bool DefaultSwapOpenClose = false;

        // Profile keys
        private const string IpProfileKey = "ArduinoIP";
        private const string PortProfileKey = "ArduinoPort";
        private const string SwapProfileKey = "SwapOpenClose";

        // Custom Action names
        private const string ActionOpenEast = "OpenEast";
        private const string ActionCloseEast = "CloseEast";
        private const string ActionOpenWest = "OpenWest";
        private const string ActionCloseWest = "CloseWest";
        private const string ActionTimedOpen = "TimedOpen";
        private const string ActionTimedOpenEast = "TimedOpenEast";
        private const string ActionTimedOpenWest = "TimedOpenWest";
        private const string ActionStop = "Stop";

        // Timed open constraints
        private const int TimedOpenMinSeconds = 1;
        private const int TimedOpenMaxSeconds = 120;

        private bool _connected;
        private ArduinoClient _client;
        private System.Threading.Timer _timedStopTimer;

        // Settings loaded from ASCOM Profile
        private string _arduinoIP = DefaultIP;
        private int _arduinoPort = DefaultPort;
        private bool _swapOpenClose = DefaultSwapOpenClose;

        public Dome()
        {
            ReadProfile();
        }

        // ================================================================
        // ASCOM Registration — called by regasm to register with ASCOM Profile
        // ================================================================

        [ComRegisterFunction]
        public static void RegisterASCOM(Type t)
        {
            using (var p = new Profile())
            {
                p.DeviceType = "Dome";
                p.Register(DriverId, DriverDescription);
            }
        }

        [ComUnregisterFunction]
        public static void UnregisterASCOM(Type t)
        {
            using (var p = new Profile())
            {
                p.DeviceType = "Dome";
                p.Unregister(DriverId);
            }
        }

        // ================================================================
        // IDomeV2 — Connection & Setup
        // ================================================================

        public bool Connected
        {
            get { return _connected; }
            set
            {
                if (value == _connected)
                    return;

                if (value)
                {
                    // Connect — test reachability first
                    ReadProfile();
                    var testClient = new ArduinoClient(_arduinoIP, _arduinoPort, 5);
                    try
                    {
                        testClient.GetSimpleStatus();
                    }
                    catch (Exception ex)
                    {
                        testClient.Dispose();
                        throw new ASCOM.NotConnectedException(
                            $"Cannot reach AstroShell at {_arduinoIP}:{_arduinoPort} — {ex.Message}");
                    }

                    _client?.Dispose();
                    _client = testClient;
                    _connected = true;
                }
                else
                {
                    // Disconnect — cancel any timed stop
                    CancelTimedStop();
                    _connected = false;
                    _client?.Dispose();
                    _client = null;
                }
            }
        }

        public void SetupDialog()
        {
            ReadProfile();

            using (var form = new SetupDialogForm())
            {
                form.ArduinoIP = _arduinoIP;
                form.ArduinoPort = _arduinoPort;
                form.SwapOpenClose = _swapOpenClose;

                if (form.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                {
                    _arduinoIP = form.ArduinoIP;
                    _arduinoPort = form.ArduinoPort;
                    _swapOpenClose = form.SwapOpenClose;
                    WriteProfile();

                    // Update live client if connected
                    if (_connected && _client != null)
                        _client.UpdateTarget(_arduinoIP, _arduinoPort);
                }
            }
        }

        // ================================================================
        // IDomeV2 — Capabilities
        // ================================================================

        public bool CanFindHome { get { return false; } }
        public bool CanPark { get { return false; } }
        public bool CanSetAltitude { get { return false; } }
        public bool CanSetAzimuth { get { return false; } }
        public bool CanSetPark { get { return false; } }
        public bool CanSetShutter { get { return true; } }
        public bool CanSlave { get { return false; } }
        public bool CanSyncAzimuth { get { return false; } }

        // ================================================================
        // IDomeV2 — Shutter Control
        // ================================================================

        public ShutterState ShutterStatus
        {
            get
            {
                CheckConnected();
                try
                {
                    var status = _client.GetDetailedStatus();

                    // If any motor is running, report the direction
                    if (status.Motor1Direction == MotorDirection.Opening ||
                        status.Motor2Direction == MotorDirection.Opening)
                        return ShutterState.shutterOpening;

                    if (status.Motor1Direction == MotorDirection.Closing ||
                        status.Motor2Direction == MotorDirection.Closing)
                        return ShutterState.shutterClosing;

                    // Both motors stopped — check physical state
                    if (status.Shutter1State == PhysicalShutterState.Open &&
                        status.Shutter2State == PhysicalShutterState.Open)
                        return ShutterState.shutterOpen;

                    if (status.Shutter1State == PhysicalShutterState.Closed &&
                        status.Shutter2State == PhysicalShutterState.Closed)
                        return ShutterState.shutterClosed;

                    // Mixed state (one open, one closed, or intermediate)
                    return ShutterState.shutterError;
                }
                catch (Exception)
                {
                    return ShutterState.shutterError;
                }
            }
        }

        public void OpenShutter()
        {
            CheckConnected();

            if (_swapOpenClose)
            {
                _client.SendCommand("$3");
                Thread.Sleep(3000);
                _client.SendCommand("$1");
            }
            else
            {
                _client.SendCommand("$4"); // West open first (outer shutter)
                Thread.Sleep(3000);
                _client.SendCommand("$2"); // East open second (inner shutter)
            }
        }

        public void CloseShutter()
        {
            CheckConnected();

            if (_swapOpenClose)
            {
                _client.SendCommand("$4");
                Thread.Sleep(3000);
                _client.SendCommand("$2");
            }
            else
            {
                _client.SendCommand("$3"); // West close first
                Thread.Sleep(3000);
                _client.SendCommand("$1"); // East close second
            }
        }

        public void AbortSlew()
        {
            CheckConnected();
            CancelTimedStop();
            _client.SendCommand("$5"); // Emergency STOP all motors
        }

        public bool Slewing
        {
            get
            {
                CheckConnected();
                try
                {
                    var status = _client.GetDetailedStatus();
                    return status.AnyMotorRunning;
                }
                catch
                {
                    return false;
                }
            }
        }

        // ================================================================
        // Custom Actions — Individual shutter control + timed open
        // ================================================================

        public ArrayList SupportedActions
        {
            get
            {
                return new ArrayList
                {
                    ActionOpenEast,
                    ActionCloseEast,
                    ActionOpenWest,
                    ActionCloseWest,
                    ActionTimedOpen,
                    ActionTimedOpenEast,
                    ActionTimedOpenWest,
                    ActionStop
                };
            }
        }

        /// <summary>
        /// Custom Actions for individual shutter control and timed opening.
        ///
        /// Individual shutters:
        ///   Action("OpenEast", "")   — open Shutter 1 (East) only
        ///   Action("CloseEast", "")  — close Shutter 1 (East) only
        ///   Action("OpenWest", "")   — open Shutter 2 (West) only
        ///   Action("CloseWest", "")  — close Shutter 2 (West) only
        ///   Action("Stop", "")       — emergency stop all motors
        ///
        /// Timed opening (opens for N seconds then auto-stops):
        ///   Action("TimedOpen", "30")      — open both shutters for 30 seconds
        ///   Action("TimedOpenEast", "30")   — open East shutter for 30 seconds
        ///   Action("TimedOpenWest", "30")   — open West shutter for 30 seconds
        ///
        /// Returns "OK" on success. Timed open returns "OK:30s" with actual duration.
        /// </summary>
        public string Action(string actionName, string actionParameters)
        {
            CheckConnected();

            switch (actionName)
            {
                case ActionOpenEast:
                    SendShutterCommand(true, true); // East, open
                    return "OK";

                case ActionCloseEast:
                    SendShutterCommand(true, false); // East, close
                    return "OK";

                case ActionOpenWest:
                    SendShutterCommand(false, true); // West, open
                    return "OK";

                case ActionCloseWest:
                    SendShutterCommand(false, false); // West, close
                    return "OK";

                case ActionStop:
                    CancelTimedStop();
                    _client.SendCommand("$5");
                    return "OK";

                case ActionTimedOpen:
                    return ExecuteTimedOpen(actionParameters, true, true);

                case ActionTimedOpenEast:
                    return ExecuteTimedOpen(actionParameters, true, false);

                case ActionTimedOpenWest:
                    return ExecuteTimedOpen(actionParameters, false, true);

                default:
                    throw new ASCOM.ActionNotImplementedException(actionName);
            }
        }

        /// <summary>
        /// Send a command to a specific shutter, respecting swap_open_close setting.
        /// </summary>
        private void SendShutterCommand(bool isEast, bool isOpen)
        {
            if (_swapOpenClose)
                isOpen = !isOpen;

            if (isEast)
                _client.SendCommand(isOpen ? "$2" : "$1");
            else
                _client.SendCommand(isOpen ? "$4" : "$3");
        }

        /// <summary>
        /// Open one or both shutters for a specified duration, then auto-stop.
        /// The motor will run for the requested seconds, then $5 (STOP) is sent.
        /// Arduino's own safety timeout (107s) still applies as a backstop.
        /// </summary>
        private string ExecuteTimedOpen(string parameters, bool east, bool west)
        {
            int seconds;
            if (!int.TryParse(parameters, out seconds) || seconds < TimedOpenMinSeconds || seconds > TimedOpenMaxSeconds)
                throw new ASCOM.InvalidValueException(
                    $"TimedOpen requires seconds between {TimedOpenMinSeconds} and {TimedOpenMaxSeconds}, got: '{parameters}'");

            // Cancel any previous timed stop
            CancelTimedStop();

            // Start the shutter(s)
            if (west)
                SendShutterCommand(false, true);
            if (east)
            {
                if (west) Thread.Sleep(3000); // Stagger if both
                SendShutterCommand(true, true);
            }

            // Schedule auto-stop after the requested duration
            _timedStopTimer = new System.Threading.Timer(TimedStopCallback, null, seconds * 1000, Timeout.Infinite);

            return $"OK:{seconds}s";
        }

        private void TimedStopCallback(object state)
        {
            try
            {
                if (_connected && _client != null)
                    _client.SendCommand("$5");
            }
            catch
            {
                // Best-effort stop — Arduino safety timeout is the backstop
            }
        }

        private void CancelTimedStop()
        {
            if (_timedStopTimer != null)
            {
                _timedStopTimer.Dispose();
                _timedStopTimer = null;
            }
        }

        // ================================================================
        // IDomeV2 — Properties not supported (no rotation/azimuth)
        // ================================================================

        public double Altitude
        {
            get { throw new ASCOM.PropertyNotImplementedException("Altitude", false); }
        }

        public bool AtHome
        {
            get { return false; }
        }

        public bool AtPark
        {
            get { return false; }
        }

        public double Azimuth
        {
            get { throw new ASCOM.PropertyNotImplementedException("Azimuth", false); }
        }

        public bool Slaved
        {
            get { return false; }
            set { throw new ASCOM.PropertyNotImplementedException("Slaved", true); }
        }

        // ================================================================
        // IDomeV2 — Methods not supported
        // ================================================================

        public void FindHome()
        {
            throw new ASCOM.MethodNotImplementedException("FindHome");
        }

        public void Park()
        {
            throw new ASCOM.MethodNotImplementedException("Park");
        }

        public void SetPark()
        {
            throw new ASCOM.MethodNotImplementedException("SetPark");
        }

        public void SlewToAltitude(double altitude)
        {
            throw new ASCOM.MethodNotImplementedException("SlewToAltitude");
        }

        public void SlewToAzimuth(double azimuth)
        {
            throw new ASCOM.MethodNotImplementedException("SlewToAzimuth");
        }

        public void SyncToAzimuth(double azimuth)
        {
            throw new ASCOM.MethodNotImplementedException("SyncToAzimuth");
        }

        // ================================================================
        // IDomeV2 — Common ASCOM properties
        // ================================================================

        public string Name
        {
            get { return DriverDescription; }
        }

        public string Description
        {
            get { return "AstroShell two-shutter dome controller via HTTP"; }
        }

        public string DriverInfo
        {
            get { return "AstroShell Dome ASCOM Driver v1.1.0"; }
        }

        public string DriverVersion
        {
            get { return "1.1"; }
        }

        public short InterfaceVersion
        {
            get { return 2; }
        }

        public void CommandBlind(string command, bool raw = false)
        {
            CheckConnected();
            _client.SendCommand(command);
        }

        public bool CommandBool(string command, bool raw = false)
        {
            throw new ASCOM.MethodNotImplementedException("CommandBool");
        }

        public string CommandString(string command, bool raw = false)
        {
            CheckConnected();
            return _client.GetSimpleStatus();
        }

        public void Dispose()
        {
            CancelTimedStop();
            Connected = false;
        }

        // ================================================================
        // Profile Read/Write
        // ================================================================

        private void ReadProfile()
        {
            using (var p = new Profile())
            {
                p.DeviceType = "Dome";
                _arduinoIP = p.GetValue(DriverId, IpProfileKey, string.Empty, DefaultIP);
                var portStr = p.GetValue(DriverId, PortProfileKey, string.Empty, DefaultPort.ToString());
                int port;
                _arduinoPort = int.TryParse(portStr, out port) ? port : DefaultPort;
                var swapStr = p.GetValue(DriverId, SwapProfileKey, string.Empty, DefaultSwapOpenClose.ToString());
                bool swap;
                _swapOpenClose = bool.TryParse(swapStr, out swap) && swap;
            }
        }

        private void WriteProfile()
        {
            using (var p = new Profile())
            {
                p.DeviceType = "Dome";
                p.WriteValue(DriverId, IpProfileKey, _arduinoIP);
                p.WriteValue(DriverId, PortProfileKey, _arduinoPort.ToString());
                p.WriteValue(DriverId, SwapProfileKey, _swapOpenClose.ToString());
            }
        }

        private void CheckConnected()
        {
            if (!_connected)
                throw new ASCOM.NotConnectedException("Not connected to AstroShell dome");
        }
    }
}
