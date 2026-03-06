using System;
using System.Net.Http;
using System.Threading.Tasks;

namespace ASCOM.AstroShellDome
{
    /// <summary>
    /// HTTP client for communicating with the AstroShell Arduino dome controller.
    /// Uses the dedicated $A ASCOM endpoint for status (pipe-delimited, no HTML parsing).
    /// Thread-safe — uses a single HttpClient instance with synchronization.
    /// </summary>
    public class ArduinoClient : IDisposable
    {
        private readonly HttpClient _client;
        private readonly object _lock = new object();
        private string _baseUrl;

        // Cached status to avoid hammering the Arduino on rapid polls
        private DomeStatusInfo _cachedStatus;
        private DateTime _cacheTime = DateTime.MinValue;
        private static readonly TimeSpan CacheDuration = TimeSpan.FromSeconds(1);

        public ArduinoClient(string ip, int port, int timeoutSeconds = 5)
        {
            _baseUrl = $"http://{ip}:{port}";
            var handler = new HttpClientHandler
            {
                // Don't follow redirects — Arduino sends 303 after commands
                AllowAutoRedirect = false
            };
            _client = new HttpClient(handler)
            {
                Timeout = TimeSpan.FromSeconds(timeoutSeconds)
            };
        }

        /// <summary>
        /// Update the connection target without creating a new client.
        /// </summary>
        public void UpdateTarget(string ip, int port)
        {
            lock (_lock)
            {
                _baseUrl = $"http://{ip}:{port}";
                InvalidateCache();
            }
        }

        /// <summary>
        /// Test connectivity by requesting simple status.
        /// Returns "OPEN" or "CLOSED" on success, throws on failure.
        /// </summary>
        public string GetSimpleStatus()
        {
            lock (_lock)
            {
                var response = _client.GetAsync($"{_baseUrl}/?$S").Result;
                response.EnsureSuccessStatusCode();
                return response.Content.ReadAsStringAsync().Result.Trim();
            }
        }

        /// <summary>
        /// Get detailed dome status via the $A ASCOM endpoint.
        /// Response format: S1_STATE|S1_MOTOR|S2_STATE|S2_MOTOR
        /// Example: "CLOSED|STOPPED|CLOSED|STOPPED"
        /// Results are cached for 1 second to prevent excessive polling.
        /// </summary>
        public DomeStatusInfo GetDetailedStatus()
        {
            lock (_lock)
            {
                if (_cachedStatus != null && (DateTime.UtcNow - _cacheTime) < CacheDuration)
                    return _cachedStatus;

                var response = _client.GetAsync($"{_baseUrl}/?$A").Result;
                response.EnsureSuccessStatusCode();
                var body = response.Content.ReadAsStringAsync().Result.Trim();
                var status = ParseAscomResponse(body);

                _cachedStatus = status;
                _cacheTime = DateTime.UtcNow;
                return status;
            }
        }

        /// <summary>
        /// Send a command to the Arduino (fire-and-forget).
        /// Does not follow the 303 redirect response.
        /// </summary>
        public void SendCommand(string command)
        {
            lock (_lock)
            {
                try
                {
                    _client.GetAsync($"{_baseUrl}/?{command}").Result.Dispose();
                }
                catch (AggregateException ex) when (ex.InnerException is TaskCanceledException)
                {
                    // Timeout on redirect is expected — command was sent
                }
                InvalidateCache();
            }
        }

        /// <summary>
        /// Parse the $A endpoint response: "S1_STATE|S1_MOTOR|S2_STATE|S2_MOTOR"
        /// </summary>
        private DomeStatusInfo ParseAscomResponse(string response)
        {
            var info = new DomeStatusInfo();

            var parts = response.Split('|');
            if (parts.Length < 4)
                return info; // Malformed response — return defaults (intermediate/stopped)

            info.Shutter1State = ParseShutterState(parts[0]);
            info.Motor1Direction = ParseMotorDirection(parts[1]);
            info.Shutter2State = ParseShutterState(parts[2]);
            info.Motor2Direction = ParseMotorDirection(parts[3]);

            return info;
        }

        private PhysicalShutterState ParseShutterState(string token)
        {
            switch (token.Trim().ToUpperInvariant())
            {
                case "OPEN": return PhysicalShutterState.Open;
                case "CLOSED": return PhysicalShutterState.Closed;
                default: return PhysicalShutterState.Intermediate;
            }
        }

        private MotorDirection ParseMotorDirection(string token)
        {
            switch (token.Trim().ToUpperInvariant())
            {
                case "OPENING": return MotorDirection.Opening;
                case "CLOSING": return MotorDirection.Closing;
                default: return MotorDirection.Stopped;
            }
        }

        public void InvalidateCache()
        {
            _cachedStatus = null;
            _cacheTime = DateTime.MinValue;
        }

        public void Dispose()
        {
            _client?.Dispose();
        }
    }

    /// <summary>
    /// Physical state of a single shutter as reported by Arduino limit switches.
    /// </summary>
    public enum PhysicalShutterState
    {
        Open,
        Closed,
        Intermediate
    }

    /// <summary>
    /// Current motor direction for a shutter.
    /// </summary>
    public enum MotorDirection
    {
        Stopped,
        Opening,
        Closing
    }

    /// <summary>
    /// Combined dome status from the Arduino $A ASCOM endpoint.
    /// </summary>
    public class DomeStatusInfo
    {
        public PhysicalShutterState Shutter1State { get; set; } = PhysicalShutterState.Intermediate;
        public PhysicalShutterState Shutter2State { get; set; } = PhysicalShutterState.Intermediate;
        public MotorDirection Motor1Direction { get; set; } = MotorDirection.Stopped;
        public MotorDirection Motor2Direction { get; set; } = MotorDirection.Stopped;

        /// <summary>
        /// True if any motor is currently running.
        /// </summary>
        public bool AnyMotorRunning =>
            Motor1Direction != MotorDirection.Stopped ||
            Motor2Direction != MotorDirection.Stopped;
    }
}
