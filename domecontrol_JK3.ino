//=============================================================================
// ASTROSHELL DOME CONTROLLER - SAFETY SENSOR EDITION (v4.1.0)
//=============================================================================
// Hardware: Arduino MEGA 2560 + Ethernet Shield (W5100/W5500)
//           + DS18B20 temperature probe (pin 22)
//           + VL53L0X Time-of-Flight sensor (I2C pins 20/21)
// Purpose:  Controls two-shutter astronomical dome with automatic rain protection,
//           temperature-based dynamic motor timeout, and frozen dome detection
// Author:   joergsflow (enhanced from original AstroShell code)
//
// Features:
// - Dual DC motor control with soft-start PWM
// - Web interface for remote control (smartphone/tablet friendly)
// - Automatic dome closure on Cloudwatcher IP failure (safety feature)
// - Hardware watchdog for system recovery
// - EEPROM persistence for failure counters
// - Physical button control with debouncing
// - Motor runtime tick logging for temperature correlation analysis
// - DS18B20 temperature-based dynamic motor timeout (v4.0)
// - VL53L0X frozen dome detection with auto-retry and lockout (v4.0)
// - Event notifications to Solo Pi for Pushover alerts (v4.0)
// - Conflicting signal detection: limit switches vs ToF (v4.0)
//
// v4.1.0 Changes (network/robustness review):
// - Ethernet: single chip reset per stack rebuild, health check flags dead stack (LinkOFF / 0.0.0.0),
//   rebuild only with motors idle and rate-limited (60 s), 8-h preventive reset only with dome closed
// - socketHygiene(): reclaims idle/dead W5500 sockets, keeps two LISTEN sockets (rain-checker close no longer dropped)
// - HTTP: '$' commands parsed from the request line only; page writes bounded (PageWriter), no unbounded flush()
// - I2C: Wire timeout 25 ms + bus recovery, VL53L0X init bracketed by wdt_reset()
// - Watchdog enabled first in setup(); motor pins LOW before anything else
// - Frozen dome: per-motor external-command mask, attempt counted at detection, lockout survives cancel,
//   urgent events bypass rate limiter, 30-min attempt memory
// - Atomic writes of dynamic timeouts; soft-start on direction change; sensor_fail/sensor_ok events
//
// v4.0.3 Changes:
// - Added $A ASCOM status endpoint: compact pipe-delimited response for native ASCOM driver
//   Format: S1_STATE|S1_MOTOR|S2_STATE|S2_MOTOR (e.g. "CLOSED|STOPPED|CLOSED|STOPPED")
//   States: OPEN, CLOSED, INTERMEDIATE — Motors: STOPPED, OPENING, CLOSING
//   Read-only endpoint, works even during system init
//
// v4.0 Changes:
// - DS18B20 temperature probe on pin 22 for ambient temperature reading
// - Dynamic motor timeout: linear regression model from 253 cycles analysis
// - Graceful degradation: falls back to static 6527 timeout on sensor failure
// - VL53L0X ToF sensor on I2C (pins 20/21) for frozen dome detection
// - EEPROM-stored ToF calibration baseline ($C command)
// - Frozen dome state machine: 3 retries with 20s gravity wait, then lockout
// - $U unlock command to clear frozen lockout
// - Event notifications pushed to Solo:88/event for Pushover delivery
// - Conflicting signal detection (limit switches vs ToF disagreement)
// - Temperature and ToF distance included in all HTTP posts to Solo
// - New stop reason code 6: FROZEN DOME Auto-Reverse
//
// v3.3 Changes:
// - Added motor runtime measurement in ISR ticks
// - Only valid full-runs are recorded (start at limit, stop at opposite limit)
// - Web UI toggle ($L) to enable/disable logging (default: off after reboot)
// - HTTP GET push to Cloudwatcher Solo:88 for CSV logging
// - Web UI shows live tick counter and last valid measurements
//
// Safety Note: Limit switch names are swapped due to installation wiring.
// "lim1open" actually detects physically CLOSED state, etc.
//=============================================================================

// --- Motor Timing Configuration ---
// Motor timeout in ISR ticks (~61 Hz). Safety limit if limit switches fail.
// Calculation: 107 seconds * 61 Hz = 6527 ticks
#define SMOOTH 30           // Soft-start smoothness (0=instant, 254=very slow)
#define MAX_MOT1_OPEN  6527 // Shutter 1 close timeout (107 sec)
#define MAX_MOT1_CLOSE 6527 // Shutter 1 open timeout (107 sec)
#define MAX_MOT1_VCC_CLOSE_ACTION MAX_MOT1_OPEN
#define MAX_MOT2_OPEN  6527 // Shutter 2 close timeout (107 sec)
#define MAX_MOT2_CLOSE 6527 // Shutter 2 open timeout (107 sec)
#define MAX_MOT2_VCC_CLOSE_ACTION MAX_MOT2_OPEN

// --- Network Configuration ---
// Static IP address for the dome controller
#define IP_ADR0 192
#define IP_ADR1 168
#define IP_ADR2 1
#define IP_ADR3 177

// --- Voltage Monitoring ---
#define VCC_RAW_MAX 580     // ADC calibration for 24V reading 

// --- Libraries ---
#include <SPI.h>          // SPI communication for Ethernet Shield
#include <Ethernet.h>     // W5100/W5500 Ethernet library
#include <utility/w5100.h> // W5100 mode register (chip reset) and SnSR socket states (P1)
#include <EEPROM.h>       // Persistent storage for counters
#include <avr/wdt.h>      // Hardware watchdog for automatic recovery
#include <OneWire.h>      // OneWire protocol for DS18B20 temperature probe
#include <DallasTemperature.h>  // DS18B20 high-level API
#include <Wire.h>         // I2C for VL53L0X (built-in)
#include <VL53L0X.h>      // Pololu VL53L0X Time-of-Flight sensor

// --- Feature Toggles ---
#define ENABLE_IP_AUTO_CLOSE        // Enable auto-close on Cloudwatcher IP failure
// #define ENABLE_VCC_FAIL_AUTO_CLOSE  // Disabled: VCC monitoring caused Ethernet issues

// --- Motor Direction Constants ---
// NOTE: Names are inverted due to hardware wiring swap!
#define OPEN        1     // Sends motor command that PHYSICALLY CLOSES the shutter
#define CLOSE       2     // Sends motor command that PHYSICALLY OPENS the shutter
                          // 0 = motor off

//=============================================================================
// HARDWARE PIN MAPPING - Arduino MEGA 2560
//=============================================================================

// Motor PWM outputs (directly drive H-bridge)
#define motor1a     6     // Shutter 1 motor pin A (PWM capable)
#define motor1b     9     // Shutter 1 motor pin B (PWM capable)
#define motor2a     5     // Shutter 2 motor pin A (PWM capable)
#define motor2b     3     // Shutter 2 motor pin B (PWM capable)

// Limit switches (active HIGH when endpoint reached)
// WARNING: Names are swapped due to installation wiring error!
#define lim1open    7     // Actually detects S1 PHYSICALLY CLOSED
#define lim1closed  2     // Actually detects S1 PHYSICALLY OPEN
#define lim2open    1     // Actually detects S2 PHYSICALLY CLOSED (shares TX pin!)
#define lim2closed  0     // Actually detects S2 PHYSICALLY OPEN (shares RX pin!)

// Manual control buttons (active LOW with internal pullup)
#define SW1up       A3    // Shutter 1 close button
#define SW1down     A2    // Shutter 1 open button
#define SW2up       A5    // Shutter 2 close button
#define SW2down     A4    // Shutter 2 open button
#define SWSTOP      8     // Emergency STOP button (stops all motors)

// Voltage monitoring (analog inputs)
#define VCC1        A1    // Main power supply voltage
#define VCC2        A0    // Secondary/backup voltage (currently unused)

// --- DS18B20 Temperature Probe ---
// Pin 22 is MEGA-exclusive (not on UNO), guarantees no conflict with existing wiring
#define DS18B20_PIN 22    // OneWire data pin (4.7k pullup to 5V, external Vcc power)

// --- VL53L0X Time-of-Flight Sensor ---
// I2C pins 20 (SDA) and 21 (SCL) are MEGA-exclusive, both currently free
// No pin defines needed — Wire library uses hardware I2C pins automatically

//=============================================================================
// EEPROM MEMORY MAP - Persistent storage across reboots
//=============================================================================
#define EEPROM_MAGIC_BYTE 0x42        // Identifies initialized EEPROM
#define EEPROM_ADDR_MAGIC 0           // Byte 0: Magic byte
#define EEPROM_ADDR_TOTAL_IP_FAILS 1  // Bytes 1-2: Total IP failures (uint16)
#define EEPROM_ADDR_AUTO_CLOSES 3     // Bytes 3-4: Auto-close events (uint16)
#define EEPROM_ADDR_LAST_FAIL_DAY 5   // Byte 5: Uptime day counter
#define EEPROM_ADDR_TOF_BASELINE 6    // Bytes 6-7: ToF baseline distance in mm (uint16)
#define EEPROM_ADDR_TOF_CALIB_MAGIC 8 // Byte 8: ToF calibration magic (0xA5 = calibrated)
#define TOF_CALIB_MAGIC_BYTE 0xA5     // Identifies valid ToF calibration

//=============================================================================
// DYNAMIC TIMEOUT CONFIGURATION — Linear Regression Coefficients
//=============================================================================
// Derived from 253 motor cycle measurements (-1.6C to 22.6C).
// Formula: timeout = base - (slope100 * temp_C) / 100 + margin
// All values in ISR ticks (~61 Hz). Integer math to avoid float in ISR.
//
// Direction mapping (code names are inverted due to wiring!):
//   mot1dir==OPEN  = physically closing -> M1 Closing regression
//   mot1dir==CLOSE = physically opening -> M1 Opening regression
//   mot2dir==OPEN  = physically closing -> M2 Closing regression
//   mot2dir==CLOSE = physically opening -> M2 Opening regression

#define DYN_M1_CLOSE_BASE    6096   // M1 closing: base ticks at 0C
#define DYN_M1_CLOSE_SLOPE   1945   // M1 closing: slope * 100 (19.45 ticks/C)
#define DYN_M1_CLOSE_MARGIN  601    // M1 closing: safety margin ticks

#define DYN_M1_OPEN_BASE     5629   // M1 opening: base ticks at 0C
#define DYN_M1_OPEN_SLOPE    1454   // M1 opening: slope * 100 (14.54 ticks/C)
#define DYN_M1_OPEN_MARGIN   557    // M1 opening: safety margin ticks

#define DYN_M2_CLOSE_BASE    5700   // M2 closing: base ticks at 0C
#define DYN_M2_CLOSE_SLOPE   1613   // M2 closing: slope * 100 (16.13 ticks/C)
#define DYN_M2_CLOSE_MARGIN  563    // M2 closing: safety margin ticks

#define DYN_M2_OPEN_BASE     5248   // M2 opening: base ticks at 0C
#define DYN_M2_OPEN_SLOPE    1591   // M2 opening: slope * 100 (15.91 ticks/C)
#define DYN_M2_OPEN_MARGIN   518    // M2 opening: safety margin ticks

#define DYN_TIMEOUT_MIN      2000   // Minimum dynamic timeout (clamp floor)
#define DYN_TIMEOUT_MAX      6527   // Maximum dynamic timeout (same as static)
#define TEMP_FAIL_THRESHOLD   10    // Consecutive failures before fallback to static

//=============================================================================
// FROZEN DOME DETECTION CONFIGURATION
//=============================================================================
#define TOF_OPEN_TOLERANCE     30    // mm: distance increase to detect opening
#define TOF_CHECK_TICKS       244    // ISR ticks before first ToF check (~4 sec)
#define FROZEN_GRAVITY_WAIT  20000   // ms: gravity wait with motor off
#define FROZEN_RETRY_WAIT     5000   // ms: wait between retry attempts
#define FROZEN_MAX_RETRIES       3   // Max open attempts before lockout
#define FROZEN_ATTEMPT_MEMORY 1800000UL // ms: attempt counter forgets after 30 min without a frozen verdict (P1)

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
// NOTE: Variables accessed by ISR must be declared 'volatile' to prevent
// compiler optimization issues. The ISR runs at ~61 Hz independently.

// --- Web Request Parsing ---
boolean newInfo;                      // True when parsing URL parameter after '$'

// --- Button State Tracking (ISR + main loop) ---
volatile boolean sw1up_pressed_flag, sw1down_pressed_flag;
volatile boolean sw2up_pressed_flag, sw2down_pressed_flag;

// --- Motor Control (ISR + main loop) ---
volatile byte cnt = 0;                // Debounce counter for buttons
volatile byte mot1dir = 0;            // Motor 1 direction: 0=off, 1=OPEN, 2=CLOSE
volatile byte mot2dir = 0;            // Motor 2 direction: 0=off, 1=OPEN, 2=CLOSE
volatile byte mot1speed = 0;          // Motor 1 PWM value (0-255, soft-start ramp)
volatile byte mot2speed = 0;          // Motor 2 PWM value (0-255, soft-start ramp)
volatile word mot1timer = 0;          // Motor 1 timeout countdown (ISR ticks)
volatile word mot2timer = 0;          // Motor 2 timeout countdown (ISR ticks)

// --- Stop Reason Tracking ---
// 0=Limit switch, 1=Button/SWSTOP, 2=Web command, 3=IP failure, 4=VCC failure, 5=Timeout
volatile byte stop1reason = 0;
volatile byte stop2reason = 0;

// --- Other State ---
byte vccerr = 0;                      // VCC error counter (unused currently)
volatile byte swstop_pressed_flag = 0; // STOP button state

//=============================================================================
// DS18B20 TEMPERATURE SENSOR
//=============================================================================
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);

// Temperature reading state (non-blocking async pattern)
int currentTemp_x10 = -9990;             // Current temp in tenths of C (-999.0 sentinel)
bool ds18b20_connected = false;           // True if sensor responding
byte tempFailCount = 0;                   // Consecutive read failures
unsigned long lastTempRequest = 0;        // When conversion was requested
unsigned long lastTempRead = 0;           // When last successful read happened
bool tempConversionPending = false;       // True while waiting for conversion result
const unsigned long TEMP_READ_INTERVAL = 5000;  // Request every 5 seconds
const unsigned long TEMP_CONV_TIME = 800;       // 750ms conversion + 50ms margin

// Dynamic timeout values — precomputed from temperature, read atomically by ISR
volatile word dynTimeout_M1_Close = MAX_MOT1_OPEN;   // mot1dir==OPEN = physically closing
volatile word dynTimeout_M1_Open  = MAX_MOT1_CLOSE;  // mot1dir==CLOSE = physically opening
volatile word dynTimeout_M2_Close = MAX_MOT2_OPEN;   // mot2dir==OPEN = physically closing
volatile word dynTimeout_M2_Open  = MAX_MOT2_CLOSE;  // mot2dir==CLOSE = physically opening
bool dynamicTimeoutActive = false;        // True when DS18B20 is providing valid temps

// FIX (P1): the four 16-bit timeout values are read by the ISR when a button starts a motor;
// 16-bit stores are NOT atomic on the 8-bit AVR (the old comment claimed the opposite), so
// every writer goes through this helper.
static void setDynamicTimeouts(word m1c, word m1o, word m2c, word m2o) {
  uint8_t sreg = SREG; cli();
  dynTimeout_M1_Close = m1c; dynTimeout_M1_Open = m1o;
  dynTimeout_M2_Close = m2c; dynTimeout_M2_Open = m2o;
  SREG = sreg;
}

//=============================================================================
// VL53L0X TIME-OF-FLIGHT SENSOR
//=============================================================================
VL53L0X tofSensor;

// ToF reading state
int tofDistance_mm = -1;                  // Current distance in mm (-1 = invalid/disconnected)
bool tof_connected = false;               // True if sensor responding
byte tofFailCount = 0;                    // Consecutive read failures
unsigned long lastTofRead = 0;            // Last successful read time
const unsigned long TOF_READ_INTERVAL = 200;   // Read every 200ms

// ToF calibration (EEPROM-stored baseline)
word tofBaseline_mm = 0;                  // Calibrated "closed" distance in mm
bool tofCalibrated = false;               // True if valid calibration in EEPROM

//=============================================================================
// FROZEN DOME STATE MACHINE
//=============================================================================
// States for frozen dome detection — runs in loop(), not ISR
enum FrozenDomeState {
  FD_IDLE,            // No frozen check in progress
  FD_MONITORING,      // Motor opening, counting ticks for first check
  FD_FIRST_CHECK,     // Check ToF after ~4 sec of opening
  FD_GRAVITY_WAIT,    // Motor stopped, waiting for gravity to separate halves
  FD_SECOND_CHECK,    // Check ToF again after gravity wait
  FD_CLOSING,         // Reversing motor to close (frozen confirmed)
  FD_RETRY_WAIT,      // Waiting before next retry attempt
  FD_LOCKOUT          // All retries exhausted, dome locked
};

FrozenDomeState frozenDomeState = FD_IDLE;
volatile word frozenCheckTicks = 0;       // ISR tick counter for frozen check timing
volatile bool frozenCheckActive = false;  // True when ISR should count frozenCheckTicks
byte frozenRetryCount = 0;               // Current retry attempt (0-based)
unsigned long frozenStateTimer = 0;       // Timer for gravity wait, retry wait etc.
byte frozenMotorNum = 0;                 // Which motor triggered frozen check (1 or 2)
unsigned long lastFrozenDetect = 0;      // millis() of the last frozen verdict (P1)
// FIX (P1): bitmask of motor commands issued OUTSIDE the frozen-dome cycle since the state
// machine last looked. bit0 = shutter 1 (web $1/$2, S1 buttons), bit1 = shutter 2 ($3/$4,
// S2 buttons); $5, SWSTOP and the IP/cable auto-close set both. The state machine hands
// control back when a command addresses the shutter it is working on, so it can never
// re-open a dome that somebody else just stopped or closed on purpose.
volatile byte fdExternalCommand = 0;
#define FD_CMD_M1  1
#define FD_CMD_M2  2
#define FD_CMD_ALL 3

//=============================================================================
// EVENT NOTIFICATION
//=============================================================================
unsigned long lastEventSentTime = 0;      // Rate limiter: min 10s between events
const unsigned long EVENT_MIN_INTERVAL = 10000UL;  // 10 seconds minimum between events
unsigned long lastConflictEventTime = 0;  // Separate rate limit for conflict events
const unsigned long CONFLICT_EVENT_INTERVAL = 300000UL;  // 5 minutes between conflict events

// --- Ethernet Configuration ---
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};  // MAC address (change if multiple units)
IPAddress ip(IP_ADR0, IP_ADR1, IP_ADR2, IP_ADR3);   // This controller's IP
EthernetServer server(80);                          // Web server on port 80

// --- Cloudwatcher Monitoring Target ---
// The dome monitors this IP (Lunatico Cloudwatcher Solo) for connectivity.
// If unreachable for too long, dome auto-closes as safety precaution.
IPAddress remoteStationIp(192, 168, 1, 151);
const int remoteStationPort = 80;

//=============================================================================
// IP FAILURE DETECTION CONFIGURATION
//=============================================================================
// Auto-close triggers after 10 failed connection attempts within 30 minutes.
// This ensures dome closes when Cloudwatcher is genuinely unreachable,
// while tolerating brief network glitches (switch hiccups, Solo busy, etc.).
// Timing: 10 checks × 90 second interval = 15 minutes minimum response time.

byte connectFailCount = 0;                          // Current consecutive failures
unsigned long lastConnectAttemptTimestamp = 0;      // Last connection check time
const unsigned long connectCheckInterval = 90000UL; // Check every 90 seconds
const byte maxConnectFails = 10;                    // Failures needed to trigger
unsigned long firstFailTimestamp = 0;               // When failure window started
const unsigned long maxFailTimeWindow = 1800000UL;  // 30-minute window for counting

// --- Persistent Statistics (saved to EEPROM) ---
unsigned int totalIpFailures = 0;     // Lifetime IP failure count
unsigned int totalAutoCloses = 0;     // Lifetime auto-close count
byte dayCounter = 0;                  // Days of continuous operation (0-255)
bool eepromDirty = false;             // True if counters need saving

// --- Auto-Close State Tracking ---
bool m1AutoClosedByIP = false;        // True if M1 was auto-closed due to IP fail
bool m2AutoClosedByIP = false;        // True if M2 was auto-closed due to IP fail
bool cableRemovalAutoCloseTriggered = false;  // Prevents repeated auto-close on cable removal
bool skipNextIpCheck = false;                 // Skip one IP check after Ethernet stack reset

// --- Network Monitoring Activation ---
// IP monitoring only activates once an Ethernet cable has been detected.
// This allows non-networked setups to operate without false auto-closes.
bool networkMonitoringEnabled = false; // True once cable detected (stays true)

//=============================================================================
// NETWORK WATCHDOG CONFIGURATION
//=============================================================================
// Periodically checks Ethernet health and performs preventive resets.

unsigned long lastNetworkCheck = 0;
const unsigned long NETWORK_CHECK_INTERVAL = 600000;   // Check every 10 minutes
bool ethernet_initialized = false;                     // True if Ethernet is working
unsigned long lastEthernetReset = 0;
unsigned long lastEthernetAttempt = 0;                 // FIX (P1): last setupEthernet() start (rate limit)
const unsigned long ETHERNET_RESET_INTERVAL = 28800000UL; // Preventive reset every 8 hours
unsigned long lastSuccessfulPing = 0;                  // Last successful activity

//=============================================================================
// DEBUG CONFIGURATION
//=============================================================================
// WARNING: Serial debug uses pins 0/1 which are also used for limit switches!
// Enabling debug may cause erratic motor behavior. Only enable for testing
// with limit switches temporarily disconnected.

// #define SERIAL_DEBUG_GENERAL   // General startup and web client messages
// #define SERIAL_DEBUG_IP        // IP monitoring and auto-close logic
// #define SERIAL_DEBUG_BUTTONS   // Button press detection
// #define SERIAL_DEBUG_LIMITS    // Limit switch triggering
// #define SERIAL_DEBUG_EEPROM    // EEPROM read/write operations

// --- System State ---
volatile boolean system_fully_ready = false;  // True after setup() completes

//=============================================================================
// TICK LOGGING - Motor Runtime Measurement
//=============================================================================
// Measures actual motor runtime in ISR ticks for temperature correlation analysis.
// Only valid full-runs are recorded (start at one limit, stop at opposite limit).
// Data can be pushed to external server for logging when toggle is enabled.

// --- Tick Logging Toggle (Web UI controllable, off after reboot) ---
bool tickLoggingEnabled = true;               // Enabled by default, toggle via web UI ($L)

// --- Tick Logging Server Configuration ---
const int tickLogServerPort = 88;             // Port for tick log receiver (same IP as Cloudwatcher)

// --- Motor 1 Tick Tracking (ISR variables must be volatile) ---
volatile word m1_tick_counter = 0;            // Current tick count during run
volatile bool m1_full_run_active = false;     // True if started at a limit switch
volatile bool m1_was_closing = false;         // True if direction is open→close (physically)
volatile byte m1_prev_dir = 0;                // Previous motor direction for transition detection

// --- Motor 2 Tick Tracking ---
volatile word m2_tick_counter = 0;
volatile bool m2_full_run_active = false;
volatile bool m2_was_closing = false;
volatile byte m2_prev_dir = 0;

// --- Last Valid Measurements (available for display and push) ---
volatile word m1_last_ticks_closing = 0;      // Last valid open→close ticks (M1)
volatile word m1_last_ticks_opening = 0;      // Last valid close→open ticks (M1)
volatile word m2_last_ticks_closing = 0;      // Last valid open→close ticks (M2)
volatile word m2_last_ticks_opening = 0;      // Last valid close→open ticks (M2)

// --- Data Ready Flags (signal main loop to push data) ---
volatile bool m1_data_ready = false;          // True when new valid M1 data available
volatile bool m2_data_ready = false;          // True when new valid M2 data available
volatile byte m1_last_direction = 0;          // 1=closing, 2=opening (for push)
volatile byte m2_last_direction = 0;

// --- Interrupted Stop Tracking (stops before reaching target limit) ---
volatile bool m1_interrupt_ready = false;     // True when M1 stopped before target
volatile bool m2_interrupt_ready = false;     // True when M2 stopped before target
volatile word m1_interrupt_ticks = 0;         // Tick count at interruption
volatile word m2_interrupt_ticks = 0;
volatile byte m1_interrupt_direction = 0;     // 1=was closing, 2=was opening
volatile byte m2_interrupt_direction = 0;

//=============================================================================
// FUNCTION PROTOTYPES
//=============================================================================
void setupEthernet();                           // Initialize Ethernet with retry
void networkWatchdog();                         // Monitor and recover network
void socketHygiene();                           // Drop idle clients, keep two listeners (P1)
void handleWebClient();                         // Process HTTP requests
void sendFullHtmlResponse(Print& client);          // Generate web UI HTML (P1: through PageWriter)
void checkRemoteConnectionAndAutoClose();       // IP monitoring logic
void initializeEEPROM();                        // Load/init persistent storage
void saveCountersToEEPROM();                    // Save counters if changed
void incrementDayCounter();                     // Track uptime days
void pushTickDataIfReady();                     // Push tick data to logging server
void pushInterruptDataIfReady();                // Push interrupted stop data

// --- v4.0: Sensor and safety functions ---
void setupDS18B20();                            // Initialize temperature probe
void readTemperatureAsync();                    // Non-blocking temperature reading
word computeDynamicTimeout(word base, word slope100, word margin, int temp_x10);
void updateDynamicTimeouts();                   // Recompute all 4 timeout values
void setupVL53L0X();                            // Initialize ToF sensor
void readToFDistance();                         // Read ToF distance (non-blocking)
void calibrateToF();                            // Store current ToF as baseline
void loadToFCalibration();                      // Load baseline from EEPROM
void frozenDomeStateMachine();                  // Main frozen dome detection logic
void checkConflictingSignals();                 // Limit switch vs ToF disagreement
void sendEventNotification(const char* type, const char* detail, bool urgent = false);  // Push event to Solo (P1: urgent = no rate limit)

//=============================================================================
// ISR-SAFE MOTOR START HELPERS
//=============================================================================
// Timer must be set BEFORE direction inside a critical section.
// Without this, the ISR could fire between the two writes and see
// mot*dir=running with mot*timer=0, causing an immediate false timeout.
// WARNING: Do NOT call these from inside the ISR — sei() would enable
// nested interrupts on AVR, which is dangerous.

// FIX (P2): a direction change from loop() (web command, IP auto-close while opening) never
// passes through a tick with mot*dir==0, so the ISR never zeroed mot*speed and the motor
// was reversed at full PWM. Restart the soft-start ramp whenever the direction changes.
inline void startMotor1(byte direction, word timeout) {
  cli();
  if (mot1dir != direction) mot1speed = 0;
  mot1timer = timeout;
  mot1dir = direction;
  sei();
}

inline void startMotor2(byte direction, word timeout) {
  cli();
  if (mot2dir != direction) mot2speed = 0;
  mot2timer = timeout;
  mot2dir = direction;
  sei();
}

// FIX (P1): motor start used by the frozen-dome cycle: only if the motor is still idle and no
// external command for it arrived meanwhile - checked atomically against the ISR, so a STOP
// button pressed in the same instant can never be overridden by the cycle's own start.
static bool fdStartMotor(byte motor, byte direction, word timeout) {
  bool started = false;
  const byte bit = (motor == 2) ? FD_CMD_M2 : FD_CMD_M1;
  uint8_t sreg = SREG; cli();
  if (motor == 1) {
    if (mot1dir == 0 && !(fdExternalCommand & bit)) { mot1timer = timeout; mot1dir = direction; started = true; }
  } else {
    if (mot2dir == 0 && !(fdExternalCommand & bit)) { mot2timer = timeout; mot2dir = direction; started = true; }
  }
  SREG = sreg;
  return started;
}

// FIX (P1): loop-side "|=" on the mask is a load/or/store sequence that an ISR write could
// slip into; only the ISR may use a plain |= (interrupts are off there).
static inline void fdNoteCommand(byte bits) {
  uint8_t sreg = SREG; cli();
  fdExternalCommand |= bits;
  SREG = sreg;
}

// FIX (P1): after the third frozen verdict an OPEN is refused at once (the state machine
// would only stop it on its next call, i.e. after up to a few seconds of pulling).
static inline bool fdOpenBlocked() {
  return frozenDomeState == FD_LOCKOUT ||
         (frozenRetryCount >= FROZEN_MAX_RETRIES && frozenDomeState != FD_IDLE);
}

// FIX (P1): stop the monitored motor if it is opening, on the live value, atomically
static void fdLockoutStop() {
  uint8_t sreg = SREG; cli();
  if (frozenMotorNum == 1) { if (mot1dir == CLOSE) { mot1dir = 0; stop1reason = 6; } }
  else                     { if (mot2dir == CLOSE) { mot2dir = 0; stop2reason = 6; } }
  SREG = sreg;
}

//=============================================================================
// EEPROM FUNCTIONS - Persistent Storage Management
//=============================================================================

/**
 * Initialize or load EEPROM data.
 * On first run (magic byte missing), initializes all counters to zero.
 * On subsequent boots, loads existing counter values.
 */
void initializeEEPROM() {
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC_BYTE) {
    // First run - write initial values
    #if defined(SERIAL_DEBUG_EEPROM)
    Serial.println(F("EEPROM: First run - initializing"));
    #endif
    EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_BYTE);
    EEPROM.put(EEPROM_ADDR_TOTAL_IP_FAILS, (unsigned int)0);
    EEPROM.put(EEPROM_ADDR_AUTO_CLOSES, (unsigned int)0);
    EEPROM.write(EEPROM_ADDR_LAST_FAIL_DAY, 0);
  } else {
    // Load saved values from previous session
    EEPROM.get(EEPROM_ADDR_TOTAL_IP_FAILS, totalIpFailures);
    EEPROM.get(EEPROM_ADDR_AUTO_CLOSES, totalAutoCloses);
    dayCounter = EEPROM.read(EEPROM_ADDR_LAST_FAIL_DAY);
    #if defined(SERIAL_DEBUG_EEPROM)
    Serial.print(F("EEPROM: Loaded - IP Fails: ")); Serial.print(totalIpFailures);
    Serial.print(F(", Auto-Closes: ")); Serial.print(totalAutoCloses);
    Serial.print(F(", Day: ")); Serial.println(dayCounter);
    #endif
  }
}

/**
 * Save counters to EEPROM if they've changed.
 * Uses dirty flag and 5-minute interval to minimize EEPROM wear.
 * EEPROM has ~100,000 write cycles per cell.
 */
void saveCountersToEEPROM() {
  static unsigned long lastSave = 0;

  if (eepromDirty && (millis() - lastSave > 300000UL)) {  // 5 minutes
    EEPROM.put(EEPROM_ADDR_TOTAL_IP_FAILS, totalIpFailures);
    EEPROM.put(EEPROM_ADDR_AUTO_CLOSES, totalAutoCloses);
    EEPROM.update(EEPROM_ADDR_LAST_FAIL_DAY, dayCounter);   // P2: only write if changed
    lastSave = millis();
    eepromDirty = false;
    #if defined(SERIAL_DEBUG_EEPROM)
    Serial.println(F("EEPROM: Counters saved"));
    #endif
  }
}

/**
 * Increment day counter every 24 hours of continuous operation.
 * Note: This tracks continuous uptime only. Reboots reset the 24h timer,
 * so frequent reboots will cause days to be undercounted.
 */
void incrementDayCounter() {
  static unsigned long lastDayIncrement = 0;

  if (millis() - lastDayIncrement > 86400000UL) {  // 24 hours in milliseconds
    dayCounter++;
    if (dayCounter > 250) dayCounter = 0;  // Wrap before overflow
    lastDayIncrement = millis();
    eepromDirty = true;
    EEPROM.update(EEPROM_ADDR_LAST_FAIL_DAY, dayCounter);  // Immediate save (P2: update, not write)
  }
}

//=============================================================================
// TEMPERATURE PRINT HELPER (P2)
//=============================================================================
// FIX (P2): "-5" / 10 == 0 on the integer side, so -0.1..-0.9 C printed as 0.1..0.9 C in the
// CSV push, the events and the web page (exactly the band that matters for ice).
static void printTempX10(Print& p, int t) {
  if (t < 0) { p.print('-'); t = -t; }
  p.print(t / 10);
  p.print('.');
  p.print(t % 10);
}

//=============================================================================
// TICK DATA PUSH - Send motor runtime to logging server
//=============================================================================
/**
 * Pushes tick data to external logging server when:
 * - A valid full-run measurement is available (data_ready flag set)
 * - Tick logging is enabled via web UI toggle
 * - Network is available
 *
 * Uses minimal HTTP GET request to port 88 on Cloudwatcher IP.
 * Server is expected to add timestamp and temperature, then log to CSV.
 *
 * GET /log?m=<motor>&d=<direction>&t=<ticks>
 *   m = Motor number (1 or 2)
 *   d = Direction (1=closing/open→close, 2=opening/close→open)
 *   t = Tick count
 */
void pushTickDataIfReady() {
  // Skip if logging disabled or network not ready
  if (!tickLoggingEnabled || !ethernet_initialized || !networkMonitoringEnabled) {
    return;
  }

  // Check Motor 1 data
  if (m1_data_ready) {
    word ticks = (m1_last_direction == 1) ? m1_last_ticks_closing : m1_last_ticks_opening;

    wdt_reset();  // Reset watchdog before potentially blocking call
    EthernetClient logClient;
    logClient.setConnectionTimeout(1500);  // 1.5 s bound for connect()/stop() (P2: setTimeout() only affected Stream reads; effective was 1 s)

    if (logClient.connect(remoteStationIp, tickLogServerPort)) {
      // Build HTTP GET request with temp and ToF data
      logClient.print(F("GET /log?m=1&d="));
      logClient.print(m1_last_direction);
      logClient.print(F("&t="));
      logClient.print(ticks);
      // Include temperature
      logClient.print(F("&temp="));
      if (currentTemp_x10 != -9990) {
        printTempX10(logClient, currentTemp_x10);
      } else {
        logClient.print(F("-999"));
      }
      // Include ToF distance in cm
      logClient.print(F("&tof="));
      if (tofDistance_mm > 0) {
        logClient.print(tofDistance_mm / 10);
        logClient.print(F("."));
        logClient.print(tofDistance_mm % 10);
      } else {
        logClient.print(F("-1"));
      }
      logClient.println(F(" HTTP/1.0"));
      logClient.print(F("Host: "));
      logClient.println(remoteStationIp);
      logClient.println(F("Connection: close"));
      logClient.println();

      // Don't wait for response - just disconnect
      delay(50);  // Brief delay for data to be sent
      logClient.stop();
    }
    wdt_reset();

    m1_data_ready = false;  // Clear flag regardless of success
  }

  // Check Motor 2 data
  if (m2_data_ready) {
    word ticks = (m2_last_direction == 1) ? m2_last_ticks_closing : m2_last_ticks_opening;

    wdt_reset();
    EthernetClient logClient;
    logClient.setConnectionTimeout(1500);

    if (logClient.connect(remoteStationIp, tickLogServerPort)) {
      logClient.print(F("GET /log?m=2&d="));
      logClient.print(m2_last_direction);
      logClient.print(F("&t="));
      logClient.print(ticks);
      logClient.print(F("&temp="));
      if (currentTemp_x10 != -9990) {
        printTempX10(logClient, currentTemp_x10);
      } else {
        logClient.print(F("-999"));
      }
      logClient.print(F("&tof="));
      if (tofDistance_mm > 0) {
        logClient.print(tofDistance_mm / 10);
        logClient.print(F("."));
        logClient.print(tofDistance_mm % 10);
      } else {
        logClient.print(F("-1"));
      }
      logClient.println(F(" HTTP/1.0"));
      logClient.print(F("Host: "));
      logClient.println(remoteStationIp);
      logClient.println(F("Connection: close"));
      logClient.println();

      delay(50);
      logClient.stop();
    }
    wdt_reset();

    m2_data_ready = false;
  }
}

//=============================================================================
// INTERRUPT DATA PUSH - Send interrupted stop data to logging server
//=============================================================================
/**
 * Pushes interrupted stop data when motor stopped before reaching target.
 * This captures: manual stops, web stops, emergency stops, timeouts.
 *
 * GET /interrupt?m=<motor>&d=<direction>&t=<ticks>
 *   m = Motor number (1 or 2)
 *   d = Direction (1=was closing, 2=was opening)
 *   t = Tick count at interruption
 */
void pushInterruptDataIfReady() {
  // Skip if logging disabled or network not ready
  if (!tickLoggingEnabled || !ethernet_initialized || !networkMonitoringEnabled) {
    // Still clear flags to prevent buildup
    m1_interrupt_ready = false;
    m2_interrupt_ready = false;
    return;
  }

  // Check Motor 1 interrupt
  if (m1_interrupt_ready) {
    wdt_reset();
    EthernetClient logClient;
    logClient.setConnectionTimeout(1500);

    if (logClient.connect(remoteStationIp, tickLogServerPort)) {
      logClient.print(F("GET /interrupt?m=1&d="));
      logClient.print(m1_interrupt_direction);
      logClient.print(F("&t="));
      logClient.print(m1_interrupt_ticks);
      logClient.print(F("&temp="));
      if (currentTemp_x10 != -9990) {
        printTempX10(logClient, currentTemp_x10);
      } else {
        logClient.print(F("-999"));
      }
      logClient.print(F("&tof="));
      if (tofDistance_mm > 0) {
        logClient.print(tofDistance_mm / 10);
        logClient.print(F("."));
        logClient.print(tofDistance_mm % 10);
      } else {
        logClient.print(F("-1"));
      }
      logClient.println(F(" HTTP/1.0"));
      logClient.print(F("Host: "));
      logClient.println(remoteStationIp);
      logClient.println(F("Connection: close"));
      logClient.println();

      delay(50);
      logClient.stop();
    }
    wdt_reset();

    m1_interrupt_ready = false;
  }

  // Check Motor 2 interrupt
  if (m2_interrupt_ready) {
    wdt_reset();
    EthernetClient logClient;
    logClient.setConnectionTimeout(1500);

    if (logClient.connect(remoteStationIp, tickLogServerPort)) {
      logClient.print(F("GET /interrupt?m=2&d="));
      logClient.print(m2_interrupt_direction);
      logClient.print(F("&t="));
      logClient.print(m2_interrupt_ticks);
      logClient.print(F("&temp="));
      if (currentTemp_x10 != -9990) {
        printTempX10(logClient, currentTemp_x10);
      } else {
        logClient.print(F("-999"));
      }
      logClient.print(F("&tof="));
      if (tofDistance_mm > 0) {
        logClient.print(tofDistance_mm / 10);
        logClient.print(F("."));
        logClient.print(tofDistance_mm % 10);
      } else {
        logClient.print(F("-1"));
      }
      logClient.println(F(" HTTP/1.0"));
      logClient.print(F("Host: "));
      logClient.println(remoteStationIp);
      logClient.println(F("Connection: close"));
      logClient.println();

      delay(50);
      logClient.stop();
    }
    wdt_reset();

    m2_interrupt_ready = false;
  }
}

//=============================================================================
// DS18B20 TEMPERATURE SENSOR FUNCTIONS
//=============================================================================

/**
 * Initialize DS18B20 temperature probe on pin 22.
 * Called early in setup() BEFORE Ethernet init so temperature is available
 * for dynamic timeout computation when first motor command arrives.
 * Uses async (non-blocking) conversion mode.
 */
void setupDS18B20() {
  delay(100);  // Allow DS18B20 to finish its own power-on boot before scanning bus
  ds18b20.begin();

  if (ds18b20.getDeviceCount() > 0) {
    ds18b20_connected = true;
    ds18b20.setResolution(12);            // 12-bit = 0.0625C precision, 750ms conversion
    ds18b20.setWaitForConversion(false);  // Non-blocking async conversion

    // Do one synchronous read at startup to have temp ready immediately
    ds18b20.requestTemperatures();
    delay(800);  // Wait for 12-bit conversion (750ms + margin)
    float tempC = ds18b20.getTempCByIndex(0);

    if (tempC != DEVICE_DISCONNECTED_C && tempC > -40.0f && tempC < 80.0f) {
      currentTemp_x10 = (int)(tempC * 10.0f);
      tempFailCount = 0;
      updateDynamicTimeouts();
    } else {
      currentTemp_x10 = -9990;  // Sentinel: -999.0C = invalid
      ds18b20_connected = false;
    }
  } else {
    ds18b20_connected = false;
    currentTemp_x10 = -9990;
  }
}

/**
 * Non-blocking temperature reading loop.
 * Call from loop() — requests conversion every 5 seconds,
 * reads result 800ms later.
 * If sensor was lost, attempts re-detection every 30 seconds.
 */
void readTemperatureAsync() {
  unsigned long now = millis();

  // Step 0: Periodic re-detection if sensor is confirmed dead
  // Re-scans OneWire bus every 30 seconds to detect hot-plug reconnection
  static unsigned long lastRedetectAttempt = 0;
  if (!ds18b20_connected && tempFailCount >= TEMP_FAIL_THRESHOLD) {
    if (now - lastRedetectAttempt >= 30000UL) {
      lastRedetectAttempt = now;
      ds18b20.begin();  // Re-scan OneWire bus
      if (ds18b20.getDeviceCount() > 0) {
        // Sensor found again — reinitialize
        ds18b20_connected = true;
        sendEventNotification("sensor_ok", "DS18B20 back - dynamic timeout");   // P2
        ds18b20.setResolution(12);
        ds18b20.setWaitForConversion(false);
        tempFailCount = 0;
        lastTempRead = 0;  // Force immediate read
      }
    }
    return;  // Skip normal read cycle while disconnected
  }

  // Step 1: Request new conversion if interval elapsed
  if (!tempConversionPending && (now - lastTempRead >= TEMP_READ_INTERVAL)) {
    if (ds18b20_connected || tempFailCount < TEMP_FAIL_THRESHOLD) {
      ds18b20.requestTemperatures();
      tempConversionPending = true;
      lastTempRequest = now;
    }
  }

  // Step 2: Read result after conversion time
  if (tempConversionPending && (now - lastTempRequest >= TEMP_CONV_TIME)) {
    tempConversionPending = false;
    float tempC = ds18b20.getTempCByIndex(0);

    if (tempC != DEVICE_DISCONNECTED_C && tempC > -40.0f && tempC < 80.0f) {
      int newTemp_x10 = (int)(tempC * 10.0f);

      // Only recompute timeouts if temperature actually changed
      if (newTemp_x10 != currentTemp_x10) {
        currentTemp_x10 = newTemp_x10;
        updateDynamicTimeouts();
      }

      ds18b20_connected = true;
      tempFailCount = 0;
      lastTempRead = now;
    } else {
      // Read failed
      tempFailCount++;
      if (tempFailCount >= TEMP_FAIL_THRESHOLD) {
        // Too many failures — fall back to static timeout
        if (ds18b20_connected) sendEventNotification("sensor_fail", "DS18B20 lost - static timeout");   // P2
        ds18b20_connected = false;
        dynamicTimeoutActive = false;
        setDynamicTimeouts(MAX_MOT1_OPEN, MAX_MOT1_CLOSE, MAX_MOT2_OPEN, MAX_MOT2_CLOSE);
      }
    }
  }
}

/**
 * Compute dynamic timeout using integer-only linear regression.
 * Formula: timeout = base - (slope100 * temp_C) / 100 + margin
 *
 * @param base     Base ticks at 0C (intercept)
 * @param slope100 Slope * 100 (ticks decrease per C, times 100 for integer math)
 * @param margin   Safety margin in ticks
 * @param temp_x10 Temperature in tenths of C (e.g., 215 = 21.5C)
 * @return         Clamped timeout value in ticks [DYN_TIMEOUT_MIN, DYN_TIMEOUT_MAX]
 */
word computeDynamicTimeout(word base, word slope100, word margin, int temp_x10) {
  // timeout = base - (slope100 * temp_x10) / 1000 + margin
  // Using long to prevent overflow: slope100 (max ~2000) * temp_x10 (max ~800) = ~1.6M fits in long
  long result = (long)base - ((long)slope100 * (long)temp_x10) / 1000L + (long)margin;

  // Clamp to valid range
  if (result < DYN_TIMEOUT_MIN) result = DYN_TIMEOUT_MIN;
  if (result > DYN_TIMEOUT_MAX) result = DYN_TIMEOUT_MAX;

  return (word)result;
}

/**
 * Recompute all 4 dynamic timeout values from current temperature.
 * Called whenever temperature changes. Written under cli/sei because the ISR
 * reads them and 16-bit accesses are not atomic on AVR.
 */
void updateDynamicTimeouts() {
  if (currentTemp_x10 == -9990) {
    // Invalid temperature — use static fallback
    dynamicTimeoutActive = false;
    setDynamicTimeouts(MAX_MOT1_OPEN, MAX_MOT1_CLOSE, MAX_MOT2_OPEN, MAX_MOT2_CLOSE);
    return;
  }

  // FIX (P2): the regression was fitted on -1.6..22.6 C; clamp the input UPWARD so a single
  // implausibly hot sample (probe in the sun, corrupted scratchpad) cannot shorten a run to
  // a false TIMEOUT mid-travel. Cold values only lengthen the timeout, which DYN_TIMEOUT_MAX
  // caps anyway, so no lower clamp (it would eat the margin exactly in the icing band).
  int t = currentTemp_x10;
  if (t > 300) t = 300;
  setDynamicTimeouts(
    computeDynamicTimeout(DYN_M1_CLOSE_BASE, DYN_M1_CLOSE_SLOPE, DYN_M1_CLOSE_MARGIN, t),
    computeDynamicTimeout(DYN_M1_OPEN_BASE, DYN_M1_OPEN_SLOPE, DYN_M1_OPEN_MARGIN, t),
    computeDynamicTimeout(DYN_M2_CLOSE_BASE, DYN_M2_CLOSE_SLOPE, DYN_M2_CLOSE_MARGIN, t),
    computeDynamicTimeout(DYN_M2_OPEN_BASE, DYN_M2_OPEN_SLOPE, DYN_M2_OPEN_MARGIN, t));
  dynamicTimeoutActive = true;
}

//=============================================================================
// VL53L0X TIME-OF-FLIGHT SENSOR FUNCTIONS
//=============================================================================

/**
 * Initialize VL53L0X ToF sensor on I2C (pins 20/21).
 * Sets continuous reading mode for fast non-blocking reads.
 */
// FIX (P1): I2C bus recovery. A slave interrupted mid-transfer (EMI from the motors, sensor
// brown-out, cable glitch) can hold SDA low forever. Clock out up to 9 SCL pulses and a
// STOP so it releases the bus. Open-drain emulation: LOW = PORT bit cleared first, then the
// pin becomes an output (never a push-pull HIGH); HIGH = release to the breakout's pull-ups.
// Wire.end() first so the TWI unit lets go of the pins.
static void i2cBusRecover() {
  const byte SDA_PIN = 20, SCL_PIN = 21;   // MEGA hardware I2C pins
  Wire.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(10);
  if (digitalRead(SDA_PIN) == HIGH) return;   // bus idle, nothing to do
  for (byte i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    digitalWrite(SCL_PIN, LOW); pinMode(SCL_PIN, OUTPUT); delayMicroseconds(5);   // drive SCL low
    pinMode(SCL_PIN, INPUT_PULLUP);                        delayMicroseconds(5);   // release SCL
  }
  digitalWrite(SDA_PIN, LOW); pinMode(SDA_PIN, OUTPUT); delayMicroseconds(5);   // STOP condition:
  pinMode(SCL_PIN, INPUT_PULLUP);                        delayMicroseconds(5);   // SDA low->high
  pinMode(SDA_PIN, INPUT_PULLUP);                        delayMicroseconds(5);   // while SCL high
}

// FIX (P1): (re)start the I2C master WITH a timeout. The AVR Wire library has none by default:
// a stuck bus blocks Wire.endTransmission()/requestFrom() forever - also inside the TWI
// interrupt with all interrupts off, i.e. no limit switches, no timeout, no STOP button -
// then loop() or setup() hangs, the watchdog reboots, setup() hangs again: permanent freeze.
// 25 ms per transfer, TWI hardware reset on timeout. Requires Arduino AVR core >= 1.8.3.
// Do not raise the value: VL53L0X::init() issues ~180 transfers, so a bus that answers the
// ID read and then stalls costs up to ~5 s between the wdt_reset() calls around init().
static void i2cBegin() {
  i2cBusRecover();
  Wire.begin();
  Wire.setWireTimeout(25000, true);
}

void setupVL53L0X() {
  i2cBegin();
  tofSensor.setTimeout(500);

  wdt_reset();   // FIX (P1): init() on a half-dead bus can take ~5 s (25 ms per timed-out transfer)
  bool tofInitOk = tofSensor.init();
  wdt_reset();   // also on the failure path
  if (tofInitOk) {
    tof_connected = true;
    tofSensor.setMeasurementTimingBudget(200000);  // 200ms for accuracy
    tofSensor.startContinuous();
    wdt_reset();   // FIX (P1): ~30 more I2C transfers, each bounded by the 25 ms Wire timeout
    tofFailCount = 0;

    // Take initial reading
    int reading = tofSensor.readRangeContinuousMillimeters();
    if (!tofSensor.timeoutOccurred() && reading > 0 && reading < 8000) {
      tofDistance_mm = reading;
    }
  } else {
    tof_connected = false;
    tofDistance_mm = -1;
  }
}

/**
 * Read ToF distance in non-blocking continuous mode.
 * Call from loop() — reads at TOF_READ_INTERVAL pace.
 * If sensor was lost, attempts re-initialization every 30 seconds.
 */
void readToFDistance() {
  // Periodic re-detection if sensor is confirmed dead
  if (!tof_connected && tofFailCount >= TEMP_FAIL_THRESHOLD) {
    static unsigned long lastTofRedetect = 0;
    unsigned long now = millis();
    if (now - lastTofRedetect >= 30000UL) {
      lastTofRedetect = now;
      // Attempt full re-init (I2C requires init() call after reconnect)
      i2cBegin();    // FIX (P1): recover a stuck bus before talking to the sensor again
      wdt_reset();   // FIX (P1): init() on a half-dead bus can take ~5 s
      bool tofInitOk = tofSensor.init();
      wdt_reset();   // also on the failure path
      if (tofInitOk) {
        tof_connected = true;
        sendEventNotification("sensor_ok", "VL53L0X back - frozen detection on");   // P2
        tofSensor.setMeasurementTimingBudget(200000);
        tofSensor.startContinuous();
        wdt_reset();   // FIX (P1): see setupVL53L0X()
        tofFailCount = 0;
      }
    }
    return;
  }

  unsigned long now = millis();
  if (now - lastTofRead < TOF_READ_INTERVAL) return;
  lastTofRead = now;

  int reading = tofSensor.readRangeContinuousMillimeters();

  if (!tofSensor.timeoutOccurred() && reading > 0 && reading < 8000) {
    tofDistance_mm = reading;
    tof_connected = true;
    tofFailCount = 0;
  } else {
    tofFailCount++;
    if (tofFailCount >= TEMP_FAIL_THRESHOLD) {
      if (tof_connected) sendEventNotification("sensor_fail", "VL53L0X lost - frozen detection off");   // P2
      tof_connected = false;
      tofDistance_mm = -1;
      // Frozen dome detection auto-disables when sensor disconnected
      if (frozenDomeState != FD_IDLE && frozenDomeState != FD_LOCKOUT) {
        frozenDomeState = FD_IDLE;
        frozenCheckActive = false;
      }
    }
  }
}

/**
 * Calibrate ToF baseline: store current reading as "closed" reference.
 * Dome MUST be fully closed when this is called.
 * Stores baseline in EEPROM for persistence across reboots.
 */
void calibrateToF() {
  if (!tof_connected || tofDistance_mm <= 0) return;

  tofBaseline_mm = (word)tofDistance_mm;
  tofCalibrated = true;

  // Store to EEPROM
  EEPROM.put(EEPROM_ADDR_TOF_BASELINE, tofBaseline_mm);
  EEPROM.write(EEPROM_ADDR_TOF_CALIB_MAGIC, TOF_CALIB_MAGIC_BYTE);
}

/**
 * Load ToF calibration from EEPROM (called during setup).
 */
void loadToFCalibration() {
  if (EEPROM.read(EEPROM_ADDR_TOF_CALIB_MAGIC) == TOF_CALIB_MAGIC_BYTE) {
    EEPROM.get(EEPROM_ADDR_TOF_BASELINE, tofBaseline_mm);
    if (tofBaseline_mm > 0 && tofBaseline_mm < 8000) {
      tofCalibrated = true;
    } else {
      tofCalibrated = false;
      tofBaseline_mm = 0;
    }
  } else {
    tofCalibrated = false;
    tofBaseline_mm = 0;
  }
}

//=============================================================================
// FROZEN DOME STATE MACHINE
//=============================================================================
/**
 * Detects when dome halves are frozen together at the top.
 * Uses ToF distance to verify that the gap between dome halves
 * is actually increasing after an open command.
 *
 * State machine runs in loop() (not ISR). Only the tick counter
 * (frozenCheckTicks) is incremented in the ISR for precise timing.
 *
 * Attempt cycle (up to 3 attempts):
 *   1. IDLE -> motor starts -> MONITORING
 *   2. MONITORING -> ~4 sec of ticks -> FIRST_CHECK
 *   3. FIRST_CHECK -> ToF check:
 *      - Opening detected? -> SUCCESS, back to IDLE
 *      - Not opening? -> STOP motor -> GRAVITY_WAIT
 *   4. GRAVITY_WAIT -> 20 seconds passive -> SECOND_CHECK
 *   5. SECOND_CHECK -> ToF check:
 *      - Gravity separated? -> Resume opening, IDLE
 *      - Still stuck? -> Confirmed frozen -> CLOSING
 *   6. CLOSING -> reverse to close -> RETRY_WAIT
 *   7. RETRY_WAIT -> 5 seconds -> retry or LOCKOUT
 */
void frozenDomeStateMachine() {
  // Own direction tracking — independent of ISR's m1/m2_prev_dir to avoid race condition.
  // The ISR updates m1_prev_dir at 61 Hz, always before loop() runs this function,
  // so we'd never see the transition if we used the ISR's variables.
  static byte fd_prev_mot1dir = 0;
  static byte fd_prev_mot2dir = 0;

  // Skip if ToF not calibrated or not connected — frozen detection disabled
  if (!tofCalibrated || !tof_connected) {
    if (frozenDomeState != FD_IDLE && frozenDomeState != FD_LOCKOUT) {
      frozenDomeState = FD_IDLE;
      frozenCheckActive = false;
    }
    fd_prev_mot1dir = mot1dir;
    fd_prev_mot2dir = mot2dir;
    return;
  }

  unsigned long now = millis();
  const byte myBit = (frozenMotorNum == 2) ? FD_CMD_M2 : FD_CMD_M1;

  // FIX (P1): consume the external-command mask and snapshot the motor direction in ONE
  // critical section (the ISR sets a bit and starts the motor in the same tick)
  byte cmd, curDir;
  { uint8_t sreg = SREG; cli();
    cmd = fdExternalCommand; fdExternalCommand = 0;
    curDir = (frozenMotorNum == 2) ? mot2dir : mot1dir;
    SREG = sreg; }

  // FIX (P1): a command addressed to the shutter we are working on ends the automatic cycle
  // in the waiting/reversing states: the operator or another safety function is in control
  // now, and continuing our own stop/close/retry sequence could re-open a dome that was just
  // stopped or closed on purpose. (In MONITORING/FIRST_CHECK the direction check below
  // handles stop and reversal, and a repeated OPEN must not disarm the check.)
  // After the third frozen verdict the dome is locked out instead, so repeated OPEN
  // commands cannot keep pulling on the ice.
  if ((cmd & myBit) &&
      (frozenDomeState == FD_GRAVITY_WAIT || frozenDomeState == FD_SECOND_CHECK ||
       frozenDomeState == FD_CLOSING || frozenDomeState == FD_RETRY_WAIT)) {
    frozenCheckActive = false;
    if (frozenRetryCount >= FROZEN_MAX_RETRIES) {
      fdLockoutStop();   // somebody re-issued OPEN: stop it, the lockout applies
      frozenDomeState = FD_LOCKOUT;
      sendEventNotification("frozen_lockout", "3 attempts failed - dome locked", true);
    } else {
      frozenDomeState = FD_IDLE;
    }
  }

  switch (frozenDomeState) {

    case FD_IDLE:
      // FIX (P1): the attempt counter ages out 30 min after the last frozen verdict, so an
      // opening hours later gets its three attempts again (a cancelled cycle keeps the count).
      if (frozenRetryCount > 0 && now - lastFrozenDetect > FROZEN_ATTEMPT_MEMORY) frozenRetryCount = 0;
      // Watch for a motor ENTERING the opening direction (physically opening = mot*dir==CLOSE).
      // FIX (P1): "!= CLOSE" instead of "== 0", so an open that reverses a running close
      // (web $2 while closing) is monitored as well.
      if (mot1dir == CLOSE && fd_prev_mot1dir != CLOSE) {
        frozenDomeState = FD_MONITORING;
        frozenCheckTicks = 0;
        frozenCheckActive = true;
        frozenMotorNum = 1;
      } else if (mot2dir == CLOSE && fd_prev_mot2dir != CLOSE) {
        frozenDomeState = FD_MONITORING;
        frozenCheckTicks = 0;
        frozenCheckActive = true;
        frozenMotorNum = 2;
      }
      break;

    case FD_MONITORING:
      // Abort if the motor is no longer OPENING (manual stop, limit switch, reversal).
      // FIX (P1): checked only ==0 before, so a close command within the first 4 s of an
      // open was judged "frozen" and the dome re-opened itself after the retry wait.
      if (curDir != CLOSE) {
        frozenDomeState = FD_IDLE;
        frozenCheckActive = false;
        break;
      }
      // Wait for ~4 seconds of motor running (ISR counts ticks)
      if (frozenCheckTicks >= TOF_CHECK_TICKS) {
        frozenDomeState = FD_FIRST_CHECK;
        frozenCheckActive = false;
      }
      break;

    case FD_FIRST_CHECK:
      if (curDir != CLOSE) { frozenDomeState = FD_IDLE; break; }   // stopped since the last call
      // Check if dome gap is increasing (opening detected)
      if (tofDistance_mm > (int)(tofBaseline_mm + TOF_OPEN_TOLERANCE)) {
        // Opening detected — dome is NOT frozen, all good
        frozenDomeState = FD_IDLE;
        frozenRetryCount = 0;
      } else {
        // Dome appears frozen — STOP motor immediately
        if (frozenMotorNum == 1) {
          mot1dir = 0; stop1reason = 6;  // 6 = FROZEN DOME
        } else {
          mot2dir = 0; stop2reason = 6;
        }
        // FIX (P1): count the attempt where it is detected, not only after the retry wait
        frozenRetryCount++;
        lastFrozenDetect = now;

        // Enter gravity wait phase
        frozenDomeState = FD_GRAVITY_WAIT;
        frozenStateTimer = now;

        // Send event notification
        char detail[32];
        snprintf(detail, sizeof(detail), "S%d attempt %d/%d", frozenMotorNum, frozenRetryCount, FROZEN_MAX_RETRIES);
        sendEventNotification("frozen_dome", detail);
      }
      break;

    case FD_GRAVITY_WAIT:
      if (curDir != 0) { frozenDomeState = FD_IDLE; break; }   // somebody started the motor
      // Motor is off — wait 20 seconds for gravity to separate frozen halves
      if (now - frozenStateTimer >= FROZEN_GRAVITY_WAIT) {
        frozenDomeState = FD_SECOND_CHECK;
      }
      break;

    case FD_SECOND_CHECK: {
      // Re-read ToF — did gravity separate the halves?
      // FIX (P1): motors are started atomically and only if still idle and not overridden
      bool ok = true;
      if (tofDistance_mm > (int)(tofBaseline_mm + TOF_OPEN_TOLERANCE)) {
        // Gravity worked! Resume opening
        if (frozenMotorNum == 1 && !digitalRead(lim1closed)) {
          ok = fdStartMotor(1, CLOSE, dynTimeout_M1_Open); if (ok) stop1reason = 0;
        } else if (frozenMotorNum == 2 && !digitalRead(lim2closed)) {
          ok = fdStartMotor(2, CLOSE, dynTimeout_M2_Open); if (ok) stop2reason = 0;
        }
        if (ok) {
          // FIX (P1): the resumed opening is monitored again (a re-freeze mid-travel was
          // not detected before); the counter restarts for this opening
          frozenRetryCount = 0;
          frozenDomeState = FD_MONITORING;
          frozenCheckTicks = 0;
          frozenCheckActive = true;
          sendEventNotification("frozen_clear", "Gravity separated halves");
        } else if (frozenRetryCount >= FROZEN_MAX_RETRIES) {
          frozenDomeState = FD_LOCKOUT;
          sendEventNotification("frozen_lockout", "3 attempts failed - dome locked", true);
        } else {
          frozenDomeState = FD_IDLE;
        }
      } else {
        // Still frozen — reverse motor to close
        if (frozenMotorNum == 1 && !digitalRead(lim1open)) {
          ok = fdStartMotor(1, OPEN, dynTimeout_M1_Close); if (ok) stop1reason = 6;
        } else if (frozenMotorNum == 2 && !digitalRead(lim2open)) {
          ok = fdStartMotor(2, OPEN, dynTimeout_M2_Close); if (ok) stop2reason = 6;
        }
        if (ok) frozenDomeState = FD_CLOSING;
        else if (frozenRetryCount >= FROZEN_MAX_RETRIES) {
          frozenDomeState = FD_LOCKOUT;
          sendEventNotification("frozen_lockout", "3 attempts failed - dome locked", true);
        } else frozenDomeState = FD_IDLE;
      }
      break;
    }

    case FD_CLOSING:
      // Wait for close to complete (motor reaches limit or stops)
      if (curDir == 0) {
        // FIX (P1): only a close that ran to the limit (stop reason 0/6) may retry; a stop by
        // button/SWSTOP (1), web (2), auto-close (3/4) or timeout (5) ends the cycle.
        byte r = (frozenMotorNum == 1) ? stop1reason : stop2reason;
        if (r != 0 && r != 6) {
          if (r == 5) sendEventNotification("frozen_dome", "auto-close TIMEOUT - cycle abandoned");
          frozenDomeState = FD_IDLE;
          break;
        }
        frozenDomeState = FD_RETRY_WAIT;
        frozenStateTimer = millis();
      }
      break;

    case FD_RETRY_WAIT:
      if (curDir != 0) { frozenDomeState = FD_IDLE; break; }   // somebody started the motor
      // Wait 5 seconds before next retry
      if (now - frozenStateTimer >= FROZEN_RETRY_WAIT) {
        if (frozenRetryCount >= FROZEN_MAX_RETRIES) {
          // All retries exhausted — LOCKOUT
          frozenDomeState = FD_LOCKOUT;
          sendEventNotification("frozen_lockout", "3 attempts failed - dome locked", true);   // urgent: bypasses the 10 s limiter
        } else {
          // Re-issue open command for next attempt
          bool ok = false;
          if (frozenMotorNum == 1 && !digitalRead(lim1closed)) {
            ok = fdStartMotor(1, CLOSE, dynTimeout_M1_Open); if (ok) stop1reason = 0;
          } else if (frozenMotorNum == 2 && !digitalRead(lim2closed)) {
            ok = fdStartMotor(2, CLOSE, dynTimeout_M2_Open); if (ok) stop2reason = 0;
          }
          if (ok) {
            frozenDomeState = FD_MONITORING;
            frozenCheckTicks = 0;
            frozenCheckActive = true;
          } else {
            frozenDomeState = FD_IDLE;   // somebody else took the motor (count < max here)
          }
        }
      }
      break;

    case FD_LOCKOUT:
      // All open commands are blocked until /?$U unlock
      // State persists until explicitly cleared
      break;
  }

  // Update own direction tracking at end of every call
  fd_prev_mot1dir = mot1dir;
  fd_prev_mot2dir = mot2dir;
}

//=============================================================================
// CONFLICTING SIGNAL DETECTION
//=============================================================================
/**
 * Periodic check: do limit switches and ToF sensor agree?
 * Only runs when motors are stopped and ToF is calibrated.
 * Hardware priority: limit switches ALWAYS win for motor control.
 * ToF disagreements are logged/notified but don't change motor behavior.
 */
void checkConflictingSignals() {
  static unsigned long lastConflictCheck = 0;
  unsigned long now = millis();

  // Check every 5 seconds, only when safe
  if (now - lastConflictCheck < 5000) return;
  lastConflictCheck = now;

  // Skip if preconditions not met
  if (!tofCalibrated || !tof_connected || tofDistance_mm <= 0) return;
  if (mot1dir != 0 || mot2dir != 0) return;  // Only check when motors stopped

  bool s1_closed = digitalRead(lim1open);    // lim1open = physically CLOSED
  bool s2_closed = digitalRead(lim2open);    // lim2open = physically CLOSED
  bool tof_says_open = (tofDistance_mm > (int)(tofBaseline_mm + TOF_OPEN_TOLERANCE));
  bool tof_says_closed = (tofDistance_mm <= (int)(tofBaseline_mm + TOF_OPEN_TOLERANCE));

  // Conflict: limits say closed but ToF says open
  if (s1_closed && s2_closed && tof_says_open) {
    // Rate limit conflict events to 1 per 5 minutes
    if (now - lastConflictEventTime >= CONFLICT_EVENT_INTERVAL) {
      lastConflictEventTime = now;
      char detail[48];
      snprintf(detail, sizeof(detail), "Limits=CLOSED ToF=%dmm baseline=%dmm", tofDistance_mm, tofBaseline_mm);
      sendEventNotification("conflict", detail);
    }
  }

  // Conflict: limits say open but ToF says closed (possible frozen top)
  bool s1_open = digitalRead(lim1closed);
  bool s2_open = digitalRead(lim2closed);
  if ((s1_open || s2_open) && tof_says_closed) {
    if (now - lastConflictEventTime >= CONFLICT_EVENT_INTERVAL) {
      lastConflictEventTime = now;
      char detail[48];
      snprintf(detail, sizeof(detail), "Limits=OPEN ToF=%dmm baseline=%dmm", tofDistance_mm, tofBaseline_mm);
      sendEventNotification("conflict", detail);
    }
  }
}

//=============================================================================
// EVENT NOTIFICATION — Push events to Solo Pi for Pushover delivery
//=============================================================================
/**
 * Send event notification to Solo Pi (port 88, /event endpoint).
 * Includes current temperature and ToF distance in every request.
 * Rate-limited to minimum 10 seconds between events.
 *
 * @param type   Event type string (e.g., "frozen_dome", "sensor_fail")
 * @param detail Additional detail string
 */
void sendEventNotification(const char* type, const char* detail, bool urgent) {
  // Rate limiting (FIX (P1): a lockout follows its verdict within seconds and must not be dropped)
  unsigned long now = millis();
  if (!urgent && now - lastEventSentTime < EVENT_MIN_INTERVAL) return;

  // Skip if network not ready
  if (!ethernet_initialized || !networkMonitoringEnabled) return;

  // URL-encode the detail string (replace spaces with %20) into a local buffer
  // so it can be sent in a single print() call for reliable TCP transmission
  char encoded[64];
  byte j = 0;
  for (byte i = 0; detail[i] && j < sizeof(encoded) - 4; i++) {
    if (detail[i] == ' ') {
      encoded[j++] = '%'; encoded[j++] = '2'; encoded[j++] = '0';
    } else {
      encoded[j++] = detail[i];
    }
  }
  encoded[j] = '\0';

  wdt_reset();
  EthernetClient eventClient;
  eventClient.setConnectionTimeout(1500);

  if (eventClient.connect(remoteStationIp, tickLogServerPort)) {
    eventClient.print(F("GET /event?type="));
    eventClient.print(type);
    eventClient.print(F("&detail="));
    eventClient.print(encoded);
    eventClient.print(F("&temp="));
    if (currentTemp_x10 != -9990) {
      printTempX10(eventClient, currentTemp_x10);
    } else {
      eventClient.print(F("-999"));
    }
    eventClient.print(F("&tof="));
    if (tofDistance_mm > 0) {
      // Display in cm with one decimal
      eventClient.print(tofDistance_mm / 10);
      eventClient.print(F("."));
      eventClient.print(tofDistance_mm % 10);
    } else {
      eventClient.print(F("-1"));
    }
    eventClient.println(F(" HTTP/1.0"));
    eventClient.print(F("Host: "));
    eventClient.println(remoteStationIp);
    eventClient.println(F("Connection: close"));
    eventClient.println();

    delay(50);
    eventClient.stop();
    lastEventSentTime = now;
  }
  wdt_reset();
}

//=============================================================================
// ETHERNET INITIALIZATION
//=============================================================================

/**
 * Initialize Ethernet with hardware reset and retry logic.
 * Attempts up to 3 times with delays for W5100/W5500 stabilization.
 * Sets ethernet_initialized flag on success.
 */
void setupEthernet() {
  // FIX (P1): a REAL chip reset, ONCE per call. The chip-select toggle that was here does not
  // reset the W5500, and after the first W5100Class::init() every Ethernet.begin() only
  // rewrites MAC/IP - so a wedged chip and leaked sockets survived every "reset", and each
  // call added another LISTEN socket via server.begin() until all 8 were used up (outbound
  // connect() then fails -> false IP auto-close). A software reset (MR = 0x80) closes all
  // sockets and restores the chip defaults the library relies on (2 KB socket buffers).
  // The reset restarts the PHY; the three attempts below give it up to 15 s to re-link
  // without resetting it again in between.
  lastEthernetAttempt = millis();
  const bool linkWasOff = (Ethernet.linkStatus() == LinkOFF);   // cable out at entry: one short attempt only
  wdt_reset();
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  SPI.beginTransaction(SPI_ETHERNET_SETTINGS);
  W5100.writeMR(0x80);                                        // MR RST bit = software reset
  for (byte k = 0; k < 20 && W5100.readMR() != 0; k++) delay(1);   // the library's softReset(), which is private
  SPI.endTransaction();
  delay(50);

  for (int attempt = 1; attempt <= 3; attempt++) {
    wdt_reset();  // Reset watchdog before each attempt (delays can accumulate past 8s)

    // Initialize Ethernet with static IP (no DHCP for reliability)
    Ethernet.begin(mac, ip);
    wdt_reset();
    // FIX (P1): wait for the PHY to come up (up to 5 s per attempt, was a fixed 5 s) so a
    // rebuild costs ~1-2 s instead of 5.5 s; a W5100 (link state Unknown) waits the full 5 s.
    for (byte w = 0; w < (linkWasOff ? 20 : 50) && Ethernet.linkStatus() != LinkON; w++) delay(100);
    wdt_reset();

    // Verify physical link is connected. FIX (P1): with the cable out there is nothing to
    // retry - give up now (networkWatchdog() tries again a minute later) instead of
    // blocking loop() for 3 x 7 s. If the link was up at entry it is only the PHY
    // re-negotiating after the reset: keep waiting in the next attempt.
    if (Ethernet.linkStatus() == LinkOFF) {
      if (linkWasOff) break;
      continue;
    }

    // Verify IP was assigned correctly
    IPAddress currentIP = Ethernet.localIP();
    if (currentIP != IPAddress(0,0,0,0)) {
      server.begin();
      ethernet_initialized = true;
      lastEthernetReset = millis();
      lastSuccessfulPing = millis();
      // Reset IP fail counters - prior failures are irrelevant after stack rebuild
      connectFailCount = 0;
      firstFailTimestamp = 0;
      // Skip the next IP check to let the W5100/W5500 fully stabilize
      skipNextIpCheck = true;
      return;  // Success
    }
    delay(2000);
  }

  // All attempts failed - start server anyway for local recovery
  server.begin();
  ethernet_initialized = false;
}

//=============================================================================
// NETWORK WATCHDOG
//=============================================================================

/**
 * Monitors network health and performs recovery actions.
 * - Checks link and IP status every 10 minutes
 * - Reinitializes Ethernet if problems detected
 * - Performs preventive reset every 8 hours for long-term stability
 */
void networkWatchdog() {
  unsigned long currentTime = millis();
  static unsigned long lastCableCheck = 0;

  // --- Standalone mode: check for late cable insertion every 1 minute ---
  if (!networkMonitoringEnabled) {
    if (currentTime - lastCableCheck > 60000UL) {  // 1 minute
      lastCableCheck = currentTime;
      #if defined(SERIAL_DEBUG_IP)
      Serial.print(F("Standalone: Checking for cable... Link="));
      Serial.println(Ethernet.linkStatus() == LinkON ? 1 : 0);
      #endif
      // Quick non-blocking check if cable was inserted
      if (Ethernet.linkStatus() == LinkON) {
        #if defined(SERIAL_DEBUG_IP)
        Serial.println(F("Standalone: Cable found! Initializing network..."));
        #endif
        setupEthernet();  // Now do full init since cable is present
        if (ethernet_initialized) {
          networkMonitoringEnabled = true;
          #if defined(SERIAL_DEBUG_IP)
          Serial.println(F("Standalone: Network ready - IP monitoring ENABLED"));
          #endif
        }
      }
    }
    return;  // Skip all other network checks in standalone mode
  }

  // Periodic health check (network mode only): flags the stack as dead
  if (currentTime - lastNetworkCheck > NETWORK_CHECK_INTERVAL) {
    lastNetworkCheck = currentTime;
    if (Ethernet.linkStatus() == LinkOFF) {
      ethernet_initialized = false;   // cable out: the auto-close logic handles it
    } else if (Ethernet.localIP() == IPAddress(0,0,0,0)) {
      ethernet_initialized = false;   // link up but IP lost: software issue
    }
  }

  // FIX (P1): rebuild the stack whenever it is flagged dead and the link is back. Before,
  // nothing ever set ethernet_initialized back to true once a health check had seen the
  // link down (the W5500 keeps its static IP, so the old "IP lost" test never fired):
  // every IP check then counted as a failure and each dome opening ended in a false
  // "IP Fail Auto-Close" after ~15 min - until reboot. Rate-limited to one attempt per
  // minute, deferred while a motor runs (setupEthernet() blocks loop() for up to ~16 s).
  if (!ethernet_initialized && mot1dir == 0 && mot2dir == 0 &&
      currentTime - lastEthernetAttempt > 60000UL && Ethernet.linkStatus() != LinkOFF) {
    setupEthernet();
    return;   // lastEthernetReset just moved past currentTime: skip the 8 h test this pass
  }

  // Preventive reset every 8 h for long-term stability (only when everything is working).
  // FIX (P1): it is a real chip reset now, so only with the dome fully CLOSED and the motors
  // idle: a PHY that takes long to re-link must never trigger the cable auto-close on an
  // open dome, and a STOP/CLOSE must not be delayed by the block.
  if (networkMonitoringEnabled && ethernet_initialized && mot1dir == 0 && mot2dir == 0 &&
      digitalRead(lim1open) && digitalRead(lim2open) &&
      (currentTime - lastEthernetReset > ETHERNET_RESET_INTERVAL)) {
    setupEthernet();
  }
}

//=============================================================================
// W5500 SOCKET HYGIENE (P1)
//=============================================================================
// (1) A peer that connects and never sends a byte (browser pre-connect from a phone that
//     then leaves Wi-Fi, port scan, dead peer) keeps a hardware socket ESTABLISHED forever:
//     EthernetServer::available() only returns sockets that have data, so the sketch never
//     closes it. After 7 such sockets every outbound connect() (IP check, tick log, events)
//     fails and the dome is auto-closed for no reason - until reboot. Drop idle clients.
// (2) One LISTEN socket accepts one connection; while it is busy a second SYN is refused,
//     which is how a rain-checker close command gets lost when a browser tab and NINA are
//     polling at the same moment. Keep two listeners armed.
#define WEB_PORT            80
#define SOCKET_IDLE_LIMIT   30  // s without a request byte before an idle client is dropped
#define SOCKET_LISTENERS     2  // listening sockets kept ready on port 80
#define SOCKET_DROP_TIMEOUT 100 // ms to wait for the FIN handshake of a presumably dead peer

void socketHygiene() {
  static unsigned long lastSweep = 0;
  static byte idleSecs[MAX_SOCK_NUM];
  unsigned long now = millis();
  if (now - lastSweep < 1000) return;
  lastSweep = now;
  if (!ethernet_initialized) return;

  byte listening = 0, oldest = MAX_SOCK_NUM, oldestIdle = 0;
  bool dropped = false;
  for (byte i = 0; i < MAX_SOCK_NUM; i++) {
    if (EthernetServer::server_port[i] != WEB_PORT) { idleSecs[i] = 0; continue; }
    EthernetClient c(i);
    byte st = c.status();
    if (st == SnSR::LISTEN) {
      listening++;
      idleSecs[i] = 0;
    } else if (st == SnSR::ESTABLISHED && c.available() == 0) {
      if (idleSecs[i] < 255) idleSecs[i]++;
      if (idleSecs[i] > oldestIdle) { oldestIdle = idleSecs[i]; oldest = i; }
      // at most ONE drop per sweep with a short FIN wait: a dead peer never answers the
      // FIN, and several stop() calls in one loop() pass would add up towards the watchdog
      if (idleSecs[i] >= SOCKET_IDLE_LIMIT && !dropped) {
        c.setConnectionTimeout(SOCKET_DROP_TIMEOUT);
        c.stop();
        idleSecs[i] = 0;
        dropped = true;
      }
    } else {
      idleSecs[i] = 0;
    }
  }
  // no listener left and every socket is held by an idle client (pre-connect burst from a
  // phone): reclaim the longest-idle one now instead of 30 s later, so an incoming close
  // command is not refused for half a minute
  if (listening == 0 && !dropped && oldest < MAX_SOCK_NUM && oldestIdle >= 2) {
    EthernetClient c(oldest);
    c.setConnectionTimeout(SOCKET_DROP_TIMEOUT);
    c.stop();
    idleSecs[oldest] = 0;
  }
  while (listening < SOCKET_LISTENERS) { server.begin(); listening++; }   // no-op when no socket is free
}

//=============================================================================
// SETUP - One-time initialization at power-on
//=============================================================================
void setup() {
  // --- Debug Serial (optional) ---
  #if defined(SERIAL_DEBUG_GENERAL) || defined(SERIAL_DEBUG_IP) || defined(SERIAL_DEBUG_BUTTONS) || defined(SERIAL_DEBUG_LIMITS) || defined(SERIAL_DEBUG_EEPROM)
    Serial.begin(115200);
    unsigned long setupSerialStart = millis();
    while(!Serial && (millis() - setupSerialStart < 2000)) { delay(10); }
    Serial.println(F("------------------------------"));
    Serial.println(F("Dome Control v4.0.2 - Safety Sensors"));
    Serial.println(F("Setup: Serial initialized."));
    if (lim2open == 1 || lim2open == 0 || lim2closed == 1 || lim2closed == 0) {
      Serial.println(F("WARNING: Pins 0/1 used for limit switches! Serial debug may cause malfunctions!"));
    }
  #endif

  // --- Motor outputs LOW as the very first thing (FIX (P1): the H-bridge inputs float from
  // reset until pinMode; keep that window as short as possible; the bootloader window before
  // it can only be covered by pull-downs on the driver inputs) ---
  pinMode(motor1a, OUTPUT); digitalWrite(motor1a, LOW);
  pinMode(motor1b, OUTPUT); digitalWrite(motor1b, LOW);
  pinMode(motor2a, OUTPUT); digitalWrite(motor2a, LOW);
  pinMode(motor2b, OUTPUT); digitalWrite(motor2b, LOW);

  // --- Enable hardware watchdog FIRST ---
  // FIX (P1): was enabled at the very end of setup(). Any hang before that point (sensor
  // init, Ethernet init) froze the controller permanently with no reset. Every blocking
  // step in setup() is < 8 s or calls wdt_reset() in between.
  wdt_enable(WDTO_8S);

  // --- Load persistent counters from EEPROM ---
  initializeEEPROM();

  // --- Initialize motor control state ---
  mot1dir = 0; mot2dir = 0;           // Motors off
  stop1reason = 0; stop2reason = 0;   // Clear stop reasons
  m1AutoClosedByIP = false; m2AutoClosedByIP = false;
  cableRemovalAutoCloseTriggered = false;
  vccerr = 0;
  cnt = 0;
  connectFailCount = 0;
  firstFailTimestamp = 0;

  // --- Initialize button state tracking ---
  sw1up_pressed_flag = false; sw1down_pressed_flag = false;
  sw2up_pressed_flag = false; sw2down_pressed_flag = false;
  swstop_pressed_flag = 0;

  // --- Configure GPIO pins ---
  // Motor outputs (directly drive H-bridge via PWM)
  pinMode(motor1a, OUTPUT); pinMode(motor1b, OUTPUT);
  pinMode(motor2a, OUTPUT); pinMode(motor2b, OUTPUT);

  // Limit switches with internal pullup (active HIGH = endpoint reached)
  pinMode(lim1open, INPUT_PULLUP); pinMode(lim1closed, INPUT_PULLUP);
  pinMode(lim2open, INPUT_PULLUP); pinMode(lim2closed, INPUT_PULLUP);

  // Control buttons with internal pullup (active LOW = pressed)
  pinMode(SW1up, INPUT_PULLUP); pinMode(SW1down, INPUT_PULLUP);
  pinMode(SW2up, INPUT_PULLUP); pinMode(SW2down, INPUT_PULLUP);
  pinMode(SWSTOP, INPUT_PULLUP);

  // Ethernet Shield chip select
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  // --- Configure Timer2 for ISR FIRST ---
  // This ensures physical buttons work immediately, even during Ethernet setup.
  // Timer2 generates ~61 Hz interrupt for motor control and button handling.
  // CTC mode with prescaler 1024: 16MHz / (1024 * 256) = 61 Hz
  cli();                                            // Disable interrupts
  OCR2A = 255;                                      // Compare match value
  TCCR2A |= (1 << WGM21);                           // CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
  TIMSK2 |= (1 << OCIE2A);                          // Enable compare interrupt
  sei();                                            // Enable interrupts

  system_fully_ready = true;  // Allow ISR to process motor commands immediately

  #if defined(SERIAL_DEBUG_GENERAL)
  Serial.println(F("Setup: Timer ISR configured - buttons now active."));
  #endif

  // --- Initialize sensors AFTER pins/ISR (FIX (P1): buttons and motors work even if a sensor
  // init misbehaves) and BEFORE Ethernet (sensors are fast, Ethernet is slow) ---
  // DS18B20: ~10ms init + 800ms first read — temperature ready for first motor command
  wdt_reset();
  setupDS18B20();

  // VL53L0X: I2C init — starts continuous measurement mode (up to ~5 s on a half-dead bus)
  wdt_reset();
  setupVL53L0X();
  loadToFCalibration();
  wdt_reset();

  // --- Quick cable detection before full Ethernet init ---
  // Minimal SPI init to check if cable is present
  delay(100);
  Ethernet.begin(mac, ip);  // Quick init to read link status
  delay(500);               // Brief stabilization

  #if defined(SERIAL_DEBUG_IP) || defined(SERIAL_DEBUG_GENERAL)
  Serial.print(F("Setup: Initial Link="));
  Serial.println(Ethernet.linkStatus() == LinkON ? 1 : 0);
  #endif

  if (Ethernet.linkStatus() == LinkON) {
    // Cable present - do full Ethernet setup with retries
    networkMonitoringEnabled = true;
    #if defined(SERIAL_DEBUG_IP) || defined(SERIAL_DEBUG_GENERAL)
    Serial.println(F("Setup: Cable detected - initializing network..."));
    #endif
    setupEthernet();
    #if defined(SERIAL_DEBUG_IP) || defined(SERIAL_DEBUG_GENERAL)
    Serial.println(F("Setup: Network ready, IP monitoring ENABLED."));
    #endif
  } else {
    // No cable - skip lengthy Ethernet setup, run in standalone mode
    networkMonitoringEnabled = false;
    ethernet_initialized = false;
    #if defined(SERIAL_DEBUG_IP) || defined(SERIAL_DEBUG_GENERAL)
    Serial.println(F("Setup: No cable - STANDALONE MODE, IP monitoring DISABLED."));
    #endif
  }

  // Hardware watchdog (8 s, AVR maximum) has been running since the top of setup().
  // setupEthernet() uses wdt_reset() between delays to prevent false triggers.
  wdt_reset();
}

//=============================================================================
// IP MONITORING AND AUTO-CLOSE LOGIC
//=============================================================================
/**
 * Monitors connectivity to Cloudwatcher and triggers auto-close on failure.
 *
 * Safety Logic:
 * - Checks connection every 1 minute when dome is not fully closed
 * - Counts failures within a sliding 30-minute window
 * - After 5 consecutive failures, auto-closes dome immediately
 * - Works regardless of dome state: fully open, intermediate, or moving
 * - If dome is opening, it will stop and reverse to close
 * - Recovers gracefully if connection is restored during close
 *
 * This protects the telescope from rain if the Cloudwatcher (which monitors
 * weather) becomes unreachable - we assume the worst and close the dome.
 */
void checkRemoteConnectionAndAutoClose() {
    // Skip all IP monitoring if no cable has ever been detected
    // This allows non-networked setups to operate without false auto-closes
    if (!networkMonitoringEnabled) {
      return;
    }

    // --- Debug: Show current status ---
    #if defined(SERIAL_DEBUG_IP)
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 10000) {  // Print status every 10 seconds
      lastDebugPrint = millis();
      Serial.print(F("IP Status: Link="));
      Serial.print(Ethernet.linkStatus() == LinkON ? 1 : 0);
      Serial.print(F(" EthInit="));
      Serial.print(ethernet_initialized ? 1 : 0);
      Serial.print(F(" NetMon="));
      Serial.print(networkMonitoringEnabled ? 1 : 0);
      Serial.print(F(" Fails="));
      Serial.print(connectFailCount);
      Serial.print(F("/"));
      Serial.print(maxConnectFails);
      Serial.print(F(" S1closed="));
      Serial.print(digitalRead(lim1open) ? 1 : 0);
      Serial.print(F(" S2closed="));
      Serial.println(digitalRead(lim2open) ? 1 : 0);
    }
    #endif

    bool networkProblem = false;

    if (!ethernet_initialized) {
        networkProblem = true;
        #if defined(SERIAL_DEBUG_IP)
        Serial.println(F("IP Check: EthInit=0"));
        #endif
    }

    // Determine if dome needs protection (any state except fully closed)
    // Covers: fully open, intermediate position, currently opening, currently closing
    bool domeIsNotFullyClosed;
    if (!digitalRead(lim1open) || !digitalRead(lim2open)) {
        // At least one shutter is not at closed position
        domeIsNotFullyClosed = true;
    } else if (mot1dir == CLOSE || mot2dir == CLOSE) {
        // Motors actively opening (even if at closed limit, motor is trying to open)
        domeIsNotFullyClosed = true;
    } else {
        // Both shutters at closed position and motors not opening
        domeIsNotFullyClosed = false;
    }

    if (domeIsNotFullyClosed) {
        // *** Check for network problems ***
        bool linkDown = false;
        bool needsRecovery = false;

        // FIX (P2): the link state is one 1-byte SPI read per loop() pass next to two PWM motor
        // drivers; a single corrupted read must not start the cable-removal auto-close. LinkOFF
        // has to persist for 200 ms before the cable logic acts.
        static unsigned long linkOffSince = 0;
        EthernetLinkStatus linkNow = Ethernet.linkStatus();
        if (linkNow == LinkOFF) {
            if (linkOffSince == 0) linkOffSince = millis() | 1UL;
            if (millis() - linkOffSince < 200UL) return;
        } else {
            linkOffSince = 0;
        }

        if (linkNow == LinkOFF) {
            linkDown = true;
            networkProblem = true;

            // Cable physically removed - close dome immediately (only once per removal)
            if (!cableRemovalAutoCloseTriggered) {
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F("IP Check: Cable removed - triggering immediate auto-close"));
                #endif

                bool action_taken = false;
                if (mot1dir != OPEN && !digitalRead(lim1open)) {
                    startMotor1(OPEN, dynTimeout_M1_Close);
                    m1AutoClosedByIP = true; stop1reason = 3; action_taken = true;
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F(">>> AUTO-CLOSE: S1 motor started (cable removed)"));
                    #endif
                }
                if (mot2dir != OPEN && !digitalRead(lim2open)) {
                    startMotor2(OPEN, dynTimeout_M2_Close);
                    m2AutoClosedByIP = true; stop2reason = 3; action_taken = true;
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F(">>> AUTO-CLOSE: S2 motor started (cable removed)"));
                    #endif
                }
                if (action_taken) {
                    totalAutoCloses++;
                    eepromDirty = true;
                    saveCountersToEEPROM();
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.print(F(">>> AUTO-CLOSE COUNT: "));
                    Serial.println(totalAutoCloses);
                    #endif
                }
                // FIX (P1): the safety event itself (not only a started motor) cancels a running
                // frozen-dome cycle, so its retry can never re-open the dome with the network gone.
                fdNoteCommand(FD_CMD_ALL);
                cableRemovalAutoCloseTriggered = true;  // Prevent repeated triggering
            }
            return;  // Skip further checks while cable is out
        } else if (linkNow == LinkON) {
            // Cable is back - reset the trigger flag
            if (cableRemovalAutoCloseTriggered) {
                cableRemovalAutoCloseTriggered = false;
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F("IP Check: Cable reconnected - reset trigger flag"));
                #endif
            }
            // Link ist da, aber IP prüfen
            IPAddress currentIP = Ethernet.localIP();
            if (currentIP == IPAddress(0,0,0,0)) {
                needsRecovery = true;
                networkProblem = true;
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F("IP Check: Link OK but IP lost - needs recovery"));
                #endif
            }
        }
        
        // FIX (P1): the "Auto-Recovery" block that used to be here was dead code (its flag was
        // never set on a real link loss and it required ethernet_initialized==true). All
        // rebuilds now happen in networkWatchdog(): rate-limited, deferred while a motor runs.
        if (needsRecovery) ethernet_initialized = false;
        
        // Check connection with longer intervals
        if (millis() - lastConnectAttemptTimestamp >= connectCheckInterval) {
            // Skip one check after Ethernet stack reset to let W5100/W5500 stabilize
            if (skipNextIpCheck) {
                skipNextIpCheck = false;
                lastConnectAttemptTimestamp = millis();
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F("IP Check: Skipped (post-reset stabilization)"));
                #endif
                return;
            }

            bool connectionOK = false;

            // If network has problems, count as failed attempt
            if (networkProblem) {
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F("IP Check: Network problem - counting as failed attempt"));
                #endif
                connectionOK = false;
            }
            // Only try to connect if network is OK
            else if (!linkDown && !needsRecovery && ethernet_initialized) {
                EthernetClient netClient;
                netClient.setConnectionTimeout(2000);  // 2 s bound for connect()/stop() (P2; was effectively 1 s), well below the 8 s watchdog

                #if defined(SERIAL_DEBUG_IP)
                Serial.print(F("IP Check: Connecting to "));Serial.print(remoteStationIp);Serial.println(F("..."));
                #endif

                wdt_reset();  // Reset watchdog before potentially blocking call
                unsigned long connectStart = millis();
                bool connected = netClient.connect(remoteStationIp, remoteStationPort);
                wdt_reset();  // Reset watchdog after blocking call

                if (connected && (millis() - connectStart < 5000)) {
                    connectionOK = true;
                    lastSuccessfulPing = millis();
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F("IP Check: Connection successful."));
                    #endif
                    netClient.stop();
                } else {
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F("IP Check: Connection failed - target unreachable."));
                    #endif
                    if (netClient.connected()) {
                        netClient.stop();
                    }
                }
                wdt_reset();   // P2: connect()+stop() can take 4 s together
            }
            
            if (connectionOK) {
                // SUCCESS - Reset all fail counters
                connectFailCount = 0;
                firstFailTimestamp = 0; // Reset time window
                
                // STOPPE AUTO-CLOSE BEI ERFOLGREICHER VERBINDUNG
                if (m1AutoClosedByIP && mot1dir == OPEN) {
                    mot1dir = 0; stop1reason = 0; m1AutoClosedByIP = false; fdNoteCommand(FD_CMD_M1);
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F("IP Check: M1 Auto-Close STOPPED - Connection restored"));
                    #endif
                }
                if (m2AutoClosedByIP && mot2dir == OPEN) {
                    mot2dir = 0; stop2reason = 0; m2AutoClosedByIP = false; fdNoteCommand(FD_CMD_M2);
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F("IP Check: M2 Auto-Close STOPPED - Connection restored"));
                    #endif
                }
            } else {
                // FAILURE - Increment counter and track time
                if (firstFailTimestamp == 0) {
                    firstFailTimestamp = millis(); // Start time window
                }
                
                connectFailCount++;
                totalIpFailures++; // Increment persistent counter
                eepromDirty = true;
                
                // Check if we're still within the 5-minute window
                if (millis() - firstFailTimestamp > maxFailTimeWindow) {
                    // Time window expired - reset counters
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F("IP Check: 30-minute window expired - resetting fail count"));
                    #endif
                    connectFailCount = 1; // Start fresh with this failure
                    firstFailTimestamp = millis();
                }
            }
            
            lastConnectAttemptTimestamp = millis();
            #if defined(SERIAL_DEBUG_IP)
            Serial.print(F("IP Check: Fail count: ")); Serial.print(connectFailCount);
            Serial.print(F("/")); Serial.print(maxConnectFails);
            Serial.print(F(" in ")); Serial.print((millis() - firstFailTimestamp) / 60000);
            Serial.println(F(" minutes"));
            if (networkProblem) {
                if (!ethernet_initialized) Serial.println(F("Reason: Ethernet not initialized"));
                else if (linkDown) Serial.println(F("Reason: Cable disconnected"));
                else if (needsRecovery) Serial.println(F("Reason: IP lost"));
            }
            #endif
        }
        
        // Trigger auto-close after 5 failures within 5-minute window
        if (connectFailCount >= maxConnectFails &&
            (millis() - firstFailTimestamp <= maxFailTimeWindow)) {
            #if defined(SERIAL_DEBUG_IP)
            Serial.println(F("IP Check: Max fails within 30 minutes. Triggering auto-close."));
            if (networkProblem) {
                if (!ethernet_initialized) Serial.println(F("Reason: Ethernet stack problem"));
                else if (linkDown) Serial.println(F("Reason: Cable disconnected"));
                else if (needsRecovery) Serial.println(F("Reason: IP lost"));
            } else {
                Serial.println(F("Reason: Target IP unreachable"));
            }
            #endif

            // Close both shutters immediately
            // - If motor is off: start closing
            // - If motor is opening: reverse to close (soft-start handles transition)
            // - If motor already closing: no change needed
            bool action_taken = false;
            if (mot1dir != OPEN && !digitalRead(lim1open)) {
                startMotor1(OPEN, dynTimeout_M1_Close);
                m1AutoClosedByIP = true; stop1reason = 3; action_taken = true;
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F(">>> AUTO-CLOSE: S1 motor started (5 failures)"));
                #endif
            }
            if (mot2dir != OPEN && !digitalRead(lim2open)) {
                startMotor2(OPEN, dynTimeout_M2_Close);
                m2AutoClosedByIP = true; stop2reason = 3; action_taken = true;
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F(">>> AUTO-CLOSE: S2 motor started (5 failures)"));
                #endif
            }

            fdNoteCommand(FD_CMD_ALL);   // FIX (P1): the auto-close event cancels a frozen-dome cycle
            if (action_taken) {
                totalAutoCloses++;
                eepromDirty = true;
                saveCountersToEEPROM();  // Immediate save - this is a critical event
                #if defined(SERIAL_DEBUG_IP)
                Serial.print(F(">>> AUTO-CLOSE COUNT: "));
                Serial.println(totalAutoCloses);
                #endif
            }

            connectFailCount = 0;
            firstFailTimestamp = 0;
        }
    } else { 
        // Kuppel ist geschlossen - alles zurücksetzen
        connectFailCount = 0; 
        firstFailTimestamp = 0;
        lastConnectAttemptTimestamp = millis(); 
        if (m1AutoClosedByIP) m1AutoClosedByIP = false;
        if (m2AutoClosedByIP) m2AutoClosedByIP = false;
    }
}

//=============================================================================
// BOUNDED, BUFFERED PAGE WRITER (P1)
//=============================================================================
// EthernetClient::write() -> socketSend() waits WITHOUT a time bound for TX buffer space
// while the socket stays ESTABLISHED. A browser that stops reading mid-page (phone put to
// sleep: zero TCP window) blocked loop() until the 8 s watchdog rebooted the board with
// the shutter stuck mid-travel and no auto-close (the network itself is healthy).
// This wrapper waits at most PAGE_WRITE_TIMEOUT ms for buffer space and drops the client
// otherwise, and batches the byte-wise print(F()) output into 64-byte TCP segments
// instead of one segment per character (~5400 per page before).
// Residual: the chip-level wait for the SEND acknowledge inside socketSend() is not
// controllable from the sketch; the watchdog remains the backstop for that case.
// handleWebClient() no longer calls client.flush() (unbounded in Ethernet 2.0.x): the
// chip transmits buffered data before the FIN that stop() sends.
#define PAGE_WRITE_TIMEOUT 500   // ms to wait for TX buffer space before dropping the client
#define PAGE_TOTAL_TIMEOUT 3000  // ms for the whole page (a peer draining slowly must not add up to the watchdog)

class PageWriter : public Print {
 public:
  explicit PageWriter(EthernetClient& c) : client(c), pageStart(millis()) {}
  size_t write(uint8_t b) override {
    if (failed) return 0;
    buf[len++] = b;
    if (len == sizeof(buf)) flushBuf();
    return 1;
  }
  void flushBuf() {
    if (failed || len == 0) return;
    unsigned long t0 = millis();
    while (client.availableForWrite() < (int)len) {
      if (!client.connected() || millis() - t0 > PAGE_WRITE_TIMEOUT || millis() - pageStart > PAGE_TOTAL_TIMEOUT) {
        failed = true; len = 0; client.setConnectionTimeout(200); client.stop(); return;
      }
      delay(1);
    }
    client.write(buf, len);
    len = 0;
  }
 private:
  EthernetClient& client;
  unsigned long pageStart;
  uint8_t buf[64];
  byte len = 0;
  bool failed = false;
};

//=============================================================================
// WEB CLIENT HANDLER
//=============================================================================
/**
 * Processes incoming HTTP requests from web browsers.
 *
 * URL Commands (append to base URL):
 *   /?$1  - Close Shutter 1 (physically)
 *   /?$2  - Open Shutter 1 (physically)
 *   /?$3  - Close Shutter 2 (physically)
 *   /?$4  - Open Shutter 2 (physically)
 *   /?$5  - STOP all motors
 *   /?$S  - Return plain text status (OPEN/CLOSED)
 *   /?$A  - ASCOM status: S1_STATE|S1_MOTOR|S2_STATE|S2_MOTOR (pipe-delimited)
 *   /?$R  - Reset persistent counters
 *   /?$L  - Toggle tick logging on/off
 *
 * Design notes:
 * - 1.5 second timeout prevents blocking on slow/broken connections
 * - Action commands return 303 redirect for clean browser behavior
 * - Status request returns minimal plain text for scripting
 */
void handleWebClient() {
  EthernetClient client = server.available();
  if (!client) return;

  lastSuccessfulPing = millis();  // Track activity for monitoring

  #if defined(SERIAL_DEBUG_GENERAL)
  Serial.println(F("Web: Client connected."));
  #endif

  boolean currentLineIsBlank = true;
  bool inRequestLine = true;      // FIX (P1): '$' commands only count in "GET /?$x HTTP/1.1", never in headers
  newInfo = false;                // FIX (P1): parser state must not leak from a truncated request
  unsigned long clientRequestStart = millis();
  bool action_parameter_in_url = false;
  bool plain_text_status_request = false;
  bool ascom_status_request = false;
  bool reset_counters_request = false;

  while (client.connected()) {
    // MUCH MORE AGGRESSIVE TIMEOUT - Key fix!
    if (millis() - clientRequestStart > 1500) { // 1.5s instead of 5s!
      #if defined(SERIAL_DEBUG_GENERAL)
      Serial.println(F("Web: Quick timeout to prevent blocking"));
      #endif
      client.stop(); break;
    }
    if (client.available()) {
      char c = client.read();
      if (newInfo && c == ' ') { newInfo = false; }
      if (c == '\n') { inRequestLine = false; }
      if (c == '$' && inRequestLine) { newInfo = true; }

      if (newInfo && system_fully_ready) { 
        if (c >= '1' && c <= '5') { 
          action_parameter_in_url = true; 
          // FIX (P1): tell the frozen-dome cycle which shutter got an external command
          fdNoteCommand((c == '5') ? FD_CMD_ALL : ((c == '1' || c == '2') ? FD_CMD_M1 : FD_CMD_M2));
           #if defined(SERIAL_DEBUG_GENERAL)
              Serial.print(F("Web: Action param: $")); Serial.println(c);
           #endif
        }
        
        // Check for reset counters command
        if (c == 'R' || c == 'r') {
          reset_counters_request = true;
          action_parameter_in_url = true;
        }

        // Check for tick logging toggle command
        if (c == 'L' || c == 'l') {
          tickLoggingEnabled = !tickLoggingEnabled;  // Toggle on/off
          action_parameter_in_url = true;
        }

        // $U = Unlock dome from frozen lockout
        if (c == 'U' || c == 'u') {
          if (fdOpenBlocked()) {   // FIX (P1): also while the lockout is pending after the 3rd verdict
            frozenDomeState = FD_IDLE;
            frozenRetryCount = 0;
            frozenCheckActive = false;
          }
          action_parameter_in_url = true;
        }

        // $C = Calibrate ToF baseline (dome must be closed)
        if (c == 'C' || c == 'c') {
          // Only calibrate if dome is fully closed (both shutters at closed limit)
          if (digitalRead(lim1open) && digitalRead(lim2open) && tof_connected) {
            calibrateToF();
          }
          action_parameter_in_url = true;
        }

 // IMPROVED WEB LOGIC - Automatic stop before new command
        if (c == '1') {
          // $1 = CLOSE Shutter 1 (physically)
          if (mot1dir != 0 && mot1dir != OPEN) {
              mot1dir = 0; stop1reason = 2; m1AutoClosedByIP = false;
          }
          if (!digitalRead(lim1open)) {
              if (mot1dir != OPEN) startMotor1(OPEN, dynTimeout_M1_Close);   // P2: a repeated command must not reload the timeout
              stop1reason = 0; m1AutoClosedByIP = false;                    // ... but it still takes ownership of the motion
          }
        } else if (c == '2') {
          // $2 = OPEN Shutter 1 (physically) — check frozen lockout
          if (fdOpenBlocked()) {
            // Blocked: dome is locked due to frozen detection
          } else {
            if (mot1dir != 0 && mot1dir != CLOSE) {
                mot1dir = 0; stop1reason = 2; m1AutoClosedByIP = false;
            }
            if (!digitalRead(lim1closed)) {
                if (mot1dir != CLOSE) startMotor1(CLOSE, dynTimeout_M1_Open);
                stop1reason = 0; m1AutoClosedByIP = false;
            }
          }
        } else if (c == '3') {
          // $3 = CLOSE Shutter 2 (physically)
          if (mot2dir != 0 && mot2dir != OPEN) {
              mot2dir = 0; stop2reason = 2; m2AutoClosedByIP = false;
          }
          if (!digitalRead(lim2open)) {
              if (mot2dir != OPEN) startMotor2(OPEN, dynTimeout_M2_Close);
              stop2reason = 0; m2AutoClosedByIP = false;
          }
        } else if (c == '4') {
          // $4 = OPEN Shutter 2 (physically) — check frozen lockout
          if (fdOpenBlocked()) {
            // Blocked: dome is locked due to frozen detection
          } else {
            if (mot2dir != 0 && mot2dir != CLOSE) {
                mot2dir = 0; stop2reason = 2; m2AutoClosedByIP = false;
            }
            if (!digitalRead(lim2closed)) {
                if (mot2dir != CLOSE) startMotor2(CLOSE, dynTimeout_M2_Open);
                stop2reason = 0; m2AutoClosedByIP = false;
            }
          }
        } else if (c == '5') {
          mot1dir = 0; mot2dir = 0;
          stop1reason = 2; stop2reason = 2;
          m1AutoClosedByIP = false; m2AutoClosedByIP = false;
        }
        else if (c == 'S' || c == 's') {
          plain_text_status_request = true;
          action_parameter_in_url = false;
        }
        // $A = ASCOM status endpoint — compact pipe-delimited response
        // Format: S1_STATE|S1_MOTOR|S2_STATE|S2_MOTOR
        // States: OPEN, CLOSED, INTERMEDIATE
        // Motors: STOPPED, OPENING, CLOSING
        else if (c == 'A' || c == 'a') {
          ascom_status_request = true;
          action_parameter_in_url = false;
        }
        
        if (c != '$') { newInfo = false; }
      } else if (newInfo && !system_fully_ready) { 
          if (c != '$') newInfo = false; 
          if ((c >= '1' && c <= '5') || c == 'S' || c == 's' || c == 'R' || c == 'r' || c == 'A' || c == 'a') {
               action_parameter_in_url = true;
               if (c == 'S' || c == 's') plain_text_status_request = false;
               // $A (ASCOM status) is read-only, allow even during init
               if (c == 'A' || c == 'a') { ascom_status_request = true; action_parameter_in_url = false; }
          }
      }

      if (c == '\n' && currentLineIsBlank) { 
        #if defined(SERIAL_DEBUG_GENERAL)
        Serial.println(F("Web: Sending quick response"));
        #endif

        if (reset_counters_request) {
          // Reset persistent counters
          totalIpFailures = 0;
          totalAutoCloses = 0;
          EEPROM.put(EEPROM_ADDR_TOTAL_IP_FAILS, totalIpFailures);
          EEPROM.put(EEPROM_ADDR_AUTO_CLOSES, totalAutoCloses);
          
          client.println(F("HTTP/1.1 303 See Other")); 
          client.println(F("Location: /")); 
          client.println(F("Connection: close")); 
          client.println();
        } else if (plain_text_status_request) {
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/plain"));
          client.println(F("Connection: close"));
          client.println(); 
          if (digitalRead(lim1open) && digitalRead(lim2open)) {
            client.println(F("CLOSED"));
          } else {
            client.println(F("OPEN")); 
          }
        } else if (ascom_status_request) {
          // ASCOM driver endpoint — compact pipe-delimited status
          // Format: S1_STATE|S1_MOTOR|S2_STATE|S2_MOTOR
          // Physical reality (inverted from code variable names)
          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/plain"));
          client.println(F("Connection: close"));
          client.println();

          // Shutter 1 (East) physical state
          if (digitalRead(lim1closed))      client.print(F("OPEN"));       // lim1closed = physically open
          else if (digitalRead(lim1open))   client.print(F("CLOSED"));     // lim1open = physically closed
          else                              client.print(F("INTERMEDIATE"));
          client.print('|');

          // Shutter 1 motor direction (physical)
          if (mot1dir == CLOSE)             client.print(F("OPENING"));    // CLOSE direction = physically opening
          else if (mot1dir == OPEN)         client.print(F("CLOSING"));    // OPEN direction = physically closing
          else                              client.print(F("STOPPED"));
          client.print('|');

          // Shutter 2 (West) physical state
          if (digitalRead(lim2closed))      client.print(F("OPEN"));
          else if (digitalRead(lim2open))   client.print(F("CLOSED"));
          else                              client.print(F("INTERMEDIATE"));
          client.print('|');

          // Shutter 2 motor direction (physical)
          if (mot2dir == CLOSE)             client.print(F("OPENING"));
          else if (mot2dir == OPEN)         client.print(F("CLOSING"));
          else                              client.print(F("STOPPED"));
          client.println();

        } else if (action_parameter_in_url) {
          // FASTEST possible response for motor commands
          client.println(F("HTTP/1.1 303 See Other")); 
          client.println(F("Location: /")); 
          client.println(F("Connection: close")); 
          client.println();
        } else { 
          PageWriter page(client);   // FIX (P1): bounded wait for TX space, 64-byte segments
          sendFullHtmlResponse(page);
          page.flushBuf();
        } 
        break; 
      } 
      if (c == '\n') { currentLineIsBlank = true; }
      else if (c != '\r') { currentLineIsBlank = false; }
    } 
    
    // Shorter delay for more responsiveness  
    delayMicroseconds(500); // Was delay(1) = 1000μs, now 500μs
  } 
  
  // Quick disconnect
  client.flush();
  delay(2); // Shorter delay
  client.stop();
  #if defined(SERIAL_DEBUG_GENERAL)
  Serial.println(F("Web: Quick disconnect"));
  #endif
}

//=============================================================================
// HTML WEB INTERFACE GENERATOR
//=============================================================================
/**
 * Generates the full HTML control page.
 * - Responsive design for mobile devices
 * - Auto-refreshes every 10 seconds (2 seconds during init)
 * - Shows shutter status, motor state, sensor readings
 * - Provides control buttons and IP monitoring stats
 *
 * All strings use F() macro to store in flash, saving ~3KB of RAM.
 */
void sendFullHtmlResponse(Print& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html; charset=utf-8"));
  client.println(F("Connection: close"));
  // Auto-refresh: faster during init, slower in normal operation
  if (system_fully_ready) client.println(F("Refresh: 10"));
  else client.println(F("Refresh: 2")); 
  client.println();
  client.println(F("<!DOCTYPE HTML><html><head><title>AstroShell DomeControl JK4.0</title>"));
  client.println(F("<meta name='viewport' content='width=device-width, initial-scale=1.0'>"));
  client.println(F("<style>"));
  client.println(F("body{font-family:Arial,sans-serif;margin:0;padding:10px;background-color:#f0f0f0;color:#333;}"));
  client.println(F(".container{max-width:480px;margin:0 auto;background-color:#fff;padding:5px 15px 15px 15px;border-radius:8px;box-shadow:0 0 10px rgba(0,0,0,0.1);}"));
  client.println(F("h1,h2{text-align:center;color:#333;margin-top:15px;margin-bottom:10px;} h1{margin-bottom:20px;}"));
  client.println(F("a.button{display:inline-block;width:45%;padding:12px;margin:5px 2%;border:none;border-radius:8px;color:white!important;cursor:pointer;font-size:1em;text-align:center;text-decoration:none;box-sizing:border-box;}"));
  client.println(F("a.button.fullwidth{width:90%;}")); // For STOP button
  client.println(F(".b-open{background-color:#337ab7;} .b-close{background-color:#5cb85c;} .b-stop{background-color:#dc3545;}"));
  client.println(F(".b-reset{background-color:#6c757d;font-size:0.8em;padding:8px;}"));
  client.println(F(".status{margin-top:5px;margin-bottom:15px;padding:8px;border:1px solid #ccc;border-radius:4px;text-align:center;font-size:0.95em;}"));
  client.println(F(".section{margin-bottom:15px;padding:10px;border:1px solid #eee;border-radius:5px;}")); 
  client.println(F(".button-pair{display:flex;justify-content:space-around;margin-bottom:5px;}"));
  client.println(F("table{width:100%;margin-top:10px;border-collapse:collapse;} td,th{padding:6px;border:1px solid #ddd;text-align:left;font-size:0.9em;}"));
  client.println(F("th{background-color:#f8f8f8;}"));
  client.println(F(".social-links{text-align:center;font-size:0.8em;margin-bottom:15px;}")); // Social links CSS
  client.println(F(".warning{color:#d9534f;font-weight:bold;}"));
  client.println(F("</style></head><body><div class='container'>"));
  
  if (!system_fully_ready) {
       client.println(F("<h1>Dome System Initializing...</h1>"));
       client.println(F("<div class='status'>Please wait. Web interface will be active shortly.</div>"));
  } else {
      client.println(F("<h1>AstroShell Dome Control JK4.0</h1>"));
      client.println(F("<div class='social-links'>"));
      client.print(F("<a href='https://app.astrobin.com/u/joergsflow#gallery' target='_blank' rel='noopener noreferrer'>joergsflow Astrobin</a>"));
      client.print(F(" | ")); // Separator between links
      client.print(F("<a href='https://www.instagram.com/joergsflow/' target='_blank' rel='noopener noreferrer'>Instagram</a>"));
      client.println(F("</div>"));
      
      client.println(F("<div class='section' style='background-color:#fff0f0;'>")); 
      client.print(F("<a href='/?$5' class='button b-stop fullwidth'>STOP ALL MOTORS</a>"));
      client.print(F("<a href='/?$1$3' class='button b-close fullwidth' style='margin-top:8px;'>CLOSE ALL SHUTTERS</a>"));
      client.print(F("<a href='/?$2$4' class='button b-open fullwidth' style='margin-top:8px;'>OPEN ALL SHUTTERS</a>"));
      client.println(F("</div>"));

      // --- Shutter 1 (East) ---
      client.println(F("<div class='section'><h2>Shutter 1 (East)</h2>"));
      client.println(F("<div class='button-pair'>"));
      client.print(F("<a href='/?$2' class='button b-open'>OPEN S1</a>"));  // $2 = Physically Open Shutter 1
      client.print(F("<a href='/?$1' class='button b-close'>CLOSE S1</a>"));// $1 = Physically Close Shutter 1
      client.println(F("</div>"));
      client.println(F("<div class='status'>"));
      // Status display logic for Shutter 1
      bool s1_is_physically_closed_state = digitalRead(lim1open);
      bool s1_is_physically_open_state = digitalRead(lim1closed);
      client.print(F("<strong>State:</strong> "));
      if (s1_is_physically_closed_state) client.print(F("Physically CLOSED"));
      else if (s1_is_physically_open_state) client.print(F("Physically OPEN"));
      else client.print(F("Intermediate"));
      if (mot1dir == 0 && !s1_is_physically_open_state && !s1_is_physically_closed_state) client.print(F(" (Stopped)"));

      client.print(F("<br><strong>Movement:</strong> "));
      if (mot1dir == OPEN) { client.print(F("Closing (physically)")); if(m1AutoClosedByIP) client.print(F(" (IP Fail)")); if(stop1reason==4) client.print(F(" (VCC Fail)"));}
      else if (mot1dir == CLOSE) { client.print(F("Opening (physically)"));}
      else {client.print(F("Stopped"));}

      if(mot1dir == 0) {
          client.print(F("<br><strong>Stop Reason:</strong> "));
          if (s1_is_physically_open_state && stop1reason==0) client.print(F("Limit 'Phys. Open' (lim1closed)"));
          else if (s1_is_physically_closed_state && stop1reason==0) client.print(F("Limit 'Phys. Closed' (lim1open)"));
          else if (stop1reason == 1) client.print(F("Button/SWSTOP"));
          else if (stop1reason == 2) client.print(F("Web STOP"));
          else if (stop1reason == 3 && m1AutoClosedByIP) client.print(F("IP Fail Auto-Close"));
          else if (stop1reason == 4) client.print(F("VCC Fail Auto-Close"));
          else if (stop1reason == 5) { client.print(F("TIMEOUT at ")); client.print(m1_interrupt_ticks); client.print(F(" ticks")); }
          else if (stop1reason == 6) client.print(F("FROZEN DOME Auto-Reverse"));
          else if (!s1_is_physically_open_state && !s1_is_physically_closed_state) client.print(F("Manual Stop"));
          else client.print(F("Unknown"));
      }
      client.println(F("</div></div>"));

      // --- Shutter 2 (West) ---
      client.println(F("<div class='section'><h2>Shutter 2 (West)</h2>"));
      client.println(F("<div class='button-pair'>"));
      client.print(F("<a href='/?$4' class='button b-open'>OPEN S2</a>"));  // $4 = Physically Open Shutter 2
      client.print(F("<a href='/?$3' class='button b-close'>CLOSE S2</a>"));// $3 = Physically Close Shutter 2
      client.println(F("</div>"));
      client.println(F("<div class='status'>"));
      // Status display logic for Shutter 2
      bool s2_is_physically_closed_state = digitalRead(lim2open);
      bool s2_is_physically_open_state = digitalRead(lim2closed);
      client.print(F("<strong>State:</strong> "));
      if (s2_is_physically_closed_state) client.print(F("Physically CLOSED"));
      else if (s2_is_physically_open_state) client.print(F("Physically OPEN"));
      else client.print(F("Intermediate"));
      if (mot2dir == 0 && !s2_is_physically_open_state && !s2_is_physically_closed_state) client.print(F(" (Stopped)"));

      client.print(F("<br><strong>Movement:</strong> "));
      if (mot2dir == OPEN) { client.print(F("Closing (physically)")); if(m2AutoClosedByIP) client.print(F(" (IP Fail)")); if(stop2reason==4) client.print(F(" (VCC Fail)"));}
      else if (mot2dir == CLOSE) { client.print(F("Opening (physically)"));}
      else {client.print(F("Stopped"));}

      if(mot2dir == 0) {
          client.print(F("<br><strong>Stop Reason:</strong> "));
          if (s2_is_physically_open_state && stop2reason==0) client.print(F("Limit 'Phys. Open' (lim2closed)"));
          else if (s2_is_physically_closed_state && stop2reason==0) client.print(F("Limit 'Phys. Closed' (lim2open)"));
          else if (stop2reason == 1) client.print(F("Button/SWSTOP"));
          else if (stop2reason == 2) client.print(F("Web STOP"));
          else if (stop2reason == 3 && m2AutoClosedByIP) client.print(F("IP Fail Auto-Close"));
          else if (stop2reason == 4) client.print(F("VCC Fail Auto-Close"));
          else if (stop2reason == 5) { client.print(F("TIMEOUT at ")); client.print(m2_interrupt_ticks); client.print(F(" ticks")); }
          else if (stop2reason == 6) client.print(F("FROZEN DOME Auto-Reverse"));
          else if (!s2_is_physically_open_state && !s2_is_physically_closed_state) client.print(F("Manual Stop"));
          else client.print(F("Unknown"));
      }
      client.println(F("</div></div>"));

      // Collapsible details section for system/diagnostic info (default: collapsed)
      client.println(F("<details style='margin-bottom:15px;'><summary style='cursor:pointer;padding:10px;background:#f8f8f8;border:1px solid #ddd;border-radius:5px;font-size:0.95em;font-weight:bold;'>System Details (tap to expand)</summary>"));

      // --- Sensors Section (v4.0) ---
      client.println(F("<div class='section'><h2>Sensors</h2><table>"));

      // DS18B20 Temperature
      client.print(F("<tr><td>Temperature</td><td>"));
      if (ds18b20_connected && currentTemp_x10 != -9990) {
        printTempX10(client, currentTemp_x10);
        client.print(F(" C"));
      } else if (!ds18b20_connected) {
        client.print(F("Not connected"));
      } else {
        client.print(F("READ ERROR"));
      }
      client.println(F("</td></tr>"));

      // VL53L0X ToF Distance (always in cm)
      client.print(F("<tr><td>ToF Distance</td><td>"));
      if (tof_connected && tofDistance_mm > 0) {
        client.print(tofDistance_mm / 10);
        client.print(F("."));
        client.print(tofDistance_mm % 10);
        client.print(F(" cm"));
      } else if (!tof_connected) {
        client.print(F("Not connected"));
      } else {
        client.print(F("READ ERROR"));
      }
      client.println(F("</td></tr>"));

      // Timeout mode
      client.print(F("<tr><td>Timeout Mode</td><td>"));
      if (dynamicTimeoutActive) {
        client.print(F("Dynamic ("));
        printTempX10(client, currentTemp_x10);
        client.print(F("C)"));
      } else {
        client.print(F("Static fallback (6527 ticks)"));
      }
      client.println(F("</td></tr>"));

      // Dynamic timeout values for all 4 motor-direction combos
      client.print(F("<tr><td>M1 Close/Open</td><td>"));
      client.print(dynTimeout_M1_Close); client.print(F(" / ")); client.print(dynTimeout_M1_Open);
      client.println(F(" ticks</td></tr>"));
      client.print(F("<tr><td>M2 Close/Open</td><td>"));
      client.print(dynTimeout_M2_Close); client.print(F(" / ")); client.print(dynTimeout_M2_Open);
      client.println(F(" ticks</td></tr>"));

      // Frozen dome state
      client.print(F("<tr><td>Frozen Detection</td><td>"));
      if (!tofCalibrated) {
        client.print(F("Disabled (not calibrated)"));
      } else if (!tof_connected) {
        client.print(F("Disabled (sensor disconnected)"));
      } else {
        switch (frozenDomeState) {
          case FD_IDLE: client.print(F("OK")); break;
          case FD_MONITORING: client.print(F("Monitoring...")); break;
          case FD_FIRST_CHECK: client.print(F("Checking...")); break;
          case FD_GRAVITY_WAIT: client.print(F("FROZEN! Gravity wait...")); break;
          case FD_SECOND_CHECK: client.print(F("Re-checking...")); break;
          case FD_CLOSING: client.print(F("Reversing...")); break;
          case FD_RETRY_WAIT: client.print(F("Retry wait...")); break;
          case FD_LOCKOUT:
            client.print(F("<span class='warning'>LOCKED OUT</span>"));
            break;
        }
        if (frozenRetryCount > 0 && frozenDomeState != FD_IDLE) {
          client.print(F(" (attempt "));
          client.print(frozenRetryCount);
          client.print(F("/"));
          client.print(FROZEN_MAX_RETRIES);
          client.print(F(")"));
        }
      }
      client.println(F("</td></tr>"));

      // ToF calibration baseline
      client.print(F("<tr><td>ToF Baseline</td><td>"));
      if (tofCalibrated) {
        client.print(tofBaseline_mm / 10);
        client.print(F("."));
        client.print(tofBaseline_mm % 10);
        client.print(F(" cm (tolerance: "));
        client.print(TOF_OPEN_TOLERANCE / 10);
        client.print(F("."));
        client.print(TOF_OPEN_TOLERANCE % 10);
        client.print(F(" cm)"));
      } else {
        client.print(F("Not calibrated"));
      }
      client.println(F("</td></tr>"));

      client.println(F("</table>"));

      // Action buttons for sensors
      if (frozenDomeState == FD_LOCKOUT) {
        client.print(F("<a href='/?$U' class='button b-stop fullwidth'>UNLOCK DOME</a>"));
      }
      bool domeFullyClosed = digitalRead(lim1open) && digitalRead(lim2open);
      if (domeFullyClosed && tof_connected && mot1dir == 0 && mot2dir == 0) {
        client.print(F("<a href='/?$C' class='button b-reset fullwidth'>Calibrate ToF Baseline</a>"));
      }
      client.println(F("</div>"));

      // --- IP Monitoring Status ---
      client.println(F("<div class='section'><h2>IP Monitoring</h2><table>"));
      client.print(F("<tr><td>Current IP Fails</td><td>"));
      client.print(connectFailCount);
      client.print(F("/"));
      client.print(maxConnectFails);
      if (firstFailTimestamp > 0) {
        client.print(F(" ("));
        client.print((millis() - firstFailTimestamp) / 60000);
        client.print(F(" min)"));
      }
      client.println(F("</td></tr>"));

      client.print(F("<tr><td>Total IP Failures</td><td class='"));
      if (totalIpFailures > 100) client.print(F("warning"));
      client.print(F("'>"));
      client.print(totalIpFailures);
      client.println(F("</td></tr>"));

      client.print(F("<tr><td>Total Auto-Closes</td><td class='"));
      if (totalAutoCloses > 10) client.print(F("warning"));
      client.print(F("'>"));
      client.print(totalAutoCloses);
      client.println(F("</td></tr>"));

      client.print(F("<tr><td>Days Running</td><td>"));
      client.print(dayCounter);
      client.println(F("</td></tr>"));

      client.print(F("<tr><td>Last Activity</td><td>"));
      client.print((millis() - lastSuccessfulPing) / 1000);
      client.println(F("s ago</td></tr>"));

      client.println(F("</table>"));
      client.print(F("<a href='/?$R' class='button b-reset fullwidth'>Reset Counters</a>"));
      client.println(F("</div>"));

      // --- System Status ---
      client.println(F("<div class='section'><h2>System Status</h2><table>"));
      client.print(F("<tr><th>Sensor</th><th>State</th></tr>"));
      client.print(F("<tr><td>Limit S1 Phys. Closed (Pin "));client.print(lim1open);client.print(F(")</td><td>")); client.print(digitalRead(lim1open) ? F("HIGH (Active)") : F("LOW")); client.println(F("</td></tr>"));
      client.print(F("<tr><td>Limit S1 Phys. Open (Pin "));client.print(lim1closed);client.print(F(")</td><td>")); client.print(digitalRead(lim1closed) ? F("HIGH (Active)") : F("LOW")); client.println(F("</td></tr>"));
      client.print(F("<tr><td>Limit S2 Phys. Closed (Pin "));client.print(lim2open);client.print(F(")</td><td>")); client.print(digitalRead(lim2open) ? F("HIGH (Active)") : F("LOW")); client.println(F("</td></tr>"));
      client.print(F("<tr><td>Limit S2 Phys. Open (Pin "));client.print(lim2closed);client.print(F(")</td><td>")); client.print(digitalRead(lim2closed) ? F("HIGH (Active)") : F("LOW")); client.println(F("</td></tr>"));
      client.print(F("<tr><td>VCC1 (Main Power)</td><td>")); client.print((float)analogRead(VCC1) * 24.0f / 1023.0f * (1023.0f / VCC_RAW_MAX), 1); client.println(F("V</td></tr>"));
      client.print(F("<tr><td>SWSTOP Pressed</td><td>")); client.print(!digitalRead(SWSTOP) ? F("YES") : F("NO")); client.println(F("</td></tr>"));
      client.print(F("<tr><td>Network Status</td><td>")); client.print(ethernet_initialized ? F("OK") : F("ERROR")); client.println(F("</td></tr>"));
      client.println(F("</table></div>"));

      // --- Tick Logging Section ---
      client.println(F("<div class='section'><h2>Motor Runtime Logging</h2>"));

      // Toggle button with status
      client.print(F("<p>Status: <strong>"));
      client.print(tickLoggingEnabled ? F("ENABLED") : F("DISABLED"));
      client.println(F("</strong></p>"));
      client.print(F("<a href='/?$L' class='button "));
      client.print(tickLoggingEnabled ? F("b-stop") : F("b-open"));
      client.print(F(" fullwidth'>"));
      client.print(tickLoggingEnabled ? F("Disable Logging") : F("Enable Logging"));
      client.println(F("</a>"));

      // Current tick counters (live during motor run)
      client.println(F("<h3>Current Run</h3><table>"));
      client.print(F("<tr><td>M1 Running</td><td>"));
      if (mot1dir != 0 && m1_full_run_active) {
        client.print(m1_tick_counter);
        client.print(F(" ticks ("));
        client.print(m1_was_closing ? F("closing") : F("opening"));
        client.print(F(")"));
      } else if (mot1dir != 0) {
        client.print(F("intermediate start"));
      } else {
        client.print(F("-"));
      }
      client.println(F("</td></tr>"));

      client.print(F("<tr><td>M2 Running</td><td>"));
      if (mot2dir != 0 && m2_full_run_active) {
        client.print(m2_tick_counter);
        client.print(F(" ticks ("));
        client.print(m2_was_closing ? F("closing") : F("opening"));
        client.print(F(")"));
      } else if (mot2dir != 0) {
        client.print(F("intermediate start"));
      } else {
        client.print(F("-"));
      }
      client.println(F("</td></tr></table>"));

      // Last valid measurements
      client.println(F("<h3>Last Valid Full-Runs</h3><table>"));
      client.print(F("<tr><th>Motor</th><th>Open&rarr;Close</th><th>Close&rarr;Open</th></tr>"));

      client.print(F("<tr><td>M1 (East)</td><td>"));
      if (m1_last_ticks_closing > 0) {
        client.print(m1_last_ticks_closing);
        client.print(F(" ("));
        client.print((float)m1_last_ticks_closing / 61.0f, 1);
        client.print(F("s)"));
      } else {
        client.print(F("-"));
      }
      client.print(F("</td><td>"));
      if (m1_last_ticks_opening > 0) {
        client.print(m1_last_ticks_opening);
        client.print(F(" ("));
        client.print((float)m1_last_ticks_opening / 61.0f, 1);
        client.print(F("s)"));
      } else {
        client.print(F("-"));
      }
      client.println(F("</td></tr>"));

      client.print(F("<tr><td>M2 (West)</td><td>"));
      if (m2_last_ticks_closing > 0) {
        client.print(m2_last_ticks_closing);
        client.print(F(" ("));
        client.print((float)m2_last_ticks_closing / 61.0f, 1);
        client.print(F("s)"));
      } else {
        client.print(F("-"));
      }
      client.print(F("</td><td>"));
      if (m2_last_ticks_opening > 0) {
        client.print(m2_last_ticks_opening);
        client.print(F(" ("));
        client.print((float)m2_last_ticks_opening / 61.0f, 1);
        client.print(F("s)"));
      } else {
        client.print(F("-"));
      }
      client.println(F("</td></tr></table>"));

      client.print(F("<p style='font-size:0.8em;'>Push target: "));
      client.print(remoteStationIp);
      client.print(F(":"));
      client.print(tickLogServerPort);
      client.println(F("</p></div>"));
      client.println(F("</details>"));  // Close collapsible System Details
  }
  // Persist collapsible section state across auto-refresh using localStorage
  client.println(F("<script>var d=document.querySelector('details');if(d){if(localStorage.getItem('do')==='1')d.open=true;d.addEventListener('toggle',function(){localStorage.setItem('do',d.open?'1':'0');});}</script>"));
  client.println(F("</div></body></html>"));
}

//=============================================================================
// MAIN LOOP - Runs continuously after setup()
//=============================================================================
void loop() {
  wdt_reset();  // Pet the watchdog - must be called within 8 seconds

  if (system_fully_ready) {
    networkWatchdog();          // Check/recover Ethernet
    incrementDayCounter();      // Track uptime
    saveCountersToEEPROM();     // Persist counters if dirty

    #if defined(ENABLE_IP_AUTO_CLOSE)
      checkRemoteConnectionAndAutoClose();  // Safety monitoring
    #endif

    // --- v4.0: Sensor reading and safety checks ---
    readTemperatureAsync();     // Non-blocking DS18B20 temperature reading
    readToFDistance();          // Non-blocking VL53L0X distance reading
    frozenDomeStateMachine();  // Frozen dome detection state machine
    checkConflictingSignals(); // Limit switch vs ToF disagreement

    pushTickDataIfReady();      // Push tick data to logging server if available
    pushInterruptDataIfReady(); // Push interrupted stop data if available
  }

  if (networkMonitoringEnabled) socketHygiene();  // P1: drop dead clients, keep two listeners
  handleWebClient();  // Process any pending HTTP requests
}

//=============================================================================
// TIMER2 INTERRUPT SERVICE ROUTINE
//=============================================================================
/**
 * Called ~61 times per second (every ~16ms).
 * Handles all time-critical motor control operations:
 * - Emergency STOP button monitoring
 * - Limit switch detection
 * - Motor timeout countdown
 * - PWM soft-start ramping
 * - Button debounce and processing
 *
 * IMPORTANT: Keep this ISR as fast as possible to avoid blocking
 * other system operations. Avoid Serial.print in production!
 */
ISR(TIMER2_COMPA_vect) {
  if (!system_fully_ready) {
    return;  // Don't process until setup() is complete
  }

  // Debounce counter for physical buttons
  if (cnt < 100) cnt++;

  //--- EMERGENCY STOP BUTTON ---
  // Highest priority - immediately stops all motors when pressed
  if (!digitalRead(SWSTOP)) {
    if (!swstop_pressed_flag) {
      fdExternalCommand |= FD_CMD_ALL;   // FIX (P1): STOP cancels a frozen-dome cycle even if motors are already off
      if (mot1dir != 0 || mot2dir != 0) {
        mot1dir = 0; mot2dir = 0;
        stop1reason = 1; stop2reason = 1;
        m1AutoClosedByIP = false; m2AutoClosedByIP = false;
        #if defined(SERIAL_DEBUG_BUTTONS)
        Serial.println(F("ISR: SWSTOP - motors stopped."));
        #endif
      }
      swstop_pressed_flag = 1;
    }
  } else {
    swstop_pressed_flag = 0;
  }

  //--- STOP REASON MANAGEMENT ---
  // Clear stop reason when motor is off and at a limit switch
  // (but preserve IP-fail and VCC-fail reasons for display)
  if (mot1dir == 0 && (digitalRead(lim1open) || digitalRead(lim1closed))) {
    if (!((stop1reason == 3 && m1AutoClosedByIP) || stop1reason == 4)) stop1reason = 0;
  }
  if (mot2dir == 0 && (digitalRead(lim2open) || digitalRead(lim2closed))) {
    if (!((stop2reason == 3 && m2AutoClosedByIP) || stop2reason == 4)) stop2reason = 0;
  }

  //--- LIMIT SWITCH DETECTION ---
  // Stop motor immediately when endpoint is reached
  // Note: Switch names are inverted due to wiring (lim1open = physically closed)
  if (digitalRead(lim1open) && mot1dir == OPEN) {
    mot1dir = 0;
    if (!((stop1reason == 3 && m1AutoClosedByIP) || stop1reason == 4)) stop1reason = 0;
    m1AutoClosedByIP = false;
    #if defined(SERIAL_DEBUG_LIMITS)
    Serial.println(F("ISR: M1 reached closed position -> STOP"));
    #endif
  }
  if (digitalRead(lim1closed) && mot1dir == CLOSE) {
    mot1dir = 0; stop1reason = 0; m1AutoClosedByIP = false;
    #if defined(SERIAL_DEBUG_LIMITS)
    Serial.println(F("ISR: M1 reached open position -> STOP"));
    #endif
  }
  if (digitalRead(lim2open) && mot2dir == OPEN) {
    mot2dir = 0;
    if (!((stop2reason == 3 && m2AutoClosedByIP) || stop2reason == 4)) stop2reason = 0;
    m2AutoClosedByIP = false;
    #if defined(SERIAL_DEBUG_LIMITS)
    Serial.println(F("ISR: M2 reached closed position -> STOP"));
    #endif
  }
  if (digitalRead(lim2closed) && mot2dir == CLOSE) {
    mot2dir = 0; stop2reason = 0; m2AutoClosedByIP = false;
    #if defined(SERIAL_DEBUG_LIMITS)
    Serial.println(F("ISR: M2 reached open position -> STOP"));
    #endif
  }

  //--- MOTOR TIMEOUT COUNTDOWN ---
  // Safety feature: stops motor after max time even if limit switch fails
  // Prevents mechanical damage from runaway motor
  if (mot1dir) {
    if (mot1timer) mot1timer--;
    else {
      mot1dir = 0;  // Timeout reached - stop motor
      if (!((stop1reason == 3 && m1AutoClosedByIP) || stop1reason == 4)) {
        stop1reason = 5; m1AutoClosedByIP = false;  // 5 = Timeout
      }
    }
  }
  if (mot2dir) {
    if (mot2timer) mot2timer--;
    else {
      mot2dir = 0;  // Timeout reached - stop motor
      if (!((stop2reason == 3 && m2AutoClosedByIP) || stop2reason == 4)) {
        stop2reason = 5; m2AutoClosedByIP = false;  // 5 = Timeout
      }
    }
  }

  //--- MOTOR PWM OUTPUT ---
  // Soft-start: gradually increase PWM to reduce mechanical stress
  // When motor off: both pins LOW, speed reset to 0
  // When motor on: ramp speed up by SMOOTH each tick until 255
  if (!mot1dir) {
    digitalWrite(motor1a, LOW); digitalWrite(motor1b, LOW); mot1speed = 0;
  } else {
    if (mot1speed < 255 - SMOOTH) mot1speed += SMOOTH; else mot1speed = 255;
  }
  if (!mot2dir) {
    digitalWrite(motor2a, LOW); digitalWrite(motor2b, LOW); mot2speed = 0;
  } else {
    if (mot2speed < 255 - SMOOTH) mot2speed += SMOOTH; else mot2speed = 255;
  }

  // Apply PWM to motor pins based on direction
  if (mot1dir == OPEN)  { digitalWrite(motor1a, LOW); analogWrite(motor1b, mot1speed); }
  if (mot1dir == CLOSE) { digitalWrite(motor1b, LOW); analogWrite(motor1a, mot1speed); }   // P2: release old side first
  if (mot2dir == OPEN)  { digitalWrite(motor2a, LOW); analogWrite(motor2b, mot2speed); }
  if (mot2dir == CLOSE) { digitalWrite(motor2b, LOW); analogWrite(motor2a, mot2speed); }   // P2: release old side first

  //--- PHYSICAL BUTTON HANDLING ---
  // Debounced (cnt > 6 = ~100ms), toggle behavior:
  // - If motor running: stop it
  // - If motor stopped: start in button's direction (if not at limit)
  // - Open buttons (SW1down, SW2down) blocked during frozen lockout
  if (!digitalRead(SW1up) && cnt > 6 && !sw1up_pressed_flag) {
    sw1up_pressed_flag = true; cnt = 0; fdExternalCommand |= FD_CMD_M1;
    if (mot1dir) { mot1dir = 0; stop1reason = 1; m1AutoClosedByIP = false; }
    else if (!digitalRead(lim1open)) {
        mot1dir = OPEN; mot1timer = dynTimeout_M1_Close; stop1reason = 0; m1AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW1up action"));
    #endif
  }
  if (!digitalRead(SW1down) && cnt > 6 && !sw1down_pressed_flag) {
    sw1down_pressed_flag = true; cnt = 0; fdExternalCommand |= FD_CMD_M1;
    if (mot1dir) { mot1dir = 0; stop1reason = 1; m1AutoClosedByIP = false; }
    else if (!fdOpenBlocked() && !digitalRead(lim1closed)) {
        mot1dir = CLOSE; mot1timer = dynTimeout_M1_Open; stop1reason = 0; m1AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW1down action"));
    #endif
  }
  if (!digitalRead(SW2up) && cnt > 6 && !sw2up_pressed_flag) {
    sw2up_pressed_flag = true; cnt = 0; fdExternalCommand |= FD_CMD_M2;
    if (mot2dir) { mot2dir = 0; stop2reason = 1; m2AutoClosedByIP = false; }
    else if (!digitalRead(lim2open)) {
        mot2dir = OPEN; mot2timer = dynTimeout_M2_Close; stop2reason = 0; m2AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW2up action"));
    #endif
  }
  if (!digitalRead(SW2down) && cnt > 6 && !sw2down_pressed_flag) {
    sw2down_pressed_flag = true; cnt = 0; fdExternalCommand |= FD_CMD_M2;
    if (mot2dir) { mot2dir = 0; stop2reason = 1; m2AutoClosedByIP = false; }
    else if (!fdOpenBlocked() && !digitalRead(lim2closed)) {
        mot2dir = CLOSE; mot2timer = dynTimeout_M2_Open; stop2reason = 0; m2AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW2down action"));
    #endif
  }

  //--- BUTTON RELEASE DETECTION ---
  // Clear pressed flag when button is released (allows next press)
  if (digitalRead(SW1up))   { sw1up_pressed_flag = false; }
  if (digitalRead(SW1down)) { sw1down_pressed_flag = false; }
  if (digitalRead(SW2up))   { sw2up_pressed_flag = false; }
  if (digitalRead(SW2down)) { sw2down_pressed_flag = false; }

  //--- FROZEN DOME TICK COUNTER ---
  // Counts ISR ticks when frozen dome check is active (~4 clock cycles overhead)
  if (frozenCheckActive) {
    frozenCheckTicks++;
  }

  //=========================================================================
  // TICK LOGGING - Motor Runtime Measurement (added for temperature analysis)
  //=========================================================================
  // Detects motor state transitions and measures full-run tick counts.
  // Only records valid full-runs: start at one limit, stop at opposite limit.
  // This section is positioned AFTER all motor control decisions are made.

  //--- MOTOR 1 TICK TRACKING ---
  // Detect motor START (transition from stopped to running)
  if (mot1dir != 0 && m1_prev_dir == 0) {
    // Motor just started - check if at a limit switch (valid full-run start)
    bool at_open_limit = digitalRead(lim1closed);   // lim1closed = physically OPEN
    bool at_closed_limit = digitalRead(lim1open);   // lim1open = physically CLOSED

    if (at_open_limit || at_closed_limit) {
      // Started at a limit - this could be a valid full-run
      m1_full_run_active = true;
      m1_tick_counter = 0;
      m1_was_closing = (mot1dir == OPEN);  // OPEN command = physically closing
    } else {
      // Started from intermediate position - not a full-run
      m1_full_run_active = false;
    }
  }

  // Count ticks while motor is running and full-run is active
  if (mot1dir != 0 && m1_full_run_active) {
    m1_tick_counter++;
  }

  // Detect motor STOP (transition from running to stopped)
  if (mot1dir == 0 && m1_prev_dir != 0) {
    // Motor just stopped - check if we completed a valid full-run
    if (m1_full_run_active) {
      bool at_target = false;

      if (m1_was_closing && digitalRead(lim1open)) {
        // Was closing and reached closed limit - VALID
        at_target = true;
        m1_last_ticks_closing = m1_tick_counter;
        m1_last_direction = 1;  // closing
      }
      else if (!m1_was_closing && digitalRead(lim1closed)) {
        // Was opening and reached open limit - VALID
        at_target = true;
        m1_last_ticks_opening = m1_tick_counter;
        m1_last_direction = 2;  // opening
      }

      if (at_target) {
        m1_data_ready = true;  // Signal main loop to push data
      } else {
        // Interrupted stop (manual, web, timeout, emergency) - log it
        m1_interrupt_ticks = m1_tick_counter;
        m1_interrupt_direction = m1_was_closing ? 1 : 2;
        m1_interrupt_ready = true;
      }

      m1_full_run_active = false;
    }
  }

  m1_prev_dir = mot1dir;  // Save for next iteration

  //--- MOTOR 2 TICK TRACKING ---
  // Detect motor START
  if (mot2dir != 0 && m2_prev_dir == 0) {
    bool at_open_limit = digitalRead(lim2closed);   // lim2closed = physically OPEN
    bool at_closed_limit = digitalRead(lim2open);   // lim2open = physically CLOSED

    if (at_open_limit || at_closed_limit) {
      m2_full_run_active = true;
      m2_tick_counter = 0;
      m2_was_closing = (mot2dir == OPEN);
    } else {
      m2_full_run_active = false;
    }
  }

  // Count ticks while motor is running
  if (mot2dir != 0 && m2_full_run_active) {
    m2_tick_counter++;
  }

  // Detect motor STOP
  if (mot2dir == 0 && m2_prev_dir != 0) {
    if (m2_full_run_active) {
      bool at_target = false;

      if (m2_was_closing && digitalRead(lim2open)) {
        at_target = true;
        m2_last_ticks_closing = m2_tick_counter;
        m2_last_direction = 1;
      }
      else if (!m2_was_closing && digitalRead(lim2closed)) {
        at_target = true;
        m2_last_ticks_opening = m2_tick_counter;
        m2_last_direction = 2;
      }

      if (at_target) {
        m2_data_ready = true;
      } else {
        // Interrupted stop (manual, web, timeout, emergency) - log it
        m2_interrupt_ticks = m2_tick_counter;
        m2_interrupt_direction = m2_was_closing ? 1 : 2;
        m2_interrupt_ready = true;
      }

      m2_full_run_active = false;
    }
  }

  m2_prev_dir = mot2dir;
}
