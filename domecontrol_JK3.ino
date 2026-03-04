//=============================================================================
// ASTROSHELL DOME CONTROLLER - SAFETY SENSOR EDITION (v4.0)
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
// Auto-close triggers after 10 failed connection attempts within 15 minutes.
// This ensures dome closes when Cloudwatcher is genuinely unreachable,
// while tolerating brief network glitches (switch hiccups, Solo busy, etc.).
// Timing: 10 checks × 90 second interval = 15 minutes maximum response time.

byte connectFailCount = 0;                          // Current consecutive failures
unsigned long lastConnectAttemptTimestamp = 0;      // Last connection check time
const unsigned long connectCheckInterval = 90000UL; // Check every 90 seconds
const byte maxConnectFails = 10;                    // Failures needed to trigger
unsigned long firstFailTimestamp = 0;               // When failure window started
const unsigned long maxFailTimeWindow = 900000UL;   // 15-minute window for counting

// --- Persistent Statistics (saved to EEPROM) ---
unsigned int totalIpFailures = 0;     // Lifetime IP failure count
unsigned int totalAutoCloses = 0;     // Lifetime auto-close count
byte dayCounter = 0;                  // Days of continuous operation (0-255)
bool eepromDirty = false;             // True if counters need saving

// --- Auto-Close State Tracking ---
bool m1AutoClosedByIP = false;        // True if M1 was auto-closed due to IP fail
bool m2AutoClosedByIP = false;        // True if M2 was auto-closed due to IP fail
bool cableRemovalAutoCloseTriggered = false;  // Prevents repeated auto-close on cable removal

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
const unsigned long ETHERNET_RESET_INTERVAL = 3600000; // Preventive reset every hour
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
void handleWebClient();                         // Process HTTP requests
void sendFullHtmlResponse(EthernetClient& client); // Generate web UI HTML
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
void sendEventNotification(const char* type, const char* detail);  // Push event to Solo

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
    EEPROM.write(EEPROM_ADDR_LAST_FAIL_DAY, dayCounter);
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
    EEPROM.write(EEPROM_ADDR_LAST_FAIL_DAY, dayCounter);  // Immediate save
  }
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
    logClient.setTimeout(2000);  // 2 second timeout

    if (logClient.connect(remoteStationIp, tickLogServerPort)) {
      // Build HTTP GET request with temp and ToF data
      logClient.print(F("GET /log?m=1&d="));
      logClient.print(m1_last_direction);
      logClient.print(F("&t="));
      logClient.print(ticks);
      // Include temperature
      logClient.print(F("&temp="));
      if (currentTemp_x10 != -9990) {
        logClient.print(currentTemp_x10 / 10);
        logClient.print(F("."));
        int frac = currentTemp_x10 % 10;
        if (frac < 0) frac = -frac;
        logClient.print(frac);
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
    logClient.setTimeout(2000);

    if (logClient.connect(remoteStationIp, tickLogServerPort)) {
      logClient.print(F("GET /log?m=2&d="));
      logClient.print(m2_last_direction);
      logClient.print(F("&t="));
      logClient.print(ticks);
      logClient.print(F("&temp="));
      if (currentTemp_x10 != -9990) {
        logClient.print(currentTemp_x10 / 10);
        logClient.print(F("."));
        int frac = currentTemp_x10 % 10;
        if (frac < 0) frac = -frac;
        logClient.print(frac);
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
    logClient.setTimeout(2000);

    if (logClient.connect(remoteStationIp, tickLogServerPort)) {
      logClient.print(F("GET /interrupt?m=1&d="));
      logClient.print(m1_interrupt_direction);
      logClient.print(F("&t="));
      logClient.print(m1_interrupt_ticks);
      logClient.print(F("&temp="));
      if (currentTemp_x10 != -9990) {
        logClient.print(currentTemp_x10 / 10);
        logClient.print(F("."));
        int frac = currentTemp_x10 % 10;
        if (frac < 0) frac = -frac;
        logClient.print(frac);
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
    logClient.setTimeout(2000);

    if (logClient.connect(remoteStationIp, tickLogServerPort)) {
      logClient.print(F("GET /interrupt?m=2&d="));
      logClient.print(m2_interrupt_direction);
      logClient.print(F("&t="));
      logClient.print(m2_interrupt_ticks);
      logClient.print(F("&temp="));
      if (currentTemp_x10 != -9990) {
        logClient.print(currentTemp_x10 / 10);
        logClient.print(F("."));
        int frac = currentTemp_x10 % 10;
        if (frac < 0) frac = -frac;
        logClient.print(frac);
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
        ds18b20_connected = false;
        dynamicTimeoutActive = false;
        dynTimeout_M1_Close = MAX_MOT1_OPEN;
        dynTimeout_M1_Open  = MAX_MOT1_CLOSE;
        dynTimeout_M2_Close = MAX_MOT2_OPEN;
        dynTimeout_M2_Open  = MAX_MOT2_CLOSE;
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
 * Called whenever temperature changes. Values are read atomically by ISR
 * (16-bit reads are atomic on AVR).
 */
void updateDynamicTimeouts() {
  if (currentTemp_x10 == -9990) {
    // Invalid temperature — use static fallback
    dynamicTimeoutActive = false;
    dynTimeout_M1_Close = MAX_MOT1_OPEN;
    dynTimeout_M1_Open  = MAX_MOT1_CLOSE;
    dynTimeout_M2_Close = MAX_MOT2_OPEN;
    dynTimeout_M2_Open  = MAX_MOT2_CLOSE;
    return;
  }

  dynTimeout_M1_Close = computeDynamicTimeout(DYN_M1_CLOSE_BASE, DYN_M1_CLOSE_SLOPE, DYN_M1_CLOSE_MARGIN, currentTemp_x10);
  dynTimeout_M1_Open  = computeDynamicTimeout(DYN_M1_OPEN_BASE, DYN_M1_OPEN_SLOPE, DYN_M1_OPEN_MARGIN, currentTemp_x10);
  dynTimeout_M2_Close = computeDynamicTimeout(DYN_M2_CLOSE_BASE, DYN_M2_CLOSE_SLOPE, DYN_M2_CLOSE_MARGIN, currentTemp_x10);
  dynTimeout_M2_Open  = computeDynamicTimeout(DYN_M2_OPEN_BASE, DYN_M2_OPEN_SLOPE, DYN_M2_OPEN_MARGIN, currentTemp_x10);
  dynamicTimeoutActive = true;
}

//=============================================================================
// VL53L0X TIME-OF-FLIGHT SENSOR FUNCTIONS
//=============================================================================

/**
 * Initialize VL53L0X ToF sensor on I2C (pins 20/21).
 * Sets continuous reading mode for fast non-blocking reads.
 */
void setupVL53L0X() {
  Wire.begin();
  tofSensor.setTimeout(500);

  if (tofSensor.init()) {
    tof_connected = true;
    tofSensor.setMeasurementTimingBudget(200000);  // 200ms for accuracy
    tofSensor.startContinuous();
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
      if (tofSensor.init()) {
        tof_connected = true;
        tofSensor.setMeasurementTimingBudget(200000);
        tofSensor.startContinuous();
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
  // so we'd never see the 0→CLOSE transition if we used the ISR's variables.
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

  switch (frozenDomeState) {

    case FD_IDLE:
      // Watch for motor starting an OPEN command (physically opening = mot*dir==CLOSE)
      // Uses own fd_prev_mot*dir to detect transition (not ISR's m*_prev_dir)
      if (mot1dir == CLOSE && fd_prev_mot1dir == 0) {
        // Motor 1 just started opening — begin monitoring
        frozenDomeState = FD_MONITORING;
        frozenCheckTicks = 0;
        frozenCheckActive = true;
        frozenMotorNum = 1;
      } else if (mot2dir == CLOSE && fd_prev_mot2dir == 0) {
        // Motor 2 just started opening — begin monitoring
        frozenDomeState = FD_MONITORING;
        frozenCheckTicks = 0;
        frozenCheckActive = true;
        frozenMotorNum = 2;
      }
      break;

    case FD_MONITORING:
      // Wait for ~4 seconds of motor running (ISR counts ticks)
      if (frozenCheckTicks >= TOF_CHECK_TICKS) {
        frozenDomeState = FD_FIRST_CHECK;
        frozenCheckActive = false;
      }
      // Abort if motor stopped prematurely (manual stop, limit switch, etc.)
      if ((frozenMotorNum == 1 && mot1dir == 0) || (frozenMotorNum == 2 && mot2dir == 0)) {
        frozenDomeState = FD_IDLE;
        frozenCheckActive = false;
      }
      break;

    case FD_FIRST_CHECK:
      // Check if dome gap is increasing (opening detected)
      if (tofDistance_mm > (int)(tofBaseline_mm + TOF_OPEN_TOLERANCE)) {
        // Opening detected — dome is NOT frozen, all good
        frozenDomeState = FD_IDLE;
        frozenRetryCount = 0;

        // Send clear event if we previously detected a frozen condition
        // (handled elsewhere via event system)
      } else {
        // Dome appears frozen — STOP motor immediately
        if (frozenMotorNum == 1) {
          mot1dir = 0; stop1reason = 6;  // 6 = FROZEN DOME
        } else {
          mot2dir = 0; stop2reason = 6;
        }

        // Enter gravity wait phase
        frozenDomeState = FD_GRAVITY_WAIT;
        frozenStateTimer = now;

        // Send event notification
        char detail[32];
        snprintf(detail, sizeof(detail), "S%d attempt %d/%d", frozenMotorNum, frozenRetryCount + 1, FROZEN_MAX_RETRIES);
        sendEventNotification("frozen_dome", detail);
      }
      break;

    case FD_GRAVITY_WAIT:
      // Motor is off — wait 20 seconds for gravity to separate frozen halves
      if (now - frozenStateTimer >= FROZEN_GRAVITY_WAIT) {
        frozenDomeState = FD_SECOND_CHECK;
      }
      break;

    case FD_SECOND_CHECK:
      // Re-read ToF — did gravity separate the halves?
      if (tofDistance_mm > (int)(tofBaseline_mm + TOF_OPEN_TOLERANCE)) {
        // Gravity worked! Resume opening
        if (frozenMotorNum == 1 && !digitalRead(lim1closed)) {
          mot1dir = CLOSE; mot1timer = dynTimeout_M1_Open; stop1reason = 0;
        } else if (frozenMotorNum == 2 && !digitalRead(lim2closed)) {
          mot2dir = CLOSE; mot2timer = dynTimeout_M2_Open; stop2reason = 0;
        }
        frozenDomeState = FD_IDLE;
        frozenRetryCount = 0;
        sendEventNotification("frozen_clear", "Gravity separated halves");
      } else {
        // Still frozen — reverse motor to close
        if (frozenMotorNum == 1 && !digitalRead(lim1open)) {
          mot1dir = OPEN; mot1timer = dynTimeout_M1_Close; stop1reason = 6;
        } else if (frozenMotorNum == 2 && !digitalRead(lim2open)) {
          mot2dir = OPEN; mot2timer = dynTimeout_M2_Close; stop2reason = 6;
        }
        frozenDomeState = FD_CLOSING;
      }
      break;

    case FD_CLOSING:
      // Wait for close to complete (motor reaches limit or stops)
      if ((frozenMotorNum == 1 && mot1dir == 0) || (frozenMotorNum == 2 && mot2dir == 0)) {
        frozenDomeState = FD_RETRY_WAIT;
        frozenStateTimer = millis();
      }
      break;

    case FD_RETRY_WAIT:
      // Wait 5 seconds before next retry
      if (now - frozenStateTimer >= FROZEN_RETRY_WAIT) {
        frozenRetryCount++;
        if (frozenRetryCount >= FROZEN_MAX_RETRIES) {
          // All retries exhausted — LOCKOUT
          frozenDomeState = FD_LOCKOUT;
          sendEventNotification("frozen_lockout", "3 attempts failed - dome locked");
        } else {
          // Re-issue open command for next attempt
          if (frozenMotorNum == 1 && !digitalRead(lim1closed)) {
            mot1dir = CLOSE; mot1timer = dynTimeout_M1_Open; stop1reason = 0;
          } else if (frozenMotorNum == 2 && !digitalRead(lim2closed)) {
            mot2dir = CLOSE; mot2timer = dynTimeout_M2_Open; stop2reason = 0;
          }
          // Go back to monitoring
          frozenDomeState = FD_MONITORING;
          frozenCheckTicks = 0;
          frozenCheckActive = true;
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
void sendEventNotification(const char* type, const char* detail) {
  // Rate limiting
  unsigned long now = millis();
  if (now - lastEventSentTime < EVENT_MIN_INTERVAL) return;

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
  eventClient.setTimeout(2000);

  if (eventClient.connect(remoteStationIp, tickLogServerPort)) {
    eventClient.print(F("GET /event?type="));
    eventClient.print(type);
    eventClient.print(F("&detail="));
    eventClient.print(encoded);
    eventClient.print(F("&temp="));
    if (currentTemp_x10 != -9990) {
      eventClient.print(currentTemp_x10 / 10);
      eventClient.print(F("."));
      int frac = currentTemp_x10 % 10;
      if (frac < 0) frac = -frac;
      eventClient.print(frac);
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
  for (int attempt = 1; attempt <= 3; attempt++) {
    // Hardware reset via Chip Select pin toggle
    // This recovers from stuck states in the W5100/W5500 chip
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);
    delay(100);
    digitalWrite(10, LOW);
    delay(100);
    digitalWrite(10, HIGH);

    // Initialize Ethernet with static IP (no DHCP for reliability)
    Ethernet.begin(mac, ip);
    delay(5000);  // W5100 needs time to stabilize

    // Verify physical link is connected
    if (Ethernet.linkStatus() == LinkOFF) {
      delay(2000);
      continue;  // Retry
    }

    // Verify IP was assigned correctly
    IPAddress currentIP = Ethernet.localIP();
    if (currentIP != IPAddress(0,0,0,0)) {
      server.begin();
      ethernet_initialized = true;
      lastEthernetReset = millis();
      lastSuccessfulPing = millis();
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
 * - Performs preventive reset every hour for long-term stability
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

  // Periodic health check (network mode only)
  if (currentTime - lastNetworkCheck > NETWORK_CHECK_INTERVAL) {
    lastNetworkCheck = currentTime;

    // --- Network mode: monitor health ---
    // Check if Ethernet cable is still connected
    if (Ethernet.linkStatus() == LinkOFF) {
      ethernet_initialized = false;
      // DON'T try to recover here - let auto-close handle cable removal
      // Calling setupEthernet() would block for ~22 seconds and interfere
      // with the auto-close timing in checkRemoteConnectionAndAutoClose()
      return;
    }

    // Check if IP stack is functional (only if link is up)
    IPAddress currentIP = Ethernet.localIP();
    if (currentIP == IPAddress(0,0,0,0)) {
      // Link is up but IP lost - software issue, try to recover
      ethernet_initialized = false;
      setupEthernet();
    }
  }

  // Preventive hourly reset for long-term stability (only when everything is working)
  if (networkMonitoringEnabled && ethernet_initialized &&
      (currentTime - lastEthernetReset > ETHERNET_RESET_INTERVAL)) {
    setupEthernet();
  }
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
    Serial.println(F("Dome Control v4.0 - Safety Sensors"));
    Serial.println(F("Setup: Serial initialized."));
    if (lim2open == 1 || lim2open == 0 || lim2closed == 1 || lim2closed == 0) {
      Serial.println(F("WARNING: Pins 0/1 used for limit switches! Serial debug may cause malfunctions!"));
    }
  #endif

  // --- Load persistent counters from EEPROM ---
  initializeEEPROM();

  // --- Initialize sensors BEFORE Ethernet (sensors are fast, Ethernet is slow) ---
  // DS18B20: ~10ms init + 800ms first read — temperature ready for first motor command
  setupDS18B20();

  // VL53L0X: ~10ms I2C init — starts continuous measurement mode
  setupVL53L0X();
  loadToFCalibration();

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

  // --- Enable hardware watchdog ---
  // System auto-resets if loop() blocks for more than 8 seconds.
  // This recovers from Ethernet library hangs or other lockups.
  wdt_enable(WDTO_8S);
}

//=============================================================================
// IP MONITORING AND AUTO-CLOSE LOGIC
//=============================================================================
/**
 * Monitors connectivity to Cloudwatcher and triggers auto-close on failure.
 *
 * Safety Logic:
 * - Checks connection every 1 minute when dome is not fully closed
 * - Counts failures within a sliding 5-minute window
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
        
        if (Ethernet.linkStatus() == LinkOFF) {
            linkDown = true;
            networkProblem = true;

            // Cable physically removed - close dome immediately (only once per removal)
            if (!cableRemovalAutoCloseTriggered) {
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F("IP Check: Cable removed - triggering immediate auto-close"));
                #endif

                bool action_taken = false;
                if (mot1dir != OPEN && !digitalRead(lim1open)) {
                    mot1dir = OPEN; mot1timer = dynTimeout_M1_Close;
                    m1AutoClosedByIP = true; stop1reason = 3; action_taken = true;
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F(">>> AUTO-CLOSE: S1 motor started (cable removed)"));
                    #endif
                }
                if (mot2dir != OPEN && !digitalRead(lim2open)) {
                    mot2dir = OPEN; mot2timer = dynTimeout_M2_Close;
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
                cableRemovalAutoCloseTriggered = true;  // Prevent repeated triggering
            }
            return;  // Skip further checks while cable is out
        } else if (Ethernet.linkStatus() == LinkON) {
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
        
        // *** Auto-Recovery: Ethernet reinitialisieren wenn Link wieder da ist ***
        static bool wasLinkDown = false;
        if (wasLinkDown && !linkDown && !needsRecovery && ethernet_initialized) {
            #if defined(SERIAL_DEBUG_IP)
            Serial.println(F("IP Check: Network restored - reinitializing Ethernet"));
            #endif
            setupEthernet(); // Ethernet neu initialisieren
            wasLinkDown = false;
            // Reset fail counters on recovery
            connectFailCount = 0;
            firstFailTimestamp = 0;
        } else if (linkDown || !ethernet_initialized) {
            wasLinkDown = true;
        }
        
        // IP-Recovery falls nötig
        if (needsRecovery && ethernet_initialized) {
            #if defined(SERIAL_DEBUG_IP)
            Serial.println(F("IP Check: Attempting IP recovery"));
            #endif
            setupEthernet();
        }
        
        // Check connection with longer intervals
        if (millis() - lastConnectAttemptTimestamp >= connectCheckInterval) {
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
                netClient.setTimeout(3000);  // 3 second timeout (must be < 8s watchdog)

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
            }
            
            if (connectionOK) {
                // SUCCESS - Reset all fail counters
                connectFailCount = 0;
                firstFailTimestamp = 0; // Reset time window
                
                // STOPPE AUTO-CLOSE BEI ERFOLGREICHER VERBINDUNG
                if (m1AutoClosedByIP && mot1dir == OPEN) {
                    mot1dir = 0; stop1reason = 0; m1AutoClosedByIP = false;
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F("IP Check: M1 Auto-Close STOPPED - Connection restored"));
                    #endif
                }
                if (m2AutoClosedByIP && mot2dir == OPEN) {
                    mot2dir = 0; stop2reason = 0; m2AutoClosedByIP = false;
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
                    Serial.println(F("IP Check: 5-minute window expired - resetting fail count"));
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
            Serial.println(F("IP Check: Max fails within 5 minutes. Triggering auto-close."));
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
                mot1dir = OPEN; mot1timer = dynTimeout_M1_Close;
                m1AutoClosedByIP = true; stop1reason = 3; action_taken = true;
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F(">>> AUTO-CLOSE: S1 motor started (5 failures)"));
                #endif
            }
            if (mot2dir != OPEN && !digitalRead(lim2open)) {
                mot2dir = OPEN; mot2timer = dynTimeout_M2_Close;
                m2AutoClosedByIP = true; stop2reason = 3; action_taken = true;
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F(">>> AUTO-CLOSE: S2 motor started (5 failures)"));
                #endif
            }

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
  unsigned long clientRequestStart = millis();
  bool action_parameter_in_url = false;
  bool plain_text_status_request = false;
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
      if (c == '$') { newInfo = true; }

      if (newInfo && system_fully_ready) { 
        if (c >= '1' && c <= '5') { 
          action_parameter_in_url = true; 
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
          if (frozenDomeState == FD_LOCKOUT) {
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
              mot1dir = OPEN; mot1timer = dynTimeout_M1_Close;
              stop1reason = 0; m1AutoClosedByIP = false;
          }
        } else if (c == '2') {
          // $2 = OPEN Shutter 1 (physically) — check frozen lockout
          if (frozenDomeState == FD_LOCKOUT) {
            // Blocked: dome is locked due to frozen detection
          } else {
            if (mot1dir != 0 && mot1dir != CLOSE) {
                mot1dir = 0; stop1reason = 2; m1AutoClosedByIP = false;
            }
            if (!digitalRead(lim1closed)) {
                mot1dir = CLOSE; mot1timer = dynTimeout_M1_Open;
                stop1reason = 0; m1AutoClosedByIP = false;
            }
          }
        } else if (c == '3') {
          // $3 = CLOSE Shutter 2 (physically)
          if (mot2dir != 0 && mot2dir != OPEN) {
              mot2dir = 0; stop2reason = 2; m2AutoClosedByIP = false;
          }
          if (!digitalRead(lim2open)) {
              mot2dir = OPEN; mot2timer = dynTimeout_M2_Close;
              stop2reason = 0; m2AutoClosedByIP = false;
          }
        } else if (c == '4') {
          // $4 = OPEN Shutter 2 (physically) — check frozen lockout
          if (frozenDomeState == FD_LOCKOUT) {
            // Blocked: dome is locked due to frozen detection
          } else {
            if (mot2dir != 0 && mot2dir != CLOSE) {
                mot2dir = 0; stop2reason = 2; m2AutoClosedByIP = false;
            }
            if (!digitalRead(lim2closed)) {
                mot2dir = CLOSE; mot2timer = dynTimeout_M2_Open;
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
        
        if (c != '$') { newInfo = false; }
      } else if (newInfo && !system_fully_ready) { 
          if (c != '$') newInfo = false; 
          if ((c >= '1' && c <= '5') || c == 'S' || c == 's' || c == 'R' || c == 'r') {
               action_parameter_in_url = true; 
               if (c == 'S' || c == 's') plain_text_status_request = false; 
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
        } else if (action_parameter_in_url) { 
          // FASTEST possible response for motor commands
          client.println(F("HTTP/1.1 303 See Other")); 
          client.println(F("Location: /")); 
          client.println(F("Connection: close")); 
          client.println();
        } else { 
          sendFullHtmlResponse(client);
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
void sendFullHtmlResponse(EthernetClient& client) {
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
  client.println(F(".b-open{background-color:#5cb85c;} .b-close{background-color:#337ab7;} .b-stop{background-color:#dc3545;}"));
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

      // --- Sensors Section (v4.0) ---
      client.println(F("<div class='section'><h2>Sensors</h2><table>"));

      // DS18B20 Temperature
      client.print(F("<tr><td>Temperature</td><td>"));
      if (ds18b20_connected && currentTemp_x10 != -9990) {
        client.print(currentTemp_x10 / 10);
        client.print(F("."));
        int frac = currentTemp_x10 % 10;
        if (frac < 0) frac = -frac;
        client.print(frac);
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
        client.print(currentTemp_x10 / 10);
        client.print(F("."));
        int ft = currentTemp_x10 % 10;
        if (ft < 0) ft = -ft;
        client.print(ft);
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
          client.print(frozenRetryCount + 1);
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
  }
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
  if (mot1dir == CLOSE) { analogWrite(motor1a, mot1speed); digitalWrite(motor1b, LOW); }
  if (mot2dir == OPEN)  { digitalWrite(motor2a, LOW); analogWrite(motor2b, mot2speed); }
  if (mot2dir == CLOSE) { analogWrite(motor2a, mot2speed); digitalWrite(motor2b, LOW); }

  //--- PHYSICAL BUTTON HANDLING ---
  // Debounced (cnt > 6 = ~100ms), toggle behavior:
  // - If motor running: stop it
  // - If motor stopped: start in button's direction (if not at limit)
  // - Open buttons (SW1down, SW2down) blocked during frozen lockout
  if (!digitalRead(SW1up) && cnt > 6 && !sw1up_pressed_flag) {
    sw1up_pressed_flag = true; cnt = 0;
    if (mot1dir) { mot1dir = 0; stop1reason = 1; m1AutoClosedByIP = false; }
    else if (!digitalRead(lim1open)) {
        mot1dir = OPEN; mot1timer = dynTimeout_M1_Close; stop1reason = 0; m1AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW1up action"));
    #endif
  }
  if (!digitalRead(SW1down) && cnt > 6 && !sw1down_pressed_flag) {
    sw1down_pressed_flag = true; cnt = 0;
    if (mot1dir) { mot1dir = 0; stop1reason = 1; m1AutoClosedByIP = false; }
    else if (frozenDomeState != FD_LOCKOUT && !digitalRead(lim1closed)) {
        mot1dir = CLOSE; mot1timer = dynTimeout_M1_Open; stop1reason = 0; m1AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW1down action"));
    #endif
  }
  if (!digitalRead(SW2up) && cnt > 6 && !sw2up_pressed_flag) {
    sw2up_pressed_flag = true; cnt = 0;
    if (mot2dir) { mot2dir = 0; stop2reason = 1; m2AutoClosedByIP = false; }
    else if (!digitalRead(lim2open)) {
        mot2dir = OPEN; mot2timer = dynTimeout_M2_Close; stop2reason = 0; m2AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW2up action"));
    #endif
  }
  if (!digitalRead(SW2down) && cnt > 6 && !sw2down_pressed_flag) {
    sw2down_pressed_flag = true; cnt = 0;
    if (mot2dir) { mot2dir = 0; stop2reason = 1; m2AutoClosedByIP = false; }
    else if (frozenDomeState != FD_LOCKOUT && !digitalRead(lim2closed)) {
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
