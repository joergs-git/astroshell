//=============================================================================
// ASTROSHELL DOME CONTROLLER - STABLE NETWORK VERSION rev9 (v3.3)
//=============================================================================
// Hardware: Arduino MEGA 2560 + Ethernet Shield (W5100/W5500)
// Purpose:  Controls two-shutter astronomical dome with automatic rain protection
// Author:   joergsflow (enhanced from original AstroShell code)
//
// Features:
// - Dual DC motor control with soft-start PWM
// - Web interface for remote control (smartphone/tablet friendly)
// - Automatic dome closure on Cloudwatcher IP failure (safety feature)
// - Hardware watchdog for system recovery
// - EEPROM persistence for failure counters
// - Physical button control with debouncing
// - Motor runtime tick logging for temperature correlation analysis (v3.3)
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

//=============================================================================
// EEPROM MEMORY MAP - Persistent storage across reboots
//=============================================================================
#define EEPROM_MAGIC_BYTE 0x42        // Identifies initialized EEPROM
#define EEPROM_ADDR_MAGIC 0           // Byte 0: Magic byte
#define EEPROM_ADDR_TOTAL_IP_FAILS 1  // Bytes 1-2: Total IP failures (uint16)
#define EEPROM_ADDR_AUTO_CLOSES 3     // Bytes 3-4: Auto-close events (uint16)
#define EEPROM_ADDR_LAST_FAIL_DAY 5   // Byte 5: Uptime day counter

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
// Auto-close triggers after 5 failed connection attempts within 5 minutes.
// This ensures dome closes within 5 minutes of Cloudwatcher becoming unreachable.
// Timing: 5 checks × 1 minute interval = 5 minutes maximum response time.

byte connectFailCount = 0;                          // Current consecutive failures
unsigned long lastConnectAttemptTimestamp = 0;      // Last connection check time
const unsigned long connectCheckInterval = 60000UL; // Check every 1 minute
const byte maxConnectFails = 5;                     // Failures needed to trigger
unsigned long firstFailTimestamp = 0;               // When failure window started
const unsigned long maxFailTimeWindow = 300000UL;   // 5-minute window for counting

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
bool tickLoggingEnabled = false;              // Must be enabled via web UI

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
      // Build minimal HTTP GET request
      logClient.print(F("GET /log?m=1&d="));
      logClient.print(m1_last_direction);
      logClient.print(F("&t="));
      logClient.print(ticks);
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
    Serial.println(F("Dome Control v3.3 - Stable Network"));
    Serial.println(F("Setup: Serial initialized."));
    if (lim2open == 1 || lim2open == 0 || lim2closed == 1 || lim2closed == 0) {
      Serial.println(F("WARNING: Pins 0/1 used for limit switches! Serial debug may cause malfunctions!"));
    }
  #endif

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
                    mot1dir = OPEN; mot1timer = MAX_MOT1_OPEN;
                    m1AutoClosedByIP = true; stop1reason = 3; action_taken = true;
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F(">>> AUTO-CLOSE: S1 motor started (cable removed)"));
                    #endif
                }
                if (mot2dir != OPEN && !digitalRead(lim2open)) {
                    mot2dir = OPEN; mot2timer = MAX_MOT2_OPEN;
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
                mot1dir = OPEN; mot1timer = MAX_MOT1_OPEN;
                m1AutoClosedByIP = true; stop1reason = 3; action_taken = true;
                #if defined(SERIAL_DEBUG_IP)
                Serial.println(F(">>> AUTO-CLOSE: S1 motor started (5 failures)"));
                #endif
            }
            if (mot2dir != OPEN && !digitalRead(lim2open)) {
                mot2dir = OPEN; mot2timer = MAX_MOT2_OPEN;
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

 // IMPROVED WEB LOGIC - Automatic stop before new command
        if (c == '1') {
          // Auto-stop if motor is running in different direction
          if (mot1dir != 0 && mot1dir != OPEN) {
              mot1dir = 0; stop1reason = 2; m1AutoClosedByIP = false;
          }
          // Execute command if limit allows
          if (!digitalRead(lim1open)) {
              mot1dir = OPEN; mot1timer = MAX_MOT1_OPEN;
              stop1reason = 0; m1AutoClosedByIP = false;
          }
        } else if (c == '2') {
          // Auto-stop if motor is running in different direction
          if (mot1dir != 0 && mot1dir != CLOSE) {
              mot1dir = 0; stop1reason = 2; m1AutoClosedByIP = false;
          }
          // Execute command if limit allows
          if (!digitalRead(lim1closed)) {
              mot1dir = CLOSE; mot1timer = MAX_MOT1_CLOSE;
              stop1reason = 0; m1AutoClosedByIP = false;
          }
        } else if (c == '3') {
          // Auto-stop if motor is running in different direction
          if (mot2dir != 0 && mot2dir != OPEN) {
              mot2dir = 0; stop2reason = 2; m2AutoClosedByIP = false;
          }
          // Execute command if limit allows
          if (!digitalRead(lim2open)) {
              mot2dir = OPEN; mot2timer = MAX_MOT2_OPEN;
              stop2reason = 0; m2AutoClosedByIP = false;
          }
        } else if (c == '4') {
          // Auto-stop if motor is running in different direction
          if (mot2dir != 0 && mot2dir != CLOSE) {
              mot2dir = 0; stop2reason = 2; m2AutoClosedByIP = false;
          }
          // Execute command if limit allows
          if (!digitalRead(lim2closed)) {
              mot2dir = CLOSE; mot2timer = MAX_MOT2_CLOSE;
              stop2reason = 0; m2AutoClosedByIP = false;
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
  client.println(F("<!DOCTYPE HTML><html><head><title>AstroShell DomeControl JK3.3</title>"));
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
      client.println(F("<h1>AstroShell Dome Control JK3.3</h1>"));
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
  if (!digitalRead(SW1up) && cnt > 6 && !sw1up_pressed_flag) {
    sw1up_pressed_flag = true; cnt = 0;
    if (mot1dir) { mot1dir = 0; stop1reason = 1; m1AutoClosedByIP = false; }
    else if (!digitalRead(lim1open)) {
        mot1dir = OPEN; mot1timer = MAX_MOT1_OPEN; stop1reason = 0; m1AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW1up action"));
    #endif
  }
  if (!digitalRead(SW1down) && cnt > 6 && !sw1down_pressed_flag) {
    sw1down_pressed_flag = true; cnt = 0;
    if (mot1dir) { mot1dir = 0; stop1reason = 1; m1AutoClosedByIP = false; }
    else if (!digitalRead(lim1closed)) {
        mot1dir = CLOSE; mot1timer = MAX_MOT1_CLOSE; stop1reason = 0; m1AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW1down action"));
    #endif
  }
  if (!digitalRead(SW2up) && cnt > 6 && !sw2up_pressed_flag) {
    sw2up_pressed_flag = true; cnt = 0;
    if (mot2dir) { mot2dir = 0; stop2reason = 1; m2AutoClosedByIP = false; }
    else if (!digitalRead(lim2open)) {
        mot2dir = OPEN; mot2timer = MAX_MOT2_OPEN; stop2reason = 0; m2AutoClosedByIP = false;
    }
    #if defined(SERIAL_DEBUG_BUTTONS)
    Serial.println(F("ISR: SW2up action"));
    #endif
  }
  if (!digitalRead(SW2down) && cnt > 6 && !sw2down_pressed_flag) {
    sw2down_pressed_flag = true; cnt = 0;
    if (mot2dir) { mot2dir = 0; stop2reason = 1; m2AutoClosedByIP = false; }
    else if (!digitalRead(lim2closed)) {
        mot2dir = CLOSE; mot2timer = MAX_MOT2_CLOSE; stop2reason = 0; m2AutoClosedByIP = false;
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
