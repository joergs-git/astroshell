//OPTIMIZED ARDUINO DOME DRIVER BY ASTROSHELL - STABLE NETWORK VERSION rev6
// --- Configuration Defines ---
#define SMOOTH 30 
#define MAX_MOT1_OPEN  6344
#define MAX_MOT1_CLOSE 6344
#define MAX_MOT1_VCC_CLOSE_ACTION MAX_MOT1_OPEN
#define MAX_MOT2_OPEN  6344
#define MAX_MOT2_CLOSE 6344
#define MAX_MOT2_VCC_CLOSE_ACTION MAX_MOT2_OPEN 

#define IP_ADR0 192
#define IP_ADR1 168
#define IP_ADR2 1
#define IP_ADR3 177

#define VCC_RAW_MAX 580 

// --- Libraries ---
#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>

// --- Automatic Feature Control ---
#define ENABLE_IP_AUTO_CLOSE       
// #define ENABLE_VCC_FAIL_AUTO_CLOSE  // DEACTIVATED for better performance

// --- Motor Direction Commands ---
#define OPEN        1 // Command to physically CLOSE the dome shutter
#define CLOSE       2 // Command to physically OPEN the dome shutter
                      // If 0, motor is switched off

// --- Hardware Pin Definitions (UNCHANGED) ---
#define motor1a     6 
#define motor1b     9 
#define motor2a     5 
#define motor2b     3 

#define lim1open    7 // Limit switch pin for "shutter 1 physically closed" endpoint (active HIGH)
#define lim1closed  2 // Limit switch pin for "shutter 1 physically open" endpoint (active HIGH)
#define lim2open    1 // Limit switch pin for "shutter 2 physically closed" endpoint (Arduino Pin 1, TX, active HIGH)
#define lim2closed  0 // Limit switch pin for "shutter 2 physically open" endpoint (Arduino Pin 0, RX, active HIGH)

#define SW1up       A3 
#define SW1down     A2 
#define SW2up       A5 
#define SW2down     A4 
#define SWSTOP      8  

#define VCC1        A1 
#define VCC2        A0 

// --- EEPROM Addresses for persistent counters ---
#define EEPROM_MAGIC_BYTE 0x42  // Magic byte to detect first run
#define EEPROM_ADDR_MAGIC 0     // Address for magic byte
#define EEPROM_ADDR_TOTAL_IP_FAILS 1  // Address for total IP failures (2 bytes)
#define EEPROM_ADDR_AUTO_CLOSES 3     // Address for auto-close count (2 bytes)
#define EEPROM_ADDR_LAST_FAIL_DAY 5   // Address for day counter (1 byte)

// --- Global Variables ---
boolean newInfo, sw1up_pressed_flag, sw1down_pressed_flag, sw2up_pressed_flag, sw2down_pressed_flag;
byte cnt = 0, mot1dir = 0, mot2dir = 0, mot1speed = 0, mot2speed = 0;
byte stop1reason = 0, stop2reason = 0; // 0:Limit/Timeout, 1:Button/SWSTOP, 2:Web-Stop, 3:IP-Fail, 4:VCC-Fail
byte vccerr = 0, swstop_pressed_flag = 0;
word mot1timer = 0, mot2timer = 0;

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(IP_ADR0, IP_ADR1, IP_ADR2, IP_ADR3);
EthernetServer server(80);

IPAddress remoteStationIp(192, 168, 1, 151);
const int remoteStationPort = 80;

// --- Improved IP Monitoring with longer timeframes ---
byte connectFailCount = 0;
unsigned long lastConnectAttemptTimestamp = 0;
const unsigned long connectCheckInterval = 180000UL; // 3 minutes between checks (was 1 minute)
const byte maxConnectFails = 5; // 5 fails needed (was 3)
unsigned long firstFailTimestamp = 0; // Track when first fail occurred
const unsigned long maxFailTimeWindow = 500000UL; // 8 minutes window for fails

// Persistent counters
unsigned int totalIpFailures = 0;     // Total IP failures since installation
unsigned int totalAutoCloses = 0;     // Total auto-closes triggered
byte dayCounter = 0;                  // Simple day counter (0-255)

bool m1AutoClosedByIP = false; 
bool m2AutoClosedByIP = false; 

// --- Network Stability Variables ---
unsigned long lastNetworkCheck = 0;
const unsigned long NETWORK_CHECK_INTERVAL = 600000; // 10 minutes (was 3 minutes)
bool ethernet_initialized = false;
unsigned long lastEthernetReset = 0;
const unsigned long ETHERNET_RESET_INTERVAL = 3600000; // 60 minutes (was 1 minute)
unsigned long lastSuccessfulPing = 0;

// ISR Optimization Variables - but preserve original logic
volatile bool isr_button_events = false;
volatile bool isr_limit_events = false; 
volatile bool isr_timer_events = false;
volatile byte isr_button_states = 0; // Bitfield for button states

// --- Debug Switches (ALL DISABLED for stability) ---
// #define SERIAL_DEBUG_GENERAL 
// #define SERIAL_DEBUG_IP      
// #define SERIAL_DEBUG_BUTTONS 
// #define SERIAL_DEBUG_LIMITS  
// #define SERIAL_DEBUG_EEPROM

volatile boolean system_fully_ready = false;

// --- Function Prototypes ---
void setupEthernet();
void networkWatchdog();
void handleWebClient();
void sendFullHtmlResponse(EthernetClient& client);
void checkRemoteConnectionAndAutoClose();
void handleStatusUpdates();
void initializeEEPROM();
void saveCountersToEEPROM();
void incrementDayCounter();

// --- EEPROM Functions ---
void initializeEEPROM() {
  // Check if EEPROM has been initialized
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC_BYTE) {
    #if defined(SERIAL_DEBUG_EEPROM)
    Serial.println(F("EEPROM: First run - initializing"));
    #endif
    
    // First run - initialize EEPROM
    EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_BYTE);
    EEPROM.put(EEPROM_ADDR_TOTAL_IP_FAILS, (unsigned int)0);
    EEPROM.put(EEPROM_ADDR_AUTO_CLOSES, (unsigned int)0);
    EEPROM.write(EEPROM_ADDR_LAST_FAIL_DAY, 0);
  } else {
    // Load existing values
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

void saveCountersToEEPROM() {
  static unsigned long lastSave = 0;
  
  // Only save every 5 minutes to reduce EEPROM wear
  if (millis() - lastSave > 300000UL) {
    EEPROM.put(EEPROM_ADDR_TOTAL_IP_FAILS, totalIpFailures);
    EEPROM.put(EEPROM_ADDR_AUTO_CLOSES, totalAutoCloses);
    EEPROM.write(EEPROM_ADDR_LAST_FAIL_DAY, dayCounter);
    lastSave = millis();
    
    #if defined(SERIAL_DEBUG_EEPROM)
    Serial.println(F("EEPROM: Counters saved"));
    #endif
  }
}

void incrementDayCounter() {
  static unsigned long lastDayIncrement = 0;
  
  // Simple day counter - increment every 24 hours
  if (millis() - lastDayIncrement > 86400000UL) { // 24 hours
    dayCounter++;
    if (dayCounter > 250) dayCounter = 0; // Wrap around before 255
    lastDayIncrement = millis();
    
    // Save immediately when day changes
    EEPROM.write(EEPROM_ADDR_LAST_FAIL_DAY, dayCounter);
  }
}

// --- Improved Ethernet Initialization ---
void setupEthernet() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    
    // W5100/W5500 Reset via CS pin manipulation (Pin 10 for standard Ethernet Shield)
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH); // CS high
    delay(100);
    digitalWrite(10, LOW);  // CS low  
    delay(100);
    digitalWrite(10, HIGH); // CS high again for normal communication
    
    Ethernet.begin(mac, ip);
    delay(5000); // Longer delay for stabilization
    
    // Check hardware link
    if (Ethernet.linkStatus() == LinkOFF) {
      delay(2000);
      continue;
    }
    
    // Validate IP address
    IPAddress currentIP = Ethernet.localIP();
    if (currentIP != IPAddress(0,0,0,0)) {
      server.begin();
      ethernet_initialized = true;
      lastEthernetReset = millis();
      lastSuccessfulPing = millis();
      return; // Successfully initialized
    }
    
    delay(2000);
  }
  
  // Fallback: Start server anyway
  server.begin();
  ethernet_initialized = false;
}

// --- Network Watchdog ---
void networkWatchdog() {
  unsigned long currentTime = millis();
  
  // Regular network checks - less frequent
  if (currentTime - lastNetworkCheck > NETWORK_CHECK_INTERVAL) {
    lastNetworkCheck = currentTime;
    
    // Check link status
    if (Ethernet.linkStatus() == LinkOFF) {
      ethernet_initialized = false;
    }
    
    // Check IP status
    IPAddress currentIP = Ethernet.localIP();
    if (currentIP == IPAddress(0,0,0,0)) {
      ethernet_initialized = false;
    }
    
    // Reinitialize on problems
    if (!ethernet_initialized) {
      setupEthernet();
    }
  }
  
  // Preventive reset every hour
  if (currentTime - lastEthernetReset > ETHERNET_RESET_INTERVAL) {
    setupEthernet();
  }
}

void handleStatusUpdates() {
  if (!isr_limit_events && !isr_timer_events) return;
  isr_limit_events = false;
  isr_timer_events = false;
  
  // ORIGINAL stop reason updates for limit switches
  if (mot1dir == 0 && (digitalRead(lim1open) || digitalRead(lim1closed))) {
    if (! ((stop1reason == 3 && m1AutoClosedByIP) || stop1reason == 4) ) stop1reason = 0; 
  }
  if (mot2dir == 0 && (digitalRead(lim2open) || digitalRead(lim2closed))) {
    if (! ((stop2reason == 3 && m2AutoClosedByIP) || stop2reason == 4) ) stop2reason = 0;
  }
}

void setup() {
  #if defined(SERIAL_DEBUG_GENERAL) || defined(SERIAL_DEBUG_IP) || defined(SERIAL_DEBUG_BUTTONS) || defined(SERIAL_DEBUG_LIMITS) || defined(SERIAL_DEBUG_EEPROM)
    Serial.begin(115200);
    unsigned long setupSerialStart = millis();
    while(!Serial && (millis() - setupSerialStart < 2000)) { delay(10); } 
    Serial.println(F("------------------------------"));
    Serial.println(F("Dome Control v3.1 - Stable Network"));
    Serial.println(F("Setup: Serial initialized."));
    if (lim2open == 1 || lim2open == 0 || lim2closed == 1 || lim2closed == 0) {
      Serial.println(F("WARNING: Pins 0/1 used for limit switches! Serial debug may cause malfunctions!"));
    }
  #endif

  // Initialize EEPROM and load counters
  initializeEEPROM();

  // Original initialization UNCHANGED
  mot1dir = 0; mot2dir = 0;
  stop1reason = 0; stop2reason = 0;
  m1AutoClosedByIP = false; m2AutoClosedByIP = false;
  vccerr = 0;  
  cnt = 0;      
  connectFailCount = 0;
  firstFailTimestamp = 0;

  // ISR Optimization Flags initialization
  isr_button_events = false;
  isr_limit_events = false;
  isr_timer_events = false;
  isr_button_states = 0;

  sw1up_pressed_flag = false; sw1down_pressed_flag = false;
  sw2up_pressed_flag = false; sw2down_pressed_flag = false;
  swstop_pressed_flag = 0;

  pinMode(motor1a, OUTPUT); pinMode(motor1b, OUTPUT);
  pinMode(motor2a, OUTPUT); pinMode(motor2b, OUTPUT);
  pinMode(lim1open, INPUT_PULLUP); pinMode(lim1closed, INPUT_PULLUP);
  pinMode(lim2open, INPUT_PULLUP); pinMode(lim2closed, INPUT_PULLUP);
  pinMode(SW1up, INPUT_PULLUP); pinMode(SW1down, INPUT_PULLUP);
  pinMode(SW2up, INPUT_PULLUP); pinMode(SW2down, INPUT_PULLUP);
  pinMode(SWSTOP, INPUT_PULLUP);

  // CS pin for Ethernet Shield configuration
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH); // CS pin high for SPI communication

  #if defined(SERIAL_DEBUG_GENERAL)
    Serial.println(F("Setup: Pin modes set. Initializing Ethernet..."));
  #endif

  delay(500); 

  // IMPROVED Ethernet initialization
  setupEthernet();

  #if defined(SERIAL_DEBUG_GENERAL)
  Serial.println(F("Setup: Web server started."));
  #endif

  // Original timer configuration UNCHANGED
  cli(); 
  OCR2A = 255; TCCR2A |= (1 << WGM21); 
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); 
  TIMSK2 |= (1 << OCIE2A); 
  sei(); 
  
  #if defined(SERIAL_DEBUG_GENERAL)
  Serial.println(F("Setup: Timer ISR configured."));
  Serial.println(F("Setup: Complete. ISRs will be fully processed now."));
  #endif
  system_fully_ready = true; 
}

// IMPROVED checkRemoteConnectionAndAutoClose with time window
void checkRemoteConnectionAndAutoClose() {
    // Network problems are now handled with same 15-minute rule as connection failures
    bool networkProblem = false;
    
    if (!ethernet_initialized) {
        networkProblem = true;
        #if defined(SERIAL_DEBUG_IP)
        Serial.println(F("IP Check: Ethernet not initialized - will check time window"));
        #endif
    }
    
    bool domeIsNotFullyClosed;
    if (!digitalRead(lim1open) || !digitalRead(lim2open)) { 
        domeIsNotFullyClosed = true;
    } else if (mot1dir == CLOSE || mot2dir == CLOSE) { 
        domeIsNotFullyClosed = true;
    } else { 
        domeIsNotFullyClosed = false; 
    }

    if (domeIsNotFullyClosed) {
        // *** Check for network problems ***
        bool linkDown = false;
        bool needsRecovery = false;
        
        if (Ethernet.linkStatus() == LinkOFF) {
            linkDown = true;
            networkProblem = true;
            #if defined(SERIAL_DEBUG_IP)
            Serial.println(F("IP Check: Ethernet cable disconnected"));
            #endif
        } else if (Ethernet.linkStatus() == LinkON) {
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
                #if defined(SERIAL_DEBUG_IP)
                Serial.print(F("IP Check: Connecting to "));Serial.print(remoteStationIp);Serial.println(F("..."));
                #endif
                
                unsigned long connectStart = millis();
                bool connected = netClient.connect(remoteStationIp, remoteStationPort);
                
                if (connected && (millis() - connectStart < 5000)) { // Longer timeout
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
                
                // Check if we're still within the 15-minute window
                if (millis() - firstFailTimestamp > maxFailTimeWindow) {
                    // Time window expired - reset counters
                    #if defined(SERIAL_DEBUG_IP)
                    Serial.println(F("IP Check: 8-minute window expired - resetting fail count"));
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
        
        // Only trigger auto-close if we have enough failures within the time window
        if (connectFailCount >= maxConnectFails && 
            (millis() - firstFailTimestamp <= maxFailTimeWindow)) {
            #if defined(SERIAL_DEBUG_IP)
            Serial.println(F("IP Check: Max fails within 8 minutes. Triggering auto-close."));
            if (networkProblem) {
                if (!ethernet_initialized) Serial.println(F("Reason: Ethernet stack problem"));
                else if (linkDown) Serial.println(F("Reason: Cable disconnected"));
                else if (needsRecovery) Serial.println(F("Reason: IP lost"));
            } else {
                Serial.println(F("Reason: Target IP unreachable"));
            }
            #endif
            
            bool action_taken = false;
            if (mot1dir != OPEN && !digitalRead(lim1open)) { 
                mot1dir = OPEN; mot1timer = MAX_MOT1_OPEN; 
                m1AutoClosedByIP = true; stop1reason = 3; action_taken = true;
            }
            if (mot2dir != OPEN && !digitalRead(lim2open)) { 
                mot2dir = OPEN; mot2timer = MAX_MOT2_OPEN; 
                m2AutoClosedByIP = true; stop2reason = 3; action_taken = true;
            }
            
            if (action_taken) {
                totalAutoCloses++; // Increment persistent counter
                saveCountersToEEPROM();
            }
            
            connectFailCount = 0; 
            firstFailTimestamp = 0; // Reset time window
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

void handleWebClient() {
  EthernetClient client = server.available();
  if (!client) return;
  
  lastSuccessfulPing = millis(); // Update activity
  
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
        
 // IMPROVED WEB LOGIC - Automatic stop before new command
        if (c == '1') { 
          // Auto-stop if motor is running in different direction
          if (mot1dir != 0 && mot1dir != OPEN) {
              mot1dir = 0; stop1reason = 2; m1AutoClosedByIP = false;
              delay(50); // Very short pause for clean stop
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
              delay(50); // Very short pause for clean stop
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
              delay(50); // Very short pause for clean stop
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
              delay(50); // Very short pause for clean stop
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

void sendFullHtmlResponse(EthernetClient& client) {
  // ORIGINAL HTML layout preserved with persistent counters added
  client.println(F("HTTP/1.1 200 OK")); client.println(F("Content-Type: text/html; charset=utf-8"));
  client.println(F("Connection: close"));
  if (system_fully_ready) client.println(F("Refresh: 10")); 
  else client.println(F("Refresh: 2")); 
  client.println();
  client.println(F("<!DOCTYPE HTML><html><head><title>AstroShell DomeControl JK3</title>"));
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
      client.println(F("<h1>AstroShell Dome Control JK3</h1>"));
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
          else if (!s1_is_physically_open_state && !s1_is_physically_closed_state) client.print(F("User/Timeout"));
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
          else if (!s2_is_physically_open_state && !s2_is_physically_closed_state) client.print(F("User/Timeout"));
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
  } 
  client.println(F("</div></body></html>"));
}

void loop() {
  if (system_fully_ready) { 
    networkWatchdog(); // Monitor network
    handleStatusUpdates();    // Status updates from ISR - ORIGINAL status logic
    incrementDayCounter();     // Update day counter
    saveCountersToEEPROM();   // Save counters periodically
    
    #if defined(ENABLE_IP_AUTO_CLOSE)
      checkRemoteConnectionAndAutoClose();
    #endif
  }

  handleWebClient(); // Improved client handling with faster timeouts
}

// LIGHTWEIGHT ISR - Remove VCC monitoring to fix Ethernet timeouts
ISR(TIMER2_COMPA_vect) {
  if (!system_fully_ready) { 
    return;
  }

  if (cnt < 100) cnt++;

  // ORIGINAL SWSTOP LOGIC - UNCHANGED
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
  
  // ORIGINAL STOP REASON LOGIC - UNCHANGED
  if (mot1dir == 0 && (digitalRead(lim1open) || digitalRead(lim1closed))) {
    if (! ((stop1reason == 3 && m1AutoClosedByIP) || stop1reason == 4) ) stop1reason = 0; 
  }
  if (mot2dir == 0 && (digitalRead(lim2open) || digitalRead(lim2closed))) {
    if (! ((stop2reason == 3 && m2AutoClosedByIP) || stop2reason == 4) ) stop2reason = 0;
  }
  
  // ORIGINAL LIMIT SWITCH LOGIC - UNCHANGED
  if (digitalRead(lim1open) && mot1dir == OPEN) { 
    mot1dir = 0; if (!((stop1reason == 3 && m1AutoClosedByIP) || stop1reason == 4)) stop1reason = 0; m1AutoClosedByIP = false; 
    #if defined(SERIAL_DEBUG_LIMITS)
    Serial.println(F("ISR: M1 Phys. Closing (OPEN cmd), lim1open ACTIVE (HIGH) -> STOP M1"));
    #endif
  }
  if (digitalRead(lim1closed) && mot1dir == CLOSE) { 
    mot1dir = 0; stop1reason = 0; m1AutoClosedByIP = false; 
    #if defined(SERIAL_DEBUG_LIMITS)
    Serial.println(F("ISR: M1 Phys. Opening (CLOSE cmd), lim1closed ACTIVE (HIGH) -> STOP M1"));
    #endif
  } 
  if (digitalRead(lim2open) && mot2dir == OPEN) { 
    mot2dir = 0; if (!((stop2reason == 3 && m2AutoClosedByIP) || stop2reason == 4)) stop2reason = 0; m2AutoClosedByIP = false; 
    #if defined(SERIAL_DEBUG_LIMITS)
    Serial.println(F("ISR: M2 Phys. Closing (OPEN cmd), lim2open ACTIVE (HIGH) -> STOP M2"));
    #endif
  }
  if (digitalRead(lim2closed) && mot2dir == CLOSE) { 
    mot2dir = 0; stop2reason = 0; m2AutoClosedByIP = false; 
    #if defined(SERIAL_DEBUG_LIMITS)
    Serial.println(F("ISR: M2 Phys. Opening (CLOSE cmd), lim2closed ACTIVE (HIGH) -> STOP M2"));
    #endif
  } 

  // VCC MONITORING REMOVED FROM ISR - This was causing the 5-second ping timeouts!

  // ORIGINAL TIMER LOGIC - UNCHANGED
  if (mot1dir) { if (mot1timer) mot1timer--; else { mot1dir = 0; if (!((stop1reason == 3 && m1AutoClosedByIP) || stop1reason == 4) ) { stop1reason = 0; m1AutoClosedByIP = false; } } }
  if (mot2dir) { if (mot2timer) mot2timer--; else { mot2dir = 0; if (!((stop2reason == 3 && m2AutoClosedByIP) || stop2reason == 4) ) { stop2reason = 0; m2AutoClosedByIP = false; } } }

  // ORIGINAL MOTOR CONTROL LOGIC - UNCHANGED
  if (!mot1dir) { digitalWrite(motor1a, LOW); digitalWrite(motor1b, LOW); mot1speed = 0; } 
  else { if (mot1speed < 255 - SMOOTH) mot1speed += SMOOTH; else mot1speed = 255; }
  if (!mot2dir) { digitalWrite(motor2a, LOW); digitalWrite(motor2b, LOW); mot2speed = 0; } 
  else { if (mot2speed < 255 - SMOOTH) mot2speed += SMOOTH; else mot2speed = 255; }

  if (mot1dir == OPEN) { digitalWrite(motor1a, LOW); analogWrite(motor1b, mot1speed); }
  if (mot1dir == CLOSE) { analogWrite(motor1a, mot1speed); digitalWrite(motor1b, LOW); }
  if (mot2dir == OPEN) { digitalWrite(motor2a, LOW); analogWrite(motor2b, mot2speed); }
  if (mot2dir == CLOSE) { analogWrite(motor2a, mot2speed); digitalWrite(motor2b, LOW); }

  // ORIGINAL BUTTON LOGIC - EXACTLY AS IN YOUR ORIGINAL CODE - UNCHANGED
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

  // ORIGINAL BUTTON RELEASE LOGIC - UNCHANGED
  if (digitalRead(SW1up)) { sw1up_pressed_flag = false; }
  if (digitalRead(SW1down)) { sw1down_pressed_flag = false; }
  if (digitalRead(SW2up)) { sw2up_pressed_flag = false; }
  if (digitalRead(SW2down)) { sw2down_pressed_flag = false; }
}
