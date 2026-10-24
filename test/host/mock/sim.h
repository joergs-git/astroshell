// Host simulation state shared by all Arduino mocks and the test driver.
// The firmware (domecontrol_JK3.ino) is compiled unchanged on the host against
// these mocks; the test driver drives time, pins, network and sensors.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>
#include <functional>

namespace sim {

// --- time ------------------------------------------------------------------
inline unsigned long g_millis = 0;
inline unsigned long g_microsAcc = 0;
inline unsigned long g_lastWdtReset = 0;
inline bool g_wdtEnabled = false;
inline int g_wdtStarved = 0;                 // times the 8 s watchdog window was exceeded

// --- GPIO --------------------------------------------------------------------
constexpr int NPINS = 70;
inline int g_pinIn[NPINS];                   // level returned by digitalRead()
inline int g_pwm[NPINS];                     // last output level (digitalWrite/analogWrite)
inline int g_pinMode[NPINS];

// --- hooks (installed by the test driver) ------------------------------------
inline std::function<void()> g_isrFn;        // the sketch's Timer2 ISR
inline std::function<void()> g_afterTick;    // dome physics, runs after every ISR tick
inline bool g_inIsr = false;

// --- Ethernet ----------------------------------------------------------------
enum LinkState { LINK_UNKNOWN = 0, LINK_ON = 1, LINK_OFF = 2 };
inline int g_link = LINK_ON;
inline bool g_ipLost = false;                // localIP() reports 0.0.0.0
inline std::map<uint16_t, bool> g_reachable = {{80, true}, {88, true}};
struct ConnLog { unsigned long t; uint16_t port; bool ok; };
inline std::vector<ConnLog> g_connLog;       // every outbound connect() attempt
inline std::vector<std::string> g_outbound;  // request text sent over successful outbound connections
inline std::vector<std::string> g_pendingRequests;  // inbound HTTP requests waiting for the sketch
inline std::vector<std::string> g_responses; // raw responses the sketch produced
inline int g_serverBeginCalls = 0;
inline int g_chipResets = 0;                 // W5100.writeMR(0x80) calls
inline int g_eepromWrites = 0;

// --- sensors -----------------------------------------------------------------
inline bool g_dsPresent = false;
inline float g_tempC = 20.0f;
inline bool g_tofPresent = false;
inline int g_tofMm = 100;
inline bool g_tofTimeoutFlag = false;
inline std::function<int()> g_tofFn;         // optional: distance derived from the physics

// --- EEPROM ------------------------------------------------------------------
inline uint8_t g_eeprom[4096];

// --- time advance ------------------------------------------------------------
// One ISR period is ~16.4 ms (61 Hz). Every delay() in the sketch advances time
// in these steps so the ISR and the physics keep running while loop() blocks,
// exactly as on the real board.
inline void tickOnce() {
  g_millis += 16;
  if (g_wdtEnabled && g_millis - g_lastWdtReset > 8000) { g_wdtStarved++; g_lastWdtReset = g_millis; }
  if (g_isrFn && !g_inIsr) { g_inIsr = true; g_isrFn(); g_inIsr = false; }
  if (g_afterTick) g_afterTick();
}
inline void advanceMs(unsigned long ms) {
  static unsigned long rem = 0;
  rem += ms;
  while (rem >= 16) { tickOnce(); rem -= 16; }
}

} // namespace sim
