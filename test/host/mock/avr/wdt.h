#pragma once
#include "../Arduino.h"
#define WDTO_8S 9
inline void wdt_enable(int) { sim::g_wdtEnabled = true; sim::g_lastWdtReset = sim::g_millis; }
inline void wdt_reset() { sim::g_lastWdtReset = sim::g_millis; }
