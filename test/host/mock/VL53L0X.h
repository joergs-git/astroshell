#pragma once
#include "Arduino.h"
class VL53L0X {
 public:
  void setTimeout(uint16_t) {}
  bool init() { return sim::g_tofPresent; }
  void setMeasurementTimingBudget(uint32_t) {}
  void startContinuous() {}
  uint16_t readRangeContinuousMillimeters() {
    if (!sim::g_tofPresent) { sim::g_tofTimeoutFlag = true; return 65535; }
    sim::g_tofTimeoutFlag = false;
    return (uint16_t)(sim::g_tofFn ? sim::g_tofFn() : sim::g_tofMm);
  }
  bool timeoutOccurred() { return sim::g_tofTimeoutFlag; }
};
