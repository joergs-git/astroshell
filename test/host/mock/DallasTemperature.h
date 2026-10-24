#pragma once
#include "Arduino.h"
#include "OneWire.h"
#define DEVICE_DISCONNECTED_C -127.0f
class DallasTemperature {
 public:
  explicit DallasTemperature(OneWire*) {}
  void begin() {}
  uint8_t getDeviceCount() { return sim::g_dsPresent ? 1 : 0; }
  void setResolution(uint8_t) {}
  void setWaitForConversion(bool) {}
  void requestTemperatures() {}
  float getTempCByIndex(uint8_t) { return sim::g_dsPresent ? sim::g_tempC : DEVICE_DISCONNECTED_C; }
};
