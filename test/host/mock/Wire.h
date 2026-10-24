#pragma once
#include "Arduino.h"
class TwoWire { public: void begin() {} void end() {} void setWireTimeout(uint32_t, bool) {} };
inline TwoWire Wire;
