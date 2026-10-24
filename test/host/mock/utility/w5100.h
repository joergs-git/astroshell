#pragma once
#include "../Ethernet.h"
class SnSR { public: static const uint8_t CLOSED = 0x00, LISTEN = 0x14, ESTABLISHED = 0x17; };
class W5100Class { public: void writeMR(uint8_t) { sim::g_chipResets++; } uint8_t readMR() { return 0; } };
inline W5100Class W5100;
