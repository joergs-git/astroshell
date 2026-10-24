#pragma once
#include "Arduino.h"
#define SPI_ETHERNET_SETTINGS 0
class SPIClass { public: void beginTransaction(int) {} void endTransaction() {} };
inline SPIClass SPI;
