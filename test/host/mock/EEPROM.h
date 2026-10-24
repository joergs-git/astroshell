// EEPROM mock. NOTE: on the AVR an (unsigned) int is 16 bit, on the host 32 bit.
// put()/get() of int-sized values therefore move only 2 bytes so that the
// sketch's EEPROM memory map (uint16 counters at bytes 1-4) behaves as on the MEGA.
#pragma once
#include "Arduino.h"
#include <type_traits>
class EEPROMClass {
 public:
  uint8_t read(int a) { return sim::g_eeprom[a]; }
  void write(int a, uint8_t v) { sim::g_eeprom[a] = v; sim::g_eepromWrites++; }
  void update(int a, uint8_t v) { if (sim::g_eeprom[a] != v) write(a, v); }
  template <typename T> const T& put(int a, const T& v) {
    size_t n = avrSize<T>();
    memcpy(&sim::g_eeprom[a], &v, n);          // little-endian on both targets
    sim::g_eepromWrites += (int)n;
    return v;
  }
  template <typename T> T& get(int a, T& v) {
    size_t n = avrSize<T>();
    v = T();
    memcpy(&v, &sim::g_eeprom[a], n);
    return v;
  }
 private:
  template <typename T> static size_t avrSize() {
    if (std::is_same<T, int>::value || std::is_same<T, unsigned int>::value) return 2;
    return sizeof(T);
  }
};
inline EEPROMClass EEPROM;
