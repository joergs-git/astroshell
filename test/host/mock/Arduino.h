// Minimal Arduino core mock for host-side logic tests of domecontrol_JK3.ino
#pragma once
#include "sim.h"

typedef bool boolean;
typedef uint8_t byte;
typedef uint16_t word;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 54
#define A1 55
#define A2 56
#define A3 57
#define A4 58
#define A5 59

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))

// AVR register / interrupt surface used by the sketch
inline uint8_t SREG = 0;
#define cli() ((void)0)
#define sei() ((void)0)
#define ISR(vector) void vector()
inline uint8_t OCR2A = 0, TCCR2A = 0, TCCR2B = 0, TIMSK2 = 0;
#define WGM21 1
#define CS22 2
#define CS21 1
#define CS20 0
#define OCIE2A 1

inline unsigned long millis() { return sim::g_millis; }
inline void delay(unsigned long ms) { sim::advanceMs(ms); }
inline void delayMicroseconds(unsigned int us) {
  sim::g_microsAcc += us;
  while (sim::g_microsAcc >= 1000) { sim::g_microsAcc -= 1000; sim::advanceMs(1); }
}
inline void pinMode(uint8_t pin, uint8_t mode) { sim::g_pinMode[pin] = mode; }
inline void digitalWrite(uint8_t pin, uint8_t v) { sim::g_pwm[pin] = v ? 255 : 0; }
inline void analogWrite(uint8_t pin, int v) { sim::g_pwm[pin] = v; }
inline int digitalRead(uint8_t pin) { return sim::g_pinIn[pin] ? HIGH : LOW; }
inline int analogRead(uint8_t) { return 580; }   // VCC_RAW_MAX = 24 V

class IPAddress {
 public:
  uint8_t b[4];
  IPAddress() { b[0] = b[1] = b[2] = b[3] = 0; }
  IPAddress(uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3) { b[0] = a0; b[1] = a1; b[2] = a2; b[3] = a3; }
  bool operator==(const IPAddress& o) const { return memcmp(b, o.b, 4) == 0; }
  bool operator!=(const IPAddress& o) const { return !(*this == o); }
  uint8_t operator[](int i) const { return b[i]; }
  std::string str() const { char s[20]; snprintf(s, sizeof s, "%u.%u.%u.%u", b[0], b[1], b[2], b[3]); return s; }
};

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t* buf, size_t n) { size_t k = 0; for (size_t i = 0; i < n; i++) k += write(buf[i]); return k; }
  size_t print(const char* s) { size_t k = 0; while (*s) k += write((uint8_t)*s++); return k; }
  size_t print(const __FlashStringHelper* s) { return print(reinterpret_cast<const char*>(s)); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char v) { return printU((unsigned long)v); }
  size_t print(int v) { return printS((long)v); }
  size_t print(unsigned int v) { return printU((unsigned long)v); }
  size_t print(long v) { return printS(v); }
  size_t print(unsigned long v) { return printU(v); }
  size_t print(double v, int digits = 2) { char s[48]; snprintf(s, sizeof s, "%.*f", digits, v); return print(s); }
  size_t print(const IPAddress& ip) { return print(ip.str().c_str()); }
  size_t println() { return print("\r\n"); }
  template <typename T> size_t println(T v) { size_t n = print(v); return n + println(); }
  size_t println(double v, int digits) { size_t n = print(v, digits); return n + println(); }
 private:
  size_t printU(unsigned long v) { char s[24]; snprintf(s, sizeof s, "%lu", v); return print(s); }
  size_t printS(long v) { char s[24]; snprintf(s, sizeof s, "%ld", v); return print(s); }
};
