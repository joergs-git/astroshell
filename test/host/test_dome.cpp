//=============================================================================
// AstroShell dome controller - host-side logic tests
//=============================================================================
// The firmware is included unchanged and compiled against the mocks in mock/.
// A small dome physics model turns the PWM pins the firmware drives into shutter
// movement and limit-switch levels. The physical truth of each site (which PWM
// pin opens, which input pin means "physically closed") is written down here
// independently of the firmware, so the tests verify what the dome DOES, not
// which internal constant the code happens to use.
//
// Build: make -C test/host test   (runs both site profiles)
//=============================================================================
#include <string>
#include <vector>
#include <functional>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

#include <Arduino.h>
#ifndef FIRMWARE_PATH
#define FIRMWARE_PATH "../../domecontrol_JK3.ino"
#endif
#include FIRMWARE_PATH

// Compatibility with firmware revisions before v4.3.0 (make test FW=<old.ino>): one raw sample decided
// a limit switch, one 6527-tick timeout served every motor and direction, the Solo was always .151.
#ifndef LIM_MIN_RUN_TICKS_CLOSE                       // pre-4.3.3: one window for both directions, no limit fault, no override
#define PRE_V433 1
#ifndef LIM_MIN_RUN_TICKS
#define LIM_MIN_RUN_TICKS 0                           // pre-4.3.1: no far-end plausibility
#define PRE_V432 1                                    // pre-4.3.2: no limGlitch* diagnostics
#endif
#define LIM_MIN_RUN_TICKS_CLOSE LIM_MIN_RUN_TICKS
#define LIM_MIN_RUN_TICKS_OPEN  LIM_MIN_RUN_TICKS
#endif
#ifndef LIM_CONFIRM_TICKS
#define LIM_CONFIRM_TICKS 1
#define limRead(pin) digitalRead(pin)                 // pre-4.3.0: raw reads, no confirmation, no reversal hold
static volatile byte m1LimitHold = 0, m2LimitHold = 0;
#endif
#ifndef STATIC_TIMEOUT_M1_CLOSE
#define STATIC_TIMEOUT_M1_CLOSE 6527
#define STATIC_TIMEOUT_M1_OPEN  6527
#define STATIC_TIMEOUT_M2_CLOSE 6527
#define STATIC_TIMEOUT_M2_OPEN  6527
#endif
#ifndef HAVE_TIMEOUT_REGRESSION
#define HAVE_TIMEOUT_REGRESSION 1
#endif
#ifndef SOLO_IP_ADR3
#define SOLO_IP_ADR3 151
#endif

#ifndef SITE
#define SITE 1
#endif

//-----------------------------------------------------------------------------
// Site truth (hardware facts, independent of the firmware)
//-----------------------------------------------------------------------------
// Board connectors: motor1a = 6, motor1b = 9, motor2a = 5, motor2b = 3,
// limit inputs 7 ("lim1open"), 2 ("lim1closed"), 1 ("lim2open"), 0 ("lim2closed"),
// buttons A3 (SW1up), A2 (SW1down), A5 (SW2up), A4 (SW2down), 8 (STOP).
struct SiteTruth {
  const char* name;
  int openPwm1, closePwm1, physOpen1, physClosed1;   // shutter 1
  int openPwm2, closePwm2, physOpen2, physClosed2;   // shutter 2
  int btnOpen1, btnClose1, btnOpen2, btnClose2;      // which panel button physically opens/closes
};
// Germany: mirrored installation. motor*b (direction 1) physically CLOSES, pin 7/1 = physically closed.
static const SiteTruth GERMANY  = {"GERMANY (mirrored)",  6, 9, 2, 7,  5, 3, 0, 1,  A2, A3, A4, A5};
// Portugal: original AstroShell design. motor*b (direction 1) physically OPENS, pin 7/1 = physically open
// (nocapacitor2023.ino: command '1' -> OPEN -> stops at lim1open which the original displays as "Opened").
static const SiteTruth PORTUGAL = {"PORTUGAL (standard)", 9, 6, 7, 2,  3, 5, 1, 0,  A3, A2, A5, A4};
static const SiteTruth& T = (SITE == 2) ? PORTUGAL : GERMANY;

static const char* pinName(int p) {
  switch (p) { case 7: return "lim1open"; case 2: return "lim1closed"; case 1: return "lim2open"; case 0: return "lim2closed"; }
  return "?";
}

//-----------------------------------------------------------------------------
// Dome physics
//-----------------------------------------------------------------------------
static int g_travel = 5000;             // ISR ticks for a full run (~82 s at 61 Hz)
struct Shutter {
  int pos = 0;                          // 0 = physically closed .. g_travel = physically open
  int openPwm = 0, closePwm = 0, physOpenPin = 0, physClosedPin = 0;
  bool stuckOpening = false;            // frozen: cannot move towards open
  bool closedLimitBroken = false, openLimitBroken = false;
  int closedGlitchTicks = 0, openGlitchTicks = 0;   // spurious HIGH samples injected on a limit input (v4.3.0)
  bool closedStuck = false, openStuck = false;      // line open: the input reads HIGH wherever the shutter is (v4.3.3)
  long ticksMovingOpen = 0, ticksMovingClosed = 0;
};
static Shutter S[2];
static int g_hbridgeShorts = 0;

static void physicsStep() {
  for (int i = 0; i < 2; i++) {
    Shutter& s = S[i];
    bool o = sim::g_pwm[s.openPwm] > 0, c = sim::g_pwm[s.closePwm] > 0;
    if (o && c) g_hbridgeShorts++;
    else if (o) { s.ticksMovingOpen++; if (!s.stuckOpening && s.pos < g_travel) s.pos++; }
    else if (c) { s.ticksMovingClosed++; if (s.pos > 0) s.pos--; }
    sim::g_pinIn[s.physOpenPin]   = (s.pos >= g_travel && !s.openLimitBroken) ? HIGH : LOW;
    sim::g_pinIn[s.physClosedPin] = (s.pos <= 0 && !s.closedLimitBroken) ? HIGH : LOW;
    if (s.openGlitchTicks > 0)   { sim::g_pinIn[s.physOpenPin]   = HIGH; s.openGlitchTicks--; }     // electrical glitch on the line
    if (s.closedGlitchTicks > 0) { sim::g_pinIn[s.physClosedPin] = HIGH; s.closedGlitchTicks--; }
    if (s.openStuck)   sim::g_pinIn[s.physOpenPin]   = HIGH;
    if (s.closedStuck) sim::g_pinIn[s.physClosedPin] = HIGH;
  }
}

//-----------------------------------------------------------------------------
// Harness
//-----------------------------------------------------------------------------
static int g_fails = 0;
#define CHECK(cond) do { if (!(cond)) { g_fails++; printf("    FAIL %d: %s\n", __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { long _a = (long)(a), _b = (long)(b); if (_a != _b) { g_fails++; printf("    FAIL %d: %s == %s  (%ld != %ld)\n", __LINE__, #a, #b, _a, _b); } } while (0)
#define CHECK_STREQ(a, b) do { std::string _a = (a), _b = (b); if (_a != _b) { g_fails++; printf("    FAIL %d: %s == \"%s\"  (got \"%s\")\n", __LINE__, #a, _b.c_str(), _a.c_str()); } } while (0)
#define CHECK_CONTAINS(str, sub) do { std::string _s = (str); if (_s.find(sub) == std::string::npos) { g_fails++; printf("    FAIL %d: missing \"%s\"\n", __LINE__, std::string(sub).c_str()); } } while (0)
#define CHECK_NOT_CONTAINS(str, sub) do { std::string _s = (str); if (_s.find(sub) != std::string::npos) { g_fails++; printf("    FAIL %d: unexpected \"%s\"\n", __LINE__, std::string(sub).c_str()); } } while (0)

static void resetSim() {
  sim::g_millis = 0; sim::g_microsAcc = 0; sim::g_lastWdtReset = 0; sim::g_wdtEnabled = false; sim::g_wdtStarved = 0;
  for (int i = 0; i < sim::NPINS; i++) { sim::g_pinIn[i] = HIGH; sim::g_pwm[i] = 0; sim::g_pinMode[i] = 0; }
  sim::g_link = sim::LINK_ON; sim::g_ipLost = false;
  sim::g_reachable = {{80, true}, {88, true}};
  sim::g_connLog.clear(); sim::g_outbound.clear(); sim::g_pendingRequests.clear(); sim::g_responses.clear();
  sim::g_serverBeginCalls = 0; sim::g_chipResets = 0; sim::g_eepromWrites = 0;
  sim::g_dsPresent = false; sim::g_tempC = 20.0f; sim::g_tofPresent = false; sim::g_tofMm = 100; sim::g_tofFn = nullptr;
  memset(sim::g_eeprom, 0xFF, sizeof(sim::g_eeprom));
  for (int i = 0; i < 64; i++) sim::g_conns[i] = sim::Conn();
  g_travel = 5000; g_hbridgeShorts = 0;
  S[0] = Shutter(); S[1] = Shutter();
  S[0].openPwm = T.openPwm1; S[0].closePwm = T.closePwm1; S[0].physOpenPin = T.physOpen1; S[0].physClosedPin = T.physClosed1;
  S[1].openPwm = T.openPwm2; S[1].closePwm = T.closePwm2; S[1].physOpenPin = T.physOpen2; S[1].physClosedPin = T.physClosed2;
  S[0].pos = 2500; S[1].pos = 2500;
  sim::g_isrFn = [] { TIMER2_COMPA_vect(); };
  sim::g_afterTick = physicsStep;
  physicsStep();
}
static void setPos(int p1, int p2) { S[0].pos = p1; S[1].pos = p2; physicsStep(); }
static void boot() { setup(); }

static void runTicks(int n) { for (int i = 0; i < n; i++) { sim::tickOnce(); loop(); } }
static void runMs(unsigned long ms) { unsigned long end = sim::g_millis + ms; while (sim::g_millis < end) { sim::tickOnce(); loop(); } }

static std::string http(const std::string& path) {
  sim::g_pendingRequests.push_back("GET " + path + " HTTP/1.1\r\nHost: dome\r\nConnection: close\r\n\r\n");
  size_t before = sim::g_responses.size();
  for (int i = 0; i < 50 && sim::g_responses.size() == before; i++) { sim::tickOnce(); loop(); }
  if (sim::g_responses.size() == before) return "<<NO RESPONSE>>";
  return sim::g_responses.back();
}
static std::string bodyOf(const std::string& r) { size_t p = r.find("\r\n\r\n"); return p == std::string::npos ? r : r.substr(p + 4); }
static std::string trimmed(std::string s) { while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ')) s.pop_back(); return s; }
static std::string ascom() { return trimmed(bodyOf(http("/?$A"))); }
static std::string status() { return trimmed(bodyOf(http("/?$S"))); }
static std::string page() { return bodyOf(http("/")); }
static std::string section(const std::string& pg, const char* from, const char* to) {
  size_t a = pg.find(from); if (a == std::string::npos) return "";
  size_t b = pg.find(to, a); return pg.substr(a, b == std::string::npos ? std::string::npos : b - a);
}
static std::string s1(const std::string& pg) { return section(pg, "Shutter 1 (East)", "Shutter 2 (West)"); }
static std::string s2(const std::string& pg) { return section(pg, "Shutter 2 (West)", "System Details"); }

static bool physClosed(int i) { return S[i].pos <= 0; }
static bool physOpen(int i) { return S[i].pos >= g_travel; }
static bool motorOff(int i) { return i == 0 ? mot1dir == 0 : mot2dir == 0; }
static bool movingToClosed(int i) { return sim::g_pwm[S[i].closePwm] > 0 && sim::g_pwm[S[i].openPwm] == 0; }
static bool movingToOpen(int i) { return sim::g_pwm[S[i].openPwm] > 0 && sim::g_pwm[S[i].closePwm] == 0; }
static bool idle(int i) { return sim::g_pwm[S[i].openPwm] == 0 && sim::g_pwm[S[i].closePwm] == 0; }
static int connects(uint16_t port, bool okOnly = false, bool failOnly = false) {
  int n = 0; for (auto& c : sim::g_connLog) if (c.port == port && (!okOnly || c.ok) && (!failOnly || !c.ok)) n++; return n;
}
static bool outboundHas(const char* sub) { for (auto& r : sim::g_outbound) if (r.find(sub) != std::string::npos) return true; return false; }
static int outboundCount(const char* sub) { int n = 0; for (auto& r : sim::g_outbound) if (r.find(sub) != std::string::npos) n++; return n; }
static void forceClose(int shutter) { http(shutter == 1 ? "/?$F1" : "/?$F3"); http(shutter == 1 ? "/?$G1" : "/?$G3"); }   // v4.3.3: $F arms, $G for the same shutter starts
static void pressButton(int pin) { sim::g_pinIn[pin] = LOW; runTicks(2); sim::g_pinIn[pin] = HIGH; runTicks(8); }   // ISR debounce: cnt > 6 ticks
static void seedEepromInitialized() {   // EEPROM as left behind by an earlier firmware: magic + zero counters
  sim::g_eeprom[0] = 0x42; for (int a = 1; a <= 5; a++) sim::g_eeprom[a] = 0;
}
static void seedTofCalibration(uint16_t baseline) { sim::g_eeprom[6] = baseline & 0xFF; sim::g_eeprom[7] = baseline >> 8; sim::g_eeprom[8] = 0xA5; }
static void tofFromPhysics() {   // gap between the halves grows with the opening shutter
  sim::g_tofPresent = true;
  sim::g_tofFn = [] { int p = S[0].pos > S[1].pos ? S[0].pos : S[1].pos; return 100 + p / 5; };
}
#ifdef LIMIT_WIRING_SWAPPED   // only the dual-site firmware has the $W toggle
static void ensureRemoteCheck(bool on) { if (remoteCheckEnabled != on) http("/?$W"); }
#else
static void ensureRemoteCheck(bool) {}
#endif

struct Scenario { const char* name; void (*fn)(); };
static std::vector<Scenario>& registry() { static std::vector<Scenario> r; return r; }
struct Reg { Reg(const char* n, void (*f)()) { registry().push_back({n, f}); } };
#define SCENARIO(id, name) static void id(); static Reg reg_##id(name, id); static void id()

//=============================================================================
// SCENARIOS - web commands and physical outcome
//=============================================================================
SCENARIO(close_s1_from_open, "$1 closes shutter 1 physically and reports it") {
  setPos(g_travel, 2500); boot();
  http("/?$1"); runTicks(5);
  CHECK(movingToClosed(0));
  CHECK_CONTAINS(ascom(), "INTERMEDIATE|CLOSING|INTERMEDIATE|STOPPED");
  CHECK_CONTAINS(s1(page()), "Closing (physically)");
  runMs(120000);
  CHECK(physClosed(0)); CHECK(motorOff(0)); CHECK(idle(0)); CHECK_EQ(stop1reason, 0);
  CHECK_CONTAINS(ascom(), "CLOSED|STOPPED|INTERMEDIATE|STOPPED");
  std::string p = page();
  CHECK_CONTAINS(s1(p), "Physically CLOSED");
  CHECK_CONTAINS(s1(p), std::string("Limit 'Phys. Closed' (") + pinName(T.physClosed1) + ")");
  CHECK_CONTAINS(p, std::string("Limit S1 Phys. Closed (Pin ") + std::to_string(T.physClosed1) + ")</td><td>HIGH (Active)");
  CHECK_CONTAINS(p, std::string("Limit S1 Phys. Open (Pin ") + std::to_string(T.physOpen1) + ")</td><td>LOW");
}

SCENARIO(open_s1_from_closed, "$2 opens shutter 1 physically and reports it") {
  setPos(0, 2500); boot();
  http("/?$2"); runTicks(5);
  CHECK(movingToOpen(0));
  CHECK_CONTAINS(ascom(), "INTERMEDIATE|OPENING|");
  CHECK_CONTAINS(s1(page()), "Opening (physically)");
  runMs(120000);
  CHECK(physOpen(0)); CHECK(motorOff(0)); CHECK_EQ(stop1reason, 0);
  CHECK_CONTAINS(ascom(), "OPEN|STOPPED|INTERMEDIATE|STOPPED");
  std::string p = page();
  CHECK_CONTAINS(s1(p), "Physically OPEN");
  CHECK_CONTAINS(s1(p), std::string("Limit 'Phys. Open' (") + pinName(T.physOpen1) + ")");
  CHECK_CONTAINS(p, std::string("Limit S1 Phys. Open (Pin ") + std::to_string(T.physOpen1) + ")</td><td>HIGH (Active)");
}

SCENARIO(close_open_s2, "$3/$4 close and open shutter 2 physically") {
  setPos(2500, g_travel); boot();
  http("/?$3"); runTicks(5);
  CHECK(movingToClosed(1)); CHECK(idle(0));
  CHECK_CONTAINS(ascom(), "INTERMEDIATE|STOPPED|INTERMEDIATE|CLOSING");
  runMs(120000);
  CHECK(physClosed(1)); CHECK(motorOff(1));
  CHECK_CONTAINS(ascom(), "INTERMEDIATE|STOPPED|CLOSED|STOPPED");
  std::string p = page();
  CHECK_CONTAINS(s2(p), "Physically CLOSED");
  CHECK_CONTAINS(s2(p), std::string("Limit 'Phys. Closed' (") + pinName(T.physClosed2) + ")");
  CHECK_CONTAINS(p, std::string("Limit S2 Phys. Closed (Pin ") + std::to_string(T.physClosed2) + ")</td><td>HIGH (Active)");
  http("/?$4"); runTicks(5);
  CHECK(movingToOpen(1));
  CHECK_CONTAINS(ascom(), "INTERMEDIATE|STOPPED|INTERMEDIATE|OPENING");
  runMs(120000);
  CHECK(physOpen(1)); CHECK(motorOff(1));
  CHECK_CONTAINS(ascom(), "INTERMEDIATE|STOPPED|OPEN|STOPPED");
  CHECK_CONTAINS(s2(page()), "Physically OPEN");
}

SCENARIO(commands_at_limits_are_ignored, "close when closed / open when open does not start a motor") {
  setPos(0, g_travel); boot();
  http("/?$1"); http("/?$4"); runTicks(20);
  CHECK(motorOff(0)); CHECK(motorOff(1)); CHECK(idle(0)); CHECK(idle(1));
  CHECK_EQ(S[0].pos, 0); CHECK_EQ(S[1].pos, g_travel);
  CHECK_CONTAINS(ascom(), "CLOSED|STOPPED|OPEN|STOPPED");
}

SCENARIO(reverse_while_closing, "$2 during a close reverses with a fresh soft-start") {
  setPos(g_travel, 2500); boot();
  http("/?$1"); runTicks(100);
  CHECK(movingToClosed(0)); CHECK_EQ(mot1speed, 255);
  int posBefore = S[0].pos;
  http("/?$2");
  CHECK(mot1speed < 255);            // ramp restarted on direction change
  runTicks(20);
  CHECK(movingToOpen(0)); CHECK(S[0].pos > posBefore - 2);
  runMs(120000);
  CHECK(physOpen(0)); CHECK(motorOff(0));
}

SCENARIO(stop_all, "$5 stops both motors with stop reason 'Web STOP'") {
  setPos(2500, 2500); boot();
  http("/?$1$4"); runTicks(30);
  CHECK(movingToClosed(0)); CHECK(movingToOpen(1));
  http("/?$5"); runTicks(2);
  CHECK(motorOff(0)); CHECK(motorOff(1)); CHECK(idle(0)); CHECK(idle(1));
  CHECK_EQ(stop1reason, 2); CHECK_EQ(stop2reason, 2);
  std::string p = page();
  CHECK_CONTAINS(s1(p), "Web STOP"); CHECK_CONTAINS(s2(p), "Web STOP");
  CHECK_CONTAINS(ascom(), "INTERMEDIATE|STOPPED|INTERMEDIATE|STOPPED");
}

SCENARIO(close_all_open_all, "CLOSE ALL / OPEN ALL buttons move both shutters") {
  setPos(2500, 2500); boot();
  http("/?$1$3"); runTicks(5);
  CHECK(movingToClosed(0)); CHECK(movingToClosed(1));
  runMs(120000);
  CHECK(physClosed(0)); CHECK(physClosed(1));
  CHECK_STREQ(status(), "CLOSED");
  http("/?$2$4"); runTicks(5);
  CHECK(movingToOpen(0)); CHECK(movingToOpen(1));
  runMs(120000);
  CHECK(physOpen(0)); CHECK(physOpen(1));
  CHECK_STREQ(status(), "OPEN");
  CHECK_CONTAINS(ascom(), "OPEN|STOPPED|OPEN|STOPPED");
}

SCENARIO(plain_status, "$S is CLOSED only when both shutters are physically closed") {
  setPos(0, 0); boot();
  CHECK_STREQ(status(), "CLOSED");
  setPos(0, 1); runTicks(LIM_CONFIRM_TICKS); CHECK_STREQ(status(), "OPEN");    // v4.3.0: levels are confirmed over LIM_CONFIRM_TICKS
  setPos(1, 0); runTicks(LIM_CONFIRM_TICKS); CHECK_STREQ(status(), "OPEN");
  setPos(g_travel, g_travel); runTicks(LIM_CONFIRM_TICKS); CHECK_STREQ(status(), "OPEN");
}

//=============================================================================
// SCENARIOS - panel buttons and STOP
//=============================================================================
SCENARIO(panel_buttons, "panel buttons: toggle start/stop, physical direction per site") {
  setPos(2500, 2500); boot();
  pressButton(T.btnClose1); CHECK(movingToClosed(0));
  pressButton(T.btnClose1); CHECK(motorOff(0)); CHECK_EQ(stop1reason, 1);
  CHECK_CONTAINS(s1(page()), "Button/SWSTOP");
  pressButton(T.btnOpen1);  CHECK(movingToOpen(0));
  pressButton(T.btnOpen1);  CHECK(motorOff(0));
  pressButton(T.btnClose2); CHECK(movingToClosed(1));
  pressButton(T.btnClose2); CHECK(motorOff(1));
  pressButton(T.btnOpen2);  CHECK(movingToOpen(1));
  runMs(120000);
  CHECK(physOpen(1)); CHECK(motorOff(1));
}

SCENARIO(emergency_stop, "STOP button halts both motors immediately") {
  setPos(2500, 2500); boot();
  http("/?$2$3"); runTicks(30);
  CHECK(movingToOpen(0)); CHECK(movingToClosed(1));
  sim::g_pinIn[8] = LOW; runTicks(1);
  CHECK(motorOff(0)); CHECK(motorOff(1)); CHECK(idle(0)); CHECK(idle(1));
  CHECK_EQ(stop1reason, 1); CHECK_EQ(stop2reason, 1);
  sim::g_pinIn[8] = HIGH; runTicks(5);
  CHECK(motorOff(0)); CHECK(motorOff(1));
}

//=============================================================================
// SCENARIOS - timeouts
//=============================================================================
#if ENABLE_DS18B20_DYNAMIC_TIMEOUT && !HAVE_TIMEOUT_REGRESSION
SCENARIO(probe_without_regression, "DS18B20 built in on a site without a regression: temperature shown, timeouts stay at the static site values") {
  sim::g_dsPresent = true; sim::g_tempC = 10.0f;
  setPos(2500, 2500); boot();
  CHECK(ds18b20_connected); CHECK_EQ(currentTemp_x10, 100); CHECK(!dynamicTimeoutActive);
  CHECK_EQ(dynTimeout_M1_Close, STATIC_TIMEOUT_M1_CLOSE); CHECK_EQ(dynTimeout_M1_Open, STATIC_TIMEOUT_M1_OPEN);
  CHECK_EQ(dynTimeout_M2_Close, STATIC_TIMEOUT_M2_CLOSE); CHECK_EQ(dynTimeout_M2_Open, STATIC_TIMEOUT_M2_OPEN);
  std::string p = page();
  CHECK_CONTAINS(p, "Temperature</td><td>10.0 C");
  CHECK_CONTAINS(p, "Timeout Mode</td><td>Static (site profile)");
  http("/?$3"); CHECK(movingToClosed(1)); CHECK(mot2timer > STATIC_TIMEOUT_M2_CLOSE - 20 && mot2timer <= STATIC_TIMEOUT_M2_CLOSE);
  http("/?$5");
  sim::g_tempC = 12.0f;                                  // a CHANGED reading drives updateDynamicTimeouts() on the periodic path
  runMs(30000); CHECK(!dynamicTimeoutActive); CHECK_EQ(currentTemp_x10, 120);
  CHECK_EQ(dynTimeout_M2_Close, STATIC_TIMEOUT_M2_CLOSE);
}
#elif ENABLE_DS18B20_DYNAMIC_TIMEOUT
SCENARIO(dynamic_timeout_follows_physical_direction, "closing uses the Close regression, opening the Open regression (DS18B20 at 10 C)") {
  sim::g_dsPresent = true; sim::g_tempC = 10.0f;
  setPos(2500, 2500); boot();
  CHECK(dynamicTimeoutActive);
  CHECK_EQ(dynTimeout_M1_Close, 6503); CHECK_EQ(dynTimeout_M1_Open, 6041);
  CHECK_EQ(dynTimeout_M2_Close, 6102); CHECK_EQ(dynTimeout_M2_Open, 5607);
  http("/?$1"); CHECK(movingToClosed(0)); CHECK(mot1timer > 6490 && mot1timer <= 6503);
  http("/?$5");
  http("/?$2"); CHECK(movingToOpen(0));   CHECK(mot1timer > 6028 && mot1timer <= 6041);
  http("/?$5");
  http("/?$3"); CHECK(movingToClosed(1)); CHECK(mot2timer > 6089 && mot2timer <= 6102);
  http("/?$5");
  http("/?$4"); CHECK(movingToOpen(1));   CHECK(mot2timer > 5594 && mot2timer <= 5607);
  http("/?$5");
  pressButton(T.btnClose1); CHECK(movingToClosed(0)); CHECK(mot1timer > 6490 && mot1timer <= 6503);
  pressButton(T.btnClose1);
  pressButton(T.btnOpen1);  CHECK(movingToOpen(0));   CHECK(mot1timer > 6028 && mot1timer <= 6041);
  pressButton(T.btnOpen1);
  pressButton(T.btnClose2); CHECK(movingToClosed(1)); CHECK(mot2timer > 6089 && mot2timer <= 6102);
  pressButton(T.btnClose2);
  pressButton(T.btnOpen2);  CHECK(movingToOpen(1));   CHECK(mot2timer > 5594 && mot2timer <= 5607);
}

#else
SCENARIO(temp_layer_compiled_out, "DS18B20 layer compiled out: a present probe is never used, static site timeouts, UI says so, pushes carry temp=-999") {
  sim::g_dsPresent = true; sim::g_tempC = 10.0f;     // the mock would answer, the firmware must not ask
  setPos(g_travel, 2500); boot(); ensureRemoteCheck(true);
  CHECK(!ds18b20_connected); CHECK_EQ(currentTemp_x10, -9990); CHECK(!dynamicTimeoutActive);
  CHECK_EQ(dynTimeout_M1_Close, STATIC_TIMEOUT_M1_CLOSE); CHECK_EQ(dynTimeout_M1_Open, STATIC_TIMEOUT_M1_OPEN);
  CHECK_EQ(dynTimeout_M2_Close, STATIC_TIMEOUT_M2_CLOSE); CHECK_EQ(dynTimeout_M2_Open, STATIC_TIMEOUT_M2_OPEN);
  std::string p = page();
  CHECK_CONTAINS(p, "Temperature</td><td>Not installed");
  CHECK_CONTAINS(p, "Timeout Mode</td><td>Static (site profile)");
  http("/?$1"); CHECK(mot1timer > STATIC_TIMEOUT_M1_CLOSE - 20 && mot1timer <= STATIC_TIMEOUT_M1_CLOSE);
  runMs(120000); CHECK(physClosed(0));
  CHECK(outboundHas("GET /log?m=1&d=1&t=50")); CHECK(outboundHas("&temp=-999&tof=-1"));
  runMs(60000); CHECK(!dynamicTimeoutActive); CHECK(!ds18b20_connected);
}
#endif

SCENARIO(static_timeout_without_sensor, "no DS18B20: static site timeout, broken limit switch stops the motor with reason 5") {
  setPos(g_travel, 2500); boot();
  CHECK(!dynamicTimeoutActive);
  CHECK_CONTAINS(page(), "Static (site profile)");
  S[0].closedLimitBroken = true;
  http("/?$1"); CHECK(mot1timer > STATIC_TIMEOUT_M1_CLOSE - 20 && mot1timer <= STATIC_TIMEOUT_M1_CLOSE);
  runMs(100000);
  CHECK(physClosed(0)); CHECK(movingToClosed(0));     // arrived, but the switch never reports it: motor still on
  runMs(35000);
  CHECK(motorOff(0)); CHECK_EQ(stop1reason, 5); CHECK_EQ(m1_interrupt_ticks, STATIC_TIMEOUT_M1_CLOSE);
  CHECK_CONTAINS(s1(page()), "TIMEOUT at " + std::to_string(STATIC_TIMEOUT_M1_CLOSE) + " ticks");
}

SCENARIO(site_static_timeouts, "the static timeouts come from the site profile: Portugal's West close may take longer than 6527 ticks") {
  g_travel = 7000;                                      // 115 s run: longer than the old 107 s cap
  setPos(g_travel, g_travel); boot();
  CHECK_EQ(dynTimeout_M1_Close, STATIC_TIMEOUT_M1_CLOSE); CHECK_EQ(dynTimeout_M1_Open, STATIC_TIMEOUT_M1_OPEN);
  CHECK_EQ(dynTimeout_M2_Close, STATIC_TIMEOUT_M2_CLOSE); CHECK_EQ(dynTimeout_M2_Open, STATIC_TIMEOUT_M2_OPEN);
  http("/?$3"); runMs(140000);
  if (SITE == 2) {                                      // Portugal: 7930 ticks (130 s), the run completes
    CHECK_EQ(STATIC_TIMEOUT_M2_CLOSE, 7930); CHECK_EQ(STATIC_TIMEOUT_M1_CLOSE, 7320);
    CHECK(physClosed(1)); CHECK(motorOff(1)); CHECK_EQ(stop2reason, 0);
    CHECK_CONTAINS(s2(page()), "Limit 'Phys. Closed'");
  } else {                                              // Germany: 6527 ticks (107 s), the run is cut
    CHECK_EQ(STATIC_TIMEOUT_M2_CLOSE, 6527);
    CHECK(motorOff(1)); CHECK(!physClosed(1)); CHECK_EQ(stop2reason, 5); CHECK_EQ(m2_interrupt_ticks, 6527);
  }
}

SCENARIO(static_timeouts_by_button_and_web, "panel buttons and web commands load the static timeout of the physical direction") {
  setPos(2500, 2500); boot();
  pressButton(T.btnClose1); CHECK(movingToClosed(0)); CHECK(mot1timer > STATIC_TIMEOUT_M1_CLOSE - 20 && mot1timer <= STATIC_TIMEOUT_M1_CLOSE);
  pressButton(T.btnClose1); CHECK(motorOff(0));
  pressButton(T.btnOpen1);  CHECK(movingToOpen(0));   CHECK(mot1timer > STATIC_TIMEOUT_M1_OPEN - 20 && mot1timer <= STATIC_TIMEOUT_M1_OPEN);
  pressButton(T.btnOpen1);  CHECK(motorOff(0));
  pressButton(T.btnClose2); CHECK(movingToClosed(1)); CHECK(mot2timer > STATIC_TIMEOUT_M2_CLOSE - 20 && mot2timer <= STATIC_TIMEOUT_M2_CLOSE);
  pressButton(T.btnClose2); CHECK(motorOff(1));
  pressButton(T.btnOpen2);  CHECK(movingToOpen(1));   CHECK(mot2timer > STATIC_TIMEOUT_M2_OPEN - 20 && mot2timer <= STATIC_TIMEOUT_M2_OPEN);
  pressButton(T.btnOpen2);  CHECK(motorOff(1));
  http("/?$1"); CHECK(mot1timer > STATIC_TIMEOUT_M1_CLOSE - 20 && mot1timer <= STATIC_TIMEOUT_M1_CLOSE); http("/?$5");
  http("/?$2"); CHECK(mot1timer > STATIC_TIMEOUT_M1_OPEN - 20  && mot1timer <= STATIC_TIMEOUT_M1_OPEN);  http("/?$5");
  http("/?$3"); CHECK(mot2timer > STATIC_TIMEOUT_M2_CLOSE - 20 && mot2timer <= STATIC_TIMEOUT_M2_CLOSE); http("/?$5");
  http("/?$4"); CHECK(mot2timer > STATIC_TIMEOUT_M2_OPEN - 20  && mot2timer <= STATIC_TIMEOUT_M2_OPEN);  http("/?$5");
}

//=============================================================================
// SCENARIOS - limit switch confirmation (v4.3.0)
//=============================================================================
SCENARIO(limit_glitch_ignored, "a spurious HIGH on the closed-limit input shorter than LIM_CONFIRM_TICKS does not stop a closing shutter") {
  setPos(g_travel, g_travel); boot(); ensureRemoteCheck(true);
  http("/?$3"); runTicks(250);
  CHECK(movingToClosed(1)); CHECK(m2_full_run_active);
  int posBefore = S[1].pos;
  S[1].closedGlitchTicks = LIM_CONFIRM_TICKS > 1 ? LIM_CONFIRM_TICKS - 1 : 1;   // Portugal 2026-11-02: ONE such sample stopped the motor after 252 ticks
  runTicks(LIM_CONFIRM_TICKS + 5);
  CHECK(movingToClosed(1)); CHECK(m2_full_run_active); CHECK(S[1].pos < posBefore);
  CHECK(!outboundHas("GET /log?m=2")); CHECK(!outboundHas("GET /interrupt?m=2"));
  CHECK_CONTAINS(ascom(), "INTERMEDIATE|CLOSING");
  runMs(130000);
  CHECK(physClosed(1)); CHECK(motorOff(1)); CHECK_EQ(stop2reason, 0);
  CHECK(outboundHas("GET /log?m=2&d=1&t=50"));         // exactly one complete run of ~g_travel ticks
  CHECK(m2_last_ticks_closing >= g_travel && m2_last_ticks_closing <= g_travel + LIM_CONFIRM_TICKS + 2);
#ifndef PRE_V432
  CHECK_EQ(limGlitchCount, 0);                        // below the confirmation window nothing is even counted
  CHECK_CONTAINS(page(), "Ignored limit levels</td><td>0</td>");
#endif
}

SCENARIO(far_end_glitch_during_full_run_ignored, "a long spurious 'closed' level early in a close from the open stop is ignored (far end implausible)") {
  setPos(g_travel, g_travel); boot(); ensureRemoteCheck(true);
  http("/?$3"); runTicks(200);                        // 3.3 s: where the Portuguese West shutter was stopped
  CHECK(movingToClosed(1)); CHECK(m2_full_run_active);
  S[1].closedGlitchTicks = 30;                         // ~0.5 s HIGH, far longer than the confirmation window
  runTicks(60);
  CHECK(movingToClosed(1)); CHECK(m2_full_run_active);
  CHECK(!outboundHas("GET /log?m=2")); CHECK(!outboundHas("GET /interrupt?m=2"));
#ifndef PRE_V432
  CHECK_EQ(limGlitchCount, 1); CHECK_EQ(limGlitchMotor, 2); CHECK(limGlitchTicks >= 195 && limGlitchTicks <= 215);   // one episode, counted once
  CHECK(outboundHas("GET /event?type=limit_glitch&detail=M2%20at%20"));
  CHECK_CONTAINS(page(), "Ignored limit levels</td><td>1 (last: M2 at ");
#endif
  runMs(130000);
  CHECK(physClosed(1)); CHECK(motorOff(1)); CHECK_EQ(stop2reason, 0);
  CHECK(m2_last_ticks_closing >= g_travel); CHECK(outboundHas("GET /log?m=2&d=1&t=50"));
#ifndef PRE_V432
  CHECK_EQ(limGlitchCount, 1);                        // the real arrival is not an episode
#endif
}

SCENARIO(far_end_level_after_min_run_counts, "the same level after LIM_MIN_RUN_TICKS_CLOSE is taken as the end stop") {
  setPos(g_travel, g_travel); boot();
  http("/?$3"); runTicks(LIM_MIN_RUN_TICKS_CLOSE + 50);
  CHECK(movingToClosed(1));
  S[1].closedGlitchTicks = 30;
  runTicks(LIM_CONFIRM_TICKS + 2);
  CHECK(motorOff(1)); CHECK(!physClosed(1));
}

SCENARIO(reversal_to_start_limit_stops_at_once, "a close from the open stop reversed after 5 s stops at the open stop without overrun") {
  setPos(g_travel, g_travel); boot();
  http("/?$3"); runTicks(300);
  CHECK(movingToClosed(1)); CHECK(S[1].pos < g_travel - 250);
  http("/?$4"); runTicks(5); CHECK(movingToOpen(1));
  long before = S[1].ticksMovingOpen;
  runMs(10000);
  CHECK(physOpen(1)); CHECK(motorOff(1));
  CHECK(S[1].ticksMovingOpen - before <= 300 + LIM_CONFIRM_TICKS + 10);   // no pushing against the open stop
}

SCENARIO(limit_level_at_threshold_is_real, "a HIGH that persists for LIM_CONFIRM_TICKS counts as the limit once the far end is plausible: the motor stops") {
  setPos(g_travel, g_travel); boot();
  http("/?$3"); runTicks(LIM_MIN_RUN_TICKS_CLOSE + 50);   // v4.3.1: before that the far end is implausible and ignored
  S[1].closedGlitchTicks = LIM_CONFIRM_TICKS;
  runTicks(LIM_CONFIRM_TICKS + 2);
  CHECK(motorOff(1)); CHECK(!physClosed(1));
}

SCENARIO(real_limit_stops_within_confirmation_window, "a shutter arriving at its end stop is stopped within LIM_CONFIRM_TICKS ticks") {
  setPos(300, 2500); boot();
  http("/?$1"); runTicks(5); CHECK(movingToClosed(0));
  int n = 0; while (!physClosed(0) && n < 1000) { runTicks(1); n++; }
  CHECK(physClosed(0));
  int k = 0; while (!motorOff(0) && k < 50) { runTicks(1); k++; }
  CHECK(k <= LIM_CONFIRM_TICKS); CHECK_EQ(stop1reason, 0);
  CHECK_CONTAINS(ascom(), "CLOSED|STOPPED|");
}

//=============================================================================
// SCENARIOS - limit fault, emergency override, plausibility windows, start-limit release (v4.3.3)
//=============================================================================
// Portugal 2026-11-20: pin 0 (West "closed") and pin 1 (West "open") both HIGH with the shutter open.
// The page said "Physically CLOSED", $S said CLOSED and no command could close the shutter.
SCENARIO(limit_fault_reported, "both end switches of a shutter active: LIMIT FAULT on the page, $S OPEN, $A INTERMEDIATE, no motion, one limit_fault event") {
  setPos(0, g_travel); S[1].closedStuck = true; physicsStep();        // West fully open, its "closed" line open-circuit
  boot(); ensureRemoteCheck(true); runTicks(LIM_CONFIRM_TICKS);
  std::string pg = page();
  CHECK_CONTAINS(s2(pg), "LIMIT FAULT (both end switches active)");
  CHECK_NOT_CONTAINS(s2(pg), "Physically CLOSED"); CHECK_NOT_CONTAINS(s2(pg), "Physically OPEN"); CHECK_NOT_CONTAINS(s2(pg), "Intermediate");
  CHECK_CONTAINS(s2(pg), "Unknown (limit fault)"); CHECK_NOT_CONTAINS(s2(pg), "Limit 'Phys.");
  CHECK_CONTAINS(s2(pg), "/?$F3");                                    // the override is offered where the fault is
  CHECK_CONTAINS(s1(pg), "Physically CLOSED"); CHECK_NOT_CONTAINS(s1(pg), "LIMIT FAULT"); CHECK_NOT_CONTAINS(s1(pg), "/?$F1");
  CHECK_STREQ(status(), "OPEN");                                      // was CLOSED: the rain checker took the dome for closed
  CHECK_STREQ(ascom(), "CLOSED|STOPPED|INTERMEDIATE|STOPPED");        // ASCOM driver: ShutterError
  http("/?$3"); runTicks(30); CHECK(motorOff(1));                     // nothing automatic or normal moves it
  http("/?$4"); runTicks(30); CHECK(motorOff(1));
  pressButton(T.btnClose2); runTicks(30); CHECK(motorOff(1));
  pressButton(T.btnOpen2); runTicks(30); CHECK(motorOff(1));
  CHECK_EQ(S[1].pos, g_travel); CHECK_EQ(S[1].ticksMovingClosed, 0); CHECK_EQ(S[1].ticksMovingOpen, 0);
  runMs(15000);
  CHECK(outboundHas("GET /event?type=limit_fault&detail=S2%20both%20end%20switches%20active"));
  runMs(20UL * 60 * 1000);
  CHECK_EQ(outboundCount("type=limit_fault"), 1);                     // once per episode, not every few minutes
  S[1].closedStuck = false; runTicks(LIM_CONFIRM_TICKS + 1);          // the line is repaired
  CHECK_CONTAINS(s2(page()), "Physically OPEN"); CHECK_STREQ(ascom(), "CLOSED|STOPPED|OPEN|STOPPED");
  http("/?$3"); runTicks(10); CHECK(movingToClosed(1));
  runMs(130000); CHECK(physClosed(1)); CHECK_STREQ(status(), "CLOSED");
  CHECK_EQ(outboundCount("type=limit_fault"), 1);
}

SCENARIO(limit_fault_open_lines, "the 'open' lines of a closed dome open-circuit: limit fault on both shutters, $S OPEN, nothing moves") {
  setPos(0, 0); S[0].openStuck = true; S[1].openStuck = true; physicsStep();
  boot(); ensureRemoteCheck(true); runTicks(LIM_CONFIRM_TICKS);
  CHECK_STREQ(status(), "OPEN"); CHECK_STREQ(ascom(), "INTERMEDIATE|STOPPED|INTERMEDIATE|STOPPED");
  std::string pg = page();
  CHECK_CONTAINS(s1(pg), "LIMIT FAULT"); CHECK_CONTAINS(s2(pg), "LIMIT FAULT");
  http("/?$2$4"); runTicks(30); http("/?$1$3"); runTicks(30);
  CHECK(motorOff(0)); CHECK(motorOff(1)); CHECK_EQ(S[0].pos, 0); CHECK_EQ(S[1].pos, 0);
  runMs(5000);
  CHECK(outboundHas("GET /event?type=limit_fault&detail=S1%20S2%20both%20end%20switches%20active"));
}

SCENARIO(limit_fault_transient_not_pushed, "both levels active for less than the confirmation time: shown, but no limit_fault event") {
  setPos(0, g_travel); boot(); ensureRemoteCheck(true); runMs(3000);
  S[1].closedGlitchTicks = 30;                                         // 0.5 s on the "closed" line at the open stop
  runTicks(LIM_CONFIRM_TICKS + 2);
  CHECK_CONTAINS(s2(page()), "LIMIT FAULT");
  runMs(20000);
  CHECK_CONTAINS(s2(page()), "Physically OPEN");
  CHECK(!outboundHas("type=limit_fault"));
}

SCENARIO(limit_fault_never_closed_automatically, "cable pulled and Cloudwatcher lost with one shutter in limit fault: the healthy one closes, the faulty one is never driven") {
  setPos(2500, g_travel); S[1].closedStuck = true; physicsStep();
  boot(); ensureRemoteCheck(true); runMs(5000);
  sim::g_link = sim::LINK_OFF; runMs(1000);
  CHECK(movingToClosed(0)); CHECK(motorOff(1)); CHECK(idle(1));
  runMs(120000);
  CHECK(physClosed(0)); CHECK(motorOff(1)); CHECK_EQ(S[1].pos, g_travel); CHECK_EQ(S[1].ticksMovingClosed, 0);
  sim::g_link = sim::LINK_ON; runMs(5000);
  CHECK_STREQ(status(), "OPEN");                                       // and nobody is told the dome is closed
  http("/?$2"); runMs(100000); CHECK(physOpen(0));                     // second path: the Cloudwatcher stops answering
  sim::g_reachable[80] = false;
  runMs(19UL * 60 * 1000);
  CHECK(physClosed(0)); CHECK(totalAutoCloses >= 2);
  CHECK(motorOff(1)); CHECK_EQ(S[1].pos, g_travel); CHECK_EQ(S[1].ticksMovingClosed, 0);
}

SCENARIO(force_close_field_case, "$F3 closes the open West shutter whose 'closed' line is stuck: end switch ignored, runs to the close timeout, never logged as a run") {
  g_travel = 5485;
  setPos(0, g_travel); S[1].closedStuck = true; physicsStep();
  boot(); ensureRemoteCheck(true); runMs(4000);                        // the limit_fault event went out ~3 s ago
  long timeout = dynTimeout_M2_Close;
  forceClose(2); runTicks(10);
  CHECK(movingToClosed(1));
  CHECK(motorOff(0)); CHECK(idle(0)); CHECK_EQ(S[0].ticksMovingClosed, 0);   // the other shutter is not touched
#ifndef PRE_V433
  CHECK(m2ForceClose); CHECK(!m1ForceClose); CHECK(!m2_full_run_active);
#endif
  std::string pg = page();
  CHECK_CONTAINS(s2(pg), "Closing (physically)"); CHECK_CONTAINS(s2(pg), "FORCED, end switch ignored");
  CHECK_CONTAINS(ascom(), "|CLOSING");
  http("/?$3"); runTicks(10); CHECK(movingToClosed(1));                // a rain-checker close during the run does not end it
  runTicks(600);                                                       // open switch released, only the stuck "closed" level is left
  CHECK(!physClosed(1)); CHECK(movingToClosed(1));
  CHECK_STREQ(status(), "OPEN");                                       // a travelling shutter is never reported closed
  CHECK_STREQ(ascom(), "CLOSED|STOPPED|INTERMEDIATE|CLOSING");
  CHECK_NOT_CONTAINS(s2(page()), "Physically CLOSED");
  int n = 0; while (!physClosed(1) && n < 9000) { runTicks(1); n++; }
  CHECK(physClosed(1)); CHECK(movingToClosed(1));                      // arrived, still driving: nobody pressed STOP
  n = 0; while (!motorOff(1) && n < 9000) { runTicks(1); n++; }
  CHECK(motorOff(1));
  CHECK(S[1].ticksMovingClosed >= timeout - 3 && S[1].ticksMovingClosed <= timeout + 3);   // the timeout is the backstop
  runMs(3000);
#ifndef PRE_V433
  CHECK(!m2ForceClose);
#endif
  CHECK_STREQ(status(), "CLOSED"); CHECK_CONTAINS(s2(page()), "Physically CLOSED");
  CHECK(!outboundHas("GET /log?m=2")); CHECK(!outboundHas("GET /interrupt?m=2"));
  CHECK(outboundHas("GET /event?type=limit_fault&detail=S2%20both%20end%20switches%20active"));
  CHECK_EQ(outboundCount("GET /event?type=force_close&detail=S2%20forced%20close%20started"), 1);   // not lost 4 s after the limit_fault push
  http("/?$3"); runTicks(30); CHECK(motorOff(1));                      // closed now: a normal close does nothing
}

SCENARIO(force_close_supervised_stop, "a forced close of a falsely 'closed' shutter is ended by STOP when the operator sees it closed") {
  setPos(0, 2200); S[1].closedStuck = true; physicsStep();             // mid-travel, the page says CLOSED
  boot(); runTicks(LIM_CONFIRM_TICKS);
  CHECK_CONTAINS(s2(page()), "Physically CLOSED");                     // not detectable from two switch levels
  http("/?$3"); runTicks(30); CHECK(motorOff(1)); CHECK_EQ(S[1].pos, 2200);
  forceClose(2); runTicks(5); CHECK(movingToClosed(1));
  int n = 0; while (!physClosed(1) && n < 9000) { runTicks(1); n++; }
  CHECK(physClosed(1));
  http("/?$5"); runTicks(3);
  CHECK(motorOff(1)); CHECK(idle(1)); CHECK(S[1].ticksMovingClosed <= 2200 + 10);
#ifndef PRE_V433
  CHECK(!m2ForceClose);
#endif
  CHECK_CONTAINS(page(), "<h2>Emergency Override</h2>");
  runTicks(LIM_CONFIRM_TICKS);
  CHECK_STREQ(status(), "OPEN");                                       // ended by STOP: the firmware cannot know, the level stays doubted
  CHECK_CONTAINS(s2(page()), "LIMIT FAULT ('closed' end switch not trusted)");
}

SCENARIO(force_close_stopped_early_no_known_position, "an override stopped half way with a flaky 'closed' line: the next OPEN is no full run and stops at the open stop at once") {
  setPos(0, g_travel); S[1].closedStuck = true; physicsStep();         // limit fault at the open stop
  boot(); runTicks(LIM_CONFIRM_TICKS);
  forceClose(2); runTicks(2500);
  CHECK(movingToClosed(1));
  http("/?$5"); runTicks(3 * LIM_CONFIRM_TICKS);                       // stopped half way, the stuck level still reads "closed"
  CHECK(motorOff(1)); CHECK(!physClosed(1)); CHECK(!physOpen(1));
#ifndef PRE_V433
  CHECK_EQ(m2_known_stop, 0);                                          // a level that was ignored during the run proves no position
#endif
  CHECK_STREQ(status(), "OPEN"); CHECK_STREQ(ascom(), "CLOSED|STOPPED|INTERMEDIATE|STOPPED");   // and the shutter is not reported closed
  int pos = S[1].pos;
  long before = S[1].ticksMovingOpen;
  http("/?$4"); runTicks(300);
  CHECK(movingToOpen(1));
#ifndef PRE_V433
  CHECK(!m2_full_run_active);
#endif
  S[1].closedStuck = false;                                            // the line makes contact again once the motor shakes it
  runMs(120000);
  CHECK(physOpen(1)); CHECK(motorOff(1));
  CHECK(S[1].ticksMovingOpen - before <= (g_travel - pos) + LIM_CONFIRM_TICKS + 3);   // no pushing against the open stop
}

SCENARIO(force_close_is_one_run, "the override ends with its run: STOP, panel STOP, reversal and a chained $5$1 leave a normal motor behind") {
  setPos(2500, 2500); boot();
  forceClose(1); runTicks(20); CHECK(movingToClosed(0)); CHECK(motorOff(1));
  pressButton(SWSTOP); CHECK(motorOff(0));
#ifndef PRE_V433
  CHECK(!m1ForceClose);
#endif
  forceClose(1); runTicks(20); CHECK(movingToClosed(0));
  pressButton(T.btnClose1); CHECK(motorOff(0));                        // a panel direction button stops a running motor, forced or not
  pressButton(T.btnClose1); runTicks(5); CHECK(movingToClosed(0));     // and the next press starts a normal close
#ifndef PRE_V433
  CHECK(!m1ForceClose);
#endif
  http("/?$5"); runTicks(3);
  forceClose(1); runTicks(20); CHECK(movingToClosed(0));
  int posAtReverse = S[0].pos; long openBefore = S[0].ticksMovingOpen;
  http("/?$2"); runTicks(5); CHECK(movingToOpen(0));                   // reversal: a normal opening
#ifndef PRE_V433
  CHECK(!m1ForceClose);
#endif
  runMs(60000); CHECK(physOpen(0)); CHECK(motorOff(0));
  CHECK(S[0].ticksMovingOpen - openBefore <= (g_travel - posAtReverse) + LIM_CONFIRM_TICKS + 5);   // stopped by the open end switch
  forceClose(1); runTicks(20); CHECK(movingToClosed(0));
  http("/?$5$1"); runTicks(5);                                         // stop and normal close in ONE request: no ISR tick in between
  CHECK(movingToClosed(0));
#ifndef PRE_V433
  CHECK(!m1ForceClose);
#endif
  long before = S[0].ticksMovingClosed;
  runMs(120000);
  CHECK(physClosed(0)); CHECK(motorOff(0));
  CHECK(S[0].ticksMovingClosed - before <= g_travel + LIM_CONFIRM_TICKS + 5);   // ended at the end switch, not at the timeout
  CHECK_STREQ(ascom().substr(0, 14), "CLOSED|STOPPED");
}

SCENARIO(force_close_only_on_request, "only $F1 / $F3 force: $F, $F2, $F4, $F5 and $Fx do nothing, and $F1 on a healthy closed shutter pushes until the timeout") {
  setPos(2500, 2500); boot();
  for (int k = 0; k < 2; k++) { http("/?$F"); http("/?$F2"); http("/?$F4"); http("/?$F5"); http("/?$Fx"); http("/?$F0");
                                http("/?$G"); http("/?$G2"); http("/?$G4"); http("/?$G5"); http("/?$Gx"); http("/?$G0"); }
  runTicks(30);
  CHECK(motorOff(0)); CHECK(motorOff(1)); CHECK_EQ(S[0].pos, 2500); CHECK_EQ(S[1].pos, 2500);
  http("/?$f3"); http("/?$g3"); runTicks(10); CHECK(movingToClosed(1)); CHECK(motorOff(0));
  http("/?$5"); runTicks(3); CHECK(motorOff(1));
  setPos(0, 2500); runTicks(LIM_CONFIRM_TICKS + 1);                    // shutter 1 really closed, switch fine
  long timeout = dynTimeout_M1_Close;
  forceClose(1); runTicks(10); CHECK(movingToClosed(0));               // the price of the override: it does not stop by itself
  runMs(140000);
  CHECK(motorOff(0)); CHECK(S[0].ticksMovingClosed >= timeout - 3 && S[0].ticksMovingClosed <= timeout + 3);
}

SCENARIO(far_end_window_close_field_case, "a 'closed' level 3268 ticks into a West close (Portugal 2026-11-20) is ignored now: the run completes and is logged once") {
  g_travel = 5485;
  setPos(0, g_travel); boot(); ensureRemoteCheck(true);
  http("/?$3");
  while (m2_tick_counter < 3264 && !motorOff(1)) runTicks(1);
  S[1].closedGlitchTicks = 30;
  runTicks(60);
  CHECK(movingToClosed(1)); CHECK(m2_full_run_active);                 // v4.3.2 stopped here with the shutter 40 % open
  CHECK(!outboundHas("GET /log?m=2"));
#ifndef PRE_V432
  CHECK_EQ(limGlitchCount, 1); CHECK_EQ(limGlitchMotor, 2); CHECK(limGlitchTicks >= 3260 && limGlitchTicks <= 3275);
#endif
  runMs(130000);
  CHECK(physClosed(1)); CHECK(motorOff(1));
  CHECK(outboundHas("GET /log?m=2&d=1&t=54")); CHECK_EQ(outboundCount("GET /log?m=2"), 1);
  CHECK(m2_last_ticks_closing >= g_travel && m2_last_ticks_closing <= g_travel + LIM_CONFIRM_TICKS + 2);
}

SCENARIO(far_end_window_per_direction, "the window follows the physical direction: at 4000 ticks a 'closed' level is still ignored, an 'open' level counts") {
  setPos(g_travel, 0); boot();
  http("/?$1"); http("/?$4");                                          // shutter 1 closes, shutter 2 opens
  runTicks(4000);
  CHECK(movingToClosed(0)); CHECK(movingToOpen(1));
  S[0].closedGlitchTicks = 30; S[1].openGlitchTicks = 30;
  runTicks(LIM_CONFIRM_TICKS + 3);
  CHECK(movingToClosed(0));                                            // 4000 < LIM_MIN_RUN_TICKS_CLOSE
  CHECK(LIM_MIN_RUN_TICKS_OPEN > 4000 || motorOff(1));                 // 4000 >= LIM_MIN_RUN_TICKS_OPEN: taken as the end stop
  CHECK(LIM_MIN_RUN_TICKS_CLOSE > 4000); CHECK(LIM_MIN_RUN_TICKS_OPEN < LIM_MIN_RUN_TICKS_CLOSE);
  runMs(60000);
  CHECK(physClosed(0)); CHECK(motorOff(0));
}

SCENARIO(valid_run_needs_min_ticks, "a run stopped by hand while a spurious far-end level is present is logged as interrupted, not as a valid full run") {
  setPos(0, g_travel); boot(); ensureRemoteCheck(true);
  http("/?$3"); runTicks(1000);
  S[1].closedGlitchTicks = 120;
  runTicks(20); CHECK(movingToClosed(1));
  http("/?$5"); runMs(3000);
  CHECK(motorOff(1)); CHECK(!physClosed(1));
  CHECK(!outboundHas("GET /log?m=2")); CHECK(outboundHas("GET /interrupt?m=2&d=1&t=10"));
  CHECK_EQ(m2_last_ticks_closing, 0);
}

SCENARIO(stuck_start_limit_near_target, "a stuck 'closed' level with the shutter near the open end: OPEN stops at the open stop after the release check, not after the plausibility window") {
#ifdef PRE_V433
  const int LIM_RELEASE_TICKS = 915;                                   // the bound this scenario asks for
#endif
  setPos(0, g_travel - 500); S[1].closedStuck = true; physicsStep();
  boot(); ensureRemoteCheck(true); runMs(15000);
  CHECK_CONTAINS(s2(page()), "Physically CLOSED");                     // the false start limit
  http("/?$4");
  int n = 0, reached = -1;
  while (n < 9000) { runTicks(1); n++; if (reached < 0 && physOpen(1)) reached = n; if (n > 10 && motorOff(1)) break; }
  CHECK(reached > 0 && reached <= 510); CHECK(motorOff(1));
  CHECK(n - reached <= LIM_RELEASE_TICKS - 500 + LIM_CONFIRM_TICKS + 5);   // ~7 s against the stop (v4.3.2: 32 s; with the new window alone 54 s)
#ifndef PRE_V433
  CHECK_EQ(limStuckCount, 1); CHECK_EQ(limStuckMotor, 2);
#endif
  runMs(5000);
  CHECK(!outboundHas("GET /log?m=2"));                                 // never a valid full run
  CHECK(outboundHas(("GET /interrupt?m=2&d=2&t=" + std::to_string(LIM_RELEASE_TICKS) + "&").c_str()));
  CHECK(outboundHas("GET /event?type=limit_fault&detail=S2%20both%20end%20switches%20active"));   // urgent: not held back by the event spacing
  runMs(60000);                                                        // limit_stuck came 2 s after the limit_glitch of the same run: retried
  CHECK(outboundHas("GET /event?type=limit_stuck&detail=M2%20start%20limit%20not%20released%20(1%20since%20boot)"));
  CHECK_EQ(outboundCount("type=limit_stuck"), 1); CHECK_EQ(outboundCount("type=limit_fault"), 1);
  std::string pg = page();
  CHECK_CONTAINS(pg, "Start limit not released</td><td>1 (last: M2)");
  CHECK_CONTAINS(s2(pg), "LIMIT FAULT");                               // open stop reached, "closed" still active
  CHECK_STREQ(status(), "OPEN");
}

SCENARIO(stuck_start_limit_full_travel, "a 'closed' switch that never releases: the opening still ends at the open stop and is not logged as a valid run") {
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  S[1].closedStuck = true;
  http("/?$4"); runTicks(1100);                                        // past LIM_RELEASE_TICKS
  CHECK(movingToOpen(1));                                              // the check never stops a motor
  runMs(120000);
  CHECK(physOpen(1)); CHECK(motorOff(1));
  CHECK(S[1].ticksMovingOpen <= g_travel + LIM_CONFIRM_TICKS + 3);     // no pushing against the open stop
  CHECK(!outboundHas("GET /log?m=2")); CHECK(outboundHas("GET /interrupt?m=2&d=2&t=50"));
  CHECK(outboundHas("type=limit_stuck"));
  runMs(20000);
  CHECK(outboundHas("type=limit_fault"));
}

SCENARIO(stalled_shutter_reported, "a shutter that does not leave its stop: counted and pushed after 15 s, the timeout still ends the run and logs it") {
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  S[0].stuckOpening = true;
  http("/?$2"); runTicks(1100);                                        // past LIM_RELEASE_TICKS
  CHECK(movingToOpen(0));
#ifndef PRE_V433
  CHECK_EQ(limStuckCount, 1); CHECK_EQ(limStuckMotor, 1);
#endif
  runMs(5000);
  CHECK(outboundHas("GET /event?type=limit_stuck&detail=M1%20start%20limit%20not%20released"));
  runMs(125000);
  CHECK(motorOff(0)); CHECK_EQ(S[0].pos, 0);
  CHECK(outboundHas(("GET /interrupt?m=1&d=2&t=" + std::to_string(dynTimeout_M1_Open)).c_str()));
  CHECK(!outboundHas("GET /log?m=1"));
}

SCENARIO(healthy_cycle_no_diagnostics, "a normal open/close cycle of both shutters: four valid runs, no fault, no stuck start limit, no override") {
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$2$4"); runMs(120000);
  CHECK(physOpen(0)); CHECK(physOpen(1)); CHECK_STREQ(ascom(), "OPEN|STOPPED|OPEN|STOPPED");
  http("/?$1$3"); runMs(120000);
  CHECK(physClosed(0)); CHECK(physClosed(1)); CHECK_STREQ(status(), "CLOSED");
  CHECK(outboundHas("GET /log?m=1&d=2&t=50")); CHECK(outboundHas("GET /log?m=2&d=2&t=50"));
  CHECK(outboundHas("GET /log?m=1&d=1&t=50")); CHECK(outboundHas("GET /log?m=2&d=1&t=50"));
  CHECK(!outboundHas("GET /interrupt")); CHECK(!outboundHas("GET /event"));
#ifndef PRE_V433
  CHECK_EQ(limStuckCount, 0); CHECK(!m1ForceClose); CHECK(!m2ForceClose);
#endif
  std::string pg = page();
  CHECK_CONTAINS(pg, "Start limit not released</td><td>0</td>");
  CHECK_NOT_CONTAINS(pg, "LIMIT FAULT");
  CHECK_NOT_CONTAINS(s1(pg), "/?$F"); CHECK_NOT_CONTAINS(s2(pg), "/?$F");   // the override lives in System Details only
  CHECK_CONTAINS(pg, "/?$F1"); CHECK_CONTAINS(pg, "/?$F3");
}

SCENARIO(force_close_two_step, "the override needs two different GET requests: $F3 only arms (the page offers CONFIRM NOW = $G3), $G3 within 30 s starts; repeating $F3 never starts; it expires, is per shutter, and STOP, OPEN or the panel STOP disarm") {
  auto raw = [](const std::string& r) { sim::g_pendingRequests.push_back(r); for (int i = 0; i < 120; i++) { sim::tickOnce(); loop(); } };
  setPos(2500, 2500); boot();
  http("/?$G3"); runTicks(30); CHECK(motorOff(1));                       // nothing armed: the start request alone does nothing
  http("/?$F3"); runTicks(60);
  CHECK(motorOff(0)); CHECK(motorOff(1)); CHECK_EQ(S[1].pos, 2500);        // armed: a stray single request moves nothing
  std::string pg = page();
  CHECK_CONTAINS(s2(pg), "Override ARMED"); CHECK_CONTAINS(s2(pg), "CONFIRM NOW: FORCE CLOSE S2 (West)");
  CHECK_CONTAINS(s2(pg), "/?$G3"); CHECK_NOT_CONTAINS(s2(pg), "/?$F3");   // the confirm link differs from the arming link
  CHECK_NOT_CONTAINS(s1(pg), "Override ARMED"); CHECK_NOT_CONTAINS(s1(pg), "/?$F1"); CHECK_NOT_CONTAINS(s1(pg), "/?$G1");
  for (int k = 0; k < 8; k++) { http("/?$F3"); runMs(2500); }            // the arming request again and again (double click, retried GET): never a start
  CHECK(motorOff(1)); CHECK_EQ(S[1].pos, 2500);
  http("/?$F3$F3"); runTicks(30); CHECK(motorOff(1));
  runMs(31000);                                                          // not confirmed in time
  CHECK_NOT_CONTAINS(s2(page()), "Override ARMED");
#ifndef PRE_V433
  CHECK_EQ(forceArmShutter, 0);                                          // forgotten for good, not only out of its window
#endif
  http("/?$G3"); runTicks(30); CHECK(motorOff(1));                       // too late
  http("/?$F3"); http("/?$G1"); runTicks(30); CHECK(motorOff(0)); CHECK(motorOff(1));   // armed for shutter 2: $G1 starts nothing
  http("/?$F1"); http("/?$G3"); runTicks(30); CHECK(motorOff(0)); CHECK(motorOff(1));   // arming the other shutter replaced it
  http("/?$5"); http("/?$G1"); runTicks(30); CHECK(motorOff(0));                        // STOP cancels the arming
  http("/?$F3"); http("/?$4"); http("/?$5"); http("/?$G3"); runTicks(30); CHECK(motorOff(1));   // so does an OPEN for that shutter
  http("/?$F3"); pressButton(SWSTOP); http("/?$G3"); runTicks(30); CHECK(motorOff(1));  // and the panel STOP button
  http("/?$F3"); raw("HEAD /?$G3 HTTP/1.1\r\nHost: dome\r\n\r\n"); CHECK(motorOff(1));  // a HEAD is no confirmation
  raw("HEAD /?$F1 HTTP/1.1\r\nHost: dome\r\n\r\n"); http("/?$G1"); runTicks(30); CHECK(motorOff(0));   // and arms nothing
  http("/?$F3"); runMs(5000);
  http("/?$G3"); runTicks(10); CHECK(movingToClosed(1)); CHECK(motorOff(0));   // the deliberate pair starts
  CHECK_NOT_CONTAINS(s2(page()), "Override ARMED");
  http("/?$5"); runTicks(3); CHECK(motorOff(1));
  http("/?$F1$G1"); runTicks(10); CHECK(movingToClosed(0)); CHECK(motorOff(1));   // both in one deliberate request
  http("/?$5"); runTicks(3); CHECK(motorOff(0));
}

SCENARIO(force_close_parser, "$F next to other commands, in headers and in a cut-off request never starts or disturbs anything unasked") {
  auto raw = [](const std::string& r) { sim::g_pendingRequests.push_back(r); for (int i = 0; i < 120; i++) { sim::tickOnce(); loop(); } };
  setPos(2500, 2500); boot();
  http("/?$1$F3"); http("/?$1$G3"); runTicks(10);                        // normal close S1 twice, override for S2 armed then started
  CHECK(movingToClosed(0)); CHECK(movingToClosed(1));
#ifndef PRE_V433
  CHECK(!m1ForceClose); CHECK(m2ForceClose);                             // only the shutter named after the F
#endif
  http("/?$5"); runTicks(3); CHECK(motorOff(0)); CHECK(motorOff(1));
  http("/?$3"); runTicks(10); CHECK(movingToClosed(1));
  http("/?$F15"); http("/?$G15"); runTicks(10);                          // "$F1" and "$G1"; the 5 behind them is no STOP
  CHECK(movingToClosed(1)); CHECK(movingToClosed(0));
#ifndef PRE_V433
  CHECK(m1ForceClose); CHECK(!m2ForceClose);
#endif
  http("/?$5"); runTicks(3); CHECK(motorOff(0)); CHECK(motorOff(1));
  runMs(31000);                                                          // nothing armed any more
  raw("GET / HTTP/1.1\r\nHost: dome\r\nReferer: http://dome/?$F3\r\nCookie: last=$F1\r\n\r\n");
  raw("GET / HTTP/1.1\r\nHost: dome\r\nReferer: http://dome/?$G3\r\nCookie: last=$G1\r\n\r\n");
  CHECK(motorOff(0)); CHECK(motorOff(1));                                // commands count in the request line only
  raw("GET /?$F"); raw("GET /?$G");                                      // cut off after the letter
  http("/?$3"); runTicks(10);
  CHECK(movingToClosed(1)); CHECK(motorOff(0));
#ifndef PRE_V433
  CHECK(!m2ForceClose);                                                  // the 3 of a later request is a normal close
#endif
  http("/?$5"); runTicks(3);
}

SCENARIO(stale_closed_level_runs_on, "a 'closed' level that comes 1000 ticks into a West close and stays, after a normal full opening: the close runs on to its timeout and ends closed, is not reported closed on the way and is no valid run") {
  g_travel = 5485;
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$4"); runMs(120000); CHECK(physOpen(1));                     // the evening's opening: a valid full run
  CHECK(outboundHas("GET /log?m=2&d=2&t=54"));
#ifndef PRE_V433
  CHECK(m2_known_proven);
#endif
  long timeout = dynTimeout_M2_Close;
  http("/?$3"); runTicks(1000);
  S[1].closedStuck = true;                                             // the line opens for good
  runTicks(200);
  CHECK(movingToClosed(1));
  CHECK_STREQ(status(), "OPEN"); CHECK_STREQ(ascom(), "CLOSED|STOPPED|INTERMEDIATE|CLOSING");
  while (m2_tick_counter < LIM_MIN_RUN_TICKS_CLOSE + 100 && !motorOff(1)) runTicks(1);
  CHECK(movingToClosed(1)); CHECK(!physClosed(1));                     // until now the run ended here: 23 % open, "CLOSED", logged as valid
  CHECK_STREQ(status(), "OPEN");
  int n = 0; while (!physClosed(1) && n < 9000) { runTicks(1); n++; }
  CHECK(physClosed(1));
  long atArrival = S[1].ticksMovingClosed;
  n = 0; while (!motorOff(1) && n < 9000) { runTicks(1); n++; }
  CHECK(motorOff(1));
  CHECK(S[1].ticksMovingClosed >= timeout - 3 && S[1].ticksMovingClosed <= timeout + 3);   // the motor timeout ends it
  CHECK(S[1].ticksMovingClosed - atArrival <= timeout - g_travel + 3);                     // the price: timeout minus travel against the stop
  runMs(30000);
  CHECK_STREQ(status(), "CLOSED");                                     // and now it is closed
  CHECK(!outboundHas("GET /log?m=2&d=1"));
  CHECK(outboundHas(("GET /interrupt?m=2&d=1&t=" + std::to_string(timeout) + "&").c_str()));
  CHECK(outboundHas("GET /event?type=limit_glitch&detail=M2%20at%20100"));
  CHECK_EQ(outboundCount("GET /event?type=limit_fault&detail=S2%20closed%20switch%20stuck,%20closed%20on%20timeout"), 1);
  http("/?$4"); runMs(120000);                                         // the next opening is still accepted and ends in a limit fault
  CHECK(physOpen(1)); CHECK(motorOff(1));
  CHECK(S[1].ticksMovingOpen <= 2 * g_travel + 2 * LIM_CONFIRM_TICKS + 6);   // no pushing against the open stop
  CHECK(outboundHas("type=limit_stuck&detail=M2%20start%20limit%20not%20released"));   // the stuck switch is named on the way
  CHECK_CONTAINS(s2(page()), "LIMIT FAULT (both end switches active)"); CHECK_STREQ(status(), "OPEN");
  CHECK_EQ(outboundCount("type=limit_fault&detail=S2%20both%20end%20switches%20active"), 1);
}

SCENARIO(stale_closed_level_no_proven_start, "the same fault after a restart with the dome open (position known from boot only): the close ends at the window, is NOT reported closed, raises limit_fault and offers the override") {
  g_travel = 5485;
  setPos(0, g_travel); boot(); ensureRemoteCheck(true);
  http("/?$3"); runTicks(1000);
  S[1].closedStuck = true;
  runMs(130000);
  CHECK(motorOff(1)); CHECK(!physClosed(1));
  CHECK(S[1].ticksMovingClosed >= LIM_MIN_RUN_TICKS_CLOSE - 2 && S[1].ticksMovingClosed <= LIM_MIN_RUN_TICKS_CLOSE + LIM_CONFIRM_TICKS + 2);
  CHECK_STREQ(status(), "OPEN"); CHECK_STREQ(ascom(), "CLOSED|STOPPED|INTERMEDIATE|STOPPED");   // was: CLOSED, with the shutter 23 % open
  std::string pg = page();
  CHECK_CONTAINS(s2(pg), "LIMIT FAULT ('closed' end switch not trusted)"); CHECK_NOT_CONTAINS(s2(pg), "Physically CLOSED");
  CHECK_CONTAINS(s2(pg), "/?$F3");
  CHECK(!outboundHas("GET /log?m=2")); CHECK(outboundHas("GET /interrupt?m=2&d=1&t=42"));
  CHECK_EQ(outboundCount("GET /event?type=limit_fault&detail=S2%20closed%20switch%20not%20trusted,%20close%20stopped"), 1);
  http("/?$3"); runTicks(30); CHECK(motorOff(1));                      // a normal close cannot help
  forceClose(2); runTicks(10); CHECK(movingToClosed(1));               // the supervised override can
  int n = 0; while (!physClosed(1) && n < 9000) { runTicks(1); n++; }
  http("/?$5"); runTicks(3); CHECK(physClosed(1)); CHECK(motorOff(1));
  S[1].closedStuck = false; runTicks(LIM_CONFIRM_TICKS + 2);           // (the line is repaired; the switch itself is active, the shutter is closed)
  CHECK_STREQ(status(), "OPEN");                                       // still doubted: the level has not been seen inactive since
  http("/?$4"); runTicks(60); http("/?$3"); runMs(10000);              // one short move off the switch and back clears it
  CHECK(physClosed(1)); CHECK_STREQ(status(), "CLOSED");
}

SCENARIO(true_early_closed_level_is_no_runon, "a FALSE start position (an 'open' level that stopped an opening half way, or one present at boot) never turns a true early 'closed' level into a run-on: the close ends at the window, not at the timeout") {
  g_travel = 5485;
  setPos(0, 1000); boot(); ensureRemoteCheck(true);
  http("/?$4"); runTicks(1506);
  S[1].openStuck = true;                                               // the "open" line opens at 2506/5485: the opening stops there
  runMs(20000);
  CHECK(motorOff(1)); CHECK(!physOpen(1));
  int pos = S[1].pos;
  runMs(600000);
  http("/?$3"); runTicks(300);
  S[1].openStuck = false;                                              // ... and makes contact again when the motor shakes it
  int n = 0; while (!physClosed(1) && n < 9000) { runTicks(1); n++; }
  CHECK(physClosed(1));
  long atArrival = S[1].ticksMovingClosed;
  runMs(140000);
  CHECK(motorOff(1));
  CHECK(S[1].ticksMovingClosed - atArrival <= LIM_MIN_RUN_TICKS_CLOSE - pos + LIM_CONFIRM_TICKS + 3);   // to the end of the window as before, not 89 s to the timeout
  CHECK(!outboundHas("closed%20on%20timeout"));
  CHECK_STREQ(status(), "OPEN");                                       // doubted, although true: the position was wrong, somebody has to look
}

SCENARIO(true_early_closed_level_boot_case, "the same from boot: a false 'open' level at boot with the shutter nearly closed") {
  g_travel = 5485;
  setPos(0, 700); S[1].openStuck = true; physicsStep();
  boot(); ensureRemoteCheck(true);
  CHECK_CONTAINS(s2(page()), "Physically OPEN");                       // not detectable at rest
  http("/?$3"); runTicks(300);
  S[1].openStuck = false;
  int n = 0; while (!physClosed(1) && n < 9000) { runTicks(1); n++; }
  long atArrival = S[1].ticksMovingClosed;
  runMs(140000);
  CHECK(motorOff(1));
  CHECK(S[1].ticksMovingClosed - atArrival <= LIM_MIN_RUN_TICKS_CLOSE - 700 + LIM_CONFIRM_TICKS + 3);   // 58 s as before the run-on, not 118 s
  CHECK(!outboundHas("closed%20on%20timeout"));
}

SCENARIO(stop_during_stale_level, "STOP while a false 'closed' level is being ignored: the shutter stands half open and is not reported closed until that level has dropped") {
  g_travel = 5485;
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$4"); runMs(120000); CHECK(physOpen(1));
  http("/?$3"); runTicks(1000);
  S[1].closedStuck = true;
  runTicks(1000);
  CHECK(movingToClosed(1));
  http("/?$5"); runTicks(20);
  CHECK(motorOff(1)); CHECK(!physClosed(1));
  CHECK_STREQ(status(), "OPEN"); CHECK_STREQ(ascom(), "CLOSED|STOPPED|INTERMEDIATE|STOPPED");   // was CLOSED, 63 % open
  std::string pg = page();
  CHECK_CONTAINS(s2(pg), "LIMIT FAULT ('closed' end switch not trusted)"); CHECK_CONTAINS(s2(pg), "/?$F3");
  S[1].closedStuck = false; runTicks(LIM_CONFIRM_TICKS + 2);           // the line makes contact again
  CHECK_CONTAINS(s2(page()), "Intermediate"); CHECK_NOT_CONTAINS(s2(page()), "LIMIT FAULT");
  http("/?$3"); runMs(120000);
  CHECK(physClosed(1)); CHECK_STREQ(status(), "CLOSED");
}

SCENARIO(brief_glitch_across_window_end, "a short 'closed' glitch that straddles the end of the window of a healthy close neither stops it nor raises the run-on alarm; the run stays valid") {
  g_travel = 5485;
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$4"); runMs(120000); CHECK(physOpen(1));
  http("/?$3");
  while (m2_tick_counter < LIM_MIN_RUN_TICKS_CLOSE - 24 && !motorOff(1)) runTicks(1);
  S[1].closedGlitchTicks = 60;                                         // active from 24 ticks before to 36 ticks after the window end
  runTicks(200);
  CHECK(movingToClosed(1));
  runMs(130000);
  CHECK(physClosed(1)); CHECK(motorOff(1));
  CHECK(S[1].ticksMovingClosed <= g_travel + LIM_CONFIRM_TICKS + 3);
  CHECK(outboundHas("GET /log?m=2&d=1&t=54"));
  CHECK(!outboundHas("type=limit_fault"));
  CHECK_STREQ(status(), "CLOSED");
}

SCENARIO(known_position_survives_line_dip, "a short dip of the start end switch line at rest does not cost the next run its far-end rule (the shutter cannot move without its motor)") {
  g_travel = 5485;
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$4"); runMs(120000); CHECK(physOpen(1));
  S[1].openLimitBroken = true; runTicks(6); S[1].openLimitBroken = false; runTicks(20);   // 6 ticks LOW at the open stop
  CHECK_CONTAINS(s2(page()), "Physically OPEN");
  http("/?$3"); runTicks(208);
  S[1].closedGlitchTicks = 30;                                         // the documented glitch 3.5 s into the close
  runTicks(60);
  CHECK(movingToClosed(1));                                            // ignored as before
#ifndef PRE_V432
  CHECK_EQ(limGlitchCount, 1);
#endif
  runMs(130000);
  CHECK(physClosed(1)); CHECK(outboundHas("GET /log?m=2&d=1&t=54"));
}

SCENARIO(doubt_survives_line_dip, "a doubted 'closed' level stays doubted through a dip of the line at rest: the shutter stopped at the window is not reported closed again") {
  g_travel = 5485;
  setPos(0, g_travel); boot(); ensureRemoteCheck(true);
  http("/?$3"); runTicks(1000); S[1].closedStuck = true; runMs(130000);
  CHECK(motorOff(1)); CHECK(!physClosed(1)); CHECK_STREQ(status(), "OPEN");
  S[1].closedStuck = false; runTicks(6); S[1].closedStuck = true; runMs(5000);   // 6 ticks LOW, then active again
  CHECK_STREQ(status(), "OPEN"); CHECK_STREQ(ascom(), "CLOSED|STOPPED|INTERMEDIATE|STOPPED");   // was CLOSED again, 23 % open
  CHECK_CONTAINS(s2(page()), "LIMIT FAULT ('closed' end switch not trusted)");
}

SCENARIO(release_check_keeps_stale_level_doubted, "a sticky 'open' switch plus a false 'closed' level: when the release check ends the ignoring, that 'closed' level stays doubted also after the open switch lets go") {
  g_travel = 5485;
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$4"); runMs(120000); CHECK(physOpen(1));
  S[1].openStuck = true;                                               // the open switch does not let go
  http("/?$3"); runTicks(500); S[1].closedStuck = true;                // and the "closed" line opens
  int n = 500; while (!motorOff(1) && n < 9000) { runTicks(1); n++; }
  CHECK(n <= 930); CHECK(!physClosed(1));                              // stopped when the release check gave up the full run
  runTicks(30);
  CHECK_CONTAINS(s2(page()), "LIMIT FAULT (both end switches active)");
  S[1].openStuck = false; runMs(5000);                                 // at rest the open switch lets go
  CHECK_STREQ(status(), "OPEN"); CHECK_STREQ(ascom(), "CLOSED|STOPPED|INTERMEDIATE|STOPPED");   // was CLOSED, 83 % open
  CHECK_CONTAINS(s2(page()), "LIMIT FAULT ('closed' end switch not trusted)");
}

SCENARIO(failed_release_voids_proven_start, "after a close that failed its release check the open stop is no proven start any more: the next close does not run on past a true early 'closed' level") {
  g_travel = 5485;
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$4"); runMs(120000); CHECK(physOpen(1));
  S[1].openStuck = true;                                               // the "open" line opens at rest: invisible
  http("/?$3"); runTicks(2200); http("/?$5"); runTicks(30);            // first close: release check fails, stopped half way
  CHECK(!physClosed(1));
#ifndef PRE_V433
  CHECK_EQ(limStuckCount, 1); CHECK(!m2_known_proven);
#endif
  int pos = S[1].pos;
  runMs(60000);
  http("/?$3"); runTicks(300);
  S[1].openStuck = false;                                              // the line makes contact again
  int n = 0; while (!physClosed(1) && n < 9000) { runTicks(1); n++; }
  long atArrival = S[1].ticksMovingClosed;
  runMs(140000);
  CHECK(motorOff(1));
  CHECK(S[1].ticksMovingClosed - atArrival <= LIM_MIN_RUN_TICKS_CLOSE - pos + LIM_CONFIRM_TICKS + 3);   // to the window, not 76 s to the timeout
  CHECK(!outboundHas("closed%20on%20timeout"));
}

SCENARIO(false_open_level_after_window_is_not_proven, "an opening that ends on a false 'open' level just after its window is no proven start: the close from there does not run on past the true 'closed' level") {
  g_travel = 5485;
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$4"); runTicks(3900);
  S[1].openStuck = true;                                               // false "open" level 3900 ticks into the opening: taken as the arrival
  runMs(20000);
  CHECK(motorOff(1)); CHECK(!physOpen(1));
  int pos = S[1].pos;
#ifndef PRE_V433
  CHECK(!m2_known_proven);
#endif
  runMs(60000);
  http("/?$3"); runTicks(300);
  S[1].openStuck = false;
  int n = 0; while (!physClosed(1) && n < 9000) { runTicks(1); n++; }
  long atArrival = S[1].ticksMovingClosed;
  runMs(140000);
  CHECK(motorOff(1));
  CHECK(S[1].ticksMovingClosed - atArrival <= LIM_MIN_RUN_TICKS_CLOSE - pos + LIM_CONFIRM_TICKS + 3);   // 5 s, not 66 s to the timeout
  CHECK(!outboundHas("closed%20on%20timeout"));
}

SCENARIO(brief_glitch_across_window_end_boot_start, "the same glitch on the first close after a restart with the dome open (no proven start): the close ends at the window, but is reported as not closed and raises limit_fault") {
  g_travel = 5485;
  setPos(0, g_travel); boot(); ensureRemoteCheck(true);
  http("/?$3");
  while (m2_tick_counter < LIM_MIN_RUN_TICKS_CLOSE - 24 && !motorOff(1)) runTicks(1);
  S[1].closedGlitchTicks = 60;
  runMs(130000);
  CHECK(motorOff(1)); CHECK(!physClosed(1));
  CHECK_STREQ(status(), "OPEN");                                       // honest: the line is not active any more, the shutter stands 23 % open
  CHECK(!outboundHas("GET /log?m=2")); CHECK(outboundHas("GET /interrupt?m=2&d=1&t=42"));
  CHECK_EQ(outboundCount("type=limit_fault&detail=S2%20closed%20switch%20not%20trusted,%20close%20stopped"), 1);
  http("/?$3"); runMs(60000);                                          // a second close finishes the job
  CHECK(physClosed(1)); CHECK_STREQ(status(), "CLOSED");
}

SCENARIO(new_event_not_delayed_by_backoff, "a limit fault that begins while an older event is waiting out a long retry pause is pushed at once") {
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  sim::g_reachable[88] = false;
  S[0].stuckOpening = true;
  http("/?$2"); runMs(40UL * 60 * 1000);                               // limit_stuck cannot be delivered: the pause grows to 16 min
  CHECK(!outboundHas("type=limit_stuck"));
  sim::g_reachable[88] = true;
  S[1].openStuck = true;                                               // now shutter 2 shows both end switches
  runMs(15000);
  CHECK_EQ(outboundCount("type=limit_fault&detail=S2%20both%20end%20switches%20active"), 1);   // within seconds, not after the old pause
  runMs(30000);
  CHECK_EQ(outboundCount("type=limit_stuck"), 1);                      // and the old one follows 10 s behind
}

SCENARIO(stale_open_level_ends_at_window, "the mirror case on an opening still ends at the window (no pushing against the open stop), as an interrupted run from a position that is not known afterwards") {
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  http("/?$4"); runTicks(1000);
  S[1].openStuck = true;
  runMs(120000);
  CHECK(motorOff(1)); CHECK(!physOpen(1));
  CHECK(S[1].ticksMovingOpen >= LIM_MIN_RUN_TICKS_OPEN - 2 && S[1].ticksMovingOpen <= LIM_MIN_RUN_TICKS_OPEN + LIM_CONFIRM_TICKS + 2);
  CHECK(!outboundHas("GET /log?m=2")); CHECK(outboundHas("GET /interrupt?m=2&d=2&t=38"));
#ifndef PRE_V433
  CHECK_EQ(m2_known_stop, 0);
#endif
  http("/?$3"); runTicks(20); CHECK(movingToClosed(1));
#ifndef PRE_V433
  CHECK(!m2_full_run_active);                                          // unknown position: no far-end rule, the real stop counts at once
#endif
  runMs(120000);
  CHECK(physClosed(1)); CHECK(motorOff(1));
  CHECK(S[1].ticksMovingClosed <= S[1].ticksMovingOpen + LIM_CONFIRM_TICKS + 5);   // no pushing against the closed stop
}

SCENARIO(level_appearing_at_rest_is_no_start_limit, "an end switch level that appears while the motor is off (a line that opens at rest, mid-travel) is no start limit: the next run stops at its real end stop at once") {
  setPos(2200, 2500); boot();
  S[0].closedStuck = true; S[1].openStuck = true;                      // shutter 1 now reads "closed", shutter 2 "open"; neither moved
  runTicks(LIM_CONFIRM_TICKS + 2);
  CHECK_CONTAINS(s1(page()), "Physically CLOSED"); CHECK_CONTAINS(s2(page()), "Physically OPEN");
  http("/?$2"); http("/?$3"); runTicks(300);                           // open shutter 1, close shutter 2
  CHECK(movingToOpen(0)); CHECK(movingToClosed(1));
#ifndef PRE_V433
  CHECK(!m1_full_run_active); CHECK(!m2_full_run_active);
#endif
  S[0].closedStuck = false; S[1].openStuck = false;                    // the false levels drop inside the release time
  runMs(120000);
  CHECK(physOpen(0)); CHECK(physClosed(1)); CHECK(motorOff(0)); CHECK(motorOff(1));
  CHECK(S[0].ticksMovingOpen <= (g_travel - 2200) + LIM_CONFIRM_TICKS + 3);   // before: 998 ticks against the open stop
  CHECK(S[1].ticksMovingClosed <= 2500 + LIM_CONFIRM_TICKS + 3);              // before: 1718 ticks against the closed stop
  CHECK(!outboundHas("GET /log"));
}

SCENARIO(limit_fault_not_starved, "a line that flaps on one shutter does not hide a steady limit fault on the other, and a flapping fault is reported at most once per 5 min") {
  setPos(g_travel, g_travel); boot(); ensureRemoteCheck(true); runMs(3000);
  S[1].closedStuck = true;                                             // shutter 2: steady fault
  for (int k = 0; k < 20; k++) { S[0].closedStuck = true; runMs(500); S[0].closedStuck = false; runMs(500); }   // shutter 1: 1 Hz, never 1 s long
  CHECK_EQ(outboundCount("type=limit_fault&detail=S2%20both%20end%20switches%20active"), 1);
  CHECK_EQ(outboundCount("type=limit_fault"), 1);
  for (int k = 0; k < 60; k++) { S[0].closedStuck = true; runMs(1500); S[0].closedStuck = false; runMs(500); }  // now 1.5 s long episodes for 2 min
  CHECK_EQ(outboundCount("type=limit_fault"), 1);                      // queued, but one such event went out less than 5 min ago
  for (int k = 0; k < 100; k++) { S[0].closedStuck = true; runMs(1500); S[0].closedStuck = false; runMs(500); }
  CHECK_EQ(outboundCount("type=limit_fault&detail=S1%20both%20end%20switches%20active"), 1);
  CHECK_EQ(outboundCount("type=limit_fault"), 2);
}

SCENARIO(limit_event_retry_backoff, "with the Solo down a limit fault is retried with a growing pause and is still delivered although the episode ended long ago") {
  setPos(g_travel, g_travel); boot(); ensureRemoteCheck(true); runMs(3000);
  sim::g_reachable[88] = false;
  int before = connects(88, false, true);
  S[1].closedStuck = true; runMs(5000); S[1].closedStuck = false;      // a 5 s episode that nobody could be told about
  runMs(30UL * 60 * 1000);
  int tries = connects(88, false, true) - before;
  CHECK(tries >= 4 && tries <= 6);                                     // 60, 120, 240, 480, 960 s apart
  CHECK(!outboundHas("type=limit_fault"));
  sim::g_reachable[88] = true;
  runMs(17UL * 60 * 1000);
  CHECK_EQ(outboundCount("type=limit_fault&detail=S2%20both%20end%20switches%20active"), 1);
}

//=============================================================================
// SCENARIOS - Cloudwatcher link supervision (IP auto-close)
//=============================================================================
SCENARIO(ip_autoclose_closes_dome, "Cloudwatcher unreachable: 10 failed checks close both shutters physically") {
  setPos(2500, 2500); boot(); ensureRemoteCheck(true);
  sim::g_reachable[80] = false;
  runMs(14UL * 60 * 1000);
  CHECK_EQ(S[0].pos, 2500); CHECK_EQ(S[1].pos, 2500); CHECK_EQ(totalAutoCloses, 0);
  CHECK(connectFailCount >= 8);
  CHECK_CONTAINS(page(), "Current IP Fails</td><td>" + std::to_string(connectFailCount) + "/10");
  runMs(3UL * 60 * 1000);
  CHECK_EQ(totalAutoCloses, 1);
  CHECK(S[0].pos < 2500); CHECK(S[1].pos < 2500);
  CHECK(movingToClosed(0) || physClosed(0)); CHECK(movingToClosed(1) || physClosed(1));
  CHECK_EQ(stop1reason, 3); CHECK(m1AutoClosedByIP);
  CHECK_CONTAINS(s1(page()), "(IP Fail)");
  runMs(3UL * 60 * 1000);
  CHECK(physClosed(0)); CHECK(physClosed(1)); CHECK(motorOff(0)); CHECK(motorOff(1));
  CHECK_STREQ(status(), "CLOSED");
  CHECK(connects(80, false, true) >= 10 && connects(80, false, true) <= 12);
  CHECK_EQ(connects(80, true), 0);
  std::string p = page();   // at the limit the ISR normalises reason 3 -> 0: the limit switch is shown
  CHECK_CONTAINS(s1(p), "Limit 'Phys. Closed'"); CHECK_NOT_CONTAINS(s1(p), "IP Fail");
  CHECK_EQ(totalAutoCloses, 1);
}

SCENARIO(ip_autoclose_reverses_opening, "auto-close reverses a shutter that is opening") {
  setPos(2500, 0); boot(); ensureRemoteCheck(true);
  sim::g_reachable[80] = false;
  runMs(16UL * 60 * 1000 + 10000);     // 9 failures so far, 10th at ~16.5 min
  CHECK_EQ(totalAutoCloses, 0);
  http("/?$2$4"); runTicks(5);
  CHECK(movingToOpen(0)); CHECK(movingToOpen(1));
  runMs(60000);
  CHECK_EQ(totalAutoCloses, 1);
  CHECK(movingToClosed(0)); CHECK(movingToClosed(1));
  CHECK_EQ(stop1reason, 3); CHECK_EQ(stop2reason, 3);
  runMs(150000);
  CHECK(physClosed(0)); CHECK(physClosed(1)); CHECK(motorOff(0)); CHECK(motorOff(1));
}

SCENARIO(ip_autoclose_stops_when_link_returns, "auto-close is stopped when the Cloudwatcher answers again") {
  g_travel = 6000;                      // 98 s run: longer than one 90 s check interval
  setPos(6000, 6000); boot(); ensureRemoteCheck(true);
  sim::g_reachable[80] = false;
  runMs(17UL * 60 * 1000);
  CHECK_EQ(totalAutoCloses, 1); CHECK(movingToClosed(0)); CHECK(movingToClosed(1));
  sim::g_reachable[80] = true;
  runMs(100000);
  CHECK(motorOff(0)); CHECK(motorOff(1)); CHECK(!physClosed(0)); CHECK(!physClosed(1));
  CHECK(!m1AutoClosedByIP); CHECK(!m2AutoClosedByIP); CHECK_EQ(stop1reason, 0);
}

SCENARIO(autoclose_reverses_shutter_just_started_from_closed_stop, "cable pulled right after an open from the closed stop: the shutter is reversed although its confirmed level still says closed") {
  setPos(2500, 0); boot(); ensureRemoteCheck(true);
  runMs(5000);
  sim::g_link = sim::LINK_OFF; runTicks(11);            // 176 ms: inside the 200 ms LinkOFF filter
  http("/?$4"); CHECK_EQ(mot2dir, DIR_PHYS_OPEN);       // S2 starts opening at 192 ms, still confirmed "closed"
  CHECK(limRead(LIM2_PHYS_CLOSED));
  for (int i = 0; i < 4 && totalAutoCloses == 0; i++) runTicks(1);   // the cable auto-close decides inside the window
  CHECK_EQ(totalAutoCloses, 1);
  CHECK_EQ(mot2dir, DIR_PHYS_CLOSE); CHECK_EQ(mot1dir, DIR_PHYS_CLOSE);   // S2 reversed (review fix), S1 closes too
#if LIM_CONFIRM_TICKS > 1
  CHECK(m2LimitHold > 0);                               // the stale "closed" level must not end the reversal at once
#endif
  runMs(5000);
  CHECK(physClosed(1)); CHECK(motorOff(1)); CHECK_EQ(m2LimitHold, 0);
  CHECK_CONTAINS(ascom(), "|CLOSED|STOPPED");
}

SCENARIO(link_glitch_single_read_ignored, "one LinkOFF read is not a pulled cable: no auto-close") {
  setPos(2500, 2500); boot(); ensureRemoteCheck(true);
  runMs(5000);
  sim::g_link = sim::LINK_OFF; runTicks(1); sim::g_link = sim::LINK_ON;
  runMs(5000);
  CHECK_EQ(totalAutoCloses, 0); CHECK(idle(0)); CHECK(idle(1)); CHECK_EQ(S[0].pos, 2500); CHECK_EQ(S[1].pos, 2500);
  sim::g_link = sim::LINK_OFF; runMs(1000);              // a real removal still closes
  CHECK_EQ(totalAutoCloses, 1); CHECK(movingToClosed(0)); CHECK(movingToClosed(1));
}

SCENARIO(no_ip_checks_when_closed, "a closed dome never polls the Cloudwatcher and never moves") {
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  sim::g_reachable[80] = false;
  runMs(30UL * 60 * 1000);
  CHECK_EQ(connects(80), 0); CHECK_EQ(totalAutoCloses, 0);
  CHECK(idle(0)); CHECK(idle(1)); CHECK_EQ(S[0].pos, 0); CHECK_EQ(S[1].pos, 0);
}

SCENARIO(cable_removed_closes_dome, "Ethernet cable pulled with the dome open: immediate physical close, once") {
  setPos(2500, g_travel); boot(); ensureRemoteCheck(true);
  runMs(5000);
  http("/?$2"); runTicks(30); CHECK(movingToOpen(0));   // S1 is opening when the cable goes
  sim::g_link = sim::LINK_OFF;
  runMs(1000);
  CHECK(movingToClosed(0)); CHECK(movingToClosed(1)); CHECK_EQ(totalAutoCloses, 1);
  CHECK_EQ(stop1reason, 3); CHECK(m1AutoClosedByIP);
  runMs(120000);
  CHECK(physClosed(0)); CHECK(physClosed(1)); CHECK(motorOff(0)); CHECK(motorOff(1));
  CHECK_EQ(totalAutoCloses, 1); CHECK(cableRemovalAutoCloseTriggered);
  sim::g_link = sim::LINK_ON;
  http("/?$2"); runTicks(5); CHECK(movingToOpen(0));   // dome in use again
  runMs(2000);
  CHECK(!cableRemovalAutoCloseTriggered);
}

//=============================================================================
// SCENARIOS - tick logging and events sent to the Solo
//=============================================================================
SCENARIO(tick_push_direction, "a full close run is logged as d=1 (closing), a full open run as d=2") {
  setPos(g_travel, 2500); boot(); ensureRemoteCheck(true);
  http("/?$1"); runMs(120000);
  CHECK(physClosed(0));
  CHECK(outboundHas("GET /log?m=1&d=1&t=50"));
  CHECK(!outboundHas("GET /log?m=1&d=2"));
  http("/?$2"); runMs(120000);
  CHECK(physOpen(0));
  CHECK(outboundHas("GET /log?m=1&d=2&t=50"));
  http("/?$4"); runMs(60000); CHECK(physOpen(1));     // from mid-travel to the open stop: not a full run, not logged
  CHECK(!outboundHas("GET /log?m=2"));
  http("/?$3"); runMs(120000); CHECK(physClosed(1));
  CHECK(outboundHas("GET /log?m=2&d=1&t=50")); CHECK(!outboundHas("GET /log?m=2&d=2"));
  http("/?$4"); runMs(120000); CHECK(physOpen(1));
  CHECK(outboundHas("GET /log?m=2&d=2&t=50"));
}

SCENARIO(interrupt_push_direction, "an interrupted close is logged on /interrupt with d=1") {
  setPos(g_travel, 2500); boot(); ensureRemoteCheck(true);
  http("/?$1"); runTicks(100); http("/?$5"); runMs(3000);
  CHECK(outboundHas("GET /interrupt?m=1&d=1&t=1"));
}

//=============================================================================
// SCENARIOS - frozen dome detection (VL53L0X) - only in the ToF-enabled build
//=============================================================================
#if ENABLE_TOF_FROZEN_DETECTION
SCENARIO(frozen_detection_normal_opening, "opening with a growing gap is never judged frozen") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  CHECK(tofCalibrated); CHECK(tof_connected);
  CHECK_CONTAINS(page(), "Frozen Detection</td><td>OK");
  http("/?$2"); runMs(10000);
  CHECK(movingToOpen(0)); CHECK_EQ(frozenDomeState, FD_IDLE); CHECK(stop1reason != 6);
  runMs(120000);
  CHECK(physOpen(0)); CHECK(motorOff(0)); CHECK_EQ(frozenRetryCount, 0);
  CHECK(!outboundHas("frozen"));
  http("/?$1"); runMs(120000);         // closing with a shrinking gap: never checked, never flagged
  CHECK(physClosed(0)); CHECK_EQ(frozenDomeState, FD_IDLE); CHECK(stop1reason != 6);
}

SCENARIO(frozen_at_closed_lockout_unlock, "frozen at the closed position: 3 attempts, lockout, $U unlocks") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  S[0].stuckOpening = true;
  http("/?$2"); runMs(6000);
  CHECK(motorOff(0)); CHECK(idle(0)); CHECK_EQ(frozenDomeState, FD_GRAVITY_WAIT); CHECK_EQ(frozenRetryCount, 1);
  // (stop reason 6 is not visible here: at a limit switch the ISR clears it on the next tick)
  CHECK(outboundHas("type=frozen_dome&detail=S1%20attempt%201/3"));   // sent within 10 s of boot: needs the v4.2.0 rate-limiter fix
  CHECK_CONTAINS(page(), "FROZEN! Gravity wait");
  runMs(100000);
  CHECK_EQ(frozenDomeState, FD_LOCKOUT); CHECK_EQ(frozenRetryCount, 3);
  CHECK(outboundHas("type=frozen_lockout"));
  CHECK_EQ(S[0].ticksMovingClosed, 0);          // already at the closed limit: never reversed into the end stop
  CHECK_EQ(S[0].pos, 0);
  CHECK_CONTAINS(page(), "LOCKED OUT");
  http("/?$2"); runTicks(10); CHECK(motorOff(0)); CHECK(idle(0));
  pressButton(T.btnOpen1); CHECK(motorOff(0)); CHECK(idle(0));
  http("/?$1"); runTicks(10); CHECK(motorOff(0));   // close is never blocked (at limit here)
  S[0].stuckOpening = false;
  http("/?$U"); CHECK_EQ(frozenDomeState, FD_IDLE); CHECK_EQ(frozenRetryCount, 0);
  http("/?$2"); runTicks(5); CHECK(movingToOpen(0));
  runMs(120000); CHECK(physOpen(0));
}

SCENARIO(frozen_mid_travel_reverses_to_closed, "frozen mid-travel: motor stops, then reverses PHYSICALLY closed before each retry") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(2000, 0); boot(); ensureRemoteCheck(true);
  S[0].stuckOpening = true;
  sim::g_tofFn = [] { return S[0].stuckOpening ? 100 : 100 + S[0].pos / 5; };   // ice: gap stays at baseline
  http("/?$2"); runMs(6000);
  CHECK(motorOff(0)); CHECK_EQ(stop1reason, 6); CHECK_EQ(frozenDomeState, FD_GRAVITY_WAIT);
  runMs(22000);
  CHECK(movingToClosed(0)); CHECK_EQ(frozenDomeState, FD_CLOSING); CHECK_EQ(stop1reason, 6);
  CHECK_CONTAINS(s1(page()), "Closing (physically)");
  runMs(40000);
  CHECK(physClosed(0)); CHECK(S[0].ticksMovingClosed >= 2000);
  runMs(80000);
  CHECK_EQ(frozenDomeState, FD_LOCKOUT); CHECK_EQ(frozenRetryCount, 3);
  CHECK(outboundHas("type=frozen_lockout"));
  CHECK(physClosed(0)); CHECK(motorOff(0));
}

SCENARIO(frozen_cycle_cancelled_by_close_command, "a close command during the gravity wait ends the cycle and closes the dome") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(2000, 0); boot(); ensureRemoteCheck(true);
  S[0].stuckOpening = true;
  sim::g_tofFn = [] { return S[0].stuckOpening ? 100 : 100 + S[0].pos / 5; };
  http("/?$2"); runMs(6000);
  CHECK_EQ(frozenDomeState, FD_GRAVITY_WAIT);
  http("/?$1"); runTicks(5);
  CHECK(movingToClosed(0)); CHECK_EQ(frozenDomeState, FD_IDLE);
  runMs(60000);
  CHECK(physClosed(0)); CHECK(motorOff(0)); CHECK_EQ(frozenDomeState, FD_IDLE);
}

SCENARIO(frozen_s2_mid_travel_lockout, "shutter 2: frozen mid-travel reverses physically closed, locks out, panel open button blocked, $U unlocks") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(0, 2000); boot(); ensureRemoteCheck(true);
  S[1].stuckOpening = true;
  sim::g_tofFn = [] { return S[1].stuckOpening ? 100 : 100 + S[1].pos / 5; };
  http("/?$4"); runMs(6000);
  CHECK(motorOff(1)); CHECK_EQ(stop2reason, 6); CHECK_EQ(frozenDomeState, FD_GRAVITY_WAIT); CHECK_EQ(frozenMotorNum, 2);
  CHECK(outboundHas("detail=S2%20attempt%201/3"));
  CHECK(idle(0));                                                   // shutter 1 untouched throughout
  runMs(22000);
  CHECK(movingToClosed(1)); CHECK_EQ(frozenDomeState, FD_CLOSING); CHECK_EQ(stop2reason, 6);
  CHECK_CONTAINS(s2(page()), "Closing (physically)");
  runMs(40000);
  CHECK(physClosed(1)); CHECK(S[1].ticksMovingClosed >= 2000);
  runMs(80000);
  CHECK_EQ(frozenDomeState, FD_LOCKOUT); CHECK_EQ(frozenRetryCount, 3);
  CHECK(outboundHas("type=frozen_lockout"));
  http("/?$4"); runTicks(10); CHECK(motorOff(1)); CHECK(idle(1));
  pressButton(T.btnOpen2); CHECK(motorOff(1)); CHECK(idle(1));
  CHECK(idle(0)); CHECK_EQ(S[0].pos, 0);
  S[1].stuckOpening = false;
  http("/?$U"); CHECK_EQ(frozenDomeState, FD_IDLE);
  http("/?$4"); runTicks(5); CHECK(movingToOpen(1));
  runMs(120000); CHECK(physOpen(1));
}

SCENARIO(lockout_never_blocks_closing, "during a lockout the other, open shutter still closes via web and panel button; opening stays blocked") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(0, g_travel); boot(); ensureRemoteCheck(true);
  S[0].stuckOpening = true;
  sim::g_tofFn = [] { return S[0].stuckOpening ? 100 : 100 + S[0].pos / 5; };   // the sensor watches the frozen half only
  http("/?$2"); runMs(100000);
  CHECK_EQ(frozenDomeState, FD_LOCKOUT);
  http("/?$3"); runTicks(5); CHECK(movingToClosed(1));              // web close works
  runTicks(200); http("/?$5"); runTicks(2); CHECK(motorOff(1)); CHECK(!physClosed(1)); CHECK(!physOpen(1));
  http("/?$4"); runTicks(10); CHECK(motorOff(1)); CHECK(idle(1));    // web open blocked mid-travel
  pressButton(T.btnOpen2); CHECK(motorOff(1)); CHECK(idle(1));      // panel open blocked mid-travel
  pressButton(T.btnClose2); CHECK(movingToClosed(1));               // panel close works (BUTTON_MAY_START folds to true)
  runMs(120000); CHECK(physClosed(1)); CHECK(motorOff(1));
  CHECK_EQ(frozenDomeState, FD_LOCKOUT);                            // lockout untouched by closing
  CHECK_EQ(S[0].pos, 0); CHECK(idle(0));
}

SCENARIO(force_close_during_lockout, "frozen lockout on one shutter: the override still closes the other one; an OPEN refused by the lockout does not end it, STOP does") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(0, g_travel); boot(); ensureRemoteCheck(true);
  S[0].stuckOpening = true;
  sim::g_tofFn = [] { return S[0].stuckOpening ? 100 : 100 + S[0].pos / 5; };   // the sensor watches the frozen half only
  http("/?$2"); runMs(100000);
  CHECK_EQ(frozenDomeState, FD_LOCKOUT);
  S[1].closedStuck = true; runTicks(LIM_CONFIRM_TICKS + 2);            // shutter 2 is open and its "closed" line opens: limit fault
  CHECK_CONTAINS(s2(page()), "LIMIT FAULT");
  forceClose(2); runTicks(20); CHECK(movingToClosed(1));
  http("/?$4"); runTicks(10); CHECK(movingToClosed(1));                // refused during the lockout, changes nothing
  CHECK_EQ(frozenDomeState, FD_LOCKOUT);
  http("/?$5"); runTicks(3); CHECK(motorOff(1));
  CHECK(idle(0)); CHECK_EQ(S[0].pos, 0);
}

SCENARIO(cable_autoclose_during_frozen_cycle, "cable pulled during the gravity wait: shutter closes and the frozen cycle never re-opens it") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(2000, 0); boot(); ensureRemoteCheck(true);
  S[0].stuckOpening = true;
  sim::g_tofFn = [] { return S[0].stuckOpening ? 100 : 100 + S[0].pos / 5; };
  http("/?$2"); runMs(6000);
  CHECK_EQ(frozenDomeState, FD_GRAVITY_WAIT);
  sim::g_link = sim::LINK_OFF;
  runMs(1000);
  CHECK(movingToClosed(0)); CHECK_EQ(totalAutoCloses, 1); CHECK_EQ(stop1reason, 3); CHECK_EQ(frozenDomeState, FD_IDLE);
  runMs(60000);
  CHECK(physClosed(0)); CHECK(motorOff(0));
  long openTicks = S[0].ticksMovingOpen;
  runMs(180000);
  CHECK(physClosed(0)); CHECK(idle(0)); CHECK_EQ(S[0].ticksMovingOpen, openTicks); CHECK_EQ(frozenDomeState, FD_IDLE);
}

SCENARIO(ip_autoclose_during_frozen_cycle, "10th Cloudwatcher failure during the gravity wait: shutter closes and stays closed") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(2000, 0); boot(); ensureRemoteCheck(true);
  sim::g_reachable[80] = false;
  S[0].stuckOpening = true;
  sim::g_tofFn = [] { return S[0].stuckOpening ? 100 : 100 + S[0].pos / 5; };
  while (sim::g_millis < 995000UL) { sim::tickOnce(); loop(); }      // 9 failures so far, 10th at ~1010 s
  CHECK_EQ(totalAutoCloses, 0);
  http("/?$2"); runMs(5000);
  CHECK_EQ(frozenDomeState, FD_GRAVITY_WAIT); CHECK(motorOff(0));   // gravity wait spans ~999 s .. 1019 s
  runMs(25000);
  CHECK_EQ(totalAutoCloses, 1); CHECK(movingToClosed(0) || physClosed(0)); CHECK_EQ(frozenDomeState, FD_IDLE);
  runMs(60000);
  CHECK(physClosed(0)); CHECK(motorOff(0));
  long openTicks = S[0].ticksMovingOpen;
  runMs(180000);
  CHECK(physClosed(0)); CHECK(idle(0)); CHECK_EQ(S[0].ticksMovingOpen, openTicks);
}

SCENARIO(tof_calibration_only_when_closed, "$C stores the baseline only with both shutters physically closed") {
  sim::g_tofPresent = true; sim::g_tofMm = 123;
  setPos(0, 2500); boot();
  http("/?$C"); CHECK(!tofCalibrated); CHECK_EQ(sim::g_eeprom[8], 0xFF);
  CHECK_NOT_CONTAINS(page(), "Calibrate ToF Baseline");
  setPos(0, 0); runTicks(5);
  CHECK_CONTAINS(page(), "Calibrate ToF Baseline");
  http("/?$C"); CHECK(tofCalibrated); CHECK_EQ(tofBaseline_mm, 123); CHECK_EQ(sim::g_eeprom[8], 0xA5);
  CHECK_CONTAINS(page(), "ToF Baseline</td><td>12.3 cm");
}

SCENARIO(conflict_event, "limits say closed but the ToF sees a gap: conflict event") {
  sim::g_tofPresent = true; sim::g_tofMm = 100; seedTofCalibration(100);
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  runMs(10000);
  CHECK(!outboundHas("type=conflict"));
  sim::g_tofMm = 200;
  runMs(10000);
  CHECK(outboundHas("type=conflict&detail=Limits=CLOSED"));   // within 5 min of boot: needs the v4.2.0 rate-limiter fix
}

#else
SCENARIO(tof_layer_compiled_out, "ToF layer compiled out: a present sensor is never used, UI says so, $C/$U inert, frozen logic idle") {
  sim::g_tofPresent = true; sim::g_tofMm = 100;      // the mock would answer, the firmware must not ask
  setPos(0, 0); boot(); ensureRemoteCheck(true);
  CHECK(!tof_connected); CHECK_EQ(tofDistance_mm, -1); CHECK(!tofCalibrated);
  std::string p = page();
  CHECK_CONTAINS(p, "ToF Distance</td><td>Not installed");
  CHECK_CONTAINS(p, "Frozen Detection</td><td>Disabled (not built in)");
  CHECK_NOT_CONTAINS(p, "Calibrate ToF Baseline");
  CHECK_CONTAINS(p, "Push target: 192.168." + std::to_string(IP_ADR2) + "." + std::to_string(SOLO_IP_ADR3) + ":88</p>");   // check on: pushes active, Solo per site
  http("/?$C"); CHECK(!tofCalibrated); CHECK_EQ(sim::g_eeprom[8], 0xFF);
  S[0].stuckOpening = true;                          // ice, but nobody can see it
  http("/?$2"); runMs(30000);
  CHECK_EQ(frozenDomeState, FD_IDLE); CHECK(movingToOpen(0)); CHECK(stop1reason != 6);
  CHECK(!outboundHas("frozen")); CHECK(!outboundHas("conflict"));
  http("/?$U"); CHECK_EQ(frozenDomeState, FD_IDLE);
  runMs(120000);
  CHECK(motorOff(0)); CHECK_EQ(m1_interrupt_ticks, STATIC_TIMEOUT_M1_OPEN);   // only the motor timeout ends the pull (reason 5 is normalised at the limit)
}
#endif

//=============================================================================
// SCENARIOS - long-term behaviour
//=============================================================================
SCENARIO(preventive_reset_only_when_closed, "the 8 h Ethernet reset waits until the dome is physically closed") {
  setPos(2500, 2500); boot();
  CHECK_EQ(sim::g_chipResets, 1);
  runMs(8UL * 3600 * 1000 + 600000);
  CHECK_EQ(sim::g_chipResets, 1);
  http("/?$1$3"); runMs(120000);
  CHECK(physClosed(0)); CHECK(physClosed(1));
  runMs(5000);
  CHECK_EQ(sim::g_chipResets, 2);
  CHECK(ethernet_initialized);
}

//=============================================================================
// SCENARIOS - dual-site firmware only: $W Cloudwatcher check toggle
//=============================================================================
#ifdef LIMIT_WIRING_SWAPPED
static const uint8_t CFG_ON  = (uint8_t)((SITE << 1) | 1);   // byte 9 written by THIS site, check on
static const uint8_t CFG_OFF = (uint8_t)((SITE << 1) | 0);   // byte 9 written by THIS site, check off
static const uint8_t CFG_DEF = DEFAULT_REMOTE_CHECK ? CFG_ON : CFG_OFF;

SCENARIO(remote_check_default_and_toggle, "$W toggles the Cloudwatcher check, shows it and persists it with the site id in EEPROM byte 9") {
  setPos(2500, 2500); boot();
  CHECK_EQ(remoteCheckEnabled, DEFAULT_REMOTE_CHECK);
  CHECK_EQ(sim::g_eeprom[9], CFG_DEF);
  CHECK_CONTAINS(page(), std::string("Cloudwatcher Check</td><td>") + (DEFAULT_REMOTE_CHECK ? "ENABLED" : "DISABLED"));
  CHECK_CONTAINS(page(), DEFAULT_REMOTE_CHECK ? "href='/?$W0' class='button b-stop fullwidth' style='margin-top:8px;'>Disable Cloudwatcher Check"
                                              : "href='/?$W1' class='button b-open fullwidth' style='margin-top:8px;'>Enable Cloudwatcher Check");
  http("/?$W");
  CHECK_EQ(remoteCheckEnabled, !DEFAULT_REMOTE_CHECK);
  CHECK_EQ(sim::g_eeprom[9], DEFAULT_REMOTE_CHECK ? CFG_OFF : CFG_ON);
  CHECK_CONTAINS(page(), std::string("Cloudwatcher Check</td><td>") + (DEFAULT_REMOTE_CHECK ? "DISABLED" : "ENABLED"));
  http("/?$W");
  CHECK_EQ(remoteCheckEnabled, DEFAULT_REMOTE_CHECK);
  CHECK_EQ(sim::g_eeprom[9], CFG_DEF);
  CHECK_CONTAINS(page(), WIRING_LABEL);
}

SCENARIO(remote_check_explicit_forms, "$W1 / $W0 are idempotent and only a real change resets the failure window") {
  setPos(2500, 2500); boot();
  http("/?$W0"); CHECK(!remoteCheckEnabled); CHECK_EQ(sim::g_eeprom[9], CFG_OFF);
  http("/?$W0"); CHECK(!remoteCheckEnabled); CHECK_EQ(sim::g_eeprom[9], CFG_OFF);   // double click: stays off
  http("/?$W1"); CHECK(remoteCheckEnabled);  CHECK_EQ(sim::g_eeprom[9], CFG_ON);
  connectFailCount = 3; firstFailTimestamp = 1000;
  http("/?$W1"); CHECK(remoteCheckEnabled);  CHECK_EQ(connectFailCount, 3);          // no change: window untouched
  http("/?$W0"); CHECK(!remoteCheckEnabled); CHECK_EQ(connectFailCount, 0); CHECK_EQ(firstFailTimestamp, 0);
  http("/?$W1$1"); CHECK(remoteCheckEnabled); runTicks(5); CHECK(movingToClosed(0));   // a command may follow
  http("/?$5");
  http("/?$W1"); http("/?$W1"); CHECK(remoteCheckEnabled);
  CHECK(motorOff(0));                                                               // "$W1": the '1' is not CLOSE S1
  http("/?$w0"); CHECK(!remoteCheckEnabled);                                        // lower case accepted
}

SCENARIO(remote_check_persists_off, "byte 9 written by this site with the check off keeps it off after a reboot") {
  seedEepromInitialized(); sim::g_eeprom[9] = CFG_OFF;
  setPos(2500, 2500); boot();
  CHECK(!remoteCheckEnabled); CHECK_EQ(sim::g_eeprom[9], CFG_OFF);
  CHECK_CONTAINS(page(), "Cloudwatcher Check</td><td>DISABLED");
}

SCENARIO(remote_check_persists_on, "byte 9 written by this site with the check on keeps it on after a reboot") {
  seedEepromInitialized(); sim::g_eeprom[9] = CFG_ON;
  setPos(2500, 2500); boot();
  CHECK(remoteCheckEnabled); CHECK_EQ(sim::g_eeprom[9], CFG_ON);
  CHECK_CONTAINS(page(), "Cloudwatcher Check</td><td>ENABLED");
}

SCENARIO(remote_check_reapplies_default_after_site_change, "byte 9 written by a build for the OTHER site is ignored: this site's default applies") {
  const uint8_t otherSite = (SITE == 1) ? 2 : 1;
  seedEepromInitialized();
  sim::g_eeprom[9] = (uint8_t)((otherSite << 1) | (DEFAULT_REMOTE_CHECK ? 0 : 1));   // opposite of our default
  setPos(2500, 2500); boot();
  CHECK_EQ(remoteCheckEnabled, DEFAULT_REMOTE_CHECK);
  CHECK_EQ(sim::g_eeprom[9], CFG_DEF);
}

SCENARIO(remote_check_upgrade_from_v41, "EEPROM written by v4.1.0 (byte 9 untouched): site default applies, nothing else is lost") {
  seedEepromInitialized();
  sim::g_eeprom[1] = 7; sim::g_eeprom[3] = 2; sim::g_eeprom[5] = 42;   // counters from the old firmware
  setPos(2500, 2500); boot();
  CHECK_EQ(remoteCheckEnabled, DEFAULT_REMOTE_CHECK);
  CHECK_EQ(totalIpFailures, 7); CHECK_EQ(totalAutoCloses, 2); CHECK_EQ(dayCounter, 42);
  CHECK_EQ(sim::g_eeprom[9], CFG_DEF);
}

SCENARIO(remote_check_off_no_autoclose, "check off: unreachable Cloudwatcher never polls, never closes, counters stay 0") {
  setPos(2500, 2500); boot(); ensureRemoteCheck(false);
  sim::g_reachable[80] = false;
  runMs(60UL * 60 * 1000);
  CHECK_EQ(connects(80), 0); CHECK_EQ(totalAutoCloses, 0); CHECK_EQ(connectFailCount, 0);
  CHECK(idle(0)); CHECK(idle(1)); CHECK_EQ(S[0].pos, 2500); CHECK_EQ(S[1].pos, 2500);
  CHECK_CONTAINS(page(), "Current IP Fails</td><td>0/10");
  CHECK_CONTAINS(page(), "Push target: 192.168." + std::to_string(IP_ADR2) + "." + std::to_string(SOLO_IP_ADR3) + ":88 (pushes suspended while the Cloudwatcher check is off)");
}

SCENARIO(remote_check_off_no_cable_autoclose, "check off: a pulled cable does not close the dome") {
  setPos(2500, 2500); boot(); ensureRemoteCheck(false);
  runMs(5000);
  sim::g_link = sim::LINK_OFF;
  runMs(5UL * 60 * 1000);
  CHECK_EQ(totalAutoCloses, 0); CHECK(idle(0)); CHECK(idle(1)); CHECK_EQ(S[0].pos, 2500);
}

SCENARIO(remote_check_off_no_pushes, "check off: no tick, interrupt or event traffic to the Solo") {
  tofFromPhysics(); seedTofCalibration(100);
  setPos(g_travel, 0); boot(); ensureRemoteCheck(false);
  http("/?$1"); runMs(120000); CHECK(physClosed(0));
  http("/?$2"); runTicks(100); http("/?$5"); runMs(3000);
#if ENABLE_TOF_FROZEN_DETECTION
  S[0].stuckOpening = true; sim::g_tofFn = [] { return 100; };
  http("/?$1"); runMs(60000); CHECK(physClosed(0));
  http("/?$2"); runMs(6000); CHECK_EQ(frozenDomeState, FD_GRAVITY_WAIT);   // frozen verdict reached (reason 6 is cleared at the limit)
#endif
  CHECK_EQ(connects(88), 0);
  CHECK(sim::g_outbound.empty());
}

SCENARIO(remote_check_toggle_resets_counters, "switching the check off and on clears the failure window") {
  setPos(2500, 2500); boot(); ensureRemoteCheck(true);
  sim::g_reachable[80] = false;
  runMs(6UL * 60 * 1000);
  CHECK(connectFailCount >= 2);
  http("/?$W"); CHECK_EQ(connectFailCount, 0); CHECK_EQ(firstFailTimestamp, 0);
  http("/?$W"); CHECK_EQ(connectFailCount, 0);
  runMs(60000); CHECK_EQ(connectFailCount, 0);      // no check before the 90 s interval has elapsed again
  runMs(12UL * 60 * 1000);
  CHECK_EQ(totalAutoCloses, 0);                     // window restarted: 10 fails need ~15 min again
  runMs(6UL * 60 * 1000);
  CHECK_EQ(totalAutoCloses, 1);
}

SCENARIO(remote_check_off_leaves_running_close_alone, "switching the check off during an auto-close lets the close finish") {
  setPos(2500, 2500); boot(); ensureRemoteCheck(true);
  sim::g_reachable[80] = false;
  runMs(17UL * 60 * 1000);
  CHECK_EQ(totalAutoCloses, 1); CHECK(movingToClosed(0));
  http("/?$W"); runTicks(5);
  CHECK(movingToClosed(0)); CHECK(movingToClosed(1));
  runMs(120000);
  CHECK(physClosed(0)); CHECK(physClosed(1)); CHECK(motorOff(0)); CHECK(motorOff(1));
}
#endif

//=============================================================================
// Runner: one forked process per scenario (fresh firmware globals every time)
//=============================================================================
int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);
  const char* filter = argc > 1 ? argv[1] : nullptr;
  printf("=== AstroShell logic tests - SITE %d: %s - ToF layer %s - DS18B20 layer %s - firmware %s ===\n", SITE, T.name,
         ENABLE_TOF_FROZEN_DETECTION ? "built in" : "compiled out",
         ENABLE_DS18B20_DYNAMIC_TIMEOUT ? (HAVE_TIMEOUT_REGRESSION ? "built in" : "built in, no regression") : "compiled out", FIRMWARE_PATH);
  int passed = 0, failed = 0;
  for (auto& sc : registry()) {
    if (filter && !strstr(sc.name, filter)) continue;
    printf("  [%s]\n", sc.name);
    fflush(stdout);
    pid_t pid = fork();
    if (pid == 0) {
      resetSim();
      sc.fn();
      CHECK_EQ(sim::g_wdtStarved, 0);       // the 8 s hardware watchdog must never be starved
      CHECK_EQ(g_hbridgeShorts, 0);         // both PWM pins of one motor must never be active together
      _exit(g_fails > 200 ? 200 : g_fails);
    }
    int st = 0; waitpid(pid, &st, 0);
    int rc = WIFEXITED(st) ? WEXITSTATUS(st) : 201;
    if (rc == 0) { passed++; printf("    PASS\n"); }
    else { failed++; printf("    FAIL (%s)\n", rc == 201 ? "crashed" : (std::to_string(rc) + " checks").c_str()); }
  }
  printf("=== %d passed, %d failed ===\n", passed, failed);
  return failed ? 1 : 0;
}
