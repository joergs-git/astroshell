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
  setPos(0, 1);  CHECK_STREQ(status(), "OPEN");
  setPos(1, 0);  CHECK_STREQ(status(), "OPEN");
  setPos(g_travel, g_travel); CHECK_STREQ(status(), "OPEN");
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

SCENARIO(static_timeout_without_sensor, "no DS18B20: static 6527 tick timeout, broken limit switch stops the motor with reason 5") {
  setPos(g_travel, 2500); boot();
  CHECK(!dynamicTimeoutActive);
  CHECK_CONTAINS(page(), "Static fallback (6527 ticks)");
  S[0].closedLimitBroken = true;
  http("/?$1"); CHECK(mot1timer > 6510 && mot1timer <= 6527);
  runMs(100000);
  CHECK(physClosed(0)); CHECK(movingToClosed(0));     // arrived, but the switch never reports it: motor still on
  runMs(20000);
  CHECK(motorOff(0)); CHECK_EQ(stop1reason, 5); CHECK_EQ(m1_interrupt_ticks, 6527);
  CHECK_CONTAINS(s1(page()), "TIMEOUT at 6527 ticks");
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
  setPos(g_travel, g_travel); runTicks(5);
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
// SCENARIOS - frozen dome detection (VL53L0X)
//=============================================================================
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
  S[0].stuckOpening = true; sim::g_tofFn = [] { return 100; };
  http("/?$1"); runMs(60000); CHECK(physClosed(0));
  http("/?$2"); runMs(6000); CHECK_EQ(frozenDomeState, FD_GRAVITY_WAIT);   // frozen verdict reached (reason 6 is cleared at the limit)
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
  printf("=== AstroShell logic tests - SITE %d: %s - firmware %s ===\n", SITE, T.name, FIRMWARE_PATH);
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
