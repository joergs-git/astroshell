// Ethernet library mock: inbound requests come from sim::g_pendingRequests,
// outbound connect() consults sim::g_reachable[port] and blocks for the
// configured connection timeout on failure (as the W5x00 driver does).
#pragma once
#include "Arduino.h"

enum EthernetLinkStatus { Unknown, LinkON, LinkOFF };
#define MAX_SOCK_NUM 8

namespace sim {
struct Conn {
  bool used = false, open = false, inbound = false;
  uint16_t port = 0;
  std::string rx; size_t rxpos = 0;
  std::string tx;
};
inline Conn g_conns[64];
inline int allocConn() {
  for (int i = 0; i < 64; i++) if (!g_conns[i].used) { g_conns[i] = Conn(); g_conns[i].used = true; return i; }
  return -1;
}
}

class EthernetClient : public Print {
 public:
  int id = -1;
  uint16_t timeout = 1000;
  EthernetClient() {}
  explicit EthernetClient(uint8_t) {}                 // socket-index constructor (socketHygiene): no connection
  EthernetClient(int cid, bool) : id(cid) {}
  operator bool() const { return id >= 0; }
  int connect(IPAddress, uint16_t port) {
    bool ok = sim::g_link == sim::LINK_ON && sim::g_reachable.count(port) && sim::g_reachable[port];
    sim::g_connLog.push_back({sim::g_millis, port, ok});
    if (!ok) { sim::advanceMs(timeout); return 0; }
    id = sim::allocConn();
    sim::Conn& c = sim::g_conns[id];
    c.open = true; c.inbound = false; c.port = port;
    sim::advanceMs(5);
    return 1;
  }
  uint8_t connected() { return id >= 0 && sim::g_conns[id].open; }
  int available() { if (!connected()) return 0; sim::Conn& c = sim::g_conns[id]; return (int)(c.rx.size() - c.rxpos); }
  int read() { if (available() <= 0) return -1; sim::Conn& c = sim::g_conns[id]; return (unsigned char)c.rx[c.rxpos++]; }
  size_t write(uint8_t b) override { if (!connected()) return 0; sim::g_conns[id].tx.push_back((char)b); return 1; }
  size_t write(const uint8_t* buf, size_t n) override { if (!connected()) return 0; sim::g_conns[id].tx.append((const char*)buf, n); return n; }
  int availableForWrite() { return 2048; }
  void flush() {}
  void stop() {
    if (id >= 0) {
      sim::Conn& c = sim::g_conns[id];
      if (c.open) { c.open = false; if (c.inbound) sim::g_responses.push_back(c.tx); else sim::g_outbound.push_back(c.tx); }
      c.used = false;
    }
    id = -1;
  }
  void setConnectionTimeout(uint16_t t) { timeout = t; }
  uint8_t status() { return 0; }
};

class EthernetServer {
 public:
  explicit EthernetServer(uint16_t p) : port(p) {}
  void begin() { sim::g_serverBeginCalls++; }
  EthernetClient available() {
    if (sim::g_pendingRequests.empty()) return EthernetClient();
    int id = sim::allocConn();
    sim::Conn& c = sim::g_conns[id];
    c.open = true; c.inbound = true; c.port = port; c.rx = sim::g_pendingRequests.front();
    sim::g_pendingRequests.erase(sim::g_pendingRequests.begin());
    return EthernetClient(id, true);
  }
  inline static uint16_t server_port[MAX_SOCK_NUM] = {0};
  uint16_t port;
};

class EthernetClass {
 public:
  IPAddress localIp;
  void begin(uint8_t*, IPAddress ip) { localIp = ip; }
  EthernetLinkStatus linkStatus() { return sim::g_link == sim::LINK_ON ? LinkON : sim::g_link == sim::LINK_OFF ? LinkOFF : Unknown; }
  IPAddress localIP() { return sim::g_ipLost ? IPAddress(0, 0, 0, 0) : localIp; }
};
inline EthernetClass Ethernet;
