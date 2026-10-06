// Whole-system simulation on the PC: 1 chalet + 10 hubs + their tip-up nodes + sonar nodes, running the
// REAL protocol code of lib/IceMesh (ChaletRole, HubRole, planner, line table, sonar chain, codec, store).
// What is simulated around it (ALL ESTIMATES, to replace with field measurements):
//   - radio: log-distance path loss + per-link shadowing + obstacles, per-packet fading, burst outages
//     (Gilbert-Elliott), packet success vs margin over the mode's theoretical sensitivity, collisions with
//     capture, half-duplex;
//   - timing: each hub has its own clock (ppm drift), RX timestamp jitter and TX start latency; a packet
//     only counts if it lands inside the receiver's listening window (preamble tolerance + RX extension);
//   - ESP-NOW node -> hub: per-attempt success, node wake/retry behaviour of the v1 sensor firmware
//     (alert at once + every 5 s while tripped, flag reset seen at the next 5 s wake, heartbeat 60 s),
//     hub offline timeout 90 s (lora_node config);
//   - sonar: real SonarSource (prototype scene + processing) on several holes, FOCUS switched by the chalet.
//
//   g++ -std=c++11 -O2 -I../../lib/IceMesh/src mesh_sim.cpp -o mesh_sim && ./mesh_sim [scenario|all] [seeds] [minutes]
#include <hub_role.h>
#include <chalet_role.h>
#include <sonar_sim.h>
#include <sonar_link.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <map>

using namespace icemesh;
using namespace icemesh::tdma;

// ------------------------------------------------------------------------------------------------
// Random
struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed = 1) : s(seed * 0x9E3779B97F4A7C15ull + 1) {}
  uint64_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
  double u() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double gauss() { double a = u(); if (a < 1e-12) a = 1e-12; return std::sqrt(-2 * std::log(a)) * std::cos(6.283185307179586 * u()); }
  bool chance(double p) { return u() < p; }
  double range(double a, double b) { return a + (b - a) * u(); }
};

// ------------------------------------------------------------------------------------------------
// Model constants (estimates)
static const double TX_DBM = 20, ANT_DBI = 2, PL0 = 31.7, PL_EXP = 3.5;   // near-ground over ice, people, snow
static const double CHALET_GAIN_DB = 6;          // chalet antenna up high
static const double SHADOW_SIGMA = 4, FADE_SIGMA = 3;
static const double BURST_DB = 25;               // a person / snowmobile next to an antenna
static const double CAPTURE_DB = 6;
static const double PREAMBLE_TOL_US = 3 * 1024;  // a receiver can start up to ~3 symbols late (SF9/500)
static const double RX_EXT_US = 2000;            // firmware RX extension when a header is in progress
static const int CHALET_ID = 100, NH = 10;

struct Scenario {
  std::string name, what;
  double extra_db = 0;            // added loss on every LoRa link
  double flat_loss = 0;           // extra random loss on every LoRa packet (interference)
  double burst_p = 0.002, burst_exit = 0.25;   // per link per frame: enter / leave a burst outage
  double espnow_p = 0.85;         // ESP-NOW single attempt success
  double ppm = 20;                // hub clock error, uniform +-ppm
  bool storm = false;             // 10 holes trip within 5 s at 12 min
  bool failures = false;          // link cut, relay hub reboot, hub walking away
  bool adaptive = false;
  int sonar_holes = 4;
  uint16_t frame_ms = 1000;
  uint8_t allowance = 96;
  bool tx_after_miss = true;     // current HubSync::mayTransmit: a hub that missed one beacon still sends
  double hub_timeout_s = 90;     // lora_node NODE_TIMEOUT_SEC (node heartbeat is 60 s)
  int sonar_tries = 1;           // ESP-NOW attempts per sonar block
  int hubs_on = NH;              // hubs powered (the first N of the layout)
};

// ------------------------------------------------------------------------------------------------
struct NodeSim {
  uint8_t id; int hub;
  bool tripped = false;           // true flag state
  double next_trip = 0, clear_at = 0;
  double next_send = 0;           // next ESP-NOW attempt (alert / status / heartbeat)
  uint8_t hub_state = LS_IDLE;    // what the hub believes
  double hub_heard = 0;           // last time the hub heard it
  // metrics of the current trip
  double t_trip = -1, t_hub = -1, t_chalet = -1, t_clear = -1, t_chalet_clear = -1;
};
struct HubSim {
  int idx; uint8_t id; double x, y, obstacle_db = 0;
  HubRole<24> role;
  double ppm = 0, off_us = 0;
  bool alive = true; double dead_until = 0;
  std::vector<int> nodes;
  int sonar_node = -1; sonar::SonarSource* sonar = nullptr;
};

struct Stats {
  std::vector<double> alert_lat, clear_lat, hub_lat, sonar_lat;
  int trips = 0, missed = 0, trips_counted = 0;
  long pkt_tx = 0, pkt_rx_chalet = 0, timing_lost = 0, collisions = 0, joins = 0;
  long false_offline_events = 0; double false_offline_s = 0;
  long frames = 0, slots_planned = 0, frames_all_hubs = 0;
  long focus_made = 0, focus_got = 0, base_age_samples = 0; double base_age_sum = 0, base_age_max = 0;
  double max_sync_err = 0, min_lead_margin = 1e9, min_tail_margin = 1e9;
  double util_sum = 0; uint32_t dropped_slots = 0;
  std::map<int, std::pair<long, long> > hub_slots;  // hub -> (scheduled, received at chalet)
  int max_relayed = 0; long relay_frames = 0;
  int rejoin_frames = -1;
  double silence_prop_max = 0;
};

static double pct(std::vector<double> v, double p) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  size_t k = static_cast<size_t>(p * (v.size() - 1) + 0.5);
  return v[k];
}

// ------------------------------------------------------------------------------------------------
class World {
 public:
  Scenario sc; Rng rng; Stats st;
  ChaletRole<10, 32> chalet;
  sonar::SonarStore<16, 256> store;
  std::vector<HubSim> hubs;
  std::vector<NodeSim> nodes;
  double shadow[NH + 1][NH + 1];
  bool burst[NH + 1][NH + 1];
  std::map<uint8_t, uint8_t> chalet_state;
  std::map<uint8_t, double> offline_since;
  // sonar generation times: node -> (ping index -> true time)
  std::map<uint8_t, std::map<uint16_t, double> > ping_time;
  uint32_t focus_seq = 0; uint8_t focus = 0;
  double now_s = 0;
  double silence_set_at = -1; bool silence_seen[NH];

  World(const Scenario& s, uint64_t seed) : sc(s), rng(seed) {
    chalet.planner.cfg.network_id = 0x42; chalet.planner.cfg.self_id = CHALET_ID;
    chalet.planner.cfg.allowance = sc.allowance; chalet.planner.cfg.test_allowance = 160; chalet.planner.cfg.frame_ms = sc.frame_ms;
    chalet.planner.cfg.adaptive = sc.adaptive;
    chalet.sonar_sink = &store;
    chalet.flags = BF_SONAR_SIM;
    // layout (m): 7 direct hubs at 200-1200 m, 2 behind obstacles (shore trees, island), 1 far out
    const double pos[NH][3] = {{200, 50, 0}, {350, -120, 0}, {500, 80, 0}, {650, -60, 0}, {300, 250, 0}, {450, -300, 0},
                               {800, 100, 0}, {900, 300, 32}, {-150, 420, 30}, {1400, 60, 6}};
    hubs.resize(NH);
    for (int h = 0; h < NH; h++) {
      HubSim& H = hubs[h];
      H.idx = h; H.id = static_cast<uint8_t>(h + 1); H.x = pos[h][0]; H.y = pos[h][1]; H.obstacle_db = pos[h][2];
      H.role.network_id = 0x42; H.role.self = H.id;
      H.ppm = rng.range(-sc.ppm, sc.ppm); H.off_us = rng.range(0, 3e8);
      if (h >= sc.hubs_on) H.alive = false;
    }
    for (int a = 0; a <= NH; a++) for (int b = 0; b <= NH; b++) { shadow[a][b] = 0; burst[a][b] = false; }
    for (int a = 0; a <= NH; a++) for (int b = a + 1; b <= NH; b++) shadow[a][b] = shadow[b][a] = rng.gauss() * SHADOW_SIGMA;
    // 3 tip-up nodes per hub (IDs 10*hub + k), one sonar node on `sonar_holes` hubs (incl. a relayed one)
    for (int h = 0; h < NH; h++)
      for (int k = 0; k < 3; k++) {
        NodeSim n; n.id = static_cast<uint8_t>(10 * (h + 1) + k); n.hub = h;
        n.next_trip = rng.range(60, 600); n.next_send = rng.range(0, 60);
        hubs[h].nodes.push_back(static_cast<int>(nodes.size())); nodes.push_back(n);
      }
    const int sonar_hubs[4] = {2, 7, 4, 9};   // hub 8 sits behind the ridge (relayed)
    for (int k = 0; k < sc.sonar_holes && k < 4; k++) {
      HubSim& H = hubs[sonar_hubs[k]];
      H.sonar_node = H.nodes[0];
      H.sonar = new sonar::SonarSource();
      H.sonar->begin(nodes[H.sonar_node].id, k == 0 ? 0u : 1000u + k, static_cast<uint16_t>(rng.next()));
    }
    for (int h = 0; h < NH; h++) silence_seen[h] = false;
  }
  ~World() { for (auto& H : hubs) delete H.sonar; }

  // ---------------- radio ----------------
  int ix(int hub_idx) const { return hub_idx < 0 ? 0 : hub_idx + 1; }   // 0 = chalet
  double dist(int a, int b) const {
    const double ax = a < 0 ? 0 : hubs[a].x, ay = a < 0 ? 0 : hubs[a].y, bx = b < 0 ? 0 : hubs[b].x, by = b < 0 ? 0 : hubs[b].y;
    return std::max(1.0, std::hypot(ax - bx, ay - by));
  }
  double extraLoss(int h) const {      // time-varying, per device (scenario failures)
    if (h < 0 || !sc.failures) return 0;
    double e = 0;
    if (h == 6 && now_s > 1500) e += std::min(45.0, (now_s - 1500) * 0.25);   // hub 7 carried away
    return e;
  }
  double meanRssi(int a, int b) const {
    double pl = PL0 + 10 * PL_EXP * std::log10(dist(a, b));
    if (a < 0 || b < 0) { const int h = a < 0 ? b : a; pl += hubs[h].obstacle_db; pl -= CHALET_GAIN_DB; }
    if (sc.failures && now_s > 600 && now_s < 780 && ((a < 0 && b == 2) || (b < 0 && a == 2))) pl += 45;   // link cut 3 min
    return TX_DBM + 2 * ANT_DBI - pl - shadow[ix(a)][ix(b)] - sc.extra_db - extraLoss(a) - extraLoss(b);
  }
  // one packet a -> b in mode m: returns received RSSI or NAN
  double packet(int a, int b, RadioMode m) {
    if ((a >= 0 && !hubs[a].alive) || (b >= 0 && !hubs[b].alive)) return NAN;
    double r = meanRssi(a, b) + rng.gauss() * FADE_SIGMA;
    if (burst[ix(a)][ix(b)]) r -= BURST_DB;
    const double margin = r - modeInfo(m).sens_dbm_x10 / 10.0;
    const double p = 1.0 / (1.0 + std::exp(-(margin - 1.0) / 0.7));   // ~3 dB from 10 % to 95 % (est.)
    if (!rng.chance(p) || rng.chance(sc.flat_loss)) return NAN;
    return r;
  }
  void stepBursts() {
    for (int a = 0; a <= NH; a++) for (int b = a + 1; b <= NH; b++) {
      bool& s = burst[a][b];
      if (s) { if (rng.chance(sc.burst_exit)) s = false; } else if (rng.chance(sc.burst_p)) s = true;
      burst[b][a] = s;
    }
  }

  // ---------------- clocks ----------------
  double local(const HubSim& H, double t_us) const { return t_us * (1 + H.ppm * 1e-6) + H.off_us; }
  double trueOf(const HubSim& H, double l_us) const { return (l_us - H.off_us) / (1 + H.ppm * 1e-6); }
  uint32_t l32(double v) const { return static_cast<uint32_t>(static_cast<uint64_t>(v) & 0xFFFFFFFFull); }
  double unwrap(const HubSim& H, uint32_t l, double near_true) const {   // lib time (u32) -> true time near `near_true`
    const double ln = local(H, near_true);
    double d = static_cast<double>(static_cast<int32_t>(l - l32(ln)));
    return trueOf(H, ln + d);
  }

  // ---------------- nodes (ESP-NOW) ----------------
  double bundle() { return 1 - std::pow(1 - sc.espnow_p, 5); }   // unicast 3 tries + 2 broadcasts
  void deliver(NodeSim& n, double t) {
    n.hub_heard = t;
    const uint8_t s = n.tripped ? LS_TRIPPED : LS_IDLE;
    if (n.hub_state != s && n.tripped && n.t_hub < 0) n.t_hub = t;
    n.hub_state = s;
  }
  void stepNodes(double t0, double t1) {
    for (auto& n : nodes) {
      HubSim& H = hubs[n.hub];
      if (n.hub >= sc.hubs_on) continue;
      // trips: Poisson, mean every ~6 min per hole, flag up 20-90 s
      if (!n.tripped && t1 >= n.next_trip) {
        n.tripped = true; n.t_trip = n.next_trip; n.t_hub = n.t_chalet = n.t_clear = n.t_chalet_clear = -1;
        n.clear_at = n.t_trip + rng.range(20, 90); n.next_send = n.t_trip + 0.15; st.trips++;
        // previous clear never arrived: the chalet still shows this hole as FISH ON (visible at once)
        if (chalet_state.count(n.id) && chalet_state[n.id] == LS_TRIPPED) n.t_chalet = n.t_trip;
      }
      if (n.tripped && t1 >= n.clear_at) {
        n.tripped = false; n.t_clear = n.clear_at;
        n.next_send = n.clear_at + rng.range(0, 5) + 0.15;    // reset seen at the next 5 s wake
        n.next_trip = n.clear_at + 60 + rng.range(0, 1) * 600;
      }
      while (n.next_send <= t1) {
        const double t = n.next_send;
        if (H.alive && rng.chance(bundle())) deliver(n, t);
        n.next_send = t + (n.tripped ? 5.0 : 60.0);
      }
      // hub view: offline after 90 s without a message (lora_node NODE_TIMEOUT_SEC)
      uint8_t view = n.hub_state;
      if (t1 - n.hub_heard > sc.hub_timeout_s) view = LS_OFFLINE;
      if (H.alive) H.role.table.observe(n.id, view, 0, 0, 90, static_cast<uint32_t>(t1 * 1000));
      (void)t0;
    }
  }

  // ---------------- sonar ----------------
  void stepSonar(double t) {
    for (auto& H : hubs) {
      if (!H.sonar || !H.alive) continue;
      const uint8_t nid = nodes[H.sonar_node].id;
      const bool f = H.role.focusNode() == nid;
      for (int k = 0; k < 4; k++) {
        sonar::Block out[2];
        const uint8_t n = H.sonar->tick(f, out, 2);
        ping_time[nid][H.sonar->lastPing().index] = t + 0.25 * k;
        if (nid == focus) st.focus_made++;
        for (uint8_t i = 0; i < n; i++)
          if (rng.chance(1 - std::pow(1 - sc.espnow_p, sc.sonar_tries))) H.role.sonar.push(out[i].data, out[i].len, H.role.plan.frame);
      }
    }
  }

  // ---------------- one superframe ----------------
  void frame(uint16_t f) {
    const double T = f * (sc.frame_ms * 1000.0);    // true REF of this frame (chalet clock = true time)
    now_s = T / 1e6;
    stepBursts();
    // scenario events
    if (sc.failures && f == static_cast<uint16_t>(1000000 / sc.frame_ms)) {                 // relay candidate hub 7 reboots: 20 s off, state lost
      hubs[6].alive = false; hubs[6].dead_until = 1020;
      hubs[6].role = HubRole<24>(); hubs[6].role.network_id = 0x42; hubs[6].role.self = hubs[6].id;
    }
    for (auto& H : hubs) if (!H.alive && now_s >= H.dead_until && H.dead_until > 0) H.alive = true;
    const int alive_target = sc.hubs_on;
    if (sc.storm && f == static_cast<uint16_t>(720000 / sc.frame_ms)) for (int k = 0; k < 10; k++) { NodeSim& n = nodes[3 * k + 1]; if (!n.tripped) n.next_trip = now_s + rng.range(0, 5); }
    if (static_cast<int>(now_s) % 120 == 30 && std::fmod(now_s, 1.0) < sc.frame_ms / 1000.0) {                            // chalet switches FOCUS between sonar holes
      std::vector<uint8_t> s; for (auto& H : hubs) if (H.sonar) s.push_back(nodes[H.sonar_node].id);
      if (!s.empty()) { focus = s[(static_cast<int>(now_s) / 120) % s.size()]; chalet.focus_node = focus; }
    }
    if (f == 300) { chalet.flags |= BF_SILENCED; silence_set_at = now_s; }
    stepNodes(now_s - sc.frame_ms / 1000.0, now_s);
    stepSonar(now_s);

    uint8_t bbuf[MAX_PACKET];
    const size_t blen = chalet.startFrame(f, bbuf, sizeof bbuf);
    const Beacon plan = chalet.beacon;
    SlotTime times[MAX_SLOTS]; memcpy(times, chalet.times, sizeof times);
    st.frames++;
    st.dropped_slots = chalet.planner.droppedSlots();
    // utilisation: last slot end / frame
    if (plan.n_slots) st.util_sum += (times[plan.n_slots - 1].end + 0.0) / (sc.frame_ms * 1000.0);
    int hub_slots = 0;
    for (uint8_t i = 0; i < plan.n_slots; i++) if (plan.slots[i].kind == SLOT_HUB) {
      hub_slots++; st.slots_planned++; st.hub_slots[plan.slots[i].owner].first++;
      if (plan.slots[i].via) st.relay_frames++;
    }
    if (hub_slots >= alive_target) st.frames_all_hubs++;
    else if (getenv("SIM_MISSING") && now_s > 60) {
      fprintf(stderr, "f=%u missing:", f);
      for (int h = 0; h < NH; h++) { bool in = false; for (uint8_t i = 0; i < plan.n_slots; i++) in |= plan.slots[i].kind == SLOT_HUB && plan.slots[i].owner == hubs[h].id; if (!in) { const HubInfo* hi = chalet.planner.find(hubs[h].id); fprintf(stderr, " H%d(%s via=%d)", h + 1, hi ? "known" : "unknown", hi ? hi->via : -1); } }
      fprintf(stderr, " slots=%u\n", plan.n_slots);
    }

    // beacon
    const double b_air = beaconAirtimeUs(static_cast<uint16_t>(blen));
    bool fresh[NH];
    for (int h = 0; h < NH; h++) {
      fresh[h] = false;
      HubSim& H = hubs[h];
      if (!H.alive) continue;
      if (H.role.sync.synced()) {                   // listening window around the expected beacon
        const double exp_ref = unwrap(H, H.role.sync.ref() + H.role.sync.frameUs(), T);
        if (std::fabs(exp_ref - T) > BEACON_WINDOW_US) continue;
      }
      const double r = packet(-1, h, MODE_SF9_BW500);
      if (std::isnan(r)) continue;
      const double rx_end = T + b_air + 20 + rng.gauss() * 10;    // RxDone ISR jitter (est.)
      uint8_t cmd;
      fresh[h] = H.role.onBeacon(bbuf, blen, l32(local(H, rx_end)), static_cast<int8_t>(std::max(-128.0, r)), 20, cmd);
      if (fresh[h]) {
        const double err = unwrap(H, H.role.sync.ref(), T) - T;
        st.max_sync_err = std::max(st.max_sync_err, std::fabs(err));
      }
    }
    // slots
    bool missed_done = false;
    auto doMissed = [&]() {
      if (missed_done) return; missed_done = true;
      for (int h = 0; h < NH; h++) if (hubs[h].alive && !fresh[h]) hubs[h].role.onBeaconMissed();
    };
    for (uint8_t i = 0; i < plan.n_slots; i++) {
      const Slot& s = plan.slots[i];
      if (s.kind != SLOT_ECHO) doMissed();
      const RadioMode m = slotMode(s);
      struct Tx { int h; uint8_t buf[MAX_PACKET]; size_t len; double t0, t1; bool echo; };
      std::vector<Tx> txs;
      for (int h = 0; h < NH; h++) {
        HubSim& H = hubs[h];
        if (!H.alive || !H.role.have_plan || i >= H.role.plan.n_slots) continue;
        const SlotAction a = H.role.action(i, rng.chance(1.0 / 3));
        if (a != ACT_TX_HUB && a != ACT_TX_ECHO && a != ACT_TX_JOIN) continue;
        if (a == ACT_TX_ECHO && !fresh[h]) continue;          // firmware: a hub that missed the beacon is listening here
        if (!sc.tx_after_miss && !fresh[h]) continue;
        Tx x; x.h = h; x.echo = (a == ACT_TX_ECHO);
        if (a == ACT_TX_HUB) x.len = H.role.buildHub(x.buf, MAX_PACKET, 0, 4000, 1, false, 0);
        else if (a == ACT_TX_ECHO) x.len = H.role.buildEcho(x.buf, MAX_PACKET);
        else { x.len = H.role.buildJoin(x.buf, MAX_PACKET); st.joins++; }
        if (x.len == 0) continue;
        const RadioMode hm = slotMode(H.role.plan.slots[i]);
        const double l_tx = static_cast<double>(H.role.sync.ref()) + H.role.times[i].tx + (a == ACT_TX_JOIN ? rng.range(0, 2000) : 0);
        x.t0 = unwrap(H, l32(l_tx), T + H.role.times[i].tx) + rng.range(150, 400);   // TX start latency (est.)
        x.t1 = x.t0 + modeAirtimeUs(hm, static_cast<uint16_t>(x.len));
        txs.push_back(x); st.pkt_tx++;
      }
      if (txs.size() > 1) st.collisions++;
      // receivers: chalet (not during echo slots) + every hub listening
      auto receive = [&](int rx) -> int {          // returns index into txs or -1
        std::vector<std::pair<double, int> > got;
        for (size_t k = 0; k < txs.size(); k++) {
          if (txs[k].h == rx) return -1;           // half duplex
          const double r = packet(txs[k].h, rx, m);
          if (!std::isnan(r)) got.push_back(std::make_pair(r, static_cast<int>(k)));
        }
        if (got.empty()) return -1;
        std::sort(got.rbegin(), got.rend());
        if (got.size() > 1 && got[0].first - got[1].first < CAPTURE_DB) return -1;
        return got[0].second;
      };
      // chalet
      if (s.kind != SLOT_ECHO) {
        const int k = receive(-1);
        bool ok = false;
        if (k >= 0) {
          const double open = T + times[i].start, close = T + times[i].end + RX_EXT_US;
          const Tx& x = txs[k];
          st.min_lead_margin = std::min(st.min_lead_margin, x.t0 - open);
          st.min_tail_margin = std::min(st.min_tail_margin, close - x.t1);
          if (x.t0 < open - PREAMBLE_TOL_US || x.t1 > close) {
            st.timing_lost++;
            if (getenv("SIM_DEBUG")) {
              const HubSim& H = hubs[x.h];
              fprintf(stderr, "timing f=%u slot=%u kind=%u owner=%u tx_hub=%u fresh=%d echo_plan=%d hubplanframe=%u streak_ok=%d lead=%.0f tail=%.0f len=%u allow=%u hubslots=%u chaletslots=%u\n",
                      f, i, s.kind, s.owner, H.id, fresh[x.h], H.role.plan_from_echo, H.role.plan.frame, H.role.sync.mayTransmit(),
                      x.t0 - open, close - x.t1, (unsigned)x.len, s.allowance, H.role.plan.n_slots, plan.n_slots);
            }
          }
          else {
            ChaletRxResult res;
            if (chalet.onSlotPacket(i, x.buf, x.len, -100, 20, res)) {
              st.pkt_rx_chalet++;
              applyChanges(res.changes, res.n_changes, f);
            }
            ok = true;
          }
        }
        if (!ok) chalet.onSlotEmpty(i, false);
      }
      // hubs
      for (int h = 0; h < NH; h++) {
        HubSim& H = hubs[h];
        if (!H.alive) continue;
        const bool listens = H.role.have_plan ? (i < H.role.plan.n_slots && H.role.action(i, false) == ACT_RX) : true;
        if (!listens) continue;
        const int k = receive(h);
        if (k < 0) continue;
        const Tx& x = txs[k];
        const double rx_end = x.t1 + 20 + rng.gauss() * 10;
        if (x.echo) {
          if (!fresh[h]) { uint8_t cmd; fresh[h] = H.role.onBeacon(x.buf, x.len, l32(local(H, rx_end)), -100, 0, cmd); }
        } else {
          H.role.onSlotPacket(i, x.buf, x.len, -100, 20);
        }
      }
    }
    doMissed();
    for (int h = 0; h < NH; h++) {
      HubSim& H = hubs[h];
      if (!H.alive) continue;
      // hubs without a plan: random JOIN somewhere in the frame (ALOHA, 1 in 3 frames)
      if (H.role.wantsAsyncJoin() && rng.chance(1.0 / 3)) {
        uint8_t jb[MAX_PACKET]; const size_t jl = H.role.buildJoin(jb, sizeof jb); st.joins++;
        const double util = plan.n_slots ? (times[plan.n_slots - 1].end + 0.0) / (sc.frame_ms * 1000.0) : 0.1;
        if (!rng.chance(util)) {                    // lands in a busy slot otherwise
          if (!std::isnan(packet(h, -1, MODE_SF9_BW500))) chalet.onAsyncPacket(jb, jl, -110);
          for (int o = 0; o < NH; o++) if (o != h && !std::isnan(packet(h, o, MODE_SF9_BW500))) hubs[o].role.onAsyncPacket(jb, jl, -100, 0);
        }
      }
      H.role.endFrame();
      if (silence_set_at >= 0 && !silence_seen[h] && (H.role.plan.flags & BF_SILENCED) && H.role.have_plan) {
        silence_seen[h] = true; st.silence_prop_max = std::max(st.silence_prop_max, now_s - silence_set_at);
      }
    }
    // delivery per scheduled hub (direct or through its relay)
    for (uint8_t i = 0; i < plan.n_slots; i++) if (plan.slots[i].kind == SLOT_HUB) {
      const HubInfo* hi = chalet.planner.find(plan.slots[i].owner);
      if (hi && hi->last_any == f) st.hub_slots[plan.slots[i].owner].second++;
    }
    NodeChange ex[16];
    const uint8_t ne = chalet.expireNodes(ex, 16);
    applyChanges(ex, ne, f);
    st.max_relayed = std::max(st.max_relayed, countRelayed(plan));
    sonarMetrics(f);
    // false offline (node alive, its hub alive, chalet says OFFLINE)
    for (auto& n : nodes) {
      auto it = chalet_state.find(n.id);
      const bool off = it != chalet_state.end() && it->second == LS_OFFLINE;
      if (off && hubs[n.hub].alive) st.false_offline_s += sc.frame_ms / 1000.0;
    }
  }
  int countRelayed(const Beacon& b) const { int n = 0; for (uint8_t i = 0; i < b.n_slots; i++) if (b.slots[i].kind == SLOT_HUB && b.slots[i].via) n++; return n; }

  void applyChanges(const NodeChange* ch, uint8_t n, uint16_t f) {
    for (uint8_t k = 0; k < n; k++) {
      const uint8_t id = ch[k].node, ns = ch[k].new_state;
      const uint8_t old = chalet_state.count(id) ? chalet_state[id] : 0xFF;
      chalet_state[id] = ns;
      if (ns == LS_OFFLINE && old != LS_OFFLINE && old != 0xFF) st.false_offline_events++;
      for (auto& nd : nodes) if (nd.id == id) {
        if (ns == LS_TRIPPED && nd.t_trip >= 0 && nd.t_chalet < 0) nd.t_chalet = now_s;
        if (ns == LS_IDLE && nd.t_clear >= 0 && nd.t_chalet_clear < 0) nd.t_chalet_clear = now_s;
      }
    }
  }
  void sonarMetrics(uint16_t f) {
    if (focus) {
      const sonar::StoredPing* out[256];
      const uint16_t n = store.pingsSince(focus, focus_seq, out, 256);
      for (uint16_t k = 0; k < n; k++) {
        st.focus_got++;
        auto& m = ping_time[focus];
        auto it = m.find(out[k]->p.index);
        if (it != m.end()) st.sonar_lat.push_back(now_s - it->second);
        focus_seq = std::max(focus_seq, out[k]->seq);
      }
      if (n == 0) focus_seq = std::max(focus_seq, store.lastSeq());
    }
    for (auto& H : hubs) {
      if (!H.sonar) continue;
      const uint8_t id = nodes[H.sonar_node].id;
      if (id == focus) continue;
      const sonar::NodeSonar* ns = store.find(id);
      if (!ns || now_s < 60) continue;
      const double age = static_cast<int16_t>(f - ns->frame) * sc.frame_ms / 1000.0;
      st.base_age_sum += age; st.base_age_samples++; st.base_age_max = std::max(st.base_age_max, age);
    }
  }
  void finishTrips() {
    for (auto& n : nodes) {
      (void)n;
    }
  }
};

// ------------------------------------------------------------------------------------------------
struct Agg {
  std::vector<double> alert, clear, hub, son;
  long trips = 0, missed = 0, tx = 0, rxc = 0, timing = 0, coll = 0, joins = 0, foff_ev = 0;
  double foff_s = 0, frames = 0, all_hubs = 0, util = 0, sync = 0, lead = 1e9, tail = 1e9, base_sum = 0, base_n = 0, base_max = 0, sil = 0;
  long fmade = 0, fgot = 0; uint32_t dropped = 0; int max_rel = 0;
  std::map<int, std::pair<long, long> > slots;
};

static void runOne(const Scenario& sc, uint64_t seed, int minutes, Agg& A) {
  World w(sc, seed);
  const int frames = minutes * 60000 / sc.frame_ms;
  const double end_s = minutes * 60.0;
  // trip bookkeeping per trip (nodes reuse fields) -> record at clear/next trip
  std::vector<double> last_trip(w.nodes.size(), -1);
  for (int f = 1; f <= frames; f++) {
    w.frame(static_cast<uint16_t>(f));
    for (size_t k = 0; k < w.nodes.size(); k++) {
      NodeSim& n = w.nodes[k];
      // a trip is final 60 s after its clear (or at the end): record it once
      if (n.t_trip >= 0 && n.t_trip != last_trip[k] && ((n.t_clear >= 0 && w.now_s > n.t_clear + 60) || f == frames)) {
        last_trip[k] = n.t_trip;
        if (n.t_trip < 90 || n.t_trip > end_s - 120) continue;   // warm-up / too close to the end
        A.trips++;
        if (n.t_chalet < 0) A.missed++;
        else A.alert.push_back(n.t_chalet - n.t_trip);
        if (getenv("SIM_LATE") && (n.t_chalet < 0 || n.t_chalet - n.t_trip > 10)) {
          const HubInfo* hi = w.chalet.planner.find(w.hubs[n.hub].id);
          fprintf(stderr, "late node %u hub H%d trip %.0f s: hub knew after %.1f s, chalet after %.1f s, cleared after %.0f s, hub via=%d now\n",
                  n.id, n.hub + 1, n.t_trip, n.t_hub >= 0 ? n.t_hub - n.t_trip : -1.0, n.t_chalet >= 0 ? n.t_chalet - n.t_trip : -1.0,
                  n.t_clear - n.t_trip, hi ? hi->via : -1);
        }
        if (n.t_hub >= 0) A.hub.push_back(n.t_hub - n.t_trip);
        if (n.t_clear >= 0 && n.t_chalet_clear >= 0) A.clear.push_back(n.t_chalet_clear - n.t_clear);
      }
    }
  }
  Stats& s = w.st;
  A.son.insert(A.son.end(), s.sonar_lat.begin(), s.sonar_lat.end());
  A.tx += s.pkt_tx; A.rxc += s.pkt_rx_chalet; A.timing += s.timing_lost; A.coll += s.collisions; A.joins += s.joins;
  A.foff_ev += s.false_offline_events; A.foff_s += s.false_offline_s; A.frames += s.frames; A.all_hubs += s.frames_all_hubs;
  A.util += s.util_sum; A.sync = std::max(A.sync, s.max_sync_err); A.lead = std::min(A.lead, s.min_lead_margin); A.tail = std::min(A.tail, s.min_tail_margin);
  A.base_sum += s.base_age_sum; A.base_n += s.base_age_samples; A.base_max = std::max(A.base_max, s.base_age_max);
  A.fmade += s.focus_made; A.fgot += s.focus_got; A.dropped += s.dropped_slots; A.max_rel = std::max(A.max_rel, s.max_relayed);
  A.sil = std::max(A.sil, s.silence_prop_max);
  for (auto& kv : s.hub_slots) { A.slots[kv.first].first += kv.second.first; A.slots[kv.first].second += kv.second.second; }
}

static void printMargins() {
  Scenario sc; World w(sc, 1);
  printf("Link margins at SF9/500 (mean, no fading; chalet = C):\n| hub | dist to C (m) | margin to C (dB) | best hub link (dB) |\n|---|---|---|---|\n");
  for (int h = 0; h < NH; h++) {
    const double m = w.meanRssi(-1, h) - modeInfo(MODE_SF9_BW500).sens_dbm_x10 / 10.0;
    double best = -999; int bo = -1;
    for (int o = 0; o < NH; o++) if (o != h) { const double x = w.meanRssi(o, h) - modeInfo(MODE_SF9_BW500).sens_dbm_x10 / 10.0; if (x > best) { best = x; bo = o; } }
    printf("| %d | %.0f | %.1f | %.1f (hub %d) |\n", h + 1, w.dist(-1, h), m, best, bo + 1);
  }
  printf("\n");
}

// Variants: "current" = firmware as it is; "fixA" / "fixB" = candidate fixes (build with -DICEMESH_MAX_SLOTS=16).
static bool applyVariant(Scenario& s, const std::string& v) {
  if (v == "current") return true;
  if (MAX_SLOTS < 16) { fprintf(stderr, "variant %s needs -DICEMESH_MAX_SLOTS=16\n", v.c_str()); return false; }
  s.hub_timeout_s = 150; s.sonar_tries = 2;
  if (v == "fixA") { s.adaptive = true; return true; }                 // keep 1 s frames, faster modes where the margin allows
  if (v == "fixB") { s.frame_ms = 2000; s.adaptive = false; return true; }   // SF9 only, 2 s frames
  if (v == "fixAB") { s.frame_ms = 2000; s.adaptive = true; return true; }
  return false;
}

int main(int argc, char** argv) {
  const std::string which = argc > 1 ? argv[1] : "all";
  const int seeds = argc > 2 ? atoi(argv[2]) : 3;
  const int minutes = argc > 3 ? atoi(argv[3]) : 30;
  const std::string variant = argc > 4 ? argv[4] : "current";
  std::vector<Scenario> scs;
  { Scenario s; s.name = "baseline"; s.what = "good conditions"; scs.push_back(s); }
  { Scenario s; s.name = "degraded"; s.what = "+10 dB loss everywhere, bursts x5, ESP-NOW 50 %"; s.extra_db = 10; s.burst_p = 0.01; s.espnow_p = 0.5; scs.push_back(s); }
  { Scenario s; s.name = "storm"; s.what = "10 holes trip within 5 s at 12 min, FOCUS rotates incl. relayed sonar"; s.storm = true; scs.push_back(s); }
  { Scenario s; s.name = "failures"; s.what = "hub 3 cut from chalet 10-13 min, hub 7 reboots at 16:40, hub 7 carried away from 25 min"; s.failures = true; scs.push_back(s); }
  { Scenario s; s.name = "hostile"; s.what = "20 % extra packet loss, ESP-NOW 50 %, clocks +-50 ppm"; s.flat_loss = 0.2; s.espnow_p = 0.5; s.ppm = 50; scs.push_back(s); }
  { Scenario s; s.name = "small6"; s.what = "baseline with only hubs 1-6 (all direct)"; s.hubs_on = 6; scs.push_back(s); }
  { Scenario s; s.name = "small6_degr"; s.what = "degraded with only hubs 1-6"; s.hubs_on = 6; s.extra_db = 10; s.burst_p = 0.01; s.espnow_p = 0.5; scs.push_back(s); }
  { Scenario s; s.name = "no_tx_miss"; s.what = "hostile, but a hub never sends after a missed beacon"; s.flat_loss = 0.2; s.espnow_p = 0.5; s.ppm = 50; s.tx_after_miss = false; scs.push_back(s); }
  if (getenv("SIM_MARGINS")) printMargins();
  if (!getenv("SIM_NOHEAD")) {
    printf("| scenario | variant | trips | missed | alert p50/p95/max (s) | clear p95 (s) | false offline (node-min) | frames with all 10 hubs | hub delivery | FOCUS pings | sonar delay p95 (s) | BASE age avg/max (s) | timing losses | max sync err (us) |\n");
    printf("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");
  }
  for (auto sc : scs) {
    if (which != "all" && which != sc.name) continue;
    if (!applyVariant(sc, variant)) return 1;
    Agg A;
    for (int k = 1; k <= seeds; k++) runOne(sc, static_cast<uint64_t>(k) * 7919u, minutes, A);
    long sched = 0, got = 0; for (auto& kv : A.slots) { sched += kv.second.first; got += kv.second.second; }
    printf("| %s | %s | %ld | %ld | %.0f / %.0f / %.0f | %.0f | %.1f | %.0f %% | %.0f %% | %.0f %% | %.0f | %.1f / %.0f | %ld | %.0f |\n",
           sc.name.c_str(), variant.c_str(), A.trips, A.missed, pct(A.alert, 0.5), pct(A.alert, 0.95), pct(A.alert, 1.0), pct(A.clear, 0.95),
           A.foff_s / 60.0, 100 * A.all_hubs / A.frames, sched ? 100.0 * got / sched : 0,
           A.fmade ? 100.0 * A.fgot / A.fmade : 0, pct(A.son, 0.95), A.base_n ? A.base_sum / A.base_n : 0, A.base_max, A.timing, A.sync);
    fflush(stdout);
    if (getenv("SIM_DETAIL")) {
      printf("\n  %s per hub (delivered/scheduled): ", sc.name.c_str());
      for (auto& kv : A.slots) printf("H%d %ld/%ld  ", kv.first, kv.second.second, kv.second.first);
      printf("\n  joins %ld, slots with 2+ senders %ld, dropped slots %u, max relayed %d, lead margin min %.0f us, tail margin min %.0f us, silence reached all hubs in %.0f s\n\n",
             A.joins, A.coll, A.dropped, A.max_rel, A.lead, A.tail, A.sil);
    }
  }
  return 0;
}
