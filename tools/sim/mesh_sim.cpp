// Whole-system simulation on the PC, running the REAL protocol code of lib/IceMesh (ChaletRole, HubRole,
// planner, line table, ESP-NOW backbone + transport policy, sonar chain, codec, store).
// Two layouts: "pockets3" (Frank's field setup: 3 pockets x 3 holes, the hub is one of the holes, sonar
// on every hole) and "hubs10" (10 hubs x 3 tip-ups, 4 sonars, relayed hubs: the scaling case).
// What is simulated around the code (ALL ESTIMATES, to replace with field measurements):
//   - LoRa 915 MHz: log-distance path loss + per-link shadowing + obstacles, per-packet fading, burst outages,
//     success vs margin over the mode's theoretical sensitivity, collisions with capture, half duplex,
//     foreign LoRa traffic per channel (duty cycle, random strength), 8 channels;
//   - ESP-NOW 2.4 GHz backbone: log-distance path loss (exponent per scenario), sensitivity 1 Mbps / LR,
//     broadcast without retries, relays (max 2 hops) as configured;
//   - timing: hub clocks (ppm), RX jitter, TX start latency, listening windows (as in sim v1);
//   - tip-ups: v1 sensor behaviour (alert at once + every 5 s while tripped, reset seen at the next 5 s wake,
//     heartbeat 60 s, 5 tries per send), hub offline timeout (config);
//   - sonar: real SonarSource per sonar hole, FOCUS switched by the chalet every 2 min.
//
//   g++ -std=c++11 -O2 -Ilib/IceMesh/src tools/sim/mesh_sim.cpp -o mesh_sim && ./mesh_sim [scenario|all] [seeds] [minutes]
#include <hub_role.h>
#include <chalet_role.h>
#include <eb_link.h>
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
#include <set>

using namespace icemesh;
using namespace icemesh::tdma;

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed = 1) : s(seed * 0x9E3779B97F4A7C15ull + 1) {}
  uint64_t next() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
  double u() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double gauss() { double a = u(); if (a < 1e-12) a = 1e-12; return std::sqrt(-2 * std::log(a)) * std::cos(6.283185307179586 * u()); }
  bool chance(double p) { return u() < p; }
  double range(double a, double b) { return a + (b - a) * u(); }
};

// ---------------- model constants (estimates) ----------------
static const double LORA_TX = 20, LORA_ANT = 2, LORA_PL0 = 31.7, LORA_EXP = 3.5, CHALET_LORA_GAIN = 6;
static const double SHADOW_SIGMA = 4, FADE_SIGMA = 3, BURST_DB = 25, CAPTURE_DB = 6;
static const double PREAMBLE_TOL_US = 3 * 1024, RX_EXT_US = 2000;
static const double WIFI_TX = 20, WIFI_ANT = 0, WIFI_PL0 = 40.2, CHALET_WIFI_GAIN = 3;
static const double WIFI_SENS_1M = -98, WIFI_SENS_LR = -104;    // 802.11b 1 Mbps (datasheet) / LR 250 kbps (est.)
static const double FOREIGN_PKT_US = 80000;                      // a typical foreign LoRa packet (est.)
static const int CHALET_ID = 100;

struct Scenario {
  std::string name, layout = "pockets3", what;
  uint8_t transport = TR_AUTO;
  bool lr = false;
  double eb_exp = 3.0;                 // ESP-NOW path loss exponent (open ice ~2.5-3, people/snow ~3.5)
  std::vector<int> relay_hubs;         // hub indices with the relay setting on
  std::vector<std::pair<double, double> > relays;   // dedicated relay boards (x, y)
  double extra_db = 0, flat_loss = 0, wifi_loss = 0.03;
  double burst_p = 0.002, burst_exit = 0.25;
  double espnow_p = 0.85;              // tip-up -> hub single attempt
  double ppm = 20;
  double duty[LORA_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};   // foreign LoRa duty cycle per channel
  double foreign_dbm_min = -115, foreign_dbm_max = -85;
  double jam_from = -1, jam_to = -1;   // all LoRa channels unusable (radio fault / heavy jamming)
  double busy_ch0_from = -1;           // channel 0 becomes 60 % busy from this time
  bool auto_channel = true;
  double hub_timeout_s = 150;          // firmware v2 value (sim finding S3)
  int sonar_tries = 2;                 // firmware v2 value (sim finding S4)
  bool storm = false;
};

struct NodeSim {
  uint8_t id; int hub; bool hub_hole = false; bool sonar = false;
  bool tripped = false; double next_trip = 0, clear_at = 0, next_send = 0;
  uint8_t hub_state = LS_IDLE; double hub_heard = 0;
  double t_trip = -1, t_hub = -1, t_chalet = -1, t_clear = -1, t_chalet_clear = -1;
  sonar::SonarSource* src = nullptr;
};
struct HubSim {
  int idx; uint8_t id; double x, y, obst = 0;
  HubRole<24> role; eb::TransportPolicy pol;
  double ppm = 0, off_us = 0; bool alive = true;
  uint8_t ch = 0; double last_hop = 0, heard_net = -100;
  bool relay = false;
  uint16_t eb_seq = 0; eb::Dedup dedup;
  std::vector<int> nodes;
  bool used_eb = false;
};

struct Stats {
  std::vector<double> alert, clear, son;
  long trips = 0, missed = 0;
  double foff_s = 0, frames = 0, frames_all = 0, base_sum = 0, base_n = 0, base_max = 0, sync = 0, frame_ms_sum = 0;
  long fmade = 0, fgot = 0, timing = 0, ch_switch = 0;
  long pkt_lora = 0, pkt_eb = 0;        // hub packets delivered to the chalet per path
  double eb_time = 0;                   // hub-seconds with the backbone in use
};

static double pct(std::vector<double> v, double p) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  return v[static_cast<size_t>(p * (v.size() - 1) + 0.5)];
}

class World {
 public:
  Scenario sc; Rng rng; Stats st;
  ChaletRole<10, 32> chalet;
  sonar::SonarStore<32, 512> store;
  std::vector<HubSim> hubs;
  std::vector<NodeSim> nodes;
  int NH = 0, NR = 0;                  // hubs, dedicated relays
  std::vector<double> rx_, ry_;        // dedicated relay positions
  std::vector<eb::Dedup> relay_dedup;
  std::vector<std::vector<double> > shadow;
  std::vector<std::vector<bool> > burst;
  std::map<uint8_t, uint8_t> chalet_state;
  std::map<uint8_t, std::map<uint16_t, double> > ping_time;
  uint32_t focus_seq = 0; uint8_t focus = 0;
  double T = 0, now_s = 0;
  uint8_t chalet_ch = 0, cmd_seq = 1; int ch_countdown = -1; uint8_t ch_target = 0; double last_ch_eval = 0;
  uint16_t chalet_eb_seq = 0; eb::Dedup chalet_dedup;

  World(const Scenario& s, uint64_t seed) : sc(s), rng(seed) {
    chalet.planner.cfg.network_id = 0x42; chalet.planner.cfg.self_id = CHALET_ID;
    chalet.planner.cfg.allowance = 96; chalet.planner.cfg.test_allowance = 160;
    chalet.planner.cfg.frame_ms = 1000; chalet.planner.cfg.frame_ms_max = 2000;
    chalet.sonar_sink = &store;
    chalet.flags = BF_SONAR_SIM;
    std::vector<std::vector<double> > pos;   // x, y, obstacle dB to the chalet (LoRa)
    if (sc.layout == "pockets3") pos = {{250, 30, 0}, {450, 180, 0}, {650, -120, 0}};
    else pos = {{200, 50, 0}, {350, -120, 0}, {500, 80, 0}, {650, -60, 0}, {300, 250, 0}, {450, -300, 0},
                {800, 100, 0}, {900, 300, 32}, {-150, 420, 30}, {1400, 60, 6}};
    NH = static_cast<int>(pos.size());
    hubs.resize(NH);
    for (int h = 0; h < NH; h++) {
      HubSim& H = hubs[h];
      H.idx = h; H.id = static_cast<uint8_t>(h + 1); H.x = pos[h][0]; H.y = pos[h][1]; H.obst = pos[h][2];
      H.role.network_id = 0x42; H.role.self = H.id;
      H.ppm = rng.range(-sc.ppm, sc.ppm); H.off_us = rng.range(0, 3e8);
      H.ch = static_cast<uint8_t>(rng.next() % LORA_CHANNELS);   // hubs start anywhere and search
      H.pol.begin(0);
    }
    for (int k : sc.relay_hubs) if (k < NH) hubs[k].relay = true;
    NR = static_cast<int>(sc.relays.size());
    for (auto& r : sc.relays) { rx_.push_back(r.first); ry_.push_back(r.second); }
    relay_dedup.resize(NR);
    const int ND = 1 + NH + NR;
    shadow.assign(ND, std::vector<double>(ND, 0)); burst.assign(ND, std::vector<bool>(ND, false));
    for (int a = 0; a < ND; a++) for (int b = a + 1; b < ND; b++) shadow[a][b] = shadow[b][a] = rng.gauss() * SHADOW_SIGMA;
    // holes
    const bool p3 = sc.layout == "pockets3";
    for (int h = 0; h < NH; h++)
      for (int k = 0; k < 3; k++) {
        NodeSim n; n.hub = h;
        n.hub_hole = p3 && k == 0;
        n.id = n.hub_hole ? hubs[h].id : static_cast<uint8_t>(10 * (h + 1) + k);
        n.next_trip = rng.range(60, 600); n.next_send = rng.range(0, 60);
        n.sonar = p3 ? true : (k == 0 && (h == 2 || h == 7 || h == 4 || h == 9));
        hubs[h].nodes.push_back(static_cast<int>(nodes.size())); nodes.push_back(n);
      }
    for (auto& n : nodes) if (n.sonar) { n.src = new sonar::SonarSource(); n.src->begin(n.id, 1000u + n.id, static_cast<uint16_t>(rng.next())); }
    chalet_ch = pickChannel();
  }
  ~World() { for (auto& n : nodes) delete n.src; }

  // ---------------- geometry / devices: 0 = chalet, 1..NH hubs, NH+1.. relays ----------------
  double dx(int d) const { return d == 0 ? 0 : (d <= NH ? hubs[d - 1].x : rx_[d - 1 - NH]); }
  double dy(int d) const { return d == 0 ? 0 : (d <= NH ? hubs[d - 1].y : ry_[d - 1 - NH]); }
  double dist(int a, int b) const { return std::max(1.0, std::hypot(dx(a) - dx(b), dy(a) - dy(b))); }
  bool alive(int d) const { return d == 0 || d > NH || hubs[d - 1].alive; }

  // ---------------- LoRa ----------------
  double duty(uint8_t ch) const {
    if (sc.jam_from >= 0 && now_s >= sc.jam_from && now_s < sc.jam_to) return 1.0;
    if (ch == 0 && sc.busy_ch0_from >= 0 && now_s >= sc.busy_ch0_from) return 0.6;
    return sc.duty[ch];
  }
  double loraMean(int a, int b) const {
    double pl = LORA_PL0 + 10 * LORA_EXP * std::log10(dist(a, b));
    if (a == 0 || b == 0) { const int h = a == 0 ? b : a; pl += hubs[h - 1].obst; pl -= CHALET_LORA_GAIN; }
    return LORA_TX + 2 * LORA_ANT - pl - shadow[a][b] - sc.extra_db;
  }
  // packet a -> b on channel ch, mode m, airtime us: RSSI or NAN
  double lora(int a, int b, RadioMode m, uint8_t ch, double air_us) {
    if (!alive(a) || !alive(b)) return NAN;
    if (sc.jam_from >= 0 && now_s >= sc.jam_from && now_s < sc.jam_to) return NAN;   // LoRa unusable (strong jammer / radio fault)
    double r = loraMean(a, b) + rng.gauss() * FADE_SIGMA;
    if (burst[a][b]) r -= BURST_DB;
    const double margin = r - modeInfo(m).sens_dbm_x10 / 10.0;
    if (!rng.chance(1.0 / (1.0 + std::exp(-(margin - 1.0) / 0.7))) || rng.chance(sc.flat_loss)) return NAN;
    const double d = duty(ch);
    if (d > 0) {
      const double p_overlap = d >= 1 ? 1 : 1 - std::exp(-d * (air_us + FOREIGN_PKT_US) / FOREIGN_PKT_US);
      if (rng.chance(p_overlap) && r - rng.range(sc.foreign_dbm_min, sc.foreign_dbm_max) < CAPTURE_DB) return NAN;
    }
    return r;
  }
  uint8_t pickChannel() {   // chalet scan: quietest channel (measurement noise est.); keeps 915 MHz unless clearly busier
    uint8_t best = 0; double bd = duty(0) + rng.range(0, 0.03) - 0.05;
    for (uint8_t c = 1; c < LORA_CHANNELS; c++) { const double m = duty(c) + rng.range(0, 0.03); if (m < bd) { bd = m; best = c; } }
    return best;
  }
  // ---------------- ESP-NOW backbone ----------------
  double wifiOk(int a, int b) {
    if (!alive(a) || !alive(b)) return false;
    double pl = WIFI_PL0 + 10 * sc.eb_exp * std::log10(dist(a, b));
    double r = WIFI_TX + 2 * WIFI_ANT - pl - shadow[a][b] + (a == 0 || b == 0 ? CHALET_WIFI_GAIN : 0) + rng.gauss() * FADE_SIGMA;
    if (burst[a][b]) r -= BURST_DB;
    const double margin = r - (sc.lr ? WIFI_SENS_LR : WIFI_SENS_1M);
    return rng.chance(1.0 / (1.0 + std::exp(-(margin - 1.0) / 0.7))) && !rng.chance(sc.wifi_loss);
  }
  bool isRelay(int d) const { return d > NH || (d >= 1 && hubs[d - 1].relay); }
  // floods one backbone frame from device `from`; returns per device the hop count at which it got it (-1 = never)
  std::vector<int> flood(int from) {
    const int ND = 1 + NH + NR;
    std::vector<int> got(ND, -1); got[from] = 0;
    std::vector<int> senders = {from};
    for (int hop = 0; hop <= eb::MAX_HOPS && !senders.empty(); hop++) {
      std::vector<int> next;
      for (int s : senders)
        for (int d = 0; d < ND; d++) if (got[d] < 0 && wifiOk(s, d)) { got[d] = hop + 1; if (isRelay(d) && hop < eb::MAX_HOPS) next.push_back(d); }
      senders = next;
    }
    for (int d = 0; d < ND; d++) if (got[d] > 0) got[d] -= 1;   // relays passed through
    return got;
  }

  // ---------------- clocks ----------------
  double local(const HubSim& H, double t) const { return t * (1 + H.ppm * 1e-6) + H.off_us; }
  double trueOf(const HubSim& H, double l) const { return (l - H.off_us) / (1 + H.ppm * 1e-6); }
  uint32_t l32(double v) const { return static_cast<uint32_t>(static_cast<uint64_t>(v) & 0xFFFFFFFFull); }
  double unwrap(const HubSim& H, uint32_t l, double near_true) const {
    const double ln = local(H, near_true);
    return trueOf(H, ln + static_cast<double>(static_cast<int32_t>(l - l32(ln))));
  }

  // ---------------- tip-ups ----------------
  void deliver(NodeSim& n, double t) {
    n.hub_heard = t;
    const uint8_t s = n.tripped ? LS_TRIPPED : LS_IDLE;
    if (n.hub_state != s && n.tripped && n.t_hub < 0) n.t_hub = t;
    n.hub_state = s;
  }
  void stepNodes(double t1) {
    const double bundle = 1 - std::pow(1 - sc.espnow_p, 5);
    for (auto& n : nodes) {
      HubSim& H = hubs[n.hub];
      if (!n.tripped && t1 >= n.next_trip) {
        n.tripped = true; n.t_trip = n.next_trip; n.t_hub = n.t_chalet = n.t_clear = n.t_chalet_clear = -1;
        n.clear_at = n.t_trip + rng.range(20, 90); n.next_send = n.t_trip + (n.hub_hole ? 0.05 : 0.15);
        if (chalet_state.count(n.id) && chalet_state[n.id] == LS_TRIPPED) n.t_chalet = n.t_trip;
      }
      if (n.tripped && t1 >= n.clear_at) {
        n.tripped = false; n.t_clear = n.clear_at;
        n.next_send = n.clear_at + (n.hub_hole ? 0.05 : rng.range(0, 5) + 0.15);
        n.next_trip = n.clear_at + 60 + rng.range(0, 1) * 600;
      }
      while (n.next_send <= t1) {
        const double t = n.next_send;
        if (H.alive && (n.hub_hole || rng.chance(bundle))) deliver(n, t);
        n.next_send = t + (n.hub_hole ? 1.0 : (n.tripped ? 5.0 : 60.0));
      }
      uint8_t view = n.hub_state;
      if (!n.hub_hole && t1 - n.hub_heard > sc.hub_timeout_s) view = LS_OFFLINE;
      if (H.alive) H.role.table.observe(n.id, view, 0, 0, 90, static_cast<uint32_t>(t1 * 1000));
    }
  }
  void stepSonar(double t, double dt) {
    const int ticks = static_cast<int>(dt * 4 + 0.5);
    for (auto& n : nodes) {
      if (!n.src) continue;
      HubSim& H = hubs[n.hub];
      if (!H.alive) continue;
      const bool f = H.role.focusNode() == n.id;
      for (int k = 0; k < ticks; k++) {
        sonar::Block out[2];
        const uint8_t c = n.src->tick(f, out, 2);
        ping_time[n.id][n.src->lastPing().index] = t + 0.25 * k;
        if (n.id == focus) st.fmade++;
        for (uint8_t i = 0; i < c; i++)
          if (n.hub_hole || rng.chance(1 - std::pow(1 - sc.espnow_p, sc.sonar_tries))) H.role.sonar.push(out[i].data, out[i].len, H.role.frameNow());
      }
    }
  }

  // ---------------- chalet channel management ----------------
  void manageChannel(uint16_t f) {
    if (ch_countdown >= 0) {
      chalet.cmd = CMD_SET_CHANNEL; chalet.cmd_seq = cmd_seq; chalet.cmd_target = static_cast<uint8_t>(ch_countdown); chalet.cmd_value = ch_target;
      if (ch_countdown == 0) { chalet_ch = ch_target; ch_countdown = -1; st.ch_switch++; }
      else ch_countdown--;
      return;
    }
    chalet.cmd = CMD_NONE;
    if (!sc.auto_channel || now_s - last_ch_eval < 60) return;
    last_ch_eval = now_s;
    const double cur = duty(chalet_ch);
    if (cur > 0.25) {
      const uint8_t best = pickChannel();
      if (best != chalet_ch && duty(best) < cur - 0.15) { ch_target = best; ch_countdown = 5; cmd_seq++; }
    }
    (void)f;
  }

  // ---------------- one superframe ----------------
  void frame(uint16_t f) {
    for (size_t a = 0; a < burst.size(); a++) for (size_t b = a + 1; b < burst.size(); b++) {
      bool s = burst[a][b];
      if (s) { if (rng.chance(sc.burst_exit)) s = false; } else if (rng.chance(sc.burst_p)) s = true;
      burst[a][b] = burst[b][a] = s;
    }
    if (static_cast<int>(now_s) / 120 != static_cast<int>(now_s - 2) / 120 || f == 30) {
      std::vector<uint8_t> s; for (auto& n : nodes) if (n.src) s.push_back(n.id);
      if (!s.empty()) { focus = s[(static_cast<int>(now_s) / 120) % s.size()]; chalet.focus_node = focus; }
    }
    if (sc.storm && std::fabs(now_s - 720) < 1.0) for (auto& n : nodes) if (!n.tripped && rng.chance(0.5)) n.next_trip = now_s + rng.range(0, 5);
    manageChannel(f);
    chalet.net_cfg = makeNetCfg(sc.transport, sc.lr, chalet_ch);

    uint8_t bbuf[MAX_PACKET];
    const size_t blen = chalet.startFrame(f, bbuf, sizeof bbuf);
    const Beacon plan = chalet.beacon;
    const double frame_us = plan.frame_10ms * 10000.0;
    SlotTime times[MAX_SLOTS]; memcpy(times, chalet.times, sizeof times);
    const double dt = frame_us / 1e6;
    stepNodes(now_s);
    stepSonar(now_s, dt);
    st.frames++; st.frame_ms_sum += frame_us / 1000;
    const bool lora_on = sc.transport != TR_ESPNOW;
    int hub_slots = 0;
    for (uint8_t i = 0; i < plan.n_slots; i++) if (plan.slots[i].kind == SLOT_HUB) hub_slots++;
    int alive_hubs = 0; for (auto& H : hubs) alive_hubs += H.alive;
    if (!lora_on || hub_slots >= alive_hubs) st.frames_all++;
    if (getenv("SIM_PLAN") && now_s > 120) {
      fprintf(stderr, "frame %u ms hubslots %d slots %u allow %u missing:", plan.frame_10ms * 10u, hub_slots, plan.n_slots, chalet.planner.lastAllowance());
      for (int h = 0; h < NH; h++) { bool in = false; for (uint8_t i = 0; i < plan.n_slots; i++) in |= plan.slots[i].kind == SLOT_HUB && plan.slots[i].owner == hubs[h].id;
        if (!in) { const HubInfo* hi = chalet.planner.find(hubs[h].id); fprintf(stderr, " H%d(%s,ch%u/%u,eb%d)", h + 1, hi ? "known" : "unknown", hubs[h].ch, chalet_ch, hubs[h].used_eb); } }
      fprintf(stderr, "\n");
    }

    // ---- LoRa: beacon, echo, slots ----
    std::vector<bool> fresh(NH, false);
    if (lora_on) {
      const double b_air = beaconAirtimeUs(static_cast<uint16_t>(blen));
      for (int h = 0; h < NH; h++) {
        HubSim& H = hubs[h];
        if (!H.alive || !H.pol.useLora(H.role.transport(), ms())) continue;
        if (H.ch != chalet_ch) continue;
        if (H.role.sync.synced()) {
          const double exp_ref = unwrap(H, H.role.sync.ref() + H.role.sync.frameUs(), T);
          if (std::fabs(exp_ref - T) > BEACON_WINDOW_US) continue;
        }
        const double r = lora(0, h + 1, MODE_SF9_BW500, chalet_ch, b_air);
        if (std::isnan(r)) continue;
        uint8_t cmd;
        fresh[h] = H.role.onBeacon(bbuf, blen, l32(local(H, T + b_air + 20 + rng.gauss() * 10)), static_cast<int8_t>(std::max(-128.0, r)), 20, cmd);
        if (fresh[h]) { H.pol.onLoraBeacon(ms()); st.sync = std::max(st.sync, std::fabs(unwrap(H, H.role.sync.ref(), T) - T)); }
      }
      bool missed_done = false;
      auto doMissed = [&]() { if (missed_done) return; missed_done = true; for (int h = 0; h < NH; h++) if (hubs[h].alive && !fresh[h]) hubs[h].role.onBeaconMissed(); };
      for (uint8_t i = 0; i < plan.n_slots; i++) {
        const Slot& s = plan.slots[i];
        if (s.kind != SLOT_ECHO) doMissed();
        const RadioMode m = slotMode(s);
        struct Tx { int h; uint8_t buf[MAX_PACKET]; size_t len; double t0, t1; bool echo; };
        std::vector<Tx> txs;
        for (int h = 0; h < NH; h++) {
          HubSim& H = hubs[h];
          if (!H.alive || H.ch != chalet_ch || !H.role.have_plan || i >= H.role.plan.n_slots) continue;
          if (!H.pol.useLora(H.role.transport(), ms())) continue;
          const SlotAction a = H.role.action(i, rng.chance(1.0 / 3));
          if (a != ACT_TX_HUB && a != ACT_TX_ECHO && a != ACT_TX_JOIN) continue;
          if (a == ACT_TX_ECHO && !fresh[h]) continue;
          Tx x; x.h = h; x.echo = a == ACT_TX_ECHO;
          if (a == ACT_TX_HUB) x.len = H.role.buildHub(x.buf, MAX_PACKET, 0, 4000, 1, false, 0);
          else if (a == ACT_TX_ECHO) x.len = H.role.buildEcho(x.buf, MAX_PACKET);
          else x.len = H.role.buildJoin(x.buf, MAX_PACKET);
          if (!x.len) continue;
          const double l_tx = static_cast<double>(H.role.sync.ref()) + H.role.times[i].tx + (a == ACT_TX_JOIN ? rng.range(0, 2000) : 0);
          x.t0 = unwrap(H, l32(l_tx), T + H.role.times[i].tx) + rng.range(150, 400);
          x.t1 = x.t0 + modeAirtimeUs(slotMode(H.role.plan.slots[i]), static_cast<uint16_t>(x.len));
          txs.push_back(x);
        }
        auto receive = [&](int rxd) -> int {
          std::vector<std::pair<double, int> > got;
          for (size_t k = 0; k < txs.size(); k++) {
            if (txs[k].h + 1 == rxd) return -1;
            const double r = lora(txs[k].h + 1, rxd, m, chalet_ch, txs[k].t1 - txs[k].t0);
            if (!std::isnan(r)) got.push_back(std::make_pair(r, static_cast<int>(k)));
          }
          if (got.empty()) return -1;
          std::sort(got.rbegin(), got.rend());
          if (got.size() > 1 && got[0].first - got[1].first < CAPTURE_DB) return -1;
          return got[0].second;
        };
        if (s.kind != SLOT_ECHO) {
          const int k = receive(0);
          bool ok = false;
          if (k >= 0) {
            const Tx& x = txs[k];
            if (x.t0 < T + times[i].start - PREAMBLE_TOL_US || x.t1 > T + times[i].end + RX_EXT_US) st.timing++;
            else {
              ChaletRxResult res;
              if (chalet.onSlotPacket(i, x.buf, x.len, -100, 20, res)) { applyChanges(res.changes, res.n_changes); if (!x.echo) st.pkt_lora++; }
              ok = true;
            }
          }
          if (!ok) chalet.onSlotEmpty(i, false);
        }
        for (int h = 0; h < NH; h++) {
          HubSim& H = hubs[h];
          if (!H.alive || H.ch != chalet_ch) continue;
          const bool listens = H.role.have_plan ? (i < H.role.plan.n_slots && H.role.action(i, false) == ACT_RX) : true;
          if (!listens) continue;
          const int k = receive(h + 1);
          if (k < 0) continue;
          const Tx& x = txs[k];
          H.heard_net = now_s;
          if (x.echo) { if (!fresh[h]) { uint8_t cmd; fresh[h] = H.role.onBeacon(x.buf, x.len, l32(local(H, x.t1 + 20)), -100, 0, cmd); if (fresh[h]) H.pol.onLoraBeacon(ms()); } }
          else H.role.onSlotPacket(i, x.buf, x.len, -100, 20);
        }
      }
      doMissed();
      // unsynced hubs: ALOHA JOIN, channel search (a hub hops to the next channel every ~2.5 s without a beacon)
      for (int h = 0; h < NH; h++) {
        HubSim& H = hubs[h];
        if (!H.alive) continue;
        if (!H.role.sync.synced() && now_s - H.last_hop > 2.5 && now_s - H.heard_net > 10) { H.ch = static_cast<uint8_t>((H.ch + 1) % LORA_CHANNELS); H.last_hop = now_s; }
        if (H.role.wantsAsyncJoin() && H.ch == chalet_ch && rng.chance(1.0 / 3)) {
          uint8_t jb[MAX_PACKET]; const size_t jl = H.role.buildJoin(jb, sizeof jb);
          const double util = plan.n_slots ? times[plan.n_slots - 1].end / frame_us : 0.1;
          if (!rng.chance(util) && !std::isnan(lora(h + 1, 0, MODE_SF9_BW500, chalet_ch, 31000))) chalet.onAsyncPacket(jb, jl, -110);
        }
        H.role.endFrame();
        // channel switch announced in the beacon: follow it
        if (H.role.channel_switch_pending && seqDiff(static_cast<uint16_t>(f + 1), H.role.channel_switch_frame) >= 0) {
          H.ch = H.role.channel_next; H.role.channel_switch_pending = false;
        }
      }
    } else {
      for (int h = 0; h < NH; h++) if (hubs[h].alive) hubs[h].role.onBeaconMissed();
    }

    // ---- ESP-NOW backbone: chalet beacon, then hub packets (flooded through relays) ----
    if (sc.transport != TR_LORA || true) {
      uint8_t eb_pl[MAX_PACKET];
      const size_t eb_len = chalet.buildEbBeacon(eb_pl, sizeof eb_pl);
      const bool send_beacon = sc.transport != TR_LORA;
      if (send_beacon) {
        chalet_eb_seq++;
        const std::vector<int> got = flood(0);
        for (int h = 0; h < NH; h++) if (got[h + 1] >= 0) {
          HubSim& H = hubs[h]; uint8_t cmd;
          if (!H.dedup.seen(CHALET_ID, eb::MSG_EB_BEACON, chalet_eb_seq) && H.role.onEbBeacon(eb_pl, eb_len, cmd)) {
            H.pol.onEbBeacon(ms());
            if (!H.role.sync.synced()) H.ch = netChannel(H.role.info_net_cfg);   // backbone beacon gives the LoRa channel
          }
          if (H.role.channel_switch_pending && seqDiff(static_cast<uint16_t>(f + 1), H.role.channel_switch_frame) >= 0) { H.ch = H.role.channel_next; H.role.channel_switch_pending = false; }
        }
      }
      for (int h = 0; h < NH; h++) {
        HubSim& H = hubs[h];
        if (!H.alive) continue;
        H.pol.tick(ms());
        const bool use = H.pol.useEb(H.role.transport(), ms());
        H.used_eb = use;
        if (!use) continue;
        st.eb_time += dt;
        uint8_t hp[MAX_PACKET];
        const size_t hl = H.role.buildHubFree(hp, eb::MAX_PAYLOAD, 0, 4000, 1, H.role.frameNow());
        if (!hl) continue;
        H.eb_seq++;
        const std::vector<int> got = flood(h + 1);
        if (got[0] >= 0 && !chalet_dedup.seen(H.id, eb::MSG_EB_HUB, H.eb_seq)) {
          ChaletRxResult res;
          if (chalet.onEbHubPacket(hp, hl, static_cast<uint8_t>(got[0]), res)) { applyChanges(res.changes, res.n_changes); st.pkt_eb++; }
        }
      }
    }

    if (getenv("SIM_HUB") && now_s >= atof(getenv("SIM_T0")) && now_s <= atof(getenv("SIM_T1"))) {
      const int h = atoi(getenv("SIM_HUB")) - 1; const HubSim& H = hubs[h];
      const HubInfo* hi = chalet.planner.find(H.id);
      int slot = -1, via = -1; for (uint8_t i = 0; i < plan.n_slots; i++) if (plan.slots[i].kind == SLOT_HUB && plan.slots[i].owner == H.id) { slot = i; via = plan.slots[i].via; }
      int echo_owner = -1; for (uint8_t i = 0; i < plan.n_slots; i++) if (plan.slots[i].kind == SLOT_ECHO) echo_owner = plan.slots[i].owner;
      fprintf(stderr, "t=%.0f H%d slot=%d via=%d echo=%d fresh=%d plan=%d synced=%d info: via=%d last_any=%d frame=%u relay_cand=%d pending=%d\n", now_s, h + 1, slot, via, echo_owner,
              (int)fresh[h], H.role.have_plan, H.role.sync.synced(), hi ? hi->via : -2, hi ? (int)hi->last_any : -1, f, hi ? hi->relay_cand : -1, (int)H.role.table.anyPending());
    }
    NodeChange ex[16];
    applyChanges(ex, chalet.expireNodes(ex, 16));
    metrics();
    T += frame_us; now_s = T / 1e6;
  }
  uint32_t ms() const { return static_cast<uint32_t>(now_s * 1000); }

  void applyChanges(const NodeChange* ch, uint8_t n) {
    for (uint8_t k = 0; k < n; k++) {
      const uint8_t id = ch[k].node, ns = ch[k].new_state;
      chalet_state[id] = ns;
      for (auto& nd : nodes) if (nd.id == id) {
        if (ns == LS_TRIPPED && nd.t_trip >= 0 && nd.t_chalet < 0) nd.t_chalet = now_s;
        if (ns == LS_IDLE && nd.t_clear >= 0 && nd.t_chalet_clear < 0) nd.t_chalet_clear = now_s;
      }
    }
  }
  void metrics() {
    if (focus) {
      const sonar::StoredPing* out[512];
      const uint16_t n = store.pingsSince(focus, focus_seq, out, 512);
      for (uint16_t k = 0; k < n; k++) {
        st.fgot++;
        auto& m = ping_time[focus]; auto it = m.find(out[k]->p.index);
        if (it != m.end()) st.son.push_back(now_s - it->second);
        focus_seq = std::max(focus_seq, out[k]->seq);
      }
      if (n == 0) focus_seq = std::max(focus_seq, store.lastSeq());
    }
    for (auto& nd : nodes) {
      if (!nd.src || nd.id == focus || now_s < 60) continue;
      const sonar::NodeSonar* ns = store.find(nd.id);
      if (!ns) continue;
      const double age = std::max(0.0, now_s - frameTime(ns->frame));
      st.base_sum += age; st.base_n++; st.base_max = std::max(st.base_max, age);
    }
    for (auto& nd : nodes) {
      auto it = chalet_state.find(nd.id);
      if (it != chalet_state.end() && it->second == LS_OFFLINE && hubs[nd.hub].alive) st.foff_s += chalet.beacon.frame_10ms / 100.0;
    }
  }
  std::map<uint16_t, double> frame_t;
  double frameTime(uint16_t f) { auto it = frame_t.find(f); return it == frame_t.end() ? now_s : it->second; }
  void markFrame(uint16_t f) { frame_t[f] = now_s; if (frame_t.size() > 4000) frame_t.erase(frame_t.begin()); }
};

struct Agg { Stats s; long trips = 0, missed = 0; };

static void runOne(const Scenario& sc, uint64_t seed, int minutes, Agg& A) {
  World w(sc, seed);
  std::vector<double> last(w.nodes.size(), -1);
  const double end_s = minutes * 60.0;
  for (uint16_t f = 1; w.now_s < end_s; f++) {
    w.markFrame(f);
    w.frame(f);
    for (size_t k = 0; k < w.nodes.size(); k++) {
      NodeSim& n = w.nodes[k];
      if (n.t_trip >= 0 && n.t_trip != last[k] && ((n.t_clear >= 0 && w.now_s > n.t_clear + 60) || w.now_s >= end_s)) {
        last[k] = n.t_trip;
        if (n.t_trip < 90 || n.t_trip > end_s - 120) continue;
        A.trips++;
        if (n.t_chalet < 0) A.missed++; else A.s.alert.push_back(n.t_chalet - n.t_trip);
        if (n.t_clear >= 0 && n.t_chalet_clear >= 0) A.s.clear.push_back(n.t_chalet_clear - n.t_clear);
        if (getenv("SIM_LATE") && (n.t_chalet < 0 || n.t_chalet - n.t_trip > 10))
          fprintf(stderr, "late node %u hub %d trip %.0f s: hub %.1f s, chalet %.1f s, hub on backbone=%d ch hub/chalet %u/%u\n", n.id, n.hub + 1, n.t_trip,
                  n.t_hub >= 0 ? n.t_hub - n.t_trip : -1.0, n.t_chalet >= 0 ? n.t_chalet - n.t_trip : -1.0, w.hubs[n.hub].used_eb, w.hubs[n.hub].ch, w.chalet_ch);
      }
    }
  }
  Stats& s = w.st; Stats& a = A.s;
  a.son.insert(a.son.end(), s.son.begin(), s.son.end());
  a.foff_s += s.foff_s; a.frames += s.frames; a.frames_all += s.frames_all; a.frame_ms_sum += s.frame_ms_sum;
  a.base_sum += s.base_sum; a.base_n += s.base_n; a.base_max = std::max(a.base_max, s.base_max); a.sync = std::max(a.sync, s.sync);
  a.fmade += s.fmade; a.fgot += s.fgot; a.timing += s.timing; a.ch_switch += s.ch_switch;
  a.pkt_lora += s.pkt_lora; a.pkt_eb += s.pkt_eb; a.eb_time += s.eb_time;
}

static std::vector<Scenario> scenarios() {
  std::vector<Scenario> v;
  auto add = [&](Scenario s) { v.push_back(s); };
  // ---- Frank's field setup: 3 pockets x 3 holes ----
  { Scenario s; s.name = "p3_lora"; s.what = "3 pockets, quiet LoRa, Auto"; add(s); }
  { Scenario s; s.name = "p3_ch0_busy"; s.what = "channel 915 MHz gets 60 % foreign traffic at 5 min; chalet moves the network"; s.busy_ch0_from = 300; s.auto_channel = true; add(s); }
  { Scenario s; s.name = "p3_ch0_busy_fixed"; s.what = "same, channel change off"; s.busy_ch0_from = 300; s.auto_channel = false; add(s); }
  { Scenario s; s.name = "p3_all_busy"; s.what = "every LoRa channel 35 % foreign traffic, Auto, relays: hub 1 + 1 board"; for (int c = 0; c < 8; c++) s.duty[c] = 0.35; s.relay_hubs = {0}; s.relays = {{330, 0}}; add(s); }
  { Scenario s; s.name = "p3_lora_out"; s.what = "LoRa unusable 10-20 min, Auto, relays: hub 1 + 1 board"; s.jam_from = 600; s.jam_to = 1200; s.relay_hubs = {0}; s.relays = {{330, 0}}; add(s); }
  { Scenario s; s.name = "p3_lora_out_norelay"; s.what = "LoRa unusable 10-20 min, Auto, no relay"; s.jam_from = 600; s.jam_to = 1200; add(s); }
  { Scenario s; s.name = "p3_espnow"; s.what = "ESP-NOW only, relays: hub 1 + 1 board"; s.transport = TR_ESPNOW; s.relay_hubs = {0}; s.relays = {{330, 0}}; add(s); }
  { Scenario s; s.name = "p3_espnow_lr"; s.what = "ESP-NOW only + LR, relays: hub 1 + 1 board"; s.transport = TR_ESPNOW; s.lr = true; s.relay_hubs = {0}; s.relays = {{330, 0}}; add(s); }
  { Scenario s; s.name = "p3_espnow_norelay"; s.what = "ESP-NOW only, no relay"; s.transport = TR_ESPNOW; add(s); }
  { Scenario s; s.name = "p3_espnow_pess"; s.what = "ESP-NOW only, relays, pessimistic 2.4 GHz (people, snow)"; s.transport = TR_ESPNOW; s.eb_exp = 3.5; s.relay_hubs = {0}; s.relays = {{330, 0}}; add(s); }
  { Scenario s; s.name = "p3_espnow_pess_lr"; s.what = "ESP-NOW only + LR, relays, pessimistic 2.4 GHz"; s.transport = TR_ESPNOW; s.lr = true; s.eb_exp = 3.5; s.relay_hubs = {0}; s.relays = {{330, 0}}; add(s); }
  { Scenario s; s.name = "p3_degraded"; s.what = "+10 dB LoRa loss, bursts x5, tip-up ESP-NOW 50 %"; s.extra_db = 10; s.burst_p = 0.01; s.espnow_p = 0.5; add(s); }
  // ---- scaling: 10 hubs ----
  { Scenario s; s.name = "h10_lora"; s.layout = "hubs10"; s.what = "10 hubs, quiet LoRa (frame length chosen by the chalet)"; add(s); }
  { Scenario s; s.name = "h10_degraded"; s.layout = "hubs10"; s.what = "10 hubs, +10 dB, bursts x5, ESP-NOW 50 %"; s.extra_db = 10; s.burst_p = 0.01; s.espnow_p = 0.5; add(s); }
  { Scenario s; s.name = "h10_storm"; s.layout = "hubs10"; s.what = "10 hubs, half the holes trip within 5 s at 12 min"; s.storm = true; add(s); }
  { Scenario s; s.name = "h10_busy"; s.layout = "hubs10"; s.what = "10 hubs, every channel 20 % foreign traffic"; for (int c = 0; c < 8; c++) s.duty[c] = 0.2; add(s); }
  return v;
}

int main(int argc, char** argv) {
  const std::string which = argc > 1 ? argv[1] : "all";
  const int seeds = argc > 2 ? atoi(argv[2]) : 3;
  const int minutes = argc > 3 ? atoi(argv[3]) : 30;
  if (!getenv("SIM_NOHEAD")) {
    printf("| scenario | trips | missed | alert p50/p95/max (s) | clear p95 (s) | false offline (node-min) | frames all hubs | frame (ms) | hub pkts LoRa / backbone | hub-time on backbone | FOCUS pings | sonar delay p95 (s) | BASE age avg/max (s) | channel moves |\n");
    printf("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n");
  }
  for (auto& sc : scenarios()) {
    const bool prefix = !which.empty() && which[which.size() - 1] == '*';
    if (which != "all" && which != sc.name && !(prefix && sc.name.compare(0, which.size() - 1, which, 0, which.size() - 1) == 0)) continue;
    Agg A;
    const int k0 = getenv("SIM_SEED") ? atoi(getenv("SIM_SEED")) : 1;
    for (int k = k0; k < k0 + seeds; k++) runOne(sc, static_cast<uint64_t>(k) * 7919u, minutes, A);
    const Stats& s = A.s;
    const double hub_time = minutes * 60.0 * seeds * (sc.layout == "pockets3" ? 3 : 10);
    printf("| %s | %ld | %ld | %.0f / %.0f / %.0f | %.0f | %.1f | %.0f %% | %.0f | %ld / %ld | %.0f %% | %.0f %% | %.0f | %.1f / %.0f | %ld |\n",
           sc.name.c_str(), A.trips, A.missed, pct(s.alert, 0.5), pct(s.alert, 0.95), pct(s.alert, 1.0), pct(s.clear, 0.95), s.foff_s / 60.0,
           100 * s.frames_all / s.frames, s.frame_ms_sum / s.frames, s.pkt_lora, s.pkt_eb, 100 * s.eb_time / hub_time,
           s.fmade ? 100.0 * s.fgot / s.fmade : 0, pct(s.son, 0.95), s.base_n ? s.base_sum / s.base_n : 0, s.base_max, s.ch_switch);
    fflush(stdout);
  }
  if (getenv("SIM_SCENARIOS")) { printf("\n"); for (auto& sc : scenarios()) printf("| %s | %s |\n", sc.name.c_str(), sc.what.c_str()); }
  return 0;
}
