// Chalet side: who is out there, how to reach them, and the slot plan of the next beacon.
//
// - A hub heard directly in the last DIRECT_WINDOW frames gets a direct slot.
// - A hub not heard directly but reported by a direct hub (relayed packet, neighbour list or
//   JOIN) is reached through that hub: the relay gets an ECHO slot (repeats the beacon), the
//   remote hub gets a slot in SF9/500, and the relay's own slot (later in the frame) grows to
//   carry the remote packet. One relay hop only.
// - One JOIN slot per frame lets unknown hubs announce themselves.
// - Direct hubs: fixed SF9/500, adaptive (RSSI margin + hysteresis), or radio-test modes.
// - Event acks (hub, node, seq) are repeated in ACK_REPEAT beacons.
// All thresholds are ESTIMATES to tune with the radio test mode.
#ifndef ICEMESH_CHALET_PLANNER_H
#define ICEMESH_CHALET_PLANNER_H
#include <stdint.h>
#include <string.h>
#include "seq.h"
#include "radio_modes.h"
#include "tdma_proto.h"
#include "tdma_schedule.h"

namespace icemesh {
namespace tdma {

struct ModeStats {
  uint32_t scheduled, received, crc_err;
  int32_t rssi_sum, snr_q4_sum;
  int8_t rssi_min, rssi_max, snr_min_q4, snr_max_q4;
};

struct HubInfo {
  bool used;
  uint8_t id;
  uint16_t last_direct, last_any;
  bool ever_direct;
  uint8_t direct_hits;         // direct receptions in the current window (hysteresis remote -> direct)
  uint8_t via;                 // 0 = direct, else relay hub id
  uint8_t mode;                // adaptive mode for direct slots
  uint8_t hist;                // last 8 direct-slot outcomes in this mode, bit0 = latest, 1 = received
  uint8_t hist_n;
  uint8_t miss_streak;
  uint16_t hold_until;
  int8_t win_rssi[8];
  int8_t last_rssi, last_snr_q4;
  uint8_t relay_cand; int8_t relay_cand_rssi; uint16_t relay_cand_frame;
  bool has_health; uint16_t health_frame; Health health;
  ModeStats stats[MODE_COUNT];
};

struct PlannerConfig {
  uint8_t network_id;
  uint8_t self_id;
  uint8_t test_mode;        // TestMode
  bool adaptive;
  uint8_t allowance;        // bytes per hub slot, normal operation
  uint8_t test_allowance;   // bytes per hub slot in test modes
  uint16_t frame_ms;
};

template <uint8_t MAX_HUBS = 10>
class ChaletPlanner {
 public:
  enum {
    HUB_TIMEOUT_FRAMES = 30,
    DIRECT_WINDOW = 5,
    RELAY_CAND_TIMEOUT = 10,
    ACK_REPEAT = 3,
    MAX_RELAYS = 3,
    MIN_ALLOWANCE = 40,
    RELAY_SWITCH_DB = 6,      // est. hysteresis between relay candidates
    DIRECT_HITS_TO_RETURN = 3,// a remote hub needs this many direct receptions in DIRECT_WINDOW to go direct
    JOIN_LEN = 7,
    UP_MARGIN_DB = 12,        // est.: step to a faster mode when the weakest of 8 packets keeps this margin
    UP_HOLD_FRAMES = 20,
    DOWN_HOLD_FRAMES = 30,
  };

  PlannerConfig cfg;

  ChaletPlanner() {
    memset(&cfg, 0, sizeof(cfg));
    cfg.allowance = 96; cfg.test_allowance = 160; cfg.frame_ms = 1000;
    for (uint8_t i = 0; i < MAX_HUBS; i++) hubs_[i].used = false;
    n_acks_ = 0;
  }

  // ---- inputs ----------------------------------------------------------------------------
  // Hub packet received directly. in_own_slot: it arrived in the slot planned for this hub.
  void onDirectPacket(uint8_t hub, uint16_t frame, int8_t rssi, int8_t snr_q4, uint8_t mode, bool in_own_slot) {
    HubInfo* h = get(hub, frame);
    if (h == nullptr) return;
    noteDirect(*h, frame);
    h->last_rssi = rssi; h->last_snr_q4 = snr_q4;
    if (!in_own_slot || !validMode(mode)) return;
    ModeStats& s = h->stats[mode];
    if (s.received == 0) { s.rssi_min = s.rssi_max = rssi; s.snr_min_q4 = s.snr_max_q4 = snr_q4; }
    s.scheduled++; s.received++; s.rssi_sum += rssi; s.snr_q4_sum += snr_q4;
    if (rssi < s.rssi_min) s.rssi_min = rssi;
    if (rssi > s.rssi_max) s.rssi_max = rssi;
    if (snr_q4 < s.snr_min_q4) s.snr_min_q4 = snr_q4;
    if (snr_q4 > s.snr_max_q4) s.snr_max_q4 = snr_q4;
    if (mode == h->mode) {
      h->hist = static_cast<uint8_t>((h->hist << 1) | 1);
      h->win_rssi[h->hist_n % 8] = rssi;
      if (h->hist_n < 255) h->hist_n++;
      h->miss_streak = 0;
      adaptUp(*h, frame);
    }
  }

  // A direct slot of `hub` passed without a valid packet.
  void onSlotMissed(uint8_t hub, uint16_t frame, uint8_t mode, bool crc_error) {
    HubInfo* h = findMut(hub);
    if (h == nullptr || !validMode(mode)) return;
    ModeStats& s = h->stats[mode];
    s.scheduled++;
    if (crc_error) s.crc_err++;
    if (mode == h->mode) {
      h->hist = static_cast<uint8_t>(h->hist << 1);
      if (h->hist_n < 255) h->hist_n++;
      h->miss_streak++;
      if (cfg.adaptive && cfg.test_mode == TEST_OFF && h->miss_streak >= 2 && h->mode != MODE_SF9_BW500) {
        setMode(*h, slowerMode(static_cast<RadioMode>(h->mode)), frame, DOWN_HOLD_FRAMES);
      }
    }
  }

  void onRelayed(uint8_t hub, uint8_t via, uint16_t frame) {
    HubInfo* h = get(hub, frame);
    if (h == nullptr) return;
    h->last_any = frame;
    noteRelayCandidate(*h, via, 0, frame, true);
  }

  // `reporter` hears `heard` (from reporter's health neighbour list or join list).
  void onNeighbor(uint8_t reporter, uint8_t heard, int8_t rssi, uint16_t frame) {
    if (reporter == heard || heard == cfg.self_id || heard == ID_NONE) return;
    HubInfo* h = findMut(heard);
    if (h == nullptr) return;        // neighbour lists only help known hubs (joins create them)
    noteRelayCandidate(*h, reporter, rssi, frame, false);
  }

  // JOIN heard directly (via = 0) or reported by relay hub `via`.
  void onJoin(uint8_t hub, uint8_t via, int8_t rssi, uint16_t frame) {
    if (hub == ID_NONE || hub == cfg.self_id) return;
    HubInfo* h = get(hub, frame);
    if (h == nullptr) return;
    h->last_any = frame;
    if (via == 0) { noteDirect(*h, frame); h->last_rssi = rssi; }
    else noteRelayCandidate(*h, via, rssi, frame, true);
  }

  void onHealth(uint8_t hub, const Health& hl, uint16_t frame) {
    HubInfo* h = findMut(hub);
    if (h == nullptr) return;
    h->health = hl; h->has_health = true; h->health_frame = frame;
    for (uint8_t i = 0; i < hl.n_nb; i++) onNeighbor(hub, hl.nb[i].hub, hl.nb[i].rssi, frame);
  }

  void addAck(uint8_t hub, uint8_t node, uint8_t seq) {
    for (uint8_t i = 0; i < n_acks_; i++)
      if (acks_[i].a.hub == hub && acks_[i].a.node == node) { acks_[i].a.seq = seq; acks_[i].ttl = ACK_REPEAT; return; }
    if (n_acks_ >= MAX_ACKS) {                 // drop the one with the least repeats left
      uint8_t w = 0;
      for (uint8_t i = 1; i < n_acks_; i++) if (acks_[i].ttl < acks_[w].ttl) w = i;
      acks_[w] = acks_[--n_acks_];
    }
    acks_[n_acks_].a.hub = hub; acks_[n_acks_].a.node = node; acks_[n_acks_].a.seq = seq;
    acks_[n_acks_].ttl = ACK_REPEAT;
    n_acks_++;
  }

  // ---- plan -------------------------------------------------------------------------------
  // Fills slots, acks, test mode and frame length of `bc` for `frame`. Flags/cmd/silence are the caller's.
  void buildBeacon(uint16_t frame, Beacon& bc) {
    expire(frame);
    classify(frame);
    bc.network_id = cfg.network_id; bc.src = cfg.self_id; bc.frame = frame;
    bc.frame_10ms = static_cast<uint8_t>(cfg.frame_ms / 10);
    bc.test_mode = cfg.test_mode;
    // acks
    bc.n_acks = 0;
    for (uint8_t i = 0; i < n_acks_ && bc.n_acks < MAX_ACKS; i++) bc.acks[bc.n_acks++] = acks_[i].a;
    for (uint8_t i = 0; i < n_acks_;) {
      if (--acks_[i].ttl == 0) acks_[i] = acks_[--n_acks_]; else i++;
    }
    uint8_t allow = (cfg.test_mode != TEST_OFF) ? cfg.test_allowance : cfg.allowance;
    for (;;) {
      fill(frame, bc, allow);
      const uint16_t blen = static_cast<uint16_t>(beaconSize(bc.n_slots, bc.n_acks));
      if (planFits(bc.slots, bc.n_slots, static_cast<uint32_t>(cfg.frame_ms) * 1000UL, blen)) break;
      if (allow > MIN_ALLOWANCE + 8) { allow = static_cast<uint8_t>(allow - 8); continue; }
      while (bc.n_slots > 0 && !planFits(bc.slots, bc.n_slots, static_cast<uint32_t>(cfg.frame_ms) * 1000UL,
                                         static_cast<uint16_t>(beaconSize(bc.n_slots, bc.n_acks)))) {
        bc.n_slots--;              // last resort: drop the last direct slots
        dropped_slots_++;
      }
      break;
    }
    last_allowance_ = allow;
  }

  // ---- read access ------------------------------------------------------------------------
  const HubInfo* hubAt(uint8_t i) const { return (i < MAX_HUBS && hubs_[i].used) ? &hubs_[i] : nullptr; }
  static uint8_t capacity() { return MAX_HUBS; }
  const HubInfo* find(uint8_t id) const {
    for (uint8_t i = 0; i < MAX_HUBS; i++) if (hubs_[i].used && hubs_[i].id == id) return &hubs_[i];
    return nullptr;
  }
  uint8_t lastAllowance() const { return last_allowance_; }
  uint32_t droppedSlots() const { return dropped_slots_; }
  void resetStats() {
    for (uint8_t i = 0; i < MAX_HUBS; i++) memset(hubs_[i].stats, 0, sizeof(hubs_[i].stats));
  }

  // The mode a direct hub slot uses in `frame` (also used by the receiver to set its radio).
  uint8_t directModeFor(const HubInfo& h, uint8_t index, uint16_t frame) const {
    switch (cfg.test_mode) {
      case TEST_ROTATE: return static_cast<uint8_t>((frame + index) % MODE_COUNT);
      case TEST_FIX_SF9: return MODE_SF9_BW500;
      case TEST_FIX_SF8: return MODE_SF8_BW500;
      case TEST_FIX_SF7: return MODE_SF7_BW500;
      default: return cfg.adaptive ? h.mode : static_cast<uint8_t>(MODE_SF9_BW500);
    }
  }

 private:
  struct PendingAck { Ack a; uint8_t ttl; };

  HubInfo* findMut(uint8_t id) {
    for (uint8_t i = 0; i < MAX_HUBS; i++) if (hubs_[i].used && hubs_[i].id == id) return &hubs_[i];
    return nullptr;
  }
  HubInfo* get(uint8_t id, uint16_t frame) {
    if (id == ID_NONE || id == cfg.self_id) return nullptr;
    HubInfo* h = findMut(id);
    if (h != nullptr) return h;
    for (uint8_t i = 0; i < MAX_HUBS; i++) {
      if (!hubs_[i].used) {
        HubInfo& n = hubs_[i];
        memset(&n, 0, sizeof(n));
        n.used = true; n.id = id; n.last_any = frame; n.last_direct = static_cast<uint16_t>(frame - 1000);
        n.mode = MODE_SF9_BW500;
        return &n;
      }
    }
    return nullptr;
  }

  void noteDirect(HubInfo& h, uint16_t frame) {
    if (seqDiff(frame, h.last_direct) > static_cast<int16_t>(DIRECT_WINDOW)) h.direct_hits = 0;
    if (h.direct_hits < 255) h.direct_hits++;
    h.last_direct = frame; h.last_any = frame; h.ever_direct = true;
  }

  // Relay choice. A relay that actually carried the hub's packet ("proven") or reported its JOIN
  // is taken at once; a neighbour report must beat the current candidate by RELAY_SWITCH_DB.
  void noteRelayCandidate(HubInfo& h, uint8_t relay, int8_t rssi, uint16_t frame, bool proven) {
    if (relay == ID_NONE || relay == h.id || relay == cfg.self_id) return;
    const bool stale = seqDiff(frame, h.relay_cand_frame) > static_cast<int16_t>(RELAY_CAND_TIMEOUT);
    const bool replace = h.relay_cand == ID_NONE || stale || relay == h.relay_cand || proven ||
                         rssi >= h.relay_cand_rssi + RELAY_SWITCH_DB;
    if (!replace) return;
    if (proven && rssi == 0) {                       // carried the packet, link RSSI unknown
      if (relay != h.relay_cand) h.relay_cand_rssi = -127;
    } else {
      h.relay_cand_rssi = rssi;
    }
    h.relay_cand = relay;
    h.relay_cand_frame = frame;
  }

  bool directRecent(const HubInfo& h, uint16_t frame) const {
    return h.ever_direct && seqDiff(frame, h.last_direct) <= static_cast<int16_t>(DIRECT_WINDOW);
  }

  void expire(uint16_t frame) {
    for (uint8_t i = 0; i < MAX_HUBS; i++)
      if (hubs_[i].used && seqDiff(frame, hubs_[i].last_any) > static_cast<int16_t>(HUB_TIMEOUT_FRAMES))
        hubs_[i].used = false;
  }

  void classify(uint16_t frame) {
    for (uint8_t i = 0; i < MAX_HUBS; i++) {
      HubInfo& h = hubs_[i];
      if (!h.used) continue;
      const bool direct_ok = directRecent(h, frame) &&
                             (h.via == 0 || h.direct_hits >= DIRECT_HITS_TO_RETURN);
      if (direct_ok) { h.via = 0; continue; }
      const HubInfo* r = (h.relay_cand != ID_NONE) ? findMut(h.relay_cand) : nullptr;
      const bool cand_ok = r != nullptr && r->via == 0 && directRecent(*r, frame) &&
                           seqDiff(frame, h.relay_cand_frame) <= static_cast<int16_t>(RELAY_CAND_TIMEOUT);
      h.via = cand_ok ? h.relay_cand : 0;   // no usable relay: keep trying a direct slot
    }
  }

  void fill(uint16_t frame, Beacon& bc, uint8_t allow) {
    bc.n_slots = 0;
    uint8_t relays[MAX_RELAYS]; uint8_t n_relays = 0;
    for (uint8_t i = 0; i < MAX_HUBS; i++) {
      const HubInfo& h = hubs_[i];
      if (!h.used || h.via == 0) continue;
      bool known = false;
      for (uint8_t k = 0; k < n_relays; k++) if (relays[k] == h.via) known = true;
      if (!known && n_relays < MAX_RELAYS) relays[n_relays++] = h.via;
    }
    // 1. echo slots
    for (uint8_t k = 0; k < n_relays && bc.n_slots < MAX_SLOTS; k++) push(bc, SLOT_ECHO, relays[k], MODE_SF9_BW500, 0, 0);
    // 2. join slot
    if (bc.n_slots < MAX_SLOTS) push(bc, SLOT_JOIN, ID_NONE, MODE_SF9_BW500, JOIN_LEN, 0);
    // 3. remote hubs (before their relays, so the relay can forward in the same frame)
    for (uint8_t i = 0; i < MAX_HUBS && bc.n_slots < MAX_SLOTS; i++) {
      const HubInfo& h = hubs_[i];
      if (!h.used || h.via == 0) continue;
      bool relay_ok = false;
      for (uint8_t k = 0; k < n_relays; k++) if (relays[k] == h.via) relay_ok = true;
      if (relay_ok) push(bc, SLOT_HUB, h.id, MODE_SF9_BW500, allow, h.via);
    }
    // 4. direct hubs
    uint8_t idx = 0;
    for (uint8_t i = 0; i < MAX_HUBS && bc.n_slots < MAX_SLOTS; i++) {
      const HubInfo& h = hubs_[i];
      if (!h.used || h.via != 0) continue;
      uint16_t a = allow;
      for (uint8_t s = 0; s < bc.n_slots; s++)       // relay carries its remote hubs' packets
        if (bc.slots[s].kind == SLOT_HUB && bc.slots[s].via == h.id) a = static_cast<uint16_t>(a + bc.slots[s].allowance - 2);
      if (a > MAX_PACKET) a = MAX_PACKET;
      push(bc, SLOT_HUB, h.id, directModeFor(h, idx++, frame), static_cast<uint8_t>(a), 0);
    }
    // echo allowance = beacon length (known now that slots and acks are set)
    const uint8_t blen = static_cast<uint8_t>(beaconSize(bc.n_slots, bc.n_acks));
    for (uint8_t s = 0; s < bc.n_slots; s++) if (bc.slots[s].kind == SLOT_ECHO) bc.slots[s].allowance = blen;
  }

  static void push(Beacon& bc, uint8_t kind, uint8_t owner, uint8_t mode, uint8_t allowance, uint8_t via) {
    Slot& s = bc.slots[bc.n_slots++];
    s.kind = kind; s.owner = owner; s.mode = mode; s.allowance = allowance; s.via = via;
  }

  void setMode(HubInfo& h, RadioMode m, uint16_t frame, uint16_t hold) {
    h.mode = m; h.hist = 0; h.hist_n = 0; h.miss_streak = 0;
    h.hold_until = static_cast<uint16_t>(frame + hold);
  }

  void adaptUp(HubInfo& h, uint16_t frame) {
    if (!cfg.adaptive || cfg.test_mode != TEST_OFF || h.mode == MODE_SF7_BW500) return;
    if (seqDiff(frame, h.hold_until) < 0 || h.hist_n < 8 || h.hist != 0xFF) return;
    int8_t mn = h.win_rssi[0];
    for (uint8_t i = 1; i < 8; i++) if (h.win_rssi[i] < mn) mn = h.win_rssi[i];
    const RadioMode next = fasterMode(static_cast<RadioMode>(h.mode));
    const int16_t margin_x10 = static_cast<int16_t>(mn * 10 - modeInfo(next).sens_dbm_x10);
    if (margin_x10 >= UP_MARGIN_DB * 10) setMode(h, next, frame, UP_HOLD_FRAMES);
  }

  HubInfo hubs_[MAX_HUBS];
  PendingAck acks_[MAX_ACKS];
  uint8_t n_acks_;
  uint8_t last_allowance_ = 0;
  uint32_t dropped_slots_ = 0;
};

}  // namespace tdma
}  // namespace icemesh
#endif
