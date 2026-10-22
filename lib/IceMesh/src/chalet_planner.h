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
  int8_t win_snr_q4[8];        // D79: SNR goes with RSSI into the step-up decision
  uint8_t trial_left;          // D79: frames the new faster mode still has to pass (a miss = straight back)
  uint8_t trial_prev;          // mode to fall back to during the trial
  uint8_t trial_fails;         // D80: failed trials in a row (each one doubles the hold before the next try, x8 max)
  int8_t last_rssi, last_snr_q4;
  uint8_t relay_cand; int8_t relay_cand_rssi; uint16_t relay_cand_frame;
  bool has_health; uint16_t health_frame; Health health;
  ModeStats stats[MODE_COUNT];
  // D81 (5.5): the first hop of a relayed hub (hub -> its relay) has its own adaptive mode, fed by "the relay
  // carried it this frame" and the relay's health neighbour entry (RSSI / SNR of this hub as the relay hears it)
  uint8_t hop_mode, hop_hist, hop_hist_n, hop_miss, hop_trial_left, hop_trial_prev, hop_trial_fails;
  uint16_t hop_hold_until, hop_seen_frame;
  bool hop_carried;            // SEC_RELAY for this hub seen in the current frame
  int8_t hop_rssi, hop_snr_q4; bool hop_has_signal; uint16_t hop_signal_frame;
};

struct PlannerConfig {
  uint8_t network_id;
  uint8_t self_id;
  uint8_t test_mode;        // TestMode
  bool adaptive;
  uint8_t allowance;        // bytes per hub slot, normal operation
  uint8_t test_allowance;   // bytes per hub slot in test modes
  uint16_t frame_ms;         // shortest frame (normally 1000)
  uint16_t frame_ms_max;     // longest frame the planner may step up to when the hubs do not fit (2000)
  uint8_t focus_hub;        // hub reporting the FOCUS sonar node (ID_NONE = none)
  uint8_t extra_hubs[2];    // D81 (5.4): hubs of up to two more streamed holes, kept only while they fit (ID_NONE = none)
  uint8_t range_hub;        // D80 range test: this hub's slot cycles SF9 -> SF5 one mode per frame (ID_NONE = off)
  uint8_t focus_allowance;  // its slot allowance (est. 180 B = FOCUS stream + line state)
};

template <uint8_t MAX_HUBS = 10>
class ChaletPlanner {
 public:
  // D81 (5.1): the frame is 0.5-2 s long, so every timeout here is in milliseconds and turned into frames with fr()
  // at the frame length in use. The *_FRAMES names stay as their value at a 1 s frame (tests).
  enum {
    HUB_TIMEOUT_FRAMES = 30,
    DIRECT_WINDOW = 5,
    RELAY_CAND_TIMEOUT = 10,
    RELAY_KEEP_FRAMES = 60,
    EXTRA_FOCUS = 2,          // D81: streamed holes beyond the first (follow the action watches the top 3)
    EXTRA_HOLD_MS = 10000,    // D81: a dropped extra is not tried again for this long
    ACK_REPEAT = 3,
    MAX_RELAYS = 3,
    MIN_ALLOWANCE = 40,
    RELAY_SWITCH_DB = 6,      // est. hysteresis between relay candidates
    DIRECT_HITS_TO_RETURN = 3,// a remote hub needs this many direct receptions in DIRECT_WINDOW to go direct
    JOIN_LEN = 7,
    KEEP_ALLOWANCE = 64,      // a frame "fits" when every hub keeps at least this many bytes (est.)
    FOCUS_KEEP_MARGIN = 40,   // ... and the FOCUS hub at least focus_allowance - this (sonar stream, sim)
    FRAME_STEP_MS = 500,
    DOWN_FRAMES = 20,         // beacons in a row where the shorter frame fits before stepping down
    UP_MARGIN_DB = 12,        // est.: step to a faster mode when the weakest of 8 packets keeps this margin
    UP_SNR_MARGIN_DB = 6,     // D79 est.: ... and the weakest SNR keeps this over the next mode's demodulation limit
    TRIAL_FRAMES = 3,         // D79: a faster mode is on trial for its first frames: one miss = back where it was
    FW_FAST_MODES = 2,        // D80: hub firmware (Health.fw) that knows SF6 / SF5; older hubs stop at SF7
    UP_HOLD_FRAMES = 20,
    DOWN_HOLD_FRAMES = 30,
    HOP_UP_EXTRA_DB = 3,      // D81 est.: the relay's RSSI of the far hub arrives every 8 frames only: ask a little more margin
  };
  // frames for `ms` at the frame length in use (at least 1)
  uint16_t fr(uint32_t ms) const { const uint16_t f = frame_ms_ ? frame_ms_ : (cfg.frame_ms ? cfg.frame_ms : 1000); const uint32_t n = ms / f; return n < 1 ? 1 : (n > 65535 ? 65535 : static_cast<uint16_t>(n)); }

  PlannerConfig cfg;

  ChaletPlanner() {
    memset(&cfg, 0, sizeof(cfg));
    cfg.allowance = 96; cfg.test_allowance = 160; cfg.frame_ms = 1000;
    cfg.focus_hub = ID_NONE; cfg.focus_allowance = 180; cfg.frame_ms_max = 2000; cfg.range_hub = ID_NONE;
    cfg.extra_hubs[0] = cfg.extra_hubs[1] = ID_NONE;
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
    if (mode == h->mode && h->id != cfg.range_hub) {
      h->hist = static_cast<uint8_t>((h->hist << 1) | 1);
      h->win_rssi[h->hist_n % 8] = rssi;
      h->win_snr_q4[h->hist_n % 8] = snr_q4;
      if (h->hist_n < 255) h->hist_n++;
      h->miss_streak = 0;
      if (h->trial_left && --h->trial_left == 0) h->trial_fails = 0;   // D79: one more good frame; D80: trial passed
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
    if (mode == h->mode && h->id != cfg.range_hub) {
      h->hist = static_cast<uint8_t>(h->hist << 1);
      if (h->hist_n < 255) h->hist_n++;
      h->miss_streak++;
      if (cfg.adaptive && cfg.test_mode == TEST_OFF && h->trial_left && h->mode != MODE_SF9_BW500) {   // D79: trial failed
        const uint8_t f = h->trial_fails < 3 ? h->trial_fails : 3;   // D80 back-off: hold x1, x2, x4, x8
        const uint8_t fails = static_cast<uint8_t>(h->trial_fails < 255 ? h->trial_fails + 1 : 255);
        setMode(*h, static_cast<RadioMode>(h->trial_prev), frame, static_cast<uint16_t>(fr(DOWN_HOLD_FRAMES * 1000UL) << f));
        h->trial_fails = fails;
      } else if (cfg.adaptive && cfg.test_mode == TEST_OFF && h->miss_streak >= 2 && h->mode != MODE_SF9_BW500) {
        setMode(*h, slowerMode(static_cast<RadioMode>(h->mode)), frame, fr(DOWN_HOLD_FRAMES * 1000UL));
      }
    }
  }

  void onRelayed(uint8_t hub, uint8_t via, uint16_t frame) {
    HubInfo* h = get(hub, frame);
    if (h == nullptr) return;
    h->last_any = frame;
    if (via == h->via && h->hop_seen_frame == frame) h->hop_carried = true;   // D81 (5.5): the first hop worked this frame
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
    for (uint8_t i = 0; i < hl.n_nb; i++) {
      onNeighbor(hub, hl.nb[i].hub, hl.nb[i].rssi, frame);
      HubInfo* r = findMut(hl.nb[i].hub);   // D81 (5.5): the relay's view of its remote hub feeds the hop's step-up margin
      if (r != nullptr && r->via == hub) { r->hop_rssi = hl.nb[i].rssi; r->hop_snr_q4 = hl.nb[i].snr_q4; r->hop_has_signal = true; r->hop_signal_frame = frame; }
    }
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
  // Frame length: cfg.frame_ms while every hub keeps a slot of >= KEEP_ALLOWANCE bytes; otherwise it steps
  // up by 500 ms to cfg.frame_ms_max (sim finding S1: 10 hubs at SF9/500 do not fit 1 s). It steps back down
  // after DOWN_FRAMES beacons in a row where the shorter frame would fit. Hubs follow the beacon's length.
  void buildBeacon(uint16_t frame, Beacon& bc) {
    expire(frame);
    classify(frame);
    scoreHops(frame);
    bc.network_id = cfg.network_id; bc.src = cfg.self_id; bc.frame = frame;
    bc.test_mode = cfg.test_mode;
    // acks
    bc.n_acks = 0;
    for (uint8_t i = 0; i < n_acks_ && bc.n_acks < MAX_ACKS; i++) bc.acks[bc.n_acks++] = acks_[i].a;
    for (uint8_t i = 0; i < n_acks_;) {
      if (--acks_[i].ttl == 0) acks_[i] = acks_[--n_acks_]; else i++;
    }
    const uint16_t fmin = cfg.frame_ms, fmax = cfg.frame_ms_max < cfg.frame_ms ? cfg.frame_ms : cfg.frame_ms_max;
    if (frame_ms_ < fmin || frame_ms_ > fmax) frame_ms_ = fmin;
    if (frame_ms_ > fmin) {
      Beacon probe = bc;
      const uint16_t shorter = static_cast<uint16_t>(frame_ms_ - FRAME_STEP_MS < fmin ? fmin : frame_ms_ - FRAME_STEP_MS);
      if (plan(frame, probe, shorter, nullptr)) {
        if (++down_streak_ >= fr(DOWN_FRAMES * 1000UL)) { frame_ms_ = shorter; down_streak_ = 0; }
      } else {
        down_streak_ = 0;
      }
    }
    uint8_t allow = 0;
    while (!plan(frame, bc, frame_ms_, nullptr) && frame_ms_ < fmax) {
      frame_ms_ = static_cast<uint16_t>(frame_ms_ + FRAME_STEP_MS > fmax ? fmax : frame_ms_ + FRAME_STEP_MS);
      down_streak_ = 0;
    }
    plan(frame, bc, frame_ms_, &allow);
    grow(frame, bc, frame_ms_);
    bc.frame_10ms = static_cast<uint8_t>(frame_ms_ / 10);
    last_allowance_ = allow;
    for (uint8_t i = 0; i < MAX_HUBS; i++) if (hubs_[i].used && hubs_[i].via != 0 && hubScheduled(bc, hubs_[i].id)) hubs_[i].hop_seen_frame = frame;
  }
  // D81 (5.2): the time left in the frame goes to bigger turns, fastest hubs first (a turn of 255 B is 40 ms at SF5,
  // 318 ms at SF9). After the frame-length decision, so a big turn never keeps the frame long.
  // Only hubs that stream a hole grow (the others have nothing to fill a big turn with, and every hub listens to every
  // turn: a long empty window costs the others battery). GROW_KEEP_US stays free for the chalet's channel sample and
  // the random JOIN listen that live between the last slot and the next beacon.
  enum { GROW_KEEP_US = 100000 };
  void grow(uint16_t frame, Beacon& bc, uint16_t fm) {
    (void)frame;
    if (cfg.test_mode != TEST_OFF) return;
    const uint32_t fus = static_cast<uint32_t>(fm) * 1000UL;
    if (fus <= GROW_KEEP_US) return;
    const uint16_t blen = static_cast<uint16_t>(beaconSize(bc.n_slots, bc.n_acks));
    for (uint8_t m = MODE_COUNT; m-- > 0;) for (uint8_t pass = 0; pass < 2; pass++) {   // fastest mode first; streaming hubs before the others
      for (uint8_t s = 0; s < bc.n_slots; s++) {
        Slot& sl = bc.slots[s];
        if (sl.kind != SLOT_HUB || sl.via != 0 || sl.mode != m || sl.allowance >= MAX_PACKET || sl.owner == cfg.range_hub) continue;
        if (focusCount(sl.owner) == 0 || pass == 1) continue;
        { uint8_t own = 0; for (uint8_t k = 0; k < bc.n_slots; k++) if (bc.slots[k].kind == SLOT_HUB && bc.slots[k].owner == sl.owner) own++; if (own > 1) continue; }   // two turns already: it asked for what it needs
        const uint8_t keep = sl.allowance;
        sl.allowance = MAX_PACKET;
        if (!planFits(bc.slots, bc.n_slots, fus - GROW_KEEP_US, blen)) sl.allowance = keep;
      }
    }
  }
  // D81 (5.4): which of the extra streamed holes the plan kept (0 = the first extra, 1 = the second)
  bool extraKept(uint8_t k) const { return k < extras_kept_; }
  uint16_t frameMs() const { return frame_ms_; }

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
  // D80: fastest mode this hub's firmware knows (until its first health section arrives: the old set)
  static RadioMode maxModeFor(const HubInfo& h) { return (h.has_health && h.health.fw >= FW_FAST_MODES) ? MODE_SF5_BW500 : MODE_SF7_BW500; }

  uint8_t directModeFor(const HubInfo& h, uint8_t index, uint16_t frame) const {
    if (cfg.range_hub != ID_NONE && h.id == cfg.range_hub)   // D80 range test: one mode per frame, SF9 first
      return static_cast<uint8_t>(frame % (static_cast<uint8_t>(maxModeFor(h)) + 1));
    switch (cfg.test_mode) {
      case TEST_ROTATE: return static_cast<uint8_t>((frame + index) % (static_cast<uint8_t>(maxModeFor(h)) + 1));
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
        n.mode = MODE_SF9_BW500; n.hop_mode = MODE_SF9_BW500;
        n.hold_until = frame; n.hop_hold_until = frame;   // D81: a hold of 0 is "half a wrap away" in the upper half of the frame counter
        return &n;
      }
    }
    return nullptr;
  }

  void noteDirect(HubInfo& h, uint16_t frame) {
    if (seqDiff(frame, h.last_direct) > static_cast<int16_t>(fr(DIRECT_WINDOW * 1000UL))) h.direct_hits = 0;
    if (h.direct_hits < 255) h.direct_hits++;
    h.last_direct = frame; h.last_any = frame; h.ever_direct = true;
  }

  // Relay choice. A relay that actually carried the hub's packet ("proven") or reported its JOIN
  // is taken at once; a neighbour report must beat the current candidate by RELAY_SWITCH_DB.
  void noteRelayCandidate(HubInfo& h, uint8_t relay, int8_t rssi, uint16_t frame, bool proven) {
    if (relay == ID_NONE || relay == h.id || relay == cfg.self_id) return;
    const bool stale = seqDiff(frame, h.relay_cand_frame) > static_cast<int16_t>(fr(RELAY_CAND_TIMEOUT * 1000UL));
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
    return h.ever_direct && seqDiff(frame, h.last_direct) <= static_cast<int16_t>(fr(DIRECT_WINDOW * 1000UL));
  }

  void expire(uint16_t frame) {
    for (uint8_t i = 0; i < MAX_HUBS; i++)
      if (hubs_[i].used && hubs_[i].id != cfg.range_hub &&   // D80: the range test hub stays (its misses are the data)
          seqDiff(frame, hubs_[i].last_any) > static_cast<int16_t>(fr(HUB_TIMEOUT_FRAMES * 1000UL)))
        hubs_[i].used = false;
  }

  void classify(uint16_t frame) {
    for (uint8_t i = 0; i < MAX_HUBS; i++) {
      HubInfo& h = hubs_[i];
      if (!h.used) continue;
      if (h.id == cfg.range_hub) { h.via = 0; continue; }   // D80 range test: always its own direct slot, even at the edge
      const bool direct_ok = directRecent(h, frame) &&
                             (h.via == 0 || h.direct_hits >= DIRECT_HITS_TO_RETURN);
      if (direct_ok) { h.via = 0; continue; }
      const HubInfo* r = (h.relay_cand != ID_NONE) ? findMut(h.relay_cand) : nullptr;
      // a fresh candidate, or an older one (RELAY_KEEP_FRAMES) when the hub is not heard directly at all:
      // a direct slot nobody hears is worse than a relay that worked a minute ago (sim h10, hub behind a ridge)
      const int16_t age = seqDiff(frame, h.relay_cand_frame);
      const bool cand_ok = r != nullptr && r->via == 0 && directRecent(*r, frame) &&
                           (age <= static_cast<int16_t>(fr(RELAY_CAND_TIMEOUT * 1000UL)) ||
                            (!directRecent(h, frame) && age <= static_cast<int16_t>(fr(RELAY_KEEP_FRAMES * 1000UL))));
      const uint8_t nv = cand_ok ? h.relay_cand : 0;   // no usable relay: keep trying a direct slot
      if (nv != h.via) resetHop(h, frame);
      h.via = nv;
    }
  }

  // ---- D81 (5.5): adaptive mode on the first hop of a relayed hub (far hub -> its relay) -------------------------
  static void resetHop(HubInfo& h, uint16_t frame) {
    h.hop_mode = MODE_SF9_BW500; h.hop_hist = 0; h.hop_hist_n = 0; h.hop_miss = 0; h.hop_trial_left = 0; h.hop_trial_fails = 0;
    h.hop_hold_until = frame; h.hop_has_signal = false; h.hop_carried = false; h.hop_seen_frame = static_cast<uint16_t>(frame - 2);
  }
  void setHopMode(HubInfo& h, RadioMode m, uint16_t frame, uint16_t hold) {
    h.hop_mode = m; h.hop_hist = 0; h.hop_hist_n = 0; h.hop_miss = 0; h.hop_trial_left = 0;
    h.hop_hold_until = static_cast<uint16_t>(frame + hold);
  }
  uint8_t hopModeFor(const HubInfo& h) const {
    return (cfg.adaptive && cfg.test_mode == TEST_OFF) ? h.hop_mode : static_cast<uint8_t>(MODE_SF9_BW500);
  }
  // Called once per frame, before the new plan: last frame's remote slots are scored. A miss only counts when the relay's
  // own packet got through (otherwise the loss was on the second hop, which has its own adaptation).
  void scoreHops(uint16_t frame) {
    const uint16_t prev = static_cast<uint16_t>(frame - 1);
    for (uint8_t i = 0; i < MAX_HUBS; i++) {
      HubInfo& h = hubs_[i];
      if (!h.used || h.via == 0 || h.hop_seen_frame != prev) continue;
      const HubInfo* r = find(h.via);
      const bool relay_heard = r != nullptr && r->last_direct == prev;
      if (h.hop_carried) {
        h.hop_hist = static_cast<uint8_t>((h.hop_hist << 1) | 1); if (h.hop_hist_n < 255) h.hop_hist_n++;
        h.hop_miss = 0;
        if (h.hop_trial_left && --h.hop_trial_left == 0) h.hop_trial_fails = 0;
        adaptHopUp(h, r, frame);
      } else if (relay_heard && cfg.adaptive && cfg.test_mode == TEST_OFF) {
        h.hop_hist = static_cast<uint8_t>(h.hop_hist << 1); if (h.hop_hist_n < 255) h.hop_hist_n++;
        h.hop_miss++;
        if (h.hop_trial_left && h.hop_mode != MODE_SF9_BW500) {
          const uint8_t f = h.hop_trial_fails < 3 ? h.hop_trial_fails : 3;
          const uint8_t fails = static_cast<uint8_t>(h.hop_trial_fails < 255 ? h.hop_trial_fails + 1 : 255);
          setHopMode(h, static_cast<RadioMode>(h.hop_trial_prev), frame, static_cast<uint16_t>(fr(DOWN_HOLD_FRAMES * 1000UL) << f));
          h.hop_trial_fails = fails;
        } else if (h.hop_miss >= 2 && h.hop_mode != MODE_SF9_BW500) {
          setHopMode(h, slowerMode(static_cast<RadioMode>(h.hop_mode)), frame, fr(DOWN_HOLD_FRAMES * 1000UL));
        }
      }
      h.hop_carried = false;
    }
  }
  void adaptHopUp(HubInfo& h, const HubInfo* relay, uint16_t frame) {
    if (!cfg.adaptive || cfg.test_mode != TEST_OFF || relay == nullptr) return;
    const RadioMode next = fasterMode(static_cast<RadioMode>(h.hop_mode));
    if (next == h.hop_mode) return;
    const uint8_t cap = static_cast<uint8_t>(maxModeFor(h)) < static_cast<uint8_t>(maxModeFor(*relay)) ? static_cast<uint8_t>(maxModeFor(h)) : static_cast<uint8_t>(maxModeFor(*relay));
    if (static_cast<uint8_t>(next) > cap) return;   // the hub or its relay runs old firmware
    if (seqDiff(frame, h.hop_hold_until) < 0 || h.hop_hist_n < 8 || h.hop_hist != 0xFF || h.hop_trial_left) return;
    if (!h.hop_has_signal || seqDiff(frame, h.hop_signal_frame) > static_cast<int16_t>(fr(30000UL))) return;   // no fresh relay-side signal
    const int16_t margin_x10 = static_cast<int16_t>(h.hop_rssi * 10 - modeInfo(next).sens_dbm_x10);
    const int16_t snr_margin_x10 = static_cast<int16_t>((h.hop_snr_q4 * 10) / 4 - modeInfo(next).snr_limit_x10);
    if (margin_x10 >= (UP_MARGIN_DB + HOP_UP_EXTRA_DB) * 10 && snr_margin_x10 >= UP_SNR_MARGIN_DB * 10) {
      const uint8_t prev = h.hop_mode;
      setHopMode(h, next, frame, fr(UP_HOLD_FRAMES * 1000UL));
      h.hop_trial_left = TRIAL_FRAMES; h.hop_trial_prev = prev;
    }
  }

  // One plan for a frame of `fm` ms: shrink FOCUS, then all allowances, then drop the last slots.
  // Returns true when nothing was dropped and the allowance stayed >= KEEP_ALLOWANCE.
  // allow_out != nullptr: final plan (counts dropped slots).
  bool plan(uint16_t frame, Beacon& bc, uint16_t fm, uint8_t* allow_out) {
    const uint32_t fus = static_cast<uint32_t>(fm) * 1000UL;
    uint8_t allow = (cfg.test_mode != TEST_OFF) ? cfg.test_allowance : cfg.allowance;
    uint8_t focus = (cfg.focus_hub != ID_NONE && cfg.focus_allowance > allow) ? cfg.focus_allowance : allow;
    bool dropped = false;
    // D81 (5.4): extras are counted from the front (the caller packs them); once one was dropped the plan does not try
    // it again for EXTRA_HOLD_MS, so a plan at the limit does not flicker between 2 and 3 streams every beacon
    uint8_t want_extras = 0;
    for (uint8_t k = 0; k < EXTRA_FOCUS && cfg.extra_hubs[k] != ID_NONE; k++) want_extras = static_cast<uint8_t>(k + 1);
    if (allow_out == nullptr) { extras_kept_ = want_extras; }   // a probe: never touches the hold
    else if (seqDiff(frame, extras_retry_) < 0 && extras_cap_ < want_extras) extras_kept_ = extras_cap_;
    else extras_kept_ = want_extras;
    for (;;) {
      fill(frame, bc, allow, focus);
      const uint16_t blen = static_cast<uint16_t>(beaconSize(bc.n_slots, bc.n_acks));
      const bool fits = planFits(bc.slots, bc.n_slots, fus, blen);
      if (fits && !full_) break;
      if (extras_kept_ > 0) {                                                            // D81: extra streamed holes go first
        extras_kept_--;
        if (allow_out != nullptr) { extras_cap_ = extras_kept_; extras_retry_ = static_cast<uint16_t>(frame + fr(EXTRA_HOLD_MS)); }
        continue;
      }
      if (fits && full_) { dropped = true; break; }  // fits, but a hub found no slot (beacon full: MAX_SLOTS)
      if (focus > allow + 16) { focus = static_cast<uint8_t>(focus - 16); continue; }   // shrink FOCUS first
      if (allow > MIN_ALLOWANCE + 8) { allow = static_cast<uint8_t>(allow - 8); focus = allow; continue; }
      while (bc.n_slots > 0 && !planFits(bc.slots, bc.n_slots, fus, static_cast<uint16_t>(beaconSize(bc.n_slots, bc.n_acks)))) {
        bc.n_slots--;              // last resort: drop the last direct slots
        dropped = true;
        if (allow_out) dropped_slots_++;
      }
      break;
    }
    if (allow_out) *allow_out = allow;
    const bool focus_ok = cfg.focus_hub == ID_NONE || cfg.focus_allowance <= allow ||
                          focus + FOCUS_KEEP_MARGIN >= cfg.focus_allowance || !hubScheduled(bc, cfg.focus_hub);
    return !dropped && allow >= KEEP_ALLOWANCE && focus_ok;
  }

  // streamed holes this hub reports (the first focus plus the extras the current plan keeps)
  uint8_t focusCount(uint8_t hub) const {
    uint8_t n = (cfg.focus_hub != ID_NONE && hub == cfg.focus_hub) ? 1 : 0;
    for (uint8_t k = 0; k < extras_kept_ && k < EXTRA_FOCUS; k++) if (cfg.extra_hubs[k] == hub) n++;
    return n;
  }
  static bool hubScheduled(const Beacon& bc, uint8_t id) {
    for (uint8_t s = 0; s < bc.n_slots; s++) if (bc.slots[s].kind == SLOT_HUB && bc.slots[s].owner == id) return true;
    return false;
  }

  void fill(uint16_t frame, Beacon& bc, uint8_t allow, uint8_t focus) {
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
      // a remote FOCUS hub: its relay must carry it too, keep the sum under one packet
      uint8_t a = allow;
      const uint8_t nf = focusCount(h.id);
      if (nf && focus > allow) {
        const uint16_t want = static_cast<uint16_t>(allow + nf * (focus - allow));
        const uint16_t room = static_cast<uint16_t>(MAX_PACKET - allow);
        a = static_cast<uint8_t>(want < room ? want : room);
        if (a < allow) a = allow;
      }
      if (relay_ok) push(bc, SLOT_HUB, h.id, hopModeFor(h), a, h.via);
    }
    // 4. direct hubs. D81 (5.4): a hub streaming n holes wants allow + n x (focus - allow); past one packet it gets a
    // second slot (5.3) when the plan has room (the shrink order drops the extra holes first)
    uint8_t idx = 0;
    for (uint8_t i = 0; i < MAX_HUBS && bc.n_slots < MAX_SLOTS; i++) {
      const HubInfo& h = hubs_[i];
      if (!h.used || h.via != 0) continue;
      const uint8_t nf = focusCount(h.id);
      uint16_t a = allow;
      if (nf && focus > allow) a = static_cast<uint16_t>(allow + nf * (focus - allow));
      for (uint8_t s = 0; s < bc.n_slots; s++)       // relay carries its remote hubs' packets
        if (bc.slots[s].kind == SLOT_HUB && bc.slots[s].via == h.id) a = static_cast<uint16_t>(a + bc.slots[s].allowance - 2);
      const uint8_t mode = directModeFor(h, idx++, frame);
      if (a > MAX_PACKET && h.id != cfg.range_hub && bc.n_slots + 1 < MAX_SLOTS) {   // two turns: the first one must still hold the relayed packets
        if (a > 2 * MAX_PACKET) a = 2 * MAX_PACKET;
        uint16_t need = allow;
        for (uint8_t s = 0; s < bc.n_slots; s++) if (bc.slots[s].kind == SLOT_HUB && bc.slots[s].via == h.id) need = static_cast<uint16_t>(need + bc.slots[s].allowance - 2);
        uint16_t f1 = static_cast<uint16_t>((a + 1) / 2); if (f1 < need) f1 = need; if (f1 > MAX_PACKET) f1 = MAX_PACKET;
        if (a - f1 > MAX_PACKET) f1 = static_cast<uint16_t>(a - MAX_PACKET);
        const uint8_t first = static_cast<uint8_t>(f1);
        push(bc, SLOT_HUB, h.id, mode, first, 0);
        push(bc, SLOT_HUB, h.id, mode, static_cast<uint8_t>(a - first), 0);
      } else if (a > MAX_PACKET) {
        push(bc, SLOT_HUB, h.id, mode, MAX_PACKET, 0);
      } else {
        push(bc, SLOT_HUB, h.id, mode, static_cast<uint8_t>(a), 0);
      }
    }
    // any known hub left without a slot because the beacon is full?
    full_ = false;
    for (uint8_t i = 0; i < MAX_HUBS && !full_; i++) {
      const HubInfo& h = hubs_[i];
      if (!h.used) continue;
      bool in = false;
      for (uint8_t s = 0; s < bc.n_slots && !in; s++) in = bc.slots[s].kind == SLOT_HUB && bc.slots[s].owner == h.id;
      if (!in) full_ = bc.n_slots >= MAX_SLOTS;
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
    h.mode = m; h.hist = 0; h.hist_n = 0; h.miss_streak = 0; h.trial_left = 0;
    h.hold_until = static_cast<uint16_t>(frame + hold);
  }

  // D79: step up when the weakest of the last 8 packets keeps UP_MARGIN_DB over the next mode's sensitivity AND its SNR
  // keeps UP_SNR_MARGIN_DB over that mode's demodulation limit; the new mode is then on trial for TRIAL_FRAMES frames.
  void adaptUp(HubInfo& h, uint16_t frame) {
    if (!cfg.adaptive || cfg.test_mode != TEST_OFF || h.id == cfg.range_hub || fasterMode(static_cast<RadioMode>(h.mode)) == h.mode) return;
    if (static_cast<uint8_t>(fasterMode(static_cast<RadioMode>(h.mode))) > static_cast<uint8_t>(maxModeFor(h))) return;   // D80: old firmware
    if (seqDiff(frame, h.hold_until) < 0 || h.hist_n < 8 || h.hist != 0xFF || h.trial_left) return;
    int8_t mn = h.win_rssi[0], sn = h.win_snr_q4[0];
    for (uint8_t i = 1; i < 8; i++) { if (h.win_rssi[i] < mn) mn = h.win_rssi[i]; if (h.win_snr_q4[i] < sn) sn = h.win_snr_q4[i]; }
    const RadioMode next = fasterMode(static_cast<RadioMode>(h.mode));
    const int16_t margin_x10 = static_cast<int16_t>(mn * 10 - modeInfo(next).sens_dbm_x10);
    const int16_t snr_margin_x10 = static_cast<int16_t>((sn * 10) / 4 - modeInfo(next).snr_limit_x10);
    if (margin_x10 >= UP_MARGIN_DB * 10 && snr_margin_x10 >= UP_SNR_MARGIN_DB * 10) {
      const uint8_t prev = h.mode;
      setMode(h, next, frame, fr(UP_HOLD_FRAMES * 1000UL));
      h.trial_left = TRIAL_FRAMES; h.trial_prev = prev;
    }
  }

  HubInfo hubs_[MAX_HUBS];
  PendingAck acks_[MAX_ACKS];
  uint8_t n_acks_;
  uint8_t last_allowance_ = 0;
  uint16_t frame_ms_ = 0;
  uint16_t down_streak_ = 0;
  uint8_t extras_kept_ = 0, extras_cap_ = 0;
  uint16_t extras_retry_ = 0;
  bool full_ = false;
  uint32_t dropped_slots_ = 0;
};

}  // namespace tdma
}  // namespace icemesh
#endif
