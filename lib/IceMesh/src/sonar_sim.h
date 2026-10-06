// Sonar source for the test mode: fake raw pings (sonar_scene.h, Frank's prototype scene) ->
// node processing v0 (sonar_proc.h, port of the prototype's Proc) -> sonar blocks (sonar_codec.h),
// with the node-side block cadence: BASE summaries, or FOCUS = DATA + track info + BG blocks.
// On a real sonar node only the first stage changes (TUSS4470 instead of SonarScene).
#ifndef ICEMESH_SONAR_SIM_H
#define ICEMESH_SONAR_SIM_H
#include <stdint.h>
#include <string.h>
#include "sonar_codec.h"
#include "sonar_scene.h"
#include "sonar_proc.h"

namespace icemesh {
namespace sonar {

struct Block { uint8_t len; uint8_t data[MAX_BLOCK]; };

// Display value (0..1) -> 2-bit level: residual cells and the grey static scene.
inline uint8_t residLevel(float v) { return v < 0.30f ? 1 : (v < 0.50f ? 2 : 3); }
inline uint8_t staticLevel(float v) { return v < 0.06f ? 0 : (v < 0.20f ? 1 : (v < 0.40f ? 2 : 3)); }

// Call tick() at 4 Hz (one ping each).
//   BASE  (not focus): a summary every BASE_EVERY pings, and at once when a fish shows up.
//   FOCUS: a DATA block every FOCUS_N pings (track info in every INFO_EVERY-th one) + one BG segment
//          every BG_EVERY pings; a new BG version when the static scene changed in > BG_CHANGE bins.
class SonarSource {
 public:
  enum { BASE_EVERY = 16, FOCUS_N = 4, BG_EVERY = 8, INFO_EVERY = 2, BG_CHANGE = 12 };
  SonarScene scene;
  SonarProc proc;
  uint32_t blocks_out = 0, pings_dropped = 0;

  // seed 0 = exactly the prototype's hole; otherwise a per-hole variant
  void begin(uint8_t node, uint32_t seed, uint16_t start_index = 0) {
    begin(node, seed ? SceneConfig::forHole(seed) : SceneConfig::prototype(), start_index);
  }
  void begin(uint8_t node, const SceneConfig& cfg, uint16_t start_index) {
    node_ = node; scene.begin(cfg); proc.reset(); proc.bait_m = cfg.bait_m;
    index_ = start_index; n_pend_ = 0; since_base_ = BASE_EVERY; since_bg_ = BG_EVERY; bg_seg_ = 0;
    bg_ver_ = 0; bg_valid_ = false; data_blocks_ = 0; act_bits_ = 0; was_focus_ = false; last_fish_ = 0;
    memset(slot_id_, 0, sizeof(slot_id_)); memset(slot_used_, 0, sizeof(slot_used_));
    memset(&last_, 0, sizeof(last_)); memset(&out_, 0, sizeof(out_)); memset(bg_, 0, sizeof(bg_));
  }

  // One ping. Writes up to `max_out` blocks; returns how many.
  uint8_t tick(bool focus, Block* out, uint8_t max_out) {
    static uint8_t codes[sp::NFREQ][BINS];   // shared scratch (one source runs at a time)
    scene.ping(codes);
    proc.step(codes, sp::NFREQ, out_);
    buildPing(last_);
    uint8_t fish = 0;
    for (uint8_t i = 0; i < last_.n_targets; i++) if (last_.t[i].track != 0) fish++;
    act_bits_ = static_cast<uint16_t>((act_bits_ << 1) | (fish ? 1u : 0u));
    if (focus != was_focus_) { n_pend_ = 0; bg_seg_ = 0; since_bg_ = BG_EVERY; data_blocks_ = 0; was_focus_ = focus; }

    uint8_t n_out = 0;
    if (focus) {
      pend_[n_pend_++] = last_;
      if (n_pend_ >= FOCUS_N && n_out < max_out) {
        TrackInfo info[MAX_TARGETS];
        DataHeader h;
        h.node = node_; h.nf_neg = nfNeg(); h.bg_ver = bg_ver_; h.hard = out_.hard;
        h.n_info = (data_blocks_ % INFO_EVERY) == 0 ? trackInfo(info) : 0;
        h.info = h.n_info ? info : nullptr;
        uint8_t used = 0;
        const size_t len = encodeData(h, pend_, n_pend_, out[n_out].data, MAX_BLOCK, &used);
        if (len > 0) { out[n_out].len = static_cast<uint8_t>(len); n_out++; blocks_out++; data_blocks_++; }
        pings_dropped += static_cast<uint32_t>(n_pend_ - used);
        n_pend_ = 0;
      }
      if (++since_bg_ >= BG_EVERY && n_out < max_out) {
        updateBackground();
        const size_t len = encodeBgSegment(node_, bg_ver_, bg_seg_, bg_, out[n_out].data, MAX_BLOCK);
        if (len > 0) { out[n_out].len = static_cast<uint8_t>(len); n_out++; blocks_out++; }
        bg_seg_ = static_cast<uint8_t>((bg_seg_ + 1) % BG_SEGMENTS);
        since_bg_ = 0;
      }
    } else {
      since_base_++;
      const bool fish_new = fish > 0 && last_fish_ == 0 && since_base_ >= 4;
      if ((since_base_ >= BASE_EVERY || fish_new) && n_out < max_out) {
        const Summary s = summary();
        const size_t len = encodeSummary(s, out[n_out].data, MAX_BLOCK);
        if (len > 0) { out[n_out].len = static_cast<uint8_t>(len); n_out++; blocks_out++; }
        since_base_ = 0;
      }
    }
    last_fish_ = fish;
    return n_out;
  }

  // Summary of the latest ping: fish (not the bait), nearest one to the bait line.
  Summary summary() const {
    Summary s;
    memset(&s, 0, sizeof(s));
    s.node = node_; s.ping = last_.index; s.bottom_cm = last_.bottom_cm; s.bg_ver = bg_ver_; s.hard = out_.hard;
    uint8_t a = 0;
    for (uint16_t b = act_bits_; b; b >>= 1) a = static_cast<uint8_t>(a + (b & 1u));
    s.activity = a > 15 ? 15 : a;
    s.nearest_cm = DEPTH_NONE;
    int32_t best = 1 << 30;
    const int32_t bait_cm = static_cast<int32_t>(proc.bait_m * 100.0f + 0.5f);
    for (uint8_t i = 0; i < last_.n_targets; i++) {
      const Target& t = last_.t[i];
      if (t.track == 0) continue;
      s.n_targets++;
      const int32_t d = t.depth_cm > bait_cm ? t.depth_cm - bait_cm : bait_cm - t.depth_cm;
      if (d < best) { best = d; s.nearest_cm = t.depth_cm; s.nearest_level = t.level; }
    }
    return s;
  }
  uint8_t node() const { return node_; }
  const Ping& lastPing() const { return last_; }
  const SonarProc::Out& lastProc() const { return out_; }

 private:
  uint8_t nfNeg() const { const float n = -static_cast<float>(out_.nf); return static_cast<uint8_t>(n < 0 ? 0 : (n > 255 ? 255 : n + 0.5f)); }

  // Track IDs of the processing (unbounded) -> 3-bit codec slots: 0 = bait, 1-7 reused.
  int slotFor(uint32_t id, bool bait) {
    for (int k = 0; k < 8; k++) if (slot_used_[k] && slot_id_[k] == id) {
      if (bait == (k == 0)) return k;
      slot_used_[k] = false;                                   // label changed: move it
    }
    if (bait) { if (!slot_used_[0]) { slot_used_[0] = true; slot_id_[0] = id; return 0; } }
    for (int k = 1; k < 8; k++) if (!slot_used_[k]) { slot_used_[k] = true; slot_id_[k] = id; return k; }
    return -1;
  }

  void buildPing(Ping& p) {
    memset(&p, 0, sizeof(p));
    p.index = index_++;
    p.bottom_cm = clampDepth(static_cast<int32_t>(out_.bottom * 100.0f + 0.5f));
    // free slots of tracks the processing dropped
    for (int k = 0; k < 8; k++) {
      if (!slot_used_[k]) continue;
      bool alive = false;
      for (uint8_t i = 0; i < out_.n && !alive; i++) alive = out_.t[i].id == slot_id_[k];
      if (!alive) slot_used_[k] = false;
    }
    // live targets: seen in this ping, visible, not cover; bait first, then strongest
    // (near-bottom echoes only while they differ from the static scene: weeds stay grey, like the
    //  prototype's per-pixel "changed" test)
    const float* chg = SonarProc::binChanged();
    int idx[SonarProc::MAX_TRACKS]; int n = 0;
    for (uint8_t i = 0; i < out_.n; i++) {
      const TrackOut& t = out_.t[i];
      if (t.miss != 0 || t.label == LBL_COVER || t.v <= 0.08f) continue;
      if (t.label == LBL_NEAR_BOTTOM) {
        const int b = static_cast<int>(static_cast<float>(t.depth) / 0.025f + 0.5f);
        if (b < 0 || b >= BINS || chg[b] < 0.5f) continue;
      }
      idx[n++] = i;
    }
    for (int a = 1; a < n; a++) {
      const int v = idx[a]; int b = a - 1;
      while (b >= 0 && rank(idx[b]) > rank(v)) { idx[b + 1] = idx[b]; b--; }
      idx[b + 1] = v;
    }
    int16_t bins[MAX_TARGETS];
    for (int a = 0; a < n && p.n_targets < MAX_TARGETS; a++) {
      const TrackOut& t = out_.t[idx[a]];
      const int slot = slotFor(t.id, t.label == LBL_BAIT);
      if (slot < 0) continue;
      Target& x = p.t[p.n_targets];
      x.track = static_cast<uint8_t>(slot);
      x.depth_cm = clampDepth(static_cast<int32_t>(t.depth * 100.0f + 0.5f));
      const float v = static_cast<float>(t.v);
      x.strength = static_cast<uint8_t>(v <= 0.f ? 1 : (v >= 1.f ? 31 : (v * 31.0f + 0.5f < 1.f ? 1 : v * 31.0f + 0.5f)));
      x.level = strengthToLevel(x.strength);
      const int wb = static_cast<int>(static_cast<float>(t.width) / 0.025f + 0.5f);
      x.width = static_cast<uint8_t>(wb < 1 ? 1 : (wb > 8 ? 8 : wb));
      bins[p.n_targets] = static_cast<int16_t>(depthToBin(x.depth_cm));
      p.n_targets++;
    }
    // residual cells: strongest changed bins in the water column, away from the targets
    const float* vv = SonarProc::binValue();
    const float* al = SonarProc::binChanged();
    const int bI = static_cast<int>(out_.bottom / 0.025f + 0.5f);
    struct C { int16_t bin; float score; } best[MAX_RESID]; int nb = 0;
    for (int i = 8; i < bI - 2 && i < BINS; i++) {
      if (al[i] < 0.5f || vv[i] < 0.2f) continue;
      bool near = false;
      for (uint8_t k = 0; k < p.n_targets && !near; k++) near = i >= bins[k] - 3 && i <= bins[k] + 6;
      if (near) continue;
      const float sc = al[i] * vv[i];
      if (nb < MAX_RESID) best[nb++] = C{static_cast<int16_t>(i), sc};
      else { int w = 0; for (int k = 1; k < nb; k++) if (best[k].score < best[w].score) w = k; if (sc > best[w].score) best[w] = C{static_cast<int16_t>(i), sc}; }
    }
    for (int a = 1; a < nb; a++) { const C v = best[a]; int b = a - 1; while (b >= 0 && best[b].bin > v.bin) { best[b + 1] = best[b]; b--; } best[b + 1] = v; }
    for (int a = 0; a < nb; a++) { p.r[a].bin = static_cast<uint16_t>(best[a].bin); p.r[a].level = residLevel(vv[best[a].bin]); }
    p.n_resid = static_cast<uint8_t>(nb);
  }
  int rank(int i) const {   // bait, fish, near bottom; stronger first inside a group
    const TrackOut& t = out_.t[i];
    const int g = t.label == LBL_BAIT ? 0 : (t.label == LBL_FISH ? 1 : 2);
    return g * 1000 - static_cast<int>(static_cast<float>(t.v) * 999.0f);
  }

  // Echo character of the targets of the latest ping.
  uint8_t trackInfo(TrackInfo* info) const {
    uint8_t n = 0;
    for (uint8_t k = 0; k < last_.n_targets; k++) {
      const uint8_t slot = last_.t[k].track;
      if (!slot_used_[slot]) continue;
      for (uint8_t i = 0; i < out_.n; i++) {
        const TrackOut& t = out_.t[i];
        if (t.id != slot_id_[slot]) continue;
        TrackInfo& x = info[n++];
        x.track = slot;
        const float fq = static_cast<float>(t.flick) / 0.25f + 0.5f, sq = static_cast<float>(t.spread) / 0.5f + 0.5f;
        x.flick_q = static_cast<uint8_t>(fq > 15 ? 15 : fq); x.spread_q = static_cast<uint8_t>(sq > 15 ? 15 : sq);
        const int wb = static_cast<int>(static_cast<float>(t.width) / 0.025f + 0.5f);
        x.elen = static_cast<uint8_t>(wb < 1 ? 1 : (wb > 16 ? 16 : wb));
        x.mature = t.has_stats && t.hist_n >= 8;
        break;
      }
    }
    return n;
  }

  // Static scene -> 2-bit grey levels; new version when it changed enough (restart the segments).
  void updateBackground() {
    const float* sv = SonarProc::binStatic();
    uint8_t cur[BINS]; int changed = 0;
    for (int i = 0; i < BINS; i++) { cur[i] = staticLevel(sv[i]); if (cur[i] != bg_[i]) changed++; }
    if (!bg_valid_ || changed > BG_CHANGE) {
      memcpy(bg_, cur, BINS);
      if (bg_valid_) bg_ver_ = static_cast<uint8_t>((bg_ver_ + 1) & 15u);
      bg_valid_ = true; bg_seg_ = 0;
    }
  }

  uint8_t node_ = 0;
  uint16_t index_ = 0;
  Ping pend_[MAX_PINGS];
  uint8_t n_pend_ = 0;
  uint16_t since_base_ = 0, since_bg_ = 0;
  uint8_t bg_seg_ = 0, bg_ver_ = 0;
  bool bg_valid_ = false;
  uint8_t bg_[BINS];
  uint32_t data_blocks_ = 0;
  uint16_t act_bits_ = 0;
  bool was_focus_ = false;
  uint8_t last_fish_ = 0;
  uint32_t slot_id_[8];
  bool slot_used_[8];
  Ping last_;
  SonarProc::Out out_;
};

}  // namespace sonar
}  // namespace icemesh
#endif
