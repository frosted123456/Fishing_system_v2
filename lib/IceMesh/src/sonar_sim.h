// Fake sonar for test mode: a seeded scene (bottom, weeds, jigging bait, fish that come and go,
// noise cells) that outputs what the real node processing would output: processed pings
// (bottom, targets with track IDs, residual cells) and a 2-bit background profile.
// SonarSource adds the node-side block cadence (BASE summaries, or FOCUS = DATA + BG blocks)
// so the simulated data goes through the real codec and transport.
// Everything here is invented test data: NOT a model of real fish behaviour or of TUSS4470 output.
#ifndef ICEMESH_SONAR_SIM_H
#define ICEMESH_SONAR_SIM_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "sonar_codec.h"

namespace icemesh {
namespace sonar {

class SonarSim {
 public:
  enum { PINGS_PER_S = 4, MAX_FISH = 2 };

  void begin(uint32_t seed) {
    rng_ = seed ? seed * 2654435761u : 0x9E3779B9u;
    if (rng_ == 0) rng_ = 1;
    t_ = 0;
    bottom0_mm_ = static_cast<int32_t>(range(2500, 9500));    // 2.5-9.5 m
    bait_off_mm_ = static_cast<int32_t>(range(300, 1200));    // bait 30-120 cm above bottom
    weed_mm_ = static_cast<int32_t>(range(0, 450));
    jig_wait_ = static_cast<uint16_t>(range(20, 40));
    jig_t_ = 255;
    next_track_ = 1;
    memset(fish_, 0, sizeof(fish_));
    for (uint8_t i = 0; i < sizeof(weed_pat_); i++) weed_pat_[i] = static_cast<uint8_t>(rnd() & 0xFF);
    bg_ver_ = 0;
    bg_bottom_bin_ = 0xFFFF;
    bottom_mm_ = bottom0_mm_;
    rebuildBackground();
  }

  // Advances one ping (1/4 s) and returns it.
  void step(Ping& p) {
    memset(&p, 0, sizeof(p));
    p.index = static_cast<uint16_t>(t_);
    // bottom: slow +-4 cm swell over 90 s; rare loss of bottom lock
    const float ph = static_cast<float>(t_ % 360u) * (6.2831853f / 360.0f);
    bottom_mm_ = bottom0_mm_ + static_cast<int32_t>(40.0f * sinf(ph));
    const bool lost = (rnd() % 400u) == 0;
    p.bottom_cm = lost ? DEPTH_NONE : clampDepth(bottom_mm_ / 10);
    if (abs32(static_cast<int32_t>(depthToBin(clampDepth(bottom_mm_ / 10))) - static_cast<int32_t>(bg_bottom_bin_)) >= 2)
      rebuildBackground();

    // bait: jig every 5-10 s (lift 25 cm in 2 pings, fall back in 8)
    const int32_t rest = bottom_mm_ - bait_off_mm_;
    int32_t bait = rest;
    if (jig_t_ == 255) { if (jig_wait_ > 0) jig_wait_--; else jig_t_ = 0; }
    if (jig_t_ != 255) {
      if (jig_t_ < 2) bait = rest - 125 * (jig_t_ + 1);
      else bait = rest - 250 + 250 * (jig_t_ - 1) / 8;
      if (++jig_t_ >= 10) { jig_t_ = 255; jig_wait_ = static_cast<uint16_t>(range(20, 40)); }
    }
    bait += static_cast<int32_t>(range(0, 20)) - 10;
    bait_mm_ = bait;
    addTarget(p, 0, bait, 2, 1);

    // fish
    for (uint8_t i = 0; i < MAX_FISH; i++) {
      Fish& f = fish_[i];
      if (!f.active) { if ((rnd() % 80u) == 0) spawn(f); continue; }
      moveFish(f);
      if (f.active) addTarget(p, f.track, f.depth_mm, f.level, f.width);
    }

    // residual cells: noise + fish arc tails
    uint16_t bins[MAX_RESID]; uint8_t lv[MAX_RESID]; uint8_t n = 0;
    if ((rnd() % 4u) == 0 && n < MAX_RESID) { bins[n] = static_cast<uint16_t>(range(8, depthToBin(clampDepth(bottom_mm_ / 10)))); lv[n++] = 1; }
    for (uint8_t i = 0; i < MAX_FISH && n < MAX_RESID; i++) {
      const Fish& f = fish_[i];
      if (!f.active || (rnd() & 1u)) continue;
      const int32_t b = static_cast<int32_t>(depthToBin(clampDepth(f.depth_mm / 10))) + ((rnd() & 1u) ? -1 : 1) * (f.width / 2 + 1);
      if (b > 0 && b < BINS) { bins[n] = static_cast<uint16_t>(b); lv[n++] = 1; }
    }
    // sort + unique
    for (uint8_t a = 0; a < n; a++)
      for (uint8_t b = static_cast<uint8_t>(a + 1); b < n; b++)
        if (bins[b] < bins[a]) { uint16_t tb = bins[a]; bins[a] = bins[b]; bins[b] = tb; uint8_t tl = lv[a]; lv[a] = lv[b]; lv[b] = tl; }
    for (uint8_t a = 0; a < n; a++) {
      if (p.n_resid > 0 && p.r[p.n_resid - 1].bin == bins[a]) continue;
      p.r[p.n_resid].bin = bins[a]; p.r[p.n_resid].level = lv[a]; p.n_resid++;
    }
    t_++;
  }

  const uint8_t* background() const { return bg_; }
  uint8_t bgVersion() const { return bg_ver_; }
  uint16_t nextIndex() const { return static_cast<uint16_t>(t_); }
  int32_t baitMm() const { return bait_mm_; }
  uint8_t fishCount() const { uint8_t n = 0; for (uint8_t i = 0; i < MAX_FISH; i++) n += fish_[i].active ? 1 : 0; return n; }

 private:
  struct Fish { bool active; uint8_t track, level, width, phase; int32_t depth_mm, goal_mm; int16_t speed; uint16_t stay; };

  uint32_t rnd() { rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5; return rng_; }
  uint32_t range(uint32_t lo, uint32_t hi) { return hi <= lo ? lo : lo + rnd() % (hi - lo); }
  static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

  static void addTarget(Ping& p, uint8_t track, int32_t depth_mm, uint8_t level, uint8_t width) {
    if (p.n_targets >= MAX_TARGETS || level == 0) return;
    Target& t = p.t[p.n_targets++];
    t.track = track; t.depth_cm = clampDepth(depth_mm / 10); t.level = level; t.width = width;
  }

  void spawn(Fish& f) {
    f.active = true;
    f.track = next_track_;
    next_track_ = static_cast<uint8_t>(next_track_ >= 7 ? 1 : next_track_ + 1);
    f.level = (rnd() % 10u) < 6 ? 3 : 2;
    f.width = static_cast<uint8_t>(range(2, 6));
    f.phase = 0;
    const bool from_below = (rnd() & 1u) != 0;
    f.depth_mm = from_below ? bottom_mm_ - static_cast<int32_t>(range(100, 400)) : bait_mm_ - static_cast<int32_t>(range(600, 1500));
    if (f.depth_mm < 300) f.depth_mm = 300;
    f.goal_mm = bait_mm_ + static_cast<int32_t>(range(0, 200)) - 100;
    f.speed = static_cast<int16_t>(range(10, 30));     // mm per ping (4-12 cm/s)
    f.stay = static_cast<uint16_t>(range(40, 160));    // 10-40 s near the bait
  }

  void moveFish(Fish& f) {
    switch (f.phase) {
      case 0: {   // approach
        const int32_t d = f.goal_mm - f.depth_mm;
        if (abs32(d) <= f.speed) { f.depth_mm = f.goal_mm; f.phase = 1; }
        else f.depth_mm += d > 0 ? f.speed : -f.speed;
        break;
      }
      case 1: {   // hover near the bait, follows a jig, sometimes strikes
        f.goal_mm = bait_mm_ + 80;
        const int32_t d = f.goal_mm - f.depth_mm;
        f.depth_mm += d / 3 + static_cast<int32_t>(range(0, 30)) - 15;
        if (f.stay > 0) f.stay--; else { f.phase = 2; f.goal_mm = (rnd() & 1u) ? bottom_mm_ - 100 : f.depth_mm - 1500; }
        break;
      }
      default: {  // leave, fade out
        const int32_t d = f.goal_mm - f.depth_mm;
        f.depth_mm += d > 0 ? 40 : -40;
        if (abs32(d) < 200) f.level = f.level > 1 ? static_cast<uint8_t>(f.level - 1) : 0;
        if (abs32(d) <= 40 || f.level == 0) f.active = false;
        break;
      }
    }
    if (f.depth_mm < 200) f.depth_mm = 200;
    if (f.depth_mm > bottom_mm_ - 50) f.depth_mm = bottom_mm_ - 50;
  }

  void rebuildBackground() {
    const uint16_t b = depthToBin(clampDepth(bottom_mm_ / 10));
    memset(bg_, 0, sizeof(bg_));
    for (uint16_t i = 0; i < 4; i++) bg_[i] = 2;            // transducer ring-down
    for (uint16_t i = 4; i < 8; i++) bg_[i] = 1;
    const uint16_t weed_bins = static_cast<uint16_t>(weed_mm_ / static_cast<int32_t>(BIN_MM));
    for (uint16_t k = 1; k <= weed_bins && k < b; k++) {
      const uint8_t r = weed_pat_[k % sizeof(weed_pat_)];
      bg_[b - k] = (k < weed_bins / 2 && (r & 3u) == 0) ? 2 : ((r & 1u) ? 1 : 0);
    }
    for (uint16_t i = b; i < BINS; i++) {
      const uint16_t k = static_cast<uint16_t>(i - b);
      bg_[i] = k < 4 ? 3 : (k < 15 ? 2 : (k < 40 ? 1 : 0));
    }
    if (bg_bottom_bin_ != 0xFFFF) bg_ver_ = static_cast<uint8_t>((bg_ver_ + 1) & 15u);
    bg_bottom_bin_ = b;
  }

  uint32_t rng_ = 1;
  uint32_t t_ = 0;
  int32_t bottom0_mm_ = 5000, bottom_mm_ = 5000, bait_off_mm_ = 600, bait_mm_ = 4400, weed_mm_ = 0;
  uint16_t jig_wait_ = 30;
  uint8_t jig_t_ = 255;
  uint8_t next_track_ = 1;
  Fish fish_[MAX_FISH];
  uint8_t weed_pat_[32];
  uint8_t bg_[BINS];
  uint8_t bg_ver_ = 0;
  uint16_t bg_bottom_bin_ = 0xFFFF;
};

struct Block { uint8_t len; uint8_t data[MAX_BLOCK]; };

// Node-side cadence. Call tick() at 4 Hz (one ping each).
//   BASE  (not focus): a summary every BASE_EVERY pings, and at once when a fish shows up.
//   FOCUS: a DATA block every FOCUS_N pings + one BG segment every BG_EVERY pings (full profile in 16 s).
class SonarSource {
 public:
  enum { BASE_EVERY = 16, FOCUS_N = 4, BG_EVERY = 8, GAIN_SIM = 0x80 };
  SonarSim sim;
  uint32_t blocks_out = 0, pings_dropped = 0;

  void begin(uint8_t node, uint32_t seed) {
    node_ = node; sim.begin(seed);
    n_pend_ = 0; since_base_ = BASE_EVERY; since_bg_ = BG_EVERY; bg_seg_ = 0; act_bits_ = 0;
    was_focus_ = false; last_bg_ver_ = sim.bgVersion(); last_fish_ = 0;
    memset(&last_, 0, sizeof(last_));
  }

  // One ping. Writes up to `max_out` blocks; returns how many.
  uint8_t tick(bool focus, Block* out, uint8_t max_out) {
    uint8_t n_out = 0;
    Ping p;
    sim.step(p);
    last_ = p;
    uint8_t fish = 0;
    for (uint8_t i = 0; i < p.n_targets; i++) if (p.t[i].track != 0) fish++;
    act_bits_ = static_cast<uint16_t>((act_bits_ << 1) | (fish ? 1u : 0u));
    if (focus != was_focus_) { n_pend_ = 0; bg_seg_ = 0; since_bg_ = BG_EVERY; was_focus_ = focus; }
    if (sim.bgVersion() != last_bg_ver_) { last_bg_ver_ = sim.bgVersion(); bg_seg_ = 0; since_bg_ = BG_EVERY; }

    if (focus) {
      pend_[n_pend_++] = p;
      if (n_pend_ >= FOCUS_N && n_out < max_out) {
        uint8_t used = 0;
        const size_t len = encodeData(node_, GAIN_SIM, sim.bgVersion(), pend_, n_pend_, out[n_out].data, MAX_BLOCK, &used);
        if (len > 0) { out[n_out].len = static_cast<uint8_t>(len); n_out++; blocks_out++; }
        pings_dropped += static_cast<uint32_t>(n_pend_ - used);
        n_pend_ = 0;
      }
      if (++since_bg_ >= BG_EVERY && n_out < max_out) {
        const size_t len = encodeBgSegment(node_, sim.bgVersion(), bg_seg_, sim.background(), out[n_out].data, MAX_BLOCK);
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

  // Summary of the latest ping (nearest fish to the bait; the bait itself is track 0).
  Summary summary() const {
    Summary s;
    memset(&s, 0, sizeof(s));
    s.node = node_; s.ping = last_.index; s.bottom_cm = last_.bottom_cm; s.bg_ver = sim.bgVersion();
    uint16_t bits = act_bits_; uint8_t a = 0;
    while (bits) { a = static_cast<uint8_t>(a + (bits & 1u)); bits >>= 1; }
    s.activity = a > 15 ? 15 : a;
    s.nearest_cm = DEPTH_NONE;
    int32_t best = 1 << 30;
    const int32_t bait_cm = sim.baitMm() / 10;
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

 private:
  uint8_t node_ = 0;
  Ping pend_[MAX_PINGS];
  uint8_t n_pend_ = 0;
  uint16_t since_base_ = 0, since_bg_ = 0;
  uint8_t bg_seg_ = 0;
  uint16_t act_bits_ = 0;
  bool was_focus_ = false;
  uint8_t last_bg_ver_ = 0;
  uint8_t last_fish_ = 0;
  Ping last_;
};

}  // namespace sonar
}  // namespace icemesh
#endif
