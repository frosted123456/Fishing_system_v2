// Sonar block codec (protocol v2, review fixes applied). Every block decodes on its own:
//   - node ID in every block, block length given by the carrier (one SEC_SONAR section or one
//     ESP-NOW frame per block), so a hub drops whole blocks and never truncates one;
//   - first ping index (16 bit) instead of seq x N, so N can change between blocks;
//   - every target carries a 3-bit track ID; absolute on its first ping in the block, delta after;
//   - no pixel layer yet (deferred); residual cells are the strongest moving cells, absolute per ping.
//
// Block types (byte 0 = node, byte 1 = type<<4 | x):
//   BT_BASE  summary every few seconds (x = activity 0-15)          ~7-8 B
//   BT_DATA  N = 1..15 processed pings (x = N-1), + optional track info (echo character)
//   BT_BG    1/8 of the 2-bit static-scene profile (x = bg version)
// Depths in cm (11 bit, 2047 = none). Target strength 0-31 = the display value of the prototype
// (0..1 colour scale x 31); levels 0-3 (none/weak/medium/strong) are derived from it.
#ifndef ICEMESH_SONAR_CODEC_H
#define ICEMESH_SONAR_CODEC_H
#include <stdint.h>
#include <string.h>
#include "bitstream.h"

namespace icemesh {
namespace sonar {

static const uint16_t BINS = 488;           // 2.5 cm bins, 0-12.2 m
static const uint16_t BIN_MM = 25;
static const uint16_t DEPTH_NONE = 2047;    // no bottom lock / no target
static const uint16_t DEPTH_MAX = 2046;
static const uint8_t MAX_TARGETS = 5;       // per ping (3-bit count)
static const uint8_t MAX_RESID = 7;
static const uint8_t MAX_PINGS = 15;        // per DATA block
static const uint8_t BG_SEGMENTS = 8;
static const uint8_t BG_SEG_BINS = 61;      // 8 x 61 = 488
static const uint8_t MAX_BLOCK = 120;       // bytes; fits an ESP-NOW frame and a SEC_SONAR section
static const uint8_t RESID_RICE_K = 6;
static const uint8_t BG_RICE_K = 3;

enum BlockType : uint8_t { BT_NONE = 0, BT_BASE = 1, BT_DATA = 2, BT_BG = 3 };

// track: 0 = the bait (lure), 1-7 = other tracks (IDs reused). width in bins 1-8 (first appearance in a block).
struct Target { uint8_t track; uint16_t depth_cm; uint8_t strength; uint8_t width; uint8_t level; };
inline uint8_t strengthToLevel(uint8_t s) { return s == 0 ? 0 : (s < 11 ? 1 : (s < 21 ? 2 : 3)); }
// Echo character of a track (prototype "Echo character" card), sent with every 2nd DATA block.
struct TrackInfo {
  uint8_t track;
  uint8_t flick_q;     // flicker, 0.25 dB steps (0-15)
  uint8_t spread_q;    // |210 - 190 kHz|, 0.5 dB steps (0-15)
  uint8_t elen;        // echo length, bins 1-16 (2.5 cm)
  bool mature;         // >= 8 detections (else "collecting pings")
};
enum BottomHard : uint8_t { BH_HARD = 0, BH_MEDIUM = 1, BH_SOFT = 2, BH_UNKNOWN = 3 };
struct Resid { uint16_t bin; uint8_t level; };
struct Ping {
  uint16_t index;           // ping counter (wraps)
  uint16_t bottom_cm;       // DEPTH_NONE = no bottom
  uint8_t n_targets;
  Target t[MAX_TARGETS];
  uint8_t n_resid;
  Resid r[MAX_RESID];       // ascending bins
};

struct Summary {
  uint8_t node;
  uint8_t activity;         // 0-15: pings with a fish target over the last window (saturating)
  uint16_t ping;
  uint16_t bottom_cm;
  uint8_t hard;             // BottomHard
  uint8_t n_targets;        // fish (not the bait), 0-7
  uint16_t nearest_cm;      // valid when n_targets > 0
  uint8_t nearest_level;
  uint8_t bg_ver;
};

struct DataHeader {
  uint8_t node;
  uint8_t nf_neg;           // noise floor, -dB (84 = -84 dB)
  uint8_t bg_ver;
  uint8_t hard;             // BottomHard
  const TrackInfo* info;    // optional echo character of the tracks
  uint8_t n_info;
};

struct DataBlock {
  uint8_t node;
  uint8_t nf_neg;
  uint8_t bg_ver;
  uint8_t hard;
  uint8_t n;
  Ping pings[MAX_PINGS];
  uint8_t n_info;
  TrackInfo info[MAX_TARGETS];
};

inline uint16_t clampDepth(int32_t cm) { return static_cast<uint16_t>(cm < 0 ? 0 : (cm > DEPTH_MAX ? DEPTH_MAX : cm)); }
inline uint16_t depthToBin(uint16_t cm) { const uint32_t b = static_cast<uint32_t>(cm) * 10u / BIN_MM; return static_cast<uint16_t>(b >= BINS ? BINS - 1 : b); }
inline uint16_t binToDepth(uint16_t bin) { return static_cast<uint16_t>(static_cast<uint32_t>(bin) * BIN_MM / 10u); }

// Reads node and type of a block without decoding it.
inline bool peekBlock(const uint8_t* in, size_t len, uint8_t& node, uint8_t& type) {
  if (len < 2) return false;
  node = in[0]; type = static_cast<uint8_t>(in[1] >> 4);
  return type == BT_BASE || type == BT_DATA || type == BT_BG;
}

// ---- BASE summary -----------------------------------------------------------------------------
inline size_t encodeSummary(const Summary& s, uint8_t* out, size_t cap) {
  BitWriter w(out, cap);
  w.put(s.node, 8); w.put(BT_BASE, 4); w.put(s.activity > 15 ? 15 : s.activity, 4);
  w.put(s.ping, 16);
  w.put(s.bottom_cm > DEPTH_NONE ? DEPTH_NONE : s.bottom_cm, 11);
  w.put(s.hard & 3u, 2);
  const uint8_t nt = s.n_targets > 7 ? 7 : s.n_targets;
  w.put(nt, 3);
  if (nt > 0) { w.put(s.nearest_cm > DEPTH_MAX ? DEPTH_MAX : s.nearest_cm, 11); w.put(s.nearest_level & 3u, 2); }
  w.put(s.bg_ver & 15u, 4);
  return w.overflow() ? 0 : w.bytes();
}

inline bool decodeSummary(const uint8_t* in, size_t len, Summary& s) {
  memset(&s, 0, sizeof(s));
  BitReader r(in, len);
  s.node = static_cast<uint8_t>(r.get(8));
  if (r.get(4) != BT_BASE) return false;
  s.activity = static_cast<uint8_t>(r.get(4));
  s.ping = static_cast<uint16_t>(r.get(16));
  s.bottom_cm = static_cast<uint16_t>(r.get(11));
  s.hard = static_cast<uint8_t>(r.get(2));
  s.n_targets = static_cast<uint8_t>(r.get(3));
  if (s.n_targets > 0) { s.nearest_cm = static_cast<uint16_t>(r.get(11)); s.nearest_level = static_cast<uint8_t>(r.get(2)); }
  else s.nearest_cm = DEPTH_NONE;
  s.bg_ver = static_cast<uint8_t>(r.get(4));
  return !r.bad() && r.bitsLeft() < 8;
}

// ---- DATA (N pings) -----------------------------------------------------------------------------
namespace detail {
struct TrackMemo { bool seen[8]; uint16_t depth[8]; };

inline void putPing(BitWriter& w, const Ping& p, bool first, uint16_t& prev_bottom, TrackMemo& tm) {
  // bottom: absolute on the first ping, then 4-bit delta (+-7 cm), -8 = escape + absolute
  const uint16_t b = p.bottom_cm > DEPTH_NONE ? DEPTH_NONE : p.bottom_cm;
  if (first) w.put(b, 11);
  else {
    const int32_t d = static_cast<int32_t>(b) - static_cast<int32_t>(prev_bottom);
    const bool small = b != DEPTH_NONE && prev_bottom != DEPTH_NONE && d >= -7 && d <= 7;
    if (small) w.putSigned(d, 4); else { w.putSigned(-8, 4); w.put(b, 11); }
  }
  prev_bottom = b;
  const uint8_t nt = p.n_targets > MAX_TARGETS ? MAX_TARGETS : p.n_targets;
  const uint8_t nr = p.n_resid > MAX_RESID ? MAX_RESID : p.n_resid;
  w.put(nt, 3); w.put(nr, 3);
  for (uint8_t i = 0; i < nt; i++) {
    const Target& t = p.t[i];
    const uint8_t id = t.track & 7u;
    const uint16_t dep = t.depth_cm > DEPTH_MAX ? DEPTH_MAX : t.depth_cm;
    const uint8_t st = t.strength > 31 ? 31 : t.strength;
    w.put(id, 3);
    if (!tm.seen[id]) {
      w.put(dep, 11); w.put(st, 5);
      w.put(static_cast<uint32_t>((t.width < 1 ? 1 : (t.width > 8 ? 8 : t.width)) - 1), 3);
      tm.seen[id] = true;
    } else {
      const int32_t d = static_cast<int32_t>(dep) - static_cast<int32_t>(tm.depth[id]);
      if (d >= -31 && d <= 31) w.putSigned(d, 6); else { w.putSigned(-32, 6); w.put(dep, 11); }
      w.put(st, 5);
    }
    tm.depth[id] = dep;
  }
  int32_t prev = -1;
  for (uint8_t i = 0; i < nr; i++) {
    const uint16_t bin = p.r[i].bin >= BINS ? BINS - 1 : p.r[i].bin;
    const int32_t gap = static_cast<int32_t>(bin) - prev - 1;
    w.putRice(static_cast<uint32_t>(gap < 0 ? 0 : gap), RESID_RICE_K);
    w.put(p.r[i].level & 3u, 2);
    prev = gap < 0 ? prev + 1 : bin;
  }
}
}  // namespace detail

// Encodes as many of the n pings as fit in `cap` (and MAX_PINGS). Pings must have consecutive
// indices starting at pings[0].index. Track info is added after the pings when it fits (else dropped).
// Returns the length (0 = not even one ping fits); n_used = pings encoded.
inline size_t encodeData(const DataHeader& h, const Ping* pings, uint8_t n, uint8_t* out, size_t cap, uint8_t* n_used = nullptr) {
  if (n_used) *n_used = 0;
  if (n == 0) return 0;
  if (n > MAX_PINGS) n = MAX_PINGS;
  BitWriter w(out, cap);
  w.put(h.node, 8); w.put(BT_DATA, 4);
  const size_t n_at = w.bits();
  w.put(0, 4);
  w.put(pings[0].index, 16); w.put(h.nf_neg, 8); w.put(h.bg_ver & 15u, 4); w.put(h.hard & 3u, 2);
  const size_t info_flag_at = w.bits();
  w.put(0, 1); w.put(0, 1);
  if (w.overflow()) return 0;
  detail::TrackMemo tm;
  memset(&tm, 0, sizeof(tm));
  uint16_t prev_bottom = DEPTH_NONE;
  uint8_t done = 0;
  for (uint8_t i = 0; i < n; i++) {
    const size_t mark = w.bits();
    const detail::TrackMemo tm_save = tm;
    const uint16_t pb_save = prev_bottom;
    detail::putPing(w, pings[i], i == 0, prev_bottom, tm);
    if (w.overflow()) { w.rewind(mark); tm = tm_save; prev_bottom = pb_save; break; }
    done++;
  }
  if (done == 0) return 0;
  w.patch(n_at, static_cast<uint32_t>(done - 1), 4);
  if (h.info != nullptr && h.n_info > 0) {
    const size_t mark = w.bits();
    const uint8_t ni = h.n_info > MAX_TARGETS ? MAX_TARGETS : h.n_info;
    w.put(ni, 3);
    for (uint8_t i = 0; i < ni; i++) {
      const TrackInfo& t = h.info[i];
      w.put(t.track & 7u, 3); w.put(t.flick_q > 15 ? 15 : t.flick_q, 4); w.put(t.spread_q > 15 ? 15 : t.spread_q, 4);
      w.put(static_cast<uint32_t>((t.elen < 1 ? 1 : (t.elen > 16 ? 16 : t.elen)) - 1), 4); w.put(t.mature ? 1 : 0, 1);
    }
    if (w.overflow()) w.rewind(mark); else w.patch(info_flag_at, 1, 1);
  }
  if (n_used) *n_used = done;
  return w.bytes();
}

inline bool decodeData(const uint8_t* in, size_t len, DataBlock& d) {
  memset(&d, 0, sizeof(d));
  BitReader r(in, len);
  d.node = static_cast<uint8_t>(r.get(8));
  if (r.get(4) != BT_DATA) return false;
  d.n = static_cast<uint8_t>(r.get(4) + 1);
  const uint16_t ping0 = static_cast<uint16_t>(r.get(16));
  d.nf_neg = static_cast<uint8_t>(r.get(8));
  d.bg_ver = static_cast<uint8_t>(r.get(4));
  d.hard = static_cast<uint8_t>(r.get(2));
  const bool has_info = r.get(1) != 0;
  r.get(1);
  if (d.n > MAX_PINGS) return false;
  detail::TrackMemo tm;
  memset(&tm, 0, sizeof(tm));
  uint16_t prev_bottom = DEPTH_NONE;
  for (uint8_t i = 0; i < d.n; i++) {
    Ping& p = d.pings[i];
    p.index = static_cast<uint16_t>(ping0 + i);
    if (i == 0) p.bottom_cm = static_cast<uint16_t>(r.get(11));
    else {
      const int32_t dd = r.getSigned(4);
      if (dd == -8) p.bottom_cm = static_cast<uint16_t>(r.get(11));
      else {
        const int32_t v = static_cast<int32_t>(prev_bottom) + dd;
        if (prev_bottom == DEPTH_NONE || v < 0 || v > DEPTH_MAX) return false;
        p.bottom_cm = static_cast<uint16_t>(v);
      }
    }
    prev_bottom = p.bottom_cm;
    p.n_targets = static_cast<uint8_t>(r.get(3));
    p.n_resid = static_cast<uint8_t>(r.get(3));
    if (p.n_targets > MAX_TARGETS) return false;
    for (uint8_t k = 0; k < p.n_targets; k++) {
      Target& t = p.t[k];
      t.track = static_cast<uint8_t>(r.get(3));
      if (!tm.seen[t.track]) {
        t.depth_cm = static_cast<uint16_t>(r.get(11)); t.strength = static_cast<uint8_t>(r.get(5));
        t.width = static_cast<uint8_t>(r.get(3) + 1);
        tm.seen[t.track] = true;
      } else {
        const int32_t dd = r.getSigned(6);
        if (dd == -32) t.depth_cm = static_cast<uint16_t>(r.get(11));
        else {
          const int32_t v = static_cast<int32_t>(tm.depth[t.track]) + dd;
          if (v < 0) return false;
          t.depth_cm = static_cast<uint16_t>(v);
        }
        t.strength = static_cast<uint8_t>(r.get(5));
        t.width = 0;   // filled below from the first appearance
      }
      t.level = strengthToLevel(t.strength);
      if (t.depth_cm > DEPTH_MAX) return false;
      tm.depth[t.track] = t.depth_cm;
    }
    int32_t prev = -1;
    for (uint8_t k = 0; k < p.n_resid; k++) {
      const uint32_t gap = r.getRice(RESID_RICE_K, 16);
      const int32_t bin = prev + 1 + static_cast<int32_t>(gap);
      if (bin >= BINS) return false;
      p.r[k].bin = static_cast<uint16_t>(bin);
      p.r[k].level = static_cast<uint8_t>(r.get(2));
      prev = bin;
    }
    if (r.bad()) return false;
  }
  if (has_info) {
    d.n_info = static_cast<uint8_t>(r.get(3));
    if (d.n_info > MAX_TARGETS) return false;
    for (uint8_t i = 0; i < d.n_info; i++) {
      TrackInfo& t = d.info[i];
      t.track = static_cast<uint8_t>(r.get(3)); t.flick_q = static_cast<uint8_t>(r.get(4)); t.spread_q = static_cast<uint8_t>(r.get(4));
      t.elen = static_cast<uint8_t>(r.get(4) + 1); t.mature = r.get(1) != 0;
    }
  }
  // width is sent once per track per block: copy it to the later pings
  uint8_t width[8];
  memset(width, 0, sizeof(width));
  for (uint8_t i = 0; i < d.n; i++)
    for (uint8_t k = 0; k < d.pings[i].n_targets; k++) {
      Target& t = d.pings[i].t[k];
      if (t.width) width[t.track] = t.width; else t.width = width[t.track] ? width[t.track] : 1;
    }
  return !r.bad() && r.bitsLeft() < 8;
}

// ---- BG (background profile segment) ---------------------------------------------------------------
// levels: BINS values 0-3. Segment s covers bins [s*61, s*61+61). Run-length: level (2 b) + Rice(run-1).
inline size_t encodeBgSegment(uint8_t node, uint8_t ver, uint8_t seg, const uint8_t* levels, uint8_t* out, size_t cap) {
  if (seg >= BG_SEGMENTS) return 0;
  BitWriter w(out, cap);
  w.put(node, 8); w.put(BT_BG, 4); w.put(ver & 15u, 4); w.put(seg, 3);
  const uint16_t start = static_cast<uint16_t>(seg * BG_SEG_BINS);
  uint16_t i = 0;
  while (i < BG_SEG_BINS) {
    const uint8_t lv = levels[start + i] & 3u;
    uint16_t run = 1;
    while (i + run < BG_SEG_BINS && (levels[start + i + run] & 3u) == lv) run++;
    w.put(lv, 2); w.putRice(static_cast<uint32_t>(run - 1), BG_RICE_K);
    i = static_cast<uint16_t>(i + run);
  }
  return w.overflow() ? 0 : w.bytes();
}

// Writes the segment into levels_out[BINS]. Returns false on a malformed block (levels_out untouched).
inline bool decodeBgSegment(const uint8_t* in, size_t len, uint8_t& node, uint8_t& ver, uint8_t& seg, uint8_t* levels_out) {
  BitReader r(in, len);
  node = static_cast<uint8_t>(r.get(8));
  if (r.get(4) != BT_BG) return false;
  ver = static_cast<uint8_t>(r.get(4));
  seg = static_cast<uint8_t>(r.get(3));
  if (seg >= BG_SEGMENTS) return false;
  uint8_t tmp[BG_SEG_BINS];
  uint16_t i = 0;
  while (i < BG_SEG_BINS) {
    const uint8_t lv = static_cast<uint8_t>(r.get(2));
    const uint32_t run = r.getRice(BG_RICE_K, 16) + 1;
    if (r.bad() || i + run > BG_SEG_BINS) return false;
    for (uint32_t k = 0; k < run; k++) tmp[i + k] = lv;
    i = static_cast<uint16_t>(i + run);
  }
  if (r.bitsLeft() >= 8) return false;
  memcpy(levels_out + seg * BG_SEG_BINS, tmp, BG_SEG_BINS);
  return true;
}

}  // namespace sonar
}  // namespace icemesh
#endif
