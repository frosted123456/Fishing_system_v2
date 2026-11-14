// Sonar block codec (protocol v2, review fixes applied). Every block decodes on its own:
//   - node ID in every block, block length given by the carrier (one SEC_SONAR section or one
//     ESP-NOW frame per block), so a hub drops whole blocks and never truncates one;
//   - first ping index (16 bit) instead of seq x N, so N can change between blocks;
//   - every target carries a 3-bit track ID; absolute on its first ping in the block, delta after;
//   - no pixel layer yet (deferred); residual cells are the strongest moving cells, absolute per ping.
//
// Block types (byte 0 = node, byte 1 = type<<4 | x):
//   BT_BASE  summary every 2 s (x = activity 0-15): bottom + EVERY target of the latest ping   ~9-16 B
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

enum BlockType : uint8_t { BT_NONE = 0, BT_BASE = 1, BT_DATA = 2, BT_BG = 3, BT_ECHO = 4, BT_HD = 5 };   // D144: BT_HD = high-definition echogram line
// D93 echogram line: the raw ping of the FOCUS hole, 488 bins folded 4:1 (max) into 122 cells of 10 cm, each a 4-bit
// level over the noise floor (0 = at the floor, 15 = 60 dB over, ECHO_DB_PER_LEVEL per step). One line per second in
// focus, 66 bytes: what the hub's A-scan shows, on the chalet, under the detector's marks.
static const uint8_t ECHO_FOLD = 4;
static const uint8_t ECHO_CELLS = 122;        // 488 / 4
static const uint8_t ECHO_BYTES = 61;         // 2 cells per byte, low nibble = the even cell
static const uint8_t ECHO_DB_PER_LEVEL = 4;
static const uint8_t ECHO_BLOCK_LEN = 5 + 61;
// D112: scale = cell size, 0 = 10 cm (122 cells = 12.2 m), 1 = 5 cm (6.1 m), 2 = 2.5 cm (3.05 m): the hub picks the
// finest that still covers the bottom (+25 % +0.3 m), so a 0.8 m barrel gets 2.5 cm cells and 5 m of water 5 cm.
static const uint8_t ECHO_SCALES = 3;
struct EchoLine { uint8_t node; uint16_t index; uint8_t nf_neg; uint8_t scale; uint8_t cells[ECHO_BYTES]; uint8_t sharp; uint8_t marks; uint8_t fine; };   // D126: sharp = 1 when the hub sharpened the line (bit 3 of byte 1); D133: marks = 1: cells[] holds an echo-marks payload (bit 2 of byte 1)
// D133 echo-marks payload (in EchoLine.cells, block length 5 + 6 + 3 n):
//   [0] window start bin a, [1] window length n bins (the baseline below is valid over [a, a + n)),
//   [2] floor F and [3] baseline at bin a, both (dB + 128) x 2 (0.5 dB), [4] tau index t (tau = 0.02 x 100^(t/29) m),
//   [5] count of marks, then 3 bytes per mark, little-endian 24 bits: depth in 1/8 bin (12 b) | level over the noise
//   floor in 0.5 dB (7 b) << 12 | width at -6 dB in 1/2 bin (5 b) << 19 (D133b, was 1/4). Baseline(bin i) = 10 log10(F + (B0 - F) r^(i - a)),
//   r = exp(-0.025 / tau), never under the noise floor.
// D142 "show all" line (knob echo 2): the same 122 cells, but the 16 levels are spent where the weak echoes are: 1 dB steps
// from the noise floor to 6 dB, wider above (the plain line's 4 dB steps put a fish 3 dB over the floor in level 0 or 1).
// Flag = bit 7 of the nf byte (the noise floor is never under -127 dB); raw lines only, never sharpened.
inline const uint8_t* echoFineDb() { static const uint8_t T[16] = {0, 1, 2, 3, 4, 5, 6, 8, 10, 13, 16, 20, 25, 31, 38, 46}; return T; }
inline uint8_t echoFineLevel(float over) {   // nearest table entry
  const uint8_t* T = echoFineDb(); uint8_t l = 0;
  while (l < 15 && over >= 0.5f * static_cast<float>(T[l] + T[l + 1])) l++;
  return l;
}
static const uint8_t MARKS_HDR = 6, MARKS_MAX = 18;   // 6 + 3 x 18 = 60 <= ECHO_BYTES
inline uint8_t echoFold(uint8_t scale) { return static_cast<uint8_t>(ECHO_FOLD >> (scale < ECHO_SCALES ? scale : 0)); }   // bins per cell: 4 / 2 / 1
inline uint8_t echoCell(const EchoLine& e, uint8_t i) { return static_cast<uint8_t>(i & 1 ? e.cells[i >> 1] >> 4 : e.cells[i >> 1] & 15u); }
inline void echoSetCell(EchoLine& e, uint8_t i, uint8_t lv) { uint8_t& b = e.cells[i >> 1]; b = static_cast<uint8_t>(i & 1 ? (b & 0x0F) | ((lv & 15u) << 4) : (b & 0xF0) | (lv & 15u)); }
// BASE v4 status flags (D43)
enum : uint8_t { ST_RING = 0x01, ST_NEAR_BAIT = 0x02, ST_BAIT_COVER = 0x04 };

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
  uint16_t nearest_cm;      // valid when n_targets > 0 (derived from the list: fish closest to the bait)
  uint8_t nearest_level;
  uint8_t bg_ver;
  uint8_t status;           // v4 (D43): ST_* flags (ring-down long, fish near the bait, bait in cover)
  uint8_t bottom_snr;       // v4: bottom echo above the noise floor, dB (0-63): level check at setup
  uint8_t n_list;           // targets of the latest ping (fish and bait), for the per-hole sonar glance
  Target list[MAX_TARGETS]; // track 0 = bait, else 1; depth_cm and level only (strength/width not sent)
};

// nearest fish to the bait line (or the shallowest fish when no bait echo), from the list
inline void summaryNearest(Summary& s) {
  s.nearest_cm = DEPTH_NONE; s.nearest_level = 0;
  uint16_t bait = DEPTH_NONE, best = 0xFFFF;
  for (uint8_t i = 0; i < s.n_list; i++) if (s.list[i].track == 0) bait = s.list[i].depth_cm;
  for (uint8_t i = 0; i < s.n_list; i++) {
    const Target& t = s.list[i];
    if (t.track == 0) continue;
    const uint16_t d = bait == DEPTH_NONE ? t.depth_cm : static_cast<uint16_t>(t.depth_cm > bait ? t.depth_cm - bait : bait - t.depth_cm);
    if (d < best) { best = d; s.nearest_cm = t.depth_cm; s.nearest_level = t.level; }
  }
}

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
  return type == BT_BASE || type == BT_DATA || type == BT_BG || type == BT_ECHO || type == BT_HD;
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
  w.put(s.bg_ver & 15u, 4);
  w.put(s.status & 15u, 4);                                   // v4
  w.put(s.bottom_snr > 63 ? 63 : s.bottom_snr, 6);            // v4
  // v3: every target of the latest ping: depth 11 b, level 2 b, bait 1 b (14 b each)
  const uint8_t nl = s.n_list > MAX_TARGETS ? MAX_TARGETS : s.n_list;
  w.put(nl, 3);
  for (uint8_t i = 0; i < nl; i++) {
    w.put(s.list[i].depth_cm > DEPTH_MAX ? DEPTH_MAX : s.list[i].depth_cm, 11);
    w.put(s.list[i].level & 3u, 2);
    w.put(s.list[i].track == 0 ? 1u : 0u, 1);
  }
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
  s.bg_ver = static_cast<uint8_t>(r.get(4));
  s.status = static_cast<uint8_t>(r.get(4));
  s.bottom_snr = static_cast<uint8_t>(r.get(6));
  s.n_list = static_cast<uint8_t>(r.get(3));
  if (s.n_list > MAX_TARGETS) return false;
  for (uint8_t i = 0; i < s.n_list; i++) {
    Target& t = s.list[i];
    t.depth_cm = static_cast<uint16_t>(r.get(11));
    t.level = static_cast<uint8_t>(r.get(2));
    t.track = r.get(1) ? 0 : 1;
  }
  summaryNearest(s);
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

// ---- ECHO (D93 echogram line) ------------------------------------------------------------------------
// [node][BT_ECHO << 4 | scale][index lo][index hi][nf_neg][61 bytes of 4-bit cells]: fixed 66 bytes, no bit packing needed.
inline size_t encodeEcho(const EchoLine& e, uint8_t* out, size_t cap) {
  if (cap < ECHO_BLOCK_LEN) return 0;
  out[0] = e.node; out[1] = static_cast<uint8_t>((BT_ECHO << 4) | (e.scale < ECHO_SCALES ? e.scale : 0) | (e.marks ? 0x04 : 0) | (e.sharp ? 0x08 : 0)); out[2] = static_cast<uint8_t>(e.index); out[3] = static_cast<uint8_t>(e.index >> 8);
  out[4] = static_cast<uint8_t>((e.nf_neg > 127 ? 127 : e.nf_neg) | ((e.fine && !e.marks) ? 0x80 : 0));   // D142: bit 7 = fine levels
  if (e.marks) {   // D133: variable length, only the marks that exist
    const uint8_t nm = e.cells[5] < MARKS_MAX ? e.cells[5] : MARKS_MAX, pl = static_cast<uint8_t>(MARKS_HDR + 3 * nm);
    memcpy(out + 5, e.cells, pl); out[5 + 5] = nm;
    return static_cast<size_t>(5 + pl);
  }
  memcpy(out + 5, e.cells, ECHO_BYTES);
  return ECHO_BLOCK_LEN;
}
inline bool decodeEcho(const uint8_t* in, size_t len, EchoLine& e) {
  if (len < 5 + MARKS_HDR || (in[1] >> 4) != BT_ECHO) return false;
  e.node = in[0]; e.index = static_cast<uint16_t>(in[2] | (in[3] << 8)); e.nf_neg = static_cast<uint8_t>(in[4] & 0x7F); e.fine = (in[4] & 0x80) && !(in[1] & 0x04) ? 1 : 0;   // D142
  e.scale = static_cast<uint8_t>(in[1] & 0x03); if (e.scale >= ECHO_SCALES) e.scale = 0; e.sharp = (in[1] & 0x08) ? 1 : 0;   // D126   // D112 (0 from older hubs)
  e.marks = (in[1] & 0x04) ? 1 : 0;   // D133
  memset(e.cells, 0, ECHO_BYTES);
  if (e.marks) {
    const uint8_t nm = in[5 + 5];
    if (nm > MARKS_MAX || len != static_cast<size_t>(5 + MARKS_HDR + 3 * nm)) return false;
    memcpy(e.cells, in + 5, len - 5);
    return true;
  }
  if (len != ECHO_BLOCK_LEN) return false;
  memcpy(e.cells, in + 5, ECHO_BYTES);
  return true;
}
// bins (dB) -> cells: the strongest bin of each group of 4 / 2 / 1 (D112 scale), as levels over the floor
inline void echoFromBins(EchoLine& e, uint8_t node, uint16_t index, const float* db, float nf, uint8_t scale = 0, bool fine = false) {
  e.node = node; e.index = index; e.scale = scale < ECHO_SCALES ? scale : 0; e.sharp = 0; e.marks = 0; e.fine = fine ? 1 : 0;
  const uint8_t fold = echoFold(e.scale);
  const float n = -nf; e.nf_neg = static_cast<uint8_t>(n < 0 ? 0 : (n > 127 ? 127 : n + 0.5f));   // D142: 7 bits on the air
  memset(e.cells, 0, ECHO_BYTES);
  for (uint8_t c = 0; c < ECHO_CELLS; c++) {
    float m = -1000.f;
    for (uint8_t k = 0; k < fold; k++) { const float v = db[c * fold + k]; if (v > m) m = v; }
    if (fine) { echoSetCell(e, c, echoFineLevel(m - nf)); continue; }   // D142
    const float l = (m - nf) / ECHO_DB_PER_LEVEL + 0.5f;
    echoSetCell(e, c, static_cast<uint8_t>(l < 0 ? 0 : (l > 15 ? 15 : l)));
  }
}

// ---- HD (D144 high-definition echogram line) -----------------------------------------------------------------------
// The raw ping of the FOCUS hole at full depth resolution: one cell per 2.5 cm bin (fold 1; 5 cm = fold 2 when a very
// noisy line would not fit a block), 16 levels over the noise floor on the D142 table (1 dB steps to 6 dB, wider above),
// only from the surface to the bottom + 0.6 m. Empty water costs almost nothing: the cells are sent as
//   [zero run: Rice(kr)] then [cells of an echo: the step from the cell before, zig-zag, Rice(kv)] until a cell is 0 again.
// Block: [node][BT_HD << 4][index lo][index hi][nf_neg], then bits: cells n (9) | fold - 1 (1) | all (1) | kr (3) | kv (2),
// then the tokens. 12 Nov study (docs/LORA_COMPRESSION_2026-11-12.md): 36 B median for 5 m of water, 23 B in the bin,
// against 66 B for the 10 cm / 4 dB line; a context model + table coding gave the same size on quiet water.
// all = 1: levels from the floor (knob echo 2 "show all"); 0 = cells under the cut were set to 0 (texture under the marks).
static const uint8_t HD_HDR = 5;
static const float HD_BELOW_M = 0.6f;          // water sent below the bottom (the bottom echo's thickness)
struct HdLine { uint8_t node; uint16_t index; uint8_t nf_neg; uint8_t fold; uint8_t all; uint16_t n; uint8_t lv[BINS]; };
inline size_t encodeHd(const HdLine& h, uint8_t* out, size_t cap) {
  if (cap < static_cast<size_t>(HD_HDR + 3) || h.n == 0 || (h.fold != 1 && h.fold != 2) || static_cast<uint32_t>(h.n) * h.fold > BINS) return 0;
  uint32_t cr[8], cv[4];
  for (uint8_t k = 0; k < 8; k++) cr[k] = 0;
  for (uint8_t k = 0; k < 4; k++) cv[k] = 0;
  for (uint8_t pass = 0; pass < 2; pass++) {   // pass 0: cost of every Rice parameter; pass 1: write with the cheapest
    uint8_t kr = 0, kv = 0;
    if (pass == 1) { for (uint8_t k = 1; k < 8; k++) if (cr[k] < cr[kr]) kr = k; for (uint8_t k = 1; k < 4; k++) if (cv[k] < cv[kv]) kv = k; }
    BitWriter w(out + HD_HDR, pass == 1 ? cap - HD_HDR : 0);
    if (pass == 1) { w.put(h.n, 9); w.put(static_cast<uint32_t>(h.fold - 1), 1); w.put(h.all ? 1u : 0u, 1); w.put(kr, 3); w.put(kv, 2); }
    uint16_t i = 0;
    while (i < h.n) {
      uint16_t j = i;
      while (j < h.n && (h.lv[j] & 15u) == 0) j++;
      const uint32_t run = static_cast<uint32_t>(j - i);
      if (pass == 0) { for (uint8_t k = 0; k < 8; k++) cr[k] += (run >> k) + 1u + k; } else w.putRice(run, kr);
      i = j;
      int32_t prev = 0;
      while (i < h.n) {
        const int32_t v = static_cast<int32_t>(h.lv[i] & 15u), d = v - prev;
        const uint32_t z = d >= 0 ? static_cast<uint32_t>(2 * d) : static_cast<uint32_t>(-2 * d - 1);
        if (pass == 0) { for (uint8_t k = 0; k < 4; k++) cv[k] += (z >> k) + 1u + k; } else w.putRice(z, kv);
        prev = v; i++;
        if (v == 0) break;   // back at the floor: the next token is a zero run again
      }
    }
    if (pass == 1) {
      if (w.overflow()) return 0;
      out[0] = h.node; out[1] = static_cast<uint8_t>(BT_HD << 4); out[2] = static_cast<uint8_t>(h.index); out[3] = static_cast<uint8_t>(h.index >> 8);
      out[4] = static_cast<uint8_t>(h.nf_neg > 127 ? 127 : h.nf_neg);
      return HD_HDR + w.bytes();
    }
  }
  return 0;
}
// Header of an HD block + a full check of its tokens. lv = nullptr: check only (the chalet store keeps the block itself and
// needs no 488-byte line); else lv[0..BINS) gets the levels (0 past the last cell).
struct HdInfo { uint8_t node; uint16_t index; uint8_t nf_neg; uint8_t fold; uint8_t all; uint16_t n; };
inline bool hdParse(const uint8_t* in, size_t len, HdInfo& h, uint8_t* lv) {
  if (len < static_cast<size_t>(HD_HDR + 3) || (in[1] >> 4) != BT_HD) return false;
  h.node = in[0]; h.index = static_cast<uint16_t>(in[2] | (in[3] << 8)); h.nf_neg = static_cast<uint8_t>(in[4] & 0x7F);
  BitReader r(in + HD_HDR, len - HD_HDR);
  h.n = static_cast<uint16_t>(r.get(9)); h.fold = static_cast<uint8_t>(r.get(1) + 1); h.all = static_cast<uint8_t>(r.get(1));
  const uint8_t kr = static_cast<uint8_t>(r.get(3)), kv = static_cast<uint8_t>(r.get(2));
  if (h.n == 0 || static_cast<uint32_t>(h.n) * h.fold > BINS) return false;
  uint16_t i = 0;
  while (i < h.n) {
    const uint32_t run = r.getRice(kr, BINS);
    if (r.bad() || run > static_cast<uint32_t>(h.n - i)) return false;
    for (uint32_t k = 0; k < run; k++) { if (lv) lv[i] = 0; i++; }
    int32_t prev = 0;
    while (i < h.n) {
      const uint32_t z = r.getRice(kv, 32);
      if (r.bad()) return false;
      const int32_t v = prev + ((z & 1u) ? -static_cast<int32_t>((z + 1u) >> 1) : static_cast<int32_t>(z >> 1));
      if (v < 0 || v > 15) return false;
      if (lv) lv[i] = static_cast<uint8_t>(v);
      i++; prev = v;
      if (v == 0) break;
    }
  }
  if (lv) for (uint16_t k = h.n; k < BINS; k++) lv[k] = 0;
  return !r.bad() && r.bitsLeft() < 8;
}
inline bool decodeHd(const uint8_t* in, size_t len, HdLine& h) {
  HdInfo o;
  if (!hdParse(in, len, o, h.lv)) return false;
  h.node = o.node; h.index = o.index; h.nf_neg = o.nf_neg; h.fold = o.fold; h.all = o.all; h.n = o.n;
  return true;
}
// bins (dB) -> HD cells: the strongest bin of each cell, fine level over the floor; levels under cut_db are 0 (cut 0 = show all)
inline void hdFromBins(HdLine& h, uint8_t node, uint16_t index, const float* db, float nf, float bottom_m, uint8_t fold, float cut_db) {
  h.node = node; h.index = index; h.fold = fold == 2 ? 2 : 1; h.all = cut_db <= 0.f ? 1 : 0;
  const float n = -nf; h.nf_neg = static_cast<uint8_t>(n < 0 ? 0 : (n > 127 ? 127 : n + 0.5f));
  int32_t nb = bottom_m > 0.f ? static_cast<int32_t>((bottom_m + HD_BELOW_M) / (static_cast<float>(BIN_MM) * 0.001f) + 0.999f) : static_cast<int32_t>(BINS);
  if (nb > BINS) nb = BINS;
  if (nb < 2 * h.fold) nb = 2 * h.fold;
  h.n = static_cast<uint16_t>(nb / h.fold);
  for (uint16_t c = 0; c < h.n; c++) {
    float m = db[c * h.fold];
    if (h.fold == 2 && db[c * 2 + 1] > m) m = db[c * 2 + 1];
    const float over = m - nf;
    h.lv[c] = over < cut_db ? 0 : echoFineLevel(over);
  }
  for (uint16_t c = h.n; c < BINS; c++) h.lv[c] = 0;
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
