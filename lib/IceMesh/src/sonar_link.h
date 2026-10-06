// Sonar blocks over the mesh.
//   Hub:    SonarOutbox  - blocks from nodes (ESP-NOW) or virtual nodes wait here; each hub packet takes
//                          whole blocks (one SEC_SONAR section each) while they fit. No ACK: best effort.
//   Chalet: SonarStore   - decodes blocks: per-node summary + background profile, and a ring of pings
//                          (FOCUS stream) that the web page polls with a sequence number.
#ifndef ICEMESH_SONAR_LINK_H
#define ICEMESH_SONAR_LINK_H
#include <stdint.h>
#include <string.h>
#include "seq.h"
#include "tdma_proto.h"
#include "sonar_codec.h"

namespace icemesh {
namespace sonar {

class SonarOutbox {
 public:
  enum { CAP = 12, MAX_AGE_FRAMES = 4 };   // a block older than ~4 s is stale for the display (est.)
  uint32_t pushed = 0, sent = 0, dropped = 0, expired = 0;

  SonarOutbox() { clear(); }
  void clear() { memset(items_, 0, sizeof(items_)); order_ = 0; }

  // Queues one block. BASE replaces the node's older BASE, BG replaces the same node+segment.
  bool push(const uint8_t* blk, uint8_t len, uint16_t frame) {
    uint8_t node, type;
    if (len == 0 || len > MAX_BLOCK || !peekBlock(blk, len, node, type)) { dropped++; return false; }
    const uint8_t seg = (type == BT_BG && len >= 3) ? static_cast<uint8_t>(blk[2] >> 5) : 0;
    int slot = -1;
    for (uint8_t i = 0; i < CAP && slot < 0; i++) {
      const Item& it = items_[i];
      if (it.used && it.node == node && it.type == type && type != BT_DATA && (type != BT_BG || it.seg == seg)) slot = i;
    }
    for (uint8_t i = 0; i < CAP && slot < 0; i++) if (!items_[i].used) slot = i;
    if (slot < 0) { slot = victim(); dropped++; }
    Item& it = items_[slot];
    it.used = true; it.node = node; it.type = type; it.seg = seg; it.len = len; it.frame = frame; it.order = ++order_;
    memcpy(it.data, blk, len);
    pushed++;
    return true;
  }

  // Adds whole blocks as SEC_SONAR sections: BASE first, then DATA oldest first, then BG.
  uint16_t fill(tdma::PacketWriter& w, uint16_t frame) {
    expire(frame);
    uint16_t added = 0;
    static const uint8_t prio[3] = {BT_BASE, BT_DATA, BT_BG};
    for (uint8_t p = 0; p < 3; p++) {
      for (;;) {
        int best = -1;
        for (uint8_t i = 0; i < CAP; i++) {
          const Item& it = items_[i];
          if (!it.used || it.type != prio[p] || it.len > w.freeForValue()) continue;
          if (best < 0 || it.order < items_[best].order) best = i;
        }
        if (best < 0) break;
        Item& it = items_[best];
        if (!w.add(tdma::SEC_SONAR, it.data, it.len)) break;
        added = static_cast<uint16_t>(added + it.len + 2);
        it.used = false;
        sent++;
      }
    }
    return added;
  }

  uint8_t count() const { uint8_t n = 0; for (uint8_t i = 0; i < CAP; i++) n += items_[i].used ? 1 : 0; return n; }

 private:
  struct Item { bool used; uint8_t node, type, seg, len; uint16_t frame; uint32_t order; uint8_t data[MAX_BLOCK]; };

  void expire(uint16_t frame) {
    for (uint8_t i = 0; i < CAP; i++)
      if (items_[i].used && seqDiff(frame, items_[i].frame) > static_cast<int16_t>(MAX_AGE_FRAMES)) { items_[i].used = false; expired++; }
  }
  // Oldest block of the least useful type: BG, then DATA, then BASE.
  int victim() const {
    static const uint8_t order[3] = {BT_BG, BT_DATA, BT_BASE};
    for (uint8_t p = 0; p < 3; p++) {
      int best = -1;
      for (uint8_t i = 0; i < CAP; i++)
        if (items_[i].used && items_[i].type == order[p] && (best < 0 || items_[i].order < items_[best].order)) best = i;
      if (best >= 0) return best;
    }
    return 0;
  }

  Item items_[CAP];
  uint32_t order_;
};

// Interface the chalet RX path calls for each SEC_SONAR section.
class SonarSink {
 public:
  virtual ~SonarSink() {}
  virtual void onSonarBlock(uint8_t hub, const uint8_t* blk, uint8_t len, uint16_t frame) = 0;
};

struct NodeSonar {
  bool used;
  uint8_t node, hub;
  uint16_t frame;           // last frame anything arrived
  bool has_sum;
  Summary sum;              // from BASE, or derived from the latest DATA ping
  bool has_ping;
  uint16_t last_ping;
  uint8_t bg_ver;
  uint8_t bg_mask;          // segments received for bg_ver
  uint8_t gain;
  uint16_t act_bits;         // fish present in the last 16 DATA pings (activity when streaming)
  uint8_t bg[BINS];
};

struct StoredPing { uint32_t seq; uint8_t node; Ping p; };

template <uint8_t MAXN = 16, uint16_t RING = 128>
class SonarStore : public SonarSink {
 public:
  uint32_t blocks_ok = 0, blocks_bad = 0, old_pings = 0;

  SonarStore() { clear(); }
  void clear() { memset(nodes_, 0, sizeof(nodes_)); head_ = 0; count_ = 0; seq_ = 0; }

  void onSonarBlock(uint8_t hub, const uint8_t* blk, uint8_t len, uint16_t frame) override {
    uint8_t node, type;
    if (!peekBlock(blk, len, node, type)) { blocks_bad++; return; }
    NodeSonar* ns = slot(node, frame);
    if (ns == nullptr) { blocks_bad++; return; }
    switch (type) {
      case BT_BASE: {
        Summary s;
        if (!decodeSummary(blk, len, s)) { blocks_bad++; return; }
        if (ns->has_sum && isOld(s.ping, ns->sum.ping) && s.ping != ns->sum.ping) { old_pings++; return; }
        ns->sum = s; ns->has_sum = true;
        break;
      }
      case BT_DATA: {
        DataBlock d;
        if (!decodeData(blk, len, d)) { blocks_bad++; return; }
        ns->gain = d.gain;
        for (uint8_t i = 0; i < d.n; i++) {
          const Ping& p = d.pings[i];
          if (ns->has_ping && isOld(p.index, ns->last_ping)) { old_pings++; continue; }   // duplicate (relay) or late
          ns->last_ping = p.index; ns->has_ping = true;
          StoredPing& sp = ring_[head_];
          sp.seq = ++seq_; sp.node = node; sp.p = p;
          head_ = static_cast<uint16_t>((head_ + 1) % RING);
          if (count_ < RING) count_++;
          deriveSummary(*ns, p, d.bg_ver);
        }
        break;
      }
      case BT_BG: {
        uint8_t n2, ver, seg;
        uint8_t tmp[BINS];
        memcpy(tmp, ns->bg, BINS);
        if (!decodeBgSegment(blk, len, n2, ver, seg, tmp)) { blocks_bad++; return; }
        if (ver != ns->bg_ver) { ns->bg_ver = ver; ns->bg_mask = 0; }
        memcpy(ns->bg + seg * BG_SEG_BINS, tmp + seg * BG_SEG_BINS, BG_SEG_BINS);
        ns->bg_mask = static_cast<uint8_t>(ns->bg_mask | (1u << seg));
        break;
      }
      default: blocks_bad++; return;
    }
    ns->hub = hub; ns->frame = frame;
    blocks_ok++;
  }

  const NodeSonar* find(uint8_t node) const {
    for (uint8_t i = 0; i < MAXN; i++) if (nodes_[i].used && nodes_[i].node == node) return &nodes_[i];
    return nullptr;
  }
  const NodeSonar* at(uint8_t i) const { return (i < MAXN && nodes_[i].used) ? &nodes_[i] : nullptr; }
  uint8_t capacity() const { return MAXN; }
  uint32_t lastSeq() const { return seq_; }

  // Pings of `node` with seq > since, oldest first. Returns the count written to out.
  uint16_t pingsSince(uint8_t node, uint32_t since, const StoredPing** out, uint16_t max) const {
    uint16_t n = 0;
    for (uint16_t k = 0; k < count_ && n < max; k++) {
      const uint16_t i = static_cast<uint16_t>((head_ + RING - count_ + k) % RING);
      const StoredPing& sp = ring_[i];
      if (sp.node == node && sp.seq > since) out[n++] = &sp;
    }
    return n;
  }

 private:
  // Not newer, but not so far back that it must be a node restart (counter back to 0).
  static bool isOld(uint16_t idx, uint16_t last) {
    const int16_t d = seqDiff(idx, last);
    return d <= 0 && d > -static_cast<int16_t>(RESTART_GAP);
  }
  enum { RESTART_GAP = 256 };

  NodeSonar* slot(uint8_t node, uint16_t frame) {
    for (uint8_t i = 0; i < MAXN; i++) if (nodes_[i].used && nodes_[i].node == node) return &nodes_[i];
    int oldest = -1;
    for (uint8_t i = 0; i < MAXN; i++) {
      if (!nodes_[i].used) { oldest = i; break; }
      if (oldest < 0 || seqDiff(nodes_[oldest].frame, nodes_[i].frame) > 0) oldest = i;
    }
    if (oldest < 0) return nullptr;
    NodeSonar& ns = nodes_[oldest];
    memset(&ns, 0, sizeof(ns));
    ns.used = true; ns.node = node; ns.frame = frame; ns.bg_ver = 0xFF;
    return &ns;
  }

  static void deriveSummary(NodeSonar& ns, const Ping& p, uint8_t bg_ver) {
    Summary& s = ns.sum;
    bool fish = false;
    for (uint8_t i = 0; i < p.n_targets; i++) if (p.t[i].track != 0) fish = true;
    ns.act_bits = static_cast<uint16_t>((ns.act_bits << 1) | (fish ? 1u : 0u));
    uint8_t act = 0;
    for (uint16_t b = ns.act_bits; b; b >>= 1) act = static_cast<uint8_t>(act + (b & 1u));
    memset(&s, 0, sizeof(s));
    s.node = ns.node; s.ping = p.index; s.bottom_cm = p.bottom_cm; s.bg_ver = bg_ver; s.activity = act > 15 ? 15 : act;
    s.nearest_cm = DEPTH_NONE;
    uint16_t bait = DEPTH_NONE;
    for (uint8_t i = 0; i < p.n_targets; i++) if (p.t[i].track == 0) bait = p.t[i].depth_cm;
    uint16_t best = 0xFFFF;
    for (uint8_t i = 0; i < p.n_targets; i++) {
      const Target& t = p.t[i];
      if (t.track == 0) continue;
      s.n_targets++;
      const uint16_t d = bait == DEPTH_NONE ? t.depth_cm : static_cast<uint16_t>(t.depth_cm > bait ? t.depth_cm - bait : bait - t.depth_cm);
      if (d < best) { best = d; s.nearest_cm = t.depth_cm; s.nearest_level = t.level; }
    }
    ns.has_sum = true;
  }

  NodeSonar nodes_[MAXN];
  StoredPing ring_[RING];
  uint16_t head_, count_;
  uint32_t seq_;
};

}  // namespace sonar
}  // namespace icemesh
#endif
