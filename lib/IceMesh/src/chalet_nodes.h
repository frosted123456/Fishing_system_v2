// Chalet side: one line state per node, merged from every hub that reports it.
// A node heard by two pockets is reported by both hubs; only its OWNER hub's records change
// the displayed state. Ownership moves when the owner stops reporting the node for
// OWNER_TIMEOUT frames, or reports it OFFLINE while another hub still has it.
#ifndef ICEMESH_CHALET_NODES_H
#define ICEMESH_CHALET_NODES_H
#include <stdint.h>
#include "seq.h"
#include "tdma_proto.h"

namespace icemesh {
namespace tdma {

struct NodeView {
  bool used;
  uint8_t node, owner, state, turns, flags, battery;   // battery 0-100, 255 = unknown
  uint16_t owner_frame;                                // last frame the owner reported it
};

struct NodeChange { uint8_t node, owner, old_state, new_state, turns, flags; };
static const uint8_t STATE_UNKNOWN = 0xFF;

template <uint8_t MAX_NODES = 32>
class ChaletNodes {
 public:
  enum { OWNER_TIMEOUT_FRAMES = 10, OFFLINE_TIMEOUT_FRAMES = 90 };   // 1 frame ≈ 1 s (est. values)

  ChaletNodes() { clear(); }
  void clear() { for (uint8_t i = 0; i < MAX_NODES; i++) v_[i].used = false; }

  // Returns true when the displayed state changed (`ch` filled).
  bool onRecord(uint8_t hub, const LineRecord& r, uint16_t frame, NodeChange& ch) {
    if (r.node == ID_NONE) return false;
    NodeView* v = findMut(r.node);
    if (v == nullptr) {
      v = alloc(frame);
      if (v == nullptr) return false;
      v->used = true; v->node = r.node; v->owner = hub; v->state = STATE_UNKNOWN; v->battery = 255;
      v->owner_frame = frame;
    }
    const bool owner_stale = seqDiff(frame, v->owner_frame) > static_cast<int16_t>(OWNER_TIMEOUT_FRAMES);
    const bool take = (v->owner == hub) || owner_stale || (v->state == LS_OFFLINE && r.state != LS_OFFLINE);
    if (!take) return false;
    v->owner = hub;
    v->owner_frame = frame;
    v->turns = r.turns;
    v->flags = r.flags;
    if (r.state == v->state) return false;
    ch.node = v->node; ch.owner = hub; ch.old_state = v->state; ch.new_state = r.state;
    ch.turns = r.turns; ch.flags = r.flags;
    v->state = r.state;
    return true;
  }

  void onNodeInfo(uint8_t hub, uint8_t node, uint8_t battery_pct, uint8_t flags) {
    NodeView* v = findMut(node);
    if (v != nullptr && v->owner == hub) { v->battery = battery_pct; v->flags = flags; }
  }

  // Nodes whose owner stopped reporting them go OFFLINE. Returns the number of changes written.
  uint8_t expire(uint16_t frame, NodeChange* out, uint8_t max) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_NODES && n < max; i++) {
      NodeView& v = v_[i];
      if (!v.used || v.state == LS_OFFLINE) continue;
      if (seqDiff(frame, v.owner_frame) > static_cast<int16_t>(OFFLINE_TIMEOUT_FRAMES)) {
        out[n].node = v.node; out[n].owner = v.owner; out[n].old_state = v.state; out[n].new_state = LS_OFFLINE;
        out[n].turns = v.turns; out[n].flags = v.flags;
        v.state = LS_OFFLINE;
        n++;
      }
    }
    return n;
  }

  const NodeView* find(uint8_t node) const {
    for (uint8_t i = 0; i < MAX_NODES; i++) if (v_[i].used && v_[i].node == node) return &v_[i];
    return nullptr;
  }
  const NodeView* at(uint8_t i) const { return (i < MAX_NODES && v_[i].used) ? &v_[i] : nullptr; }
  static uint8_t capacity() { return MAX_NODES; }
  // Drops a node from the table (demo network switched off). Returns false if it was not there.
  bool forget(uint8_t node) {
    NodeView* v = findMut(node);
    if (v == nullptr) return false;
    v->used = false;
    return true;
  }

 private:
  NodeView* findMut(uint8_t node) {
    for (uint8_t i = 0; i < MAX_NODES; i++) if (v_[i].used && v_[i].node == node) return &v_[i];
    return nullptr;
  }
  NodeView* alloc(uint16_t frame) {
    NodeView* victim = nullptr;
    for (uint8_t i = 0; i < MAX_NODES; i++) {
      if (!v_[i].used) return &v_[i];
      // only an OFFLINE node may be replaced, the one silent the longest
      if (v_[i].state == LS_OFFLINE &&
          (victim == nullptr || seqDiff(frame, v_[i].owner_frame) > seqDiff(frame, victim->owner_frame)))
        victim = &v_[i];
    }
    return victim;
  }
  NodeView v_[MAX_NODES];
};

}  // namespace tdma
}  // namespace icemesh
#endif
