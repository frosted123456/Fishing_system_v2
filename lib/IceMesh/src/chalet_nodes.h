// Chalet side: one line state per node, merged from every hub that reports it.
// A node heard by two pockets is reported by both hubs; only its OWNER hub's records change
// the displayed state. Ownership moves when the owner stops reporting the node for
// OWNER_TIMEOUT frames, or reports it OFFLINE while another hub still has it.
// D79 (E2): an OFFLINE record from a non-owner never takes the node over (a hub that lost the node keeps saying
// OFFLINE for a long time while the other hub hears it fine), and when the owner itself says OFFLINE while another hub
// reported the node alive within OWNER_TIMEOUT frames, that hub becomes the owner with its record instead of the node
// flashing OFFLINE -> TRIPPED (which re-rang the alarm).
#ifndef ICEMESH_CHALET_NODES_H
#define ICEMESH_CHALET_NODES_H
#include <stdint.h>
#include "seq.h"
#include "tdma_proto.h"

namespace icemesh {
namespace tdma {

struct NodeView {
  bool used;
  uint8_t node, owner, state; uint16_t turns; uint8_t flags, battery;   // battery 0-100, 255 = unknown
  uint16_t owner_frame;                                // last frame the owner reported it
  uint8_t alt_hub, alt_state, alt_flags; uint16_t alt_turns, alt_frame;   // D79: latest non-OFFLINE record from another hub
};

struct NodeChange { uint8_t node, owner, old_state, new_state; uint16_t turns; uint8_t flags; };
static const uint8_t STATE_UNKNOWN = 0xFF;

template <uint8_t MAX_NODES = 32>
class ChaletNodes {
 public:
  enum { OWNER_TIMEOUT_FRAMES = 10, OFFLINE_TIMEOUT_FRAMES = 90 };   // seconds (their value in frames at a 1 s frame)
  // D81 (5.1): the frame is 0.5-2 s; timeouts are seconds turned into frames at the length in use
  void setFrameMs(uint16_t ms) { frame_ms_ = ms ? ms : 1000; }
  uint16_t ownerTimeout() const { const uint32_t n = OWNER_TIMEOUT_FRAMES * 1000UL / frame_ms_; return n < 1 ? 1 : static_cast<uint16_t>(n); }
  uint16_t offlineTimeout() const { const uint32_t n = OFFLINE_TIMEOUT_FRAMES * 1000UL / frame_ms_; return n < 1 ? 1 : static_cast<uint16_t>(n); }

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
      v->owner_frame = frame; v->alt_hub = ID_NONE; v->alt_frame = frame;
    }
    if (v->owner != hub) {
      if (r.state != LS_OFFLINE) { v->alt_hub = hub; v->alt_state = r.state; v->alt_turns = r.turns; v->alt_flags = r.flags; v->alt_frame = frame; }
      const bool owner_stale = seqDiff(frame, v->owner_frame) > static_cast<int16_t>(ownerTimeout());
      const bool take = r.state != LS_OFFLINE && (owner_stale || v->state == LS_OFFLINE);   // D79: OFFLINE never takes over
      if (!take) return false;
      return apply(v, hub, r.state, r.turns, r.flags, frame, ch);
    }
    // the owner says OFFLINE but another hub heard the node alive just now: that hub takes over, no OFFLINE blip
    if (r.state == LS_OFFLINE && v->alt_hub != ID_NONE && v->alt_hub != hub && seqDiff(frame, v->alt_frame) <= static_cast<int16_t>(ownerTimeout())) {
      // replay the alt record unless it would ring a trip the owner already walked through (TRIPPED -> IDLE): then the
      // alt hub just becomes the owner with the current displayed state and reports from now on
      const bool trip = v->alt_state == LS_TRIPPED || v->alt_state == LS_RUNNING || v->alt_state == LS_TRIP_SEEN;
      return apply(v, v->alt_hub, trip && v->state != v->alt_state ? v->state : v->alt_state, v->alt_turns, v->alt_flags, v->alt_frame, ch);
    }
    return apply(v, hub, r.state, r.turns, r.flags, frame, ch);
  }
  bool apply(NodeView* v, uint8_t hub, uint8_t state, uint16_t turns, uint8_t flags, uint16_t frame, NodeChange& ch) {
    v->owner = hub; v->owner_frame = frame; v->turns = turns; v->flags = flags;
    if (state == v->state) return false;
    ch.node = v->node; ch.owner = hub; ch.old_state = v->state; ch.new_state = state; ch.turns = turns; ch.flags = flags;
    v->state = state;
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
      if (seqDiff(frame, v.owner_frame) > static_cast<int16_t>(offlineTimeout())) {
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
  uint16_t frame_ms_ = 1000;
};

}  // namespace tdma
}  // namespace icemesh
#endif
