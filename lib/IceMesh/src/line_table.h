// Hub side: line state of the nodes this hub hears, with event latching.
// A state change creates a new event (seq + 1, pending). The record is sent in every hub packet
// and stays "pending" until the beacon acks exactly (this hub, node, seq) — no ack bitmap, so a
// newer event can never be cleared by the ack of an older one.
#ifndef ICEMESH_LINE_TABLE_H
#define ICEMESH_LINE_TABLE_H
#include <stdint.h>
#include "tdma_proto.h"

namespace icemesh {
namespace tdma {

template <uint8_t MAX_NODES = 24>
class LineTable {
 public:
  LineTable() { clear(); }
  void clear() { for (uint8_t i = 0; i < MAX_NODES; i++) e_[i].used = false; }

  // Latest observation of a node. Returns true when it created a new event.
  bool observe(uint8_t node, uint8_t state, uint8_t turns, uint8_t flags, uint8_t battery_pct, uint32_t now_ms) {
    if (node == ID_NONE) return false;
    E* e = find(node);
    bool created = false;
    if (e == nullptr) {
      e = alloc(now_ms);
      if (e == nullptr) return false;
      e->used = true; e->node = node; e->seq = 0; e->state = state;
      e->pending = (state != LS_IDLE);   // a node that appears already tripped is an event
      if (e->pending) e->seq = 1;
      created = e->pending;
    } else if (state != e->state) {
      e->state = state;
      e->seq = static_cast<uint8_t>((e->seq + 1) & 0x0F);
      e->pending = true;
      created = true;
    }
    e->turns = turns; e->flags = flags; e->battery = battery_pct;
    e->last_seen = now_ms;
    return created;
  }

  // Beacon acknowledged (node, seq) for this hub.
  void ack(uint8_t node, uint8_t seq) {
    E* e = find(node);
    if (e != nullptr && e->pending && e->seq == (seq & 0x0F)) e->pending = false;
  }

  // Up to `max` records: all pending ones first, then the others starting at the `rot`-th
  // non-pending entry (cyclic). Rotating `rot` makes every node appear even when the slot
  // allowance only fits part of the table. Returns the count; `pending_out` = pending ones.
  uint8_t records(LineRecord* out, uint8_t max, uint8_t rot = 0, uint8_t* pending_out = nullptr) const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_NODES && n < max; i++)
      if (e_[i].used && e_[i].pending) put(out[n++], e_[i]);
    if (pending_out != nullptr) *pending_out = n;
    const uint8_t np = nonPendingCount();
    for (uint8_t k = 0; k < np && n < max; k++) {
      const E* e = nthNonPending(static_cast<uint8_t>((rot + k) % np));
      if (e != nullptr) put(out[n++], *e);
    }
    return n;
  }

  uint8_t nonPendingCount() const {
    uint8_t c = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++) if (e_[i].used && !e_[i].pending) c++;
    return c;
  }

  bool anyPending() const {
    for (uint8_t i = 0; i < MAX_NODES; i++) if (e_[i].used && e_[i].pending) return true;
    return false;
  }

  // Forget nodes silent for `drop_after_ms` whose last event was acked.
  uint8_t expire(uint32_t now_ms, uint32_t drop_after_ms) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++) {
      E& e = e_[i];
      if (e.used && !e.pending && (now_ms - e.last_seen) >= drop_after_ms) { e.used = false; n++; }
    }
    return n;
  }

  uint8_t count() const { uint8_t n = 0; for (uint8_t i = 0; i < MAX_NODES; i++) if (e_[i].used) n++; return n; }

  bool batteryOf(uint8_t node, uint8_t& pct, uint8_t& flags) const {
    for (uint8_t i = 0; i < MAX_NODES; i++)
      if (e_[i].used && e_[i].node == node) { pct = e_[i].battery; flags = e_[i].flags; return true; }
    return false;
  }
  // i-th used node (for rotating NODEINFO); returns false past the end.
  bool nodeAt(uint8_t idx, uint8_t& node) const {
    uint8_t k = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++) if (e_[i].used) { if (k == idx) { node = e_[i].node; return true; } k++; }
    return false;
  }

 private:
  struct E { bool used; bool pending; uint8_t node, state, seq, turns, flags, battery; uint32_t last_seen; };
  static void put(LineRecord& r, const E& e) {
    r.node = e.node; r.state = e.state; r.seq = e.seq; r.pending = e.pending; r.turns = e.turns; r.flags = e.flags;
  }
  const E* nthNonPending(uint8_t k) const {
    for (uint8_t i = 0; i < MAX_NODES; i++)
      if (e_[i].used && !e_[i].pending) { if (k == 0) return &e_[i]; k--; }
    return nullptr;
  }
  E* find(uint8_t node) { for (uint8_t i = 0; i < MAX_NODES; i++) if (e_[i].used && e_[i].node == node) return &e_[i]; return nullptr; }
  E* alloc(uint32_t now_ms) {
    E* victim = nullptr;
    for (uint8_t i = 0; i < MAX_NODES; i++) {
      if (!e_[i].used) return &e_[i];
      if (!e_[i].pending && (victim == nullptr || (now_ms - e_[i].last_seen) > (now_ms - victim->last_seen))) victim = &e_[i];
    }
    return victim;   // nullptr if every entry has a pending event (never drop an unacked event)
  }
  E e_[MAX_NODES];
};

}  // namespace tdma
}  // namespace icemesh
#endif
