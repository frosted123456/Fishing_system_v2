// Non-blocking priority TX queue (docs/protocol_v2.md §5). Fixed memory, no heap, no delay().
//
// Priorities: P0 alert/ACK > P1 control (config, silence, reset) > P2 aggregate/scene/track > P3 stream.
// - FIFO inside a priority; the scheduler takes the highest-priority item that is READY
//   (not_before reached), so a deferred item (CAD busy, relay jitter) does not block others.
// - Per-priority depth; when full the oldest item of that priority is dropped (counted).
// - Supersede: an item pushed with supersede=true replaces a queued item with the same
//   (priority, key) in place (newest aggregate / stream frame wins, queue position kept).
// - TTL per item (default per priority); expired items are dropped (counted).
// - tag: free 32-bit field for the caller, e.g. (origin << 16 | seq) to cancel a pending relay
//   when the same packet is overheard from another hub (phase 2).
#ifndef ICEMESH_TX_QUEUE_H
#define ICEMESH_TX_QUEUE_H
#include <stdint.h>
#include <string.h>
#include "seq.h"

namespace icemesh {

enum Priority : uint8_t {
  PRIO_ALERT = 0,
  PRIO_CONTROL = 1,
  PRIO_AGGREGATE = 2,
  PRIO_STREAM = 3,
  PRIO_COUNT = 4,
};

inline uint32_t defaultTtlMs(Priority p) {
  switch (p) {
    case PRIO_ALERT:     return 30000;
    case PRIO_CONTROL:   return 30000;
    case PRIO_AGGREGATE: return 10000;
    default:             return 1000;
  }
}

struct TxOptions {
  Priority prio;
  uint32_t not_before_ms;   // absolute millis(); use `now` to send as soon as possible
  uint32_t ttl_ms;          // 0 = default for the priority
  uint16_t key;             // supersede key, e.g. (type << 8) | origin
  bool supersede;
  uint32_t tag;

  static TxOptions make(Priority p, uint32_t now_ms) {
    TxOptions o;
    o.prio = p; o.not_before_ms = now_ms; o.ttl_ms = 0; o.key = 0; o.supersede = false; o.tag = 0;
    return o;
  }
};

enum PushResult : uint8_t {
  PUSH_OK = 0,
  PUSH_SUPERSEDED,
  PUSH_OK_DROPPED_OLDEST,
  PUSH_REJECTED,            // empty, too long or bad priority
};

template <uint16_t FRAME_MAX = 255>
struct TxItem {
  bool used;
  Priority prio;
  uint16_t len;
  uint8_t attempts;         // incremented by defer()
  bool supersede;
  uint16_t key;
  uint32_t order;           // FIFO order inside a priority
  uint32_t enqueued_ms;
  uint32_t not_before_ms;
  uint32_t ttl_ms;
  uint32_t tag;
  uint8_t data[FRAME_MAX];
};

struct TxStats {
  uint32_t pushed[PRIO_COUNT];
  uint32_t superseded[PRIO_COUNT];
  uint32_t dropped_overflow[PRIO_COUNT];
  uint32_t dropped_ttl[PRIO_COUNT];
  uint32_t cancelled[PRIO_COUNT];
};

template <uint16_t FRAME_MAX = 255, uint8_t D0 = 8, uint8_t D1 = 8, uint8_t D2 = 4, uint8_t D3 = 2>
class TxQueue {
 public:
  typedef TxItem<FRAME_MAX> Item;
  enum { CAPACITY = D0 + D1 + D2 + D3 };

  TxQueue() { clear(); }

  void clear() {
    for (uint8_t i = 0; i < CAPACITY; i++) items_[i].used = false;
    next_order_ = 0;
    memset(&stats_, 0, sizeof(stats_));
  }

  PushResult push(const uint8_t* data, uint16_t len, const TxOptions& o, uint32_t now_ms) {
    if (data == nullptr || len == 0 || len > FRAME_MAX || o.prio >= PRIO_COUNT) return PUSH_REJECTED;
    const uint32_t ttl = o.ttl_ms ? o.ttl_ms : defaultTtlMs(o.prio);

    if (o.supersede) {
      for (uint8_t i = 0; i < CAPACITY; i++) {
        Item& it = items_[i];
        if (it.used && it.prio == o.prio && it.supersede && it.key == o.key) {
          fill(it, data, len, o, ttl, now_ms, it.order);
          stats_.superseded[o.prio]++;
          return PUSH_SUPERSEDED;
        }
      }
    }

    PushResult result = PUSH_OK;
    if (count(o.prio) >= depth(o.prio)) {
      Item* oldest = oldestOf(o.prio);
      oldest->used = false;
      stats_.dropped_overflow[o.prio]++;
      result = PUSH_OK_DROPPED_OLDEST;
    }
    for (uint8_t i = 0; i < CAPACITY; i++) {
      if (!items_[i].used) {
        fill(items_[i], data, len, o, ttl, now_ms, next_order_++);
        stats_.pushed[o.prio]++;
        return result;
      }
    }
    return PUSH_REJECTED;  // unreachable: depths sum to CAPACITY
  }

  // Highest-priority ready item (FIFO inside the priority), or nullptr.
  // Drops TTL-expired items on the way. Priorities below `lowest` are not considered
  // (e.g. pass PRIO_AGGREGATE when the stream airtime budget is exhausted).
  Item* peekReady(uint32_t now_ms, Priority lowest = PRIO_STREAM) {
    expire(now_ms);
    Item* best = nullptr;
    for (uint8_t i = 0; i < CAPACITY; i++) {
      Item& it = items_[i];
      if (!it.used || it.prio > lowest || !timeReached(now_ms, it.not_before_ms)) continue;
      if (best == nullptr || it.prio < best->prio ||
          (it.prio == best->prio && static_cast<int32_t>(it.order - best->order) < 0)) {
        best = &it;
      }
    }
    return best;
  }

  // Earliest not_before among queued items (for sleep/scheduling); false if empty.
  bool nextReadyTime(uint32_t& out_ms) const {
    bool any = false;
    for (uint8_t i = 0; i < CAPACITY; i++) {
      const Item& it = items_[i];
      if (!it.used) continue;
      if (!any || static_cast<int32_t>(it.not_before_ms - out_ms) < 0) out_ms = it.not_before_ms;
      any = true;
    }
    return any;
  }

  // Channel busy / TX failed: retry later. The item keeps its place in the FIFO.
  void defer(Item* it, uint32_t until_ms) {
    if (it == nullptr) return;
    it->not_before_ms = until_ms;
    if (it->attempts < 255) it->attempts++;
  }

  void remove(Item* it) { if (it != nullptr) it->used = false; }

  uint8_t cancelByTag(uint32_t tag) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < CAPACITY; i++) {
      Item& it = items_[i];
      if (it.used && it.tag == tag) { it.used = false; stats_.cancelled[it.prio]++; n++; }
    }
    return n;
  }

  uint8_t expire(uint32_t now_ms) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < CAPACITY; i++) {
      Item& it = items_[i];
      if (it.used && (now_ms - it.enqueued_ms) >= it.ttl_ms) {
        it.used = false;
        stats_.dropped_ttl[it.prio]++;
        n++;
      }
    }
    return n;
  }

  uint8_t count(Priority p) const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < CAPACITY; i++) if (items_[i].used && items_[i].prio == p) n++;
    return n;
  }
  uint8_t total() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < CAPACITY; i++) if (items_[i].used) n++;
    return n;
  }
  static uint8_t depth(Priority p) {
    switch (p) { case PRIO_ALERT: return D0; case PRIO_CONTROL: return D1; case PRIO_AGGREGATE: return D2; default: return D3; }
  }
  const TxStats& stats() const { return stats_; }

 private:
  static void fill(Item& it, const uint8_t* data, uint16_t len, const TxOptions& o, uint32_t ttl,
                   uint32_t now_ms, uint32_t order) {
    it.used = true;
    it.prio = o.prio;
    it.len = len;
    memcpy(it.data, data, len);
    it.attempts = 0;
    it.supersede = o.supersede;
    it.key = o.key;
    it.order = order;
    it.enqueued_ms = now_ms;
    it.not_before_ms = o.not_before_ms;
    it.ttl_ms = ttl;
    it.tag = o.tag;
  }

  Item* oldestOf(Priority p) {
    Item* o = nullptr;
    for (uint8_t i = 0; i < CAPACITY; i++) {
      Item& it = items_[i];
      if (it.used && it.prio == p && (o == nullptr || static_cast<int32_t>(it.order - o->order) < 0)) o = &it;
    }
    return o;
  }

  Item items_[CAPACITY];
  uint32_t next_order_;
  TxStats stats_;
};

}  // namespace icemesh
#endif
