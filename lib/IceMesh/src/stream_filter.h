// Newest-wins filter for the STREAM class (docs/protocol_v2.md §3.2).
// One "last forwarded seq" per origin, no ID cache. d = seqDiff(seq, last):
//   unknown / idle origin -> FORWARD
//   d > 0                 -> FORWARD (newer)
//   -RESTART_GAP < d <= 0 -> DROP_OLD (duplicate or older copy)
//   d <= -RESTART_GAP     -> FORWARD_RESTART (origin rebooted)
// Safe because STREAM packets live at most 1 s in any queue and a stream sends far fewer
// than RESTART_GAP packets per second.
#ifndef ICEMESH_STREAM_FILTER_H
#define ICEMESH_STREAM_FILTER_H
#include <stdint.h>
#include "seq.h"

namespace icemesh {

enum StreamResult : uint8_t {
  STREAM_FORWARD = 0,
  STREAM_FORWARD_RESTART,
  STREAM_DROP_OLD,
};

inline bool isForward(StreamResult r) { return r != STREAM_DROP_OLD; }

template <uint8_t MAX_ORIGINS = 16>
class StreamFilter {
 public:
  enum { RESTART_GAP = 16 };
  static const uint32_t IDLE_EXPIRY_MS = 10000;  // a focus stream silent for 10 s is over

  StreamFilter() { clear(); }
  void clear() { for (uint8_t i = 0; i < MAX_ORIGINS; i++) e_[i].used = false; }

  StreamResult check(uint8_t origin, uint16_t seq, uint32_t now_ms) {
    Entry* e = find(origin, now_ms);
    if (e == nullptr) {
      e = allocate();
      e->used = true;
      e->origin = origin;
      e->last = seq;
      e->last_heard = now_ms;
      return STREAM_FORWARD;
    }
    const int16_t d = seqDiff(seq, e->last);
    if (d > 0) { e->last = seq; e->last_heard = now_ms; return STREAM_FORWARD; }
    if (d > -static_cast<int16_t>(RESTART_GAP)) return STREAM_DROP_OLD;
    e->last = seq;
    e->last_heard = now_ms;
    return STREAM_FORWARD_RESTART;
  }

 private:
  struct Entry { bool used; uint8_t origin; uint16_t last; uint32_t last_heard; };

  Entry* find(uint8_t origin, uint32_t now_ms) {
    for (uint8_t i = 0; i < MAX_ORIGINS; i++) {
      Entry& e = e_[i];
      if (e.used && e.origin == origin) {
        if ((now_ms - e.last_heard) >= IDLE_EXPIRY_MS) { e.used = false; return nullptr; }
        return &e;
      }
    }
    return nullptr;
  }
  Entry* allocate() {
    Entry* victim = &e_[0];
    for (uint8_t i = 0; i < MAX_ORIGINS; i++) {
      if (!e_[i].used) return &e_[i];
      if ((int32_t)(e_[i].last_heard - victim->last_heard) < 0) victim = &e_[i];
    }
    return victim;  // least recently heard stream is replaced
  }

  Entry e_[MAX_ORIGINS];
};

}  // namespace icemesh
#endif
