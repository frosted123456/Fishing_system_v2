// Per-origin duplicate filter for the RELIABLE class (docs/protocol_v2.md §3.1).
//
// Each origin keeps `highest` (newest seq seen) and a bitmap: bit i set = seq (highest - i) seen.
// d = seqDiff(seq, highest):
//   first frame / idle origin -> ACCEPT_NEW (window initialised)
//   d > 0                     -> ACCEPT_NEW (window slides)
//   d == 0                    -> DUPLICATE
//   -WINDOW < d < 0           -> ACCEPT_LATE once (reordered copy), then DUPLICATE
//   d <= -WINDOW              -> ACCEPT_RESTART (origin rebooted and restarted its counter)
// The restart rule is safe because queues drop RELIABLE packets after 30 s and no origin
// sends more than WINDOW reliable packets in 30 s (see protocol_v2.md, open question Q6).
//
// Fixed memory, no heap. Time is passed in (millis()), so the class is host-testable.
#ifndef ICEMESH_DEDUP_WINDOW_H
#define ICEMESH_DEDUP_WINDOW_H
#include <stdint.h>
#include "seq.h"

namespace icemesh {

enum DedupResult : uint8_t {
  ACCEPT_NEW = 0,
  ACCEPT_LATE,
  ACCEPT_RESTART,
  DUPLICATE,
};

inline bool isAccepted(DedupResult r) { return r != DUPLICATE; }

// MAX_ORIGINS: table size (LRU eviction when full). Bits: uint32_t (window 32) or uint64_t (64).
template <uint8_t MAX_ORIGINS = 32, typename Bits = uint32_t>
class DedupWindow {
 public:
  enum { WINDOW = sizeof(Bits) * 8 };
  static const uint32_t IDLE_EXPIRY_MS = 10UL * 60UL * 1000UL;  // forget silent origins

  DedupWindow() { clear(); }

  void clear() {
    for (uint8_t i = 0; i < MAX_ORIGINS; i++) entries_[i].used = false;
    evictions_ = 0;
  }

  // Classifies (origin, seq) and records it when accepted.
  DedupResult check(uint8_t origin, uint16_t seq, uint32_t now_ms) {
    Entry* e = find(origin, now_ms);
    if (e == nullptr) {
      e = allocate(origin);
      init(*e, seq, now_ms);
      return ACCEPT_NEW;
    }
    e->last_heard = now_ms;
    const int16_t d = seqDiff(seq, e->highest);
    if (d > 0) {
      e->bits = (d >= static_cast<int16_t>(WINDOW)) ? Bits(1) : static_cast<Bits>((e->bits << d) | Bits(1));
      e->highest = seq;
      return ACCEPT_NEW;
    }
    if (d == 0) return DUPLICATE;
    if (d > -static_cast<int16_t>(WINDOW)) {
      const Bits mask = static_cast<Bits>(Bits(1) << (-d));
      if (e->bits & mask) return DUPLICATE;
      e->bits = static_cast<Bits>(e->bits | mask);
      return ACCEPT_LATE;
    }
    init(*e, seq, now_ms);
    return ACCEPT_RESTART;
  }

  // Read-only query (does not record).
  bool seen(uint8_t origin, uint16_t seq, uint32_t now_ms) const {
    const Entry* e = findConst(origin, now_ms);
    if (e == nullptr) return false;
    const int16_t d = seqDiff(seq, e->highest);
    if (d > 0 || d <= -static_cast<int16_t>(WINDOW)) return false;
    return (e->bits & static_cast<Bits>(Bits(1) << (-d))) != 0;
  }

  uint8_t size(uint32_t now_ms) const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < MAX_ORIGINS; i++)
      if (entries_[i].used && !idle(entries_[i], now_ms)) n++;
    return n;
  }
  uint32_t evictions() const { return evictions_; }

 private:
  struct Entry {
    bool used;
    uint8_t origin;
    uint16_t highest;
    Bits bits;
    uint32_t last_heard;
  };

  static bool idle(const Entry& e, uint32_t now_ms) { return (now_ms - e.last_heard) >= IDLE_EXPIRY_MS; }

  static void init(Entry& e, uint16_t seq, uint32_t now_ms) {
    e.highest = seq;
    e.bits = Bits(1);
    e.last_heard = now_ms;
  }

  Entry* find(uint8_t origin, uint32_t now_ms) {
    for (uint8_t i = 0; i < MAX_ORIGINS; i++) {
      Entry& e = entries_[i];
      if (e.used && e.origin == origin) {
        if (idle(e, now_ms)) { e.used = false; return nullptr; }
        return &e;
      }
    }
    return nullptr;
  }

  const Entry* findConst(uint8_t origin, uint32_t now_ms) const {
    for (uint8_t i = 0; i < MAX_ORIGINS; i++) {
      const Entry& e = entries_[i];
      if (e.used && e.origin == origin) return idle(e, now_ms) ? nullptr : &e;
    }
    return nullptr;
  }

  Entry* allocate(uint8_t origin) {
    Entry* victim = nullptr;
    for (uint8_t i = 0; i < MAX_ORIGINS; i++) {
      Entry& e = entries_[i];
      if (!e.used) { victim = &e; break; }
      if (victim == nullptr || (int32_t)(e.last_heard - victim->last_heard) < 0) victim = &e;
    }
    if (victim->used) evictions_++;
    victim->used = true;
    victim->origin = origin;
    return victim;
  }

  Entry entries_[MAX_ORIGINS];
  uint32_t evictions_;
};

}  // namespace icemesh
#endif
