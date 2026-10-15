// FISH ON alarm latch of the chalet (D41 / D47): the rules only, no Arduino, so they are host-tested.
// Per hole: the line state (flag up = tripped) comes from the radio; the ALARM is what the chalet shows,
// sounds and sends to the phone. It starts with a trip and stays after the line resets, until someone
// silences it (or, with a hold time, that long after the line reset). Silencing while the hole is still
// tripped acknowledges it: shown until the line resets, then gone, no comeback. A new trip re-arms.
#pragma once
#include <stdint.h>

namespace icemesh {

struct AlarmLatch {
  uint32_t since_ms;   // 0 = no alarm; else when it started (never 0 while set)
  bool acked;          // silenced while the line was still up

  void start(uint32_t now) { since_ms = now ? now : 1; acked = false; }
  // silence pressed: a hole whose line is back down is cleared, a hole still up stays shown, acknowledged
  void ack(bool line_up) { if (line_up) acked = true; else since_ms = 0; }
  // the line went back down: an acknowledged alarm ends here
  void lineReset() { if (acked) { since_ms = 0; acked = false; } }
  // hold time (ms, 0 = until silenced): an alarm ends hold_ms after the line reset, by itself
  bool hold(bool line_up, uint32_t now, uint32_t hold_ms) {
    if (!hold_ms || !since_ms || line_up || now - since_ms <= hold_ms) return false;
    since_ms = 0; acked = false; return true;
  }
  bool active(bool line_up) const { return line_up || since_ms != 0; }
};

}  // namespace icemesh
