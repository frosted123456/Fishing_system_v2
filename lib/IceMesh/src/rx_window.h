// ESP-NOW receive windows (D208): an awake sonar tip-up keeps its Wi-Fi receiver on for WINDOW_MS of every INTERVAL_MS
// instead of all the time (the receiver is ~60 % of its battery draw). ESP-IDF "connectionless" power save: the tip-up
// can TRANSMIT at any moment, it only HEARS inside its windows, and nothing tells the hub when those are.
// So: (1) the hub sends every frame meant for a tip-up COPIES times, SPACING_MS apart: whatever the phase, one copy
// starts inside a window (covered(), host-tested with send jitter); (2) the tip-up opens its receiver fully for a
// moment after its own status / alert (the hub answers those at once) and while it pings; (3) Guard: if the hub has
// not been heard for HEARD_TIMEOUT_MS while the sonar runs, the receiver goes back to always-on (= the behaviour
// before D208), and after MAX_TRIPS such events it stays there until the next boot. Pure logic, no Arduino.
#pragma once
#include <stdint.h>

namespace icemesh {
namespace rxwin {

static const uint16_t INTERVAL_MS = 100;
static const uint16_t WINDOW_MS = 35;
static const uint8_t COPIES = 5;          // 0, 20, 40, 60, 80 ms: one full interval
static const uint16_t SPACING_MS = 20;
static const uint16_t GUARD_MS = 6;       // radio power-up at the window start + frame airtime (est.)

// true when at least one copy starts inside the usable part of a window. phase_ms = where in the receiver's interval
// the first copy goes out (0..INTERVAL_MS-1); extra_ms[k] = how late copy k left (loop jitter), may be nullptr.
inline bool covered(uint16_t phase_ms, const uint16_t* extra_ms, uint8_t copies = COPIES, uint16_t spacing_ms = SPACING_MS,
                    uint16_t window_ms = WINDOW_MS, uint16_t guard_ms = GUARD_MS) {
  for (uint8_t k = 0; k < copies; k++) {
    const uint32_t t = (uint32_t)phase_ms + (uint32_t)k * spacing_ms + (extra_ms ? extra_ms[k] : 0);
    const uint16_t in = (uint16_t)(t % INTERVAL_MS);
    if (in >= guard_ms && in + 1 < window_ms) return true;   // + 1 ms of airtime before the window closes
  }
  return false;
}

// Tip-up side: when must the receiver be fully on although windows are allowed?
struct Guard {
  static const uint32_t HEARD_TIMEOUT_MS = 25000;   // the hub repeats its control every 10 s while a hole streams
  static const uint32_t RETRY_AFTER_MS = 60000;     // back to windows this long after the hub was heard again
  static const uint8_t MAX_TRIPS = 3;

  bool open;            // receiver forced always-on by the guard
  uint8_t trips;
  uint32_t last_heard, opened_at, started_at;
  bool heard_since_open, running;

  Guard() : open(false), trips(0), last_heard(0), opened_at(0), started_at(0), heard_since_open(false), running(false) {}

  void heard(uint32_t now_ms) { last_heard = now_ms; if (open) heard_since_open = true; }
  bool latched() const { return trips >= MAX_TRIPS; }

  // call every loop pass. sonar_running = this tip-up is awake and streaming (the only state windows are used in).
  // Returns true = keep the receiver always on.
  bool update(uint32_t now_ms, bool sonar_running) {
    if (!sonar_running) { running = false; return true; }                 // every other state listens all the time
    if (!running) { running = true; started_at = now_ms; }                // the silence clock starts with the sonar
    if (latched()) return true;
    const uint32_t ref = (uint32_t)(last_heard - started_at) < 0x80000000UL ? last_heard : started_at;   // the later of the two
    if (!open) {
      if ((uint32_t)(now_ms - ref) > HEARD_TIMEOUT_MS) { open = true; trips++; opened_at = now_ms; heard_since_open = false; }
    } else if (heard_since_open && (uint32_t)(now_ms - opened_at) > RETRY_AFTER_MS) {
      open = false;
    }
    return open;
  }
};

}  // namespace rxwin
}  // namespace icemesh
