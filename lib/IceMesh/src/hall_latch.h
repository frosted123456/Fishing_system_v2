// Spool-shaft Hall latch (US1881 bipolar latch + 4 magnets, poles alternating): the output flips every
// quarter turn of the shaft. There is no "flag up" level: a trip is N flips close together, and the trip
// ends when the shaft has been still for a while (line stopped running). Pure logic, no Arduino: the
// firmware feeds it flips (ISR edge count while awake, level compare across deep sleep) and a time in ms
// that keeps running through deep sleep. The chalet keeps its own alarm latched until silenced (D41), so
// the end of a trip here only means "line not moving any more".
#pragma once
#include <stdint.h>
#include <type_traits>

namespace icemesh {

// Plain struct on purpose (no constructor, no member initializers): it lives in RTC memory, and a C++
// constructor would run again on every deep-sleep wake and wipe it. Zero = power-on state; call
// settings() on every boot.
struct HallLatch {
  // settings
  uint8_t trigger_flips;            // 1 = quarter turn, 2 = half turn (wind / bait false alerts)
  uint32_t window_ms;               // flips further apart than this start a new count
  uint32_t clear_ms;                // trip ends after the shaft is still this long
  // state
  bool init;
  uint8_t level;                    // last known output level
  uint8_t run;                      // flips in the current count
  bool on;                          // trip active (FISH ON)
  uint32_t total;                   // flips since power-on (quarter turns, either way)
  uint64_t last_ms;                 // time of the last flip

  void settings(uint8_t trigger, uint32_t window, uint32_t clear) {
    trigger_flips = trigger ? trigger : 1; window_ms = window; clear_ms = clear;
  }
  // n flips seen at time t
  void flip(uint32_t n, uint64_t t) {
    if (!n) return;
    if (t - last_ms > window_ms) run = 0;
    run = (uint8_t)(run + n > 255 ? 255 : run + n);
    total += n;
    last_ms = t;
    if (run >= trigger_flips) on = true;
  }
  // after a boot: level now; woke_by_pin = the deep-sleep wake was the pin. A pin wake with the SAME level
  // means it flipped and flipped back before we could read it: at least 2 flips.
  void boot(uint8_t level_now, bool woke_by_pin, uint64_t t) {
    if (!init) { init = true; level = level_now; last_ms = t; return; }
    const uint32_t n = level_now != level ? 1u : (woke_by_pin ? 2u : 0u);
    level = level_now;
    flip(n, t);
  }
  // awake: edges counted by the pin interrupt since the last call, and the level now
  void edges(uint32_t n, uint8_t level_now, uint64_t t) {
    if (!n && level_now != level) n = 1;   // a flip the interrupt missed
    level = level_now;
    flip(n, t);
  }
  // true while a trip is active; ends it after clear_ms of stillness
  bool active(uint64_t t) {
    if (on && t - last_ms >= clear_ms) { on = false; run = 0; }
    return on;
  }
  // level to wake on in deep sleep: the opposite of now
  uint8_t wakeLevel() const { return level ? 0 : 1; }
};

static_assert(std::is_trivial<HallLatch>::value, "HallLatch lives in RTC memory: no constructor / member initializers");

}  // namespace icemesh
