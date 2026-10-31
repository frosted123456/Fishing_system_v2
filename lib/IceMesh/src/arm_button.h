// Arming button (D61): one momentary push button to GND on every offshore module (tip-up, hub hole).
//   short press (< 3 s)          armed <-> disarmed; while the hole is tripped (FISH ON): clears the trip instead
//   hold 3 s                     status report now (fires once, while the button is still held)
// Disarmed = spool movement is ignored (no FISH ON, no alert); heartbeats keep going. The chalet can set
// the same state remotely (CMD_SET_ARM -> DEVCMD_ARM): last action wins. Pure logic, no Arduino: the
// firmware feeds the pin level and a time in ms and acts on what comes back.
#pragma once
#include <stdint.h>

namespace icemesh {

enum ArmEvent : uint8_t { ARM_EV_NONE = 0, ARM_EV_SHORT = 1, ARM_EV_LONG = 2 };
enum ArmAction : uint8_t { ARM_NONE = 0, ARM_TOGGLE_ARM = 1, ARM_CLEAR_TRIP = 2, ARM_REPORT = 3 };

// Press detector: 30 ms debounce, SHORT on release before 3 s, LONG once when the hold reaches 3 s.
// Plain struct, zero = idle; a wake with the button already held: feed(true, now) first thing.
struct ArmButton {
  static const uint32_t BOUNCE_MS = 30;
  static const uint32_t HOLD_MS = 3000;
  bool init;
  bool raw;              // last level fed
  uint32_t raw_since;    // when `raw` first appeared
  bool down;             // debounced level
  uint32_t down_since;   // start of the current press (first raw sample, so the debounce does not delay the 3 s)
  bool long_fired;

  ArmEvent feed(bool pressed, uint32_t now_ms) {
    if (!init) {
      init = true; raw = down = pressed; raw_since = down_since = now_ms; long_fired = false;
    } else if (pressed != raw) {
      raw = pressed; raw_since = now_ms;
    }
    if (raw != down && now_ms - raw_since >= BOUNCE_MS) {   // level held long enough: a real edge
      down = raw;
      if (down) { down_since = raw_since; long_fired = false; }
      else if (!long_fired) return ARM_EV_SHORT;
    }
    if (down && !long_fired && now_ms - down_since >= HOLD_MS) { long_fired = true; return ARM_EV_LONG; }
    return ARM_EV_NONE;
  }
};

// Armed / disarmed state and what a press does. Plain struct, zero = ARMED (it lives in RTC memory on the
// tip-up like HallLatch: no constructor; the firmware loads the saved value on power-on).
struct ArmState {
  bool disarmed;

  bool armed() const { return !disarmed; }
  void set(bool armed_now) { disarmed = !armed_now; }   // remote command / saved setting: last action wins

  ArmAction apply(ArmEvent e, bool tripped) {
    if (e == ARM_EV_LONG) return ARM_REPORT;
    if (e != ARM_EV_SHORT) return ARM_NONE;
    if (tripped) return ARM_CLEAR_TRIP;                  // the press ends the trip, the state stays
    disarmed = !disarmed;
    return ARM_TOGGLE_ARM;
  }
};

}  // namespace icemesh
