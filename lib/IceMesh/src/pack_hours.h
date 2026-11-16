// Pack hours (D205): hours this module has run on its battery pack since the pack was last marked new.
// Why a clock and not a voltage: the Energizer L91 (Li/FeS2) holds ~1.4-1.5 V per cell under load for most of its life
// and drops only in the last 10-15 %, and the cold lowers it too, so a voltage reading says little until the end.
// Time run x the module's measured current is the better gauge. Pure logic, no Arduino: the firmware feeds millis(),
// writes seconds() to flash when feed() says so, and calls reset() when a fresh pack goes in.
// Counts awake time only: a tip-up in deep sleep draws microamps, so its sleep is left out on purpose.
#pragma once
#include <stdint.h>
#include <stdio.h>

namespace icemesh {

struct PackClock {
  static const uint32_t SAVE_EVERY_S = 300;   // flash write every 5 min of run time: a power-off loses at most 5 min

  uint32_t sec;        // seconds on this pack
  uint32_t saved;      // value last written to flash
  uint32_t ms;         // sub-second remainder
  uint32_t last;       // millis() at the previous feed
  bool dirty;          // a reset not yet written

  PackClock() : sec(0), saved(0), ms(0), last(0), dirty(false) {}

  void load(uint32_t seconds, uint32_t now_ms) { sec = saved = seconds; ms = 0; last = now_ms; dirty = false; }
  void reset(uint32_t now_ms) { load(0, now_ms); dirty = true; }

  // true = write seconds() to flash now, then markSaved()
  bool feed(uint32_t now_ms) {
    const uint32_t d = now_ms - last;   // unsigned difference: survives the 49-day millis() wrap
    last = now_ms;
    ms += d;
    sec += ms / 1000; ms %= 1000;
    return dirty || sec - saved >= SAVE_EVERY_S;
  }
  void markSaved() { saved = sec; dirty = false; }
  uint32_t seconds() const { return sec; }

  // "Pk7.3h" style text: one decimal under 100 h, whole hours above. Returns the length written.
  static int format(uint32_t seconds, char* out, unsigned n) {
    if (!out || !n) return 0;
    const uint32_t tenths = seconds / 360;
    if (tenths < 1000) return snprintf(out, n, "%lu.%luh", (unsigned long)(tenths / 10), (unsigned long)(tenths % 10));
    return snprintf(out, n, "%luh", (unsigned long)(seconds / 3600));
  }
};

// D205b: the chalet page reads and resets it through the wiring-check path (no new message type): CMD_SELFTEST /
// DEVCMD_SELFTEST value PACK_CMD_READ / PACK_CMD_NEW (0 = wiring check, 1-2 = tune, 3 = firmware update mode), the
// answer is a 12-byte PackResult in the SelfTestResult slot (SEC_SELFTEST) with ver = PACK_RESULT_VER.
enum : uint8_t { PACK_RESULT_VER = 3 };
enum : uint8_t { PACK_CMD_READ = 4, PACK_CMD_NEW = 5 };
enum : uint8_t { PACK_FL_RENEWED = 0x01 };   // this answer follows a reset (the count is the new pack's)
struct PackResult {
  uint8_t ver;         // PACK_RESULT_VER
  uint8_t flags;       // PACK_FL_*
  uint8_t sec[4];      // seconds on this pack, little-endian (bytes: no alignment in a packed slot)
  uint8_t spare[6];

  static PackResult make(uint32_t seconds, bool renewed) {
    PackResult r = PackResult();
    r.ver = PACK_RESULT_VER; r.flags = renewed ? PACK_FL_RENEWED : 0;
    for (int i = 0; i < 4; i++) r.sec[i] = (uint8_t)(seconds >> (8 * i));
    return r;
  }
  static bool is(const uint8_t* r12) { return r12 != nullptr && r12[0] == PACK_RESULT_VER; }
  static uint32_t seconds(const uint8_t* r12) {
    uint32_t s = 0; for (int i = 3; i >= 0; i--) s = (s << 8) | r12[2 + i]; return s;
  }
};
static_assert(sizeof(PackResult) == 12, "PackResult travels in the 12-byte SelfTestResult slot");

}  // namespace icemesh
