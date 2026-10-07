// Sonar processing knobs (D42): one set for every sonar hole, chosen at the chalet (OLED / web),
// sent in the beacon (CMD_SONAR_PARAM: target = knob index, value = 0-255), kept by each hub and passed
// to its tip-ups in the sonar control message. Defaults = the prototype's "Processed" preset, so the
// processing stays bit-exact with the prototype (test_sonar_proc) until a knob is moved.
// The values are guesses tuned on simulated pings only: re-tune them on the first real recordings.
#ifndef ICEMESH_SONAR_PARAMS_H
#define ICEMESH_SONAR_PARAMS_H
#include <stdint.h>

namespace icemesh {
namespace sonar {

enum ParamId : uint8_t {
  P_SNR_DB = 0,      // detection: echo must be this many dB above the noise floor
  P_PROM_DB,         // detection: peak must stand this many dB above its surroundings
  P_CONFIRM,         // pings a target must be seen before it is reported (filters single-ping noise)
  P_KEEP,            // pings a target is kept without echo (gaps in a fish trace)
  P_GATE_CM,         // max move of a target between pings (cm)
  P_DEADZONE_DM,     // no target closer than this below the transducer (0.1 m): ring-down / ice / bubbles
  P_BOTTOM_SMOOTH,   // bottom line smoothing, % of the new value per ping (lower = steadier)
  P_STATIC_LEARN,    // how fast the static scene (bottom, weeds) is learned, 0.001 per ping
  P_RANGE_DB,        // colour dynamic range (dB): smaller = more contrast, weak echoes drop out
  P_TVG,             // range compensation (time-varied gain) 1 on / 0 off
  // acquisition knobs for the TUSS4470 driver (NOT WRITTEN YET): kept and sent, not used by anything today
  P_PULSE_CYCLES,    // cycles per ping (longer = more energy, less resolution)
  P_GAIN,            // receiver gain step (TUSS4470 VGA, meaning set by the driver)
  P_PING_HZ_X4,      // pings per second x4 (4 = 1/s ... 16 = 4/s)
  P_COUNT
};

struct ParamInfo { const char* key; const char* name; uint8_t lo, hi, def; const char* unit; };

inline const ParamInfo& paramInfo(uint8_t id) {
  static const ParamInfo T[P_COUNT] = {
    {"snr", "Detection threshold", 4, 30, 10, "dB"},
    {"prom", "Peak contrast", 1, 15, 5, "dB"},
    {"confirm", "Confirm pings", 1, 10, 3, ""},
    {"keep", "Keep without echo", 1, 20, 5, "pings"},
    {"gate", "Max target move", 5, 100, 32, "cm/ping"},
    {"dead", "Dead zone", 0, 50, 9, "x0.1 m"},
    {"bsmooth", "Bottom smoothing", 5, 100, 25, "%"},
    {"learn", "Static scene learning", 1, 100, 12, "x0.001"},
    {"range", "Colour range", 20, 80, 46, "dB"},
    {"tvg", "Range compensation", 0, 1, 1, ""},
    {"cycles", "Pulse cycles (driver)", 1, 32, 8, ""},
    {"gain", "Receiver gain (driver)", 0, 15, 8, ""},
    {"pinghz", "Ping rate (driver)", 1, 16, 16, "x0.25/s"},
  };
  return T[id < P_COUNT ? id : 0];
}

struct Params {
  uint8_t v[P_COUNT];
  Params() { defaults(); }
  void defaults() { for (uint8_t i = 0; i < P_COUNT; i++) v[i] = paramInfo(i).def; }
  bool set(uint8_t id, uint8_t val) {   // clamped; false if unknown or unchanged
    if (id >= P_COUNT) return false;
    const ParamInfo& p = paramInfo(id);
    const uint8_t c = val < p.lo ? p.lo : (val > p.hi ? p.hi : val);
    if (v[id] == c) return false;
    v[id] = c; return true;
  }
  uint8_t operator[](uint8_t id) const { return id < P_COUNT ? v[id] : 0; }
};

// "Noise filter" presets for the OLED (one knob for the main detection values)
enum FilterPreset : uint8_t { FILT_LOW = 0, FILT_NORMAL = 1, FILT_HIGH = 2, FILT_CUSTOM = 3 };
inline void applyPreset(Params& p, uint8_t preset) {
  static const uint8_t S[3][4] = {   // snr, prom, confirm, keep
    {7, 4, 2, 6}, {10, 5, 3, 5}, {14, 7, 5, 4}};
  if (preset > FILT_HIGH) return;
  p.set(P_SNR_DB, S[preset][0]); p.set(P_PROM_DB, S[preset][1]); p.set(P_CONFIRM, S[preset][2]); p.set(P_KEEP, S[preset][3]);
}
inline uint8_t presetOf(const Params& p) {   // looks at the 4 detection knobs only
  for (uint8_t k = 0; k < 3; k++) {
    Params q; applyPreset(q, k);
    if (q[P_SNR_DB] == p[P_SNR_DB] && q[P_PROM_DB] == p[P_PROM_DB] && q[P_CONFIRM] == p[P_CONFIRM] && q[P_KEEP] == p[P_KEEP]) return k;
  }
  return FILT_CUSTOM;
}

}  // namespace sonar
}  // namespace icemesh
#endif
