// Sonar processing knobs (D42): one set for every sonar hole, chosen at the chalet (OLED / web),
// sent in the beacon (CMD_SONAR_PARAM: target = knob index, value = 0-255), kept by each hub and passed
// to its tip-ups in the sonar control message. Defaults = the prototype's "Processed" preset, plus the D57
// interference knobs (jitter / spike / echolen, ON by default): test_sonar_proc turns those 3 off and stays
// bit-exact with the prototype.
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
  // acquisition (src/sensor_node/sonar_tuss4470.*, D43): written blind, to verify in the bucket
  P_PULSE_CYCLES,    // burst cycles in base mode (longer = more energy / range)
  P_GAIN,            // receiver gain 0-3 = 10 / 12.5 / 15 / 20 V/V (the driver maps to the TUSS4470 codes)
  P_PING_HZ_X4,      // ping rate while something moves (x0.25/s: 16 = 4/s)
  P_CYCLES_FOCUS,    // burst cycles in focus mode (shorter = 2-3 cm resolution)
  P_IDLE_HZ_X4,      // ping rate when nothing moves (x0.25/s: 4 = 1/s)
  P_AVG,             // bursts averaged per ping in base mode (1-4, +3-6 dB)
  P_FREQ_MODE,       // 0 = 3 bursts per ping (190/200/210), 1 = one frequency per ping, rotating, 2 = 200 kHz only
  P_SOUND,           // sound speed - 1350 m/s (53 = 1403 m/s, water 0-2 degC)
  P_BPF,             // TUSS4470 band-pass code for 200 kHz (reg 0x10; 0x1D = 196.8 kHz, datasheet); 190/210 = -1/+1
  P_THRESH,          // OUT_4 comparator threshold 0-15 (reg 0x17 bits 3:0), edge timing
  P_BOTTOM_MIN,      // D43: bottom search starts here (x0.1 m; prototype 0.6 m). Bucket test: 1-2
  P_VDRV,            // D45: transducer drive VDRV in volts (reg 0x16 = V - 5, 5-20 V); must stay below the
                     //      MT3608 output (VPWR > VDRV + 0.3 V, + the shield 1N4001: MT3608 >= VDRV + 1.5 V, D55). Full bridge: 2 x VDRV peak-to-peak
  P_VMODE,           // D53: VDRV regulator mode. 0 = always on (shield reference firmware, proven in water), 1 = off while
                     //      listening + re-charge before each burst (datasheet 7.3.1, quieter receive, unproven)
  // D57 interference / false-target rejection (8 sonars on one lake: every hole hears the others' pings)
  P_JITTER_MS,       // random 0..N ms added before every burst: another sonar's ping lands at a different depth each
                     //      time, so the spike filter can recognise it (0 = off)
  P_SPIKE_DB,        // cross-ping spike filter: a sample this many dB above everything near it in the 2 previous pings
                     //      is interference and is replaced (0 = off). Costs a new echo two pings of delay
  P_ECHOLEN,         // single-echo test: a fish echo at -6 dB is at most this x0.1 times the expected echo length
                     //      (ping length + 1 bin); longer = weeds / layer / noise, not reported (0 = off)
  P_BOTTOM_MAX,      // D104: deepest bottom (x0.1 m, 0 = whole range). Barrel / bucket test only: its multiple can out-echo the floor
  P_FTRIM,           // D111: burst frequency of the 200 kHz mode, 180 + ftrim kHz (20 = 200 kHz); 190 / 210 = -/+10 kHz. Set by TUNE
  P_DRIVE,           // D118: 0 = direct drive (internal H-bridge), 1 = transformer (PRE_DRIVER_MODE: OUTA/OUTB drive two external FETs)
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
    {"dead", "Dead zone", 0, 50, 5, "x0.1 m"},
    {"bsmooth", "Bottom smoothing", 5, 100, 25, "%"},
    {"learn", "Static scene learning", 1, 100, 12, "x0.001"},
    {"range", "Colour range", 20, 80, 46, "dB"},
    {"tvg", "Range compensation", 0, 1, 1, ""},
    {"cycles", "Burst cycles, base", 1, 32, 32, "cycles"},   // D109: 07nov sweep, 16 -> 32 = +4 dB on the floor, ring-down end unchanged
    {"gain", "Receiver gain", 0, 3, 2, "10/12.5/15/20 V/V"},
    {"pinghz", "Ping rate, active", 1, 16, 16, "x0.25/s"},
    {"fcycles", "Burst cycles, focus", 1, 32, 32, "cycles"},   // D107/D109: was 8; 07nov sweep 8 -> 16 -> 32 = +6.8 / +10.8 dB on the floor, ring-down end unchanged
    {"idlehz", "Ping rate, idle", 1, 16, 4, "x0.25/s"},
    {"avg", "Bursts averaged", 1, 4, 2, "per ping"},
    {"freq", "Frequency mode", 0, 2, 2, "0 3/ping 1 rot 2 200k"},
    {"sound", "Sound speed", 0, 255, 53, "+1350 m/s"},
    {"bpf", "Band-pass code (200k)", 1, 62, 29, "0x1D=196.8kHz"},
    {"thresh", "Edge threshold", 0, 15, 7, "reg 0x17"},
    {"bmin", "Bottom search from", 1, 50, 6, "x0.1 m"},
    {"vdrv", "Drive voltage", 5, 20, 20, "V (MT3608 at 22 V)"},   // D109: the MT3608 goes to 22 V (VPWR ~21.2 V after the 1N4001): 20 V regulates with margin
    {"vmode", "VDRV mode", 0, 2, 0, "0 on (proven) 1 quiet 2 external VDRV (D108)"},
    {"jitter", "Ping timing jitter", 0, 20, 8, "ms"},
    {"spike", "Interference filter", 0, 30, 10, "dB (0 off)"},
    {"echolen", "Max fish echo length", 0, 60, 30, "x0.1 ping (0 off)"},
    {"bmax", "Bottom search to", 0, 120, 0, "x0.1 m (0 = all)"},
    {"ftrim", "Burst frequency", 0, 40, 19, "180 + kHz (20 = 200 kHz)"},   // D113: TUNE 07nov 15:15, peak 198.8 kHz (parabola through 196-200), -3 dB band 196-202 kHz
    {"drive", "Drive stage", 0, 1, 0, "0 direct 1 transformer (pod v0.8 option)"},
  };
  return T[id < P_COUNT ? id : 0];
}

// D95: the default a knob had in firmware that saved knobs WITHOUT the defaults record ("d", up to D94): a saved value
// equal to it was never set by hand and follows the new default on the first boot. Only the knobs whose default changed.
inline uint8_t paramLegacyDefault(uint8_t id) {
  if (id == P_FREQ_MODE) return 0;      // D94: 2
  if (id == P_DEADZONE_DM) return 9;    // D88: 5
  return paramInfo(id).def;
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
