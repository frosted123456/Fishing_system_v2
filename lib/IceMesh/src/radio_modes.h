// LoRa radio modes used by the TDMA mesh (docs/review_protocol_proposal.md §1-2).
// All 500 kHz: single-channel digital modulation (DTS) under RSS-247 §5.2 — no hopping needed.
// (125 kHz single-channel would have to frequency-hop; not offered here.)
#ifndef ICEMESH_RADIO_MODES_H
#define ICEMESH_RADIO_MODES_H
#include <stdint.h>
#include "airtime.h"

namespace icemesh {

enum RadioMode : uint8_t {
  MODE_SF9_BW500 = 0,   // most robust: beacon, echo, join, relay links, default for hubs
  MODE_SF8_BW500 = 1,
  MODE_SF7_BW500 = 2,
  MODE_SF6_BW500 = 3,   // D79: SX126x-only short spreading factors: 62.5 kbps at SF5 (Semtech's headline figure)
  MODE_SF5_BW500 = 4,   // fastest: a 96 B slot in ~17 ms instead of 133 ms; ~12 dB less link budget than SF9 (est.)
  MODE_COUNT = 5,
};

struct ModeInfo {
  uint8_t sf;
  uint16_t bw_khz;
  int16_t sens_dbm_x10;  // SX1262 datasheet typical sensitivity at 500 kHz (not measured on this board)
  int16_t snr_limit_x10; // ESTIMATE: demodulation SNR limit (SF12 -20 dB ... SF5 -2.5 dB, 2.5 dB per step)
  const char* name;
};

inline const ModeInfo& modeInfo(RadioMode m) {
  static const ModeInfo table[MODE_COUNT] = {
      {9, 500, -1210, -125, "SF9/500"},   // sensitivity: SX1262 datasheet typ. at 500 kHz (radio review 2026-10-22; the
      {8, 500, -1180, -100, "SF8/500"},   // theory values were ~2.5 dB optimistic)
      {7, 500, -1160, -75, "SF7/500"},
      {6, 500, -1130, -50, "SF6/500"},
      {5, 500, -1110, -25, "SF5/500"},
  };
  return table[m < MODE_COUNT ? m : MODE_SF9_BW500];
}

inline bool validMode(uint8_t m) { return m < MODE_COUNT; }

// More robust (slower) / less robust (faster) neighbour. Returns the same mode at the ends.
inline RadioMode slowerMode(RadioMode m) { return m == MODE_SF9_BW500 ? m : static_cast<RadioMode>(m - 1); }
inline RadioMode fasterMode(RadioMode m) { return m == MODE_SF5_BW500 ? m : static_cast<RadioMode>(m + 1); }

// Project PHY: coding rate 4/5, preamble 8, CRC on, explicit header (src/lora_node/config.h).
inline LoRaPhy modePhy(RadioMode m) {
  LoRaPhy p;
  p.sf = modeInfo(m).sf;
  p.bw_hz = static_cast<uint32_t>(modeInfo(m).bw_khz) * 1000UL;
  p.cr_denom = 5;
  p.preamble = p.sf < 7 ? 12 : 8;   // D79: SX126x asks for >= 12 preamble symbols at SF5 / SF6
  p.crc = true;
  p.explicit_header = true;
  return p;
}

inline uint32_t modeAirtimeUs(RadioMode m, uint16_t len) { return airtimeUs(len, modePhy(m)); }

// LoRa channels (500 kHz, inside 902-928 MHz with margin). Index 0 = the v2 default 915.0 MHz.
// The chalet picks one (quietest at start-up, or set from the phone); hubs search them in this order.
static const uint8_t LORA_CHANNELS = 8;
inline float loraChannelMHz(uint8_t ch) {
  static const float f[LORA_CHANNELS] = {915.0f, 904.0f, 907.0f, 910.0f, 913.0f, 918.0f, 921.0f, 924.0f};
  return f[ch < LORA_CHANNELS ? ch : 0];
}

}  // namespace icemesh
#endif
