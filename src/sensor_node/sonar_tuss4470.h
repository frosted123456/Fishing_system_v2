// TUSS4470 sonar driver for the ESP32 WROOM tip-up (Open Echo shield 002), D43 / roadmap phase 3.
// WRITTEN BLIND: nothing here has run on hardware yet. Verify step by step in the bucket test
// (docs/SONAR_DRIVER.md). Built only with SONAR_REAL=1 (env sensor_wroom_sonar).
//
// One ping = per frequency (1 or 3): [converter off, only if an enable pin is wired] -> TUSS4470 time-of-flight start
// (reg 0x1B) -> burst of N cycles on IO2 from the RMT (12.5 ns steps) -> I2S-DMA ADC capture of the
// log-amp output (VOUT) -> time zero = the transmit leakage onset -> samples averaged into the
// processing's 2.5 cm depth bins at the set sound speed (1403 m/s default) -> 8-bit codes
// (95 dB / 255 codes, the prototype scale). Several bursts can be averaged (power) per ping.
// The OUT_4 comparator edge is time-stamped by the MCPWM capture unit (12.5 ns) for edge timing.
#pragma once
#include <Arduino.h>
#include <sonar_scene.h>   // BINS, NFREQ
#include <sonar_params.h>

#ifndef SONAR_REAL
#define SONAR_REAL 0
#endif

// ---- wiring: PROPOSED GPIO numbers, NOT CONFIRMED ----
// Frank's power/cable design (2026-10-07): 3 x L91 AA -> TPS63020 3.3 V (ESP32, Hall, shield logic) and
// MT3608 boost 12-15 V straight off the pack -> shield VIN. NO converter enable wire: idle = TUSS4470
// sleep mode over SPI (reg 0x1B bit 7). Cable up the mast: 3.3 V, GND, Hall out, VSPI SCLK/SDI/SDO/NCS,
// IO1, IO2, echo. GPIO numbers below are still proposals. VOUT on an ADC1 pin (I2S-ADC works on ADC1
// only: GPIO32-39). WROOM tip-up already uses 15 (Hall), 2 (LED), 34 (battery). GPIO5 is a strapping
// pin: fine as SPI CS (idles high).
#ifndef SONAR_PIN_SCK
#define SONAR_PIN_SCK   18
#define SONAR_PIN_MISO  19
#define SONAR_PIN_MOSI  23
#define SONAR_PIN_CS    5
#define SONAR_PIN_IO1   26
#define SONAR_PIN_IO2   25      // burst (RMT out)
#define SONAR_PIN_O4    27      // comparator OUT_4 (MCPWM capture), -1 = not wired (no edge timing)
#define SONAR_PIN_VOUT  36      // ADC1_CH0 (VP)
#define SONAR_PIN_BOOST -1      // converter enable: not in Frank's design (MT3608 always on). A GPIO here
                                // = converter off while listening (roadmap item 2), if ever added
#endif

// ---- acquisition constants (est.) ----
#define SONAR_ADC_HZ          150000   // I2S-ADC sample rate: ~5 samples per 2.5 cm bin at 1403 m/s
#define SONAR_CHARGE_MS       4        // pause before each burst (VDRV refill), est.
// VDRV_CTRL (reg 0x16) bits 3:0: VDRV = level + 5 V (datasheet). It is regulated from VPWR (MT3608 12-15 V),
// so it must stay below VPWR: 0x06 = 11 V for a 12 V MT3608 setting, est. (regulator headroom not checked).
// Set to 15 V? 0x09 = 14 V. open_echo's 0x0F = 20 V needs a 24-28 V supply.
#ifndef SONAR_VDRV_LEVEL
#define SONAR_VDRV_LEVEL      0x06
#endif
#define SONAR_WAKE_MS         5        // sleep -> active before the first burst (VDRV recharge), est.
#define SONAR_FREQ_HZ_0       190000
#define SONAR_FREQ_HZ_1       200000
#define SONAR_FREQ_HZ_2       210000

namespace tuss {

struct PingInfo {
  uint8_t nfreq;          // 1 or 3 (codes rows filled)
  int8_t rot_f;           // >= 0: rotating, this ping's frequency index (codes[0] holds it)
  int16_t t0;             // sample index of time zero (transmit onset) in the last capture
  int32_t edge_um;        // OUT_4 first edge after the blind zone, depth in micrometres (-1 none)
  uint16_t raw_max;       // largest raw ADC value of the capture (clipping check, 4095 = clipped)
  uint32_t us;            // time the ping took
  uint32_t adc_hz;        // MEASURED sample rate of the last capture (samples / capture time). Must read
                          // ~SONAR_ADC_HZ: the depth scale depends on it (bench line / STAT, first bucket step)
};

bool begin();                                             // false = TUSS4470 not answering (SPI)
void sleep();                                             // TUSS4470 sleep mode (reg 0x1B bit 7); safe before
                                                          // begin(). The next ping() wakes it.
bool ok();
// One ping with the current knobs. codes[f][bin]: 8-bit log codes (prototype scale).
bool ping(const icemesh::sonar::Params& p, bool focus, uint8_t codes[icemesh::sonar::sp::NFREQ][icemesh::sonar::BINS], PingInfo& info);
uint8_t readReg(uint8_t addr);
void writeReg(uint8_t addr, uint8_t v);
// Calibration: 17-point ADC curve (raw 0, 256, ... 4096 -> corrected), kept in NVS "sonarcal".
void setCalibration(const uint16_t pts[17]);
void getCalibration(uint16_t pts[17]);

}  // namespace tuss
