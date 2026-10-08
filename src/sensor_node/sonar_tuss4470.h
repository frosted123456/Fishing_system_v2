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
// sleep mode over SPI (reg 0x1B bit 7). Mast cable (shielded 12-core): 3.3 V, GND, Hall out, SCLK, MOSI,
// MISO, CS, IO1, IO2, echo (= VOUT) + echo-GND twisted, 1 spare. Frank's plan: VSPI 18/19/23/5, IO1/IO2 on
// two free GPIOs (e.g. 25, 26), echo on ADC1, Hall on an RTC GPIO. IO1/IO2/echo numbers still to confirm. VOUT on an ADC1 pin (I2S-ADC works on ADC1
// only: GPIO32-39). WROOM tip-up already uses 27 (Hall latch), 2 (LED), 34 (battery). GPIO5 is a strapping
// pin: fine as SPI CS (idles high).
#ifndef SONAR_PIN_SCK
#define SONAR_PIN_SCK   18
#define SONAR_PIN_MISO  19
#define SONAR_PIN_MOSI  23
#define SONAR_PIN_CS    5
#define SONAR_PIN_IO1   26
#define SONAR_PIN_IO2   25      // burst (RMT out)
#define SONAR_PIN_O4    -1      // comparator OUT_4 (MCPWM capture): not in the 12-core mast cable (one echo
                                // wire = VOUT). The spare core + a free GPIO here enables edge timing
#define SONAR_PIN_VOUT  36      // ADC1_CH0 (VP)
#define SONAR_PIN_BOOST -1      // converter enable: not in Frank's design (MT3608 always on). A GPIO here
                                // = converter off while listening (roadmap item 2), if ever added
#endif

// ---- acquisition constants (est.) ----
#define SONAR_ADC_HZ          150000   // I2S-ADC sample rate: ~5 samples per 2.5 cm bin at 1403 m/s
#define SONAR_CHARGE_MS       1        // settle before each burst; the real VDRV refill is the VDRV_READY wait in capture()
// Drive voltage: knob "vdrv" (volts). VDRV_CTRL (reg 0x16) bits 3:0 = VDRV - 5 V, 5-20 V (datasheet), charged
// from VPWR (MT3608): keep VPWR > VDRV + 0.3 V (datasheet), so knob = MT3608 setting - 1 V. Default 11 V for the
// 12.0 V setting. MT3608 at 21 V -> vdrv 20 = 40 V p-p across the transducer (full bridge, 2 x VDRV).
#define SONAR_WAKE_MS         10       // sleep -> SPI ready (datasheet power-up time 10 ms; sleep exit likely faster)
// Log-amp output: 29.7 mV/dB typical with VOUT_SCALE_SEL = 0 (3.3 V map, datasheet; 25-33 mV/dB over parts),
// ADC 12-bit over ~3.3 V -> 0.0271 dB per count (est.: the real ADC full scale and the part's slope differ;
// the CAL curve corrects the ADC, the slope stays an estimate until a known-level test).
#define SONAR_MV_PER_DB       29.7f
#define SONAR_ADC_FS_MV       3300.0f
// Time zero: the transmit leakage on VOUT in the first 3 ms, else (no leakage visible) this sample index,
// measured on the bench: burst start after the ADC start (est.)
#define SONAR_T0_FALLBACK     30
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
  uint8_t fault;          // DEV_STAT bits after the burst: 0x04 wrong pulse count, 0x02 driver stuck (clock lost)
  uint8_t t0_ok;          // 1 = time zero from the leakage, 0 = SONAR_T0_FALLBACK used (no leakage seen)
  uint32_t adc_hz;        // MEASURED sample rate of the last capture (samples / capture time). Must read
                          // ~SONAR_ADC_HZ: the depth scale depends on it (bench line / STAT, first bucket step)
};

bool begin();                                             // false = TUSS4470 not answering (SPI)
uint8_t lastStatus();                                     // SPI status bits of the last frame (datasheet 7.4:
                                                          // bit5 VDRV_READY, 4 PULSE_NUM_FLT, 3 DRV_PULSE_FLT, 2 EE_CRC_FLT, 1:0 state)
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
