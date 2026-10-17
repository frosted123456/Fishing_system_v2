// TUSS4470 sonar driver (TUSS4470 shield 002), D43 / roadmap phase 3. Shared by both firmwares:
// the classic ESP32 WROOM tip-up (src/sensor_node) and the Heltec WiFi LoRa 32 V3 / ESP32-S3 roles
// (src/lora_node). WRITTEN BLIND: nothing here has run on hardware yet. Verify step by step in the
// bucket test (docs/SONAR_DRIVER.md). Built only with SONAR_REAL=1 (env sensor_wroom_sonar).
//
// One ping = per frequency (1 or 3): [converter off, only if an enable pin is wired] -> TUSS4470 time-of-flight start
// (reg 0x1B) -> burst of N cycles on IO2 from the RMT (12.5 ns steps) -> DMA ADC capture of the
// log-amp output (VOUT) -> time zero = the transmit leakage onset -> samples averaged into the
// processing's 2.5 cm depth bins at the set sound speed (1403 m/s default) -> 8-bit codes
// (95 dB / 255 codes, the prototype scale). Several bursts can be averaged (power) per ping.
// The OUT_4 comparator edge is time-stamped by the MCPWM capture unit (12.5 ns) for edge timing.
//
// Classic ESP32 vs ESP32-S3 (same SPI / RMT / MCPWM code, only the ADC capture differs):
// - ESP32: I2S built-in ADC mode (I2S0 DMA), 156.25 kHz from the 160 MHz PLL, conversion-limit
//   restart quirk handled per D58 (clearConvLimit / nolimit fallback).
// - S3: no I2S-ADC. The digital ADC controller + GDMA (IDF 4.4 adc_digi_* API) samples ADC1 at
//   83.3 kHz, the driver's ceiling (SOC_ADC_SAMPLE_FREQ_THRES_HIGH = 83333). The controller runs at
//   5 MHz (APB 80 MHz / 16) and triggers every `interval` ticks with a /2: rate = 2.5 MHz / interval,
//   interval = trunc(2.5 MHz / requested) (Discord / IDF 4.4 adc_hal.c finding): 83333 -> interval 30
//   -> 83333.3 Hz. clockHz() applies that formula; no conversion limit, no restart loss, nolimit = 1.
//   ~2.97 samples per 2.5 cm bin instead of ~5.6: still above the 2 per bin the depth maths wants.
//   Output = 4 bytes per sample (type2: 12-bit data + channel + unit), no I2S pair swap.
#pragma once
#include <Arduino.h>
#include <sonar_scene.h>   // BINS, NFREQ
#include <sonar_params.h>

#ifndef SONAR_REAL
#define SONAR_REAL 0
#endif

// ---- wiring: winter 2026 build (shield 002 + DevKit V1), header table in the repo README (D55) ----
// Frank's power/cable design (2026-10-07): 3 x L91 AA -> TPS63020 3.3 V (ESP32, Hall, shield logic) and
// MT3608 boost 12-15 V straight off the pack -> shield VIN. NO converter enable wire: idle = TUSS4470
// sleep mode over SPI (reg 0x1B bit 7). Mast cable (shielded 12-core): 3.3 V, GND, Hall out, SCLK, MOSI,
// MISO, CS, IO1, IO2, echo (= VOUT) + echo-GND twisted, 1 spare. Frank's plan: VSPI 18/19/23/5, IO1/IO2 on
// two free GPIOs (e.g. 25, 26), echo on ADC1, Hall on an RTC GPIO. IO1/IO2/echo numbers still to confirm. VOUT on an ADC1 pin (I2S-ADC works on ADC1
// only: GPIO32-39). WROOM tip-up already uses 27 (Hall latch), 2 (LED), 34 (battery). GPIO5 is a strapping
// pin: fine as SPI CS (idles high).
//
// Heltec WiFi LoRa 32 V3 (ESP32-S3, lora_node roles): the core defines WIFI_LoRa_32_V3 for that board
// (BOARD_HELTEC_V3 accepted too). Free header pins (est., not yet verified on hardware): the V3 already uses
// 8-14 (LoRa SX1262 SPI/DIO/RST/BUSY), 17/18/21 (OLED), 41/42 (CardKB I2C), 7 (buzzer), 4 (reed), 2 (battery
// divider), 35 (LED), 36 (Vext), 1/37 (VBAT_Read / ADC_Ctrl), 5 (button), 0 (PRG). The sonar gets its own SPI
// host (SPI3, "HSPI" on the S3): the radio owns the default SPI object. VOUT must be an ADC1 pin (GPIO1-10):
// the digital ADC DMA on the S3 is ADC1 only. GPIO33/34 are plain GPIOs on the V3 (no octal PSRAM).
// The SONAR_PIN_SCK..VOUT block is all-or-nothing: override all seven together (build flags) or none.
#ifndef SONAR_PIN_SCK
#if defined(WIFI_LoRa_32_V3) || defined(BOARD_HELTEC_V3)
#define SONAR_PIN_SCK   47      // est.
#define SONAR_PIN_MISO  33      // est.
#define SONAR_PIN_MOSI  48      // est.
#define SONAR_PIN_CS    34      // est.
#define SONAR_PIN_IO1   38      // est.
#define SONAR_PIN_IO2   39      // burst (RMT out), est.
#define SONAR_PIN_VOUT  6       // ADC1_CH5, est.
#else                           // BOARD_ESP32WROOM (DevKit V1), also the default for any other board
#define SONAR_PIN_SCK   18
#define SONAR_PIN_MISO  19
#define SONAR_PIN_MOSI  23
#define SONAR_PIN_CS    5
#define SONAR_PIN_IO1   26
#define SONAR_PIN_IO2   25      // burst (RMT out)
#define SONAR_PIN_VOUT  36      // ADC1_CH0 (VP)
#endif
#endif
#ifndef SONAR_PIN_O4
#define SONAR_PIN_O4    -1      // comparator OUT_4 (MCPWM capture): not in the 12-core mast cable (one echo
                                // wire = VOUT). Pod board v0.3: GPIO32 (build flag). Heltec V3: none
#endif
#ifndef SONAR_PIN_BOOST
#define SONAR_PIN_BOOST -1      // MT3608 EN. Shield + modules: none (always on). Pod board v0.3: GPIO33 (build
                                // flag): HIGH while the sonar runs, LOW in sleep() -> no boost quiescent in sleep.
                                // Never toggled during a ping (VPWR feeds the TUSS4470 regulator, D53 vmode 0).
                                // Heltec V3: none
#endif
#define SONAR_BOOST_SETTLE_MS 20   // MT3608 soft start + C8/C9 charge to 24 V (est., measure on the bench)

// ---- acquisition constants (est.) ----
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define SONAR_ADC_HZ          83333    // digital ADC request: the IDF 4.4 ceiling, interval 2.5 MHz / 83333 = 30 (truncated)
                                       // -> 83333.3 Hz, ~2.97 samples per 2.5 cm bin. The depth maths uses clockHz() (D58)
#else
#define SONAR_ADC_HZ          156250   // I2S-ADC sample rate request: 160 MHz PLL / 32 exactly (integer divider, no
                                       // fractional dither), ~5.6 samples per 2.5 cm bin. The depth maths uses the rate
                                       // read back from the clock registers (clockHz()), not this constant (D58)
#endif
#define SONAR_CHARGE_MS       1        // settle before each burst; the real VDRV refill is the VDRV_READY wait in capture()
// Drive voltage: knob "vdrv" (volts). VDRV_CTRL (reg 0x16) bits 3:0 = VDRV - 5 V, 5-20 V (datasheet), charged
// from VPWR (MT3608): keep VPWR > VDRV + 0.3 V (datasheet). Default 20 V (chip max, the reference tests ran
// 15-20 V) = MT3608 at 21.5 V (shield 1N4001 ~0.8 V + 0.3 V headroom, D55), 40 V p-p across the transducer (full bridge, 2 x VDRV). MT3608 still at 12 V:
// KNOB vdrv 11. Knob "vmode": 0 = regulator always on (reference firmware, proven), 1 = off while listening (datasheet).
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
  uint32_t adc_hz;        // D58: delivered sample rate of the last capture, measured between full DMA buffers
                          // (~0.1 %). Below clockHz() by more than 0.3 % = the ADC lost samples (conversion-limit
                          // restarts, see capture()): clk_ok = 0 and the driver keeps the limit on for the next pings
  uint8_t clk_ok;         // 1 = delivered rate within 0.3 % of the clock rate
  uint8_t nolimit;        // 1 = this capture ran with the conversion limit cleared (continuous sampling). S3: always 1
};

bool begin();                                             // false = TUSS4470 not answering (SPI)
uint32_t clockHz();                                       // D58: ADC sample rate from the I2S clock registers (ESP32) or the
                                                          // digital controller interval formula (S3), crystal-locked
                                                          // (+-20 ppm): the depth scale. 0 before begin()
uint32_t measuredHz();                                    // D58: sample rate measured over ~200 ms at begin() (sanity)
uint8_t lastStatus();                                     // SPI status bits of the last frame (datasheet 7.4:
                                                          // bit5 VDRV_READY, 4 PULSE_NUM_FLT, 3 DRV_PULSE_FLT, 2 EE_CRC_FLT, 1:0 state)
void sleep();                                             // TUSS4470 sleep mode (reg 0x1B bit 7); safe before
                                                          // begin(). The next ping() wakes it.
bool ok();
// One ping with the current knobs. codes[f][bin]: 8-bit log codes (prototype scale).
bool ping(const icemesh::sonar::Params& p, bool focus, uint8_t codes[icemesh::sonar::sp::NFREQ][icemesh::sonar::BINS], PingInfo& info);
uint8_t readReg(uint8_t addr);
void writeReg(uint8_t addr, uint8_t v);
// D56: the last capture's samples (last frequency / last average of the last ping), after the I2S pair swap and
// the CAL curve, 12 bit. For the RAW recording line. n = 0 before the first ping.
void lastRaw(const uint16_t*& samples, size_t& n, uint32_t& burst_hz);
// Calibration: 17-point ADC curve (raw 0, 256, ... 4096 -> corrected), kept in NVS "sonarcal".
void setCalibration(const uint16_t pts[17]);
void getCalibration(uint16_t pts[17]);

}  // namespace tuss
