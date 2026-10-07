// TUSS4470 sonar driver - see sonar_tuss4470.h. WRITTEN BLIND, to verify in the bucket (docs/SONAR_DRIVER.md).
#include "sonar_tuss4470.h"
#if SONAR_REAL
#include <SPI.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <driver/rmt.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>
#include <soc/io_mux_reg.h>
#include <driver/gpio.h>
#include <math.h>

using namespace icemesh::sonar;

namespace tuss {

static SPIClass vspi(VSPI);
static bool g_ok = false;
static uint16_t g_cal[17];
static const rmt_channel_t RMT_CH = RMT_CHANNEL_0;
static const size_t MAX_SAMPLES = 2800;           // 12.2 m round trip at 1403 m/s = 17.4 ms = 2610 samples at 150 kHz
static uint16_t g_raw[MAX_SAMPLES];
static volatile uint32_t g_cap_burst = 0, g_cap_edge = 0;
static volatile bool g_edge_seen = false;
static volatile uint32_t g_blind_ticks = 0;
static bool g_asleep = false, g_spi_up = false;
// TOF_CONFIG (reg 0x1B, datasheet): bit 7 SLEEP_MODE_EN, bit 6 STDBY_MODE_EN, bit 1 VDRV_TRIGGER, bit 0 CMD_TRIGGER
static const uint8_t TOF_SLEEP = 0x80, TOF_TRIGGER = 0x01;

// ---- SPI register access (open_echo protocol: 16-bit frame, odd parity bit, SPI mode 1) ----
static uint8_t parity16(uint16_t v) { uint8_t ones = 0; for (int i = 0; i < 16; i++) ones += (v >> i) & 1; return (uint8_t)((ones + 1) % 2); }
static uint16_t xfer(uint8_t b0, uint8_t b1) {
  b0 |= parity16((uint16_t)((b0 << 8) | b1));
  gpio_hold_dis((gpio_num_t)SONAR_PIN_CS);   // sleep() holds NCS high for deep sleep; any frame releases it
  vspi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(SONAR_PIN_CS, LOW);
  const uint8_t r0 = vspi.transfer(b0), r1 = vspi.transfer(b1);
  digitalWrite(SONAR_PIN_CS, HIGH);
  vspi.endTransaction();
  return (uint16_t)((r0 << 8) | r1);
}
void writeReg(uint8_t addr, uint8_t v) { xfer((uint8_t)((addr & 0x3F) << 1), v); }
uint8_t readReg(uint8_t addr) { return (uint8_t)(xfer((uint8_t)(0x80 | ((addr & 0x3F) << 1)), 0) & 0xFF); }
static void spiUp() {
  if (g_spi_up) return;
  pinMode(SONAR_PIN_CS, OUTPUT); digitalWrite(SONAR_PIN_CS, HIGH);
  vspi.begin(SONAR_PIN_SCK, SONAR_PIN_MISO, SONAR_PIN_MOSI, SONAR_PIN_CS);
  g_spi_up = true;
}
// out of sleep mode; first releases the NCS hold that sleep() set, or no frame could reach the chip
static void wake() {
  gpio_hold_dis((gpio_num_t)SONAR_PIN_CS);
  digitalWrite(SONAR_PIN_CS, HIGH);
  writeReg(0x1B, 0x00);
  g_asleep = false;
}

// ---- comparator edge: MCPWM capture (CAP0 = burst start on IO2, CAP1 = OUT_4), 80 MHz ticks ----
static bool IRAM_ATTR capCb(mcpwm_unit_t, mcpwm_capture_channel_id_t ch, const cap_event_data_t* e, void*) {
  // CAP0 sees every cycle of the burst: only the FIRST edge of a burst (> 1 ms after the previous
  // edge) is time zero, so the blind window and edge_um count from the burst start like t0
  if (ch == MCPWM_SELECT_CAP0) {
    static uint32_t last = 0;
    if ((uint32_t)(e->cap_value - last) > 80000UL) { g_cap_burst = e->cap_value; g_edge_seen = false; }
    last = e->cap_value;
  }
  else if (ch == MCPWM_SELECT_CAP1 && !g_edge_seen && (uint32_t)(e->cap_value - g_cap_burst) > g_blind_ticks) {
    g_cap_edge = e->cap_value; g_edge_seen = true;
  }
  return false;
}

static void loadCal() {
  Preferences p; p.begin("sonarcal", true);
  const size_t n = p.getBytes("c", g_cal, sizeof(g_cal));
  p.end();
  if (n != sizeof(g_cal)) for (int i = 0; i < 17; i++) g_cal[i] = (uint16_t)(i * 256 > 4095 ? 4095 : i * 256);   // identity
}
void setCalibration(const uint16_t pts[17]) {
  memcpy(g_cal, pts, sizeof(g_cal));
  Preferences p; p.begin("sonarcal", false); p.putBytes("c", g_cal, sizeof(g_cal)); p.end();
}
void getCalibration(uint16_t pts[17]) { memcpy(pts, g_cal, sizeof(g_cal)); }
static inline uint16_t cal(uint16_t raw) {   // piecewise linear, 16 segments
  const uint16_t k = raw >> 8, f = raw & 0xFF;
  return (uint16_t)(g_cal[k] + (((int32_t)g_cal[k + 1] - g_cal[k]) * f >> 8));
}

bool begin() {
  loadCal();
  pinMode(SONAR_PIN_IO1, OUTPUT); digitalWrite(SONAR_PIN_IO1, HIGH);   // as open_echo: IO1 high, burst on IO2
  if (SONAR_PIN_BOOST >= 0) { pinMode(SONAR_PIN_BOOST, OUTPUT); digitalWrite(SONAR_PIN_BOOST, HIGH); }
  spiUp();
  wake();                                                              // out of sleep (if a deep sleep left it there)
  // presence check: write/read back the threshold register
  writeReg(0x17, 0x5A);
  g_ok = readReg(0x17) == 0x5A;
  writeReg(0x16, 0x06);   // VDRV 11 V until the first ping sets the knob (safe with the 12 V MT3608 setting)

  // edge timing: MCPWM capture (IDF 4.4 legacy API). IO2 is BOTH the RMT burst output and the CAP0
  // input: mcpwm_gpio_init makes the pin input-only, so it runs FIRST, the RMT then takes the pin as
  // output (that clears the input enable) and the input is switched back on last. The in-matrix route
  // to CAP0 stays. Bench check: "edge" on the bench line must not be "-" with a target in the water.
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, SONAR_PIN_IO2);
  if (SONAR_PIN_O4 >= 0) mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, SONAR_PIN_O4);
  mcpwm_capture_config_t cc = {};
  cc.cap_edge = MCPWM_POS_EDGE; cc.cap_prescale = 1; cc.capture_cb = capCb; cc.user_data = nullptr;
  mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &cc);
  if (SONAR_PIN_O4 >= 0) mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, &cc);

  // burst generator: RMT channel 0 on IO2, 80 MHz (12.5 ns)
  rmt_config_t rc = RMT_DEFAULT_CONFIG_TX((gpio_num_t)SONAR_PIN_IO2, RMT_CH);
  rc.clk_div = 1; rc.mem_block_num = 2;
  rmt_config(&rc); rmt_driver_install(RMT_CH, 0, 0);
  PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[SONAR_PIN_IO2]);   // keep CAP0 seeing the burst we drive

  // ADC: I2S built-in ADC mode (ADC1 only), continuous DMA
  i2s_config_t ic = {};
  ic.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
  ic.sample_rate = SONAR_ADC_HZ; ic.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  ic.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT; ic.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  ic.dma_buf_count = 8; ic.dma_buf_len = 512; ic.use_apll = false;
  i2s_driver_install(I2S_NUM_0, &ic, 0, nullptr);
  const adc1_channel_t ch = (adc1_channel_t)digitalPinToAnalogChannel(SONAR_PIN_VOUT);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ch, ADC_ATTEN_DB_12);   // = old DB_11: ~0-3.1 V input range (log-amp VOUT must stay below)
  i2s_set_adc_mode(ADC_UNIT_1, ch);
  i2s_adc_disable(I2S_NUM_0);

  return g_ok;
}
bool ok() { return g_ok; }

// Before every deep sleep and when the sonar stops: the shield stays powered (no enable wire), so this
// is its only low-power state. Works without begin() (only SPI is brought up).
void sleep() {
  spiUp();
  writeReg(0x1B, TOF_SLEEP);
  g_asleep = true;
  // NCS held HIGH through deep sleep: a floating CS + SCLK could clock in a frame and wake the chip.
  // (A 10 k pull-up on NCS at the shield does the same in hardware.)
  digitalWrite(SONAR_PIN_CS, HIGH);
  gpio_hold_en((gpio_num_t)SONAR_PIN_CS);
  gpio_deep_sleep_hold_en();
}

static void burst(uint32_t hz, uint8_t cycles) {
  static rmt_item32_t items[33];
  const uint32_t half = 40000000UL / hz;   // 80 MHz ticks per half period
  if (cycles > 32) cycles = 32;
  for (uint8_t i = 0; i < cycles; i++) { items[i].level0 = 1; items[i].duration0 = half; items[i].level1 = 0; items[i].duration1 = half; }
  items[cycles].val = 0;   // end marker
  rmt_write_items(RMT_CH, items, cycles + 1, false);
}

// one capture at one frequency -> dB per bin (power average into `acc`)
static bool capture(uint32_t hz, uint8_t cycles, float sound, float acc[BINS], PingInfo& info) {
  const size_t want = MAX_SAMPLES;
  if (SONAR_PIN_BOOST >= 0) digitalWrite(SONAR_PIN_BOOST, LOW);   // item 2: quiet receive (VDRV holds the burst)
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_adc_enable(I2S_NUM_0);
  writeReg(0x1B, TOF_TRIGGER);                                    // time-of-flight start
  burst(hz, cycles);
  size_t got = 0, rd = 0;
  uint8_t* dst = reinterpret_cast<uint8_t*>(g_raw);
  // rate check: time between the end of the first DMA read and the end of the last one, for the
  // samples in between (the first read's start latency is left out)
  uint32_t tA = 0; size_t gotA = 0;
  while (got < want * 2) {
    const size_t chunk = (want * 2 - got) < 1024 ? (want * 2 - got) : 1024;   // one DMA buffer (512 x 16 bit) per read
    if (i2s_read(I2S_NUM_0, dst + got, chunk, &rd, pdMS_TO_TICKS(40)) != ESP_OK || rd == 0) break;
    got += rd;
    if (!tA) { tA = micros(); gotA = got; }
  }
  const uint32_t tB = micros();
  info.adc_hz = (tB > tA && got > gotA) ? (uint32_t)((uint64_t)(got - gotA) / 2 * 1000000ULL / (tB - tA)) : 0;
  i2s_adc_disable(I2S_NUM_0);
  writeReg(0x1B, 0x00);
  if (SONAR_PIN_BOOST >= 0) digitalWrite(SONAR_PIN_BOOST, HIGH);  // recharge until the next ping
  const size_t n = got / 2;
  if (n < 200) return false;
  // I2S-ADC quirk on the classic ESP32: samples come in swapped pairs; 12 data bits, channel in the top 4
  uint16_t mx = 0;
  for (size_t i = 0; i + 1 < n; i += 2) { const uint16_t a = g_raw[i]; g_raw[i] = g_raw[i + 1]; g_raw[i + 1] = a; }
  for (size_t i = 0; i < n; i++) { g_raw[i] = cal(g_raw[i] & 0x0FFF); if (g_raw[i] > mx) mx = g_raw[i]; }
  info.raw_max = mx;
  // time zero: first sample above 80 % of the largest value of the first 3 ms (transmit leakage on VOUT)
  const size_t lim = n < SONAR_ADC_HZ * 3 / 1000 ? n : SONAR_ADC_HZ * 3 / 1000;
  uint16_t m0 = 0; for (size_t i = 0; i < lim; i++) if (g_raw[i] > m0) m0 = g_raw[i];
  size_t t0 = 0; while (t0 < lim && g_raw[t0] < (uint16_t)(m0 * 4 / 5)) t0++;
  info.t0 = (int16_t)t0;
  // bins: depth d -> round trip 2 d / c -> sample t0 + 2 d / c * rate; mean of the samples of each bin
  const float spb = 2.0f * 0.025f / sound * SONAR_ADC_HZ;   // samples per bin (~5.3)
  for (int b = 0; b < BINS; b++) {
    const size_t s0 = t0 + (size_t)(b * spb), s1 = t0 + (size_t)((b + 1) * spb);
    uint32_t sum = 0; size_t k = 0;
    for (size_t s = s0; s < s1 && s < n; s++, k++) sum += g_raw[s];
    // raw -> dB (prototype scale -100..-5 dB over the ADC range: 95 dB / 4096 counts, est. - calibrate)
    const float db = k ? -100.0f + 95.0f * (float)sum / (float)k / 4096.0f : -100.0f;
    acc[b] += powf(10.0f, db / 10.0f);
  }
  return true;
}

bool ping(const Params& p, bool focus, uint8_t codes[sp::NFREQ][BINS], PingInfo& info) {
  if (!g_ok) return false;
  const uint32_t t_start = micros();
  if (g_asleep) { wake(); delay(SONAR_WAKE_MS); }
  const float sound = 1350.0f + p[P_SOUND];
  const uint8_t cycles = focus ? p[P_CYCLES_FOCUS] : p[P_PULSE_CYCLES];
  const uint8_t avg = focus ? 1 : p[P_AVG];
  writeReg(0x16, (uint8_t)((p[P_VDRV] - 5) & 0x0F));   // VDRV = knob volts (10 mA charge, bit 4 = 0)
  writeReg(0x13, p[P_GAIN] & 3);        // LNA gain
  writeReg(0x10, p[P_BPF] & 0x3F);      // band-pass centre (one code for the 3 frequencies: TODO per-frequency codes, datasheet Table 7.1)
  writeReg(0x17, p[P_THRESH]);          // OUT_4 threshold
  // BURST_PULSE bits 5:0 = pulse count, and 0 = CONTINUOUS burst (datasheet): never write 0. Bits 7:6
  // (half-bridge, pre-driver mode) stay 0. The RMT on IO2 sends the real count.
  writeReg(0x1A, (uint8_t)((cycles < 1 ? 1 : cycles) & 0x3F));
  g_blind_ticks = (uint32_t)(2.0f * p[P_DEADZONE_DM] * 0.1f / sound * 80e6f);
  static uint8_t rot = 0;
  const uint8_t mode = p[P_FREQ_MODE];
  const uint32_t HZ[3] = {SONAR_FREQ_HZ_0, SONAR_FREQ_HZ_1, SONAR_FREQ_HZ_2};
  uint8_t fset[3]; uint8_t nf;
  if (mode == 0) { fset[0] = 0; fset[1] = 1; fset[2] = 2; nf = 3; info.rot_f = -1; }
  else if (mode == 1) { fset[0] = rot; nf = 1; info.rot_f = (int8_t)rot; rot = (uint8_t)((rot + 1) % 3); }
  else { fset[0] = 1; nf = 1; info.rot_f = -1; }
  static float acc[BINS];
  for (uint8_t f = 0; f < nf; f++) {
    for (int b = 0; b < BINS; b++) acc[b] = 0;
    uint8_t good = 0;
    for (uint8_t a = 0; a < avg; a++) {
      delay(SONAR_CHARGE_MS);
      if (capture(HZ[fset[f]], cycles, sound, acc, info)) good++;
    }
    if (!good) return false;
    for (int b = 0; b < BINS; b++) {   // power mean -> dB -> code (95 dB / 255)
      const float db = 10.0f * log10f(acc[b] / good + 1e-12f);
      const float c = (db + 100.0f) * 255.0f / 95.0f;
      codes[f][b] = (uint8_t)(c < 0 ? 0 : (c > 255 ? 255 : c + 0.5f));
    }
  }
  info.nfreq = nf;
  info.edge_um = g_edge_seen ? (int32_t)((float)(g_cap_edge - g_cap_burst) / 80e6f * sound / 2.0f * 1e6f) : -1;
  info.us = micros() - t_start;
  return true;
}

}  // namespace tuss

#else   // SONAR_REAL == 0: nothing (the fake sonar runs instead)
namespace tuss {
bool begin() { return false; }
void sleep() {}
bool ok() { return false; }
bool ping(const icemesh::sonar::Params&, bool, uint8_t[icemesh::sonar::sp::NFREQ][icemesh::sonar::BINS], PingInfo&) { return false; }
uint8_t readReg(uint8_t) { return 0; }
void writeReg(uint8_t, uint8_t) {}
void setCalibration(const uint16_t[17]) {}
void getCalibration(uint16_t pts[17]) { for (int i = 0; i < 17; i++) pts[i] = (uint16_t)(i * 256); }
}
#endif
