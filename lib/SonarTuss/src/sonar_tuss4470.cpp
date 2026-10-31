// TUSS4470 sonar driver - see sonar_tuss4470.h. WRITTEN BLIND, to verify in the bucket (docs/SONAR_DRIVER.md).
#include "sonar_tuss4470.h"
#include <string.h>
#if SONAR_REAL
#include <SPI.h>
#include <Preferences.h>
#include <driver/adc.h>
#include <driver/rmt.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>
#include <soc/io_mux_reg.h>
#include <driver/gpio.h>
#include <math.h>
#if CONFIG_IDF_TARGET_ESP32S3
#define SONAR_ADC_DIGI 1            // S3: digital ADC controller + GDMA (IDF 4.4 adc_digi_* API), see the header
#include <soc/soc_caps.h>           // SOC_ADC_DIGI_MAX_BITWIDTH
#else
#define SONAR_ADC_DIGI 0            // classic ESP32: I2S built-in ADC mode
#include <driver/i2s.h>
#include <soc/i2s_struct.h>
#include <soc/syscon_struct.h>
#endif

using namespace icemesh::sonar;

namespace tuss {

#if CONFIG_IDF_TARGET_ESP32S3
static SPIClass g_spi(HSPI);        // S3: HSPI = SPI3 host; the LoRa radio (RadioLib) owns the default SPI (SPI2) on 8-11
#else
static SPIClass g_spi(VSPI);        // classic ESP32: VSPI (18/19/23/5)
#endif
static bool g_ok = false;
static uint16_t g_cal[17];
static const rmt_channel_t RMT_CH = RMT_CHANNEL_0;
static const size_t MAX_SAMPLES = 3300;           // 488 bins x 5.57 samples = 2717 after time zero, + up to 3 ms (468) of
                                                  // pre-burst samples before the leakage (time-zero search window).
                                                  // S3 at 83.3 kHz: 488 x 2.97 = 1449 + 250 fit with room to spare
static uint16_t g_raw[MAX_SAMPLES];
static size_t g_raw_n = 0;          // D56: samples in g_raw from the last capture (RAW line)
// D58: ADC timing. The classic ESP32 clocks its DMA ADC from the 160 MHz PLL through the I2S dividers: the rate
// is crystal-exact, but the DIG ADC controller must start with a "max conversions" limit (255 in this driver)
// and restarts when it hits it, losing ~2 samples every 256 (esp-idf #10612, PR #1991). Espressif's own fix
// (commit 23dc9a9) clears that limit 60 us after the start: continuous sampling. We do the same, and check
// every capture: delivered rate (full DMA buffers against the us timer) vs the register rate. If a capture
// comes up short, the next ones keep the limit (the restart loss is then a known 0.8 %, not a stall).
// S3: no limit mechanism (conv_limit_en = 0 is the normal mode), so nolimit stays 1 and there is no fallback;
// the per-capture rate check (clk_ok) is kept as the sanity figure.
static uint32_t g_clock_hz = 0, g_meas_hz = 0;
static bool g_nolimit = false;      // clear the conversion limit at each capture
#if !SONAR_ADC_DIGI
static uint8_t g_short_n = 0;       // consecutive captures short of the clock rate while in nolimit mode
static uint16_t g_limited_n = 0;    // captures run in limited mode since the last fallback (retry nolimit after 64)
static void clearConvLimit() { delayMicroseconds(60); SYSCON.saradc_ctrl2.meas_num_limit = 0; }   // as esp-idf 23dc9a9
static uint32_t readClockHz() {     // f = 160 MHz / (div_num + div_b / div_a) / rx_bck_div_num / 2
  const uint32_t n = I2S0.clkm_conf.clkm_div_num, b = I2S0.clkm_conf.clkm_div_b, a = I2S0.clkm_conf.clkm_div_a;
  const uint32_t bck = I2S0.sample_rate_conf.rx_bck_div_num;
  const double div = (double)n + (a ? (double)b / (double)a : 0.0);
  if (div <= 0 || !bck) return 0;
  return (uint32_t)(160000000.0 / div / (double)bck / 2.0 + 0.5);
}
#else
// S3 digital ADC: one DMA interrupt = ADC_DMA_CHUNK bytes (256 samples x 4 B, ~3 ms at 83.3 kHz); the driver ring
// holds two whole captures so nothing drops while the loop below reads (a ring overflow is a failed capture).
static const uint32_t ADC_DMA_CHUNK = 1024;
static const uint32_t ADC_RING_BYTES = 28 * ADC_DMA_CHUNK;   // >= 2 x 3300 x 4 B; a chunk multiple, so no read ever
                                                             // straddles the ring end (byte ring: a split = a short read)
static adc1_channel_t g_adc_ch = ADC1_CHANNEL_0;
static uint8_t g_dma[ADC_DMA_CHUNK];                // one chunk of type2 words, unpacked into g_raw
static void adcDrain() {                            // samples converted between the last read and the stop stay in the
  uint8_t tmp[256]; uint32_t rd = 0;                // ring (adc_digi_start does not flush it): throw them away
  while (adc_digi_read_bytes(tmp, sizeof(tmp), &rd, 0) != ESP_ERR_TIMEOUT && rd) {}
}
// rate = 2.5 MHz / interval, interval = trunc(2.5 MHz / requested): adc_hal.c (IDF 4.4) sets the controller clock to
// APB 80 MHz / (ADC_LL_CLKM_DIV_NUM_DEFAULT 15 + 1) = 5 MHz and Fs = Fd / interval / 2
static uint32_t digiClockHz(uint32_t req) { const uint32_t iv = req ? 2500000UL / req : 0; return iv ? 2500000UL / iv : 0; }
static bool adcInit() {
  const int ch = digitalPinToAnalogChannel(SONAR_PIN_VOUT);
  if (ch < 0 || ch >= ADC1_CHANNEL_MAX) return false;   // ADC1 (GPIO1-10) only: the S3 digital ADC DMA is ADC1 only
  g_adc_ch = (adc1_channel_t)ch;
  adc_digi_init_config_t ic = {};
  ic.max_store_buf_size = ADC_RING_BYTES;
  ic.conv_num_each_intr = ADC_DMA_CHUNK;
  ic.adc1_chan_mask = 1u << g_adc_ch;
  ic.adc2_chan_mask = 0;
  if (adc_digi_initialize(&ic) != ESP_OK) return false;
  adc_digi_pattern_config_t pat = {};                // copied by the driver: a local is fine
  pat.atten = ADC_ATTEN_DB_12;                       // = old DB_11: ~0-3.1 V input range (log-amp VOUT must stay below)
  pat.channel = (uint8_t)g_adc_ch;
  pat.unit = 0;                                      // ADC1
  pat.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;         // 12 (the only width the S3 driver accepts)
  adc_digi_configuration_t dc = {};
  dc.conv_limit_en = 0; dc.conv_limit_num = 0;       // no conversion limit on the S3 (ESP32-only quirk)
  dc.pattern_num = 1; dc.adc_pattern = &pat;
  dc.sample_freq_hz = SONAR_ADC_HZ;                  // 611..83333 accepted
  dc.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  dc.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;          // 4 bytes per sample on the S3
  if (adc_digi_controller_configure(&dc) != ESP_OK) return false;
  g_clock_hz = digiClockHz(SONAR_ADC_HZ);
  return true;
}
// read one chunk, unpack up to `max` samples of our channel into dst; returns the count, `words` = conversions in the
// chunk (every channel, for the rate check); 0 on timeout / ring overflow (fail = true)
static size_t adcReadChunk(uint16_t* dst, size_t max, size_t& words, bool& fail) {
  uint32_t rd = 0;
  const esp_err_t e = adc_digi_read_bytes(g_dma, ADC_DMA_CHUNK, &rd, 40);
  if (e == ESP_ERR_TIMEOUT || e == ESP_ERR_INVALID_STATE || rd == 0) { fail = true; return 0; }   // INVALID_STATE = overflow
  words = rd / 4;
  size_t n = 0;
  for (uint32_t i = 0; i + 4 <= rd && n < max; i += 4) {
    const adc_digi_output_data_t* d = reinterpret_cast<const adc_digi_output_data_t*>(g_dma + i);
    if (d->type2.unit != 0 || d->type2.channel != (uint32_t)g_adc_ch) continue;   // not ours (arbiter junk)
    dst[n++] = (uint16_t)d->type2.data;
  }
  return n;
}
#endif
static uint32_t g_raw_hz = 0;       // D56: burst frequency of that capture
static volatile uint32_t g_cap_burst = 0, g_cap_edge = 0;
static volatile bool g_edge_seen = false;
static volatile uint32_t g_blind_ticks = 0;
static bool g_asleep = false, g_spi_up = false;
static bool g_swapped = false;   // D70: MISO / MOSI found swapped at begin() and the bus re-opened that way
static uint8_t g_stat = 0;   // status bits of the last SPI reply (high byte bits 5:0)
// TOF_CONFIG (reg 0x1B, datasheet): bit 7 SLEEP_MODE_EN, bit 6 STDBY_MODE_EN, bit 1 VDRV_TRIGGER, bit 0 CMD_TRIGGER
static const uint8_t TOF_SLEEP = 0x80, TOF_TRIGGER = 0x01;

// ---- SPI register access (TUSS4470 SPI protocol: 16-bit frame, odd parity bit, SPI mode 1) ----
static uint8_t parity16(uint16_t v) { uint8_t ones = 0; for (int i = 0; i < 16; i++) ones += (v >> i) & 1; return (uint8_t)((ones + 1) % 2); }
static uint16_t xfer(uint8_t b0, uint8_t b1) {
  b0 |= parity16((uint16_t)((b0 << 8) | b1));
  gpio_hold_dis((gpio_num_t)SONAR_PIN_CS);   // sleep() holds NCS high for deep sleep; any frame releases it
  g_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(SONAR_PIN_CS, LOW);
  const uint8_t r0 = g_spi.transfer(b0), r1 = g_spi.transfer(b1);
  digitalWrite(SONAR_PIN_CS, HIGH);
  g_spi.endTransaction();
  g_stat = (uint8_t)((r0 >> 1) & 0x3F);   // reply bits 14:9 = STAT5..0 (bit 15 parity error, bit 8 parity; datasheet figure 7-9)
  return (uint16_t)((r0 << 8) | r1);
}
void writeReg(uint8_t addr, uint8_t v) { xfer((uint8_t)((addr & 0x3F) << 1), v); }
uint8_t readReg(uint8_t addr) { return (uint8_t)(xfer((uint8_t)(0x80 | ((addr & 0x3F) << 1)), 0) & 0xFF); }
static void spiUp() {
  if (g_spi_up) return;
  pinMode(SONAR_PIN_CS, OUTPUT); digitalWrite(SONAR_PIN_CS, HIGH);
  if (g_swapped) g_spi.begin(SONAR_PIN_SCK, SONAR_PIN_MOSI, SONAR_PIN_MISO, SONAR_PIN_CS);
  else g_spi.begin(SONAR_PIN_SCK, SONAR_PIN_MISO, SONAR_PIN_MOSI, SONAR_PIN_CS);
  g_spi_up = true;
}
static bool probeId() { readReg(0x1D); return readReg(0x1D) == 0xB9; }   // first frame after a wake can be stale
// out of sleep mode; first releases the NCS hold that sleep() set, or no frame could reach the chip
static void wake() {
  if (SONAR_PIN_BOOST >= 0) {   // pod board v0.3: boost back on, let VPWR reach 24 V before the first burst
    gpio_hold_dis((gpio_num_t)SONAR_PIN_BOOST);
    pinMode(SONAR_PIN_BOOST, OUTPUT); digitalWrite(SONAR_PIN_BOOST, HIGH);
    delay(SONAR_BOOST_SETTLE_MS);
  }
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
  pinMode(SONAR_PIN_IO1, OUTPUT); digitalWrite(SONAR_PIN_IO1, HIGH);   // as the shield reference firmware: IO1 high, burst on IO2
  if (SONAR_PIN_BOOST >= 0) { pinMode(SONAR_PIN_BOOST, OUTPUT); digitalWrite(SONAR_PIN_BOOST, HIGH); }
  spiUp();
  wake();                                                              // out of sleep (if a deep sleep left it there)
  // presence check: DEVICE_ID (0x1D) reads 0xB9 (datasheet 7.6.1.12); a floating or stuck MISO gives 0x00 / 0xFF
  g_ok = probeId();
  if (!g_ok) { delay(10); g_ok = probeId(); }   // a slow wake is not a wiring fault
  if (!g_ok && !g_swapped) {   // D70: the one wiring slip the firmware can live with: MISO and MOSI crossed
    g_spi.end(); g_spi_up = false; g_swapped = true;
    spiUp(); wake();
    g_ok = probeId();
    if (!g_ok) { g_spi.end(); g_spi_up = false; g_swapped = false; spiUp(); }   // back to the documented pins
  }
  writeReg(0x10, 0x1D);   // 196.8 kHz band-pass until the first ping sets the knob
  writeReg(0x14, 0x1C);   // DEV_CTRL_3: IO_MODE 0 = clock on IO2, burst enabled by CMD_TRIGGER (datasheet 7.3.2);
                          // DRV_PLS_FLT_DT = 7 (check disabled, D72): with the deglitch at its 64 us default the
                          // DRV_PULSE_FLT bit is set on EVERY ping, because CMD_TRIGGER stays on for the ~20 ms capture
                          // after the 80 us burst and IO2 is quiet (datasheet 7.6.1.5). First bench, 2026-10-18.
                          // PULSE_NUM_FLT (pulses received != BURST_PULSE) stays the meaningful driver check.
  writeReg(0x16, 0x0F);   // VDRV 20 V, regulator always on, not Hi-Z: the reference firmware init (vmode 0); ping() sets the knob

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

#if !SONAR_ADC_DIGI
  // ADC: I2S built-in ADC mode (ADC1 only), continuous DMA
  i2s_config_t ic = {};
  ic.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
  ic.sample_rate = SONAR_ADC_HZ; ic.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  ic.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT; ic.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  ic.dma_buf_count = 8; ic.dma_buf_len = 512; ic.use_apll = false;
  if (i2s_driver_install(I2S_NUM_0, &ic, 0, nullptr) != ESP_OK) { g_ok = false; return false; }
  const adc1_channel_t ch = (adc1_channel_t)digitalPinToAnalogChannel(SONAR_PIN_VOUT);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ch, ADC_ATTEN_DB_12);   // = old DB_11: ~0-3.1 V input range (log-amp VOUT must stay below)
  i2s_set_adc_mode(ADC_UNIT_1, ch);
  // D58: the clock registers are written by i2s_adc_enable(); read them back once for the real rate, and
  // measure the delivered rate over ~200 ms of full DMA buffers (sanity figure on the INFO line)
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_adc_enable(I2S_NUM_0);
  g_clock_hz = readClockHz();
  clearConvLimit();   // measure the mode the captures run in
  {
    uint8_t* dst = reinterpret_cast<uint8_t*>(g_raw); size_t rd = 0; uint32_t tA = 0, tB = 0; uint32_t full = 0;
    const uint32_t t_end = millis() + 200;
    while (millis() < t_end) {
      if (i2s_read(I2S_NUM_0, dst, 1024, &rd, pdMS_TO_TICKS(40)) != ESP_OK || rd != 1024) break;
      if (!tA) tA = micros(); else { tB = micros(); full++; }
    }
    g_meas_hz = (full && tB > tA) ? (uint32_t)((uint64_t)full * 512 * 1000000ULL / (tB - tA)) : 0;
  }
  i2s_adc_disable(I2S_NUM_0);
#else
  // ADC: S3 digital controller + DMA, ADC1 only, 83.3 kHz (see the header). Same ~200 ms delivered-rate
  // measurement as above, counting whole DMA chunks (256 words each) against the us timer
  if (!adcInit()) { g_ok = false; return false; }
  if (adc_digi_start() != ESP_OK) { g_ok = false; return false; }
  {
    uint32_t rd = 0, tA = 0, tB = 0, full = 0;
    const uint32_t t_end = millis() + 200;
    while (millis() < t_end) {
      if (adc_digi_read_bytes(g_dma, ADC_DMA_CHUNK, &rd, 40) != ESP_OK || rd != ADC_DMA_CHUNK) break;
      if (!tA) tA = micros(); else { tB = micros(); full++; }
    }
    g_meas_hz = (full && tB > tA) ? (uint32_t)((uint64_t)full * (ADC_DMA_CHUNK / 4) * 1000000ULL / (tB - tA)) : 0;
  }
  adc_digi_stop();
  adcDrain();
#endif
  g_nolimit = true;

  return g_ok;
}
uint32_t clockHz() { return g_clock_hz; }
uint32_t measuredHz() { return g_meas_hz; }
bool ok() { return g_ok; }
uint8_t lastStatus() { return g_stat; }

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
  if (SONAR_PIN_BOOST >= 0) {   // pod board v0.3: boost off through deep sleep (EN held low)
    pinMode(SONAR_PIN_BOOST, OUTPUT); digitalWrite(SONAR_PIN_BOOST, LOW);
    gpio_hold_en((gpio_num_t)SONAR_PIN_BOOST);
  }
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
static uint8_t g_vmode = 0;   // knob vmode: 0 = VDRV regulator always on (reference firmware), 1 = quiet listen (datasheet)

static bool capture(uint32_t hz, uint8_t cycles, float sound, float acc[BINS], PingInfo& info) {
#if SONAR_ADC_DIGI
  // only as many samples as the bins need: 3 ms time-zero window + 488 bins at this rate (+ margin); at 83.3 kHz
  // ~1750 samples = 21 ms per capture instead of 40 ms for the whole buffer (ping cost, radio-quiet horizon)
  const uint32_t rate0 = g_clock_hz ? g_clock_hz : SONAR_ADC_HZ;
  size_t want = (size_t)(rate0 * 3 / 1000) + (size_t)(2.0f * 0.025f / sound * (float)rate0 * BINS) + 64;
  if (want > MAX_SAMPLES) want = MAX_SAMPLES;
#else
  const size_t want = MAX_SAMPLES;
#endif
  if (g_vmode == 1) {   // DIS_VDRV_REG_LSTN mode: the regulator is off, charge VDRV for this burst first (up to
    writeReg(0x1B, 0x02);                                         // 4 ms: before the ADC starts, so the burst stays within
    for (int w = 0; w < 20 && !(lastStatus() & 0x20); w++) { delayMicroseconds(200); readReg(0x1C); }   // the time-zero window)
  }
  g_edge_seen = false;
#if !SONAR_ADC_DIGI
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_adc_enable(I2S_NUM_0);
  info.nolimit = g_nolimit ? 1 : 0;
  if (g_nolimit) clearConvLimit();                                // D58: continuous sampling
  writeReg(0x1B, TOF_TRIGGER);                                    // CMD_TRIGGER, as the reference firmware (vmode 1: VDRV_TRIGGER back to 0 "just before the burst", datasheet 7.3.1)
  burst(hz, cycles);
  size_t got = 0, rd = 0;
  uint8_t* dst = reinterpret_cast<uint8_t*>(g_raw);
  // rate check (D58): time from the end of the first full DMA buffer to the end of the last read, for the samples in
  // between (the first read's start latency is left out; the old version timed only up to a partial read and came
  // out ~10 % low). Task wake-up jitter is some tens of us on ~18 ms: the 0.5 % tolerance is well under the 0.78 %
  // loss of the limited mode, and one short capture is not enough to give nolimit up (3 in a row are; a stall is).
  uint32_t tA = 0, tB = 0; size_t gotA = 0; bool timeout = false;
  while (got < want * 2) {
    const size_t chunk = (want * 2 - got) < 1024 ? (want * 2 - got) : 1024;   // one DMA buffer (512 x 16 bit) per read
    if (i2s_read(I2S_NUM_0, dst + got, chunk, &rd, pdMS_TO_TICKS(40)) != ESP_OK || rd == 0) { timeout = true; break; }
    got += rd;
    if (!tA) { if (rd == 1024) { tA = micros(); gotA = got; } } else tB = micros();
  }
  info.adc_hz = (tB > tA && got > gotA) ? (uint32_t)((uint64_t)(got - gotA) / 2 * 1000000ULL / (tB - tA)) : 0;
  info.clk_ok = (g_clock_hz && info.adc_hz && (uint32_t)labs((long)info.adc_hz - (long)g_clock_hz) * 1000 <= g_clock_hz * 5) ? 1 : 0;
  if (g_nolimit) {
    if (timeout) { g_nolimit = false; g_short_n = 0; g_limited_n = 0; }
    else if (!info.clk_ok) { if (++g_short_n >= 3) { g_nolimit = false; g_short_n = 0; g_limited_n = 0; } }
    else g_short_n = 0;
  } else if (++g_limited_n >= 64) { g_nolimit = true; g_limited_n = 0; }   // try the continuous mode again now and then
  i2s_adc_disable(I2S_NUM_0);
  writeReg(0x1B, 0x00);
  const size_t n = got / 2;
  g_raw_n = 0;
  if (n < 200) return false;
  g_raw_n = n; g_raw_hz = hz;
  // I2S-ADC quirk on the classic ESP32: samples come in swapped pairs; 12 data bits, channel in the top 4
  uint16_t mx = 0;
  for (size_t i = 0; i + 1 < n; i += 2) { const uint16_t a = g_raw[i]; g_raw[i] = g_raw[i + 1]; g_raw[i + 1] = a; }
  for (size_t i = 0; i < n; i++) { g_raw[i] = cal(g_raw[i] & 0x0FFF); if (g_raw[i] > mx) mx = g_raw[i]; }
#else
  // S3: start the digital controller before the burst, read ~want samples (one 256-word DMA chunk per read,
  // unpacked from the 4-byte type2 words, other channels skipped), stop. The same rate check as above: whole
  // chunks between the first full chunk and the last read against the us timer. No limit mode: nolimit = 1,
  // no fallback; a short capture only clears clk_ok (sanity figure on the bench line).
  adcDrain();
  if (adc_digi_start() != ESP_OK) return false;
  info.nolimit = 1;
  writeReg(0x1B, TOF_TRIGGER);                                    // CMD_TRIGGER, as the reference firmware (vmode 1: VDRV_TRIGGER back to 0 "just before the burst", datasheet 7.3.1)
  burst(hz, cycles);
  size_t got = 0, words = 0, wordsA = 0; uint32_t tA = 0, tB = 0; bool fail = false;
  while (got < want) {
    size_t w = 0;
    const size_t k = adcReadChunk(g_raw + got, want - got, w, fail);
    if (fail) break;
    got += k; words += w;
    if (!tA) { if (w == ADC_DMA_CHUNK / 4) { tA = micros(); wordsA = words; } } else tB = micros();
  }
  adc_digi_stop();
  adcDrain();
  info.adc_hz = (tB > tA && words > wordsA) ? (uint32_t)((uint64_t)(words - wordsA) * 1000000ULL / (tB - tA)) : 0;
  info.clk_ok = (g_clock_hz && info.adc_hz && (uint32_t)labs((long)info.adc_hz - (long)g_clock_hz) * 1000 <= g_clock_hz * 5) ? 1 : 0;
  writeReg(0x1B, 0x00);
  const size_t n = got;
  g_raw_n = 0;
  if (n < 200) return false;
  g_raw_n = n; g_raw_hz = hz;
  uint16_t mx = 0;   // 12-bit data already unpacked; no pair swap on the S3
  for (size_t i = 0; i < n; i++) { g_raw[i] = cal(g_raw[i] & 0x0FFF); if (g_raw[i] > mx) mx = g_raw[i]; }
#endif
  info.raw_max = mx;
  // time zero: first sample above 80 % of the largest value of the first 3 ms (transmit leakage on VOUT),
  // if that leakage stands clearly (> ~27 dB) above the level a few ms later; else a fixed bench value
  // D58: the register rate sets the depth scale; in limited mode the stream is short of 2 samples per 256
  const uint32_t rate = (g_clock_hz ? g_clock_hz : SONAR_ADC_HZ) * (info.nolimit ? 256UL : 254UL) / 256UL;
  const size_t lim = n < rate * 3 / 1000 ? n : rate * 3 / 1000;
  uint16_t m0 = 0; for (size_t i = 0; i < lim; i++) if (g_raw[i] > m0) m0 = g_raw[i];
  // reference = the quietest 100-sample window after the search window (not the first one: on a 2-3 m lake the
  // bottom echo sits right there and would hide the leakage)
  uint32_t best = 0xFFFFFFFFUL;
  for (size_t i = lim; i + 100 <= n; i += 100) { uint32_t w = 0; for (size_t k = i; k < i + 100; k++) w += g_raw[k]; if (w < best) best = w; }
  const uint16_t ref = best != 0xFFFFFFFFUL ? (uint16_t)(best / 100) : 0;
  size_t t0 = 0;
  if (m0 > ref + 1000) { while (t0 < lim && g_raw[t0] < (uint16_t)(m0 * 4 / 5)) t0++; info.t0_ok = 1; }
  else { t0 = SONAR_T0_FALLBACK; info.t0_ok = 0; }
  info.t0 = (int16_t)t0;
  // bins: depth d -> round trip 2 d / c -> sample t0 + 2 d / c * rate; mean of the samples of each bin
  const float spb = 2.0f * 0.025f / sound * (float)rate;   // samples per bin (~5.6)
  for (int b = 0; b < BINS; b++) {
    const size_t s0 = t0 + (size_t)(b * spb), s1 = t0 + (size_t)((b + 1) * spb);
    uint32_t sum = 0; size_t k = 0;
    for (size_t s = s0; s < s1 && s < n; s++, k++) sum += g_raw[s];
    // raw -> dB: log-amp slope (datasheet typ.) over the ADC full scale; -100 dB at code 0 (relative scale,
    // the processing works on differences to its noise floor). Clipped above code 255 (-5 dB) later.
    const float db = k ? -100.0f + (float)sum / (float)k * (SONAR_ADC_FS_MV / 4096.0f / SONAR_MV_PER_DB) : -100.0f;
    acc[b] += powf(10.0f, db / 10.0f);
  }
  return true;
}

void lastRaw(const uint16_t*& samples, size_t& n, uint32_t& burst_hz) { samples = g_raw; n = g_raw_n; burst_hz = g_raw_hz; }

bool ping(const Params& p, bool focus, uint8_t codes[sp::NFREQ][BINS], PingInfo& info) {
  if (!g_ok) return false;
  const uint32_t t_start = micros();
  if (g_asleep) { wake(); delay(SONAR_WAKE_MS); }
  const float sound = 1350.0f + p[P_SOUND];
  const uint8_t cycles = focus ? p[P_CYCLES_FOCUS] : p[P_PULSE_CYCLES];
  const uint8_t avg = focus ? 1 : p[P_AVG];
  // VDRV_CTRL (0x16): bits 3:0 = VDRV - 5 V. vmode 0 = regulator always on, default charge current: exactly what the
  // the reference firmware writes (0x0F at 20 V) and what has fish on screen. vmode 1 = bit 6 DIS_VDRV_REG_LSTN (no
  // charging while listening) + bit 4 20 mA charge, re-armed by VDRV_TRIGGER in capture(): datasheet 7.3.1, unproven.
  g_vmode = p[P_VMODE];
  writeReg(0x16, (uint8_t)((g_vmode == 1 ? 0x50 : 0x00) | ((p[P_VDRV] - 5) & 0x0F)));
  // DEV_CTRL_2 (0x13): bits 1:0 LNA_GAIN codes are NOT in order (0 = 15, 1 = 10, 2 = 20, 3 = 12.5 V/V,
  // datasheet): knob 0..3 = 10, 12.5, 15, 20 V/V. Bit 2 VOUT_SCALE_SEL = 0 (3.3 V map), both log-amp stages on.
  static const uint8_t LNA_CODE[4] = {1, 3, 0, 2};
  writeReg(0x13, LNA_CODE[p[P_GAIN] & 3]);
  // ECHO_INT_CONFIG (0x17): bits 3:0 threshold, bit 4 comparator enable (only with OUT_4 wired)
  writeReg(0x17, (uint8_t)((p[P_THRESH] & 0x0F) | (SONAR_PIN_O4 >= 0 ? 0x10 : 0)));
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
    // BPF_CONFIG_1 (0x10) centre frequency, datasheet table: 0x1C 185.8, 0x1D 196.8, 0x1E 206.1, 0x1F 218.3 kHz:
    // one code step per 10 kHz here, so knob = the 200 kHz code, 190 = knob - 1, 210 = knob + 1
    const int bpf = (int)p[P_BPF] + ((int)fset[f] - 1);
    writeReg(0x10, (uint8_t)(bpf < 0 ? 0 : (bpf > 63 ? 63 : bpf)));
    uint8_t good = 0;
    for (uint8_t a = 0; a < avg; a++) {
      delay(SONAR_CHARGE_MS);
      // D57: random 0..jitter ms before every shot, so 8 tip-ups on one lake never stay in step: another tip-up's
      // ping then lands at a different depth on each of our pings and the cross-ping spike filter removes it
      if (p[P_JITTER_MS]) delay(esp_random() % ((uint32_t)p[P_JITTER_MS] + 1));
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
  info.fault = (uint8_t)(readReg(0x1C) & 0x06);   // DEV_STAT: PULSE_NUM_FLT (bit 2) / DRV_PULSE_FLT (bit 1) of the last burst
  info.edge_um = g_edge_seen ? (int32_t)((float)(g_cap_edge - g_cap_burst) / 80e6f * sound / 2.0f * 1e6f) : -1;
  info.us = micros() - t_start;
  return true;
}

bool pinsSwapped() { return g_swapped; }

// D70 wiring check of the shield: SPI answer, status bits, VDRV, one ping with the default knobs, and a look at
// the echo trace: time-zero leakage seen = the burst reaches the receiver (driver + IO2 alive), ring-down length
// = the transducer is loading the output (open header: a few bins; transducer: tens). Heuristic, says so on the page.
void shieldTest(ShieldTest& t) {
  memset(&t, 0, sizeof(t));
  t.spi_ok = g_ok || (g_spi_up && probeId());   // begin() may have failed on a shield wired since: say so, no promotion
  t.swapped = g_swapped;
  if (!g_ok) return;                             // the ping needs the drivers begin() installs
  const bool was_asleep = g_asleep;              // an idle shield goes back to sleep after the test
  wake();
  writeReg(0x16, 0x0F);   // VDRV 20 V regulator on (vmode 0), as begin()
  delay(30);              // VDRV charges C12 (est.)
  readReg(0x1C);
  const uint8_t st = lastStatus();
  t.vdrv_ready = (st & 0x20) != 0;
  t.ee_fault = (st & 0x04) != 0;
  icemesh::sonar::Params p;   // defaults
  uint8_t codes[icemesh::sonar::sp::NFREQ][icemesh::sonar::BINS];
  PingInfo pi;
  if (!ping(p, false, codes, pi)) { if (was_asleep) sleep(); return; }
  t.pinged = true;
  t.pulse_fault = (pi.fault & 0x04) != 0; t.drv_fault = (pi.fault & 0x02) != 0;
  t.leakage = pi.t0_ok != 0;
  t.raw_max = pi.raw_max;
  // noise floor: median-ish of the last quarter of the bins; ring: bins after time zero above floor + 20 codes
  const uint8_t* c = codes[0];
  uint32_t sum = 0; uint16_t n = 0;
  for (uint16_t i = icemesh::sonar::BINS * 3 / 4; i < icemesh::sonar::BINS; i++) { sum += c[i]; n++; }
  t.noise_code = n ? (uint8_t)(sum / n) : 0;
  uint16_t ring = 0;
  for (uint16_t i = 0; i < icemesh::sonar::BINS; i++) { if (c[i] > (uint16_t)t.noise_code + 20) ring = i + 1; else if (i > ring + 4) break; }
  t.ring_bins = ring > 255 ? 255 : (uint8_t)ring;
  if (was_asleep) sleep();
}

}  // namespace tuss

#else   // SONAR_REAL == 0: nothing (the fake sonar runs instead)
namespace tuss {
bool begin() { return false; }
uint32_t clockHz() { return 0; }
uint32_t measuredHz() { return 0; }
void sleep() {}
bool ok() { return false; }
bool ping(const icemesh::sonar::Params&, bool, uint8_t[icemesh::sonar::sp::NFREQ][icemesh::sonar::BINS], PingInfo&) { return false; }
uint8_t readReg(uint8_t) { return 0; }
uint8_t lastStatus() { return 0; }
void writeReg(uint8_t, uint8_t) {}
bool pinsSwapped() { return false; }
void shieldTest(ShieldTest& t) { memset(&t, 0, sizeof(t)); t.not_built = true; }
void lastRaw(const uint16_t*& samples, size_t& n, uint32_t& burst_hz) { samples = nullptr; n = 0; burst_hz = 0; }
void setCalibration(const uint16_t[17]) {}
void getCalibration(uint16_t pts[17]) { for (int i = 0; i < 17; i++) pts[i] = (uint16_t)(i * 256); }
}
#endif
