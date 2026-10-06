// TDMA LoRa mesh — radio task. See mesh_radio.h and docs/protocol_v2.md.
//
// Timing: every slot time is relative to REF = start of the chalet beacon, computed as
// TxDone - airtime(beacon) at the chalet and RxDone - airtime(beacon) at a hub (or from a relay
// hub's ECHO), so REF is exactly periodic whatever the beacon length. Interrupt
// times are taken in the DIO1 ISR with esp_timer_get_time(); TX starts are placed with
// vTaskDelay + a short busy-wait. Values marked "est." are estimates for the radio test mode.
#include "mesh_radio.h"
#include <RadioLib.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "config.h"
#include <hub_role.h>
#include <chalet_role.h>
#include <sonar_link.h>
#include <eb_link.h>

using namespace icemesh;
using namespace icemesh::tdma;

// ---------------------------------------------------------------------------------------------
// Radio + shared state
// ---------------------------------------------------------------------------------------------
static SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

static TaskHandle_t g_task = nullptr;
static SemaphoreHandle_t g_mx = nullptr;
static QueueHandle_t g_nodeq = nullptr;

static volatile bool g_irq = false;
static volatile uint32_t g_irq_us = 0;

static bool g_chalet = false;
static bool g_radio_ok = false;
static uint8_t g_net = 0x42;
static RadioMode g_cur_mode = MODE_COUNT;

static HubRole<24>* g_hub = nullptr;
static ChaletRole<10, 48>* g_ch = nullptr;   // 10 hubs, 48 nodes (3 tip-ups per hub + spares)

// counters (written by the task, read by loop)
static volatile uint32_t g_rx_ok = 0, g_rx_crc = 0, g_tx = 0, g_last_rx_ms = 0;
static volatile uint16_t g_frame = 0;

// hub side state shared with loop
static volatile uint16_t g_battery_mv = 0;
static volatile uint8_t g_silence_req = 0;        // HF_SILENCE_ON / HF_SILENCE_OFF while pending
static volatile uint8_t g_silence_req_frames = 0;
static volatile bool g_silence_event = false;
static volatile bool g_silence_state = false;
static volatile bool g_reset_event = false;
// Survives ESP.restart() (not power-off): the RESET ALL already executed, so the beacon repeating it
// for 5 s does not reboot this hub a second time.
RTC_NOINIT_ATTR static uint32_t g_rtc_cmd_magic;
RTC_NOINIT_ATTR static uint8_t g_rtc_cmd_seq;
static uint32_t g_test_counter = 0;
static uint16_t g_last_beacon_len = 40;

// chalet side config shared with loop
static volatile uint8_t g_cfg_test = TEST_OFF;
static volatile bool g_cfg_adaptive = false;
static volatile bool g_cfg_reset_stats = false;
static volatile uint8_t g_cfg_focus = 0;
static volatile bool g_cfg_reset_all = false;
static volatile bool g_cfg_sonar_sim = false;
static sonar::SonarStore<16, 128>* g_sonar = nullptr;   // chalet only (~16 KB)
static uint8_t g_cmd_seq = 0;
static uint32_t g_silence_until_ms = 0;
static uint8_t g_self = 0;

// ---- v2 network config: LoRa channel, transport, ESP-NOW backbone (docs/protocol_v2.md §6c) ----
// Values marked est. come from the simulation (docs/SIM_RESULTS.md), not from field measurements.
static const float BUSY_DBM = -100.0f;          // est.: RSSI above this on a channel counts as foreign activity
static const uint8_t CH_BUSY_PCT = 25;          // est.: Auto moves away from a channel busier than this...
static const uint8_t CH_BETTER_PCT = 15;        // ...when another one is quieter by at least this much
static const uint32_t CH_EVAL_MS = 60000;
static const uint32_t SCAN_DWELL_US = 60000;    // chalet: one other channel sampled per frame (between frames)
static const uint32_t BOOT_SCAN_US = 250000;    // chalet: per channel at boot (Auto)
static const uint32_t HUB_HOP_MS = 2500;        // hub without beacon: next channel every 2.5 s...
static const uint32_t HUB_HOP_QUIET_MS = 10000; // ...unless this network was heard in the last 10 s
static const uint8_t CMD_BEACONS = 6;           // each beacon command is repeated in 6 beacons (SET_CHANNEL: target 5..0)
static const bool EB_LR = ESPNOW_LONG_RANGE_MODE;   // backbone rate follows the ice side (LR on hubs and tip-ups)

static volatile uint8_t g_lora_ch = 0;          // network channel
static uint8_t g_tuned_ch = 0;                  // channel the SX1262 is tuned to (radio task only)
static volatile uint8_t g_req_ch = 0xFF;        // hub: channel named by a backbone beacon (applied by the radio task)
static volatile uint32_t g_heard_net_ms = 0;    // hub: last packet of this network (LoRa) or backbone channel hint
static uint32_t g_last_hop_ms = 0;
static volatile uint32_t g_pending_since_ms = 0;
static volatile bool g_lora_idle = false;       // hub: LoRa off (ESP-NOW only)
static volatile bool g_ch_event = false;
static volatile uint8_t g_cfg_transport = TR_AUTO;
static volatile uint8_t g_cfg_channel = MESH_CH_AUTO;
static volatile bool g_cfg_rescan = false;
static volatile uint8_t g_busy_pct[LORA_CHANNELS] = {255, 255, 255, 255, 255, 255, 255, 255};
static uint32_t g_busy_n[LORA_CHANNELS], g_busy_hit[LORA_CHANNELS];   // radio task only
static uint8_t g_scan_rr = 0;
static uint32_t g_ch_eval_ms = 0;
static volatile uint16_t g_ch_moves = 0;
static uint8_t g_ch_switch_after = 0xFF;        // chalet: retune at the end of this frame
static volatile uint32_t g_eb_beacon_until_ms = 0;
struct QueuedCmd { uint8_t cmd, target, value; };
static QueuedCmd g_cmdq[8];                     // chalet, under lock
static uint8_t g_cmdq_n = 0;
static uint8_t g_cmd_beacons_left = 0;
// backbone
static MeshEbSendFn g_eb_send = nullptr;
static volatile bool g_eb_relay = false;
static eb::Dedup* g_dedup = nullptr;            // under lock
static eb::TransportPolicy g_pol;               // hub, under lock
static uint16_t g_eb_seq = 0;
static uint32_t g_eb_last_ms = 0;
static volatile bool g_eb_kick = false;
static volatile bool g_hub_eb_on = false, g_hub_lora_on = true;
static volatile uint32_t g_eb_rx = 0, g_eb_tx = 0, g_eb_relayed = 0, g_eb_dup = 0;
static volatile uint32_t g_eb_beacon_ms = 0;    // hub: last backbone beacon
static volatile bool g_eb_beacon_seen = false;
static volatile uint32_t g_eb_hub_last_ms = 0;  // chalet: last hub packet over the backbone
static volatile bool g_relay_event = false;
static volatile uint8_t g_relay_dev = 0;
static volatile bool g_relay_on = false;
struct HubPath { uint8_t id, flags, hops; uint32_t lora_ms, eb_ms; };
static HubPath g_paths[12];                     // chalet, under lock: how each hub was last heard
// Setup phase (chalet): no automatic channel move until a hub has been in the network for 5 min, or
// 10 min after boot (a channel so busy that no hub can join). Hubs: TransportPolicy::armed().
static const uint32_t CH_ARM_HUB_MS = 300000, CH_ARM_BOOT_MS = 600000;
static volatile uint32_t g_first_hub_ms = 0;
static bool chaletArmed() {
  const uint32_t now = millis();
  return (g_first_hub_ms != 0 && now - g_first_hub_ms >= CH_ARM_HUB_MS) || now >= CH_ARM_BOOT_MS;
}
static uint32_t chaletArmInS() {
  if (chaletArmed()) return 0;
  const uint32_t now = millis();
  uint32_t a = CH_ARM_BOOT_MS - now;
  if (g_first_hub_ms != 0) { const uint32_t b = CH_ARM_HUB_MS - (now - g_first_hub_ms); if (b < a) a = b; }
  return (a + 999) / 1000;
}

struct Lock {
  Lock() { xSemaphoreTake(g_mx, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(g_mx); }
};

static inline uint32_t nowUs() { return static_cast<uint32_t>(esp_timer_get_time()); }
static inline bool reached(uint32_t t) { return static_cast<int32_t>(nowUs() - t) >= 0; }

static void IRAM_ATTR onDio1() {
  g_irq_us = static_cast<uint32_t>(esp_timer_get_time());
  g_irq = true;
  BaseType_t woken = pdFALSE;
  if (g_task != nullptr) vTaskNotifyGiveFromISR(g_task, &woken);
  if (woken) portYIELD_FROM_ISR();
}

static void sleepUntil(uint32_t t_us) {
  int32_t d = static_cast<int32_t>(t_us - nowUs());
  if (d > 3000) vTaskDelay(pdMS_TO_TICKS((d - 2000) / 1000));
  while (static_cast<int32_t>(t_us - nowUs()) > 0) { /* short busy-wait for an exact TX start */ }
}

// Wait for DIO1 until `deadline_us`. Returns true on interrupt.
static bool waitIrq(uint32_t deadline_us) {
  for (;;) {
    if (g_irq) return true;
    const int32_t rem = static_cast<int32_t>(deadline_us - nowUs());
    if (rem <= 0) return g_irq;
    if (rem < 1500) continue;                       // last ms: spin, a tick would overshoot the deadline
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS((rem - 1000) / 1000));
  }
}

static void clearIrq() { g_irq = false; ulTaskNotifyTake(pdTRUE, 0); }

// Standby with the oscillator kept running: STDBY_RC would make every TX/RX start wait for the
// TCXO again (RadioLib's TCXO delay, ~5 ms by default — longer than the 3 ms slot LEAD).
static inline void standbyXosc() { radio.standby(RADIOLIB_SX126X_STANDBY_XOSC); }

static bool setMode(RadioMode m) {
  if (m == g_cur_mode) return true;
  standbyXosc();
  const ModeInfo& mi = modeInfo(m);
  int s1 = radio.setBandwidth(static_cast<float>(mi.bw_khz));
  int s2 = radio.setSpreadingFactor(mi.sf);
  if (s1 != RADIOLIB_ERR_NONE || s2 != RADIOLIB_ERR_NONE) { g_cur_mode = MODE_COUNT; return false; }
  g_cur_mode = m;
  return true;
}

static volatile uint32_t g_tx_late_skips = 0;
static const uint32_t MAX_TX_LATE_US = 1000;   // est.: later than this, the packet would overrun its slot

// Transmit at `at_us`. Slot transmissions (slot = true) are dropped if we are already more than
// MAX_TX_LATE_US late; others go out now. `end_us` = TxDone time.
static bool txAt(const uint8_t* buf, size_t len, RadioMode m, uint32_t at_us, uint32_t& end_us, bool slot = true) {
  if (!g_radio_ok || len == 0) return false;
  setMode(m);
  sleepUntil(at_us);
  if (slot && static_cast<int32_t>(nowUs() - at_us) > static_cast<int32_t>(MAX_TX_LATE_US)) {
    g_tx_late_skips = g_tx_late_skips + 1;
    return false;
  }
  clearIrq();
  if (radio.startTransmit(buf, len) != RADIOLIB_ERR_NONE) { standbyXosc(); return false; }
  const uint32_t limit = nowUs() + modeAirtimeUs(m, static_cast<uint16_t>(len)) + 50000;   // est. margin
  const bool ok = waitIrq(limit);
  end_us = ok ? g_irq_us : nowUs();
  radio.finishTransmit();
  standbyXosc();
  if (ok) g_tx = g_tx + 1;
  return ok;
}

// ---- LoRa channel + activity meter ----
static void tune(uint8_t ch) {
  if (!g_radio_ok || ch >= LORA_CHANNELS || ch == g_tuned_ch) return;
  standbyXosc();
  if (radio.setFrequency(loraChannelMHz(ch)) == RADIOLIB_ERR_NONE) g_tuned_ch = ch;
}

// One instantaneous RSSI sample (radio in RX) for the activity meter of channel ch.
static inline void busySample(uint8_t ch) {
  if (ch >= LORA_CHANNELS) return;
  g_busy_n[ch]++;
  if (radio.getRSSI(false) > BUSY_DBM) g_busy_hit[ch]++;
}

// waitIrq() that also samples the RSSI about once per ms (FreeRTOS tick) while waiting.
static bool waitIrqSampling(uint32_t deadline_us, uint8_t ch) {
  for (;;) {
    if (g_irq) return true;
    const int32_t rem = static_cast<int32_t>(deadline_us - nowUs());
    if (rem <= 0) return g_irq;
    if (rem < 2500) return waitIrq(deadline_us);
    busySample(ch);
    ulTaskNotifyTake(pdTRUE, 1);
  }
}

// Listen on channel ch until until_us only to measure activity (nothing is decoded), then retune
// to the network channel.
static void sampleChannel(uint8_t ch, uint32_t until_us) {
  if (!g_radio_ok) { sleepUntil(until_us); return; }
  tune(ch);
  setMode(MODE_SF9_BW500);
  clearIrq();
  radio.startReceive();
  while (static_cast<int32_t>(until_us - nowUs()) > 2500) {
    busySample(ch);
    vTaskDelay(1);
  }
  sleepUntil(until_us);
  standbyXosc();
  clearIrq();
  tune(g_lora_ch);
}

// Busy % per channel from the samples, then halve the counters (older samples fade out).
static void busyPublish() {
  for (uint8_t c = 0; c < LORA_CHANNELS; c++) {
    if (g_busy_n[c] >= 50) g_busy_pct[c] = static_cast<uint8_t>((g_busy_hit[c] * 100UL + g_busy_n[c] / 2) / g_busy_n[c]);
    g_busy_n[c] /= 2; g_busy_hit[c] /= 2;
  }
}

// Auto rule (same as the simulation): stay unless the current channel is busier than CH_BUSY_PCT and
// another one is quieter by CH_BETTER_PCT; 915.0 MHz (channel 0) gets a 5 % preference. -1 = stay.
static int betterChannel(uint8_t cur) {
  const uint8_t pc = g_busy_pct[cur];
  if (pc == 255 || pc <= CH_BUSY_PCT) return -1;
  int best = -1, bq = 1000;
  for (uint8_t c = 0; c < LORA_CHANNELS; c++) {
    if (c == cur || g_busy_pct[c] == 255) continue;
    const int q = g_busy_pct[c] - (c == 0 ? 5 : 0);
    if (q < bq) { bq = q; best = c; }
  }
  if (best >= 0 && g_busy_pct[best] + CH_BETTER_PCT < pc) return best;
  return -1;
}

struct RxRes { bool irq; bool got; bool crc; uint8_t len; int8_t rssi; int8_t snr_q4; uint32_t end_us; };

static const uint32_t RX_EXTEND_US = 2000;     // est.: < LEAD_US, so a late packet never eats the next slot

// Listen in mode `m` from `open_us` to `close_us`. A packet whose header arrived before close
// may finish up to `extend_us` later (bounded so it never runs into the next slot / beacon).
// meter_ch < LORA_CHANNELS: also feeds the activity meter while waiting (chalet, between frames).
static RxRes rxWindow(RadioMode m, uint32_t open_us, uint32_t close_us, uint8_t* buf, uint32_t extend_us = RX_EXTEND_US,
                      uint8_t meter_ch = 0xFF) {
  RxRes r;
  memset(&r, 0, sizeof(r));
  if (!g_radio_ok) { sleepUntil(close_us); return r; }
  setMode(m);
  if (!reached(open_us)) sleepUntil(open_us);
  if (reached(close_us)) return r;
  clearIrq();
  radio.startReceive();
  bool irq = meter_ch < LORA_CHANNELS ? waitIrqSampling(close_us, meter_ch) : waitIrq(close_us);
  if (!irq) {
    const uint32_t flags = radio.getIrqFlags();
    if ((flags & 0x0010) && extend_us > 0) irq = waitIrq(close_us + extend_us);   // HeaderValid: packet in progress
  }
  if (irq) {
    r.irq = true;
    r.end_us = g_irq_us;
    const size_t n = radio.getPacketLength();
    if (n > 0 && n <= MAX_PACKET) {
      const int st = radio.readData(buf, n);
      if (st == RADIOLIB_ERR_NONE) {
        r.got = true; r.len = static_cast<uint8_t>(n);
        r.rssi = static_cast<int8_t>(radio.getRSSI());
        r.snr_q4 = static_cast<int8_t>(radio.getSNR() * 4.0f);
        g_rx_ok = g_rx_ok + 1;
        g_last_rx_ms = millis();
      } else if (st == RADIOLIB_ERR_CRC_MISMATCH) {
        r.crc = true;
        g_rx_crc = g_rx_crc + 1;
      }
    }
  }
  standbyXosc();
  return r;
}

// ---------------------------------------------------------------------------------------------
// Chalet
// ---------------------------------------------------------------------------------------------
static void cmdQueue(uint8_t cmd, uint8_t target, uint8_t value) {   // under lock
  if (cmd == CMD_SET_CHANNEL) {
    if (g_ch->cmd == CMD_SET_CHANNEL && g_ch->cmd_value == value) return;          // that switch is running
    for (uint8_t i = 0; i < g_cmdq_n; i++)
      if (g_cmdq[i].cmd == CMD_SET_CHANNEL) { g_cmdq[i].value = value; return; }   // newest choice wins
  }
  if (g_cmdq_n < sizeof(g_cmdq) / sizeof(g_cmdq[0])) { g_cmdq[g_cmdq_n].cmd = cmd; g_cmdq[g_cmdq_n].target = target; g_cmdq[g_cmdq_n].value = value; g_cmdq_n++; }
}

static void chaletApplyConfig() {   // under lock
  PlannerConfig& c = g_ch->planner.cfg;
  if (c.test_mode != g_cfg_test) { c.test_mode = g_cfg_test; g_ch->planner.resetStats(); }
  c.adaptive = g_cfg_adaptive;
  if (g_cfg_reset_stats) { g_ch->planner.resetStats(); g_cfg_reset_stats = false; }
  g_ch->focus_node = g_cfg_focus;
  if (g_cfg_reset_all) { g_cfg_reset_all = false; cmdQueue(CMD_RESET_ALL, 0, 0); }
  // one command at a time, each in CMD_BEACONS beacons (also sent on the backbone)
  if (g_ch->cmd != CMD_NONE && g_cmd_beacons_left == 0) g_ch->cmd = CMD_NONE;
  if (g_ch->cmd == CMD_NONE && g_cmdq_n > 0) {
    const QueuedCmd q = g_cmdq[0];
    for (uint8_t i = 1; i < g_cmdq_n; i++) g_cmdq[i - 1] = g_cmdq[i];
    g_cmdq_n--;
    g_cmd_seq++;
    if (g_cmd_seq == 0) g_cmd_seq = 1;   // a hub starts with last seq 0
    g_ch->cmd = q.cmd; g_ch->cmd_seq = g_cmd_seq; g_ch->cmd_target = q.target; g_ch->cmd_value = q.value;
    g_cmd_beacons_left = CMD_BEACONS;
  }
  if (g_ch->cmd == CMD_SET_CHANNEL) g_ch->cmd_target = static_cast<uint8_t>(g_cmd_beacons_left - 1);   // beacons left before the switch
  const bool silenced = g_silence_state && static_cast<int32_t>(g_silence_until_ms - millis()) > 0;
  if (g_silence_state && !silenced) { g_silence_state = false; g_silence_event = true; }
  g_ch->flags = static_cast<uint8_t>((silenced ? BF_SILENCED : 0) | (g_cfg_sonar_sim ? BF_SONAR_SIM : 0));
  const uint32_t rem_s = silenced ? (g_silence_until_ms - millis()) / 1000 : 0;
  g_ch->silence_10s = static_cast<uint8_t>(rem_s / 10 > 255 ? 255 : rem_s / 10);
  g_ch->net_cfg = makeNetCfg(g_cfg_transport, EB_LR, g_lora_ch);
}

static void chaletBeaconBuilt() {   // under lock, after startFrame
  if (g_ch->cmd == CMD_NONE || g_cmd_beacons_left == 0) return;
  g_cmd_beacons_left--;
  if (g_ch->cmd == CMD_SET_CHANNEL && g_cmd_beacons_left == 0) g_ch_switch_after = g_ch->cmd_value;   // after the target-0 beacon
}

static void notePath(uint8_t hub, uint8_t flags, bool eb, uint8_t hops) {   // under lock
  HubPath* p = nullptr;
  for (uint8_t i = 0; i < 12 && p == nullptr; i++) if (g_paths[i].id == hub) p = &g_paths[i];
  for (uint8_t i = 0; i < 12 && p == nullptr; i++) if (g_paths[i].id == 0) p = &g_paths[i];
  if (p == nullptr) {   // replace the hub heard longest ago
    p = &g_paths[0];
    for (uint8_t i = 1; i < 12; i++) {
      const uint32_t a = g_paths[i].lora_ms > g_paths[i].eb_ms ? g_paths[i].lora_ms : g_paths[i].eb_ms;
      const uint32_t b = p->lora_ms > p->eb_ms ? p->lora_ms : p->eb_ms;
      if (a < b) p = &g_paths[i];
    }
    memset(p, 0, sizeof(*p));
  }
  p->id = hub; p->flags = flags;
  if (eb) { p->eb_ms = millis(); p->hops = hops; } else { p->lora_ms = millis(); }
}

static void chaletHandle(const ChaletRxResult& res, bool eb = false, uint8_t hops = 0) {   // under lock
  if (res.hub != 0 && res.hub != ID_NONE) notePath(res.hub, res.flags, eb, hops);
  for (uint8_t k = 0; k < res.n_changes; k++) {
    MeshNodeUpdate u;
    u.node = res.changes[k].node; u.owner = res.changes[k].owner; u.old_state = res.changes[k].old_state;
    u.new_state = res.changes[k].new_state; u.turns = res.changes[k].turns; u.flags = res.changes[k].flags;
    xQueueSend(g_nodeq, &u, 0);
  }
  if (res.flags & HF_SILENCE_ON) {
    if (!g_silence_state) g_silence_event = true;
    g_silence_state = true; g_silence_until_ms = millis() + SILENCE_AUTO_CLEAR_MS;
  } else if (res.flags & HF_SILENCE_OFF) {
    if (g_silence_state) g_silence_event = true;
    g_silence_state = false;
  }
}

// Boot: fixed channel, or (Auto) the channel used last time — every device starts there, so the
// network comes up the same way however the devices are switched on. The scan only fills the
// activity meter; Auto moves wait for the end of the setup phase (chaletArmed).
static void chaletBootChannel() {
  if (g_cfg_channel < LORA_CHANNELS) { g_lora_ch = g_cfg_channel; tune(g_lora_ch); return; }
  tune(g_lora_ch);
  for (uint8_t c = 0; c < LORA_CHANNELS; c++) sampleChannel(c, nowUs() + BOOT_SCAN_US);
  busyPublish();
  tune(g_lora_ch);
}

// Between frames: sample one other channel (round robin), then listen on the network channel for
// random JOINs while sampling its activity. Never runs into the beacon (until = beacon - 4 ms).
static void chaletGap(uint32_t until, bool lora) {
  static uint8_t rx[MAX_PACKET];
  if (static_cast<int32_t>(until - nowUs()) > static_cast<int32_t>(SCAN_DWELL_US + 20000)) {
    g_scan_rr = static_cast<uint8_t>((g_scan_rr + 1) % LORA_CHANNELS);
    if (g_scan_rr == g_lora_ch) g_scan_rr = static_cast<uint8_t>((g_scan_rr + 1) % LORA_CHANNELS);
    sampleChannel(g_scan_rr, nowUs() + SCAN_DWELL_US);
  }
  while (static_cast<int32_t>(until - nowUs()) > 0) {
    if (lora) {
      RxRes r = rxWindow(MODE_SF9_BW500, nowUs(), until, rx, 0, g_lora_ch);   // never extend into the beacon
      if (r.got) { Lock l; g_ch->onAsyncPacket(rx, r.len, r.rssi); }
    } else {
      sampleChannel(g_lora_ch, until);
    }
  }
}

// Every CH_EVAL_MS (or on request): publish the activity, then queue a channel change if the fixed
// setting differs or (Auto) the channel got busy.
static void chaletChannelEval() {
  if (!g_cfg_rescan && millis() - g_ch_eval_ms < CH_EVAL_MS) return;
  g_cfg_rescan = false;
  g_ch_eval_ms = millis();
  busyPublish();
  int to = -1;
  if (g_cfg_channel < LORA_CHANNELS) { if (g_cfg_channel != g_lora_ch) to = g_cfg_channel; }
  else if (chaletArmed()) to = betterChannel(g_lora_ch);   // setup phase: never move by itself
  if (to >= 0) { Lock l; cmdQueue(CMD_SET_CHANNEL, 0, static_cast<uint8_t>(to)); }
}

static void chaletTask() {
  static uint8_t buf[MAX_PACKET];
  static uint8_t rx[MAX_PACKET];
  static uint8_t ebp[MAX_PACKET];
  static uint8_t ebf[eb::MAX_FRAME];
  uint16_t frame = 1;
  chaletBootChannel();
  g_ch_eval_ms = millis();
  uint32_t next = nowUs() + 200000;
  for (;;) {
    const bool lora = g_cfg_transport != TR_ESPNOW;   // ESP-NOW only: no LoRa TX, frames keep running for the backbone
    chaletGap(next - 4000, lora);
    Beacon plan;
    SlotTime times[MAX_SLOTS];
    size_t blen, eblen = 0;
    uint32_t frame_us;
    {
      Lock l;
      chaletApplyConfig();
      blen = g_ch->startFrame(frame, buf, sizeof(buf));
      chaletBeaconBuilt();
      plan = g_ch->beacon;
      memcpy(times, g_ch->times, sizeof(times));
      // the planner may lengthen the frame (1 / 1.5 / 2 s) when the hubs do not fit
      frame_us = plan.frame_10ms ? static_cast<uint32_t>(plan.frame_10ms) * 10000UL
                                 : static_cast<uint32_t>(g_ch->planner.cfg.frame_ms) * 1000UL;
      g_frame = frame;
      const uint32_t now = millis();
      // backbone beacon: Auto / ESP-NOW, 2 min after leaving them, and while hubs talk on the backbone
      const bool eb_beacon = g_eb_send != nullptr &&
                             (g_cfg_transport != TR_LORA || static_cast<int32_t>(g_eb_beacon_until_ms - now) > 0 ||
                              now - g_eb_hub_last_ms < 30000UL);
      if (eb_beacon) eblen = g_ch->buildEbBeacon(ebp, sizeof(ebp));
    }
    uint32_t end = 0, ref = next;
    if (lora) {
      if (txAt(buf, blen, MODE_SF9_BW500, next, end, false)) ref = refFromBeaconEnd(end, static_cast<uint16_t>(blen));
    } else {
      sleepUntil(next);
    }
    if (eblen > 0) {
      eb::Header h;
      h.net = g_net; h.sender = g_self; h.type = eb::MSG_EB_BEACON; h.hops = 0; h.origin = g_self; h.seq = ++g_eb_seq;
      const size_t n = eb::encode(h, ebp, eblen, ebf, sizeof(ebf));
      if (n > 0 && g_eb_send(ebf, n)) g_eb_tx = g_eb_tx + 1;
    }
    for (uint8_t i = 0; lora && i < plan.n_slots; i++) {
      const Slot& s = plan.slots[i];
      if (s.kind == SLOT_ECHO) continue;                         // our own beacon, repeated
      RxRes r = rxWindow(slotMode(s), ref + times[i].start, ref + times[i].end, rx);
      Lock l;
      if (r.got) {
        ChaletRxResult res;
        if (g_ch->onSlotPacket(i, rx, r.len, r.rssi, r.snr_q4, res)) chaletHandle(res);
      } else {
        g_ch->onSlotEmpty(i, r.crc);
      }
    }
    {
      Lock l;
      if (g_first_hub_ms == 0) {   // setup phase clock starts with the first hub in the network
        for (uint8_t i = 0; i < g_ch->planner.capacity(); i++) if (g_ch->planner.hubAt(i) != nullptr) { g_first_hub_ms = millis(); break; }
      }
      NodeChange ex[8];
      const uint8_t n = g_ch->expireNodes(ex, 8);
      for (uint8_t k = 0; k < n; k++) {
        MeshNodeUpdate u = {ex[k].node, ex[k].owner, ex[k].old_state, ex[k].new_state, ex[k].turns, ex[k].flags};
        xQueueSend(g_nodeq, &u, 0);
      }
    }
    if (g_ch_switch_after != 0xFF) {   // the target-0 beacon went out: the network moves now
      g_lora_ch = g_ch_switch_after; g_ch_switch_after = 0xFF;
      tune(g_lora_ch);
      g_ch_moves = g_ch_moves + 1; g_ch_event = true;
    }
    chaletChannelEval();
    frame++;
    next += frame_us;
    if (static_cast<int32_t>(next - nowUs()) < 5000) next = nowUs() + 20000;   // fell behind: restart timing
  }
}

// ---------------------------------------------------------------------------------------------
// Hub
// ---------------------------------------------------------------------------------------------
static void hubBeaconPost(uint8_t cmd) {   // under lock: events for loop
  if (cmd == CMD_RESET_ALL) {
    g_reset_event = true;
    g_rtc_cmd_magic = 0xC0DE5EED; g_rtc_cmd_seq = g_hub->last_cmd_seq;
  }
  if (cmd == CMD_SET_CHANNEL) g_pending_since_ms = millis();
  if (cmd == CMD_SET_RELAY) { g_relay_dev = g_hub->last_cmd_target; g_relay_on = g_hub->last_cmd_value != 0; g_relay_event = true; }
  const bool s = g_hub->silenced();   // beacon or backbone beacon
  if (s != g_silence_state) { g_silence_state = s; g_silence_event = true; }
}

// Handle a packet heard at any time. Returns true if it installed a new plan (beacon/echo).
static bool hubOnPacket(int slot, const uint8_t* p, const RxRes& r) {
  Lock l;
  Header h;
  if (!readHeader(p, r.len, g_net, h)) return false;
  g_heard_net_ms = millis();                      // our network is on this channel: stop hopping
  if (h.type == PT_BEACON || h.type == PT_ECHO) {
    uint8_t cmd = CMD_NONE;
    if (g_hub->onBeacon(p, r.len, r.end_us, r.rssi, r.snr_q4, cmd)) {
      g_pol.onLoraBeacon(millis());
      if (h.type == PT_BEACON) g_last_beacon_len = r.len;
      g_frame = g_hub->plan.frame;
      hubBeaconPost(cmd);
      return true;
    }
    return false;
  }
  if (slot >= 0) g_hub->onSlotPacket(static_cast<uint8_t>(slot), p, r.len, r.rssi, r.snr_q4);
  else g_hub->onAsyncPacket(p, r.len, r.rssi, r.snr_q4);
  return false;
}

// Listen in SF9/500 between open and close; other packets (joins...) don't end the window.
// Returns true when a beacon/echo installed a new plan.
static bool listenFor(uint32_t open_us, uint32_t close_us) {
  static uint8_t rx[MAX_PACKET];
  while (!reached(close_us)) {
    RxRes r = rxWindow(MODE_SF9_BW500, open_us, close_us, rx);
    if (!r.irq) return false;                     // window closed with nothing (more)
    if (r.got && hubOnPacket(-1, rx, r)) return true;
    open_us = nowUs();
  }
  return false;
}

// Radio task: follow a channel switch announced by the chalet (after the target-0 beacon), or, while
// unsynced, the channel named by the backbone beacon.
static void hubChannelUpdate(bool synced) {
  uint8_t to = 0xFF;
  {
    Lock l;
    if (g_hub->channel_switch_pending) {
      g_req_ch = 0xFF;   // the backbone beacon still names the old channel during the countdown
      const bool due = seqDiff(static_cast<uint16_t>(g_hub->frameNow() + 1), g_hub->channel_switch_frame) >= 0;
      if (due || (!synced && millis() - g_pending_since_ms > 15000UL)) {
        to = g_hub->channel_next; g_hub->channel_switch_pending = false;
      }
    } else if (!synced && g_req_ch != 0xFF) {
      to = g_req_ch; g_req_ch = 0xFF;
    }
  }
  if (to < LORA_CHANNELS && to != g_lora_ch) {
    g_lora_ch = to; tune(to); g_ch_event = true;
    g_heard_net_ms = millis();
  }
}

static void hubSearch(uint32_t& next_join_us) {
  static uint8_t rx[MAX_PACKET];
  // no beacon: next channel every HUB_HOP_MS, unless this network was heard recently
  const uint32_t now = millis();
  if (now - g_heard_net_ms > HUB_HOP_QUIET_MS && now - g_last_hop_ms > HUB_HOP_MS) {
    g_lora_ch = static_cast<uint8_t>((g_lora_ch + 1) % LORA_CHANNELS);
    tune(g_lora_ch);
    g_last_hop_ms = now;
  }
  const uint32_t close = nowUs() + 300000;
  RxRes r = rxWindow(MODE_SF9_BW500, nowUs(), close, rx);
  if (r.got && hubOnPacket(-1, rx, r)) return;
  if (reached(next_join_us)) {
    uint8_t jb[MAX_PACKET];
    size_t n;
    { Lock l; n = g_hub->buildJoin(jb, sizeof(jb)); }
    uint32_t end;
    txAt(jb, n, MODE_SF9_BW500, nowUs(), end, false);
    next_join_us = nowUs() + 1500000 + (esp_random() % 3000000);   // ALOHA: 1.5-4.5 s (est.)
  }
}

// Runs the slots of the current plan that are still ahead. Returns when the frame is done or a
// new plan was installed (echo received mid-frame).
static void hubFrame() {
  static uint8_t buf[MAX_PACKET];
  static uint8_t rx[MAX_PACKET];
  Beacon plan; SlotTime times[MAX_SLOTS]; uint32_t ref; uint32_t frame_us;
  {
    Lock l;
    plan = g_hub->plan; memcpy(times, g_hub->times, sizeof(times));
    ref = g_hub->sync.ref(); frame_us = g_hub->sync.frameUs();
  }
  for (uint8_t i = 0; i < plan.n_slots; i++) {
    if (static_cast<int32_t>(ref + times[i].end - nowUs()) < 0) continue;     // already past
    const Slot& s = plan.slots[i];
    SlotAction act;
    { Lock l; act = g_hub->action(i, (esp_random() % 3) == 0); }
    uint32_t end;
    switch (act) {
      case ACT_TX_HUB: {
        size_t n;
        {
          Lock l;
          uint8_t flags = static_cast<uint8_t>((g_eb_relay ? HF_EB_RELAY : 0) | (g_hub_eb_on ? HF_EB_PATH : 0));
          if (g_silence_req_frames > 0) { flags |= g_silence_req; g_silence_req_frames = g_silence_req_frames - 1; }
          const bool test = plan.test_mode != TEST_OFF;
          n = g_hub->buildHub(buf, sizeof(buf), flags, g_battery_mv, static_cast<uint16_t>(millis() / 60000UL),
                              test, static_cast<uint16_t>(g_test_counter));
          g_test_counter++;
        }
        txAt(buf, n, slotMode(s), ref + times[i].tx, end);
        break;
      }
      case ACT_TX_ECHO: {
        size_t n;
        { Lock l; n = g_hub->buildEcho(buf, sizeof(buf)); }
        txAt(buf, n, MODE_SF9_BW500, ref + times[i].tx, end);
        break;
      }
      case ACT_TX_JOIN: {
        size_t n;
        { Lock l; n = g_hub->buildJoin(buf, sizeof(buf)); }
        txAt(buf, n, MODE_SF9_BW500, ref + times[i].tx + (esp_random() % 2000), end);
        break;
      }
      case ACT_RX: {
        RxRes r = rxWindow(slotMode(s), ref + times[i].start, ref + times[i].end, rx);
        if (r.got && hubOnPacket(i, rx, r)) return;     // echo gave a new plan: restart with it
        break;
      }
      default:
        break;
    }
  }
  { Lock l; g_hub->endFrame(); }
  hubChannelUpdate(true);

  // next beacon: listen around its expected start (REF), then the echo slots of the old plan
  // (echo positions shift with the new beacon length, hence the wider windows)
  const uint32_t ref_next = ref + frame_us;
  const uint32_t bair = beaconAirtimeUs(g_last_beacon_len);
  if (listenFor(ref_next - BEACON_WINDOW_US, ref_next + bair + BEACON_WINDOW_US)) return;
  for (uint8_t i = 0; i < plan.n_slots && plan.slots[i].kind == SLOT_ECHO; i++) {
    if (plan.slots[i].owner == g_hub->self) continue;
    if (listenFor(ref_next + times[i].start - 10000, ref_next + times[i].end + BEACON_WINDOW_US)) return;
  }
  Lock l;
  g_hub->onBeaconMissed();
  g_frame = g_hub->sync.frame();
}

static void hubTask() {
  uint32_t next_join = nowUs() + 500000 + (esp_random() % 1000000);
  tune(g_lora_ch);
  for (;;) {
    bool synced, lora_on;
    {
      Lock l;
      const uint32_t now = millis();
      g_pol.tick(now);
      lora_on = g_pol.useLora(g_hub->transport(), now);
      if (!lora_on) g_hub->have_plan = false;   // slot timing is stale when LoRa comes back
      synced = g_hub->have_plan && g_hub->sync.synced();
    }
    g_hub_lora_on = lora_on;
    if (!lora_on) {   // ESP-NOW only: radio idle, keep following channel changes for the way back
      if (!g_lora_idle) { standbyXosc(); g_lora_idle = true; }
      hubChannelUpdate(false);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    g_lora_idle = false;
    hubChannelUpdate(synced);
    if (!synced) hubSearch(next_join);
    else hubFrame();
  }
}

// ---------------------------------------------------------------------------------------------
// Task + public API
// ---------------------------------------------------------------------------------------------
static void radioTask(void*) {
  if (g_chalet) chaletTask();
  else hubTask();
}

static bool radioInit() {
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  const ModeInfo& mi = modeInfo(MODE_SF9_BW500);
  int st = radio.begin(LORA_FREQUENCY, static_cast<float>(mi.bw_khz), mi.sf, LORA_CODING_RATE,
                       LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) return false;
  radio.setCRC(2);                       // hardware CRC on (explicit, protocol relies on it)
  radio.setDio1Action(onDio1);
  g_cur_mode = MODE_SF9_BW500;
  return true;
}

bool meshBegin(uint8_t self_id, bool chalet, uint8_t network_id) {
  g_chalet = chalet;
  g_net = network_id;
  g_self = self_id;
  g_mx = xSemaphoreCreateMutex();
  g_dedup = new eb::Dedup();
  g_eb_seq = static_cast<uint16_t>(esp_random());   // a rebooted device must not look like a duplicate
  g_pol.begin(millis());
  g_cmd_seq = static_cast<uint8_t>(esp_random());   // a rebooted chalet must not reuse the last command seq
  g_nodeq = xQueueCreate(32, sizeof(MeshNodeUpdate));
  if (chalet) {
    g_ch = new ChaletRole<10, 48>();
    g_sonar = new sonar::SonarStore<16, 128>();
    g_ch->sonar_sink = g_sonar;
    PlannerConfig& c = g_ch->planner.cfg;
    c.network_id = network_id; c.self_id = self_id; c.frame_ms = 1000;
    c.allowance = 96; c.test_allowance = 160;          // est. budgets, see docs/protocol_v2.md
    c.test_mode = g_cfg_test; c.adaptive = g_cfg_adaptive;
  } else {
    g_hub = new HubRole<24>();
    g_hub->network_id = network_id;
    g_hub->self = self_id;
    if (g_rtc_cmd_magic == 0xC0DE5EED) g_hub->last_cmd_seq = g_rtc_cmd_seq;
  }
  g_radio_ok = radioInit();
  g_tuned_ch = 0;   // radioInit tunes LORA_FREQUENCY = channel 0; the task retunes to g_lora_ch
  // Core 1 (same as loop), higher priority than loop: the radio preempts loop() for slot timing.
  xTaskCreatePinnedToCore(radioTask, "mesh", 8192, nullptr, 5, &g_task, 1);
  return g_radio_ok;
}

void meshHubObserveNode(uint8_t node, uint8_t state, uint8_t turns, uint8_t flags, uint8_t battery_pct) {
  if (g_hub == nullptr) return;
  Lock l;
  if (g_hub->table.observe(node, state, turns, flags, battery_pct, millis())) g_eb_kick = true;
  g_hub->table.expire(millis(), 10UL * 60UL * 1000UL);   // forget nodes silent for 10 min once acked (est.)
}

void meshHubSetBattery(uint16_t mv) { g_battery_mv = mv; }

void meshHubView(MeshHubView& v) {
  memset(&v, 0, sizeof(v));
  if (g_hub == nullptr) return;
  Lock l;
  v.synced = g_hub->have_plan && g_hub->sync.synced();
  v.from_echo = g_hub->plan_from_echo;
  v.frame = g_hub->sync.frame();
  v.beacon_rssi = g_hub->beacon_rssi;
  v.beacon_snr = g_hub->beacon_snr_q4 / 4.0f;
  v.lost64 = g_hub->sync.lostLast64();
  v.sync_err_us = g_hub->sync.lastErrUs();
  const int idx = g_hub->ownSlotIndex();
  v.own_slot_mode = idx >= 0 ? g_hub->plan.slots[idx].mode : -1;
  v.allowance = idx >= 0 ? g_hub->plan.slots[idx].allowance : 0;
  v.test_mode = g_hub->plan.test_mode;
  v.beacons = g_hub->beacons_rx; v.echoes = g_hub->echoes_rx; v.tx = g_tx;
  const uint32_t now = millis();
  v.lora_ch = g_lora_ch;
  v.transport = g_hub->transport();
  v.lora_on = g_hub_lora_on; v.eb_on = g_hub_eb_on;
  v.eb_fallback = g_pol.fallbackActive();
  v.eb_beacon_age_ms = g_eb_beacon_seen ? now - g_eb_beacon_ms : 0xFFFFFFFFUL;
  v.relay = g_eb_relay;
}

void meshRequestSilence(bool on) {
  if (g_chalet) {
    if (g_mx == nullptr) return;
    Lock l;
    g_silence_state = on;
    g_silence_until_ms = millis() + SILENCE_AUTO_CLEAR_MS;
  } else {
    g_silence_req = on ? HF_SILENCE_ON : HF_SILENCE_OFF;
    g_silence_req_frames = 3;            // repeated in 3 hub packets (no ack for this request)
  }
}

bool meshPollSilence(bool& on) {
  if (!g_silence_event) return false;
  g_silence_event = false;
  on = g_silence_state;
  return true;
}

bool meshPollReset() {
  if (!g_reset_event) return false;
  g_reset_event = false;
  return true;
}

void meshCounters(MeshCounters& c) {
  c.radio_ok = g_radio_ok; c.rx_ok = g_rx_ok; c.rx_crc = g_rx_crc; c.tx = g_tx;
  c.last_rx_ms = g_last_rx_ms; c.frame = g_frame;
}

uint8_t meshTestMode() {
  if (g_chalet) return g_cfg_test;
  if (g_hub == nullptr) return 0;
  Lock l;
  return g_hub->have_plan ? g_hub->plan.test_mode : 0;
}

bool meshPollNodeUpdate(MeshNodeUpdate& u) {
  return g_nodeq != nullptr && xQueueReceive(g_nodeq, &u, 0) == pdTRUE;
}

uint8_t meshNodeSnapshot(MeshNodeSnapshot* out, uint8_t max) {
  if (g_ch == nullptr) return 0;
  Lock l;
  uint8_t n = 0;
  for (uint8_t i = 0; i < g_ch->nodes.capacity() && n < max; i++) {
    const NodeView* v = g_ch->nodes.at(i);
    if (v == nullptr) continue;
    out[n].node = v->node; out[n].owner = v->owner; out[n].state = v->state;
    out[n].turns = v->turns; out[n].flags = v->flags; out[n].battery = v->battery;
    n++;
  }
  return n;
}

void meshSetTestMode(uint8_t mode) { if (mode < TEST_MODE_COUNT) g_cfg_test = mode; }
void meshSetAdaptive(bool on) { g_cfg_adaptive = on; }
bool meshAdaptive() { return g_cfg_adaptive; }
void meshResetStats() { g_cfg_reset_stats = true; }
void meshSendResetAll() { g_cfg_reset_all = true; }
void meshSetFocusNode(uint8_t node) { g_cfg_focus = node; }

// ---------------------------------------------------------------------------------------------
// v2 network config + ESP-NOW backbone (docs/protocol_v2.md §6c)
// ---------------------------------------------------------------------------------------------
void meshSetStartChannel(uint8_t ch) { if (ch < LORA_CHANNELS) g_lora_ch = ch; }

void meshSetTransport(uint8_t tr) {
  if (tr > TR_ESPNOW) return;
  if (tr != g_cfg_transport) g_eb_beacon_until_ms = millis() + 120000UL;   // hubs on the backbone hear the change
  g_cfg_transport = tr;
}

uint8_t meshTransport() {
  if (g_chalet) return g_cfg_transport;
  if (g_hub == nullptr || g_mx == nullptr) return TR_AUTO;
  Lock l;
  return g_hub->transport();
}

void meshSetLoraChannel(uint8_t ch) {
  if (ch >= LORA_CHANNELS && ch != MESH_CH_AUTO) return;
  g_cfg_channel = ch;
  if (g_ch != nullptr) g_cfg_rescan = true;   // radio running: fixed channel applied / Auto re-evaluated now
  else if (ch < LORA_CHANNELS) g_lora_ch = ch;
}
uint8_t meshLoraChannelSetting() { return g_cfg_channel; }
uint8_t meshLoraChannel() { return g_lora_ch; }
float meshLoraChannelMHz(uint8_t ch) { return loraChannelMHz(ch); }
void meshRescanChannels() { g_cfg_rescan = true; }
void meshChannelBusy(uint8_t* pct8) { for (uint8_t c = 0; c < LORA_CHANNELS; c++) pct8[c] = g_busy_pct[c]; }

uint32_t meshSetupArmInS() {
  if (g_mx == nullptr) return 0;
  if (g_chalet) return chaletArmInS();
  Lock l;
  return g_pol.armInS(millis());
}

bool meshPollChannelChanged(uint8_t& ch) {
  if (!g_ch_event) return false;
  g_ch_event = false;
  ch = g_lora_ch;
  return true;
}

void meshSetEbSender(MeshEbSendFn fn) { g_eb_send = fn; }
void meshSetEbRelay(bool on) { g_eb_relay = on; }
bool meshEbRelay() { return g_eb_relay; }

void meshSetDeviceRelay(uint8_t dev, bool on) {
  if (g_ch == nullptr || dev == 0 || dev == ID_NONE) return;
  Lock l;
  cmdQueue(CMD_SET_RELAY, dev, on ? 1 : 0);
}

bool meshPollRelayCmd(uint8_t& dev, bool& on) {
  if (!g_relay_event) return false;
  g_relay_event = false;
  dev = g_relay_dev; on = g_relay_on;
  return true;
}

// One backbone frame (ESP-NOW type 0x70 / 0x71) from loop(): drop copies, relay, then use it.
void meshEbReceive(const uint8_t* frame, size_t len) {
  if (g_mx == nullptr || g_dedup == nullptr) return;
  eb::Header h;
  const uint8_t* pl;
  size_t n;
  if (!eb::decode(frame, len, g_net, h, pl, n) || h.origin == g_self) return;
  {
    Lock l;
    if (g_dedup->seen(h.origin, h.type, h.seq)) { g_eb_dup = g_eb_dup + 1; return; }
  }
  g_eb_rx = g_eb_rx + 1;
  if (g_eb_relay && g_eb_send != nullptr) {   // relay first: latency adds up per hop
    static uint8_t copy[eb::MAX_FRAME];
    memcpy(copy, frame, len);
    if (eb::prepareRelay(copy, len, g_self) && g_eb_send(copy, len)) g_eb_relayed = g_eb_relayed + 1;
  }
  if (g_chalet) {
    if (h.type != eb::MSG_EB_HUB || g_ch == nullptr) return;
    Lock l;
    ChaletRxResult res;
    g_eb_hub_last_ms = millis();
    if (g_ch->onEbHubPacket(pl, n, h.hops, res)) chaletHandle(res, true, h.hops);
    else if (res.hub != 0) notePath(res.hub, res.flags, true, h.hops);
    return;
  }
  if (h.type != eb::MSG_EB_BEACON || g_hub == nullptr) return;
  Lock l;
  uint8_t cmd = CMD_NONE;
  if (!g_hub->onEbBeacon(pl, n, cmd)) return;
  const uint32_t now = millis();
  g_pol.onEbBeacon(now);
  g_eb_beacon_ms = now; g_eb_beacon_seen = true;
  if (!(g_hub->have_plan && g_hub->sync.synced())) {   // no LoRa beacon: the backbone says which channel
    const uint8_t c = netChannel(g_hub->info_net_cfg);
    if (c < LORA_CHANNELS && c != g_lora_ch && !g_hub->channel_switch_pending) g_req_ch = c;
    g_heard_net_ms = now;
  }
  hubBeaconPost(cmd);
}

// Hub, from loop(): send the hub packet on the backbone when the transport policy says so — every
// second, sooner (250 ms) after a line change.
void meshEbTick() {
  if (g_hub == nullptr || g_mx == nullptr) return;
  static uint8_t pl[eb::MAX_PAYLOAD];
  static uint8_t f[eb::MAX_FRAME];
  size_t n = 0;
  {
    Lock l;
    const uint32_t now = millis();
    g_pol.tick(now);
    const bool use = g_pol.useEb(g_hub->transport(), now);
    g_hub_eb_on = use;
    if (!use || g_eb_send == nullptr) return;
    const uint32_t gap = now - g_eb_last_ms;
    if (gap < 1000UL && !(g_eb_kick && gap >= 250UL)) return;
    g_eb_kick = false;
    g_eb_last_ms = now;
    uint8_t flags = static_cast<uint8_t>((g_eb_relay ? HF_EB_RELAY : 0) | HF_EB_PATH);
    if (g_silence_req_frames > 0) { flags |= g_silence_req; g_silence_req_frames = g_silence_req_frames - 1; }
    n = g_hub->buildHubFree(pl, sizeof(pl), flags, g_battery_mv, static_cast<uint16_t>(now / 60000UL), g_hub->frameNow());
  }
  if (n == 0) return;
  eb::Header h;
  h.net = g_net; h.sender = g_self; h.type = eb::MSG_EB_HUB; h.hops = 0; h.origin = g_self; h.seq = ++g_eb_seq;
  const size_t fl = eb::encode(h, pl, n, f, sizeof(f));
  if (fl > 0 && g_eb_send(f, fl)) g_eb_tx = g_eb_tx + 1;
}

const char* meshModeName(uint8_t mode) { return validMode(mode) ? modeInfo(static_cast<RadioMode>(mode)).name : "-"; }
const char* meshTestModeName(uint8_t m) {
  switch (m) {
    case TEST_OFF: return "off"; case TEST_ROTATE: return "rotate"; case TEST_FIX_SF9: return "SF9/500";
    case TEST_FIX_SF8: return "SF8/500"; case TEST_FIX_SF7: return "SF7/500"; default: return "?";
  }
}

String meshRadioJson() {
  String s;
  s.reserve(4500);
  s += "{\"role\":\""; s += g_chalet ? "chalet" : "hub"; s += "\"";
  s += ",\"radio_ok\":"; s += g_radio_ok ? "true" : "false";
  s += ",\"frame\":"; s += g_frame;
  s += ",\"rx_ok\":"; s += g_rx_ok; s += ",\"rx_crc\":"; s += g_rx_crc; s += ",\"tx\":"; s += g_tx;
  s += ",\"test_mode\":\""; s += meshTestModeName(meshTestMode()); s += "\"";
  s += ",\"adaptive\":"; s += g_cfg_adaptive ? "true" : "false";
  // network config + backbone
  static const char* const TR_NAMES[] = {"auto", "lora", "espnow"};
  const uint8_t tr = meshTransport();
  s += ",\"transport\":\""; s += TR_NAMES[tr <= TR_ESPNOW ? tr : 0]; s += "\"";
  {
    uint32_t arm_s;
    if (g_chalet) arm_s = chaletArmInS();
    else { Lock l; arm_s = g_pol.armInS(millis()); }
    s += ",\"setup\":"; s += arm_s ? "true" : "false"; s += ",\"arm_in_s\":"; s += arm_s;
  }
  s += ",\"channel\":"; s += g_lora_ch;
  s += ",\"channel_mhz\":"; s += String(loraChannelMHz(g_lora_ch), 1);
  s += ",\"eb\":{\"lr\":"; s += EB_LR ? "true" : "false";
  s += ",\"relay\":"; s += g_eb_relay ? "true" : "false";
  s += ",\"rx\":"; s += g_eb_rx; s += ",\"tx\":"; s += g_eb_tx;
  s += ",\"relayed\":"; s += g_eb_relayed; s += ",\"dup\":"; s += g_eb_dup; s += "}";
  if (g_ch != nullptr) {
    s += ",\"channel_setting\":"; s += g_cfg_channel == MESH_CH_AUTO ? String("\"auto\"") : String(g_cfg_channel);
    s += ",\"channel_moves\":"; s += g_ch_moves;
    s += ",\"channels\":[";
    for (uint8_t c = 0; c < LORA_CHANNELS; c++) {
      if (c) s += ",";
      s += "{\"ch\":"; s += c; s += ",\"mhz\":"; s += String(loraChannelMHz(c), 1);
      s += ",\"busy\":"; s += g_busy_pct[c] == 255 ? String("null") : String(g_busy_pct[c]); s += "}";
    }
    s += "]";
    // how each hub was last heard (LoRa slot / backbone), ages in s (-1 = never)
    HubPath paths[12];
    { Lock l; memcpy(paths, g_paths, sizeof(paths)); }
    const uint32_t now = millis();
    s += ",\"paths\":[";
    bool fp = true;
    for (uint8_t i = 0; i < 12; i++) {
      const HubPath& p = paths[i];
      if (p.id == 0) continue;
      if (!fp) s += ",";
      fp = false;
      s += "{\"id\":"; s += p.id;
      s += ",\"lora_age\":"; s += p.lora_ms ? static_cast<long>((now - p.lora_ms) / 1000) : -1L;
      s += ",\"eb_age\":"; s += p.eb_ms ? static_cast<long>((now - p.eb_ms) / 1000) : -1L;
      s += ",\"eb_hops\":"; s += p.hops;
      s += ",\"relay\":"; s += (p.flags & HF_EB_RELAY) ? "true" : "false";
      s += ",\"eb_path\":"; s += (p.flags & HF_EB_PATH) ? "true" : "false";
      s += "}";
    }
    s += "]";
  }
  if (g_ch != nullptr) {
    // copy under the lock, format outside it (keeps the radio task's timing unaffected)
    static HubInfo hubs[10];
    uint8_t nh = 0;
    uint8_t allowance; uint32_t dropped, bad, test_bytes;
    {
      Lock l;
      allowance = g_ch->planner.lastAllowance(); dropped = g_ch->planner.droppedSlots();
      bad = g_ch->bad_packets; test_bytes = g_ch->test_bytes_rx;
      for (uint8_t i = 0; i < g_ch->planner.capacity() && nh < 10; i++) {
        const HubInfo* h = g_ch->planner.hubAt(i);
        if (h != nullptr) hubs[nh++] = *h;
      }
    }
    s += ",\"allowance\":"; s += allowance;
    s += ",\"dropped_slots\":"; s += dropped;
    s += ",\"bad_packets\":"; s += bad;
    s += ",\"test_bytes\":"; s += test_bytes;
    s += ",\"hubs\":[";
    bool first = true;
    for (uint8_t i = 0; i < nh; i++) {
      const HubInfo* h = &hubs[i];
      if (!first) s += ",";
      first = false;
      s += "{\"id\":"; s += h->id; s += ",\"via\":"; s += h->via;
      s += ",\"mode\":\""; s += meshModeName(h->mode); s += "\"";
      s += ",\"rssi\":"; s += h->last_rssi; s += ",\"snr\":"; s += h->last_snr_q4 / 4.0f;
      s += ",\"modes\":[";
      for (uint8_t m = 0; m < MODE_COUNT; m++) {
        const ModeStats& st = h->stats[m];
        if (m) s += ",";
        s += "{\"mode\":\""; s += meshModeName(m); s += "\"";
        s += ",\"sched\":"; s += st.scheduled; s += ",\"rx\":"; s += st.received; s += ",\"crc\":"; s += st.crc_err;
        if (st.received) {
          s += ",\"rssi_avg\":"; s += static_cast<float>(st.rssi_sum) / st.received;
          s += ",\"rssi_min\":"; s += st.rssi_min; s += ",\"rssi_max\":"; s += st.rssi_max;
          s += ",\"snr_avg\":"; s += static_cast<float>(st.snr_q4_sum) / st.received / 4.0f;
          s += ",\"snr_min\":"; s += st.snr_min_q4 / 4.0f;
        }
        s += "}";
      }
      s += "]";
      if (h->has_health) {
        const Health& hl = h->health;
        s += ",\"health\":{\"battery_mv\":"; s += hl.battery_mv;
        s += ",\"uptime_min\":"; s += hl.uptime_min;
        s += ",\"beacon_rssi\":"; s += hl.beacon_rssi; s += ",\"beacon_snr\":"; s += hl.beacon_snr_q4 / 4.0f;
        s += ",\"beacon_lost64\":"; s += hl.beacon_lost; s += ",\"sync_err_us\":"; s += static_cast<int32_t>(hl.sync_err_10us) * 10;
        s += ",\"neighbors\":[";
        for (uint8_t k = 0; k < hl.n_nb; k++) {
          if (k) s += ",";
          s += "{\"hub\":"; s += hl.nb[k].hub; s += ",\"rssi\":"; s += hl.nb[k].rssi; s += "}";
        }
        s += "]}";
      }
      s += "}";
    }
    s += "]";
  }
  if (g_hub != nullptr) {
    MeshHubView v;
    meshHubView(v);
    s += ",\"synced\":"; s += v.synced ? "true" : "false";
    s += ",\"via_echo\":"; s += v.from_echo ? "true" : "false";
    s += ",\"beacon_rssi\":"; s += v.beacon_rssi; s += ",\"beacon_snr\":"; s += v.beacon_snr;
    s += ",\"lost64\":"; s += v.lost64; s += ",\"sync_err_us\":"; s += v.sync_err_us;
    s += ",\"slot_mode\":\""; s += v.own_slot_mode >= 0 ? meshModeName(static_cast<uint8_t>(v.own_slot_mode)) : "none"; s += "\"";
    s += ",\"allowance\":"; s += v.allowance;
    s += ",\"lora_on\":"; s += v.lora_on ? "true" : "false";
    s += ",\"eb_on\":"; s += v.eb_on ? "true" : "false";
    s += ",\"eb_fallback\":"; s += v.eb_fallback ? "true" : "false";
    s += ",\"eb_beacon_age\":"; s += v.eb_beacon_age_ms == 0xFFFFFFFFUL ? -1L : static_cast<long>(v.eb_beacon_age_ms / 1000);
  }
  s += "}";
  return s;
}

void meshPrintStatus(Print& out) {
  MeshCounters c;
  meshCounters(c);
  out.printf("[mesh] %s radio=%s frame=%u rx=%lu crc=%lu tx=%lu test=%s\n", g_chalet ? "chalet" : "hub",
             c.radio_ok ? "ok" : "ERR", c.frame, (unsigned long)c.rx_ok, (unsigned long)c.rx_crc,
             (unsigned long)c.tx, meshTestModeName(meshTestMode()));
  if (g_hub != nullptr) {
    MeshHubView v;
    meshHubView(v);
    out.printf("[mesh] synced=%d echo=%d beacon %d dBm SNR %.1f lost64=%u syncErr=%ld us slot=%s allow=%u\n",
               v.synced, v.from_echo, v.beacon_rssi, v.beacon_snr, v.lost64, (long)v.sync_err_us,
               v.own_slot_mode >= 0 ? meshModeName(static_cast<uint8_t>(v.own_slot_mode)) : "none", v.allowance);
  }
  if (g_ch != nullptr) {
    // copy under the lock, print after: Serial blocks (~1 ms per 10 chars) and must never
    // delay the radio task's beacon or slots
    static HubInfo hubs[10];
    uint8_t nh = 0;
    {
      Lock l;
      for (uint8_t i = 0; i < g_ch->planner.capacity() && nh < 10; i++) {
        const HubInfo* h = g_ch->planner.hubAt(i);
        if (h != nullptr) hubs[nh++] = *h;
      }
    }
    for (uint8_t i = 0; i < nh; i++) {
      const HubInfo* h = &hubs[i];
      out.printf("[mesh] hub %u via %u mode %s last %d dBm SNR %.1f |", h->id, h->via, meshModeName(h->mode),
                 h->last_rssi, h->last_snr_q4 / 4.0f);
      for (uint8_t m = 0; m < MODE_COUNT; m++) {
        const ModeStats& st = h->stats[m];
        out.printf(" %s %lu/%lu", meshModeName(m), (unsigned long)st.received, (unsigned long)st.scheduled);
        if (st.received) out.printf(" (%.0f dBm, SNR %.1f)", (float)st.rssi_sum / st.received, (float)st.snr_q4_sum / st.received / 4.0f);
      }
      out.println();
    }
  }
  out.printf("[mesh] late TX skipped: %lu\n", (unsigned long)g_tx_late_skips);
  static const char* const TR_NAMES[] = {"Auto", "LoRa only", "ESP-NOW only"};
  const uint8_t tr = meshTransport();
  {
    uint32_t arm_s;
    if (g_chalet) arm_s = chaletArmInS();
    else { Lock l; arm_s = g_pol.armInS(millis()); }
    if (arm_s) out.printf("[net] SETUP PHASE: automatic fallbacks arm in %lu s (network complete and stable for 5 min)\n", (unsigned long)arm_s);
    else out.println(F("[net] running: automatic fallbacks armed"));
  }
  out.printf("[net] transport %s | LoRa ch %u (%.1f MHz)%s | backbone rx %lu tx %lu relayed %lu dup %lu | relay %s\n",
             TR_NAMES[tr <= TR_ESPNOW ? tr : 0], g_lora_ch, loraChannelMHz(g_lora_ch),
             g_chalet ? (g_cfg_channel == MESH_CH_AUTO ? " auto" : " fixed") : "",
             (unsigned long)g_eb_rx, (unsigned long)g_eb_tx, (unsigned long)g_eb_relayed, (unsigned long)g_eb_dup,
             g_eb_relay ? "ON" : "off");
  if (g_hub != nullptr) {
    MeshHubView v;
    meshHubView(v);
    out.printf("[net] hub sends on: %s%s%s | backbone beacon %s\n", v.lora_on ? "LoRa " : "", v.eb_on ? "ESP-NOW " : "",
               v.eb_fallback ? "(Auto fallback)" : "",
               v.eb_beacon_age_ms == 0xFFFFFFFFUL ? "never heard" : (String(v.eb_beacon_age_ms / 1000) + " s ago").c_str());
  }
  if (g_ch != nullptr) {
    out.print("[net] activity %:");
    for (uint8_t c = 0; c < LORA_CHANNELS; c++) {
      if (g_busy_pct[c] == 255) out.printf(" %.0f:-", loraChannelMHz(c));
      else out.printf(" %.0f:%u", loraChannelMHz(c), g_busy_pct[c]);
    }
    out.printf(" | moves %u\n", g_ch_moves);
  }
}

uint8_t meshHubSummaries(MeshHubSummary* out, uint8_t max) {
  if (g_ch == nullptr) return 0;
  Lock l;
  uint8_t n = 0;
  for (uint8_t i = 0; i < g_ch->planner.capacity() && n < max; i++) {
    const HubInfo* h = g_ch->planner.hubAt(i);
    if (h == nullptr) continue;
    MeshHubSummary& o = out[n++];
    o.id = h->id; o.via = h->via; o.mode = h->mode; o.rssi = h->last_rssi; o.snr = h->last_snr_q4 / 4.0f;
    o.rx = 0; o.sched = 0;
    for (uint8_t m = 0; m < MODE_COUNT; m++) { o.rx += h->stats[m].received; o.sched += h->stats[m].scheduled; }
  }
  return n;
}

// ---------------------------------------------------------------------------------------------
// Sonar (test mode + FOCUS)
// ---------------------------------------------------------------------------------------------
uint8_t meshSonarVirtualId(uint8_t hub_id, uint8_t k) { return static_cast<uint8_t>(128 + (hub_id & 0x0F) * 8 + (k & 7)); }
void meshSetSonarSim(bool on) { g_cfg_sonar_sim = on; }

bool meshSonarSim() {
  if (g_chalet) return g_cfg_sonar_sim;
  if (g_hub == nullptr) return false;
  Lock l;
  return g_hub->sonarSim();
}

uint8_t meshFocusNode() {
  if (g_chalet) return g_cfg_focus;
  if (g_hub == nullptr) return 0;
  Lock l;
  return g_hub->focusNode();
}

bool meshHubPushSonar(const uint8_t* blk, uint8_t len) {
  if (g_hub == nullptr) return false;
  Lock l;
  return g_hub->sonar.push(blk, len, g_hub->frameNow());   // frame from the beacon or the backbone beacon
}

String meshSonarListJson() {
  String s;
  s.reserve(1600);
  s = "{\"sim\":"; s += g_cfg_sonar_sim ? "true" : "false";
  s += ",\"focus\":"; s += g_cfg_focus;
  s += ",\"frame\":"; s += g_frame;
  s += ",\"nodes\":[";
  if (g_sonar != nullptr) {
    static sonar::NodeSonar copy[16];
    uint8_t n = 0;
    uint32_t ok = 0, bad = 0;
    {
      Lock l;
      for (uint8_t i = 0; i < g_sonar->capacity() && n < 16; i++) {
        const sonar::NodeSonar* ns = g_sonar->at(i);
        if (ns != nullptr) copy[n++] = *ns;
      }
      ok = g_sonar->blocks_ok; bad = g_sonar->blocks_bad;
    }
    for (uint8_t i = 0; i < n; i++) {
      const sonar::NodeSonar& ns = copy[i];
      char b[200];
      snprintf(b, sizeof(b),
               "%s{\"node\":%u,\"hub\":%u,\"age\":%d,\"ping\":%u,\"bottom\":%u,\"hard\":%u,\"fish\":%u,\"near\":%u,\"lvl\":%u,\"act\":%u,\"bgver\":%u,\"bgmask\":%u,\"sum\":%s}",
               i ? "," : "", ns.node, ns.hub, static_cast<int>(static_cast<int16_t>(g_frame - ns.frame)), ns.sum.ping,
               ns.sum.bottom_cm, ns.sum.hard, ns.sum.n_targets, ns.sum.nearest_cm, ns.sum.nearest_level, ns.sum.activity,
               ns.bg_ver, ns.bg_mask, ns.has_sum ? "true" : "false");
      s += b;
    }
    s += "],\"blocks_ok\":"; s += ok; s += ",\"blocks_bad\":"; s += bad;
    s += "}";
    return s;
  }
  s += "]}";
  return s;
}

// {"node":N,"last":seq,"pings":[[seq,index,bottom,[[track,depth,strength,width],..],[[bin,level],..],hard,nf_neg,
//                                 [[track,flick_q,spread_q,elen,mature],..]],..]}   (see sonar_codec.h for units)
String meshSonarPingsJson(uint8_t node, uint32_t since, uint8_t max_pings) {
  String s;
  if (g_sonar == nullptr) return F("{\"pings\":[]}");
  if (max_pings > 64) max_pings = 64;
  static sonar::StoredPing copy[64];
  uint16_t n = 0;
  uint32_t last;
  {
    Lock l;
    const sonar::StoredPing* ptr[64];
    n = g_sonar->pingsSince(node, since, ptr, 64);
    const uint16_t skip = n > max_pings ? static_cast<uint16_t>(n - max_pings) : 0;   // newest ones
    for (uint16_t i = skip; i < n; i++) copy[i - skip] = *ptr[i];
    n = static_cast<uint16_t>(n - skip);
    last = g_sonar->lastSeq();
  }
  s.reserve(80 + n * 110);
  s = "{\"node\":"; s += node; s += ",\"last\":"; s += last; s += ",\"pings\":[";
  for (uint16_t i = 0; i < n; i++) {
    const sonar::Ping& p = copy[i].p;
    char b[64];
    snprintf(b, sizeof(b), "%s[%lu,%u,%u,[", i ? "," : "", static_cast<unsigned long>(copy[i].seq), p.index, p.bottom_cm);
    s += b;
    for (uint8_t k = 0; k < p.n_targets; k++) {
      snprintf(b, sizeof(b), "%s[%u,%u,%u,%u]", k ? "," : "", p.t[k].track, p.t[k].depth_cm, p.t[k].strength, p.t[k].width);
      s += b;
    }
    s += "],[";
    for (uint8_t k = 0; k < p.n_resid; k++) {
      snprintf(b, sizeof(b), "%s[%u,%u]", k ? "," : "", p.r[k].bin, p.r[k].level);
      s += b;
    }
    snprintf(b, sizeof(b), "],%u,%u,[", copy[i].hard, copy[i].nf_neg);
    s += b;
    for (uint8_t k = 0; k < copy[i].n_info; k++) {
      const sonar::TrackInfo& x = copy[i].info[k];
      snprintf(b, sizeof(b), "%s[%u,%u,%u,%u,%u]", k ? "," : "", x.track, x.flick_q, x.spread_q, x.elen, x.mature ? 1 : 0);
      s += b;
    }
    s += "]]";
  }
  s += "]}";
  return s;
}

String meshSonarBgJson(uint8_t node) {
  if (g_sonar == nullptr) return F("{}");
  static uint8_t lv[sonar::BINS];
  uint8_t ver = 0, mask = 0;
  bool found = false;
  {
    Lock l;
    const sonar::NodeSonar* ns = g_sonar->find(node);
    if (ns != nullptr) { memcpy(lv, ns->bg, sizeof(lv)); ver = ns->bg_ver; mask = ns->bg_mask; found = true; }
  }
  String s;
  s.reserve(560);
  s = "{\"node\":"; s += node; s += ",\"ver\":"; s += ver; s += ",\"mask\":"; s += mask;
  s += ",\"bin_mm\":"; s += sonar::BIN_MM; s += ",\"levels\":\"";
  if (found) for (uint16_t i = 0; i < sonar::BINS; i++) s += static_cast<char>('0' + (lv[i] & 3));
  s += "\"}";
  return s;
}
