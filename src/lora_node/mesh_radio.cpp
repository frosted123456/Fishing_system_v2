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
static ChaletRole<10, 32>* g_ch = nullptr;

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
static uint32_t g_cmd_until_ms = 0;
static uint32_t g_silence_until_ms = 0;

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

struct RxRes { bool irq; bool got; bool crc; uint8_t len; int8_t rssi; int8_t snr_q4; uint32_t end_us; };

static const uint32_t RX_EXTEND_US = 2000;     // est.: < LEAD_US, so a late packet never eats the next slot

// Listen in mode `m` from `open_us` to `close_us`. A packet whose header arrived before close
// may finish up to `extend_us` later (bounded so it never runs into the next slot / beacon).
static RxRes rxWindow(RadioMode m, uint32_t open_us, uint32_t close_us, uint8_t* buf, uint32_t extend_us = RX_EXTEND_US) {
  RxRes r;
  memset(&r, 0, sizeof(r));
  if (!g_radio_ok) { sleepUntil(close_us); return r; }
  setMode(m);
  if (!reached(open_us)) sleepUntil(open_us);
  if (reached(close_us)) return r;
  clearIrq();
  radio.startReceive();
  bool irq = waitIrq(close_us);
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
static void chaletApplyConfig() {   // under lock
  PlannerConfig& c = g_ch->planner.cfg;
  if (c.test_mode != g_cfg_test) { c.test_mode = g_cfg_test; g_ch->planner.resetStats(); }
  c.adaptive = g_cfg_adaptive;
  if (g_cfg_reset_stats) { g_ch->planner.resetStats(); g_cfg_reset_stats = false; }
  g_ch->focus_node = g_cfg_focus;
  if (g_cfg_reset_all) { g_cfg_reset_all = false; g_cmd_seq++; g_ch->cmd = CMD_RESET_ALL; g_ch->cmd_seq = g_cmd_seq; g_cmd_until_ms = millis() + 5000; }
  if (g_ch->cmd != CMD_NONE && static_cast<int32_t>(millis() - g_cmd_until_ms) >= 0) g_ch->cmd = CMD_NONE;
  const bool silenced = g_silence_state && static_cast<int32_t>(g_silence_until_ms - millis()) > 0;
  if (g_silence_state && !silenced) { g_silence_state = false; g_silence_event = true; }
  g_ch->flags = static_cast<uint8_t>((silenced ? BF_SILENCED : 0) | (g_cfg_sonar_sim ? BF_SONAR_SIM : 0));
  const uint32_t rem_s = silenced ? (g_silence_until_ms - millis()) / 1000 : 0;
  g_ch->silence_10s = static_cast<uint8_t>(rem_s / 10 > 255 ? 255 : rem_s / 10);
}

static void chaletHandle(const ChaletRxResult& res) {   // under lock
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

static void chaletTask() {
  static uint8_t buf[MAX_PACKET];
  static uint8_t rx[MAX_PACKET];
  uint16_t frame = 1;
  uint32_t next = nowUs() + 200000;
  for (;;) {
    // listen for random JOINs until the next frame
    while (static_cast<int32_t>(next - 4000 - nowUs()) > 0) {
      RxRes r = rxWindow(MODE_SF9_BW500, nowUs(), next - 4000, rx, 0);   // never extend into the beacon
      if (r.got) { Lock l; g_ch->onAsyncPacket(rx, r.len, r.rssi); }
    }
    Beacon plan;
    SlotTime times[MAX_SLOTS];
    size_t blen;
    uint32_t frame_us;
    {
      Lock l;
      chaletApplyConfig();
      blen = g_ch->startFrame(frame, buf, sizeof(buf));
      plan = g_ch->beacon;
      memcpy(times, g_ch->times, sizeof(times));
      frame_us = static_cast<uint32_t>(g_ch->planner.cfg.frame_ms) * 1000UL;
      g_frame = frame;
    }
    uint32_t end = 0, ref = next;
    if (txAt(buf, blen, MODE_SF9_BW500, next, end, false)) ref = refFromBeaconEnd(end, static_cast<uint16_t>(blen));
    for (uint8_t i = 0; i < plan.n_slots; i++) {
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
      NodeChange ex[8];
      const uint8_t n = g_ch->expireNodes(ex, 8);
      for (uint8_t k = 0; k < n; k++) {
        MeshNodeUpdate u = {ex[k].node, ex[k].owner, ex[k].old_state, ex[k].new_state, ex[k].turns, ex[k].flags};
        xQueueSend(g_nodeq, &u, 0);
      }
    }
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
  const bool s = (g_hub->plan.flags & BF_SILENCED) != 0;
  if (s != g_silence_state) { g_silence_state = s; g_silence_event = true; }
}

// Handle a packet heard at any time. Returns true if it installed a new plan (beacon/echo).
static bool hubOnPacket(int slot, const uint8_t* p, const RxRes& r) {
  Lock l;
  Header h;
  if (!readHeader(p, r.len, g_net, h)) return false;
  if (h.type == PT_BEACON || h.type == PT_ECHO) {
    uint8_t cmd = CMD_NONE;
    if (g_hub->onBeacon(p, r.len, r.end_us, r.rssi, r.snr_q4, cmd)) {
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

static void hubSearch(uint32_t& next_join_us) {
  static uint8_t rx[MAX_PACKET];
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
          uint8_t flags = 0;
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
  for (;;) {
    bool synced;
    { Lock l; synced = g_hub->have_plan && g_hub->sync.synced(); }
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
  g_mx = xSemaphoreCreateMutex();
  g_cmd_seq = static_cast<uint8_t>(esp_random());   // a rebooted chalet must not reuse the last command seq
  g_nodeq = xQueueCreate(32, sizeof(MeshNodeUpdate));
  if (chalet) {
    g_ch = new ChaletRole<10, 32>();
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
  // Core 1 (same as loop), higher priority than loop: the radio preempts loop() for slot timing.
  xTaskCreatePinnedToCore(radioTask, "mesh", 8192, nullptr, 5, &g_task, 1);
  return g_radio_ok;
}

void meshHubObserveNode(uint8_t node, uint8_t state, uint8_t turns, uint8_t flags, uint8_t battery_pct) {
  if (g_hub == nullptr) return;
  Lock l;
  g_hub->table.observe(node, state, turns, flags, battery_pct, millis());
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

const char* meshModeName(uint8_t mode) { return validMode(mode) ? modeInfo(static_cast<RadioMode>(mode)).name : "-"; }
const char* meshTestModeName(uint8_t m) {
  switch (m) {
    case TEST_OFF: return "off"; case TEST_ROTATE: return "rotate"; case TEST_FIX_SF9: return "SF9/500";
    case TEST_FIX_SF8: return "SF8/500"; case TEST_FIX_SF7: return "SF7/500"; default: return "?";
  }
}

String meshRadioJson() {
  String s;
  s.reserve(3000);
  s += "{\"role\":\""; s += g_chalet ? "chalet" : "hub"; s += "\"";
  s += ",\"radio_ok\":"; s += g_radio_ok ? "true" : "false";
  s += ",\"frame\":"; s += g_frame;
  s += ",\"rx_ok\":"; s += g_rx_ok; s += ",\"rx_crc\":"; s += g_rx_crc; s += ",\"tx\":"; s += g_tx;
  s += ",\"test_mode\":\""; s += meshTestModeName(meshTestMode()); s += "\"";
  s += ",\"adaptive\":"; s += g_cfg_adaptive ? "true" : "false";
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
  return g_hub->sonar.push(blk, len, g_hub->plan.frame);
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
