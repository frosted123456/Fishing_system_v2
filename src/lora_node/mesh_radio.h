// TDMA LoRa mesh — firmware glue between the radio (SX1262 via RadioLib, own FreeRTOS task)
// and the rest of lora_node (loop(), UI, web). All protocol logic is in lib/IceMesh (host-tested);
// this file only does timing, SPI and locking.
#pragma once
#include <Arduino.h>
#include <stdint.h>

// Line states (same values as icemesh::tdma::LineState)
enum : uint8_t { MESH_LS_IDLE = 0, MESH_LS_TRIPPED = 1, MESH_LS_RUNNING = 2, MESH_LS_FAULT = 3, MESH_LS_OFFLINE = 4, MESH_LS_TRIP_SEEN = 5 };   // D79: 5 = tripped and reset while unacked

struct MeshNodeUpdate {        // chalet: owner-resolved node state change
  uint8_t node, owner, old_state, new_state; uint16_t turns; uint8_t flags;
};

struct MeshNodeSnapshot {      // chalet: current node view
  uint8_t node, owner, state; uint16_t turns; uint8_t flags, battery;
};

struct MeshCounters {
  bool radio_ok;
  uint32_t rx_ok, rx_crc, tx;
  uint32_t last_rx_ms;
  uint16_t frame;
};

struct MeshHubView {           // hub: for OLED / serial during range tests
  bool synced;
  bool from_echo;
  uint16_t frame;
  int8_t beacon_rssi;
  float beacon_snr;
  uint8_t lost64;
  int32_t sync_err_us;
  int own_slot_mode;           // -1 = no slot yet
  uint8_t allowance;
  uint8_t test_mode;
  uint32_t beacons, echoes, tx;
  uint8_t lora_ch;             // LoRa channel index (0 = 915.0 MHz, see loraChannelMHz)
  uint8_t transport;           // network setting seen in the beacon (MESH_TR_*)
  bool lora_on, eb_on;         // paths this hub transmits on right now
  bool eb_fallback;            // Auto: backbone switched on because LoRa beacons stopped
  uint32_t eb_beacon_age_ms;   // since the last backbone beacon (0xFFFFFFFF = never)
  bool relay;                  // this device rebroadcasts backbone frames
};

// Start the radio task. chalet = ROLE_GATEWAY_OFFSHORE, every other LoRa role is a hub.
bool meshBegin(uint8_t self_id, bool chalet, uint8_t network_id);

// ---- hub side (call from loop) ----
void meshHubObserveNode(uint8_t node, uint8_t state, uint16_t turns, uint8_t flags, uint8_t battery_pct);
void meshHubSetBattery(uint16_t mv);
void meshHubView(MeshHubView& v);
// D62: for the sonar capture: true when no LoRa transmit is on the air, the last one ended more than guard_us
// ago and no own transmission (slot, echo, join) is planned within horizon_us. The transmit runs in the radio
// task (higher priority than loop): it cannot be held back, only avoided. LoRa off (ESP-NOW only) = quiet.
bool meshLoraQuiet(uint32_t horizon_us, uint32_t guard_us);

// ---- both ----
void meshRequestSilence(bool on);            // hub: request in the next packets; chalet: beacon state
bool meshPollSilence(bool& on);              // network silence state changed (beacon on hubs, request on chalet)
bool meshPollReset();                        // hub: chalet sent RESET ALL
void meshCounters(MeshCounters& c);
uint8_t meshTestMode();                      // hub: as seen in the beacon; chalet: configured

// ---- chalet side ----
bool meshPollNodeUpdate(MeshNodeUpdate& u);
uint8_t meshNodeSnapshot(MeshNodeSnapshot* out, uint8_t max);
void meshSetTestMode(uint8_t mode);          // 0 off, 1 rotate SF9/8/7, 2 SF9, 3 SF8, 4 SF7
void meshSetAdaptive(bool on);
bool meshAdaptive();
void meshResetStats();
void meshSendResetAll();
void meshSetFocusNode(uint8_t node);
void meshAutoFocus(uint8_t node);      // D79: FISH ON -> watch that hole for AUTO_FOCUS_MS (chalet)
void meshAutoFocusTick();
void meshSetFollow(bool on);           // D80 follow the action: focus follows the sonar activity (chalet)
bool meshFollowOn();
void meshFollowState(char* why, size_t n, uint32_t& pin_s, uint32_t& trip_s);   // D80: for the map marker
// D80 range test (chalet): one hub's slot cycles SF9 -> SF5, slot results queued for the logger (rangetest.cpp)
#include <range_test.h>
void meshRangeStart(uint8_t hub);
void meshRangeStop();
uint8_t meshRangeHub();
bool meshRangePop(icemesh::tdma::RangeSample& s);
uint32_t meshRangeLost();
bool meshRangeHubSide(uint8_t hub, icemesh::tdma::RangeHubSide& out, bool& direct, uint8_t& max_sf);
bool meshHubRelaysOthers(uint8_t hub);   // D80: another hub's traffic goes through it
static const uint32_t AUTO_FOCUS_MS = 120000UL;
String meshRadioJson();                      // /api/radio

// ---- both: human readable status (serial) ----
void meshPrintStatus(Print& out);
const char* meshModeName(uint8_t mode);
const char* meshTestModeName(uint8_t mode);

// ---- chalet: per-hub summary for the OLED ----
struct MeshHubSummary { uint8_t id, via, mode; int8_t rssi; float snr; uint32_t rx, sched; };
uint8_t meshHubSummaries(MeshHubSummary* out, uint8_t max);

// ---- v2 network config: LoRa channel, transport, ESP-NOW backbone (docs/protocol_v2.md §6c) ----
enum : uint8_t { MESH_TR_AUTO = 0, MESH_TR_LORA = 1, MESH_TR_ESPNOW = 2, MESH_TR_PREFER_EB = 3 };   // = icemesh::tdma::Transport (D71: ESP-NOW first)
static const uint8_t MESH_CH_AUTO = 255;
void meshSetStartChannel(uint8_t ch);        // both, before meshBegin: last channel used (NVS); hub starts searching there
void meshSetTransport(uint8_t tr);           // chalet: MESH_TR_*
uint8_t meshTransport();                     // chalet: configured; hub: as seen in the beacon (Auto before any)
void meshSetLoraChannel(uint8_t ch);         // chalet: 0-7 fixed or MESH_CH_AUTO (scan at boot, move when busy)
uint8_t meshLoraChannelSetting();            // chalet
uint8_t meshLoraChannel();                   // both: channel in use
float meshLoraChannelMHz(uint8_t ch);
void meshRescanChannels();                   // chalet: re-evaluate now (Auto) / apply the fixed channel
void meshChannelBusy(uint8_t* pct8);         // chalet: foreign activity per channel in % (255 = no data yet)
// ESP-NOW backbone. The sender callback broadcasts one frame with esp_now_send.
typedef bool (*MeshEbSendFn)(const uint8_t* frame, size_t len);
void meshSetEbSender(MeshEbSendFn fn);
void meshEbReceive(const uint8_t* frame, size_t len);   // loop(): every ESP-NOW frame of type 0x70 / 0x71
void meshEbTick();                                       // loop(): hub sends its packet on the backbone when needed
void meshSetEbRelay(bool on);                            // this device rebroadcasts backbone frames
bool meshEbRelay();
void meshSetDeviceRelay(uint8_t dev, bool on);           // chalet: CMD_SET_RELAY in the beacon (hub or tip-up ID)
bool meshPollDevCmd(uint8_t& cmd, uint8_t& target, uint8_t& value);   // hub: SET_RELAY / SET_SIM received (MESH_CMD_*)
void meshSetHoleSim(uint8_t hole, uint8_t value);       // chalet: CMD_SET_SIM in the beacon (hole ID or 255 = all)
enum : uint8_t { MESH_CMD_SET_RELAY = 3, MESH_CMD_SET_SIM = 4, MESH_CMD_SONAR_PARAM = 5, MESH_CMD_SET_BAIT = 6, MESH_CMD_SET_ARM = 7, MESH_CMD_SELFTEST = 8 };   // = icemesh::tdma::BeaconCmd
void meshSetSonarParam(uint8_t id, uint8_t value);   // chalet: knob for every sonar hole (CMD_SONAR_PARAM)
void meshSetBait(uint8_t hole, uint8_t v5);          // chalet: bait depth of one hole, 5 cm steps (CMD_SET_BAIT)
void meshSetArm(uint8_t hole, bool armed);           // chalet: arm / disarm one hole or 255 = all (CMD_SET_ARM, D61)
bool meshCmdPending(uint8_t cmd, uint8_t target, uint8_t& value);   // D68: queued or being beaconed (page feedback)
void meshSetSelfTest(uint8_t hole);                                   // D70: chalet asks a hole for its wiring check
void meshHubSelfTestReport(uint8_t node, const uint8_t* r12);         // D70: hub: a hole's result, to the chalet
bool meshSelfTestResult(uint8_t node, uint8_t out12[12], uint32_t& age_ms);   // D70: chalet: last result
enum : uint8_t { MESH_SIM_SONAR = 0x01, MESH_SIM_HALL = 0x02, MESH_LF_LOWBAT = 0x01, MESH_LF_SIM = 0x02, MESH_LF_DISARMED = 0x04 };
bool meshPollChannelChanged(uint8_t& ch);                // both: channel changed (to save in NVS)
uint32_t meshSetupArmInS();                              // both: setup phase, s until the automatic fallbacks arm (0 = armed)

// ---- sonar: test mode (fake data) + FOCUS stream. Blocks: lib/IceMesh/src/sonar_codec.h ----
// Virtual sonar nodes on hubs use IDs 128 + (hub ID & 0x0F) * 8 + k: keep real node IDs below 128.
uint8_t meshSonarVirtualId(uint8_t hub_id, uint8_t k);
void meshSetSonarSim(bool on);               // chalet: BF_SONAR_SIM in the beacon
bool meshSonarSim();                         // chalet: configured; hub: as seen in the beacon
uint8_t meshFocusNode();                     // chalet: configured; hub: from the beacon
uint8_t meshFocusNodes(uint8_t* out3);        // D81: streamed holes (watched + up to 2 extras), count
bool meshIsFocus(uint8_t node);
bool meshHubPushSonar(const uint8_t* blk, uint8_t len);   // hub: queue one block for the next slots
String meshSonarListJson();                  // chalet: /api/sonar (one summary per sonar node)
String meshSonarPingsJson(uint8_t node, uint32_t since, uint8_t max_pings);   // chalet: /api/sonar/pings
String meshSonarBgJson(uint8_t node);        // chalet: /api/sonar/bg (2-bit background profile)
String meshSonarGlanceJson(uint16_t since);           // chalet: /api/sonar/glance (every hole, ~3 min of summaries)

// ---- raw views for the OLED screens (chalet) ----
struct MeshSonarLite { uint16_t bottom_cm; uint8_t hard, activity, n; uint16_t t[5]; uint16_t age_frames; uint16_t age_s; uint8_t status, bottom_snr; };   // status: ST_* (D43)   // t: depth | level << 11 | bait << 13
struct MeshPingLite { uint16_t bottom_cm; uint8_t n; uint16_t d[5]; uint8_t lv[5]; uint8_t bait_mask; };
struct MeshHubLink { uint8_t id; int8_t rssi; int32_t lora_age_s, eb_age_s; uint8_t hops; bool demo; };
bool meshSonarSummary(uint8_t node, MeshSonarLite& out);
struct MeshBaseRec { uint16_t frame, bottom_cm; uint8_t n; uint16_t t[5]; };   // one BASE summary (2 s), t as above
uint8_t meshSonarHistory(uint8_t node, MeshBaseRec* out, uint8_t max);        // newest `max`, oldest first (chalet)
uint8_t meshFocusPings(uint8_t node, MeshPingLite* out, uint8_t max);   // newest `max` pings, oldest first
uint8_t meshHubLinks(MeshHubLink* out, uint8_t max);

// ---- v2 demo network (chalet only, no other hardware; docs/SONAR_SIM.md) ----
void meshDemoSet(uint8_t hubs, uint8_t holes_per_hub);   // 0 hubs = off; 1-4 hubs, 1-4 holes each
uint8_t meshDemoHubs();
uint8_t meshDemoHoles();
bool meshDemoNode(uint8_t id);                            // a demo hole ID (fixed IDs, on or off)
uint8_t meshDemoSim(uint8_t id);                          // its simulation value (0xFF = not running)
void meshDemoSetSim(uint8_t id, uint8_t value);           // 255 = every demo hole (applied at once)
void meshDemoName(uint8_t id, char* out, size_t n);       // "Demo A1"
void meshDemoTick();                                      // chalet loop()
void meshDemoSetParams(const uint8_t* v, uint8_t n);      // the chalet's sonar knobs for the demo sonar
void meshDemoSetBait(uint8_t id, uint8_t v5);             // demo hole bait depth (5 cm steps)
