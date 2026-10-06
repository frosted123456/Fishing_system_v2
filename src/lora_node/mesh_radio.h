// TDMA LoRa mesh — firmware glue between the radio (SX1262 via RadioLib, own FreeRTOS task)
// and the rest of lora_node (loop(), UI, web). All protocol logic is in lib/IceMesh (host-tested);
// this file only does timing, SPI and locking.
#pragma once
#include <Arduino.h>
#include <stdint.h>

// Line states (same values as icemesh::tdma::LineState)
enum : uint8_t { MESH_LS_IDLE = 0, MESH_LS_TRIPPED = 1, MESH_LS_RUNNING = 2, MESH_LS_FAULT = 3, MESH_LS_OFFLINE = 4 };

struct MeshNodeUpdate {        // chalet: owner-resolved node state change
  uint8_t node, owner, old_state, new_state, turns, flags;
};

struct MeshNodeSnapshot {      // chalet: current node view
  uint8_t node, owner, state, turns, flags, battery;
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
};

// Start the radio task. chalet = ROLE_GATEWAY_OFFSHORE, every other LoRa role is a hub.
bool meshBegin(uint8_t self_id, bool chalet, uint8_t network_id);

// ---- hub side (call from loop) ----
void meshHubObserveNode(uint8_t node, uint8_t state, uint8_t turns, uint8_t flags, uint8_t battery_pct);
void meshHubSetBattery(uint16_t mv);
void meshHubView(MeshHubView& v);

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
String meshRadioJson();                      // /api/radio

// ---- both: human readable status (serial) ----
void meshPrintStatus(Print& out);
const char* meshModeName(uint8_t mode);
const char* meshTestModeName(uint8_t mode);

// ---- chalet: per-hub summary for the OLED ----
struct MeshHubSummary { uint8_t id, via, mode; int8_t rssi; float snr; uint32_t rx, sched; };
uint8_t meshHubSummaries(MeshHubSummary* out, uint8_t max);

// ---- sonar: test mode (fake data) + FOCUS stream. Blocks: lib/IceMesh/src/sonar_codec.h ----
// Virtual sonar nodes on hubs use IDs 128 + (hub ID & 0x0F) * 8 + k: keep real node IDs below 128.
uint8_t meshSonarVirtualId(uint8_t hub_id, uint8_t k);
void meshSetSonarSim(bool on);               // chalet: BF_SONAR_SIM in the beacon
bool meshSonarSim();                         // chalet: configured; hub: as seen in the beacon
uint8_t meshFocusNode();                     // chalet: configured; hub: from the beacon
bool meshHubPushSonar(const uint8_t* blk, uint8_t len);   // hub: queue one block for the next slots
String meshSonarListJson();                  // chalet: /api/sonar (one summary per sonar node)
String meshSonarPingsJson(uint8_t node, uint32_t since, uint8_t max_pings);   // chalet: /api/sonar/pings
String meshSonarBgJson(uint8_t node);        // chalet: /api/sonar/bg (2-bit background profile)
