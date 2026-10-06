// Chalet role: builds the beacon of each frame and digests what it hears in each slot.
#ifndef ICEMESH_CHALET_ROLE_H
#define ICEMESH_CHALET_ROLE_H
#include <stdint.h>
#include <string.h>
#include "tdma_proto.h"
#include "tdma_schedule.h"
#include "chalet_planner.h"
#include "chalet_nodes.h"
#include "chalet_rx.h"

namespace icemesh {
namespace tdma {

template <uint8_t MAX_HUBS = 10, uint8_t MAX_NODES = 32>
class ChaletRole {
 public:
  ChaletPlanner<MAX_HUBS> planner;
  ChaletNodes<MAX_NODES> nodes;
  Beacon beacon;
  SlotTime times[MAX_SLOTS];
  uint16_t frame = 0;
  uint16_t beacon_len = 0;
  uint32_t packets_rx = 0, bad_packets = 0, test_bytes_rx = 0;

  // Flags/command the caller wants in the next beacon.
  uint8_t flags = 0;          // BF_SILENCED ... (BF_TEST / BF_ADAPTIVE are set from the planner config)
  uint8_t cmd = CMD_NONE;
  uint8_t cmd_seq = 0;
  uint8_t silence_10s = 0;
  uint8_t focus_node = 0;
  uint8_t cmd_target = 0, cmd_value = 0;
  uint8_t net_cfg = 0;                       // makeNetCfg(transport, ESP-NOW LR, LoRa channel)
  sonar::SonarSink* sonar_sink = nullptr;   // gets every SEC_SONAR block (direct or relayed)

  // ESP-NOW backbone: hubs heard there (independent of the LoRa plan)
  struct EbHub { uint8_t id; uint16_t last_frame; uint32_t rx; uint8_t hops; };
  EbHub eb_hubs[MAX_HUBS];
  uint32_t eb_rx = 0;

  ChaletRole() { memset(&beacon, 0, sizeof(beacon)); memset(eb_hubs, 0, sizeof(eb_hubs)); }

  // Builds and encodes the beacon for `frame_no`. Returns its length.
  size_t startFrame(uint16_t frame_no, uint8_t* buf, size_t cap) {
    frame = frame_no;
    // the hub that reports the focus node gets the bigger FOCUS allowance
    const NodeView* fv = focus_node ? nodes.find(focus_node) : nullptr;
    planner.cfg.focus_hub = fv != nullptr ? fv->owner : ID_NONE;
    planner.buildBeacon(frame, beacon);
    beacon.flags = static_cast<uint8_t>((flags & ~(BF_TEST | BF_ADAPTIVE)) |
                                        (planner.cfg.test_mode != TEST_OFF ? BF_TEST : 0) |
                                        (planner.cfg.adaptive ? BF_ADAPTIVE : 0));
    beacon.cmd = cmd; beacon.cmd_seq = cmd_seq; beacon.silence_10s = silence_10s; beacon.focus_node = focus_node;
    beacon.cmd_target = cmd_target; beacon.cmd_value = cmd_value; beacon.net_cfg = net_cfg;
    const size_t len = encodeBeacon(beacon, PT_BEACON, buf, cap);
    beacon_len = static_cast<uint16_t>(len);
    computeSlotTimes(beacon.slots, beacon.n_slots, times, beacon_len);
    return len;
  }

  // A valid packet was received during slot i. Fills `res` for HUB packets (node changes etc.).
  bool onSlotPacket(uint8_t i, const uint8_t* p, size_t len, int8_t rssi, int8_t snr_q4, ChaletRxResult& res) {
    memset(&res, 0, sizeof(res));
    Header h;
    if (i >= beacon.n_slots || !readHeader(p, len, planner.cfg.network_id, h)) { bad_packets++; return false; }
    const Slot& s = beacon.slots[i];
    packets_rx++;
    if (h.type == PT_JOIN) { planner.onJoin(h.src, 0, rssi, frame); return false; }   // JOIN slot or random
    if (h.type != PT_HUB) return false;
    planner.onDirectPacket(h.src, frame, rssi, snr_q4, s.mode, s.kind == SLOT_HUB && s.owner == h.src);
    res = consumeHubPacket(p, len, planner.cfg.network_id, frame, planner, nodes, sonar_sink);
    test_bytes_rx += res.test_bytes;
    if (res.malformed) bad_packets++;
    return res.ok;
  }

  // Packet heard outside the slots (random JOIN of a hub that has no plan yet).
  void onAsyncPacket(const uint8_t* p, size_t len, int8_t rssi) {
    Header h;
    if (!readHeader(p, len, planner.cfg.network_id, h)) { bad_packets++; return; }
    packets_rx++;
    if (h.type == PT_JOIN) planner.onJoin(h.src, 0, rssi, frame);
  }

  // Slot i ended without a valid packet (crc_error = something was received but failed CRC).
  void onSlotEmpty(uint8_t i, bool crc_error) {
    if (i >= beacon.n_slots) return;
    const Slot& s = beacon.slots[i];
    if (s.kind == SLOT_HUB && s.via == 0) planner.onSlotMissed(s.owner, frame, s.mode, crc_error);
  }

  uint8_t expireNodes(NodeChange* out, uint8_t max) { return nodes.expire(frame, out, max); }

  // ESP-NOW backbone beacon: this frame's beacon without slots (acks, flags, commands, network config).
  size_t buildEbBeacon(uint8_t* buf, size_t cap) const {
    Beacon b = beacon;
    b.n_slots = 0;
    return encodeBeacon(b, PT_BEACON, buf, cap);
  }

  // Hub packet received over the ESP-NOW backbone (`hops` = relays it went through).
  bool onEbHubPacket(const uint8_t* p, size_t len, uint8_t hops, ChaletRxResult& res) {
    memset(&res, 0, sizeof(res));
    Header h;
    if (!readHeader(p, len, planner.cfg.network_id, h) || h.type != PT_HUB) { bad_packets++; return false; }
    eb_rx++;
    res = consumeHubPacket(p, len, planner.cfg.network_id, frame, planner, nodes, sonar_sink);
    if (res.malformed) bad_packets++;
    int slot = -1;
    for (uint8_t i = 0; i < MAX_HUBS && slot < 0; i++) if (eb_hubs[i].id == h.src) slot = i;
    for (uint8_t i = 0; i < MAX_HUBS && slot < 0; i++) if (eb_hubs[i].id == 0) slot = i;
    if (slot < 0) {   // replace the oldest
      slot = 0;
      for (uint8_t i = 1; i < MAX_HUBS; i++) if (seqDiff(eb_hubs[slot].last_frame, eb_hubs[i].last_frame) > 0) slot = i;
      eb_hubs[slot].rx = 0;
    }
    eb_hubs[slot].id = h.src; eb_hubs[slot].last_frame = frame; eb_hubs[slot].rx++; eb_hubs[slot].hops = hops;
    return res.ok;
  }
};

}  // namespace tdma
}  // namespace icemesh
#endif
