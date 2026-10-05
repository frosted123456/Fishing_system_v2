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
  uint32_t packets_rx = 0, bad_packets = 0, test_bytes_rx = 0;

  // Flags/command the caller wants in the next beacon.
  uint8_t flags = 0;          // BF_SILENCED ... (BF_TEST / BF_ADAPTIVE are set from the planner config)
  uint8_t cmd = CMD_NONE;
  uint8_t cmd_seq = 0;
  uint8_t silence_10s = 0;
  uint8_t focus_node = 0;

  ChaletRole() { memset(&beacon, 0, sizeof(beacon)); }

  // Builds and encodes the beacon for `frame_no`. Returns its length.
  size_t startFrame(uint16_t frame_no, uint8_t* buf, size_t cap) {
    frame = frame_no;
    planner.buildBeacon(frame, beacon);
    beacon.flags = static_cast<uint8_t>((flags & ~(BF_TEST | BF_ADAPTIVE)) |
                                        (planner.cfg.test_mode != TEST_OFF ? BF_TEST : 0) |
                                        (planner.cfg.adaptive ? BF_ADAPTIVE : 0));
    beacon.cmd = cmd; beacon.cmd_seq = cmd_seq; beacon.silence_10s = silence_10s; beacon.focus_node = focus_node;
    computeSlotTimes(beacon.slots, beacon.n_slots, times);
    return encodeBeacon(beacon, PT_BEACON, buf, cap);
  }

  // A valid packet was received during slot i. Fills `res` for HUB packets (node changes etc.).
  bool onSlotPacket(uint8_t i, const uint8_t* p, size_t len, int8_t rssi, int8_t snr_q4, ChaletRxResult& res) {
    memset(&res, 0, sizeof(res));
    Header h;
    if (i >= beacon.n_slots || !readHeader(p, len, planner.cfg.network_id, h)) { bad_packets++; return false; }
    const Slot& s = beacon.slots[i];
    packets_rx++;
    if (h.type == PT_JOIN && s.kind == SLOT_JOIN) { planner.onJoin(h.src, 0, rssi, frame); return false; }
    if (h.type != PT_HUB) return false;
    planner.onDirectPacket(h.src, frame, rssi, snr_q4, s.mode, s.kind == SLOT_HUB && s.owner == h.src);
    res = consumeHubPacket(p, len, planner.cfg.network_id, frame, planner, nodes);
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
};

}  // namespace tdma
}  // namespace icemesh
#endif
