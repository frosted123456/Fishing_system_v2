// Hub role: everything a hub does in a superframe, without touching the radio.
// The firmware radio task calls these at the right times; the PC simulation calls them directly.
//
// Per frame:  beacon/echo received -> onBeacon()  (sync, acks, plan, commands)
//             for each slot i: action(i) tells TX (own slot / echo / join) or RX (listen)
//             packets heard in a slot -> onSlotPacket(i, ...)  (joins, neighbours, relay duty)
#ifndef ICEMESH_HUB_ROLE_H
#define ICEMESH_HUB_ROLE_H
#include <stdint.h>
#include <string.h>
#include "tdma_proto.h"
#include "tdma_schedule.h"
#include "line_table.h"
#include "hub_sync.h"
#include "hub_packet.h"

namespace icemesh {
namespace tdma {

enum SlotAction : uint8_t { ACT_IDLE = 0, ACT_RX, ACT_TX_HUB, ACT_TX_ECHO, ACT_TX_JOIN };

template <uint8_t MAXN = 24>
class HubRole {
 public:
  enum { HEALTH_EVERY = 8 };   // frames between health sections (est.)

  uint8_t network_id = 0x42;
  uint8_t self = 0;
  uint8_t fw = 1;
  LineTable<MAXN> table;
  HubSync sync;

  // last plan
  Beacon plan;
  SlotTime times[MAX_SLOTS];
  bool have_plan = false;
  bool plan_from_echo = false;
  uint8_t echo_src = 0;

  // last beacon link quality + counters
  int8_t beacon_rssi = 0;
  int8_t beacon_snr_q4 = 0;
  uint32_t beacons_rx = 0, echoes_rx = 0;
  uint8_t last_cmd_seq = 0;
  uint8_t last_cmd_target = 0, last_cmd_value = 0;   // arguments of the last command returned by onBeacon/onEbBeacon
  sonar::SonarOutbox sonar;   // blocks waiting for this hub's slot

  // Chalet state from the latest beacon, LoRa or ESP-NOW backbone (whichever came last)
  bool have_info = false;
  uint8_t info_flags = 0, info_focus = 0, info_net_cfg = 0, info_test = 0;
  uint16_t info_frame = 0;
  uint32_t eb_beacons_rx = 0;
  // LoRa channel switch announced by the chalet (CMD_SET_CHANNEL)
  bool channel_switch_pending = false;
  uint8_t channel_next = 0;
  uint16_t channel_switch_frame = 0;   // first frame on the new channel

  uint8_t focusNode() const { return have_info ? info_focus : 0; }
  bool sonarSim() const { return have_info && (info_flags & BF_SONAR_SIM) != 0; }
  bool silenced() const { return have_info && (info_flags & BF_SILENCED) != 0; }
  uint8_t transport() const { return have_info ? netTransport(info_net_cfg) : static_cast<uint8_t>(TR_AUTO); }
  bool espNowLr() const { return have_info && netLr(info_net_cfg); }

  HubRole() { memset(&plan, 0, sizeof(plan)); resetFrameScratch(); n_joins_ = 0; n_nb_last_ = 0; nodeinfo_idx_ = 0; }

  // ---- beacon / echo --------------------------------------------------------------------
  // rx_end_us: RxDone time (REF is derived from it).
  uint16_t beacon_len = 0;  // length of the installed beacon/echo Returns true when a plan for the current frame was installed.
  // `cmd_out` gets a new command (CMD_*) when the beacon carries one not seen before.
  bool onBeacon(const uint8_t* p, size_t len, uint32_t rx_end_us, int8_t rssi, int8_t snr_q4, uint8_t& cmd_out) {
    cmd_out = CMD_NONE;
    Beacon b; PacketType t;
    if (!decodeBeacon(p, len, network_id, b, t)) return false;
    if (t == PT_ECHO && b.src == self) return false;
    if (t == PT_ECHO && have_plan && !plan_from_echo && b.frame == plan.frame) return false;  // already have the original
    uint32_t ref = refFromBeaconEnd(rx_end_us, static_cast<uint16_t>(len));
    SlotTime tt[MAX_SLOTS];
    computeSlotTimes(b.slots, b.n_slots, tt, static_cast<uint16_t>(len));   // echo has the beacon's length
    if (t == PT_ECHO) {
      int idx = -1;
      for (uint8_t i = 0; i < b.n_slots; i++) if (b.slots[i].kind == SLOT_ECHO && b.slots[i].owner == b.src) idx = i;
      if (idx < 0) return false;
      ref = refFromSlotPacket(rx_end_us, tt[idx], MODE_SF9_BW500, static_cast<uint16_t>(len));
      echoes_rx++;
    } else {
      beacons_rx++;
    }
    sync.onBeacon(ref, b.frame, static_cast<uint32_t>(b.frame_10ms) * 10000UL);
    plan = b;
    beacon_len = static_cast<uint16_t>(len);
    memcpy(times, tt, sizeof(tt));
    have_plan = true;
    plan_from_echo = (t == PT_ECHO);
    fresh_beacon_ = true;
    echo_src = (t == PT_ECHO) ? b.src : 0;
    beacon_rssi = rssi; beacon_snr_q4 = snr_q4;
    applyInfo(plan, cmd_out);
    resetFrameScratch();
    return true;
  }

  // Chalet beacon received over the ESP-NOW backbone (no slots, no timing): acks, commands, flags.
  bool onEbBeacon(const uint8_t* p, size_t len, uint8_t& cmd_out) {
    cmd_out = CMD_NONE;
    Beacon b; PacketType t;
    if (!decodeBeacon(p, len, network_id, b, t) || t != PT_BEACON) return false;
    eb_beacons_rx++;
    applyInfo(b, cmd_out);
    return true;
  }

  // No beacon this frame: keep last plan, free-run. Echo slots and JOIN are not used then.
  void onBeaconMissed() {
    fresh_beacon_ = false;
    sync.onMissedBeacon();
    if (!sync.synced()) have_plan = false;
    if (have_plan) plan.frame = sync.frame();
    resetFrameScratch();
  }

  bool hasOwnSlot() const { return have_plan && ownSlotIndex() >= 0; }
  int ownSlotIndex() const {
    for (uint8_t i = 0; i < plan.n_slots; i++) if (plan.slots[i].kind == SLOT_HUB && plan.slots[i].owner == self) return i;
    return -1;
  }

  // What to do during slot i. `join_roll` = caller's random choice to try the JOIN slot this frame.
  SlotAction action(uint8_t i, bool join_roll) const {
    if (!have_plan || i >= plan.n_slots) return ACT_IDLE;
    const Slot& s = plan.slots[i];
    const bool fresh = sync.mayTransmit();
    switch (s.kind) {
      case SLOT_HUB:
        if (s.owner == self) return fresh ? ACT_TX_HUB : ACT_IDLE;
        return ACT_RX;
      case SLOT_ECHO:
        if (s.owner == self) return (fresh_beacon_ && !plan_from_echo) ? ACT_TX_ECHO : ACT_IDLE;   // only repeat a beacon actually heard
        return ACT_RX;
      case SLOT_JOIN:
        if (!hasOwnSlot() && join_roll && fresh) return ACT_TX_JOIN;
        return ACT_RX;
      default:
        return ACT_IDLE;
    }
  }

  // ---- packets this hub transmits -----------------------------------------------------------
  size_t buildEcho(uint8_t* buf, size_t cap) const {
    Beacon b = plan;
    b.src = self;
    return encodeBeacon(b, PT_ECHO, buf, cap);
  }
  size_t buildJoin(uint8_t* buf, size_t cap) const {
    PacketWriter w(buf, cap);
    if (!w.beginJoin(network_id, self, plan.frame, fw)) return 0;
    return w.size();
  }
  // health: pass the current battery etc.; neighbours and beacon stats are filled here.
  size_t buildHub(uint8_t* buf, size_t cap, uint8_t flags, uint16_t battery_mv, uint16_t uptime_min,
                  bool test, uint16_t test_counter, HubBuildResult* res = nullptr) {
    const int idx = ownSlotIndex();
    if (idx < 0) return 0;
    Health h;
    memset(&h, 0, sizeof(h));
    const bool with_health = (plan.frame % HEALTH_EVERY) == (self % HEALTH_EVERY);
    if (with_health) {
      h.battery_mv = battery_mv; h.uptime_min = uptime_min; h.fw = fw;
      h.beacon_rssi = beacon_rssi; h.beacon_snr_q4 = beacon_snr_q4; h.beacon_lost = sync.lostLast64();
      const int32_t e = sync.lastErrUs() / 10;
      h.sync_err_10us = static_cast<int16_t>(e > 32767 ? 32767 : (e < -32768 ? -32768 : e));
      h.n_nb = n_nb_last_;
      memcpy(h.nb, nb_last_, sizeof(Neighbor) * n_nb_last_);
    }
    HubBuild in;
    memset(&in, 0, sizeof(in));
    in.network_id = network_id; in.self = self; in.frame = plan.frame; in.flags = flags;
    in.allowance = plan.slots[idx].allowance;
    in.health = with_health ? &h : nullptr;
    in.relayed = relayed_; in.n_relayed = n_relayed_;
    in.joins = joins_; in.n_joins = n_joins_;
    in.nodeinfo_start = nodeinfo_idx_;
    in.line_rot = line_rot_;
    in.test = test; in.test_counter = test_counter; in.test_mode_seen = plan.test_mode;
    in.sonar = &sonar;
    const HubBuildResult r = buildHubPacket(in, table, buf, cap);
    nodeinfo_idx_ = r.nodeinfo_next;
    line_rot_ = r.line_rot_next;
    if (r.len > 0) n_joins_ = 0;   // joins reported (repeated by the joiner if lost)
    if (res != nullptr) *res = r;
    return r.len;
  }

  // Hub packet for the ESP-NOW backbone: same content as the slot packet (line state first, sonar,
  // node info, health every HEALTH_EVERY packets) without a slot plan; `cap` <= EB payload.
  size_t buildHubFree(uint8_t* buf, size_t cap, uint8_t flags, uint16_t battery_mv, uint16_t uptime_min, uint16_t frame) {
    Health h;
    memset(&h, 0, sizeof(h));
    const bool with_health = (eb_count_++ % HEALTH_EVERY) == 0;
    if (with_health) {
      h.battery_mv = battery_mv; h.uptime_min = uptime_min; h.fw = fw;
      h.beacon_rssi = beacon_rssi; h.beacon_snr_q4 = beacon_snr_q4; h.beacon_lost = sync.lostLast64();
      h.n_nb = n_nb_last_;
      memcpy(h.nb, nb_last_, sizeof(Neighbor) * n_nb_last_);
    }
    HubBuild in;
    memset(&in, 0, sizeof(in));
    in.network_id = network_id; in.self = self; in.frame = frame; in.flags = flags;
    in.allowance = static_cast<uint8_t>(cap > MAX_PACKET ? MAX_PACKET : cap);
    in.health = with_health ? &h : nullptr;
    in.nodeinfo_start = nodeinfo_idx_;
    in.line_rot = line_rot_;
    in.sonar = &sonar;
    const HubBuildResult r = buildHubPacket(in, table, buf, cap);
    nodeinfo_idx_ = r.nodeinfo_next;
    line_rot_ = r.line_rot_next;
    return r.len;
  }

  // ---- packets heard in slot i ------------------------------------------------------------
  void onSlotPacket(uint8_t i, const uint8_t* p, size_t len, int8_t rssi, int8_t snr_q4) {
    Header h;
    if (!readHeader(p, len, network_id, h) || h.src == self) return;
    if (h.type == PT_JOIN) { addJoin(h.src); noteNeighbor(h.src, rssi, snr_q4); return; }
    if (h.type != PT_HUB) return;
    noteNeighbor(h.src, rssi, snr_q4);
    if (i < plan.n_slots && plan.slots[i].kind == SLOT_HUB && plan.slots[i].via == self && n_relayed_ < MAX_RELAYED && len <= MAX_PACKET) {
      relayed_[n_relayed_].len = static_cast<uint8_t>(len);
      memcpy(relayed_[n_relayed_].data, p, len);
      n_relayed_++;
    }
  }

  // Packet heard outside any slot (a hub without plan sends JOIN at random times).
  void onAsyncPacket(const uint8_t* p, size_t len, int8_t rssi, int8_t snr_q4) {
    Header h;
    if (!readHeader(p, len, network_id, h) || h.src == self || h.type != PT_JOIN) return;
    addJoin(h.src);
    noteNeighbor(h.src, rssi, snr_q4);
  }

  // A hub that hears neither beacon nor echo announces itself at random (ALOHA, low rate).
  bool wantsAsyncJoin() const { return !have_plan; }

  // Call once per frame end (after the last slot): neighbours heard this frame become the report.
  void endFrame() {
    memcpy(nb_last_, nb_cur_, sizeof(nb_cur_));
    n_nb_last_ = n_nb_cur_;
  }

  uint8_t relayedCount() const { return n_relayed_; }

 private:
  void resetFrameScratch() { n_relayed_ = 0; n_nb_cur_ = 0; }

  // Shared by the LoRa beacon/echo and the ESP-NOW backbone beacon.
  void applyInfo(const Beacon& b, uint8_t& cmd_out) {
    applyAcks(b, self, table);
    have_info = true;
    info_flags = b.flags; info_focus = b.focus_node; info_net_cfg = b.net_cfg; info_test = b.test_mode; info_frame = b.frame;
    if (b.cmd != CMD_NONE && b.cmd_seq != last_cmd_seq) {
      last_cmd_seq = b.cmd_seq; cmd_out = b.cmd; last_cmd_target = b.cmd_target; last_cmd_value = b.cmd_value;
      if (b.cmd == CMD_SET_CHANNEL) {
        channel_switch_pending = true; channel_next = b.cmd_value;
        channel_switch_frame = static_cast<uint16_t>(b.frame + b.cmd_target + 1);
      }
    }
  }

  void addJoin(uint8_t src) {
    for (uint8_t k = 0; k < n_joins_; k++) if (joins_[k] == src) return;
    if (n_joins_ < MAX_JOINS) joins_[n_joins_++] = src;
  }

  void noteNeighbor(uint8_t hub, int8_t rssi, int8_t snr_q4) {
    for (uint8_t k = 0; k < n_nb_cur_; k++) if (nb_cur_[k].hub == hub) { nb_cur_[k].rssi = rssi; nb_cur_[k].snr_q4 = snr_q4; return; }
    if (n_nb_cur_ < MAX_NEIGHBORS) { nb_cur_[n_nb_cur_].hub = hub; nb_cur_[n_nb_cur_].rssi = rssi; nb_cur_[n_nb_cur_].snr_q4 = snr_q4; n_nb_cur_++; }
  }

  RelayItem relayed_[MAX_RELAYED];
  uint8_t n_relayed_;
  uint8_t joins_[MAX_JOINS];
  uint8_t n_joins_;
  Neighbor nb_cur_[MAX_NEIGHBORS], nb_last_[MAX_NEIGHBORS];
  uint8_t n_nb_cur_, n_nb_last_;
  uint8_t nodeinfo_idx_;
  uint8_t line_rot_ = 0;
  bool fresh_beacon_ = false;
  uint32_t eb_count_ = 0;
};

}  // namespace tdma
}  // namespace icemesh
#endif
