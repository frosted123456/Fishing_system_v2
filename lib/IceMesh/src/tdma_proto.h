// TDMA mesh packet formats (v2). Little-endian, byte-aligned, every packet decodes on its own.
//
// Common header (6 B): [0] network_id [1] 0xF2 [2] packet type [3] src id [4..5] superframe (u16)
//
// BEACON / ECHO (chalet, or a relay hub repeating it for hubs that cannot hear the chalet):
//   [6] flags  [7] cmd  [8] cmd_seq  [9] silence (x10 s)  [10] focus node  [11] frame length (x10 ms)
//   [12] test mode  [13] cmd target  [14] cmd value  [15] network config (transport, ESP-NOW LR, LoRa channel)
//   [16] n_slots, n x 5 B {kind, owner, mode, allowance, via}
//   then n_acks, n x 3 B {hub, node, seq}
// HUB packet: [6] flags, then sections {type u8, len u8, value[len]} until the end.
// JOIN packet: [6] firmware version.
//
// Node identity is a global 8-bit ID (1..254); events are acked per (hub, node, seq).
#ifndef ICEMESH_TDMA_PROTO_H
#define ICEMESH_TDMA_PROTO_H
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace icemesh {
namespace tdma {

static const uint8_t VER = 0xF2;
static const uint8_t HDR_LEN = 6;
static const uint8_t MAX_PACKET = 255;
#ifndef ICEMESH_MAX_SLOTS
#define ICEMESH_MAX_SLOTS 16    // slots per beacon: 10 hubs + JOIN + up to 3 echo slots + margin (sim finding S1)
#endif
static const uint8_t MAX_SLOTS = ICEMESH_MAX_SLOTS;
static const uint8_t MAX_ACKS = 16;
static const uint8_t ID_NONE = 0;

enum PacketType : uint8_t { PT_BEACON = 1, PT_ECHO = 2, PT_HUB = 3, PT_JOIN = 4 };
enum SlotKind : uint8_t { SLOT_ECHO = 1, SLOT_JOIN = 2, SLOT_HUB = 3 };

enum BeaconFlags : uint8_t { BF_TEST = 0x01, BF_ADAPTIVE = 0x02, BF_SILENCED = 0x04, BF_SONAR_SIM = 0x08 };
// Commands (repeated in beacons for a few seconds, executed once per cmd_seq):
//   CMD_RESET_ALL     reboot every node
//   CMD_SET_CHANNEL   target = beacons left before the switch, value = LoRa channel index
//   CMD_SET_RELAY     target = device ID, value = 1 relay on / 0 off (ESP-NOW backbone)
enum BeaconCmd : uint8_t { CMD_NONE = 0, CMD_RESET_ALL = 1, CMD_SET_CHANNEL = 2, CMD_SET_RELAY = 3 };

// Network configuration byte, in every beacon (LoRa and ESP-NOW backbone):
//   bits 0-1 transport (TR_*), bit 2 ESP-NOW long-range rate, bits 4-7 current LoRa channel index
enum Transport : uint8_t { TR_AUTO = 0, TR_LORA = 1, TR_ESPNOW = 2 };
inline uint8_t makeNetCfg(uint8_t transport, bool lr, uint8_t channel) {
  return static_cast<uint8_t>((transport & 3u) | (lr ? 4u : 0u) | ((channel & 15u) << 4));
}
inline uint8_t netTransport(uint8_t c) { const uint8_t t = static_cast<uint8_t>(c & 3u); return t > static_cast<uint8_t>(TR_ESPNOW) ? static_cast<uint8_t>(TR_AUTO) : t; }
inline bool netLr(uint8_t c) { return (c & 4u) != 0; }
inline uint8_t netChannel(uint8_t c) { return static_cast<uint8_t>(c >> 4); }
enum TestMode : uint8_t { TEST_OFF = 0, TEST_ROTATE = 1, TEST_FIX_SF9 = 2, TEST_FIX_SF8 = 3, TEST_FIX_SF7 = 4, TEST_MODE_COUNT = 5 };

// HF_EB_RELAY: this hub rebroadcasts ESP-NOW backbone frames. HF_EB_PATH: this hub currently sends on the
// backbone too (fallback / ESP-NOW mode). Both are reports for the chalet's Radio view only.
enum HubFlags : uint8_t { HF_SILENCE_ON = 0x01, HF_SILENCE_OFF = 0x02, HF_PENDING = 0x04, HF_EB_RELAY = 0x08, HF_EB_PATH = 0x10 };

enum SectionType : uint8_t {
  SEC_LINE = 1,      // line-state records, 3 B each — always first
  SEC_HEALTH = 2,    // hub health + neighbour list
  SEC_JOINS = 3,     // hub IDs heard in the JOIN slot (relay hubs report them)
  SEC_NODEINFO = 4,  // node battery etc., 3 B each, rotating
  SEC_RELAY = 5,     // a remote hub's packet body, re-packed by its relay hub
  SEC_TEST = 6,      // radio test filler
  SEC_SONAR = 7,     // one sonar block (sonar_codec.h) per section, never truncated
};

enum LineState : uint8_t { LS_IDLE = 0, LS_TRIPPED = 1, LS_RUNNING = 2, LS_FAULT = 3, LS_OFFLINE = 4 };
enum RecordFlags : uint8_t { RF_LOWBAT = 0x01 };

struct Slot { uint8_t kind, owner, mode, allowance, via; };
struct Ack { uint8_t hub, node, seq; };

struct Beacon {
  uint8_t network_id;
  uint8_t src;
  uint16_t frame;
  uint8_t flags;
  uint8_t cmd;
  uint8_t cmd_seq;
  uint8_t silence_10s;
  uint8_t focus_node;
  uint8_t frame_10ms;
  uint8_t test_mode;
  uint8_t cmd_target;
  uint8_t cmd_value;
  uint8_t net_cfg;
  uint8_t n_slots;
  Slot slots[MAX_SLOTS];
  uint8_t n_acks;
  Ack acks[MAX_ACKS];
};

struct Header { uint8_t network_id; uint8_t type; uint8_t src; uint16_t frame; };

inline void putHeader(uint8_t* b, uint8_t net, PacketType t, uint8_t src, uint16_t frame) {
  b[0] = net; b[1] = VER; b[2] = t; b[3] = src;
  b[4] = static_cast<uint8_t>(frame & 0xFF); b[5] = static_cast<uint8_t>(frame >> 8);
}

inline bool readHeader(const uint8_t* b, size_t len, uint8_t net, Header& h) {
  if (b == nullptr || len < HDR_LEN || b[0] != net || b[1] != VER) return false;
  if (b[2] < PT_BEACON || b[2] > PT_JOIN) return false;
  h.network_id = b[0]; h.type = b[2]; h.src = b[3];
  h.frame = static_cast<uint16_t>(b[4] | (b[5] << 8));
  return h.src != ID_NONE;
}

static const uint8_t BEACON_FIXED = 17;
inline size_t beaconSize(uint8_t n_slots, uint8_t n_acks) { return BEACON_FIXED + 5u * n_slots + 1u + 3u * n_acks; }

inline size_t encodeBeacon(const Beacon& bc, PacketType type, uint8_t* b, size_t cap) {
  if (bc.n_slots > MAX_SLOTS || bc.n_acks > MAX_ACKS || (type != PT_BEACON && type != PT_ECHO)) return 0;
  const size_t need = beaconSize(bc.n_slots, bc.n_acks);
  if (b == nullptr || cap < need) return 0;
  putHeader(b, bc.network_id, type, bc.src, bc.frame);
  b[6] = bc.flags; b[7] = bc.cmd; b[8] = bc.cmd_seq; b[9] = bc.silence_10s;
  b[10] = bc.focus_node; b[11] = bc.frame_10ms; b[12] = bc.test_mode;
  b[13] = bc.cmd_target; b[14] = bc.cmd_value; b[15] = bc.net_cfg; b[16] = bc.n_slots;
  size_t o = BEACON_FIXED;
  for (uint8_t i = 0; i < bc.n_slots; i++) {
    const Slot& s = bc.slots[i];
    b[o++] = s.kind; b[o++] = s.owner; b[o++] = s.mode; b[o++] = s.allowance; b[o++] = s.via;
  }
  b[o++] = bc.n_acks;
  for (uint8_t i = 0; i < bc.n_acks; i++) { b[o++] = bc.acks[i].hub; b[o++] = bc.acks[i].node; b[o++] = bc.acks[i].seq; }
  return o;
}

// Accepts BEACON and ECHO. `type` tells which one it was.
inline bool decodeBeacon(const uint8_t* b, size_t len, uint8_t net, Beacon& bc, PacketType& type) {
  Header h;
  if (!readHeader(b, len, net, h) || (h.type != PT_BEACON && h.type != PT_ECHO) || len < BEACON_FIXED + 1u) return false;
  type = static_cast<PacketType>(h.type);
  bc.network_id = h.network_id; bc.src = h.src; bc.frame = h.frame;
  bc.flags = b[6]; bc.cmd = b[7]; bc.cmd_seq = b[8]; bc.silence_10s = b[9];
  bc.focus_node = b[10]; bc.frame_10ms = b[11]; bc.test_mode = b[12];
  bc.cmd_target = b[13]; bc.cmd_value = b[14]; bc.net_cfg = b[15]; bc.n_slots = b[16];
  if (bc.n_slots > MAX_SLOTS || bc.frame_10ms == 0) return false;
  size_t o = BEACON_FIXED;
  if (len < o + 5u * bc.n_slots + 1u) return false;
  for (uint8_t i = 0; i < bc.n_slots; i++) {
    Slot& s = bc.slots[i];
    s.kind = b[o++]; s.owner = b[o++]; s.mode = b[o++]; s.allowance = b[o++]; s.via = b[o++];
  }
  bc.n_acks = b[o++];
  if (bc.n_acks > MAX_ACKS || len < o + 3u * bc.n_acks) return false;
  for (uint8_t i = 0; i < bc.n_acks; i++) { bc.acks[i].hub = b[o++]; bc.acks[i].node = b[o++]; bc.acks[i].seq = b[o++]; }
  return true;
}

// ---- line-state record (3 B) ----------------------------------------------------------
struct LineRecord { uint8_t node; uint8_t state; uint8_t seq; bool pending; uint8_t turns; uint8_t flags; };
static const uint8_t LINE_RECORD_LEN = 3;

inline void encodeLine(const LineRecord& r, uint8_t* b) {
  b[0] = r.node;
  b[1] = static_cast<uint8_t>(((r.state & 0x07) << 5) | ((r.seq & 0x0F) << 1) | (r.pending ? 1 : 0));
  const uint8_t turns = r.turns > 31 ? 31 : r.turns;
  b[2] = static_cast<uint8_t>((turns << 3) | (r.flags & 0x07));
}
inline void decodeLine(const uint8_t* b, LineRecord& r) {
  r.node = b[0];
  r.state = static_cast<uint8_t>(b[1] >> 5);
  r.seq = static_cast<uint8_t>((b[1] >> 1) & 0x0F);
  r.pending = (b[1] & 1) != 0;
  r.turns = static_cast<uint8_t>(b[2] >> 3);
  r.flags = static_cast<uint8_t>(b[2] & 0x07);
}

// ---- health section ---------------------------------------------------------------------
static const uint8_t MAX_NEIGHBORS = 8;
struct Neighbor { uint8_t hub; int8_t rssi; int8_t snr_q4; };
struct Health {
  uint16_t battery_mv;
  uint16_t uptime_min;
  uint8_t fw;
  int8_t beacon_rssi;      // dBm
  int8_t beacon_snr_q4;    // dB x4
  uint8_t beacon_lost;     // beacons missed out of the last 64 frames
  int16_t sync_err_10us;   // measured beacon end - predicted, in 10 µs
  uint8_t n_nb;
  Neighbor nb[MAX_NEIGHBORS];
};
static const uint8_t HEALTH_FIXED_LEN = 11;

inline uint8_t encodeHealth(const Health& h, uint8_t* b, uint8_t cap) {
  const uint8_t n = h.n_nb > MAX_NEIGHBORS ? MAX_NEIGHBORS : h.n_nb;
  const uint8_t need = static_cast<uint8_t>(HEALTH_FIXED_LEN + 1 + 3 * n);
  if (cap < need) return 0;
  b[0] = static_cast<uint8_t>(h.battery_mv); b[1] = static_cast<uint8_t>(h.battery_mv >> 8);
  b[2] = static_cast<uint8_t>(h.uptime_min); b[3] = static_cast<uint8_t>(h.uptime_min >> 8);
  b[4] = h.fw; b[5] = static_cast<uint8_t>(h.beacon_rssi); b[6] = static_cast<uint8_t>(h.beacon_snr_q4);
  b[7] = h.beacon_lost;
  b[8] = static_cast<uint8_t>(h.sync_err_10us & 0xFF); b[9] = static_cast<uint8_t>((uint16_t)h.sync_err_10us >> 8);
  b[10] = 0;  // reserved
  b[11] = n;
  for (uint8_t i = 0; i < n; i++) {
    b[12 + 3 * i] = h.nb[i].hub;
    b[13 + 3 * i] = static_cast<uint8_t>(h.nb[i].rssi);
    b[14 + 3 * i] = static_cast<uint8_t>(h.nb[i].snr_q4);
  }
  return need;
}

inline bool decodeHealth(const uint8_t* b, uint8_t len, Health& h) {
  if (len < HEALTH_FIXED_LEN + 1) return false;
  h.battery_mv = static_cast<uint16_t>(b[0] | (b[1] << 8));
  h.uptime_min = static_cast<uint16_t>(b[2] | (b[3] << 8));
  h.fw = b[4]; h.beacon_rssi = static_cast<int8_t>(b[5]); h.beacon_snr_q4 = static_cast<int8_t>(b[6]);
  h.beacon_lost = b[7];
  h.sync_err_10us = static_cast<int16_t>(static_cast<uint16_t>(b[8] | (b[9] << 8)));
  h.n_nb = b[11];
  if (h.n_nb > MAX_NEIGHBORS || len < HEALTH_FIXED_LEN + 1 + 3 * h.n_nb) return false;
  for (uint8_t i = 0; i < h.n_nb; i++) {
    h.nb[i].hub = b[12 + 3 * i];
    h.nb[i].rssi = static_cast<int8_t>(b[13 + 3 * i]);
    h.nb[i].snr_q4 = static_cast<int8_t>(b[14 + 3 * i]);
  }
  return true;
}

// ---- section writer / reader -------------------------------------------------------------
class PacketWriter {
 public:
  PacketWriter(uint8_t* buf, size_t cap) : b_(buf), cap_(cap > MAX_PACKET ? MAX_PACKET : cap), len_(0) {}

  bool beginHub(uint8_t net, uint8_t src, uint16_t frame, uint8_t flags) {
    if (cap_ < HDR_LEN + 1) return false;
    putHeader(b_, net, PT_HUB, src, frame);
    b_[HDR_LEN] = flags;
    len_ = HDR_LEN + 1;
    return true;
  }
  bool beginJoin(uint8_t net, uint8_t src, uint16_t frame, uint8_t fw) {
    if (cap_ < HDR_LEN + 1) return false;
    putHeader(b_, net, PT_JOIN, src, frame);
    b_[HDR_LEN] = fw;
    len_ = HDR_LEN + 1;
    return true;
  }
  // Free bytes for the VALUE of one more section (2 B section header subtracted).
  size_t freeForValue() const { return (cap_ > len_ + 2) ? (cap_ - len_ - 2) : 0; }
  bool add(uint8_t type, const uint8_t* value, uint8_t n) {
    if (len_ + 2 + n > cap_) return false;
    b_[len_++] = type;
    b_[len_++] = n;
    if (n) memcpy(b_ + len_, value, n);
    len_ += n;
    return true;
  }
  void setFlags(uint8_t f) { if (len_ > HDR_LEN) b_[HDR_LEN] = f; }
  size_t size() const { return len_; }

 private:
  uint8_t* b_;
  size_t cap_;
  size_t len_;
};

class SectionReader {
 public:
  // body = bytes after the hub flags byte (i.e. packet + HDR_LEN + 1), len = their count
  SectionReader(const uint8_t* body, size_t len) : p_(body), n_(len), o_(0), bad_(false) {}
  bool next(uint8_t& type, const uint8_t*& value, uint8_t& len) {
    if (o_ + 2 > n_) { bad_ = (o_ != n_); return false; }
    type = p_[o_]; len = p_[o_ + 1];
    if (o_ + 2 + len > n_) { bad_ = true; return false; }
    value = p_ + o_ + 2;
    o_ += 2 + len;
    return true;
  }
  bool malformed() const { return bad_; }

 private:
  const uint8_t* p_;
  size_t n_;
  size_t o_;
  bool bad_;
};

}  // namespace tdma
}  // namespace icemesh
#endif
