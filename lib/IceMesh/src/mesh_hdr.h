// v2 common LoRa header (docs/protocol_v2.md §2). 8 bytes, little-endian.
//
//  off size field
//   0   1   network_id
//   1   1   origin       creator of the payload, never rewritten (1..254)
//   2   1   ver          0xF0 | version  (0xF2)
//   3   1   ctl          [7:6] class  [5:4] hops  [3] ack_req  [2:0] reserved (sent as 0, ignored on RX)
//   4   2   seq          per (origin, class)
//   6   1   sender       last transmitter, rewritten each hop
//   7   1   type         payload type
#ifndef ICEMESH_MESH_HDR_H
#define ICEMESH_MESH_HDR_H
#include <stddef.h>
#include <stdint.h>

namespace icemesh {

static const uint8_t PROTO_VERSION = 2;
static const uint8_t VER_BYTE = 0xF0 | PROTO_VERSION;  // 0xF2
static const uint8_t HDR_LEN = 8;
static const uint8_t MAX_HOPS = 3;                     // fits the 2-bit hop field
static const uint8_t MAX_FRAME = 255;                  // SX1262 max payload
static const uint8_t MAX_PAYLOAD = MAX_FRAME - HDR_LEN;
static const uint8_t ID_NONE = 0;                      // reserved node IDs
static const uint8_t ID_BROADCAST = 255;

enum TrafficClass : uint8_t {
  CLASS_RELIABLE = 0,
  CLASS_STREAM = 1,
  CLASS_LINK = 2,  // never relayed
};

struct Header {
  uint8_t network_id;
  uint8_t origin;
  TrafficClass cls;
  uint8_t hops;
  bool ack_req;
  uint16_t seq;
  uint8_t sender;
  uint8_t type;
};

enum DecodeResult : uint8_t {
  DECODE_OK = 0,
  DECODE_SHORT,     // fewer than HDR_LEN bytes
  DECODE_NETWORK,   // other network_id
  DECODE_VERSION,   // not a v2 packet (v1 packet, other protocol, other version)
  DECODE_CLASS,     // class 3 (reserved)
  DECODE_HOPS,      // LINK class with hops != 0
  DECODE_ORIGIN,    // origin 0 or 255
};

inline uint8_t packCtl(TrafficClass cls, uint8_t hops, bool ack_req) {
  return static_cast<uint8_t>(((cls & 0x03) << 6) | ((hops & 0x03) << 4) | (ack_req ? 0x08 : 0x00));
}

// Writes the header into buf. Returns HDR_LEN, or 0 if the header is invalid or cap < HDR_LEN.
inline size_t encodeHeader(const Header& h, uint8_t* buf, size_t cap) {
  if (buf == nullptr || cap < HDR_LEN) return 0;
  if (h.cls > CLASS_LINK || h.hops > MAX_HOPS) return 0;
  if (h.origin == ID_NONE || h.origin == ID_BROADCAST) return 0;
  if (h.cls == CLASS_LINK && h.hops != 0) return 0;
  buf[0] = h.network_id;
  buf[1] = h.origin;
  buf[2] = VER_BYTE;
  buf[3] = packCtl(h.cls, h.hops, h.ack_req);
  buf[4] = static_cast<uint8_t>(h.seq & 0xFF);
  buf[5] = static_cast<uint8_t>(h.seq >> 8);
  buf[6] = h.sender;
  buf[7] = h.type;
  return HDR_LEN;
}

inline DecodeResult decodeHeader(const uint8_t* buf, size_t len, uint8_t expected_network, Header& out) {
  if (buf == nullptr || len < HDR_LEN) return DECODE_SHORT;
  if (buf[0] != expected_network) return DECODE_NETWORK;
  if (buf[2] != VER_BYTE) return DECODE_VERSION;
  const uint8_t cls = static_cast<uint8_t>(buf[3] >> 6);
  if (cls > CLASS_LINK) return DECODE_CLASS;
  const uint8_t hops = static_cast<uint8_t>((buf[3] >> 4) & 0x03);
  if (cls == CLASS_LINK && hops != 0) return DECODE_HOPS;
  if (buf[1] == ID_NONE || buf[1] == ID_BROADCAST) return DECODE_ORIGIN;
  out.network_id = buf[0];
  out.origin = buf[1];
  out.cls = static_cast<TrafficClass>(cls);
  out.hops = hops;
  out.ack_req = (buf[3] & 0x08) != 0;
  out.seq = static_cast<uint16_t>(buf[4] | (buf[5] << 8));
  out.sender = buf[6];
  out.type = buf[7];
  return DECODE_OK;
}

// Prepares an already-decoded packet for relaying, in place: hops + 1, sender = self.
// Returns false (packet must not be relayed) for LINK class or when hops would exceed MAX_HOPS.
inline bool prepareRelay(uint8_t* buf, size_t len, uint8_t self_id) {
  if (buf == nullptr || len < HDR_LEN) return false;
  const uint8_t cls = static_cast<uint8_t>(buf[3] >> 6);
  if (cls == CLASS_LINK || cls > CLASS_LINK) return false;
  const uint8_t hops = static_cast<uint8_t>((buf[3] >> 4) & 0x03);
  if (hops >= MAX_HOPS) return false;
  buf[3] = static_cast<uint8_t>((buf[3] & ~0x30) | ((hops + 1) << 4));
  buf[6] = self_id;
  return true;
}

}  // namespace icemesh
#endif
