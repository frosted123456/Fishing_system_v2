// ESP-NOW backbone (EB): a second path between hubs and the chalet that does not use LoRa, for when the
// LoRa channel is too busy. ESP-NOW broadcast on the Wi-Fi channel, up to EB_MAX_HOPS relays: any device
// with the relay setting on (hub, spare ESP32, tip-up node kept awake) rebroadcasts frames it has not seen.
//
// Frame: [0] network id  [1] sender (this hop)  [2] type  [3] hops so far  [4] origin id  [5..6] origin seq
//        [7..] payload.  Byte 2 is the message type, like every ESP-NOW message of the system.
//   MSG_EB_BEACON (0x70): chalet beacon without slots (tdma_proto.h): acks, flags, commands, network config
//   MSG_EB_HUB    (0x71): hub packet (tdma_proto.h), the same bytes a hub sends in its LoRa slot
//
// TransportPolicy decides per hub whether to use LoRa, the backbone, or both, from the chalet's setting
// (Auto / LoRa / ESP-NOW) and what the hub actually hears, so a hub never needs reprogramming on the ice.
#ifndef ICEMESH_EB_LINK_H
#define ICEMESH_EB_LINK_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "seq.h"
#include "tdma_proto.h"

namespace icemesh {
namespace eb {

static const uint8_t MSG_EB_BEACON = 0x70;
static const uint8_t MSG_EB_HUB = 0x71;
static const uint8_t HDR = 7;
static const uint8_t MAX_FRAME = 250;          // ESP-NOW payload limit (v1 API)
static const uint8_t MAX_PAYLOAD = MAX_FRAME - HDR;
static const uint8_t MAX_HOPS = 2;

struct Header { uint8_t net, sender, type, hops, origin; uint16_t seq; };

inline size_t encode(const Header& h, const uint8_t* payload, size_t n, uint8_t* out, size_t cap) {
  if (n > MAX_PAYLOAD || cap < HDR + n) return 0;
  out[0] = h.net; out[1] = h.sender; out[2] = h.type; out[3] = h.hops; out[4] = h.origin;
  out[5] = static_cast<uint8_t>(h.seq & 0xFF); out[6] = static_cast<uint8_t>(h.seq >> 8);
  if (n) memcpy(out + HDR, payload, n);
  return HDR + n;
}

inline bool decode(const uint8_t* in, size_t len, uint8_t net, Header& h, const uint8_t*& payload, size_t& n) {
  if (in == nullptr || len < HDR + 1 || len > MAX_FRAME || in[0] != net) return false;
  if (in[2] != MSG_EB_BEACON && in[2] != MSG_EB_HUB) return false;
  h.net = in[0]; h.sender = in[1]; h.type = in[2]; h.hops = in[3]; h.origin = in[4];
  h.seq = static_cast<uint16_t>(in[5] | (in[6] << 8));
  if (h.origin == tdma::ID_NONE || h.hops > MAX_HOPS) return false;
  payload = in + HDR; n = len - HDR;
  return true;
}

// Relay: rewrite sender and hop count in place. False when the frame must not go further.
inline bool prepareRelay(uint8_t* frame, size_t len, uint8_t self) {
  if (len < HDR || frame[3] >= MAX_HOPS || frame[4] == self) return false;
  frame[1] = self; frame[3] = static_cast<uint8_t>(frame[3] + 1);
  return true;
}

// Recently seen (origin, type, seq): drops copies arriving through several relays.
class Dedup {
 public:
  enum { CAP = 48 };
  Dedup() { memset(e_, 0, sizeof(e_)); head_ = 0; }
  // true = already seen (and it is not recorded again)
  bool seen(uint8_t origin, uint8_t type, uint16_t seq) {
    for (uint8_t i = 0; i < CAP; i++) if (e_[i].used && e_[i].origin == origin && e_[i].type == type && e_[i].seq == seq) return true;
    E& x = e_[head_]; x.used = true; x.origin = origin; x.type = type; x.seq = seq;
    head_ = static_cast<uint8_t>((head_ + 1) % CAP);
    return false;
  }
 private:
  struct E { bool used; uint8_t origin, type; uint16_t seq; };
  E e_[CAP];
  uint8_t head_;
};

// Per hub: which path(s) to transmit on.
//   TR_LORA   : LoRa only; safety net: the backbone too after SAFETY_MS without any LoRa beacon
//               (covers a hub that missed the switch to ESP-NOW).
//   TR_ESPNOW : backbone only; safety net: LoRa too after SAFETY_MS without any backbone beacon.
//   TR_AUTO   : LoRa, plus the backbone after FALLBACK_MS without a LoRa beacon, until RETURN_BEACONS
//               LoRa beacons in a row came back.
// Sending on both is safe: the chalet keeps one state per node and drops repeated sonar pings.
//
// SETUP PHASE: after power-on the automatic fallbacks (Auto -> backbone, LoRa-only safety net) are not
// armed: devices are switched on one by one while the holes are prepared, and a missing chalet then is
// normal, not a failure. They arm after ARM_MS of LoRa beacons without a break longer than FALLBACK_MS
// (the network ran complete for 5 min), and stay armed until the next reboot. ESP-NOW only, set by the
// user, is never gated (explicit choice). A hub that has not heard a single LoRa beacon since power-on
// is still searching: it also sends on the backbone (a chalet set to ESP-NOW only, or out of LoRa
// reach, still gets it); that stops at the first LoRa beacon and is not a fallback.
class TransportPolicy {
 public:
  enum { FALLBACK_MS = 10000, SAFETY_MS = 60000, RETURN_BEACONS = 5, STREAK_GAP_MS = 3500, ARM_MS = 300000 };

  void begin(uint32_t now_ms) { last_lora_ = last_eb_ = now_ms; streak_ = 0; eb_active_ = false; started_ = true; stable_ = false; armed_ = false; heard_lora_ = false; }
  void onLoraBeacon(uint32_t now_ms) {
    if (!started_) begin(now_ms);
    heard_lora_ = true;
    if (!stable_ || now_ms - last_lora_ > FALLBACK_MS) { stable_ = true; stable_since_ = now_ms; }   // (re)start the arming run
    streak_ = (now_ms - last_lora_ <= STREAK_GAP_MS) ? static_cast<uint16_t>(streak_ + 1) : 1;
    last_lora_ = now_ms;
    if (!armed_ && now_ms - stable_since_ >= ARM_MS) armed_ = true;
    if (eb_active_ && streak_ >= RETURN_BEACONS) eb_active_ = false;
  }
  void onEbBeacon(uint32_t now_ms) { if (!started_) begin(now_ms); last_eb_ = now_ms; }
  void tick(uint32_t now_ms) {
    if (!started_) begin(now_ms);
    if (armed_ && now_ms - last_lora_ > FALLBACK_MS) { eb_active_ = true; streak_ = 0; }
  }
  void arm() { armed_ = true; }   // user: "setup done" (and tests)
  bool useLora(uint8_t mode, uint32_t now_ms) const {
    if (mode == tdma::TR_ESPNOW) return now_ms - last_eb_ > SAFETY_MS;
    return true;
  }
  bool useEb(uint8_t mode, uint32_t now_ms) const {
    if (mode == tdma::TR_ESPNOW) return true;
    if (mode == tdma::TR_LORA) return armed_ && now_ms - last_lora_ > SAFETY_MS;
    return eb_active_ || !heard_lora_;
  }
  bool armed() const { return armed_; }
  // seconds until the fallbacks arm (0 = armed; ARM_MS/1000 while no beacon run is going)
  uint32_t armInS(uint32_t now_ms) const {
    if (armed_) return 0;
    if (!stable_ || now_ms - last_lora_ > FALLBACK_MS) return ARM_MS / 1000;
    const uint32_t run = now_ms - stable_since_;
    return run >= ARM_MS ? 0 : (ARM_MS - run + 999) / 1000;
  }
  bool fallbackActive() const { return eb_active_; }
  bool searching() const { return !heard_lora_; }
  uint32_t msSinceLora(uint32_t now_ms) const { return now_ms - last_lora_; }
  uint32_t msSinceEb(uint32_t now_ms) const { return now_ms - last_eb_; }

 private:
  uint32_t last_lora_ = 0, last_eb_ = 0, stable_since_ = 0;
  uint16_t streak_ = 0;
  bool eb_active_ = false, started_ = false, stable_ = false, armed_ = false, heard_lora_ = false;
};

}  // namespace eb
}  // namespace icemesh
#endif
