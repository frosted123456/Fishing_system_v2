// Hub side: builds the hub packet for its slot, in priority order, within the slot allowance:
//   1. LINE records (pending events first)   — always first
//   2. RELAY sections (remote hubs' packets heard in this frame)
//   3. HEALTH (every HEALTH_EVERY frames, or when requested)
//   4. JOINS heard (relay duty)
//   5. NODEINFO (rotating, battery per node)
//   6. TEST filler (radio test mode only)
// Also: applies beacon acks to the line table, and re-packs a remote hub packet for relaying.
#ifndef ICEMESH_HUB_PACKET_H
#define ICEMESH_HUB_PACKET_H
#include <stdint.h>
#include <string.h>
#include "tdma_proto.h"
#include "line_table.h"

namespace icemesh {
namespace tdma {

static const uint8_t MAX_RELAYED = 3;
static const uint8_t MAX_JOINS = 4;

struct RelayItem { uint8_t len; uint8_t data[MAX_PACKET]; };   // a full remote HUB packet as received

struct HubBuild {
  uint8_t network_id;
  uint8_t self;
  uint16_t frame;
  uint8_t flags;              // HubFlags (silence requests)
  uint8_t allowance;          // max packet length from the beacon
  const Health* health;       // nullptr = no health this frame
  const RelayItem* relayed;   // remote packets to forward
  uint8_t n_relayed;
  const uint8_t* joins;       // hub IDs heard in the JOIN slot
  uint8_t n_joins;
  uint8_t nodeinfo_start;     // rotation index
  uint8_t line_rot;           // rotation of non-pending line records
  bool test;                  // fill the rest with a TEST section
  uint16_t test_counter;
  uint8_t test_mode_seen;     // echoed in the TEST section
};

struct HubBuildResult { size_t len; uint8_t line_records; uint8_t line_total; uint8_t relayed_full; uint8_t relayed_partial; uint8_t nodeinfo_next; uint8_t line_rot_next; };

// Re-packs a remote hub packet into a RELAY section value: [orig hub][orig frame lo][orig flags][orig sections...]
// If the whole body does not fit in `max_value`, only its first section (LINE) is kept.
inline uint8_t packRelayValue(const uint8_t* pkt, uint8_t len, uint8_t* out, size_t max_value, bool& partial) {
  partial = false;
  if (len < HDR_LEN + 1 || max_value < 3) return 0;
  const uint8_t body_len = static_cast<uint8_t>(len - HDR_LEN - 1);
  out[0] = pkt[3]; out[1] = pkt[4]; out[2] = pkt[HDR_LEN];
  if (3u + body_len <= max_value) {
    memcpy(out + 3, pkt + HDR_LEN + 1, body_len);
    return static_cast<uint8_t>(3 + body_len);
  }
  SectionReader rd(pkt + HDR_LEN + 1, body_len);
  uint8_t t, n; const uint8_t* v;
  if (!rd.next(t, v, n) || t != SEC_LINE) return 0;
  uint8_t keep = static_cast<uint8_t>(n - n % LINE_RECORD_LEN);   // whole records only
  while (keep >= LINE_RECORD_LEN && 3u + 2u + keep > max_value) keep = static_cast<uint8_t>(keep - LINE_RECORD_LEN);
  if (keep < LINE_RECORD_LEN || 3u + 2u + keep > max_value) return 0;
  out[3] = SEC_LINE; out[4] = keep;
  memcpy(out + 5, v, keep);
  partial = true;
  return static_cast<uint8_t>(5 + keep);
}

template <uint8_t MAXN>
inline HubBuildResult buildHubPacket(const HubBuild& in, const LineTable<MAXN>& table, uint8_t* buf, size_t cap) {
  HubBuildResult r;
  memset(&r, 0, sizeof(r));
  const size_t limit = in.allowance < cap ? in.allowance : cap;
  PacketWriter w(buf, limit);
  if (!w.beginHub(in.network_id, in.self, in.frame, in.flags)) return r;
  uint8_t tmp[MAX_PACKET];

  // 1. line state
  LineRecord recs[MAXN];
  uint8_t pend = 0;
  const uint8_t total = table.records(recs, MAXN, in.line_rot, &pend);
  r.line_total = total;
  uint8_t fit = 0;
  while (fit < total && static_cast<size_t>(fit + 1) * LINE_RECORD_LEN <= w.freeForValue()) fit++;
  {
    const uint8_t np = table.nonPendingCount();
    const uint8_t sent_np = fit > pend ? static_cast<uint8_t>(fit - pend) : 0;
    r.line_rot_next = np ? static_cast<uint8_t>((in.line_rot + sent_np) % np) : 0;
  }
  for (uint8_t i = 0; i < fit; i++) encodeLine(recs[i], tmp + i * LINE_RECORD_LEN);
  bool any_pending = false;
  for (uint8_t i = 0; i < fit; i++) any_pending |= recs[i].pending;
  if (fit > 0) w.add(SEC_LINE, tmp, static_cast<uint8_t>(fit * LINE_RECORD_LEN));
  r.line_records = fit;
  w.setFlags(static_cast<uint8_t>(in.flags | (any_pending ? HF_PENDING : 0)));

  // 2. relayed remote packets
  for (uint8_t i = 0; i < in.n_relayed; i++) {
    bool partial = false;
    const uint8_t n = packRelayValue(in.relayed[i].data, in.relayed[i].len, tmp, w.freeForValue(), partial);
    if (n > 0 && w.add(SEC_RELAY, tmp, n)) { if (partial) r.relayed_partial++; else r.relayed_full++; }
  }
  // 3. health
  if (in.health != nullptr) {
    const size_t room = w.freeForValue();
    const uint8_t n = encodeHealth(*in.health, tmp, static_cast<uint8_t>(room > 255 ? 255 : room));
    if (n > 0) w.add(SEC_HEALTH, tmp, n);
  }
  // 4. joins
  if (in.n_joins > 0 && w.freeForValue() >= 1) {
    uint8_t n = in.n_joins > MAX_JOINS ? MAX_JOINS : in.n_joins;
    if (n > w.freeForValue()) n = static_cast<uint8_t>(w.freeForValue());
    w.add(SEC_JOINS, in.joins, n);
  }
  // 5. node info, rotating: 3 B per node {node, battery %, flags}
  r.nodeinfo_next = in.nodeinfo_start;
  {
    const uint8_t count = table.count();
    uint8_t n = 0;
    uint8_t idx = in.nodeinfo_start;
    while (count > 0 && n < count && static_cast<size_t>(n + 1) * 3 <= w.freeForValue() && n < 8) {
      uint8_t node = 0, pct = 0, fl = 0;
      if (!table.nodeAt(static_cast<uint8_t>(idx % count), node)) break;
      table.batteryOf(node, pct, fl);
      tmp[n * 3] = node; tmp[n * 3 + 1] = pct; tmp[n * 3 + 2] = fl;
      n++; idx++;
    }
    if (n > 0) { w.add(SEC_NODEINFO, tmp, static_cast<uint8_t>(n * 3)); r.nodeinfo_next = static_cast<uint8_t>(idx % (count ? count : 1)); }
  }
  // 6. test filler up to the allowance
  if (in.test && w.freeForValue() >= 3) {
    const size_t n = w.freeForValue() > 255 ? 255 : w.freeForValue();
    tmp[0] = static_cast<uint8_t>(in.test_counter); tmp[1] = static_cast<uint8_t>(in.test_counter >> 8);
    tmp[2] = in.test_mode_seen;
    for (size_t i = 3; i < n; i++) tmp[i] = static_cast<uint8_t>(0xA5 ^ i);
    w.add(SEC_TEST, tmp, static_cast<uint8_t>(n));
  }
  r.len = w.size();
  return r;
}

// Applies the acks of a beacon/echo addressed to this hub. Returns how many matched.
template <uint8_t MAXN>
inline uint8_t applyAcks(const Beacon& bc, uint8_t self, LineTable<MAXN>& table) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < bc.n_acks; i++)
    if (bc.acks[i].hub == self) { table.ack(bc.acks[i].node, bc.acks[i].seq); n++; }
  return n;
}

}  // namespace tdma
}  // namespace icemesh
#endif
