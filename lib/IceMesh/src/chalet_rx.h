// Chalet side: consumes one received HUB packet (direct, possibly carrying RELAY sections).
#ifndef ICEMESH_CHALET_RX_H
#define ICEMESH_CHALET_RX_H
#include <stdint.h>
#include "tdma_proto.h"
#include "chalet_nodes.h"
#include "chalet_planner.h"
#include "sonar_link.h"

namespace icemesh {
namespace tdma {

struct ChaletRxResult {
  bool ok;
  uint8_t hub;
  uint8_t flags;              // hub flags (silence requests, pending)
  uint8_t n_changes;
  NodeChange changes[16];
  uint8_t n_records;
  uint8_t n_relayed;
  uint16_t test_bytes;
  uint8_t n_sonar;            // sonar blocks handed to the sink
  bool malformed;
};

namespace detail {
template <uint8_t MH, uint8_t MN>
inline void consumeSections(uint8_t hub, const uint8_t* body, size_t len, uint16_t frame, bool allow_relay,
                            ChaletPlanner<MH>& planner, ChaletNodes<MN>& nodes, ChaletRxResult& out,
                            sonar::SonarSink* sink) {
  SectionReader rd(body, len);
  uint8_t t, n; const uint8_t* v;
  while (rd.next(t, v, n)) {
    switch (t) {
      case SEC_LINE:
        for (uint8_t i = 0; i + LINE_RECORD_LEN <= n; i = static_cast<uint8_t>(i + LINE_RECORD_LEN)) {
          LineRecord r;
          decodeLine(v + i, r);
          out.n_records++;
          if (r.pending) planner.addAck(hub, r.node, r.seq);
          NodeChange ch;
          if (nodes.onRecord(hub, r, frame, ch) && out.n_changes < 16) out.changes[out.n_changes++] = ch;
        }
        break;
      case SEC_HEALTH: {
        Health h;
        if (decodeHealth(v, n, h)) planner.onHealth(hub, h, frame);
        break;
      }
      case SEC_JOINS:
        for (uint8_t i = 0; i < n; i++) planner.onJoin(v[i], hub, 0, frame);
        break;
      case SEC_NODEINFO:
        for (uint8_t i = 0; i + 3 <= n; i = static_cast<uint8_t>(i + 3)) nodes.onNodeInfo(hub, v[i], v[i + 1], v[i + 2]);
        break;
      case SEC_RELAY:
        if (allow_relay && n >= 3 && v[0] != ID_NONE) {
          planner.onRelayed(v[0], hub, frame);
          out.flags = static_cast<uint8_t>(out.flags | (v[2] & (HF_SILENCE_ON | HF_SILENCE_OFF)));   // remote hub's requests
          out.n_relayed++;
          consumeSections(v[0], v + 3, n - 3u, frame, false, planner, nodes, out, sink);   // one hop only
        }
        break;
      case SEC_SONAR:
        if (sink != nullptr && n >= 2) { sink->onSonarBlock(hub, v, n, frame); out.n_sonar++; }
        break;
      case SEC_TEST:
        out.test_bytes = static_cast<uint16_t>(out.test_bytes + n);
        break;
      default:
        break;   // unknown / reserved sections are skipped (forward compatible)
    }
  }
  if (rd.malformed()) out.malformed = true;
}
}  // namespace detail

template <uint8_t MH, uint8_t MN>
inline ChaletRxResult consumeHubPacket(const uint8_t* pkt, size_t len, uint8_t net, uint16_t frame,
                                      ChaletPlanner<MH>& planner, ChaletNodes<MN>& nodes,
                                      sonar::SonarSink* sink = nullptr) {
  ChaletRxResult out;
  memset(&out, 0, sizeof(out));
  Header h;
  if (!readHeader(pkt, len, net, h) || h.type != PT_HUB || len < HDR_LEN + 1) return out;
  out.ok = true;
  out.hub = h.src;
  out.flags = pkt[HDR_LEN];
  detail::consumeSections(h.src, pkt + HDR_LEN + 1, len - HDR_LEN - 1, frame, true, planner, nodes, out, sink);
  return out;
}

}  // namespace tdma
}  // namespace icemesh
#endif
