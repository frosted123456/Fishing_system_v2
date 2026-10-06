// Host tool: records a fake-sonar trace with the REAL codec/store (lib/IceMesh) as the JSON the
// chalet serves (/api/sonar, /api/sonar/pings, /api/sonar/bg), one entry per second.
// Used to check the /sonar page in a desktop browser without hardware (tools/sonar_preview.py).
//   g++ -std=c++11 -O1 -Ilib/IceMesh/src tools/sonar_trace.cpp -o .preview/sonar_trace && .preview/sonar_trace 180 > .preview/trace.json
#include <sonar_sim.h>
#include <sonar_link.h>
#include <cstdio>
#include <cstdlib>
using namespace icemesh::sonar;

int main(int argc, char** argv) {
  const int secs = argc > 1 ? atoi(argv[1]) : 120;
  const uint8_t ids[4] = {136, 137, 144, 152};
  const uint8_t hubs[4] = {1, 1, 2, 3};
  const uint8_t focus = 144;
  SonarSource src[4];
  for (int i = 0; i < 4; i++) src[i].begin(ids[i], ids[i] * 7919u + hubs[i]);
  static SonarStore<16, 4096> st;
  uint32_t last_seq = 0;
  printf("[\n");
  for (int s = 0; s < secs; s++) {
    for (int t = 0; t < 4; t++)
      for (int i = 0; i < 4; i++) {
        Block out[2];
        const uint8_t n = src[i].tick(ids[i] == focus, out, 2);
        for (uint8_t k = 0; k < n; k++) st.onSonarBlock(hubs[i], out[k].data, out[k].len, static_cast<uint16_t>(s));
      }
    printf("%s{\"list\":{\"sim\":true,\"focus\":%u,\"frame\":%d,\"nodes\":[", s ? "," : "", focus, s);
    int first = 1;
    for (uint8_t i = 0; i < st.capacity(); i++) {
      const NodeSonar* ns = st.at(i);
      if (!ns) continue;
      printf("%s{\"node\":%u,\"hub\":%u,\"age\":%d,\"ping\":%u,\"bottom\":%u,\"fish\":%u,\"near\":%u,\"lvl\":%u,\"act\":%u,\"bgver\":%u,\"bgmask\":%u,\"sum\":%s}",
             first ? "" : ",", ns->node, ns->hub, s - ns->frame, ns->sum.ping, ns->sum.bottom_cm, ns->sum.n_targets, ns->sum.nearest_cm,
             ns->sum.nearest_level, ns->sum.activity, ns->bg_ver, ns->bg_mask, ns->has_sum ? "true" : "false");
      first = 0;
    }
    printf("],\"blocks_ok\":%u,\"blocks_bad\":%u},\"pings\":[", static_cast<unsigned>(st.blocks_ok), static_cast<unsigned>(st.blocks_bad));
    const StoredPing* out[64];
    const uint16_t n = st.pingsSince(focus, last_seq, out, 64);
    for (uint16_t k = 0; k < n; k++) {
      const Ping& p = out[k]->p;
      printf("%s[%u,%u,%u,[", k ? "," : "", static_cast<unsigned>(out[k]->seq), p.index, p.bottom_cm);
      for (uint8_t j = 0; j < p.n_targets; j++) printf("%s[%u,%u,%u,%u]", j ? "," : "", p.t[j].track, p.t[j].depth_cm, p.t[j].level, p.t[j].width);
      printf("],[");
      for (uint8_t j = 0; j < p.n_resid; j++) printf("%s[%u,%u]", j ? "," : "", p.r[j].bin, p.r[j].level);
      printf("]]");
      last_seq = out[k]->seq;
    }
    const NodeSonar* fn = st.find(focus);
    printf("],\"bg\":{\"node\":%u,\"ver\":%u,\"mask\":%u,\"bin_mm\":25,\"levels\":\"", focus, fn ? fn->bg_ver : 0, fn ? fn->bg_mask : 0);
    if (fn) for (uint16_t b = 0; b < BINS; b++) putchar('0' + (fn->bg[b] & 3));
    printf("\"}}\n");
  }
  printf("]\n");
  return 0;
}
