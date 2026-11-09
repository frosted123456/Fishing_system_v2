// Host tool (D101): replay a hub bench recording (bench page "Save log", SONAR/KNOBS lines) through the WHOLE chain:
// SonarProc -> SonarSource payload -> codec blocks -> SonarStore (the chalet) -> what the /sonar page gets. Prints JSON:
// per ping the raw dB codes, the processor bottom/tracks, then "chalet": pings / raw echogram lines / background.
//   g++ -std=c++11 -O1 -Ilib/IceMesh/src -Itest/shim tools/sonar_chain.cpp -o .preview/sonar_chain
//   .preview/sonar_chain log/rec.txt [knob=val ...] > .preview/chain.json && CHAIN=.preview/chain.json python tools/web_preview.py
// -DSONAR_DEBUG_BOTTOM prints the bottom decision per ping on stderr; DBG=1 in the environment prints the track gates.
#include <sonar_sim.h>
#include <sonar_link.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream>
using namespace icemesh::sonar;

int main(int argc, char** argv) {
  static SonarSource src; static SonarStore<4, 256, 16, 250> store; static Block blk[4];
  src.begin(1, 0xF15A0001UL, 0); src.echo_on = true; if (getenv("ECHO_EVERY")) src.echo_every = (uint8_t)atoi(getenv("ECHO_EVERY"));   // D107: 1 = the hub on ESP-NOW
  Params over; bool ov[P_COUNT] = {false};
  for (int a = 2; a < argc; a++) { const char* e = strchr(argv[a], '='); if (!e) continue; std::string k(argv[a], e - argv[a]); for (uint8_t i = 0; i < P_COUNT; i++) if (k == paramInfo(i).key) { ov[i] = true; over.v[i] = (uint8_t)atoi(e + 1); } }
  std::ifstream in(argv[1]); std::string line; int n = 0; uint16_t frame = 0;
  static uint8_t codes[sp::NFREQ][BINS];
  printf("{\"pings\":[");
  bool first = true; unsigned long t0ms = 0;
  while (std::getline(in, line)) {
    if (line.compare(0, 6, "KNOBS ") == 0) {
      std::istringstream ss(line.substr(6)); std::string tok; ss >> tok;
      while (ss >> tok) { const size_t e = tok.find('='); if (e == std::string::npos) continue; for (uint8_t i = 0; i < P_COUNT; i++) if (tok.substr(0, e) == paramInfo(i).key) src.proc.prm.set(i, (uint8_t)atoi(tok.c_str() + e + 1)); }
      for (uint8_t i = 0; i < P_COUNT; i++) if (ov[i]) src.proc.prm.v[i] = over.v[i];
      continue;
    }
    if (line.compare(0, 6, "SONAR ") != 0) continue;
    std::istringstream ss(line.substr(6)); unsigned long ms; int nf, rot, t0; long edge; unsigned rm; unsigned long us;
    ss >> ms >> nf >> rot >> t0 >> edge >> rm >> us;
    if (!t0ms) t0ms = ms;
    std::string hex; int f = 0;
    while (ss >> hex && f < sp::NFREQ) { if (hex.size() < 2 * BINS) break; for (int b = 0; b < BINS; b++) codes[f][b] = (uint8_t)strtoul(hex.substr(2 * b, 2).c_str(), nullptr, 16); f++; }
    if (f == 0) continue;
    const uint8_t nb = src.process(codes, (uint8_t)f, -1, 1.0f, true, blk, 4);   // ~1 ping/s in these files: dt 1 s
    for (uint8_t k = 0; k < nb; k++) store.onSonarBlock(1, blk[k].data, blk[k].len, frame);
    frame++;
    n++;
    const SonarProc::Out& o = src.lastProc();
    printf("%s{\"i\":%d,\"t\":%.1f,\"raw\":[", first ? "" : ",", n, (ms - t0ms) / 1000.0); first = false;
    for (int b = 0; b < BINS; b++) printf("%s%d", b ? "," : "", codes[f == 3 ? 1 : 0][b]);
    printf("],\"bottom\":%.3f,\"snr\":%.1f,\"nf\":%.1f,\"ring\":%.3f,\"tracks\":[", (double)o.bottom, (double)o.bottom_snr, (double)o.nf, (double)o.ring_m);
    for (uint8_t k = 0; k < o.n; k++) printf("%s[%.3f,%u]", k ? "," : "", (double)o.t[k].depth, (unsigned)o.t[k].label);
    printf("]");
    if (getenv("DBG")) { const Ping& lp = src.lastPing(); fprintf(stderr, "ping %d sent=%u", n, lp.n_targets); for (uint8_t k = 0; k < o.n; k++) { const int b = (int)(o.t[k].depth / 0.025f + 0.5f); fprintf(stderr, "  [%.2f L%u miss%u v=%.2f chg=%.2f st=%.1f d=%.1f]", (double)o.t[k].depth, o.t[k].label, o.t[k].miss, (double)o.t[k].v, SonarProc::binChanged()[b], src.proc.binStaticDb()[b], SonarProc::binDb()[b]); fprintf(stderr, " sig=%.1f", src.proc.binSig()[b]); } fprintf(stderr, "\n"); }
    printf("}");
  }
  printf("],\"chalet\":{\"pings\":[");
  { const StoredPing* out[256]; const uint16_t m = store.pingsSince(1, 0, out, 256);
    for (uint16_t i = 0; i < m; i++) { const Ping& p = out[i]->p; printf("%s{\"index\":%u,\"bottom_cm\":%u,\"t\":[", i ? "" : "", p.index, p.bottom_cm);
      for (uint8_t k = 0; k < p.n_targets; k++) printf("%s[%u,%u,%u,%u]", k ? "," : "", p.t[k].track, p.t[k].depth_cm, p.t[k].strength, p.t[k].width);
      printf("],\"r\":["); for (uint8_t k = 0; k < p.n_resid; k++) printf("%s[%u,%u]", k ? "," : "", p.r[k].bin, p.r[k].level); printf("]}%s", i + 1 < m ? "," : ""); } }
  printf("],\"echo\":[");
  { const StoredEcho* out[250]; const uint16_t m = store.echoSince(1, 0, out, 250);
    for (uint16_t i = 0; i < m; i++) { printf("%s{\"index\":%u,\"nf\":%u,\"c\":\"", i ? "," : "", out[i]->e.index, out[i]->e.nf_neg); for (uint8_t c = 0; c < ECHO_CELLS; c++) putchar("0123456789ABCDEF"[echoCell(out[i]->e, c)]); printf("\",\"k\":%u,\"s\":%u,\"m\":%u}", out[i]->e.scale, out[i]->e.sharp, out[i]->e.marks); }
    fprintf(stderr, "echo lines: %u bytes sent\n", (unsigned)src.echo_bytes); }
  printf("],\"bg\":\"");
  { const NodeSonar* ns = store.find(1); if (ns) for (int b = 0; b < BINS; b++) putchar('0' + (ns->bg[b] & 3)); }
  printf("\",\"bg_mask\":%u},\"knobs\":{", store.find(1) ? store.find(1)->bg_mask : 0);
  for (uint8_t i = 0; i < P_COUNT; i++) printf("%s\"%s\":%u", i ? "," : "", paramInfo(i).key, src.proc.prm.v[i]);
  printf("}}\n");
  return 0;
}
