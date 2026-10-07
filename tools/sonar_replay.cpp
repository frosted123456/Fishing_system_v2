// Replays a real-sonar recording (serial "REC ON" lines of the WROOM sonar node, D43) through the SAME
// processing as the node (lib/IceMesh SonarSource::process), on the PC. Tune the knobs here on your
// bucket / lake recordings, then set the winners from the chalet (Settings > Sonar or web Processing).
//   g++ -std=c++11 -O2 -Ilib/IceMesh/src tools/sonar_replay.cpp -o .preview/sonar_replay
//   .preview/sonar_replay [knob=value ...] log.txt            -> one CSV line per ping (or "< log.txt")
//   .preview/sonar_replay --trace [knob=value ...] < log.txt  -> JSON trace for tools/web_preview.py
//   .preview/sonar_replay --make-fake 200 > fake.txt           -> a fake recording (tests this tool)
// Line format: SONAR <ms> <nfreq> <rot_f> <t0> <edge_um> <raw_max> <us> <hex codes>... (488 bins each)
#include <sonar_sim.h>
#include <sonar_link.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
using namespace icemesh::sonar;

static int hexv(char c) { return c >= '0' && c <= '9' ? c - '0' : (c >= 'A' && c <= 'F' ? c - 'A' + 10 : (c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1)); }

int main(int argc, char** argv) {
  bool trace = false; int fake = 0;
  Params prm;
  std::ifstream file; std::istream* in = &std::cin;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--trace") trace = true;
    else if (a == "--make-fake" && i + 1 < argc) fake = atoi(argv[++i]);
    else {
      const size_t eq = a.find('=');
      bool okk = false;
      if (eq == std::string::npos && !file.is_open()) {   // a log file name instead of "< log.txt"
        file.open(a.c_str());
        if (file.is_open()) { in = &file; continue; }
      }
      if (eq != std::string::npos)
        for (uint8_t k = 0; k < P_COUNT; k++)
          if (a.substr(0, eq) == paramInfo(k).key) { prm.set(k, (uint8_t)atoi(a.c_str() + eq + 1)); okk = true; }
      if (!okk) { fprintf(stderr, "not a knob=value and not a readable file: %s (knobs: ", a.c_str()); for (uint8_t k = 0; k < P_COUNT; k++) fprintf(stderr, "%s ", paramInfo(k).key); fprintf(stderr, ")\n"); return 1; }
    }
  }
  if (fake > 0) {   // a recording made by the fake scene, same format as the node prints
    static SonarScene sc; sc.begin(SceneConfig::prototype());
    static uint8_t codes[sp::NFREQ][BINS];
    for (int k = 0; k < fake; k++) {
      sc.ping(codes);
      printf("SONAR %d 3 -1 40 -1 3000 52000", k * 250);
      for (int f = 0; f < 3; f++) { putchar(' '); for (int b = 0; b < BINS; b++) printf("%02X", codes[f][b]); }
      putchar('\n');
    }
    return 0;
  }
  static SonarSource src; src.begin(1, 0, 0); src.proc.prm = prm;
  static SonarStore<4, 4096> st;
  static uint8_t codes[sp::NFREQ][BINS];
  std::string line; long last_ms = -1; int n = 0, frame = 0; uint32_t last_seq = 0;
  if (!trace) printf("ms,nfreq,bottom_m,noise_db,bottom_snr_db,ring_m,ring_alarm,status,targets(depth_m:label:strength)\n");
  else printf("[\n");
  while (std::getline(*in, line)) {
    // the line may carry a monitor prefix ("12:00:01.123 > SONAR ...") and a Windows '\r'
    const size_t at = line.find("SONAR ");
    if (at == std::string::npos) continue;
    if (!line.empty() && line[line.size() - 1] == '\r') line.erase(line.size() - 1);
    std::istringstream is(line.substr(at + 6));
    long ms; int nf, rot, t0; long edge; unsigned rmax; unsigned long us;
    if (!(is >> ms >> nf >> rot >> t0 >> edge >> rmax >> us)) continue;
    if (nf < 1 || nf > 3) continue;
    bool bad = false;
    for (int f = 0; f < nf && !bad; f++) {
      std::string h; is >> h;
      if ((int)h.size() < 2 * BINS) { bad = true; break; }
      for (int b = 0; b < BINS; b++) { const int hi = hexv(h[2 * b]), lo = hexv(h[2 * b + 1]); if (hi < 0 || lo < 0) { bad = true; break; } codes[f][b] = (uint8_t)(hi * 16 + lo); }
    }
    if (bad) { fprintf(stderr, "skipped a bad line\n"); continue; }
    const float dt = last_ms >= 0 && ms > last_ms ? (ms - last_ms) / 1000.0f : 0.25f;
    last_ms = ms;
    Block out[2];
    const uint8_t nb = src.process(codes, (uint8_t)nf, (int8_t)rot, dt, false, out, 2);
    const SonarProc::Out& o = src.lastProc();
    n++;
    if (!trace) {
      printf("%ld,%d,%.3f,%.1f,%.1f,%.2f,%d,%u,", ms, nf, (double)o.bottom, (double)o.nf, (double)o.bottom_snr, (double)o.ring_m, o.ring_alarm ? 1 : 0, src.status());
      static const char* const L[] = {"fish", "bait", "nearbottom", "cover"};
      for (uint8_t k = 0; k < o.n; k++) printf("%s%.3f:%s:%.0f", k ? " " : "", (double)o.t[k].depth, L[o.t[k].label & 3], (double)o.t[k].s);
      printf("\n");
    } else {
      for (uint8_t k = 0; k < nb; k++) st.onSonarBlock(1, out[k].data, out[k].len, (uint16_t)frame);
      // focus-style trace: every ping, like tools/sonar_trace.cpp (pings of node 1)
      const Ping& p = src.lastPing();
      printf("%s{\"ms\":%ld,\"ping\":[%u,%u,%u,[", n > 1 ? "," : "", ms, (unsigned)++last_seq, p.index, p.bottom_cm);
      for (uint8_t j = 0; j < p.n_targets; j++) printf("%s[%u,%u,%u,%u]", j ? "," : "", p.t[j].track, p.t[j].depth_cm, p.t[j].strength, p.t[j].width);
      printf("],[");
      for (uint8_t j = 0; j < p.n_resid; j++) printf("%s[%u,%u]", j ? "," : "", p.r[j].bin, p.r[j].level);
      printf("],%u,%u,[]]}\n", o.hard, (unsigned)(-o.nf < 0 ? 0 : -o.nf));
      if (n % 4 == 0) frame++;
    }
  }
  if (trace) printf("]\n");
  fprintf(stderr, "%d pings replayed\n", n);
  return n > 0 ? 0 : 2;
}
