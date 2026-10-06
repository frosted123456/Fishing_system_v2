// Shared by test_sonar_proc (double: bit-for-bit port check) and test_sonar_proc_float (float, as on the ESP32).
#include <unity.h>
#include <sonar_proc.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>

using namespace icemesh::sonar;

struct RefTrack { int id; double depth, vel; int label; double v, s; int miss; };
struct RefPing { int k; double t, bottom, nf; int hard; double ratio; bool has_ratio; std::vector<RefTrack> tr; };
struct Ref { std::vector<std::vector<int> > D[12]; std::vector<RefPing> P; };

static bool loadRef(Ref& ref) {
  const char* path = getenv("PROTO_REF") ? getenv("PROTO_REF") : "fixtures/proto_ref.txt";
  std::ifstream in(path);
  if (!in) { printf("missing %s (node tools/proto_reference.js > test/fixtures/proto_ref.txt)\n", path); return false; }
  std::string line;
  for (int k = 0; k < 12; k++) ref.D[k].resize(3);
  while (std::getline(in, line)) {
    std::istringstream is(line); std::string tag; is >> tag;
    if (tag == "D") { int k, f; is >> k >> f; int x; while (is >> x) ref.D[k][f].push_back(x); }
    else if (tag == "P") {
      RefPing p; std::string ratio; int n;
      is >> p.k >> p.t >> p.bottom >> p.nf >> p.hard >> ratio >> n;
      p.has_ratio = ratio != "nan"; p.ratio = p.has_ratio ? atof(ratio.c_str()) : 0;
      for (int i = 0; i < n; i++) { RefTrack t; is >> t.id >> t.depth >> t.vel >> t.label >> t.v >> t.s >> t.miss; p.tr.push_back(t); }
      ref.P.push_back(p);
    }
  }
  return ref.P.size() > 100;
}
