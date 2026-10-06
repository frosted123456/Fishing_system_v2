// Sonar codec on pings from the prototype chain (scene -> processing): round trips, escapes,
// truncation, fuzz, measured sizes.
#include <unity.h>
#include <sonar_sim.h>
#include <cstdio>
#include <cstdlib>

using namespace icemesh;
using namespace icemesh::sonar;

void setUp(void) {}
void tearDown(void) {}

static bool samePing(const Ping& a, const Ping& b) {
  if (a.index != b.index || a.bottom_cm != b.bottom_cm || a.n_targets != b.n_targets || a.n_resid != b.n_resid) return false;
  for (uint8_t i = 0; i < a.n_targets; i++)
    if (a.t[i].track != b.t[i].track || a.t[i].depth_cm != b.t[i].depth_cm || a.t[i].strength != b.t[i].strength) return false;
  for (uint8_t i = 0; i < a.n_resid; i++) if (a.r[i].bin != b.r[i].bin || a.r[i].level != b.r[i].level) return false;
  return true;
}
static DataHeader hdr(uint8_t node) { DataHeader h; memset(&h, 0, sizeof h); h.node = node; h.nf_neg = 84; h.bg_ver = 3; h.hard = BH_SOFT; return h; }

// pings from the real chain (prototype scene + processing)
struct Chain {
  SonarSource src;
  explicit Chain(uint32_t seed) { src.begin(9, seed); }
  Ping next() { Block b[2]; src.tick(false, b, 2); return src.lastPing(); }
};

static void test_bitstream(void) {
  uint8_t buf[16];
  BitWriter w(buf, sizeof buf);
  w.put(5, 3); w.putSigned(-3, 4); w.putRice(37, 3); w.put(0xABCD, 16);
  const size_t at = w.bits(); w.put(0, 4); w.patch(at, 9, 4);
  const size_t mark = w.bits(); w.put(0x3FF, 10); w.rewind(mark);
  TEST_ASSERT_FALSE(w.overflow());
  BitReader r(buf, w.bytes());
  TEST_ASSERT_EQUAL(5, r.get(3));
  TEST_ASSERT_EQUAL(-3, r.getSigned(4));
  TEST_ASSERT_EQUAL(37, r.getRice(3));
  TEST_ASSERT_EQUAL(0xABCD, r.get(16));
  TEST_ASSERT_EQUAL(9, r.get(4));
  TEST_ASSERT_FALSE(r.bad());
  uint8_t small[1]; BitWriter w2(small, 1); w2.put(0x1FF, 9);
  TEST_ASSERT_TRUE(w2.overflow());
}

static void test_summary_roundtrip(void) {
  Summary s; memset(&s, 0, sizeof s);
  s.node = 137; s.activity = 9; s.ping = 65530; s.bottom_cm = 612; s.hard = BH_MEDIUM; s.n_targets = 4; s.bg_ver = 7;
  // v3: every target of the latest ping (bait first here), nearest is derived on decode
  const uint16_t dep[5] = {400, 540, 120, 590, 300}; const uint8_t lv[5] = {2, 3, 1, 2, 1};
  for (int k = 0; k < 5; k++) { s.list[k].depth_cm = dep[k]; s.list[k].level = lv[k]; s.list[k].track = k == 0 ? 0 : 1; }
  s.n_list = 5;
  uint8_t b[24];
  const size_t n = encodeSummary(s, b, sizeof b);
  TEST_ASSERT_EQUAL(16, n);   // 55 + 5 x 14 = 125 bits
  Summary d;
  TEST_ASSERT_TRUE(decodeSummary(b, n, d));
  TEST_ASSERT_EQUAL(137, d.node); TEST_ASSERT_EQUAL(9, d.activity); TEST_ASSERT_EQUAL(65530, d.ping);
  TEST_ASSERT_EQUAL(612, d.bottom_cm); TEST_ASSERT_EQUAL(BH_MEDIUM, d.hard); TEST_ASSERT_EQUAL(4, d.n_targets); TEST_ASSERT_EQUAL(7, d.bg_ver);
  TEST_ASSERT_EQUAL(5, d.n_list);
  for (int k = 0; k < 5; k++) { TEST_ASSERT_EQUAL(dep[k], d.list[k].depth_cm); TEST_ASSERT_EQUAL(lv[k], d.list[k].level); TEST_ASSERT_EQUAL(k == 0 ? 0 : 1, d.list[k].track); }
  TEST_ASSERT_EQUAL(300, d.nearest_cm); TEST_ASSERT_EQUAL(1, d.nearest_level);   // closest fish to the bait at 400 cm
  s.n_targets = 0; s.n_list = 0;
  const size_t n0 = encodeSummary(s, b, sizeof b);
  TEST_ASSERT_EQUAL(7, n0);   // 55 bits: no target
  TEST_ASSERT_TRUE(decodeSummary(b, n0, d));
  TEST_ASSERT_EQUAL(DEPTH_NONE, d.nearest_cm);
  TEST_ASSERT_FALSE(decodeSummary(b, n0 - 1, d));   // truncated
}

static void test_data_roundtrip_chain(void) {
  int with_targets = 0, max_t = 0;
  for (uint32_t seed = 0; seed <= 3; seed++) {
    Chain ch(seed);
    for (int blk = 0; blk < 120; blk++) {
      Ping p[4];
      for (int k = 0; k < 4; k++) { p[k] = ch.next(); if (p[k].n_targets) with_targets++; if (p[k].n_targets > max_t) max_t = p[k].n_targets; }
      TrackInfo info[2]; info[0].track = 0; info[0].flick_q = 3; info[0].spread_q = 1; info[0].elen = 4; info[0].mature = true;
      info[1].track = 5; info[1].flick_q = 15; info[1].spread_q = 9; info[1].elen = 16; info[1].mature = false;
      DataHeader h = hdr(static_cast<uint8_t>(seed)); h.info = info; h.n_info = (blk & 1) ? 2 : 0;
      uint8_t b[MAX_BLOCK]; uint8_t used = 0;
      const size_t n = encodeData(h, p, 4, b, sizeof b, &used);
      TEST_ASSERT_TRUE(n > 0);
      TEST_ASSERT_EQUAL(4, used);
      DataBlock d;
      TEST_ASSERT_TRUE(decodeData(b, n, d));
      TEST_ASSERT_EQUAL(4, d.n); TEST_ASSERT_EQUAL(seed, d.node); TEST_ASSERT_EQUAL(84, d.nf_neg); TEST_ASSERT_EQUAL(BH_SOFT, d.hard);
      TEST_ASSERT_EQUAL(h.n_info, d.n_info);
      if (d.n_info) { TEST_ASSERT_EQUAL(5, d.info[1].track); TEST_ASSERT_EQUAL(15, d.info[1].flick_q); TEST_ASSERT_EQUAL(16, d.info[1].elen); TEST_ASSERT_FALSE(d.info[1].mature); }
      for (int k = 0; k < 4; k++) TEST_ASSERT_TRUE(samePing(p[k], d.pings[k]));
    }
  }
  printf("chain pings: %d of 1920 with targets, up to %d targets in one ping\n", with_targets, max_t);
  TEST_ASSERT_TRUE(with_targets > 1500);
}

static void test_data_escapes(void) {
  Ping p[4]; memset(p, 0, sizeof p);
  for (int k = 0; k < 4; k++) p[k].index = static_cast<uint16_t>(65534 + k);   // index wraps inside the block
  p[0].bottom_cm = 500; p[1].bottom_cm = 530; p[2].bottom_cm = DEPTH_NONE; p[3].bottom_cm = 498;   // jump, lost, back
  p[0].n_targets = 1; p[0].t[0].track = 0; p[0].t[0].depth_cm = 400; p[0].t[0].strength = 15; p[0].t[0].width = 1;
  p[1].n_targets = 2; p[1].t[0] = p[0].t[0]; p[1].t[0].depth_cm = 330;            // -70 cm: escape
  p[1].t[1].track = 5; p[1].t[1].depth_cm = 200; p[1].t[1].strength = 31; p[1].t[1].width = 4;   // appears mid-block
  p[2].n_targets = 1; p[2].t[0] = p[1].t[1]; p[2].t[0].depth_cm = 231;            // track 0 gone, 5 +31
  p[3].n_targets = 5;
  for (int k = 0; k < 5; k++) { p[3].t[k].track = static_cast<uint8_t>(k + 1); p[3].t[k].depth_cm = static_cast<uint16_t>(100 + 50 * k); p[3].t[k].strength = static_cast<uint8_t>(6 * k + 1); p[3].t[k].width = 2; }
  p[3].n_resid = 3; p[3].r[0].bin = 0; p[3].r[0].level = 1; p[3].r[1].bin = 1; p[3].r[1].level = 2; p[3].r[2].bin = 487; p[3].r[2].level = 3;
  uint8_t b[MAX_BLOCK]; uint8_t used = 0;
  const size_t n = encodeData(hdr(9), p, 4, b, sizeof b, &used);
  DataBlock d;
  TEST_ASSERT_TRUE(decodeData(b, n, d));
  for (int k = 0; k < 4; k++) TEST_ASSERT_TRUE(samePing(p[k], d.pings[k]));
  TEST_ASSERT_EQUAL(1, d.pings[3].index);
  TEST_ASSERT_EQUAL(4, d.pings[2].t[0].width);    // width carried from the track's first appearance
  TEST_ASSERT_EQUAL(3, d.pings[1].t[1].level);    // level derived from strength 31
  TEST_ASSERT_EQUAL(1, d.pings[3].t[0].level);    // strength 1
}

static void test_data_fits_cap(void) {
  Chain ch(2);
  Ping p[MAX_PINGS];
  for (int k = 0; k < 40; k++) ch.next();
  for (int k = 0; k < MAX_PINGS; k++) p[k] = ch.next();
  TrackInfo info[5]; memset(info, 0, sizeof info);
  DataHeader h = hdr(1); h.info = info; h.n_info = 5;
  uint8_t b[24]; uint8_t used = 0;
  const size_t n = encodeData(h, p, MAX_PINGS, b, sizeof b, &used);
  TEST_ASSERT_TRUE(n > 0 && n <= sizeof b);
  TEST_ASSERT_TRUE(used >= 1 && used < MAX_PINGS);
  DataBlock d;
  TEST_ASSERT_TRUE(decodeData(b, n, d));
  TEST_ASSERT_EQUAL(used, d.n);
  for (int k = 0; k < used; k++) TEST_ASSERT_TRUE(samePing(p[k], d.pings[k]));
  uint8_t tiny[4];
  TEST_ASSERT_EQUAL(0, encodeData(h, p, 1, tiny, sizeof tiny, &used));
}

static void test_truncation_detected(void) {
  Chain ch(1);
  for (int blk = 0; blk < 150; blk++) {
    Ping p[4];
    for (int k = 0; k < 4; k++) p[k] = ch.next();
    TrackInfo info[1]; info[0].track = 1; info[0].flick_q = 2; info[0].spread_q = 2; info[0].elen = 3; info[0].mature = true;
    DataHeader h = hdr(2); h.info = info; h.n_info = blk & 1;
    uint8_t b[MAX_BLOCK];
    const size_t n = encodeData(h, p, 4, b, sizeof b);
    DataBlock d;
    for (size_t cut = 0; cut < n; cut++) TEST_ASSERT_FALSE(decodeData(b, cut, d));
  }
}

static void test_fuzz_no_crash(void) {
  uint32_t x = 99;
  uint8_t buf[MAX_BLOCK]; uint8_t lv[BINS];
  int ok = 0;
  for (int i = 0; i < 20000; i++) {
    const size_t n = 1 + (x % MAX_BLOCK);
    for (size_t k = 0; k < n; k++) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; buf[k] = static_cast<uint8_t>(x); }
    buf[1] = static_cast<uint8_t>((buf[1] & 0x0F) | ((1 + i % 3) << 4));
    DataBlock d; Summary s; uint8_t nd, ver, seg;
    if (decodeData(buf, n, d)) {
      ok++;
      TEST_ASSERT_TRUE(d.n_info <= MAX_TARGETS);
      for (uint8_t p = 0; p < d.n; p++) {
        TEST_ASSERT_TRUE(d.pings[p].n_targets <= MAX_TARGETS && d.pings[p].n_resid <= MAX_RESID);
        for (uint8_t r = 0; r < d.pings[p].n_resid; r++) TEST_ASSERT_TRUE(d.pings[p].r[r].bin < BINS);
      }
    }
    decodeSummary(buf, n, s);
    decodeBgSegment(buf, n, nd, ver, seg, lv);
  }
  printf("fuzz: %d random DATA blocks decoded (bounds checked)\n", ok);
}

static void test_bg_roundtrip(void) {
  uint8_t lv[BINS]; for (int i = 0; i < BINS; i++) lv[i] = static_cast<uint8_t>(i < 4 ? 2 : (i > 220 && i < 240 ? 3 : (i > 200 && i <= 220 && (i % 3) ? 1 : 0)));
  uint8_t got[BINS]; memset(got, 0xEE, sizeof got);
  for (uint8_t s = 0; s < BG_SEGMENTS; s++) {
    uint8_t b[MAX_BLOCK];
    const size_t n = encodeBgSegment(77, 5, s, lv, b, sizeof b);
    TEST_ASSERT_TRUE(n > 0);
    uint8_t node, ver, seg;
    TEST_ASSERT_TRUE(decodeBgSegment(b, n, node, ver, seg, got));
    TEST_ASSERT_EQUAL(77, node); TEST_ASSERT_EQUAL(s, seg); TEST_ASSERT_EQUAL(5, ver);
    TEST_ASSERT_FALSE(decodeBgSegment(b, n - 1, node, ver, seg, got));
  }
  TEST_ASSERT_EQUAL_MEMORY(lv, got, BINS);
}

static void test_source_cadence_and_sizes(void) {
  Block out[2];
  // BASE: one summary per 8 pings = 2 s (plus one when a fish shows up)
  SonarSource s; s.begin(130, 42);
  int base_blocks = 0; size_t base_bytes = 0;
  for (int k = 0; k < 4 * 300; k++) {
    const uint8_t n = s.tick(false, out, 2);
    for (uint8_t i = 0; i < n; i++) {
      uint8_t node = 0, type = 0; TEST_ASSERT_TRUE(peekBlock(out[i].data, out[i].len, node, type));
      TEST_ASSERT_EQUAL(BT_BASE, type); base_blocks++; base_bytes += out[i].len;
    }
  }
  TEST_ASSERT_TRUE(base_blocks >= 300 / 2 && base_blocks <= 300 / 2 * 2);
  TEST_ASSERT_TRUE(base_bytes / base_blocks <= 16);
  // FOCUS: DATA every 4 pings (track info every 2nd), BG every 8 pings
  size_t data_bytes = 0, bg_bytes = 0; int data_blocks = 0, bg_blocks = 0, max_block = 0, info_blocks = 0, versions = 0;
  const int secs = 300, holes = 4;
  for (uint32_t seed = 0; seed < static_cast<uint32_t>(holes); seed++) {
    SonarSource f; f.begin(static_cast<uint8_t>(seed + 1), seed);
    int last_ver = -1;
    for (int k = 0; k < 4 * secs; k++) {
      const uint8_t n = f.tick(true, out, 2);
      for (uint8_t i = 0; i < n; i++) {
        uint8_t node = 0, type = 0; peekBlock(out[i].data, out[i].len, node, type);
        if (out[i].len > max_block) max_block = out[i].len;
        if (type == BT_DATA) {
          DataBlock d; TEST_ASSERT_TRUE(decodeData(out[i].data, out[i].len, d));
          data_blocks++; data_bytes += out[i].len; if (d.n_info) info_blocks++;
        } else {
          bg_blocks++; bg_bytes += out[i].len;
          const int v = out[i].data[1] & 15; if (v != last_ver) { versions++; last_ver = v; }
        }
      }
    }
    TEST_ASSERT_EQUAL(0, f.pings_dropped);
  }
  const double t = static_cast<double>(holes) * secs;
  const double per_ping = static_cast<double>(data_bytes) / (data_blocks * 4.0);
  const double focus_bps = (data_bytes + bg_bytes + 2.0 * (data_blocks + bg_blocks)) / t;   // + 2 B section header
  printf("sizes (prototype chain, measured): BASE %.1f B/block %.2f B/s | FOCUS DATA %.2f B/ping (track info in %d%% of blocks), "
         "BG %.1f B/segment, %d bg versions in %d s, FOCUS %.1f B/s incl. headers, max block %d B\n",
         static_cast<double>(base_bytes) / base_blocks, base_bytes / 300.0, per_ping, 100 * info_blocks / data_blocks,
         static_cast<double>(bg_bytes) / bg_blocks, versions, holes * secs, focus_bps, max_block);
  TEST_ASSERT_TRUE(focus_bps < 60.0);   // FOCUS budget from the protocol review
  TEST_ASSERT_TRUE(max_block <= MAX_BLOCK);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_bitstream);
  RUN_TEST(test_summary_roundtrip);
  RUN_TEST(test_data_roundtrip_chain);
  RUN_TEST(test_data_escapes);
  RUN_TEST(test_data_fits_cap);
  RUN_TEST(test_truncation_detected);
  RUN_TEST(test_fuzz_no_crash);
  RUN_TEST(test_bg_roundtrip);
  RUN_TEST(test_source_cadence_and_sizes);
  return UNITY_END();
}
