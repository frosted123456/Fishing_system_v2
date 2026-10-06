// Same chain in float (what the ESP32 runs): close to the prototype, not bit-exact.
#include "../test_sonar_proc/ref_compare.h"
#include <ctime>

void setUp(void) {}
void tearDown(void) {}

static void test_float_close_to_prototype(void) {
  Ref ref; TEST_ASSERT_TRUE(loadRef(ref));
  SonarScene sc; sc.begin(SceneConfig::prototype());
  SonarProc pr; pr.bait_m = 4.57f;
  static uint8_t codes[3][BINS];
  SonarProc::Out o;
  int bottom_ok = 0, hard_ok = 0, tracks_ok = 0, n = 0;
  double max_bottom = 0, max_nf = 0;
  const clock_t c0 = clock();
  for (size_t k = 0; k < ref.P.size(); k++) {
    sc.ping(codes); pr.step(codes, 3, o);
    const RefPing& r = ref.P[k];
    n++;
    max_bottom = std::max(max_bottom, std::fabs(o.bottom - r.bottom));
    max_nf = std::max(max_nf, std::fabs(o.nf - r.nf));
    if (std::fabs(o.bottom - r.bottom) < 0.025) bottom_ok++;
    if (o.hard == r.hard) hard_ok++;
    // every live reference target (seen this ping, visible) has a float target with the same label within 5 cm
    bool all = true;
    for (size_t i = 0; i < r.tr.size(); i++) {
      const RefTrack& x = r.tr[i];
      if (x.miss != 0 || x.v <= 0.08 || x.label == LBL_COVER) continue;
      bool found = false;
      for (uint8_t j = 0; j < o.n && !found; j++) found = o.t[j].miss == 0 && o.t[j].label == x.label && std::fabs(o.t[j].depth - x.depth) < 0.05;
      all = all && found;
    }
    if (all) tracks_ok++;
  }
  const double ms = 1000.0 * (clock() - c0) / CLOCKS_PER_SEC / n;
  printf("float vs prototype over %d pings: bottom %d%% (max %.4f m), nf max %.3f dB, hardness %d%%, live targets %d%%, %.3f ms/ping on this PC\n",
         n, 100 * bottom_ok / n, max_bottom, max_nf, 100 * hard_ok / n, 100 * tracks_ok / n, ms);
  TEST_ASSERT_GREATER_OR_EQUAL(99, 100 * bottom_ok / n);
  TEST_ASSERT_GREATER_OR_EQUAL(95, 100 * hard_ok / n);
  TEST_ASSERT_GREATER_OR_EQUAL(90, 100 * tracks_ok / n);
}

static void test_other_holes_run(void) {
  // per-hole variants: bottom found near the configured depth, bait labelled near the bait line
  for (uint32_t s = 1; s <= 6; s++) {
    SceneConfig cfg = SceneConfig::forHole(s);
    SonarScene sc; sc.begin(cfg);
    SonarProc pr; pr.bait_m = cfg.bait_m;
    static uint8_t codes[3][BINS];
    SonarProc::Out o;
    int bait_seen = 0;
    for (int k = 0; k < 400; k++) {
      sc.ping(codes); pr.step(codes, 3, o);
      for (uint8_t j = 0; j < o.n; j++) if (o.t[j].label == LBL_BAIT && o.t[j].miss == 0) { bait_seen++; break; }
    }
    printf("hole %u: bottom cfg %.2f m found %.2f m, bait %.2f m seen in %d/400 pings, %s bottom -> %s\n", s, cfg.bottom_m, o.bottom,
           cfg.bait_m, bait_seen, cfg.sand ? "sand" : "mud", o.hard == HARD_HARD ? "hard" : o.hard == HARD_SOFT ? "soft" : "medium");
    TEST_ASSERT_TRUE(std::fabs(o.bottom - cfg.bottom_m) < 0.1f);
    TEST_ASSERT_GREATER_OR_EQUAL(200, bait_seen);
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_float_close_to_prototype);
  RUN_TEST(test_other_holes_run);
  return UNITY_END();
}
