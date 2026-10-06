// C++ port vs Frank's JavaScript prototype, same seed: built in double precision like the browser.
#define ICEMESH_SONAR_REAL double
#include "ref_compare.h"

void setUp(void) {}
void tearDown(void) {}

static Ref ref;

static void test_scene_codes_match_prototype(void) {
  TEST_ASSERT_TRUE(loadRef(ref));
  SonarScene sc; sc.begin(SceneConfig::prototype());
  static uint8_t codes[3][BINS];
  int diff = 0, maxd = 0;
  for (int k = 0; k < 12; k++) {
    sc.ping(codes);
    for (int f = 0; f < 3; f++) {
      TEST_ASSERT_EQUAL(BINS, static_cast<int>(ref.D[k][f].size()));
      for (int i = 0; i < BINS; i++) { const int dd = std::abs(codes[f][i] - ref.D[k][f][i]); if (dd) { diff++; if (dd > maxd) maxd = dd; } }
    }
  }
  printf("scene: %d of %d codes differ from the prototype (max %d step)\n", diff, 12 * 3 * BINS, maxd);
  TEST_ASSERT_LESS_OR_EQUAL(12 * 3 * BINS / 1000, diff);   // libm last-bit differences only
  TEST_ASSERT_LESS_OR_EQUAL(1, maxd);
}

static void test_proc_matches_prototype(void) {
  SonarScene sc; sc.begin(SceneConfig::prototype());
  SonarProc pr; pr.bait_m = 4.57;
  static uint8_t codes[3][BINS];
  SonarProc::Out o;
  int bad = 0; int first_bad = -1;
  for (size_t k = 0; k < ref.P.size(); k++) {
    sc.ping(codes);
    pr.step(codes, 3, o);
    const RefPing& r = ref.P[k];
    bool ok = std::fabs(o.bottom - r.bottom) < 1e-5 && std::fabs(o.nf - r.nf) < 1e-4 && o.hard == r.hard &&
              o.has_ratio == r.has_ratio && (!r.has_ratio || std::fabs(o.ratio - r.ratio) < 1e-4) && o.n == r.tr.size();
    for (size_t i = 0; ok && i < r.tr.size(); i++) {
      const TrackOut& t = o.t[i]; const RefTrack& x = r.tr[i];
      ok = static_cast<int>(t.id) == x.id && t.label == x.label && t.miss == x.miss &&
           std::fabs(t.depth - x.depth) < 1e-5 && std::fabs(t.vel - x.vel) < 1e-4 && std::fabs(t.v - x.v) < 1e-4 && std::fabs(t.s - x.s) < 1e-4;
    }
    if (!ok) { bad++; if (first_bad < 0) first_bad = static_cast<int>(k); }
  }
  printf("proc: %d of %u pings differ from the prototype (first %d)\n", bad, static_cast<unsigned>(ref.P.size()), first_bad);
  TEST_ASSERT_EQUAL(0, bad);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_scene_codes_match_prototype);
  RUN_TEST(test_proc_matches_prototype);
  return UNITY_END();
}
