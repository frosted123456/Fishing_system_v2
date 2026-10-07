// Sonar knobs (sonar_params.h, D42): table, clamping, presets, and that the knobs act on the processing
// (a stricter detection threshold reports fewer targets on the same pings; defaults stay = prototype).
#include <unity.h>
#include <sonar_sim.h>
#include <cstdio>

using namespace icemesh::sonar;

void setUp(void) {}
void tearDown(void) {}

static void test_table_and_clamp(void) {
  Params p;
  for (uint8_t i = 0; i < P_COUNT; i++) {
    const ParamInfo& f = paramInfo(i);
    TEST_ASSERT_TRUE(f.lo <= f.def && f.def <= f.hi);
    TEST_ASSERT_EQUAL(f.def, p[i]);
  }
  TEST_ASSERT_TRUE(p.set(P_SNR_DB, 200));          // clamped to the max
  TEST_ASSERT_EQUAL(paramInfo(P_SNR_DB).hi, p[P_SNR_DB]);
  TEST_ASSERT_FALSE(p.set(P_SNR_DB, 250));         // unchanged after clamping
  TEST_ASSERT_FALSE(p.set(P_COUNT, 1));            // unknown knob
}

static void test_presets(void) {
  Params p;
  TEST_ASSERT_EQUAL(FILT_NORMAL, presetOf(p));     // defaults = Normal
  applyPreset(p, FILT_HIGH); TEST_ASSERT_EQUAL(FILT_HIGH, presetOf(p));
  applyPreset(p, FILT_LOW); TEST_ASSERT_EQUAL(FILT_LOW, presetOf(p));
  p.set(P_GATE_CM, 50); TEST_ASSERT_EQUAL(FILT_LOW, presetOf(p));     // other knobs do not change the preset
  p.set(P_SNR_DB, 12); TEST_ASSERT_EQUAL(FILT_CUSTOM, presetOf(p));
}

static uint32_t targetsOver(uint8_t snr, int pings) {
  static SonarSource s; s.begin(7, 4241, 0);
  s.proc.prm.set(P_SNR_DB, snr);
  Block out[2]; uint32_t n = 0;
  for (int k = 0; k < pings; k++) { s.tick(false, out, 2); n += s.lastProc().n; }
  return n;
}

static void test_knob_acts(void) {
  const uint32_t normal = targetsOver(10, 200), strict = targetsOver(26, 200);
  printf("targets over 200 pings: snr 10 dB -> %u, snr 26 dB -> %u\n", (unsigned)normal, (unsigned)strict);
  TEST_ASSERT_TRUE(normal > 0);
  TEST_ASSERT_TRUE(strict < normal);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_table_and_clamp);
  RUN_TEST(test_presets);
  RUN_TEST(test_knob_acts);
  return UNITY_END();
}
