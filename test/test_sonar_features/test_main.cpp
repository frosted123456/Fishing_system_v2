// D43 sonar features on the fake chain: ring-down alarm, fish-near-bait and bait-in-cover flags,
// bottom echo strength (level check), one frequency per ping (rotating), adaptive ping rate, bait per hole.
#include <unity.h>
#include <sonar_sim.h>
#include <cstdio>

using namespace icemesh::sonar;

void setUp(void) {}
void tearDown(void) {}

static SonarSource S;   // ~17 KB: static

static void test_ring_alarm(void) {
  S.begin(5, 1234, 0); S.proc.prm.set(P_IDLE_HZ_X4, 16);
  Block b[2];
  for (int k = 0; k < 120; k++) { S.tick(false, b, 2); TEST_ASSERT_FALSE(S.lastProc().ring_alarm); }
  const float normal = static_cast<float>(S.lastProc().ring_base_m);
  S.scene.ring_tau = R(0.25);   // slush on the face: much longer ring-down
  bool alarm = false; int when = -1;
  for (int k = 0; k < 60 && !alarm; k++) { S.tick(false, b, 2); alarm = S.lastProc().ring_alarm; when = k; }
  printf("ring-down: normal %.2f m, with slush %.2f m, alarm after %d pings\n", normal, static_cast<float>(S.lastProc().ring_m), when + 1);
  TEST_ASSERT_TRUE(alarm);
  TEST_ASSERT_TRUE((S.status() & ST_RING) != 0);
  S.scene.ring_tau = R(0.045);
  for (int k = 0; k < 20; k++) S.tick(false, b, 2);
  TEST_ASSERT_FALSE(S.lastProc().ring_alarm);   // clears when the face is clean again
}

static void test_level_and_flags(void) {
  S.begin(6, 0, 0); S.proc.prm.set(P_IDLE_HZ_X4, 16);   // the prototype hole: fish visit the bait
  Block b[2]; int near = 0, cover = 0; float snr = 0;
  for (int k = 0; k < 400; k++) {
    S.tick(false, b, 2);
    if (S.status() & ST_NEAR_BAIT) near++;
    if (S.status() & ST_BAIT_COVER) cover++;
    snr = static_cast<float>(S.lastProc().bottom_snr);
  }
  const Summary s = S.summary();
  printf("bottom echo %.1f dB over noise (summary %u), near-bait pings %d / 400, cover %d\n", snr, s.bottom_snr, near, cover);
  TEST_ASSERT_TRUE(snr > 20.0f);           // hard sand bottom, transducer straight: strong
  TEST_ASSERT_TRUE(s.bottom_snr > 20);
  TEST_ASSERT_TRUE(near > 0);              // the prototype story has fish near the lure
  TEST_ASSERT_EQUAL(0, cover);             // bait ~1 m over a bare bottom
  // bait moved down next to the bottom: in cover
  S.scene.setBait(R(5.45)); S.proc.bait_m = R(5.45);
  for (int k = 0; k < 40; k++) S.tick(false, b, 2);
  TEST_ASSERT_TRUE((S.status() & ST_BAIT_COVER) != 0);
}

static void test_rotation_finds_the_same(void) {
  Block b[2];
  S.begin(7, 4241, 0); S.proc.prm.set(P_IDLE_HZ_X4, 16);
  float b3 = 0; uint32_t t3 = 0;
  for (int k = 0; k < 300; k++) { S.tick(false, b, 2); t3 += S.lastProc().n; b3 = static_cast<float>(S.lastProc().bottom); }
  S.begin(7, 4241, 0); S.proc.prm.set(P_IDLE_HZ_X4, 16); S.proc.prm.set(P_FREQ_MODE, 1);
  float b1 = 0; uint32_t t1 = 0;
  for (int k = 0; k < 300; k++) { S.tick(false, b, 2); t1 += S.lastProc().n; b1 = static_cast<float>(S.lastProc().bottom); }
  printf("3 bursts/ping: bottom %.2f m, %u targets; rotating 1/ping: bottom %.2f m, %u targets\n", b3, (unsigned)t3, b1, (unsigned)t1);
  TEST_ASSERT_TRUE(b1 > b3 - 0.1f && b1 < b3 + 0.1f);
  TEST_ASSERT_TRUE(t1 > t3 / 3);           // still sees the targets (some smear)
}

static void test_adaptive_rate(void) {
  Block b[2];
  S.begin(8, 999, 0);
  S.proc.prm.set(P_PING_HZ_X4, 16); S.proc.prm.set(P_IDLE_HZ_X4, 4);
  TEST_ASSERT_TRUE(S.ticksPerPing(false) == 1 || S.ticksPerPing(false) == 4);   // active (fish seen) or idle
  TEST_ASSERT_EQUAL(1, S.ticksPerPing(true));             // focus: full rate
  uint16_t first = S.lastPing().index; int pings = 0;
  for (int k = 0; k < 40; k++) { S.tick(false, b, 2); }
  pings = static_cast<uint16_t>(S.lastPing().index - first);
  printf("40 ticks (10 s): %d pings with idle 1/s, active 4/s\n", pings);
  TEST_ASSERT_TRUE(pings >= 10 && pings <= 40);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_ring_alarm);
  RUN_TEST(test_level_and_flags);
  RUN_TEST(test_rotation_finds_the_same);
  RUN_TEST(test_adaptive_rate);
  return UNITY_END();
}
