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

// Bucket test (D43 bmin knob): water 0.40 m deep. The default search starts at 0.6 m (prototype) and
// can only find the 2nd bottom echo at 0.80 m; bmin=2 (0.2 m) finds the real bottom.
static void bucketPings(uint8_t bmin, float& bottom) {
  static uint8_t c[sp::NFREQ][BINS];
  auto code = [](float db) { const float v = (db + 100.0f) * 255.0f / 95.0f; return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v + 0.5f)); };
  S.begin(8, 0, 0); S.proc.prm.set(P_BOTTOM_MIN, bmin); S.proc.prm.set(P_DEADZONE_DM, 1);
  Block b[2];
  for (int k = 0; k < 12; k++) {
    for (int f = 0; f < sp::NFREQ; f++)
      for (int i = 0; i < BINS; i++) {
        float db = -84.0f + ((i * 7 + k * 3 + f) % 5) * 0.5f;          // noise
        if (i < 4) db = -10.0f - 6.0f * i;                              // ring-down
        if (i >= 16 && i <= 17) db = -20.0f;                            // bottom 0.40 m
        if (i >= 32 && i <= 33) db = -40.0f;                            // 2nd echo 0.80 m
        if (i >= 48 && i <= 49) db = -55.0f;                            // 3rd echo 1.20 m
        c[f][i] = code(db);
      }
    S.process(c, sp::NFREQ, -1, 0.25f, false, b, 2);
  }
  bottom = static_cast<float>(S.lastProc().bottom);
}
static void test_bucket_bottom_min(void) {
  float def = 0, bucket = 0;
  bucketPings(6, def); bucketPings(2, bucket);
  printf("bucket 0.40 m: bottom with bmin 0.6 m (default) %.2f m, with bmin 0.2 m %.2f m\n", def, bucket);
  TEST_ASSERT_TRUE(def > 0.7f);                          // the trap the knob exists for
  TEST_ASSERT_TRUE(bucket > 0.36f && bucket < 0.44f);
}

// D47: synthetic pings (codes), noise -84 dB, ring-down, a bottom with a 2nd echo, and extras
static void synthPing(uint8_t c[sp::NFREQ][BINS], int k, float bottom_m, float bottom_db, float fish_m, float fish_db, bool second) {
  auto code = [](float db) { const float v = (db + 100.0f) * 255.0f / 95.0f; return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v + 0.5f)); };
  const int bb = (int)(bottom_m / 0.025f + 0.5f), fb = fish_m > 0 ? (int)(fish_m / 0.025f + 0.5f) : -100;
  for (int f = 0; f < sp::NFREQ; f++)
    for (int i = 0; i < BINS; i++) {
      float db = -84.0f + ((i * 7 + k * 3 + f) % 5) * 0.5f;
      if (i < 4) db = -10.0f - 6.0f * i;
      if (i >= bb && i <= bb + 1) db = bottom_db;
      if (i >= bb + 2 && i <= bb + 4) db = bottom_db - 12.0f;         // echo tail
      if (second && i >= 2 * bb && i <= 2 * bb + 1) db = bottom_db - 22.0f;
      if (i >= fb && i <= fb + 1) db = fish_db;
      c[f][i] = code(db);
    }
}
static void test_bottom_second_echo_beats_school(void) {
  static uint8_t c[sp::NFREQ][BINS]; Block b[2];
  // soft mud bottom at 4.0 m (-45 dB, 2nd echo at 8.0 m), a dense school at 2.5 m louder (-38 dB)
  S.begin(9, 0, 0);
  for (int k = 0; k < 12; k++) { synthPing(c, k, 4.0f, -45.0f, 2.5f, -38.0f, true); S.process(c, sp::NFREQ, -1, 0.25f, false, b, 2); }
  const float bot = static_cast<float>(S.lastProc().bottom);
  // same without any second echo: the strongest window wins (prototype rule), i.e. the school
  S.begin(9, 0, 0);
  for (int k = 0; k < 12; k++) { synthPing(c, k, 4.0f, -45.0f, 2.5f, -38.0f, false); S.process(c, sp::NFREQ, -1, 0.25f, false, b, 2); }
  const float noCheck = static_cast<float>(S.lastProc().bottom);
  printf("school over soft bottom: bottom %.2f m with the 2nd-echo check, %.2f m without a 2nd echo\n", bot, noCheck);
  TEST_ASSERT_TRUE(bot > 3.9f && bot < 4.1f);
  TEST_ASSERT_TRUE(noCheck > 2.4f && noCheck < 2.6f);
}
static void test_deep_hole_noise_floor(void) {
  static uint8_t c[sp::NFREQ][BINS]; Block b[2];
  // bottom at 11.5 m: no water below it in the 12.2 m record; a fish at 6 m must still be found
  S.begin(10, 0, 0); S.proc.prm.set(P_IDLE_HZ_X4, 16);
  int seen = 0; float nf = 0;
  for (int k = 0; k < 20; k++) {
    synthPing(c, k, 11.5f, -30.0f, 6.0f, -55.0f, false);
    for (int f = 0; f < sp::NFREQ; f++) for (int i = 465; i < BINS; i++) c[f][i] = (uint8_t)((-42.0f + 100.0f) * 255.0f / 95.0f);   // soft bottom: long tail to the end
    S.process(c, sp::NFREQ, -1, 0.25f, false, b, 2);
    nf = static_cast<float>(S.lastProc().nf);
    for (uint8_t i = 0; i < S.lastProc().n; i++) if (S.lastProc().t[i].depth > 5.9f && S.lastProc().t[i].depth < 6.1f) seen++;
  }
  printf("deep hole 11.5 m: noise floor %.1f dB (real -84), fish at 6 m seen in %d pings\n", nf, seen);
  TEST_ASSERT_TRUE(nf < -75.0f);
  TEST_ASSERT_TRUE(seen > 10);
}
static void test_no_bait_no_bait_label(void) {
  static uint8_t c[sp::NFREQ][BINS]; Block b[2];
  // a still target exactly at the prototype bait depth (4.57 m); hole with no bait set -> a fish, no flags
  S.begin(11, 0, 0); S.proc.prm.set(P_IDLE_HZ_X4, 16); S.proc.bait_m = R(-1);
  int bait = 0, fish = 0; uint8_t st = 0;
  for (int k = 0; k < 30; k++) {
    synthPing(c, k, 6.0f, -30.0f, 4.57f, -50.0f, true);
    S.process(c, sp::NFREQ, -1, 0.25f, false, b, 2);
    st |= S.status();
    for (uint8_t i = 0; i < S.lastProc().n; i++) { if (S.lastProc().t[i].label == LBL_BAIT) bait++; else if (S.lastProc().t[i].label == LBL_FISH) fish++; }
  }
  TEST_ASSERT_EQUAL(0, bait);
  TEST_ASSERT_TRUE(fish > 0);
  TEST_ASSERT_EQUAL(0, st & (ST_NEAR_BAIT | ST_BAIT_COVER));
  // the same hole with the bait set there: labelled bait
  S.begin(11, 0, 0); S.proc.prm.set(P_IDLE_HZ_X4, 16); S.proc.bait_m = R(4.57);
  bait = 0;
  for (int k = 0; k < 30; k++) {
    synthPing(c, k, 6.0f, -30.0f, 4.57f, -50.0f, true);
    S.process(c, sp::NFREQ, -1, 0.25f, false, b, 2);
    for (uint8_t i = 0; i < S.lastProc().n; i++) if (S.lastProc().t[i].label == LBL_BAIT) bait++;
  }
  printf("still echo at 4.57 m: bait label x%d with the bait set, 0 without\n", bait);
  TEST_ASSERT_TRUE(bait > 0);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_ring_alarm);
  RUN_TEST(test_level_and_flags);
  RUN_TEST(test_rotation_finds_the_same);
  RUN_TEST(test_adaptive_rate);
  RUN_TEST(test_bucket_bottom_min);
  RUN_TEST(test_bottom_second_echo_beats_school);
  RUN_TEST(test_deep_hole_noise_floor);
  RUN_TEST(test_no_bait_no_bait_label);
  return UNITY_END();
}
