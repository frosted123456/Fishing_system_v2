// Spool-shaft Hall latch logic (hall_latch.h): trips by flip count, deep-sleep wakes, clearing.
#include <unity.h>
#include <hall_latch.h>

using icemesh::HallLatch;

static HallLatch mk(uint8_t trig = 1) { HallLatch h = HallLatch(); h.settings(trig, 10000, 30000); return h; }

void setUp(void) {}
void tearDown(void) {}

static void test_quarter_turn_trips(void) {
  HallLatch h = mk(); h.boot(1, false, 0);
  TEST_ASSERT_FALSE(h.active(1000));
  h.boot(0, true, 5000);                 // pin wake, level changed: 1 flip
  TEST_ASSERT_TRUE(h.active(5000));
  TEST_ASSERT_EQUAL_UINT8(1, h.wakeLevel());   // sleep again: wake on the next flip (back to 1)
}

static void test_half_turn_setting(void) {
  HallLatch h = mk(2); h.boot(1, false, 0);
  h.boot(0, true, 1000);                 // one flip: e.g. resetting the flag by hand, or wind
  TEST_ASSERT_FALSE(h.active(1000));
  h.boot(1, true, 30000);                // another flip 29 s later: outside the window, new count
  TEST_ASSERT_FALSE(h.active(30000));
  h.boot(0, true, 32000);                // a second flip 2 s later: half turn -> trip
  TEST_ASSERT_TRUE(h.active(32000));
}

static void test_fast_double_flip_during_boot(void) {
  HallLatch h = mk(2); h.boot(1, false, 0);
  h.boot(1, true, 1000);                 // woke by the pin but the level is back: >= 2 flips
  TEST_ASSERT_TRUE(h.active(1000));
  TEST_ASSERT_EQUAL_UINT32(2, h.total);
}

static void test_timer_wake_no_flip(void) {
  HallLatch h = mk(); h.boot(0, false, 0);
  h.boot(0, false, 60000);               // heartbeat wake, nothing moved
  TEST_ASSERT_FALSE(h.active(60000));
  TEST_ASSERT_EQUAL_UINT32(0, h.total);
}

static void test_running_line_keeps_trip_then_clears(void) {
  HallLatch h = mk(); h.boot(0, false, 0);
  h.edges(1, 1, 1000);
  TEST_ASSERT_TRUE(h.active(1000));
  for (uint64_t t = 2000; t <= 20000; t += 1000) { h.edges(4, 1, t); TEST_ASSERT_TRUE(h.active(t)); }   // spool spinning
  TEST_ASSERT_TRUE(h.active(20000 + 29999));                // still < 30 s after the last flip
  TEST_ASSERT_FALSE(h.active(20000 + 30000));               // line stopped 30 s: trip over
  TEST_ASSERT_EQUAL_UINT32(1 + 19 * 4, h.total);
  h.edges(1, 0, 60000);                                      // runs again later: a new trip
  TEST_ASSERT_TRUE(h.active(60000));
}

static void test_missed_edge_seen_by_level(void) {
  HallLatch h = mk(); h.boot(0, false, 0);
  h.edges(0, 1, 500);                    // interrupt saw nothing but the level changed
  TEST_ASSERT_TRUE(h.active(500));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_quarter_turn_trips);
  RUN_TEST(test_half_turn_setting);
  RUN_TEST(test_fast_double_flip_during_boot);
  RUN_TEST(test_timer_wake_no_flip);
  RUN_TEST(test_running_line_keeps_trip_then_clears);
  RUN_TEST(test_missed_edge_seen_by_level);
  return UNITY_END();
}
