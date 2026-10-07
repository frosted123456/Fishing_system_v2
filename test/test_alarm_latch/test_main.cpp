// FISH ON alarm latch rules (alarm_latch.h), including the D47 fix: silenced while tripped must not come back.
#include <unity.h>
#include <alarm_latch.h>

using icemesh::AlarmLatch;
void setUp(void) {}
void tearDown(void) {}

static void test_latched_until_silenced(void) {
  AlarmLatch a = AlarmLatch();
  TEST_ASSERT_FALSE(a.active(false));
  a.start(1000);
  TEST_ASSERT_TRUE(a.active(true));
  TEST_ASSERT_TRUE(a.active(false));                 // line reset: still an alarm
  TEST_ASSERT_FALSE(a.hold(false, 100000, 0));       // no hold time: stays
  a.ack(false);                                      // silence with the line down: cleared
  TEST_ASSERT_FALSE(a.active(false));
}

static void test_silenced_while_tripped_ends_at_reset(void) {
  AlarmLatch a = AlarmLatch();
  a.start(1000);
  a.ack(true);                                       // silenced while the flag is still up
  TEST_ASSERT_TRUE(a.active(true));                  // shown while tripped
  a.lineReset();                                     // flag put back down 10 min later
  TEST_ASSERT_FALSE(a.active(false));                // no comeback (the old bug)
}

static void test_new_trip_rearms(void) {
  AlarmLatch a = AlarmLatch();
  a.start(1000); a.ack(true);
  a.start(5000);                                     // tripped again (or a second trip) before the reset
  TEST_ASSERT_FALSE(a.acked);
  a.lineReset();
  TEST_ASSERT_TRUE(a.active(false));                 // the new trip is not acknowledged: stays until silenced
}

static void test_hold_time(void) {
  AlarmLatch a = AlarmLatch();
  a.start(1000);
  TEST_ASSERT_FALSE(a.hold(true, 1000 + 999999, 60000));   // line still up: never expires
  TEST_ASSERT_FALSE(a.hold(false, 1000 + 59000, 60000));
  TEST_ASSERT_TRUE(a.hold(false, 1000 + 60001, 60000));    // 1 min after the trip, line down: gone
  TEST_ASSERT_FALSE(a.active(false));
}

static void test_start_never_zero(void) {
  AlarmLatch a = AlarmLatch();
  a.start(0);                                        // millis() == 0 at boot
  TEST_ASSERT_TRUE(a.active(false));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_latched_until_silenced);
  RUN_TEST(test_silenced_while_tripped_ends_at_reset);
  RUN_TEST(test_new_trip_rearms);
  RUN_TEST(test_hold_time);
  RUN_TEST(test_start_never_zero);
  return UNITY_END();
}
