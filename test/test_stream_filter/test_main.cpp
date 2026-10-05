#include <unity.h>
#include <stream_filter.h>

using namespace icemesh;

void setUp(void) {}
void tearDown(void) {}

static void test_newer_forwarded(void) {
  StreamFilter<> f;
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(20, 5, 0));
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(20, 6, 250));
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(20, 9, 500));     // gaps are fine (lost packets)
}

static void test_duplicate_and_older_dropped(void) {
  StreamFilter<> f;
  f.check(20, 10, 0);
  TEST_ASSERT_EQUAL(STREAM_DROP_OLD, f.check(20, 10, 1));     // same packet via another path
  f.check(20, 12, 2);
  TEST_ASSERT_EQUAL(STREAM_DROP_OLD, f.check(20, 11, 3));     // late: newest already passed
  TEST_ASSERT_EQUAL(STREAM_DROP_OLD, f.check(20, 12 - 15, 4));// d = -15, still "old"
}

static void test_restart(void) {
  StreamFilter<> f;
  f.check(20, 400, 0);
  TEST_ASSERT_EQUAL(STREAM_FORWARD_RESTART, f.check(20, 400 - 16, 1));
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(20, 400 - 15, 2));
  TEST_ASSERT_EQUAL(STREAM_FORWARD_RESTART, f.check(20, 0, 3));  // reboot to seq 0
}

static void test_wrap(void) {
  StreamFilter<> f;
  f.check(20, 65534, 0);
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(20, 65535, 1));
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(20, 0, 2));
  TEST_ASSERT_EQUAL(STREAM_DROP_OLD, f.check(20, 65535, 3));
}

static void test_origins_independent_and_idle(void) {
  StreamFilter<> f;
  f.check(20, 100, 0);
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(21, 50, 0));
  TEST_ASSERT_EQUAL(STREAM_DROP_OLD, f.check(20, 100, 1));
  // 10 s of silence: next packet is accepted whatever its seq
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(20, 95, 10000));
}

static void test_table_full_replaces_oldest_stream(void) {
  StreamFilter<2> f;
  f.check(1, 10, 0);
  f.check(2, 10, 5);
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(3, 10, 6));       // evicts origin 1
  TEST_ASSERT_EQUAL(STREAM_DROP_OLD, f.check(2, 10, 7));      // 2 kept
  TEST_ASSERT_EQUAL(STREAM_FORWARD, f.check(1, 10, 8));       // 1 was forgotten
}

// 4 Hz stream over 3 paths with jitter: exactly the in-order newest packets go through,
// and the cache never "flushes" like the v1 32-entry ring would (issue 3).
static void test_4hz_three_paths_10_minutes(void) {
  StreamFilter<> f;
  uint16_t last_fwd = 0;
  bool any = false;
  int forwarded = 0, out_of_order = 0;
  for (uint32_t i = 0; i < 4u * 600u; i++) {
    const uint32_t t = i * 250;
    const uint16_t seq = static_cast<uint16_t>(i);
    const uint16_t copies[3] = {seq, seq, static_cast<uint16_t>(i >= 2 ? i - 2 : i)};
    for (int k = 0; k < 3; k++) {
      if (isForward(f.check(30, copies[k], t + k * 40))) {
        if (any && !seqNewer(copies[k], last_fwd)) out_of_order++;
        last_fwd = copies[k];
        any = true;
        forwarded++;
      }
    }
  }
  TEST_ASSERT_EQUAL(2400, forwarded);
  TEST_ASSERT_EQUAL(0, out_of_order);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_newer_forwarded);
  RUN_TEST(test_duplicate_and_older_dropped);
  RUN_TEST(test_restart);
  RUN_TEST(test_wrap);
  RUN_TEST(test_origins_independent_and_idle);
  RUN_TEST(test_table_full_replaces_oldest_stream);
  RUN_TEST(test_4hz_three_paths_10_minutes);
  return UNITY_END();
}
