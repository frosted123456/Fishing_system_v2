#include <unity.h>
#include <dedup_window.h>

using namespace icemesh;

void setUp(void) {}
void tearDown(void) {}

static void test_first_frame_and_in_order(void) {
  DedupWindow<> w;
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(5, 100, 0));
  for (uint16_t s = 101; s < 200; s++) TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(5, s, s));
}

static void test_exact_duplicate_dropped(void) {
  DedupWindow<> w;
  w.check(5, 10, 0);
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(5, 10, 1));
  w.check(5, 11, 2);
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(5, 10, 3));   // older copy of an already seen seq
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(5, 11, 4));
}

static void test_reordered_accepted_once(void) {
  DedupWindow<> w;
  w.check(5, 10, 0);
  w.check(5, 14, 1);                                  // 11..13 still missing
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(5, 12, 2));
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(5, 12, 3));
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(5, 11, 4));
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(5, 13, 5));
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(5, 10, 6));
  TEST_ASSERT_TRUE(w.seen(5, 13, 7));
}

static void test_window_edge(void) {
  DedupWindow<> w;                                    // window 32
  w.check(5, 1000, 0);
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(5, 1000 - 31, 1));   // d = -31: inside
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(5, 1000 - 31, 2));
  TEST_ASSERT_EQUAL(ACCEPT_RESTART, w.check(5, 1000 - 32, 3)); // d = -32: restart
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(5, 1000 - 32, 4));      // new window starts there
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(5, 1000 - 31, 5));
}

static void test_origin_restart_after_reboot(void) {
  DedupWindow<> w;
  for (uint16_t s = 1; s <= 500; s++) w.check(9, s, s);
  TEST_ASSERT_EQUAL(ACCEPT_RESTART, w.check(9, 1, 600));      // rebooted, counter back to 1
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(9, 2, 601));
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(9, 1, 602));
}

static void test_wrap_65535_to_0(void) {
  DedupWindow<> w;
  w.check(3, 65533, 0);
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(3, 65535, 1));
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(3, 0, 2));
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(3, 1, 3));
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(3, 65534, 4));       // reordered across the wrap
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(3, 65535, 5));
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(3, 0, 6));
}

static void test_big_forward_jump_clears_bitmap(void) {
  DedupWindow<> w;
  w.check(3, 10, 0);
  w.check(3, 11, 1);
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(3, 100, 2));          // jump of 89 >= window
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(3, 99, 3));          // old bits must not leak
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(3, 80, 4));
}

static void test_origins_are_independent(void) {
  DedupWindow<> w;
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(1, 7, 0));
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(2, 7, 0));            // same seq, other origin
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(1, 7, 1));
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(2, 7, 1));
}

static void test_table_full_evicts_least_recent(void) {
  DedupWindow<4> w;
  for (uint8_t o = 1; o <= 4; o++) w.check(o, 100, o);         // origin 1 is the oldest
  w.check(1, 101, 10);                                         // refresh origin 1 -> 2 is oldest
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(5, 50, 11));           // evicts origin 2
  TEST_ASSERT_EQUAL(1, w.evictions());
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(1, 101, 12));           // 1 kept
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(2, 100, 13));          // 2 forgotten -> new again
  TEST_ASSERT_EQUAL(4, w.size(13));
}

static void test_idle_origin_forgotten(void) {
  DedupWindow<> w;
  w.check(4, 300, 0);
  const uint32_t later = DedupWindow<>::IDLE_EXPIRY_MS;
  TEST_ASSERT_FALSE(w.seen(4, 300, later));
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(4, 300, later));        // treated as a fresh origin
  TEST_ASSERT_EQUAL(1, w.size(later));
}

static void test_millis_rollover(void) {
  DedupWindow<> w;
  w.check(4, 1, 0xFFFFFF00u);
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(4, 1, 0x00000100u));     // 512 ms later, after rollover
}

static void test_window_64(void) {
  typedef DedupWindow<8, uint64_t> Window64;
  Window64 w;
  TEST_ASSERT_EQUAL(64, (int)Window64::WINDOW);
  w.check(1, 1000, 0);
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(1, 1000 - 63, 1));
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(1, 1000 - 63, 2));
  TEST_ASSERT_EQUAL(ACCEPT_NEW, w.check(1, 1040, 3));          // shift by 40 keeps 937 (now d=-103) out
  TEST_ASSERT_EQUAL(ACCEPT_LATE, w.check(1, 1000 - 20, 4));    // d = -60, unseen
  TEST_ASSERT_EQUAL(DUPLICATE, w.check(1, 1000, 5));           // d = -40, seen before the shift
}

// Simulates the hub mesh: the same 200 frames arrive over 3 paths, shuffled.
static void test_three_paths_each_frame_once(void) {
  DedupWindow<> w;
  int accepted = 0;
  uint32_t now = 0;
  for (uint16_t s = 0; s < 200; s++) {
    // path A on time, path B one frame late, path C three frames late
    const uint16_t arrivals[3] = {s, (uint16_t)(s >= 1 ? s - 1 : s), (uint16_t)(s >= 3 ? s - 3 : s)};
    for (int k = 0; k < 3; k++)
      if (isAccepted(w.check(11, arrivals[k], now += 7))) accepted++;
  }
  TEST_ASSERT_EQUAL(200, accepted);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_first_frame_and_in_order);
  RUN_TEST(test_exact_duplicate_dropped);
  RUN_TEST(test_reordered_accepted_once);
  RUN_TEST(test_window_edge);
  RUN_TEST(test_origin_restart_after_reboot);
  RUN_TEST(test_wrap_65535_to_0);
  RUN_TEST(test_big_forward_jump_clears_bitmap);
  RUN_TEST(test_origins_are_independent);
  RUN_TEST(test_table_full_evicts_least_recent);
  RUN_TEST(test_idle_origin_forgotten);
  RUN_TEST(test_millis_rollover);
  RUN_TEST(test_window_64);
  RUN_TEST(test_three_paths_each_frame_once);
  return UNITY_END();
}
