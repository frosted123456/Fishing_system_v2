#include <unity.h>
#include <rx_ring.h>
#include <thread>

using namespace icemesh;

void setUp(void) {}
void tearDown(void) {}

static void test_fifo_and_copy(void) {
  RxRing<4, 16> r;
  const uint8_t mac[6] = {1, 2, 3, 4, 5, 6};
  uint8_t d[3] = {7, 8, 9};
  TEST_ASSERT_TRUE(r.push(mac, d, 3, -40));
  d[0] = 0;                                   // caller buffer reused: ring holds a copy
  RxRing<4, 16>::Frame f;
  TEST_ASSERT_TRUE(r.pop(f));
  TEST_ASSERT_EQUAL(3, f.len);
  TEST_ASSERT_EQUAL(7, f.data[0]);
  TEST_ASSERT_EQUAL(-40, f.rssi);
  TEST_ASSERT_EQUAL_MEMORY(mac, f.mac, 6);
  TEST_ASSERT_FALSE(r.pop(f));
}

static void test_full_drops_and_counts(void) {
  RxRing<4, 16> r;                            // capacity = SLOTS - 1 = 3
  uint8_t d[1] = {0};
  for (uint8_t i = 0; i < 3; i++) { d[0] = i; TEST_ASSERT_TRUE(r.push(nullptr, d, 1, 0)); }
  TEST_ASSERT_FALSE(r.push(nullptr, d, 1, 0));
  TEST_ASSERT_EQUAL(1, r.dropped());
  RxRing<4, 16>::Frame f;
  for (uint8_t i = 0; i < 3; i++) { TEST_ASSERT_TRUE(r.pop(f)); TEST_ASSERT_EQUAL(i, f.data[0]); }
  TEST_ASSERT_TRUE(r.empty());
}

static void test_rejects_bad_length(void) {
  RxRing<4, 16> r;
  uint8_t d[17] = {0};
  TEST_ASSERT_FALSE(r.push(nullptr, d, 17, 0));
  TEST_ASSERT_FALSE(r.push(nullptr, d, 0, 0));
  TEST_ASSERT_FALSE(r.push(nullptr, nullptr, 3, 0));
  TEST_ASSERT_EQUAL(3, r.dropped());
}

// Producer and consumer on two threads: every frame arrives once, in order, or is counted as dropped.
static void test_two_threads(void) {
  static RxRing<8, 8> r;
  const uint32_t N = 200000;
  std::thread prod([&]() {
    for (uint32_t i = 0; i < N; i++) {
      uint8_t d[4];
      memcpy(d, &i, 4);
      while (!r.push(nullptr, d, 4, 0)) { std::this_thread::yield(); }
    }
  });
  uint32_t expect = 0;
  bool order_ok = true;
  RxRing<8, 8>::Frame f;
  while (expect < N) {
    if (r.pop(f)) {
      uint32_t v;
      memcpy(&v, f.data, 4);
      if (v != expect) order_ok = false;
      expect++;
    }
  }
  prod.join();
  TEST_ASSERT_TRUE(order_ok);
  TEST_ASSERT_EQUAL_UINT32(N, expect);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_fifo_and_copy);
  RUN_TEST(test_full_drops_and_counts);
  RUN_TEST(test_rejects_bad_length);
  RUN_TEST(test_two_threads);
  return UNITY_END();
}
