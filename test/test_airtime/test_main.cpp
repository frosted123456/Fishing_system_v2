#include <unity.h>
#include <airtime.h>

using namespace icemesh;

void setUp(void) {}
void tearDown(void) {}

// Reference values: Semtech formula computed independently in Python (docs/protocol_v2.md §6).
static void test_known_values_sf9(void) {
  const LoRaPhy p = phySF(9);
  TEST_ASSERT_UINT32_WITHIN(1000, 595000, airtimeUs(108, p));   // v1 aggregate
  TEST_ASSERT_UINT32_WITHIN(1000, 185000, airtimeUs(18, p));    // v1 LoRa alert
  TEST_ASSERT_UINT32_WITHIN(1000, 329000, airtimeUs(52, p));
  TEST_ASSERT_UINT32_WITHIN(1000, 267000, airtimeUs(39, p));    // v2 aggregate, 3 nodes
  TEST_ASSERT_UINT32_WITHIN(1000, 411000, airtimeUs(68, p));    // stream 8 B hdr + 60 B
}

static void test_known_values_other_sf(void) {
  TEST_ASSERT_UINT32_WITHIN(1000, 185000, airtimeUs(108, phySF(7)));
  TEST_ASSERT_UINT32_WITHIN(1000, 103000, airtimeUs(52, phySF(7)));
  TEST_ASSERT_UINT32_WITHIN(1000, 1067000, airtimeUs(108, phySF(10)));
}

static void test_monotonic_in_length(void) {
  const LoRaPhy p = phySF(9);
  uint32_t prev = 0;
  for (uint16_t n = 1; n <= 255; n++) {
    const uint32_t t = airtimeUs(n, p);
    TEST_ASSERT_GREATER_OR_EQUAL(prev, t);
    prev = t;
  }
}

static void test_ldro_kicks_in_at_sf11_125k(void) {
  // SF11/125 kHz: symbol = 16.384 ms -> LDRO on. By hand: ceil((400-44+28+16)/36) = 12 blocks
  // -> 8 + 12*5 = 68 payload symbols + 12.25 preamble = 80.25 * 16.384 ms = 1314.816 ms.
  // (Without LDRO it would be ceil(400/44)=10 blocks -> 70.25 symbols = 1151 ms.)
  TEST_ASSERT_UINT32_WITHIN(1000, 1314816, airtimeUs(50, phySF(11)));
}

static void test_budget_limits_long_run_share(void) {
  AirtimeBudget b(200, 4000);                 // 20 %
  const uint32_t pkt_us = airtimeUs(68, phySF(9));
  uint32_t now = 0, radio_free_at = 0;
  for (now = 0; now < 600000; now += 10) {    // 10 minutes, try to send as fast as possible
    if (now >= radio_free_at && b.canSend(now)) {
      b.spend(pkt_us, now);
      radio_free_at = now + pkt_us / 1000;
    }
  }
  const double share = static_cast<double>(b.totalSpentUs()) / (600000.0 * 1000.0);
  TEST_ASSERT_TRUE(share <= 0.205);
  TEST_ASSERT_TRUE(share >= 0.19);
}

static void test_budget_never_locks_out_big_packet(void) {
  AirtimeBudget b(100, 1000);                 // capacity 100 ms of airtime only
  const uint32_t big = airtimeUs(255, phySF(9));   // ~1.2 s
  TEST_ASSERT_TRUE(b.canSend(0));
  b.spend(big, 0);
  TEST_ASSERT_FALSE(b.canSend(1000));
  TEST_ASSERT_TRUE(b.canSend(13000));         // debt repaid after ~big/0.1
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_known_values_sf9);
  RUN_TEST(test_known_values_other_sf);
  RUN_TEST(test_monotonic_in_length);
  RUN_TEST(test_ldro_kicks_in_at_sf11_125k);
  RUN_TEST(test_budget_limits_long_run_share);
  RUN_TEST(test_budget_never_locks_out_big_packet);
  return UNITY_END();
}
