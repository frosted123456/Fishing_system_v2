#include <unity.h>
#include <tx_queue.h>
#include <airtime.h>

using namespace icemesh;

typedef TxQueue<64> Q;   // small frames are enough for the tests

void setUp(void) {}
void tearDown(void) {}

static uint8_t buf[64];
static const uint8_t* frame(uint8_t marker, uint16_t len = 10) {
  for (uint16_t i = 0; i < len; i++) buf[i] = marker;
  return buf;
}

static void test_priority_order(void) {
  Q q;
  q.push(frame(3), 10, TxOptions::make(PRIO_STREAM, 0), 0);
  q.push(frame(2), 10, TxOptions::make(PRIO_AGGREGATE, 0), 0);
  q.push(frame(1), 10, TxOptions::make(PRIO_CONTROL, 0), 0);
  q.push(frame(0), 10, TxOptions::make(PRIO_ALERT, 0), 0);
  for (uint8_t expect = 0; expect < 4; expect++) {
    Q::Item* it = q.peekReady(1);
    TEST_ASSERT_NOT_NULL(it);
    TEST_ASSERT_EQUAL(expect, it->data[0]);
    q.remove(it);
  }
  TEST_ASSERT_NULL(q.peekReady(1));
}

static void test_fifo_inside_priority(void) {
  Q q;
  for (uint8_t m = 0; m < 5; m++) q.push(frame(m), 10, TxOptions::make(PRIO_CONTROL, 0), m);
  for (uint8_t m = 0; m < 5; m++) {
    Q::Item* it = q.peekReady(10);
    TEST_ASSERT_EQUAL(m, it->data[0]);
    q.remove(it);
  }
}

static void test_not_before_and_deferred_does_not_block(void) {
  Q q;
  TxOptions a = TxOptions::make(PRIO_ALERT, 0);
  a.not_before_ms = 500;                                   // relay jitter
  q.push(frame(0xA), 10, a, 0);
  q.push(frame(0xB), 10, TxOptions::make(PRIO_AGGREGATE, 0), 0);
  Q::Item* it = q.peekReady(100);
  TEST_ASSERT_EQUAL(0xB, it->data[0]);                     // alert not ready yet
  q.remove(it);
  TEST_ASSERT_NULL(q.peekReady(499));
  it = q.peekReady(500);
  TEST_ASSERT_EQUAL(0xA, it->data[0]);
  q.defer(it, 800);                                        // CAD busy
  TEST_ASSERT_EQUAL(1, it->attempts);
  TEST_ASSERT_NULL(q.peekReady(799));
  TEST_ASSERT_NOT_NULL(q.peekReady(800));
}

static void test_supersede_keeps_newest_and_position(void) {
  Q q;
  TxOptions agg = TxOptions::make(PRIO_AGGREGATE, 0);
  agg.supersede = true; agg.key = 0x4001;
  q.push(frame(1), 10, agg, 0);
  q.push(frame(9), 10, TxOptions::make(PRIO_AGGREGATE, 0), 1);   // other aggregate, after
  TEST_ASSERT_EQUAL(PUSH_SUPERSEDED, q.push(frame(2), 12, agg, 2));
  TEST_ASSERT_EQUAL(2, q.count(PRIO_AGGREGATE));
  Q::Item* it = q.peekReady(3);
  TEST_ASSERT_EQUAL(2, it->data[0]);                       // newest content...
  TEST_ASSERT_EQUAL(12, it->len);
  q.remove(it);                                            // ...in the original (first) position
  TEST_ASSERT_EQUAL(9, q.peekReady(3)->data[0]);
  TEST_ASSERT_EQUAL(1, q.stats().superseded[PRIO_AGGREGATE]);

  TxOptions other = agg; other.key = 0x4002;               // different key: no supersede
  TEST_ASSERT_EQUAL(PUSH_OK, q.push(frame(3), 10, other, 4));
}

static void test_overflow_drops_oldest_of_that_priority(void) {
  Q q;   // stream depth 2
  q.push(frame(1), 10, TxOptions::make(PRIO_STREAM, 0), 0);
  q.push(frame(2), 10, TxOptions::make(PRIO_STREAM, 0), 1);
  q.push(frame(7), 10, TxOptions::make(PRIO_ALERT, 0), 1);
  TEST_ASSERT_EQUAL(PUSH_OK_DROPPED_OLDEST, q.push(frame(3), 10, TxOptions::make(PRIO_STREAM, 0), 2));
  TEST_ASSERT_EQUAL(2, q.count(PRIO_STREAM));
  TEST_ASSERT_EQUAL(1, q.count(PRIO_ALERT));               // other priorities untouched
  TEST_ASSERT_EQUAL(1, q.stats().dropped_overflow[PRIO_STREAM]);
  q.remove(q.peekReady(3));                                // the alert
  TEST_ASSERT_EQUAL(2, q.peekReady(3)->data[0]);           // 1 was dropped
}

static void test_ttl_expiry(void) {
  Q q;
  q.push(frame(1), 10, TxOptions::make(PRIO_STREAM, 0), 0);      // default TTL 1 s
  TxOptions a = TxOptions::make(PRIO_ALERT, 0); a.ttl_ms = 5000;
  q.push(frame(2), 10, a, 0);
  q.push(frame(3), 10, TxOptions::make(PRIO_AGGREGATE, 0), 0);   // default 10 s
  TEST_ASSERT_EQUAL(3, q.total());
  q.expire(999);   TEST_ASSERT_EQUAL(3, q.total());
  q.expire(1000);  TEST_ASSERT_EQUAL(2, q.total());
  TEST_ASSERT_EQUAL(1, q.stats().dropped_ttl[PRIO_STREAM]);
  TEST_ASSERT_EQUAL(3, q.peekReady(5000)->data[0]);              // alert expired inside peekReady
  TEST_ASSERT_EQUAL(1, q.stats().dropped_ttl[PRIO_ALERT]);
}

static void test_lowest_allowed_excludes_stream(void) {
  Q q;
  q.push(frame(1), 10, TxOptions::make(PRIO_STREAM, 0), 0);
  TEST_ASSERT_NULL(q.peekReady(1, PRIO_AGGREGATE));
  TEST_ASSERT_NOT_NULL(q.peekReady(1, PRIO_STREAM));
}

static void test_cancel_by_tag(void) {
  Q q;
  TxOptions r = TxOptions::make(PRIO_ALERT, 0); r.tag = (7u << 16) | 1234u;
  q.push(frame(1), 10, r, 0);
  q.push(frame(2), 10, TxOptions::make(PRIO_ALERT, 0), 0);
  TEST_ASSERT_EQUAL(1, q.cancelByTag((7u << 16) | 1234u));
  TEST_ASSERT_EQUAL(1, q.total());
  TEST_ASSERT_EQUAL(1, q.stats().cancelled[PRIO_ALERT]);
}

static void test_rejects_bad_input(void) {
  Q q;
  TEST_ASSERT_EQUAL(PUSH_REJECTED, q.push(frame(1), 0, TxOptions::make(PRIO_ALERT, 0), 0));
  TEST_ASSERT_EQUAL(PUSH_REJECTED, q.push(frame(1), 65, TxOptions::make(PRIO_ALERT, 0), 0));
  TEST_ASSERT_EQUAL(PUSH_REJECTED, q.push(nullptr, 5, TxOptions::make(PRIO_ALERT, 0), 0));
  TEST_ASSERT_EQUAL(0, q.total());
}

static void test_millis_rollover(void) {
  Q q;
  const uint32_t t0 = 0xFFFFFF00u;
  TxOptions o = TxOptions::make(PRIO_CONTROL, t0); o.not_before_ms = t0 + 0x200;  // after the wrap
  q.push(frame(1), 10, o, t0);
  TEST_ASSERT_NULL(q.peekReady(t0 + 0x100));
  TEST_ASSERT_NOT_NULL(q.peekReady(t0 + 0x200));
  uint32_t next = 0;
  TEST_ASSERT_TRUE(q.nextReadyTime(next));
  TEST_ASSERT_EQUAL_UINT32(t0 + 0x200, next);
}

// 10 minutes of a hub at SF9: aggregates every 6 s, a focus stream every 2 s (newest wins),
// alerts at random-ish times, one radio, stream capped at 20 % airtime.
static void test_scheduler_simulation_sf9(void) {
  TxQueue<255> q;
  AirtimeBudget stream_budget(200, 4000);
  const LoRaPhy phy = phySF(9);
  uint8_t pkt[255] = {0};
  uint32_t busy_until = 0, max_alert_wait = 0, max_airtime_ms = 0;
  uint64_t stream_us = 0, total_us = 0;
  int alerts_sent = 0, aggregates_sent = 0, streams_sent = 0;
  const uint32_t alert_times[] = {7300, 61234, 61240, 299999, 300001, 455555, 599000};
  unsigned next_alert = 0;
  const uint32_t END = 600000;

  for (uint32_t now = 0; now < END; now++) {
    if (now % 6000 == 0) {
      TxOptions o = TxOptions::make(PRIO_AGGREGATE, now); o.supersede = true; o.key = 0x4001;
      q.push(pkt, 39, o, now);                         // 3-node v2 aggregate
    }
    if (now % 2000 == 500) {
      TxOptions o = TxOptions::make(PRIO_STREAM, now); o.supersede = true; o.key = 0x6014;
      q.push(pkt, 68, o, now);                         // 8 pings batched
    }
    if (next_alert < sizeof(alert_times) / sizeof(alert_times[0]) && now == alert_times[next_alert]) {
      q.push(pkt, 26, TxOptions::make(PRIO_ALERT, now), now);
      next_alert++;
    }
    if (!timeReached(now, busy_until)) continue;
    const Priority lowest = stream_budget.canSend(now) ? PRIO_STREAM : PRIO_AGGREGATE;
    TxQueue<255>::Item* it = q.peekReady(now, lowest);
    if (it == nullptr) continue;
    const uint32_t us = airtimeUs(it->len, phy);
    const uint32_t ms = (us + 999) / 1000;
    if (ms > max_airtime_ms) max_airtime_ms = ms;
    busy_until = now + ms;
    total_us += us;
    if (it->prio == PRIO_STREAM) { stream_budget.spend(us, now); stream_us += us; streams_sent++; }
    if (it->prio == PRIO_AGGREGATE) aggregates_sent++;
    if (it->prio == PRIO_ALERT) {
      alerts_sent++;
      if (now - it->enqueued_ms > max_alert_wait) max_alert_wait = now - it->enqueued_ms;
    }
    q.remove(it);
  }

  TEST_ASSERT_EQUAL(7, alerts_sent);
  TEST_ASSERT_LESS_OR_EQUAL(max_airtime_ms + 300, max_alert_wait);   // <= one packet + 2nd alert
  TEST_ASSERT_EQUAL(100, aggregates_sent);                          // none lost
  TEST_ASSERT_EQUAL(0, (int)q.stats().dropped_ttl[PRIO_AGGREGATE]);
  TEST_ASSERT_TRUE(stream_us <= (uint64_t)(0.205 * END * 1000));    // 20 % cap holds
  TEST_ASSERT_TRUE(streams_sent >= 280);                            // ~300 offered, (411 ms each)
  TEST_ASSERT_TRUE(total_us < (uint64_t)(0.30 * END * 1000));       // channel not saturated
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_priority_order);
  RUN_TEST(test_fifo_inside_priority);
  RUN_TEST(test_not_before_and_deferred_does_not_block);
  RUN_TEST(test_supersede_keeps_newest_and_position);
  RUN_TEST(test_overflow_drops_oldest_of_that_priority);
  RUN_TEST(test_ttl_expiry);
  RUN_TEST(test_lowest_allowed_excludes_stream);
  RUN_TEST(test_cancel_by_tag);
  RUN_TEST(test_rejects_bad_input);
  RUN_TEST(test_millis_rollover);
  RUN_TEST(test_scheduler_simulation_sf9);
  return UNITY_END();
}
