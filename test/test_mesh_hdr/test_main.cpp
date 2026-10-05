#include <unity.h>
#include <mesh_hdr.h>
#include <seq.h>

using namespace icemesh;

void setUp(void) {}
void tearDown(void) {}

static Header sample() {
  Header h;
  h.network_id = 0x42; h.origin = 7; h.cls = CLASS_RELIABLE; h.hops = 1; h.ack_req = true;
  h.seq = 0xBEEF; h.sender = 9; h.type = 0x41;
  return h;
}

static void test_layout_matches_spec(void) {
  uint8_t b[HDR_LEN];
  TEST_ASSERT_EQUAL(HDR_LEN, encodeHeader(sample(), b, sizeof b));
  const uint8_t expect[HDR_LEN] = {0x42, 7, 0xF2, (0 << 6) | (1 << 4) | 0x08, 0xEF, 0xBE, 9, 0x41};
  TEST_ASSERT_EQUAL_MEMORY(expect, b, HDR_LEN);
}

static void test_round_trip_all_classes(void) {
  const TrafficClass classes[] = {CLASS_RELIABLE, CLASS_STREAM, CLASS_LINK};
  for (unsigned i = 0; i < 3; i++) {
    Header h = sample();
    h.cls = classes[i];
    h.hops = (h.cls == CLASS_LINK) ? 0 : 3;
    h.ack_req = (i == 1);
    uint8_t b[HDR_LEN];
    TEST_ASSERT_EQUAL(HDR_LEN, encodeHeader(h, b, sizeof b));
    Header d;
    TEST_ASSERT_EQUAL(DECODE_OK, decodeHeader(b, sizeof b, 0x42, d));
    TEST_ASSERT_EQUAL(h.cls, d.cls);
    TEST_ASSERT_EQUAL(h.hops, d.hops);
    TEST_ASSERT_EQUAL(h.ack_req, d.ack_req);
    TEST_ASSERT_EQUAL_UINT16(h.seq, d.seq);
    TEST_ASSERT_EQUAL(h.origin, d.origin);
    TEST_ASSERT_EQUAL(h.sender, d.sender);
    TEST_ASSERT_EQUAL(h.type, d.type);
  }
}

static void test_encode_rejects_invalid(void) {
  uint8_t b[HDR_LEN];
  Header h = sample(); h.hops = 4;               TEST_ASSERT_EQUAL(0, encodeHeader(h, b, sizeof b));
  h = sample(); h.origin = ID_NONE;              TEST_ASSERT_EQUAL(0, encodeHeader(h, b, sizeof b));
  h = sample(); h.origin = ID_BROADCAST;         TEST_ASSERT_EQUAL(0, encodeHeader(h, b, sizeof b));
  h = sample(); h.cls = CLASS_LINK; h.hops = 1;  TEST_ASSERT_EQUAL(0, encodeHeader(h, b, sizeof b));
  h = sample();                                  TEST_ASSERT_EQUAL(0, encodeHeader(h, b, HDR_LEN - 1));
}

static void test_decode_rejects(void) {
  uint8_t b[HDR_LEN];
  Header d;
  encodeHeader(sample(), b, sizeof b);
  TEST_ASSERT_EQUAL(DECODE_SHORT, decodeHeader(b, HDR_LEN - 1, 0x42, d));
  TEST_ASSERT_EQUAL(DECODE_NETWORK, decodeHeader(b, sizeof b, 0x43, d));
  uint8_t c[HDR_LEN];
  for (unsigned i = 0; i < HDR_LEN; i++) c[i] = b[i];
  c[2] = 0xF1;  TEST_ASSERT_EQUAL(DECODE_VERSION, decodeHeader(c, sizeof c, 0x42, d));
  c[2] = 0xF2; c[3] = 0xC0;  TEST_ASSERT_EQUAL(DECODE_CLASS, decodeHeader(c, sizeof c, 0x42, d));
  c[3] = (2 << 6) | (1 << 4);  TEST_ASSERT_EQUAL(DECODE_HOPS, decodeHeader(c, sizeof c, 0x42, d));
  c[3] = 0; c[1] = 0;        TEST_ASSERT_EQUAL(DECODE_ORIGIN, decodeHeader(c, sizeof c, 0x42, d));
}

static void test_v1_packets_are_not_v2(void) {
  // v1 aggregate header: network_id, sender_id, msg_type(0x40)... ; v1 alert 0x41; config 0x50
  const uint8_t v1types[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x40, 0x41, 0x42, 0x50, 0x51, 0x80, 0x81, 0x90};
  for (unsigned i = 0; i < sizeof v1types; i++) {
    uint8_t p[HDR_LEN] = {0x42, 1, v1types[i], 0, 0, 0, 0, 0};
    Header d;
    TEST_ASSERT_EQUAL(DECODE_VERSION, decodeHeader(p, sizeof p, 0x42, d));
  }
}

static void test_reserved_ctl_bits_ignored(void) {
  uint8_t b[HDR_LEN];
  encodeHeader(sample(), b, sizeof b);
  b[3] |= 0x07;
  Header d;
  TEST_ASSERT_EQUAL(DECODE_OK, decodeHeader(b, sizeof b, 0x42, d));
  TEST_ASSERT_EQUAL(1, d.hops);
}

static void test_prepare_relay(void) {
  Header h = sample(); h.hops = 0;
  uint8_t b[HDR_LEN];
  encodeHeader(h, b, sizeof b);
  for (uint8_t expect = 1; expect <= MAX_HOPS; expect++) {
    TEST_ASSERT_TRUE(prepareRelay(b, sizeof b, 33));
    Header d;
    TEST_ASSERT_EQUAL(DECODE_OK, decodeHeader(b, sizeof b, 0x42, d));
    TEST_ASSERT_EQUAL(expect, d.hops);
    TEST_ASSERT_EQUAL(33, d.sender);
    TEST_ASSERT_EQUAL(7, d.origin);       // origin never changes
    TEST_ASSERT_TRUE(d.ack_req);          // other ctl bits preserved
  }
  TEST_ASSERT_FALSE(prepareRelay(b, sizeof b, 33));  // hop limit reached

  Header l = sample(); l.cls = CLASS_LINK; l.hops = 0;
  encodeHeader(l, b, sizeof b);
  TEST_ASSERT_FALSE(prepareRelay(b, sizeof b, 33));  // LINK never relayed
}

static void test_seq_diff_wraps(void) {
  TEST_ASSERT_EQUAL(1, seqDiff(0, 65535));
  TEST_ASSERT_EQUAL(-1, seqDiff(65535, 0));
  TEST_ASSERT_EQUAL(10, seqDiff(5, 65531));
  TEST_ASSERT_TRUE(seqNewer(2, 65534));
  TEST_ASSERT_FALSE(seqNewer(7, 7));
  TEST_ASSERT_EQUAL(32767, seqDiff(32767, 0));
  TEST_ASSERT_EQUAL(-32768, seqDiff(32768, 0));
}

static void test_time_reached_wraps(void) {
  TEST_ASSERT_TRUE(timeReached(100, 100));
  TEST_ASSERT_FALSE(timeReached(99, 100));
  TEST_ASSERT_TRUE(timeReached(5, 0xFFFFFFF0u));    // millis() rolled over
  TEST_ASSERT_FALSE(timeReached(0xFFFFFFF0u, 5));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_layout_matches_spec);
  RUN_TEST(test_round_trip_all_classes);
  RUN_TEST(test_encode_rejects_invalid);
  RUN_TEST(test_decode_rejects);
  RUN_TEST(test_v1_packets_are_not_v2);
  RUN_TEST(test_reserved_ctl_bits_ignored);
  RUN_TEST(test_prepare_relay);
  RUN_TEST(test_seq_diff_wraps);
  RUN_TEST(test_time_reached_wraps);
  return UNITY_END();
}
