#include <unity.h>
#include <tdma_proto.h>
#include <tdma_schedule.h>
#include <radio_modes.h>

using namespace icemesh;
using namespace icemesh::tdma;

void setUp(void) {}
void tearDown(void) {}

static Beacon sampleBeacon() {
  Beacon b;
  memset(&b, 0, sizeof(b));
  b.network_id = 0x42; b.src = 100; b.frame = 0xBEEF; b.flags = BF_TEST | BF_SILENCED; b.cmd = CMD_RESET_ALL;
  b.cmd_seq = 7; b.silence_10s = 30; b.focus_node = 12; b.frame_10ms = 100; b.test_mode = TEST_ROTATE;
  b.n_slots = 3;
  b.slots[0] = {SLOT_ECHO, 2, MODE_SF9_BW500, 40, 0};
  b.slots[1] = {SLOT_JOIN, 0, MODE_SF9_BW500, 7, 0};
  b.slots[2] = {SLOT_HUB, 5, MODE_SF7_BW500, 96, 2};
  b.n_acks = 2;
  b.acks[0] = {5, 21, 3};
  b.acks[1] = {2, 22, 15};
  return b;
}

static void test_beacon_round_trip(void) {
  uint8_t buf[MAX_PACKET];
  const Beacon b = sampleBeacon();
  const size_t n = encodeBeacon(b, PT_BEACON, buf, sizeof buf);
  TEST_ASSERT_EQUAL(beaconSize(3, 2), n);
  TEST_ASSERT_EQUAL(14 + 15 + 1 + 6, n);
  Beacon d; PacketType t;
  TEST_ASSERT_TRUE(decodeBeacon(buf, n, 0x42, d, t));
  TEST_ASSERT_EQUAL(PT_BEACON, t);
  TEST_ASSERT_EQUAL_UINT16(0xBEEF, d.frame);
  TEST_ASSERT_EQUAL(100, d.src);
  TEST_ASSERT_EQUAL(BF_TEST | BF_SILENCED, d.flags);
  TEST_ASSERT_EQUAL(CMD_RESET_ALL, d.cmd);
  TEST_ASSERT_EQUAL(12, d.focus_node);
  TEST_ASSERT_EQUAL(3, d.n_slots);
  TEST_ASSERT_EQUAL(SLOT_HUB, d.slots[2].kind);
  TEST_ASSERT_EQUAL(2, d.slots[2].via);
  TEST_ASSERT_EQUAL(96, d.slots[2].allowance);
  TEST_ASSERT_EQUAL(2, d.n_acks);
  TEST_ASSERT_EQUAL(15, d.acks[1].seq);
}

static void test_echo_type_and_rejects(void) {
  uint8_t buf[MAX_PACKET];
  Beacon b = sampleBeacon(); b.src = 2;
  const size_t n = encodeBeacon(b, PT_ECHO, buf, sizeof buf);
  Beacon d; PacketType t;
  TEST_ASSERT_TRUE(decodeBeacon(buf, n, 0x42, d, t));
  TEST_ASSERT_EQUAL(PT_ECHO, t);
  TEST_ASSERT_FALSE(decodeBeacon(buf, n, 0x43, d, t));       // other network
  TEST_ASSERT_FALSE(decodeBeacon(buf, n - 1, 0x42, d, t));   // truncated
  buf[1] = 0xF1;
  TEST_ASSERT_FALSE(decodeBeacon(buf, n, 0x42, d, t));       // wrong version
  TEST_ASSERT_EQUAL(0, encodeBeacon(b, PT_HUB, buf, sizeof buf));
  TEST_ASSERT_EQUAL(0, encodeBeacon(b, PT_BEACON, buf, 10));
}

static void test_v1_packets_rejected(void) {
  // v1 LoRa aggregate: [0x42, sender, 0x40, ...] ; v1 alert 0x41 — byte 1 is never 0xF2 for valid v1 IDs < 0xF2
  uint8_t v1[20] = {0x42, 1, 0x40, 0, 0, 0};
  Header h;
  TEST_ASSERT_FALSE(readHeader(v1, sizeof v1, 0x42, h));
}

static void test_line_record_bits(void) {
  LineRecord r = {77, LS_RUNNING, 13, true, 45, RF_LOWBAT};
  uint8_t b[3];
  encodeLine(r, b);
  LineRecord d;
  decodeLine(b, d);
  TEST_ASSERT_EQUAL(77, d.node);
  TEST_ASSERT_EQUAL(LS_RUNNING, d.state);
  TEST_ASSERT_EQUAL(13, d.seq);
  TEST_ASSERT_TRUE(d.pending);
  TEST_ASSERT_EQUAL(31, d.turns);           // saturates at 31
  TEST_ASSERT_EQUAL(RF_LOWBAT, d.flags);
  for (uint8_t st = 0; st < 8; st++) for (uint8_t sq = 0; sq < 16; sq++) {
    LineRecord x = {1, st, sq, (sq & 1) != 0, sq, static_cast<uint8_t>(st & 7)};
    encodeLine(x, b); decodeLine(b, d);
    TEST_ASSERT_EQUAL(st, d.state); TEST_ASSERT_EQUAL(sq, d.seq); TEST_ASSERT_EQUAL(x.pending, d.pending);
  }
}

static void test_health_round_trip(void) {
  Health h; memset(&h, 0, sizeof(h));
  h.battery_mv = 4123; h.uptime_min = 600; h.fw = 3; h.beacon_rssi = -97; h.beacon_snr_q4 = -30;
  h.beacon_lost = 5; h.sync_err_10us = -123; h.n_nb = 2;
  h.nb[0] = {7, -110, -40}; h.nb[1] = {9, -80, 28};
  uint8_t b[64];
  const uint8_t n = encodeHealth(h, b, sizeof b);
  TEST_ASSERT_EQUAL(HEALTH_FIXED_LEN + 1 + 6, n);
  Health d;
  TEST_ASSERT_TRUE(decodeHealth(b, n, d));
  TEST_ASSERT_EQUAL(4123, d.battery_mv);
  TEST_ASSERT_EQUAL(-97, d.beacon_rssi);
  TEST_ASSERT_EQUAL(-123, d.sync_err_10us);
  TEST_ASSERT_EQUAL(2, d.n_nb);
  TEST_ASSERT_EQUAL(-110, d.nb[0].rssi);
  TEST_ASSERT_FALSE(decodeHealth(b, n - 1, d));
  TEST_ASSERT_EQUAL(0, encodeHealth(h, b, 10));
}

static void test_writer_reader(void) {
  uint8_t buf[40];
  PacketWriter w(buf, sizeof buf);
  TEST_ASSERT_TRUE(w.beginHub(0x42, 5, 300, HF_SILENCE_ON));
  const uint8_t a[3] = {1, 2, 3};
  TEST_ASSERT_TRUE(w.add(SEC_LINE, a, 3));
  TEST_ASSERT_EQUAL(40 - 7 - 5 - 2, w.freeForValue());
  uint8_t big[40] = {0};
  TEST_ASSERT_FALSE(w.add(SEC_TEST, big, 27));               // 7+5+2+27 = 41 > 40
  TEST_ASSERT_TRUE(w.add(SEC_TEST, big, 26));                // exactly full
  TEST_ASSERT_EQUAL(40, w.size());
  Header h;
  TEST_ASSERT_TRUE(readHeader(buf, w.size(), 0x42, h));
  TEST_ASSERT_EQUAL(PT_HUB, h.type);
  TEST_ASSERT_EQUAL(300, h.frame);
  SectionReader rd(buf + HDR_LEN + 1, w.size() - HDR_LEN - 1);
  uint8_t t, n; const uint8_t* v;
  TEST_ASSERT_TRUE(rd.next(t, v, n)); TEST_ASSERT_EQUAL(SEC_LINE, t); TEST_ASSERT_EQUAL(3, n); TEST_ASSERT_EQUAL(2, v[1]);
  TEST_ASSERT_TRUE(rd.next(t, v, n)); TEST_ASSERT_EQUAL(SEC_TEST, t); TEST_ASSERT_EQUAL(26, n);
  TEST_ASSERT_FALSE(rd.next(t, v, n));
  TEST_ASSERT_FALSE(rd.malformed());
  SectionReader bad(buf + HDR_LEN + 1, w.size() - HDR_LEN - 2);   // last section cut by 1 byte
  TEST_ASSERT_TRUE(bad.next(t, v, n));
  TEST_ASSERT_FALSE(bad.next(t, v, n));
  TEST_ASSERT_TRUE(bad.malformed());
}

static void test_modes(void) {
  TEST_ASSERT_EQUAL(9, modeInfo(MODE_SF9_BW500).sf);
  TEST_ASSERT_EQUAL(500, modeInfo(MODE_SF7_BW500).bw_khz);
  // 500 kHz is 4x faster than 125 kHz at the same SF (same symbols, 1/4 symbol time)
  const uint32_t a500 = modeAirtimeUs(MODE_SF9_BW500, 108);
  TEST_ASSERT_UINT32_WITHIN(500, airtimeUs(108, phySF(9)) / 4, a500);
  TEST_ASSERT_EQUAL(MODE_SF9_BW500, slowerMode(MODE_SF9_BW500));
  TEST_ASSERT_EQUAL(MODE_SF8_BW500, fasterMode(MODE_SF9_BW500));
  TEST_ASSERT_EQUAL(MODE_SF7_BW500, fasterMode(MODE_SF7_BW500));
  // a full 255 B packet stays under 400 ms in every mode
  for (uint8_t m = 0; m < MODE_COUNT; m++) TEST_ASSERT_LESS_OR_EQUAL(400000, modeAirtimeUs(static_cast<RadioMode>(m), 255));
}

static void test_schedule_times_and_ref(void) {
  Slot s[3] = {{SLOT_JOIN, 0, MODE_SF9_BW500, 7, 0}, {SLOT_HUB, 5, MODE_SF9_BW500, 96, 0}, {SLOT_HUB, 6, MODE_SF7_BW500, 96, 0}};
  SlotTime t[3];
  const uint32_t end = computeSlotTimes(s, 3, t);
  TEST_ASSERT_EQUAL_UINT32(BEACON_GAP_US, t[0].start);
  TEST_ASSERT_EQUAL_UINT32(t[0].start + LEAD_US, t[0].tx);
  TEST_ASSERT_EQUAL_UINT32(t[0].end, t[1].start);
  TEST_ASSERT_EQUAL_UINT32(t[1].start + LEAD_US + modeAirtimeUs(MODE_SF9_BW500, 96) + TAIL_US, t[1].end);
  TEST_ASSERT_EQUAL_UINT32(t[2].end, end);
  // receiver recovers REF from a 60-byte packet sent at the start of slot 1
  const uint32_t ref = 123456789u;
  const uint32_t rx_end = ref + t[1].tx + modeAirtimeUs(MODE_SF9_BW500, 60);
  TEST_ASSERT_EQUAL_UINT32(ref, refFromSlotPacket(rx_end, t[1], MODE_SF9_BW500, 60));
  TEST_ASSERT_TRUE(planFits(s, 3, 1000000, 80000));
  Slot many[MAX_SLOTS];
  for (uint8_t i = 0; i < MAX_SLOTS; i++) many[i] = {SLOT_HUB, (uint8_t)(i + 1), MODE_SF9_BW500, 255, 0};
  TEST_ASSERT_FALSE(planFits(many, MAX_SLOTS, 1000000, 80000));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_beacon_round_trip);
  RUN_TEST(test_echo_type_and_rejects);
  RUN_TEST(test_v1_packets_rejected);
  RUN_TEST(test_line_record_bits);
  RUN_TEST(test_health_round_trip);
  RUN_TEST(test_writer_reader);
  RUN_TEST(test_modes);
  RUN_TEST(test_schedule_times_and_ref);
  return UNITY_END();
}
