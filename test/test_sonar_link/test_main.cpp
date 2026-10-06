// Sonar transport: hub outbox, chalet store, relay re-pack keeps whole sonar blocks.
#include <unity.h>
#include <hub_packet.h>
#include <chalet_rx.h>
#include <sonar_sim.h>
#include <cstdio>

using namespace icemesh;
using namespace icemesh::tdma;
using namespace icemesh::sonar;

void setUp(void) {}
void tearDown(void) {}

static size_t makeBase(uint8_t node, uint16_t ping, uint8_t* b) {
  Summary s; memset(&s, 0, sizeof s); s.node = node; s.ping = ping; s.bottom_cm = 500;
  return encodeSummary(s, b, MAX_BLOCK);
}
static size_t makeData(uint8_t node, uint16_t ping0, uint8_t n, uint8_t* b, uint8_t fill_resid = 0) {
  Ping p[MAX_PINGS]; memset(p, 0, sizeof p);
  for (uint8_t i = 0; i < n; i++) {
    p[i].index = static_cast<uint16_t>(ping0 + i); p[i].bottom_cm = 500;
    p[i].n_resid = fill_resid;
    for (uint8_t r = 0; r < fill_resid; r++) { p[i].r[r].bin = static_cast<uint16_t>(r * 60 + i); p[i].r[r].level = 1; }
  }
  DataHeader h; memset(&h, 0, sizeof h); h.node = node;
  return encodeData(h, p, n, b, MAX_BLOCK);
}
static size_t makeBg(uint8_t node, uint8_t seg, uint8_t ver, uint8_t* b) {
  uint8_t lv[BINS]; for (uint16_t i = 0; i < BINS; i++) lv[i] = static_cast<uint8_t>((i / 7) & 3u);
  return encodeBgSegment(node, ver, seg, lv, b, MAX_BLOCK);
}

static void test_outbox_replace_and_priority(void) {
  SonarOutbox ob;
  uint8_t b[MAX_BLOCK];
  ob.push(b, static_cast<uint8_t>(makeBase(130, 1, b)), 0);
  ob.push(b, static_cast<uint8_t>(makeBase(130, 17, b)), 0);   // replaces
  ob.push(b, static_cast<uint8_t>(makeBg(130, 2, 0, b)), 0);
  ob.push(b, static_cast<uint8_t>(makeBg(130, 2, 0, b)), 0);   // replaces (same segment)
  ob.push(b, static_cast<uint8_t>(makeBg(130, 3, 0, b)), 0);
  ob.push(b, static_cast<uint8_t>(makeData(130, 100, 4, b)), 0);
  ob.push(b, static_cast<uint8_t>(makeData(130, 104, 4, b)), 0);
  TEST_ASSERT_EQUAL(5, ob.count());
  uint8_t pkt[MAX_PACKET];
  PacketWriter w(pkt, sizeof pkt); w.beginHub(0x42, 1, 0, 0);
  ob.fill(w, 0);
  TEST_ASSERT_EQUAL(0, ob.count());
  SectionReader rd(pkt + HDR_LEN + 1, w.size() - HDR_LEN - 1);
  uint8_t t, n; const uint8_t* v; uint8_t types[8]; int k = 0;
  while (rd.next(t, v, n) && k < 8) { TEST_ASSERT_EQUAL(SEC_SONAR, t); uint8_t nd = 0, ty = 0; peekBlock(v, n, nd, ty); types[k++] = ty; }
  TEST_ASSERT_EQUAL(5, k);
  TEST_ASSERT_EQUAL(BT_BASE, types[0]); TEST_ASSERT_EQUAL(BT_DATA, types[1]); TEST_ASSERT_EQUAL(BT_DATA, types[2]);
  TEST_ASSERT_EQUAL(BT_BG, types[3]); TEST_ASSERT_EQUAL(BT_BG, types[4]);
  Summary s; DataBlock d;
  SectionReader rd2(pkt + HDR_LEN + 1, w.size() - HDR_LEN - 1);
  rd2.next(t, v, n); TEST_ASSERT_TRUE(decodeSummary(v, n, s)); TEST_ASSERT_EQUAL(17, s.ping);
  rd2.next(t, v, n); TEST_ASSERT_TRUE(decodeData(v, n, d)); TEST_ASSERT_EQUAL(100, d.pings[0].index);   // oldest DATA first
}

static void test_outbox_whole_blocks_and_expiry(void) {
  SonarOutbox ob;
  uint8_t b[MAX_BLOCK];
  const uint8_t big = static_cast<uint8_t>(makeData(131, 0, 8, b, 7));
  ob.push(b, big, 10);
  const uint8_t small = static_cast<uint8_t>(makeBase(132, 0, b));
  ob.push(b, small, 10);
  uint8_t pkt[MAX_PACKET];
  PacketWriter w(pkt, HDR_LEN + 1 + 2 + small + 5);   // room for the BASE only
  w.beginHub(0x42, 1, 10, 0);
  ob.fill(w, 10);
  TEST_ASSERT_EQUAL(1, ob.count());                    // DATA did not fit: kept whole for the next packet
  TEST_ASSERT_EQUAL(HDR_LEN + 1 + 2 + small, w.size());
  PacketWriter w2(pkt, sizeof pkt); w2.beginHub(0x42, 1, 20, 0);
  ob.fill(w2, 20);                                     // 10 frames later: stale, dropped
  TEST_ASSERT_EQUAL(0, ob.count());
  TEST_ASSERT_EQUAL(1, ob.expired);
}

static void test_outbox_eviction(void) {
  SonarOutbox ob;
  uint8_t b[MAX_BLOCK];
  for (uint8_t i = 0; i < SonarOutbox::CAP; i++) ob.push(b, static_cast<uint8_t>(makeBase(static_cast<uint8_t>(140 + i), 0, b)), 0);
  ob.push(b, static_cast<uint8_t>(makeData(150, 0, 4, b)), 0);   // full: evicts the oldest BASE
  TEST_ASSERT_EQUAL(SonarOutbox::CAP, ob.count());
  TEST_ASSERT_EQUAL(1, ob.dropped);
  uint8_t junk[3] = {1, 0xF0, 0};
  TEST_ASSERT_FALSE(ob.push(junk, 3, 0));
}

static void test_store_dedup_restart_bg(void) {
  SonarStore<4, 32> st;
  uint8_t b[MAX_BLOCK];
  const uint8_t n = static_cast<uint8_t>(makeData(133, 1000, 4, b));
  st.onSonarBlock(1, b, n, 5);
  st.onSonarBlock(2, b, n, 5);                         // same block again (direct + relay)
  const StoredPing* out[64];
  TEST_ASSERT_EQUAL(4, st.pingsSince(133, 0, out, 64));
  TEST_ASSERT_EQUAL(4, st.old_pings);
  const uint32_t mark = st.lastSeq();
  const uint8_t n2 = static_cast<uint8_t>(makeData(133, 1004, 4, b));
  st.onSonarBlock(1, b, n2, 6);
  TEST_ASSERT_EQUAL(4, st.pingsSince(133, mark, out, 64));
  TEST_ASSERT_EQUAL(1004, out[0]->p.index);
  const uint8_t n3 = static_cast<uint8_t>(makeData(133, 0, 4, b));   // node restarted: counter back to 0
  st.onSonarBlock(1, b, n3, 7);
  TEST_ASSERT_EQUAL(12, st.pingsSince(133, 0, out, 64));
  // node restarts again right after: index 0 is just behind the last one (3): accepted after RESTART_RUN old pings
  const uint8_t r1 = static_cast<uint8_t>(makeData(133, 0, 4, b)); st.onSonarBlock(1, b, r1, 7);   // 4 old in a row
  const uint8_t r2 = static_cast<uint8_t>(makeData(133, 0, 4, b)); st.onSonarBlock(1, b, r2, 7);   // 8th old -> accepted from here
  TEST_ASSERT_TRUE(st.pingsSince(133, 0, out, 64) > 12);
  // background: segments of version 3, then a new version resets the mask
  for (uint8_t s = 0; s < 8; s++) { const uint8_t m = static_cast<uint8_t>(makeBg(133, s, 3, b)); st.onSonarBlock(1, b, m, 8); }
  TEST_ASSERT_EQUAL(0xFF, st.find(133)->bg_mask);
  TEST_ASSERT_EQUAL(3, st.find(133)->bg_ver);
  const uint8_t m = static_cast<uint8_t>(makeBg(133, 0, 4, b)); st.onSonarBlock(1, b, m, 9);
  TEST_ASSERT_EQUAL(1, st.find(133)->bg_mask);
  TEST_ASSERT_TRUE(st.find(133)->has_sum);
  TEST_ASSERT_EQUAL(1, st.find(133)->hub);
  uint8_t bad[4] = {133, 0x20, 0xFF, 0xFF};
  const uint32_t bad_before = st.blocks_bad;
  st.onSonarBlock(1, bad, 4, 9);
  TEST_ASSERT_EQUAL(bad_before + 1, st.blocks_bad);
}

static void test_relay_keeps_whole_sonar_blocks(void) {
  // remote hub 7: 10 line records + 3 sonar blocks; relay room too small for all of it
  LineTable<24> t7;
  for (uint8_t nd = 1; nd <= 10; nd++) t7.observe(nd, LS_IDLE, 0, 0, 70, 0);
  SonarOutbox ob;
  uint8_t b[MAX_BLOCK];
  ob.push(b, static_cast<uint8_t>(makeBase(150, 3, b)), 1);
  ob.push(b, static_cast<uint8_t>(makeData(151, 40, 8, b, 7)), 1);
  ob.push(b, static_cast<uint8_t>(makeBg(151, 0, 0, b)), 1);
  HubBuild in7; memset(&in7, 0, sizeof in7); in7.network_id = 0x42; in7.self = 7; in7.frame = 1; in7.allowance = 200; in7.sonar = &ob;
  RelayItem item; item.len = static_cast<uint8_t>(buildHubPacket(in7, t7, item.data, sizeof item.data).len);
  uint8_t out[MAX_PACKET]; bool partial = false;
  const uint8_t n = packRelayValue(item.data, item.len, out, 70, partial);
  TEST_ASSERT_TRUE(partial);
  TEST_ASSERT_TRUE(n > 3 && n <= 70);
  SectionReader rd(out + 3, n - 3u);
  uint8_t t, len; const uint8_t* v; int sonar_ok = 0;
  while (rd.next(t, v, len)) {
    if (t == SEC_LINE) TEST_ASSERT_EQUAL(0, len % 3);
    if (t == SEC_SONAR) {
      uint8_t nd = 0, ty = 0; TEST_ASSERT_TRUE(peekBlock(v, len, nd, ty));
      Summary s; DataBlock d; uint8_t a, c, e; uint8_t lv[BINS];
      const bool ok = (ty == BT_BASE && decodeSummary(v, len, s)) || (ty == BT_DATA && decodeData(v, len, d)) ||
                      (ty == BT_BG && decodeBgSegment(v, len, a, c, e, lv));
      TEST_ASSERT_TRUE(ok);
      sonar_ok++;
    }
  }
  TEST_ASSERT_FALSE(rd.malformed());
  TEST_ASSERT_TRUE(sonar_ok >= 1);
  // the chalet gets the blocks through the relay hub's packet
  LineTable<24> t5;
  HubBuild in5; memset(&in5, 0, sizeof in5); in5.network_id = 0x42; in5.self = 5; in5.frame = 1; in5.allowance = 255;
  in5.relayed = &item; in5.n_relayed = 1;
  uint8_t pkt[MAX_PACKET];
  const size_t plen = buildHubPacket(in5, t5, pkt, sizeof pkt).len;
  ChaletPlanner<6> planner; ChaletNodes<16> nodes; SonarStore<8, 64> store;
  planner.cfg.network_id = 0x42; planner.cfg.self_id = 100;
  ChaletRxResult rx = consumeHubPacket(pkt, plen, 0x42, 1, planner, nodes, &store);
  TEST_ASSERT_EQUAL(3, rx.n_sonar);
  TEST_ASSERT_NOT_NULL(store.find(150));
  TEST_ASSERT_EQUAL(7, store.find(151)->hub);            // attributed to the remote hub, not the relay
}

static void test_planner_focus_allowance(void) {
  ChaletPlanner<6> p;
  p.cfg.network_id = 0x42; p.cfg.self_id = 100;
  for (uint16_t f = 1; f <= 3; f++) { p.onDirectPacket(1, f, -90, 20, MODE_SF9_BW500, false); p.onDirectPacket(2, f, -90, 20, MODE_SF9_BW500, false); }
  p.cfg.focus_hub = 2;
  Beacon b; p.buildBeacon(4, b);
  uint8_t a1 = 0, a2 = 0;
  for (uint8_t i = 0; i < b.n_slots; i++) if (b.slots[i].kind == SLOT_HUB) { if (b.slots[i].owner == 1) a1 = b.slots[i].allowance; else a2 = b.slots[i].allowance; }
  TEST_ASSERT_EQUAL(96, a1);
  TEST_ASSERT_EQUAL(180, a2);
  TEST_ASSERT_TRUE(planFits(b.slots, b.n_slots, 1000000UL, static_cast<uint16_t>(beaconSize(b.n_slots, b.n_acks))));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_outbox_replace_and_priority);
  RUN_TEST(test_outbox_whole_blocks_and_expiry);
  RUN_TEST(test_outbox_eviction);
  RUN_TEST(test_store_dedup_restart_bg);
  RUN_TEST(test_relay_keeps_whole_sonar_blocks);
  RUN_TEST(test_planner_focus_allowance);
  return UNITY_END();
}
