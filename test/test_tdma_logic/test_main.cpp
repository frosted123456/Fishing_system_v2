#include <unity.h>
#include <line_table.h>
#include <chalet_nodes.h>
#include <hub_sync.h>
#include <chalet_planner.h>
#include <hub_packet.h>
#include <chalet_rx.h>

using namespace icemesh;
using namespace icemesh::tdma;

void setUp(void) {}
void tearDown(void) {}

// ---------------- LineTable (hub) ----------------
static void test_line_event_and_ack(void) {
  LineTable<8> t;
  TEST_ASSERT_FALSE(t.observe(10, LS_IDLE, 0, 0, 90, 0));      // new idle node: no event
  TEST_ASSERT_FALSE(t.anyPending());
  TEST_ASSERT_TRUE(t.observe(10, LS_TRIPPED, 0, 0, 90, 100));  // event seq 1
  TEST_ASSERT_FALSE(t.observe(10, LS_TRIPPED, 2, 0, 90, 200)); // same state: no new event
  LineRecord r[8];
  TEST_ASSERT_EQUAL(1, t.records(r, 8));
  TEST_ASSERT_EQUAL(1, r[0].seq); TEST_ASSERT_TRUE(r[0].pending); TEST_ASSERT_EQUAL(2, r[0].turns);
  t.ack(10, 1);
  t.records(r, 8);
  TEST_ASSERT_FALSE(r[0].pending);
}

static void test_ack_of_older_event_does_not_clear_newer(void) {
  LineTable<8> t;
  t.observe(10, LS_IDLE, 0, 0, 90, 0);
  t.observe(10, LS_TRIPPED, 0, 0, 90, 1);   // seq 1
  t.observe(10, LS_RUNNING, 0, 0, 90, 2);   // seq 2, before the ack for seq 1 arrived
  t.ack(10, 1);                             // late ack of the older event
  LineRecord r[8];
  t.records(r, 8);
  TEST_ASSERT_EQUAL(2, r[0].seq);
  TEST_ASSERT_TRUE(r[0].pending);           // the bitmap design would have lost this
  t.ack(10, 2);
  t.records(r, 8);
  TEST_ASSERT_FALSE(r[0].pending);
}

static void test_seq_wraps_4_bits(void) {
  LineTable<4> t;
  t.observe(1, LS_IDLE, 0, 0, 0, 0);
  for (int i = 0; i < 20; i++) t.observe(1, (i & 1) ? LS_IDLE : LS_TRIPPED, 0, 0, 0, i);
  LineRecord r[4];
  t.records(r, 4);
  TEST_ASSERT_EQUAL(20 % 16, r[0].seq);
}

static void test_pending_first_and_expiry(void) {
  LineTable<4> t;
  t.observe(1, LS_IDLE, 0, 0, 0, 0);
  t.observe(2, LS_IDLE, 0, 0, 0, 0);
  t.observe(3, LS_IDLE, 0, 0, 0, 0);
  t.observe(3, LS_TRIPPED, 0, 0, 0, 10);
  LineRecord r[4];
  TEST_ASSERT_EQUAL(3, t.records(r, 4));
  TEST_ASSERT_EQUAL(3, r[0].node);          // pending first
  TEST_ASSERT_EQUAL(1, t.records(r, 1));    // truncated to 1 -> still the pending one
  TEST_ASSERT_EQUAL(3, r[0].node);
  TEST_ASSERT_EQUAL(2, t.expire(400000, 300000));   // 1 and 2 dropped, 3 kept (pending)
  TEST_ASSERT_EQUAL(1, t.count());
}

static void test_table_full_never_drops_pending(void) {
  LineTable<2> t;
  t.observe(1, LS_IDLE, 0, 0, 0, 0);
  t.observe(1, LS_TRIPPED, 0, 0, 0, 1);
  t.observe(2, LS_IDLE, 0, 0, 0, 2);
  t.observe(2, LS_TRIPPED, 0, 0, 0, 3);
  TEST_ASSERT_FALSE(t.observe(3, LS_TRIPPED, 0, 0, 0, 4));   // no room: both entries pending
  TEST_ASSERT_EQUAL(2, t.count());
  t.ack(1, 1);
  TEST_ASSERT_TRUE(t.observe(3, LS_TRIPPED, 0, 0, 0, 5));    // replaces acked node 1
}

// ---------------- ChaletNodes ----------------
static LineRecord rec(uint8_t node, uint8_t st, uint8_t seq) { LineRecord r = {node, st, seq, true, 0, 0}; return r; }

static void test_owner_and_duplicates(void) {
  ChaletNodes<8> n;
  NodeChange ch;
  TEST_ASSERT_TRUE(n.onRecord(1, rec(40, LS_IDLE, 0), 10, ch));      // first report
  TEST_ASSERT_EQUAL(STATE_UNKNOWN, ch.old_state);
  TEST_ASSERT_FALSE(n.onRecord(2, rec(40, LS_TRIPPED, 5), 10, ch));  // hub 2 is not owner: ignored
  TEST_ASSERT_EQUAL(LS_IDLE, n.find(40)->state);
  TEST_ASSERT_TRUE(n.onRecord(1, rec(40, LS_TRIPPED, 1), 11, ch));   // owner reports the trip
  TEST_ASSERT_EQUAL(LS_TRIPPED, ch.new_state);
  TEST_ASSERT_FALSE(n.onRecord(1, rec(40, LS_TRIPPED, 1), 12, ch));  // repeat: no change
}

static void test_owner_moves_when_owner_loses_node(void) {
  ChaletNodes<8> n;
  NodeChange ch;
  n.onRecord(1, rec(40, LS_TRIPPED, 1), 10, ch);
  TEST_ASSERT_FALSE(n.onRecord(2, rec(40, LS_TRIPPED, 3), 11, ch));
  n.onRecord(1, rec(40, LS_OFFLINE, 2), 12, ch);                     // owner lost it
  TEST_ASSERT_EQUAL(LS_OFFLINE, n.find(40)->state);
  TEST_ASSERT_TRUE(n.onRecord(2, rec(40, LS_TRIPPED, 3), 13, ch));   // hub 2 takes over
  TEST_ASSERT_EQUAL(2, n.find(40)->owner);
  // owner silent > OWNER_TIMEOUT: another hub takes over without an OFFLINE step
  TEST_ASSERT_TRUE(n.onRecord(1, rec(40, LS_IDLE, 4), 13 + ChaletNodes<8>::OWNER_TIMEOUT_FRAMES + 1, ch));
  TEST_ASSERT_EQUAL(1, n.find(40)->owner);
}

static void test_node_offline_after_timeout(void) {
  ChaletNodes<8> n;
  NodeChange ch, out[4];
  n.onRecord(1, rec(7, LS_IDLE, 0), 100, ch);
  TEST_ASSERT_EQUAL(0, n.expire(100 + ChaletNodes<8>::OFFLINE_TIMEOUT_FRAMES, out, 4));
  TEST_ASSERT_EQUAL(1, n.expire(101 + ChaletNodes<8>::OFFLINE_TIMEOUT_FRAMES, out, 4));
  TEST_ASSERT_EQUAL(LS_OFFLINE, out[0].new_state);
  TEST_ASSERT_EQUAL(0, n.expire(500, out, 4));                       // reported once
}

// ---------------- HubSync ----------------
static void test_hub_sync(void) {
  HubSync s;
  TEST_ASSERT_FALSE(s.synced());
  s.onBeacon(1000000, 10, 1000000);
  TEST_ASSERT_TRUE(s.mayTransmit());
  s.onBeacon(2000150, 11, 1000000);
  TEST_ASSERT_EQUAL(150, s.lastErrUs());
  s.onMissedBeacon();                                   // free-run frame 12
  TEST_ASSERT_TRUE(s.mayTransmit());
  TEST_ASSERT_EQUAL(12, s.frame());
  TEST_ASSERT_EQUAL_UINT32(3000150, s.ref());
  s.onMissedBeacon();
  TEST_ASSERT_FALSE(s.mayTransmit());                   // 2 misses: listen only
  TEST_ASSERT_TRUE(s.synced());
  s.onMissedBeacon(); s.onMissedBeacon();
  TEST_ASSERT_FALSE(s.synced());                        // > MAX_FREERUN: search again
  TEST_ASSERT_EQUAL(4, s.lostLast64());
  s.onBeacon(0xFFFFFF00u, 20, 1000000);                 // 32-bit µs wrap
  s.onBeacon(0xFFFFFF00u + 1000000u, 21, 1000000);
  TEST_ASSERT_EQUAL(0, s.lastErrUs());
}

// ---------------- Planner ----------------
typedef ChaletPlanner<6> Planner;
static Planner makePlanner() {
  Planner p;
  p.cfg.network_id = 0x42; p.cfg.self_id = 100; p.cfg.allowance = 96; p.cfg.test_allowance = 160; p.cfg.frame_ms = 1000;
  return p;
}
static int findSlot(const Beacon& b, uint8_t kind, uint8_t owner) {
  for (uint8_t i = 0; i < b.n_slots; i++) if (b.slots[i].kind == kind && b.slots[i].owner == owner) return i;
  return -1;
}

static void test_plan_direct_hubs_and_join(void) {
  Planner p = makePlanner();
  Beacon b;
  p.buildBeacon(1, b);
  TEST_ASSERT_EQUAL(1, b.n_slots);                       // join slot only
  TEST_ASSERT_EQUAL(SLOT_JOIN, b.slots[0].kind);
  p.onJoin(5, 0, -90, 1);
  p.onJoin(6, 0, -95, 1);
  p.buildBeacon(2, b);
  TEST_ASSERT_EQUAL(3, b.n_slots);
  TEST_ASSERT_TRUE(findSlot(b, SLOT_HUB, 5) >= 0);
  TEST_ASSERT_EQUAL(MODE_SF9_BW500, b.slots[findSlot(b, SLOT_HUB, 6)].mode);
  TEST_ASSERT_EQUAL(96, b.slots[findSlot(b, SLOT_HUB, 6)].allowance);
  TEST_ASSERT_EQUAL(100, b.frame_10ms);
}

static void test_plan_relay_for_remote_hub(void) {
  Planner p = makePlanner();
  p.onJoin(5, 0, -90, 1);              // 5 direct
  p.onJoin(7, 5, 0, 1);                // 7 join heard by 5
  Beacon b;
  p.buildBeacon(2, b);
  const int echo = findSlot(b, SLOT_ECHO, 5);
  const int join = findSlot(b, SLOT_JOIN, 0);
  const int remote = findSlot(b, SLOT_HUB, 7);
  const int relay = findSlot(b, SLOT_HUB, 5);
  TEST_ASSERT_TRUE(echo == 0 && join == 1 && remote == 2 && relay == 3);   // echo, join, remote, relay
  TEST_ASSERT_EQUAL(5, b.slots[remote].via);
  TEST_ASSERT_EQUAL(MODE_SF9_BW500, b.slots[remote].mode);
  TEST_ASSERT_EQUAL(96 + 96 - 2, b.slots[relay].allowance);              // carries the remote packet
  TEST_ASSERT_EQUAL(beaconSize(b.n_slots, b.n_acks), b.slots[echo].allowance);
  // hub 7 heard directly 3 times -> becomes direct (hysteresis)
  p.onDirectPacket(7, 3, -100, 0, MODE_SF9_BW500, true);
  p.buildBeacon(3, b);
  TEST_ASSERT_EQUAL(5, b.slots[findSlot(b, SLOT_HUB, 7)].via);
  p.onDirectPacket(7, 4, -100, 0, MODE_SF9_BW500, true);
  p.onDirectPacket(7, 5, -100, 0, MODE_SF9_BW500, true);
  p.onDirectPacket(5, 5, -80, 0, MODE_SF9_BW500, true);
  p.buildBeacon(6, b);
  TEST_ASSERT_EQUAL(0, b.slots[findSlot(b, SLOT_HUB, 7)].via);
  TEST_ASSERT_EQUAL(-1, findSlot(b, SLOT_ECHO, 5));
}

static void test_direct_hub_falls_back_to_relay(void) {
  Planner p = makePlanner();
  p.onJoin(5, 0, -90, 1);
  p.onJoin(6, 0, -90, 1);
  Health h; memset(&h, 0, sizeof(h)); h.n_nb = 1; h.nb[0] = {6, -95, 0};
  Beacon b;
  for (uint16_t f = 2; f < 12; f++) {        // 6 silent towards the chalet from frame 2; 5 hears 6
    p.onDirectPacket(5, f, -80, 0, MODE_SF9_BW500, true);
    p.onHealth(5, h, f);
    p.onRelayed(6, 5, f);
    p.buildBeacon(f, b);
  }
  TEST_ASSERT_EQUAL(5, b.slots[findSlot(b, SLOT_HUB, 6)].via);
  TEST_ASSERT_TRUE(findSlot(b, SLOT_ECHO, 5) >= 0);
}

static void test_hub_expires(void) {
  Planner p = makePlanner();
  p.onJoin(5, 0, -90, 1);
  Beacon b;
  p.buildBeacon(2, b);
  TEST_ASSERT_TRUE(findSlot(b, SLOT_HUB, 5) >= 0);
  p.buildBeacon(2 + Planner::HUB_TIMEOUT_FRAMES + 1, b);
  TEST_ASSERT_EQUAL(-1, findSlot(b, SLOT_HUB, 5));
}

static void test_acks_repeat_then_stop(void) {
  Planner p = makePlanner();
  p.addAck(5, 21, 3);
  p.addAck(5, 21, 4);                        // same node: newest seq replaces
  Beacon b;
  for (int i = 0; i < Planner::ACK_REPEAT; i++) {
    p.buildBeacon(static_cast<uint16_t>(10 + i), b);
    TEST_ASSERT_EQUAL(1, b.n_acks);
    TEST_ASSERT_EQUAL(4, b.acks[0].seq);
  }
  p.buildBeacon(20, b);
  TEST_ASSERT_EQUAL(0, b.n_acks);
}

static void test_plan_shrinks_to_fit(void) {
  Planner p = makePlanner();
  p.cfg.test_mode = TEST_FIX_SF9;
  p.cfg.test_allowance = 255;
  for (uint8_t id = 1; id <= 6; id++) p.onJoin(id, 0, -90, 1);
  Beacon b;
  p.buildBeacon(2, b);
  TEST_ASSERT_TRUE(planFits(b.slots, b.n_slots, 1000000, static_cast<uint16_t>(beaconSize(b.n_slots, b.n_acks))));
  TEST_ASSERT_TRUE(p.lastAllowance() < 255);
  TEST_ASSERT_EQUAL(7, b.n_slots);           // nobody dropped, allowance reduced instead
}

static void test_test_rotate_modes_and_stats(void) {
  Planner p = makePlanner();
  p.cfg.test_mode = TEST_ROTATE;
  p.onJoin(5, 0, -90, 1);
  Beacon b;
  uint8_t seen[MODE_COUNT] = {0};
  for (uint16_t f = 2; f < 8; f++) {
    p.buildBeacon(f, b);
    const Slot& s = b.slots[findSlot(b, SLOT_HUB, 5)];
    seen[s.mode]++;
    if (s.mode == MODE_SF7_BW500) p.onSlotMissed(5, f, s.mode, true);
    else p.onDirectPacket(5, f, static_cast<int8_t>(-90 - s.mode), 4, s.mode, true);
  }
  for (uint8_t m = 0; m < MODE_COUNT; m++) TEST_ASSERT_EQUAL(2, seen[m]);
  const HubInfo* h = p.find(5);
  TEST_ASSERT_EQUAL(2, h->stats[MODE_SF9_BW500].received);
  TEST_ASSERT_EQUAL(2, h->stats[MODE_SF7_BW500].scheduled);
  TEST_ASSERT_EQUAL(0, h->stats[MODE_SF7_BW500].received);
  TEST_ASSERT_EQUAL(2, h->stats[MODE_SF7_BW500].crc_err);
  TEST_ASSERT_EQUAL(-91, h->stats[MODE_SF8_BW500].rssi_min);
}

static void test_adaptive_up_and_down(void) {
  Planner p = makePlanner();
  p.cfg.adaptive = true;
  p.onJoin(5, 0, -60, 1);
  Beacon b;
  uint16_t f = 2;
  for (; f < 12; f++) { p.buildBeacon(f, b); p.onDirectPacket(5, f, -60, 40, MODE_SF9_BW500, true); }
  // strong link (-60 dBm, margin to SF8 sens -121 = 61 dB >= 12) -> SF8 after 8 good packets
  p.buildBeacon(f, b);
  TEST_ASSERT_EQUAL(MODE_SF8_BW500, b.slots[findSlot(b, SLOT_HUB, 5)].mode);
  // two misses in SF8 -> back to SF9
  p.onSlotMissed(5, f, MODE_SF8_BW500, false); f++;
  p.onSlotMissed(5, f, MODE_SF8_BW500, false); f++;
  p.buildBeacon(f, b);
  TEST_ASSERT_EQUAL(MODE_SF9_BW500, b.slots[findSlot(b, SLOT_HUB, 5)].mode);
  // weak link (-115 dBm) never steps up
  Planner q = makePlanner(); q.cfg.adaptive = true; q.onJoin(6, 0, -115, 1);
  for (uint16_t g = 2; g < 60; g++) { q.buildBeacon(g, b); q.onDirectPacket(6, g, -115, -20, MODE_SF9_BW500, true); }
  q.buildBeacon(60, b);
  TEST_ASSERT_EQUAL(MODE_SF9_BW500, b.slots[findSlot(b, SLOT_HUB, 6)].mode);
}

// ---------------- hub packet / chalet rx ----------------
static void test_hub_packet_priorities_and_truncation(void) {
  LineTable<24> t;
  for (uint8_t n = 1; n <= 20; n++) t.observe(n, LS_IDLE, 0, 0, 80, 0);
  t.observe(15, LS_TRIPPED, 0, RF_LOWBAT, 80, 1);
  Health h; memset(&h, 0, sizeof(h)); h.battery_mv = 4000;
  HubBuild in; memset(&in, 0, sizeof(in));
  in.network_id = 0x42; in.self = 5; in.frame = 9; in.allowance = 40; in.health = &h; in.test = true;
  uint8_t buf[MAX_PACKET];
  HubBuildResult r = buildHubPacket(in, t, buf, sizeof buf);
  TEST_ASSERT_TRUE(r.len <= 40);
  TEST_ASSERT_EQUAL(20, r.line_total);
  TEST_ASSERT_EQUAL((40 - 7 - 2) / 3, r.line_records);    // 10 records fit
  TEST_ASSERT_EQUAL(HF_PENDING, buf[HDR_LEN] & HF_PENDING);
  LineRecord first; decodeLine(buf + HDR_LEN + 1 + 2, first);
  TEST_ASSERT_EQUAL(15, first.node);                       // pending event first

  in.allowance = 200;
  r = buildHubPacket(in, t, buf, sizeof buf);
  TEST_ASSERT_EQUAL(200, r.len);                            // test filler pads to the allowance
  SectionReader rd(buf + HDR_LEN + 1, r.len - HDR_LEN - 1);
  uint8_t ty, n; const uint8_t* v; uint8_t order[8]; uint8_t k = 0;
  while (rd.next(ty, v, n) && k < 8) order[k++] = ty;
  TEST_ASSERT_EQUAL(SEC_LINE, order[0]);
  TEST_ASSERT_EQUAL(SEC_HEALTH, order[1]);
  TEST_ASSERT_EQUAL(SEC_NODEINFO, order[2]);
  TEST_ASSERT_EQUAL(SEC_TEST, order[k - 1]);
}

static void test_relay_repack_and_chalet_consume(void) {
  // remote hub 7: node 31 tripped
  LineTable<24> t7; t7.observe(31, LS_IDLE, 0, 0, 70, 0); t7.observe(31, LS_TRIPPED, 3, 0, 70, 1);
  HubBuild in7; memset(&in7, 0, sizeof(in7)); in7.network_id = 0x42; in7.self = 7; in7.frame = 50; in7.allowance = 96;
  RelayItem item;
  item.len = static_cast<uint8_t>(buildHubPacket(in7, t7, item.data, sizeof item.data).len);
  // relay hub 5 forwards it
  LineTable<24> t5; t5.observe(40, LS_IDLE, 0, 0, 90, 0);
  HubBuild in5; memset(&in5, 0, sizeof(in5)); in5.network_id = 0x42; in5.self = 5; in5.frame = 50; in5.allowance = 190;
  in5.relayed = &item; in5.n_relayed = 1;
  const uint8_t joins[1] = {9}; in5.joins = joins; in5.n_joins = 1;
  uint8_t buf[MAX_PACKET];
  HubBuildResult r = buildHubPacket(in5, t5, buf, sizeof buf);
  TEST_ASSERT_EQUAL(1, r.relayed_full);
  Planner p = makePlanner();
  ChaletNodes<16> nodes;
  ChaletRxResult rx = consumeHubPacket(buf, r.len, 0x42, 50, p, nodes);
  TEST_ASSERT_TRUE(rx.ok);
  TEST_ASSERT_EQUAL(5, rx.hub);
  TEST_ASSERT_EQUAL(2, rx.n_records);
  TEST_ASSERT_EQUAL(1, rx.n_relayed);
  TEST_ASSERT_EQUAL(LS_TRIPPED, nodes.find(31)->state);
  TEST_ASSERT_EQUAL(7, nodes.find(31)->owner);
  TEST_ASSERT_EQUAL(3, nodes.find(31)->turns);
  TEST_ASSERT_NOT_NULL(p.find(7));        // relayed hub known
  TEST_ASSERT_NOT_NULL(p.find(9));        // join reported by 5
  Beacon b; p.buildBeacon(51, b);
  bool acked = false;
  for (uint8_t i = 0; i < b.n_acks; i++) if (b.acks[i].hub == 7 && b.acks[i].node == 31 && b.acks[i].seq == 1) acked = true;
  TEST_ASSERT_TRUE(acked);
  // remote side applies the ack (it receives the beacon through the echo)
  TEST_ASSERT_EQUAL(1, applyAcks(b, 7, t7));
  TEST_ASSERT_FALSE(t7.anyPending());
}

static void test_relay_partial_when_too_big(void) {
  LineTable<24> t7;
  for (uint8_t n = 1; n <= 10; n++) t7.observe(n, LS_IDLE, 0, 0, 70, 0);
  HubBuild in7; memset(&in7, 0, sizeof(in7)); in7.network_id = 0x42; in7.self = 7; in7.frame = 1; in7.allowance = 120; in7.test = true;
  RelayItem item;
  item.len = static_cast<uint8_t>(buildHubPacket(in7, t7, item.data, sizeof item.data).len);
  uint8_t out[MAX_PACKET]; bool partial = false;
  const uint8_t n = packRelayValue(item.data, item.len, out, 40, partial);
  TEST_ASSERT_TRUE(partial);
  TEST_ASSERT_TRUE(n <= 40);
  TEST_ASSERT_EQUAL(SEC_LINE, out[3]);
  TEST_ASSERT_EQUAL(0, out[4] % 3);         // whole records only
}

static void test_line_rotation_covers_all_nodes(void) {
  LineTable<24> t;
  for (uint8_t n = 1; n <= 20; n++) t.observe(n, LS_IDLE, 0, 0, 80, 0);
  t.observe(7, LS_TRIPPED, 0, 0, 80, 1);                    // one pending
  HubBuild in; memset(&in, 0, sizeof(in));
  in.network_id = 0x42; in.self = 5; in.allowance = 7 + 2 + 5 * 3;   // 5 records per packet
  bool seen[21] = {false};
  uint8_t buf[MAX_PACKET];
  for (int f = 0; f < 6; f++) {
    HubBuildResult r = buildHubPacket(in, t, buf, sizeof buf);
    TEST_ASSERT_EQUAL(5, r.line_records);
    LineRecord first; decodeLine(buf + HDR_LEN + 1 + 2, first);
    TEST_ASSERT_EQUAL(7, first.node);                         // pending always first
    for (uint8_t k = 0; k < r.line_records; k++) { LineRecord x; decodeLine(buf + HDR_LEN + 3 + 3 * k, x); seen[x.node] = true; }
    in.line_rot = r.line_rot_next;
  }
  for (uint8_t n = 1; n <= 20; n++) TEST_ASSERT_TRUE(seen[n]);   // 19 others / 4 per packet -> 5 packets
}

static void test_relayed_silence_request(void) {
  LineTable<24> t7; t7.observe(31, LS_IDLE, 0, 0, 70, 0);
  HubBuild in7; memset(&in7, 0, sizeof(in7)); in7.network_id = 0x42; in7.self = 7; in7.frame = 3; in7.allowance = 60;
  in7.flags = HF_SILENCE_ON;
  RelayItem item; item.len = static_cast<uint8_t>(buildHubPacket(in7, t7, item.data, sizeof item.data).len);
  LineTable<24> t5;
  HubBuild in5; memset(&in5, 0, sizeof(in5)); in5.network_id = 0x42; in5.self = 5; in5.frame = 3; in5.allowance = 120;
  in5.relayed = &item; in5.n_relayed = 1;
  uint8_t buf[MAX_PACKET];
  const size_t len = buildHubPacket(in5, t5, buf, sizeof buf).len;
  Planner p = makePlanner(); ChaletNodes<16> nodes;
  ChaletRxResult rx = consumeHubPacket(buf, len, 0x42, 3, p, nodes);
  TEST_ASSERT_TRUE((rx.flags & HF_SILENCE_ON) != 0);
}

static void test_relay_repack_odd_length_safe(void) {
  // malformed remote LINE section of 7 bytes (not a multiple of 3) must not wrap
  uint8_t pkt[32]; PacketWriter w(pkt, sizeof pkt); w.beginHub(0x42, 7, 1, 0);
  uint8_t v[7] = {1, 2, 3, 4, 5, 6, 7}; w.add(SEC_LINE, v, 7);
  uint8_t pad[10] = {0}; w.add(SEC_TEST, pad, 10);
  uint8_t out[MAX_PACKET]; bool partial = false;
  const uint8_t n = packRelayValue(pkt, static_cast<uint8_t>(w.size()), out, 12, partial);
  TEST_ASSERT_TRUE(n <= 12);
  if (n) TEST_ASSERT_EQUAL(0, out[4] % 3);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_line_event_and_ack);
  RUN_TEST(test_ack_of_older_event_does_not_clear_newer);
  RUN_TEST(test_seq_wraps_4_bits);
  RUN_TEST(test_pending_first_and_expiry);
  RUN_TEST(test_table_full_never_drops_pending);
  RUN_TEST(test_owner_and_duplicates);
  RUN_TEST(test_owner_moves_when_owner_loses_node);
  RUN_TEST(test_node_offline_after_timeout);
  RUN_TEST(test_hub_sync);
  RUN_TEST(test_plan_direct_hubs_and_join);
  RUN_TEST(test_plan_relay_for_remote_hub);
  RUN_TEST(test_direct_hub_falls_back_to_relay);
  RUN_TEST(test_hub_expires);
  RUN_TEST(test_acks_repeat_then_stop);
  RUN_TEST(test_plan_shrinks_to_fit);
  RUN_TEST(test_test_rotate_modes_and_stats);
  RUN_TEST(test_adaptive_up_and_down);
  RUN_TEST(test_hub_packet_priorities_and_truncation);
  RUN_TEST(test_relay_repack_and_chalet_consume);
  RUN_TEST(test_relay_partial_when_too_big);
  RUN_TEST(test_line_rotation_covers_all_nodes);
  RUN_TEST(test_relayed_silence_request);
  RUN_TEST(test_relay_repack_odd_length_safe);
  return UNITY_END();
}
