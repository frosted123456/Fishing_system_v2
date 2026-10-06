// ESP-NOW backbone: frames, relay rules, dedup, transport policy, hub <-> chalet over the backbone,
// planner frame length (1 s -> 2 s with 10 hubs and back), beacon commands.
#include <unity.h>
#include <eb_link.h>
#include <hub_role.h>
#include <chalet_role.h>
#include <cstdio>

using namespace icemesh;
using namespace icemesh::tdma;

void setUp(void) {}
void tearDown(void) {}

static void test_frame_roundtrip_and_relay(void) {
  uint8_t pl[40]; for (int i = 0; i < 40; i++) pl[i] = static_cast<uint8_t>(i);
  eb::Header h; h.net = 0x42; h.sender = 5; h.type = eb::MSG_EB_HUB; h.hops = 0; h.origin = 5; h.seq = 0x1234;
  uint8_t f[eb::MAX_FRAME];
  const size_t n = eb::encode(h, pl, sizeof pl, f, sizeof f);
  TEST_ASSERT_EQUAL(eb::HDR + 40, n);
  eb::Header d; const uint8_t* p; size_t pn;
  TEST_ASSERT_TRUE(eb::decode(f, n, 0x42, d, p, pn));
  TEST_ASSERT_EQUAL(0x1234, d.seq); TEST_ASSERT_EQUAL(40, pn); TEST_ASSERT_EQUAL(39, p[39]);
  TEST_ASSERT_FALSE(eb::decode(f, n, 0x43, d, p, pn));       // other network
  TEST_ASSERT_TRUE(eb::prepareRelay(f, n, 9));                // hop 1
  TEST_ASSERT_TRUE(eb::prepareRelay(f, n, 10));               // hop 2
  TEST_ASSERT_FALSE(eb::prepareRelay(f, n, 11));              // max 2 relays
  TEST_ASSERT_TRUE(eb::decode(f, n, 0x42, d, p, pn));
  TEST_ASSERT_EQUAL(2, d.hops); TEST_ASSERT_EQUAL(10, d.sender); TEST_ASSERT_EQUAL(5, d.origin);
  uint8_t own[eb::MAX_FRAME]; memcpy(own, f, n); own[3] = 0;
  TEST_ASSERT_FALSE(eb::prepareRelay(own, n, 5));             // never relay your own frame
  TEST_ASSERT_EQUAL(0, eb::encode(h, pl, eb::MAX_PAYLOAD + 1, f, sizeof f));
}

static void test_dedup(void) {
  eb::Dedup dd;
  TEST_ASSERT_FALSE(dd.seen(5, eb::MSG_EB_HUB, 7));
  TEST_ASSERT_TRUE(dd.seen(5, eb::MSG_EB_HUB, 7));
  TEST_ASSERT_FALSE(dd.seen(5, eb::MSG_EB_BEACON, 7));
  TEST_ASSERT_FALSE(dd.seen(6, eb::MSG_EB_HUB, 7));
  for (uint16_t k = 0; k < eb::Dedup::CAP; k++) dd.seen(9, eb::MSG_EB_HUB, static_cast<uint16_t>(100 + k));
  TEST_ASSERT_FALSE(dd.seen(5, eb::MSG_EB_HUB, 7));           // pushed out after CAP newer frames
}

static void test_policy(void) {
  eb::TransportPolicy tp; uint32_t t = 1000; tp.begin(t);
  // setup phase: no beacon yet (chalet not switched on): searching, status also on the backbone
  t += 600000; tp.tick(t);
  TEST_ASSERT_FALSE(tp.armed()); TEST_ASSERT_TRUE(tp.searching()); TEST_ASSERT_TRUE(tp.useEb(TR_AUTO, t));
  TEST_ASSERT_FALSE(tp.useEb(TR_LORA, t));
  t += 1000; tp.onLoraBeacon(t); tp.tick(t);
  TEST_ASSERT_FALSE(tp.searching()); TEST_ASSERT_FALSE(tp.useEb(TR_AUTO, t));   // first beacon: LoRa only
  // beacons for 4 min, a 20 s break (chalet moved), beacons again: the 5 min run restarts
  for (int k = 0; k < 240; k++) { t += 1000; tp.onLoraBeacon(t); tp.tick(t); }
  t += 20000; tp.tick(t);
  TEST_ASSERT_FALSE(tp.useEb(TR_AUTO, t));
  for (int k = 0; k < 300; k++) { t += 1000; tp.onLoraBeacon(t); tp.tick(t); }   // 300 beacons = 299 s
  TEST_ASSERT_FALSE(tp.armed()); TEST_ASSERT_TRUE(tp.armInS(t) > 0);
  t += 1000; tp.onLoraBeacon(t); tp.tick(t);
  TEST_ASSERT_TRUE(tp.armed()); TEST_ASSERT_EQUAL(0, tp.armInS(t));
  TEST_ASSERT_TRUE(tp.useLora(TR_AUTO, t)); TEST_ASSERT_FALSE(tp.useEb(TR_AUTO, t));
  t += 11000; tp.tick(t);                                      // armed, LoRa silent 11 s
  TEST_ASSERT_TRUE(tp.useEb(TR_AUTO, t)); TEST_ASSERT_TRUE(tp.useLora(TR_AUTO, t));
  for (int k = 0; k < 4; k++) { t += 1000; tp.onLoraBeacon(t); tp.tick(t); }
  TEST_ASSERT_TRUE(tp.useEb(TR_AUTO, t));                      // 4 beacons back: not yet
  t += 1000; tp.onLoraBeacon(t); tp.tick(t);
  TEST_ASSERT_FALSE(tp.useEb(TR_AUTO, t));                     // 5 in a row: LoRa only again
  // forced modes and their safety nets
  TEST_ASSERT_FALSE(tp.useEb(TR_LORA, t));
  TEST_ASSERT_TRUE(tp.useEb(TR_LORA, t + 61000));
  tp.onEbBeacon(t);
  TEST_ASSERT_FALSE(tp.useLora(TR_ESPNOW, t + 1000)); TEST_ASSERT_TRUE(tp.useEb(TR_ESPNOW, t + 1000));
  TEST_ASSERT_TRUE(tp.useLora(TR_ESPNOW, t + 61000));
}

static void test_hub_chalet_over_backbone(void) {
  ChaletRole<10, 32> ch; ch.planner.cfg.network_id = 0x42; ch.planner.cfg.self_id = 100;
  ch.net_cfg = makeNetCfg(TR_ESPNOW, true, 3);
  HubRole<24> hub; hub.network_id = 0x42; hub.self = 4;
  hub.table.observe(41, LS_TRIPPED, 2, 0, 80, 0);
  uint8_t b[MAX_PACKET], hp[MAX_PACKET], f[eb::MAX_FRAME];
  // hub -> (relay 7) -> chalet
  const size_t hl = hub.buildHubFree(hp, eb::MAX_PAYLOAD, HF_EB_RELAY | HF_EB_PATH, 4000, 1, 1);
  TEST_ASSERT_TRUE(hl > HDR_LEN + 3);
  eb::Header h; h.net = 0x42; h.sender = 4; h.type = eb::MSG_EB_HUB; h.hops = 0; h.origin = 4; h.seq = 1;
  size_t fl = eb::encode(h, hp, hl, f, sizeof f);
  TEST_ASSERT_TRUE(eb::prepareRelay(f, fl, 7));
  eb::Header d; const uint8_t* p; size_t pn;
  TEST_ASSERT_TRUE(eb::decode(f, fl, 0x42, d, p, pn));
  ch.startFrame(1, b, sizeof b);
  ChaletRxResult r;
  TEST_ASSERT_TRUE(ch.onEbHubPacket(p, pn, d.hops, r));
  TEST_ASSERT_EQUAL(1, r.n_changes); TEST_ASSERT_EQUAL(41, r.changes[0].node); TEST_ASSERT_EQUAL(LS_TRIPPED, r.changes[0].new_state);
  TEST_ASSERT_EQUAL(1, ch.eb_hubs[0].hops);
  TEST_ASSERT_EQUAL(4, r.hub);                                 // relay / backbone flags reach the chalet (Radio view)
  TEST_ASSERT_EQUAL(HF_EB_RELAY | HF_EB_PATH, r.flags & (HF_EB_RELAY | HF_EB_PATH));
  TEST_ASSERT_TRUE(hub.table.anyPending());
  // chalet beacon over the backbone carries the ack and the network config
  ch.startFrame(2, b, sizeof b);
  const size_t bl = ch.buildEbBeacon(b, sizeof b);
  TEST_ASSERT_EQUAL(beaconSize(0, ch.beacon.n_acks), bl);
  uint8_t cmd;
  TEST_ASSERT_TRUE(hub.onEbBeacon(b, bl, cmd));
  TEST_ASSERT_FALSE(hub.table.anyPending());
  TEST_ASSERT_EQUAL(TR_ESPNOW, hub.transport()); TEST_ASSERT_TRUE(hub.espNowLr());
  TEST_ASSERT_FALSE(hub.have_plan);                           // the backbone gives no slot plan
}

static void test_channel_and_relay_commands(void) {
  ChaletRole<10, 32> ch; ch.planner.cfg.network_id = 0x42; ch.planner.cfg.self_id = 100;
  HubRole<24> hub; hub.network_id = 0x42; hub.self = 4;
  uint8_t b[MAX_PACKET], cmd = 0;
  ch.cmd = CMD_SET_CHANNEL; ch.cmd_seq = 9; ch.cmd_target = 3; ch.cmd_value = 5;
  ch.startFrame(20, b, sizeof b);
  const size_t bl = ch.buildEbBeacon(b, sizeof b);
  TEST_ASSERT_TRUE(hub.onEbBeacon(b, bl, cmd));
  TEST_ASSERT_EQUAL(CMD_SET_CHANNEL, cmd);
  TEST_ASSERT_TRUE(hub.channel_switch_pending); TEST_ASSERT_EQUAL(5, hub.channel_next); TEST_ASSERT_EQUAL(24, hub.channel_switch_frame);
  ch.cmd_target = 2; ch.startFrame(21, b, sizeof b);          // same command repeated: executed once
  TEST_ASSERT_TRUE(hub.onEbBeacon(b, ch.buildEbBeacon(b, sizeof b), cmd));
  TEST_ASSERT_EQUAL(CMD_NONE, cmd);
  ch.cmd = CMD_SET_RELAY; ch.cmd_seq = 10; ch.cmd_target = 4; ch.cmd_value = 1; ch.startFrame(22, b, sizeof b);
  TEST_ASSERT_TRUE(hub.onEbBeacon(b, ch.buildEbBeacon(b, sizeof b), cmd));
  TEST_ASSERT_EQUAL(CMD_SET_RELAY, cmd); TEST_ASSERT_EQUAL(4, hub.last_cmd_target); TEST_ASSERT_EQUAL(1, hub.last_cmd_value);
  // simulation of one hole (fake sonar + fake trips, 12 per hour)
  const uint8_t v = simValue(true, true, 12);
  TEST_ASSERT_EQUAL(SIM_SONAR | SIM_HALL | (12 << 2), v); TEST_ASSERT_EQUAL(12, simTripsPerHour(v));
  TEST_ASSERT_EQUAL(6, simTripsPerHour(simValue(false, true, 0)));            // 0 = default rate
  TEST_ASSERT_EQUAL(63, simTripsPerHour(simValue(false, true, 200)));         // clamped
  ch.cmd = CMD_SET_SIM; ch.cmd_seq = 11; ch.cmd_target = 41; ch.cmd_value = v; ch.startFrame(23, b, sizeof b);
  TEST_ASSERT_TRUE(hub.onEbBeacon(b, ch.buildEbBeacon(b, sizeof b), cmd));
  TEST_ASSERT_EQUAL(CMD_SET_SIM, cmd); TEST_ASSERT_EQUAL(41, hub.last_cmd_target); TEST_ASSERT_EQUAL(v, hub.last_cmd_value);
}

static void test_planner_frame_length(void) {
  ChaletPlanner<10> p; p.cfg.network_id = 0x42; p.cfg.self_id = 100;
  Beacon b;
  for (uint16_t f = 1; f <= 3; f++) for (uint8_t h = 1; h <= 3; h++) p.onDirectPacket(h, f, -90, 20, MODE_SF9_BW500, false);
  p.buildBeacon(4, b);
  TEST_ASSERT_EQUAL(100, b.frame_10ms);                        // 3 hubs: 1 s
  uint16_t f = 5;
  for (; f <= 8; f++) for (uint8_t h = 1; h <= 10; h++) p.onDirectPacket(h, f, -90, 20, MODE_SF9_BW500, false);
  p.buildBeacon(f, b);
  int hubs = 0; for (uint8_t i = 0; i < b.n_slots; i++) hubs += b.slots[i].kind == SLOT_HUB;
  printf("10 hubs: frame %u ms, %d hub slots, allowance %u B\n", b.frame_10ms * 10u, hubs, p.lastAllowance());
  TEST_ASSERT_TRUE(b.frame_10ms > 100);
  TEST_ASSERT_EQUAL(10, hubs);
  TEST_ASSERT_TRUE(p.lastAllowance() >= ChaletPlanner<10>::KEEP_ALLOWANCE);
  TEST_ASSERT_TRUE(planFits(b.slots, b.n_slots, b.frame_10ms * 10000UL, static_cast<uint16_t>(beaconSize(b.n_slots, b.n_acks))));
  // 7 hubs go away (timeout 30 frames): back to 1 s after DOWN_FRAMES beacons
  for (int k = 0; k < 80; k++, f++) { for (uint8_t h = 1; h <= 3; h++) p.onDirectPacket(h, f, -90, 20, MODE_SF9_BW500, false); p.buildBeacon(f, b); }
  TEST_ASSERT_EQUAL(100, b.frame_10ms);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_frame_roundtrip_and_relay);
  RUN_TEST(test_dedup);
  RUN_TEST(test_policy);
  RUN_TEST(test_hub_chalet_over_backbone);
  RUN_TEST(test_channel_and_relay_commands);
  RUN_TEST(test_planner_frame_length);
  return UNITY_END();
}
