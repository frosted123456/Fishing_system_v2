// Demo network (chalet only, docs/SONAR_SIM.md): fake hubs run as real HubRole objects inside the
// chalet; their packets enter through onEbHubPacket and the chalet's backbone beacon is fed back.
// This checks the chain the firmware glue (mesh_radio.cpp meshDemo*) relies on: holes appear with
// their pocket and the SIM flag, acks clear, BASE summaries reach the sonar store, CMD_SET_SIM reaches
// the fake hubs, ChaletNodes::forget removes holes, demo IDs do not collide.
#include <unity.h>
#include <hub_role.h>
#include <chalet_role.h>
#include <sonar_link.h>
#include <cstdio>

using namespace icemesh;
using namespace icemesh::tdma;

void setUp(void) {}
void tearDown(void) {}

static const uint8_t HUB0 = 121;   // same rule as mesh_radio.cpp
static uint8_t vid(uint8_t hub, uint8_t k) { return static_cast<uint8_t>(128 + (hub & 0x0F) * 8 + (k & 7)); }   // real hubs' test holes
static uint8_t holeId(uint8_t hub, uint8_t k) { return k == 0 ? hub : static_cast<uint8_t>(101 + (hub - HUB0) * 3 + (k - 1)); }

static void pushBase(HubRole<8>& r, uint8_t node, uint16_t bottom, uint16_t bait, uint16_t fish) {
  sonar::Summary s; memset(&s, 0, sizeof(s));
  s.node = node; s.ping = 8; s.bottom_cm = bottom; s.hard = sonar::BH_MEDIUM;
  s.list[0].track = 0; s.list[0].depth_cm = bait; s.list[0].level = 2;
  s.list[1].track = 1; s.list[1].depth_cm = fish; s.list[1].level = 3;
  s.n_list = 2; s.n_targets = 1; s.activity = 4;
  sonar::summaryNearest(s);
  uint8_t blk[sonar::MAX_BLOCK];
  const size_t n = sonar::encodeSummary(s, blk, sizeof blk);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_TRUE(r.sonar.push(blk, static_cast<uint8_t>(n), r.frameNow()));
}

static void test_ids_distinct(void) {
  uint8_t seen[256]; memset(seen, 0, sizeof seen);
  for (uint8_t h = 0; h < 4; h++)
    for (uint8_t k = 0; k < 4; k++) {
      const uint8_t id = holeId(static_cast<uint8_t>(HUB0 + h), k);
      TEST_ASSERT_TRUE(id != 0 && id != 255);
      TEST_ASSERT_EQUAL(0, seen[id]);
      seen[id] = 1;
    }
  // no real hub's test hole (any hub ID, any k) and not the chalet (100) can be a demo ID
  for (int hub = 1; hub < 255; hub++) for (uint8_t k = 0; k < 8; k++) TEST_ASSERT_EQUAL(0, seen[vid(static_cast<uint8_t>(hub), k)]);
  TEST_ASSERT_EQUAL(0, seen[100]);
}

static void test_demo_chain(void) {
  ChaletRole<10, 48> ch; ch.planner.cfg.network_id = 0x42; ch.planner.cfg.self_id = 100;
  sonar::SonarStore<16, 128> store; ch.sonar_sink = &store;
  HubRole<8> hub[3];
  for (uint8_t h = 0; h < 3; h++) { hub[h].network_id = 0x42; hub[h].self = static_cast<uint8_t>(HUB0 + h); }
  uint8_t b[MAX_PACKET], pl[MAX_PACKET];
  int changes = 0;
  for (uint16_t f = 1; f <= 6; f++) {
    ch.startFrame(f, b, sizeof b);
    const size_t bl = ch.buildEbBeacon(b, sizeof b);
    for (uint8_t h = 0; h < 3; h++) {
      uint8_t cmd = CMD_NONE;
      TEST_ASSERT_TRUE(hub[h].onEbBeacon(b, bl, cmd));
      for (uint8_t k = 0; k < 3; k++) {
        const uint8_t id = holeId(hub[h].self, k);
        hub[h].table.observe(id, LS_IDLE, 0, LF_SIM, 80, f * 1000u);
        if (f == 2) pushBase(hub[h], id, static_cast<uint16_t>(500 + 10 * k), 420, 300);
      }
      const size_t n = hub[h].buildHubFree(pl, sizeof pl, 0, 3900, 1, hub[h].frameNow());
      TEST_ASSERT_TRUE(n > HDR_LEN);
      ChaletRxResult r;
      TEST_ASSERT_TRUE(ch.onEbHubPacket(pl, n, 0, r));
      changes += r.n_changes;
    }
  }
  TEST_ASSERT_EQUAL(9, changes);                                  // 9 holes, each appears once
  for (uint8_t h = 0; h < 3; h++) {
    TEST_ASSERT_FALSE(hub[h].table.anyPending());                 // acks came back through the backbone beacon
    for (uint8_t k = 0; k < 3; k++) {
      const uint8_t id = holeId(hub[h].self, k);
      const NodeView* v = ch.nodes.find(id);
      TEST_ASSERT_NOT_NULL(v);
      TEST_ASSERT_EQUAL(hub[h].self, v->owner);                   // grouped by pocket on the pages
      TEST_ASSERT_TRUE((v->flags & LF_SIM) != 0);                  // marked SIM everywhere
      const sonar::NodeSonar* ns = store.find(id);
      TEST_ASSERT_NOT_NULL(ns);
      TEST_ASSERT_TRUE(ns->has_sum);
      TEST_ASSERT_EQUAL(500 + 10 * k, ns->sum.bottom_cm);
      TEST_ASSERT_EQUAL(2, ns->sum.n_list);
    }
  }
  // the chalet's simulation command reaches the fake hubs like real ones
  ch.cmd = CMD_SET_SIM; ch.cmd_seq = 7; ch.cmd_target = holeId(HUB0 + 1, 2); ch.cmd_value = simValue(true, true, 6);
  ch.startFrame(7, b, sizeof b);
  uint8_t cmd = CMD_NONE;
  TEST_ASSERT_TRUE(hub[1].onEbBeacon(b, ch.buildEbBeacon(b, sizeof b), cmd));
  TEST_ASSERT_EQUAL(CMD_SET_SIM, cmd);
  TEST_ASSERT_EQUAL(holeId(HUB0 + 1, 2), hub[1].last_cmd_target);
  // demo off: holes are dropped from the chalet table
  for (uint8_t h = 0; h < 3; h++) for (uint8_t k = 0; k < 3; k++) TEST_ASSERT_TRUE(ch.nodes.forget(holeId(hub[h].self, k)));
  TEST_ASSERT_NULL(ch.nodes.find(HUB0));
  TEST_ASSERT_FALSE(ch.nodes.forget(HUB0));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_ids_distinct);
  RUN_TEST(test_demo_chain);
  return UNITY_END();
}
