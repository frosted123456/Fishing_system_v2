// PC simulation of the TDMA mesh: 1 chalet + 3 hubs, real packet encoding, scripted links,
// random loss, joins, relay through one hub, node events, owner change, link failure.
#include <unity.h>
#include <hub_role.h>
#include <chalet_role.h>
#include <cstdlib>
#include <cstdio>

using namespace icemesh;
using namespace icemesh::tdma;

void setUp(void) {}
void tearDown(void) {}

static const uint8_t CHALET = 100;
static const uint8_t A = 1, B = 2, C = 3;
static const int NH = 3;
static const uint8_t ids[NH] = {A, B, C};

struct World {
  ChaletRole<6, 16> chalet;
  HubRole<24> hub[NH];
  bool link[4][4];          // index 0 = chalet, 1..3 = hubs A..C
  uint32_t rng = 12345;
  uint8_t loss_pct = 0;
  int collisions = 0;
  int max_abs_sync_err_us = 0;
  uint32_t node_changes = 0;
  uint8_t chalet_state[256];

  static int idx(uint8_t id) { return id == CHALET ? 0 : id; }
  uint32_t rnd() { rng = rng * 1103515245u + 12345u; return rng >> 8; }
  bool hears(uint8_t a, uint8_t b) { return link[idx(a)][idx(b)] && (rnd() % 100) >= loss_pct; }

  World() {
    memset(link, 0, sizeof(link));
    memset(chalet_state, 0xFF, sizeof(chalet_state));
    chalet.planner.cfg.network_id = 0x42; chalet.planner.cfg.self_id = CHALET;
    chalet.planner.cfg.allowance = 96; chalet.planner.cfg.test_allowance = 160; chalet.planner.cfg.frame_ms = 1000;
    for (int h = 0; h < NH; h++) { hub[h].network_id = 0x42; hub[h].self = ids[h]; }
  }
  void setLink(uint8_t a, uint8_t b, bool on) { link[idx(a)][idx(b)] = on; link[idx(b)][idx(a)] = on; }

  void applyChanges(const ChaletRxResult& r) {
    for (uint8_t k = 0; k < r.n_changes; k++) { chalet_state[r.changes[k].node] = r.changes[k].new_state; node_changes++; }
  }

  void runFrame(uint16_t f) {
    const uint32_t ref = static_cast<uint32_t>(f) * 1000000u + 777u;
    uint8_t bbuf[MAX_PACKET];
    const size_t blen = chalet.startFrame(f, bbuf, sizeof bbuf);
    bool fresh[NH] = {false, false, false};
    uint8_t cmd;
    for (int h = 0; h < NH; h++)
      if (hears(CHALET, ids[h])) fresh[h] = hub[h].onBeacon(bbuf, blen, ref + beaconAirtimeUs(static_cast<uint16_t>(blen)), -90, 20, cmd);
    const Beacon& plan = chalet.beacon;
    // echo slots first (they are first in the plan)
    uint8_t i = 0;
    for (; i < plan.n_slots && plan.slots[i].kind == SLOT_ECHO; i++) {
      int tx = -1;
      for (int h = 0; h < NH; h++) if (fresh[h] && hub[h].action(i, false) == ACT_TX_ECHO) tx = h;
      if (tx < 0) continue;
      uint8_t ebuf[MAX_PACKET];
      const size_t elen = hub[tx].buildEcho(ebuf, sizeof ebuf);
      const uint32_t rx_end = ref + chalet.times[i].tx + modeAirtimeUs(MODE_SF9_BW500, static_cast<uint16_t>(elen));
      for (int h = 0; h < NH; h++) {
        if (h == tx || fresh[h] || !hears(ids[tx], ids[h])) continue;
        fresh[h] = hub[h].onBeacon(ebuf, elen, rx_end, -100, 0, cmd);
        if (fresh[h]) {
          const int e = static_cast<int>(hub[h].sync.ref() - ref);
          if (std::abs(e) > max_abs_sync_err_us) max_abs_sync_err_us = std::abs(e);
        }
      }
    }
    for (int h = 0; h < NH; h++) if (!fresh[h]) hub[h].onBeaconMissed();
    // hubs without a plan send a random JOIN between slots (ALOHA, ~1 in 3 frames)
    for (int h = 0; h < NH; h++) {
      if (!hub[h].wantsAsyncJoin() || (rnd() % 3) != 0) continue;
      uint8_t jb[MAX_PACKET];
      const size_t jl = hub[h].buildJoin(jb, sizeof jb);
      if (hears(ids[h], CHALET)) chalet.onAsyncPacket(jb, jl, -110);
      for (int o = 0; o < NH; o++) if (o != h && hears(ids[h], ids[o])) hub[o].onAsyncPacket(jb, jl, -100, 0);
    }
    // remaining slots
    for (; i < plan.n_slots; i++) {
      int txs[NH]; int ntx = 0;
      uint8_t pk[NH][MAX_PACKET]; size_t pl[NH];
      for (int h = 0; h < NH; h++) {
        const SlotAction a = hub[h].action(i, (rnd() % 2) == 0);
        if (a == ACT_TX_HUB) { pl[h] = hub[h].buildHub(pk[h], MAX_PACKET, 0, 4000, 1, false, 0); txs[ntx++] = h; }
        else if (a == ACT_TX_JOIN) { pl[h] = hub[h].buildJoin(pk[h], MAX_PACKET); txs[ntx++] = h; }
      }
      if (ntx > 1) collisions++;
      // chalet
      if (ntx == 1 && hears(ids[txs[0]], CHALET)) {
        ChaletRxResult r;
        chalet.onSlotPacket(i, pk[txs[0]], pl[txs[0]], -95, 10, r);
        applyChanges(r);
      } else {
        chalet.onSlotEmpty(i, ntx > 1);
      }
      // other hubs listening
      if (ntx == 1) {
        for (int h = 0; h < NH; h++)
          if (h != txs[0] && hub[h].action(i, false) == ACT_RX && hears(ids[txs[0]], ids[h]))
            hub[h].onSlotPacket(i, pk[txs[0]], pl[txs[0]], -85, 20);
      }
    }
    NodeChange ex[8];
    const uint8_t ne = chalet.expireNodes(ex, 8);
    for (uint8_t k = 0; k < ne; k++) chalet_state[ex[k].node] = ex[k].new_state;
    for (int h = 0; h < NH; h++) hub[h].endFrame();
  }

  bool hubHasSlot(uint8_t id) {
    for (uint8_t i = 0; i < chalet.beacon.n_slots; i++)
      if (chalet.beacon.slots[i].kind == SLOT_HUB && chalet.beacon.slots[i].owner == id) return true;
    return false;
  }
  uint8_t viaOf(uint8_t id) {
    for (uint8_t i = 0; i < chalet.beacon.n_slots; i++)
      if (chalet.beacon.slots[i].kind == SLOT_HUB && chalet.beacon.slots[i].owner == id) return chalet.beacon.slots[i].via;
    return 0xFF;
  }
};

static World* w;

static void test_full_scenario(void) {
  static World world;
  w = &world;
  w->setLink(CHALET, A, true); w->setLink(CHALET, B, true);
  w->setLink(A, B, true); w->setLink(B, C, true);          // C only reaches B
  // nodes: A has 10, 11 and shares 40 with B; B has 20, 40; C has 31
  uint32_t t = 0;
  w->hub[0].table.observe(10, LS_IDLE, 0, 0, 90, t); w->hub[0].table.observe(11, LS_IDLE, 0, 0, 90, t);
  w->hub[0].table.observe(40, LS_IDLE, 0, 0, 90, t);
  w->hub[1].table.observe(20, LS_IDLE, 0, 0, 90, t); w->hub[1].table.observe(40, LS_IDLE, 0, 0, 90, t);
  w->hub[2].table.observe(31, LS_IDLE, 0, 0, 90, t);

  uint16_t f = 1;
  for (; f <= 15; f++) w->runFrame(f);
  TEST_ASSERT_TRUE(w->hubHasSlot(A));
  TEST_ASSERT_TRUE(w->hubHasSlot(B));
  TEST_ASSERT_TRUE(w->hubHasSlot(C));
  TEST_ASSERT_EQUAL(B, w->viaOf(C));                       // C is reached through B
  TEST_ASSERT_EQUAL(0, w->viaOf(A));
  TEST_ASSERT_EQUAL(LS_IDLE, w->chalet_state[31]);         // remote node seen through the relay
  TEST_ASSERT_LESS_OR_EQUAL(5, w->max_abs_sync_err_us);    // echo-derived timing matches the beacon

  // node 31 on remote hub C trips: chalet within 1 frame, C's event acked within 3
  w->hub[2].table.observe(31, LS_TRIPPED, 1, 0, 90, f);
  w->runFrame(f++);
  TEST_ASSERT_EQUAL(LS_TRIPPED, w->chalet_state[31]);
  TEST_ASSERT_TRUE(w->hub[2].table.anyPending());
  w->runFrame(f++); w->runFrame(f++);
  TEST_ASSERT_FALSE(w->hub[2].table.anyPending());

  // node 40 is reported by A and B; A loses it -> B takes over, ends IDLE (not OFFLINE)
  for (int k = 0; k < 3; k++) w->runFrame(f++);
  const uint8_t owner_before = w->chalet.nodes.find(40)->owner;
  w->hub[owner_before == A ? 0 : 1].table.observe(40, LS_OFFLINE, 0, 0, 0, f);
  for (int k = 0; k < 3; k++) w->runFrame(f++);
  TEST_ASSERT_EQUAL(LS_IDLE, w->chalet_state[40]);
  TEST_ASSERT_TRUE(w->chalet.nodes.find(40)->owner != owner_before);

  // chalet <-> A link fails; A must come back through B
  w->setLink(CHALET, A, false);
  for (int k = 0; k < 12; k++) w->runFrame(f++);
  TEST_ASSERT_EQUAL(B, w->viaOf(A));
  w->hub[0].table.observe(10, LS_TRIPPED, 0, 0, 90, f);
  w->runFrame(f++); w->runFrame(f++);
  TEST_ASSERT_EQUAL(LS_TRIPPED, w->chalet_state[10]);

  // 15 % loss everywhere for 200 frames: events still get through and get acked
  w->loss_pct = 15;
  for (int k = 0; k < 20; k++) w->runFrame(f++);
  w->hub[1].table.observe(20, LS_TRIPPED, 0, 0, 90, f);
  int frames_to_alert = 0;
  while (w->chalet_state[20] != LS_TRIPPED && frames_to_alert < 30) { w->runFrame(f++); frames_to_alert++; }
  TEST_ASSERT_TRUE(frames_to_alert < 30);
  for (int k = 0; k < 30; k++) w->runFrame(f++);
  TEST_ASSERT_FALSE(w->hub[1].table.anyPending());
  printf("sim: frames=%u collisions=%d alert_frames_at_15pct=%d max_sync_err_us=%d changes=%u\n",
         static_cast<unsigned>(f), w->collisions, frames_to_alert, w->max_abs_sync_err_us, static_cast<unsigned>(w->node_changes));
}

// Same topology, 25 random seeds, 20 % loss from the start (joins, relay setup and events all lossy).
static void test_many_seeds_lossy(void) {
  int worst_setup = 0, worst_alert = 0, worst_ack = 0, failures = 0;
  for (uint32_t seed = 1; seed <= 25; seed++) {
    World* x = new World();
    x->rng = seed * 7919u;
    x->loss_pct = 20;
    x->setLink(CHALET, A, true); x->setLink(CHALET, B, true); x->setLink(A, B, true); x->setLink(B, C, true);
    x->hub[2].table.observe(31, LS_IDLE, 0, 0, 90, 0);
    uint16_t f = 1;
    int setup = 0;
    while (!(x->hubHasSlot(A) && x->hubHasSlot(B) && x->hubHasSlot(C) && x->viaOf(C) == B) && setup < 60) { x->runFrame(f++); setup++; }
    x->hub[2].table.observe(31, LS_TRIPPED, 0, 0, 90, f);
    int alert = 0;
    while (x->chalet_state[31] != LS_TRIPPED && alert < 60) { x->runFrame(f++); alert++; }
    int ack = 0;
    while (x->hub[2].table.anyPending() && ack < 60) { x->runFrame(f++); ack++; }
    if (setup >= 60 || alert >= 60 || ack >= 60) failures++;
    if (setup > worst_setup) worst_setup = setup;
    if (alert > worst_alert) worst_alert = alert;
    if (ack > worst_ack) worst_ack = ack;
    delete x;
  }
  printf("seeds: worst setup=%d frames, worst alert=%d frames, worst ack=%d frames, failures=%d\n",
         worst_setup, worst_alert, worst_ack, failures);
  TEST_ASSERT_EQUAL(0, failures);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_full_scenario);
  RUN_TEST(test_many_seeds_lossy);
  return UNITY_END();
}
