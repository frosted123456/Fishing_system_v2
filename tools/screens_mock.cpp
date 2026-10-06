// Renders the v2 OLED screens (src/lora_node/screens.cpp) on the PC with the real U8g2 library, for review.
//   U8G2=<path to U8g2 src/clib>; gcc -O1 -I$U8G2 -c $U8G2/*.c && ar rcs libu8g2.a *.o
//   g++ -std=c++11 -I$U8G2 -Isrc/lora_node tools/screens_mock.cpp src/lora_node/screens.cpp libu8g2.a -o screens_mock
//   ./screens_mock outdir     -> one .pbm per screen (128x64), tools/screens_sheet.py makes the PNG sheet
#include "screens.h"
#include <sonar_sim.h>
using namespace icemesh;
#include <stdio.h>
#include <string.h>
#include <string>

static void hole(ScreenModel& m, uint8_t id, uint8_t hub, const char* name, uint8_t st, uint8_t batt, int8_t fish, uint32_t since = 0) {
  ScrHole& h = m.holes[m.n_holes++];
  memset(&h, 0, sizeof(h));
  h.id = id; h.hub = hub; snprintf(h.name, sizeof(h.name), "%s", name); h.state = st; h.batt = batt; h.fish = fish; h.since_s = since;
}
static void hub(ScreenModel& m, uint8_t id, int8_t rssi, bool lora, bool eb, uint8_t hops) {
  ScrHubLink& l = m.hubs[m.n_hubs++]; l.id = id; l.lora_rssi = rssi; l.lora_ok = lora; l.eb_ok = eb; l.eb_hops = hops;
}
static ScreenModel base(bool chalet) {
  ScreenModel m; memset(&m, 0, sizeof(m));
  m.chalet = chalet; m.self_id = chalet ? 100 : 2; m.lora_ch = 1; m.lora_mhz_x10 = 9150; m.ch_auto = true; m.busy_pct = 4;
  snprintf(m.ssid, sizeof(m.ssid), chalet ? "IceFish-Remote" : "IceFish-Hub2"); snprintf(m.pass, sizeof(m.pass), "fishon123");
  snprintf(m.url, sizeof(m.url), "http://192.168.4.1"); m.uptime_s = 7380; m.master_heard = true; m.beacon_rssi = -94;
  return m;
}
static void pockets3(ScreenModel& m) {
  hole(m, 1, 1, "Pointe", SH_OK, 92, 0); hole(m, 11, 1, "Baie", SH_OK, 81, 1); hole(m, 12, 1, "Roche", SH_OK, 77, 0);
  hole(m, 2, 2, "Drop-off", SH_OK, 88, 2); hole(m, 21, 2, "Herbier", SH_LOWBAT, 9, 0); hole(m, 22, 2, "", SH_OK, 70, 0);
  hole(m, 3, 3, "Large", SH_OK, 95, 0); hole(m, 31, 3, "Fosse", SH_OFFLINE, 255, -1); hole(m, 32, 3, "Chenal", SH_OK, 64, 3);
  hub(m, 1, -92, true, false, 0); hub(m, 2, -101, true, false, 0); hub(m, 3, -108, false, true, 1);
}

// realistic sonar data from the real fake-sonar chain (scene + processing), as the BASE/FOCUS blocks carry it
static void toCol(const sonar::Ping& p, ScrPingCol& c) {
  c.bottom_cm = p.bottom_cm >= sonar::DEPTH_NONE ? 0 : p.bottom_cm; c.n = 0;
  for (uint8_t k = 0; k < p.n_targets && c.n < 5; k++) { c.d[c.n] = p.t[k].depth_cm; c.lv[c.n] = p.t[k].level ? p.t[k].level : 1; c.n++; }
}
static void addSonar(ScreenModel& m, uint8_t focus, int pings) {
  for (uint8_t i = 0; i < m.n_holes; i++) {
    ScrHole& h = m.holes[i];
    if (h.fish < 0) continue;
    sonar::SonarSource src; src.begin(h.id, 1000u + h.id * 7u, 0);
    sonar::Block out[2];
    for (int k = 0; k < pings; k++) {
      src.tick(h.id == focus, out, 2);
      if (h.id == focus && k >= pings - 118) { toCol(src.lastPing(), m.cols[m.n_cols++]); }
    }
    const sonar::Ping& p = src.lastPing();
    h.son.valid = true; h.son.bottom_cm = p.bottom_cm >= sonar::DEPTH_NONE ? 0 : p.bottom_cm; h.son.n = 0; h.fish = 0;
    for (uint8_t k = 0; k < p.n_targets && h.son.n < 5; k++) {
      ScrTarget& t = h.son.t[h.son.n++];
      t.depth_cm = p.t[k].depth_cm; t.level = p.t[k].level ? p.t[k].level : 1; t.bait = p.t[k].track == 0;
      if (!t.bait) h.fish++;
    }
    if (h.id == focus) m.bait_cm = static_cast<uint16_t>(src.proc.bait_m * 100.0f + 0.5f);
  }
  m.focus_node = focus;
}

static void dump(u8g2_t* u, const std::string& path) {
  uint8_t* b = u8g2_GetBufferPtr(u);
  FILE* f = fopen(path.c_str(), "w");
  fprintf(f, "P1\n128 64\n");
  for (int y = 0; y < 64; y++) { for (int x = 0; x < 128; x++) fputc(((b[(y / 8) * 128 + x] >> (y % 8)) & 1) ? '1' : '0', f); fputc('\n', f); }
  fclose(f);
}

int main(int argc, char** argv) {
  const std::string out = argc > 1 ? argv[1] : ".";
  u8g2_t u; u8g2_Setup_ssd1306_128x64_noname_f(&u, U8G2_R0, u8x8_byte_empty, u8x8_dummy_cb); u8g2_InitDisplay(&u);
  struct Shot { const char* name; ScreenModel m; uint8_t page, sub; bool blink; };
  ScreenModel m;
  int k = 0;
  auto shot = [&](const char* name, const ScreenModel& mm, uint8_t page, uint8_t sub, bool blink) {
    screenDraw(&u, mm, page, sub, blink);
    char fn[64]; snprintf(fn, sizeof(fn), "/%02d_%s.pbm", k++, name); dump(&u, out + fn);
  };
  m = base(true); pockets3(m); shot("chalet_home", m, PG_HOME, 0, true);
  m = base(true); pockets3(m); m.holes[4].state = SH_FISH; m.holes[4].since_s = 35; shot("chalet_alert_on", m, PG_HOME, 0, true);
  shot("chalet_alert_off", m, PG_HOME, 0, false);
  m.silenced = true; m.silence_s = 4 * 60 + 12; shot("chalet_silenced", m, PG_HOME, 0, true);
  m = base(true); pockets3(m); m.setup_s = 260; m.hubs[2].eb_ok = false; shot("chalet_setup", m, PG_HOME, 0, true);
  m = base(true); pockets3(m); shot("chalet_holes", m, PG_HOLES, 0, true);
  m = base(true); pockets3(m); shot("chalet_network", m, PG_NETWORK, 0, true);
  m = base(true); pockets3(m); shot("chalet_connect_hotspot", m, PG_CONNECT, 0, true);
  m = base(true); pockets3(m); m.sta = true; snprintf(m.url, sizeof(m.url), "http://192.168.1.42"); shot("chalet_connect_cabin", m, PG_CONNECT, 0, true);
  m = base(true);
  for (int h = 1; h <= 10; h++) for (int j = 0; j < 4; j++) hole(m, j ? h * 10 + j : h, h, "", (h == 7 && j == 2) ? SH_OFFLINE : SH_OK, 80, 0);
  shot("chalet_home_10hubs", m, PG_HOME, 0, true);
  m = base(true); m.feet = true; pockets3(m); addSonar(m, 21, 400); shot("chalet_sonar_glance", m, PG_SONAR, 0, true);
  shot("chalet_focus", m, PG_FOCUS, 0, true);
  m.holes[0].sim = 1; m.holes[1].sim = 3; m.holes[4].sim = 1; m.holes[7].sim = 2; shot("chalet_test", m, PG_TEST, 0, true);
  m = base(false); pockets3(m); shot("hub_home", m, PG_HOME, 0, true);
  m = base(false); pockets3(m); m.master_heard = false; m.setup_s = 300; shot("hub_waiting", m, PG_HOME, 0, true);
  m = base(false); pockets3(m); m.hotspot = true; m.hotspot_min = 28; shot("hub_connect", m, PG_CONNECT, 0, true);
  m = base(false); pockets3(m); m.master_heard = false; m.on_backup = true; shot("hub_backup", m, PG_NETWORK, 0, true);
  printf("%d screens\n", k);
  return 0;
}
