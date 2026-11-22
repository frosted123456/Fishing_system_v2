// D80 range test (chalet). Frank walks with the chalet box; one hub stays put (its hole pinned with the GPS).
// That hub's slot cycles through every LoRa mode, one per frame (SF9, SF8, SF7, SF6, SF5, SF9, ...: each mode
// once every 5 s); every slot result goes to a CSV on the chalet's storage with the box's GPS fix and its
// distance to the hub. The page shows a live per-mode summary and lists the runs to download.
//
// Files: LittleFS /rt/run_NN.csv (same partition as the map tiles; a run refuses to start below 64 KB free and
// stops by itself below 32 KB, after 2 h, or when the hub is gone for 5 min). The rest of the mesh keeps running:
// trips and alerts are not affected; only the test hub's own sonar data is thin in the frames whose mode fails.
#include "chalet.h"
#include <LittleFS.h>
#include <nmea.h>
#include <range_test.h>
#include <math.h>

using namespace icemesh;
using namespace icemesh::tdma;

static const uint32_t RT_MAX_MS = 2UL * 60UL * 60UL * 1000UL;
static const uint32_t RT_GONE_MS = 5UL * 60UL * 1000UL;
static const size_t RT_MIN_FREE_START = 64UL * 1024UL, RT_MIN_FREE_RUN = 32UL * 1024UL;
static const uint8_t RT_MAX_RUNS = 30;
static uint8_t g_tr_saved = 3;   // D85c: transport before the run (put back at stop)

static bool g_on = false;
static uint8_t g_hub = 0, g_run = 0;
static char g_note[48] = "";
static uint32_t g_start_ms = 0, g_last_ok_ms = 0, g_last_flush_ms = 0, g_last_free_ms = 0, g_rows = 0;
static File g_f;
static RangeStats g_stats;
static RangeSample g_last; static bool g_has_last = false;
static float g_dist = -1;
static char g_stop_why[40] = "";

bool rangeRunning() { return g_on; }

static void runPath(uint8_t n, char* out, size_t cap) { snprintf(out, cap, "/rt/run_%02u.csv", n); }

// next free run number (1..99); `count` = runs on storage
static uint8_t nextRun(uint8_t& count) {
  uint8_t best = 0; count = 0;
  File d = LittleFS.open("/rt");
  if (!d || !d.isDirectory()) return 1;
  for (File f = d.openNextFile(); f; f = d.openNextFile()) {
    unsigned n = 0;
    if (sscanf(f.name(), "run_%u.csv", &n) == 1 && n >= 1 && n <= 99) { count++; if (n > best) best = (uint8_t)n; }
  }
  if (best < 99) return (uint8_t)(best + 1);
  for (uint8_t n = 1; n <= 99; n++) { char p[24]; runPath(n, p, sizeof(p)); if (!LittleFS.exists(p)) return n; }   // reuse a gap
  return 0;
}
static String g_runs_json; static bool g_runs_dirty = true;   // runs list cache (the GET is polled every 2 s)

// metres from the box (its current fix) to the hub's pinned position; -1 when unknown
static float distanceToHub(const GpsStatus& g) {
  if (!g.fix || !g.origin_set) return -1;
  if (holePosDm[g_hub][0] == POS_UNSET) return -1;
  int32_t x, y; gpsOffsetDm(g.lat0_e7, g.lon0_e7, g.lat_e7, g.lon_e7, x, y);
  const float dx = (x - holePosDm[g_hub][0]) / 10.0f, dy = (y - holePosDm[g_hub][1]) / 10.0f;
  return sqrtf(dx * dx + dy * dy);
}

bool rangeStart(uint8_t hub, const char* note, String& err) {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) { err = "chalet only"; return false; }
  if (g_on) { err = "a run is already going"; return false; }
  RangeHubSide hs; bool direct = false; uint8_t max_sf = 7;
  if (hub == 0 || !meshRangeHubSide(hub, hs, direct, max_sf)) { err = "that hub is not in the network right now"; return false; }
  if (!direct) { err = "that hub is relayed: the test needs a hub the chalet hears directly"; return false; }
  if (meshHubRelaysOthers(hub)) { err = "that hub relays other hubs: pick another one (their data would drop at the edge)"; return false; }
  if (mapPackMax() == 0) { err = "storage not available (LittleFS not mounted)"; return false; }
  if (mapFree() < RT_MIN_FREE_START) { err = "storage full: delete old runs or saved map areas"; return false; }
  LittleFS.mkdir("/rt");
  uint8_t count = 0;
  const uint8_t n = nextRun(count);
  if (n == 0 || count >= RT_MAX_RUNS) { err = "30 runs saved: download and delete some first"; return false; }
  char path[24]; runPath(n, path, sizeof(path));
  g_f = LittleFS.open(path, "w");
  if (!g_f) { err = "could not create the file"; return false; }
  g_hub = hub; g_run = n; g_rows = 0; g_stats.clear(); g_has_last = false; g_dist = -1; g_stop_why[0] = 0;
  strlcpy(g_note, note ? note : "", sizeof(g_note));
  for (char* p = g_note; *p; p++) if ((uint8_t)*p < 0x20 || *p == ',' || *p == '"' || *p == '\\') *p = ' ';
  GpsStatus g; const bool gps = gpsStatus(g);
  g_f.printf("# icefish range test run %u, hub %u, chalet %u, firmware D85, hub modes SF9..SF%u\n", n, hub, NODE_ID, max_sf);
  g_f.printf("# note: %s\n", g_note);
  if (gps && g.origin_set && holePosDm[hub][0] != POS_UNSET)
    g_f.printf("# hub position: %.1f m east, %.1f m north of the map origin %.7f, %.7f\n",
               holePosDm[hub][0] / 10.0, holePosDm[hub][1] / 10.0, g.lat0_e7 / 1e7, g.lon0_e7 / 1e7);
  else g_f.printf("# hub position: not pinned (no distance column; lat/lon still logged)\n");
  g_f.print(rangeCsvHeader());
  g_f.flush();
  g_start_ms = g_last_ok_ms = g_last_flush_ms = millis();
  // D85c: the backbone goes to LoRa for the run. With "ESP-NOW first" a hub next to the chalet talks ESP-NOW and keeps its
  // LoRa radio idle: it never transmits in its slot and every row of the run reads "missed" (Frank's side-by-side run 1).
  g_tr_saved = meshTransport(); meshSetTransport(MESH_TR_LORA);
  meshRangeStart(hub);
  g_on = true; g_runs_dirty = true;
  Serial.printf("RANGE: run %u started on hub %u (%s)\n", n, hub, path);
  return true;
}

void rangeStop(const char* why) {
  if (!g_on) return;
  meshRangeStop();
  meshSetTransport(g_tr_saved);   // D85c: the backbone goes back to the setting
  g_on = false;
  if (g_f) { g_f.printf("# stopped: %s, %lu rows, %lu samples lost (queue)\n", why, (unsigned long)g_rows, (unsigned long)meshRangeLost()); g_f.close(); }
  strlcpy(g_stop_why, why, sizeof(g_stop_why));
  g_runs_dirty = true;
  Serial.printf("RANGE: run %u stopped (%s), %lu rows\n", g_run, why, (unsigned long)g_rows);
}

void rangeLoop() {
  if (!g_on) return;
  const uint32_t now = millis();
  RangeSample s;
  uint8_t budget = 24;
  GpsStatus g; bool gps = false;
  RangeHubSide hs; bool hs_read = false;
  while (budget-- && meshRangePop(s)) {
    if (!gps) { gps = gpsStatus(g); if (!gps) memset(&g, 0, sizeof(g)); }
    RangeFix f; memset(&f, 0, sizeof(f));
    if (gps && g.fix) { f.fix = true; f.lat_e7 = g.lat_e7; f.lon_e7 = g.lon_e7; f.sats = g.sats; f.hdop_x10 = g.hdop_x10; f.hh = g.hh; f.mm = g.mm; f.ss = g.ss; }
    const float d = gps ? distanceToHub(g) : -1;
    if (!hs_read) { bool direct = false; uint8_t max_sf = 7; meshRangeHubSide(g_hub, hs, direct, max_sf); hs_read = true; }
    char line[200];
    const size_t n = formatRangeRow(line, sizeof(line), now - g_start_ms, f, d, s, hs);
    if (n && g_f) { g_f.write((const uint8_t*)line, n); g_rows++; }
    g_stats.add(s, d);
    g_last = s; g_has_last = true; g_dist = d;
    if (s.ok) g_last_ok_ms = now;
  }
  if (now - g_last_flush_ms > 5000) { g_last_flush_ms = now; if (g_f) g_f.flush(); }
  if (now - g_start_ms > RT_MAX_MS) rangeStop("2 h limit");
  else if (now - g_last_ok_ms > RT_GONE_MS) rangeStop("hub not heard for 5 min");
  else if (now - g_last_free_ms > 10000) { g_last_free_ms = now; if (mapFree() < RT_MIN_FREE_RUN) rangeStop("storage full"); }
  if (g_on && meshRangeHub() != g_hub) rangeStop("stopped by the radio");
}

void rangeOledLine(char* out, size_t n) {
  if (!g_on) { out[0] = 0; return; }
  char d[12] = "?m";
  if (g_dist >= 0) snprintf(d, sizeof(d), "%um", (unsigned)lroundf(g_dist));
  if (!g_has_last) { snprintf(out, n, "RANGE TEST %s", d); return; }
  snprintf(out, n, "RANGE %s SF%u %s", d, validMode(g_last.mode) ? modeInfo((RadioMode)g_last.mode).sf : 0, g_last.ok ? "ok" : "--");
}

// ---- web ------------------------------------------------------------------------------------------------
static void buildRunsJson();
void handleWebApiRange() {   // GET: live state + runs
  String s; s.reserve(1500);
  s = "{\"running\":"; s += g_on ? "true" : "false";
  s += ",\"hub\":"; s += g_hub; s += ",\"run\":"; s += g_run;
  s += ",\"note\":\""; for (const char* p = g_note; *p; p++) { if (*p == '"' || *p == '\\') s += '\\'; s += *p; } s += "\"";
  s += ",\"secs\":"; s += g_on ? (millis() - g_start_ms) / 1000 : 0;
  s += ",\"rows\":"; s += g_rows;
  s += ",\"dist\":"; if (g_dist >= 0) s += String(g_dist, 1); else s += "null";
  s += ",\"stopped\":\""; s += g_stop_why; s += "\"";
  if (g_has_last) { s += ",\"last\":{\"sf\":"; s += validMode(g_last.mode) ? modeInfo((RadioMode)g_last.mode).sf : 0; s += ",\"ok\":"; s += g_last.ok ? "true" : "false"; s += "}"; }
  s += ",\"modes\":[";
  for (uint8_t m = 0; m < MODE_COUNT; m++) {
    const RangeModeStats& x = g_stats.m[m];
    if (m) s += ",";
    s += "{\"sf\":"; s += modeInfo((RadioMode)m).sf; s += ",\"sent\":"; s += x.sent; s += ",\"ok\":"; s += x.ok; s += ",\"crc\":"; s += x.crc;
    if (x.ok) { s += ",\"rssi_min\":"; s += x.rssi_min; s += ",\"rssi_avg\":"; s += String((float)x.rssi_sum / x.ok, 1); s += ",\"snr_min\":"; s += String(x.snr_min_q4 / 4.0, 2); }
    s += ",\"max_ok_m\":"; if (x.max_ok_m >= 0) s += String(x.max_ok_m, 0); else s += "null";
    s += "}";
  }
  s += "],\"runs\":";
  if (g_runs_dirty) { buildRunsJson(); g_runs_dirty = false; }
  s += g_runs_json;
  s += "}";
  server.send(200, "application/json", s);
}

static void buildRunsJson() {
  String& s = g_runs_json; s = "[";
  File d = LittleFS.open("/rt"); bool first = true; uint8_t listed = 0;
  if (d && d.isDirectory()) {
    for (File f = d.openNextFile(); f && listed < 99; f = d.openNextFile()) {
      unsigned n = 0;
      if (sscanf(f.name(), "run_%u.csv", &n) != 1) continue;
      f.setTimeout(0);   // a short or empty file must not stall the web server
      char note[60] = ""; char l1[96], l2[96]; size_t a = f.readBytesUntil('\n', l1, sizeof(l1) - 1); l1[a] = 0; a = f.readBytesUntil('\n', l2, sizeof(l2) - 1); l2[a] = 0;
      if (strncmp(l2, "# note: ", 8) == 0) strlcpy(note, l2 + 8, sizeof(note));
      unsigned hub = 0; const char* hp = strstr(l1, "hub "); if (hp) sscanf(hp, "hub %u", &hub);
      if (!first) s += ","; first = false; listed++;
      s += "{\"n\":"; s += n; s += ",\"bytes\":"; s += (uint32_t)f.size(); s += ",\"hub\":"; s += hub; s += ",\"note\":\"";
      for (const char* p = note; *p; p++) { if (*p == '"' || *p == '\\') s += '\\'; s += *p; }
      s += "\"}";
    }
  }
  s += "]";
}

void handleWebApiRangePost() {   // {start:{hub,note}} | {stop:true} | {delete:n}
  StaticJsonDocument<192> doc;
  if (!server.hasArg("plain") || deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}"); return; }
  if (doc.containsKey("start")) {
    String err;
    if (!rangeStart((uint8_t)(doc["start"]["hub"] | 0), doc["start"]["note"] | "", err)) { server.send(409, "application/json", String("{\"error\":\"") + err + "\"}"); return; }
  } else if (doc["stop"] | false) {
    rangeStop("stopped from the page");
  } else if (doc.containsKey("delete")) {
    const int n = doc["delete"] | 0;
    if (n < 1 || n > 99) { server.send(400, "application/json", "{\"error\":\"bad run number\"}"); return; }
    if (g_on && n == g_run) { server.send(409, "application/json", "{\"error\":\"this run is going\"}"); return; }
    char path[24]; runPath((uint8_t)n, path, sizeof(path));
    LittleFS.remove(path);
    g_runs_dirty = true;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleWebApiRangeRun() {   // GET ?n= : the CSV
  const int n = server.arg("n").toInt();
  if (n < 1 || n > 99) { server.send(404, "text/plain", "no such run"); return; }
  char path[24]; runPath((uint8_t)n, path, sizeof(path));
  if (!LittleFS.exists(path)) { server.send(404, "text/plain", "no such run"); return; }
  if (g_on && n == g_run && g_f) g_f.flush();
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(500, "text/plain", "could not open the run"); return; }
  char cd[64]; snprintf(cd, sizeof(cd), "attachment; filename=\"icefish_range_run%02d.csv\"", n);
  server.sendHeader("Content-Disposition", cd);
  server.streamFile(f, "text/csv");
  f.close();
}
