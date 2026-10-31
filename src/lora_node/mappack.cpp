// D74: offline contour map ("map pack") kept on the chalet, and the optional write PIN of the web page.
//
// The pack is a JSON file made on the PC by tools/lake_prep.py (depth contours around a point, metres east /
// north of the pack origin lat0/lon0). The chalet never parses it: it stores the bytes on LittleFS (the "spiffs" partition:
// 4.8 MB with tools/partitions_chalet_8MB.csv, 1.5 MB on the default table) and streams them back to the phone,
// which draws the lines under the holes on the pocket map. Satellite tiles are fetched by the PHONE when it
// has internet: the chalet never opens a connection to the outside, so nothing here is a door in.
//
// D75 tile store: the page saves the map tiles of an area ("Save area") as files /t/<layer><z>_<x>_<y> and asks for
// them back with GET /api/tile?l=&z=&x=&y= when the phone has no internet. l: m map, s satellite, d depth overlay. The chalet never fetches a tile itself.
//                           GET /api/tile?l=m|s&z=&x=&y= -> the image (404 when none)
//                           POST /api/tile?... (multipart, field "tile") -> stored; {"error":"full"} when out of room
//                           GET /api/tiles -> {count, bytes, free}   POST /api/tiles/clear
// Endpoints (chalet only):  GET /api/map/pack  -> the file (404 when none)
//                           POST /api/map/pack (multipart, field "pack") -> stored, replaces the old one
//                           POST /api/map/clear
// PIN: settings.webPin (NVS "settings"/"webPin", empty = off, the default). When set, every POST of the page
// must carry "X-Pin: <pin>" or gets 401; the page asks once and remembers it. It is a lock against a stray
// phone on the chalet Wi-Fi, not encryption: the chalet speaks plain HTTP on its own AP / your LAN and must
// never be port-forwarded. Serial "PIN CLEAR" removes a forgotten PIN (needs the USB cable = physical access).
#include "chalet.h"
#include <LittleFS.h>

static const char* MAP_PATH = "/map_pack.json";
static size_t MAP_MAX_BYTES = 1200UL * 1024UL;   // set at mount: partition size minus LittleFS overhead (4.8 MB with tools/partitions_chalet_8MB.csv)
static bool g_fsOk = false;
static File g_up;                 // upload in progress
static size_t g_upBytes = 0;
static bool g_upFail = false, g_upSeen = false;
static const char* g_upErr = "";

static uint32_t g_tileCount = 0, g_tileBytes = 0;   // counted at boot, kept up to date on store / clear
static const size_t FS_RESERVE = 96UL * 1024UL;     // LittleFS needs free blocks to stay healthy

static void packRecount();
static void fsUsedRecount();
static size_t g_fsUsed = 0; static uint32_t g_fsUsedAt = 0;
static void tileRecount() {
  g_tileCount = 0; g_tileBytes = 0;
  File d = LittleFS.open("/t"); if (!d || !d.isDirectory()) return;
  for (File f = d.openNextFile(); f; f = d.openNextFile()) { if (!f.isDirectory()) { g_tileCount++; g_tileBytes += f.size(); } }
}

bool mapPackSetup() {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) return false;
  g_fsOk = LittleFS.begin(true);   // formats the partition on the first boot (a few seconds, once)
  if (!g_fsOk) { Serial.println(F("MAP: LittleFS mount failed (no map pack storage)")); return false; }
  MAP_MAX_BYTES = LittleFS.totalBytes() > 160UL * 1024UL ? LittleFS.totalBytes() - 128UL * 1024UL : 32UL * 1024UL;
  if (!LittleFS.exists("/t")) LittleFS.mkdir("/t");
  tileRecount(); packRecount(); fsUsedRecount();
  Serial.printf("MAP: LittleFS %u KB used of %u KB, pack %u bytes, %lu tiles (%lu KB)\n", (unsigned)(g_fsUsed / 1024), (unsigned)(LittleFS.totalBytes() / 1024), (unsigned)mapPackBytes(), (unsigned long)g_tileCount, (unsigned long)(g_tileBytes / 1024));
  return true;
}

// D84g: LittleFS.usedBytes() walks the whole partition (4.8 MB: ~600 ms, seen as /api/tiles in the PERF line, and it
// ran twice per tile upload and every 10 s of a range test). The used count is cached: exact after a mount or a
// clear, each store adds its bytes rounded up to the 4 KB block, and a count older than a minute is redone when
// asked (so the range-test log, written elsewhere, is seen within a minute).
static void fsUsedRecount() { g_fsUsed = g_fsOk ? LittleFS.usedBytes() : 0; g_fsUsedAt = millis(); }
static void fsUsedAdd(size_t bytes) { g_fsUsed += ((bytes + 4095) / 4096) * 4096 + 4096; }
size_t mapFree() {
  if (!g_fsOk) return 0;
  if (millis() - g_fsUsedAt > 60000) fsUsedRecount();
  return LittleFS.totalBytes() < g_fsUsed + FS_RESERVE ? 0 : LittleFS.totalBytes() - g_fsUsed - FS_RESERVE;
}

// ---- tiles ------------------------------------------------------------------------------------------------
static bool tilePath(char* out, size_t n) {   // from the request's l/z/x/y; false = bad request
  const String l = server.arg("l"); const long z = server.arg("z").toInt(), x = server.arg("x").toInt(), y = server.arg("y").toInt();
  if ((l != "m" && l != "s" && l != "d") || z < 0 || z > 19 || x < 0 || y < 0 || x >= (1L << z) || y >= (1L << z)) return false;
  snprintf(out, n, "/t/%s%ld_%ld_%ld", l.c_str(), z, x, y);
  return true;
}
void handleWebApiTile() {   // GET
  char path[40];
  if (!g_fsOk || !tilePath(path, sizeof(path)) || !LittleFS.exists(path)) { server.send(404, "text/plain", "no tile"); return; }
  File f = LittleFS.open(path, "r");
  if (!f) { server.send(500, "text/plain", "open"); return; }
  uint8_t magic[2] = {0, 0}; f.read(magic, 2); f.seek(0);
  server.sendHeader("Cache-Control", "max-age=86400");
  server.streamFile(f, magic[0] == 0xFF ? "image/jpeg" : (magic[0] == 0x89 ? "image/png" : "application/octet-stream"));
  f.close();
}
static char g_tilePath[40]; static size_t g_tileUpBytes = 0; static bool g_tileFail = false, g_tileSeen = false; static const char* g_tileErr = "";
void handleWebApiTileUpload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_tileUpBytes = 0; g_tileFail = false; g_tileSeen = true; g_tileErr = "";
    if (!g_fsOk) { g_tileFail = true; g_tileErr = "no storage"; return; }
    if (settings.webPin[0] && !(server.hasHeader("X-Pin") && server.header("X-Pin") == settings.webPin)) { g_tileFail = true; g_tileErr = "pin"; return; }
    if (!tilePath(g_tilePath, sizeof(g_tilePath))) { g_tileFail = true; g_tileErr = "bad tile"; return; }
    if (mapFree() < 80UL * 1024UL) { g_tileFail = true; g_tileErr = "full"; return; }
    LittleFS.remove("/t/_up"); g_up = LittleFS.open("/t/_up", "w");
    if (!g_up) { g_tileFail = true; g_tileErr = "open"; }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_tileFail || !g_up) return;
    if (g_tileUpBytes + up.currentSize > 200UL * 1024UL) { g_tileFail = true; g_tileErr = "too big"; g_up.close(); return; }
    if (g_up.write(up.buf, up.currentSize) != up.currentSize) { g_tileFail = true; g_tileErr = "full"; g_up.close(); return; }
    g_tileUpBytes += up.currentSize;
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_up) g_up.close();
    if (!g_tileFail && g_tileUpBytes > 0) {
      if (LittleFS.exists(g_tilePath)) { File o = LittleFS.open(g_tilePath, "r"); if (o) { g_tileBytes -= o.size(); g_tileCount--; o.close(); } LittleFS.remove(g_tilePath); }
      LittleFS.rename("/t/_up", g_tilePath); g_tileCount++; g_tileBytes += g_tileUpBytes; fsUsedAdd(g_tileUpBytes);
    } else { LittleFS.remove("/t/_up"); if (!g_tileFail) { g_tileFail = true; g_tileErr = "empty"; } }
  } else { if (g_up) g_up.close(); LittleFS.remove("/t/_up"); g_tileFail = true; g_tileErr = "aborted"; }
}
void handleWebApiTilePost() {
  if (!webPinOk()) return;
  if (!g_tileSeen) { server.send(400, "application/json", "{\"error\":\"multipart upload expected (field tile)\"}"); return; }
  g_tileSeen = false;
  if (g_tileFail) { server.send(strcmp(g_tileErr, "pin") == 0 ? 401 : 200, "application/json", String("{\"ok\":false,\"error\":\"") + g_tileErr + "\"}"); return; }
  server.send(200, "application/json", String("{\"ok\":true,\"bytes\":") + g_tileUpBytes + ",\"free\":" + mapFree() + "}");
}
void handleWebApiTiles() {   // GET: what the store holds
  server.send(200, "application/json", String("{\"count\":") + g_tileCount + ",\"bytes\":" + g_tileBytes + ",\"free\":" + mapFree() + ",\"pack\":" + mapPackBytes() + "}");
}
void handleWebApiTilesClear() {
  if (!webPinOk()) return;
  if (g_fsOk) {
    File d = LittleFS.open("/t");
    if (d && d.isDirectory()) { String names[64]; int n = 0; bool more = true;
      while (more) { n = 0; more = false; d.rewindDirectory();
        for (File f = d.openNextFile(); f; f = d.openNextFile()) { if (f.isDirectory()) continue; if (n < 64) names[n++] = f.path(); else more = true; }
        for (int i = 0; i < n; i++) LittleFS.remove(names[i]); }
    }
    tileRecount(); fsUsedRecount();
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

size_t mapPackMax() { return g_fsOk ? MAP_MAX_BYTES : 0; }
// D84c: the size is cached. /api/status asked it every 2 s and each call opened LittleFS (an "open failed" line on the
// serial each time when there is no pack, and a slow status reply). Refreshed at mount, after a store and after a clear.
static size_t g_packBytes = 0;
static void packRecount() {
  g_packBytes = 0;
  if (!g_fsOk || !LittleFS.exists(MAP_PATH)) return;
  File f = LittleFS.open(MAP_PATH, "r"); if (f) { g_packBytes = f.size(); f.close(); }
}
size_t mapPackBytes() { return g_packBytes; }

bool mapPackClear() { if (!g_fsOk) return false; if (LittleFS.exists(MAP_PATH)) LittleFS.remove(MAP_PATH); g_packBytes = 0; fsUsedRecount(); return true; }

// ---- PIN ----------------------------------------------------------------------------------------------------
bool webPinOk() {
  if (settings.webPin[0] == 0) return true;
  if (server.hasHeader("X-Pin") && server.header("X-Pin") == settings.webPin) return true;
  server.send(401, "application/json", "{\"error\":\"pin\"}");
  return false;
}

// ---- endpoints ----------------------------------------------------------------------------------------------
void handleWebApiMapPack() {   // GET
  if (!g_fsOk || !LittleFS.exists(MAP_PATH)) { server.send(404, "application/json", "{\"error\":\"no map pack\"}"); return; }
  File f = LittleFS.open(MAP_PATH, "r");
  if (!f) { server.send(500, "application/json", "{\"error\":\"open\"}"); return; }
  server.sendHeader("Cache-Control", "no-cache");
  server.streamFile(f, "application/json");
  f.close();
}

void handleWebApiMapPackUpload() {   // multipart body of POST /api/map/pack: runs once per chunk
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_upBytes = 0; g_upFail = false; g_upSeen = true; g_upErr = "";
    if (!g_fsOk) { g_upFail = true; g_upErr = "no storage"; return; }
    if (settings.webPin[0] && !(server.hasHeader("X-Pin") && server.header("X-Pin") == settings.webPin)) { g_upFail = true; g_upErr = "pin"; return; }
    LittleFS.remove("/map_pack.tmp");
    g_up = LittleFS.open("/map_pack.tmp", "w");
    if (!g_up) { g_upFail = true; g_upErr = "open"; }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_upFail || !g_up) return;
    if (g_upBytes == 0 && (up.currentSize == 0 || up.buf[0] != '{')) { g_upFail = true; g_upErr = "not a map pack (JSON expected)"; g_up.close(); return; }
    if (g_upBytes + up.currentSize > MAP_MAX_BYTES) { g_upFail = true; g_upErr = "too big for the chalet storage"; g_up.close(); return; }
    if (g_up.write(up.buf, up.currentSize) != up.currentSize) { g_upFail = true; g_upErr = "write (storage full?)"; g_up.close(); return; }
    g_upBytes += up.currentSize;
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_up) g_up.close();
    if (!g_upFail && g_upBytes > 0) { LittleFS.remove(MAP_PATH); LittleFS.rename("/map_pack.tmp", MAP_PATH); g_packBytes = g_upBytes; fsUsedAdd(g_upBytes); Serial.printf("MAP: pack stored, %u bytes\n", (unsigned)g_upBytes); }
    else { LittleFS.remove("/map_pack.tmp"); if (!g_upFail) { g_upFail = true; g_upErr = "empty"; } }
  } else {   // UPLOAD_FILE_ABORTED
    if (g_up) g_up.close(); LittleFS.remove("/map_pack.tmp"); g_upFail = true; g_upErr = "aborted";
  }
}

void handleWebApiMapPackPost() {   // after the upload handler
  if (!webPinOk()) return;
  if (!g_upSeen) { server.send(400, "application/json", "{\"error\":\"multipart upload expected (field pack)\"}"); return; }
  g_upSeen = false;
  if (g_upFail) { server.send(strcmp(g_upErr, "pin") == 0 ? 401 : 400, "application/json", String("{\"error\":\"") + g_upErr + "\"}"); return; }
  server.send(200, "application/json", String("{\"ok\":true,\"bytes\":") + g_upBytes + "}");
}

void handleWebApiMapClear() {
  if (!webPinOk()) return;
  mapPackClear();
  server.send(200, "application/json", "{\"ok\":true}");
}

// ---- D76: GPS actions from the page: {"origin":true} = chalet here, {"pin":<hole>} = that hole is where the box is,
// {"clear":"all"} = forget every hole position. 409 when there is no fix / origin.
void handleWebApiGpsPost() {
  StaticJsonDocument<96> doc;
  if (!server.hasArg("plain") || deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}"); return; }
  if (currentRole != ROLE_GATEWAY_OFFSHORE) { server.send(400, "application/json", "{\"error\":\"chalet only\"}"); return; }
  if (doc["origin"] | false) { if (!gpsSetOrigin()) { server.send(409, "application/json", "{\"error\":\"no GPS fix yet\"}"); return; } }
  else if (doc.containsKey("pin")) { const int id = doc["pin"] | 0; if (id <= 0 || id >= 255) { server.send(400, "application/json", "{\"error\":\"bad hole\"}"); return; }
    if (!gpsPinHole((uint8_t)id)) { server.send(409, "application/json", "{\"error\":\"no GPS fix, no origin, or more than 3 km from it\"}"); return; } }
  else if (doc.containsKey("clear")) { for (int i = 1; i < 255; i++) posSet((uint8_t)i, POS_UNSET, POS_UNSET); }
  else { server.send(400, "application/json", "{\"error\":\"origin | pin | clear\"}"); return; }
  server.send(200, "application/json", "{\"ok\":true}");
}
