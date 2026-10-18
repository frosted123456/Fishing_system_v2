// v2 (D73) GPS on the chalet box (NEO-6M class module on UART1): parsed by lib/IceMesh/src/nmea.h, used to pin
// holes on the pocket map from the OLED (Options > Pin holes (GPS)) and reported in /api/status and the wiring check.
// The module's TX must reach the ESP32's RX: the two wires are easy to cross after three years, so if no NMEA
// sentence arrives in GPS_SWAP_MS the pins are swapped once and the status says so.
#include "chalet.h"
#include <nmea.h>

#if GPS_RX_PIN >= 0
static HardwareSerial GpsSerial(1);
static icemesh::NmeaParser g_nmea;
static bool g_gpsOn = false, g_gpsSwapped = false;
static uint32_t g_gpsStartMs = 0, g_gpsLastSentenceMs = 0, g_gpsFixMs = 0;
static int32_t g_lat0 = 0, g_lon0 = 0; static bool g_originSet = false;

static void gpsOpen(bool swapped) {
  GpsSerial.end();
  const int rx = swapped ? GPS_TX_PIN : GPS_RX_PIN, tx = swapped ? GPS_RX_PIN : GPS_TX_PIN;
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, rx, tx);
  g_gpsSwapped = swapped; g_gpsStartMs = millis();
}

void gpsSetup() {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) return;
  Preferences p; p.begin("gps", true);
  g_originSet = p.getBool("orig", false); g_lat0 = p.getInt("lat0", 0); g_lon0 = p.getInt("lon0", 0);
  p.end();
  gpsOpen(false);
  g_gpsOn = true;
  Serial.printf("GPS: UART1 rx GPIO%d tx GPIO%d at %u baud%s\n", GPS_RX_PIN, GPS_TX_PIN, (unsigned)GPS_BAUD, g_originSet ? ", map origin set" : "");
}

void gpsLoop() {
  if (!g_gpsOn) return;
  int guard = 512;   // bounded work per loop()
  while (guard-- && GpsSerial.available()) {
    if (g_nmea.feed((char)GpsSerial.read())) { if (g_nmea.fix().valid) g_gpsFixMs = millis(); }
  }
  if (g_nmea.sentences()) g_gpsLastSentenceMs = millis();
  if (!g_nmea.sentences() && !g_gpsSwapped && millis() - g_gpsStartMs > GPS_SWAP_MS) {
    gpsOpen(true);   // nothing in GPS_SWAP_MS: try the other way round once
    Serial.println(F("GPS: no sentence yet, trying the UART pins swapped"));
  }
}

bool gpsStatus(GpsStatus& s) {
  memset(&s, 0, sizeof(s));
  if (!g_gpsOn) return false;
  const icemesh::GpsFix& f = g_nmea.fix();
  s.sentences = g_nmea.sentences(); s.bad = g_nmea.bad();
  s.data_age_s = g_gpsLastSentenceMs ? (millis() - g_gpsLastSentenceMs) / 1000 : 0xFFFFFFFF;
  s.fix = f.valid && millis() - g_gpsFixMs < 5000;
  s.sats = f.sats; s.hdop_x10 = f.hdop_x10; s.lat_e7 = f.lat_e7; s.lon_e7 = f.lon_e7;
  s.swapped = g_gpsSwapped; s.origin_set = g_originSet; s.lat0_e7 = g_lat0; s.lon0_e7 = g_lon0;
  s.hh = f.hh; s.mm = f.mm;
  return true;
}

bool gpsSetOrigin() {   // "Chalet here": the map's (0, 0)
  GpsStatus s; if (!gpsStatus(s) || !s.fix) return false;
  g_lat0 = s.lat_e7; g_lon0 = s.lon_e7; g_originSet = true;
  Preferences p; p.begin("gps", false); p.putBool("orig", true); p.putInt("lat0", g_lat0); p.putInt("lon0", g_lon0); p.end();
  return true;
}

bool gpsPinHole(uint8_t hole) {   // the hole is where the box is now: metres from the origin into the map store
  GpsStatus s; if (!gpsStatus(s) || !s.fix || !s.origin_set) return false;
  int32_t x, y; icemesh::gpsOffsetDm(s.lat0_e7, s.lon0_e7, s.lat_e7, s.lon_e7, x, y);
  if (x < -30000 || x > 30000 || y < -30000 || y > 30000) return false;   // 3 km: not this lake
  posSet(hole, (int16_t)x, (int16_t)y);
  return true;
}
#else
void gpsSetup() {}
void gpsLoop() {}
bool gpsStatus(GpsStatus& s) { memset(&s, 0, sizeof(s)); return false; }
bool gpsSetOrigin() { return false; }
bool gpsPinHole(uint8_t) { return false; }
#endif
