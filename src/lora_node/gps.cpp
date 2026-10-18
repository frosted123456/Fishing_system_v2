// v2 (D73) GPS on the chalet box (NEO-6M class module on UART1): parsed by lib/IceMesh/src/nmea.h, used to pin
// holes on the pocket map from the OLED (Options > Pin holes (GPS)) and reported in /api/status and the wiring check.
// The module's TX must reach the ESP32's RX: the two wires are easy to cross after three years, so if no NMEA
// sentence arrives in GPS_SWAP_MS the pins are swapped once and the status says so.
#include "chalet.h"
#include <nmea.h>
#include <driver/gpio.h>

#if GPS_RX_PIN >= 0
static HardwareSerial GpsSerial(1);
static icemesh::NmeaParser g_nmea;
static bool g_gpsOn = false;
static int8_t g_rxPin = -1;                   // the ESP32 pin the module's TX is on (found by the scan, kept in NVS)
static uint32_t g_gpsStartMs = 0, g_gpsLastSentenceMs = 0, g_gpsFixMs = 0, g_sentencesAtOpen = 0;
static int32_t g_lat0 = 0, g_lon0 = 0; static bool g_originSet = false;
// D73: the module's TX wire can be on any free header pin (wired three years ago, "pretty sure 45/46"): instead of
// guessing, the firmware listens on each candidate for GPS_SCAN_MS and keeps the first one that delivers NMEA.
// Only RX matters: the firmware never talks to the module. UART0 (43/44 = USB serial), the OLED, LoRa, CardKB,
// buzzer, LED, Vext, battery and PRG pins are not candidates.
static const int8_t GPS_CANDIDATES[] = {GPS_RX_PIN, GPS_TX_PIN, 5, 4, 45, 33, 34, 38, 39, 40, 2, 6, -1};   // not 26 / 46: the MOSFET gates
static int8_t g_scanIdx = -1;                 // -1 = not scanning
static char g_levels[96] = "";                // last round: what each candidate pin looked like electrically
static uint16_t g_lvlHigh = 0, g_lvlEdges = 0;   // samples high / level changes on the pin in use (sampled in gpsLoop)
static uint16_t g_lvlN = 0; static uint8_t g_lvlLast = 2;
static const uint32_t GPS_BAUDS[] = {9600, 38400, 115200};   // NEO-6M default, NEO-M8 / ATGM336H factory rates
static uint8_t g_baudIdx = 0;
static bool g_viaUart0 = false;               // sentences arrive on the USB serial (module TX on the header pin marked RX = GPIO44)

static void gpsOpen(int8_t rx) {
  GpsSerial.end();
  GpsSerial.begin(GPS_BAUDS[g_baudIdx], SERIAL_8N1, rx, -1);   // RX only
  // The UART driver pulls its RX pin up, so an unconnected pin reads H like a live TX would. During the scan the
  // pin is pulled DOWN instead: H then means something drives it high (a powered module's TX), L = nothing there
  // or an unpowered module (its output clamps to its dead supply).
  gpio_set_pull_mode((gpio_num_t)rx, g_scanIdx >= 0 ? GPIO_PULLDOWN_ONLY : GPIO_PULLUP_ONLY);
  g_rxPin = rx; g_gpsStartMs = millis(); g_sentencesAtOpen = g_nmea.sentences();
}

static void gpsScanStep() {   // next candidate
  g_scanIdx++;
  if (g_scanIdx > 0 && g_lvlN) {   // what the previous candidate looked like: H = idle high (a live UART TX), L = low / nothing, ~ = toggling
    char b[12]; snprintf(b, sizeof(b), " %d:%s", GPS_CANDIDATES[g_scanIdx - 1], g_lvlEdges > 20 ? "~" : (g_lvlHigh * 10 > g_lvlN * 9 ? "H" : (g_lvlHigh * 10 < g_lvlN ? "L" : "?")));
    if (strlen(g_levels) + strlen(b) < sizeof(g_levels)) strcat(g_levels, b);
  }
  g_lvlHigh = g_lvlEdges = g_lvlN = 0; g_lvlLast = 2;
  if (GPS_CANDIDATES[g_scanIdx] < 0) {   // round done at this baud rate: next rate, or back to the configured pin
    Serial.printf("GPS: pin levels this round (pull-down on):%s  (H = driven high = a powered UART TX, L = nothing / unpowered module, ~ = toggling)\n", g_levels);
    g_scanIdx = -1;
    g_baudIdx = (uint8_t)((g_baudIdx + 1) % 3);
    gpsOpen(GPS_RX_PIN);
    Serial.printf("GPS: no NMEA on any candidate pin at %lu baud (module powered? TX wire on a header pin?); next round at %lu\n",
                  (unsigned long)GPS_BAUDS[(g_baudIdx + 2) % 3], (unsigned long)GPS_BAUDS[g_baudIdx]);
    return;
  }
  gpsOpen(GPS_CANDIDATES[g_scanIdx]);
}

void gpsScanStart() {
  g_scanIdx = -1; g_levels[0] = 0;
  gpsScanStep();
  Serial.println(F("GPS: scanning the header pins for the module's TX"));
}

void gpsSetup() {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) return;
  Preferences p; p.begin("gps", true);
  g_originSet = p.getBool("orig", false); g_lat0 = p.getInt("lat0", 0); g_lon0 = p.getInt("lon0", 0);
  const int8_t saved = (int8_t)p.getChar("rx", -1);
  p.end();
  { Preferences q; q.begin("gps", true); g_baudIdx = q.getUChar("baud", 0) % 3; q.end(); }
  #if GPS_EN_PIN >= 0
  pinMode(GPS_EN_PIN, OUTPUT); digitalWrite(GPS_EN_PIN, HIGH);   // Meshtastic wiring: MOSFET on the module's GND
  #endif
  #if GPS_EN_PIN2 >= 0
  pinMode(GPS_EN_PIN2, OUTPUT); digitalWrite(GPS_EN_PIN2, HIGH);   // meshtastic.org wiring: gate on GPIO46
  #endif
  g_gpsOn = true;
  gpsOpen(saved >= 0 ? saved : GPS_RX_PIN);
  Serial.printf("GPS: UART1 rx GPIO%d at %lu baud%s%s\n", g_rxPin, (unsigned long)GPS_BAUDS[g_baudIdx], saved >= 0 ? " (pin found by an earlier scan)" : "", g_originSet ? ", map origin set" : "");
}

void gpsLoop() {
  if (!g_gpsOn) return;
  int guard = 512;   // bounded work per loop()
  while (guard-- && GpsSerial.available()) {
    if (g_nmea.feed((char)GpsSerial.read())) { if (g_nmea.fix().valid) g_gpsFixMs = millis(); }
  }
  if (g_scanIdx >= 0 && g_rxPin >= 0 && g_lvlN < 2000) {   // electrical look at the candidate (the UART input is readable)
    const uint8_t l = (uint8_t)digitalRead(g_rxPin);
    g_lvlN++; if (l) g_lvlHigh++; if (g_lvlLast != 2 && l != g_lvlLast) g_lvlEdges++; g_lvlLast = l;
  }
  const bool fresh = g_nmea.sentences() > g_sentencesAtOpen;   // sentences on the pin in use now
  if (fresh) g_gpsLastSentenceMs = millis();
  if (g_scanIdx >= 0) {
    if (fresh) {   // found: keep it
      Serial.printf("GPS: module TX found on GPIO%d at %lu baud\n", g_rxPin, (unsigned long)GPS_BAUDS[g_baudIdx]);
      Preferences p; p.begin("gps", false); p.putChar("rx", (char)g_rxPin); p.putUChar("baud", g_baudIdx); p.end();
      g_scanIdx = -1;
    } else if (millis() - g_gpsStartMs > GPS_SCAN_MS) gpsScanStep();
  } else if (!fresh && g_gpsLastSentenceMs == 0 && millis() - g_gpsStartMs > GPS_SWAP_MS) {
    gpsScanStart();   // nothing on the configured / saved pin since boot: look for it
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
  s.rx_pin = g_viaUart0 ? 44 : g_rxPin; s.scanning = g_scanIdx >= 0; s.swapped = g_viaUart0 || g_rxPin != GPS_RX_PIN; s.baud = GPS_BAUDS[g_baudIdx];
  s.levels = g_levels;
  s.origin_set = g_originSet; s.lat0_e7 = g_lat0; s.lon0_e7 = g_lon0;
  s.hh = f.hh; s.mm = f.mm;
  return true;
}

// A line that arrived on the USB serial port (UART0, GPIO44 = the header pin marked RX): the module's TX wired there
// still works, the sentences just come in through Serial instead of UART1. The scan is stopped.
void gpsFeedSerialLine(const char* line) {
  if (!g_gpsOn || line[0] != '$') return;
  bool upd = false;
  while (*line) upd |= g_nmea.feed(*line++);
  g_nmea.feed('\n');
  if (upd && g_nmea.fix().valid) g_gpsFixMs = millis();
  g_gpsLastSentenceMs = millis();
  if (!g_viaUart0) { g_viaUart0 = true; g_scanIdx = -1; Serial.println(F("GPS: sentences arrive on the USB serial (module TX on the RX pin, GPIO44): fine, using them")); }
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
void gpsScanStart() {}
void gpsFeedSerialLine(const char*) {}
bool gpsStatus(GpsStatus& s) { memset(&s, 0, sizeof(s)); return false; }
bool gpsSetOrigin() { return false; }
bool gpsPinHole(uint8_t) { return false; }
#endif
