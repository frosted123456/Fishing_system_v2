// Minimal NMEA 0183 parser for the chalet's GPS (D73): GGA for the fix (quality, satellites, HDOP, position),
// RMC for the date / time. No Arduino, no allocation: feed it bytes, read the last fix. Talker ID ignored
// ($GPGGA, $GNGGA, $BDGGA ... all accepted). Checksum verified when present. Positions in 1e-7 degrees
// (int32: +- 214 deg, 1.1 cm steps) so the maths downstream is integer. Host-tested (test/test_nmea).
#pragma once
#include <stdint.h>
#include <string.h>

namespace icemesh {

struct GpsFix {
  bool valid;          // GGA quality > 0
  uint8_t quality;     // 0 none, 1 GPS, 2 DGPS, 6 estimated ...
  uint8_t sats;
  uint16_t hdop_x10;   // HDOP x 10 (12 = 1.2)
  int32_t lat_e7, lon_e7;
  int32_t alt_dm;      // altitude, decimetres (0 when absent)
  uint8_t hh, mm, ss;  // UTC from the last GGA / RMC
};

class NmeaParser {
 public:
  enum { MAX_LINE = 100 };
  NmeaParser() { reset(); }
  void reset() { n_ = 0; in_ = false; sentences_ = 0; bad_ = 0; ggas_ = 0; memset(&fix_, 0, sizeof(fix_)); }

  // Returns true when a GGA sentence updated the fix.
  bool feed(char c) {
    if (c == '$') { in_ = true; n_ = 0; return false; }
    if (!in_) return false;
    if (c == '\r' || c == '\n') {
      in_ = false;
      if (n_ == 0) return false;
      line_[n_] = 0;
      return parseLine();
    }
    if (n_ < MAX_LINE - 1) line_[n_++] = c; else in_ = false;   // overlong: drop it
    return false;
  }
  const GpsFix& fix() const { return fix_; }
  uint32_t sentences() const { return sentences_; }   // valid sentences of any kind (wiring proof)
  uint32_t ggas() const { return ggas_; }
  uint32_t bad() const { return bad_; }               // checksum failures

 private:
  static int hexv(char c) { return c >= '0' && c <= '9' ? c - '0' : (c >= 'A' && c <= 'F' ? c - 'A' + 10 : (c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1)); }
  // field i (0 = sentence type) as a pointer into a NUL-terminated copy; returns nullptr past the end
  const char* field(uint8_t i) const {
    const char* p = body_;
    for (uint8_t k = 0; k < i; k++) { p = strchr(p, ','); if (p == nullptr) return nullptr; p++; }
    return p;
  }
  // "0.9" / "12.75" / "545" within a field (stops at ',' or end) -> value x 10, one decimal kept
  static uint32_t decimalX10(const char* f) {
    if (f == nullptr || *f == ',' || *f == 0) return 0;
    uint32_t ip = 0; const char* p = f;
    while (*p >= '0' && *p <= '9') { ip = ip * 10 + (uint32_t)(*p - '0'); p++; }
    uint32_t fr = 0;
    if (*p == '.' && p[1] >= '0' && p[1] <= '9') fr = (uint32_t)(p[1] - '0');
    return ip * 10 + fr;
  }
  static uint32_t num(const char* p, uint8_t digits) { uint32_t v = 0; for (uint8_t i = 0; i < digits && p[i] >= '0' && p[i] <= '9'; i++) v = v * 10 + (uint32_t)(p[i] - '0'); return v; }
  // "4916.4521" / "N" -> 1e-7 degrees; "12311.1200" / "W" likewise (3 degree digits)
  static bool coord(const char* f, const char* hemi, uint8_t deg_digits, int32_t& out) {
    if (f == nullptr || hemi == nullptr || *f == ',' || *f == 0) return false;
    const uint32_t deg = num(f, deg_digits);
    const char* m = f + deg_digits;
    const uint32_t mi = num(m, 2);
    const char* dot = m + 2;
    uint32_t frac = 0; uint8_t fd = 0;
    if (*dot == '.') { dot++; while (fd < 7 && dot[fd] >= '0' && dot[fd] <= '9') { frac = frac * 10 + (uint32_t)(dot[fd] - '0'); fd++; } }
    while (fd < 7) { frac *= 10; fd++; }                       // minutes fraction in 1e-7 minutes
    // degrees * 1e7 + (minutes + frac/1e7) * 1e7 / 60
    const uint64_t min_e7 = (uint64_t)mi * 10000000ULL + frac;
    int64_t v = (int64_t)deg * 10000000LL + (int64_t)(min_e7 / 60ULL);
    if (*hemi == 'S' || *hemi == 'W') v = -v;
    out = (int32_t)v;
    return true;
  }
  bool parseLine() {
    // checksum: "...*HH"
    char* star = strchr(line_, '*');
    if (star != nullptr) {
      uint8_t cs = 0;
      for (char* p = line_; p < star; p++) cs ^= (uint8_t)*p;
      const int h = hexv(star[1]), l = hexv(star[2]);
      if (h < 0 || l < 0 || cs != (uint8_t)((h << 4) | l)) { bad_++; return false; }
      *star = 0;
    }
    sentences_++;
    memcpy(body_, line_, sizeof(body_)); body_[sizeof(body_) - 1] = 0;
    const char* t = field(0);
    if (t == nullptr || strlen(t) < 5) return false;
    const char* type = t + 2;   // skip the talker ID
    if (strncmp(type, "GGA", 3) == 0) {
      ggas_++;
      const char* f1 = field(1);   // hhmmss.ss
      if (f1 != nullptr && *f1 != ',') { fix_.hh = (uint8_t)num(f1, 2); fix_.mm = (uint8_t)num(f1 + 2, 2); fix_.ss = (uint8_t)num(f1 + 4, 2); }
      const char* q = field(6);
      fix_.quality = q != nullptr ? (uint8_t)num(q, 1) : 0;
      fix_.valid = fix_.quality > 0;
      const char* s = field(7); fix_.sats = s != nullptr ? (uint8_t)num(s, 2) : 0;
      fix_.hdop_x10 = decimalX10(field(8));
      int32_t la, lo;
      if (fix_.valid && coord(field(2), field(3), 2, la) && coord(field(4), field(5), 3, lo)) { fix_.lat_e7 = la; fix_.lon_e7 = lo; }
      else fix_.valid = false;
      { const char* a = field(9); const bool neg = a != nullptr && *a == '-'; const int32_t v = (int32_t)decimalX10(neg ? a + 1 : a); fix_.alt_dm = neg ? -v : v; }
      return true;
    }
    if (strncmp(type, "RMC", 3) == 0) {
      const char* f1 = field(1);
      if (f1 != nullptr && *f1 != ',') { fix_.hh = (uint8_t)num(f1, 2); fix_.mm = (uint8_t)num(f1 + 2, 2); fix_.ss = (uint8_t)num(f1 + 4, 2); }
    }
    return false;
  }

  char line_[MAX_LINE], body_[MAX_LINE];
  uint8_t n_;
  bool in_;
  uint32_t sentences_, bad_, ggas_;
  GpsFix fix_;
};

// Metres east / north of an origin, for the pocket map (flat earth, fine for a lake): 1e-7 deg -> dm.
inline void gpsOffsetDm(int32_t lat0_e7, int32_t lon0_e7, int32_t lat_e7, int32_t lon_e7, int32_t& x_dm, int32_t& y_dm) {
  // 1 deg lat = 111 320 m -> 1e-7 deg = 0.011132 m = 0.11132 dm; lon scaled by cos(lat)
  const double lat0 = lat0_e7 * 1e-7 * 3.14159265358979 / 180.0;
  double c = 1.0, t = lat0 * lat0;   // cos by series, 4 terms (|lat| <= 90 deg): error < 3e-4 at 60 deg
  c = 1.0 - t / 2.0 + t * t / 24.0 - t * t * t / 720.0 + t * t * t * t / 40320.0;
  const double dy = (double)(lat_e7 - lat0_e7) * 0.11132;
  const double dx = (double)(lon_e7 - lon0_e7) * 0.11132 * c;
  x_dm = (int32_t)(dx < 0 ? dx - 0.5 : dx + 0.5);
  y_dm = (int32_t)(dy < 0 ? dy - 0.5 : dy + 0.5);
}

}  // namespace icemesh
