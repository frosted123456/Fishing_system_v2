// D80 range test (chalet): one hub's slot cycles through every LoRa mode (SF9 -> SF5, one per frame) while the
// chalet box is carried around; each slot result is logged with the box's GPS fix and its distance to the hub.
// No Arduino here: the sample ring, the per-mode statistics and the CSV row are host-tested (test/test_range_test).
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "radio_modes.h"

namespace icemesh {
namespace tdma {

struct RangeSample {
  uint16_t frame;
  uint8_t mode;          // RadioMode of the slot
  bool ok, crc;          // packet received / something received but CRC failed
  int8_t rssi, snr_q4;   // valid when ok
};

// Single producer (radio task) / single consumer (loop); the caller holds its lock around push / pop.
template <uint8_t CAP = 64>
class RangeRing {
 public:
  bool push(const RangeSample& s) { const uint8_t n = static_cast<uint8_t>((head_ + 1) % CAP); if (n == tail_) { lost++; return false; } r_[head_] = s; head_ = n; return true; }
  bool pop(RangeSample& s) { if (tail_ == head_) return false; s = r_[tail_]; tail_ = static_cast<uint8_t>((tail_ + 1) % CAP); return true; }
  void clear() { head_ = tail_ = 0; lost = 0; }
  uint32_t lost = 0;
 private:
  RangeSample r_[CAP];
  uint8_t head_ = 0, tail_ = 0;
};

struct RangeModeStats {
  uint32_t sent, ok, crc;
  int8_t rssi_min, rssi_max; int32_t rssi_sum;
  int8_t snr_min_q4; int32_t snr_sum_q4;
  float max_ok_m;        // farthest distance with a received packet (-1 = none / no GPS)
  float last_ok_m;
};

class RangeStats {
 public:
  RangeModeStats m[MODE_COUNT];
  RangeStats() { clear(); }
  void clear() { memset(m, 0, sizeof(m)); for (uint8_t i = 0; i < MODE_COUNT; i++) { m[i].max_ok_m = -1; m[i].last_ok_m = -1; } }
  // dist_m < 0 = unknown (no fix / hub not pinned)
  void add(const RangeSample& s, float dist_m) {
    if (!validMode(s.mode)) return;
    RangeModeStats& x = m[s.mode];
    x.sent++;
    if (s.crc) x.crc++;
    if (!s.ok) return;
    if (x.ok == 0) { x.rssi_min = x.rssi_max = s.rssi; x.snr_min_q4 = s.snr_q4; }
    x.ok++;
    if (s.rssi < x.rssi_min) x.rssi_min = s.rssi;
    if (s.rssi > x.rssi_max) x.rssi_max = s.rssi;
    if (s.snr_q4 < x.snr_min_q4) x.snr_min_q4 = s.snr_q4;
    x.rssi_sum += s.rssi; x.snr_sum_q4 += s.snr_q4;
    if (dist_m >= 0) { x.last_ok_m = dist_m; if (dist_m > x.max_ok_m) x.max_ok_m = dist_m; }
  }
};

struct RangeFix { bool fix; int32_t lat_e7, lon_e7; uint8_t sats; uint16_t hdop_x10; uint8_t hh, mm, ss; };
struct RangeHubSide { bool valid; int8_t beacon_rssi, beacon_snr_q4; uint8_t beacon_lost64; };   // from the hub's health

inline const char* rangeCsvHeader() {
  return "t_s,utc,lat,lon,sats,hdop,dist_m,frame,mode,ok,crc,rssi_dbm,snr_db,hub_beacon_rssi,hub_beacon_snr,hub_beacon_lost64\n";
}
// One CSV line (with '\n'). Empty fields where a value is unknown. Returns the length (0 if it did not fit).
inline size_t formatRangeRow(char* out, size_t cap, uint32_t t_ms, const RangeFix& f, float dist_m,
                             const RangeSample& s, const RangeHubSide& hs) {
  char lat[16] = "", lon[16] = "", sats[6] = "", hdop[12] = "", utc[16] = "", dist[16] = "", rssi[8] = "", snr[12] = "";
  char brs[8] = "", bsn[12] = "", blo[6] = "";
  if (f.fix) {
    const int32_t la = f.lat_e7, lo = f.lon_e7;
    snprintf(lat, sizeof(lat), "%s%ld.%07ld", la < 0 ? "-" : "", (long)((la < 0 ? -la : la) / 10000000L), (long)((la < 0 ? -la : la) % 10000000L));
    snprintf(lon, sizeof(lon), "%s%ld.%07ld", lo < 0 ? "-" : "", (long)((lo < 0 ? -lo : lo) / 10000000L), (long)((lo < 0 ? -lo : lo) % 10000000L));
    snprintf(sats, sizeof(sats), "%u", f.sats);
    snprintf(hdop, sizeof(hdop), "%u.%u", f.hdop_x10 / 10, f.hdop_x10 % 10);
    snprintf(utc, sizeof(utc), "%02u:%02u:%02u", f.hh % 24u, f.mm % 60u, f.ss % 60u);
  }
  if (dist_m >= 0) snprintf(dist, sizeof(dist), "%.1f", static_cast<double>(dist_m));
  if (s.ok) { snprintf(rssi, sizeof(rssi), "%d", s.rssi); snprintf(snr, sizeof(snr), "%.2f", s.snr_q4 / 4.0); }
  if (hs.valid) { snprintf(brs, sizeof(brs), "%d", hs.beacon_rssi); snprintf(bsn, sizeof(bsn), "%.2f", hs.beacon_snr_q4 / 4.0); snprintf(blo, sizeof(blo), "%u", hs.beacon_lost64); }
  const int n = snprintf(out, cap, "%lu.%03lu,%s,%s,%s,%s,%s,%s,%u,SF%u,%u,%u,%s,%s,%s,%s,%s\n",
                         (unsigned long)(t_ms / 1000), (unsigned long)(t_ms % 1000), utc, lat, lon, sats, hdop, dist,
                         s.frame, validMode(s.mode) ? modeInfo(static_cast<RadioMode>(s.mode)).sf : 0, s.ok ? 1 : 0, s.crc ? 1 : 0,
                         rssi, snr, brs, bsn, blo);
  return (n > 0 && static_cast<size_t>(n) < cap) ? static_cast<size_t>(n) : 0;
}

}  // namespace tdma
}  // namespace icemesh
