// LoRa time-on-air (Semtech SX126x formula) and an airtime token bucket.
#ifndef ICEMESH_AIRTIME_H
#define ICEMESH_AIRTIME_H
#include <stdint.h>

namespace icemesh {

struct LoRaPhy {
  uint8_t sf;            // 5..12
  uint32_t bw_hz;        // e.g. 125000
  uint8_t cr_denom;      // 5..8  (coding rate 4/5 .. 4/8)
  uint16_t preamble;     // symbols
  bool crc;
  bool explicit_header;
};

// Project settings (src/lora_node/config.h): 915 MHz, SF9, 125 kHz, 4/5, preamble 8, CRC on.
inline LoRaPhy phySF(uint8_t sf) {
  LoRaPhy p;
  p.sf = sf; p.bw_hz = 125000; p.cr_denom = 5; p.preamble = 8; p.crc = true; p.explicit_header = true;
  return p;
}

// Time on air in microseconds. Low-data-rate optimisation is assumed on when a symbol
// lasts >= 16 ms (what RadioLib does automatically).
inline uint32_t airtimeUs(uint16_t payload_len, const LoRaPhy& p) {
  const uint64_t tsym_ns = (static_cast<uint64_t>(1) << p.sf) * 1000000000ULL / p.bw_hz;
  const int de = (tsym_ns >= 16000000ULL) ? 1 : 0;
  const int ih = p.explicit_header ? 0 : 1;
  const int num = 8 * payload_len - 4 * p.sf + 28 + 16 * (p.crc ? 1 : 0) - 20 * ih;
  const int den = 4 * (p.sf - 2 * de);
  int blocks = (num > 0) ? (num + den - 1) / den : 0;
  const uint64_t n_payload = 8 + static_cast<uint64_t>(blocks) * p.cr_denom;
  // preamble + 4.25 symbols  ==  (4 * preamble + 17) / 4 symbols
  const uint64_t total_ns = (static_cast<uint64_t>(4 * p.preamble + 17) * tsym_ns) / 4 + n_payload * tsym_ns;
  return static_cast<uint32_t>(total_ns / 1000ULL);
}

// Token bucket limiting a traffic class to `permille` of the airtime.
// Tokens are microseconds of airtime. Sending is allowed while the balance is positive and
// may push it negative, so a packet longer than the burst is never locked out; the long-run
// share is still bounded by permille.
class AirtimeBudget {
 public:
  AirtimeBudget(uint16_t permille = 200, uint32_t burst_ms = 4000) { configure(permille, burst_ms, 0); }

  void configure(uint16_t permille, uint32_t burst_ms, uint32_t now_ms) {
    permille_ = permille;
    capacity_us_ = static_cast<int64_t>(burst_ms) * permille;
    tokens_us_ = capacity_us_;
    last_ms_ = now_ms;
    spent_us_ = 0;
  }

  bool canSend(uint32_t now_ms) { refill(now_ms); return tokens_us_ > 0; }
  void spend(uint32_t airtime_us, uint32_t now_ms) {
    refill(now_ms);
    tokens_us_ -= airtime_us;
    spent_us_ += airtime_us;
  }
  uint64_t totalSpentUs() const { return spent_us_; }
  uint16_t permille() const { return permille_; }

 private:
  void refill(uint32_t now_ms) {
    const uint32_t elapsed = now_ms - last_ms_;
    last_ms_ = now_ms;
    tokens_us_ += static_cast<int64_t>(elapsed) * permille_;   // ms * permille = µs
    if (tokens_us_ > capacity_us_) tokens_us_ = capacity_us_;
  }

  uint16_t permille_;
  int64_t capacity_us_;
  int64_t tokens_us_;
  uint32_t last_ms_;
  uint64_t spent_us_;
};

}  // namespace icemesh
#endif
