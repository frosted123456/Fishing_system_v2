// MSB-first bit writer / reader for the sonar codec. Bounds-checked: overflow sets a flag, never writes past cap.
#ifndef ICEMESH_BITSTREAM_H
#define ICEMESH_BITSTREAM_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace icemesh {

class BitWriter {
 public:
  BitWriter(uint8_t* buf, size_t cap_bytes) : b_(buf), cap_bits_(cap_bytes * 8), pos_(0), over_(false) {
    if (cap_bytes) memset(buf, 0, cap_bytes);
  }
  void put(uint32_t v, uint8_t bits) {
    for (int8_t i = static_cast<int8_t>(bits - 1); i >= 0; i--) putBit((v >> i) & 1u);
  }
  void putBit(uint32_t bit) {
    if (pos_ >= cap_bits_) { over_ = true; return; }
    if (bit) b_[pos_ >> 3] = static_cast<uint8_t>(b_[pos_ >> 3] | (0x80u >> (pos_ & 7)));
    pos_++;
  }
  void putSigned(int32_t v, uint8_t bits) { put(static_cast<uint32_t>(v) & ((1u << bits) - 1u), bits); }
  // Rice code: quotient in unary (q ones, one zero), then k remainder bits.
  void putRice(uint32_t v, uint8_t k) {
    uint32_t q = v >> k;
    while (q--) putBit(1);
    putBit(0);
    put(v & ((1u << k) - 1u), k);
  }
  // Overwrite `bits` bits at an earlier position (used to patch counts).
  void patch(size_t at_bit, uint32_t v, uint8_t bits) {
    for (uint8_t i = 0; i < bits; i++) {
      const size_t p = at_bit + i;
      if (p >= cap_bits_) { over_ = true; return; }
      const uint8_t mask = static_cast<uint8_t>(0x80u >> (p & 7));
      if ((v >> (bits - 1 - i)) & 1u) b_[p >> 3] = static_cast<uint8_t>(b_[p >> 3] | mask);
      else b_[p >> 3] = static_cast<uint8_t>(b_[p >> 3] & ~mask);
    }
  }
  // Roll back to an earlier bit position (clears the bits after it).
  void rewind(size_t to_bit) {
    if (to_bit > pos_) return;
    for (size_t p = to_bit; p < pos_ && p < cap_bits_; p++) b_[p >> 3] = static_cast<uint8_t>(b_[p >> 3] & ~(0x80u >> (p & 7)));
    pos_ = to_bit; over_ = false;
  }
  size_t bits() const { return pos_; }
  size_t bytes() const { return (pos_ + 7) / 8; }
  bool overflow() const { return over_; }

 private:
  uint8_t* b_;
  size_t cap_bits_;
  size_t pos_;
  bool over_;
};

class BitReader {
 public:
  BitReader(const uint8_t* buf, size_t len_bytes) : b_(buf), n_bits_(len_bytes * 8), pos_(0), bad_(false) {}
  uint32_t get(uint8_t bits) {
    uint32_t v = 0;
    for (uint8_t i = 0; i < bits; i++) v = (v << 1) | getBit();
    return v;
  }
  uint32_t getBit() {
    if (pos_ >= n_bits_) { bad_ = true; return 0; }
    const uint32_t bit = (b_[pos_ >> 3] >> (7 - (pos_ & 7))) & 1u;
    pos_++;
    return bit;
  }
  int32_t getSigned(uint8_t bits) {
    const uint32_t v = get(bits);
    const uint32_t sign = 1u << (bits - 1);
    return (v & sign) ? static_cast<int32_t>(v) - static_cast<int32_t>(1u << bits) : static_cast<int32_t>(v);
  }
  uint32_t getRice(uint8_t k, uint32_t max_q = 64) {
    uint32_t q = 0;
    while (getBit()) { if (++q > max_q || bad_) { bad_ = true; return 0; } }
    return (q << k) | get(k);
  }
  bool bad() const { return bad_; }
  size_t bitsLeft() const { return pos_ < n_bits_ ? n_bits_ - pos_ : 0; }

 private:
  const uint8_t* b_;
  size_t n_bits_;
  size_t pos_;
  bool bad_;
};

}  // namespace icemesh
#endif
