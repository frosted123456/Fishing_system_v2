// Serial-number arithmetic for 16-bit sequence numbers (wraps 65535 -> 0).
#ifndef ICEMESH_SEQ_H
#define ICEMESH_SEQ_H
#include <stdint.h>

namespace icemesh {

// Signed distance a - b in the range [-32768, 32767]. > 0 means "a is newer than b".
// (uint16 -> int16 conversion is modulo 2^16 on GCC, the only compiler used here.)
inline int16_t seqDiff(uint16_t a, uint16_t b) {
  return static_cast<int16_t>(static_cast<uint16_t>(a - b));
}

inline bool seqNewer(uint16_t a, uint16_t b) { return seqDiff(a, b) > 0; }

// Wrap-safe "now has reached t" for millis()-style 32-bit timestamps.
inline bool timeReached(uint32_t now, uint32_t t) {
  return static_cast<int32_t>(now - t) >= 0;
}

}  // namespace icemesh
#endif
