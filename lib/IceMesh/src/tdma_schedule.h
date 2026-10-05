// Superframe timing shared by the chalet and every hub. All times are µs relative to the
// frame reference REF = end of the chalet beacon transmission (TxDone at the chalet,
// RxDone at a hub). Both sides compute the same offsets from the beacon's slot list.
//
//   REF | BEACON_GAP | slot 0: LEAD  [tx airtime(allowance)]  TAIL | slot 1 ... | margin | next beacon
//
// All the timing constants below are ESTIMATES to be confirmed with the radio test mode
// (hubs report their measured beacon timing error in the health record).
#ifndef ICEMESH_TDMA_SCHEDULE_H
#define ICEMESH_TDMA_SCHEDULE_H
#include <stdint.h>
#include "radio_modes.h"
#include "tdma_proto.h"

namespace icemesh {
namespace tdma {

static const uint32_t BEACON_GAP_US = 6000;    // hubs decode the beacon and switch mode (est.)
static const uint32_t LEAD_US = 3000;          // receivers listening before TX starts (est.)
static const uint32_t TAIL_US = 3000;          // timing error / RxDone latency margin (est.)
static const uint32_t FRAME_MARGIN_US = 40000; // free time before the next beacon (est.)
static const uint32_t BEACON_WINDOW_US = 25000;// a hub listens this long before/after the expected beacon (est.)

struct SlotTime {
  uint32_t start;   // receivers start listening
  uint32_t tx;      // owner starts transmitting
  uint32_t end;     // receivers stop listening
};

inline RadioMode slotMode(const Slot& s) { return validMode(s.mode) ? static_cast<RadioMode>(s.mode) : MODE_SF9_BW500; }

inline uint32_t slotDurationUs(const Slot& s) {
  return LEAD_US + modeAirtimeUs(slotMode(s), s.allowance) + TAIL_US;
}

// Fills out[i] for each slot; returns the end of the last slot (µs after REF).
inline uint32_t computeSlotTimes(const Slot* slots, uint8_t n, SlotTime* out) {
  uint32_t t = BEACON_GAP_US;
  for (uint8_t i = 0; i < n; i++) {
    out[i].start = t;
    out[i].tx = t + LEAD_US;
    t += slotDurationUs(slots[i]);
    out[i].end = t;
  }
  return t;
}

// A receiver that got a packet of `len` bytes ending at `rx_end_us`, sent at the start of slot
// `st` in `mode`, recovers the frame reference. Used by hubs that only hear an ECHO.
inline uint32_t refFromSlotPacket(uint32_t rx_end_us, const SlotTime& st, RadioMode mode, uint16_t len) {
  return rx_end_us - (st.tx + modeAirtimeUs(mode, len));
}

// Does the plan fit in the frame? (slots after REF + margin must end before the next beacon starts)
inline bool planFits(const Slot* slots, uint8_t n, uint32_t frame_us, uint32_t beacon_airtime_us) {
  SlotTime tmp[MAX_SLOTS];
  const uint32_t end = computeSlotTimes(slots, n, tmp);
  return end + FRAME_MARGIN_US + beacon_airtime_us <= frame_us;
}

}  // namespace tdma
}  // namespace icemesh
#endif
