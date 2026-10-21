// Superframe timing shared by the chalet and every hub. All times are µs relative to the
// frame reference REF = START of the chalet beacon transmission. Nobody measures a start directly:
// the chalet takes TxDone - airtime(beacon), a hub RxDone - airtime(beacon) (same formula on both
// sides), so REF is exactly periodic whatever the beacon length.
//
//   REF [beacon airtime] BEACON_GAP | slot 0: LEAD [tx airtime(allowance)] TAIL | slot 1 ... | margin | next REF
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

inline uint32_t beaconAirtimeUs(uint16_t beacon_len) { return modeAirtimeUs(MODE_SF9_BW500, beacon_len); }

// D79 (E1): a hub listens for the next beacon until a beacon up to BEACON_GROW_MAX bytes longer than the last one would
// have ended (+ BEACON_WINDOW). A beacon that grew by more than the window (9+ new acks, or 5 new slots, in one frame)
// was missed by every hub at once. 36 B = 12 acks (the test holes of 3 hubs going OFFLINE at once); 36 B = 41 ms of extra listening, inside the JOIN slot + gaps (43 ms) that follow the beacon.
static const uint16_t BEACON_GROW_MAX = 36;
inline uint32_t beaconListenCloseUs(uint32_t ref_next, uint16_t last_len) {
  const uint16_t max_len = beaconSize(MAX_SLOTS, MAX_ACKS);
  const uint16_t len = static_cast<uint16_t>(last_len + BEACON_GROW_MAX > max_len ? max_len : last_len + BEACON_GROW_MAX);
  return ref_next + beaconAirtimeUs(len) + BEACON_WINDOW_US;
}

// REF from the end of a received/transmitted beacon of `len` bytes.
inline uint32_t refFromBeaconEnd(uint32_t end_us, uint16_t len) { return end_us - beaconAirtimeUs(len); }

// Fills out[i] for each slot; returns the end of the last slot (µs after REF).
inline uint32_t computeSlotTimes(const Slot* slots, uint8_t n, SlotTime* out, uint16_t beacon_len) {
  uint32_t t = beaconAirtimeUs(beacon_len) + BEACON_GAP_US;
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

// Does the plan fit in the frame? (beacon + slots + margin before the next REF)
inline bool planFits(const Slot* slots, uint8_t n, uint32_t frame_us, uint16_t beacon_len) {
  SlotTime tmp[MAX_SLOTS];
  const uint32_t end = computeSlotTimes(slots, n, tmp, beacon_len);
  return end + FRAME_MARGIN_US <= frame_us;
}

}  // namespace tdma
}  // namespace icemesh
#endif
