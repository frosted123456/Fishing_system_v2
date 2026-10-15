// Hub side: superframe synchronisation on the chalet beacon (or a relay hub's ECHO).
// Times are 32-bit µs (esp_timer truncated; wraps every ~71 min, handled with unsigned math).
#ifndef ICEMESH_HUB_SYNC_H
#define ICEMESH_HUB_SYNC_H
#include <stdint.h>

namespace icemesh {
namespace tdma {

class HubSync {
 public:
  enum { MAX_FREERUN = 3 };   // frames without beacon before going back to search (est.)

  HubSync() { reset(); }
  void reset() {
    synced_ = false; ref_ = 0; frame_us_ = 1000000; frame_ = 0;
    lost_streak_ = 0; lost_bits_ = 0; last_err_us_ = 0; have_err_ = false;
  }

  // Beacon (or echo) received: `ref_us` = recovered frame reference.
  void onBeacon(uint32_t ref_us, uint16_t frame, uint32_t frame_us) {
    if (synced_) {
      // predicted reference for this frame number
      const int16_t df = static_cast<int16_t>(static_cast<uint16_t>(frame - frame_));
      if (df > 0 && df <= MAX_FREERUN + 1) {
        const uint32_t predicted = ref_ + static_cast<uint32_t>(df) * frame_us_;
        last_err_us_ = static_cast<int32_t>(ref_us - predicted);
        have_err_ = true;
      }
    }
    ref_ = ref_us; frame_ = frame; frame_us_ = frame_us;
    synced_ = true; lost_streak_ = 0;
    lost_bits_ <<= 1;
  }

  // No beacon in the window: free-run one frame on the last timing.
  void onMissedBeacon() {
    if (!synced_) return;
    ref_ += frame_us_;
    frame_++;
    lost_streak_++;
    lost_bits_ = (lost_bits_ << 1) | 1;
    if (lost_streak_ > MAX_FREERUN) synced_ = false;
  }

  bool synced() const { return synced_; }
  // Transmit only on a fresh beacon or after a single miss: the slot plan may have changed.
  bool mayTransmit() const { return synced_ && lost_streak_ <= 1; }
  uint32_t ref() const { return ref_; }
  uint16_t frame() const { return frame_; }
  uint32_t frameUs() const { return frame_us_; }
  uint32_t nextBeaconEnd() const { return ref_ + frame_us_; }
  int32_t lastErrUs() const { return have_err_ ? last_err_us_ : 0; }
  uint8_t lostLast64() const {
    uint8_t n = 0; uint64_t b = lost_bits_;
    while (b) { n += static_cast<uint8_t>(b & 1); b >>= 1; }
    return n;
  }

 private:
  bool synced_;
  uint32_t ref_;
  uint32_t frame_us_;
  uint16_t frame_;
  uint8_t lost_streak_;
  uint64_t lost_bits_;
  int32_t last_err_us_;
  bool have_err_;
};

}  // namespace tdma
}  // namespace icemesh
#endif
