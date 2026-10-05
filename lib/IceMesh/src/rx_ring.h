// Single-producer / single-consumer ring buffer for received frames.
// Producer = ESP-NOW receive callback (Wi-Fi task, core 0); consumer = loop() (core 1).
// The callback only copies the frame and returns; all processing happens in loop().
#ifndef ICEMESH_RX_RING_H
#define ICEMESH_RX_RING_H
#include <stdint.h>
#include <string.h>
#include <atomic>

namespace icemesh {

template <uint8_t SLOTS = 16, uint16_t MAXLEN = 250>
class RxRing {
 public:
  struct Frame {
    uint8_t mac[6];
    int8_t rssi;
    uint16_t len;
    uint8_t data[MAXLEN];
  };

  RxRing() : head_(0), tail_(0), dropped_(0) {}

  // Producer side. Returns false (and counts a drop) when full or the frame is too long.
  bool push(const uint8_t* mac, const uint8_t* data, int len, int8_t rssi) {
    if (data == nullptr || len <= 0 || len > MAXLEN) { dropped_.fetch_add(1); return false; }
    const uint8_t h = head_.load(std::memory_order_relaxed);
    const uint8_t next = static_cast<uint8_t>((h + 1) % SLOTS);
    if (next == tail_.load(std::memory_order_acquire)) { dropped_.fetch_add(1); return false; }
    Frame& f = f_[h];
    if (mac != nullptr) memcpy(f.mac, mac, 6); else memset(f.mac, 0, 6);
    f.rssi = rssi;
    f.len = static_cast<uint16_t>(len);
    memcpy(f.data, data, static_cast<size_t>(len));
    head_.store(next, std::memory_order_release);
    return true;
  }

  // Consumer side.
  bool pop(Frame& out) {
    const uint8_t t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire)) return false;
    const Frame& f = f_[t];
    memcpy(out.mac, f.mac, 6);
    out.rssi = f.rssi;
    out.len = f.len;
    memcpy(out.data, f.data, f.len);
    tail_.store(static_cast<uint8_t>((t + 1) % SLOTS), std::memory_order_release);
    return true;
  }

  uint32_t dropped() const { return dropped_.load(); }
  bool empty() const { return tail_.load() == head_.load(); }

 private:
  Frame f_[SLOTS];
  std::atomic<uint8_t> head_;
  std::atomic<uint8_t> tail_;
  std::atomic<uint32_t> dropped_;
};

}  // namespace icemesh
#endif
