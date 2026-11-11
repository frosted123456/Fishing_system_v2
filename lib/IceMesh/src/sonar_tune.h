// D141 per-hole transducer tune (pure logic, host-tested: test/test_sonar_tune). Every transducer has its own resonance
// (TI: +-2 % typical between parts) and it moves with the water temperature (survey transducers: 0.2-2.8 % between 1 and
// 18 degC), while our two-way -3 dB band is only ~6 kHz wide (D113). One shared `ftrim` knob cannot fit nine transducers on
// the ice, so each hole finds its own burst frequency and band-pass code, on request from the chalet, once it is in the water.
//
// The caller pings at the frequency / band-pass this class asks for and reports each ping's bottom SNR:
//   step 0        the current setting (reference)
//   steps 1-11    190-210 kHz in 2 kHz steps (ftrim 10..30) at the current band-pass
//   steps 12-13   best coarse frequency -1 / +1 kHz
//   steps 14-16   band-pass 28 / 29 / 30 at the best frequency
// PINGS pings a step, the first SKIP dropped (the processing settles on the new level). 17 x 8 = 136 pings: 34 s at 4/s.
// The new setting is kept only when it beats the reference by MIN_GAIN dB (a re-run on a tuned hole changes nothing).
#ifndef ICEMESH_SONAR_TUNE_H
#define ICEMESH_SONAR_TUNE_H
#include <stdint.h>

namespace icemesh {
namespace sonar {

// 12 bytes, travels in the SelfTestResult slot (messages.h, SEC_SELFTEST) with ver = TUNE_RESULT_VER
enum : uint8_t { TUNE_RESULT_VER = 2 };
enum : uint8_t { TUNE_OK = 0, TUNE_KEPT = 1, TUNE_NO_BOTTOM = 2, TUNE_NO_SONAR = 3, TUNE_CLEARED = 4, TUNE_TIMEOUT = 5 };
// DEVCMD_SELFTEST / CMD_SELFTEST value: 0 = wiring check (D70), 1 = run the tune, 2 = forget this hole's tune
enum : uint8_t { TUNE_CMD_RUN = 1, TUNE_CMD_CLEAR = 2 };
struct TuneResult {
  uint8_t ver;         // TUNE_RESULT_VER
  uint8_t status;      // TUNE_*
  uint8_t ftrim;       // setting now in use: burst frequency = 180 + ftrim kHz
  uint8_t bpf;         // band-pass code now in use
  uint8_t snr_x2;      // bottom SNR with it, 0.5 dB
  int8_t  gain_x4;     // vs the setting before the tune, 0.25 dB (+ = better now)
  uint8_t ref_ftrim;   // the setting before
  uint8_t band_lo;     // -3 dB band of the coarse scan, ftrim units (kHz - 180)
  uint8_t band_hi;
  uint8_t ring_cm;     // ring-down length at the end, cm (255 = more)
  uint8_t nf_neg;      // noise floor, -dB
  uint8_t steps_ok;    // steps that had a bottom echo (of Tuner::NSTEP)
};

class Tuner {
 public:
  enum { NCOARSE = 11, NSTEP = 1 + NCOARSE + 2 + 3, PINGS = 8, SKIP = 2, MIN_VALID = 3 };
  static float minGain() { return 0.5f; }     // dB over the reference needed to change the setting
  static float minSnr() { return 6.0f; }      // a step under this bottom SNR does not count as a bottom

  bool active() const { return step_ >= 0; }
  uint8_t ftrim() const { return f_; }        // what to ping with while active()
  uint8_t bpf() const { return b_; }
  int8_t step() const { return step_; }

  void start(uint8_t ref_ftrim, uint8_t ref_bpf) {
    ref_f_ = ref_ftrim > 40 ? 20 : ref_ftrim; ref_b_ = ref_bpf;
    for (uint8_t i = 0; i < NSTEP; i++) snr_[i] = -99.f;
    best_f_ = ref_f_; step_ = 0; pings_ = 0; sum_ = 0.f; n_ = 0; attempts_ = 0;
    apply();
  }
  void stop() { step_ = -1; }

  // A ping that produced nothing (driver error): the caller counts them and gives up after a while
  uint16_t attempts() const { return attempts_; }
  void onAttempt() { if (attempts_ < 65535) attempts_++; }

  // One processed ping at ftrim() / bpf(). Returns true when the tune has finished: read result().
  bool onPing(bool has_bottom, float bottom_snr) {
    if (step_ < 0) return false;
    if (++pings_ > SKIP && has_bottom && bottom_snr >= minSnr()) { sum_ += bottom_snr; n_++; }
    if (pings_ < PINGS) return false;
    snr_[step_] = n_ >= MIN_VALID ? sum_ / static_cast<float>(n_) : -99.f;
    if (step_ == NCOARSE) {            // coarse scan done: its best frequency (the reference competes too)
      float best = snr_[0]; best_f_ = ref_f_;
      for (uint8_t k = 0; k < NCOARSE; k++) if (snr_[1 + k] > best + 0.01f) { best = snr_[1 + k]; best_f_ = static_cast<uint8_t>(10 + 2 * k); }
      coarse_f_ = best_f_;
    } else if (step_ == NCOARSE + 2) { // fine steps done
      float best = stepSnrAt(coarse_f_);
      if (snr_[NCOARSE + 1] > best + 0.01f) { best = snr_[NCOARSE + 1]; best_f_ = static_cast<uint8_t>(coarse_f_ > 0 ? coarse_f_ - 1 : 0); }
      if (snr_[NCOARSE + 2] > best + 0.01f) { best_f_ = static_cast<uint8_t>(coarse_f_ < 40 ? coarse_f_ + 1 : 40); }
    }
    step_++; pings_ = 0; sum_ = 0.f; n_ = 0;
    if (step_ >= NSTEP) { finish(); step_ = -1; return true; }
    apply();
    return false;
  }

  // Valid after onPing() returned true. ring_cm / nf_neg are filled by the caller.
  const TuneResult& result() const { return res_; }
  TuneResult& resultRw() { return res_; }
  bool changed() const { return res_.status == TUNE_OK; }   // store res_.ftrim / res_.bpf for this hole
  float stepSnr(uint8_t i) const { return i < NSTEP ? snr_[i] : -99.f; }

  static TuneResult status(uint8_t st, uint8_t ftrim, uint8_t bpf) {
    TuneResult r; r.ver = TUNE_RESULT_VER; r.status = st; r.ftrim = ftrim; r.bpf = bpf; r.snr_x2 = 0; r.gain_x4 = 0;
    r.ref_ftrim = ftrim; r.band_lo = 0; r.band_hi = 0; r.ring_cm = 0; r.nf_neg = 0; r.steps_ok = 0; return r;
  }

 private:
  float stepSnrAt(uint8_t f) const {   // SNR measured at a coarse frequency (or the reference)
    if (f >= 10 && f <= 30 && ((f - 10) % 2) == 0) return snr_[1 + (f - 10) / 2] > snr_[0] || f != ref_f_ ? snr_[1 + (f - 10) / 2] : snr_[0];
    return f == ref_f_ ? snr_[0] : -99.f;
  }
  void apply() {
    if (step_ == 0) { f_ = ref_f_; b_ = ref_b_; }
    else if (step_ <= NCOARSE) { f_ = static_cast<uint8_t>(10 + 2 * (step_ - 1)); b_ = ref_b_; }
    else if (step_ == NCOARSE + 1) { f_ = static_cast<uint8_t>(coarse_f_ > 0 ? coarse_f_ - 1 : 0); b_ = ref_b_; }
    else if (step_ == NCOARSE + 2) { f_ = static_cast<uint8_t>(coarse_f_ < 40 ? coarse_f_ + 1 : 40); b_ = ref_b_; }
    else { f_ = best_f_; b_ = static_cast<uint8_t>(28 + (step_ - NCOARSE - 3)); }
  }
  static uint8_t u8(float v) { return static_cast<uint8_t>(v < 0.f ? 0.f : (v > 255.f ? 255.f : v + 0.5f)); }
  void finish() {
    uint8_t okn = 0; for (uint8_t i = 0; i < NSTEP; i++) if (snr_[i] > -90.f) okn++;
    float bb = -99.f; uint8_t best_b = ref_b_;
    for (uint8_t k = 0; k < 3; k++) if (snr_[NCOARSE + 3 + k] > bb + 0.01f) { bb = snr_[NCOARSE + 3 + k]; best_b = static_cast<uint8_t>(28 + k); }
    float cb = -99.f; for (uint8_t k = 0; k < NCOARSE; k++) if (snr_[1 + k] > cb) cb = snr_[1 + k];
    int lo = -1, hi = -1; for (uint8_t k = 0; k < NCOARSE; k++) if (snr_[1 + k] > -90.f && snr_[1 + k] >= cb - 3.0f) { if (lo < 0) lo = k; hi = k; }
    res_ = status(TUNE_NO_BOTTOM, ref_f_, ref_b_);
    res_.steps_ok = okn;
    res_.band_lo = static_cast<uint8_t>(lo < 0 ? 0 : 10 + 2 * lo); res_.band_hi = static_cast<uint8_t>(hi < 0 ? 0 : 10 + 2 * hi);
    if (bb < -90.f) { if (snr_[0] > -90.f) res_.snr_x2 = u8(snr_[0] * 2.f); return; }   // no bottom at the best setting: keep the old one
    const float ref = snr_[0];
    if (ref > -90.f && bb - ref < minGain()) {   // not clearly better: nothing changes
      res_.status = TUNE_KEPT; res_.snr_x2 = u8(ref * 2.f);
      const float g = (bb - ref) * 4.f; res_.gain_x4 = static_cast<int8_t>(g < -127.f ? -127.f : (g > 127.f ? 127.f : g));
      return;
    }
    res_.status = TUNE_OK; res_.ftrim = best_f_; res_.bpf = best_b; res_.snr_x2 = u8(bb * 2.f);
    const float g = ref > -90.f ? (bb - ref) * 4.f : 127.f;   // no bottom at all before: "a lot"
    res_.gain_x4 = static_cast<int8_t>(g < -127.f ? -127.f : (g > 127.f ? 127.f : g + (g >= 0.f ? 0.5f : -0.5f)));
  }

  int8_t step_ = -1;
  uint8_t f_ = 20, b_ = 29, ref_f_ = 20, ref_b_ = 29, best_f_ = 20, coarse_f_ = 20, pings_ = 0, n_ = 0;
  uint16_t attempts_ = 0;
  float sum_ = 0.f;
  float snr_[NSTEP];
  TuneResult res_;
};

}  // namespace sonar
}  // namespace icemesh
#endif
