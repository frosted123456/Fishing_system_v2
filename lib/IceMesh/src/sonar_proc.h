// Node-side sonar processing, v0 = C++ port of the Proc class of Frank's display prototype
// (docs/prototype/sonar_display_prototype.html). Input: one ping of 8-bit log amplitude codes,
// 488 bins, 1 or 3 frequencies. Output: bottom (lock + smoothing), noise floor, bottom hardness,
// tracked targets (sub-bin depth, velocity, label Fish / Bait / Near bottom / Cover, strength,
// flicker, frequency spread, echo length), and per bin: live value, static-scene value, change.
// Processing options are the prototype's "Processed" preset (all on). Thresholds are the
// prototype's: tuned on its simulated pings only, to re-tune on real TUSS4470 recordings.
// Bit-exact with the prototype when built with ICEMESH_SONAR_REAL=double (host reference test).
#ifndef ICEMESH_SONAR_PROC_H
#define ICEMESH_SONAR_PROC_H
#include <stdint.h>
#include <string.h>
#include "sonar_scene.h"

namespace icemesh {
namespace sonar {

enum TrackLabel : uint8_t { LBL_FISH = 0, LBL_BAIT = 1, LBL_NEAR_BOTTOM = 2, LBL_COVER = 3 };
enum Hardness : uint8_t { HARD_HARD = 0, HARD_MEDIUM = 1, HARD_SOFT = 2, HARD_UNKNOWN = 3 };

struct TrackOut {
  uint32_t id;
  sreal depth, vel;      // m, m/s (+ = sinking)
  uint8_t label;         // TrackLabel
  sreal v, s;            // display value 0..1 (prototype colour scale), strength (range-compensated dB)
  uint8_t miss;          // pings since last detection (0 = seen in this ping)
  uint8_t hist_n;        // detections kept (max HIST); < 8 = "collecting pings"
  bool has_stats;        // flicker / spread valid (>= 4 detections)
  sreal flick, spread;   // dB: strength change ping to ping, |210 - 190 kHz|
  sreal width;           // m, median echo length of the last 8 detections
};

class SonarProc {
 public:
  enum { MAX_TRACKS = 12, HIST = 30, MAX_PEAKS = 48 };
  struct Out {
    sreal bottom, nf, ratio, tail;
    bool has_ratio;
    uint8_t hard;
    uint8_t n;               // tracks with age >= 3 (prototype output), in tracker order
    TrackOut t[MAX_TRACKS];
  };

  sreal bait_m = R(4.57);   // bait (lure) depth: prototype constant; real system: set per hole

  SonarProc() { reset(); }
  void reset() {
    have_bottom_ = have_nf_ = have_hi_ = have_ratio_ = have_tail_ = have_static_ = false;
    bottom_ = nf_ = hi_ = ratio_ = tail_ = R(0); bj_ = 0; n_tracks_ = 0; nid_ = 1;
  }

  // codes[f][i]: f = 0..nfreq-1 (3 = 190/200/210 kHz compounding, 1 = single frequency)
  void step(const uint8_t codes[sp::NFREQ][BINS], uint8_t nfreq, Out& out) {
    using namespace sp;
    Scratch& S = scratch();
    float* d = S.d; float* lin = S.lin; float* c = S.c; float* sm = S.sm;
    for (int f = 0; f < NFREQ; f++) for (int i = 0; i < N; i++) S.D[f][i] = codeToDb(codes[f < nfreq ? f : 0][i]);
    if (nfreq >= 3) {
      for (int i = 0; i < N; i++) {
        const sreal p = (db2p(S.D[0][i]) + db2p(S.D[1][i]) + db2p(S.D[2][i])) / R(3);
        lin[i] = static_cast<float>(p); d[i] = static_cast<float>(R(10) * std::log10(p));
      }
    } else {
      for (int i = 0; i < N; i++) { d[i] = S.D[nfreq == 2 ? 1 : 0][i]; lin[i] = static_cast<float>(db2p(d[i])); }
    }

    // Bottom: strongest 8-bin energy window, then walk back to the onset
    const int b0 = roundi(R(0.6) / BIN_M());
    sreal e = R(0); int best = b0; sreal bestE = R(-1);
    for (int i = b0; i < b0 + 8; i++) e += static_cast<sreal>(lin[i]);
    for (int i = b0; i + 8 < N; i++) { if (e > bestE) { bestE = e; best = i; } e += static_cast<sreal>(lin[i + 8]) - static_cast<sreal>(lin[i]); }
    int pk = best; for (int i = best; i < std::min(N, best + 8); i++) if (d[i] > d[pk]) pk = i;
    int on = pk; while (on > b0 && static_cast<sreal>(d[on - 1]) > static_cast<sreal>(d[pk]) - R(10)) on--;
    const sreal bNow = static_cast<sreal>(on) * BIN_M();
    if (!have_bottom_) { bottom_ = bNow; have_bottom_ = true; }
    else if (std::fabs(bNow - bottom_) < R(0.3)) { bottom_ += R(0.25) * (bNow - bottom_); bj_ = 0; }
    else if (++bj_ > 4) { bottom_ = bNow; bj_ = 0; }
    const int bI = roundi(bottom_ / BIN_M());

    // Noise floor: median of the quiet water below the bottom
    int a = bI + roundi(R(0.9) / BIN_M()), z = std::min(N, 2 * bI - roundi(R(0.4) / BIN_M()));
    if (z - a < 30) { a = N - 40; z = N; }
    int na = 0;
    for (int i = a; i < z; i++) S.arr[na++] = d[i];
    for (int i = 1; i < na; i++) { const float v = S.arr[i]; int j = i - 1; while (j >= 0 && S.arr[j] > v) { S.arr[j + 1] = S.arr[j]; j--; } S.arr[j + 1] = v; }
    const sreal med = S.arr[na >> 1];
    if (!have_nf_) { nf_ = med; have_nf_ = true; } else nf_ = nf_ + R(0.15) * (med - nf_);
    const sreal nf = nf_;

    // Range compensation and display scaling
    for (int i = 0; i < N; i++) c[i] = static_cast<float>(static_cast<sreal>(d[i]) + static_cast<sreal>(S.tvg[i]));
    sreal hiNow = R(-200);
    for (int i = bI; i < std::min(N, bI + 6); i++) hiNow = std::max(hiNow, static_cast<sreal>(c[i]));
    if (!have_hi_) { hi_ = hiNow; have_hi_ = true; } else hi_ = hi_ + R(0.1) * (hiNow - hi_);
    top_ = hi_ - R(12); lo_ = top_ - R(46);

    // Peaks with sub-bin fit
    sm[0] = c[0]; sm[N - 1] = c[N - 1];
    for (int i = 1; i < N - 1; i++) sm[i] = static_cast<float>((static_cast<sreal>(c[i - 1]) + static_cast<sreal>(c[i]) + static_cast<sreal>(c[i + 1])) / R(3));
    Peak* pk_ = S.peaks; int np = 0;
    const int i0 = roundi(R(0.9) / BIN_M()), i1 = bI - 5;
    for (int i = i0; i < i1; i++) {
      if (!(sm[i] >= sm[i - 1] && sm[i] > sm[i + 1])) continue;
      const sreal snr = static_cast<sreal>(d[i]) - nf; if (snr < R(10)) continue;
      sreal mn = R(1e30);
      for (int k = -6; k <= 6; k++) { const int j = i + k; if (j >= 0 && j < N && static_cast<sreal>(sm[j]) < mn) mn = sm[j]; }
      if (static_cast<sreal>(sm[i]) - mn < R(5)) continue;
      const sreal A = sm[i - 1], B = sm[i], Cc = sm[i + 1], den = A - R(2) * B + Cc;
      const sreal dl = den < R(0) ? clampr(R(0.5) * (A - Cc) / den, R(-0.5), R(0.5)) : R(0);
      int l = i; while (l > 0 && static_cast<sreal>(sm[l - 1]) > B - R(3)) l--;
      int r = i; while (r < N - 1 && static_cast<sreal>(sm[r + 1]) > B - R(3)) r++;
      if (np >= MAX_PEAKS) break;
      Peak& p = pk_[np++];
      p.depth = (static_cast<sreal>(i) + dl) * BIN_M(); p.bin = i; p.s = B; p.width = static_cast<sreal>(r - l + 1) * BIN_M();
      p.f0 = S.D[0][i]; p.f2 = S.D[2][i]; p.v = R(0);
    }
    for (int i = 1; i < np; i++) {   // stable sort, strongest first (Array.prototype.sort is stable)
      const Peak v = pk_[i]; int j = i - 1;
      while (j >= 0 && pk_[j].s < v.s) { pk_[j + 1] = pk_[j]; j--; }
      pk_[j + 1] = v;
    }
    int nk = 0;
    for (int i = 0; i < np; i++) {
      bool ok = true;
      for (int k = 0; k < nk; k++) if (std::abs(pk_[k].bin - pk_[i].bin) <= 4) { ok = false; break; }
      if (ok) pk_[nk++] = pk_[i];
    }
    np = nk;
    for (int i = 0; i < np; i++) pk_[i].v = mapV(pk_[i].s, d[pk_[i].bin]);

    // Tracking
    bool used[MAX_TRACKS]; for (int k = 0; k < MAX_TRACKS; k++) used[k] = false;
    int nfresh = 0;
    for (int i = 0; i < np; i++) {
      const Peak& p = pk_[i];
      int bt = -1; sreal bd = R(0.32);
      for (int k = 0; k < n_tracks_; k++) {
        if (used[k]) continue;
        const Track& T = tracks_[k];
        const sreal dd = std::fabs(T.depth + T.vel * DT() - p.depth);
        if (dd < bd) { bd = dd; bt = k; }
      }
      if (bt >= 0) {
        Track& T = tracks_[bt];
        T.vel = R(0.6) * T.vel + R(0.4) * (p.depth - T.depth) / DT(); T.depth = p.depth; T.miss = 0; T.age++;
        T.push(p); used[bt] = true;
      } else {
        const uint32_t id = nid_++;
        if (n_tracks_ + nfresh < MAX_TRACKS) {
          Track& T = S.fresh[nfresh++];
          T.id = id; T.depth = p.depth; T.vel = R(0); T.age = 1; T.miss = 0; T.hn = 0; T.hh = 0; T.label = LBL_FISH;
          T.push(p);
        }
      }
    }
    for (int k = 0; k < n_tracks_; k++) if (!used[k]) tracks_[k].miss++;
    int w = 0;
    for (int k = 0; k < n_tracks_; k++) if (tracks_[k].miss <= 5) { if (w != k) tracks_[w] = tracks_[k]; w++; }
    for (int k = 0; k < nfresh && w < MAX_TRACKS; k++) tracks_[w++] = S.fresh[k];
    n_tracks_ = w;
    for (int k = 0; k < n_tracks_; k++) tracks_[k].label = labelOf(tracks_[k]);

    // Static scene, learned everywhere except around tracked targets
    if (!have_static_) { for (int i = 0; i < N; i++) { static_[i] = d[i]; sig_[i] = 4.0f; } have_static_ = true; }
    uint8_t* mask = S.mask; memset(mask, 0, N);
    for (int k = 0; k < n_tracks_; k++) {
      const Track& T = tracks_[k];
      if (T.age < 2 || T.label == LBL_COVER) continue;
      const int b = roundi(T.depth / BIN_M());
      for (int q = -5; q <= 8; q++) { const int j = b + q; if (j >= 0 && j < N) mask[j] = 1; }
    }
    for (int i = 0; i < N; i++) {
      const sreal dev = static_cast<sreal>(d[i]) - static_cast<sreal>(static_[i]);
      if (!mask[i]) {
        static_[i] = static_cast<float>(static_cast<sreal>(static_[i]) + R(0.012) * dev);
        sig_[i] = static_cast<float>(static_cast<sreal>(sig_[i]) + R(0.02) * (std::fabs(dev) - static_cast<sreal>(sig_[i])));
      }
      S.dyn[i] = static_cast<float>(dev - (R(2.2) * static_cast<sreal>(sig_[i]) + R(2)));
    }

    // Per-bin display values (prototype pixels: grey static scene, colour where it changed)
    const sreal dim = R(0.78);
    for (int i = 0; i < N; i++) {
      S.vpix[i] = static_cast<float>(mapV(c[i], d[i]) * dim);
      S.bgv[i] = static_cast<float>(mapV(static_cast<sreal>(static_[i]) + static_cast<sreal>(S.tvg[i]), static_[i]) * R(0.85) * dim);
      S.al[i] = static_cast<float>(smooth(R(0), R(6), S.dyn[i]));
    }

    // Bottom character: second-echo ratio and echo length
    const sreal Pn = db2p(nf + R(1.6));
    sreal E1 = R(0); for (int i = bI; i < std::min(N, bI + 21); i++) E1 += static_cast<sreal>(lin[i]);
    E1 -= Pn * R(21);
    const int s2 = 2 * bI - 4, e2 = 2 * bI + 25;
    if (e2 < N) {
      sreal E2 = R(0); for (int i = s2; i < e2; i++) E2 += static_cast<sreal>(lin[i]);
      E2 -= Pn * static_cast<sreal>(e2 - s2);
      const sreal ratio = R(10) * std::log10(std::max(E2, R(1e-14)) / std::max(E1, R(1e-14)));
      if (!have_ratio_) { ratio_ = ratio; have_ratio_ = true; } else ratio_ = ratio_ + R(0.08) * (ratio - ratio_);
    }
    int bp = bI; for (int i = bI; i < std::min(N, bI + 6); i++) if (d[i] > d[bp]) bp = i;
    int j = bp; while (j < N - 1 && static_cast<sreal>(d[j + 1]) > static_cast<sreal>(d[bp]) - R(10)) j++;
    const sreal tl = static_cast<sreal>(j - bI) * BIN_M();
    if (!have_tail_) { tail_ = tl; have_tail_ = true; } else tail_ = tail_ + R(0.1) * (tl - tail_);
    uint8_t hard;
    if (have_ratio_) hard = ratio_ > R(-25) ? HARD_HARD : (ratio_ < R(-27.5) ? HARD_SOFT : HARD_MEDIUM);
    else hard = tail_ < R(0.18) ? HARD_HARD : HARD_SOFT;

    out.bottom = bottom_; out.nf = nf; out.ratio = ratio_; out.has_ratio = have_ratio_; out.tail = tail_; out.hard = hard;
    out.n = 0;
    for (int k = 0; k < n_tracks_; k++) {
      const Track& T = tracks_[k];
      if (T.age < 3) continue;
      TrackOut& o = out.t[out.n++];
      const Hist& h = T.last();
      o.id = T.id; o.depth = T.depth; o.vel = T.vel; o.label = T.label; o.v = h.v; o.s = h.s; o.miss = T.miss;
      o.hist_n = T.hn;
      o.has_stats = T.hn >= 4;
      o.flick = o.has_stats ? flick(T) : R(0);
      o.spread = o.has_stats ? spread(T) : R(0);
      o.width = widthMedian(T);
    }
  }

  // Per-bin results of the last step() of ANY SonarProc (shared scratch): read right after step().
  static const float* binValue() { return scratch().vpix; }    // live echo, prototype colour scale 0..1
  static const float* binStatic() { return scratch().bgv; }    // learned static scene (grey) 0..1
  static const float* binChanged() { return scratch().al; }    // 0..1: live echo differs from the static scene

 private:
  struct Peak { sreal depth; int bin; sreal s, width; float f0, f2; sreal v; };
  struct Hist { sreal depth, s, width, v; float f0, f2; };
  struct Track {
    uint32_t id; sreal depth, vel; uint32_t age; uint8_t miss, hn, hh, label;
    Hist h[HIST];
    void push(const Peak& p) {
      Hist& x = h[hh]; x.depth = p.depth; x.s = p.s; x.width = p.width; x.v = p.v; x.f0 = p.f0; x.f2 = p.f2;
      hh = static_cast<uint8_t>((hh + 1) % HIST); if (hn < HIST) hn++;
    }
    const Hist& back(int k) const { return h[(hh + HIST - 1 - k) % HIST]; }   // k = 0 newest
    const Hist& last() const { return back(0); }
  };
  struct Scratch {
    float D[sp::NFREQ][BINS];
    float d[BINS], lin[BINS], c[BINS], sm[BINS], dyn[BINS], vpix[BINS], bgv[BINS], al[BINS], tvg[BINS], arr[BINS];
    uint8_t mask[BINS];
    Peak peaks[MAX_PEAKS];
    Track fresh[MAX_TRACKS];
  };
  static Scratch& scratch() {
    static Scratch* s = nullptr;   // heap, allocated on first use (keeps it out of builds that never run it)
    if (s == nullptr) {
      s = new Scratch();
      for (int i = 0; i < BINS; i++) {
        const sreal r = std::max(static_cast<sreal>(i) * sp::BIN_M(), R(0.3));
        s->tvg[i] = static_cast<float>(R(40) * std::log10(r) + R(2) * sp::ALPHA() * r);
      }
    }
    return *s;
  }

  sreal mapV(sreal cv, sreal dv) const {
    const sreal x = std::pow(sp::clampr((cv - lo_) / (top_ - lo_), R(0), R(1)), R(0.75));
    return x * sp::smooth(R(3), R(9), dv - nf_);
  }

  sreal flick(const Track& T) const {   // last 12, oldest first like slice(-12)
    const int n = T.hn < 12 ? T.hn : 12;
    sreal sum = R(0); for (int k = n - 1; k >= 0; k--) sum += T.back(k).s;
    const sreal m = sum / static_cast<sreal>(n);
    sreal q = R(0); for (int k = n - 1; k >= 0; k--) { const sreal x = T.back(k).s - m; q += x * x; }
    return std::sqrt(q / static_cast<sreal>(n));
  }
  sreal spread(const Track& T) const {
    const int n = T.hn < 12 ? T.hn : 12;
    sreal sum = R(0); for (int k = n - 1; k >= 0; k--) sum += std::fabs(static_cast<sreal>(T.back(k).f2) - static_cast<sreal>(T.back(k).f0));
    return sum / static_cast<sreal>(n);
  }
  sreal widthMedian(const Track& T) const {
    const int n = T.hn < 8 ? T.hn : 8;
    sreal w[8]; for (int k = 0; k < n; k++) w[k] = T.back(k).width;
    for (int i = 1; i < n; i++) { const sreal v = w[i]; int j = i - 1; while (j >= 0 && w[j] > v) { w[j + 1] = w[j]; j--; } w[j + 1] = v; }
    return n ? w[n >> 1] : R(0);
  }
  uint8_t labelOf(const Track& T) const {
    const int n = T.hn < 30 ? T.hn : 30;
    sreal lo = R(1e30), hi = R(-1e30);
    for (int k = 0; k < n; k++) { const sreal dd = T.back(k).depth; if (dd < lo) lo = dd; if (dd > hi) hi = dd; }
    if (std::fabs(T.depth - bait_m) < R(0.32) && T.age >= 4 && T.hn >= 4 && spread(T) < R(2.5)) return LBL_BAIT;
    if (T.depth > bottom_ - R(0.5)) return (T.age > 30 && hi - lo < R(0.06)) ? LBL_COVER : LBL_NEAR_BOTTOM;
    return LBL_FISH;
  }

  bool have_bottom_, have_nf_, have_hi_, have_ratio_, have_tail_, have_static_;
  sreal bottom_, nf_, hi_, ratio_, tail_, top_ = 0, lo_ = 0;
  int bj_;
  Track tracks_[MAX_TRACKS];
  int n_tracks_;
  uint32_t nid_;
  float static_[BINS], sig_[BINS];
};

}  // namespace sonar
}  // namespace icemesh
#endif
