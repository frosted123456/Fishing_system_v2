// Fake raw sonar: C++ port of the ping simulator in Frank's display prototype
// (docs/prototype/sonar_display_prototype.html, genPing/scene). Output = what the TUSS4470 front
// end would hand to the node processing: per ping, 488 bins x 3 frequencies (190/200/210 kHz) of
// 8-bit log amplitude (dB = -100 + code * 95/255).
// Scene (prototype): 200 kHz 14 deg transducer, 18.5 ft of water, jigged lure at 15 ft, weeds,
// a perch near bottom, a walleye that comes in and strikes, a fish crossing the cone; 48 s loop.
// Other holes get the same story mapped to their own bottom / bait depth and time offset.
// INVENTED DATA for display and pipeline tests only - not a model to trust for numbers.
#ifndef ICEMESH_SONAR_SCENE_H
#define ICEMESH_SONAR_SCENE_H
#include <stdint.h>
#include <string.h>
#include <cmath>
#include <algorithm>
#include "sonar_codec.h"

namespace icemesh {
namespace sonar {

// float on the ESP32 (single-precision FPU; double is emulated and slow). The host reference test
// builds with -DICEMESH_SONAR_REAL=double to match the JavaScript prototype bit for bit.
#ifndef ICEMESH_SONAR_REAL
#define ICEMESH_SONAR_REAL float
#endif
typedef ICEMESH_SONAR_REAL sreal;
inline sreal R(double x) { return static_cast<sreal>(x); }

namespace sp {
static const int N = BINS;                   // 488 bins of 2.5 cm, 0-12.2 m
static const int NFREQ = 3;
inline sreal BIN_M() { return R(0.025); }
inline sreal DT() { return R(0.25); }
inline sreal ALPHA() { return R(0.04); }      // absorption dB/m (prototype value)
inline sreal Q() { return R(95.0 / 255.0); }  // dB per code
inline sreal db2p(sreal x) { return std::pow(R(10), x / R(10)); }
inline sreal clampr(sreal x, sreal a, sreal b) { return x < a ? a : (x > b ? b : x); }
inline sreal smooth(sreal e0, sreal e1, sreal x) { const sreal t = clampr((x - e0) / (e1 - e0), R(0), R(1)); return t * t * (R(3) - R(2) * t); }
inline sreal ease(sreal u) { u = clampr(u, R(0), R(1)); return u * u * (R(3) - R(2) * u); }
inline int roundi(sreal x) { return static_cast<int>(std::floor(x + R(0.5))); }   // Math.round
inline sreal fmodPos(sreal t, sreal m) { return std::fmod(std::fmod(t, m) + m, m); }
inline sreal codeToDb(uint8_t c) { return static_cast<float>(R(-100) + static_cast<sreal>(c) * Q()); }   // Float32 like the prototype
}  // namespace sp

// mulberry32, same bit pattern as the prototype's rnd()
struct Mulberry32 {
  uint32_t s;
  sreal next() {
    s += 0x6D2B79F5u;
    uint32_t t = (s ^ (s >> 15)) * (1u | s);
    t = (t + ((t ^ (t >> 7)) * (61u | t))) ^ t;
    return static_cast<sreal>(static_cast<double>(t ^ (t >> 14)) / 4294967296.0);
  }
};

struct SceneConfig {
  sreal bottom_m;     // prototype: 5.64 (18.5 ft)
  sreal bait_m;       // prototype: 4.57 (15 ft)
  bool sand;          // sand = hard bottom, false = mud
  bool quiet;         // quiet receive (boost converter off): no striped noise
  sreal time0;        // scene time of the first ping (prototype restart: 18 s)
  uint32_t seed;      // prototype: 20261005
  static SceneConfig prototype() { SceneConfig c; c.bottom_m = R(5.64); c.bait_m = R(4.57); c.sand = true; c.quiet = true; c.time0 = R(18); c.seed = 20261005u; return c; }
  // Same story, another hole: bottom 3.5-6.5 m, bait 0.6-1.6 m above it, sand or mud, own time offset.
  // (Deeper than ~6.5 m the simulated lure falls under the prototype's detection threshold.)
  static SceneConfig forHole(uint32_t s) {
    Mulberry32 r; r.s = s * 2654435761u + 12345u;
    SceneConfig c = prototype();
    c.bottom_m = R(3.5) + r.next() * R(3.0);
    c.bait_m = c.bottom_m - R(0.6) - r.next() * R(1.0);
    c.sand = r.next() < R(0.6);
    c.time0 = r.next() * R(48);
    c.seed = s * 7919u + 1u;
    return c;
  }
};

class SonarScene {
 public:
  enum { NWEEDS = 16 };

  void begin(const SceneConfig& c) {
    cfg_ = c;
    identity_ = (c.bottom_m == R(5.64) && c.bait_m == R(4.57));
    rng_.s = c.seed;                                         // weeds are drawn at page load...
    for (int k = 0; k < NWEEDS; k++) {
      Weed& w = weeds_[k];
      w.h = R(0.05) + rng_.next() * R(0.40); w.ts = R(-50) - rng_.next() * R(5);
      w.s = R(0.5) + rng_.next(); w.p = rng_.next() * R(6.28);
    }
    rng_.s = c.seed;                                         // ...then restart() resets the seed
    t_ = c.time0; jig_ = R(-99); phase_ = R(0); last_lt_ = sp::fmodPos(t_, R(48));
  }

  // One ping: codes[f][i] = 8-bit log amplitude of bin i at frequency f (190/200/210 kHz).
  void ping(uint8_t codes[sp::NFREQ][BINS]) {
    using namespace sp;
    const sreal t = t_, lt = fmodPos(t, R(48));
    static const sreal J[3] = {R(6), R(20), R(30)};
    for (int k = 0; k < 3; k++) if (last_lt_ < J[k] && lt >= J[k]) jig_ = t;
    last_lt_ = lt;
    const sreal bd = cfg_.bottom_m + R(0.002) * std::sin(t * R(0.7));
    const bool hard = cfg_.sand;
    const sreal Rf = hard ? R(-12) : R(-20), tail = hard ? R(0.05) : R(0.12);
    Tgt tg[4]; const int nt = scene(t, tg);
    const sreal Pn = db2p(R(-84));
    const sreal Pb = db2p(Rf - R(20) * std::log10(R(2) * bd)), Pb2 = db2p(R(2) * Rf - R(6) - R(20) * std::log10(R(4) * bd));
    const sreal q = Q();
    const int rEnd = roundi(R(0.95) / BIN_M());
    sreal* P = scratchP();
    for (int f = 0; f < NFREQ; f++) {
      for (int i = 0; i < N; i++) {
        P[i] = Pn * expo();
        if (!cfg_.quiet) {
          const sreal s = R(0.5) + R(0.5) * std::sin(R(2 * 3.14159265358979323846) * static_cast<sreal>(i) / R(7.3) + phase_ + static_cast<sreal>(f) * R(0.4));
          P[i] += Pn * R(7) * s * s * s * s * expo();
        }
      }
      for (int i = 0; i < rEnd; i++) P[i] += db2p(R(-8)) * std::exp(-(static_cast<sreal>(i) * BIN_M()) / R(0.045)) * (R(0.8) + R(0.4) * rng_.next());
      addBottom(P, bd, Pb, tail, hard ? R(0.3) : R(0.5));
      addBottom(P, R(2) * bd, Pb2, tail * R(1.8), R(0.7));
      for (int k = 0; k < NWEEDS; k++) {
        const Weed& w = weeds_[k];
        const sreal d = bd - w.h + R(0.01) * std::sin(t * w.s + w.p);
        addEcho(P, d, db2p(w.ts - R(40) * std::log10(d) - R(2) * ALPHA() * d) * expo());
      }
      for (int k = 0; k < nt; k++) {
        const Tgt& g = tg[k];
        if (g.off > R(1.25)) continue;
        const sreal ts = g.ts + g.tilt[f] - R(14) * g.off;
        addEcho(P, g.d, db2p(ts - R(40) * std::log10(g.d) - R(2) * ALPHA() * g.d) * (g.fish ? expo() : rician(R(0.15))));
      }
      for (int i = 0; i < N; i++) {
        const sreal x = clampr(R(10) * std::log10(P[i]), R(-100), R(-5));
        const int code = roundi((x + R(100)) / q);
        codes[f][i] = static_cast<uint8_t>(code < 0 ? 0 : (code > 255 ? 255 : code));
      }
    }
    t_ += DT(); phase_ += R(0.9);
  }

  void jig() { jig_ = t_; }                       // "Jig the bait" button of the prototype
  sreal time() const { return t_; }
  const SceneConfig& config() const { return cfg_; }

 private:
  struct Weed { sreal h, ts, s, p; };
  struct Tgt { sreal d, ts, tilt[3]; bool fish; sreal off; };

  static sreal* scratchP() { static sreal p[BINS]; return p; }
  sreal expo() { return -std::log(R(1) - rng_.next()); }
  sreal gauss() { sreal u = rng_.next(); if (u < R(1e-12)) u = R(1e-12); return std::sqrt(R(-2) * std::log(u)) * std::cos(R(2 * 3.14159265358979323846) * rng_.next()); }
  sreal rician(sreal s) { const sreal a = R(1) + s * gauss() * R(0.7071); const sreal b = s * gauss() * R(0.7071); return a * a + b * b; }

  // prototype depths (bait 4.57, bottom 5.64) -> this hole's bait / bottom
  sreal mapD(sreal d) const {
    if (identity_) return d;
    if (d <= R(4.57)) return d * cfg_.bait_m / R(4.57);
    return cfg_.bait_m + (d - R(4.57)) * (cfg_.bottom_m - cfg_.bait_m) / R(5.64 - 4.57);
  }

  static sreal jigOff(sreal dt) {
    if (dt < R(0) || dt > R(2.2)) return R(0);
    if (dt < R(0.25)) return R(-0.25) * (dt / R(0.25));
    const sreal u = (dt - R(0.25)) / R(1.95);
    return R(-0.25) * (R(1) - u) + R(0.035) * std::sin(dt * R(13)) * (R(1) - u);
  }

  int scene(sreal t, Tgt* tg) const {
    using namespace sp;
    const sreal lt = fmodPos(t, R(48)), BAIT = R(4.57);
    int n = 0;
    { Tgt& g = tg[n++]; g.d = BAIT + R(0.006) * std::sin(t * R(1.7)) + jigOff(t - jig_); g.ts = R(-42); g.tilt[0] = R(0.3); g.tilt[1] = R(0); g.tilt[2] = R(-0.2); g.fish = false; g.off = R(0); }
    if (lt >= R(14) && lt < R(41.5)) {          // walleye: approach, hover, strike, leave
      sreal d, off;
      if (lt < R(24)) { const sreal u = ease((lt - R(14)) / R(10)); d = R(3.40) + (R(4.85) - R(3.40)) * u; off = R(1.0) - R(0.9) * u; }
      else if (lt < R(34)) { d = R(4.85) + R(0.05) * std::sin(R(1.3) * lt); off = R(0.1) + R(0.1) * std::fabs(std::sin(R(0.7) * lt)); }
      else if (lt < R(36.5)) { const sreal u = ease((lt - R(34)) / R(2.5)); d = R(4.85) + (BAIT + R(0.05) - R(4.85)) * u; off = R(0.1); }
      else if (lt < R(37.6)) { d = BAIT + R(0.04) + R(0.02) * std::sin(lt * R(9)); off = R(0.05); }
      else { const sreal u = ease((lt - R(37.6)) / R(3.9)); d = BAIT + R(0.04) + (R(5.30) - BAIT - R(0.04)) * u; off = R(0.05) + R(1.15) * u; }
      const sreal a = std::sin(R(0.35) * t) * R(1.6);
      Tgt& g = tg[n++]; g.d = d; g.ts = R(-32); g.tilt[0] = a; g.tilt[1] = R(0); g.tilt[2] = -a; g.fish = true; g.off = off;
    }
    if (lt >= R(3) && lt < R(22)) {             // perch near bottom, in and out of the cone
      Tgt& g = tg[n++];
      g.d = R(5.30) + R(0.06) * std::sin(R(0.9) * t) + R(0.04) * std::sin(R(2.1) * t); g.ts = R(-45);
      g.tilt[0] = R(1); g.tilt[1] = R(0); g.tilt[2] = R(-1); g.fish = true; g.off = R(0.4) + R(0.4) * std::sin(R(0.5) * t);
    }
    if (lt >= R(41.5) && lt < R(46.5)) {        // fish crossing the cone
      const sreal u = (lt - R(41.5)) / R(5);
      Tgt& g = tg[n++];
      g.d = R(2.9) + R(0.4) * u; g.ts = R(-37); g.tilt[0] = R(-1); g.tilt[1] = R(0); g.tilt[2] = R(1); g.fish = true;
      g.off = std::fabs(u - R(0.5)) * R(2.2);
    }
    for (int k = 0; k < n; k++) tg[k].d = mapD(tg[k].d);
    return n;
  }

  static void addEcho(sreal* P, sreal depth, sreal pw) {
    using namespace sp;
    const sreal c = depth / BIN_M();
    const int i0 = static_cast<int>(std::floor(c));
    for (int k = -3; k <= 14; k++) {
      const int i = i0 + k; if (i < 0 || i >= N) continue;
      const sreal x = (static_cast<sreal>(i) - c) * BIN_M();
      P[i] += pw * (x < R(0) ? std::exp(-(x * x) / (R(2) * R(0.03) * R(0.03))) : std::exp(-x / R(0.07)));
    }
  }
  void addBottom(sreal* P, sreal bd, sreal Pb, sreal tail, sreal spk) {
    using namespace sp;
    const sreal c = bd / BIN_M();
    const int i0 = std::max(0, static_cast<int>(std::floor(c)) - 4);
    const int i1 = std::min(N - 1, static_cast<int>(std::ceil(c + R(1.2) / BIN_M())));
    for (int i = i0; i <= i1; i++) {
      const sreal x = (static_cast<sreal>(i) - c) * BIN_M();
      const sreal g = x < R(0) ? std::exp(-(x * x) / (R(2) * R(0.03) * R(0.03)))
                               : std::exp(-x / tail) * std::exp(-(x * x) / (R(2) * R(0.35) * R(0.35)));
      P[i] += Pb * g * rician(spk);
    }
  }

  SceneConfig cfg_;
  bool identity_ = true;
  Mulberry32 rng_;
  Weed weeds_[NWEEDS];
  sreal t_ = 0, jig_ = -99, phase_ = 0, last_lt_ = 0;
};

}  // namespace sonar
}  // namespace icemesh
#endif
