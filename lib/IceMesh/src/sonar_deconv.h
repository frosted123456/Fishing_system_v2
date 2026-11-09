// D126: sharpened (deconvolved) echo for the raw echogram line, on the hub / tip-up (docs/SONAR_DECONV.md).
// Mode 1 = the same algorithm as sharpenEcho() in web/suite.html and tools/sonar_deconv_test.py:
//   1. baseline (D126e) = floor + ring-down tail, power F + A exp(-d / tau), fitted through the lowest points (points
//      above the curve weigh 2 %): one continuous line that never rises (Frank's sketch), never under the noise floor
//   2. excess over the baseline in linear amplitude (no margin: small bumps stay)
//   3. Richardson-Lucy, 100 iterations, with the hump of one target measured on the ball (07nov 14:28, 32 cycles)
//   4. re-blur ~5 cm, split into lobes; neighbours stay ONE echo unless the deconvolved dip is >= 6 dB and the raw ping
//      itself dips >= 3 dB between them (no false fish out of a plateau); from 6 cm above the bottom on: one echo. Each
//      echo comes down to the baseline valley to valley, height-matched to the raw excess (narrower, same height);
//      inside a merged echo no dip deeper than 6 dB under the raw ping. The ring zone (0.15 m after the blind zone): raw
// Mode 2 = mode 1 + "doublet split": every lobe >= ~6 dB over the baseline is fitted with 1 and with 2 humps (free positions on
//   a 0.25-bin grid, non-negative weighted least squares, weights 1/amplitude^2: the log-amp noise is multiplicative,
//   like Poisson-weighted spectral unmixing). 2 is kept when the residual drops > 2x and the weaker hump is > 15 % of
//   the stronger; the lobe is then drawn as two narrow peaks at the fitted depths and heights. Synthetic test: two
//   echoes 5 cm apart split 88-100 %, one echo never split (docs/SONAR_DECONV.md, test 2).
// Window: blind zone .. bottom + 0.35 m (floor echo + one more hump), at most MAXW bins; outside: unchanged.
// Memory (D126f): NO heap. The caller lends 3 arrays of >= 2 x MAXW floats and one of >= N (the hub lends 4 scratch
// arrays of the sonar processor that are free between two pings: SonarProc::lendScratch). History: D126 kept a heap
// block for good (recording dead, < 16 kB free); D126b borrowed it per line, but the bench log showed the heap
// minimum falling to 2.9 kB with sharp 2 (crash territory).
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>
#include <algorithm>

namespace icemesh { namespace deconv {

static const int MAXW = 240;      // window bins (6.0 m at 2.5 cm; the 5 cm echo line reaches bottom + 0.35 m < 5 m)
static const int MAXN = 512;      // >= sonar BINS (488)
static const int HL = 9, HC = 3;  // hump length and reference (peak) index, 2.5 cm bins
inline const float* hump() { static const float H[HL] = {0.f, 0.58f, 0.86f, 1.0f, 0.88f, 0.56f, 0.28f, 0.10f, 0.04f}; return H; }

struct Work { float *B, *e, *x, *t, *s, *w, *out; };   // views into lent memory
// a0..a2: >= 2 * MAXW floats each, a3: >= N floats (the output line)
inline Work workFrom(float* a0, float* a1, float* a2, float* a3) { Work W; W.B = a0; W.e = a0 + MAXW; W.x = a1; W.t = a1 + MAXW; W.s = a2; W.w = a2 + MAXW; W.out = a3; return W; }
// D133 echo marks: what sharpen() found, for the "echo marks" line (sonar_codec.h payload). pos = bin (sub-bin, from bin 0),
// lvl = peak level dB, w = width at -6 dB in bins. Baseline as fitted: floor F and value at the window start, tau index.
static const int MARKS_CAP = 24;
struct Marks { float floorDb, b0Db; uint8_t tau; int a, n; int nm; float pos[MARKS_CAP], lvl[MARKS_CAP], w[MARKS_CAP];
  void add(float p, float l, float wd) { if (nm < MARKS_CAP) { pos[nm] = p; lvl[nm] = l; w[nm] = wd; nm++; } } };
struct WorkStore { float a[3][2 * MAXW]; float o[MAXN]; Work get() { return workFrom(a[0], a[1], a[2], o); } };   // host tools / tests

inline float lin(float db) { return std::pow(10.f, db / 20.f); }
inline float dbv(float a) { return 20.f * std::log10(a > 1e-9f ? a : 1e-9f); }
inline float humpAt(float i) {   // hump at fractional index i (0..8), linear interpolation, 0 outside
  if (i < 0.f || i > 8.f) return 0.f;
  const int k = static_cast<int>(i); const float f = i - static_cast<float>(k); const float* H = hump();
  return H[k] * (1.f - f) + H[k < 8 ? k + 1 : 8] * f;
}

// D126e baseline of y[0..n) (dB) into z: power = F + A exp(-i bin / tau), F, A >= 0; weighted least squares in relative
// power, points above the curve weigh p, 8 passes; tau on a log grid 2 cm .. 2 m picked by the symmetric ratio error
// (q - 1)^2 / q (q = model / data, ~ dB^2 without a log). Uses W.w (weights), W.t (normalised power), W.s.
inline void floorDecay(const float* y, int n, float bin, float nf, float* z, Work& W, Marks* mk = nullptr, float p = 0.02f) {
  float *w = W.w, *Q = W.t, *iq = W.s;   // iq = 1 / Q^2 (W.s is free until Richardson-Lucy)
  float pmin = 1e30f; for (int i = 0; i < n; i++) { Q[i] = std::pow(10.f, y[i] / 10.f); if (Q[i] < pmin) pmin = Q[i]; }
  if (!(pmin > 0.f)) pmin = 1e-30f;
  for (int i = 0; i < n; i++) { Q[i] /= pmin; w[i] = 1.f; iq[i] = 1.f / (Q[i] * Q[i]); }   // normalised >= 1: the sums stay in float range
  float bF = 1.f, bA = 0.f, bR = 0.f; int bT = 0;
  for (int pass = 0; pass < 8; pass++) {
    float best = 1e30f;
    for (int t = 0; t < 30; t++) {
      const float tau = 0.02f * std::pow(100.f, static_cast<float>(t) / 29.f), rr = std::exp(-bin / tau);
      float s11 = 0.f, s12 = 0.f, s22 = 0.f, r1 = 0.f, r2 = 0.f, E = 1.f;
      for (int i = 0; i < n; i++) { const float sw = w[i] * iq[i]; s11 += sw; s12 += sw * E; s22 += sw * E * E; r1 += sw * Q[i]; r2 += sw * E * Q[i]; E *= rr; }
      float cF[3], cA[3]; int nc = 0;
      const float det = s11 * s22 - s12 * s12;
      if (det > 0.f) { const float F = (r1 * s22 - r2 * s12) / det, A = (r2 * s11 - r1 * s12) / det; if (F >= 0.f && A >= 0.f) { cF[nc] = F; cA[nc] = A; nc++; } }
      if (s11 > 0.f) { cF[nc] = std::max(r1 / s11, 0.f); cA[nc] = 0.f; nc++; }
      if (s22 > 0.f) { cF[nc] = 0.f; cA[nc] = std::max(r2 / s22, 0.f); nc++; }
      for (int c = 0; c < nc; c++) {
        float r = 0.f; E = 1.f;
        for (int i = 0; i < n; i++) { const float m = cF[c] + cA[c] * E, d = m - Q[i], mq = m * Q[i]; r += w[i] * d * d / (mq > 1e-30f ? mq : 1e-30f); E *= rr; }
        if (r < best) { best = r; bF = cF[c]; bA = cA[c]; bR = rr; bT = t; }
      }
    }
    float E = 1.f; for (int i = 0; i < n; i++) { const float m = bF + bA * E; w[i] = Q[i] > m ? p : 1.f; E *= bR; }
  }
  if (mk) { const float f = bF * pmin, b0 = (bF + bA) * pmin; mk->floorDb = 10.f * std::log10(f > 1e-30f ? f : 1e-30f); mk->b0Db = 10.f * std::log10(b0 > 1e-30f ? b0 : 1e-30f); mk->tau = static_cast<uint8_t>(bT); }
  float E = 1.f;
  for (int i = 0; i < n; i++) { const float m = (bF + bA * E) * pmin; const float v = 10.f * std::log10(m > 1e-30f ? m : 1e-30f); z[i] = v > nf ? v : nf; E *= bR; }
}

// y[i] = sum_j v[j] h[i - j + ref]   (h = hump / its sum, ref = HC): forward model; adjoint with the flipped kernel
inline void convFwd(const float* v, int n, const float* h, float* res) {
  for (int i = 0; i < n; i++) { float s = 0.f; for (int k = 0; k < HL; k++) { const int m = i + HC - k; if (m >= 0 && m < n) s += v[m] * h[k]; } res[i] = s; }
}
inline void convAdj(const float* v, int n, const float* h, float* res) {
  for (int i = 0; i < n; i++) { float s = 0.f; for (int k = 0; k < HL; k++) { const int m = i - HC + k; if (m >= 0 && m < n) s += v[m] * h[k]; } res[i] = s; }
}

// 1-vs-2 hump fit on e[i0..i1) with weights w; returns 1 or 2 humps (positions in bins relative to the window, amplitudes)
inline int fitDoublet(const float* e, const float* wt, int i0, int i1, float* pos, float* amp) {
  auto atom = [](float p, int i) { return humpAt(static_cast<float>(i) - p + HC); };
  float best1 = 1e30f, p1b = 0.f, a1b = 0.f;
  for (float p = static_cast<float>(i0); p <= static_cast<float>(i1 - 1); p += 0.25f) {
    float hh = 0.f, he = 0.f; for (int i = i0; i < i1; i++) { const float h = atom(p, i); hh += wt[i] * h * h; he += wt[i] * h * e[i]; }
    const float a = hh > 0.f && he > 0.f ? he / hh : 0.f; float r = 0.f;
    for (int i = i0; i < i1; i++) { const float d = e[i] - a * atom(p, i); r += wt[i] * d * d; }
    if (r < best1) { best1 = r; p1b = p; a1b = a; }
  }
  float best2 = 1e30f, q1 = 0.f, q2 = 0.f, b1 = 0.f, b2 = 0.f;
  for (float pa = static_cast<float>(i0); pa <= static_cast<float>(i1 - 1); pa += 0.25f) {
    if (std::fabs(pa - p1b) > 4.f) continue;
    for (float pb = pa + 0.5f; pb <= pa + 8.f && pb <= static_cast<float>(i1 - 1); pb += 0.25f) {
      float s11 = 0.f, s12 = 0.f, s22 = 0.f, r1 = 0.f, r2 = 0.f;
      for (int i = i0; i < i1; i++) { const float h1 = atom(pa, i), h2 = atom(pb, i), ww = wt[i];
        s11 += ww * h1 * h1; s12 += ww * h1 * h2; s22 += ww * h2 * h2; r1 += ww * h1 * e[i]; r2 += ww * h2 * e[i]; }
      const float det = s11 * s22 - s12 * s12; if (det <= 1e-20f) continue;
      float x1 = (r1 * s22 - r2 * s12) / det, x2 = (r2 * s11 - r1 * s12) / det;
      if (x1 < 0.f || x2 < 0.f) continue;   // non-negative: the one-hump fit already covers a zero amplitude
      float r = 0.f; for (int i = i0; i < i1; i++) { const float d = e[i] - x1 * atom(pa, i) - x2 * atom(pb, i); r += wt[i] * d * d; }
      if (r < best2) { best2 = r; q1 = pa; q2 = pb; b1 = x1; b2 = x2; }
    }
  }
  const float mx = b1 > b2 ? b1 : b2, mn = b1 > b2 ? b2 : b1;
  if (best2 < 1e29f && best2 > 0.f && best1 / best2 > 2.f && mn > 0.15f * mx) { pos[0] = q1; amp[0] = b1; pos[1] = q2; amp[1] = b2; return 2; }
  pos[0] = p1b; amp[0] = a1b; return 1;
}

// db[0..N) -> out[0..N) (dB). Returns false (out untouched) when no work memory or the window is too short.
inline bool sharpen(const float* db, int N, float bin, float nf, float bottom, float dead, int mode, float* out, Work& W, Marks* mk = nullptr) {
  if (N > MAXN || bin <= 0.f) return false;
  const int a = std::max(1, static_cast<int>(std::lround(dead / bin)));
  const float endm = (bottom > 0.f ? bottom : static_cast<float>(N) * bin) + 0.35f;
  int b = static_cast<int>(std::lround(endm / bin)); if (b > N) b = N; if (b - a > MAXW) b = a + MAXW;
  const int n = b - a; if (n < 8) return false;
  for (int i = 0; i < N; i++) out[i] = db[i];
  const int k0 = std::min(n, static_cast<int>(std::lround(0.15f / bin)));
  float* B = W.B; float* e = W.e; float* x = W.x; float* t = W.t; float* s = W.s;
  if (mk) { mk->nm = 0; mk->a = a; mk->n = n; }
  floorDecay(db + a, n, bin, nf, B, W, mk);
  for (int i = 0; i < n; i++) { const float v = lin(db[a + i]) - lin(B[i]); e[i] = v > 0.f ? v : 0.f; }
  // Richardson-Lucy
  const float* H = hump(); float HS = 0.f; for (int k = 0; k < HL; k++) HS += H[k];
  float HN[HL]; for (int k = 0; k < HL; k++) HN[k] = H[k] / HS;
  float mean = 0.f; for (int i = 0; i < n; i++) mean += e[i]; mean /= static_cast<float>(n); if (mean < 1e-9f) mean = 1e-9f;
  for (int i = 0; i < n; i++) x[i] = mean;
  for (int it = 0; it < 100; it++) {
    convFwd(x, n, HN, t);
    for (int i = 0; i < n; i++) t[i] = (e[i] + 1e-12f) / (t[i] + 1e-12f);
    convAdj(t, n, HN, s);
    for (int i = 0; i < n; i++) x[i] *= s[i];
  }
  // re-blur ~5 cm (gaussian sigma 0.85 bin at 2.5 cm)
  float G[7]; for (int k = -3; k <= 3; k++) { const float u = static_cast<float>(k) * bin / 0.025f / 0.85f; G[k + 3] = std::exp(-0.5f * u * u); }
  for (int i = 0; i < n; i++) { float v = 0.f; for (int k = -3; k <= 3; k++) { const int m = i + k; if (m >= 0 && m < n) v += x[m] / HS * G[3 - k]; } s[i] = v; }
  float* so = t;   // output excess (linear)
  for (int i = 0; i < n; i++) so[i] = 0.f;
  float smax = 1e-30f, emax = 0.f; for (int i = 0; i < n; i++) { if (s[i] > smax) smax = s[i]; if (e[i] > emax) emax = e[i]; }
  const float cc = emax / smax;
  auto lvl = [&](int i) { return dbv(s[i] * cc + lin(B[i])); };   // displayed level of s (dB), roughly height-matched
  auto am = [&](int i0, int i1) { int j = i0; for (int q = i0; q < i1; q++) if (s[q] > s[j]) j = q; return j; };
  const int kb = bottom > 0.f ? static_cast<int>(std::lround((bottom - 0.06f) / bin)) - a : n;
  float* wt = W.w;   // weights for the doublet fit: 1 / total amplitude^2 (floorDecay is done with W.w)
  for (int i = 0; i < n; i++) { const float y = lin(db[a + i]); wt[i] = 1.f / (y * y > 1e-18f ? y * y : 1e-18f); }
  // one echo [g0, g1), first / last lobe peak gf / gl
  auto emit = [&](int g0, int g1, int gf, int gl) {
    const int L = g1 - g0; const float l0 = g0 > k0 ? s[g0] : 0.f, l1 = g1 < n ? s[g1 - 1] : 0.f;   // window edges are not valleys
    auto vv = [&](int q) { const float ln = L > 1 ? l0 + (l1 - l0) * static_cast<float>(q - g0) / static_cast<float>(L - 1) : l0; const float v = s[q] - ln; return v > 0.f ? v : 0.f; };
    int j = g0; for (int q = g0; q < g1; q++) if (vv(q) > vv(j)) j = q;
    const float pv = vv(j); if (pv <= 0.f) return;
    float raw = 0.f; for (int q = std::max(0, j - 1); q < std::min(n, j + 2); q++) if (e[q] > raw) raw = e[q];
    for (int q = g0; q < g1; q++) so[q] = vv(q) * raw / pv;
    const int ge = g1 == n ? n - 1 : gl;   // merged echo: no dip deeper than 6 dB under the raw ping
    if (ge > gf) for (int q = gf; q <= ge; q++) if (so[q] < 0.5f * e[q]) so[q] = 0.5f * e[q];
    if (mk) {   // D133: one mark per echo, >= 3 dB over the baseline: sub-bin peak (parabola), width at -6 dB (half amplitude)
      int jm = g0; for (int q = g0; q < g1; q++) if (so[q] > so[jm]) jm = q;
      const float pk = so[jm], lv = dbv(pk + lin(B[jm]));
      if (pk > 0.f && lv - B[jm] >= 3.f) {
        float dl = 0.f;
        if (jm > g0 && jm + 1 < g1) { const float y0 = so[jm - 1], y2 = so[jm + 1], den = y0 - 2.f * pk + y2; if (den < 0.f) { dl = 0.5f * (y0 - y2) / den; dl = dl < -0.5f ? -0.5f : (dl > 0.5f ? 0.5f : dl); } }
        int wn = 0; for (int q = g0; q < g1; q++) if (so[q] >= 0.5f * pk) wn++;
        mk->add(static_cast<float>(a + jm) + dl, lv, static_cast<float>(wn));
      }
    }
  };
  int g0 = -1, g1 = -1, gf = -1, gl = -1, gm = -1, i0 = k0; float gpl = -1e9f;   // gm / gpl: the echo's strongest bin and top level so far
  for (int i = k0 + 1; i <= n; i++) {
    const bool cut = (i == n) || (i < n - 1 && s[i] <= s[i - 1] && s[i] <= s[i + 1]);
    if (!cut) continue;
    const int i1 = i;
    if (i1 > i0) {
      const int jr = am(i0, i1);
      bool merge = false;
      float pr = -1e9f; for (int q = i0; q < i1; q++) pr = std::max(pr, lvl(q));
      if (g0 >= 0) {
        const int jl = gm; const float pl = gpl;
        float rmin = 1e9f; for (int q = jl; q <= jr; q++) rmin = std::min(rmin, db[a + q]);
        // D136: the bottom zone (from 6 cm above the bottom) never swallows an echo standing above it: a ball or a fish
        // 5-15 cm over the floor was merged into the bottom echo (07nov 14:28 replay) and vanished in the marks view
        const bool intoBottom = jr >= kb && jl < kb;
        merge = (!intoBottom && (std::min(pl, pr) - lvl(i0) < 6.f || std::min(db[a + jl], db[a + jr]) - rmin < 3.f)) || (jr >= kb && jl >= kb);
      }
      if (merge) { g1 = i1; gl = jr; if (s[jr] > s[gm]) gm = jr; if (pr > gpl) gpl = pr; }
      else { if (g0 >= 0) emit(g0, g1, gf, gl); g0 = i0; g1 = i1; gf = jr; gl = jr; gm = jr; gpl = pr; }
    }
    i0 = i1;
  }
  if (g0 >= 0) emit(g0, g1, gf, gl);
  // mode 2: doublet split, per Richardson-Lucy lobe (local fit window: the floor around a wide merged echo would swamp
  // it). Every lobe >= ~6 dB over the baseline, not on the bottom echo (a plane echo + its tail would always 'split').
  if (mode >= 2) {
    // fitted on the excess over baseline + 3 dB: the floor's own +-1 dB flicker, weighted 1/amplitude^2, would steer the fit
    float* e3 = x; for (int i = 0; i < n; i++) { const float v = lin(db[a + i]) - 1.4125f * lin(B[i]); e3[i] = v > 0.f ? v : 0.f; }
    int l0 = k0;
    for (int i = k0 + 1; i <= n; i++) {
      const bool cut = (i == n) || (i < n - 1 && s[i] <= s[i - 1] && s[i] <= s[i + 1]);
      if (!cut) continue;
      const int l1 = i;
      if (l1 > l0) {
        const int j = am(l0, l1);
        float raw = 0.f; for (int q = std::max(0, j - 1); q < std::min(n, j + 2); q++) if (e[q] > raw) raw = e[q];
        // D136: only a lobe that stands out of its surroundings in the RAW ping (>= 6 dB over the raw level 2 bins outside
        // the lobe on both sides): a plateau (barrel / bin reverb, a school, weeds) is broken by Richardson-Lucy into lobes
        // that the 2-hump fit then 'split' into false narrow pairs (replay of 07nov 14:28: marks at 0.45-0.6 m on a flat tail)
        const float rl = db[a + std::max(0, l0 - 2)], rr = db[a + std::min(n - 1, l1 + 1)], rp = db[a + j];
        const bool standsOut = rp - (rl > rr ? rl : rr) >= 6.f;   // strict on purpose: false pairs on a plateau are worse than a missed split
        if (standsOut && (bottom <= 0.f || static_cast<float>(a + j) * bin < bottom - 0.06f) && raw > lin(B[j])) {
          const int f0 = std::max(k0, l0 - 2), f1 = std::min(n, l1 + 2);
          float pos[2], amp[2]; const int m = fitDoublet(e3, wt, f0, f1, pos, amp);
          if (m == 2) {   // D133b: height-matched like every other echo: the fit ran on the excess over baseline + 3 dB, and two
            // sigma-0.5 peaks overlap: scale both so the drawn maximum equals the raw excess maximum of the lobe (review: 1-3 dB low)
            float vmax = 0.f, emax = 0.f;
            for (int q = l0; q < l1; q++) { float v = 0.f; for (int k = 0; k < 2; k++) { const float u = (static_cast<float>(q) - pos[k]) / 0.5f; v += amp[k] * std::exp(-0.5f * u * u); } if (v > vmax) vmax = v; if (e[q] > emax) emax = e[q]; }
            for (int k = 0; k < 2; k++) { float v = amp[k]; for (int k2 = 0; k2 < 2; k2++) if (k2 != k) { const float d = (pos[k] - pos[k2]) / 0.5f; v += amp[k2] * std::exp(-0.5f * d * d); } if (v > vmax) vmax = v; }
            if (vmax > 0.f && emax > 0.f) { amp[0] *= emax / vmax; amp[1] *= emax / vmax; }
          }
          if (m == 2) for (int q = f0; q < f1; q++) {   // two narrow peaks (sigma 0.5 bin: a 5 cm pair shows a clear dip)
            float v = 0.f; for (int k = 0; k < 2; k++) { const float u = (static_cast<float>(q) - pos[k]) / 0.5f; v += amp[k] * std::exp(-0.5f * u * u); }
            so[q] = (q >= l0 && q < l1) ? v : (v > so[q] ? v : so[q]);
          }
          if (m == 2 && mk) {   // D133: the lobe's mark becomes two narrow marks (width at -6 dB of a sigma 0.5 bin peak: 1.2 bins)
            int o = 0; for (int k = 0; k < mk->nm; k++) { const float p = mk->pos[k] - static_cast<float>(a); if (p >= static_cast<float>(l0) && p < static_cast<float>(l1)) continue; mk->pos[o] = mk->pos[k]; mk->lvl[o] = mk->lvl[k]; mk->w[o] = mk->w[k]; o++; }
            mk->nm = o;
            for (int k = 0; k < 2; k++) { int qb = static_cast<int>(pos[k] + 0.5f); qb = qb < 0 ? 0 : (qb >= n ? n - 1 : qb); const float lv = dbv(amp[k] + lin(B[qb])); if (lv - B[qb] >= 3.f) mk->add(static_cast<float>(a) + pos[k], lv, 1.2f); }
          }
        }
      }
      l0 = l1;
    }
  }
  for (int i = 0; i < k0; i++) so[i] = e[i];   // ring zone left raw
  if (mk) for (int i = 1; i + 1 < k0 && i + 1 < n; i++) {   // D133b: ring zone marks (raw peaks >= 6 dB over the tail), else marks mode shows nothing there
    if (!(e[i] > e[i - 1] && e[i] >= e[i + 1])) continue;
    const float lv = dbv(e[i] + lin(B[i])); if (lv - B[i] < 6.f) continue;
    // D136: and it must stand out of the raw tail around it (>= 6 dB over the raw 2 bins either side): the tail's own
    // wobble (07nov 13:36 barrel replay: dozens of spikes at 0.3-0.45 m) is not an echo
    { const float r0 = db[a + (i >= 2 ? i - 2 : 0)], r1 = db[a + (i + 2 < n ? i + 2 : n - 1)]; if (db[a + i] - (r0 > r1 ? r0 : r1) < 6.f) continue; }
    int wn = 0; for (int q = std::max(0, i - 4); q < std::min(n, i + 5); q++) if (e[q] >= 0.5f * e[i]) wn++;
    mk->add(static_cast<float>(a + i), lv, static_cast<float>(wn));
  }
  for (int i = 0; i < n; i++) { const float v = dbv(so[i] + lin(B[i])); out[a + i] = v > B[i] ? v : B[i]; }
  return true;
}

}}  // namespace
