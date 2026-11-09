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
#include "sonar_params.h"
#ifdef SONAR_DEBUG_BOTTOM
#include <cstdio>   // host replay only (tools/sonar_chain.cpp): the bottom decision per ping on stderr
#endif

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
    sreal bottom_snr;        // D43: bottom echo peak above the noise floor (dB): level check at setup
    sreal ring_m, ring_base_m;   // D43: ring-down length (m, from the transducer) and its learned normal value
    bool ring_alarm;         // D43: ring-down much longer than normal for a while (slush / frazil ice on the face?)
    uint8_t hard;
    uint16_t spikes;         // D57: samples replaced by the cross-ping spike filter in this ping (interference)
    uint16_t long_echoes;    // D57: peaks dropped by the echo-length test in this ping
    uint8_t n;               // tracks with age >= 3 (prototype output), in tracker order
    TrackOut t[MAX_TRACKS];
  };

  sreal bait_m = R(4.57);   // bait (lure) depth: prototype constant; real system: set per hole, <= 0 = no bait
                            // known (no "Bait" label, no near-bait / cover flags)
  Params prm;               // knobs (sonar_params.h); defaults = prototype
  sreal pulse_m = R(0.0561); // D57: ping length in range (c * cycles / f / 2), set by the caller per ping
  sreal dt = R(0.25);       // s between processed pings (prototype 4/s); the caller sets it when the rate changes
  sreal gateScale() const { const sreal k = dt / sp::DT(); return k < R(1) ? R(1) : (k > R(8) ? R(8) : k); }   // D91: 1 at 4 pings/s, 4 at 1 ping/s
  enum { RING_LEARN = 40, RING_ALARM_PINGS = 8, COVER_PINGS = 240, EARLY_COVER_PINGS = 12 };   // D89: a still near-bottom echo is "cover" after 240 pings (1 min at 4/s), not 30
  static sreal ringAlarmExtra() { return R(0.15); }   // m longer than normal (est., re-check on real ice)
  static sreal midwaterCoverS() { return R(600); }   // D124: a still mid-water echo becomes structure after 10 min
  // D103: the "near bottom" band (m above the detected bottom; a still echo inside it is cover after COVER_PINGS).
  // Floor 0.2 m = the bottom echo's own extent above its onset (pulse 0.06-0.11 m at 16-32 cycles + onset +-2 bins
  // + hard-bottom tail: the drum's bottom-tail track sits 0.16 m above the floor); slope 0.3 x depth = the room a
  // sloped bottom (range spread ~2 d tan(beam) sin(slope) ~ 0.1 d) and bottom-hugging cover take under the beam
  // footprint; cap 0.5 m = the D89 structure (weeds, rocks) margin. Any ice-fishing depth (> 1.7 m) keeps 0.5 m; a
  // 0.7-0.9 m drum gets 0.2-0.27 m, so a ball hung 0.25-0.45 m over the floor is a FISH there as it would be on the
  // lake (sent on every ping, never cover) instead of "near bottom" (cover after 12 / 240 pings, then never sent).
  // The page's labelOf() (web/suite.html) applies the same band.
  // D116: start from a bottom known from before (the firmware keeps the last steady one in NVS: same hole after a restart).
  // The gate (D115) then holds it from the first ping when there is an echo there; 8 pings without one = free search.
  // So a lure in the water at start-up cannot become the bottom, and a wrong seed (another hole) costs 2 s.
  void seedBottom(sreal m) { if (m > R(0.1)) { bottom_ = m; have_bottom_ = true; gate_miss_ = 0; esc_ = 0; jump_ = false; } }
  bool hasBottom() const { return have_bottom_; }
  sreal lockedBottom() const { return bottom_; }
  static sreal nearBottomBand(sreal bottom) {
    const sreal b = R(0.3) * bottom;
    return b < R(0.2) ? R(0.2) : (b > R(0.5) ? R(0.5) : b);
  }

  SonarProc() { reset(); }
  void reset() {
    have_bottom_ = have_nf_ = have_hi_ = have_ratio_ = have_tail_ = have_static_ = false;
    bottom_ = nf_ = hi_ = ratio_ = tail_ = R(0); bj_ = 0; gate_miss_ = 0; esc_ = 0; jump_ = false; n_tracks_ = 0; nid_ = 1; ping_n_ = 0; bait_id_ = 0;
    ring_n_ = 0; ring_run_ = 0; ring_base_ = R(0); rot_mask_ = 0; hist_n_ = 0;
  }

  // codes[f][i]: f = 0..nfreq-1 (3 = 190/200/210 kHz compounding, 1 = single frequency)
  void step(const uint8_t codes[sp::NFREQ][BINS], uint8_t nfreq, Out& out) {
    using namespace sp;
    Scratch& S = scratch();
    ping_n_++;
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

    // D57 cross-ping spike filter (Ryan et al. 2015 "impulse noise", look-back only, no display delay): another
    // sonar's ping (with our jitter it lands at a new depth every time) stands far above everything at that depth
    // +- one gate in the 2 previous pings. Replaced by the smaller of the 2 previous values. A real echo that just
    // appeared is held back two pings (the tracker confirms over 3 pings anyway).
    uint16_t spikes = 0;
    {
      uint8_t* raw = S.mask;   // scratch: this ping before filtering, as codes (history kept in codes: 2 x 488 B)
      for (int i = 0; i < N; i++) raw[i] = dbToCode(d[i]);
      if (prm[P_SPIKE_DB] && hist_n_ >= 2) {
        // D91: the gate knob is cm per 250 ms tick; at the idle rate (1 ping/s) a ping is 4 ticks, so the window
        // scales with dt. Before, W stayed 32 cm per PING: at 1 ping/s anything moving faster than 0.32 m/s (a jigged
        // lure, a cruising fish, the spoon in the drum) was "another sonar's ping" and ERASED before any detection.
        const int W = std::max(1, roundi(R(prm[P_GATE_CM] / 100.0) * gateScale() / BIN_M()));
        const uint8_t X = static_cast<uint8_t>(std::min(255, roundi(R(prm[P_SPIKE_DB]) / Q())));
        for (int i = 0; i < N; i++) {
          // reference = what was there in BOTH previous pings (max over +-W bins, then the smaller of the 2 pings):
          // a real echo is in both, an earlier interference spike only in one
          // (+-W for the last ping, +-2W for the one before: a fish moving a gate per ping is still inside both)
          uint8_t m1 = 0, m2 = 0;
          for (int j = std::max(0, i - W); j <= std::min(N - 1, i + W); j++) m1 = std::max(m1, h1_[j]);
          for (int j = std::max(0, i - 2 * W); j <= std::min(N - 1, i + 2 * W); j++) m2 = std::max(m2, h2_[j]);
          const uint8_t ref = std::min(m1, m2);
          if (raw[i] > ref && raw[i] - ref > X) {
            d[i] = static_cast<float>(codeToDb(std::min(h1_[i], h2_[i])));
            lin[i] = static_cast<float>(db2p(d[i])); spikes++;
          }
        }
      }
      memcpy(h2_, h1_, BINS); memcpy(h1_, raw, BINS); if (hist_n_ < 2) hist_n_++;
    }

    // D90: where the ring-down ends on THIS ping: the first bin at the noise floor + 10 dB, or the first rise of 3 dB
    // over the previous 3 bins (an echo). The reference floor is the learned one, or on the first ping the mean of
    // the last 40 bins. The bottom search and the target window both start past it: with bmin / dead inside the
    // ring-down (Shallow and Bucket presets in the 0.8 m drum, 32-cycle ring-down to 0.55 m) the strongest window
    // WAS the ring-down itself: bottom = bmin, no water column left and nothing could ever be a target.
    int rEnd = 1;
    sreal floorRef = nf_;   // the floor as known so far: the learned one, or on the first ping the mean of the last 40 bins
    if (!have_nf_) { sreal w = R(0); for (int i = N - 40; i < N; i++) w += static_cast<sreal>(d[i]); floorRef = w / R(40); }
    {
      const sreal ref = floorRef;
      int rMax = std::min(N - 1, roundi(R(1.0) / BIN_M()));   // never past 1 m (measured ring-down 0.33-0.55 m): a bad floor estimate must not push the bottom search past a shallow bottom
      // D104: no cap at the locked bottom any more (D102 had rMax = bI - 6): it fed back. A first bottom found in the tail
      // (transducer in air or just dipped: nothing past the ring-down) pulled rEnd shallower, the next search started
      // inside the ring-down, found the ring-down, and the bottom locked at bmin for good (bench 07nov: 0.10 m, 64 dB
      // SNR, Bucket preset; Shallow crawled to 0.24 m then jumped to the multiple). The 6 dB rule below covers the slow edge.
      // D102: the tail is a monotonic decay, so an echo also ends it when it climbs 6 dB over the LOWEST level seen so
      // far (from bin 3): a floor edge that rises gradually (< 3 dB per bin: tilted face, wide echo) over a 19 V tail
      // that never reaches floor + 10 dB before the floor walked THROUGH the floor echo to the 1 m cap, and the bottom
      // search then started past the floor (replay of 07nov_29po with the tail 5 dB up and the edge spread over 8 bins).
      sreal lowest = R(1e30);
      while (rEnd < rMax && static_cast<sreal>(d[rEnd]) > ref + R(10)) {
        if (rEnd >= 3) {
          const sreal back = std::max(static_cast<sreal>(d[rEnd - 1]), std::max(static_cast<sreal>(d[rEnd - 2]), static_cast<sreal>(d[rEnd - 3])));
          if (static_cast<sreal>(d[rEnd]) > back + R(3)) break;
          if (static_cast<sreal>(d[rEnd]) > lowest + R(6)) break;
          if (static_cast<sreal>(d[rEnd]) < lowest) lowest = d[rEnd];
        }
        rEnd++;
      }
    }
    // Bottom: strongest 8-bin energy window, then walk back to the onset
    const int b0 = std::min(N - 10, std::max(roundi(R(prm[P_BOTTOM_MIN] / 10.0) / BIN_M()), rEnd));   // default 6 -> 0.6 m (prototype)
    // D104: optional deepest bottom (bmax, x0.1 m, 0 = whole range). A barrel or a bucket is the one place where the floor's
    // multiple can be MUCH stronger than the floor (07nov 11:57: floor -70 dB at 0.88 m, multiple -57 dB at 1.73 m, tilted
    // face): no rule can tell that from a ball hung at half depth 10-13 dB under the floor (33po ball file), but the user
    // knows the barrel is under 1.2 m. The Bucket preset sets it; Lake and Shallow leave it at 0.
    const int nB = prm[P_BOTTOM_MAX] > 0 ? std::min(N, std::max(b0 + 10, roundi(R(prm[P_BOTTOM_MAX] / 10.0) / BIN_M()) + 8)) : N;
    sreal e = R(0); int best = b0; sreal bestE = R(-1);
    for (int i = b0; i < b0 + 8; i++) e += static_cast<sreal>(lin[i]);
    for (int i = b0; i + 8 < nB; i++) { if (e > bestE) { bestE = e; best = i; } e += static_cast<sreal>(lin[i + 8]) - static_cast<sreal>(lin[i]); }
    // D102 (was D47 / D92 / D98): the bottom is the FIRST strong echo that has a multiple, not the strongest window.
    // In the plastic drum at 19 V the floor's second echo (surface -> floor -> surface -> floor, at 2D + h with h the
    // transducer's immersion, 0.15 m on 07nov_29po) comes back as strong as the first (-55 vs -55 dB) or stronger
    // (flat hard floor under a flat surface: no spreading loss to speak of), so the strongest window was the multiple
    // at 1.57 m, the floor at 0.72 m was reported as a fish, and the bottom flipped every few seconds. D47 could not
    // help: it only ran when the strongest window had no second echo (the drum has a third at 3D + 2h), it looked for
    // the multiple at 2D exactly (secondEcho was +-4 bins: 2D + h is outside) and the floor was 'halfOfBest' (D92).
    // Rule: candidates are the local 8-bin windows at or past the ring-down, within 12 dB of the strongest, with a real
    // peak (floor + 15 dB). The SHALLOWEST candidate whose 2x (+-6 bins) holds an echo window (floor + 10 dB, 6 dB over
    // the bins before it) is the bottom: the fundamental always comes first, a lure or a fish has no multiple at twice
    // its depth while a bottom does. When no candidate has a multiple (deep lake, 2x out of range, soft mud) the
    // strongest stays, as in the prototype. What D92's halfOfBest becomes: a target at HALF the bottom's depth (the ball
    // at 17 in over the 33 in floor, a lure at mid-water) has a 'multiple' that IS the bottom, so a found multiple m
    // counts only when it is not itself the fundamental: either the series goes on (third echo at 3D + 2h = 2m - D:
    // a target at D/2 has nothing at 3D/2, the drum floor always has its third) or the multiple is no stronger than
    // the candidate + 2 dB (a lake multiple loses on each extra bounce; the ball at half depth is 10-13 dB under the
    // floor). A target weaker than the bottom by more than 12 dB (the prototype's lure at 15 ft, D92's case) never
    // enters the candidate list at all. Not a test: "the multiple has no multiple of its own": the drum shows a 4x
    // (33po: -69 dB) stronger than its 3x (-73), so the echo at 2D does have one.
    if (bestE > R(0)) {
      const sreal echoMin = db2p(floorRef + R(10));   // D98: a multiple has to be a real echo: 10 dB over the floor per bin (15 broke the school-over-soft-mud scene)
      const sreal peakMin = floorRef + R(15);         // a candidate has to be a real echo too
      const sreal bandE = bestE / R(15.85);           // within 12 dB of the strongest window
      int pick = -1;
      sreal eW = R(0), ePrev = R(-1);
      for (int i = b0; i < b0 + 8; i++) eW += static_cast<sreal>(lin[i]);
      for (int i = b0; i + 8 < nB && pick < 0; i++) {
        const sreal eNext = i + 9 < nB ? eW + static_cast<sreal>(lin[i + 8]) - static_cast<sreal>(lin[i]) : R(-1);
        const bool local = eW >= ePrev && eW > eNext;   // one window per echo: the sliding energy's local maximum
        if (local && eW >= bandE && peakOf(d, i, N) >= peakMin) {
          const int m = secondEcho(lin, i, N, echoMin);   // start of the multiple's window, -1 = none
          if (m >= 0) {
            const sreal eM = winE(lin, m);
            const bool series = echoAt(lin, 2 * m - i, N, echoMin, 4) >= 0;                  // third echo at 3D + 2h
            const bool weaker = eM <= eW * R(1.58);   // a normal multiple: at most 2 dB over its fundamental
#ifdef SONAR_DEBUG_BOTTOM
            fprintf(stderr, "cand: i=%d (%.1f dB under best) multiple@%d (%.1f dB vs cand) third %s weaker %s\n", i, 10 * std::log10(bestE / eW), m, 10 * std::log10(eM / eW), series ? "yes" : "no", weaker ? "yes" : "no");
#endif
            if (series || weaker) pick = i;
          }
        }
        ePrev = eW; eW = eNext;
      }
#ifdef SONAR_DEBUG_BOTTOM
      fprintf(stderr, "bottom: best=%d (pk %.1f dB) pick=%d\n", best, (double)peakOf(d, best, N), pick);
#endif
      if (pick >= 0) best = pick;
    }
    // D115: bottom tracking gate (Echoview / Ping1D practice). Once a bottom is locked, the bottom is the strongest
    // echo window starting within +-0.1 m of it as long as there is a real one there (peak >= floor + 15 dB); the free search above
    // only takes over after 8 pings without one (2 s: the transducer moved, a new hole). A strong target anywhere else
    // (the ball at 0.50 m, 3 dB over a shadowed floor, became 'bottom' in replays; a fish school on the lake) can no
    // longer pull the bottom away while the floor is still there.
    const int freeBest = best;   // D116: what the free search (D102 / D104) says on this ping
    if (have_bottom_ && bestE > R(0)) {
      const int bL = roundi(bottom_ / BIN_M());
      const int g0 = std::max(b0, bL - 4), g1 = std::min(nB - 8, bL + 4);   // +-10 cm: the window starts at the onset, so it sits on the bottom echo
      int gb = -1; sreal gE = R(-1), ge = R(0);
      static sreal gw[9];   // window energies at g0..g1 (+-4 bins)
      if (g1 > g0) {
        for (int i = g0; i < g0 + 8; i++) ge += static_cast<sreal>(lin[i]);
        for (int i = g0; i <= g1; i++) { gw[i - g0] = ge; if (ge > gE) gE = ge; if (i + 8 < N) ge += static_cast<sreal>(lin[i + 8]) - static_cast<sreal>(lin[i]); }
        // the window that starts CLOSEST to the locked bottom among those within 10 dB of the gate's strongest: a strong
        // target just above the floor (in the gate, stronger than the shadowed floor) no longer wins the gate
        int bd = 99;
        for (int i = g0; i <= g1; i++) if (gw[i - g0] * R(10) >= gE && std::abs(i - bL) < bd) { bd = std::abs(i - bL); gb = i; }
      }
      if (gb >= 0 && peakOf(d, gb, N) >= floorRef + R(15)) { best = gb; gate_miss_ = 0; }
      else if (gate_miss_ < 255) gate_miss_++;
      if (gate_miss_ > 0 && gate_miss_ <= 8 && gb >= 0) best = gb;   // hold the gate a little longer (a fish or the ball shadowing the floor)
      // D116: nothing is below the bottom. If the free search keeps finding a real echo DEEPER than the lock (by more than
      // the echo's own length, 20 cm), not a multiple of it (2x / 3x), and at least 3 dB STRONGER than what the gate holds,
      // for 12 pings in a row (3 s), the lock was on something in the water (a lure or a fish at start-up, a still bait):
      // move to it now. A correct lock never sees that: deeper echoes are its multiples or weaker clutter.
      const bool deeper = freeBest > bL + 8;
      // (a multiple is weaker than its fundamental on a lake; one 10 dB STRONGER is the real bottom under a lure at half
      // depth, the bait set at mid-water: that case must escape too. A barrel's multiple can be stronger: bmax, D104)
      const sreal pFree = peakOf(d, freeBest, N), pLock = peakOf(d, best, N);
      const bool mult = (std::abs(freeBest - 2 * bL) <= 6 || std::abs(freeBest - 3 * bL) <= 8) && pFree <= pLock + R(10);
      if (best != freeBest && deeper && !mult && pFree >= pLock + R(3)) {
        if (++esc_ >= 12) { best = freeBest; esc_ = 0; gate_miss_ = 9; jump_ = true; }
      } else esc_ = 0;
    }
    int pk = best; for (int i = best; i < std::min(N, best + 8); i++) if (d[i] > d[pk]) pk = i;
    // D115: a window that starts past the echo's peak (the gate holding a lock that sits on the echo's tail) has its
    // maximum at its first bin: climb back to the real peak, or the onset walk below stops on the rising side at once
    while (pk > b0 && d[pk - 1] >= d[pk]) pk--;
    // D115: the walk back to the onset also stops where the level starts to RISE again (3 dB over the lowest so far):
    // a target sitting on the floor echo's leading edge (ball 6 cm above the floor) was walked into, and the bottom
    // jumped up to the target.
    int on = pk; { sreal lowest = static_cast<sreal>(d[pk]);
      while (on > b0 && static_cast<sreal>(d[on - 1]) > static_cast<sreal>(d[pk]) - R(10) && static_cast<sreal>(d[on - 1]) <= lowest + R(3)) { on--; if (static_cast<sreal>(d[on]) < lowest) lowest = d[on]; } }
    // D88: the onset must be a RISE, not a level. When the walk-back reaches the search start without ever
    // dropping 10 dB below the peak (drum: wall echoes keep the whole stretch above the bottom - 10 dB; lake:
    // a ring-down tail still high at bmin), the bottom was reported AT bmin. Take the steepest 2-bin rise
    // between the search start and the peak instead. Unchanged when the walk finds its floor (every scene test).
    if (on == b0 && static_cast<sreal>(d[b0]) > static_cast<sreal>(d[pk]) - R(10) && pk > b0 + 1) {
      sreal br = R(-1e30); int at = pk;
      for (int i = b0 + 1; i < pk; i++) {
        const sreal s = static_cast<sreal>(d[i + 1]) - static_cast<sreal>(d[i - 1]);
        if (s > br) { br = s; at = i; }
      }
      on = at;
    }
#ifdef SONAR_DEBUG_BOTTOM
    fprintf(stderr, "search: rEnd=%d b0=%d best=%d pk=%d on=%d (pk %.1f dB) ring=%.3f\n", rEnd, b0, best, pk, on, (double)d[pk], rEnd * 0.025);
#endif
    const sreal bNow = static_cast<sreal>(on) * BIN_M();
    // D104: no echo, no bottom. A ping whose best window peaks under floor + 15 dB (transducer in air, out of the water,
    // or the floor out of the beam) does not move the bottom: before, the first such ping seeded the bottom in the tail.
    const bool echoOk = static_cast<sreal>(d[pk]) >= floorRef + R(15);
    if (!echoOk) { /* keep the last bottom */ }
    else if (!have_bottom_ || jump_) { bottom_ = bNow; have_bottom_ = true; jump_ = false; bj_ = 0; }
    else if (std::fabs(bNow - bottom_) < R(0.3)) { bottom_ += R(prm[P_BOTTOM_SMOOTH] / 100.0) * (bNow - bottom_); bj_ = 0; }
    else if (++bj_ > 4) { bottom_ = bNow; bj_ = 0; }
    const int bI = roundi(bottom_ / BIN_M());

    // Noise floor: median of the quiet water below the bottom
    int a = bI + roundi(R(0.9) / BIN_M()), z = std::min(N, 2 * bI - roundi(R(0.4) / BIN_M()));
    int na = 0;
    if (z - a < 30 && bI + 61 < N) { a = N - 40; z = N; }   // shallow: the far range is quiet water (D92: 61 = echo 21 bins + the 40-bin window, so a bottom at 11.1-11.4 m takes the D47 path instead of its own echo)
    if (z - a < 30) {
      // D47: bottom near the end of the range (deep hole): no water below it in the record. Take the
      // quietest 40-bin stretch between the dead zone and the bottom instead of the bottom echo itself.
      const int lo = std::max(1, roundi(R(prm[P_DEADZONE_DM] / 10.0) / BIN_M())), hi = bI - 8;
      a = lo; z = std::min(N, lo + 40);
      if (hi - lo > 40) {
        sreal w = R(0); for (int i = lo; i < lo + 40; i++) w += static_cast<sreal>(d[i]);
        sreal wMin = w; int at = lo;
        for (int i = lo; i + 40 < hi; i++) { w += static_cast<sreal>(d[i + 40]) - static_cast<sreal>(d[i]); if (w < wMin) { wMin = w; at = i + 1; } }
        a = at; z = at + 40;
      }
    }
    for (int i = a; i < z && i < N; i++) S.arr[na++] = d[i];
    if (na == 0) { S.arr[0] = d[N - 1]; na = 1; }
    for (int i = 1; i < na; i++) { const float v = S.arr[i]; int j = i - 1; while (j >= 0 && S.arr[j] > v) { S.arr[j + 1] = S.arr[j]; j--; } S.arr[j + 1] = v; }
    const sreal med = S.arr[na >> 1];
    if (!have_nf_) { nf_ = med; have_nf_ = true; } else nf_ = nf_ + R(0.15) * (med - nf_);
    const sreal nf = nf_;

    // Range compensation and display scaling
    const bool tvg_on = prm[P_TVG] != 0;
    for (int i = 0; i < N; i++) c[i] = static_cast<float>(static_cast<sreal>(d[i]) + (tvg_on ? static_cast<sreal>(S.tvg[i]) : R(0)));
    sreal hiNow = R(-200);
    for (int i = bI; i < std::min(N, bI + 6); i++) hiNow = std::max(hiNow, static_cast<sreal>(c[i]));
    if (!have_hi_) { hi_ = hiNow; have_hi_ = true; } else hi_ = hi_ + R(0.1) * (hiNow - hi_);
    top_ = hi_ - R(12); lo_ = top_ - R(prm[P_RANGE_DB]);

    // Peaks with sub-bin fit
    sm[0] = c[0]; sm[N - 1] = c[N - 1];
    for (int i = 1; i < N - 1; i++) sm[i] = static_cast<float>((static_cast<sreal>(c[i - 1]) + static_cast<sreal>(c[i]) + static_cast<sreal>(c[i + 1])) / R(3));
    Peak* pk_ = S.peaks; int np = 0; uint16_t long_echoes = 0;
    // D127 knob det 1 (experimental): a peak is a fish candidate when its echo stands out from the LEARNED SCENE at
    // that depth (static_ / sig_: per-bin level and spread, learned without the tracked targets) by a matched-filter
    // statistic: the excess power over the scene summed under the hump of one target (power weights), divided by the
    // scene's own fluctuation under the same hump (constant false alarm rate, like radar CFAR). It replaces "snr over
    // the noise floor" + prominence, and searches from the blind-zone knob (the scene contains the ring-down tail, so
    // a fish inside the ring tail can be seen). Threshold = snr knob / 2 in dB of that ratio (10 -> 5 dB).
    // warm-up: the first 24 pings learn the scene fast and unmasked (ring-down tail, walls, weeds) with the classic
    // detector; without it the ring tail (not yet in the scene) became a permanent false target (empty barrel 08nov)
    const bool detWarm = prm[P_DET] == 1 && ping_n_ < 24;
    const bool det1 = prm[P_DET] == 1 && have_static_ && !detWarm;
    const int iDead = std::max(1, roundi(R(prm[P_DEADZONE_DM] / 10.0) / BIN_M()));
    const int i0 = det1 ? iDead : std::max(iDead, rEnd), i1 = bI - 5;   // D90: past the ring-down too (classic)
    float* mf = S.vpix;   // free until the display values below
    if (det1) {
      static const sreal HM[5] = {R(0.3364), R(0.7396), R(1), R(0.7744), R(0.3136)};   // hump^2 (power), 5 bins around its peak
      for (int i = 0; i < N; i++) mf[i] = -100.f;
      for (int i = std::max(2, i0 - 1); i < std::min(N - 2, i1 + 1); i++) {
        sreal A = R(0), Nn = R(0);
        for (int k = -2; k <= 2; k++) { const int j = i + k; const sreal ps = db2p(static_cast<sreal>(static_[j]));
          const sreal ex = static_cast<sreal>(lin[j]) - ps; A += HM[k + 2] * (ex > R(0) ? ex : R(0));
          Nn += HM[k + 2] * ps * (db2p(static_cast<sreal>(sig_[j])) - R(1)); }
        mf[i] = static_cast<float>(R(10) * std::log10(R(1) + A / std::max(Nn, R(1e-20))));
      }
    }
    for (int i = i0; i < i1; i++) {
      if (det1) {
        if (!(mf[i] >= mf[i - 1] && mf[i] > mf[i + 1])) continue;
        if (static_cast<sreal>(mf[i]) < R(prm[P_SNR_DB]) * R(0.5)) continue;
      } else {
      if (!(sm[i] >= sm[i - 1] && sm[i] > sm[i + 1])) continue;
      const sreal snr = static_cast<sreal>(d[i]) - nf; if (snr < R(prm[P_SNR_DB])) continue;
      sreal mn = R(1e30);
      for (int k = -6; k <= 6; k++) { const int j = i + k; if (j >= 0 && j < N && static_cast<sreal>(sm[j]) < mn) mn = sm[j]; }
      if (static_cast<sreal>(sm[i]) - mn < R(prm[P_PROM_DB])) continue;
      }
      const sreal A = sm[i - 1], B = sm[i], Cc = sm[i + 1], den = A - R(2) * B + Cc;
      const sreal dl = den < R(0) ? clampr(R(0.5) * (A - Cc) / den, R(-0.5), R(0.5)) : R(0);
      if (prm[P_ECHOLEN]) {   // D57 single-echo test (Echoview single-beam SED): echo length 6 dB below the peak
        const sreal top = static_cast<sreal>(d[i]) - R(6);
        // scan kept inside the search window: the ring-down (left) and the bottom echo (right) are not this echo
        int l6 = i, r6 = i;
        if (det1) {   // D127: a weak echo's -6 dB level is inside the noise: measure the excess over the scene instead
          auto exs = [&](int j) { return static_cast<sreal>(lin[j]) - db2p(static_cast<sreal>(static_[j])); };
          const sreal t4 = R(0.25) * exs(i);
          while (l6 > i0 && exs(l6 - 1) > t4) l6--;
          while (r6 + 1 < i1 && exs(r6 + 1) > t4) r6++;
        } else {
        while (l6 > i0 && static_cast<sreal>(d[l6 - 1]) > top) l6--;
        while (r6 + 1 < i1 && static_cast<sreal>(d[r6 + 1]) > top) r6++;
        }
        const sreal len = static_cast<sreal>(r6 - l6 + 1) * BIN_M();
        if (len > (pulse_m + BIN_M()) * R(prm[P_ECHOLEN] / 10.0)) { long_echoes++; continue; }
      }
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
      int bt = -1; sreal bd = R(prm[P_GATE_CM] / 100.0) * gateScale();   // D91: per tick, scaled to the ping interval
      for (int k = 0; k < n_tracks_; k++) {
        if (used[k]) continue;
        const Track& T = tracks_[k];
        const sreal dd = std::fabs(T.depth + T.vel * dt - p.depth);
        if (dd < bd) { bd = dd; bt = k; }
      }
      if (bt >= 0) {
        Track& T = tracks_[bt];
        T.vel = R(0.6) * T.vel + R(0.4) * (p.depth - T.depth) / dt; T.depth = p.depth; T.miss = 0; T.age++;
        T.push(p); used[bt] = true;
      } else {
        const uint32_t id = nid_++;
        if (n_tracks_ + nfresh < MAX_TRACKS) {
          Track& T = S.fresh[nfresh++];
          T.id = id; T.depth = p.depth; T.vel = R(0); T.age = 1; T.miss = 0; T.hn = 0; T.hh = 0; T.label = LBL_FISH; T.early = ping_n_ <= 2 ? 1 : 0;
          T.push(p);
        }
      }
    }
    for (int k = 0; k < n_tracks_; k++) if (!used[k]) tracks_[k].miss++;
    int w = 0;
    for (int k = 0; k < n_tracks_; k++) if (tracks_[k].miss <= prm[P_KEEP]) { if (w != k) tracks_[w] = tracks_[k]; w++; }
    for (int k = 0; k < nfresh && w < MAX_TRACKS; k++) tracks_[w++] = S.fresh[k];
    n_tracks_ = w;
    assignLabels();   // D92: one bait track with hysteresis, the rest by position

    // Static scene, learned everywhere except around tracked targets
    // D90: the water column of the first ping is NOT the static scene: anything there at start-up (weeds, or a fish, or
    // the test target already in the drum) starts as "changed" and is shown; a still echo becomes cover after
    // COVER_PINGS and is learned into the scene then. Before: static = the first ping, so an object present at
    // start-up could never be sent (near-bottom targets need binChanged) and only ever showed as green background.
    if (!have_static_) {
      // D97: the column starts at the blind-zone knob, not at this ping's ring-down end: on the 2026-11-05 ball test the
      // ring-end rule stopped at the ball's own layer (0.55-0.7 m) on the first ping, the layer went into the scene as
      // "ring-down", and the tracked ball was never sent (near-bottom targets go out only while they differ from the scene)
      const int c0 = std::max(1, roundi(R(prm[P_DEADZONE_DM] / 10.0) / BIN_M()));
      for (int i = 0; i < N; i++) { static_[i] = (i >= c0 && i < i1) ? static_cast<float>(std::min(static_cast<sreal>(d[i]), nf + R(3))) : d[i]; sig_[i] = 2.5f; }   // D90: 2.5 dB start (was 4): the "changed" gate then opens ~10 dB over the scene, like the snr knob over the floor
      have_static_ = true;
    }
    uint8_t* mask = S.mask; memset(mask, 0, N);
    for (int k = 0; k < n_tracks_ && !detWarm; k++) {   // D127: no mask while the det-1 scene warms up
      const Track& T = tracks_[k];
      if (T.age < 2 || T.label == LBL_COVER) continue;
      const int b = roundi(T.depth / BIN_M());
      for (int q = -5; q <= 8; q++) { const int j = b + q; if (j >= 0 && j < N) mask[j] = 1; }
    }
    for (int i = 0; i < N; i++) {
      const sreal dev = static_cast<sreal>(d[i]) - static_cast<sreal>(static_[i]);
      if (!mask[i]) {
        static_[i] = static_cast<float>(static_cast<sreal>(static_[i]) + (detWarm ? R(0.2) : R(prm[P_STATIC_LEARN] / 1000.0)) * dev);
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
    out.bottom_snr = static_cast<sreal>(d[bp]) - nf;
    out.spikes = spikes; out.long_echoes = long_echoes;

    // D43 ring-down monitor: how far below the transducer the echo stays well above the noise floor.
    // Its normal length is learned on the first pings, then slowly; much longer for a while = alarm
    // (slush / frazil ice on the face, transducer touching the ice). Does not change anything above.
    {
      // D88: the ring-down is a DECAY from time zero. It ends where the level stops falling: at the noise floor
      // + 10 dB, or at the first echo that stands 3 dB over the last 3 bins (drum: wall echoes from 0.25 m kept
      // the old walk going to 1.0-1.5 m and raised a RING ALARM; lake: a fish in the first metre ends it early,
      // which only shortens a monitor value).
      int i = 1; const int iMax = std::min(N - 1, roundi(R(1.5) / BIN_M()));
      while (i < iMax && static_cast<sreal>(d[i]) > nf + R(10)) {
        if (i >= 3) {
          const sreal back = std::max(static_cast<sreal>(d[i - 1]), std::max(static_cast<sreal>(d[i - 2]), static_cast<sreal>(d[i - 3])));
          if (static_cast<sreal>(d[i]) > back + R(3)) break;
        }
        i++;
      }
      const sreal rl = static_cast<sreal>(i) * BIN_M();
      if (ring_n_ < RING_LEARN) { ring_base_ = (ring_base_ * static_cast<sreal>(ring_n_) + rl) / static_cast<sreal>(ring_n_ + 1); ring_n_++; }
      const bool longer = ring_n_ >= RING_LEARN && rl > ring_base_ + ringAlarmExtra();
      ring_run_ = longer ? (ring_run_ < 255 ? ring_run_ + 1 : 255) : 0;
      if (ring_n_ >= RING_LEARN && !longer) ring_base_ = ring_base_ + R(0.01) * (rl - ring_base_);
      out.ring_m = rl; out.ring_base_m = ring_base_; out.ring_alarm = ring_run_ >= RING_ALARM_PINGS;
    }
    out.n = 0;
    for (int k = 0; k < n_tracks_; k++) {
      const Track& T = tracks_[k];
      if (T.age < prm[P_CONFIRM]) continue;
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

  // D43: one frequency per ping, rotating 190 / 200 / 210 kHz (f = 0..2): compounding over the last three
  // pings (until all three are seen, the missing ones repeat this ping). Moving fish smear over ~0.75 s.
  void stepRotating(const uint8_t codes[BINS], uint8_t f, Out& out) {
    if (f >= sp::NFREQ) f = 0;
    memcpy(rot_[f], codes, BINS);
    rot_mask_ = static_cast<uint8_t>(rot_mask_ | (1u << f));
    for (uint8_t k = 0; k < sp::NFREQ; k++) if (!(rot_mask_ & (1u << k))) memcpy(rot_[k], codes, BINS);
    step(rot_, sp::NFREQ, out);
  }

  // Per-bin results of the last step() of ANY SonarProc (shared scratch): read right after step().
  static const float* binValue() { return scratch().vpix; }    // live echo, prototype colour scale 0..1
  static const float* binDb() { return scratch().d; }          // D93: this ping in dB (after the spike filter), for the echogram line
  const float* binStaticDb() const { return static_; }        // the learned static scene in dB (diagnostics)
  const float* binSig() const { return sig_; }
  static const float* binStatic() { return scratch().bgv; }    // learned static scene (grey) 0..1
  static const float* binChanged() { return scratch().al; }    // 0..1: live echo differs from the static scene
  // D126f: 4 work arrays of BINS floats (lin, c, sm, dyn) lent between two step() calls (the next step overwrites them;
  // nothing reads them after step()). Used by the echogram sharpening so it needs no heap.
  static float* lendScratch(int k) { Scratch& S = scratch(); return k == 0 ? S.lin : k == 1 ? S.c : k == 2 ? S.sm : k == 3 ? S.dyn : S.arr; }   // D133: 4 = arr (the echo line built from several pings)

 private:
  struct Peak { sreal depth; int bin; sreal s, width; float f0, f2; sreal v; };
  struct Hist { sreal depth, s, width, v; float f0, f2; };
  struct Track {
    uint32_t id; sreal depth, vel; uint32_t age; uint8_t miss, hn, hh, label;
    uint8_t early;   // D92: born on one of the first 2 pings (was in the water at start-up: a rock or weed, cover soon)
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
    static Scratch* s = nullptr;   // D99: points at a static block (no heap); the TVG table is filled on first use
    if (s == nullptr) {
      static Scratch fixed;   // D99: 36 kB in .bss instead of a `new` at the first ping (which aborted a hub with ~35 kB of heap left; no fragmentation either)
      s = &fixed;
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

  // the multiple of a bottom echo: an 8-bin echo window around 2x the window start. A lake multiple is 15-35 dB weaker
  // than the first echo (prototype: hardness from the same ratio), the drum's is as strong; anything 6 dB over the 8 bins
  // before it and above `echoMin` per bin counts. -1 when 2x is out of range (then nothing can be checked).
  static sreal peakOf(const float* d, int i, int N) { sreal m = R(-1000); for (int k = i; k < i + 8 && k < N; k++) if (static_cast<sreal>(d[k]) > m) m = d[k]; return m; }
  static sreal winE(const float* lin, int s) { sreal e = R(0); for (int k = 0; k < 8; k++) e += static_cast<sreal>(lin[s + k]); return e; }
  // D102: the start of the strongest 8-bin echo window within +-span bins of `t`, -1 when there is none or `t` is out of
  // range. An echo window averages above `echoMin` (D98: floor + 10 dB per bin: before, any 6 dB rise over the 8 bins
  // just before it counted, so with a tilted transducer a noise blip at 2x promoted a clutter band at 1.9 m to 'bottom'
  // and the real floor at 0.85 m was then reported as a fish) and stands 6 dB over the 8 bins ending 4 bins before it.
  static int echoAt(const float* lin, int t, int N, sreal echoMin, int span) {
    int at = -1; sreal eAt = R(-1);
    if (t - span - 12 < 0 || t + span + 8 > N) return -1;
    for (int s = t - span; s <= t + span; s++) {
      const sreal e = winE(lin, s);
      if (e <= eAt) continue;
      sreal around = R(0); for (int k = s - 12; k < s - 4; k++) around += static_cast<sreal>(lin[k]);
      if (e > around * R(4) && e > echoMin * R(8)) { eAt = e; at = s; }
    }
    return at;
  }
  // D102: the multiple of the window at `i` sits at 2D + h (h = the transducer's immersion, 0..0.15 m), so +-6 bins
  // around 2i (D47/D98 looked at 2i - 4 .. 2i + 3 only and missed the drum's multiple at 2D + 6 bins). It must also stay
  // within 40 dB of the window (the prototype's hardness range).
  static int secondEcho(const float* lin, int i, int N, sreal echoMin) {
    if (i < 8) return -1;
    const int m = echoAt(lin, 2 * i, N, echoMin, 6);
    return m >= 0 && winE(lin, m) > winE(lin, i) * R(1e-4) ? m : -1;
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
  // D92: labels. The bait is ONE track: the one already labelled bait keeps the label while it lives and stays within
  // 0.5 m of the bait depth (a jig stroke does not turn the lure into a fish); when there is none, the track nearest
  // the bait depth inside 0.32 m, after 4 pings, becomes it. Every other track in the bait window is a FISH (before:
  // any track within 0.32 m was "the bait", so a fish arriving AT the lure was relabelled bait and the near-bait alarm
  // went quiet at the moment it mattered). Near-bottom: a still echo is cover after COVER_PINGS (D89), or after 12
  // pings when the track was there from the first pings (a rock or weed top at start-up: it comes back as a target
  // the moment it moves, since its old spot is then in the learned scene and the new one is "changed").
  void assignLabels() {
    int bait = -1;
    if (bait_m > R(0)) {
      for (int k = 0; k < n_tracks_; k++) if (bait_id_ && tracks_[k].id == bait_id_ && std::fabs(tracks_[k].depth - bait_m) < R(0.5)) bait = k;
      if (bait < 0) {
        sreal best = R(0.32);
        for (int k = 0; k < n_tracks_; k++) {
          const Track& T = tracks_[k];
          const sreal dd = std::fabs(T.depth - bait_m);
          if (dd < best && T.age >= 4 && T.hn >= 4 && spread(T) < R(2.5)) { best = dd; bait = k; }
        }
      }
    }
    bait_id_ = bait >= 0 ? tracks_[bait].id : 0;
    for (int k = 0; k < n_tracks_; k++) tracks_[k].label = k == bait ? static_cast<uint8_t>(LBL_BAIT) : labelOf(tracks_[k]);
  }
  uint8_t labelOf(const Track& T) const {
    const int n = T.hn < 30 ? T.hn : 30;
    sreal lo = R(1e30), hi = R(-1e30);
    for (int k = 0; k < n; k++) { const sreal dd = T.back(k).depth; if (dd < lo) lo = dd; if (dd > hi) hi = dd; }
    if (T.depth > bottom_ - nearBottomBand(bottom_)) {   // D103: 0.5 m on a lake, 0.2-0.27 m in a drum
      const uint32_t still_for = T.early ? EARLY_COVER_PINGS : COVER_PINGS;
      return (T.age > still_for && hi - lo < R(0.06)) ? LBL_COVER : LBL_NEAR_BOTTOM;
    }
    // D105: a mid-water echo that has not moved (3 cm) NOR changed strength (2 dB) over the last 30 pings, after a minute,
    // is still structure, not a fish: a barrel rib at 0.42 m (07nov 12:24/12:28, there with and without the ball, the same
    // in the 33 in no-ball file) stayed "Fish 1.4 ft" for good; on a lake a branch, a stuck line or a sinker with no bait
    // depth set does the same. A fish's echo flickers several dB from ping to ping (tilt, fins) even when it holds its
    // depth; this one varied 0.3 dB. Reversible: it is a fish again the moment it moves or flickers.
    // D124: after 10 minutes (was COVER_PINGS = 1 min at 4/s): under the ice a fish can hover still by the lure for a
    // minute or more; commercial sounders never relabel a still mid-water echo. Time, not pings (1-4 pings/s).
    if (n >= 30 && static_cast<sreal>(T.age) * dt > midwaterCoverS() && hi - lo < R(0.03)) {
      sreal sl = R(1e30), sh = R(-1e30);
      for (int k = 0; k < n; k++) { const sreal v = T.back(k).s; if (v < sl) sl = v; if (v > sh) sh = v; }
      if (sh - sl < R(2)) return LBL_COVER;
    }
    return LBL_FISH;
  }

  bool have_bottom_, have_nf_, have_hi_, have_ratio_, have_tail_, have_static_;
  sreal bottom_, nf_, hi_, ratio_, tail_, top_ = 0, lo_ = 0;
  int bj_;
  uint8_t gate_miss_ = 0;   // D115: pings without an echo in the bottom gate
  uint8_t esc_ = 0; bool jump_ = false;   // D116: pings the free search saw a stronger echo below the lock; re-lock now
  Track tracks_[MAX_TRACKS];
  int n_tracks_;
  uint32_t nid_;
  uint32_t ping_n_;         // D92: pings since reset()
  uint32_t bait_id_;        // D92: the track that IS the bait (0 = none)
  float static_[BINS], sig_[BINS];
  uint16_t ring_n_; uint8_t ring_run_; sreal ring_base_;
  uint8_t rot_[sp::NFREQ][BINS]; uint8_t rot_mask_;
  uint8_t h1_[BINS], h2_[BINS]; uint8_t hist_n_;   // D57 spike filter: the 2 previous pings (codes, unfiltered)
  static uint8_t dbToCode(float db) {
    const sreal c = (static_cast<sreal>(db) + R(100)) / sp::Q();
    return static_cast<uint8_t>(c < R(0) ? 0 : (c > R(255) ? 255 : static_cast<int>(c + R(0.5))));
  }
};

}  // namespace sonar
}  // namespace icemesh
#endif
