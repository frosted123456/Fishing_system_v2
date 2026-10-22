// D80 "Follow the action" (chalet): the watched (focus) hole follows the sonar activity across the holes, from the
// summaries every hole already sends (fish count, activity 0-15, fish near the bait). No Arduino: host-tested
// (test/test_follow_action).
//
// Rules, in order:
//   - off: nothing happens (the user picks the hole by hand, as before)
//   - a manual pick (or "stop") pins that choice for PIN_MS, then following resumes
//   - FISH ON: the tripped hole is watched for TRIP_MS, whatever else is going on
//   - otherwise the busiest hole wins, if it is busy enough (MIN_SCORE), the current one has been watched for at
//     least DWELL_MS, and it is clearly busier (MARGIN) — or the current hole went quiet. Nothing going on
//     anywhere: the view stays where it is.
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>

namespace icemesh {
namespace sonar {

struct FollowIn {
  uint8_t node;
  uint8_t activity;     // Summary.activity: pings with a fish over the last window, 0-15
  uint8_t n_fish;       // Summary.n_targets (the bait is not counted)
  bool near_bait;       // Summary.status & ST_NEAR_BAIT
  uint16_t age_s;       // seconds since its last summary (D81: was frames; the frame is 0.5-2 s now)
};

class FollowAction {
 public:
  enum : uint32_t { DWELL_MS = 20000, PIN_MS = 300000, TRIP_MS = 120000 };
  enum : int { MARGIN = 5, MIN_SCORE = 4, QUIET_SCORE = 2, MAX_AGE = 10 };   // MAX_AGE in seconds (a quiet hole sends every 8 s)
  enum : uint8_t { EXTRAS = 2 };   // D81 (5.4): more holes streamed beside the watched one, when the radio has room

  bool on = true;
  uint8_t focus = 0;    // current pick (0 = none)
  uint8_t extra[EXTRAS] = {0, 0};   // D81: next busiest holes (0 = none); re-ranked every DWELL_MS, never the watched hole
  uint8_t max_extra = 1;            // D82: holes streamed beside the watched one (0-2; the user picks 1, 2 or 3 holes in all)
  char why[28] = "";    // short reason for the page / OLED

  static int score(const FollowIn& c) {
    if (c.age_s > MAX_AGE) return 0;   // stale summary: the hole may be gone
    const int fish = c.n_fish > 3 ? 3 : c.n_fish;
    return c.activity + 4 * fish + (c.near_bait ? 10 : 0);
  }

  void setMaxExtra(uint8_t n) { max_extra = n > EXTRAS ? static_cast<uint8_t>(EXTRAS) : n; extra_since_ = 0; }   // ranks again at once
  void setOn(bool v, uint32_t now) { on = v; pin_until_ = 0; trip_until_ = 0; since_ = now; if (!v) { extra[0] = extra[1] = 0; } }
  // The user picked a hole (0 = stopped the stream): keep it for PIN_MS.
  void manual(uint8_t node, uint32_t now) {
    focus = node; since_ = now; trip_until_ = 0;
    pin_until_ = now + PIN_MS; if (pin_until_ == 0) pin_until_ = 1;
    snprintf(why, sizeof(why), "%s", node ? "picked by hand" : "stopped by hand");
  }
  // FISH ON on a hole: watch it now.
  void trip(uint8_t node, uint32_t now) {
    if (!on || node == 0) return;
    focus = node; since_ = now; pin_until_ = 0;
    trip_until_ = now + TRIP_MS; if (trip_until_ == 0) trip_until_ = 1;
    snprintf(why, sizeof(why), "FISH ON");
  }
  uint32_t pinLeftMs(uint32_t now) const { return pin_until_ && (int32_t)(pin_until_ - now) > 0 ? pin_until_ - now : 0; }
  uint32_t tripLeftMs(uint32_t now) const { return trip_until_ && (int32_t)(trip_until_ - now) > 0 ? trip_until_ - now : 0; }

  // Call about once a second with every hole that has a summary. True when `focus` changed.
  bool update(const FollowIn* c, uint8_t n, uint32_t now) {
    if (!on) return false;
    const uint8_t was = focus;
    const bool changed = updatePrimary(c, n, now);
    if (focus != was) extra_since_ = 0;   // the watched hole moved: rank the extras again now
    updateExtras(c, n, now);
    return changed;
  }
  bool updatePrimary(const FollowIn* c, uint8_t n, uint32_t now) {
    if (pin_until_) { if ((int32_t)(now - pin_until_) < 0) return false; pin_until_ = 0; since_ = now - DWELL_MS; }
    if (trip_until_) { if ((int32_t)(now - trip_until_) < 0) return false; trip_until_ = 0; }
    int best = -1, best_score = 0, cur_score = 0;
    for (uint8_t i = 0; i < n; i++) {
      const int s = score(c[i]);
      if (c[i].node == focus) cur_score = s;
      if (s > best_score || (s == best_score && best >= 0 && c[i].node == focus)) { best = i; best_score = s; }
    }
    if (best < 0 || best_score < MIN_SCORE) return false;          // nothing going on: stay
    if (c[best].node == focus) { describe(c[best]); return false; }
    if (focus != 0 && now - since_ < DWELL_MS) return false;        // give the current hole its time
    if (focus != 0 && best_score < cur_score + MARGIN && cur_score >= QUIET_SCORE) return false;
    focus = c[best].node; since_ = now;
    describe(c[best]);
    return true;
  }

 private:
  void describe(const FollowIn& c) {
    if (c.near_bait) snprintf(why, sizeof(why), "fish near the bait");
    else if (c.n_fish) snprintf(why, sizeof(why), "%u fish under it", c.n_fish);
    else snprintf(why, sizeof(why), "activity");
  }
  // D81 (5.4): the extras follow the fish too, with their own dwell: re-ranked every DWELL_MS, kept while they stay
  // busy enough (QUIET_SCORE) and nothing clearly busier (MARGIN) waits. Follow on only; a manual pick does not pin them.
  void updateExtras(const FollowIn* c, uint8_t n, uint32_t now) {
    for (uint8_t k = 0; k < EXTRAS; k++) if (extra[k] == focus || k >= max_extra) extra[k] = 0;
    if (max_extra == 0) return;
    if (extra_since_ != 0 && now - extra_since_ < DWELL_MS) return;
    extra_since_ = now ? now : 1;
    // D82: the user asked for N holes, so the places are filled even when nothing is going on (the busiest first);
    // an extra leaves only when its summaries stop (gone) or a clearly busier hole waits
    int sc[EXTRAS]; for (uint8_t k = 0; k < EXTRAS; k++) sc[k] = -1;
    for (uint8_t i = 0; i < n; i++) for (uint8_t k = 0; k < EXTRAS; k++) if (extra[k] && c[i].node == extra[k] && c[i].age_s <= MAX_AGE) sc[k] = score(c[i]);
    for (uint8_t k = 0; k < EXTRAS; k++) if (extra[k] && sc[k] < 0) extra[k] = 0;   // gone
    for (;;) {   // best remaining candidate beats the weakest kept extra by MARGIN (or fills an empty place)
      int best = -1, bs = -1;
      for (uint8_t i = 0; i < n; i++) {
        if (c[i].node == focus || c[i].node == 0 || c[i].age_s > MAX_AGE) continue;
        bool kept = false; for (uint8_t k = 0; k < EXTRAS; k++) if (extra[k] == c[i].node) kept = true;
        if (kept) continue;
        const int sv = score(c[i]);
        if (sv > bs) { bs = sv; best = i; }
      }
      if (best < 0) break;
      int w = -1;
      const uint8_t lim = max_extra > static_cast<uint8_t>(EXTRAS) ? static_cast<uint8_t>(EXTRAS) : max_extra;
      for (uint8_t k = 0; k < lim; k++) if (extra[k] == 0) { w = k; break; }
      if (w < 0) { w = 0; for (uint8_t k = 1; k < lim; k++) if (sc[k] < sc[w]) w = k; if (bs < sc[w] + MARGIN) break; }
      extra[w] = c[best].node; sc[w] = bs;
    }
  }
  uint32_t since_ = 0, pin_until_ = 0, trip_until_ = 0, extra_since_ = 0;
};

}  // namespace sonar
}  // namespace icemesh
