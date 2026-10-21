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
  uint16_t age_frames;  // frames since its last summary
};

class FollowAction {
 public:
  enum : uint32_t { DWELL_MS = 20000, PIN_MS = 300000, TRIP_MS = 120000 };
  enum : int { MARGIN = 5, MIN_SCORE = 4, QUIET_SCORE = 2, MAX_AGE = 10 };

  bool on = true;
  uint8_t focus = 0;    // current pick (0 = none)
  char why[28] = "";    // short reason for the page / OLED

  static int score(const FollowIn& c) {
    if (c.age_frames > MAX_AGE) return 0;   // stale summary: the hole may be gone
    const int fish = c.n_fish > 3 ? 3 : c.n_fish;
    return c.activity + 4 * fish + (c.near_bait ? 10 : 0);
  }

  void setOn(bool v, uint32_t now) { on = v; pin_until_ = 0; trip_until_ = 0; since_ = now; }
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
  uint32_t since_ = 0, pin_until_ = 0, trip_until_ = 0;
};

}  // namespace sonar
}  // namespace icemesh
