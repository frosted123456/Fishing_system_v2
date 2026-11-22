// D116: the last steady bottom, kept in NVS ("sonar"/"bot", cm) so a restart in the same hole starts from it.
// SonarProc::seedBottom() then holds it from the first ping when there is an echo there (D115 gate), so a lure, a
// fish or the test ball in the beam at start-up cannot become the bottom. A wrong seed (another hole) costs 8 pings.
// Written only when the bottom has held within 2 cm for 240 pings (1 min at 4/s) and differs from the stored value by
// more than 3 cm: a few writes per outing, not per ping.
#pragma once
#include <Arduino.h>
#include <Preferences.h>

namespace bottommem {
inline float load() {
  Preferences p; p.begin("sonar", true); const uint16_t cm = p.getUShort("bot", 0); p.end();
  return cm >= 10 && cm < 1220 ? cm / 100.0f : 0.0f;
}
inline void tick(bool have, float bottom_m) {
  static float ref = 0, saved = -1; static uint16_t steady = 0;
  if (saved < 0) saved = load();
  if (!have || bottom_m <= 0.1f) { steady = 0; return; }
  if (fabsf(bottom_m - ref) > 0.02f) { ref = bottom_m; steady = 0; return; }
  if (steady < 240) { steady++; return; }
  if (fabsf(ref - saved) > 0.03f) {
    Preferences p; p.begin("sonar", false); p.putUShort("bot", (uint16_t)(ref * 100.0f + 0.5f)); p.end();
    saved = ref;
  }
}
}  // namespace bottommem
