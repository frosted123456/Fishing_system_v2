// Pack hours on the board (D205): the counter of lib/IceMesh/src/pack_hours.h kept in NVS ("pack" / "s").
// Shared by the hub (lora_node) and the tip-up (sensor_node). Mark a new pack: hold the arming button while switching
// the pack on, until the LED blinks 3 times (~3 s), or type PACK NEW (serial or the bench page). PACK shows the count.
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include <pack_hours.h>

namespace packnvs {

inline icemesh::PackClock& clk() { static icemesh::PackClock c; return c; }

inline void save() {
  Preferences p;
  if (p.begin("pack", false)) { p.putULong("s", clk().seconds()); p.end(); }
  clk().markSaved();
}
inline void begin() {
  Preferences p; uint32_t s = 0;
  if (p.begin("pack", true)) { s = p.getULong("s", 0); p.end(); }
  clk().load(s, millis());
}
// from loop(): a flash write every 5 min of run time (never inside a ping: loop() runs the ping itself)
inline void tick() { if (clk().feed(millis())) save(); }
// before deep sleep: the awake time of this wake reaches flash
inline void flush() { clk().feed(millis()); if (clk().seconds() != clk().saved || clk().dirty) save(); }
inline void renew(const char* why) {
  clk().reset(millis()); save();
  Serial.printf("PACK: new pack (%s), hours back to 0\n", why);
}
inline const char* text() { static char b[12]; icemesh::PackClock::format(clk().seconds(), b, sizeof(b)); return b; }

// Power-on with the button held 3 s = new pack (LED blinks 3 times). Only on a real power-on (pack switch, USB plug,
// EN button), never after a crash or a deep-sleep wake. Released before 3 s: ordinary boot, nothing changes.
// keepAlive: called while waiting (hub: the task watchdog). Returns true when the count was reset.
inline bool bootGesture(int btnPin, int ledPin, void (*keepAlive)()) {
  if (btnPin < 0 || esp_reset_reason() != ESP_RST_POWERON) return false;
  pinMode(btnPin, INPUT_PULLUP); delay(5);
  if (digitalRead(btnPin) != LOW) return false;
  const unsigned long t0 = millis();
  while (digitalRead(btnPin) == LOW && millis() - t0 < 3000UL) { if (keepAlive) keepAlive(); delay(10); }
  if (millis() - t0 < 3000UL) return false;
  renew("button held at power-on");
  if (ledPin >= 0 && ledPin != btnPin) {
    pinMode(ledPin, OUTPUT);
    for (int i = 0; i < 3; i++) { digitalWrite(ledPin, HIGH); delay(150); digitalWrite(ledPin, LOW); delay(150); }
  }
  const unsigned long t1 = millis();   // wait for the release, so the arming button does not see this press
  while (digitalRead(btnPin) == LOW && millis() - t1 < 10000UL) { if (keepAlive) keepAlive(); delay(10); }
  delay(50);
  return true;
}

// "PACK" / "PACK NEW" from serial or the bench page. Returns true (and the reply) when the line was one of them.
inline bool command(const String& line, String& reply) {
  String U = line; U.trim(); U.toUpperCase();
  if (U == "PACK NEW") { renew("PACK NEW"); reply = "Pack hours reset to 0 (new pack)"; return true; }
  if (U == "PACK") {
    reply = String("Pack hours: ") + text() + " (" + String((unsigned long)clk().seconds()) + " s run on this pack; PACK NEW after a swap)";
    return true;
  }
  return false;
}

}  // namespace packnvs
