// Phone test page for the sonar bench builds (SONAR_BENCH=1, D56): the live BENCH values, an A-scan of the last
// ping, the knobs, NOTE / RAW / record buttons, and the recording downloaded as a text file in the SAME format as
// the serial "REC ON" lines (tools/sonar_replay.cpp reads it as is). No laptop needed outside.
// Two ways to serve it (D87): begin() opens its own access point "IceFish-bench-<id>" with its own web server at
// http://192.168.4.1 (WROOM tip-up, which has no server otherwise); attach() adds the page under a prefix of an
// existing WebServer (Heltec hub: its own hotspot and server, page at http://192.168.4.1/bench/).
// Recording is buffered in RAM (~32 kB) and pulled by the page about once a second: keep the page open and the
// phone screen on while recording (the page shows "lost" lines if it fell behind).
#pragma once
#include <Arduino.h>
class WebServer;

#if defined(SONAR_BENCH) && SONAR_BENCH
namespace bench {

// command handler: the same command words as the serial port (KNOB, NOTE, RAW, REC WEB ON/OFF, REG, CAL, STAT,
// RECEVERY, RAWEVERY, INFO). Returns the reply text shown on the page.
typedef String (*CommandFn)(const String& cmd);

bool begin(uint8_t node_id, const char* password, uint8_t channel, CommandFn fn);   // own AP + web server, page at /
bool attach(WebServer& srv, const char* prefix, uint8_t node_id, CommandFn fn);   // D87: page at <prefix>/ of an existing server
void loop();                                   // serve the page (begin() mode only; attach() mode: the owner's handleClient)
bool active();
void push(const char* line);                   // one recording line (no newline) into the RAM ring
void setStatus(const char* bench_line, const uint8_t* codes, uint16_t nbins, float bin_m, bool recording,
               uint32_t rec_lines, uint16_t rec_every, uint16_t raw_every_s, uint16_t vbat_mv);
uint32_t lastSeq();
unsigned long lastServedMs();                  // millis() of the last HTTP reply (radio-quiet guard)

}  // namespace bench
#else   // not a bench build: no access point, no web server linked in
namespace bench {
typedef String (*CommandFn)(const String& cmd);
inline bool begin(uint8_t, const char*, uint8_t, CommandFn) { return false; }
inline bool attach(WebServer&, const char*, uint8_t, CommandFn) { return false; }
inline void loop() {}
inline bool active() { return false; }
inline void push(const char*) {}
inline void setStatus(const char*, const uint8_t*, uint16_t, float, bool, uint32_t, uint16_t, uint16_t, uint16_t) {}
inline uint32_t lastSeq() { return 0; }
inline unsigned long lastServedMs() { return 0; }
}  // namespace bench
#endif
