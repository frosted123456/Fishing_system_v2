// Phone test page for the sonar bench build (SONAR_BENCH=1, D56): the tip-up opens its own Wi-Fi access point
// ("IceFish-bench-<id>"), the phone browses http://192.168.4.1 and gets the live BENCH values, an A-scan of the
// last ping, the knobs, NOTE / RAW / record buttons, and downloads the recording as a text file in the SAME
// format as the serial "REC ON" lines (tools/sonar_replay.cpp reads it as is). No laptop needed outside.
// Recording is buffered in the tip-up's RAM (~32 kB) and pulled by the page about once a second: keep the page
// open and the phone screen on while recording (the page shows "lost" lines if it fell behind).
#pragma once
#include <Arduino.h>

#if defined(SONAR_BENCH) && SONAR_BENCH
namespace bench {

// command handler: the same command words as the serial port (KNOB, NOTE, RAW, REC WEB ON/OFF, REG, CAL, STAT,
// RECEVERY, RAWEVERY, INFO). Returns the reply text shown on the page.
typedef String (*CommandFn)(const String& cmd);

bool begin(uint8_t node_id, const char* password, uint8_t channel, CommandFn fn);   // AP + web server
void loop();                                   // serve the page (call between pings)
bool active();
void push(const char* line);                   // one recording line (no newline) into the RAM ring
void setStatus(const char* bench_line, const uint8_t* codes, uint16_t nbins, float bin_m, bool recording,
               uint32_t rec_lines, uint16_t rec_every, uint16_t raw_every_s);
uint32_t lastSeq();
unsigned long lastServedMs();                  // millis() of the last HTTP reply (radio-quiet guard)

}  // namespace bench
#else   // not a bench build: no access point, no web server linked in
namespace bench {
typedef String (*CommandFn)(const String& cmd);
inline bool begin(uint8_t, const char*, uint8_t, CommandFn) { return false; }
inline void loop() {}
inline bool active() { return false; }
inline void push(const char*) {}
inline void setStatus(const char*, const uint8_t*, uint16_t, float, bool, uint32_t, uint16_t, uint16_t) {}
inline uint32_t lastSeq() { return 0; }
inline unsigned long lastServedMs() { return 0; }
}  // namespace bench
#endif
