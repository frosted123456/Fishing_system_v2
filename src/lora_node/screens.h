// v2 OLED screens (128x64, Heltec V3) — chalet and hubs. Pure drawing from a ScreenModel, with the U8g2
// C API, so the exact same code renders on the PC (tools/screens_mock.cpp → PNG mockups) and on the board.
// Design: docs/SCREENS.md. One button: short press = next page (silence during an alert).
// CardKB: Esc = menu (list of pages, up/down + Enter), up/down = inside the page, Enter = OK, left/right = page.
#pragma once
#include <stdint.h>
#ifdef ARDUINO
#include <U8g2lib.h>
#else
#include "u8g2.h"
#endif

enum ScrHoleState : uint8_t { SH_OK = 0, SH_FISH = 1, SH_OFFLINE = 2, SH_FAULT = 3, SH_LOWBAT = 4 };
enum ScrPage : uint8_t { PG_HOME = 0, PG_SONAR, PG_FOCUS, PG_HOLES, PG_NETWORK, PG_TEST, PG_CONNECT, PG_OPTIONS, PG_COUNT };

// Sonar at a glance (from the BASE block of every hole): bottom + every target, not just the nearest
struct ScrTarget { uint16_t depth_cm; uint8_t level; bool bait; };   // level 1-3
struct ScrSonar { bool valid; uint16_t bottom_cm; uint8_t hard; uint8_t n; ScrTarget t[5]; uint8_t activity;
                  uint8_t status; uint8_t bottom_snr; };   // D43: status 1 = ring-down long (ice?), 2 = fish near bait, 4 = bait in cover; bottom echo dB
// FOCUS hole history (one column per ping)
struct ScrPingCol { uint16_t bottom_cm; uint8_t n; uint16_t d[5]; uint8_t lv[5]; };

struct ScrHole {
  uint8_t id, hub;           // hub = pocket (owner hub ID; the hub's own hole has hub == id)
  char name[16];             // "" = "Hole <id>"
  uint8_t state;             // ScrHoleState
  uint8_t batt;              // %, 255 = unknown
  int8_t fish;               // sonar fish count, -1 = no sonar
  uint32_t since_s;          // FISH: seconds since the trip
  ScrSonar son;              // valid = this hole has a sonar
  uint8_t sim;               // simulation on this hole: bit0 sonar, bit1 Hall sensor (trips)
};

struct ScrHubLink { uint8_t id; int8_t lora_rssi; bool lora_ok, eb_ok; uint8_t eb_hops; bool demo; };   // demo = fake hub inside the chalet

// Options page: the list (and its text entry / yes-no) is decided by the firmware, drawn here.
struct ScrRow { char label[20]; char value[18]; bool sub; };   // sub = opens a list (">")
enum ScrOptMode : uint8_t { SO_LIST = 0, SO_TEXT, SO_CONFIRM };
struct ScrOptions {
  uint8_t mode;              // ScrOptMode
  char title[22];
  uint8_t n, sel; ScrRow rows[56];
  char hint[30];             // bottom line
  // SO_TEXT: e.g. title "Wi-Fi password", line "for BELL494", text being typed
  // SO_CONFIRM: title = question, line = detail
  char line[34]; char text[66];
};

struct ScreenModel {
  bool chalet;               // false = hub
  uint8_t self_id;
  uint8_t n_holes; ScrHole holes[48];
  uint8_t n_hubs; ScrHubLink hubs[10];
  uint32_t setup_s;          // > 0: setup phase, seconds until armed
  uint8_t lora_ch;           // 1-8
  uint16_t lora_mhz_x10;     // 9150
  bool ch_auto;
  uint8_t transport;         // 0 Auto, 1 LoRa, 2 ESP-NOW
  uint8_t busy_pct;          // activity on the channel in use, 255 unknown
  bool silenced; uint32_t silence_s;
  // hub only
  bool master_heard; int8_t beacon_rssi; bool via_echo, on_backup;
  bool hotspot; uint32_t hotspot_min;
  // connect
  char ssid[28]; char pass[20]; char url[32]; bool sta;   // sta = on the cabin Wi-Fi
  char sta_ssid[33]; char sta_state[20];                  // cabin Wi-Fi name ("" = none set), "not found", ...
  bool kb;                   // CardKB present: footers say "OK:" (Enter) instead of "2x:" (double press)
  uint32_t uptime_s;
  bool feet;                 // depth units
  uint16_t son_range_cm;     // Sonar page depth scale: 0 = auto (deepest bottom + margin), else fixed
  uint16_t focus_range_cm;   // Focus page depth scale: 0 = auto, else fixed
  bool hide_weak;            // do not draw the weakest echoes (level 1)
  char note[24];             // D43: short note for the home footer ("Near bait: Pointe"), "" = none
  // FOCUS
  uint8_t focus_node; uint16_t bait_cm; uint8_t n_cols; ScrPingCol cols[118];
  // test
  uint8_t radio_test;        // 0 off, else a test mode
  const char* radio_test_name;
  ScrOptions opt;            // PG_OPTIONS (Settings) or the menu
  bool menu;                 // CardKB menu open: opt holds the list of pages
};

// Draws one frame into the buffer (the caller sends it: the firmware skips unchanged frames). `page` is ignored while a hole has FISH and alerts are not silenced (alert screen).
// `blink` toggles every 500 ms. `sub` = sub-page (holes list paging).
void screenDraw(u8g2_t* u, const ScreenModel& m, uint8_t page, uint8_t sub, bool blink);
uint8_t screenHolesPages(const ScreenModel& m);
uint16_t screenSonarAutoRange(const ScreenModel& m);   // cm, what "auto" shows now
uint16_t screenFocusAutoRange(const ScreenModel& m);
uint8_t screenSonarPages(const ScreenModel& m);
// what OK (Enter / double press) does on this page ("" = nothing): shown in the footer
const char* screenPageAction(const ScreenModel& m, uint8_t page);
