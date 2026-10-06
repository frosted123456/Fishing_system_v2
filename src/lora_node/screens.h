// v2 OLED screens (128x64, Heltec V3) — chalet and hubs. Pure drawing from a ScreenModel, with the U8g2
// C API, so the exact same code renders on the PC (tools/screens_mock.cpp → PNG mockups) and on the board.
// Design: docs/SCREENS.md. One button: short press = next page (silence during an alert).
#pragma once
#include <stdint.h>
#ifdef ARDUINO
#include <U8g2lib.h>
#else
#include "u8g2.h"
#endif

enum ScrHoleState : uint8_t { SH_OK = 0, SH_FISH = 1, SH_OFFLINE = 2, SH_FAULT = 3, SH_LOWBAT = 4 };
enum ScrPage : uint8_t { PG_HOME = 0, PG_HOLES, PG_NETWORK, PG_CONNECT, PG_COUNT };

struct ScrHole {
  uint8_t id, hub;           // hub = pocket (owner hub ID; the hub's own hole has hub == id)
  char name[16];             // "" = "Hole <id>"
  uint8_t state;             // ScrHoleState
  uint8_t batt;              // %, 255 = unknown
  int8_t fish;               // sonar fish count, -1 = no sonar
  uint32_t since_s;          // FISH: seconds since the trip
};

struct ScrHubLink { uint8_t id; int8_t lora_rssi; bool lora_ok, eb_ok; uint8_t eb_hops; };

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
  uint32_t uptime_s;
};

// Draws one frame. `page` is ignored while a hole has FISH and alerts are not silenced (alert screen).
// `blink` toggles every 500 ms. `sub` = sub-page (holes list paging).
void screenDraw(u8g2_t* u, const ScreenModel& m, uint8_t page, uint8_t sub, bool blink);
uint8_t screenHolesPages(const ScreenModel& m);
