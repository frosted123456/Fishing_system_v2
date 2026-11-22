// D207 probe: can an ESP-NOW tip-up keep its Wi-Fi receiver off most of the time on THIS toolchain?
// Not part of the product. Flash it on the WROOM (pio run -e ps_probe_wroom -t upload), meter in series with the pack
// (10 A range), serial monitor at 115200, hub and chalet running nearby (their ESP-NOW frames are what it counts).
//
// Why a probe: ESP-IDF 4.4 has "connectionless" power save for ESP-NOW (the radio wakes for `window` ms every `interval`
// ms: esp_now_set_wake_window + esp_wifi_set_connectionless_wake_interval), but it only works when Wi-Fi was initialised
// with sta_disconnected_pm = true, and the Arduino core 2.0.17 is built with that option off
// (CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE not set). Arduino's WiFi class always uses the default, so this sketch
// initialises Wi-Fi itself, with the flag on. Whether the pre-built Wi-Fi library honours it is the open question.
//
// It runs four 30 s phases in a loop and prints, per phase, the frames heard. Read the meter in each phase:
//   A  receiver always on              (today's tip-up: expect ~100 mA at 3.3 V)
//   B  awake 25 ms of every 100 ms     (works = clearly less current than A, and ~1/4 of A's frames)
//   C  awake 10 ms of every 100 ms
//   D  Wi-Fi stopped                   (the floor: CPU alone)
// Same current in A, B and C = the flag does nothing in this build: the idea needs another route (see D207).
#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <nvs_flash.h>

#ifndef PROBE_CHANNEL
#define PROBE_CHANNEL 1          // = ESPNOW_CHANNEL of the mesh (src/sensor_node/config.h)
#endif

static volatile uint32_t g_rx = 0;
static void onRecv(const uint8_t*, const uint8_t*, int) { g_rx++; }

static void say(const char* what, esp_err_t e) { Serial.printf("  %-46s %s\n", what, e == ESP_OK ? "OK" : esp_err_to_name(e)); }

static bool wifiUp() {
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  cfg.sta_disconnected_pm = true;                       // the option Arduino's build leaves off
  esp_err_t e = esp_wifi_init(&cfg); say("esp_wifi_init (sta_disconnected_pm = true)", e); if (e != ESP_OK) return false;
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_STA);
  e = esp_wifi_start(); say("esp_wifi_start", e); if (e != ESP_OK) return false;
  esp_wifi_set_channel(PROBE_CHANNEL, WIFI_SECOND_CHAN_NONE);
  say("esp_wifi_set_ps(WIFI_PS_MIN_MODEM)", esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
  e = esp_now_init(); say("esp_now_init", e); if (e != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  return true;
}
static void wifiDown() { esp_now_deinit(); esp_wifi_stop(); esp_wifi_deinit(); }

void setup() {
  Serial.begin(115200); delay(300);
  Serial.println("\nPS PROBE (D207): ESP-NOW receive windows. Read the meter in each phase.");
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
}

void loop() {
  static const struct { char name; uint16_t window, interval; bool radio; const char* text; } PH[4] = {
    {'A', 0, 0, true, "receiver always on (today)"},
    {'B', 25, 100, true, "awake 25 ms of every 100 ms"},
    {'C', 10, 100, true, "awake 10 ms of every 100 ms"},
    {'D', 0, 0, false, "Wi-Fi stopped (floor)"},
  };
  static uint32_t baseline = 0;
  for (int i = 0; i < 4; i++) {
    Serial.printf("\nPHASE %c: %s\n", PH[i].name, PH[i].text);
    bool up = false;
    if (PH[i].radio) {
      up = wifiUp();
      if (up && PH[i].interval) {
        say("esp_wifi_set_connectionless_wake_interval", esp_wifi_set_connectionless_wake_interval(PH[i].interval));
        say("esp_now_set_wake_window", esp_now_set_wake_window(PH[i].window));
      }
    }
    g_rx = 0;
    for (int s = 30; s > 0; s -= 5) { Serial.printf("  %c: read the meter now (%d s left), frames heard so far %lu\n", PH[i].name, s, (unsigned long)g_rx); delay(5000); }
    const uint32_t n = g_rx;
    if (i == 0) baseline = n;
    Serial.printf("RESULT %c: %lu frames in 30 s", PH[i].name, (unsigned long)n);
    if (i > 0 && PH[i].radio && baseline) Serial.printf(" = %lu %% of phase A", (unsigned long)(n * 100UL / baseline));
    Serial.println();
    if (PH[i].radio) wifiDown();
  }
  Serial.println("\n--- cycle done: A, B, C, D again. Note the four meter readings and the four RESULT lines. ---");
}
