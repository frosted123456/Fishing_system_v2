/*
 * ICE FISHING MESH MONITOR - Configuration
 * 
 * For Heltec WiFi LoRa 32 V3
 */

#ifndef CONFIG_H
#define CONFIG_H

// ═══════════════════════════════════════════════════════════════════════════
// NETWORK IDENTITY - CHANGE THESE TO BE UNIQUE FOR YOUR GROUP
// ═══════════════════════════════════════════════════════════════════════════

#define NETWORK_ID          0x42        // Unique ID for your mesh (0x00-0xFF)
#define NETWORK_NAME        "IceFish"   // Prefix for WiFi SSIDs
#define WIFI_PASSWORD       "fishon123" // Same for all APs in network

// ═══════════════════════════════════════════════════════════════════════════
// LORA SETTINGS - OPTIMIZED FOR SPEED AND RELIABILITY
// ═══════════════════════════════════════════════════════════════════════════

// v2 (TDMA mesh): spreading factor and bandwidth are per slot, from lib/IceMesh/src/radio_modes.h:
// SF9/500 kHz (beacon, echo, join, relay links, default), SF8/500, SF7/500 — all 500 kHz single
// channel (digital modulation, RSS-247 §5.2), no 125 kHz single-channel operation (that would have to hop).
// Not legal advice.
#define LORA_FREQUENCY      915.0       // MHz, channel centre (914.75-915.25 MHz at 500 kHz)
#define LORA_CODING_RATE    5           // 4/5 (must match radio_modes.h airtime: cr_denom = 5)
#define LORA_SYNC_WORD      0x34        // Private sync (default 0x12 is public)
#define LORA_TX_POWER       20          // Higher power (was 17, max 22)
#define LORA_PREAMBLE       8           // 8 symbols minimum reliable (saves ~40ms)

// Mesh settings: superframe 1 s, one relay hop max (see lib/IceMesh/src/tdma_schedule.h)

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW SETTINGS
// ═══════════════════════════════════════════════════════════════════════════

#define ESPNOW_CHANNEL      1           // WiFi channel (1-13)
#define ESPNOW_ENCRYPT      false

// ═══════════════════════════════════════════════════════════════════════════
// LONG RANGE MODE - For GATEWAY_ONSHORE only (on-ice gateway)
// ═══════════════════════════════════════════════════════════════════════════
// When enabled for GATEWAY_ONSHORE:
//   - Receiver sensitivity improves from ~-72dBm to ~-98dBm (~26dB gain)
//   - Range can increase 10-20x on flat ice surfaces
//   - Max TX power (21dBm) is automatically set
//   - WiFi AP is disabled (no web server access from phones on ice)
//   - Communication with sensor nodes requires matching LR mode
//
// ARCHITECTURE:
//   - GATEWAY_ONSHORE (on ice): ESP-NOW (normal rate) + LoRa, no hotspot by default (v2)
//   - GATEWAY_OFFSHORE (cabin): Normal WiFi AP for phone access, LoRa + ESP-NOW backbone (v2)
//   - User checks fish status from OFFSHORE gateway via phone
//
// CRITICAL: hubs and sensor nodes must use the SAME ESPNOW_LONG_RANGE_MODE value!
// ═══════════════════════════════════════════════════════════════════════════
// v2 (D28): ESP-NOW runs at the NORMAL rate (802.11b 1 Mbps) on every device. LR cannot share a board
// with a phone hotspot (Espressif, esp-idf #4554), and the v1 range problem was the antenna height
// (≈5 cm over the ice); the 20 cm mast gains far more than LR. Keep LR only for a range comparison:
// it must then be set to the SAME value on every hub and tip-up (LR-only and normal cannot talk).
#define ESPNOW_LONG_RANGE_MODE  false   // v2: normal rate (was true in v1)

static const uint8_t ESPNOW_BROADCAST[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ═══════════════════════════════════════════════════════════════════════════
// TIMING SETTINGS - OPTIMIZED FOR RELIABILITY
// ═══════════════════════════════════════════════════════════════════════════

#define HEARTBEAT_INTERVAL_SEC  30      // More frequent heartbeats (was 60)
#define NODE_TIMEOUT_SEC        150     // v2: >= 2.5 node heartbeats (sim finding S3: 90 s showed nodes offline after one lost heartbeat)
#define DEBOUNCE_MS             50
#define SILENCE_AUTO_CLEAR_MS   (5 * 60 * 1000)  // 5 minutes auto-unsilence

// ═══════════════════════════════════════════════════════════════════════════
// WIFI MODE CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════

// WiFi modes:
//   WIFI_MODE_AP    - Create own access point (default, works anywhere)
//   WIFI_MODE_STA   - Connect to existing network (home WiFi, phone hotspot)
//   WIFI_MODE_APSTA - Both simultaneously

#define WIFI_MODE_SETTING   WIFI_MODE_APSTA  // <<< CHANGE THIS

// If using STA mode, set your network credentials here:
// The cabin Wi-Fi name and password live in secrets.h (NOT in git: copy secrets.example.h to secrets.h
// and fill it in). Without the file the chalet still works on its own hotspot.
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef STA_SSID
#define STA_SSID            ""              // empty = no cabin network, hotspot only
#endif
#ifndef STA_PASSWORD
#define STA_PASSWORD        ""
#endif

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - HELTEC LORA32 V3 (ESP32-S3)
// ═══════════════════════════════════════════════════════════════════════════

// Power control - ACTIVE LOW (set LOW to enable peripherals)
#define VEXT_PIN            36          // Controls power to OLED and other peripherals

// OLED Display (SSD1306 128x64)
#define OLED_SDA            17
#define OLED_SCL            18
#define OLED_RST            21
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_ADDRESS        0x3C        // I2C address

// LoRa Radio (SX1262)
#define LORA_SCK            9
#define LORA_MISO           11
#define LORA_MOSI           10
#define LORA_CS             8
#define LORA_RST            12
#define LORA_DIO1           14
#define LORA_BUSY           13

// User IO
#define LED_PIN             35          // Onboard white LED
#define USER_BUTTON         0           // PRG/BOOT button (screen, silence, hotspot)
#ifndef BTN_PIN
#define BTN_PIN             5           // D61 arming button to GND (internal pull-up), hub / LoRa tip-up hole; -1 = none
#endif

// External connections
#define BUZZER_PIN          7           // CLT1036 active buzzer
#define REED_PIN            4           // Own hole sensor (hub / LoRa tip-up): reed switch or Hall latch output
// D64: the own hole can be a spool-shaft Hall latch (US1881 + alternating magnets, like the WROOM head: a trip is
// TRIGGER_FLIPS flips within HALL_WINDOW_MS, over after HALL_CLEAR_SEC still) or a plain reed (level + the OLED
// "Reed polarity" setting). Same knobs as src/sensor_node/config.h. Build flag -D HOLE_HALL_LATCH=0 for a reed.
#ifndef HOLE_HALL_LATCH
#define HOLE_HALL_LATCH     1
#endif
#define TRIGGER_FLIPS       1           // 1 = alert on a quarter turn, 2 = half turn (wind / bait false alerts)
#define HALL_WINDOW_MS      10000       // flips further apart than this start a new count (matters with 2)
#define HALL_CLEAR_SEC      30          // trip ends (line state back to normal) after the shaft is still this long

// CardKB I2C Keyboard (M5Stack)
#define CARDKB_SDA          41
#define CARDKB_SCL          42
#define CARDKB_ADDR         0x5F        // CardKB I2C address

// Battery calibration for external 3×AA with MP1584 buck converter
// External 100k/100k voltage divider on GPIO2
// Measures raw battery voltage BEFORE buck converter
// Divider ratio: 100k/(100k+100k) = 0.5, so multiply ADC reading by 2.0
#define VBAT_PIN            2           // GPIO2 for external divider (not GPIO1!)
// D66: a Heltec on its own LiPo (JST) has the onboard monitor: GPIO1 behind a 390k/100k divider (x4.9), switched
// by ADC_Ctrl GPIO37 (HIGH on V3.2, LOW on V3 / V3.1: both are tried). Default on for the chalet (LiPo), off for the
// hubs / LoRa tip-ups fed on the 3V3 pin from a pack (their divider is on VBAT_PIN). -D VBAT_ONBOARD=0/1 overrides.
#ifndef VBAT_ONBOARD
#define VBAT_ONBOARD        (NODE_ROLE == ROLE_GATEWAY_OFFSHORE)
#endif
#define VBAT_ONBOARD_PIN    1
#define VBAT_ONBOARD_CTRL   37
#define VBAT_ONBOARD_RATIO  4.9f
// D73: GPS on the chalet box (NEO-6M class, 9600 baud NMEA) on UART1. GPS_RX_PIN = the ESP32 pin the module's TX is
// on. Frank's wiring (2023, "pretty sure 45/46"): if nothing arrives in GPS_SWAP_MS the firmware swaps the two once.
// -D GPS_RX_PIN=-1 for a chalet without GPS.
#ifndef GPS_RX_PIN
#define GPS_RX_PIN          46
#endif
#ifndef GPS_TX_PIN
#define GPS_TX_PIN          45
#endif
#define GPS_BAUD            9600
#define GPS_SWAP_MS         4000
#define VBAT_DIVIDER        2.0         // External 100k+100k divider
#define VBAT_REF_MV         3300        // ESP32-S3 ADC reference with 11dB atten

// ═══════════════════════════════════════════════════════════════════════════
// BUZZER SETTINGS
// ═══════════════════════════════════════════════════════════════════════════

#define BUZZER_ALERT_BEEPS      3
#define BUZZER_BEEP_ON_MS       200
#define BUZZER_BEEP_OFF_MS      100
#define BUZZER_REPEAT_DELAY_MS  2000

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER SETTINGS
// ═══════════════════════════════════════════════════════════════════════════

#define WEB_SERVER_PORT     80
#define AP_MAX_CONNECTIONS  4
#define AP_CHANNEL          ESPNOW_CHANNEL

// ═══════════════════════════════════════════════════════════════════════════
// SONAR (D62): TUSS4470 shield 002 on the module's own hole (hub, LoRa tip-up, relay with a hole; not the chalet)
// Pins: lib/SonarTuss/src/sonar_tuss4470.h (Heltec V3 block: SCK 47, MOSI 48, MISO 33, CS 34, IO1 38, IO2 39, VOUT 6).
// ═══════════════════════════════════════════════════════════════════════════

#ifndef SONAR_REAL
#define SONAR_REAL          0           // 1 = real sonar on the own hole (envs hub_sonar / sensor_lora_sonar), needs HAS_LOCAL_SENSOR
#endif
#ifndef SONAR_BENCH
#define SONAR_BENCH         0           // 1 = bucket test: a BENCH line on Serial every 2 s (env hub_sonar_bench)
#endif
#define SONAR_PING_EST_US   160000UL    // est.: one ping = up to 6 captures x ~21 ms + settle + jitter (3 freqs, avg 2); the capture is pushed
                                        // to the next tick when a LoRa transmit is planned inside this window
#define SONAR_RF_GUARD_US   2000UL      // quiet time after the last transmit before a capture (as the WROOM, D54)

// ═══════════════════════════════════════════════════════════════════════════
// DEBUG SETTINGS
// ═══════════════════════════════════════════════════════════════════════════

#define DEBUG_SERIAL        false       // 7B FIX: Disabled for production
#define DEBUG_BAUD          115200
#define DEBUG_ESPNOW        true
#define DEBUG_LORA          true

#if DEBUG_SERIAL
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY THRESHOLDS
// ═══════════════════════════════════════════════════════════════════════════

// For LiPo (3.7V nominal)
#define LIPO_FULL_MV        4200
#define LIPO_LOW_MV         3500
#define LIPO_CRITICAL_MV    3200

// For 3x AA Lithium
#define BATTERY_FULL_MV     4500
#define BATTERY_LOW_MV      3600
#define BATTERY_CRITICAL_MV 3200

#endif // CONFIG_H
