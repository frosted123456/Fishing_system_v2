/*
 * ICE FISHING MESH MONITOR - Configuration
 *
 * Change these values to customize your network.
 * Each fishing group should use a unique NETWORK_ID to avoid interference.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ═══════════════════════════════════════════════════════════════════════════
// BUG FIX #4: COMPILE-TIME BOARD VALIDATION
// Ensure exactly one BOARD_* is defined to prevent pin conflicts
// ═══════════════════════════════════════════════════════════════════════════

#if !defined(BOARD_ESP32C3) && !defined(BOARD_ESP32CAM) && !defined(BOARD_ESP32WROOM)
  #error "No board defined! Uncomment exactly ONE of: BOARD_ESP32C3, BOARD_ESP32CAM, BOARD_ESP32WROOM"
#endif

#if (defined(BOARD_ESP32C3) && defined(BOARD_ESP32CAM)) || \
    (defined(BOARD_ESP32C3) && defined(BOARD_ESP32WROOM)) || \
    (defined(BOARD_ESP32CAM) && defined(BOARD_ESP32WROOM))
  #error "Multiple boards defined! Uncomment exactly ONE board type."
#endif

// ═══════════════════════════════════════════════════════════════════════════
// NETWORK IDENTITY - CHANGE THESE TO BE UNIQUE FOR YOUR GROUP
// ═══════════════════════════════════════════════════════════════════════════

#define NETWORK_ID          0x42        // Unique ID for your mesh (0x00-0xFF)
#define NETWORK_NAME        "IceFish"   // Prefix for WiFi SSIDs
#define WIFI_PASSWORD       "fishon123" // Same for all APs in network

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW SETTINGS
// ═══════════════════════════════════════════════════════════════════════════

#define ESPNOW_CHANNEL      1           // WiFi channel (1-13)
#define ESPNOW_ENCRYPT      false       // Encryption (adds overhead)

// ═══════════════════════════════════════════════════════════════════════════
// LONG RANGE MODE - Dramatically improves ESP-NOW range on flat ice
// ═══════════════════════════════════════════════════════════════════════════
// When enabled:
//   - Receiver sensitivity improves from ~-72dBm to ~-98dBm (~26dB gain)
//   - Range can increase 10-20x on flat ice surfaces
//   - Max TX power (21dBm) is automatically set
//   - Data rate is reduced (1Mbps instead of 54Mbps, but plenty for our small packets)
//
// CRITICAL: ALL communicating devices must have matching LR mode setting!
//   - Sensor nodes: Must enable LR mode
//   - Gateway onshore (on ice): Must enable LR mode
//   - Gateway offshore (cabin): MUST NOT use LR mode (needs normal WiFi for phone)
//
// Set to false to use standard 802.11 mode (shorter range but compatible with all devices)
// ═══════════════════════════════════════════════════════════════════════════
// v2 (D28): ESP-NOW runs at the NORMAL rate (802.11b 1 Mbps) on every device. LR cannot share a board
// with a phone hotspot (Espressif, esp-idf #4554), and the v1 range problem was the antenna height
// (≈5 cm over the ice); the 20 cm mast gains far more than LR. Keep LR only for a range comparison:
// it must then be set to the SAME value on every hub and tip-up (LR-only and normal cannot talk).
#define ESPNOW_LONG_RANGE_MODE  false   // v2: normal rate (was true in v1)

// Broadcast address for ESP-NOW
static const uint8_t ESPNOW_BROADCAST[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ═══════════════════════════════════════════════════════════════════════════
// REED SWITCH SETTINGS
// ═══════════════════════════════════════════════════════════════════════════

// Reed switch trigger polarity:
//   true  = Trigger when GPIO reads HIGH (magnet away from switch - normally-closed)
//   false = Trigger when GPIO reads LOW (magnet present - normally-open)
// Most tip-ups use a magnet that pulls away when the flag pops up, so HIGH is typical.
#define REED_ACTIVE_HIGH        true

// v2 (D46) spool-shaft Hall latch (US1881 + 4 magnets alternating): output flips every quarter turn.
// Used instead of the reed level logic when HALL_LATCH is 1 (WROOM tip-up by default; C3 keeps the reed).
#ifndef HALL_LATCH
#ifdef BOARD_ESP32WROOM
#define HALL_LATCH              1
#else
#define HALL_LATCH              0
#endif
#endif
#define TRIGGER_FLIPS           1       // 1 = alert on a quarter turn, 2 = half turn (wind / bait false alerts)
#define HALL_WINDOW_MS          10000   // flips further apart than this start a new count (matters with 2)
#define HALL_CLEAR_SEC          30      // trip ends (line state back to normal) after the shaft is still this long.
                                        // The chalet alarm stays latched until silenced (D41).

// ═══════════════════════════════════════════════════════════════════════════
// TIMING SETTINGS
// ═══════════════════════════════════════════════════════════════════════════

#define HEARTBEAT_INTERVAL_SEC  60      // Status broadcast when idle (seconds)
#define ALERT_REPEAT_MS         1000    // Re-send alert every X ms until acknowledged (loop mode)
#define ALERT_REPEAT_COUNT      10      // Max alert re-sends (loop mode)
#define NODE_TIMEOUT_SEC        120     // Mark node offline after X seconds silence
#define DEBOUNCE_MS             50      // Reed switch debounce time

// BUG FIX #3: Alert retry settings for deep sleep mode
#define ALERT_WAIT_TIMEOUT_MS   30000   // Wait 30 seconds for acknowledgment before sleeping
#define ALERT_RESEND_INTERVAL_MS 5000   // Re-send alert every 5 seconds while waiting
#define ALERT_REED_CHECK_MS     500     // Check reed switch every 500ms during wait (saves power)

// Fast heartbeat interval while FISH_ON is active
// Allows quick confirmation (within 5 seconds) when flag is reset
#define FISH_ON_HEARTBEAT_SEC   5

// Deep sleep settings (sensor nodes)
#define SLEEP_ENABLED           true    // Enable deep sleep between heartbeats
#define SLEEP_DURATION_US       (HEARTBEAT_INTERVAL_SEC * 1000000ULL)

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - ESP32-C3 SUPER MINI (Sensor Node)
// ═══════════════════════════════════════════════════════════════════════════
// WARNING: ESP32-C3 GPIO assignments differ from ESP32-WROOM!
//   - GPIO3 is safe for reed switch on ESP32-C3 Super Mini
//   - GPIO0/GPIO8/GPIO9 are boot strapping pins - avoid for critical functions
//   - GPIO2 is TX, GPIO3 is RX on some modules - check your specific board!
// ═══════════════════════════════════════════════════════════════════════════

#ifdef BOARD_ESP32C3

// BUG FIX #4: Use GPIO4 instead of GPIO3 for reed switch
// GPIO3 is RX on some ESP32-C3 modules and conflicts with serial
// GPIO4 is a safe general-purpose pin with RTC/deep-sleep wakeup support
#define REED_PIN            4           // Reed switch input (GPIO4 - safe for deep sleep wakeup)

// LED_PIN: GPIO8 is the built-in LED on ESP32-C3 Super Mini
// Set to -1 if your board doesn't have an LED or if GPIO8 causes boot issues
#define LED_PIN             8           // Onboard LED (-1 to disable)

// BUG FIX #4: GPIO0 is a boot strapping pin! Use GPIO1 instead for ADC
// GPIO1 is ADC1_CH1 and safe to use
// Set to -1 if no battery voltage divider is connected
#define VBAT_PIN            1           // Battery voltage ADC (GPIO1/ADC1_CH1, -1 to disable)

// Battery voltage divider calibration
#define VBAT_DIVIDER        2.0
#define VBAT_REF_MV         3300        // ADC reference voltage in mV

#endif // BOARD_ESP32C3

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - ESP32-CAM (Sensor Node)
// ═══════════════════════════════════════════════════════════════════════════

#ifdef BOARD_ESP32CAM

#define REED_PIN            13          // GPIO13 (avoid camera pins)
#define LED_PIN             33          // Onboard red LED
#define FLASH_PIN           4           // Flash LED (don't use for battery)
#define VBAT_PIN            -1          // No ADC available easily

#endif // BOARD_ESP32CAM

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS - ESP32-WROOM (Sensor Node)
// ═══════════════════════════════════════════════════════════════════════════

#ifdef BOARD_ESP32WROOM

#define REED_PIN            27          // Hall latch output (US1881, open drain): RTC GPIO, ext0 wake source
#define LED_PIN             2           // Onboard LED (most devkits)
#define VBAT_PIN            34          // Battery voltage ADC

#define VBAT_DIVIDER        2.0
#define VBAT_REF_MV         3300

#endif // BOARD_ESP32WROOM

// ═══════════════════════════════════════════════════════════════════════════
// DEBUG SETTINGS
// ═══════════════════════════════════════════════════════════════════════════

#define DEBUG_SERIAL        false       // 7B FIX: Disabled for production (saves power/flash)
#define DEBUG_BAUD          115200      // Serial baud rate
#define DEBUG_ESPNOW        true        // Log ESP-NOW traffic

// Debug print macros
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

// For 3x AA Lithium (4.5V nominal, 3.0V cutoff)
#define BATTERY_FULL_MV     4500
#define BATTERY_LOW_MV      3600    // ~20% remaining
#define BATTERY_CRITICAL_MV 3200    // Shutdown imminent

#endif // CONFIG_H
