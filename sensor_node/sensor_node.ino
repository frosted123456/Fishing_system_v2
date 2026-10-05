/*
 * ICE FISHING MESH MONITOR - Sensor Node
 * 
 * For: ESP32-C3 Super Mini, ESP32-CAM, ESP32-WROOM
 * Role: SENSOR_ONLY
 * 
 * Monitors reed switch, broadcasts status via ESP-NOW, deep sleeps between updates.
 * 
 * WIRING:
 *   Reed Switch: GPIO (REED_PIN) ←→ GND
 *   Battery: 3x AA Lithium → Buck converter → 3.3V/GND
 * 
 * ARDUINO IDE SETUP:
 *   Board: ESP32C3 Dev Module (or ESP32 Dev Module for WROOM/CAM)
 *   USB CDC On Boot: Enabled
 *   Upload Speed: 921600
 * 
 * CONFIGURATION:
 *   1. Uncomment correct BOARD_* define below
 *   2. Set NODE_ID to unique value (1-254)
 *   3. Set NODE_NAME for web UI display
 */

// ═══════════════════════════════════════════════════════════════════════════
// BOARD SELECTION - UNCOMMENT ONE
// ═══════════════════════════════════════════════════════════════════════════

#define BOARD_ESP32C3
// #define BOARD_ESP32CAM
// #define BOARD_ESP32WROOM

// ═══════════════════════════════════════════════════════════════════════════
// NODE CONFIGURATION - CHANGE THESE PER DEVICE
// ═══════════════════════════════════════════════════════════════════════════

#define NODE_ID     2               // <<< CHANGE THIS! Unique ID (1-254)
#define NODE_NAME   "Hole 2"        // Name shown in web UI

// ═══════════════════════════════════════════════════════════════════════════
// INCLUDES
// ═══════════════════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_sleep.h>

// Cross-compatible deep sleep GPIO wakeup support
#ifdef BOARD_ESP32C3
  #include <driver/gpio.h>
  #include <driver/rtc_io.h>
  // Include for ESP-IDF version detection
  #if __has_include("esp_idf_version.h")
    #include "esp_idf_version.h"
  #endif
#endif

#include "config.h"
#include "messages.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════════════

// BUG FIX #5: Magic number to detect RTC memory corruption
// If this value is not correct after wake, RTC data may be corrupted
#define RTC_MAGIC_NUMBER 0xF150A42E  // "FISH ON" with unique pattern
RTC_DATA_ATTR uint32_t rtcMagic = 0;

// Boot count and sequence persist across deep sleep
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t totalUptimeSec = 0;
RTC_DATA_ATTR uint16_t messageSeq = 0;      // Monotonic sequence for dedup
RTC_DATA_ATTR bool lastReedState = false;

// BUG FIX #7: Track actual sleep duration for accurate timing
// Records expected sleep duration so we can properly decrement counters on wake
RTC_DATA_ATTR uint32_t expectedSleepSec = 0;
RTC_DATA_ATTR esp_sleep_wakeup_cause_t lastWakeReason = ESP_SLEEP_WAKEUP_UNDEFINED;

// BUG FIX #11: Silence state stored as remaining seconds (not absolute time)
// After deep sleep, millis() resets to 0, making absolute timestamps meaningless
RTC_DATA_ATTR bool alertsSilenced = false;
RTC_DATA_ATTR uint32_t silenceRemainingSec = 0;  // Remaining seconds of silence

// FISH_ON fast heartbeat mode - 5s sleep instead of 60s while flag is up
// This allows quick detection (within 5s) when flag is reset
RTC_DATA_ATTR bool fishOnMonitorMode = false;

// Gateway MAC address - learned from ACK responses
// Using unicast enables hardware-level ACK from ESP-NOW
RTC_DATA_ATTR uint8_t gatewayMac[6] = {0};
RTC_DATA_ATTR bool gatewayMacKnown = false;

// Current state
volatile bool reedTriggered = false;
bool espNowReady = false;
uint8_t currentFlags = 0;
uint16_t batteryMv = 0;

// Timing
unsigned long lastHeartbeat = 0;
unsigned long alertSentTime = 0;
uint8_t alertRetryCount = 0;

// ESP-NOW send status
volatile bool sendInProgress = false;
volatile bool sendSuccess = false;

// ═══════════════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════

void setupPins();
void setupEspNow();
void readBattery();
bool readReedSwitch();
void sendStatus(uint8_t msgType);
void sendAlert();
void enterDeepSleep();
void enterDeepSleepFast();
bool espNowSendReliable(const uint8_t* data, size_t len, int maxRetries = 3);
void blinkLed(int times, int onMs, int offMs);
void onEspNowSent(const uint8_t* mac, esp_now_send_status_t status);
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);
bool validateAndInitRtcData();
uint32_t estimateActualSleepDuration(esp_sleep_wakeup_cause_t wakeReason);

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  #if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
  delay(100);
  Serial.println();
  Serial.println(F("════════════════════════════════════════"));
  Serial.println(F("   ICE FISHING MONITOR - SENSOR NODE"));
  Serial.println(F("════════════════════════════════════════"));
  #endif

  // Determine wake reason FIRST (before modifying RTC data)
  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();

  // BUG FIX #5: Validate RTC data integrity
  bool rtcValid = validateAndInitRtcData();

  // Increment boot count AFTER validation
  bootCount++;

  #if DEBUG_SERIAL
  Serial.printf("Node ID:     %d\n", NODE_ID);
  Serial.printf("Node Name:   %s\n", NODE_NAME);
  Serial.printf("Boot Count:  %lu\n", bootCount);
  Serial.printf("Network ID:  0x%02X\n", NETWORK_ID);
  if (!rtcValid) {
    Serial.println(F("WARNING: RTC data was corrupted/uninitialized - reset to defaults"));
  }
  #endif

  // BUG FIX #7: Calculate actual sleep duration for accurate timing
  uint32_t actualSleepSec = estimateActualSleepDuration(wakeReason);

  switch (wakeReason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      DEBUG_PRINTLN(F("Wake: Reed switch triggered (EXT0)!"));
      reedTriggered = true;
      break;
    case ESP_SLEEP_WAKEUP_GPIO:
      DEBUG_PRINTLN(F("Wake: Reed switch triggered (GPIO)!"));
      reedTriggered = true;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      DEBUG_PRINTLN(F("Wake: Heartbeat timer"));
      break;
    default:
      DEBUG_PRINTF("Wake: Power on / reset (%d)\n", wakeReason);
      SET_FLAG(currentFlags, FLAG_FIRST_BOOT);
      messageSeq = 0;         // RESET SEQUENCE ON REBOOT
      totalUptimeSec = 0;     // RESET UPTIME ON REBOOT
      actualSleepSec = 0;     // No sleep on fresh boot
      break;
  }

  // Store wake reason for next cycle
  lastWakeReason = wakeReason;

  // Check if we're in FISH_ON monitor mode (fast heartbeat)
  if (fishOnMonitorMode) {
    DEBUG_PRINTLN(F("In FISH_ON monitor mode (5s heartbeat)"));
  }

  // Setup hardware
  setupPins();
  
  // Read sensors
  readBattery();
  bool reedState = readReedSwitch();
  
  DEBUG_PRINTF("Battery:     %d mV\n", batteryMv);
  DEBUG_PRINTF("Reed State:  %s\n", reedState ? "OPEN (flag up)" : "CLOSED (flag down)");
  
  // Check for alert condition
  if (reedState) {
    SET_FLAG(currentFlags, FLAG_FISH_ON);
    reedTriggered = true;
  }
  
  // Check battery level
  if (batteryMv < BATTERY_LOW_MV && batteryMv > 0) {
    SET_FLAG(currentFlags, FLAG_LOW_BATTERY);
    DEBUG_PRINTLN(F("WARNING: Low battery!"));
  }
  
  // Initialize ESP-NOW
  setupEspNow();

  // BUG FIX #7 & #11: Check if silence has expired using ACTUAL sleep duration
  // After deep sleep, we decrement remaining seconds by how long we actually slept
  if (alertsSilenced && silenceRemainingSec > 0) {
    // Use actual sleep duration calculated from wake reason
    if (silenceRemainingSec > actualSleepSec) {
      silenceRemainingSec -= actualSleepSec;
      DEBUG_PRINTF("Silence remaining: %lu seconds (slept %lu sec)\n", silenceRemainingSec, actualSleepSec);
    } else {
      DEBUG_PRINTLN("Silence expired - clearing");
      alertsSilenced = false;
      silenceRemainingSec = 0;
    }
  }

  // Send message based on wake reason
  if (reedTriggered) {
    DEBUG_PRINTLN(F(">>> Sending ALERT"));
    sendAlert();
  } else {
    DEBUG_PRINTLN(F(">>> Sending HEARTBEAT"));
    sendStatus(MSG_HEARTBEAT);
  }

  delay(50);  // P3 FIX: Reduced from 100ms to 50ms - adequate for ESP-NOW TX completion

  // BUG FIX #3: Enhanced alert retry logic
  // If alert triggered, stay awake longer and re-send alerts periodically
  if (reedTriggered && SLEEP_ENABLED) {
    unsigned long waitStart = millis();
    unsigned long lastAlertSend = millis();
    uint8_t alertResendCount = 0;

    DEBUG_PRINTF("Waiting up to %d ms for acknowledgment...\n", ALERT_WAIT_TIMEOUT_MS);

    while (HAS_FLAG(currentFlags, FLAG_FISH_ON) &&
           !HAS_FLAG(currentFlags, FLAG_ACKNOWLEDGED) &&
           (millis() - waitStart < ALERT_WAIT_TIMEOUT_MS)) {

      // Check reed switch less frequently to save power (every 500ms)
      delay(ALERT_REED_CHECK_MS);

      // Re-check reed switch state
      if (!readReedSwitch()) {
        CLEAR_FLAG(currentFlags, FLAG_FISH_ON);
        DEBUG_PRINTLN(F("Flag reset - fish off or false trigger"));
        // BUG FIX #18: Always send clear status before breaking
        // Without this, the gateway never receives the clear and FISH_ON is stuck
        sendStatus(MSG_STATUS);
        delay(100);
        break;
      }

      // Re-send alert every ALERT_RESEND_INTERVAL_MS while waiting
      if (millis() - lastAlertSend >= ALERT_RESEND_INTERVAL_MS) {
        alertResendCount++;
        DEBUG_PRINTF("Re-sending alert (retry #%d)\n", alertResendCount);
        sendAlert();
        lastAlertSend = millis();
      }
    }

    if (HAS_FLAG(currentFlags, FLAG_ACKNOWLEDGED)) {
      DEBUG_PRINTLN(F("Alert acknowledged!"));
    } else if (!HAS_FLAG(currentFlags, FLAG_FISH_ON)) {
      DEBUG_PRINTLN(F("Alert cleared - flag reset"));
    } else {
      DEBUG_PRINTLN(F("Wait timeout - will re-alert on next wake"));
    }
  }
  
  // Calculate actual elapsed time since boot (before entering deep sleep)
  unsigned long elapsedMs = millis();
  totalUptimeSec += (elapsedMs / 1000);

  // Determine sleep behavior based on current state
  if (SLEEP_ENABLED) {
    // Check if FISH_ON just cleared (was in monitor mode, now reed is closed)
    if (fishOnMonitorMode && !HAS_FLAG(currentFlags, FLAG_FISH_ON)) {
      DEBUG_PRINTLN(F("Flag reset detected - exiting fast heartbeat mode"));
      fishOnMonitorMode = false;
      // Send one more status to confirm clear, then normal sleep
      sendStatus(MSG_HEARTBEAT);
      delay(100);
      enterDeepSleep();  // Normal 60s sleep
    }
    // Check if FISH_ON is active - enter or stay in monitor mode
    else if (HAS_FLAG(currentFlags, FLAG_FISH_ON)) {
      fishOnMonitorMode = true;
      DEBUG_PRINTLN(F("FISH_ON active - using fast 5s heartbeat"));
      enterDeepSleepFast();  // Short 5s sleep
    }
    // Normal operation
    else {
      fishOnMonitorMode = false;
      enterDeepSleep();  // Normal 60s sleep
    }
  }

  DEBUG_PRINTLN(F("Setup complete - entering main loop"));
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP (only runs if deep sleep disabled)
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  static unsigned long lastUptimeUpdate = 0;

  // Update uptime every second
  if (millis() - lastUptimeUpdate >= 1000) {
    totalUptimeSec++;
    lastUptimeUpdate = millis();
  }

  // Check reed switch
  bool currentReed = readReedSwitch();
  static bool lastReed = false;
  
  if (currentReed != lastReed) {
    lastReed = currentReed;
    
    if (currentReed) {
      DEBUG_PRINTLN(F("FISH ON! Flag triggered"));
      SET_FLAG(currentFlags, FLAG_FISH_ON);
      sendAlert();
    } else {
      DEBUG_PRINTLN(F("Flag reset - FISH OFF"));
      CLEAR_FLAG(currentFlags, FLAG_FISH_ON);
      // SEND IMMEDIATE STATUS UPDATE (not just wait for heartbeat)
      sendStatus(MSG_STATUS);
    }
  }
  
  // Periodic heartbeat
  if (millis() - lastHeartbeat > (HEARTBEAT_INTERVAL_SEC * 1000UL)) {
    lastHeartbeat = millis();
    readBattery();
    sendStatus(MSG_HEARTBEAT);
    DEBUG_PRINTLN(F("Heartbeat sent"));
  }
  
  // Alert retry
  if (HAS_FLAG(currentFlags, FLAG_FISH_ON) && 
      !HAS_FLAG(currentFlags, FLAG_ACKNOWLEDGED) &&
      alertRetryCount < ALERT_REPEAT_COUNT &&
      (millis() - alertSentTime > ALERT_REPEAT_MS)) {
    sendAlert();
    alertRetryCount++;
  }
  
  delay(DEBOUNCE_MS);
}

// ═══════════════════════════════════════════════════════════════════════════
// PIN SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setupPins() {
  // Configure reed switch input with internal pull-up
  // When magnet is present (flag down): switch closed, reads LOW
  // When magnet absent (flag up/FISH ON): switch open, reads HIGH
  pinMode(REED_PIN, INPUT_PULLUP);

  #ifdef LED_PIN
  if (LED_PIN >= 0) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
  }
  #endif

  #ifdef VBAT_PIN
  if (VBAT_PIN >= 0) {
    // Configure pin as analog input
    // BUG FIX #6: Pin-specific attenuation is set in readBattery()
    pinMode(VBAT_PIN, INPUT);
  }
  #endif

  blinkLed(2, 50, 50);
}

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setupEspNow() {
  DEBUG_PRINTLN(F("Initializing ESP-NOW..."));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // ═══════════════════════════════════════════════════════════════════════════
  // LONG RANGE MODE - Enable ESP32 proprietary 802.11 LR mode
  // ═══════════════════════════════════════════════════════════════════════════
  // LR mode improves receiver sensitivity from ~-72dBm to ~-98dBm
  // This provides 10-20x range improvement on flat ice surfaces
  // IMPORTANT: Both sender and receiver MUST use LR mode for communication
  // ═══════════════════════════════════════════════════════════════════════════
  #if ESPNOW_LONG_RANGE_MODE
  DEBUG_PRINTLN(F("Enabling Long Range (LR) mode..."));

  // Set maximum TX power (84 = 21dBm, which is the max allowed)
  // This must be done before setting protocol
  esp_err_t txResult = esp_wifi_set_max_tx_power(84);
  if (txResult != ESP_OK) {
    DEBUG_PRINTF("WARNING: Failed to set TX power: %d\n", txResult);
  } else {
    int8_t actualPower;
    esp_wifi_get_max_tx_power(&actualPower);
    DEBUG_PRINTF("TX power set to: %.2f dBm\n", actualPower * 0.25);
  }

  // Enable LR (Long Range) protocol
  // WIFI_PROTOCOL_LR is ESP32-specific and provides ~26dB sensitivity improvement
  esp_err_t lrResult = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  if (lrResult != ESP_OK) {
    DEBUG_PRINTF("WARNING: Failed to enable LR mode: %d\n", lrResult);
  } else {
    DEBUG_PRINTLN(F("LR mode enabled successfully"));
  }
  #else
  DEBUG_PRINTLN(F("LR mode disabled (using standard 802.11)"));
  #endif

  // CRITICAL: Set channel and verify alignment
  delay(100);  // Let WiFi mode stabilize
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(50);

  // VERIFY channel
  uint8_t primary;
  wifi_second_chan_t secondary;
  esp_wifi_get_channel(&primary, &secondary);
  DEBUG_PRINTF("WiFi channel verified: %d (expected: %d)\n", primary, ESPNOW_CHANNEL);

  if (primary != ESPNOW_CHANNEL) {
    DEBUG_PRINTLN("WARNING: Channel mismatch! Retrying...");
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(50);
  }

  DEBUG_PRINTF("MAC Address: %s\n", WiFi.macAddress().c_str());

  // AUDIT FIX: Add retry logic for ESP-NOW initialization
  // esp_now_init() can fail transiently after deep sleep wake
  esp_err_t initResult = ESP_FAIL;
  const int maxRetries = 3;

  for (int retry = 0; retry < maxRetries; retry++) {
    initResult = esp_now_init();
    if (initResult == ESP_OK) {
      break;
    }
    DEBUG_PRINTF("ESP-NOW init attempt %d failed, retrying...\n", retry + 1);
    delay(100);
  }

  if (initResult != ESP_OK) {
    DEBUG_PRINTLN(F("ERROR: ESP-NOW init failed after retries!"));
    SET_FLAG(currentFlags, FLAG_SENSOR_ERROR);
    return;
  }

  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ESPNOW_BROADCAST, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = ESPNOW_ENCRYPT;

  // AUDIT FIX: Also retry peer add in case of transient failure
  esp_err_t peerResult = ESP_FAIL;
  for (int retry = 0; retry < maxRetries; retry++) {
    peerResult = esp_now_add_peer(&peerInfo);
    if (peerResult == ESP_OK) {
      break;
    }
    // ESP_ERR_ESPNOW_EXIST means peer already added (not an error)
    if (peerResult == ESP_ERR_ESPNOW_EXIST) {
      peerResult = ESP_OK;
      break;
    }
    DEBUG_PRINTF("Add peer attempt %d failed, retrying...\n", retry + 1);
    delay(50);
  }

  if (peerResult != ESP_OK) {
    DEBUG_PRINTLN(F("ERROR: Failed to add broadcast peer after retries"));
    SET_FLAG(currentFlags, FLAG_SENSOR_ERROR);
    return;
  }

  // If we know the gateway MAC from previous session, add as unicast peer
  // This enables hardware-level ACK for more reliable delivery
  if (gatewayMacKnown) {
    esp_now_peer_info_t gwPeer = {};
    memcpy(gwPeer.peer_addr, gatewayMac, 6);
    gwPeer.channel = ESPNOW_CHANNEL;
    gwPeer.encrypt = false;

    if (esp_now_add_peer(&gwPeer) == ESP_OK) {
      DEBUG_PRINTF("Restored gateway peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   gatewayMac[0], gatewayMac[1], gatewayMac[2],
                   gatewayMac[3], gatewayMac[4], gatewayMac[5]);
    }
  }

  SET_FLAG(currentFlags, FLAG_ESPNOW_OK);
  espNowReady = true;
  DEBUG_PRINTLN(F("ESP-NOW ready"));
}

// ═══════════════════════════════════════════════════════════════════════════
// RELIABLE ESP-NOW SEND - With retry and hardware ACK support
// ═══════════════════════════════════════════════════════════════════════════

bool espNowSendReliable(const uint8_t* data, size_t len, int maxRetries) {
  bool success = false;

  // Try unicast first if gateway MAC is known (gets hardware ACK)
  if (gatewayMacKnown) {
    DEBUG_PRINTLN("ESP-NOW: Using unicast to known gateway");

    for (int retry = 0; retry < maxRetries && !success; retry++) {
      sendInProgress = true;
      sendSuccess = false;

      esp_err_t result = esp_now_send(gatewayMac, data, len);

      if (result == ESP_OK) {
        // C4 FIX: Wait for send callback with reduced timeout (50ms max)
        // Using yield() instead of delay() to prevent WiFi task starvation
        unsigned long waitStart = millis();
        while (sendInProgress && (millis() - waitStart < 50)) {
          yield();  // Allow WiFi task to process callback
        }

        if (sendSuccess) {
          DEBUG_PRINTF("ESP-NOW: Unicast success (attempt %d)\n", retry + 1);
          success = true;
        } else {
          DEBUG_PRINTF("ESP-NOW: Unicast failed (attempt %d)\n", retry + 1);
          delay(20 + random(20));  // Brief delay before retry
        }
      } else {
        DEBUG_PRINTF("ESP-NOW: Send error %d (attempt %d)\n", result, retry + 1);
        delay(20 + random(20));
      }
    }

    // If unicast failed completely, clear gateway MAC and fall back to broadcast
    if (!success) {
      DEBUG_PRINTLN("ESP-NOW: Unicast failed - clearing gateway MAC, using broadcast");
      gatewayMacKnown = false;
      memset(gatewayMac, 0, 6);
    }
  }

  // Fall back to broadcast with retry (no hardware ACK, but 2x helps)
  if (!success) {
    DEBUG_PRINTLN("ESP-NOW: Using broadcast with retry");

    for (int retry = 0; retry < 2; retry++) {
      sendInProgress = true;
      sendSuccess = false;

      esp_err_t result = esp_now_send(ESPNOW_BROADCAST, data, len);

      if (result == ESP_OK) {
        // C4 FIX: Wait for callback with reduced timeout (50ms max)
        // Using yield() instead of delay() to prevent WiFi task starvation
        unsigned long waitStart = millis();
        while (sendInProgress && (millis() - waitStart < 50)) {
          yield();  // Allow WiFi task to process callback
        }

        // For broadcast, sendSuccess just means it was transmitted
        // (no ACK available for broadcast)
        if (sendSuccess) {
          success = true;
          DEBUG_PRINTF("ESP-NOW: Broadcast sent (attempt %d)\n", retry + 1);
        }
      }

      if (retry < 1) delay(15 + random(15));  // Brief delay between broadcasts
    }
  }

  return success;
}

// ═══════════════════════════════════════════════════════════════════════════
// SENSOR READING
// ═══════════════════════════════════════════════════════════════════════════

void readBattery() {
  #ifdef VBAT_PIN
  if (VBAT_PIN >= 0) {
    // BUG FIX #6: Use pin-specific attenuation instead of global
    // This ensures proper ADC configuration regardless of other analog usage
    #if defined(BOARD_ESP32C3)
      // ESP32-C3: Use analogSetPinAttenuation for pin-specific config
      analogSetPinAttenuation(VBAT_PIN, ADC_11db);
    #elif defined(BOARD_ESP32WROOM) || defined(BOARD_ESP32CAM)
      // ESP32: Pin-specific attenuation
      analogSetPinAttenuation(VBAT_PIN, ADC_11db);
    #endif

    // Small delay after configuring ADC
    delay(5);

    uint32_t total = 0;
    const int samples = 10;
    uint32_t minReading = 4095;
    uint32_t maxReading = 0;

    for (int i = 0; i < samples; i++) {
      uint32_t reading = analogRead(VBAT_PIN);
      total += reading;
      if (reading < minReading) minReading = reading;
      if (reading > maxReading) maxReading = reading;
      delay(2);
    }

    uint32_t avgReading = total / samples;

    // BUG FIX #6: Validate ADC reading is plausible
    // If reading is 0 or max (4095), likely pin issue or not connected
    if (avgReading == 0) {
      DEBUG_PRINTLN(F("WARNING: Battery ADC reading is 0 - pin not connected?"));
      batteryMv = 0;  // Will be handled as "unknown" rather than "dead battery"
    } else if (avgReading >= 4090) {
      DEBUG_PRINTLN(F("WARNING: Battery ADC reading at max - check voltage divider"));
      batteryMv = 0;
    } else if ((maxReading - minReading) > 500) {
      // High variance suggests noise or floating pin
      DEBUG_PRINTF("WARNING: High ADC variance (min:%lu max:%lu) - noisy reading\n", minReading, maxReading);
      batteryMv = 0;
    } else {
      uint32_t adcMv = (avgReading * VBAT_REF_MV) / 4095;
      batteryMv = (uint16_t)(adcMv * VBAT_DIVIDER);
    }

    DEBUG_PRINTF("ADC: %lu (min:%lu max:%lu), Battery mV: %d\n", avgReading, minReading, maxReading, batteryMv);
  } else {
    batteryMv = 0;
  }
  #else
  batteryMv = 0;
  #endif
}

bool readReedSwitch() {
  int reading1 = digitalRead(REED_PIN);
  delay(DEBOUNCE_MS);
  int reading2 = digitalRead(REED_PIN);

  if (reading1 == reading2) {
    // Apply reed polarity setting from config.h
    // REED_ACTIVE_HIGH=true: HIGH = triggered (magnet away = flag UP = FISH ON!)
    // REED_ACTIVE_HIGH=false: LOW = triggered (magnet present = flag UP = FISH ON!)
    return REED_ACTIVE_HIGH ? (reading1 == HIGH) : (reading1 == LOW);
  }
  
  return lastReedState;
}

// ═══════════════════════════════════════════════════════════════════════════
// MESSAGE SENDING
// ═══════════════════════════════════════════════════════════════════════════

void sendStatus(uint8_t msgType) {
  if (!espNowReady) {
    DEBUG_PRINTLN(F("ESP-NOW not ready"));
    return;
  }
  
  // Increment sequence for every message
  messageSeq++;
  
  SensorMessage msg;
  msg.network_id = NETWORK_ID;
  msg.node_id = NODE_ID;
  msg.msg_type = msgType;
  msg.flags = currentFlags;
  msg.battery_mv = batteryMv;
  msg.sequence = messageSeq;
  // M3 FIX: uptime_sec is CUMULATIVE active time across all wake cycles
  // totalUptimeSec accumulates time from previous wakes, millis()/1000 adds current wake period
  msg.uptime_sec = totalUptimeSec + (millis() / 1000);

  #if DEBUG_ESPNOW
  DEBUG_PRINTF("TX [%s] ID:%d Seq:%d Flags:0x%02X Batt:%dmV\n",
               getMessageTypeName(msgType), msg.node_id, msg.sequence, msg.flags, msg.battery_mv);
  #endif

  // Use reliable send with retry and hardware ACK support
  bool success = espNowSendReliable((uint8_t*)&msg, sizeof(msg), 3);

  if (success) {
    blinkLed(1, 30, 0);
  }

  lastHeartbeat = millis();
}

void sendAlert() {
  if (!espNowReady) {
    DEBUG_PRINTLN(F("ESP-NOW not ready"));
    return;
  }

  // BUG FIX #6: When silenced, send STATUS instead of ALERT
  // This ensures gateway knows about FISH_ON flag (for fish_on_time tracking)
  // but doesn't trigger the audible buzzer (which is already silenced)
  if (alertsSilenced) {
    DEBUG_PRINTLN(F("Alert silenced - sending as STATUS"));
    sendStatus(MSG_STATUS);  // Still transmit the FISH_ON flag
    return;
  }

  // Increment sequence for every message
  messageSeq++;
  
  AlertMessage msg;
  msg.network_id = NETWORK_ID;
  msg.node_id = NODE_ID;
  msg.msg_type = MSG_ALERT;
  msg.flags = currentFlags;
  msg.battery_mv = batteryMv;
  msg.sequence = messageSeq;
  msg.uptime_sec = totalUptimeSec + (millis() / 1000);
  msg.alert_time = millis() / 1000;
  
  #if DEBUG_ESPNOW
  DEBUG_PRINTF("TX [ALERT!] ID:%d Seq:%d Flags:0x%02X\n", msg.node_id, msg.sequence, msg.flags);
  #endif

  // Use reliable send with retry and hardware ACK support
  espNowSendReliable((uint8_t*)&msg, sizeof(msg), 3);

  alertSentTime = millis();
  blinkLed(3, 50, 50);
}

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW CALLBACK
// ═══════════════════════════════════════════════════════════════════════════

void onEspNowSent(const uint8_t* mac, esp_now_send_status_t status) {
  sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  sendInProgress = false;

  #if DEBUG_ESPNOW
  DEBUG_PRINTF("ESP-NOW TX: %s\n", sendSuccess ? "OK" : "FAIL");
  #endif
}

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 3) return;

  uint8_t netId = data[0];
  uint8_t msgType = data[2];

  if (netId != NETWORK_ID) return;

  // BUG FIX #4: Handle ACK messages from gateway
  // This allows the sensor to exit the 30-second wait loop early and save battery
  if (msgType == MSG_ACK && len >= sizeof(AckMessage)) {
    AckMessage* ack = (AckMessage*)data;
    if (ack->ack_node_id == NODE_ID) {
      SET_FLAG(currentFlags, FLAG_ACKNOWLEDGED);
      DEBUG_PRINTF("Received ACK for msg type %d from node %d\n",
                   ack->ack_msg_type, ack->node_id);

      // Learn gateway MAC address for future unicast (more reliable)
      if (!gatewayMacKnown) {
        memcpy(gatewayMac, mac, 6);
        gatewayMacKnown = true;
        DEBUG_PRINTF("Learned gateway MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        // Add gateway as unicast peer for hardware ACK
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, gatewayMac, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.encrypt = false;

        // Remove if exists, then add fresh
        esp_now_del_peer(gatewayMac);
        if (esp_now_add_peer(&peer) == ESP_OK) {
          DEBUG_PRINTLN("Gateway added as unicast peer");
        }
      }
    }
    return;
  }

  if (msgType == MSG_SILENCE_SYNC) {
    if (len >= 12) {
      uint8_t silenceState = data[3];

      // AUDIT FIX: Use memcpy for safe unaligned memory access
      // Direct pointer casting to uint32_t* can cause alignment issues on some architectures
      uint32_t timestamp;
      uint32_t expireTime;
      memcpy(&timestamp, data + 4, sizeof(uint32_t));
      memcpy(&expireTime, data + 8, sizeof(uint32_t));

      alertsSilenced = (silenceState == 1);

      // BUG FIX #11 & #9: Store remaining seconds instead of absolute time
      // This survives deep sleep where millis() resets to 0
      if (alertsSilenced && expireTime > timestamp) {
        silenceRemainingSec = expireTime - timestamp;  // Seconds remaining
        DEBUG_PRINTF("Silenced for %lu seconds\n", silenceRemainingSec);
      } else {
        silenceRemainingSec = 0;
      }
    }
  }

  // Handle reset command from gateway
  if (msgType == MSG_RESET_CMD && len >= sizeof(ResetCmdMessage)) {
    ResetCmdMessage* resetMsg = (ResetCmdMessage*)data;
    DEBUG_PRINTF("Received RESET command from node %d, delay=%d sec\n",
                 resetMsg->sender_id, resetMsg->reset_delay_sec);

    // Sensor nodes reboot immediately (delay 0)
    DEBUG_PRINTLN(F("RESET CMD - rebooting now..."));
    Serial.flush();
    delay(100);
    ESP.restart();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// DEEP SLEEP
// ═══════════════════════════════════════════════════════════════════════════

void enterDeepSleep() {
  // BUG FIX #1: Determine if we should enable GPIO wakeup
  // If FISH_ON is already active, GPIO is already HIGH - enabling GPIO wakeup
  // on HIGH level would cause immediate wake, creating an infinite loop!
  bool enableGpioWake = !HAS_FLAG(currentFlags, FLAG_FISH_ON);

  if (enableGpioWake) {
    DEBUG_PRINTLN(F("Entering deep sleep..."));
    DEBUG_PRINTF("Wake in %d seconds or on reed switch (GPIO HIGH)\n", HEARTBEAT_INTERVAL_SEC);
  } else {
    DEBUG_PRINTLN(F("Entering deep sleep (FISH_ON active)..."));
    DEBUG_PRINTF("Wake in %d seconds ONLY (GPIO wakeup disabled to prevent loop)\n", HEARTBEAT_INTERVAL_SEC);
  }

  Serial.flush();
  delay(10);

  // BUG FIX #7: Record expected sleep duration for accurate timing on wake
  expectedSleepSec = HEARTBEAT_INTERVAL_SEC;

  // Timer wake - always enabled
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);

  // BUG FIX #1 & #2: GPIO wake (reed switch) - only enable when flag is DOWN
  // This prevents the infinite wake loop when FISH_ON is already active
  if (enableGpioWake) {
    #ifdef BOARD_ESP32C3
      // BUG FIX #2: ESP32-C3 uses GPIO deep sleep wakeup (not EXT0/EXT1)
      // Cross-compatible implementation for Arduino ESP32 core 2.x and 3.x
      #if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        // ESP-IDF 5.x (Arduino ESP32 core 3.x)
        esp_deep_sleep_enable_gpio_wakeup(1ULL << REED_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
      #else
        // ESP-IDF 4.x (Arduino ESP32 core 2.x)
        // Configure GPIO for deep sleep wakeup on ESP32-C3
        gpio_config_t config = {
          .pin_bit_mask = (1ULL << REED_PIN),
          .mode = GPIO_MODE_INPUT,
          .pull_up_en = GPIO_PULLUP_ENABLE,
          .pull_down_en = GPIO_PULLDOWN_DISABLE,
          .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&config);

        // Enable GPIO wakeup for deep sleep (HIGH level trigger)
        // Note: esp_sleep_enable_gpio_wakeup() works for deep sleep on ESP32-C3
        gpio_wakeup_enable((gpio_num_t)REED_PIN, GPIO_INTR_HIGH_LEVEL);
        esp_sleep_enable_gpio_wakeup();
      #endif
    #else
      // ESP32 (WROOM/CAM): Use EXT0 wakeup (RTC GPIO supported)
      esp_sleep_enable_ext0_wakeup((gpio_num_t)REED_PIN, 1);
    #endif
  }
  // If FISH_ON is active, we only use timer wakeup
  // On next timer wake, we'll re-check the reed switch and re-alert if still triggered

  // Properly shut down WiFi before sleep
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // Additional power saving: disable ADC and other peripherals
  #ifdef BOARD_ESP32C3
    // ESP32-C3 specific power down
    esp_wifi_stop();
  #endif

  esp_deep_sleep_start();
}

// ═══════════════════════════════════════════════════════════════════════════
// FAST DEEP SLEEP - Short sleep during FISH_ON monitoring
// ═══════════════════════════════════════════════════════════════════════════

void enterDeepSleepFast() {
  DEBUG_PRINTF("Fast sleep: waking in %d seconds\n", FISH_ON_HEARTBEAT_SEC);
  Serial.flush();
  delay(10);

  // Record expected sleep duration for timing calculations
  expectedSleepSec = FISH_ON_HEARTBEAT_SEC;

  // Timer wake only - short interval
  esp_sleep_enable_timer_wakeup(FISH_ON_HEARTBEAT_SEC * 1000000ULL);

  // NO GPIO wake during FISH_ON - pin is HIGH, would wake immediately
  // We rely on timer wake to poll the reed switch

  // Shut down WiFi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  #ifdef BOARD_ESP32C3
    esp_wifi_stop();
  #endif

  esp_deep_sleep_start();
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITIES
// ═══════════════════════════════════════════════════════════════════════════

void blinkLed(int times, int onMs, int offMs) {
  #ifdef LED_PIN
  if (LED_PIN < 0) return;

  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    if (offMs > 0 && i < times - 1) {
      delay(offMs);
    }
  }
  #endif
}

// ═══════════════════════════════════════════════════════════════════════════
// BUG FIX #5: RTC DATA VALIDATION
// ═══════════════════════════════════════════════════════════════════════════

bool validateAndInitRtcData() {
  // Check if RTC memory contains our magic number
  // If not, this is either first boot or RTC memory was corrupted
  if (rtcMagic != RTC_MAGIC_NUMBER) {
    // Reset all RTC variables to safe defaults
    rtcMagic = RTC_MAGIC_NUMBER;
    bootCount = 0;
    totalUptimeSec = 0;
    messageSeq = 0;
    lastReedState = false;
    expectedSleepSec = 0;
    lastWakeReason = ESP_SLEEP_WAKEUP_UNDEFINED;
    alertsSilenced = false;
    silenceRemainingSec = 0;
    fishOnMonitorMode = false;
    gatewayMacKnown = false;
    memset(gatewayMac, 0, 6);

    DEBUG_PRINTLN(F("RTC data initialized (first boot or corruption detected)"));
    return false;  // Data was not valid
  }

  // Additional sanity checks for plausible values
  bool dataValid = true;

  // H5 FIX: Sequence should not be at max (would indicate counter corruption)
  // Reset to 1 (not 0) because 0 could be misinterpreted as "older" than 65535
  // by some deduplication logic. Also set FLAG_FIRST_BOOT to signal gateway.
  if (messageSeq == 0xFFFF) {
    DEBUG_PRINTLN(F("WARNING: messageSeq at max - resetting to 1"));
    messageSeq = 1;  // H5 FIX: Reset to 1 to avoid "older" detection
    SET_FLAG(currentFlags, FLAG_FIRST_BOOT);  // H5 FIX: Signal sequence reset to gateway
    dataValid = false;
  }

  // Boot count sanity check (extremely high values might indicate corruption)
  if (bootCount > 1000000) {
    DEBUG_PRINTF("WARNING: bootCount unusually high (%lu) - possible corruption\n", bootCount);
    // Don't reset, but flag as potentially corrupted
    dataValid = false;
  }

  // silenceRemainingSec should be reasonable (max 24 hours = 86400 seconds)
  if (silenceRemainingSec > 86400) {
    DEBUG_PRINTF("WARNING: silenceRemainingSec too high (%lu) - clearing\n", silenceRemainingSec);
    silenceRemainingSec = 0;
    alertsSilenced = false;
    dataValid = false;
  }

  return dataValid;
}

// ═══════════════════════════════════════════════════════════════════════════
// BUG FIX #7: ESTIMATE ACTUAL SLEEP DURATION
// ═══════════════════════════════════════════════════════════════════════════

uint32_t estimateActualSleepDuration(esp_sleep_wakeup_cause_t wakeReason) {
  // Estimate how long we actually slept based on wake reason
  // This is an approximation since we don't have a persistent RTC clock

  switch (wakeReason) {
    case ESP_SLEEP_WAKEUP_TIMER:
      // Woke from timer - slept approximately the expected duration
      return expectedSleepSec;

    case ESP_SLEEP_WAKEUP_EXT0:
    case ESP_SLEEP_WAKEUP_GPIO:
      // Woke from GPIO - slept less than expected
      // Use half the expected duration as a reasonable estimate
      // This isn't perfect but prevents silence from lasting too long
      return expectedSleepSec / 2;

    case ESP_SLEEP_WAKEUP_EXT1:
      // EXT1 wakeup (multiple GPIO) - treat same as GPIO
      return expectedSleepSec / 2;

    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
      // Fresh boot or reset - no sleep time to account for
      return 0;
  }
}
