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
 * BUILD (PlatformIO, see platformio.ini at the project root):
 *   env sensor_c3 (ESP32-C3 Super Mini, USB CDC on boot) or sensor_wroom.
 *   BOARD_*, NODE_ID and NODE_NAME come from build_flags; the defaults below
 *   are only used when build_flags do not set them.
 *
 *   (Was sensor_node.ino for the Arduino IDE; converted to .cpp on 2026-10-05:
 *    only #include <Arduino.h> and overridable defines were added.)
 */

// ═══════════════════════════════════════════════════════════════════════════
// BOARD SELECTION - UNCOMMENT ONE
// ═══════════════════════════════════════════════════════════════════════════

#if !defined(BOARD_ESP32C3) && !defined(BOARD_ESP32CAM) && !defined(BOARD_ESP32WROOM)
#define BOARD_ESP32C3
// #define BOARD_ESP32CAM
// #define BOARD_ESP32WROOM
#endif

// ═══════════════════════════════════════════════════════════════════════════
// NODE CONFIGURATION - CHANGE THESE PER DEVICE
// ═══════════════════════════════════════════════════════════════════════════

#ifndef NODE_ID
#define NODE_ID     2               // <<< CHANGE THIS! Unique ID (1-254)
#endif
#ifndef NODE_NAME
#define NODE_NAME   "Hole 2"        // Name shown in web UI
#endif

// ═══════════════════════════════════════════════════════════════════════════
// INCLUDES
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
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
#include <hall_latch.h>         // v2 (D46): spool-shaft Hall latch
#include <arm_button.h>         // v2 (D61): arming button + armed / disarmed state
#include <driver/rtc_io.h>
#include <sys/time.h>
#include "sonar_tuss4470.h"    // v2 (D43): real sonar (TUSS4470), only with SONAR_REAL=1
#if defined(BTN_PIN) && BTN_PIN >= 0 && SONAR_REAL && (BTN_PIN == SONAR_PIN_BOOST || BTN_PIN == SONAR_PIN_O4)
#error "BTN_PIN collides with a sonar pin (the pod v0.3 env must set BTN_PIN=-1)"
#endif
#include "bench_web.h"         // v2 (D56): phone test page, bench build only
#include <sonar_sim.h>      // v2: fake sonar for the sonar test mode (lib/IceMesh)
#include <eb_link.h>        // v2: ESP-NOW backbone relay (lib/IceMesh)
#include <rx_ring.h>
#include <Preferences.h>

// ═══════════════════════════════════════════════════════════════════════════
// v2 BACKBONE RELAY (docs/protocol_v2.md §6c)
// Set from the chalet (relay on/off for this node ID, delivered by the hub as MSG_DEV_CMD right
// after this node transmits) and kept in NVS. While on, the node stays awake (no deep sleep:
// battery cost!) and rebroadcasts ESP-NOW backbone frames between hubs and the chalet.
// EB_RELAY_FORCE=1: a spare board that is always a relay.
// ═══════════════════════════════════════════════════════════════════════════
#ifndef EB_RELAY_FORCE
#define EB_RELAY_FORCE 0
#endif
RTC_DATA_ATTR bool ebRelayOn = false;       // copy of the NVS setting, read again on power-on only

// v2 SIMULATION of this hole (set from the chalet, delivered by the hub as MSG_DEV_CMD DEVCMD_SIM, kept in NVS):
// bit0 fake sonar (stays awake while the hub's sonar control says test mode), bit1 fake Hall-sensor trips:
// the flag "goes up" at random (rate = trips per hour), stays up 20-90 s, through the normal alert path.
// Messages carry FLAG_SIM so every screen marks the hole SIM.
RTC_DATA_ATTR uint8_t simBits = 0;
RTC_DATA_ATTR uint32_t simTripEnd = 0, simNextTrip = 0;   // in totalUptimeSec time
volatile int16_t devCmdSim = -1;
void simLoadSetting(bool power_on);
void simApplyCmd();
bool simTripActive();
volatile int8_t devCmdRelay = -1;
volatile int16_t devCmdBait = -1;            // v2 (D43): bait depth from the chalet (5 cm steps), applied by sonarLoop           // relay command received (-1 none, 0 off, 1 on)

// v2 (D61) ARMING BUTTON (config.h BTN_PIN, -1 = no button: the chalet's command still works) and the armed /
// disarmed state of this hole. Disarmed: spool movement is ignored (no FISH ON, no alert), heartbeats go on.
// Short press = arm / disarm (tripped: clears the trip instead), hold 3 s = status report now. Kept in RTC memory
// across deep sleep and in NVS ("node" / "armed") across reboots; button and DEVCMD_ARM both set it: last wins.
RTC_DATA_ATTR icemesh::ArmState armState;    // plain struct, zero = armed (like HallLatch)
volatile int8_t devCmdArm = -1;              // -1 none, 0 disarm, 1 arm
volatile bool devCmdSelfTest = false;        // D70: the chalet asked for the wiring check
RTC_DATA_ATTR uint16_t btnPresses = 0;       // D70: button events since power-on
void selfTestRun(bool toSerial);             // D70: run + send MSG_SELFTEST to the hub (and print)
#if BTN_PIN >= 0
static icemesh::ArmButton armBtn;
#endif
void armLoadSetting(bool power_on);
void armApplyCmd();
void armPollFor(uint32_t ms);                // delay(ms) that watches the button and the chalet's command
void armOnWake();
void armBtnWake();
static void armFlags();
static icemesh::RxRing<8, 250> ebRing;
static uint32_t ebRelayed = 0;
void ebRelayLoadSetting(bool power_on);
void ebRelayApplyCmd();
void ebRelayLoop();

// ═══════════════════════════════════════════════════════════════════════════
// v2 SONAR TEST MODE (see docs/SONAR_SIM.md)
// The chalet turns it on; hubs relay the switch (MSG_SONAR_CTRL). While it is on, this node stays
// awake (no deep sleep: battery cost!) and sends fake sonar blocks (MSG_SONAR) to its hub:
// a summary every 4 s, or the full stream when the chalet picks this hole as FOCUS.
// ═══════════════════════════════════════════════════════════════════════════
#ifndef SONAR_SIM_ALLOWED
#define SONAR_SIM_ALLOWED 1         // 0 = this node ignores the sonar test mode
#endif
#ifndef SONAR_SIM_FORCE
#define SONAR_SIM_FORCE 0           // 1 = bench test: fake sonar from boot, no chalet needed
#endif
#ifndef SONAR_BENCH
#define SONAR_BENCH 0               // 1 = bucket test (with SONAR_REAL): real sonar from boot, no hub needed,
                                    //     stays awake, prints a BENCH line every 2 s (docs/SONAR_DRIVER.md)
#endif
#define SONAR_CTRL_WAIT_MS     300  // listen after each wake-up TX (D79: was 120, a busy hub loop missed it)
#define SONAR_CTRL_TIMEOUT_MS  150000UL  // no control for this long: sonar off, back to sleep (D79: 2.5 x the heartbeat; the hub
                                         // also repeats the control every 10 s while it hears sonar from us)

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
#if HALL_LATCH
RTC_DATA_ATTR icemesh::HallLatch hall;            // v2 (D46): plain struct, zero at power-on, kept in deep sleep
RTC_DATA_ATTR uint32_t hallTurnsBase = 0;         // D69: hall.total when armed / trip cleared: the run is counted from here
static volatile uint32_t hallEdges = 0;           // counted by the pin interrupt while awake
static void IRAM_ATTR hallIsr() { hallEdges++; }
static uint64_t hallNowMs() {                     // system time keeps running through deep sleep (RTC timer)
  struct timeval tv; gettimeofday(&tv, nullptr);
  return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}
static void hallPoll() {
  noInterrupts(); const uint32_t n = hallEdges; hallEdges = 0; interrupts();
  hall.edges(n, (uint8_t)digitalRead(REED_PIN), hallNowMs());
}
// before deep sleep: wake on the NEXT flip (the opposite level), pull-up kept by the RTC IO
static void hallArmWake() {
  detachInterrupt(digitalPinToInterrupt(REED_PIN));
  hallPoll();
  esp_sleep_enable_ext0_wakeup((gpio_num_t)REED_PIN, hall.wakeLevel());
  rtc_gpio_pullup_en((gpio_num_t)REED_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)REED_PIN);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);   // RTC pull-up needs it (ext0 keeps it on too)
}
#else
static inline void hallArmWake() {}
#endif
RTC_DATA_ATTR bool sonarSimOn = false;              // v2 sonar test mode (survives a reboot loop guard)
volatile bool sonarCtrlSeen = false;
volatile bool sonarCtrlEver = false;               // any sonar control message since boot
static bool g_sonarHwTried = false, g_sonarHwOk = false;
static bool sonarHwBegin();                        // tuss::begin() once; true = TUSS4470 answers
volatile bool sonarCtrlSim = false;
volatile uint8_t sonarCtrlFocus = 0;
// v2 (D42): sonar knobs from the hub's sonar control (the chalet's set), applied by sonarLoop and saved
volatile bool sonarKnobsNew = false;
uint8_t sonarKnobsBuf[SONAR_CTRL_PARAMS];
uint8_t sonarKnobsN = 0;

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
// D54: radio-quiet sonar capture. Every ESP-NOW frame handed to the radio is counted here and in the send
// callback; before a ping the capture waits until the radio has nothing in flight (TX bursts show up in the
// I2S-ADC samples). RX cannot be gated; only TX is.
volatile uint32_t g_txQueued = 0, g_txDone = 0; volatile unsigned long g_txDoneMs = 0;
static uint32_t g_rfWaitN = 0, g_rfWaitUsMax = 0, g_rfBusyAtPing = 0;
static inline esp_err_t nowSend(const uint8_t* mac, const uint8_t* data, size_t len) {
  const esp_err_t r = esp_now_send(mac, data, len);
  if (r == ESP_OK) g_txQueued++;
  return r;
}
volatile bool sendSuccess = false;
volatile uint8_t g_ucFails = 0;   // D79 (E7): unicast failures in a row (sonar frames to a hub that went away)

// ═══════════════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════

void setupPins();
void sonarListenCtrl(uint32_t wait_ms);
void sonarLoop();
void sonarSerial();
void sonarRecord(const uint8_t codes[icemesh::sonar::sp::NFREQ][icemesh::sonar::BINS], const tuss::PingInfo& pi);
void setupEspNow();
void readBattery();
bool readReedSwitch();
void sendStatus(uint8_t msgType);
void sendAlert();
static uint16_t turnsSinceArm();   // D69
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
  #if !DEBUG_SERIAL && SONAR_REAL
  Serial.begin(DEBUG_BAUD);   // D56: sonar builds always open the port (BENCH lines, REC/STAT/KNOB/NOTE/RAW commands)
  #endif
  #if DEBUG_SERIAL
  Serial.begin(DEBUG_BAUD);
  delay(100);
  #endif
  #if DEBUG_SERIAL || SONAR_REAL
  Serial.setTimeout(20);      // readStringUntil() must not stall the loop for the default 1 s
  #endif
  #if DEBUG_SERIAL
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
      reedTriggered = !HALL_LATCH;   // latch: one flip is not a trip by itself (TRIGGER_FLIPS), the check below decides
      break;
    case ESP_SLEEP_WAKEUP_GPIO:
      DEBUG_PRINTLN(F("Wake: Reed switch triggered (GPIO)!"));
      reedTriggered = true;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      DEBUG_PRINTLN(F("Wake: Heartbeat timer"));
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      DEBUG_PRINTLN(F("Wake: arming button (EXT1)"));   // D61: handled once ESP-NOW is up (armOnWake)
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
  const bool powerOn = wakeReason != ESP_SLEEP_WAKEUP_EXT0 && wakeReason != ESP_SLEEP_WAKEUP_GPIO &&
                       wakeReason != ESP_SLEEP_WAKEUP_TIMER && wakeReason != ESP_SLEEP_WAKEUP_EXT1;
  ebRelayLoadSetting(powerOn);
  simLoadSetting(powerOn);
  if (simBits) SET_FLAG(currentFlags, FLAG_SIM);
  armLoadSetting(powerOn);   // D61

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
  if (!armState.armed()) reedTriggered = false;   // D61: disarmed, a pin wake is not an alert (reed boards)
  
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

  #if BTN_PIN >= 0
  if (wakeReason == ESP_SLEEP_WAKEUP_EXT1) armOnWake();   // D61: the press that woke us (may still be held)
  #endif

  // Send message based on wake reason
  if (reedTriggered) {
    DEBUG_PRINTLN(F(">>> Sending ALERT"));
    sendAlert();
  } else {
    DEBUG_PRINTLN(F(">>> Sending HEARTBEAT"));
    sendStatus(MSG_HEARTBEAT);
  }

  delay(50);  // P3 FIX: Reduced from 100ms to 50ms - adequate for ESP-NOW TX completion
  armApplyCmd();   // D61: the hub answers our message with pending commands; a disarm ends a trip at once

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

      // Check reed switch less frequently to save power (every 500ms); the button is watched meanwhile (D61)
      armPollFor(ALERT_REED_CHECK_MS);

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
  
  // v2: the hub answers our TX with its sonar control and pending commands (backbone relay).
  // Sonar test mode or relay on: stay awake, loop() keeps reed, heartbeat and alerts working.
  if (SONAR_SIM_ALLOWED && (SONAR_SIM_FORCE || SONAR_BENCH)) sonarSimOn = true;
  else {
    sonarListenCtrl(SONAR_CTRL_WAIT_MS);
    // D43/D58: a node with a working TUSS4470 runs its real sonar whenever its hub talks to it (the simulation bit
    // only decides fake vs real data). Before this check the real build went back to sleep here every time.
    if (SONAR_REAL && sonarCtrlEver && !sonarSimOn && sonarHwBegin()) sonarSimOn = true;
  }
  ebRelayApplyCmd();
  simApplyCmd();
  armApplyCmd();
  if (SONAR_SIM_ALLOWED && sonarSimOn && SLEEP_ENABLED) {
    DEBUG_PRINTLN(F("Sonar ON - staying awake"));
    return;
  }
  if (ebRelayOn && SLEEP_ENABLED) {
    DEBUG_PRINTLN(F("Backbone relay ON - staying awake"));
    return;
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
  #if SONAR_SIM_ALLOWED
  sonarLoop();
  #endif
  ebRelayLoop();
  simApplyCmd();
  armApplyCmd();

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
      CLEAR_FLAG(currentFlags, FLAG_ACKNOWLEDGED);   // D79 (E3): every trip is acked on its own; the retry loop below runs again
      alertRetryCount = 0;
      sendAlert();
    } else {
      DEBUG_PRINTLN(F("Flag reset - FISH OFF"));
      CLEAR_FLAG(currentFlags, FLAG_FISH_ON);
      CLEAR_FLAG(currentFlags, FLAG_ACKNOWLEDGED);
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
  
  armPollFor(DEBOUNCE_MS);   // D61: the loop pause doubles as the button poll
}

// ═══════════════════════════════════════════════════════════════════════════
// PIN SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setupPins() {
  // Configure reed switch input with internal pull-up
  // When magnet is present (flag down): switch closed, reads LOW
  // When magnet absent (flag up/FISH ON): switch open, reads HIGH
#if HALL_LATCH
  // the pad is still in RTC mode after an ext0 wake: back to a normal input with pull-up, then count
  // the flips that happened while asleep, then count edges while awake
  rtc_gpio_deinit((gpio_num_t)REED_PIN);
  pinMode(REED_PIN, INPUT_PULLUP);
  delayMicroseconds(50);   // pull-up settles (open-drain output)
  hall.settings(TRIGGER_FLIPS, HALL_WINDOW_MS, (uint32_t)HALL_CLEAR_SEC * 1000UL);
  hall.boot((uint8_t)digitalRead(REED_PIN), lastWakeReason == ESP_SLEEP_WAKEUP_EXT0, hallNowMs());
  hallEdges = 0;
  attachInterrupt(digitalPinToInterrupt(REED_PIN), hallIsr, CHANGE);
  DEBUG_PRINTF("Hall latch: level %u, %lu flips since power-on, trip %s\n", hall.level, (unsigned long)hall.total, hall.on ? "ON" : "off");
#else
  pinMode(REED_PIN, INPUT_PULLUP);
#endif

  #if BTN_PIN >= 0
  rtc_gpio_deinit((gpio_num_t)BTN_PIN);   // D61: back from the RTC mode of the ext1 wake
  pinMode(BTN_PIN, INPUT_PULLUP);
  #endif

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

  // v2: ESP-NOW at the normal rate (config.h, D28). Range comes from the antenna height (mast ≥ 20 cm):
  // at ~5 cm over the ice the surface reflection cancels most of the signal (est. ~24 dB lost at 50 m),
  // far more than LR adds. LR-only and normal devices cannot hear each other.
  // v2: maximum TX power in every mode (was only set in LR mode)
  esp_err_t txResult = esp_wifi_set_max_tx_power(84);
  if (txResult != ESP_OK) {
    DEBUG_PRINTF("WARNING: Failed to set TX power: %d\n", txResult);
  } else {
    int8_t actualPower;
    esp_wifi_get_max_tx_power(&actualPower);
    DEBUG_PRINTF("TX power set to: %.2f dBm\n", actualPower * 0.25);
  }

  #if ESPNOW_LONG_RANGE_MODE
  // range-comparison build only (config.h): every hub and tip-up must use the same setting
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

      esp_err_t result = nowSend(gatewayMac, data, len);

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
        sendInProgress = false;   // no callback will come for a refused send
        DEBUG_PRINTF("ESP-NOW: Send error %d (attempt %d)\n", result, retry + 1);
        delay(20 + random(20));
      }
    }

    // If unicast failed completely, clear gateway MAC and fall back to broadcast
    if (!success) {
      DEBUG_PRINTLN("ESP-NOW: Unicast failed - clearing gateway MAC, using broadcast");
      gatewayMacKnown = false; g_ucFails = 0;
      memset(gatewayMac, 0, 6);
    }
  }

  // Fall back to broadcast with retry (no hardware ACK, but 2x helps)
  if (!success) {
    DEBUG_PRINTLN("ESP-NOW: Using broadcast with retry");

    for (int retry = 0; retry < 2; retry++) {
      sendInProgress = true;
      sendSuccess = false;

      esp_err_t result = nowSend(ESPNOW_BROADCAST, data, len);

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
      } else sendInProgress = false;   // refused send: no callback will come

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

static bool readReedHw();
static void armDropTrip();
bool readReedSwitch() {   // v2: a simulated trip looks like the real flag
  const bool t = readReedHw() || simTripActive();
  if (armState.armed()) return t;
  if (t) armDropTrip();   // D61 disarmed: movement is ignored and nothing stays latched for the re-arm
  return false;
}

static bool readReedHw() {
#if HALL_LATCH
  hallPoll();
  return hall.active(hallNowMs());
#endif
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
  msg.turns = turnsSinceArm();   // D69

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
  msg.turns = turnsSinceArm();   // D69
  
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
  if (mac != nullptr && gatewayMacKnown && memcmp(mac, gatewayMac, 6) == 0) { if (sendSuccess) g_ucFails = 0; else if (g_ucFails < 255) g_ucFails++; }
  sendInProgress = false;
  g_txDone++; g_txDoneMs = millis();

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
        gatewayMacKnown = true; g_ucFails = 0;   // D79: a fresh start for the unicast failure count
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

  // v2 backbone frames: only relays care (copied here, handled in loop())
  if (msgType == icemesh::eb::MSG_EB_BEACON || msgType == icemesh::eb::MSG_EB_HUB) {
    if (ebRelayOn) ebRing.push(mac, data, len, 0);
    return;
  }

  // v2 device command for this node (backbone relay on/off)
  if (msgType == MSG_DEV_CMD && len >= (int)sizeof(DevCmdMessage)) {
    const DevCmdMessage* c = (const DevCmdMessage*)data;
    if (c->target == NODE_ID && c->cmd == DEVCMD_RELAY) devCmdRelay = c->value ? 1 : 0;
    if (c->target == NODE_ID && c->cmd == DEVCMD_SIM) devCmdSim = c->value;
    if (c->target == NODE_ID && c->cmd == DEVCMD_BAIT) devCmdBait = c->value;
    if (c->target == NODE_ID && c->cmd == DEVCMD_ARM) devCmdArm = c->value ? 1 : 0;   // D61
    if (c->target == NODE_ID && c->cmd == DEVCMD_SELFTEST) devCmdSelfTest = true;      // D70
    return;
  }

  // v2 sonar control from the hub (test mode switch + FOCUS node)
  if (msgType == MSG_SONAR_CTRL && len >= (int)sizeof(SonarCtrlMessage)) {
    const SonarCtrlMessage* c = (const SonarCtrlMessage*)data;
    sonarCtrlFocus = c->focus_node;
    sonarCtrlSim = (c->sim_on != 0);
    if (c->n_params > 0 && c->n_params <= SONAR_CTRL_PARAMS) {
      memcpy(sonarKnobsBuf, c->params, c->n_params); sonarKnobsN = c->n_params; sonarKnobsNew = true;
    }
    sonarCtrlSeen = true; sonarCtrlEver = true;
    return;
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
  // latch (D46): always wake on the next flip (opposite level: no wake loop); reed: only while the flag is down
  bool enableGpioWake = !HALL_LATCH && !HAS_FLAG(currentFlags, FLAG_FISH_ON);
  if (HALL_LATCH) hallArmWake();

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

  armBtnWake();   // D61: the button wakes us too (ext1, beside the Hall ext0)
  if (SONAR_REAL) tuss::sleep();   // shield stays powered (no enable wire): low-power mode over SPI
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
  // Latch (D46): wake on the next flip is safe (opposite level): line still running = counted at once
  if (HALL_LATCH) hallArmWake();

  // Shut down WiFi
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  #ifdef BOARD_ESP32C3
    esp_wifi_stop();
  #endif

  armBtnWake();   // D61
  if (SONAR_REAL) tuss::sleep();   // shield stays powered (no enable wire): low-power mode over SPI
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
#if HALL_LATCH
    hall = icemesh::HallLatch();   // garbage here = a false trip at boot
#endif
    armState = icemesh::ArmState();   // D61: zero = armed (NVS value loaded on power-on)
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

// ═══════════════════════════════════════════════════════════════════════════
// v2 SONAR TEST MODE
// ═══════════════════════════════════════════════════════════════════════════

static bool sonarHwBegin() {
  if (!SONAR_REAL) return false;
  if (!g_sonarHwTried) {
    g_sonarHwTried = true; g_sonarHwOk = tuss::begin();
    DEBUG_PRINTF("Sonar TUSS4470: %s\n", g_sonarHwOk ? "found" : "NOT answering (check wiring / SONAR_PIN_*)");
  }
  return g_sonarHwOk;
}
void sonarListenCtrl(uint32_t wait_ms) {
  const unsigned long t0 = millis();
  while (!sonarCtrlSeen && millis() - t0 < wait_ms) delay(5);
  if (sonarCtrlSeen) { sonarCtrlSeen = false; if (SONAR_SIM_ALLOWED) sonarSimOn = sonarCtrlSim && (simBits & 0x01); }
}

// ═══════════════════════════════════════════════════════════════════════════
// v2 SIMULATION OF THIS HOLE
// ═══════════════════════════════════════════════════════════════════════════
void simLoadSetting(bool power_on) {
  if (!power_on) return;
  Preferences p;
  p.begin("node", true);
  simBits = p.getUChar("sim", 0);
  p.end();
  simTripEnd = 0; simNextTrip = 0;
  if (simBits) DEBUG_PRINTF("Simulation: sonar %s, Hall trips %s\n", (simBits & 1) ? "on" : "off", (simBits & 2) ? "on" : "off");
}

void simApplyCmd() {
  const int16_t c = devCmdSim;
  if (c < 0) return;
  devCmdSim = -1;
  const uint8_t v = static_cast<uint8_t>(c);
  if (v == simBits) return;
  simBits = v; simTripEnd = 0; simNextTrip = 0;
  if (v) SET_FLAG(currentFlags, FLAG_SIM); else CLEAR_FLAG(currentFlags, FLAG_SIM);
  if (!(v & 0x01) && !SONAR_BENCH && !SONAR_SIM_FORCE && !g_sonarHwOk) sonarSimOn = false;   // a real sonar keeps running
  Preferences p;
  p.begin("node", false);
  p.putUChar("sim", v);
  p.end();
  DEBUG_PRINTF("Simulation set by the chalet: sonar %s, Hall trips %s\n", (v & 1) ? "on" : "off", (v & 2) ? "on" : "off");
}

// Simulated Hall trip: random start (mean 3600 / rate s apart), flag up for 20-90 s.
bool simTripActive() {
  if (!(simBits & 0x02)) return false;
  const uint32_t now = totalUptimeSec + millis() / 1000;
  if (simTripEnd != 0) {
    if (now < simTripEnd) return true;
    simTripEnd = 0; simNextTrip = 0;
  }
  const uint32_t rate = (simBits >> 2) ? (simBits >> 2) : 6;
  const uint32_t mean = 3600UL / rate;
  if (simNextTrip == 0) simNextTrip = now + mean / 2 + esp_random() % (mean + 1);
  if (now >= simNextTrip) {
    simTripEnd = now + 20 + esp_random() % 71;
    DEBUG_PRINTLN(F("Simulation: Hall trip (fake fish)"));
    return true;
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// v2 (D61) ARMING BUTTON / ARMED STATE
// ═══════════════════════════════════════════════════════════════════════════
static void armFlags() {
  if (armState.armed()) CLEAR_FLAG(currentFlags, FLAG_DISARMED); else SET_FLAG(currentFlags, FLAG_DISARMED);
}

void armLoadSetting(bool power_on) {
  if (power_on) {
    Preferences p;
    p.begin("node", true);
    armState.set(p.getBool("armed", true));
    p.end();
  }
  armFlags();
  if (!armState.armed()) DEBUG_PRINTLN(F("Hole DISARMED (button / chalet)"));
}

static void armSave() {
  Preferences p;
  p.begin("node", false);
  p.putBool("armed", armState.armed());
  p.end();
}

// the trip of this hole ends now (same as the line going still): latch, simulation, flag
static uint16_t turnsSinceArm() {   // D69: quarter turns reported to the hub (0 on reed heads)
#if HALL_LATCH
  const uint32_t n = hall.total - hallTurnsBase;
  return n > 65535 ? 65535 : (uint16_t)n;
#else
  return 0;
#endif
}
static void armDropTrip() {
#if HALL_LATCH
  hall.on = false; hall.run = 0;
  hallTurnsBase = hall.total;   // D69: arm / disarm / clear restart the run count
#endif
  simTripEnd = 0; simNextTrip = 0;
  CLEAR_FLAG(currentFlags, FLAG_FISH_ON);
  reedTriggered = false;
}

// LED feedback: armed = 2 short, disarmed = 1 long, trip cleared = 1 short, report sent = 3 short
static void armLed(icemesh::ArmAction a) {
  switch (a) {
    case icemesh::ARM_TOGGLE_ARM: if (armState.armed()) blinkLed(2, 80, 120); else blinkLed(1, 600, 0); break;
    case icemesh::ARM_CLEAR_TRIP: blinkLed(1, 80, 0); break;
    case icemesh::ARM_REPORT:     blinkLed(3, 80, 120); break;
    default: break;
  }
}

// what the state change means for this cycle, then tell the hub right away (the chalet shows it)
static void armChanged(bool remote) {
  armFlags();
  armDropTrip();   // both ways: a disarm drops the trip, an arm drops flips counted while disarmed
  armSave();
  DEBUG_PRINTF("Hole %s (%s)\n", armState.armed() ? "ARMED" : "DISARMED", remote ? "chalet" : "button");
  if (espNowReady) sendStatus(MSG_STATUS);
}

static void armAct(icemesh::ArmAction a) {
  if (a != icemesh::ARM_NONE) btnPresses++;   // D70
  switch (a) {
    case icemesh::ARM_TOGGLE_ARM: armChanged(false); break;
    case icemesh::ARM_CLEAR_TRIP:
      armDropTrip();
      DEBUG_PRINTLN(F("Trip cleared by the button"));
      if (espNowReady) sendStatus(MSG_STATUS);
      break;
    case icemesh::ARM_REPORT:
      readBattery();
      if (espNowReady) sendStatus(MSG_STATUS);
      DEBUG_PRINTLN(F("Status report (button held)"));
      break;
    default: return;
  }
  armLed(a);
}

void armApplyCmd() {
  if (devCmdSelfTest) { devCmdSelfTest = false; selfTestRun(true); }   // D70: answered right away, the hub is listening
  const int8_t c = devCmdArm;
  if (c < 0) return;
  devCmdArm = -1;
  const bool on = (c == 1);
  if (on == armState.armed()) return;
  armState.set(on);
  armChanged(true);
  armLed(icemesh::ARM_TOGGLE_ARM);
}

#if BTN_PIN >= 0
static void armPoll() {
  armAct(armState.apply(armBtn.feed(digitalRead(BTN_PIN) == LOW, millis()), HAS_FLAG(currentFlags, FLAG_FISH_ON)));
}

void armPollFor(uint32_t ms) {
  const unsigned long t0 = millis();
  do { armPoll(); armApplyCmd(); delay(10); } while (millis() - t0 < ms);
}

// woke by the button: already released = a short press (boot took longer than the press); still held = time
// it from now (the ~0.3 s of boot, est., is not counted: a hold must reach 3 s from here)
void armOnWake() {
  armBtn = icemesh::ArmButton();
  const bool tripped = HAS_FLAG(currentFlags, FLAG_FISH_ON);
  if (digitalRead(BTN_PIN) != LOW) { armAct(armState.apply(icemesh::ARM_EV_SHORT, tripped)); return; }
  const unsigned long t0 = millis();
  while (millis() - t0 < icemesh::ArmButton::HOLD_MS + 500UL) {
    const icemesh::ArmEvent e = armBtn.feed(digitalRead(BTN_PIN) == LOW, millis());
    if (e != icemesh::ARM_EV_NONE) { armAct(armState.apply(e, tripped)); return; }   // SHORT on release, LONG at 3 s
    delay(10);
  }
}

// before deep sleep: ext1 wake on the button (low), pull-up kept by the RTC IO. The Hall latch keeps its ext0
// wake (the ESP32 runs both); the RTC peripherals stay on for the pull-ups (hallArmWake sets that too).
void armBtnWake() {
  // a hold-for-report still held at sleep time would re-wake at once and its release would read as a short
  // press (toggle): wait for the release first (bounded)
  const unsigned long t0 = millis(); unsigned long up = 0;
  while (millis() - t0 < 5000UL) {
    if (digitalRead(BTN_PIN) == LOW) up = 0; else if (!up) up = millis(); else if (millis() - up >= icemesh::ArmButton::BOUNCE_MS) break;
    delay(5);
  }
  rtc_gpio_pullup_en((gpio_num_t)BTN_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)BTN_PIN);
  esp_sleep_enable_ext1_wakeup(1ULL << BTN_PIN, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}
#else
void armPollFor(uint32_t ms) { const unsigned long t0 = millis(); do { armApplyCmd(); delay(10); } while (millis() - t0 < ms); }
void armOnWake() {}
void armBtnWake() {}
#endif

// ═══════════════════════════════════════════════════════════════════════════
// v2 BACKBONE RELAY
// ═══════════════════════════════════════════════════════════════════════════
void ebRelayLoadSetting(bool power_on) {
  if (power_on) {
    Preferences p;
    p.begin("node", true);
    ebRelayOn = p.getBool("ebRelay", false);
    p.end();
  }
  if (EB_RELAY_FORCE) ebRelayOn = true;
  if (ebRelayOn) DEBUG_PRINTLN(F("Backbone relay: ON"));
}

void ebRelayApplyCmd() {
  const int8_t c = devCmdRelay;
  if (c < 0) return;
  devCmdRelay = -1;
  const bool on = (c == 1) || EB_RELAY_FORCE;
  if (on == ebRelayOn) return;
  ebRelayOn = on;
  Preferences p;
  p.begin("node", false);
  p.putBool("ebRelay", on);
  p.end();
  DEBUG_PRINTF("Backbone relay %s (set by the chalet)\n", on ? "ON" : "off");
}

void ebRelayLoop() {
  static icemesh::eb::Dedup dedup;
  static icemesh::RxRing<8, 250>::Frame f;
  static unsigned long lastReport = 0;
  ebRelayApplyCmd();
  uint8_t budget = 8;
  while (ebRelayOn && budget-- && ebRing.pop(f)) {
    icemesh::eb::Header h;
    const uint8_t* pl;
    size_t n;
    if (!icemesh::eb::decode(f.data, f.len, NETWORK_ID, h, pl, n)) continue;
    if (dedup.seen(h.origin, h.type, h.seq)) continue;
    if (icemesh::eb::prepareRelay(f.data, f.len, NODE_ID) && nowSend(ESPNOW_BROADCAST, f.data, f.len) == ESP_OK) ebRelayed++;
  }
  if (ebRelayOn && millis() - lastReport >= 60000UL) {
    lastReport = millis();
    DEBUG_PRINTF("Backbone relay: %lu frames relayed\n", (unsigned long)ebRelayed);
  }
  // relay turned off (and no sonar test mode): back to the normal deep-sleep cycle
  static bool awakeForRelay = false;
  if (ebRelayOn) awakeForRelay = true;
  if (awakeForRelay && !ebRelayOn && SLEEP_ENABLED && !(SONAR_SIM_ALLOWED && sonarSimOn)) {
    awakeForRelay = false;
    totalUptimeSec += millis() / 1000;
    if (HAS_FLAG(currentFlags, FLAG_FISH_ON)) { fishOnMonitorMode = true; enterDeepSleepFast(); }
    else { fishOnMonitorMode = false; enterDeepSleep(); }
  }
}

// v2 (D43) recording + bench commands on the serial port (real sonar builds), D56: + phone page (bench build),
// NOTE / RAW / INFO / KNOBS / PINGX lines. Recording lines (serial "REC ON" and the phone page share the format):
//   INFO  <ms> key=value ...            build, pins, ADC rate, bins, calibration, battery (at every start)
//   KNOBS <ms> key=value ...            all knobs (at start and after every change)
//   SONAR <ms> <nfreq> <rot_f> <t0> <edge_um> <raw_max> <us> <hex codes freq 0> [<hex freq 1> <hex freq 2>]
//   PINGX <ms> key=value ...            the same ping: measured ADC rate, faults, radio wait, processing result
//   RAW   <ms> <burst_hz> <adc_hz> <t0> <n> <3 hex digits per sample>   one full capture (on demand / every N s)
//   NOTE  <ms> <text>                   free text (phone page: prefixed with the phone's date and time)
// tools/sonar_replay.cpp replays the SONAR lines through the same processing on the PC and skips the others.
static bool sonarRec = false;          // serial recording (REC ON)
static bool sonarRecWeb = false;       // phone page recording (REC WEB ON), bench build
static uint16_t sonarRecEvery = 1;     // record every Nth ping
static uint16_t sonarRawEverySec = 30; // RAW line every N s while recording (0 = only on demand)
static bool sonarRawAsk = false;
static uint32_t sonarRecLines = 0;
static bool sonarRecAny() { return sonarRec || sonarRecWeb; }
// one recording line to every active sink. force = also when not recording (NOTE, replies)
static void recLine(const char* line, bool force = false) {
  if (sonarRec || force) Serial.println(line);
  if ((sonarRecWeb || force) && SONAR_BENCH) bench::push(line);
  if (sonarRecAny()) sonarRecLines++;
}
static void recInfo() {
  uint16_t cal[17]; tuss::getCalibration(cal);
  char name[24]; snprintf(name, sizeof(name), "%s", NODE_NAME);
  for (char* c = name; *c; c++) if (*c == ' ' || *c == '=') *c = '_';   // key=value parsing: no spaces
  char buf[512];
  int k = snprintf(buf, sizeof(buf), "INFO %lu build=%s_%s node=%d name=%s adc_hz=%lu adc_clk_hz=%lu adc_meas_hz=%lu bin_m=0.025 bins=%d freqs=%lu,%lu,%lu "
                   "mv_per_db=%.1f adc_fs_mv=%.0f pins=sck%d,miso%d,mosi%d,cs%d,io1_%d,io2_%d,vout%d,o4_%d vbat_mv=%u heap=%lu cal=",
                   millis(), __DATE__, __TIME__, NODE_ID, name, (unsigned long)(tuss::clockHz() ? tuss::clockHz() : SONAR_ADC_HZ), (unsigned long)tuss::clockHz(), (unsigned long)tuss::measuredHz(), icemesh::sonar::BINS,
                   (unsigned long)SONAR_FREQ_HZ_0, (unsigned long)SONAR_FREQ_HZ_1, (unsigned long)SONAR_FREQ_HZ_2,
                   (double)SONAR_MV_PER_DB, (double)SONAR_ADC_FS_MV, SONAR_PIN_SCK, SONAR_PIN_MISO, SONAR_PIN_MOSI, SONAR_PIN_CS,
                   SONAR_PIN_IO1, SONAR_PIN_IO2, SONAR_PIN_VOUT, SONAR_PIN_O4, batteryMv, (unsigned long)ESP.getFreeHeap());
  for (int i = 0; i < 17 && k > 0 && k < (int)sizeof(buf) - 8; i++) k += snprintf(buf + k, sizeof(buf) - k, i ? ",%u" : "%u", cal[i]);
  recLine(buf);
}
static void recKnobs(const icemesh::sonar::Params& knobs) {
  char buf[400]; int k = snprintf(buf, sizeof(buf), "KNOBS %lu", millis());
  for (uint8_t i = 0; i < icemesh::sonar::P_COUNT && k > 0 && k < (int)sizeof(buf) - 24; i++)
    k += snprintf(buf + k, sizeof(buf) - k, " %s=%u", icemesh::sonar::paramInfo(i).key, knobs[i]);
  recLine(buf);
}
void sonarRecord(const uint8_t codes[icemesh::sonar::sp::NFREQ][icemesh::sonar::BINS], const tuss::PingInfo& pi) {
  if (!sonarRecAny()) return;
  static const char H[] = "0123456789ABCDEF";
  static char line[96 + 3 * (2 * icemesh::sonar::BINS + 1)];
  int k = snprintf(line, 96, "SONAR %lu %u %d %d %ld %u %lu", millis(), pi.nfreq, pi.rot_f, pi.t0, (long)pi.edge_um, pi.raw_max, (unsigned long)pi.us);
  for (uint8_t f = 0; f < pi.nfreq && f < icemesh::sonar::sp::NFREQ; f++) {
    line[k++] = ' ';
    for (int b = 0; b < icemesh::sonar::BINS; b++) { line[k++] = H[codes[f][b] >> 4]; line[k++] = H[codes[f][b] & 15]; }
  }
  line[k] = 0;
  recLine(line);
}
static void recPingX(const tuss::PingInfo& pi, const icemesh::sonar::SonarProc::Out& o) {
  if (!sonarRecAny()) return;
  char buf[640];
  int k = snprintf(buf, sizeof(buf), "PINGX %lu adc_hz=%lu clk_ok=%u nolimit=%u t0_ok=%u fault=%u us=%lu rf_wait_n=%lu rf_busy=%lu bottom=%.3f bot_snr=%.1f nf=%.1f "
                   "ring=%.3f ring_alarm=%u hard=%u tracks=%u spikes=%u long=%u",
                   millis(), (unsigned long)pi.adc_hz, pi.clk_ok, pi.nolimit, pi.t0_ok, pi.fault, (unsigned long)pi.us, (unsigned long)g_rfWaitN,
                   (unsigned long)g_rfBusyAtPing, (double)o.bottom, (double)o.bottom_snr, (double)o.nf, (double)o.ring_m,
                   o.ring_alarm ? 1 : 0, o.hard, o.n, o.spikes, o.long_echoes);
  for (uint8_t i = 0; i < o.n && k > 0 && k < (int)sizeof(buf) - 40; i++)
    k += snprintf(buf + k, sizeof(buf) - k, " t=%.3f:%u:%.1f:%.2f:%.3f", (double)o.t[i].depth, o.t[i].label, (double)o.t[i].s, (double)o.t[i].vel, (double)o.t[i].width);
  recLine(buf);
}
// one full capture, 3 hex digits per 12-bit sample (~8.4 kB for 2800 samples). Blocks the serial port ~0.75 s.
static void recRaw(const tuss::PingInfo& pi, bool force) {
  const uint16_t* smp; size_t n; uint32_t hz;
  tuss::lastRaw(smp, n, hz);
  if (!smp || !n) { recLine("NOTE 0 RAW: no capture yet", true); return; }
  static const char H[] = "0123456789ABCDEF";
  char* buf = (char*)malloc(80 + 3 * n);
  if (!buf) { recLine("NOTE 0 RAW: out of memory", true); return; }
  int k = snprintf(buf, 80, "RAW %lu %lu %lu %d %u ", millis(), (unsigned long)hz, (unsigned long)pi.adc_hz, pi.t0, (unsigned)n);
  for (size_t i = 0; i < n; i++) { const uint16_t v = smp[i] & 0x0FFF; buf[k++] = H[v >> 8]; buf[k++] = H[(v >> 4) & 15]; buf[k++] = H[v & 15]; }
  buf[k] = 0;
  recLine(buf, force);
  free(buf);
}

// one readable line per ping for the bucket test: is the hardware right, where is the bottom
static bool sonarStatAsk = false;
static int16_t sonarKnobAskId = -1; static uint8_t sonarKnobAskVal = 0;   // serial KNOB, applied in sonarLoop
// D54: wait (max 40 ms) until every queued ESP-NOW frame has been transmitted, plus a 2 ms guard after the last
// TX callback, so no transmit burst lands inside the capture window. Counted for the bench line / STAT.
static void radioQuietForPing() {
  const uint32_t t0 = micros();
  bool waited = false;
  while (true) {
    const bool inflight = (g_txQueued != g_txDone) || sendInProgress;
    const bool guard = (millis() - g_txDoneMs) < 2UL;
    if (!inflight && !guard) break;
    if (micros() - t0 > 40000UL) { g_rfBusyAtPing++; break; }   // radio stuck: ping anyway, count it
    waited = true;
    yield();
  }
  if (waited) {
    g_rfWaitN++;
    const uint32_t us = micros() - t0;
    if (us > g_rfWaitUsMax) g_rfWaitUsMax = us;
  }
}

static char sonarBenchBuf[512];
static void sonarBenchFormat(const tuss::PingInfo& pi, const icemesh::sonar::SonarProc::Out& o) {
  char edge[16];
  if (pi.edge_um >= 0) snprintf(edge, sizeof(edge), "%.3fm", pi.edge_um / 1e6); else snprintf(edge, sizeof(edge), "-");
  int k = snprintf(sonarBenchBuf, sizeof(sonarBenchBuf), "BENCH adc=%luHz(clock %lu)%s t0=%d%s raw_max=%u%s edge=%s ping=%lums%s | bottom=%.2fm bot_snr=%.0fdB noise=%.0fdB ring=%.2fm%s spikes=%u long=%u |",
                (unsigned long)pi.adc_hz, (unsigned long)tuss::clockHz(), pi.clk_ok ? "" : (pi.nolimit ? "(SHORT)" : "(LIMITED)"), pi.t0, pi.t0_ok ? "" : "(FALLBACK)", pi.raw_max, pi.raw_max >= 4095 ? "(CLIPPED)" : "",
                edge, (unsigned long)(pi.us / 1000), pi.fault & 4 ? " BURST FAULT(pulse count)" : pi.fault & 2 ? " BURST FAULT(driver stuck)" : "", (double)o.bottom, (double)o.bottom_snr, (double)o.nf, (double)o.ring_m,
                o.ring_alarm ? "(RING ALARM)" : "", o.spikes, o.long_echoes);
  static const char* const L[] = {"fish", "bait", "nearbottom", "cover"};
  for (uint8_t i = 0; i < o.n && k > 0 && k < (int)sizeof(sonarBenchBuf) - 32; i++)
    k += snprintf(sonarBenchBuf + k, sizeof(sonarBenchBuf) - k, " %.2fm:%s", (double)o.t[i].depth, L[o.t[i].label & 3]);
  if (k > 0 && k < (int)sizeof(sonarBenchBuf) - 80)
    snprintf(sonarBenchBuf + k, sizeof(sonarBenchBuf) - k, " | rf: waited %lu pings (max %lu us), busy at ping %lu", (unsigned long)g_rfWaitN, (unsigned long)g_rfWaitUsMax, (unsigned long)g_rfBusyAtPing);
}
static void sonarBenchLine(const tuss::PingInfo& pi, const icemesh::sonar::SonarProc::Out& o) {
  sonarBenchFormat(pi, o);
  Serial.println(sonarBenchBuf);
}

// Sonar commands, from the serial port or the phone page (same words). Returns the reply (also printed).
//   REC ON|OFF (serial recording) | REC WEB ON [phone time]|REC WEB OFF (phone page) | RECEVERY <n> | RAWEVERY <s>
//   NOTE <text> | RAW | INFO | STAT | KNOB <name> <value> | KNOBS? (JSON for the page) | REG <hex addr> [<hex value>]
//   CAL [17 numbers]
static icemesh::sonar::Params* sonarKnobsPtr = nullptr;   // the live knob set (sonarLoop), for KNOBS? / INFO
String sonarCommand(const String& in, bool fromWeb) {
  String l = in; l.trim();
  const int sp0 = l.indexOf(' ');
  String w = sp0 > 0 ? l.substring(0, sp0) : l; w.toUpperCase();
  String rest = sp0 > 0 ? l.substring(sp0 + 1) : String(""); rest.trim();
  String U = l; U.toUpperCase();
  String r;
  if (U == "SELFTEST") {   // D70
    selfTestRun(true);
    r = "SELFTEST done (printed above)";
  } else if (U == "REC ON" || U == "REC OFF") {
    sonarRec = (U == "REC ON");
    if (sonarRec) { if (sonarKnobsPtr) recKnobs(*sonarKnobsPtr); recInfo(); }
    r = String("Serial recording ") + (sonarRec ? "ON" : "off");
  } else if (U.startsWith("REC WEB ON") || U == "REC WEB OFF") {
    if (!SONAR_BENCH || !bench::active()) r = "Phone recording needs the bench build (sensor_wroom_sonar_bench)";
    else {
      const bool on = U.startsWith("REC WEB ON");
      if (on && !sonarRecWeb) {
        sonarRecWeb = true; sonarRecLines = 0;
        recInfo(); if (sonarKnobsPtr) recKnobs(*sonarKnobsPtr);
        String t = "NOTE " + String(millis()) + " recording start, phone time " + (l.length() > 10 ? l.substring(10) : String("?"));
        t.trim(); recLine(t.c_str());
        sonarRawAsk = true;   // one RAW capture right at the start
      } else if (!on && sonarRecWeb) {
        String t = "NOTE " + String(millis()) + " recording stop"; recLine(t.c_str());
        sonarRecWeb = false;
      }
      r = String("Phone recording ") + (sonarRecWeb ? "ON" : "off");
    }
  } else if (w == "RECEVERY") {
    const long v = rest.toInt(); if (v >= 1 && v <= 20) sonarRecEvery = (uint16_t)v;
    r = "Record every " + String(sonarRecEvery) + " ping(s)";
  } else if (w == "RAWEVERY") {
    const long v = rest.toInt(); if (v >= 0 && v <= 600 && rest.length()) sonarRawEverySec = (uint16_t)v;
    r = "RAW every " + String(sonarRawEverySec) + " s while recording (0 = on demand only)";
  } else if (w == "NOTE") {
    String t = "NOTE " + String(millis()) + " " + rest;
    t.replace("\n", " "); t.replace("\r", " ");
    recLine(t.c_str(), true);
    return "Noted";
  } else if (w == "RAW") {
    sonarRawAsk = true; r = "RAW capture after the next ping";
  } else if (w == "INFO") {
    recInfo(); if (sonarKnobsPtr) recKnobs(*sonarKnobsPtr); r = "INFO sent";
  } else if (w == "KNOBS?") {
    String j = "[";
    for (uint8_t k = 0; k < icemesh::sonar::P_COUNT; k++) {
      const icemesh::sonar::ParamInfo& pi = icemesh::sonar::paramInfo(k);
      if (k) j += ",";
      j += "{\"key\":\""; j += pi.key; j += "\",\"label\":\""; j += pi.name; j += "\",\"min\":"; j += String(pi.lo);
      j += ",\"max\":"; j += String(pi.hi); j += ",\"unit\":\""; j += pi.unit; j += "\",\"val\":";
      j += String(sonarKnobsPtr ? (*sonarKnobsPtr)[k] : pi.def); j += "}";
    }
    return j + "]";
  } else if (w == "KNOB") {   // bench: set one knob on this node only (not saved; the chalet's set wins later)
    String a = rest; a.toLowerCase(); const int sp = a.indexOf(' ');
    bool found = false;
    for (uint8_t k = 0; sp > 0 && k < icemesh::sonar::P_COUNT; k++)
      if (a.substring(0, sp) == icemesh::sonar::paramInfo(k).key) { sonarKnobAskId = k; const long v = a.substring(sp + 1).toInt(); sonarKnobAskVal = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); found = true; }
    if (found) r = "Knob " + a.substring(0, sp) + " -> " + a.substring(sp + 1) + " (applied before the next ping)";
    else {
      r = "KNOB <name> <value>, names:";
      for (uint8_t k = 0; k < icemesh::sonar::P_COUNT; k++) { r += " "; r += icemesh::sonar::paramInfo(k).key; }
    }
  } else if (w == "STAT") {
    sonarStatAsk = true;
    r = tuss::ok() ? String(sonarBenchBuf[0] ? sonarBenchBuf : "STAT: next ping") : String("TUSS4470 not answering (SPI wiring / SONAR_PIN_*)");
  } else if (w == "REG" && rest.length()) {
    const int sp = rest.indexOf(' ');
    const uint8_t addr = (uint8_t)strtol(rest.substring(0, sp > 0 ? sp : rest.length()).c_str(), nullptr, 16);
    if (sp > 0) tuss::writeReg(addr, (uint8_t)strtol(rest.substring(sp + 1).c_str(), nullptr, 16));
    char b[48]; snprintf(b, sizeof(b), "TUSS4470 reg 0x%02X = 0x%02X", addr, tuss::readReg(addr));
    r = b;
    if (sonarRecAny()) { String t = "NOTE " + String(millis()) + " " + r; recLine(t.c_str()); }
  } else if (w == "CAL") {
    uint16_t pts[17]; tuss::getCalibration(pts);
    if (rest.length()) {
      uint16_t np[17]; int k = 0; bool okv = true; char* c = const_cast<char*>(rest.c_str()); char* e;
      while (k < 17) { const long v = strtol(c, &e, 10); if (e == c) break; if (v < 0 || v > 4095) okv = false; np[k++] = (uint16_t)v; c = e; }
      if (k == 17 && okv) { tuss::setCalibration(np); memcpy(pts, np, sizeof(pts)); }
      else r = "CAL needs 17 numbers 0..4095 (raw 0, 256 ... 4096 -> corrected); stored curve unchanged\n";
    }
    r += "ADC calibration:"; for (int k = 0; k < 17; k++) { r += " "; r += String(pts[k]); }
  } else if (l.length()) {
    r = "Sonar: REC ON|OFF, REC WEB ON|OFF, RECEVERY n, RAWEVERY s, NOTE text, RAW, INFO, STAT, KNOB name value, REG addr [value], CAL [17 numbers]";
  }
  if (r.length() && (!fromWeb || DEBUG_SERIAL || SONAR_BENCH)) Serial.println(r);
  return r;
}
#if SONAR_BENCH && SONAR_REAL
static String sonarWebCommand(const String& c) { return sonarCommand(c, true); }
#endif
void sonarSerial() {   // D64: line collected at Enter (readStringUntil + 20 ms timeout split typed commands per key)
  if (!SONAR_REAL) return;
  static String acc;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (acc.length()) { String l = acc; acc = ""; sonarCommand(l, false); } }
    else if (c == 8 || c == 127) { if (acc.length()) acc.remove(acc.length() - 1); }
    else if (acc.length() < 200) acc += c;
  }
}

void sonarLoop() {
  static icemesh::sonar::SonarSource src;
  static bool started = false;
  static unsigned long lastTick = 0, lastCtrl = 0;
  static uint8_t focus = 0;
  static icemesh::sonar::Params knobs;
  static bool knobsLoaded = false;
  sonarKnobsPtr = &knobs;
  #if SONAR_BENCH && SONAR_REAL
  static bool benchWebTried = false;
  if (!benchWebTried) {   // D56: phone test page on its own access point (bench build only)
    benchWebTried = true;
    const bool ok = bench::begin(NODE_ID, WIFI_PASSWORD, ESPNOW_CHANNEL, sonarWebCommand);
    Serial.printf("Bench page: %s (Wi-Fi \"IceFish-bench-%d\", http://192.168.4.1)\n", ok ? "on" : "FAILED", NODE_ID);
  }
  bench::loop();
  #endif
  if (!knobsLoaded) {   // the last set received, kept across deep sleep / reboot
    knobsLoaded = true;
    Preferences p; p.begin("sonar", true);
    uint8_t v[icemesh::sonar::P_COUNT]; const size_t n = p.getBytes("p", v, sizeof(v)); p.end();
    for (uint8_t i = 0; i < n && i < icemesh::sonar::P_COUNT; i++) knobs.set(i, v[i]);
    src.proc.prm = knobs;
  }
  // v2 (D43) real sonar: started once; a node with a working TUSS4470 runs its sonar whenever its hub
  // talks to it (sonar control seen), fake data only when the chalet asks for a sonar simulation
  const bool hwOk = sonarHwBegin();
  static uint8_t bait5 = 0;
  static bool baitLoaded = false;
  if (!baitLoaded) { baitLoaded = true; Preferences p; p.begin("node", true); bait5 = p.getUChar("bait", 0); p.end(); }
  if (devCmdBait >= 0) {   // v2 (D43): this hole's bait depth (labels "Bait", near-bait, cover)
    const uint8_t v = (uint8_t)devCmdBait; devCmdBait = -1;
    if (v && v != bait5) { bait5 = v; Preferences p; p.begin("node", false); p.putUChar("bait", v); p.end(); }
  }
  if (bait5) { const float m = bait5 * 0.05f; if (src.proc.bait_m != m) { src.proc.bait_m = m; src.scene.setBait(m); } }
  else if (SONAR_REAL && hwOk && src.proc.bait_m > 0) src.proc.bait_m = -1;   // real hole, no bait set: no bait labels / flags (D47)
  if (sonarKnobsNew) {
    sonarKnobsNew = false;
    bool changed = false;
    for (uint8_t i = 0; i < sonarKnobsN && i < icemesh::sonar::P_COUNT; i++) changed |= knobs.set(i, sonarKnobsBuf[i]);
    if (changed) {
      src.proc.prm = knobs;
      Preferences p; p.begin("sonar", false); p.putBytes("p", knobs.v, icemesh::sonar::P_COUNT); p.end();
      DEBUG_PRINTLN(F("Sonar knobs updated by the chalet"));
      if (sonarRecAny()) recKnobs(knobs);
    }
  }
  sonarSerial();
  if (sonarKnobAskId >= 0) {
    const uint8_t id = (uint8_t)sonarKnobAskId; sonarKnobAskId = -1;
    knobs.set(id, sonarKnobAskVal); src.proc.prm = knobs;
    Serial.printf("knob %s = %u %s (this node, not saved)\n", icemesh::sonar::paramInfo(id).key, knobs[id], icemesh::sonar::paramInfo(id).unit);
    if (sonarRecAny()) recKnobs(knobs);
  }
  if (sonarCtrlSeen) {
    sonarCtrlSeen = false;
    lastCtrl = millis();
    focus = sonarCtrlFocus;
    sonarSimOn = (sonarCtrlSim && (simBits & 0x01)) || SONAR_SIM_FORCE || SONAR_BENCH || hwOk;   // (name kept: "sonar running")
  }
  const bool fake = !hwOk || (sonarCtrlSim && (simBits & 0x01)) || SONAR_SIM_FORCE;
  if (sonarSimOn && !started) {
    src.begin(NODE_ID, 0xF15A0000UL + NODE_ID, (uint16_t)esp_random());
    started = true; lastTick = millis(); lastCtrl = millis();
  }
  if (!sonarSimOn) {
    if (started) {
      started = false;
      DEBUG_PRINTLN(F("Sonar test mode OFF"));
      if (SONAR_REAL) tuss::sleep();   // relay / no-sleep nodes too: the shield has no power switch
      if (SLEEP_ENABLED && !ebRelayOn) {   // a backbone relay stays awake
        totalUptimeSec += millis() / 1000;
        if (HAS_FLAG(currentFlags, FLAG_FISH_ON)) { fishOnMonitorMode = true; enterDeepSleepFast(); }
        else { fishOnMonitorMode = false; enterDeepSleep(); }
      }
    }
    return;
  }
  if (!SONAR_SIM_FORCE && !SONAR_BENCH && millis() - lastCtrl > SONAR_CTRL_TIMEOUT_MS) {   // hub gone or test mode off
    sonarSimOn = false;
    return;
  }
  if (millis() - lastTick > 1000UL) lastTick = millis() - 250UL;           // don't burst after a stall
  while (millis() - lastTick >= 250UL) {                                     // 4 pings/s
    lastTick += 250UL;
    icemesh::sonar::Block out[2];
    static uint32_t usMax = 0, usSum = 0, cnt = 0; static unsigned long lastReport = 0;
    const uint32_t t0 = micros();
    uint8_t n;
    if (fake) {
      if (SONAR_BENCH && SONAR_REAL && !hwOk) {   // bucket test with no TUSS4470 answer: say it, loudly
        static unsigned long warned = 0;
        if (millis() - warned >= 5000UL) { warned = millis(); Serial.println(F("BENCH: TUSS4470 NOT answering -> fake data. Check SPI wiring, SONAR_PIN_*, 3.3 V / 5 V to the shield")); }
      }
      n = src.tick(focus == NODE_ID, out, 2);   // fake raw ping -> processing -> blocks
    } else {   // real ping when due (adaptive rate), processing, blocks; recording on serial if asked
      static uint8_t tick_n = 0;
      static uint8_t codes[icemesh::sonar::sp::NFREQ][icemesh::sonar::BINS];
      const uint8_t tpp = src.ticksPerPing(focus == NODE_ID);
      n = 0;
      if (++tick_n >= tpp) {
        tick_n = 0;
        tuss::PingInfo pi;
        radioQuietForPing();
        while (bench::active() && millis() - bench::lastServedMs() < 15) yield();   // let the page's reply leave the radio first
        if (tuss::ping(knobs, focus == NODE_ID, codes, pi)) {
          n = src.process(codes, pi.nfreq, pi.rot_f, 0.25f * tpp, focus == NODE_ID, out, 2);
          static uint32_t recCount = 0; static unsigned long lastRaw = 0;
          if (sonarRecAny() && (recCount++ % sonarRecEvery) == 0) { sonarRecord(codes, pi); recPingX(pi, src.lastProc()); }
          if (sonarRawAsk || (sonarRecAny() && sonarRawEverySec && millis() - lastRaw >= sonarRawEverySec * 1000UL)) {
            recRaw(pi, sonarRawAsk && !sonarRecAny());   // asked while not recording: print it anyway
            sonarRawAsk = false; lastRaw = millis();
          }
          #if SONAR_BENCH
          sonarBenchFormat(pi, src.lastProc());
          bench::setStatus(sonarBenchBuf, pi.nfreq == 3 ? codes[1] : codes[0], icemesh::sonar::BINS, 0.025f, sonarRecWeb,
                           sonarRecLines, sonarRecEvery, sonarRawEverySec);
          #endif
          static unsigned long lastBench = 0;
          if (sonarStatAsk || (SONAR_BENCH && !sonarRec && millis() - lastBench >= 2000UL)) {
            sonarStatAsk = false; lastBench = millis();
            sonarBenchLine(pi, src.lastProc());
          }
        }
      }
    }
    const uint32_t dt = micros() - t0;
    if (dt > usMax) usMax = dt;
    usSum += dt; cnt++;
    if (millis() - lastReport >= 60000UL) {
      lastReport = millis();
      DEBUG_PRINTF("Sonar sim: %s, ping cost avg %lu us max %lu us\n", focus == NODE_ID ? "FOCUS" : "base",
                   (unsigned long)(usSum / cnt), (unsigned long)usMax);
      usMax = 0; usSum = 0; cnt = 0;
    }
    for (uint8_t i = 0; i < n; i++) {
      uint8_t frame[3 + icemesh::sonar::MAX_BLOCK];
      frame[0] = NETWORK_ID; frame[1] = NODE_ID; frame[2] = MSG_SONAR;
      memcpy(frame + 3, out[i].data, out[i].len);
      // best effort, no retry: a lost block only leaves a gap on the waterfall. D79 (E7): after 8 unicast failures in a
      // row the hub is gone (or moved): broadcast, so any hub in range gets the stream; the next status frame relearns the MAC
      if (gatewayMacKnown && g_ucFails >= 8) { gatewayMacKnown = false; memset(gatewayMac, 0, 6); g_ucFails = 0; DEBUG_PRINTLN(F("ESP-NOW: hub silent on unicast, sonar back to broadcast")); }
      nowSend(gatewayMacKnown ? gatewayMac : ESPNOW_BROADCAST, frame, 3 + out[i].len);
    }
  }
}


// ═══════════════════════════════════════════════════════════════════════════
// v2 (D70) WIRING SELF-TEST: sensor pin, button, battery, shield (lib/SonarTuss shieldTest), radio. Asked by the
// chalet page (CMD_SELFTEST -> hub -> DEVCMD_SELFTEST) or typed on serial (SELFTEST). The 12-byte result goes to the
// hub (MSG_SELFTEST) which repeats it to the chalet in a SEC_SELFTEST section; the page turns the bits into words.
// ═══════════════════════════════════════════════════════════════════════════
static void selfTestFill(SelfTestResult& r) {
  memset(&r, 0, sizeof(r));
  r.ver = 1;
  const uint8_t lvl = (uint8_t)digitalRead(REED_PIN);
  r.sensor = (lvl ? ST_SENS_LEVEL : 0);
  #if HALL_LATCH
  r.sensor |= ST_SENS_HALL; if (hall.total) r.sensor |= ST_SENS_FLIPS;
  r.flips = hall.total > 255 ? 255 : (uint8_t)hall.total;
  #endif
  #if BTN_PIN >= 0
  r.button = (digitalRead(BTN_PIN) ? ST_BTN_RELEASED : 0) | (btnPresses ? ST_BTN_PRESSED_SINCE_BOOT : 0);
  r.presses = btnPresses > 255 ? 255 : (uint8_t)btnPresses;
  #else
  r.button = ST_BTN_NONE;
  #endif
  readBattery();
  r.battery_mv = batteryMv;
  if (SONAR_REAL) sonarHwBegin();   // review: the head begins the shield lazily; the check must not probe a bus never opened
  tuss::ShieldTest t; tuss::shieldTest(t);
  if (t.pinged) r.sensor |= ST_SENS_PINGED;
  if (t.not_built) r.shield = ST_SH_NOT_BUILT;
  else {
    r.shield = (t.spi_ok ? ST_SH_SPI_OK : 0) | (t.swapped ? ST_SH_PINS_SWAPPED : 0) | (t.vdrv_ready ? ST_SH_VDRV_READY : 0) |
               (t.pulse_fault ? ST_SH_PULSE_FLT : 0) | (t.drv_fault ? ST_SH_DRV_FLT : 0) | (t.ee_fault ? ST_SH_EE_FLT : 0) | (t.leakage ? ST_SH_LEAKAGE : 0);
    r.raw_max_16 = (uint8_t)(t.raw_max / 16); r.ring_bins = t.ring_bins; r.noise_code = t.noise_code;
  }
  r.rssi_abs = 0;   // the head does not keep the hub's RSSI (the hub reports its side)
}

void selfTestRun(bool toSerial) {
  SelfTestMessage m;
  m.network_id = NETWORK_ID; m.node_id = NODE_ID; m.msg_type = MSG_SELFTEST;
  selfTestFill(m.r);
  if (toSerial) {
    Serial.printf("SELFTEST sensor=0x%02X (level %u, %u edges) button=0x%02X (%u presses) batt=%u mV shield=0x%02X raw_max=%u ring=%u bins noise=%u\n",
                  m.r.sensor, m.r.sensor & 1, m.r.flips, m.r.button, m.r.presses, m.r.battery_mv, m.r.shield, m.r.raw_max_16 * 16, m.r.ring_bins, m.r.noise_code);
  }
  if (espNowReady) espNowSendReliable((uint8_t*)&m, sizeof(m), 3);
}
