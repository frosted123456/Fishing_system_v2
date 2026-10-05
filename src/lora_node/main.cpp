/*
 * ICE FISHING MESH MONITOR - LoRa Node
 * 
 * For: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * Roles: SENSOR_LORA, RELAY_LORA, GATEWAY_ONSHORE, GATEWAY_OFFSHORE
 * 
 * BUILD (PlatformIO, see platformio.ini at the project root):
 *   Board, core and library versions are pinned in platformio.ini.
 *   Pick the env for the role (hub / cabin / relay / sensor_lora) and set
 *   NODE_ID, NODE_NAME, NODE_ROLE, HAS_LOCAL_SENSOR there with build_flags.
 *   The defaults below are only used when build_flags do not set them.
 *
 *   (Was lora_node.ino for the Arduino IDE; converted to .cpp on 2026-10-05:
 *    only #include <Arduino.h> and missing prototypes were added.)
 */

// ═══════════════════════════════════════════════════════════════════════════
// NODE CONFIGURATION - CHANGE THESE PER DEVICE
// ═══════════════════════════════════════════════════════════════════════════

#ifndef NODE_ID
#define NODE_ID             1                     // <<< CHANGE THIS! (or -D NODE_ID=… in platformio.ini)
#endif
#ifndef NODE_NAME
#define NODE_NAME           "Gateway Ice"         // Name for web UI
#endif
#ifndef NODE_ROLE
#define NODE_ROLE           ROLE_SENSOR_LORA  // See roles below
#endif
#ifndef HAS_LOCAL_SENSOR
#define HAS_LOCAL_SENSOR    true                  // Reed switch attached?
#endif

/*
 * ROLES:
 *   ROLE_GATEWAY_ONSHORE  - On ice: ESP-NOW hub + LoRa uplink + web + sensor
 *   ROLE_GATEWAY_OFFSHORE - In cabin: LoRa endpoint + web + display
 *   ROLE_SENSOR_LORA      - On ice: Reed sensor + ESP-NOW + LoRa relay
 *   ROLE_RELAY_LORA       - LoRa mesh relay only (no sensor)
 */

// ═══════════════════════════════════════════════════════════════════════════
// INCLUDES
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>  // BUG FIX #8: Hardware watchdog timer
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>

#include "config.h"
#include "messages.h"
#include <rx_ring.h>

// ═══════════════════════════════════════════════════════════════════════════
// UI LAYOUT CONSTANTS - 128x64 OLED
// ═══════════════════════════════════════════════════════════════════════════

// Screen dimensions (already in config.h as OLED_WIDTH/HEIGHT, but define here for UI use)
#define UI_SCREEN_WIDTH        128
#define UI_SCREEN_HEIGHT       64

// Standard zones for consistent layout
#define UI_HEADER_Y            7       // Header text baseline (for 5x7 or 6x10 font)
#define UI_SEPARATOR_Y         9       // Horizontal line below header
#define UI_CONTENT_START_Y     12      // First content line start
#define UI_CONTENT_END_Y       52      // Last content line (leave room for footer)
#define UI_FOOTER_Y            62      // Footer/help text baseline

// Font metrics - u8g2_font_6x10_tr (LARGE)
#define FONT_LARGE_HEIGHT      10
#define FONT_LARGE_WIDTH       6
#define LINE_SPACING_LARGE     10      // Line spacing for 6x10 font

// Font metrics - u8g2_font_5x7_tr (MEDIUM)
#define FONT_MEDIUM_HEIGHT     7
#define FONT_MEDIUM_WIDTH      5
#define LINE_SPACING_MEDIUM    8       // Line spacing for 5x7 font

// Font metrics - u8g2_font_tom_thumb_4x6_tr (SMALL)
#define FONT_SMALL_HEIGHT      6
#define FONT_SMALL_WIDTH       4
#define LINE_SPACING_SMALL     7       // Line spacing for 4x6 font

// Calculated max lines in content zone
#define MAX_LINES_LARGE        ((UI_CONTENT_END_Y - UI_CONTENT_START_Y) / LINE_SPACING_LARGE)   // 4
#define MAX_LINES_MEDIUM       ((UI_CONTENT_END_Y - UI_CONTENT_START_Y) / LINE_SPACING_MEDIUM)  // 5

// Live status grid layout (5 columns x 2 rows)
#define GRID_COLS              5
#define GRID_ROWS              2
#define GRID_CELL_WIDTH        25
#define GRID_CELL_HEIGHT       21      // Reduced from 24 to fit footer (21*2=42, starts at Y=11, ends at Y=53)
#define GRID_START_X           1
#define GRID_START_Y           11

// Menu and list layout
#define MENU_MAX_VISIBLE       4       // Max visible menu items (with footer space)
#define MENU_ITEM_HEIGHT       10      // Height per menu item

// ═══════════════════════════════════════════════════════════════════════════
// HARDWARE OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

// Use software I2C for reliable custom pin operation on Heltec V3
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
WebServer server(WEB_SERVER_PORT);
Preferences preferences;
TwoWire CardKBWire = TwoWire(1);  // Second I2C bus for CardKB

// WiFi credentials (loaded from NVS or config.h defaults)
char storedSsid[33] = "";
char storedPassword[65] = "";

// ═══════════════════════════════════════════════════════════════════════════
// MENU SYSTEM
// ═══════════════════════════════════════════════════════════════════════════

enum MenuScreen : uint8_t {
  SCREEN_LIVE_STATUS = 0,   // Default - node grid
  SCREEN_MAIN_MENU,         // Main menu
  SCREEN_NODE_LIST,         // List of nodes
  SCREEN_NODE_DETAILS,      // Single node details
  SCREEN_ALERT_HISTORY,     // Recent alerts
  SCREEN_SETTINGS,          // Settings submenu
  SCREEN_WIFI_CONFIG,       // WiFi SSID/password entry
  SCREEN_NETWORK_INFO,      // IP addresses, uptime, etc
  SCREEN_REBOOT_CONFIRM,    // Reboot confirmation
  SCREEN_SETTING_EDIT,      // Editing a numeric setting
  SCREEN_RESET_ALL_CONFIRM  // Reset all nodes confirmation
};

// Persisted settings
struct DeviceSettings {
  bool buzzerEnabled;
  uint16_t alertHoldSec;      // Minimum alert display time
  uint16_t heartbeatSec;      // Heartbeat interval
  uint8_t displayBrightness;  // 0-255
  bool webServerEnabled;      // Enable/disable web server
  uint8_t wifiModeSetting;    // 0=AP, 1=STA, 2=APSTA
  bool reedActiveHigh;        // Reed switch polarity: true=trigger on HIGH
  uint8_t reserved[5];        // Future use
};

DeviceSettings settings = {
  .buzzerEnabled = true,
  .alertHoldSec = 30,
  .heartbeatSec = 60,
  .displayBrightness = 255,
  .webServerEnabled = true,
  .wifiModeSetting = 2,       // Default: AP+STA mode
  .reedActiveHigh = true,     // Default: trigger on HIGH (magnet away)
  .reserved = {0}
};

// Remote config state
uint8_t pendingConfigSeq = 0;
uint8_t pendingConfigTarget = 0;
unsigned long pendingConfigTime = 0;
bool pendingConfigWaiting = false;
const unsigned long CONFIG_ACK_TIMEOUT_MS = 15000;  // 15 seconds (allows for hops)

// Config message deduplication (prevents relay loops)
#define CONFIG_DEDUP_SIZE 8
struct ConfigDedupEntry {
  uint8_t origin_id;
  uint8_t config_seq;
  uint32_t received_at;
};
ConfigDedupEntry configDedup[CONFIG_DEDUP_SIZE];
uint8_t configDedupIdx = 0;

// Alert history
#define ALERT_HISTORY_SIZE 10
struct AlertRecord {
  uint8_t nodeId;
  uint32_t timestamp;     // millis() when occurred
  bool active;            // Still ongoing?
};
AlertRecord alertHistory[ALERT_HISTORY_SIZE];
uint8_t alertHistoryIdx = 0;
uint8_t alertHistoryCount = 0;

// Menu state
MenuScreen currentScreen = SCREEN_LIVE_STATUS;
MenuScreen previousScreen = SCREEN_LIVE_STATUS;
uint8_t menuSelection = 0;
uint8_t menuScrollOffset = 0;
uint8_t selectedNodeIdx = 0;
uint8_t editingSettingIdx = 0;
int32_t editingValue = 0;
uint8_t liveStatusPage = 0;  // Current page on live status screen

// Silence state
bool alertsSilenced = false;
uint32_t silenceTime = 0;
uint32_t silenceExpireTime = 0;  // When silence auto-expires

// CardKB state
bool cardKbAvailable = false;
char inputBuffer[65] = "";
uint8_t inputPos = 0;
uint8_t inputField = 0;  // 0=SSID, 1=Password
char inputSsid[33] = "";
char inputPassword[65] = "";

// Button silencing (for devices without CardKB)
unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 300;

// OLED sleep mode
bool displaySleeping = false;
unsigned long lastActivityTime = 0;
const unsigned long DISPLAY_SLEEP_MS = 5UL * 60UL * 1000UL;  // 5 minutes

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════════════

NodeRole currentRole = NODE_ROLE;
NetworkState network;

// M1 FIX: Mutex for network.nodes[] access (ESP-NOW callback vs main loop)
// Using minimal critical sections to avoid stack overflow issues
static portMUX_TYPE networkMux = portMUX_INITIALIZER_UNLOCKED;

// Note: Thread safety between ESP-NOW callback and main loop was attempted with
// portMUX spinlock, but caused stack overflow on IDLE task. ESP-NOW callbacks
// are serialized by WiFi driver and race conditions are unlikely in practice.
// M1 FIX: Re-enabled with minimal critical sections around state mutations only.

bool localReedState = false;
bool localFishOn = false;
uint16_t localBatteryMv = 0;
uint16_t localSequence = 0;       // Our own message sequence counter
uint32_t localUptimeSec = 0;      // Our own uptime tracker

bool espNowReady = false;
bool loraReady = false;
bool wifiApActive = false;
bool wifiStaConnected = false;
String staIpAddress = "";
String apIpAddress = "";

unsigned long lastLoRaTx = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastReedCheck = 0;
unsigned long lastHeartbeatTime = 0;  // Track heartbeat timing

// ═══════════════════════════════════════════════════════════════════════════
// RADIO STATE MACHINE (fixes packet loss race condition)
// ═══════════════════════════════════════════════════════════════════════════
// The SX1262 DIO1 interrupt fires for BOTH TX complete and RX complete.
// We must track state to distinguish them and never lose pending RX packets.

enum RadioState : uint8_t {
    RADIO_STATE_RX = 0,      // Listening for packets
    RADIO_STATE_TX = 1,      // Transmitting (expect TX complete interrupt)
    RADIO_STATE_IDLE = 2     // Between operations
};

volatile bool loraInterrupt = false;     // DIO1 interrupt occurred (RX case)
volatile bool loraTxComplete = false;    // C1 FIX: Separate flag for TX complete
volatile RadioState radioState = RADIO_STATE_RX;

// Legacy name for compatibility with existing code
#define loraReceived loraInterrupt

// BUG FIX #17b: LoRaAggregateMessage is now 107 bytes (6 header + 10*10 nodes + 1 checksum)
// after optimizing NodeStatusCompact from 14 to 10 bytes per node
uint8_t loraBuffer[128];  // Sufficient for 107-byte optimized message

// Diagnostic counter: Packets saved from loss by processPendingLoRaRx()
uint32_t loraRxSavedFromLoss = 0;

bool activeAlerts = false;
unsigned long lastBuzzerTime = 0;
uint8_t buzzerState = 0;

uint16_t txSequence = 0;  // H1 FIX: Use uint16_t to prevent faster-than-expected wraparound

// ═══════════════════════════════════════════════════════════════════════════
// LORA DIAGNOSTIC COUNTERS & WATCHDOG (IMPROVEMENT 1, BUG FIX #1, #2)
// ═══════════════════════════════════════════════════════════════════════════

uint32_t loraRxCount = 0;           // Packets received successfully
uint32_t loraTxCount = 0;           // Packets transmitted successfully
uint32_t loraRxErrors = 0;          // Receive errors
uint32_t loraTxErrors = 0;          // Transmit errors
uint32_t loraChecksumFails = 0;     // Checksum verification failures
uint32_t loraWatchdogResets = 0;    // Watchdog-triggered radio reinitializations
uint32_t loraStartRxRetries = 0;    // startReceive retry count

unsigned long lastLoRaRxTime = 0;   // When last valid packet was received

// Relay deduplication cache
DedupeEntry dedupeCache[DEDUP_CACHE_SIZE];
uint8_t dedupeCacheIdx = 0;

// ISSUE 1 FIX (v2 phase 1, C3): separate relay dedup cache for LoRa aggregates.
// Key = (origin hub, lora_sequence). Relays no longer overwrite sender_id, so
// sender_id IS the origin hub. Kept apart from dedupeCache so aggregates neither
// collide with alert keys nor push alert entries out of the shared ring.
#define AGG_DEDUP_CACHE_SIZE 32   // 30 s window x 1 aggregate / 6 s = 5 per hub -> room for 6 hubs
DedupeEntry aggDedupCache[AGG_DEDUP_CACHE_SIZE];
uint8_t aggDedupCacheIdx = 0;

// ═══════════════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════

void setupDisplay();
void setupLoRa();
void setupEspNow();
void setupWiFiAP();
void setupWebServer();
void setupBuzzer();
void setupLocalSensor();
void readBattery();

// Display sleep/wake functions
void wakeDisplay();
void sleepDisplay();
void registerActivity();

void loopLoRa();
void loopLoRaWatchdog();           // BUG FIX #2: LoRa watchdog
bool ensureLoRaReceiveMode();      // BUG FIX #1: Reliable startReceive
int loRaTransmitWithWait(uint8_t* data, size_t len);  // Transmit with BUSY wait
void loopLocalSensor();
void loopDisplay();
void loopWebServer();
void loopBuzzer();
void loopNodeTimeout();
void checkWiFiStatus();
void verifyWiFiChannel();          // BUG FIX #6: WiFi channel verification

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);
void loopEspNowRx();
void onLoRaReceive();

void processEspNowMessage(const uint8_t* data, int len, int rssi);
void processLoRaMessage(const uint8_t* data, int len, int rssi);

// Deduplication functions
bool shouldAcceptMessage(uint8_t nodeId, uint16_t seq, uint32_t uptime, uint8_t newFlags);
bool isDuplicateForRelay(uint8_t nodeId, uint16_t seq);
void recordForDedup(uint8_t nodeId, uint16_t seq);
bool isAggregateDuplicate(uint8_t originHub, uint8_t loraSeq);
void recordAggregateForDedup(uint8_t originHub, uint8_t loraSeq);
bool canClearFishOn(uint8_t nodeId);
NodeState* findOrCreateNode(uint8_t nodeId);

void updateNodeState(uint8_t nodeId, uint8_t flags, uint16_t batteryMv, uint16_t seq, uint32_t uptime, int8_t rssi);
void sendLoRaAggregate();
void sendLoRaAggregateWithRetry(int retries = 3);
void relayLoRaAlert(uint8_t originId, uint8_t flags, uint16_t batteryMv, uint16_t seq);

void updateDisplay();
void triggerBuzzer(uint8_t pattern);
void handleWebRoot();
void handleWebApi();
void handleWebWifiConfig();
void handleWebSettings();
void handleWebApiSettingsGet();
void handleWebApiSettingsPost();

// WiFi credential management
void loadWifiCredentials();
void saveWifiCredentials(const char* ssid, const char* password);
void checkSerialWifiConfig();

// Settings management
void loadSettings();
void saveSettings();

// Node naming
void saveNodeName(uint8_t nodeId, const char* name);
String loadNodeName(uint8_t nodeId);
void loadAllNodeNames();

// CardKB keyboard
void setupCardKB();
void loopCardKB();
void loopButton();
void handleKeyPress(char key);

// Menu system
void navigateToScreen(MenuScreen screen);
void handleMenuInput(char key);
void drawCurrentScreen();
void drawLiveStatus();
void drawMainMenu();
void drawNodeList();
void drawNodeDetails();
void drawAlertHistory();
void drawSettingsMenu();
void drawWifiConfig();
void drawNetworkInfo();
void drawRebootConfirm();
void drawSettingEdit();
void drawResetAllConfirm();
void showOverlayMessage(const char* message, uint16_t delayMs = 1500);

// Alert management
void recordAlert(uint8_t nodeId);
void silenceAlerts();
void sendSilenceSync();
void loopAutoUnsilence();

// Remote config (gateway to gateway)
bool sendRemoteConfig(uint8_t targetNodeId);
bool isConfigDuplicate(uint8_t originId, uint8_t seq);
void recordConfigForDedup(uint8_t originId, uint8_t seq);
void handleWebRemoteConfig();
void handleWebApiRemoteConfig();
void handleWebApiRemoteConfigStatus();

// Prototypes the Arduino IDE generated automatically for the .ino (needed since the move to .cpp)
bool loRaChannelBusy();
void updateAlertState();
void handleWebApiNodeName();
void handleWebApiSilence();
void handleResetAllConfirmInput(char key);
void sendResetAllCommand();

// BUG FIX #17: Battery mV <-> percentage conversion helpers
// For 3xAA lithium: 4500mV = 100%, 3200mV = 0%
inline uint8_t batteryMvToPercent(uint16_t mv) {
    if (mv >= 4500) return 100;
    if (mv <= 3200) return 0;
    return (uint8_t)((mv - 3200) * 100 / 1300);
}

inline uint16_t batteryPercentToMv(uint8_t pct) {
    if (pct >= 100) return 4500;
    return 3200 + (pct * 1300 / 100);
}

// Input handlers (per screen)
void handleLiveStatusInput(char key);
void handleMainMenuInput(char key);
void handleNodeListInput(char key);
void handleNodeDetailsInput(char key);
void handleAlertHistoryInput(char key);
void handleSettingsInput(char key);
void handleWifiConfigInput(char key);
void handleNetworkInfoInput(char key);
void handleRebootConfirmInput(char key);
void handleSettingEditInput(char key);

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(500);

  Serial.println();
  Serial.println(F("════════════════════════════════════════════════"));
  Serial.println(F("   ICE FISHING MESH MONITOR - LORA NODE"));
  Serial.println(F("════════════════════════════════════════════════"));
  Serial.printf("Node ID:     %d\n", NODE_ID);
  Serial.printf("Node Name:   %s\n", NODE_NAME);
  Serial.printf("Role:        %s\n", getRoleName(currentRole));
  Serial.printf("Network ID:  0x%02X\n", NETWORK_ID);
  Serial.println();

  // BUG FIX #8: Enable hardware watchdog timer (30 second timeout)
  esp_task_wdt_init(30, true);  // 30 second timeout, panic on timeout
  esp_task_wdt_add(NULL);       // Add current task to watchdog
  DEBUG_PRINTLN(F("Hardware watchdog enabled (30s timeout)"));
  
  // Initialize network state
  memset(&network, 0, sizeof(network));
  network.network_id = NETWORK_ID;
  memset(alertHistory, 0, sizeof(alertHistory));
  
  // Load persisted settings
  loadSettings();
  
  // BUG FIX #8: Initialize self-node completely at startup
  // Previously missing: last_seen, last_uptime, last_seq, initialized
  // This ensures first LoRa aggregate has correct self-node data
  network.nodes[0].node_id = NODE_ID;
  network.nodes[0].role = currentRole;
  network.nodes[0].online = true;
  network.nodes[0].initialized = true;
  network.nodes[0].last_seen = millis();
  network.nodes[0].last_direct_seen = millis();  // Self is always "direct"
  network.nodes[0].last_uptime = 0;  // Boot time
  network.nodes[0].last_seq = localSequence;
  network.nodes[0].fish_on_time = 0;
  network.nodes[0].via_espnow = false;
  network.nodes[0].via_lora = false;
  network.nodes[0].received_direct = true;  // Self is always considered direct
  strncpy(network.nodes[0].name, NODE_NAME, sizeof(network.nodes[0].name) - 1);
  network.node_count = 1;
  
  // Initialize hardware
  setupDisplay();
  lastActivityTime = millis();  // Initialize display sleep timer
  setupBuzzer();
  setupLoRa();
  
  // Role-specific setup
  switch (currentRole) {
    case ROLE_GATEWAY_ONSHORE:
      setupEspNow();
      setupWiFiAP();
      if (settings.webServerEnabled) {
        setupWebServer();
      } else {
        DEBUG_PRINTLN(F("Web server disabled by settings"));
      }
      setupCardKB();  // CardKB for WiFi config
      if (HAS_LOCAL_SENSOR) setupLocalSensor();
      // Read initial battery voltage
      pinMode(VBAT_PIN, INPUT);
      readBattery();
      DEBUG_PRINTF("Initial battery: %dmV\n", localBatteryMv);
      break;

    case ROLE_GATEWAY_OFFSHORE:
      setupWiFiAP();
      if (settings.webServerEnabled) {
        setupWebServer();
      } else {
        DEBUG_PRINTLN(F("Web server disabled by settings"));
      }
      setupCardKB();  // CardKB for WiFi config
      // Read initial battery voltage
      pinMode(VBAT_PIN, INPUT);
      readBattery();
      DEBUG_PRINTF("Initial battery: %dmV\n", localBatteryMv);
      break;

    case ROLE_SENSOR_LORA:
      setupEspNow();
      setupLocalSensor();
      setupCardKB();  // CardKB for silence control
      break;

    case ROLE_RELAY_LORA:
      setupEspNow();
      setupCardKB();  // CardKB for silence control
      break;
  }

  // Setup user button for silencing (GPIO0 on Heltec V3)
  pinMode(USER_BUTTON, INPUT_PULLUP);
  DEBUG_PRINTLN(F("User button (GPIO0) enabled for silencing"));

  // Startup display
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(10, 30, "READY");
  display.drawStr(10, 45, getRoleName(currentRole));
  display.sendBuffer();
  
  triggerBuzzer(1);
  delay(1000);
  
  Serial.println(F("Setup complete\n"));
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // BUG FIX #8: Feed hardware watchdog to prevent reset
  esp_task_wdt_reset();

  // BUG FIX #14: Refresh self-node status every 5 seconds
  // This prevents the self node from appearing stale and ensures
  // last_seen is always current for LoRa aggregate broadcasts
  static unsigned long lastSelfRefresh = 0;
  if (millis() - lastSelfRefresh > 5000) {
    lastSelfRefresh = millis();
    network.nodes[0].last_seen = millis();
    network.nodes[0].online = true;
    network.nodes[0].last_uptime = millis() / 1000;
  }

  // Handle keyboard input
  loopCardKB();
  loopButton();  // Handle PRG button for silencing

  // ESP-NOW frames queued by the Wi-Fi task callback (C9)
  if (espNowReady) loopEspNowRx();

  // Always process LoRa (even in menus, to not miss alerts)
  loopLoRa();

  // BUG FIX #2: Check LoRa radio health and recover if needed
  loopLoRaWatchdog();

  // Check for auto-unsilence timeout
  loopAutoUnsilence();

  // Check for remote config ACK timeout
  if (pendingConfigWaiting && (millis() - pendingConfigTime > CONFIG_ACK_TIMEOUT_MS)) {
    DEBUG_PRINTF("Remote config ACK timeout for node %d\n", pendingConfigTarget);
    pendingConfigWaiting = false;
    // Note: Config may have been applied even if ACK was lost
  }

  switch (currentRole) {
    case ROLE_GATEWAY_ONSHORE:
      // 7A FIX: Skip web server and WiFi status checks in LR mode
      // ESP-NOW LR requires STA mode radio which is unchanged in setupEspNow()
      // We only disable web server polling since there's no AP to serve
      #if !ESPNOW_LONG_RANGE_MODE
      if (settings.webServerEnabled) loopWebServer();
      checkWiFiStatus();
      #endif
      loopLocalSensor();
      checkSerialWifiConfig();

      // Update our own state periodically (heartbeat)
      // BUG FIX #12: Increment sequence on heartbeat to give receivers clear signal
      if (millis() - lastHeartbeatTime > (HEARTBEAT_INTERVAL_SEC * 1000UL)) {
        lastHeartbeatTime = millis();
        localUptimeSec = millis() / 1000;
        localSequence++;  // BUG FIX #12: Increment sequence on heartbeat

        readBattery();  // Read battery voltage on heartbeat

        // Update our own node state
        network.nodes[0].last_seq = localSequence;
        network.nodes[0].last_uptime = localUptimeSec;
        network.nodes[0].last_seen = millis();
        network.nodes[0].battery_mv = localBatteryMv;

        DEBUG_PRINTF("Self heartbeat: seq=%d uptime=%lu\n", localSequence, localUptimeSec);
      }

      // Add random jitter (0-500ms) to prevent synchronized collisions with other gateway
      static uint32_t txJitter = random(0, 500);
      // BUG FIX #16e: Defer TX if we just received a packet
      // This prevents immediate collision when both gateways try to respond
      {
        bool recentRx = (millis() - lastLoRaRxTime < 1500);
        if (!recentRx && millis() - lastLoRaTx > (LORA_TX_INTERVAL_MS + txJitter)) {
          sendLoRaAggregate();
          lastLoRaTx = millis();
          txJitter = random(0, 500);  // New jitter for next transmission
        }
      }
      break;

    case ROLE_GATEWAY_OFFSHORE:
      if (settings.webServerEnabled) loopWebServer();
      checkWiFiStatus();
      checkSerialWifiConfig();

      // Update our own state periodically (heartbeat)
      // BUG FIX #12: Increment sequence on heartbeat to give receivers clear signal
      if (millis() - lastHeartbeatTime > (HEARTBEAT_INTERVAL_SEC * 1000UL)) {
        lastHeartbeatTime = millis();
        localUptimeSec = millis() / 1000;
        localSequence++;  // BUG FIX #12: Increment sequence on heartbeat

        readBattery();  // Read battery voltage on heartbeat

        network.nodes[0].last_seq = localSequence;
        network.nodes[0].last_uptime = localUptimeSec;
        network.nodes[0].last_seen = millis();
        network.nodes[0].battery_mv = localBatteryMv;

        DEBUG_PRINTF("Self heartbeat: seq=%d uptime=%lu\n", localSequence, localUptimeSec);
      }

      // CRITICAL FIX: Offshore gateway must transmit LoRa aggregates
      // This enables bidirectional communication - onshore can see offshore status
      // Offshore gateway transmits with 5 second offset from onshore
      // This provides adequate separation for TX windows with 10s interval
      {
        static const uint32_t OFFSHORE_TX_OFFSET = 5000;
        static uint32_t txJitterOffshore = random(0, 300);
        // BUG FIX #16e: Defer TX if we just received a packet
        // This prevents immediate collision when both gateways try to respond
        bool recentRxOffshore = (millis() - lastLoRaRxTime < 1500);
        if (!recentRxOffshore && millis() - lastLoRaTx > (LORA_TX_INTERVAL_MS + OFFSHORE_TX_OFFSET + txJitterOffshore)) {
          sendLoRaAggregate();
          lastLoRaTx = millis();
          txJitterOffshore = random(0, 300);  // New jitter for next transmission
        }
      }
      break;

    case ROLE_SENSOR_LORA:
      loopLocalSensor();
      break;
      
    case ROLE_RELAY_LORA:
      break;
  }
  
  // Display current screen
  loopDisplay();
  loopBuzzer();
  loopNodeTimeout();

  // DEBUG: Print LoRa health every 30 seconds
  static unsigned long lastHealthPrint = 0;
  if (millis() - lastHealthPrint > 30000) {
    lastHealthPrint = millis();

    DEBUG_PRINTLN(F("═══ LoRa Health ═══"));
    DEBUG_PRINTF("Ready: %s\n", loraReady ? "YES" : "NO");
    DEBUG_PRINTF("RX count: %lu\n", loraRxCount);
    DEBUG_PRINTF("TX count: %lu\n", loraTxCount);
    DEBUG_PRINTF("RX errors: %lu\n", loraRxErrors);
    DEBUG_PRINTF("TX errors: %lu\n", loraTxErrors);
    DEBUG_PRINTF("Watchdog resets: %lu\n", loraWatchdogResets);
    DEBUG_PRINTF("RX saved from loss: %lu\n", loraRxSavedFromLoss);
    DEBUG_PRINTF("Last RX: %lu sec ago\n", (millis() - lastLoRaRxTime) / 1000);
    DEBUG_PRINTF("Nodes online: %d\n", network.node_count);
    DEBUG_PRINTLN(F("═══════════════════"));
  }

  yield();
}

// ═══════════════════════════════════════════════════════════════════════════
// DISPLAY
// ═══════════════════════════════════════════════════════════════════════════

void setupDisplay() {
  DEBUG_PRINTLN(F("Initializing display..."));

  // CRITICAL: Enable Vext power for OLED (active LOW on Heltec V3)
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);  // LOW = power ON
  delay(100);  // Let power stabilize

  // Initialize display (software I2C handles pins internally)
  display.begin();
  display.setFont(u8g2_font_6x10_tr);
  display.clearBuffer();
  display.drawStr(10, 30, "Starting...");
  display.sendBuffer();

  DEBUG_PRINTLN(F("Display ready"));
}

void wakeDisplay() {
  if (!displaySleeping) return;

  DEBUG_PRINTLN(F("Waking display..."));

  // Re-enable VEXT power (active LOW on Heltec V3)
  digitalWrite(VEXT_PIN, LOW);
  delay(50);  // Let power stabilize

  // Reinitialize display
  display.begin();
  display.setFont(u8g2_font_6x10_tr);

  displaySleeping = false;
  lastActivityTime = millis();

  DEBUG_PRINTLN(F("Display awake"));
}

void sleepDisplay() {
  if (displaySleeping) return;

  DEBUG_PRINTLN(F("Display entering sleep..."));

  // Clear display first
  display.clearBuffer();
  display.sendBuffer();
  delay(10);

  // Turn off VEXT power (HIGH = off on Heltec V3)
  digitalWrite(VEXT_PIN, HIGH);

  displaySleeping = true;
  DEBUG_PRINTLN(F("Display sleeping"));
}

void registerActivity() {
  lastActivityTime = millis();
  if (displaySleeping) {
    wakeDisplay();
  }
}

void loopDisplay() {
  // Check for sleep timeout (only when on live status screen)
  if (!displaySleeping && currentScreen == SCREEN_LIVE_STATUS) {
    if (millis() - lastActivityTime > DISPLAY_SLEEP_MS) {
      sleepDisplay();
      return;
    }
  }

  // Don't update display if sleeping
  if (displaySleeping) return;

  // Rate limit display updates
  if (millis() - lastDisplayUpdate < 200) return;
  lastDisplayUpdate = millis();

  drawCurrentScreen();
}

// Route to appropriate screen drawing function
void drawCurrentScreen() {
  switch (currentScreen) {
    case SCREEN_LIVE_STATUS:    drawLiveStatus(); break;
    case SCREEN_MAIN_MENU:      drawMainMenu(); break;
    case SCREEN_NODE_LIST:      drawNodeList(); break;
    case SCREEN_NODE_DETAILS:   drawNodeDetails(); break;
    case SCREEN_ALERT_HISTORY:  drawAlertHistory(); break;
    case SCREEN_SETTINGS:       drawSettingsMenu(); break;
    case SCREEN_WIFI_CONFIG:    drawWifiConfig(); break;
    case SCREEN_NETWORK_INFO:   drawNetworkInfo(); break;
    case SCREEN_REBOOT_CONFIRM: drawRebootConfirm(); break;
    case SCREEN_SETTING_EDIT:   drawSettingEdit(); break;
    case SCREEN_RESET_ALL_CONFIRM: drawResetAllConfirm(); break;
    default:                    drawLiveStatus(); break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// CONFIRMATION OVERLAY - Brief message display after actions
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Show centered overlay message, auto-dismiss after delayMs
 * Used for: "Saved!", "Sent!", "Silenced", etc.
 */
void showOverlayMessage(const char* message, uint16_t delayMs) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);

  // Calculate centered position
  int textWidth = strlen(message) * FONT_LARGE_WIDTH;
  int x = (UI_SCREEN_WIDTH - textWidth) / 2;
  int y = UI_SCREEN_HEIGHT / 2 + 3;

  // Draw rounded rectangle background
  int boxWidth = textWidth + 16;
  int boxX = (UI_SCREEN_WIDTH - boxWidth) / 2;
  display.drawRFrame(boxX, y - 12, boxWidth, 18, 3);

  display.drawStr(x, y, message);
  display.sendBuffer();
  delay(delayMs);
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: LIVE STATUS (Default view - node grid)
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Grid: Y=11 to Y=53 (2 rows × 21px = 42px)
 * - Footer: Y=55-63 (status bar)
 * - Max X: 5 cells × 25px = 125px + 1px offset = 126px ✓
 */
void drawLiveStatus() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  // Calculate pagination
  const int NODES_PER_PAGE = GRID_COLS * GRID_ROWS;  // 10 nodes per page
  int totalPages = (network.node_count + NODES_PER_PAGE - 1) / NODES_PER_PAGE;
  if (totalPages < 1) totalPages = 1;

  // Clamp page to valid range
  if (liveStatusPage >= totalPages) {
    liveStatusPage = totalPages - 1;
  }

  int startNode = liveStatusPage * NODES_PER_PAGE;
  int endNode = min(startNode + NODES_PER_PAGE, (int)network.node_count);

  // Header with page indicator (truncated to fit 128px)
  char header[24];
  if (totalPages > 1) {
    snprintf(header, sizeof(header), "%s %d/%d P%d/%d",
             currentRole == ROLE_GATEWAY_OFFSHORE ? "RMT" : "ICE",
             network.node_count, MAX_NODES,
             liveStatusPage + 1, totalPages);
  } else {
    snprintf(header, sizeof(header), "%s [%d/%d]",
             currentRole == ROLE_GATEWAY_OFFSHORE ? "REMOTE" : "ICE FISH",
             network.node_count, MAX_NODES);
  }
  display.drawStr(0, UI_HEADER_Y, header);
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  bool blinkOn = ((millis() / 500) % 2) == 0;

  // Draw nodes for current page using UI constants
  for (int idx = startNode; idx < endNode; idx++) {
    int i = idx - startNode;  // Position within page (0-9)
    int row = i / GRID_COLS;
    int col = i % GRID_COLS;
    int x = GRID_START_X + col * GRID_CELL_WIDTH;
    int y = GRID_START_Y + row * GRID_CELL_HEIGHT;

    NodeState* node = &network.nodes[idx];
    bool isFishOn = HAS_FLAG(node->flags, FLAG_FISH_ON);
    bool isLowBat = HAS_FLAG(node->flags, FLAG_LOW_BATTERY);
    bool isSelf = (node->node_id == NODE_ID);  // Self-node detection

    // Draw cell based on status
    if (!node->online) {
      display.drawFrame(x, y, GRID_CELL_WIDTH - 1, GRID_CELL_HEIGHT - 1);
      display.setFont(u8g2_font_5x7_tr);
      display.drawStr(x + 10, y + 12, "?");
    } else if (isFishOn) {
      if (blinkOn) {
        display.drawBox(x, y, GRID_CELL_WIDTH - 1, GRID_CELL_HEIGHT - 1);
        display.setDrawColor(0);
      } else {
        display.drawFrame(x, y, GRID_CELL_WIDTH - 1, GRID_CELL_HEIGHT - 1);
        display.drawLine(x + 2, y + 2, x + GRID_CELL_WIDTH - 4, y + GRID_CELL_HEIGHT - 4);
        display.drawLine(x + GRID_CELL_WIDTH - 4, y + 2, x + 2, y + GRID_CELL_HEIGHT - 4);
      }
    } else {
      display.drawFrame(x, y, GRID_CELL_WIDTH - 1, GRID_CELL_HEIGHT - 1);
    }

    // Self-node visual distinction: double border on gateway's own node
    if (isSelf && node->online && !isFishOn) {
      // Draw inner frame for double-border effect
      display.drawFrame(x + 2, y + 2, GRID_CELL_WIDTH - 5, GRID_CELL_HEIGHT - 5);
    }

    // Node ID
    display.setFont(u8g2_font_5x7_tr);
    char idStr[6];
    snprintf(idStr, sizeof(idStr), "%d", node->node_id);
    int idWidth = strlen(idStr) * FONT_MEDIUM_WIDTH;
    display.drawStr(x + (GRID_CELL_WIDTH - 1 - idWidth) / 2, y + 7, idStr);

    // Status indicator
    const char* statusIcon = !node->online ? "--" : (isFishOn ? "!!" : "OK");
    display.drawStr(x + 7, y + 14, statusIcon);

    // Battery (only if online) - adjusted Y position for smaller cell
    if (node->online) {
      char batStr[5];
      if (isLowBat) {
        snprintf(batStr, sizeof(batStr), "LO!");
      } else {
        uint8_t pct = batteryMvToPercent(node->battery_mv);
        snprintf(batStr, sizeof(batStr), "%d%%", pct);
      }
      display.setFont(u8g2_font_tom_thumb_4x6_tr);
      display.drawStr(x + 3, y + 20, batStr);
    }

    display.setDrawColor(1);
  }

  // Fill empty cells on current page
  int nodesOnPage = endNode - startNode;
  display.setFont(u8g2_font_5x7_tr);
  for (int i = nodesOnPage; i < NODES_PER_PAGE; i++) {
    int row = i / GRID_COLS;
    int col = i % GRID_COLS;
    int x = GRID_START_X + col * GRID_CELL_WIDTH;
    int y = GRID_START_Y + row * GRID_CELL_HEIGHT;
    display.drawFrame(x, y, GRID_CELL_WIDTH - 1, GRID_CELL_HEIGHT - 1);
    display.drawStr(x + 8, y + 12, "-");
  }

  // Bottom status bar (Y=55-63 zone, footer at Y=62)
  display.setFont(u8g2_font_5x7_tr);
  if (activeAlerts && !alertsSilenced) {
    if (blinkOn) {
      display.drawBox(0, 55, UI_SCREEN_WIDTH, 9);
      display.setDrawColor(0);
    }
    display.drawStr(32, UI_FOOTER_Y, "!! FISH ON !!");
    display.setDrawColor(1);
  } else if (alertsSilenced) {
    uint32_t remainingSec = 0;
    if (silenceExpireTime > millis()) {
      remainingSec = (silenceExpireTime - millis()) / 1000;
    }
    char silenceStr[24];
    snprintf(silenceStr, sizeof(silenceStr), "SILENT %lum%02lus",
             remainingSec / 60, remainingSec % 60);
    display.drawStr(0, UI_FOOTER_Y, silenceStr);
  } else {
    char status[28];
    if (totalPages > 1) {
      snprintf(status, sizeof(status), "</>:pg S:sil ESC:menu");
    } else {
      const char* ip = wifiStaConnected ? staIpAddress.c_str() : apIpAddress.c_str();
      snprintf(status, sizeof(status), "%s ESC:menu", ip);
    }
    display.drawStr(0, UI_FOOTER_Y, status);
  }

  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: MAIN MENU
// ═══════════════════════════════════════════════════════════════════════════

const char* mainMenuItems[] = {
  "Live Status",
  "Node List",
  "Alert History",
  "Settings",
  "Network Info",
  "Reboot",
  "Reset All Nodes"  // Only available on GATEWAY_ONSHORE
};
#define MAIN_MENU_COUNT 7

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Content: Y=20,29,38,47 (4 items, 9px spacing with 5x7 font)
 * - Footer: Y=62 (font 5x7)
 * - Max X: "Reset All Nodes" = 15 chars × 5px = 75px ✓
 */
void drawMainMenu() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);  // Use medium font for more items

  display.drawStr(35, UI_HEADER_Y, "MAIN MENU");
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  // Calculate visible window - max 4 items to fit content zone
  int visibleStart = menuScrollOffset;
  int visibleEnd = min(visibleStart + MENU_MAX_VISIBLE, (int)MAIN_MENU_COUNT);

  for (int i = visibleStart; i < visibleEnd; i++) {
    int displayRow = i - visibleStart;  // 0, 1, 2, 3
    int y = 20 + (displayRow * 9);      // Y=20, 29, 38, 47 (ends at 47+7=54, before footer)

    if (i == menuSelection) {
      display.drawBox(0, y - 6, UI_SCREEN_WIDTH, 9);
      display.setDrawColor(0);
    }
    display.drawStr(10, y, mainMenuItems[i]);
    display.setDrawColor(1);
  }

  // Show scroll indicators with context (item N of M)
  display.setFont(u8g2_font_tom_thumb_4x6_tr);
  if (menuScrollOffset > 0) {
    display.drawStr(118, 16, "^");  // More items above
  }
  if (visibleEnd < MAIN_MENU_COUNT) {
    char scrollInfo[8];
    snprintf(scrollInfo, sizeof(scrollInfo), "%d/%d", menuSelection + 1, MAIN_MENU_COUNT);
    display.drawStr(110, 53, scrollInfo);
  }

  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(0, UI_FOOTER_Y, "^v:sel Enter:ok ESC:back");
  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: NODE LIST
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Content: Y=20,29,38,47 (4 items, 9px spacing with 5x7 font)
 * - Footer: Y=62 (font 5x7)
 * - Max X: "16: OFF FISH!" = 13 chars × 5px = 65px ✓
 */
void drawNodeList() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  display.drawStr(35, UI_HEADER_Y, "NODE LIST");
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  if (network.node_count == 0) {
    display.drawStr(30, 35, "No nodes");
  } else {
    int visibleStart = menuScrollOffset;
    int visibleEnd = min(visibleStart + MENU_MAX_VISIBLE, (int)network.node_count);

    for (int i = visibleStart; i < visibleEnd; i++) {
      NodeState* node = &network.nodes[i];
      int displayRow = i - visibleStart;
      int y = 20 + (displayRow * 9);  // Y=20, 29, 38, 47

      if (i == menuSelection) {
        display.drawBox(0, y - 6, UI_SCREEN_WIDTH, 9);
        display.setDrawColor(0);
      }

      char line[24];
      snprintf(line, sizeof(line), "%d: %s %s",
               node->node_id,
               node->online ? "ON" : "OFF",
               HAS_FLAG(node->flags, FLAG_FISH_ON) ? "FISH!" : "");
      display.drawStr(5, y, line);
      display.setDrawColor(1);
    }

    // Show scroll indicators with context
    display.setFont(u8g2_font_tom_thumb_4x6_tr);
    if (menuScrollOffset > 0) {
      display.drawStr(118, 16, "^");
    }
    if (visibleEnd < network.node_count) {
      char scrollInfo[8];
      snprintf(scrollInfo, sizeof(scrollInfo), "%d/%d", menuSelection + 1, network.node_count);
      display.drawStr(108, 53, scrollInfo);
    }
  }

  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(0, UI_FOOTER_Y, "^v:sel Enter:details");
  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: NODE DETAILS
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Content: Y=18,26,34,42,50 (5 lines, 8px spacing with 5x7 font)
 * - Footer: Y=62 (navigation hints)
 * - Max X: "Battery: 4500mV [====]" = 22 chars × 5px = 110px ✓
 */
void drawNodeDetails() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);  // Use smaller font to fit 5 lines

  if (selectedNodeIdx >= network.node_count) {
    display.drawStr(20, 35, "Node not found");
    display.sendBuffer();
    return;
  }

  NodeState* node = &network.nodes[selectedNodeIdx];

  char title[20];
  snprintf(title, sizeof(title), "NODE %d", node->node_id);
  display.drawStr(40, UI_HEADER_Y, title);
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  char line[32];

  // Line 1: Name (truncated if needed)
  snprintf(line, sizeof(line), "Name: %.12s", node->name[0] ? node->name : "--");
  display.drawStr(0, 18, line);

  // Line 2: Status
  snprintf(line, sizeof(line), "Status: %s", node->online ? "Online" : "Offline");
  display.drawStr(0, 26, line);

  // Line 3: Battery with visual bar
  uint8_t pct = batteryMvToPercent(node->battery_mv);
  int bars = (pct + 10) / 20;  // 0-5 bars
  char batBar[7] = "[    ]";
  for (int i = 0; i < bars && i < 4; i++) batBar[i + 1] = '=';
  snprintf(line, sizeof(line), "Batt: %d%% %s", pct, batBar);
  display.drawStr(0, 34, line);

  // Line 4: RSSI with visual bars
  int rssiLevel = 0;
  if (node->rssi > -60) rssiLevel = 4;
  else if (node->rssi > -75) rssiLevel = 3;
  else if (node->rssi > -90) rssiLevel = 2;
  else if (node->rssi > -100) rssiLevel = 1;
  char rssiBar[6] = "[...]";
  for (int i = 0; i < rssiLevel && i < 4; i++) rssiBar[i + 1] = '|';
  snprintf(line, sizeof(line), "RSSI: %ddBm %s", node->rssi, rssiBar);
  display.drawStr(0, 42, line);

  // Line 5: Last seen
  if (node->last_seen > 0) {
    uint32_t ago = (millis() - node->last_seen) / 1000;
    if (ago < 60) {
      snprintf(line, sizeof(line), "Seen: %lus ago", ago);
    } else if (ago < 3600) {
      snprintf(line, sizeof(line), "Seen: %lum ago", ago / 60);
    } else {
      snprintf(line, sizeof(line), "Seen: %luh ago", ago / 3600);
    }
  } else {
    snprintf(line, sizeof(line), "Seen: never");
  }
  display.drawStr(0, 50, line);

  // Footer with navigation hints
  display.drawStr(0, UI_FOOTER_Y, "ESC:back </>:prev/next");

  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: ALERT HISTORY
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Content: Y=19,27,35,43 (4 alerts, 8px spacing with 5x7 font)
 * - Scroll info: Y=51 (if more alerts)
 * - Footer: Y=62 (font 5x7)
 * - Max X: "Node 16: 999h ago *" = 19 chars × 5px = 95px ✓
 */
void drawAlertHistory() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  display.drawStr(25, UI_HEADER_Y, "ALERT HISTORY");
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  if (alertHistoryCount == 0) {
    display.drawStr(30, 35, "No alerts");
  } else {
    // Show max 4 alerts to fit in content zone
    const int MAX_VISIBLE_ALERTS = 4;
    int startIdx = (alertHistoryIdx - min(alertHistoryCount, (uint8_t)MAX_VISIBLE_ALERTS) + ALERT_HISTORY_SIZE) % ALERT_HISTORY_SIZE;
    int count = min(MAX_VISIBLE_ALERTS, (int)alertHistoryCount);

    for (int i = 0; i < count; i++) {
      int idx = (startIdx + i) % ALERT_HISTORY_SIZE;
      AlertRecord* rec = &alertHistory[idx];
      int y = 19 + (i * 8);  // Y=19, 27, 35, 43 (ends at 43+7=50, before footer)

      uint32_t ago = (millis() - rec->timestamp) / 1000;
      char line[24];
      if (ago < 60) {
        snprintf(line, sizeof(line), "Node %d: %lus ago%s",
                 rec->nodeId, ago, rec->active ? " *" : "");
      } else if (ago < 3600) {
        snprintf(line, sizeof(line), "Node %d: %lum ago%s",
                 rec->nodeId, ago / 60, rec->active ? " *" : "");
      } else {
        snprintf(line, sizeof(line), "Node %d: %luh ago%s",
                 rec->nodeId, ago / 3600, rec->active ? " *" : "");
      }
      display.drawStr(0, y, line);
    }

    // Show scroll indicator if more alerts exist
    if (alertHistoryCount > MAX_VISIBLE_ALERTS) {
      display.setFont(u8g2_font_tom_thumb_4x6_tr);
      char moreInfo[12];
      snprintf(moreInfo, sizeof(moreInfo), "+%d more", alertHistoryCount - MAX_VISIBLE_ALERTS);
      display.drawStr(90, 51, moreInfo);
      display.setFont(u8g2_font_5x7_tr);
    }
  }

  display.drawStr(0, UI_FOOTER_Y, "ESC:back");
  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: SETTINGS MENU
// ═══════════════════════════════════════════════════════════════════════════

const char* settingsMenuLabels[] = {
  "WiFi Config",
  "Buzzer",
  "Alert Hold (s)",
  "Heartbeat (s)",
  "Web Server",
  "WiFi Mode",
  "Reed Polarity",
  "Back"
};
#define SETTINGS_MENU_COUNT 8

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Content: Y=18,26,34,42,50 (5 items, 8px spacing with 5x7 font)
 * - Footer: Y=62 (font 5x7)
 * - Max X: "WiFi Mode: AP (forced)" = 21 chars × 5px = 105px ✓
 */
void drawSettingsMenu() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  display.drawStr(40, UI_HEADER_Y, "SETTINGS");
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  // Calculate visible window - max 5 items with smaller font (fits in content zone)
  const int MAX_VISIBLE = 5;
  int visibleStart = menuScrollOffset;
  int visibleEnd = min(visibleStart + MAX_VISIBLE, (int)SETTINGS_MENU_COUNT);

  for (int i = visibleStart; i < visibleEnd; i++) {
    int displayRow = i - visibleStart;
    int y = 18 + (displayRow * 8);  // Y=18, 26, 34, 42, 50 (ends at 50+7=57, before footer)

    if (i == menuSelection) {
      display.drawBox(0, y - 6, UI_SCREEN_WIDTH, 8);
      display.setDrawColor(0);
    }

    char line[28];
    switch (i) {
      case 0:  // WiFi Config
        snprintf(line, sizeof(line), "%s", settingsMenuLabels[i]);
        break;
      case 1:  // Buzzer
        snprintf(line, sizeof(line), "%s: %s", settingsMenuLabels[i],
                 settings.buzzerEnabled ? "ON" : "OFF");
        break;
      case 2:  // Alert Hold
        snprintf(line, sizeof(line), "%s: %d", settingsMenuLabels[i],
                 settings.alertHoldSec);
        break;
      case 3:  // Heartbeat
        snprintf(line, sizeof(line), "%s: %d", settingsMenuLabels[i],
                 settings.heartbeatSec);
        break;
      case 4:  // Web Server
        snprintf(line, sizeof(line), "%s: %s", settingsMenuLabels[i],
                 settings.webServerEnabled ? "ON" : "OFF");
        break;
      case 5:  // WiFi Mode
        {
          const char* modeStr;
          if (currentRole == ROLE_GATEWAY_ONSHORE) {
            modeStr = "AP (forced)";
          } else {
            switch (settings.wifiModeSetting) {
              case 0: modeStr = "AP Only"; break;
              case 1: modeStr = "STA Only"; break;
              case 2: modeStr = "AP+STA"; break;
              default: modeStr = "AP+STA"; break;
            }
          }
          snprintf(line, sizeof(line), "%s: %s", settingsMenuLabels[i], modeStr);
        }
        break;
      case 6:  // Reed Polarity
        snprintf(line, sizeof(line), "%s: %s", settingsMenuLabels[i],
                 settings.reedActiveHigh ? "HIGH" : "LOW");
        break;
      case 7:  // Back
        snprintf(line, sizeof(line), "< %s", settingsMenuLabels[i]);
        break;
    }
    display.drawStr(3, y, line);
    display.setDrawColor(1);
  }

  // Show scroll indicators with context
  display.setFont(u8g2_font_tom_thumb_4x6_tr);
  if (menuScrollOffset > 0) {
    display.drawStr(120, 15, "^");
  }
  if (visibleEnd < SETTINGS_MENU_COUNT) {
    char scrollInfo[8];
    snprintf(scrollInfo, sizeof(scrollInfo), "%d/%d", menuSelection + 1, SETTINGS_MENU_COUNT);
    display.drawStr(108, 53, scrollInfo);
  }

  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(0, UI_FOOTER_Y, "^v:sel Enter:edit ESC:back");
  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: WIFI CONFIG (text entry)
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Content: SSID at Y=22, Pass at Y=34, field indicator at Y=46
 * - Footer: Y=62 (help text)
 * - Max X: text truncated with "..." if too long
 */
void drawWifiConfig() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  display.drawStr(30, UI_HEADER_Y, "WIFI CONFIG");
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  // SSID field - show last 14 chars with "..." prefix if too long
  display.drawStr(0, 22, "SSID:");
  if (inputField == 0) {
    // Show input with truncation for long strings
    int maxChars = 14;  // (128-30) / 5 = ~19, leave room for cursor
    int len = strlen(inputBuffer);
    if (len > maxChars) {
      char truncated[18];
      snprintf(truncated, sizeof(truncated), "...%s", inputBuffer + len - maxChars + 3);
      display.drawStr(30, 22, truncated);
      int cursorX = 30 + (maxChars * 5);
      if ((millis() / 500) % 2) {
        display.drawStr(cursorX, 22, "_");
      }
    } else {
      display.drawStr(30, 22, inputBuffer);
      int cursorX = 30 + (inputPos * 5);
      if (cursorX < 120 && (millis() / 500) % 2) {
        display.drawStr(cursorX, 22, "_");
      }
    }
  } else {
    int len = strlen(inputSsid);
    if (len > 14) {
      char truncated[18];
      snprintf(truncated, sizeof(truncated), "...%s", inputSsid + len - 11);
      display.drawStr(30, 22, truncated);
    } else {
      display.drawStr(30, 22, inputSsid);
    }
  }

  // Password field
  display.drawStr(0, 34, "Pass:");
  if (inputField == 1) {
    int len = strlen(inputBuffer);
    int maxChars = 14;
    if (len > maxChars) {
      char truncated[18];
      snprintf(truncated, sizeof(truncated), "...%s", inputBuffer + len - maxChars + 3);
      display.drawStr(30, 34, truncated);
      int cursorX = 30 + (maxChars * 5);
      if ((millis() / 500) % 2) {
        display.drawStr(cursorX, 34, "_");
      }
    } else {
      display.drawStr(30, 34, inputBuffer);
      int cursorX = 30 + (inputPos * 5);
      if (cursorX < 120 && (millis() / 500) % 2) {
        display.drawStr(cursorX, 34, "_");
      }
    }
  } else {
    char masked[16];
    int len = min(14, (int)strlen(inputPassword));
    memset(masked, '*', len);
    masked[len] = '\0';
    display.drawStr(30, 34, masked);
  }

  // Field indicator and help
  display.drawStr(0, 46, inputField == 0 ? ">[SSID]" : ">[PASS]");
  display.drawStr(50, 46, "Tab:next");

  display.drawStr(0, UI_FOOTER_Y, "Enter:save ESC:cancel");
  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: NETWORK INFO
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Content: Y=18,26,34,42,50 (5 lines, 8px spacing with 5x7 font)
 * - Footer: Y=62 (font 5x7)
 */
void drawNetworkInfo() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  // Page-based network info with LoRa diagnostics
  // menuScrollOffset: 0 = basic info, 1 = LoRa stats
  char line[32];

  if (menuScrollOffset == 0) {
    // Page 1: Basic network info
    display.drawStr(20, UI_HEADER_Y, "NETWORK INFO 1/2");
    display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

    snprintf(line, sizeof(line), "Node: %d (%.10s)", NODE_ID, NODE_NAME);
    display.drawStr(0, 18, line);

    snprintf(line, sizeof(line), "Role: %s", getRoleName(currentRole));
    display.drawStr(0, 26, line);

    if (wifiApActive) {
      snprintf(line, sizeof(line), "AP: %s", apIpAddress.c_str());
      display.drawStr(0, 34, line);
    } else {
      display.drawStr(0, 34, "AP: inactive");
    }

    if (wifiStaConnected) {
      snprintf(line, sizeof(line), "STA: %s", staIpAddress.c_str());
    } else {
      snprintf(line, sizeof(line), "STA: disconnected");
    }
    display.drawStr(0, 42, line);

    // ESP-NOW status
    display.drawStr(0, 50, espNowReady ? "ESP-NOW: ready" : "ESP-NOW: error");

    display.drawStr(0, UI_FOOTER_Y, "^v:page ESC:back");
  } else {
    // Page 2: LoRa diagnostics
    display.drawStr(25, UI_HEADER_Y, "LORA STATS 2/2");
    display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

    uint32_t upSec = millis() / 1000;
    snprintf(line, sizeof(line), "Up:%luh%lum LoRa:%s",
             upSec / 3600, (upSec % 3600) / 60, loraReady ? "OK" : "ERR");
    display.drawStr(0, 18, line);

    snprintf(line, sizeof(line), "RX:%lu TX:%lu", loraRxCount, loraTxCount);
    display.drawStr(0, 26, line);

    snprintf(line, sizeof(line), "RXerr:%lu TXerr:%lu", loraRxErrors, loraTxErrors);
    display.drawStr(0, 34, line);

    snprintf(line, sizeof(line), "ChkFail:%lu WDrst:%lu", loraChecksumFails, loraWatchdogResets);
    display.drawStr(0, 42, line);

    snprintf(line, sizeof(line), "Nodes:%d Alerts:%d", network.node_count, network.alert_count);
    display.drawStr(0, 50, line);

    display.drawStr(0, UI_FOOTER_Y, "^v:page ESC:back");
  }

  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: REBOOT CONFIRM
// ═══════════════════════════════════════════════════════════════════════════

void drawRebootConfirm() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);

  display.drawStr(40, 25, "REBOOT?");
  display.drawStr(15, 40, "Enter: Yes  ESC: No");

  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: RESET ALL NODES CONFIRMATION
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Content: Y=18,26,34,42 (4 lines, 8px spacing)
 * - Footer: Y=62
 */
void drawResetAllConfirm() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  display.drawStr(15, UI_HEADER_Y, "RESET ALL NODES?");
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  display.drawStr(5, 18, "This will reboot ALL");
  display.drawStr(5, 26, "nodes in the network!");
  display.drawStr(5, 36, "Sensors may miss alerts");
  display.drawStr(5, 44, "during reset period.");

  display.drawStr(0, UI_FOOTER_Y, "Enter:Yes ESC:Cancel");
  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// SCREEN: SETTING EDIT (numeric value)
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Layout verification for 128x64 OLED:
 * - Header: Y=7 (font 5x7)
 * - Separator: Y=9
 * - Value display: Y=32 (centered)
 * - Help: Y=48
 * - Footer: Y=62
 */
void drawSettingEdit() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);

  const char* label = "";
  int minVal = 0, maxVal = 999;

  switch (editingSettingIdx) {
    case 2: label = "Alert Hold (sec)"; minVal = 5; maxVal = 300; break;
    case 3: label = "Heartbeat (sec)"; minVal = 10; maxVal = 600; break;
  }

  display.drawStr(20, UI_HEADER_Y, label);
  display.drawLine(0, UI_SEPARATOR_Y, UI_SCREEN_WIDTH, UI_SEPARATOR_Y);

  // Large value display centered
  display.setFont(u8g2_font_6x10_tr);
  char valStr[16];
  snprintf(valStr, sizeof(valStr), "< %ld >", editingValue);
  int valWidth = strlen(valStr) * FONT_LARGE_WIDTH;
  display.drawStr((UI_SCREEN_WIDTH - valWidth) / 2, 32, valStr);

  // Help text
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(10, 45, "^v:+/-1  </>:+/-10");

  display.drawStr(0, UI_FOOTER_Y, "Enter:save ESC:cancel");

  display.sendBuffer();
}

// ═══════════════════════════════════════════════════════════════════════════
// LORA
// ═══════════════════════════════════════════════════════════════════════════

void setupLoRa() {
  DEBUG_PRINTLN(F("Initializing LoRa..."));

  // CRITICAL: Ensure VEXT power is enabled (Heltec V3 specific)
  // This is also done in setupDisplay, but we do it here too for safety
  // in case setupLoRa is called during a reset without full init
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);  // LOW = ON for Heltec V3
  delay(100);  // Let power stabilize

  // Initialize SPI
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  // Explicit pin modes for SX1262
  pinMode(LORA_RST, OUTPUT);
  pinMode(LORA_BUSY, INPUT);
  pinMode(LORA_DIO1, INPUT);

  // Hardware reset before init
  digitalWrite(LORA_RST, LOW);
  delay(20);
  digitalWrite(LORA_RST, HIGH);
  delay(100);  // Wait for radio to boot (increased from 50ms)

  int state = radio.begin(
    LORA_FREQUENCY,
    LORA_BANDWIDTH / 1000.0,
    LORA_SPREADING,
    LORA_CODING_RATE,
    LORA_SYNC_WORD,
    LORA_TX_POWER,
    LORA_PREAMBLE
  );

  if (state != RADIOLIB_ERR_NONE) {
    DEBUG_PRINTF("LoRa init failed: %d\n", state);
    loraRxErrors++;
    return;
  }

  // Set DIO1 action for RX interrupt
  radio.setDio1Action(onLoRaReceive);

  // Explicitly set to RX mode
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    DEBUG_PRINTF("LoRa startReceive failed: %d\n", state);
    loraRxErrors++;
    return;
  }

  loraReady = true;
  radioState = RADIO_STATE_RX;
  lastLoRaRxTime = millis();
  DEBUG_PRINTLN(F("LoRa ready - listening"));
}

// ═══════════════════════════════════════════════════════════════════════════
// RELIABLE LORA RECEIVE MODE - Forces state machine reset
// ═══════════════════════════════════════════════════════════════════════════

bool ensureLoRaReceiveMode() {
  if (!loraReady) return false;

  // CRITICAL: Wait for BUSY pin to go LOW before sending commands
  // SX1262 sets BUSY high during TX/RX operations
  unsigned long busyStart = millis();
  while (digitalRead(LORA_BUSY) == HIGH) {
    if (millis() - busyStart > 1000) {
      DEBUG_PRINTLN("ERROR: LoRa BUSY pin stuck HIGH - hardware reset needed");
      // Force hardware reset
      digitalWrite(LORA_RST, LOW);
      delay(10);
      digitalWrite(LORA_RST, HIGH);
      delay(50);
      busyStart = millis();  // Retry after reset
    }
    delay(1);
  }

  // CRITICAL: Force standby first to reset radio state machine
  // This prevents the radio from getting stuck in TX mode
  int standbyState = radio.standby();
  if (standbyState != RADIOLIB_ERR_NONE) {
    DEBUG_PRINTF("LoRa standby failed: %d\n", standbyState);
  }
  delay(2);  // SX1262 needs ~1ms to enter standby

  // Try to enter RX mode with retries
  for (int retry = 0; retry < 5; retry++) {
    int state = radio.startReceive();

    if (state == RADIOLIB_ERR_NONE) {
      if (retry > 0) {
        DEBUG_PRINTF("LoRa startReceive OK on retry %d\n", retry);
      }
      loraInterrupt = false;   // Clear any stale RX interrupt flag
      loraTxComplete = false;  // C1 FIX: Clear TX complete flag
      radioState = RADIO_STATE_RX;
      return true;
    }

    loraStartRxRetries++;
    DEBUG_PRINTF("LoRa startReceive failed (attempt %d): %d\n", retry + 1, state);

    // If chip not responding, try hardware reset
    if (state == RADIOLIB_ERR_SPI_CMD_TIMEOUT || retry >= 2) {
      DEBUG_PRINTLN("LoRa chip unresponsive - hardware reset");

      // Hardware reset sequence
      pinMode(LORA_RST, OUTPUT);
      digitalWrite(LORA_RST, LOW);
      delay(20);
      digitalWrite(LORA_RST, HIGH);
      delay(50);

      // Reinitialize radio
      int initState = radio.begin(
        LORA_FREQUENCY,
        LORA_BANDWIDTH / 1000.0,
        LORA_SPREADING,
        LORA_CODING_RATE,
        LORA_SYNC_WORD,
        LORA_TX_POWER,
        LORA_PREAMBLE
      );

      if (initState == RADIOLIB_ERR_NONE) {
        radio.setDio1Action(onLoRaReceive);
        DEBUG_PRINTLN("LoRa reinitialized after reset");
      } else {
        DEBUG_PRINTF("LoRa reinit failed: %d\n", initState);
      }
    }

    delay(20 * (retry + 1));  // Increasing delay between retries
  }

  loraRxErrors++;
  DEBUG_PRINTLN("LoRa startReceive FAILED after all retries!");
  radioState = RADIO_STATE_IDLE;
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// PROCESS PENDING RX - Must be called BEFORE transmitting to avoid packet loss
// ═══════════════════════════════════════════════════════════════════════════
void processPendingLoRaRx() {
    // If no interrupt pending or we're not in RX state, nothing to do
    if (!loraInterrupt || radioState != RADIO_STATE_RX) {
        return;
    }

    // Check if radio is ready (BUSY low)
    if (digitalRead(LORA_BUSY) == HIGH) {
        return;  // Radio busy, can't read now
    }

    // Check if there's actually a packet to read
    int len = radio.getPacketLength();
    if (len <= 0 || len > sizeof(loraBuffer)) {
        // No valid packet - clear the spurious interrupt
        loraInterrupt = false;
        return;
    }

    // Valid packet waiting - read and process it NOW before TX
    DEBUG_PRINTLN("LoRa: Processing pending RX before TX");
    loraRxSavedFromLoss++;  // Track how often we save packets

    int state = radio.readData(loraBuffer, len);
    loraInterrupt = false;  // Clear after reading

    if (state == RADIOLIB_ERR_NONE) {
        int rssi = radio.getRSSI();
        float snr = radio.getSNR();
        loraRxCount++;
        lastLoRaRxTime = millis();

        DEBUG_PRINTF("LoRa RX (pre-TX): %d bytes, RSSI:%d, SNR:%.1f\n", len, rssi, snr);
        processLoRaMessage(loraBuffer, len, rssi);
    } else {
        loraRxErrors++;
        DEBUG_PRINTF("LoRa RX error (pre-TX): %d\n", state);
    }

    // Return to RX mode for clean state
    ensureLoRaReceiveMode();
}

// ═══════════════════════════════════════════════════════════════════════════
// LORA TRANSMIT WITH BUSY WAIT - Wait for TX complete before returning
// ═══════════════════════════════════════════════════════════════════════════

// Helper function: Transmit data and wait for completion
// Returns RADIOLIB_ERR_NONE on success, error code otherwise
int loRaTransmitWithWait(uint8_t* data, size_t len) {
    if (!loraReady) return RADIOLIB_ERR_UNKNOWN;

    // CRITICAL FIX: Process any pending RX packet BEFORE transmitting
    // This prevents the race condition where we lose received packets
    processPendingLoRaRx();

    // Transition to TX state - next interrupt will be TX complete (sets loraTxComplete)
    radioState = RADIO_STATE_TX;
    loraTxComplete = false;  // C1 FIX: Clear TX complete flag before transmit

    int result = radio.transmit(data, len);

    if (result == RADIOLIB_ERR_NONE) {
        // C2 FIX: Wait for TX complete with max attempt counter (200 iterations = ~2 seconds)
        // SX1262 sets BUSY high during TX
        unsigned long txStart = millis();
        int busyAttempts = 0;
        const int maxBusyAttempts = 200;  // ~2 seconds at 10ms per iteration

        while (digitalRead(LORA_BUSY) == HIGH && busyAttempts < maxBusyAttempts) {
            delay(10);
            busyAttempts++;
        }

        // C2 FIX: If BUSY stuck HIGH, force radio reset and reinitialize
        if (digitalRead(LORA_BUSY) == HIGH) {
            DEBUG_PRINTLN("ERROR: BUSY stuck HIGH after TX - forcing radio reset");
            loraWatchdogResets++;

            // Force hardware reset
            loraReady = false;
            digitalWrite(LORA_RST, LOW);
            delay(20);
            digitalWrite(LORA_RST, HIGH);
            delay(100);

            // Reinitialize radio
            setupLoRa();

            loraTxComplete = false;
            loraInterrupt = false;
            radioState = RADIO_STATE_RX;
            return RADIOLIB_ERR_TX_TIMEOUT;
        }

        loraTxCount++;
    } else {
        loraTxErrors++;
        DEBUG_PRINTF("LoRa TX error: %d\n", result);
    }

    // TX complete - clear interrupt flags
    loraTxComplete = false;  // C1 FIX: Clear TX complete flag
    loraInterrupt = false;   // Clear any stale RX flag
    radioState = RADIO_STATE_IDLE;

    // C3 FIX: ALWAYS return to RX mode after TX (even on error)
    ensureLoRaReceiveMode();
    radioState = RADIO_STATE_RX;

    return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// UNIFIED LORA TRANSMIT - Handles TX with proper state cleanup
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Transmit a LoRa packet with proper state management
 *
 * CRITICAL: Processes pending RX before TX to prevent packet loss.
 * Uses radio state tracking to distinguish TX-complete from RX-complete interrupts.
 */
bool loRaTransmitPacket(uint8_t* data, size_t len, const char* label, int maxRetries = 3) {
    if (!loraReady) {
        DEBUG_PRINTF("LoRa TX %s: Radio not ready\n", label);
        return false;
    }

    // CRITICAL FIX: Process any pending RX packet BEFORE transmitting
    processPendingLoRaRx();

    // Wait for BUSY pin to be LOW (radio ready for commands)
    unsigned long busyWait = millis();
    while (digitalRead(LORA_BUSY) == HIGH) {
        if (millis() - busyWait > 500) {
            DEBUG_PRINTF("LoRa TX %s: BUSY timeout\n", label);
            break;
        }
        delay(1);
    }

    // Force standby before transmit
    radio.standby();
    delay(2);  // SX1262 needs ~1ms to enter standby

    // BUG FIX #16d: Skip LBT if we received a packet recently
    // If we just received, the channel is clearly working - no need for CAD
    bool skipLBT = (millis() - lastLoRaRxTime < 2000);
    bool channelClear = skipLBT;

    if (!skipLBT) {
        // LBT with CAD - max 2 attempts with longer backoff (200-500ms)
        for (int attempt = 0; attempt < 2; attempt++) {
            if (!loRaChannelBusy()) {
                channelClear = true;
                break;
            }
            // Longer backoff with jitter to avoid synchronized collisions
            int backoffMs = 200 + random(0, 300);
            DEBUG_PRINTF("LoRa TX %s: Channel busy, backoff %dms (attempt %d)\n",
                         label, backoffMs, attempt + 1);
            delay(backoffMs);
        }
    }

    // Don't spam the log if we proceed anyway
    if (!channelClear) {
        DEBUG_PRINTF("LoRa TX %s: Proceeding after LBT\n", label);
    }

    // Transition to TX state - next interrupt will be TX complete (sets loraTxComplete)
    radioState = RADIO_STATE_TX;
    loraTxComplete = false;  // C1 FIX: Clear TX complete flag before transmit

    // Transmit with retries
    bool success = false;
    int retryDelay = 50;

    for (int retry = 0; retry < maxRetries; retry++) {
        int result = radio.transmit(data, len);

        if (result == RADIOLIB_ERR_NONE) {
            // C2 FIX: Wait for TX complete with max attempt counter
            unsigned long txStart = millis();
            int busyAttempts = 0;
            const int maxBusyAttempts = 200;

            while (digitalRead(LORA_BUSY) == HIGH && busyAttempts < maxBusyAttempts) {
                delay(10);
                busyAttempts++;
            }

            // C2 FIX: If BUSY stuck HIGH, force radio reset
            if (digitalRead(LORA_BUSY) == HIGH) {
                DEBUG_PRINTF("LoRa TX %s: BUSY stuck - forcing radio reset\n", label);
                loraWatchdogResets++;
                loraReady = false;
                digitalWrite(LORA_RST, LOW);
                delay(20);
                digitalWrite(LORA_RST, HIGH);
                delay(100);
                setupLoRa();
                loraTxComplete = false;
                loraInterrupt = false;
                radioState = RADIO_STATE_RX;
                return false;
            }

            loraTxCount++;
            if (retry > 0) {
                DEBUG_PRINTF("LoRa TX %s OK (attempt %d)\n", label, retry + 1);
            } else {
                DEBUG_PRINTF("LoRa TX %s OK\n", label);
            }
            success = true;
            break;
        }

        loraTxErrors++;
        DEBUG_PRINTF("LoRa TX %s failed: %d (attempt %d)\n", label, result, retry + 1);

        if (retry < maxRetries - 1) {
            delay(retryDelay);
            retryDelay *= 2;
            radio.standby();
            delay(2);  // SX1262 needs ~1ms to enter standby
        }
    }

    // TX complete (or failed) - clear interrupt flags and return to RX
    loraTxComplete = false;  // C1 FIX: Clear TX complete flag
    loraInterrupt = false;
    radioState = RADIO_STATE_IDLE;

    // C3 FIX: ALWAYS return to RX mode after TX
    if (!ensureLoRaReceiveMode()) {
        DEBUG_PRINTF("LoRa TX %s: WARNING - failed to return to RX mode!\n", label);
    }
    radioState = RADIO_STATE_RX;

    return success;
}

// ═══════════════════════════════════════════════════════════════════════════
// LORA WATCHDOG - Detects and recovers stuck radio
// ═══════════════════════════════════════════════════════════════════════════

void loopLoRaWatchdog() {
  static unsigned long lastCheck = 0;
  static uint32_t lastRxCountCheck = 0;
  static uint8_t stuckCount = 0;

  // Check every 10 seconds (more aggressive - was 15)
  if (millis() - lastCheck < 10000) return;
  lastCheck = millis();

  if (!loraReady) return;

  // WATCHDOG 1: RX count not increasing - aggressive full reset after 20 seconds
  if (loraRxCount == lastRxCountCheck) {
    stuckCount++;
    DEBUG_PRINTF("LoRa watchdog: RX stalled for %d checks\n", stuckCount);

    if (stuckCount >= 12) {  // 120 seconds with no RX - give time for normal operation
      DEBUG_PRINTLN("LoRa watchdog: Forcing FULL HARDWARE RESET");
      loraWatchdogResets++;

      // Full hardware reset sequence
      loraReady = false;
      digitalWrite(LORA_RST, LOW);
      delay(20);
      digitalWrite(LORA_RST, HIGH);
      delay(100);

      setupLoRa();  // Reinitialize
      stuckCount = 0;
      lastRxCountCheck = loraRxCount;
      return;
    } else {
      // First check - try soft recovery first
      ensureLoRaReceiveMode();
    }
  } else {
    stuckCount = 0;  // Reset counter on success
  }
  lastRxCountCheck = loraRxCount;

  // WATCHDOG 2: Verify radio is actually in RX mode
  // (RadioLib doesn't expose this directly, but we can try startReceive which is idempotent)
  int state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    DEBUG_PRINTF("LoRa watchdog: Radio not in RX mode (err=%d) - recovering\n", state);
    ensureLoRaReceiveMode();
  }
}

void IRAM_ATTR onLoRaReceive() {
    // C1 FIX: Distinguish TX-complete from RX-complete in ISR itself
    // This prevents race conditions where TX interrupt is misinterpreted as RX
    if (radioState == RADIO_STATE_TX) {
        loraTxComplete = true;  // TX done - handled by transmit function
    } else {
        loraInterrupt = true;   // RX packet available
    }
}

// FIXED: Use hardware CAD instead of broken RSSI-based detection
// The SX1262 RSSI reading after standby() is invalid/garbage
bool loRaChannelBusy() {
    if (!loraReady) return false;

    // Ensure we're not in the middle of something
    if (digitalRead(LORA_BUSY) == HIGH) {
        return true;
    }

    // Use SX1262 Channel Activity Detection
    // This properly detects LoRa preambles, unlike raw RSSI
    int state = radio.scanChannel();

    if (state == RADIOLIB_LORA_DETECTED) {
        DEBUG_PRINTLN("CAD: LoRa activity detected");
        // BUG FIX #16c: Return to RX mode after CAD detects activity
        // scanChannel() leaves radio in undefined state - we need to
        // be in RX mode to receive while waiting for the channel to clear
        radio.startReceive();
        radioState = RADIO_STATE_RX;
        return true;
    } else if (state == RADIOLIB_CHANNEL_FREE) {
        return false;
    } else {
        // CAD failed - assume clear and proceed
        DEBUG_PRINTF("CAD error: %d - assuming clear\n", state);
        return false;
    }
}

// CRITICAL: Wait for clear channel with random backoff
bool loRaWaitForClearChannel(int maxAttempts = 10) {
  if (!loraReady) return false;

  for (int attempt = 0; attempt < maxAttempts; attempt++) {
    if (!loRaChannelBusy()) {
      return true;  // Channel is clear
    }

    // Random backoff: 20-80ms to avoid synchronized collisions
    delay(random(20, 80));
  }

  DEBUG_PRINTLN("LoRa channel busy - transmitting anyway (fallback)");
  return true;  // Proceed anyway after max attempts
}

void loopLoRa() {
    if (!loraReady) return;

    // Periodic RX mode verification (every 5 seconds)
    static unsigned long lastRxVerify = 0;
    if (millis() - lastRxVerify > 5000) {
        lastRxVerify = millis();
        if (radioState == RADIO_STATE_RX && digitalRead(LORA_BUSY) == LOW) {
            radio.startReceive();  // Ensure we're listening
        }
    }

    if (!loraInterrupt) return;

    // If we're in TX state, this interrupt is TX complete - ignore
    if (radioState == RADIO_STATE_TX) {
        DEBUG_PRINTLN("LoRa: TX complete interrupt (handled by TX function)");
        loraInterrupt = false;
        return;
    }

    loraInterrupt = false;

    // Check BUSY pin - if HIGH, radio is still processing
    if (digitalRead(LORA_BUSY) == HIGH) {
        DEBUG_PRINTLN("LoRa: Interrupt while BUSY - ignoring");
        return;
    }

    int len = radio.getPacketLength();

    // Invalid length = spurious interrupt
    if (len <= 0 || len > sizeof(loraBuffer)) {
        if (len == 0) {
            if (millis() - lastLoRaTx > 500) {
                DEBUG_PRINTLN("LoRa: Spurious interrupt (len=0)");
            }
        } else {
            DEBUG_PRINTF("LoRa: Invalid packet length %d\n", len);
        }

        // Return to RX mode
        radio.standby();
        delay(2);
        radio.startReceive();
        return;
    }

    // Valid packet - read and process
    int state = radio.readData(loraBuffer, len);

    if (state == RADIOLIB_ERR_NONE) {
        int rssi = radio.getRSSI();
        float snr = radio.getSNR();
        loraRxCount++;
        lastLoRaRxTime = millis();

        DEBUG_PRINTF("LoRa RX: %d bytes, RSSI:%d, SNR:%.1f (total:%lu)\n",
                     len, rssi, snr, loraRxCount);

        processLoRaMessage(loraBuffer, len, rssi);
    } else {
        loraRxErrors++;
        DEBUG_PRINTF("LoRa RX error: %d\n", state);
    }

    // Return to RX mode
    ensureLoRaReceiveMode();
}

// Check if we've seen this config message recently (prevents loops)
// H4 FIX: Extended from 30 to 60 seconds for better reliability with multi-hop
bool isConfigDuplicate(uint8_t originId, uint8_t seq) {
  uint32_t now = millis();
  for (int i = 0; i < CONFIG_DEDUP_SIZE; i++) {
    if (configDedup[i].origin_id == originId &&
        configDedup[i].config_seq == seq &&
        (now - configDedup[i].received_at) < 60000) {  // H4 FIX: 60 second window (was 30)
      return true;
    }
  }
  return false;
}

void recordConfigForDedup(uint8_t originId, uint8_t seq) {
  configDedup[configDedupIdx].origin_id = originId;
  configDedup[configDedupIdx].config_seq = seq;
  configDedup[configDedupIdx].received_at = millis();
  configDedupIdx = (configDedupIdx + 1) % CONFIG_DEDUP_SIZE;
}

void processLoRaMessage(const uint8_t* data, int len, int rssi) {
  // Basic header validation
  if (len < 3) {
    DEBUG_PRINTF("LoRa: Message too short (%d bytes, min 3)\n", len);
    loraRxErrors++;
    return;
  }

  uint8_t netId = data[0];
  uint8_t msgType = data[2];

  if (netId != NETWORK_ID) return;

  switch (msgType) {
    case MSG_LORA_AGGREGATE: {
      if (len < (int)sizeof(LoRaAggregateMessage)) {
        DEBUG_PRINTF("LoRa AGG too short: %d < %d\n", len, (int)sizeof(LoRaAggregateMessage));
        loraRxErrors++;
        break;
      }

      LoRaAggregateMessage* msg = (LoRaAggregateMessage*)data;

      // BUG FIX #3: Verify checksum before processing
      uint8_t expectedChecksum = calculateChecksum((uint8_t*)msg, sizeof(LoRaAggregateMessage) - 1);
      if (msg->checksum != expectedChecksum) {
        loraChecksumFails++;
        DEBUG_PRINTF("LoRa AGG checksum FAIL: got 0x%02X, expected 0x%02X (fails: %lu)\n",
                     msg->checksum, expectedChecksum, loraChecksumFails);
        break;  // Reject corrupted packet
      }

      DEBUG_PRINTF("LoRa AGG from %d: %d nodes, hop=%d, seq=%d\n",
                   msg->sender_id, msg->node_count, msg->hop_count, msg->lora_sequence);

      // ISSUE 1 FIX: sender_id is the ORIGIN hub (relays keep it). Our own aggregate
      // echoed back by a relay carries our ID - nothing to learn from it, never relay it.
      if (msg->sender_id == NODE_ID) {
        break;
      }

      // BUG FIX #13: ALWAYS refresh sender node status on valid aggregate receipt
      // This fixes the "gateway goes offline" bug - the sender's last_seen must be updated
      // regardless of whether individual node entries pass deduplication
      {
        NodeState* senderNode = findOrCreateNode(msg->sender_id);
        if (senderNode) {
          senderNode->last_seen = millis();
          senderNode->online = true;
          senderNode->via_lora = true;
          // ISSUE 1 FIX: RSSI only describes the origin hub when we heard it directly
          if (msg->hop_count == 0) {
            senderNode->rssi = rssi;
          }
          senderNode->role = msg->sender_role;  // Set sender's role from aggregate
          // BUG FIX #2: AGG_FLAG_NODE_OFFLINE is now in agg_flags, not flags
          // No need to clear it from flags anymore
          DEBUG_PRINTF("Sender %d status refreshed: online=true, rssi=%d, role=%d\n",
                       msg->sender_id, rssi, msg->sender_role);
        }
      }

      // Process each node in aggregate
      for (int i = 0; i < msg->node_count && i < 10; i++) {
        NodeStatusCompact* ns = &msg->nodes[i];
        if (ns->node_id == 0 || ns->node_id == NODE_ID) continue;

        // BUG FIX #2: Check agg_flags for offline status (not flags)
        bool reportedOffline = HAS_FLAG(ns->agg_flags, AGG_FLAG_NODE_OFFLINE);

        if (reportedOffline) {
          // Source says node is offline - mark offline here too
          NodeState* node = findOrCreateNode(ns->node_id);
          if (node) {
            // FIX: Don't accept offline report if we have recent direct contact
            // Another gateway may have lost the sensor while we still have it
            // Our direct ESP-NOW knowledge takes precedence over their LoRa report
            if (node->received_direct) {
              bool directRecent = (millis() - node->last_direct_seen < NODE_TIMEOUT_SEC * 1000UL);
              if (directRecent) {
                DEBUG_PRINTF("Node %d: Ignoring offline report from %d (we have recent direct contact)\n",
                             ns->node_id, msg->sender_id);
                continue;  // Skip to next node in aggregate
              }
            }

            if (node->online) {
              DEBUG_PRINTF("Node %d marked OFFLINE (reported by node %d)\n",
                           ns->node_id, msg->sender_id);
              node->online = false;

              // Clear FISH_ON state when node goes offline
              if (HAS_FLAG(node->flags, FLAG_FISH_ON)) {
                CLEAR_FLAG(node->flags, FLAG_FISH_ON);
                node->fish_on_time = 0;
                // Update alert history
                for (int j = 0; j < ALERT_HISTORY_SIZE; j++) {
                  if (alertHistory[j].nodeId == node->node_id && alertHistory[j].active) {
                    alertHistory[j].active = false;
                  }
                }
              }
            }
            // Update tracking but don't set online
            // NOTE: Do NOT clear via_espnow - preserve direct path indicator
            node->via_lora = true;
          }
          // Skip updateNodeState - would set online=true
          continue;
        }

        // Node is online - process normally with deduplication
        // BUG FIX #2: No need to clear FLAG_NODE_OFFLINE from flags anymore

        // NOTE: FIX A (ESP-NOW authority block) was removed to enable gateway-to-gateway sync.
        // Timeout logic now uses last_direct_seen to prevent echo from keeping dead nodes alive.
        // See Change D in implementation for details.

        // BUG FIX #17: Convert optimized fields back for processing
        // uptime_min -> uptime_sec (multiply by 60)
        // battery_pct -> battery_mv (use helper function)
        uint32_t uptimeSec = (uint32_t)ns->uptime_min * 60;
        uint16_t batteryMv = batteryPercentToMv(ns->battery_pct);

        if (shouldAcceptMessage(ns->node_id, ns->sequence, uptimeSec, ns->flags)) {
          updateNodeState(ns->node_id, ns->flags, batteryMv,
                          ns->sequence, uptimeSec, rssi);
          // Track that we received via LoRa
          // NOTE: Do NOT clear via_espnow - it indicates we CAN receive directly
          // If we clear it, FIX B would incorrectly skip this node in our aggregate
          NodeState* node = findOrCreateNode(ns->node_id);
          if (node) {
            node->via_lora = true;
            // via_espnow is preserved - will be refreshed on next direct message

            // BUG FIX #1: Reconstruct fish_on_time from elapsed seconds
            // This synchronizes hold timers across gateways (within LoRa latency)
            if (HAS_FLAG(ns->flags, FLAG_FISH_ON) && ns->fish_on_elapsed_sec > 0) {
              // Reconstruct when FISH_ON started relative to local millis()
              uint32_t reconstructedFishOnTime = millis() - ((uint32_t)ns->fish_on_elapsed_sec * 1000);
              // Only update if node's fish_on_time isn't already set or differs significantly
              if (node->fish_on_time == 0) {
                node->fish_on_time = reconstructedFishOnTime;
                DEBUG_PRINTF("Node %d: Reconstructed fish_on_time from elapsed %u sec\n",
                             ns->node_id, ns->fish_on_elapsed_sec);
              }
            }

            // BUG FIX #1: Trust sender's hold_expired flag for clearing FISH_ON
            // If the authoritative gateway says hold time has expired, we trust it
            if (HAS_FLAG(ns->agg_flags, AGG_FLAG_HOLD_EXPIRED)) {
              DEBUG_PRINTF("Node %d: Hold time expired (reported by sender)\n", ns->node_id);
              // Mark our local fish_on_time as very old so canClearFishOn() will return true
              if (node->fish_on_time > 0) {
                // Set fish_on_time to a value that guarantees hold time has passed
                node->fish_on_time = millis() - (settings.alertHoldSec * 1000UL) - 1000;
              }
            }
          }
        }
      }

      // Relay if allowed (and not already relayed)
      // All LoRa-capable nodes relay aggregates for true mesh operation
      // This enables redundant paths: GW1 → GW2 → Offshore when GW1 can't reach directly
      if (msg->hop_count < LORA_MAX_HOPS) {
        // ISSUE 1 FIX: dedup key = (origin hub, lora_sequence). The old key
        // (network_id, network_id<<8 | lora_sequence) was the same for every hub, so a
        // hub refused to relay another hub's aggregate whenever their 8-bit counters matched.
        // sender_id is NOT overwritten any more: it stays the origin hub.
        if (!isAggregateDuplicate(msg->sender_id, msg->lora_sequence)) {
          recordAggregateForDedup(msg->sender_id, msg->lora_sequence);
          msg->hop_count++;
          // Recalculate checksum after modification
          msg->checksum = calculateChecksum((uint8_t*)msg, sizeof(LoRaAggregateMessage) - 1);

          // BUG FIX #5: Reduced max delay (50ms instead of 200ms)
          delay(random(20, 50));
          // Use helper that waits for BUSY and returns to RX mode
          loRaTransmitWithWait((uint8_t*)msg, sizeof(LoRaAggregateMessage));
          DEBUG_PRINTF("LoRa: Relayed aggregate, hop=%d\n", msg->hop_count);
        }
      }
      break;
    }

    case MSG_LORA_ALERT: {
      if (len < (int)sizeof(LoRaAlertMessage)) {
        DEBUG_PRINTF("LoRa ALERT too short: %d < %d\n", len, (int)sizeof(LoRaAlertMessage));
        loraRxErrors++;
        break;
      }

      LoRaAlertMessage* msg = (LoRaAlertMessage*)data;
      DEBUG_PRINTF("LoRa ALERT from %d (origin %d) seq=%d hop=%d uptime=%lu\n",
                   msg->sender_id, msg->origin_id, msg->sequence, msg->hop_count, msg->uptime_sec);

      // BUG FIX #4: Use actual uptime from message instead of 0
      if (shouldAcceptMessage(msg->origin_id, msg->sequence, msg->uptime_sec, msg->flags)) {
        updateNodeState(msg->origin_id, msg->flags, msg->battery_mv,
                        msg->sequence, msg->uptime_sec, rssi);
        // Track that this message came via LoRa
        // NOTE: Do NOT clear via_espnow - it indicates we CAN receive directly
        NodeState* node = findOrCreateNode(msg->origin_id);
        if (node) {
          node->via_lora = true;
          // via_espnow is preserved - will be refreshed on next direct message
        }

        // Sound buzzer on gateways
        if (currentRole == ROLE_GATEWAY_ONSHORE || currentRole == ROLE_GATEWAY_OFFSHORE) {
          triggerBuzzer(3);
        }
      }

      // Relay if allowed
      if ((currentRole == ROLE_RELAY_LORA || currentRole == ROLE_SENSOR_LORA) &&
          msg->hop_count < LORA_MAX_HOPS) {
        // Use origin_id for dedup (correct - already fixed previously)
        if (!isDuplicateForRelay(msg->origin_id, msg->sequence)) {
          recordForDedup(msg->origin_id, msg->sequence);
          msg->hop_count++;
          msg->sender_id = NODE_ID;
          // BUG FIX #5: Reduced max delay
          delay(random(10, 30));
          // Use helper that waits for BUSY and returns to RX mode
          loRaTransmitWithWait((uint8_t*)msg, sizeof(LoRaAlertMessage));
          DEBUG_PRINTF("LoRa: Relayed alert, hop=%d\n", msg->hop_count);
        }
      }
      break;
    }

    case MSG_SILENCE_SYNC: {
      if (len < (int)sizeof(LoRaSilenceSyncMessage)) {
        DEBUG_PRINTF("LoRa SILENCE too short: %d < %d\n", len, (int)sizeof(LoRaSilenceSyncMessage));
        loraRxErrors++;
        break;
      }

      LoRaSilenceSyncMessage* msg = (LoRaSilenceSyncMessage*)data;

      DEBUG_PRINTF("LoRa SILENCE from %d (origin %d): %s, hop=%d\n",
                   msg->sender_id, msg->origin_id,
                   msg->silence_state ? "ON" : "OFF", msg->hop_count);

      // Apply silence state
      alertsSilenced = (msg->silence_state == 1);
      if (alertsSilenced) {
        silenceTime = millis();
        // BUG FIX #9: Use elapsed time calculation instead of absolute timestamps
        // Convert expire_time to remaining seconds, then calculate local expire
        if (msg->expire_time > msg->timestamp) {
          uint32_t remainingAtSend = msg->expire_time - msg->timestamp;
          // M4 FIX: Sanity check - max 1 hour silence duration
          if (remainingAtSend > 3600) remainingAtSend = 3600;
          silenceExpireTime = millis() + (remainingAtSend * 1000);
          DEBUG_PRINTF("Silence will expire in %lu seconds\n", remainingAtSend);
        } else {
          silenceExpireTime = millis() + SILENCE_AUTO_CLEAR_MS;
        }
      } else {
        silenceTime = 0;
        silenceExpireTime = 0;
      }

      // Relay via ESP-NOW to local sensor nodes
      if (espNowReady && (currentRole == ROLE_GATEWAY_ONSHORE ||
                          currentRole == ROLE_GATEWAY_OFFSHORE)) {
        SilenceSyncMessage espMsg;
        espMsg.network_id = NETWORK_ID;
        espMsg.sender_id = NODE_ID;
        espMsg.msg_type = MSG_SILENCE_SYNC;
        espMsg.silence_state = msg->silence_state;
        espMsg.timestamp = msg->timestamp;
        espMsg.expire_time = msg->expire_time;

        esp_now_send(ESPNOW_BROADCAST, (uint8_t*)&espMsg, sizeof(espMsg));
        DEBUG_PRINTLN("ESP-NOW: Relayed silence sync to local nodes");
      }

      // Relay via LoRa if within hop limit (for multi-gateway setups)
      if (msg->hop_count < LORA_MAX_HOPS) {
        // Dedup check
        if (!isDuplicateForRelay(msg->origin_id, msg->timestamp)) {
          recordForDedup(msg->origin_id, msg->timestamp);

          msg->hop_count++;
          msg->sender_id = NODE_ID;
          // BUG FIX #5: Reduced max delay
          delay(random(20, 50));
          // Use helper that waits for BUSY and returns to RX mode
          loRaTransmitWithWait((uint8_t*)msg, sizeof(LoRaSilenceSyncMessage));
          DEBUG_PRINTF("LoRa: Relayed silence sync, hop=%d\n", msg->hop_count);
        }
      }
      break;
    }

    case MSG_RESET_CMD: {
      if (len < (int)sizeof(ResetCmdMessage)) {
        DEBUG_PRINTF("LoRa RESET too short: %d < %d\n", len, (int)sizeof(ResetCmdMessage));
        loraRxErrors++;
        break;
      }

      ResetCmdMessage* msg = (ResetCmdMessage*)data;
      DEBUG_PRINTF("LoRa RESET CMD from %d, delay=%d sec\n",
                   msg->sender_id, msg->reset_delay_sec);

      // Show reset message on display
      display.clearBuffer();
      display.setFont(u8g2_font_6x10_tr);
      display.drawStr(30, 30, "RESET CMD");
      display.drawStr(25, 45, "Rebooting...");
      display.sendBuffer();

      // Apply delay based on role
      uint8_t delayToUse = msg->reset_delay_sec;
      // Override with role-based delay
      switch (currentRole) {
        case ROLE_SENSOR_LORA: delayToUse = 0; break;
        case ROLE_RELAY_LORA: delayToUse = 1; break;
        case ROLE_GATEWAY_OFFSHORE: delayToUse = 2; break;
        case ROLE_GATEWAY_ONSHORE: delayToUse = 3; break;
        default: break;
      }

      if (delayToUse > 0) {
        DEBUG_PRINTF("Waiting %d seconds before reset...\n", delayToUse);
        delay(delayToUse * 1000);
      }

      DEBUG_PRINTLN(F("Executing reset..."));
      Serial.flush();
      delay(100);
      ESP.restart();
      break;
    }

    case MSG_CONFIG_UPDATE: {
      if (len < sizeof(ConfigUpdateMessage)) {
        DEBUG_PRINTLN(F("LoRa: Config update too short"));
        break;
      }

      ConfigUpdateMessage* cfg = (ConfigUpdateMessage*)data;

      // Verify checksum
      uint8_t expectedChecksum = calculateChecksum((uint8_t*)cfg, sizeof(ConfigUpdateMessage) - 1);
      if (cfg->checksum != expectedChecksum) {
        DEBUG_PRINTLN(F("LoRa: Config update checksum failed"));
        loraChecksumFails++;
        break;
      }

      // Check for duplicate (prevents relay loops)
      if (isConfigDuplicate(cfg->origin_id, cfg->config_seq)) {
        DEBUG_PRINTF("LoRa: Config update duplicate, origin=%d seq=%d\n",
                     cfg->origin_id, cfg->config_seq);
        break;
      }

      // Record for dedup BEFORE processing/relaying
      recordConfigForDedup(cfg->origin_id, cfg->config_seq);

      DEBUG_PRINTF("LoRa: Config update from origin=%d, target=%d, hop=%d, seq=%d\n",
                   cfg->origin_id, cfg->target_id, cfg->hop_count, cfg->config_seq);

      // Check if this config is for us
      bool forUs = (cfg->target_id == NODE_ID) || (cfg->target_id == 0);

      if (forUs) {
        DEBUG_PRINTLN(F("LoRa: Applying config update..."));

        // Apply settings
        settings.buzzerEnabled = cfg->buzzerEnabled != 0;
        settings.alertHoldSec = constrain(cfg->alertHoldSec, 5, 300);
        settings.heartbeatSec = constrain(cfg->heartbeatSec, 10, 600);
        settings.reedActiveHigh = cfg->reedActiveHigh != 0;

        // Save to NVS
        saveSettings();

        DEBUG_PRINTF("LoRa: Config applied - buzzer=%d, alertHold=%d, heartbeat=%d, reedHigh=%d\n",
                     settings.buzzerEnabled, settings.alertHoldSec,
                     settings.heartbeatSec, settings.reedActiveHigh);

        // Send ACK back toward origin
        ConfigAckMessage ack;
        ack.network_id = NETWORK_ID;
        ack.sender_id = NODE_ID;
        ack.msg_type = MSG_CONFIG_ACK;
        ack.hop_count = 0;
        ack.origin_id = cfg->origin_id;
        ack.config_seq = cfg->config_seq;
        ack.success = 1;
        ack.target_id = NODE_ID;
        memset(ack.reserved, 0, sizeof(ack.reserved));
        ack.checksum = calculateChecksum((uint8_t*)&ack, sizeof(ack) - 1);

        // Small delay before ACK to avoid collision
        delay(50 + random(100));

        loRaTransmitPacket((uint8_t*)&ack, sizeof(ack), "CONFIG_ACK");
        DEBUG_PRINTF("LoRa: Config ACK sent to origin=%d\n", cfg->origin_id);
      }

      // Relay if not at max hops AND (not for us specifically OR target is broadcast)
      bool shouldRelay = (cfg->hop_count < LORA_MAX_HOPS) &&
                         (cfg->target_id != NODE_ID || cfg->target_id == 0);

      if (shouldRelay) {
        DEBUG_PRINTF("LoRa: Relaying config update, hop %d -> %d\n",
                     cfg->hop_count, cfg->hop_count + 1);

        // Copy and increment hop count
        ConfigUpdateMessage relay = *cfg;
        relay.hop_count++;
        relay.checksum = calculateChecksum((uint8_t*)&relay, sizeof(relay) - 1);

        // Random delay to avoid collisions with other relays
        delay(100 + random(200));

        loRaTransmitPacket((uint8_t*)&relay, sizeof(relay), "CONFIG_RELAY");
      }

      break;
    }

    case MSG_CONFIG_ACK: {
      if (len < sizeof(ConfigAckMessage)) {
        DEBUG_PRINTLN(F("LoRa: Config ACK too short"));
        break;
      }

      ConfigAckMessage* ack = (ConfigAckMessage*)data;

      // Verify checksum
      uint8_t expectedChecksum = calculateChecksum((uint8_t*)ack, sizeof(ConfigAckMessage) - 1);
      if (ack->checksum != expectedChecksum) {
        DEBUG_PRINTLN(F("LoRa: Config ACK checksum failed"));
        loraChecksumFails++;
        break;
      }

      DEBUG_PRINTF("LoRa: Config ACK from node=%d, origin=%d, seq=%d, hop=%d\n",
                   ack->sender_id, ack->origin_id, ack->config_seq, ack->hop_count);

      // Check if this ACK is for us (we're the origin)
      if (ack->origin_id == NODE_ID) {
        // Check if this matches our pending config
        if (pendingConfigWaiting && ack->config_seq == pendingConfigSeq) {
          pendingConfigWaiting = false;
          DEBUG_PRINTF("LoRa: Config ACK received! Node %d applied config, success=%d\n",
                       ack->sender_id, ack->success);

          // Could trigger UI feedback here (beep, display message, etc.)
        }
      } else {
        // Not for us - relay toward origin if under hop limit
        if (ack->hop_count < LORA_MAX_HOPS) {
          DEBUG_PRINTF("LoRa: Relaying config ACK toward origin=%d, hop %d -> %d\n",
                       ack->origin_id, ack->hop_count, ack->hop_count + 1);

          // Copy and increment hop count
          ConfigAckMessage relay = *ack;
          relay.hop_count++;
          relay.checksum = calculateChecksum((uint8_t*)&relay, sizeof(relay) - 1);

          // Random delay
          delay(50 + random(100));

          loRaTransmitPacket((uint8_t*)&relay, sizeof(relay), "ACK_RELAY");
        }
      }

      break;
    }
  }
}

void sendLoRaAggregate() {
  if (!loraReady) return;

  // FIX: Rotation for >10 nodes - static offset cycles through all nodes
  // LoRa packet can only hold 10 nodes, but network supports up to 16 (MAX_NODES)
  static uint8_t nodeOffset = 0;

  LoRaAggregateMessage msg;
  memset(&msg, 0, sizeof(msg));

  msg.network_id = NETWORK_ID;
  msg.sender_id = NODE_ID;
  msg.msg_type = MSG_LORA_AGGREGATE;
  msg.hop_count = 0;
  msg.lora_sequence = txSequence++;
  msg.sender_role = currentRole;  // Include role so receivers know gateway vs sensor

  // Ensure our own state is current before sending
  network.nodes[0].last_seen = millis();
  network.nodes[0].last_uptime = millis() / 1000;

  // FIX: Pack nodes intelligently to handle >10 nodes
  // Priority: Always include self (node 0), then rotate through others
  int packedCount = 0;

  // Always send self (gateway/node 0) in slot 0
  if (network.node_count > 0) {
    msg.nodes[packedCount].node_id = network.nodes[0].node_id;
    msg.nodes[packedCount].flags = network.nodes[0].flags;
    msg.nodes[packedCount].agg_flags = 0;  // BUG FIX #2: Self is always online
    // BUG FIX #17: Convert battery mV to percentage, uptime sec to minutes
    msg.nodes[packedCount].battery_pct = batteryMvToPercent(network.nodes[0].battery_mv);
    msg.nodes[packedCount].sequence = network.nodes[0].last_seq;
    msg.nodes[packedCount].uptime_min = (uint16_t)(network.nodes[0].last_uptime / 60);

    // BUG FIX #1: Include elapsed FISH_ON time for hold timer sync
    if (HAS_FLAG(network.nodes[0].flags, FLAG_FISH_ON) && network.nodes[0].fish_on_time > 0) {
      uint32_t elapsedMs = millis() - network.nodes[0].fish_on_time;
      msg.nodes[packedCount].fish_on_elapsed_sec = (uint16_t)(elapsedMs / 1000);
      // BUG FIX #1: Set hold expired flag if our local hold time has passed
      if (canClearFishOn(network.nodes[0].node_id)) {
        SET_FLAG(msg.nodes[packedCount].agg_flags, AGG_FLAG_HOLD_EXPIRED);
      }
    } else {
      msg.nodes[packedCount].fish_on_elapsed_sec = 0;
    }
    packedCount++;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // FIX B - SENDER SIDE: FILTER OUT ECHOED NODES
  // ═══════════════════════════════════════════════════════════════════════════
  // Only include nodes where THIS gateway is the authoritative source.
  // This prevents the feedback loop where the offshore gateway echoes back
  // sensor data it learned from the onshore gateway.
  //
  // Rules for including a node in the aggregate:
  // 1. Self (node 0) - always included
  // 2. Nodes tracked via ESP-NOW (via_espnow == true) - we're authoritative
  // 3. Skip nodes only learned via LoRa (via_lora == true && via_espnow == false)
  //
  // This is defense-in-depth alongside Fix A (receiver-side check).
  // ═══════════════════════════════════════════════════════════════════════════

  // Fill remaining 9 slots with authoritative nodes only
  if (network.node_count > 1) {
    int slotsAvailable = 9;  // Max 9 more slots after self
    int nodesScanned = 0;
    int nodesRemaining = network.node_count - 1;  // Exclude node 0

    // Scan through nodes, skipping those we only know via LoRa
    for (int i = 0; i < nodesRemaining && packedCount < 10; i++) {
      // Calculate which node index to check (skip node 0, rotate through)
      int nodeIndex = 1 + ((nodeOffset + i) % nodesRemaining);
      nodesScanned++;

      NodeState* node = &network.nodes[nodeIndex];

      // FIX B: Skip nodes that we only know via LoRa (not authoritative)
      // These were learned from another gateway - don't echo them back
      if (node->via_lora && !node->via_espnow) {
        DEBUG_PRINTF("Skipping node %d in aggregate (learned via LoRa, not authoritative)\n",
                     node->node_id);
        continue;  // Skip this node - we're not the authoritative source
      }

      // Include this node - we're authoritative (via ESP-NOW or it's our own role)
      msg.nodes[packedCount].node_id = node->node_id;
      msg.nodes[packedCount].flags = node->flags;
      msg.nodes[packedCount].agg_flags = 0;  // BUG FIX #2: Initialize agg_flags
      // BUG FIX #17: Convert battery mV to percentage, uptime sec to minutes
      msg.nodes[packedCount].battery_pct = batteryMvToPercent(node->battery_mv);
      msg.nodes[packedCount].sequence = node->last_seq;
      msg.nodes[packedCount].uptime_min = (uint16_t)(node->last_uptime / 60);

      // BUG FIX #2: Set AGG_FLAG_NODE_OFFLINE in agg_flags (not flags)
      // This propagates offline status to remote gateways without colliding with FLAG_CONFIG_MODE
      if (!node->online) {
        SET_FLAG(msg.nodes[packedCount].agg_flags, AGG_FLAG_NODE_OFFLINE);
        DEBUG_PRINTF("Including offline node %d with OFFLINE flag\n", node->node_id);
      }

      // BUG FIX #1: Include elapsed FISH_ON time for hold timer sync
      if (HAS_FLAG(node->flags, FLAG_FISH_ON) && node->fish_on_time > 0) {
        uint32_t elapsedMs = millis() - node->fish_on_time;
        msg.nodes[packedCount].fish_on_elapsed_sec = (uint16_t)(elapsedMs / 1000);
        // BUG FIX #1: Set hold expired flag if our local hold time has passed
        if (canClearFishOn(node->node_id)) {
          SET_FLAG(msg.nodes[packedCount].agg_flags, AGG_FLAG_HOLD_EXPIRED);
        }
      } else {
        msg.nodes[packedCount].fish_on_elapsed_sec = 0;
      }

      packedCount++;
    }

    // Advance offset for next transmission (rotate through all nodes)
    nodeOffset = (nodeOffset + nodesScanned) % max(1, nodesRemaining);
  }

  msg.node_count = packedCount;

  DEBUG_PRINTF("LoRa TX AGG: %d/%d nodes (offset=%d), self_seq=%d\n",
               packedCount, network.node_count, nodeOffset, msg.nodes[0].sequence);

  msg.checksum = calculateChecksum((uint8_t*)&msg, sizeof(msg) - 1);

  // Record own transmission (defence in depth: echoes of our own aggregate are
  // already dropped on receive because sender_id == NODE_ID)
  recordAggregateForDedup(NODE_ID, msg.lora_sequence);

  // Use unified transmit helper with proper TX-complete interrupt handling
  loRaTransmitPacket((uint8_t*)&msg, sizeof(msg), "AGG");
}

// ═══════════════════════════════════════════════════════════════════════════
// SEND LORA AGGREGATE WITH RETRY - For reliable status change propagation
// ═══════════════════════════════════════════════════════════════════════════
// Sends aggregate multiple times to ensure offshore gateway receives
// status changes even if some packets are lost.

void sendLoRaAggregateWithRetry(int retries) {
  if (!loraReady) return;

  DEBUG_PRINTF("Sending LoRa aggregate with %d retries for status change\n", retries);

  for (int i = 0; i < retries; i++) {
    sendLoRaAggregate();

    if (i < retries - 1) {
      // Random delay between retries to avoid synchronized collisions
      // P4 NOTE: Future optimization - could use adaptive timing based on
      // channel activity/RSSI to reduce collisions and improve throughput
      int delayMs = 150 + random(100);
      DEBUG_PRINTF("Retry %d/%d - waiting %dms\n", i + 1, retries, delayMs);
      delay(delayMs);
    }
  }

  // Update last TX time to prevent immediate scheduled aggregate
  lastLoRaTx = millis();

  DEBUG_PRINTLN("Status change aggregate complete");
}

void relayLoRaAlert(uint8_t originId, uint8_t flags, uint16_t batteryMv, uint16_t seq) {
  if (!loraReady) return;

  // BUG FIX #4: Find originating node to get its uptime
  uint32_t originUptime = 0;
  NodeState* originNode = findOrCreateNode(originId);
  if (originNode) {
    originUptime = originNode->last_uptime;
  }

  LoRaAlertMessage msg;
  msg.network_id = NETWORK_ID;
  msg.sender_id = NODE_ID;
  msg.msg_type = MSG_LORA_ALERT;
  msg.hop_count = 0;
  msg.origin_id = originId;
  msg.flags = flags;
  msg.battery_mv = batteryMv;
  msg.sequence = seq;
  msg.uptime_sec = originUptime;  // BUG FIX #4: Include actual uptime
  msg.alert_time = millis() / 1000;

  DEBUG_PRINTF("LoRa TX ALERT for node %d seq=%d uptime=%lu\n", originId, seq, originUptime);

  // Use unified transmit helper with proper TX-complete interrupt handling
  loRaTransmitPacket((uint8_t*)&msg, sizeof(msg), "ALERT");
}

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW
// ═══════════════════════════════════════════════════════════════════════════

void setupEspNow() {
  DEBUG_PRINTLN(F("Initializing ESP-NOW..."));

  // ═══════════════════════════════════════════════════════════════════════════
  // GATEWAY_ONSHORE: Use STA-only mode with Long Range protocol
  // Other roles: Use AP+STA for normal WiFi compatibility
  // ═══════════════════════════════════════════════════════════════════════════
  #if ESPNOW_LONG_RANGE_MODE
  if (currentRole == ROLE_GATEWAY_ONSHORE) {
    // STA-only mode for LR - no AP (phones can't connect, but that's intentional)
    // User accesses status from OFFSHORE gateway in cabin
    DEBUG_PRINTLN(F("GATEWAY_ONSHORE: Using STA-only mode for Long Range"));
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Set maximum TX power (84 = 21dBm max)
    esp_err_t txResult = esp_wifi_set_max_tx_power(84);
    if (txResult != ESP_OK) {
      DEBUG_PRINTF("WARNING: Failed to set TX power: %d\n", txResult);
    } else {
      int8_t actualPower;
      esp_wifi_get_max_tx_power(&actualPower);
      DEBUG_PRINTF("TX power set to: %.2f dBm\n", actualPower * 0.25);
    }

    // Enable Long Range protocol
    // WIFI_PROTOCOL_LR improves sensitivity from ~-72dBm to ~-98dBm
    esp_err_t lrResult = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    if (lrResult != ESP_OK) {
      DEBUG_PRINTF("WARNING: Failed to enable LR mode: %d\n", lrResult);
    } else {
      DEBUG_PRINTLN(F("Long Range (LR) mode enabled - sensitivity ~-98dBm"));
    }
  } else {
    // Other roles use standard AP+STA mode
    WiFi.mode(WIFI_AP_STA);
  }
  #else
  // LR mode disabled - all roles use standard AP+STA
  WiFi.mode(WIFI_AP_STA);
  #endif

  // Set ESP-NOW channel
  delay(100);  // Let WiFi mode stabilize
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(50);

  // Verify channel
  uint8_t primary;
  wifi_second_chan_t secondary;
  esp_wifi_get_channel(&primary, &secondary);
  DEBUG_PRINTF("WiFi channel: %d (expected: %d)\n", primary, ESPNOW_CHANNEL);

  if (primary != ESPNOW_CHANNEL) {
    DEBUG_PRINTLN(F("WARNING: Channel mismatch! Retrying..."));
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(50);
  }

  DEBUG_PRINTF("MAC: %s\n", WiFi.macAddress().c_str());

  if (esp_now_init() != ESP_OK) {
    DEBUG_PRINTLN(F("ESP-NOW init failed!"));
    return;
  }

  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  espNowReady = true;
  DEBUG_PRINTLN(F("ESP-NOW ready"));
}

// v2 (C9): the ESP-NOW callback runs in the Wi-Fi task. It only copies the frame into a
// lock-free ring; processEspNowMessage() runs from loop() (loopEspNowRx). This removes the
// v1 race where LoRa was driven from the Wi-Fi task while loop() used the radio too.
static icemesh::RxRing<16, 250> espNowRing;

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  espNowRing.push(mac, data, len, 0);   // core 2.x callback gives no RSSI
}

void loopEspNowRx() {
  static icemesh::RxRing<16, 250>::Frame f;   // static: 258 B kept off the loop stack
  uint8_t budget = 8;                          // bounded work per loop() pass
  while (budget-- && espNowRing.pop(f)) {
    processEspNowMessage(f.data, f.len, f.rssi);
  }
}

void processEspNowMessage(const uint8_t* data, int len, int rssi) {
  // Basic header validation
  if (len < 3) {
    DEBUG_PRINTF("ESP-NOW: Message too short (%d bytes, min 3)\n", len);
    return;
  }

  uint8_t netId = data[0];
  uint8_t nodeId = data[1];
  uint8_t msgType = data[2];

  if (netId != NETWORK_ID) return;
  if (nodeId == NODE_ID) return;

  switch (msgType) {
    case MSG_STATUS:
    case MSG_HEARTBEAT: {
      if (len < (int)sizeof(SensorMessage)) {
        DEBUG_PRINTF("ESP-NOW %s too short: %d < %d\n",
                     getMessageTypeName(msgType), len, (int)sizeof(SensorMessage));
        break;
      }

      SensorMessage* msg = (SensorMessage*)data;

      DEBUG_PRINTF("ESP-NOW [%s] Node %d: seq=%d flags=0x%02X batt=%dmV\n",
                   getMessageTypeName(msgType), msg->node_id, msg->sequence,
                   msg->flags, msg->battery_mv);

      // Get current state BEFORE update to detect changes
      NodeState* existingNode = findOrCreateNode(msg->node_id);
      bool wasFishOn = existingNode ? HAS_FLAG(existingNode->flags, FLAG_FISH_ON) : false;
      bool wasOnline = existingNode ? existingNode->online : false;

      // DEDUP: Check if we should accept this message
      if (shouldAcceptMessage(msg->node_id, msg->sequence, msg->uptime_sec, msg->flags)) {
        updateNodeState(msg->node_id, msg->flags, msg->battery_mv,
                        msg->sequence, msg->uptime_sec, rssi);

        // Track that this message came via ESP-NOW
        NodeState* node = findOrCreateNode(msg->node_id);
        if (node) {
          node->via_espnow = true;
          node->via_lora = false;
          node->last_direct_seen = millis();  // Track direct ESP-NOW timestamp
          node->received_direct = true;       // Mark as having direct path

          // Detect status changes that need immediate propagation
          bool nowFishOn = HAS_FLAG(node->flags, FLAG_FISH_ON);
          bool statusChanged = false;

          // FISH_ON state changed
          if (wasFishOn != nowFishOn) {
            DEBUG_PRINTF("Node %d: FISH_ON changed %d -> %d\n",
                         msg->node_id, wasFishOn, nowFishOn);
            statusChanged = true;
          }

          // Node came online (was offline or new)
          if (!wasOnline && node->online) {
            DEBUG_PRINTF("Node %d: Came online\n", msg->node_id);
            statusChanged = true;
          }

          // If status changed, send immediate aggregate with retry
          if (statusChanged && loraReady &&
              (currentRole == ROLE_GATEWAY_ONSHORE || currentRole == ROLE_GATEWAY_OFFSHORE)) {
            DEBUG_PRINTLN("Status change detected - sending immediate LoRa aggregate");
            sendLoRaAggregateWithRetry(3);
          }
        }
      } else {
        DEBUG_PRINTF("ESP-NOW: Rejected stale/dup from node %d seq=%d\n", msg->node_id, msg->sequence);
      }

      // RELAY: Check if we should relay via LoRa
      if (loraReady && (currentRole == ROLE_GATEWAY_ONSHORE ||
                        currentRole == ROLE_SENSOR_LORA ||
                        currentRole == ROLE_RELAY_LORA)) {
        if (!isDuplicateForRelay(msg->node_id, msg->sequence)) {
          recordForDedup(msg->node_id, msg->sequence);
          // Only relay alerts immediately, status goes in aggregate
        }
      }
      break;
    }
    
    case MSG_ALERT: {
      if (len < (int)sizeof(AlertMessage)) {
        DEBUG_PRINTF("ESP-NOW ALERT too short: %d < %d\n", len, (int)sizeof(AlertMessage));
        break;
      }

      AlertMessage* msg = (AlertMessage*)data;

      DEBUG_PRINTF("ESP-NOW ALERT! Node %d seq=%d\n", msg->node_id, msg->sequence);

      // DEDUP: Check if we should accept (FISH_ON always accepted)
      if (shouldAcceptMessage(msg->node_id, msg->sequence, msg->uptime_sec, msg->flags)) {
        updateNodeState(msg->node_id, msg->flags, msg->battery_mv,
                        msg->sequence, msg->uptime_sec, rssi);
        // Track that this message came via ESP-NOW
        NodeState* node = findOrCreateNode(msg->node_id);
        if (node) {
          node->via_espnow = true;
          node->via_lora = false;
          node->last_direct_seen = millis();  // Track direct ESP-NOW timestamp
          node->received_direct = true;       // Mark as having direct path
        }
        triggerBuzzer(3);  // Sound alert

        // BUG FIX #4: Send ACK back to the sensor node
        // This allows the sensor to exit its 30-second wait loop early and save battery
        if (espNowReady) {
          AckMessage ack;
          ack.network_id = NETWORK_ID;
          ack.node_id = NODE_ID;
          ack.msg_type = MSG_ACK;
          ack.ack_node_id = msg->node_id;
          ack.ack_msg_type = MSG_ALERT;
          ack.reserved = 0;
          esp_now_send(ESPNOW_BROADCAST, (uint8_t*)&ack, sizeof(ack));
          DEBUG_PRINTF("Sent ACK to node %d for ALERT\n", msg->node_id);
        }
      }

      // RELAY: Always relay alerts (dedup at receiver)
      if (loraReady && (currentRole == ROLE_GATEWAY_ONSHORE ||
                        currentRole == ROLE_SENSOR_LORA ||
                        currentRole == ROLE_RELAY_LORA)) {
        if (!isDuplicateForRelay(msg->node_id, msg->sequence)) {
          recordForDedup(msg->node_id, msg->sequence);
          relayLoRaAlert(msg->node_id, msg->flags, msg->battery_mv, msg->sequence);
        }
      }
      break;
    }

    case MSG_SILENCE_SYNC: {
      if (len < (int)sizeof(SilenceSyncMessage)) {
        DEBUG_PRINTF("ESP-NOW SILENCE too short: %d < %d\n", len, (int)sizeof(SilenceSyncMessage));
        break;
      }

      SilenceSyncMessage* msg = (SilenceSyncMessage*)data;

      DEBUG_PRINTF("ESP-NOW SILENCE from %d: %s\n",
                   msg->sender_id, msg->silence_state ? "ON" : "OFF");

      // Apply silence state
      alertsSilenced = (msg->silence_state == 1);
      if (alertsSilenced) {
        silenceTime = millis();
        // BUG FIX #9: Use elapsed time calculation instead of absolute timestamps
        if (msg->expire_time > msg->timestamp) {
          uint32_t remainingAtSend = msg->expire_time - msg->timestamp;
          // M4 FIX: Sanity check - max 1 hour silence duration
          if (remainingAtSend > 3600) remainingAtSend = 3600;
          silenceExpireTime = millis() + (remainingAtSend * 1000);
          DEBUG_PRINTF("Silence will expire in %lu seconds\n", remainingAtSend);
        } else {
          silenceExpireTime = millis() + SILENCE_AUTO_CLEAR_MS;
        }
      } else {
        silenceTime = 0;
        silenceExpireTime = 0;
      }

      // Relay via LoRa if we're a gateway (so remote gateway gets it)
      if (loraReady && (currentRole == ROLE_GATEWAY_ONSHORE ||
                        currentRole == ROLE_GATEWAY_OFFSHORE)) {
        LoRaSilenceSyncMessage loraMsg;
        loraMsg.network_id = NETWORK_ID;
        loraMsg.sender_id = NODE_ID;
        loraMsg.msg_type = MSG_SILENCE_SYNC;
        loraMsg.hop_count = 0;
        loraMsg.silence_state = msg->silence_state;
        loraMsg.origin_id = msg->sender_id;
        loraMsg.timestamp = msg->timestamp;
        loraMsg.expire_time = msg->expire_time;

        // Use helper that waits for BUSY and returns to RX mode
        loRaTransmitWithWait((uint8_t*)&loraMsg, sizeof(loraMsg));
        DEBUG_PRINTLN("LoRa: Relayed silence sync");
      }
      break;
    }

    case MSG_RESET_CMD: {
      if (len < (int)sizeof(ResetCmdMessage)) {
        DEBUG_PRINTF("ESP-NOW RESET too short: %d < %d\n", len, (int)sizeof(ResetCmdMessage));
        break;
      }

      ResetCmdMessage* msg = (ResetCmdMessage*)data;
      DEBUG_PRINTF("ESP-NOW RESET CMD from %d, delay=%d sec\n",
                   msg->sender_id, msg->reset_delay_sec);

      // Show reset message on display - use showOverlayMessage style
      display.clearBuffer();
      display.setFont(u8g2_font_6x10_tr);
      display.drawStr(30, 30, "RESET CMD");
      display.drawStr(25, 45, "Rebooting...");
      display.sendBuffer();

      // Apply delay based on role
      uint8_t delayToUse = msg->reset_delay_sec;
      // Override with role-based delay if not from gateway
      switch (currentRole) {
        case ROLE_SENSOR_LORA: delayToUse = 0; break;
        case ROLE_RELAY_LORA: delayToUse = 1; break;
        case ROLE_GATEWAY_OFFSHORE: delayToUse = 2; break;
        case ROLE_GATEWAY_ONSHORE: delayToUse = 3; break;
        default: break;
      }

      if (delayToUse > 0) {
        DEBUG_PRINTF("Waiting %d seconds before reset...\n", delayToUse);
        delay(delayToUse * 1000);
      }

      DEBUG_PRINTLN(F("Executing reset..."));
      Serial.flush();
      delay(100);
      ESP.restart();
      break;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// NODE STATE WITH DEDUPLICATION
// ═══════════════════════════════════════════════════════════════════════════

// Find or create a node entry
NodeState* findOrCreateNode(uint8_t nodeId) {
  // Find existing
  for (int i = 0; i < network.node_count; i++) {
    if (network.nodes[i].node_id == nodeId) {
      return &network.nodes[i];
    }
  }
  
  // Create new if space available
  if (network.node_count >= MAX_NODES) {
    DEBUG_PRINTLN(F("Max nodes reached!"));
    return nullptr;
  }
  
  int idx = network.node_count++;
  NodeState* node = &network.nodes[idx];
  memset(node, 0, sizeof(NodeState));
  node->node_id = nodeId;
  node->initialized = false;
  node->received_direct = false;  // Not yet heard via ESP-NOW
  node->last_direct_seen = 0;     // No direct contact yet
  snprintf(node->name, sizeof(node->name), "Node %d", nodeId);
  return node;
}

// Core deduplication logic - ASYMMETRIC: FISH_ON is sacred
bool shouldAcceptMessage(uint8_t nodeId, uint16_t seq, uint32_t uptime, uint8_t newFlags) {
  NodeState* node = findOrCreateNode(nodeId);
  if (!node) return false;
  
  bool incomingFishOn = HAS_FLAG(newFlags, FLAG_FISH_ON);
  bool currentFishOn = HAS_FLAG(node->flags, FLAG_FISH_ON);
  
  // First message from this node - always accept
  if (!node->initialized) {
    DEBUG_PRINTF("Node %d: First message, seq=%d\n", nodeId, seq);
    node->initialized = true;
    node->last_seq = seq;
    node->last_uptime = uptime;
    return true;
  }
  
  // RULE 1: FISH_ON always accepted (never miss an alert!)
  if (incomingFishOn && !currentFishOn) {
    DEBUG_PRINTF("Node %d: FISH_ON! seq=%d (accepting regardless)\n", nodeId, seq);
    node->last_seq = max(node->last_seq, seq);
    node->last_uptime = uptime;
    node->clear_confirm_count = 0;  // Reset clear counter on new alert
    return true;
  }
  
  // RULE 2: Refresh of existing FISH_ON - accept if seq >= last
  if (incomingFishOn && currentFishOn) {
    // Use wraparound-safe comparison
    if (isSequenceNewer(seq, node->last_seq) || seq == node->last_seq) {
      node->last_seq = seq;
      node->last_uptime = uptime;
      return true;
    }
    // Stale fish_on refresh - ignore but don't log as error
    return false;
  }
  
  // RULE 3: OK trying to clear FISH_ON - must have HIGHER sequence
  // Trust the clear if: hold time elapsed OR multiple consecutive confirmations
  if (!incomingFishOn && currentFishOn) {
    // BUG FIX #18: Check for reboot FIRST (low seq + low uptime)
    // Without this, a rebooted sensor can never clear FISH_ON because
    // its new low sequence is rejected as "stale"
    if (!isSequenceNewer(seq, node->last_seq) && uptime < node->last_uptime) {
      DEBUG_PRINTF("Node %d: Detected reboot during FISH_ON clear, seq=%d uptime=%lu\n",
                   nodeId, seq, uptime);
      node->last_seq = seq;
      node->last_uptime = uptime;
      node->clear_confirm_count = 0;  // Reset for clean state
      return true;  // Accept reboot - this clears FISH_ON
    }

    // Use wraparound-safe comparison
    if (isSequenceNewer(seq, node->last_seq)) {
      bool holdTimeElapsed = canClearFishOn(nodeId);

      if (holdTimeElapsed) {
        // Hold time elapsed - clear immediately
        DEBUG_PRINTF("Node %d: Clearing FISH_ON (hold time elapsed), seq=%d\n", nodeId, seq);
        node->clear_confirm_count = 0;
        node->last_seq = seq;
        node->last_uptime = uptime;
        return true;
      }

      // Hold time not elapsed - count consecutive clear confirmations
      node->clear_confirm_count++;
      DEBUG_PRINTF("Node %d: Clear attempt %d/2 (hold time not elapsed), seq=%d\n",
                   nodeId, node->clear_confirm_count, seq);

      // After 2 consecutive clear confirmations, trust the sensor
      // This means the flag has been down for at least one full heartbeat cycle (5+ seconds)
      if (node->clear_confirm_count >= 2) {
        DEBUG_PRINTF("Node %d: Clearing FISH_ON (2 consecutive confirmations)\n", nodeId);
        node->clear_confirm_count = 0;
        node->last_seq = seq;
        node->last_uptime = uptime;
        return true;
      }

      // BUG FIX #18: First clear attempt - DON'T update last_seq yet
      // If we update last_seq but return false (not accepting), the aggregate will
      // contain the NEW sequence but OLD flags (FISH_ON still set). When offshore
      // receives this, it will update its stored sequence, making subsequent REAL
      // clears appear "stale" and keeping FISH_ON stuck.
      //
      // By NOT updating last_seq here, the aggregate continues to send the old
      // sequence with FISH_ON=true (consistent state). On second confirmation,
      // we update last_seq and clear the flag atomically.
      //
      // Note: We DO update last_uptime to track timing for reboot detection.
      node->last_uptime = uptime;
      DEBUG_PRINTF("Node %d: Clear attempt %d/2 - waiting for confirmation\n",
                   nodeId, node->clear_confirm_count);
      return false;
    }
    // Stale OK - reject, keep FISH_ON
    DEBUG_PRINTF("Node %d: Stale OK seq=%d <= %d, keeping FISH_ON\n", nodeId, seq, node->last_seq);
    return false;
  }
  
  // RULE 4: OK to OK - standard sequence check
  if (!incomingFishOn && !currentFishOn) {
    // Explicit reboot flag - always accept and reset tracking
    if (HAS_FLAG(newFlags, FLAG_FIRST_BOOT)) {
      DEBUG_PRINTF("Node %d: FIRST_BOOT flag detected, resetting sequence\n", nodeId);
      node->last_seq = seq;
      node->last_uptime = uptime;
      return true;
    }

    // Check for reboot (low seq + low uptime)
    if (!isSequenceNewer(seq, node->last_seq) && uptime < node->last_uptime) {
      DEBUG_PRINTF("Node %d: Detected reboot, seq=%d uptime=%d\n", nodeId, seq, uptime);
      node->last_seq = seq;
      node->last_uptime = uptime;
      return true;
    }

    // Use wraparound-safe comparison
    if (isSequenceNewer(seq, node->last_seq)) {
      node->last_seq = seq;
      node->last_uptime = uptime;
      return true;
    }

    // FIX: If sequence is the SAME, it is a valid "Keep Alive" heartbeat.
    // We want to return true so updateNodeState() runs and refreshes last_seen.
    // This allows the 5-second LoRa transmit to refresh connectivity status
    // even when the sequence only increments every 60 seconds.
    if (seq == node->last_seq) {
      // Do not update last_uptime/last_seq here,
      // but returning true allows the caller to update rssi and last_seen.
      return true;
    }

    // Lower sequence - truly stale
    return false;
  }
  
  return false;
}

// Check if FISH_ON minimum hold time has elapsed
bool canClearFishOn(uint8_t nodeId) {
  for (int i = 0; i < network.node_count; i++) {
    if (network.nodes[i].node_id == nodeId) {
      if (network.nodes[i].fish_on_time == 0) return true;
      return (millis() - network.nodes[i].fish_on_time) > (settings.alertHoldSec * 1000UL);
    }
  }
  return true;
}

// Relay deduplication - short term cache
bool isDuplicateForRelay(uint8_t nodeId, uint16_t seq) {
  uint32_t now = millis();
  
  for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
    if (dedupeCache[i].node_id == nodeId && 
        dedupeCache[i].sequence == seq &&
        (now - dedupeCache[i].received_at) < DEDUP_WINDOW_MS) {
      return true;  // Already relayed recently
    }
  }
  return false;
}

void recordForDedup(uint8_t nodeId, uint16_t seq) {
  dedupeCache[dedupeCacheIdx].node_id = nodeId;
  dedupeCache[dedupeCacheIdx].sequence = seq;
  dedupeCache[dedupeCacheIdx].received_at = millis();
  dedupeCacheIdx = (dedupeCacheIdx + 1) % DEDUP_CACHE_SIZE;
}

// ISSUE 1 FIX: aggregate relay dedup, keyed by (origin hub, lora_sequence), 30 s window
bool isAggregateDuplicate(uint8_t originHub, uint8_t loraSeq) {
  uint32_t now = millis();
  for (int i = 0; i < AGG_DEDUP_CACHE_SIZE; i++) {
    if (aggDedupCache[i].received_at != 0 &&
        aggDedupCache[i].node_id == originHub &&
        aggDedupCache[i].sequence == loraSeq &&
        (now - aggDedupCache[i].received_at) < DEDUP_WINDOW_MS) {
      return true;
    }
  }
  return false;
}

void recordAggregateForDedup(uint8_t originHub, uint8_t loraSeq) {
  aggDedupCache[aggDedupCacheIdx].node_id = originHub;
  aggDedupCache[aggDedupCacheIdx].sequence = loraSeq;
  aggDedupCache[aggDedupCacheIdx].received_at = millis() | 1;  // never 0 (0 = empty slot)
  aggDedupCacheIdx = (aggDedupCacheIdx + 1) % AGG_DEDUP_CACHE_SIZE;
}

// Update node state (called only after shouldAcceptMessage returns true)
void updateNodeState(uint8_t nodeId, uint8_t flags, uint16_t batteryMv, uint16_t seq, uint32_t uptime, int8_t rssi) {
  NodeState* node = findOrCreateNode(nodeId);
  if (!node) return;

  bool wasFishOn = HAS_FLAG(node->flags, FLAG_FISH_ON);
  bool nowFishOn = HAS_FLAG(flags, FLAG_FISH_ON);
  
  // Track when FISH_ON started and record alert
  if (nowFishOn && !wasFishOn) {
    node->fish_on_time = millis();
    node->clear_confirm_count = 0;  // Reset clear counter on new alert
    recordAlert(nodeId);  // Add to alert history
    alertsSilenced = false;  // New alert clears silence
    registerActivity();  // Wake display on FISH_ON alert
    DEBUG_PRINTF("Node %d: FISH_ON started at %lu\n", nodeId, node->fish_on_time);
  } else if (!nowFishOn && wasFishOn) {
    node->fish_on_time = 0;
    // Mark alert as inactive in history
    for (int i = 0; i < ALERT_HISTORY_SIZE; i++) {
      if (alertHistory[i].nodeId == nodeId && alertHistory[i].active) {
        alertHistory[i].active = false;
      }
    }
    DEBUG_PRINTF("Node %d: FISH_ON cleared\n", nodeId);
  }
  
  node->flags = flags;
  node->battery_mv = batteryMv;
  node->last_seq = seq;
  node->last_uptime = uptime;
  node->rssi = rssi;
  node->last_seen = millis();

  // Wake display if node is coming online from offline state
  if (!node->online) {
    registerActivity();
  }

  node->online = true;
  node->initialized = true;
  
  // Update global alert state
  updateAlertState();
  
  network.last_update = millis();
}

void updateAlertState() {
  activeAlerts = false;
  for (int i = 0; i < network.node_count; i++) {
    if (network.nodes[i].online && HAS_FLAG(network.nodes[i].flags, FLAG_FISH_ON)) {
      activeAlerts = true;
      break;
    }
  }
}

void loopNodeTimeout() {
  unsigned long now = millis();

  // M1 FIX: Use minimal critical sections to protect state mutations
  // Avoid debug prints inside critical sections to prevent stack overflow

  for (int i = 0; i < network.node_count; i++) {
    // M1 FIX: Read volatile data in critical section
    portENTER_CRITICAL(&networkMux);
    uint8_t nodeId = network.nodes[i].node_id;
    bool isOnline = network.nodes[i].online;
    bool receivedDirect = network.nodes[i].received_direct;
    uint32_t lastDirectSeen = network.nodes[i].last_direct_seen;
    uint32_t lastSeen = network.nodes[i].last_seen;
    uint8_t flags = network.nodes[i].flags;
    portEXIT_CRITICAL(&networkMux);

    if (nodeId == NODE_ID) continue;

    // Mark node offline after timeout
    // For nodes we receive directly (ESP-NOW), check BOTH direct AND indirect paths
    if (isOnline) {
      bool shouldMarkOffline = false;

      if (receivedDirect) {
        // FIX: Only use direct ESP-NOW path for timeout when we're authoritative
        bool directStale = (now - lastDirectSeen > NODE_TIMEOUT_SEC * 1000UL);
        shouldMarkOffline = directStale;

        if (shouldMarkOffline) {
          DEBUG_PRINTF("Node %d: Offline (direct: %lums ago, indirect: %lums ago)\n",
                       nodeId, now - lastDirectSeen, now - lastSeen);
        }
      } else {
        // Only known via LoRa - use last_seen only
        shouldMarkOffline = (now - lastSeen > NODE_TIMEOUT_SEC * 1000UL);
      }

      if (shouldMarkOffline) {
        // M1 FIX: Enter critical section for state mutations
        portENTER_CRITICAL(&networkMux);
        network.nodes[i].online = false;
        network.nodes[i].received_direct = false;
        network.nodes[i].via_espnow = false;
        network.nodes[i].via_lora = false;

        // AUTO-CLEAR FISH_ON if node goes offline
        bool hadFishOn = HAS_FLAG(network.nodes[i].flags, FLAG_FISH_ON);
        if (hadFishOn) {
          CLEAR_FLAG(network.nodes[i].flags, FLAG_FISH_ON);
          network.nodes[i].fish_on_time = 0;
        }
        portEXIT_CRITICAL(&networkMux);

        registerActivity();  // Wake display when node goes offline (outside critical)

        if (hadFishOn) {
          DEBUG_PRINTF("Node %d offline - auto-clearing FISH_ON\n", nodeId);
          // Mark alert as inactive in history
          for (int j = 0; j < ALERT_HISTORY_SIZE; j++) {
            if (alertHistory[j].nodeId == nodeId && alertHistory[j].active) {
              alertHistory[j].active = false;
            }
          }
        }

        DEBUG_PRINTF("Node %d offline\n", nodeId);
      }
    }
  }

  updateAlertState();
}

// ═══════════════════════════════════════════════════════════════════════════
// BATTERY MONITORING
// ═══════════════════════════════════════════════════════════════════════════

void readBattery() {
  #if VBAT_PIN >= 0
    // Configure ADC attenuation for ESP32-S3 (11dB for 0-3.3V range)
    analogSetPinAttenuation(VBAT_PIN, ADC_11db);
    delay(5);  // Let ADC settle after config

    // P1 FIX: Take fewer samples with shorter delay for better performance
    // Reduced from 10 samples/2ms to 5 samples/1ms (saves ~15ms per reading)
    uint32_t total = 0;
    const int samples = 5;  // P1 FIX: Was 10
    uint32_t minReading = 4095;
    uint32_t maxReading = 0;

    for (int i = 0; i < samples; i++) {
      uint32_t reading = analogRead(VBAT_PIN);
      total += reading;
      if (reading < minReading) minReading = reading;
      if (reading > maxReading) maxReading = reading;
      delay(1);  // P1 FIX: Was 2ms
    }

    uint32_t avgReading = total / samples;

    // Validate ADC reading is plausible
    if (avgReading == 0) {
      DEBUG_PRINTLN(F("WARNING: Battery ADC reading is 0 - pin not connected?"));
      localBatteryMv = 0;
    } else if (avgReading >= 4090) {
      DEBUG_PRINTLN(F("WARNING: Battery ADC reading at max - check voltage divider"));
      localBatteryMv = 0;
    } else if ((maxReading - minReading) > 500) {
      DEBUG_PRINTF("WARNING: High ADC variance (min:%lu max:%lu) - noisy reading\n", minReading, maxReading);
      localBatteryMv = 0;
    } else {
      // Calculate voltage: (ADC * ref_mv / 4095) * divider_ratio
      uint32_t adcMv = (avgReading * VBAT_REF_MV) / 4095;
      localBatteryMv = (uint16_t)(adcMv * VBAT_DIVIDER);
    }

    // Update self-node battery in network state
    network.nodes[0].battery_mv = localBatteryMv;

    // Check for low battery condition
    if (localBatteryMv > 0 && localBatteryMv < BATTERY_LOW_MV) {
      SET_FLAG(network.nodes[0].flags, FLAG_LOW_BATTERY);
      DEBUG_PRINTF("WARNING: Low battery! %dmV\n", localBatteryMv);
    } else {
      CLEAR_FLAG(network.nodes[0].flags, FLAG_LOW_BATTERY);
    }

    DEBUG_PRINTF("Battery: ADC=%lu (min:%lu max:%lu) -> %dmV\n",
                 avgReading, minReading, maxReading, localBatteryMv);
  #else
    localBatteryMv = 0;
    network.nodes[0].battery_mv = 0;
  #endif
}

// ═══════════════════════════════════════════════════════════════════════════
// LOCAL SENSOR
// ═══════════════════════════════════════════════════════════════════════════

void setupLocalSensor() {
  DEBUG_PRINTLN(F("Setting up local sensor..."));
  pinMode(REED_PIN, INPUT_PULLUP);
}

void loopLocalSensor() {
  if (!HAS_LOCAL_SENSOR) return;

  if (millis() - lastReedCheck < 100) return;
  lastReedCheck = millis();

  // Read reed switch with polarity setting
  bool reedRaw = digitalRead(REED_PIN);
  bool reedState = settings.reedActiveHigh ? (reedRaw == HIGH) : (reedRaw == LOW);
  
  if (reedState != localReedState) {
    localReedState = reedState;
    localSequence++;  // Increment our sequence
    
    if (reedState) {
      localFishOn = true;
      SET_FLAG(network.nodes[0].flags, FLAG_FISH_ON);
      network.nodes[0].fish_on_time = millis();  // Record when FISH_ON started
      DEBUG_PRINTF("LOCAL: Fish on! seq=%d\n", localSequence);
      triggerBuzzer(3);

      // Update our own state immediately
      network.nodes[0].last_seq = localSequence;
      network.nodes[0].battery_mv = localBatteryMv;
      network.nodes[0].last_seen = millis();
      network.nodes[0].last_uptime = millis() / 1000;

      // Record in alert history
      recordAlert(NODE_ID);

      // Relay via LoRa with 3x retry for reliability
      for (int i = 0; i < 3; i++) {
        relayLoRaAlert(NODE_ID, network.nodes[0].flags, localBatteryMv, localSequence);
        if (i < 2) delay(150 + random(100));
      }
      DEBUG_PRINTLN("LOCAL: FISH_ON alert sent with 3x retry");
    } else {
      localFishOn = false;
      CLEAR_FLAG(network.nodes[0].flags, FLAG_FISH_ON);
      network.nodes[0].fish_on_time = 0;  // Clear fish_on_time
      DEBUG_PRINTF("LOCAL: Reset seq=%d\n", localSequence);

      // Update our own state
      network.nodes[0].last_seq = localSequence;
      network.nodes[0].battery_mv = localBatteryMv;
      network.nodes[0].last_seen = millis();
      network.nodes[0].last_uptime = millis() / 1000;

      // Send immediate LoRa status update with retry when FISH_ON clears
      if (loraReady) {
        DEBUG_PRINTLN("LOCAL: FISH_ON cleared - sending LoRa aggregate with retry");
        sendLoRaAggregateWithRetry(3);
      }
    }

    updateAlertState();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// BUZZER
// ═══════════════════════════════════════════════════════════════════════════

void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void triggerBuzzer(uint8_t pattern) {
  // Respect buzzer enabled setting
  if (!settings.buzzerEnabled) return;
  
  // Respect silence state (but only for alert patterns, not startup beep)
  if (alertsSilenced && pattern > 1) return;
  
  buzzerState = pattern * 2;
  lastBuzzerTime = 0;
}

void loopBuzzer() {
  if (buzzerState == 0) {
    // Auto-repeat alerts if enabled and not silenced
    if (activeAlerts && !alertsSilenced && settings.buzzerEnabled &&
        (millis() - lastBuzzerTime > BUZZER_REPEAT_DELAY_MS)) {
      buzzerState = BUZZER_ALERT_BEEPS * 2;
      lastBuzzerTime = 0;
    }
    return;
  }
  
  unsigned long interval = (buzzerState % 2 == 0) ? BUZZER_BEEP_ON_MS : BUZZER_BEEP_OFF_MS;
  
  if (millis() - lastBuzzerTime >= interval) {
    lastBuzzerTime = millis();
    
    if (buzzerState % 2 == 0) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
    
    buzzerState--;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// WIFI SETUP (AP, STA, or AP+STA mode)
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// WIFI CREDENTIAL MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

void loadWifiCredentials() {
  preferences.begin("wifi", true);  // Read-only
  
  String ssid = preferences.getString("ssid", "");
  String pass = preferences.getString("pass", "");
  
  preferences.end();
  
  if (ssid.length() > 0) {
    strncpy(storedSsid, ssid.c_str(), sizeof(storedSsid) - 1);
    strncpy(storedPassword, pass.c_str(), sizeof(storedPassword) - 1);
    DEBUG_PRINTF("Loaded WiFi credentials: %s\n", storedSsid);
  } else {
    // Use defaults from config.h
    strncpy(storedSsid, STA_SSID, sizeof(storedSsid) - 1);
    strncpy(storedPassword, STA_PASSWORD, sizeof(storedPassword) - 1);
    DEBUG_PRINTLN(F("Using default WiFi credentials from config"));
  }
}

void saveWifiCredentials(const char* ssid, const char* password) {
  preferences.begin("wifi", false);  // Read-write
  preferences.putString("ssid", ssid);
  preferences.putString("pass", password);
  preferences.end();
  
  strncpy(storedSsid, ssid, sizeof(storedSsid) - 1);
  strncpy(storedPassword, password, sizeof(storedPassword) - 1);
  
  DEBUG_PRINTF("Saved WiFi credentials: %s\n", storedSsid);
}

// Check serial for WiFi config commands: WIFI:ssid:password
void checkSerialWifiConfig() {
  if (!Serial.available()) return;
  
  String line = Serial.readStringUntil('\n');
  line.trim();
  
  if (line.startsWith("WIFI:")) {
    // Format: WIFI:ssid:password
    int firstColon = line.indexOf(':', 5);
    if (firstColon > 5) {
      String ssid = line.substring(5, firstColon);
      String pass = line.substring(firstColon + 1);
      
      if (ssid.length() > 0) {
        saveWifiCredentials(ssid.c_str(), pass.c_str());
        Serial.println(F("WiFi credentials saved! Restarting..."));
        delay(1000);
        ESP.restart();
      }
    }
    Serial.println(F("Format: WIFI:ssid:password"));
  } else if (line == "WIFI?") {
    Serial.printf("Current SSID: %s\n", storedSsid);
    Serial.printf("STA Connected: %s\n", wifiStaConnected ? "Yes" : "No");
    if (wifiStaConnected) {
      Serial.printf("STA IP: %s\n", staIpAddress.c_str());
    }
    Serial.printf("AP Active: %s\n", wifiApActive ? "Yes" : "No");
    if (wifiApActive) {
      Serial.printf("AP IP: %s\n", apIpAddress.c_str());
    }
  } else if (line == "WIFI CLEAR") {
    preferences.begin("wifi", false);
    preferences.clear();
    preferences.end();
    Serial.println(F("WiFi credentials cleared!"));
  }
}

void setupWiFiAP() {
  DEBUG_PRINTLN(F("Setting up WiFi..."));

  // ═══════════════════════════════════════════════════════════════════════════
  // GATEWAY_ONSHORE with LR mode: Skip WiFi AP setup entirely
  // ═══════════════════════════════════════════════════════════════════════════
  // When Long Range mode is enabled for GATEWAY_ONSHORE:
  //   - WiFi is already configured as STA-only with LR protocol in setupEspNow()
  //   - No AP is needed (users check status from OFFSHORE gateway in cabin)
  //   - Web server is not accessible anyway without AP
  //   - This preserves the LR protocol setting (AP mode would break it)
  // ═══════════════════════════════════════════════════════════════════════════
  #if ESPNOW_LONG_RANGE_MODE
  if (currentRole == ROLE_GATEWAY_ONSHORE) {
    DEBUG_PRINTLN(F("GATEWAY_ONSHORE: Skipping WiFi AP (using LR mode for ESP-NOW)"));
    DEBUG_PRINTLN(F("  → No web server on ice gateway"));
    DEBUG_PRINTLN(F("  → Check status from offshore gateway (cabin) instead"));
    wifiApActive = false;
    wifiStaConnected = false;
    return;  // Skip all WiFi AP setup
  }
  #endif

  // Load credentials from NVS (or use defaults)
  loadWifiCredentials();

  char apSsid[32];
  snprintf(apSsid, sizeof(apSsid), "%s-%s",
           NETWORK_NAME,
           currentRole == ROLE_GATEWAY_OFFSHORE ? "Remote" : "Ice");

  // Determine WiFi mode based on settings
  // Note: GATEWAY_ONSHORE with LR mode already returned above
  uint8_t effectiveWifiMode = settings.wifiModeSetting;
  #if !ESPNOW_LONG_RANGE_MODE
  // Only force AP mode for GATEWAY_ONSHORE when LR mode is disabled
  if (currentRole == ROLE_GATEWAY_ONSHORE) {
    DEBUG_PRINTLN(F("GATEWAY_ONSHORE: Forcing AP-only mode (no STA)"));
    effectiveWifiMode = 0;  // Force AP mode
  }
  #endif

  switch (effectiveWifiMode) {
    case 0:  // AP only
      DEBUG_PRINTLN(F("Mode: AP only"));
      WiFi.mode(WIFI_AP);
      WiFi.softAP(apSsid, WIFI_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONNECTIONS);
      apIpAddress = WiFi.softAPIP().toString();
      DEBUG_PRINTF("AP SSID: %s\n", apSsid);
      DEBUG_PRINTF("AP IP:   %s\n", apIpAddress.c_str());
      wifiApActive = true;
      break;

    case 1:  // STA only
      DEBUG_PRINTLN(F("Mode: STA only"));
      WiFi.mode(WIFI_STA);
      WiFi.begin(storedSsid, storedPassword);
      DEBUG_PRINTF("Connecting to: %s\n", storedSsid);

      // Wait for connection (with timeout)
      {
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 30) {
          delay(500);
          DEBUG_PRINT(".");
          attempts++;
        }
        DEBUG_PRINTLN();
      }

      if (WiFi.status() == WL_CONNECTED) {
        wifiStaConnected = true;
        staIpAddress = WiFi.localIP().toString();
        DEBUG_PRINTF("Connected! IP: %s\n", staIpAddress.c_str());
      } else {
        DEBUG_PRINTLN(F("Connection failed! Falling back to AP mode"));
        WiFi.mode(WIFI_AP);
        WiFi.softAP(apSsid, WIFI_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONNECTIONS);
        apIpAddress = WiFi.softAPIP().toString();
        wifiApActive = true;
      }
      break;

    case 2:  // AP + STA
    default:
      DEBUG_PRINTLN(F("Mode: AP + STA"));
      WiFi.mode(WIFI_AP_STA);

      // Start AP first
      WiFi.softAP(apSsid, WIFI_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONNECTIONS);
      apIpAddress = WiFi.softAPIP().toString();
      DEBUG_PRINTF("AP SSID: %s\n", apSsid);
      DEBUG_PRINTF("AP IP:   %s\n", apIpAddress.c_str());
      wifiApActive = true;

      // Try to connect to STA network with timeout
      WiFi.begin(storedSsid, storedPassword);
      DEBUG_PRINTF("Connecting to: %s", storedSsid);

      // Wait up to 10 seconds for connection
      {
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
          delay(500);
          DEBUG_PRINT(".");
          attempts++;
        }
        DEBUG_PRINTLN();
      }

      if (WiFi.status() == WL_CONNECTED) {
        wifiStaConnected = true;
        staIpAddress = WiFi.localIP().toString();
        DEBUG_PRINTF("STA connected! IP: %s\n", staIpAddress.c_str());
      } else {
        DEBUG_PRINTLN(F("STA connection failed (continuing with AP only)"));
        WiFi.disconnect();  // Clean up failed connection
      }
      break;
  }
  
  // Set channel for ESP-NOW compatibility
  // CRITICAL: Wait for AP to stabilize, then force channel and verify
  delay(100);  // Let AP stabilize
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(50);

  // VERIFY channel alignment
  uint8_t primary;
  wifi_second_chan_t secondary;
  esp_wifi_get_channel(&primary, &secondary);
  DEBUG_PRINTF("WiFi channel verified: %d (expected: %d)\n", primary, ESPNOW_CHANNEL);

  if (primary != ESPNOW_CHANNEL) {
    DEBUG_PRINTLN("WARNING: Channel mismatch detected! Retrying...");
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(50);
    esp_wifi_get_channel(&primary, &secondary);
    DEBUG_PRINTF("Channel after retry: %d\n", primary);
  }
}

// Call this in loop to check STA connection status
void checkWiFiStatus() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 5000) return;
  lastCheck = millis();

  if (WIFI_MODE_SETTING == WIFI_MODE_APSTA || WIFI_MODE_SETTING == WIFI_MODE_STA) {
    if (WiFi.status() == WL_CONNECTED && !wifiStaConnected) {
      wifiStaConnected = true;
      staIpAddress = WiFi.localIP().toString();
      DEBUG_PRINTF("STA connected! IP: %s\n", staIpAddress.c_str());
      // BUG FIX #6: Verify channel after STA connects
      verifyWiFiChannel();
    } else if (WiFi.status() != WL_CONNECTED && wifiStaConnected) {
      wifiStaConnected = false;
      staIpAddress = "";
      DEBUG_PRINTLN(F("STA disconnected"));
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// BUG FIX #6: WiFi channel verification after STA connection
// ═══════════════════════════════════════════════════════════════════════════

void verifyWiFiChannel() {
  uint8_t primary;
  wifi_second_chan_t secondary;
  esp_wifi_get_channel(&primary, &secondary);

  DEBUG_PRINTF("WiFi channel check: current=%d, expected=%d\n", primary, ESPNOW_CHANNEL);

  if (primary != ESPNOW_CHANNEL) {
    DEBUG_PRINTLN("WARNING: WiFi channel mismatch detected after STA connection!");

    // When STA is connected to an AP on a different channel, the ESP32
    // is forced to use that AP's channel. ESP-NOW will only work if all
    // devices are on the same channel.

    // Option 1: Warn user and document the behavior
    DEBUG_PRINTF("STA connected to AP on channel %d, ESP-NOW may not work with other nodes on channel %d\n",
                 primary, ESPNOW_CHANNEL);

    // P2 FIX: Try to reset channel with exponential backoff (10ms, 20ms, 40ms)
    // This gives the WiFi stack time to process the change
    int backoffDelays[] = {10, 20, 40};
    bool channelResetSuccess = false;

    for (int attempt = 0; attempt < 3 && !channelResetSuccess; attempt++) {
      esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
      delay(backoffDelays[attempt]);  // P2 FIX: Exponential backoff

      // Verify the result
      esp_wifi_get_channel(&primary, &secondary);
      if (primary == ESPNOW_CHANNEL) {
        DEBUG_PRINTF("Channel reset successful on attempt %d\n", attempt + 1);
        channelResetSuccess = true;
      }
    }

    if (!channelResetSuccess) {
      DEBUG_PRINTF("Channel reset failed - ESP-NOW using channel %d (STA AP channel)\n", primary);
      DEBUG_PRINTLN("ESP-NOW sensor nodes must be on the same channel as the connected WiFi network");
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// WEB SERVER
// ═══════════════════════════════════════════════════════════════════════════

void setupWebServer() {
  DEBUG_PRINTLN(F("Setting up web server..."));

  server.on("/", HTTP_GET, handleWebRoot);
  server.on("/api/status", HTTP_GET, handleWebApi);
  server.on("/api/silence", HTTP_POST, handleWebApiSilence);
  server.on("/api/node/name", HTTP_POST, handleWebApiNodeName);
  server.on("/settings", HTTP_GET, handleWebSettings);
  server.on("/api/settings", HTTP_GET, handleWebApiSettingsGet);
  server.on("/api/settings", HTTP_POST, handleWebApiSettingsPost);
  server.on("/remote-config", HTTP_GET, handleWebRemoteConfig);
  server.on("/api/remote-config", HTTP_POST, handleWebApiRemoteConfig);
  server.on("/api/remote-config/status", HTTP_GET, handleWebApiRemoteConfigStatus);

  server.begin();
  DEBUG_PRINTLN(F("Web server ready on port 80"));
}

void loopWebServer() {
  server.handleClient();
}

void handleWebRoot() {
  String html = F(R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
  <title>Ice Fishing Monitor</title>
  <style>
    :root {
      --bg-primary: #0f0f1a;
      --bg-secondary: #1a1a2e;
      --bg-card: #16213e;
      --bg-card-hover: #1f2b47;
      --text-primary: #f0f0f0;
      --text-secondary: #a0a0a0;
      --accent-blue: #4fc3f7;
      --accent-green: #00d26a;
      --accent-red: #ff5252;
      --accent-orange: #ffc107;
      --accent-gray: #555;
      --border-radius: 12px;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      background: linear-gradient(135deg, var(--bg-primary) 0%, var(--bg-secondary) 100%);
      color: var(--text-primary);
      min-height: 100vh;
      padding: 16px;
    }
    .container { max-width: 800px; margin: 0 auto; }
    .header { text-align: center; padding: 16px 0; border-bottom: 1px solid rgba(79,195,247,0.2); margin-bottom: 16px; }
    .header h1 { color: var(--accent-blue); font-size: 24px; font-weight: 600; }
    .header-info { display: flex; justify-content: center; gap: 20px; margin-top: 10px; font-size: 12px; color: var(--text-secondary); }
    .alert-banner {
      background: linear-gradient(135deg, #d32f2f, #b71c1c);
      color: white; padding: 20px; border-radius: var(--border-radius);
      text-align: center; font-size: 26px; font-weight: bold;
      margin-bottom: 16px; display: none;
      animation: alertPulse 0.8s ease-in-out infinite;
      box-shadow: 0 0 30px rgba(255,82,82,0.4);
    }
    .alert-banner.active { display: block; }
    @keyframes alertPulse { 0%,100% { transform: scale(1); } 50% { transform: scale(1.02); opacity: 0.9; } }
    .silence-btn {
      display: block; width: 100%; padding: 14px; margin-bottom: 16px;
      background: var(--bg-card); border: 2px solid var(--accent-blue);
      border-radius: var(--border-radius); color: var(--accent-blue);
      font-size: 16px; font-weight: 600; cursor: pointer; transition: all 0.2s;
    }
    .silence-btn:hover { background: var(--accent-blue); color: var(--bg-primary); }
    .silence-btn.silenced { background: var(--accent-orange); border-color: var(--accent-orange); color: #000; }
    .node-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(130px, 1fr)); gap: 10px; margin-bottom: 20px; }
    .node-card {
      background: var(--bg-card); border-radius: var(--border-radius); padding: 14px;
      text-align: center; border: 2px solid transparent; transition: all 0.2s; position: relative;
    }
    .node-card:hover { background: var(--bg-card-hover); transform: translateY(-2px); }
    .node-card.online { border-color: var(--accent-green); }
    .node-card.offline { border-color: var(--accent-gray); opacity: 0.6; }
    .node-card.fish {
      border-color: var(--accent-red);
      background: linear-gradient(135deg, #4a1a1a, #2a0a0a);
      animation: fishPulse 0.6s ease-in-out infinite;
    }
    .node-card.lowbat { border-color: var(--accent-orange); }
    @keyframes fishPulse { 0%,100% { box-shadow: 0 0 0 0 rgba(255,82,82,0.4); } 50% { box-shadow: 0 0 20px 4px rgba(255,82,82,0.6); } }
    .node-id { font-size: 28px; font-weight: bold; color: var(--accent-blue); margin-bottom: 6px; }
    .node-card.fish .node-id { color: var(--accent-red); }
    .node-card.offline .node-id { color: var(--accent-gray); }
    .node-name { font-size: 10px; color: var(--text-secondary); margin-bottom: 6px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .node-role { font-size: 10px; color: var(--text-secondary); margin-top: 2px; font-style: italic; }
    .node-status { display: inline-block; padding: 3px 10px; border-radius: 16px; font-size: 11px; font-weight: 600; margin-bottom: 6px; }
    .node-card.online .node-status { background: rgba(0,210,106,0.2); color: var(--accent-green); }
    .node-card.offline .node-status { background: rgba(85,85,85,0.3); color: var(--accent-gray); }
    .node-card.fish .node-status { background: rgba(255,82,82,0.3); color: var(--accent-red); }
    .node-battery { font-size: 12px; color: var(--text-secondary); display: flex; align-items: center; justify-content: center; gap: 4px; }
    .node-battery .bar { width: 36px; height: 7px; background: var(--bg-secondary); border-radius: 4px; overflow: hidden; }
    .node-battery .bar-fill { height: 100%; background: var(--accent-green); border-radius: 4px; }
    .node-card.lowbat .bar-fill { background: var(--accent-orange); }
    .stats-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-bottom: 16px; }
    .stat-card { background: var(--bg-card); border-radius: var(--border-radius); padding: 12px; text-align: center; }
    .stat-value { font-size: 20px; font-weight: bold; color: var(--accent-blue); }
    .stat-label { font-size: 11px; color: var(--text-secondary); margin-top: 2px; }
    .legend { display: flex; justify-content: center; flex-wrap: wrap; gap: 14px; margin-bottom: 16px; font-size: 11px; }
    .legend-item { display: flex; align-items: center; gap: 5px; }
    .legend-dot { width: 10px; height: 10px; border-radius: 50%; border: 2px solid; }
    .legend-dot.ok { background: transparent; border-color: var(--accent-green); }
    .legend-dot.fish { background: var(--accent-red); border-color: var(--accent-red); }
    .legend-dot.lowbat { background: transparent; border-color: var(--accent-orange); }
    .legend-dot.offline { background: transparent; border-color: var(--accent-gray); }
    .footer { text-align: center; padding: 12px; color: var(--text-secondary); font-size: 11px; border-top: 1px solid rgba(79,195,247,0.1); }
    @media (max-width: 480px) { .node-grid { grid-template-columns: repeat(2, 1fr); } .stats-grid { grid-template-columns: repeat(3, 1fr); } }
  </style>
</head>
<body>
  <div class="container">
    <header class="header">
      <h1>Ice Fishing Monitor <button id="sound-toggle" onclick="toggleSound()" style="font-size:16px;background:none;border:none;cursor:pointer">🔊</button></h1>
      <div class="header-info">
        <span id="uptime">Uptime: --</span>
        <span id="node-count">Nodes: --</span>
        <span id="update-time">Updated: --</span>
      </div>
    </header>
    <div id="alert-banner" class="alert-banner">FISH ON!</div>
    <button id="silence-btn" class="silence-btn" onclick="toggleSilence()">Silence Alerts</button>
    <div id="node-grid" class="node-grid"></div>
    <div class="legend">
      <div class="legend-item"><div class="legend-dot ok"></div> Online</div>
      <div class="legend-item"><div class="legend-dot fish"></div> FISH ON!</div>
      <div class="legend-item"><div class="legend-dot lowbat"></div> Low Batt</div>
      <div class="legend-item"><div class="legend-dot offline"></div> Offline</div>
    </div>
    <div class="stats-grid">
      <div class="stat-card"><div class="stat-value" id="lora-rx">--</div><div class="stat-label">LoRa RX</div></div>
      <div class="stat-card"><div class="stat-value" id="lora-tx">--</div><div class="stat-label">LoRa TX</div></div>
      <div class="stat-card"><div class="stat-value" id="wifi-mode">--</div><div class="stat-label">WiFi</div></div>
    </div>
    <footer class="footer">Ice Fishing Mesh Monitor v2.0 | <a href="/settings" style="color:#4fc3f7">Settings</a> | <a href="/remote-config" style="color:#4fc3f7">Remote Config</a></footer>
  </div>
  <script>
    let silenced = false;
    let alertSoundEnabled = localStorage.getItem('alertSound') !== 'false';
    let lastAlertState = {};
    let audioContext = null;

    function formatUptime(s) { const h = Math.floor(s/3600), m = Math.floor((s%3600)/60); return h > 0 ? h+'h '+m+'m' : m+'m'; }

    // Web Audio API alert beep
    function playAlertBeep() {
      if (!alertSoundEnabled) return;
      try {
        if (!audioContext) audioContext = new (window.AudioContext || window.webkitAudioContext)();
        if (audioContext.state === 'suspended') audioContext.resume();
        const oscillator = audioContext.createOscillator();
        const gainNode = audioContext.createGain();
        oscillator.connect(gainNode);
        gainNode.connect(audioContext.destination);
        oscillator.frequency.value = 800;
        oscillator.type = 'square';
        gainNode.gain.setValueAtTime(0.3, audioContext.currentTime);
        gainNode.gain.exponentialRampToValueAtTime(0.01, audioContext.currentTime + 0.5);
        oscillator.start(audioContext.currentTime);
        oscillator.stop(audioContext.currentTime + 0.5);
      } catch(e) { console.log('Audio error:', e); }
    }

    function checkForNewAlerts(nodes) {
      nodes.forEach(n => {
        const wasAlert = lastAlertState[n.id] || false;
        const isAlert = n.fish;
        if (isAlert && !wasAlert) playAlertBeep();
        lastAlertState[n.id] = isAlert;
      });
    }

    function toggleSound() {
      alertSoundEnabled = !alertSoundEnabled;
      localStorage.setItem('alertSound', alertSoundEnabled);
      document.getElementById('sound-toggle').textContent = alertSoundEnabled ? '🔊' : '🔇';
      if (alertSoundEnabled && audioContext && audioContext.state === 'suspended') audioContext.resume();
    }

    function editNodeName(nodeId, currentName) {
      const newName = prompt('Enter name for Node ' + nodeId + ' (max 15 chars):', currentName || '');
      if (newName !== null && newName.length <= 15) {
        fetch('/api/node/name', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ nodeId: nodeId, name: newName })
        }).then(r => r.json()).then(data => {
          if (data.success) updateStatus();
        }).catch(e => console.error(e));
      }
    }

    function updateStatus() {
      fetch('/api/status').then(r => r.json()).then(data => {
        document.getElementById('uptime').textContent = 'Uptime: ' + formatUptime(data.uptime);
        document.getElementById('node-count').textContent = 'Nodes: ' + data.node_count;
        document.getElementById('update-time').textContent = 'Updated: ' + new Date().toLocaleTimeString();
        if (data.lora) { document.getElementById('lora-rx').textContent = data.lora.rx_count || 0; document.getElementById('lora-tx').textContent = data.lora.tx_count || 0; }
        document.getElementById('wifi-mode').textContent = data.wifi.sta_connected ? 'STA' : (data.wifi.ap_active ? 'AP' : '--');
        const grid = document.getElementById('node-grid');
        const banner = document.getElementById('alert-banner');
        grid.innerHTML = '';
        let hasAlert = false;
        checkForNewAlerts(data.nodes);
        data.nodes.forEach(n => {
          const card = document.createElement('div');
          card.className = 'node-card';
          if (!n.online) card.classList.add('offline'); else card.classList.add('online');
          if (n.fish) { card.classList.add('fish'); hasAlert = true; }
          if (n.lowbat) card.classList.add('lowbat');
          const pct = Math.min(100, Math.max(0, Math.round((n.battery - 3200) / 13)));
          const status = !n.online ? 'OFFLINE' : (n.fish ? 'FISH ON!' : 'OK');
          card.innerHTML = '<div class="node-id">' + n.id + '</div>' +
            '<div class="node-name">' + (n.name || 'Node ' + n.id) + '</div>' +
            '<div class="node-role">' + (n.role || '') + '</div>' +
            '<div class="node-status">' + status + '</div>' +
            '<div class="node-battery"><div class="bar"><div class="bar-fill" style="width:' + pct + '%"></div></div><span>' + pct + '%</span></div>';
          card.onclick = () => editNodeName(n.id, n.name);
          card.style.cursor = 'pointer';
          grid.appendChild(card);
        });
        banner.classList.toggle('active', hasAlert && !silenced);
        const btn = document.getElementById('silence-btn');
        btn.classList.toggle('silenced', silenced);
        btn.textContent = silenced ? 'Alerts Silenced - Tap to Enable' : 'Silence Alerts';
      }).catch(e => console.error(e));
    }

    function toggleSilence() {
      fetch('/api/silence', { method: 'POST' }).then(r => r.json()).then(data => { silenced = data.silenced; updateStatus(); }).catch(e => console.error(e));
    }

    // Initialize sound toggle button state
    document.getElementById('sound-toggle').textContent = alertSoundEnabled ? '🔊' : '🔇';
    updateStatus();
    setInterval(updateStatus, 2000);
  </script>
</body>
</html>
)rawliteral");

  server.send(200, "text/html", html);
}

void handleWebApi() {
  // BUG FIX #10: Increased from 1024 to 2048 for 16 nodes (~80 bytes each)
  StaticJsonDocument<2048> doc;
  doc["network_id"] = network.network_id;
  doc["node_count"] = network.node_count;
  doc["uptime"] = millis() / 1000;

  // WiFi status
  JsonObject wifi = doc.createNestedObject("wifi");
  wifi["ap_active"] = wifiApActive;
  wifi["ap_ip"] = apIpAddress;
  wifi["sta_connected"] = wifiStaConnected;
  wifi["sta_ip"] = staIpAddress;

  // IMPROVEMENT 1: Include LoRa diagnostic counters
  JsonObject lora = doc.createNestedObject("lora");
  lora["ready"] = loraReady;
  lora["rx_count"] = loraRxCount;
  lora["tx_count"] = loraTxCount;
  lora["rx_errors"] = loraRxErrors;
  lora["tx_errors"] = loraTxErrors;
  lora["checksum_fails"] = loraChecksumFails;
  lora["watchdog_resets"] = loraWatchdogResets;

  JsonArray nodes = doc.createNestedArray("nodes");
  for (int i = 0; i < network.node_count; i++) {
    JsonObject n = nodes.createNestedObject();
    n["id"] = network.nodes[i].node_id;
    n["name"] = network.nodes[i].name;
    n["online"] = network.nodes[i].online;
    n["fish"] = HAS_FLAG(network.nodes[i].flags, FLAG_FISH_ON);
    n["lowbat"] = HAS_FLAG(network.nodes[i].flags, FLAG_LOW_BATTERY);
    n["battery"] = network.nodes[i].battery_mv;
    n["rssi"] = network.nodes[i].rssi;
    // IMPROVEMENT 2: Include via tracking
    n["via_espnow"] = network.nodes[i].via_espnow;
    n["via_lora"] = network.nodes[i].via_lora;
    // IMPROVEMENT 2: Include last-seen timestamp
    n["last_seen_sec"] = (millis() - network.nodes[i].last_seen) / 1000;
    // Add role name for display
    const char* roleName;
    switch (network.nodes[i].role) {
      case ROLE_SENSOR_ONLY:      roleName = "Sensor"; break;
      case ROLE_SENSOR_LORA:      roleName = "Sensor+LoRa"; break;
      case ROLE_RELAY_LORA:       roleName = "Relay"; break;
      case ROLE_GATEWAY_ONSHORE:  roleName = "Gateway Ice"; break;
      case ROLE_GATEWAY_OFFSHORE: roleName = "Gateway Remote"; break;
      default:                    roleName = "Unknown"; break;
    }
    n["role"] = roleName;
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

/*
 * API endpoint: POST /api/node/name
 * Set a custom name for a node
 * Body: {"nodeId": 2, "name": "Bob's Hole"}
 */
void handleWebApiNodeName() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"POST required\"}");
    return;
  }

  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No body\"}");
    return;
  }

  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  uint8_t nodeId = doc["nodeId"] | 0;
  const char* name = doc["name"] | "";

  if (nodeId == 0 || strlen(name) > 15) {
    server.send(400, "application/json", "{\"error\":\"Invalid nodeId or name (max 15 chars)\"}");
    return;
  }

  saveNodeName(nodeId, name);
  server.send(200, "application/json", "{\"success\":true}");
}

/*
 * API endpoint: POST /api/silence
 * Toggle silence state
 */
void handleWebApiSilence() {
  silenceAlerts();

  StaticJsonDocument<64> doc;
  doc["silenced"] = alertsSilenced;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleWebSettings() {
  // Get current role name for display
  const char* myRole;
  switch (currentRole) {
    case ROLE_GATEWAY_ONSHORE:  myRole = "Gateway Ice (Onshore)"; break;
    case ROLE_GATEWAY_OFFSHORE: myRole = "Gateway Remote (Offshore)"; break;
    case ROLE_SENSOR_LORA:      myRole = "Sensor + LoRa"; break;
    case ROLE_RELAY_LORA:       myRole = "LoRa Relay"; break;
    default:                    myRole = "Unknown"; break;
  }

  String html = F(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Settings - Ice Fishing Monitor</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #1a1a2e; color: #eee; margin: 0; padding: 20px; }
    .container { max-width: 500px; margin: 0 auto; }
    h1 { color: #4fc3f7; margin-bottom: 5px; }
    .device-info { color: #888; margin-bottom: 20px; font-size: 14px; }
    .form-group { margin-bottom: 15px; }
    label { display: block; margin-bottom: 5px; color: #aaa; }
    input[type="number"], select { width: 100%; padding: 10px; border: 1px solid #333; background: #252538; color: #fff; border-radius: 4px; font-size: 16px; }
    .checkbox-group { display: flex; align-items: center; gap: 10px; padding: 10px 0; }
    input[type="checkbox"] { width: 20px; height: 20px; }
    .checkbox-group label { margin: 0; }
    button { background: #4fc3f7; color: #000; border: none; padding: 12px 24px; border-radius: 4px; cursor: pointer; font-size: 16px; margin-right: 10px; margin-top: 10px; }
    button:hover { background: #03a9f4; }
    .back { background: #555; color: #fff; }
    .message { padding: 10px; border-radius: 4px; margin-bottom: 15px; display: none; }
    .message.show { display: block; }
    .success { background: #2e7d32; }
    .error { background: #c62828; }
    .note { color: #888; font-size: 12px; margin-top: 20px; padding: 10px; border-left: 3px solid #4fc3f7; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Device Settings</h1>
    <div class="device-info">)rawliteral");

  html += myRole;
  html += F(" &bull; Node ID: ");
  html += String(NODE_ID);
  html += F(R"rawliteral(</div>
    <div id="message" class="message"></div>
    <form id="settingsForm">
      <div class="form-group checkbox-group">
        <input type="checkbox" id="buzzerEnabled" name="buzzerEnabled">
        <label for="buzzerEnabled">Buzzer Enabled</label>
      </div>
      <div class="form-group">
        <label for="alertHoldSec">Alert Hold Time (seconds)</label>
        <input type="number" id="alertHoldSec" name="alertHoldSec" min="5" max="300">
        <small style="color:#666">Minimum time alert stays active (5-300)</small>
      </div>
      <div class="form-group">
        <label for="heartbeatSec">Heartbeat Interval (seconds)</label>
        <input type="number" id="heartbeatSec" name="heartbeatSec" min="10" max="600">
        <small style="color:#666">Status broadcast interval (10-600)</small>
      </div>
      <div class="form-group checkbox-group">
        <input type="checkbox" id="reedActiveHigh" name="reedActiveHigh">
        <label for="reedActiveHigh">Reed Switch: Trigger on HIGH</label>
      </div>
      <div>
        <button type="submit">Save Settings</button>
        <button type="button" class="back" onclick="location.href='/'">Back to Status</button>
      </div>
    </form>
    <div class="note">
      <strong>Note:</strong> Changes take effect immediately. Some settings may require a device reboot.
    </div>
  </div>
  <script>
    const msgEl = document.getElementById('message');

    function showMessage(text, isError) {
      msgEl.textContent = text;
      msgEl.className = 'message show ' + (isError ? 'error' : 'success');
      setTimeout(() => { msgEl.className = 'message'; }, 4000);
    }

    async function loadSettings() {
      try {
        const resp = await fetch('/api/settings');
        if (!resp.ok) throw new Error('Failed to load');
        const data = await resp.json();
        document.getElementById('buzzerEnabled').checked = data.buzzerEnabled;
        document.getElementById('alertHoldSec').value = data.alertHoldSec;
        document.getElementById('heartbeatSec').value = data.heartbeatSec;
        document.getElementById('reedActiveHigh').checked = data.reedActiveHigh;
      } catch (e) {
        showMessage('Failed to load settings: ' + e.message, true);
      }
    }

    document.getElementById('settingsForm').addEventListener('submit', async (e) => {
      e.preventDefault();
      const data = {
        buzzerEnabled: document.getElementById('buzzerEnabled').checked,
        alertHoldSec: parseInt(document.getElementById('alertHoldSec').value) || 30,
        heartbeatSec: parseInt(document.getElementById('heartbeatSec').value) || 60,
        reedActiveHigh: document.getElementById('reedActiveHigh').checked
      };
      try {
        const resp = await fetch('/api/settings', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(data)
        });
        if (!resp.ok) throw new Error('Save failed');
        showMessage('Settings saved successfully!', false);
      } catch (e) {
        showMessage('Failed to save: ' + e.message, true);
      }
    });

    loadSettings();
  </script>
</body>
</html>
)rawliteral");

  server.send(200, "text/html", html);
}

void handleWebApiSettingsGet() {
  StaticJsonDocument<256> doc;
  doc["buzzerEnabled"] = settings.buzzerEnabled;
  doc["alertHoldSec"] = settings.alertHoldSec;
  doc["heartbeatSec"] = settings.heartbeatSec;
  doc["reedActiveHigh"] = settings.reedActiveHigh;
  doc["displayBrightness"] = settings.displayBrightness;
  doc["webServerEnabled"] = settings.webServerEnabled;
  doc["wifiModeSetting"] = settings.wifiModeSetting;
  doc["nodeId"] = NODE_ID;
  doc["role"] = (uint8_t)currentRole;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleWebApiSettingsPost() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No body\"}");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    DEBUG_PRINTF("JSON parse error: %s\n", error.c_str());
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  bool changed = false;

  if (doc.containsKey("buzzerEnabled")) {
    settings.buzzerEnabled = doc["buzzerEnabled"].as<bool>();
    changed = true;
  }
  if (doc.containsKey("alertHoldSec")) {
    int val = doc["alertHoldSec"].as<int>();
    settings.alertHoldSec = constrain(val, 5, 300);
    changed = true;
  }
  if (doc.containsKey("heartbeatSec")) {
    int val = doc["heartbeatSec"].as<int>();
    settings.heartbeatSec = constrain(val, 10, 600);
    changed = true;
  }
  if (doc.containsKey("reedActiveHigh")) {
    settings.reedActiveHigh = doc["reedActiveHigh"].as<bool>();
    changed = true;
  }

  if (changed) {
    saveSettings();
    DEBUG_PRINTLN(F("Settings updated via web API"));
  }

  server.send(200, "application/json", "{\"success\":true}");
}

void handleWebRemoteConfig() {
  // Build list of known gateway nodes for dropdown
  String nodeOptions = "";
  for (int i = 0; i < network.node_count; i++) {
    NodeState* node = &network.nodes[i];
    // Include gateways and relays (roles 3, 4, 5)
    if (node->role >= ROLE_RELAY_LORA && node->node_id != NODE_ID) {
      char opt[64];
      const char* roleName;
      switch (node->role) {
        case ROLE_GATEWAY_ONSHORE:  roleName = "Gateway Ice"; break;
        case ROLE_GATEWAY_OFFSHORE: roleName = "Gateway Remote"; break;
        case ROLE_RELAY_LORA:       roleName = "Relay"; break;
        default:                    roleName = "Node"; break;
      }
      snprintf(opt, sizeof(opt), "<option value=\"%d\">Node %d - %s</option>",
               node->node_id, node->node_id, roleName);
      nodeOptions += opt;
    }
  }

  // If no other gateways found, show manual entry
  if (nodeOptions.length() == 0) {
    nodeOptions = "<option value=\"0\">No gateways discovered</option>";
  }

  String html = F(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Remote Config - Ice Fishing Monitor</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; background: #1a1a2e; color: #eee; margin: 0; padding: 20px; }
    .container { max-width: 500px; margin: 0 auto; }
    h1 { color: #4fc3f7; margin-bottom: 5px; }
    .subtitle { color: #aaa; margin-bottom: 20px; }
    .form-group { margin-bottom: 15px; }
    label { display: block; margin-bottom: 5px; color: #aaa; }
    input[type="number"], select { width: 100%; padding: 10px; border: 1px solid #333; background: #252538; color: #fff; border-radius: 4px; font-size: 16px; }
    .checkbox-group { display: flex; align-items: center; gap: 10px; padding: 10px 0; }
    input[type="checkbox"] { width: 20px; height: 20px; }
    .checkbox-group label { margin: 0; }
    button { background: #4fc3f7; color: #000; border: none; padding: 12px 24px; border-radius: 4px; cursor: pointer; font-size: 16px; margin-right: 10px; margin-top: 10px; }
    button:hover { background: #03a9f4; }
    .send-btn { background: #f44336; color: #fff; }
    .send-btn:hover { background: #d32f2f; }
    .back { background: #555; color: #fff; }
    .message { padding: 10px; border-radius: 4px; margin-bottom: 15px; display: none; }
    .message.show { display: block; }
    .success { background: #2e7d32; }
    .error { background: #c62828; }
    .pending { background: #f57c00; }
    .info-box { background: #252538; padding: 15px; border-radius: 4px; margin-bottom: 20px; border-left: 3px solid #4fc3f7; }
    .info-box p { margin: 5px 0; color: #aaa; font-size: 13px; }
    .status-indicator { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 8px; }
    .status-indicator.waiting { background: #f57c00; animation: pulse 1s infinite; }
    .status-indicator.success { background: #4caf50; }
    .status-indicator.failed { background: #f44336; }
    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
  </style>
</head>
<body>
  <div class="container">
    <h1>Remote Gateway Config</h1>
    <p class="subtitle">Push settings to another gateway via LoRa (supports multi-hop)</p>

    <div class="info-box">
      <p><strong>How it works:</strong></p>
      <p>Settings are sent via LoRa to the target gateway</p>
      <p>Messages relay through intermediate nodes if needed</p>
      <p>Target applies settings and sends acknowledgment back</p>
      <p>Timeout: 15 seconds (allows for multi-hop delay)</p>
    </div>

    <div id="message" class="message"></div>

    <form id="remoteForm">
      <div class="form-group">
        <label for="targetNode">Target Gateway</label>
        <select id="targetNode">
)rawliteral");

  html += nodeOptions;

  html += F(R"rawliteral(
        </select>
      </div>

      <div class="form-group">
        <label for="manualTarget">Or enter Node ID manually:</label>
        <input type="number" id="manualTarget" min="1" max="254" placeholder="e.g. 4">
      </div>

      <hr style="border-color:#333; margin: 20px 0;">

      <div class="form-group checkbox-group">
        <input type="checkbox" id="buzzerEnabled" checked>
        <label for="buzzerEnabled">Buzzer Enabled</label>
      </div>

      <div class="form-group">
        <label for="alertHoldSec">Alert Hold Time (seconds)</label>
        <input type="number" id="alertHoldSec" value="30" min="5" max="300">
      </div>

      <div class="form-group">
        <label for="heartbeatSec">Heartbeat Interval (seconds)</label>
        <input type="number" id="heartbeatSec" value="60" min="10" max="600">
      </div>

      <div class="form-group checkbox-group">
        <input type="checkbox" id="reedActiveHigh" checked>
        <label for="reedActiveHigh">Reed Switch: Trigger on HIGH</label>
      </div>

      <div>
        <button type="submit" class="send-btn">Send Config</button>
        <button type="button" class="back" onclick="location.href='/'">Back</button>
      </div>
    </form>

    <div id="ack-status" style="margin-top: 20px; display: none;">
      <span class="status-indicator waiting" id="status-dot"></span>
      <span id="status-text">Waiting for acknowledgment...</span>
    </div>
  </div>
  <script>
    const msgEl = document.getElementById('message');
    const ackStatus = document.getElementById('ack-status');
    const statusDot = document.getElementById('status-dot');
    const statusText = document.getElementById('status-text');
    let pollInterval = null;

    function showMessage(text, type) {
      msgEl.textContent = text;
      msgEl.className = 'message show ' + type;
      if (type !== 'pending') {
        setTimeout(() => { msgEl.className = 'message'; }, 5000);
      }
    }

    function getTargetNode() {
      const manual = document.getElementById('manualTarget').value;
      if (manual && parseInt(manual) > 0) {
        return parseInt(manual);
      }
      return parseInt(document.getElementById('targetNode').value) || 0;
    }

    async function checkAckStatus() {
      try {
        const resp = await fetch('/api/remote-config/status');
        const data = await resp.json();

        if (!data.waiting) {
          clearInterval(pollInterval);
          pollInterval = null;

          if (data.lastSuccess) {
            statusDot.className = 'status-indicator success';
            statusText.textContent = 'Config acknowledged!';
            showMessage('Remote gateway applied the config successfully!', 'success');
          } else {
            statusDot.className = 'status-indicator failed';
            statusText.textContent = 'Timeout - no acknowledgment received';
            showMessage('No ACK received. Config may still have been applied.', 'error');
          }

          setTimeout(() => { ackStatus.style.display = 'none'; }, 5000);
        }
      } catch (e) {
        console.error('Status check failed:', e);
      }
    }

    document.getElementById('remoteForm').addEventListener('submit', async (e) => {
      e.preventDefault();

      const targetNode = getTargetNode();
      if (!targetNode || targetNode < 1) {
        showMessage('Please select or enter a valid target node ID', 'error');
        return;
      }

      const data = {
        targetNode: targetNode,
        buzzerEnabled: document.getElementById('buzzerEnabled').checked,
        alertHoldSec: parseInt(document.getElementById('alertHoldSec').value) || 30,
        heartbeatSec: parseInt(document.getElementById('heartbeatSec').value) || 60,
        reedActiveHigh: document.getElementById('reedActiveHigh').checked
      };

      showMessage('Sending config via LoRa...', 'pending');
      ackStatus.style.display = 'block';
      statusDot.className = 'status-indicator waiting';
      statusText.textContent = 'Waiting for acknowledgment...';

      try {
        const resp = await fetch('/api/remote-config', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(data)
        });
        const result = await resp.json();

        if (result.success) {
          showMessage('Config sent to node ' + targetNode + '! Waiting for ACK...', 'pending');
          if (pollInterval) clearInterval(pollInterval);
          pollInterval = setInterval(checkAckStatus, 1000);
          setTimeout(() => {
            if (pollInterval) {
              clearInterval(pollInterval);
              checkAckStatus();
            }
          }, 16000);
        } else {
          showMessage('Failed: ' + (result.error || 'Unknown error'), 'error');
          ackStatus.style.display = 'none';
        }
      } catch (e) {
        showMessage('Error: ' + e.message, 'error');
        ackStatus.style.display = 'none';
      }
    });
  </script>
</body>
</html>
)rawliteral");

  server.send(200, "text/html", html);
}

void handleWebApiRemoteConfig() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No body\"}");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  uint8_t targetNode = doc["targetNode"] | 0;
  if (targetNode == 0 || targetNode == NODE_ID) {
    server.send(400, "application/json", "{\"error\":\"Invalid target node\"}");
    return;
  }

  // Build config from request (use current settings as defaults)
  DeviceSettings configToSend = settings;

  if (doc.containsKey("buzzerEnabled")) {
    configToSend.buzzerEnabled = doc["buzzerEnabled"].as<bool>();
  }
  if (doc.containsKey("alertHoldSec")) {
    configToSend.alertHoldSec = constrain(doc["alertHoldSec"].as<int>(), 5, 300);
  }
  if (doc.containsKey("heartbeatSec")) {
    configToSend.heartbeatSec = constrain(doc["heartbeatSec"].as<int>(), 10, 600);
  }
  if (doc.containsKey("reedActiveHigh")) {
    configToSend.reedActiveHigh = doc["reedActiveHigh"].as<bool>();
  }

  // Temporarily swap settings for sendRemoteConfig
  DeviceSettings originalSettings = settings;
  settings = configToSend;

  // Send config
  bool sent = sendRemoteConfig(targetNode);

  // Restore original settings (we're only sending, not applying locally)
  settings = originalSettings;

  if (sent) {
    server.send(200, "application/json", "{\"success\":true}");
    DEBUG_PRINTF("Remote config sent to node %d\n", targetNode);
  } else {
    server.send(500, "application/json", "{\"error\":\"LoRa send failed\"}");
  }
}

void handleWebApiRemoteConfigStatus() {
  StaticJsonDocument<128> doc;
  doc["waiting"] = pendingConfigWaiting;
  doc["targetNode"] = pendingConfigTarget;
  doc["seq"] = pendingConfigSeq;

  // Calculate if last config was successful (not waiting + recent)
  bool recentAttempt = (millis() - pendingConfigTime) < (CONFIG_ACK_TIMEOUT_MS + 5000);
  doc["lastSuccess"] = !pendingConfigWaiting && recentAttempt && (pendingConfigTarget != 0);

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// ═══════════════════════════════════════════════════════════════════════════
// CARDKB KEYBOARD & MENU INPUT
// ═══════════════════════════════════════════════════════════════════════════

// CardKB key codes
#define KEY_UP      0xB5
#define KEY_DOWN    0xB6
#define KEY_LEFT    0xB4
#define KEY_RIGHT   0xB7
#define KEY_ENTER   0x0D
#define KEY_ESC     0x1B
#define KEY_TAB     0x09
#define KEY_BACKSP  0x08
#define KEY_SPACE   0x20

void setupCardKB() {
  DEBUG_PRINTLN(F("Initializing CardKB..."));
  
  CardKBWire.begin(CARDKB_SDA, CARDKB_SCL, 100000);
  
  // Check if CardKB is present
  CardKBWire.beginTransmission(CARDKB_ADDR);
  if (CardKBWire.endTransmission() == 0) {
    cardKbAvailable = true;
    DEBUG_PRINTLN(F("CardKB detected!"));
  } else {
    cardKbAvailable = false;
    DEBUG_PRINTLN(F("CardKB not found"));
  }
}

void loopCardKB() {
  if (!cardKbAvailable) return;

  CardKBWire.requestFrom(CARDKB_ADDR, 1);
  if (CardKBWire.available()) {
    char key = CardKBWire.read();
    if (key != 0) {
      handleKeyPress(key);
    }
  }
}

/*
 * Enhanced button handling for devices without CardKB:
 * - Single press: Toggle silence (existing)
 * - Double press (within 500ms): Cycle through live status pages
 * - Long press (2+ seconds): Wake display if sleeping
 */
void loopButton() {
  static unsigned long buttonPressStart = 0;
  static uint8_t buttonPressCount = 0;
  static unsigned long lastButtonRelease = 0;

  bool pressed = (digitalRead(USER_BUTTON) == LOW);
  unsigned long now = millis();

  if (pressed && buttonPressStart == 0) {
    // Button just pressed - record start time
    buttonPressStart = now;
  }
  else if (!pressed && buttonPressStart > 0) {
    // Button just released
    unsigned long pressDuration = now - buttonPressStart;
    buttonPressStart = 0;

    if (pressDuration > 2000) {
      // Long press (2+ seconds) - wake display
      DEBUG_PRINTLN(F("PRG long press - waking display"));
      registerActivity();
    }
    else if (pressDuration > BUTTON_DEBOUNCE_MS) {
      // Short press - count for double-press detection
      registerActivity();
      if (now - lastButtonRelease < 500) {
        buttonPressCount++;
      } else {
        buttonPressCount = 1;
      }
      lastButtonRelease = now;
    }
  }

  // Process button count after settle time (500ms after last release)
  if (buttonPressCount > 0 && (now - lastButtonRelease > 500)) {
    if (buttonPressCount == 1) {
      // Single press - toggle silence
      DEBUG_PRINTLN(F("PRG single press - toggling silence"));
      silenceAlerts();
      showOverlayMessage(alertsSilenced ? "Silenced" : "Unsilenced", 1000);
    }
    else if (buttonPressCount >= 2) {
      // Double press - cycle page (on live status only)
      DEBUG_PRINTLN(F("PRG double press - cycling page"));
      if (currentScreen == SCREEN_LIVE_STATUS) {
        int totalPages = (network.node_count + 9) / 10;
        if (totalPages > 1) {
          liveStatusPage = (liveStatusPage + 1) % totalPages;
        }
      }
    }
    buttonPressCount = 0;
  }
}

void handleKeyPress(char key) {
  registerActivity();  // Wake display on any key press
  DEBUG_PRINTF("Key: 0x%02X\n", (uint8_t)key);

  // Route input based on current screen
  switch (currentScreen) {
    case SCREEN_LIVE_STATUS:
      handleLiveStatusInput(key);
      break;
    case SCREEN_MAIN_MENU:
      handleMainMenuInput(key);
      break;
    case SCREEN_NODE_LIST:
      handleNodeListInput(key);
      break;
    case SCREEN_NODE_DETAILS:
      handleNodeDetailsInput(key);
      break;
    case SCREEN_ALERT_HISTORY:
      handleAlertHistoryInput(key);
      break;
    case SCREEN_SETTINGS:
      handleSettingsInput(key);
      break;
    case SCREEN_WIFI_CONFIG:
      handleWifiConfigInput(key);
      break;
    case SCREEN_NETWORK_INFO:
      handleNetworkInfoInput(key);
      break;
    case SCREEN_REBOOT_CONFIRM:
      handleRebootConfirmInput(key);
      break;
    case SCREEN_SETTING_EDIT:
      handleSettingEditInput(key);
      break;
    case SCREEN_RESET_ALL_CONFIRM:
      handleResetAllConfirmInput(key);
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// INPUT HANDLERS PER SCREEN
// ═══════════════════════════════════════════════════════════════════════════

void handleLiveStatusInput(char key) {
  int totalPages = (network.node_count + 9) / 10;
  if (totalPages < 1) totalPages = 1;

  switch (key) {
    case KEY_ESC:
      navigateToScreen(SCREEN_MAIN_MENU);
      break;
    case KEY_LEFT:
      if (liveStatusPage > 0) {
        liveStatusPage--;
        DEBUG_PRINTF("Live status page: %d/%d\n", liveStatusPage + 1, totalPages);
      }
      break;
    case KEY_RIGHT:
      if (liveStatusPage < totalPages - 1) {
        liveStatusPage++;
        DEBUG_PRINTF("Live status page: %d/%d\n", liveStatusPage + 1, totalPages);
      }
      break;
    case KEY_SPACE:
    case 's':
    case 'S':
      silenceAlerts();
      break;
  }
}

void handleMainMenuInput(char key) {
  const int VISIBLE_ITEMS = 5;  // How many items fit on screen

  switch (key) {
    case KEY_UP:
      if (menuSelection > 0) {
        menuSelection--;
        if (menuSelection < menuScrollOffset) {
          menuScrollOffset = menuSelection;
        }
      }
      break;
    case KEY_DOWN:
      if (menuSelection < MAIN_MENU_COUNT - 1) {
        menuSelection++;
        if (menuSelection >= menuScrollOffset + VISIBLE_ITEMS) {
          menuScrollOffset = menuSelection - VISIBLE_ITEMS + 1;
        }
      }
      break;
    case KEY_ENTER:
      switch (menuSelection) {
        case 0: navigateToScreen(SCREEN_LIVE_STATUS); break;
        case 1: menuSelection = 0; navigateToScreen(SCREEN_NODE_LIST); break;
        case 2: navigateToScreen(SCREEN_ALERT_HISTORY); break;
        case 3: menuSelection = 0; navigateToScreen(SCREEN_SETTINGS); break;
        case 4: navigateToScreen(SCREEN_NETWORK_INFO); break;
        case 5: navigateToScreen(SCREEN_REBOOT_CONFIRM); break;
        case 6:  // Reset All Nodes - only for GATEWAY_ONSHORE
          if (currentRole == ROLE_GATEWAY_ONSHORE) {
            navigateToScreen(SCREEN_RESET_ALL_CONFIRM);
          } else {
            display.clearBuffer();
            display.setFont(u8g2_font_5x7_tr);
            display.drawStr(5, 25, "Only available on");
            display.drawStr(5, 40, "Gateway (Ice)");
            display.sendBuffer();
            delay(1500);
          }
          break;
      }
      break;
    case KEY_ESC:
      navigateToScreen(SCREEN_LIVE_STATUS);
      break;
  }
}

void handleNodeListInput(char key) {
  switch (key) {
    case KEY_UP:
      if (menuSelection > 0) {
        menuSelection--;
        if (menuSelection < menuScrollOffset) menuScrollOffset = menuSelection;
      }
      break;
    case KEY_DOWN:
      if (menuSelection < network.node_count - 1) {
        menuSelection++;
        if (menuSelection >= menuScrollOffset + 4) menuScrollOffset = menuSelection - 3;
      }
      break;
    case KEY_ENTER:
      selectedNodeIdx = menuSelection;
      navigateToScreen(SCREEN_NODE_DETAILS);
      break;
    case KEY_ESC:
      menuSelection = 1;  // "Node List" in main menu
      navigateToScreen(SCREEN_MAIN_MENU);
      break;
  }
}

void handleNodeDetailsInput(char key) {
  switch (key) {
    case KEY_ESC:
    case KEY_ENTER:
      navigateToScreen(SCREEN_NODE_LIST);
      break;
    case KEY_LEFT:
      if (selectedNodeIdx > 0) selectedNodeIdx--;
      break;
    case KEY_RIGHT:
      if (selectedNodeIdx < network.node_count - 1) selectedNodeIdx++;
      break;
  }
}

void handleAlertHistoryInput(char key) {
  if (key == KEY_ESC || key == KEY_ENTER) {
    menuSelection = 2;  // "Alert History" in main menu
    navigateToScreen(SCREEN_MAIN_MENU);
  }
}

void handleSettingsInput(char key) {
  const int VISIBLE_ITEMS = 5;

  switch (key) {
    case KEY_UP:
      if (menuSelection > 0) {
        menuSelection--;
        if (menuSelection < menuScrollOffset) {
          menuScrollOffset = menuSelection;
        }
      }
      break;
    case KEY_DOWN:
      if (menuSelection < SETTINGS_MENU_COUNT - 1) {
        menuSelection++;
        if (menuSelection >= menuScrollOffset + VISIBLE_ITEMS) {
          menuScrollOffset = menuSelection - VISIBLE_ITEMS + 1;
        }
      }
      break;
    case KEY_ENTER:
      switch (menuSelection) {
        case 0:  // WiFi Config
          // Pre-fill with current credentials
          strncpy(inputSsid, storedSsid, sizeof(inputSsid) - 1);
          strncpy(inputPassword, storedPassword, sizeof(inputPassword) - 1);
          strncpy(inputBuffer, inputSsid, sizeof(inputBuffer) - 1);
          inputPos = strlen(inputBuffer);
          inputField = 0;
          navigateToScreen(SCREEN_WIFI_CONFIG);
          break;
        case 1:  // Buzzer toggle
          settings.buzzerEnabled = !settings.buzzerEnabled;
          saveSettings();
          showOverlayMessage(settings.buzzerEnabled ? "Buzzer ON" : "Buzzer OFF", 1000);
          break;
        case 2:  // Alert Hold
          editingSettingIdx = 2;
          editingValue = settings.alertHoldSec;
          navigateToScreen(SCREEN_SETTING_EDIT);
          break;
        case 3:  // Heartbeat
          editingSettingIdx = 3;
          editingValue = settings.heartbeatSec;
          navigateToScreen(SCREEN_SETTING_EDIT);
          break;
        case 4:  // Web Server toggle
          settings.webServerEnabled = !settings.webServerEnabled;
          saveSettings();
          // Show message that reboot is required
          display.clearBuffer();
          display.setFont(u8g2_font_6x10_tr);
          display.drawStr(10, 25, "Web Server:");
          display.drawStr(10, 40, settings.webServerEnabled ? "ENABLED" : "DISABLED");
          display.drawStr(10, 55, "Reboot required!");
          display.sendBuffer();
          delay(1500);
          break;
        case 5:  // WiFi Mode cycle
          // GATEWAY_ONSHORE cannot change WiFi mode (forced AP only)
          if (currentRole == ROLE_GATEWAY_ONSHORE) {
            display.clearBuffer();
            display.setFont(u8g2_font_5x7_tr);
            display.drawStr(5, 25, "Gateway on ice must");
            display.drawStr(5, 40, "use AP mode only!");
            display.sendBuffer();
            delay(1500);
          } else {
            // Cycle through modes: 0 (AP) -> 1 (STA) -> 2 (AP+STA) -> 0
            settings.wifiModeSetting = (settings.wifiModeSetting + 1) % 3;
            saveSettings();
            // Show message that reboot is required
            display.clearBuffer();
            display.setFont(u8g2_font_6x10_tr);
            const char* modeStr;
            switch (settings.wifiModeSetting) {
              case 0: modeStr = "AP Only"; break;
              case 1: modeStr = "STA Only"; break;
              case 2: modeStr = "AP+STA"; break;
              default: modeStr = "AP+STA"; break;
            }
            display.drawStr(10, 25, "WiFi Mode:");
            display.drawStr(10, 40, modeStr);
            display.drawStr(10, 55, "Reboot required!");
            display.sendBuffer();
            delay(1500);
          }
          break;
        case 6:  // Reed Polarity toggle
          settings.reedActiveHigh = !settings.reedActiveHigh;
          saveSettings();
          showOverlayMessage(settings.reedActiveHigh ? "Reed: HIGH" : "Reed: LOW", 1000);
          break;
        case 7:  // Back
          menuSelection = 3;  // "Settings" in main menu
          navigateToScreen(SCREEN_MAIN_MENU);
          break;
      }
      break;
    case KEY_ESC:
      menuSelection = 3;
      navigateToScreen(SCREEN_MAIN_MENU);
      break;
  }
}

void handleWifiConfigInput(char key) {
  switch (key) {
    case KEY_ESC:
      // Cancel without saving
      menuSelection = 0;
      navigateToScreen(SCREEN_SETTINGS);
      break;
    case KEY_ENTER:
      if (inputField == 0) {
        // Save SSID, move to password
        strncpy(inputSsid, inputBuffer, sizeof(inputSsid) - 1);
        strncpy(inputBuffer, inputPassword, sizeof(inputBuffer) - 1);
        inputPos = strlen(inputBuffer);
        inputField = 1;
      } else {
        // Save and restart
        strncpy(inputPassword, inputBuffer, sizeof(inputPassword) - 1);
        if (strlen(inputSsid) > 0) {
          saveWifiCredentials(inputSsid, inputPassword);
          display.clearBuffer();
          display.setFont(u8g2_font_6x10_tr);
          display.drawStr(25, 30, "WiFi Saved!");
          display.drawStr(20, 45, "Restarting...");
          display.sendBuffer();
          delay(2000);
          ESP.restart();
        }
      }
      break;
    case KEY_TAB:
      // Switch fields
      if (inputField == 0) {
        strncpy(inputSsid, inputBuffer, sizeof(inputSsid) - 1);
        strncpy(inputBuffer, inputPassword, sizeof(inputBuffer) - 1);
        inputPos = strlen(inputBuffer);
        inputField = 1;
      } else {
        strncpy(inputPassword, inputBuffer, sizeof(inputPassword) - 1);
        strncpy(inputBuffer, inputSsid, sizeof(inputBuffer) - 1);
        inputPos = strlen(inputBuffer);
        inputField = 0;
      }
      break;
    case KEY_BACKSP:
    case 0x7F:  // Delete
      if (inputPos > 0) {
        inputPos--;
        inputBuffer[inputPos] = '\0';
      }
      break;
    default:
      // Printable characters
      if (key >= 32 && key < 127) {
        // AUDIT FIX #6: Defense-in-depth bounds check to prevent buffer overflow
        // inputBuffer is 65 chars, max useful length is 63 (for password)
        // Add explicit bounds check in case inputPos gets corrupted
        if (inputPos >= sizeof(inputBuffer) - 1) {
          DEBUG_PRINTLN("WARN: inputPos out of bounds, rejecting character");
          break;
        }
        size_t maxLen = (inputField == 0) ? 31 : 63;
        if (inputPos < maxLen) {
          inputBuffer[inputPos++] = key;
          inputBuffer[inputPos] = '\0';
        }
      }
      break;
  }
}

void handleNetworkInfoInput(char key) {
  switch (key) {
    case KEY_UP:
    case KEY_DOWN:
      // Toggle between page 0 and page 1
      menuScrollOffset = (menuScrollOffset == 0) ? 1 : 0;
      break;
    case KEY_ESC:
    case KEY_ENTER:
      menuSelection = 4;  // "Network Info" in main menu
      menuScrollOffset = 0;  // Reset scroll when leaving
      navigateToScreen(SCREEN_MAIN_MENU);
      break;
  }
}

void handleRebootConfirmInput(char key) {
  switch (key) {
    case KEY_ENTER:
    case 'y':
    case 'Y':
      display.clearBuffer();
      display.setFont(u8g2_font_6x10_tr);
      display.drawStr(30, 35, "Rebooting...");
      display.sendBuffer();
      delay(1000);
      ESP.restart();
      break;
    case KEY_ESC:
    case 'n':
    case 'N':
      menuSelection = 5;  // "Reboot" in main menu
      navigateToScreen(SCREEN_MAIN_MENU);
      break;
  }
}

void handleResetAllConfirmInput(char key) {
  switch (key) {
    case KEY_ENTER:
    case 'y':
    case 'Y':
      // Send reset command to all nodes
      sendResetAllCommand();
      break;
    case KEY_ESC:
    case 'n':
    case 'N':
      menuSelection = 6;  // "Reset All" in main menu
      navigateToScreen(SCREEN_MAIN_MENU);
      break;
  }
}

void handleSettingEditInput(char key) {
  int minVal = 5, maxVal = 600;
  
  if (editingSettingIdx == 2) { minVal = 5; maxVal = 300; }
  else if (editingSettingIdx == 3) { minVal = 10; maxVal = 600; }
  
  switch (key) {
    case KEY_UP:
      if (editingValue < maxVal) editingValue++;
      break;
    case KEY_DOWN:
      if (editingValue > minVal) editingValue--;
      break;
    case KEY_RIGHT:
      editingValue = min((int32_t)maxVal, editingValue + 10);
      break;
    case KEY_LEFT:
      editingValue = max((int32_t)minVal, editingValue - 10);
      break;
    case KEY_ENTER:
      // Save value
      if (editingSettingIdx == 2) settings.alertHoldSec = editingValue;
      else if (editingSettingIdx == 3) settings.heartbeatSec = editingValue;
      saveSettings();
      navigateToScreen(SCREEN_SETTINGS);
      break;
    case KEY_ESC:
      navigateToScreen(SCREEN_SETTINGS);
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// NAVIGATION & SCREEN MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

void navigateToScreen(MenuScreen screen) {
  previousScreen = currentScreen;
  currentScreen = screen;
  
  // Reset scroll when entering lists
  if (screen == SCREEN_NODE_LIST || screen == SCREEN_MAIN_MENU || screen == SCREEN_SETTINGS) {
    menuScrollOffset = 0;
  }
  
  DEBUG_PRINTF("Navigate: %d -> %d\n", previousScreen, currentScreen);
}

// ═══════════════════════════════════════════════════════════════════════════
// SETTINGS PERSISTENCE
// ═══════════════════════════════════════════════════════════════════════════

void loadSettings() {
  preferences.begin("settings", true);  // Read-only

  settings.buzzerEnabled = preferences.getBool("buzzer", true);
  settings.alertHoldSec = preferences.getUShort("alertHold", 30);
  settings.heartbeatSec = preferences.getUShort("heartbeat", 60);
  settings.displayBrightness = preferences.getUChar("brightness", 255);
  settings.webServerEnabled = preferences.getBool("webServer", true);
  settings.wifiModeSetting = preferences.getUChar("wifiMode", 2);  // Default: AP+STA
  settings.reedActiveHigh = preferences.getBool("reedHigh", true);  // Default: trigger on HIGH

  preferences.end();

  DEBUG_PRINTF("Settings loaded: buzzer=%d alertHold=%d heartbeat=%d webServer=%d wifiMode=%d reedHigh=%d\n",
               settings.buzzerEnabled, settings.alertHoldSec, settings.heartbeatSec,
               settings.webServerEnabled, settings.wifiModeSetting, settings.reedActiveHigh);
}

void saveSettings() {
  preferences.begin("settings", false);  // Read-write

  preferences.putBool("buzzer", settings.buzzerEnabled);
  preferences.putUShort("alertHold", settings.alertHoldSec);
  preferences.putUShort("heartbeat", settings.heartbeatSec);
  preferences.putUChar("brightness", settings.displayBrightness);
  preferences.putBool("webServer", settings.webServerEnabled);
  preferences.putUChar("wifiMode", settings.wifiModeSetting);
  preferences.putBool("reedHigh", settings.reedActiveHigh);

  preferences.end();

  DEBUG_PRINTLN(F("Settings saved"));
}

// ═══════════════════════════════════════════════════════════════════════════
// NODE NAMING (stored in gateway NVS)
// ═══════════════════════════════════════════════════════════════════════════

/*
 * Save a custom name for a node
 * Key format: "name_XX" where XX is node_id (1-254)
 * Max name length: 15 characters
 */
void saveNodeName(uint8_t nodeId, const char* name) {
  if (nodeId == 0 || nodeId > 254) return;

  char key[12];
  snprintf(key, sizeof(key), "name_%d", nodeId);

  preferences.begin("nodenames", false);
  preferences.putString(key, name);
  preferences.end();

  // Update in-memory state
  NodeState* node = findOrCreateNode(nodeId);
  if (node) {
    strncpy(node->name, name, sizeof(node->name) - 1);
    node->name[sizeof(node->name) - 1] = '\0';
  }

  DEBUG_PRINTF("Node %d name saved: %s\n", nodeId, name);
}

/*
 * Load a saved node name from NVS
 * Returns empty string if no name saved
 */
String loadNodeName(uint8_t nodeId) {
  if (nodeId == 0 || nodeId > 254) return "";

  char key[12];
  snprintf(key, sizeof(key), "name_%d", nodeId);

  preferences.begin("nodenames", true);
  String name = preferences.getString(key, "");
  preferences.end();

  return name;
}

/*
 * Load all saved node names on boot
 * Call after nodes are discovered/created
 */
void loadAllNodeNames() {
  for (int i = 0; i < network.node_count; i++) {
    String savedName = loadNodeName(network.nodes[i].node_id);
    if (savedName.length() > 0) {
      strncpy(network.nodes[i].name, savedName.c_str(), sizeof(network.nodes[i].name) - 1);
      network.nodes[i].name[sizeof(network.nodes[i].name) - 1] = '\0';
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// ALERT MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

void recordAlert(uint8_t nodeId) {
  AlertRecord* rec = &alertHistory[alertHistoryIdx];
  rec->nodeId = nodeId;
  rec->timestamp = millis();
  rec->active = true;
  
  alertHistoryIdx = (alertHistoryIdx + 1) % ALERT_HISTORY_SIZE;
  if (alertHistoryCount < ALERT_HISTORY_SIZE) alertHistoryCount++;
  
  // Clear silence when new alert comes in
  alertsSilenced = false;
  
  DEBUG_PRINTF("Alert recorded: node %d\n", nodeId);
}

void loopAutoUnsilence() {
  // BUG FIX #7: Use silenceExpireTime for auto-unsilence check
  // Previously used silenceTime + SILENCE_AUTO_CLEAR_MS, which didn't account
  // for silence commands received from other nodes (where expiration time differs)
  if (alertsSilenced && silenceExpireTime > 0 && millis() >= silenceExpireTime) {
    DEBUG_PRINTLN("Auto-unsilencing after timeout");
    alertsSilenced = false;
    silenceTime = 0;
    silenceExpireTime = 0;

    // Broadcast unsilence to network
    sendSilenceSync();
  }
}

void silenceAlerts() {
  alertsSilenced = !alertsSilenced;  // Toggle silence state

  if (alertsSilenced) {
    silenceTime = millis();
    silenceExpireTime = millis() + SILENCE_AUTO_CLEAR_MS;
    DEBUG_PRINTF("Alerts silenced for %d seconds\n", SILENCE_AUTO_CLEAR_MS / 1000);
  } else {
    silenceTime = 0;
    silenceExpireTime = 0;
    DEBUG_PRINTLN("Alerts unsilenced manually");
  }

  // Send silence sync to network
  sendSilenceSync();
}

void sendSilenceSync() {
  DEBUG_PRINTF("Broadcasting silence state: %d\n", alertsSilenced);

  // Broadcast via ESP-NOW to local nodes
  if (espNowReady) {
    SilenceSyncMessage espMsg;
    espMsg.network_id = NETWORK_ID;
    espMsg.sender_id = NODE_ID;
    espMsg.msg_type = MSG_SILENCE_SYNC;
    espMsg.silence_state = alertsSilenced ? 1 : 0;
    espMsg.timestamp = millis() / 1000;
    espMsg.expire_time = alertsSilenced ? (silenceExpireTime / 1000) : 0;

    esp_now_send(ESPNOW_BROADCAST, (uint8_t*)&espMsg, sizeof(espMsg));
    DEBUG_PRINTLN("ESP-NOW: Silence sync sent");
  }

  // Broadcast via LoRa to remote gateways and relays
  if (loraReady) {
    LoRaSilenceSyncMessage loraMsg;
    loraMsg.network_id = NETWORK_ID;
    loraMsg.sender_id = NODE_ID;
    loraMsg.msg_type = MSG_SILENCE_SYNC;
    loraMsg.hop_count = 0;
    loraMsg.silence_state = alertsSilenced ? 1 : 0;
    loraMsg.origin_id = NODE_ID;
    loraMsg.reserved = 0;
    loraMsg.timestamp = millis() / 1000;
    loraMsg.expire_time = alertsSilenced ? (silenceExpireTime / 1000) : 0;

    // Use unified transmit helper with proper TX-complete interrupt handling
    loRaTransmitPacket((uint8_t*)&loraMsg, sizeof(loraMsg), "SILENCE");
    DEBUG_PRINTLN("LoRa: Silence sync sent");
  }
}

bool sendRemoteConfig(uint8_t targetNodeId) {
  if (!loraReady) {
    DEBUG_PRINTLN(F("LoRa not ready for config send"));
    return false;
  }

  ConfigUpdateMessage cfg;
  cfg.network_id = NETWORK_ID;
  cfg.origin_id = NODE_ID;  // We are the origin
  cfg.msg_type = MSG_CONFIG_UPDATE;
  cfg.hop_count = 0;        // Starting hop
  cfg.target_id = targetNodeId;
  cfg.config_seq = ++pendingConfigSeq;
  cfg.buzzerEnabled = settings.buzzerEnabled ? 1 : 0;
  cfg.alertHoldSec = settings.alertHoldSec;
  cfg.heartbeatSec = settings.heartbeatSec;
  cfg.reedActiveHigh = settings.reedActiveHigh ? 1 : 0;
  memset(cfg.reserved, 0, sizeof(cfg.reserved));
  cfg.checksum = calculateChecksum((uint8_t*)&cfg, sizeof(cfg) - 1);

  // Record in our own dedup cache (so we don't process our own relay)
  recordConfigForDedup(NODE_ID, cfg.config_seq);

  // Track pending config for ACK
  pendingConfigTarget = targetNodeId;
  pendingConfigTime = millis();
  pendingConfigWaiting = true;

  DEBUG_PRINTF("LoRa: Sending config to node %d, seq=%d\n", targetNodeId, cfg.config_seq);

  // Send with retry for reliability
  for (int i = 0; i < 2; i++) {
    loRaTransmitPacket((uint8_t*)&cfg, sizeof(cfg), "CONFIG");
    if (i < 1) delay(300 + random(200));
  }

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// RESET ALL NODES COMMAND
// ═══════════════════════════════════════════════════════════════════════════

void sendResetAllCommand() {
  DEBUG_PRINTLN(F("Sending RESET ALL NODES command..."));

  // Show progress on display
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(15, 20, "Sending RESET");
  display.drawStr(15, 35, "to all nodes...");
  display.sendBuffer();

  ResetCmdMessage msg;
  msg.network_id = NETWORK_ID;
  msg.sender_id = NODE_ID;
  msg.msg_type = MSG_RESET_CMD;
  msg.reset_delay_sec = 0;  // Delay managed by receivers based on role
  msg.timestamp = millis() / 1000;

  // Send via ESP-NOW to reach sensor nodes
  if (espNowReady) {
    esp_now_send(ESPNOW_BROADCAST, (uint8_t*)&msg, sizeof(msg));
    DEBUG_PRINTLN("ESP-NOW: Reset command sent");
    delay(100);  // Give it time to transmit
  }

  // Send via LoRa to reach offshore gateway and relays
  if (loraReady) {
    loRaTransmitPacket((uint8_t*)&msg, sizeof(msg), "RESET");
    DEBUG_PRINTLN("LoRa: Reset command sent");
  }

  // Update display
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(20, 25, "Reset sent!");
  display.drawStr(10, 45, "Rebooting in 3s...");
  display.sendBuffer();

  // Gateway reboots last (after 3 seconds)
  DEBUG_PRINTLN(F("Gateway will reboot in 3 seconds..."));
  delay(3000);

  DEBUG_PRINTLN(F("Gateway rebooting..."));
  Serial.flush();
  delay(100);
  ESP.restart();
}
