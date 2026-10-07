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
#include <esp_timer.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>

#include "config.h"
#include "messages.h"
#include <rx_ring.h>
#include "mesh_radio.h"
#include "screens.h"
#include <sonar_sim.h>             // v2: fake sonar for the test mode (virtual sonar nodes on hubs)
#include <sonar_params.h>          // v2 (D42): sonar processing knobs, one set for every sonar hole
static_assert(icemesh::sonar::P_COUNT <= SONAR_CTRL_PARAMS, "sonar knobs do not fit the sonar control message");

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

// v2: hardware I2C for the OLED (v1 used software I2C: one full screen took tens of ms of bit-banging
// in loop(), 5 times a second, which slowed the web page). Set OLED_HW_I2C to 0 to go back to software
// I2C if a board shows a blank screen. OLED_I2C_HZ: 400 kHz = SSD1306 fast mode (spec).
#ifndef OLED_HW_I2C
#define OLED_HW_I2C 1
#endif
#ifndef OLED_I2C_HZ
#define OLED_I2C_HZ 400000UL
#endif
#if OLED_HW_I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
#else
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);
#endif
// LoRa radio (SX1262) is owned by mesh_radio.cpp (TDMA radio task)
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
  uint8_t radioTestMode;      // v2 chalet: 0 off, 1 rotate SF9/8/7, 2 SF9, 3 SF8, 4 SF7 (500 kHz)
  bool adaptiveRadio;         // v2 chalet: per-hub adaptive SF (RSSI margin + hysteresis)
  bool sonarSim;              // v2 chalet: sonar test mode (fake sonar data on hubs and nodes)
  uint8_t sonarVirtualNodes;  // v2 hub: virtual sonar nodes generated by this hub in test mode (0-4)
  uint8_t transportMode;      // v2 chalet: 0 Auto, 1 LoRa only, 2 ESP-NOW only (hubs follow the beacon)
  uint8_t loraChannel;        // v2 chalet: 0-7 fixed, 255 = Auto (scan + move when busy)
  bool ebRelay;               // v2 any device: rebroadcast ESP-NOW backbone frames
  bool ebChaletLr;            // v2 chalet: LR on for ESP-NOW. Kills the phone hotspot (Espressif: no per-interface LR) - bench only
  uint8_t lastLoraCh;         // v2: LoRa channel last used (start point after a reboot)
  bool buzzerPassive;         // v2: passive buzzer (needs a tone) instead of an active one (sounds on DC)
  uint8_t alarmHoldMin;       // v2: FISH ON alarm after the line resets: 0 = until silenced, else minutes
  bool unitsMetric;           // v2 OLED: metres instead of feet
  uint8_t sonDepthIdx;        // v2 OLED Sonar page depth scale: 0 = auto, 1-8 = fixed (DEPTH_STEPS)
  uint8_t focusDepthIdx;      // v2 OLED Focus page depth scale: 0 = auto, 1-8 = fixed
  bool hideWeak;              // v2 OLED: do not draw the weakest echoes
  bool nearBaitBeep;          // v2 (D43): short beep when a fish comes near a bait (off by default)
};

DeviceSettings settings = {
  .buzzerEnabled = true,
  .alertHoldSec = 30,
  .heartbeatSec = 60,
  .displayBrightness = 255,
  .webServerEnabled = true,
  .wifiModeSetting = 2,       // Default: AP+STA mode
  .reedActiveHigh = true,     // Default: trigger on HIGH (magnet away)
  .radioTestMode = 0,
  .adaptiveRadio = false,     // fixed SF9/500 until range tests say otherwise
  .sonarSim = false,
  .sonarVirtualNodes = 2,
  .transportMode = 0,
  .loraChannel = 255,
  .ebRelay = false,
  .ebChaletLr = false,
  .lastLoraCh = 0,
  .buzzerPassive = false,
  .alarmHoldMin = 0,
  .unitsMetric = false,
  .sonDepthIdx = 0,
  .focusDepthIdx = 0,
  .hideWeak = false,
  .nearBaitBeep = false
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

// v2: relay dedup caches removed — duplicates are handled by the TDMA mesh (lib/IceMesh)

// ═══════════════════════════════════════════════════════════════════════════
// FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════

void setupDisplay();
void setupEspNow();
void setupWiFiAP();
void setupWebServer();
void setupBuzzer();
void setupLocalSensor();
void readBattery();

// Display sleep/wake functions
void wakeDisplay();
void sleepDisplay();
extern bool scrDirty;
void registerActivity();

void loopLocalSensor();
void loopDisplay();
static void scrDraw();
static void apName(char* out, size_t n);
void drawRadioTest();
static void scrStep(int dir);
static void scrAction(bool kb = false);
void loopOptions();
static void optBuild(ScrOptions* o);
static void menuBuild(ScrOptions* o);
void loopWebServer();
void loopBuzzer();
void loopNodeTimeout();
void checkWiFiStatus();
void verifyWiFiChannel();          // BUG FIX #6: WiFi channel verification

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);
void loopEspNowRx();
volatile bool sonarCtrlKick = false;
uint32_t sonarTickUsMax = 0, sonarTickUsSum = 0, sonarTickCount = 0;   // hub: CPU cost of one fake ping (scene + processing + codec)
void meshLoop();
void meshSyncNodesNow();
int findNodeIndexForMesh(uint8_t nodeId);
void sendSilenceSyncEspNow();
void drawRadioTest();
void handleWebApiRadio();
void handleWebApiRadioPost();
void handleWebApiSim();
void handleWebApiSimPost();

void processEspNowMessage(const uint8_t* data, int len, int rssi);
void setupEbChalet();
static bool ebSend(const uint8_t* frame, size_t len);
static void devCmdOnNodeHeard(uint8_t node);
static void devCmdQueue(uint8_t node, uint8_t cmd, uint8_t value);
static void relayRequest(uint8_t dev, bool on);
void simRequest(uint8_t node, bool sonar, bool hall);
void simAll(bool on);
void demoSet(uint8_t hubs, uint8_t holes);
bool simAnyOn();
static inline uint8_t simValueOf(bool sonar, bool hall, uint8_t tph) {
  // 62 max: 63 with sonar + fish would make 0xFF, the "not a demo hole" mark of meshDemoSim()
  return static_cast<uint8_t>((sonar ? MESH_SIM_SONAR : 0) | (hall ? MESH_SIM_HALL : 0) | ((tph > 62 ? 62 : tph) << 2));
}

// Deduplication functions
bool shouldAcceptMessage(uint8_t nodeId, uint16_t seq, uint32_t uptime, uint8_t newFlags);
bool canClearFishOn(uint8_t nodeId);
NodeState* findOrCreateNode(uint8_t nodeId);

void updateNodeState(uint8_t nodeId, uint8_t flags, uint16_t batteryMv, uint16_t seq, uint32_t uptime, int8_t rssi);

void updateDisplay();
void triggerBuzzer(uint8_t pattern);
// v2 OLED depth scales (Sonar / Focus pages): index 0 = auto, 1-8 = fixed, in the display unit
static const uint8_t DEPTH_FT[8] = {5, 10, 15, 20, 25, 30, 40, 60};
static const uint8_t DEPTH_M[8] = {2, 3, 4, 5, 6, 8, 10, 12};
uint16_t depthStepCm(uint8_t idx);
void depthStepText(uint8_t idx, char* out, size_t n);
// v2 (D42) sonar knobs: kept in NVS ("sonar"), chalet sends them in the beacon, hubs pass them to tip-ups
icemesh::sonar::Params sonarPrm;
uint32_t sonarPrmChangedMs = 0;   // hub: when the knobs last changed (sonar control repeats them for 1 min)
uint16_t sonarPrmGen = 0;         // bumps on every change: local fake sonars re-apply
void sonarKnobsLoad();
void sonarKnobApply(uint8_t id, uint8_t v);   // set + save + apply here (hub: from the beacon)
void sonarKnobSet(uint8_t id, uint8_t v);     // chalet: set + send to every sonar hole
void sonarKnobsResend();                      // chalet: send the whole set again
void sonarKnobsNewHubs();
// v2 (D43) bait depth per hole, 5 cm steps (0 = not set: the processing keeps its default)
uint8_t baitCm5[256];
uint16_t baitGen = 0;
void baitLoad();
void baitSet(uint8_t hole, uint8_t v5);      // chalet: save + send (CMD_SET_BAIT) + demo
void baitApply(uint8_t hole, uint8_t v5);    // save + use here (hub: own / test holes, or forward to the tip-up)
void baitResendAll();
void buzzerStop();
void buzzerTest();
void buzzerApplyType();
void handleWebRoot();
void handleWebApi();
void handleWebApiSonar();
void handleWebApiSonarPost();
void handleWebApiKnobs();
void handleWebApiKnobsPost();
void handleWebApiSonarPings();
void handleWebApiSonarBg();
void sonarHubLoop();
static uint8_t hubHoleSim(uint8_t node);
static bool hubSimTripped(uint8_t node);
static void hubSimApply(uint8_t target, uint8_t value);
void handleWebWifiConfig();
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
// v2 PRG button state (see loopButton)
static uint32_t buttonHeldMs = 0;            // > 0 while the button is held (display shows the hold bar)
static uint32_t connectInfoUntil = 0;        // show the "how to connect" screen until then
static uint32_t hubHotspotUntil = 0;         // hub hotspot auto-off time (0 = off)
static void drawConnectInfo();
void hubHotspotOn();
void hubHotspotOff();
void resetNetworkSettings();
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
void handleWebRemoteConfig();
void handleWebApiRemoteConfig();
void handleWebApiRemoteConfigStatus();

// Prototypes the Arduino IDE generated automatically for the .ino (needed since the move to .cpp)
void updateAlertState();
bool nodeAlarm(const NodeState& n);
void alarmStart(NodeState* n);
void tripRecord(uint8_t node);
void nearBaitWatch();
extern uint8_t nearBaitNode;
extern uint32_t nearBaitAt;
void alarmAck();
void alarmLineReset(NodeState* n);
void loopAlarmHold();
void handleWebApiNodeName();
void handleWebApiNodeBait();
void handleWebApiTrips();
// v2 (D43) trip recordings (see tripRecord)
struct TripRec { uint8_t node; uint32_t at_s; uint8_t n; MeshBaseRec r[30]; };
TripRec tripRecs[8];
uint8_t tripHead = 0, tripCount = 0;
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
  Serial.printf("Firmware:    v2, built %s %s\n", __DATE__, __TIME__);
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
  sonarKnobsLoad();
  baitLoad();
  
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
  // v2: TDMA LoRa mesh (own FreeRTOS task). Chalet = GATEWAY_OFFSHORE, every other role is a hub.
  if (currentRole == ROLE_GATEWAY_OFFSHORE) {
    meshSetTestMode(settings.radioTestMode);
    meshSetAdaptive(settings.adaptiveRadio);
    meshSetSonarSim(settings.sonarSim);
    meshSetTransport(settings.transportMode);
    meshSetLoraChannel(settings.loraChannel);
  }
  meshSetStartChannel(settings.lastLoraCh);
  meshSetEbRelay(settings.ebRelay);
  loraReady = meshBegin(NODE_ID, currentRole == ROLE_GATEWAY_OFFSHORE, NETWORK_ID);
  Serial.printf("LoRa mesh: %s, %s\n", loraReady ? "radio OK" : "RADIO INIT FAILED",
                currentRole == ROLE_GATEWAY_OFFSHORE ? "chalet (beacon master)" : "hub");
  
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
      setupEbChalet();   // v2: ESP-NOW backbone (the chalet had no ESP-NOW before)
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

  if (espNowReady) meshSetEbSender(ebSend);   // v2: backbone frames go out as ESP-NOW broadcasts

  // Setup user button for silencing (GPIO0 on Heltec V3)
  pinMode(USER_BUTTON, INPUT_PULLUP);
  DEBUG_PRINTLN(F("User button (GPIO0) enabled for silencing"));

  // Startup display
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tr);
  display.drawStr(0, 11, "Ice Fishing v2");
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 27, currentRole == ROLE_GATEWAY_OFFSHORE ? "Chalet (offshore)" :
                         currentRole == ROLE_GATEWAY_ONSHORE ? "Hub (on the ice)" : getRoleName(currentRole));
  display.drawStr(0, 60, cardKbAvailable ? "Ready - Esc = menu" : "Ready");
  display.sendBuffer();
  
  triggerBuzzer(1);
  delay(1000);
  
  Serial.println(F("Setup complete\n"));
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

// v2 loop timing (serial PERF): where loop() spends its time, to find what makes the page or screen slow
enum PerfSlot : uint8_t { PF_LOOP = 0, PF_DISPLAY, PF_WEB, PF_DEMO, PF_MESH, PF_COUNT };
struct PerfAcc { uint64_t sum_us; uint32_t max_us, n; };
static PerfAcc perfAcc[PF_COUNT];
static uint32_t perfSince = 0;
struct PerfScope {
  uint8_t i; uint32_t t0;
  explicit PerfScope(uint8_t k) : i(k), t0(micros()) {}
  ~PerfScope() { const uint32_t d = micros() - t0; PerfAcc& a = perfAcc[i]; a.sum_us += d; if (d > a.max_us) a.max_us = d; a.n++; }
};
void perfPrint() {
  static const char* const NAMES[PF_COUNT] = {"loop", "display", "web", "demo", "mesh"};
  const uint32_t ms = millis() - perfSince;
  Serial.printf("Loop timing over %lu s (avg / max per call, share of time):\n", (unsigned long)(ms / 1000));
  for (uint8_t i = 0; i < PF_COUNT; i++) {
    const PerfAcc& a = perfAcc[i];
    Serial.printf("  %-8s %6lu calls  %7.2f ms / %7.2f ms  %5.1f %%\n", NAMES[i], (unsigned long)a.n,
                  a.n ? (float)a.sum_us / 1000.0f / a.n : 0.0f, a.max_us / 1000.0f, ms ? (float)a.sum_us / 10.0f / ms : 0.0f);
  }
  Serial.printf("  free heap %u B, min %u B\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  memset(perfAcc, 0, sizeof(perfAcc)); perfSince = millis();
}

void loop() {
  PerfScope perfLoop(PF_LOOP);
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
  loopOptions(); // v2: Wi-Fi scan started from Options > Wi-Fi

  // ESP-NOW frames queued by the Wi-Fi task callback (C9)
  if (espNowReady) loopEspNowRx();

  // v2: TDMA mesh <-> node table, silence, commands, counters (the radio runs in its own task)
  { PerfScope p(PF_MESH); meshLoop(); }

  // Check for auto-unsilence timeout
  loopAutoUnsilence();
  loopAlarmHold();

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
      if (wifiApActive && settings.webServerEnabled) loopWebServer();   // v2: hub hotspot on demand (PRG hold 3 s)
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

      // v2: no periodic LoRa aggregate — this hub sends its line states in its TDMA slot
      break;

    case ROLE_GATEWAY_OFFSHORE:
      { PerfScope p(PF_DEMO); meshDemoTick(); }   // v2: demo network (fake hubs inside this box), when on
      sonarKnobsNewHubs();   // v2 (D42): a hub that shows up gets the sonar knobs
      nearBaitWatch();       // v2 (D43)
      if (settings.webServerEnabled) { PerfScope p(PF_WEB); loopWebServer(); }
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

      // v2: the chalet transmits the beacon from the radio task (no aggregates)
      break;

    case ROLE_SENSOR_LORA:
      loopLocalSensor();
      break;
      
    case ROLE_RELAY_LORA:
      break;
  }
  
  // Display current screen
  { PerfScope p(PF_DISPLAY); loopDisplay(); }
  loopBuzzer();
  loopNodeTimeout();

  // Radio status on serial: every 5 s in radio test mode, every 30 s with DEBUG_SERIAL
  static unsigned long lastHealthPrint = 0;
  const unsigned long healthEvery = meshTestMode() != 0 ? 5000UL : 30000UL;
  if ((DEBUG_SERIAL || meshTestMode() != 0) && millis() - lastHealthPrint > healthEvery) {
    lastHealthPrint = millis();
    meshPrintStatus(Serial);
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

  // Initialize display (I2C pins from the constructor; hardware I2C: bus clock first)
#if OLED_HW_I2C
  display.setBusClock(OLED_I2C_HZ);
#endif
  display.begin();
  // v2 boot screen: firmware generation, role and build date, so the board says what it runs
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tr);
  display.drawStr(0, 11, "Ice Fishing v2");
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(0, 27, currentRole == ROLE_GATEWAY_OFFSHORE ? "Chalet (offshore)" :
                         currentRole == ROLE_GATEWAY_ONSHORE ? "Hub (on the ice)" : getRoleName(currentRole));
  {
    char b[32]; snprintf(b, sizeof(b), "built %.6s %.5s", __DATE__, __TIME__);
    display.drawStr(0, 41, b);
  }
  display.drawStr(0, 60, "Starting...");
  display.sendBuffer();

  DEBUG_PRINTLN(F("Display ready"));
}

void wakeDisplay() {
  if (!displaySleeping) return;
  scrDirty = true;

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
  if (!displaySleeping && currentScreen == SCREEN_LIVE_STATUS && meshTestMode() == 0) {
    if (millis() - lastActivityTime > DISPLAY_SLEEP_MS) {
      sleepDisplay();
      return;
    }
  }

  // Don't update display if sleeping
  if (displaySleeping) return;
  if (buttonHeldMs > 0) return;   // the hold bar owns the screen while the button is held

  // Rate limit display updates
  if (millis() - lastDisplayUpdate < 200) return;
  lastDisplayUpdate = millis();

  scrDraw();   // v2 screens (the v1 menu screens below are no longer reached)
}

// Route to appropriate screen drawing function
void drawCurrentScreen() {
  switch (currentScreen) {
    case SCREEN_LIVE_STATUS:    if (meshTestMode() != 0) drawRadioTest(); else drawLiveStatus(); break;
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
  scrDirty = true;
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



// ═══════════════════════════════════════════════════════════════════════════
// RELIABLE LORA RECEIVE MODE - Forces state machine reset
// ═══════════════════════════════════════════════════════════════════════════





// ═══════════════════════════════════════════════════════════════════════════
// LORA TRANSMIT WITH BUSY WAIT - Wait for TX complete before returning
// ═══════════════════════════════════════════════════════════════════════════



// ═══════════════════════════════════════════════════════════════════════════
// UNIFIED LORA TRANSMIT - Handles TX with proper state cleanup
// ═══════════════════════════════════════════════════════════════════════════



// ═══════════════════════════════════════════════════════════════════════════
// LORA WATCHDOG - Detects and recovers stuck radio
// ═══════════════════════════════════════════════════════════════════════════



















// ═══════════════════════════════════════════════════════════════════════════
// SEND LORA AGGREGATE WITH RETRY - For reliable status change propagation
// ═══════════════════════════════════════════════════════════════════════════
// Sends aggregate multiple times to ensure offshore gateway receives
// status changes even if some packets are lost.





// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW
// ═══════════════════════════════════════════════════════════════════════════

void setupEspNow() {
  DEBUG_PRINTLN(F("Initializing ESP-NOW..."));

  // ═══════════════════════════════════════════════════════════════════════════
  // GATEWAY_ONSHORE: Use STA-only mode with Long Range protocol
  // Other roles: Use AP+STA for normal WiFi compatibility
  // ═══════════════════════════════════════════════════════════════════════════
  // v2: the ice hub runs ESP-NOW on its station interface, no AP (the hub hotspot is on demand, step 2).
  if (currentRole == ROLE_GATEWAY_ONSHORE) {
    DEBUG_PRINTLN(F("GATEWAY_ONSHORE: station mode for ESP-NOW (no AP)"));
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

    #if ESPNOW_LONG_RANGE_MODE
    // range-comparison build only (config.h): LR on the station, no hotspot possible on this board
    esp_err_t lrResult = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    if (lrResult != ESP_OK) {
      DEBUG_PRINTF("WARNING: Failed to enable LR mode: %d\n", lrResult);
    } else {
      DEBUG_PRINTLN(F("Long Range (LR) mode enabled"));
    }
    #endif
  } else {
    // Other roles use standard AP+STA mode
    WiFi.mode(WIFI_AP_STA);
  }

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

// =============================================================================================
// v2 ESP-NOW BACKBONE (lib/IceMesh/src/eb_link.h, docs/protocol_v2.md §6c)
// Espressif (esp-idf #4554): "currently we don't support to set softAP and STA in LR mode separately" -
// with LR enabled on either interface the AP beacons in LR and phones cannot find the hotspot.
// So the chalet (phone hotspot) never enables LR by default: its ESP-NOW runs at the normal rate
// (802.11b 1 Mbps) and only reaches devices that are NOT LR-only (ESPNOW_LONG_RANGE_MODE false on the
// ice side). EBMODE LR turns LR on for bench tests only: the phone hotspot then disappears.
// =============================================================================================
static wifi_interface_t ebIf = WIFI_IF_STA;

void setupEbChalet() {
  const wifi_mode_t m = WiFi.getMode();
  if (m == WIFI_MODE_NULL) { DEBUG_PRINTLN(F("Backbone: Wi-Fi off, no ESP-NOW")); return; }
  // LR must never be enabled on the AP interface: an LR-enabled AP sends its beacon in LR and phones
  // can no longer see/join it (ESP-IDF Wi-Fi guide, "LR Compatibility"). LR goes on the station
  // interface only, so an AP-only chalet gets its (unconnected) station interface added.
  if (m == WIFI_MODE_AP) { WiFi.mode(WIFI_AP_STA); delay(100); }
  ebIf = WIFI_IF_STA;
  esp_err_t e = ESP_OK;
  // order matters (esp-idf #9933 / #11751): protocol after Wi-Fi start, ESP-NOW rate after esp_now_init
  if (settings.ebChaletLr) {
    e = esp_wifi_set_protocol(ebIf, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    if (e != ESP_OK) DEBUG_PRINTF("Backbone: set protocol failed: %d\n", e);
    Serial.println(F("WARNING backbone: LR on - the phone hotspot is expected to disappear (EBMODE NORMAL to undo)"));
  }
  uint8_t primary; wifi_second_chan_t secondary;
  esp_wifi_get_channel(&primary, &secondary);
  if (primary != ESPNOW_CHANNEL) {
    Serial.printf("WARNING backbone: Wi-Fi is on channel %u, hubs use %u (station joined a router?)\n", primary, ESPNOW_CHANNEL);
  }
  if (esp_now_init() != ESP_OK) { Serial.println(F("Backbone: ESP-NOW init failed")); return; }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ESPNOW_BROADCAST, 6);
  peer.channel = 0;   // v2: 0 = the current Wi-Fi channel (the router's when the chalet joined one)
  peer.ifidx = ebIf;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) { Serial.println(F("Backbone: broadcast peer failed")); return; }
  if (settings.ebChaletLr) {
    e = esp_wifi_config_espnow_rate(ebIf, WIFI_PHY_RATE_LORA_250K);
    if (e != ESP_OK) Serial.printf("Backbone: LR rate not set (%d)\n", e);
  }
  espNowReady = true;
  Serial.printf("Backbone: ESP-NOW on the station interface, %s\n", settings.ebChaletLr ? "LR (bench test)" : "normal rate");
  #if ESPNOW_LONG_RANGE_MODE
  if (!settings.ebChaletLr) Serial.println(F("NOTE backbone: hubs/tip-ups built with ESPNOW_LONG_RANGE_MODE (LR only) cannot hear the chalet"));
  #endif
}

static bool ebSend(const uint8_t* frame, size_t len) {
  return espNowReady && esp_now_send(ESPNOW_BROADCAST, frame, len) == ESP_OK;
}

// Hub: commands for tip-up nodes (relay on/off). A sleeping node only listens right after it
// transmits, so the command goes out after each of its next 3 messages (it is idempotent).
struct PendingDevCmd { uint8_t node, cmd, value, sends; uint32_t since; };
static PendingDevCmd devCmds[8];

static void devCmdQueue(uint8_t node, uint8_t cmd, uint8_t value) {
  PendingDevCmd* slot = nullptr;
  for (auto& d : devCmds) if (d.node == node && d.cmd == cmd) slot = &d;
  for (auto& d : devCmds) if (slot == nullptr && d.node == 0) slot = &d;
  if (slot == nullptr) slot = &devCmds[0];
  slot->node = node; slot->cmd = cmd; slot->value = value; slot->sends = 0; slot->since = millis();
}

static void devCmdOnNodeHeard(uint8_t node) {
  for (auto& d : devCmds) {
    if (d.node == 0) continue;
    if (millis() - d.since > 3600000UL) { d.node = 0; continue; }   // node never showed up: drop after 1 h
    if (d.node != node) continue;
    DevCmdMessage m;
    m.network_id = NETWORK_ID; m.sender_id = NODE_ID; m.msg_type = MSG_DEV_CMD;
    m.target = node; m.cmd = d.cmd; m.value = d.value;
    esp_now_send(ESPNOW_BROADCAST, (uint8_t*)&m, sizeof(m));
    if (++d.sends >= 3) d.node = 0;
  }
}

// Chalet: relay settings requested from this page (hubs report theirs; tip-ups do not).
struct RelayReq { uint8_t dev; bool on; };
static RelayReq relayReqs[16];

static void relayRequest(uint8_t dev, bool on) {
  RelayReq* slot = nullptr;
  for (auto& r : relayReqs) if (r.dev == dev) slot = &r;
  for (auto& r : relayReqs) if (slot == nullptr && r.dev == 0) slot = &r;
  if (slot == nullptr) slot = &relayReqs[0];
  slot->dev = dev; slot->on = on;
  meshSetDeviceRelay(dev, on);
}

// Chalet: simulation requested per hole (the hole reports what it really runs with LF_SIM -> FLAG_SIM).
struct SimReq { uint8_t node; uint8_t value; };
static SimReq simReqs[48];
static bool chSimAllSet = false;
static uint8_t chSimAllValue = 0;
static uint8_t simRate = 6;                  // fake trips per hour (Hall simulation)

static uint8_t simRequested(uint8_t node) {
  for (const auto& r : simReqs) if (r.node == node) return r.value;
  return chSimAllSet ? chSimAllValue : 0;
}

void simRequest(uint8_t node, bool sonar, bool hall) {
  const uint8_t v = (sonar || hall) ? simValueOf(sonar, hall, simRate) : 0;
  if (node == 255) meshDemoSetSim(255, v);                              // demo holes: at once (they live here)
  else if (meshDemoSim(node) != 0xFF) { meshDemoSetSim(node, v); return; }
  if (node == 255) {
    chSimAllSet = true; chSimAllValue = v;
    memset(simReqs, 0, sizeof(simReqs));
  } else {
    SimReq* slot = nullptr;
    for (auto& r : simReqs) if (r.node == node) slot = &r;
    for (auto& r : simReqs) if (slot == nullptr && r.node == 0) slot = &r;
    if (slot == nullptr) slot = &simReqs[0];
    slot->node = node; slot->value = v;
  }
  meshSetHoleSim(node, v);
}

// OLED / serial: everything on (test holes on the hubs + fake sonar and trips on every hole) or off
void simAll(bool on) {
  settings.sonarSim = on; saveSettings(); meshSetSonarSim(on);
  simRequest(255, on, on);
}

// v2 demo network (chalet): fake hubs + holes inside this box (mesh_radio.cpp meshDemo*). Turning it
// off or smaller removes the demo holes from the tables, so no offline ghosts stay on the pages.
void demoSet(uint8_t hubs, uint8_t holes) {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) return;
  meshDemoSet(hubs, holes);
  int w = 1;
  for (int i = 1; i < network.node_count; i++) {
    const NodeState& n = network.nodes[i];
    const bool gone = meshDemoNode(n.node_id) && meshDemoSim(n.node_id) == 0xFF;
    if (!gone) { if (w != i) network.nodes[w] = network.nodes[i]; w++; }
  }
  for (int i = w; i < network.node_count; i++) memset(&network.nodes[i], 0, sizeof(NodeState));
  network.node_count = w;
  updateAlertState();
  Serial.printf("Demo network: %u hub(s) x %u hole(s)%s\n", meshDemoHubs(), meshDemoHoles(), meshDemoHubs() ? "" : " - off");
}

bool simAnyOn() {
  if (settings.sonarSim) return true;
  for (int i = 1; i < network.node_count; i++) {
    const uint8_t dv = meshDemoSim(network.nodes[i].node_id);   // demo hole: always SIM-flagged, use what it runs
    if (dv != 0xFF) { if (dv & 3) return true; continue; }
    if (HAS_FLAG(network.nodes[i].flags, FLAG_SIM)) return true;
  }
  return false;
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
  if (msgType == MSG_EB_BEACON || msgType == MSG_EB_HUB) { meshEbReceive(data, (size_t)len); return; }   // v2 backbone
  if (currentRole == ROLE_GATEWAY_OFFSHORE) return;   // the chalet only uses the backbone
  if (nodeId == NODE_ID) return;
  if (msgType == MSG_STATUS || msgType == MSG_ALERT || msgType == MSG_HEARTBEAT) devCmdOnNodeHeard(nodeId);

  // v2 sonar: a node just transmitted, so it listens for a moment: answer with the sonar control
  if (msgType != MSG_SONAR && msgType != MSG_SONAR_CTRL) sonarCtrlKick = true;

  switch (msgType) {
    case MSG_SONAR: {
      // one block per frame; the block's own node ID must be the sender (no spoofed holes)
      if (currentRole != ROLE_GATEWAY_OFFSHORE && len > 5 && len - 3 <= 120 && data[3] == nodeId) {
        meshHubPushSonar(data + 3, (uint8_t)(len - 3));
      }
      break;
    }
    case MSG_SONAR_CTRL:
      break;   // from another hub: nodes act on it, hubs follow their own beacon

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

          // v2: the change reaches the chalet in this hub's next TDMA slot (line-state record,
          // latched until the beacon acks it)
          if (statusChanged) meshSyncNodesNow();
        }
      } else {
        DEBUG_PRINTF("ESP-NOW: Rejected stale/dup from node %d seq=%d\n", msg->node_id, msg->sequence);
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

      // v2: forwarded to the chalet as a line-state event in this hub's next TDMA slot
      meshSyncNodesNow();
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
        buzzerStop(); alarmAck();   // v2: silenced elsewhere = acknowledged here too
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

      // v2: ask the chalet to silence the whole network (flag in this hub's next packets)
      if (loraReady) meshRequestSilence(alertsSilenced);
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

// v2: index of an existing node entry, -1 if unknown (no creation)
int findNodeIndexForMesh(uint8_t nodeId) {
  for (int i = 0; i < network.node_count; i++) if (network.nodes[i].node_id == nodeId) return i;
  return -1;
}

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









// Update node state (called only after shouldAcceptMessage returns true)
void updateNodeState(uint8_t nodeId, uint8_t flags, uint16_t batteryMv, uint16_t seq, uint32_t uptime, int8_t rssi) {
  NodeState* node = findOrCreateNode(nodeId);
  if (!node) return;

  bool wasFishOn = HAS_FLAG(node->flags, FLAG_FISH_ON);
  bool nowFishOn = HAS_FLAG(flags, FLAG_FISH_ON);
  
  // Track when FISH_ON started and record alert
  if (nowFishOn && !wasFishOn) {
    node->fish_on_time = millis();
    alarmStart(node);
    node->clear_confirm_count = 0;  // Reset clear counter on new alert
    recordAlert(nodeId);  // Add to alert history
    alertsSilenced = false;  // New alert clears silence
    registerActivity();  // Wake display on FISH_ON alert
    DEBUG_PRINTF("Node %d: FISH_ON started at %lu\n", nodeId, node->fish_on_time);
  } else if (!nowFishOn && wasFishOn) {
    node->fish_on_time = 0;
    alarmLineReset(node);
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

// v2 FISH ON alarm latch (D41): a trip starts an alarm that stays (screen, buzzer, phone) after the line
// resets, until someone silences it - or, with Alarm hold = N min, N min after the trip. Silencing
// acknowledges: holes whose line is back to normal are cleared, holes still tripped stay shown (silenced).
// A new trip while silenced turns the sound back on. An offline hole keeps its alarm (tip-up dragged?).
bool nodeAlarm(const NodeState& n) { return HAS_FLAG(n.flags, FLAG_FISH_ON) || n.alarm_ms != 0; }

void alarmStart(NodeState* n) {
  n->alarm_ms = millis(); if (n->alarm_ms == 0) n->alarm_ms = 1;   // (recordAlert turns the sound back on)
  n->alarm_acked = false;
  tripRecord(n->node_id);   // v2 (D43): keep the sonar of the minute before
}

void alarmAck() {
  for (int i = 0; i < network.node_count; i++) {
    if (!HAS_FLAG(network.nodes[i].flags, FLAG_FISH_ON)) network.nodes[i].alarm_ms = 0;
    else network.nodes[i].alarm_acked = true;   // still tripped: shown until the line resets, then gone
  }
  updateAlertState();
}
// the line of a hole went back to normal: an alarm silenced during the trip ends here (D47)
void alarmLineReset(NodeState* n) {
  if (n->alarm_acked) { n->alarm_ms = 0; n->alarm_acked = false; }
}

void loopAlarmHold() {
  static uint32_t last = 0;
  if (settings.alarmHoldMin == 0 || millis() - last < 1000) return;
  last = millis();
  const uint32_t hold = settings.alarmHoldMin * 60000UL;
  bool changed = false;
  for (int i = 0; i < network.node_count; i++) {
    NodeState& n = network.nodes[i];
    if (n.alarm_ms && !HAS_FLAG(n.flags, FLAG_FISH_ON) && millis() - n.alarm_ms > hold) { n.alarm_ms = 0; changed = true; }
  }
  if (changed) updateAlertState();
}

void updateAlertState() {
  activeAlerts = false;
  for (int i = 0; i < network.node_count; i++) {
    const NodeState& n = network.nodes[i];
    if ((n.online && HAS_FLAG(n.flags, FLAG_FISH_ON)) || n.alarm_ms != 0) {
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
      alarmStart(&network.nodes[0]);
      DEBUG_PRINTF("LOCAL: Fish on! seq=%d\n", localSequence);
      triggerBuzzer(3);

      // Update our own state immediately
      network.nodes[0].last_seq = localSequence;
      network.nodes[0].battery_mv = localBatteryMv;
      network.nodes[0].last_seen = millis();
      network.nodes[0].last_uptime = millis() / 1000;

      // Record in alert history
      recordAlert(NODE_ID);

      // v2: reported as a line-state event in the next TDMA slot (latched until acked)
      meshSyncNodesNow();
    } else {
      localFishOn = false;
      CLEAR_FLAG(network.nodes[0].flags, FLAG_FISH_ON);
      network.nodes[0].fish_on_time = 0;  // Clear fish_on_time
      alarmLineReset(&network.nodes[0]);
      DEBUG_PRINTF("LOCAL: Reset seq=%d\n", localSequence);

      // Update our own state
      network.nodes[0].last_seq = localSequence;
      network.nodes[0].battery_mv = localBatteryMv;
      network.nodes[0].last_seen = millis();
      network.nodes[0].last_uptime = millis() / 1000;

      meshSyncNodesNow();
    }

    updateAlertState();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// BUZZER
// ═══════════════════════════════════════════════════════════════════════════

// v2: the beeps are timed by an esp_timer (10 ms tick), not by loop(). Before, loop() switched the pin,
// so every pause of loop() (screen transfer, web request, the 1 s message boxes, the Wi-Fi wait) made
// beeps longer or uneven, and a beep could stay on for a whole message box. Also fixed: on/off times
// were swapped (100 ms on / 200 ms off instead of 200 / 100).
// Buzzer type (Settings > Buzzer type, saved): active = sounds when powered (DC on the pin);
// passive = needs a tone (LEDC square wave). A passive buzzer driven with DC only clicks; an active one
// driven with a tone sounds rough. Pick the one that sounds clean with Settings > Buzzer test.
#ifndef BUZZER_TONE_HZ
#define BUZZER_TONE_HZ 2700   // typical resonance of small passive buzzers (est.; the part's datasheet wins)
#endif
static const uint8_t BUZZ_LEDC_CH = 6;   // LEDC channel for the passive tone (free in this firmware)
static esp_timer_handle_t buzzTimer = nullptr;
static portMUX_TYPE buzzMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t buzzSteps = 0;   // halves left: even = next half is ON
static volatile uint16_t buzzLeftMs = 0;
static volatile bool buzzPinOn = false;
static bool buzzLedc = false;

static void buzzPin(bool on) {
  if (buzzLedc) ledcWriteTone(BUZZ_LEDC_CH, on ? BUZZER_TONE_HZ : 0);
  else digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
  buzzPinOn = on;
}

static void buzzTick(void*) {   // esp_timer task, every 10 ms
  uint8_t steps; uint16_t left;
  portENTER_CRITICAL(&buzzMux);
  if (buzzLeftMs > 10) { buzzLeftMs = (uint16_t)(buzzLeftMs - 10); portEXIT_CRITICAL(&buzzMux); return; }
  steps = buzzSteps;
  if (steps) { buzzSteps = (uint8_t)(steps - 1); buzzLeftMs = (steps % 2 == 0) ? BUZZER_BEEP_ON_MS : BUZZER_BEEP_OFF_MS; }
  left = buzzLeftMs;
  portEXIT_CRITICAL(&buzzMux);
  (void)left;
  const bool on = steps != 0 && steps % 2 == 0;
  if (on != buzzPinOn) buzzPin(on);
}

static void buzzStart(uint8_t beeps) {
  portENTER_CRITICAL(&buzzMux);
  buzzSteps = (uint8_t)(beeps * 2); buzzLeftMs = 0;
  portEXIT_CRITICAL(&buzzMux);
}

void buzzerStop() {
  portENTER_CRITICAL(&buzzMux);
  buzzSteps = 0; buzzLeftMs = 0;
  portEXIT_CRITICAL(&buzzMux);
}

// active <-> passive (also at boot)
void buzzerApplyType() {
  buzzerStop();
  if (buzzLedc) { ledcWriteTone(BUZZ_LEDC_CH, 0); ledcDetachPin(BUZZER_PIN); buzzLedc = false; }
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  if (settings.buzzerPassive) {
    ledcSetup(BUZZ_LEDC_CH, BUZZER_TONE_HZ, 8);
    ledcAttachPin(BUZZER_PIN, BUZZ_LEDC_CH);
    ledcWriteTone(BUZZ_LEDC_CH, 0);
    buzzLedc = true;
  }
  buzzPinOn = false;
}

void setupBuzzer() {
  buzzerApplyType();
  if (buzzTimer == nullptr) {
    esp_timer_create_args_t a = {};
    a.callback = buzzTick; a.name = "buzz";
    if (esp_timer_create(&a, &buzzTimer) == ESP_OK) esp_timer_start_periodic(buzzTimer, 10000);
  }
}

// Settings > Buzzer test / serial BUZZ: two beeps, even when the buzzer is off or alerts are silenced
void buzzerTest() { buzzStart(2); lastBuzzerTime = millis(); }

void triggerBuzzer(uint8_t pattern) {
  // Respect buzzer enabled setting
  if (!settings.buzzerEnabled) return;
  
  // Respect silence state (but only for alert patterns, not startup beep)
  if (alertsSilenced && pattern > 1) return;
  
  buzzStart(pattern);
  lastBuzzerTime = millis();
}

// loop(): only decides when an alert pattern repeats (the timer plays it)
void loopBuzzer() {
  if (buzzSteps != 0) { lastBuzzerTime = millis(); return; }   // still playing: the pause starts after it
  if (activeAlerts && !alertsSilenced && settings.buzzerEnabled &&
      millis() - lastBuzzerTime > BUZZER_REPEAT_DELAY_MS) {
    buzzStart(BUZZER_ALERT_BEEPS);
    lastBuzzerTime = millis();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// WIFI SETUP (AP, STA, or AP+STA mode)
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// WIFI CREDENTIAL MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════

void loadWifiCredentials() {
  // v2: the Wi-Fi written in config.h wins once after it is changed and flashed (before: a network
  // saved earlier from the web page / serial / OLED always won, so a new config.h was silently ignored).
  // Afterwards the OLED / web page / serial can change it again; "Forget network" saves an empty one.
  uint32_t h = 2166136261u;   // FNV-1a of the config.h name + password
  for (const char* c = STA_SSID "\x1f" STA_PASSWORD; *c; c++) { h ^= (uint8_t)*c; h *= 16777619u; }
  preferences.begin("wifi", false);
  if (preferences.getUInt("cfgHash", 0) != h) {
    const bool first = !preferences.isKey("cfgHash");
    const bool saved = preferences.getString("ssid", "").length() > 0;
    if (STA_SSID[0] && !(first && saved)) {   // never an empty config.h name; a board's first v2 boot keeps its saved network
      preferences.putString("ssid", STA_SSID);
      preferences.putString("pass", STA_PASSWORD);
      Serial.printf("Wi-Fi: config.h network changed -> using %s\n", STA_SSID);
    }
    preferences.putUInt("cfgHash", h);
  }
  String ssid = preferences.getString("ssid", "");
  String pass = preferences.getString("pass", "");
  preferences.end();

  memset(storedSsid, 0, sizeof(storedSsid)); memset(storedPassword, 0, sizeof(storedPassword));
  strncpy(storedSsid, ssid.c_str(), sizeof(storedSsid) - 1);
  strncpy(storedPassword, pass.c_str(), sizeof(storedPassword) - 1);
  Serial.printf("Wi-Fi: cabin network %s\n", storedSsid[0] ? storedSsid : "(none)");
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
  } else if (line.startsWith("TEST")) {
    // v2 radio test mode (chalet decides, hubs follow the beacon): TEST OFF|ROTATE|SF9|SF8|SF7
    String a = line.substring(4); a.trim(); a.toUpperCase();
    int m = -1;
    if (a == "OFF") m = 0; else if (a == "ROTATE") m = 1; else if (a == "SF9") m = 2; else if (a == "SF8") m = 3; else if (a == "SF7") m = 4;
    if (currentRole != ROLE_GATEWAY_OFFSHORE) {
      Serial.println(F("Test mode is set on the chalet (GATEWAY_OFFSHORE); hubs follow its beacon."));
    } else if (m < 0) {
      Serial.println(F("Usage: TEST OFF|ROTATE|SF9|SF8|SF7"));
    } else {
      settings.radioTestMode = (uint8_t)m; saveSettings(); meshSetTestMode((uint8_t)m); meshResetStats();
      Serial.printf("Radio test mode: %s\n", meshTestModeName((uint8_t)m));
    }
  } else if (line.startsWith("ADAPT")) {
    String a = line.substring(5); a.trim(); a.toUpperCase();
    if (currentRole == ROLE_GATEWAY_OFFSHORE && (a == "ON" || a == "OFF")) {
      settings.adaptiveRadio = (a == "ON"); saveSettings(); meshSetAdaptive(settings.adaptiveRadio);
      Serial.printf("Adaptive SF: %s\n", settings.adaptiveRadio ? "on" : "off");
    } else {
      Serial.println(F("Usage (chalet only): ADAPT ON|OFF"));
    }
  } else if (line.startsWith("SONAR")) {
    // v2 sonar test mode (chalet): fake sonar on every hub (virtual nodes) and on nodes with the sim switch
    String a = line.substring(5); a.trim(); a.toUpperCase();
    if (currentRole == ROLE_GATEWAY_OFFSHORE && (a == "ON" || a == "OFF")) {
      settings.sonarSim = (a == "ON"); saveSettings(); meshSetSonarSim(settings.sonarSim);
      Serial.printf("Sonar test mode: %s\n", settings.sonarSim ? "on" : "off");
    } else {
      Serial.println(F("Usage (chalet only): SONAR ON|OFF"));
    }
  } else if (line.startsWith("FOCUS")) {
    String a = line.substring(5); a.trim(); a.toUpperCase();
    const int n = (a == "OFF") ? 0 : a.toInt();
    if (currentRole == ROLE_GATEWAY_OFFSHORE && n >= 0 && n <= 255 && (a == "OFF" || n > 0)) {
      meshSetFocusNode((uint8_t)n);
      Serial.printf("FOCUS node: %d\n", n);
    } else {
      Serial.println(F("Usage (chalet only): FOCUS <node id>|OFF"));
    }
  } else if (line.startsWith("SIMNODES")) {
    String a = line.substring(8); a.trim();
    const int n = a.toInt();
    if (currentRole != ROLE_GATEWAY_OFFSHORE && a.length() == 0) {
      Serial.printf("Virtual sonar nodes: %u (test mode %s), fake ping cost avg %lu us, max %lu us over %lu pings\n",
                    settings.sonarVirtualNodes, meshSonarSim() ? "on" : "off",
                    (unsigned long)(sonarTickCount ? sonarTickUsSum / sonarTickCount : 0), (unsigned long)sonarTickUsMax, (unsigned long)sonarTickCount);
    } else if (currentRole != ROLE_GATEWAY_OFFSHORE && n >= 0 && n <= 4) {
      settings.sonarVirtualNodes = (uint8_t)n; saveSettings();
      Serial.printf("Virtual sonar nodes on this hub: %d (IDs from %u)\n", n, meshSonarVirtualId(NODE_ID, 0));
    } else {
      Serial.println(F("Usage (hub only): SIMNODES 0-4 (no number = stats)"));
    }
  } else if (line.startsWith("TRANSPORT")) {
    // v2: which radio carries hub <-> chalet traffic. Set on the chalet, hubs follow its beacon.
    String a = line.substring(9); a.trim(); a.toUpperCase();
    int t = -1;
    if (a == "AUTO") t = MESH_TR_AUTO; else if (a == "LORA") t = MESH_TR_LORA; else if (a == "ESPNOW" || a == "ESP-NOW") t = MESH_TR_ESPNOW;
    if (currentRole == ROLE_GATEWAY_OFFSHORE && t >= 0) {
      settings.transportMode = (uint8_t)t; saveSettings(); meshSetTransport((uint8_t)t);
      Serial.printf("Transport: %s\n", a.c_str());
    } else {
      Serial.println(F("Usage (chalet only): TRANSPORT AUTO|LORA|ESPNOW   (hubs follow the beacon)"));
    }
  } else if (line.startsWith("CHANNEL")) {
    // v2: LoRa channel 1-8 (1 = 915.0 MHz), AUTO, or SCAN (re-evaluate now). Chalet only.
    String a = line.substring(7); a.trim(); a.toUpperCase();
    const int n = a.toInt();
    if (currentRole != ROLE_GATEWAY_OFFSHORE) {
      Serial.printf("LoRa channel %u (%.1f MHz) - set on the chalet, hubs follow it\n", meshLoraChannel() + 1, meshLoraChannelMHz(meshLoraChannel()));
    } else if (a == "AUTO") {
      settings.loraChannel = MESH_CH_AUTO; saveSettings(); meshSetLoraChannel(MESH_CH_AUTO);
      Serial.println(F("LoRa channel: Auto (moves when the channel gets busy)"));
    } else if (a == "SCAN") {
      meshRescanChannels();
      Serial.println(F("Channel evaluation requested (see RADIO for activity %)"));
    } else if (n >= 1 && n <= 8) {
      settings.loraChannel = (uint8_t)(n - 1); saveSettings(); meshSetLoraChannel((uint8_t)(n - 1));
      Serial.printf("LoRa channel %d (%.1f MHz), announced to the hubs before the switch\n", n, meshLoraChannelMHz((uint8_t)(n - 1)));
    } else {
      Serial.printf("LoRa channel %u (%.1f MHz), setting %s. Usage: CHANNEL AUTO|1-8|SCAN\n", meshLoraChannel() + 1,
                    meshLoraChannelMHz(meshLoraChannel()), settings.loraChannel == MESH_CH_AUTO ? "Auto" : "fixed");
    }
  } else if (line.startsWith("RELAY")) {
    // v2 backbone relay: RELAY ON|OFF (this device), chalet: RELAY <device id> ON|OFF (sent in the beacon)
    String a = line.substring(5); a.trim(); a.toUpperCase();
    const int sp = a.indexOf(' ');
    if (a == "ON" || a == "OFF") {
      settings.ebRelay = (a == "ON"); saveSettings(); meshSetEbRelay(settings.ebRelay);
      Serial.printf("Backbone relay on this device: %s\n", settings.ebRelay ? "ON" : "off");
    } else if (currentRole == ROLE_GATEWAY_OFFSHORE && sp > 0) {
      const int dev = a.substring(0, sp).toInt();
      String v = a.substring(sp + 1); v.trim();
      if (dev > 0 && dev < 255 && (v == "ON" || v == "OFF")) {
        relayRequest((uint8_t)dev, v == "ON");
        Serial.printf("Relay %s requested for device %d (beacon command; tip-ups get it when they next transmit)\n", v.c_str(), dev);
      } else {
        Serial.println(F("Usage: RELAY <device id> ON|OFF"));
      }
    } else {
      Serial.printf("Backbone relay: %s. Usage: RELAY ON|OFF%s\n", meshEbRelay() ? "ON" : "off",
                    currentRole == ROLE_GATEWAY_OFFSHORE ? ", RELAY <device id> ON|OFF" : "");
    }
  } else if (line.startsWith("HOTSPOT")) {
    // v2: hub hotspot on demand (same as holding PRG 3 s)
    String a = line.substring(7); a.trim(); a.toUpperCase();
    if (currentRole == ROLE_GATEWAY_OFFSHORE) Serial.println(F("The chalet hotspot is always on (see WIFI settings)"));
    else if (a == "ON") hubHotspotOn();
    else if (a == "OFF") hubHotspotOff();
    else Serial.printf("Hotspot %s. Usage: HOTSPOT ON|OFF\n", wifiApActive ? "ON" : "off");
  } else if (line.startsWith("DEMO")) {
    // v2 demo network (chalet): DEMO <hubs 0-4> [holes 1-4] | DEMO OFF
    String a = line.substring(4); a.trim(); a.toUpperCase();
    if (currentRole != ROLE_GATEWAY_OFFSHORE) Serial.println(F("The demo network runs on the chalet"));
    else if (a == "OFF" || a == "0") demoSet(0, 3);
    else if (a.toInt() >= 1 && a.toInt() <= 4) { const int sp = a.indexOf(' '); demoSet((uint8_t)a.toInt(), sp > 0 ? (uint8_t)a.substring(sp + 1).toInt() : 3); }
    else Serial.printf("Demo network: %u hub(s) x %u hole(s). Usage: DEMO <1-4> [holes 1-4] | DEMO OFF\n", meshDemoHubs(), meshDemoHoles());
  } else if (line.startsWith("SIM")) {
    // v2 simulation (chalet): SIM ALL ON|OFF, SIM <hole> SONAR|HALL|BOTH|OFF, SIM RATE <trips/hour>, SIM
    String a = line.substring(3); a.trim(); a.toUpperCase();
    const int sp = a.indexOf(' ');
    String w1 = sp > 0 ? a.substring(0, sp) : a, w2 = sp > 0 ? a.substring(sp + 1) : "";
    w2.trim();
    if (currentRole != ROLE_GATEWAY_OFFSHORE) {
      Serial.println(F("Simulation is set on the chalet"));
    } else if (w1 == "ALL" && (w2 == "ON" || w2 == "OFF")) {
      simAll(w2 == "ON"); Serial.printf("Simulation of every hole: %s\n", w2.c_str());
    } else if (w1 == "RATE" && w2.toInt() >= 1 && w2.toInt() <= 63) {
      simRate = (uint8_t)w2.toInt(); Serial.printf("Fake trips: %u per hour (applies to the next SIM command)\n", simRate);
    } else if (w1.toInt() > 0 && (w2 == "SONAR" || w2 == "HALL" || w2 == "BOTH" || w2 == "OFF")) {
      simRequest((uint8_t)w1.toInt(), w2 == "SONAR" || w2 == "BOTH", w2 == "HALL" || w2 == "BOTH");
      Serial.printf("Simulation hole %d: %s (sent in the beacon)\n", w1.toInt(), w2.c_str());
    } else {
      Serial.println(F("Usage (chalet): SIM ALL ON|OFF | SIM <hole> SONAR|HALL|BOTH|OFF | SIM RATE <1-63>"));
      Serial.print(F("Holes running a simulation:"));
      for (int i = 1; i < network.node_count; i++) if (HAS_FLAG(network.nodes[i].flags, FLAG_SIM)) Serial.printf(" %u", network.nodes[i].node_id);
      Serial.println();
    }
  } else if (line == "NETRESET") {
    resetNetworkSettings();   // same as holding PRG 10 s
  } else if (line.startsWith("EBMODE")) {
    String a = line.substring(6); a.trim(); a.toUpperCase();
    if (currentRole == ROLE_GATEWAY_OFFSHORE && (a == "NORMAL" || a == "LR")) {
      settings.ebChaletLr = (a == "LR"); saveSettings();
      Serial.printf("Chalet ESP-NOW: %s - reboot to apply%s\n", settings.ebChaletLr ? "LR" : "normal rate",
                    settings.ebChaletLr ? " (the phone hotspot will disappear: bench test only)" : "");
    } else {
      Serial.println(F("Usage (chalet only): EBMODE NORMAL|LR  (reboot; LR removes the phone hotspot)"));
    }
  } else if (line == "BUZZ" || line == "BUZZ ACTIVE" || line == "BUZZ PASSIVE") {
    // v2: BUZZ = two test beeps; BUZZ ACTIVE|PASSIVE = buzzer type (saved), then the test
    if (line != "BUZZ") { settings.buzzerPassive = (line == "BUZZ PASSIVE"); saveSettings(); buzzerApplyType(); }
    Serial.printf("Buzzer: %s, test beeps\n", settings.buzzerPassive ? "passive (tone)" : "active (DC)");
    buzzerTest();
  } else if (line.startsWith("BAIT")) {
    // v2 (D43) bait depth per hole (chalet): BAIT <hole> <cm>  (0 = not set)
    String a = line.substring(4); a.trim();
    const int sp = a.indexOf(' ');
    if (sp > 0 && currentRole == ROLE_GATEWAY_OFFSHORE) baitSet((uint8_t)a.substring(0, sp).toInt(), (uint8_t)constrain(a.substring(sp + 1).toInt() / 5, 0, 255));
    else Serial.println(F("Usage (chalet): BAIT <hole> <depth cm>"));
  } else if (line.startsWith("KNOB")) {
    // v2 (D42) sonar knobs (chalet): KNOBS = list, KNOB <key> <value>, KNOB DEFAULTS, KNOB RESEND
    using namespace icemesh::sonar;
    String a = line.substring(line.startsWith("KNOBS") ? 5 : 4); a.trim();
    const int sp = a.indexOf(' ');
    String k = sp > 0 ? a.substring(0, sp) : a, v = sp > 0 ? a.substring(sp + 1) : "";
    k.toLowerCase(); v.trim();
    if (k == "defaults") { for (uint8_t i = 0; i < P_COUNT; i++) sonarKnobSet(i, paramInfo(i).def); }
    else if (k == "resend") { sonarKnobsResend(); }
    else if (k.length() && v.length()) { for (uint8_t i = 0; i < P_COUNT; i++) if (k == paramInfo(i).key) sonarKnobSet(i, (uint8_t)constrain(v.toInt(), 0, 255)); }
    for (uint8_t i = 0; i < P_COUNT; i++) {
      const ParamInfo& f = paramInfo(i);
      Serial.printf("  %-8s %3u %-8s (%u-%u, default %u)  %s\n", f.key, sonarPrm[i], f.unit, f.lo, f.hi, f.def, f.name);
    }
    Serial.println(F("Usage: KNOB <key> <value> | KNOB DEFAULTS | KNOB RESEND (chalet: sent to every sonar hole)"));
  } else if (line == "PERF") {
    perfPrint();   // v2: loop timing since the last PERF
  } else if (line == "RADIO") {
    meshPrintStatus(Serial);
  } else if (line == "RADIO RESET") {
    meshResetStats();
    Serial.println(F("Radio stats reset"));
  } else if (line == "WIFI CLEAR") {
    preferences.begin("wifi", false);
    preferences.clear();
    preferences.end();
    Serial.println(F("WiFi credentials cleared!"));
  }
}

// =============================================================================================
// v2 CABIN WI-FI (chalet): why it is not connected, retry, join from the OLED (Options > Wi-Fi)
//   - the reason of the last drop/failure is kept (ESP-IDF wifi_err_reason_t) and shown as words
//   - if the router was not there at boot, retry after 2 min, then 4, 8 ... 30 min. Each try scans
//     the channels for ~1-2 s (phones on the hotspot and ESP-NOW may miss that moment), hence the back-off
//   - joining a network from the OLED saves it (same NVS keys as the web page / serial WIFI:) and
//     connects without a reboot
// =============================================================================================
static const uint32_t STA_RETRY_FIRST_MS = 2UL * 60UL * 1000UL, STA_RETRY_MAX_MS = 30UL * 60UL * 1000UL;
static volatile uint8_t staReason = 0;        // last disconnect reason, 0 = none yet
static uint32_t staTryAt = 0;                 // millis() of the last WiFi.begin from here
static uint32_t staNextTry = 0, staBackoffMs = STA_RETRY_FIRST_MS;
static bool wifiScanBusy = false;

static void onStaDisconnected(arduino_event_id_t, arduino_event_info_t info) {
  const uint8_t r = info.wifi_sta_disconnected.reason;
  if (r != 8) staReason = r;   // 8 = ASSOC_LEAVE: our own WiFi.disconnect(), keeps the real reason
}

static bool staWanted() {
  return currentRole == ROLE_GATEWAY_OFFSHORE && storedSsid[0] && settings.wifiModeSetting != 0;
}

static void staBegin() {
  staReason = 0;
  staTryAt = millis(); if (staTryAt == 0) staTryAt = 1;
  WiFi.begin(storedSsid, storedPassword);
  Serial.printf("Wi-Fi: trying %s\n", storedSsid);
}

// called every 5 s while not connected (checkWiFiStatus)
static void staRetryTick() {
  if (!staWanted() || wifiScanBusy) return;
  const uint32_t now = millis();
  if (staNextTry == 0 || static_cast<int32_t>(now - staNextTry) < 0) return;
  staBegin();
  staNextTry = now + staBackoffMs;
  staBackoffMs = staBackoffMs * 2 > STA_RETRY_MAX_MS ? STA_RETRY_MAX_MS : staBackoffMs * 2;
}

void wifiRetryNow() {
  if (!staWanted()) return;
  staBackoffMs = STA_RETRY_FIRST_MS;
  staNextTry = millis() + staBackoffMs;
  staBegin();
}

void wifiJoin(const char* ssid, const char* pass) {
  saveWifiCredentials(ssid, pass);
  storedSsid[sizeof(storedSsid) - 1] = 0; storedPassword[sizeof(storedPassword) - 1] = 0;
  if (settings.wifiModeSetting == 0) { settings.wifiModeSetting = 2; saveSettings(); }   // AP only would never join
  if (WiFi.getMode() == WIFI_MODE_AP) WiFi.mode(WIFI_AP_STA);
  wifiStaConnected = false; staIpAddress = "";
  WiFi.disconnect(false);
  wifiRetryNow();
}

void wifiForget() {
  saveWifiCredentials("", "");
  WiFi.disconnect(false);
  wifiStaConnected = false; staIpAddress = ""; staReason = 0; staTryAt = 0;
  Serial.println(F("Wi-Fi: cabin network forgotten (hotspot only)"));
}

// short words for the OLED
void wifiStateText(char* out, size_t n) {
  const uint8_t r = staReason;
  if (!storedSsid[0]) snprintf(out, n, "none set");
  else if (wifiStaConnected) snprintf(out, n, "connected");
  else if (settings.wifiModeSetting == 0) snprintf(out, n, "off (AP only)");
  else if (staTryAt && millis() - staTryAt < 15000 && r == 0) snprintf(out, n, "connecting...");
  else if (r == 201) snprintf(out, n, "not found");                                         // NO_AP_FOUND
  else if (r == 2 || r == 15 || r == 202 || r == 204) snprintf(out, n, "bad password?");  // AUTH_EXPIRE, 4WAY/HANDSHAKE timeout, AUTH_FAIL
  else if (r == 0) snprintf(out, n, "not connected");
  else snprintf(out, n, "failed (%u)", r);
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
  if (currentRole == ROLE_GATEWAY_ONSHORE) {   // v2: no hotspot on the ice hub by default (on demand: step 2)
    DEBUG_PRINTLN(F("GATEWAY_ONSHORE: no WiFi AP (status on the chalet page)"));
    wifiApActive = false;
    wifiStaConnected = false;
    return;  // Skip all WiFi AP setup
  }

  // Load credentials from NVS (or use defaults)
  loadWifiCredentials();
  // v2: keep why the cabin Wi-Fi fails or drops (OLED: not found / bad password?) - registered before the first try
  WiFi.onEvent(onStaDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  char apSsid[32];
  snprintf(apSsid, sizeof(apSsid), "%s-%s",
           NETWORK_NAME,
           currentRole == ROLE_GATEWAY_OFFSHORE ? "Remote" : "Ice");

  // Determine WiFi mode based on settings
  // Note: GATEWAY_ONSHORE already returned above (no hotspot by default)
  uint8_t effectiveWifiMode = settings.wifiModeSetting;

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
      if (storedSsid[0]) WiFi.begin(storedSsid, storedPassword);   // v2: no network set: no 15 s wait
      DEBUG_PRINTF("Connecting to: %s\n", storedSsid);

      // Wait for connection (with timeout)
      {
        int attempts = 0;
        while (storedSsid[0] && WiFi.status() != WL_CONNECTED && attempts < 30) {
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
      if (storedSsid[0]) WiFi.begin(storedSsid, storedPassword);   // v2: no network set: no 10 s wait
      DEBUG_PRINTF("Connecting to: %s", storedSsid);

      // Wait up to 10 seconds for connection
      {
        int attempts = 0;
        while (storedSsid[0] && WiFi.status() != WL_CONNECTED && attempts < 20) {
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
  
  staNextTry = millis() + STA_RETRY_FIRST_MS;   // v2: boot try done above, retries after 2 min (staRetryTick)

  // Set channel for ESP-NOW compatibility
  // CRITICAL: Wait for AP to stabilize, then force channel and verify
  delay(100);  // Let AP stabilize
  uint8_t primary;
  wifi_second_chan_t secondary;
  if (WiFi.status() == WL_CONNECTED) {
    // v2: joined the cabin router - its channel wins. Forcing another channel here would fight the
    // router link (ESP-IDF: do not set the channel while the station is connected). ESP-NOW follows it.
    esp_wifi_get_channel(&primary, &secondary);
    Serial.printf("Wi-Fi: on the cabin router, channel %u (hubs use %u for ESP-NOW; LoRa is not affected)\n", primary, ESPNOW_CHANNEL);
    return;
  }
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(50);

  // VERIFY channel alignment
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

  if (settings.wifiModeSetting != 0) {   // v2: the saved mode (Settings / web), not only the config.h default
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
      // v2: back to the hubs' channel (it followed the router while joined); allowed now the station is down
      if (currentRole == ROLE_GATEWAY_OFFSHORE) esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    }
    if (wifiStaConnected) { staBackoffMs = STA_RETRY_FIRST_MS; staNextTry = millis() + STA_RETRY_FIRST_MS; }   // no stale retry time after a long connection
    else staRetryTick();
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

  if (primary != ESPNOW_CHANNEL && currentRole == ROLE_GATEWAY_OFFSHORE) {
    // v2: the chalet keeps the router's channel (forcing it would fight the router link). LoRa is not
    // affected; the ESP-NOW backup with the hubs needs the router on channel ESPNOW_CHANNEL.
    Serial.printf("Wi-Fi: router on channel %u, hubs use %u - ESP-NOW backup with the hubs needs the router on %u\n",
                  primary, ESPNOW_CHANNEL, ESPNOW_CHANNEL);
    return;
  }
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
  server.on("/api/node/bait", HTTP_POST, handleWebApiNodeBait);    // v2 (D43): bait depth per hole
  server.on("/api/trips", HTTP_GET, handleWebApiTrips);            // v2 (D43): sonar of the minute before each trip
  server.on("/settings", HTTP_GET, handleWebRoot);            // v2: one suite page, the path picks the tab
  server.on("/api/settings", HTTP_GET, handleWebApiSettingsGet);
  server.on("/api/settings", HTTP_POST, handleWebApiSettingsPost);
  server.on("/remote-config", HTTP_GET, handleWebRemoteConfig);   // not linked from the suite: no remote config over the v2 mesh yet
  server.on("/api/remote-config", HTTP_POST, handleWebApiRemoteConfig);
  server.on("/api/remote-config/status", HTTP_GET, handleWebApiRemoteConfigStatus);
  server.on("/radio", HTTP_GET, handleWebRoot);
  server.on("/api/radio", HTTP_GET, handleWebApiRadio);
  server.on("/api/radio", HTTP_POST, handleWebApiRadioPost);
  server.on("/sonar", HTTP_GET, handleWebRoot);
  server.on("/api/sonar", HTTP_GET, handleWebApiSonar);
  server.on("/api/sonar", HTTP_POST, handleWebApiSonarPost);
  server.on("/api/sonar/glance", HTTP_GET, []() {   // v2: every hole, ~3 min of sonar summaries (since = frame)
    const uint16_t since = (uint16_t)strtoul(server.arg("since").c_str(), nullptr, 10);
    server.send(200, "application/json", meshSonarGlanceJson(since));
  });
  server.on("/api/sim", HTTP_GET, handleWebApiSim);
  server.on("/api/sonar/knobs", HTTP_GET, handleWebApiKnobs);       // v2 (D42): sonar processing knobs
  server.on("/api/sonar/knobs", HTTP_POST, handleWebApiKnobsPost);
  server.on("/api/sim", HTTP_POST, handleWebApiSimPost);
  server.on("/api/sonar/pings", HTTP_GET, handleWebApiSonarPings);
  server.on("/api/sonar/bg", HTTP_GET, handleWebApiSonarBg);

  server.begin();
  DEBUG_PRINTLN(F("Web server ready on port 80"));
}

void loopWebServer() {
  server.handleClient();
}

// Chalet web suite: one page, tabs Holes / Sonar / Radio / Settings (theme and sonar views from Frank's
// prototype, docs/prototype). The FISH ON bar, sound and silence work on every tab. /, /sonar, /radio and
// /settings all serve it (the path picks the tab). Kept in flash (~58 KB), sent without a RAM copy.
#include "web_suite.h"   // SUITE_PAGE: generated from web/suite.html by tools/gen_web.py (do not edit the .h)

void handleWebRoot() {
  server.send_P(200, "text/html", SUITE_PAGE);
}

void handleWebApi() {
  // BUG FIX #10 / v2: sized for the node count (up to MAX_NODES = 48, ~240 B each incl. copied strings)
  DynamicJsonDocument doc(1536 + 240 * (size_t)network.node_count);
  doc["network_id"] = network.network_id;
  doc["node_count"] = network.node_count;
  doc["uptime"] = millis() / 1000;
  // v2 suite: state the page needs on every tab
  doc["node_id"] = NODE_ID;
  doc["role"] = currentRole == ROLE_GATEWAY_OFFSHORE ? "Chalet" : "Hub";
  doc["fw"] = "v2";
  doc["silenced"] = alertsSilenced;
  doc["silence_left_sec"] = (alertsSilenced && silenceExpireTime > millis()) ? (silenceExpireTime - millis()) / 1000 : 0;
  doc["sonar_sim"] = meshSonarSim();
  doc["trips"] = tripCount;   // v2 (D43): trip recordings kept (download /api/trips)
  doc["radio_test"] = meshTestModeName(meshTestMode());
  // v2 network one-liner: setup phase countdown (0 = running), link, LoRa channel (1-8), master
  doc["setup_arm_s"] = meshSetupArmInS();
  { static const char* const TRN[] = {"auto", "lora", "espnow"}; const uint8_t t = meshTransport(); doc["transport"] = TRN[t <= 2 ? t : 0]; }
  doc["lora_ch"] = meshLoraChannel() + 1;
  doc["master"] = "chalet";

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
    n["fish"] = nodeAlarm(network.nodes[i]);   // v2: the alarm (latched), not only the line state
    n["line"] = HAS_FLAG(network.nodes[i].flags, FLAG_FISH_ON);
    n["bait"] = baitCm5[network.nodes[i].node_id] * 5;   // v2 (D43): cm, 0 = not set
    n["lowbat"] = HAS_FLAG(network.nodes[i].flags, FLAG_LOW_BATTERY);
    n["sim"] = HAS_FLAG(network.nodes[i].flags, FLAG_SIM) != 0;   // v2: this hole runs a simulation
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
// v2 (D43) POST {"nodeId":n,"cm":depth}  (cm 0 = not set)
void handleWebApiNodeBait() {
  StaticJsonDocument<96> doc;
  if (!server.hasArg("plain") || deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}"); return; }
  const int id = doc["nodeId"] | 0, cm = doc["cm"] | 0;
  if (currentRole != ROLE_GATEWAY_OFFSHORE || id <= 0 || id >= 255 || cm < 0 || cm > 1275) { server.send(400, "application/json", "{\"error\":\"bad hole or depth\"}"); return; }
  baitSet((uint8_t)id, (uint8_t)((cm + 2) / 5));
  server.send(200, "application/json", "{\"ok\":true}");
}

// v2 (D43) trip recordings: ?download=1 sends a file. recs: [frame, bottom_cm, target...] (target = depth | level<<11 | bait<<13)
void handleWebApiTrips() {
  String s; s.reserve(256 + tripCount * 1100);
  s = "{\"uptime\":"; s += millis() / 1000; s += ",\"trips\":[";
  for (uint8_t k = 0; k < tripCount; k++) {
    const TripRec& t = tripRecs[(tripHead + 8 - tripCount + k) % 8];
    if (k) s += ",";
    const int ix = findNodeIndexForMesh(t.node);
    s += "{\"node\":"; s += t.node; s += ",\"name\":\"";
    if (ix >= 0) for (const char* c = network.nodes[ix].name; *c; c++) if (*c != '"' && *c != '\\') s += *c;
    s += "\",\"at_s\":"; s += t.at_s; s += ",\"bait_cm\":"; s += baitCm5[t.node] * 5; s += ",\"recs\":[";
    for (uint8_t r = 0; r < t.n; r++) {
      if (r) s += ",";
      s += "["; s += t.r[r].frame; s += ","; s += t.r[r].bottom_cm;
      for (uint8_t j = 0; j < t.r[r].n && j < 5; j++) { s += ","; s += t.r[r].t[j]; }
      s += "]";
    }
    s += "]}";
  }
  s += "]}";
  if (server.hasArg("download")) server.sendHeader("Content-Disposition", "attachment; filename=\"trip_recordings.json\"");
  server.send(200, "application/json", s);
}

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


void handleWebApiSettingsGet() {
  StaticJsonDocument<256> doc;
  doc["buzzerEnabled"] = settings.buzzerEnabled;
  doc["alertHoldSec"] = settings.alertHoldSec;
  doc["alarmHoldMin"] = settings.alarmHoldMin;
  doc["nearBaitBeep"] = settings.nearBaitBeep;
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
  if (doc.containsKey("nearBaitBeep")) { settings.nearBaitBeep = doc["nearBaitBeep"].as<bool>(); changed = true; }
  if (doc.containsKey("alarmHoldMin")) {   // v2: 0 = until silenced
    settings.alarmHoldMin = (uint8_t)constrain(doc["alarmHoldMin"].as<int>(), 0, 120);
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
// =============================================================================================
// v2 OLED (src/lora_node/screens.cpp, docs/SCREENS.md): the model is built from the live state at
// each redraw (5 Hz); the drawing code is the one rendered on the PC for the mockups.
// Short press / CardKB right = next page, double press / Enter = the page's action (written at the
// bottom), during an alert the button silences. Pages return to Home after 60 s.
// =============================================================================================
static ScreenModel scr;
static uint8_t scrPage = PG_HOME, scrSub = 0;
bool scrDirty = true;                  // another screen used the display: resend the next frame
static bool scrMenu = false;           // CardKB menu (list of pages) open
static uint8_t scrMenuSel = 0;
static uint32_t scrLastInput = 0;
static MeshPingLite scrPings[118];

static uint8_t holeStateOf(const NodeState& n) {
  if (!n.online) return SH_OFFLINE;
  if (nodeAlarm(n)) return SH_FISH;
  if (HAS_FLAG(n.flags, FLAG_SENSOR_ERROR)) return SH_FAULT;
  if (HAS_FLAG(n.flags, FLAG_LOW_BATTERY)) return SH_LOWBAT;
  return SH_OK;
}

static void buildScreenModel() {
  ScreenModel& m = scr;
  const bool chalet = currentRole == ROLE_GATEWAY_OFFSHORE;
  const uint32_t now = millis();
  m.chalet = chalet; m.self_id = NODE_ID; m.feet = !settings.unitsMetric; m.uptime_s = now / 1000;
  m.son_range_cm = depthStepCm(settings.sonDepthIdx); m.focus_range_cm = depthStepCm(settings.focusDepthIdx);
  m.hide_weak = settings.hideWeak;
  m.note[0] = 0;
  if (nearBaitNode && millis() - nearBaitAt < 15000UL) {   // D43: a fish came near a bait in the last 15 s
    const int ix = findNodeIndexForMesh(nearBaitNode);
    if (ix >= 0 && network.nodes[ix].name[0]) snprintf(m.note, sizeof(m.note), "Near bait: %s", network.nodes[ix].name);
    else snprintf(m.note, sizeof(m.note), "Near bait: hole %u", nearBaitNode);
  }
  // holes, grouped by pocket (owner hub), the hub's own hole first
  static MeshNodeSnapshot snap[48];
  const uint8_t ns = chalet ? meshNodeSnapshot(snap, 48) : 0;
  m.n_holes = 0;
  for (int i = 0; i < network.node_count && m.n_holes < 48; i++) {
    const NodeState& n = network.nodes[i];
    if (n.node_id == 0 || !n.initialized) continue;
    if (n.node_id == NODE_ID && (chalet || (!HAS_LOCAL_SENSOR && !hubHoleSim(NODE_ID)))) continue;
    ScrHole& h = m.holes[m.n_holes];
    memset(&h, 0, sizeof(h));
    h.id = n.node_id; h.hub = chalet ? 0 : NODE_ID;
    for (uint8_t k = 0; k < ns; k++) if (snap[k].node == n.node_id) h.hub = snap[k].owner;
    if (h.hub == 0) h.hub = n.node_id;
    snprintf(h.name, sizeof(h.name), "%s", n.name);
    h.state = holeStateOf(n);
    if (n.node_id == NODE_ID && hubSimTripped(NODE_ID)) h.state = SH_FISH;
    h.batt = n.battery_mv ? batteryMvToPercent(n.battery_mv) : 255;
    h.since_s = h.state == SH_FISH ? (now - (n.alarm_ms ? n.alarm_ms : (n.fish_on_time ? n.fish_on_time : now))) / 1000 : 0;
    { const uint8_t dv = meshDemoSim(n.node_id); h.sim = dv != 0xFF ? (dv & 3) : (HAS_FLAG(n.flags, FLAG_SIM) ? 1 : 0); }
    h.fish = -1;
    MeshSonarLite sl;
    if (chalet && meshSonarSummary(n.node_id, sl)) {
      h.son.valid = true; h.son.bottom_cm = sl.bottom_cm; h.son.hard = sl.hard; h.son.activity = sl.activity; h.son.n = 0; h.fish = 0;
      h.son.status = sl.status; h.son.bottom_snr = sl.bottom_snr;
      for (uint8_t k = 0; k < sl.n && k < 5; k++) {
        ScrTarget& t = h.son.t[h.son.n++];
        t.depth_cm = sl.t[k] & 0x7FF; t.level = (sl.t[k] >> 11) & 3; t.bait = (sl.t[k] >> 13) & 1;
        if (!t.bait) h.fish++;
      }
    }
    m.n_holes++;
  }
  // sort by pocket, the hub's own hole first, then ID
  for (uint8_t a = 1; a < m.n_holes; a++)
    for (uint8_t b = a; b > 0; b--) {
      const ScrHole& x = m.holes[b - 1]; const ScrHole& y = m.holes[b];
      const uint32_t kx = (uint32_t)x.hub << 9 | (x.id == x.hub ? 0 : 256) | x.id, ky = (uint32_t)y.hub << 9 | (y.id == y.hub ? 0 : 256) | y.id;
      if (kx <= ky) break;
      const ScrHole t = m.holes[b - 1]; m.holes[b - 1] = m.holes[b]; m.holes[b] = t;
    }
  // network
  m.setup_s = meshSetupArmInS();
  m.lora_ch = meshLoraChannel() + 1; m.lora_mhz_x10 = (uint16_t)(meshLoraChannelMHz(meshLoraChannel()) * 10.0f + 0.5f);
  m.ch_auto = meshLoraChannelSetting() == MESH_CH_AUTO; m.transport = meshTransport();
  uint8_t busy[8]; meshChannelBusy(busy); m.busy_pct = busy[meshLoraChannel() & 7];
  m.silenced = alertsSilenced; m.silence_s = (alertsSilenced && silenceExpireTime > now) ? (silenceExpireTime - now) / 1000 : 0;
  m.n_hubs = 0;
  if (chalet) {
    MeshHubLink hl[10];
    const uint8_t nh = meshHubLinks(hl, 10);
    for (uint8_t k = 0; k < nh; k++) {
      ScrHubLink& o = m.hubs[m.n_hubs++];
      o.id = hl[k].id; o.lora_rssi = hl[k].rssi; o.eb_hops = hl[k].hops;
      o.lora_ok = hl[k].lora_age_s >= 0 && hl[k].lora_age_s < 30; o.eb_ok = hl[k].eb_age_s >= 0 && hl[k].eb_age_s < 30;
      o.demo = hl[k].demo;
    }
  } else {
    MeshHubView v; meshHubView(v);
    m.master_heard = v.synced; m.beacon_rssi = v.beacon_rssi; m.via_echo = v.from_echo; m.on_backup = v.eb_on;
    m.hotspot = wifiApActive; m.hotspot_min = hubHotspotUntil ? (hubHotspotUntil - now) / 60000UL + 1 : 0;
  }
  // focus (chalet)
  m.focus_node = meshFocusNode(); m.n_cols = 0; m.bait_cm = 0;
  if (chalet && m.focus_node && scrPage == PG_FOCUS) {
    const uint8_t np = meshFocusPings(m.focus_node, scrPings, 118);
    for (uint8_t k = 0; k < np; k++) {
      ScrPingCol& c = m.cols[m.n_cols++];
      c.bottom_cm = scrPings[k].bottom_cm; c.n = scrPings[k].n;
      for (uint8_t t = 0; t < c.n; t++) { c.d[t] = scrPings[k].d[t]; c.lv[t] = scrPings[k].lv[t]; if (scrPings[k].bait_mask & (1u << t)) m.bait_cm = c.d[t]; }
    }
  }
  // connect
  char ssid[32]; apName(ssid, sizeof(ssid));
  snprintf(m.ssid, sizeof(m.ssid), "%s", ssid); snprintf(m.pass, sizeof(m.pass), "%s", WIFI_PASSWORD);
  m.sta = wifiStaConnected;
  snprintf(m.url, sizeof(m.url), "http://%s", wifiStaConnected ? staIpAddress.c_str() : apIpAddress.c_str());
  m.radio_test = meshTestMode(); m.radio_test_name = meshTestModeName(m.radio_test);
  m.kb = cardKbAvailable;
  snprintf(m.sta_ssid, sizeof(m.sta_ssid), "%s", storedSsid);
  wifiStateText(m.sta_state, sizeof(m.sta_state));
  m.menu = scrMenu;
  if (scrMenu) menuBuild(&m.opt);
  else if (scrPage == PG_OPTIONS) optBuild(&m.opt);
}

// =============================================================================================
// v2 OPTIONS page (CardKB). One list; Wi-Fi and Simulation open their own list. Up/down = move,
// Enter = change / open, Esc or left = back. A value changes at once and is saved (same settings
// as the web page and the serial commands). Text entry for the Wi-Fi name / password, yes/no before
// network reset, reboot and "forget Wi-Fi". Without a CardKB the page only shows the values.
// docs/SCREENS.md
// =============================================================================================
enum OptMenu : uint8_t { OM_MAIN = 0, OM_WIFI, OM_SCAN, OM_SIM, OM_DISPLAY, OM_SONAR };
enum OptItem : uint8_t {
  OI_NONE = 0, OI_WIFI, OI_BUZZER, OI_HOLD, OI_LINK, OI_LORACH, OI_SIM, OI_HOTSPOT, OI_RELAY, OI_REED,
  OI_NETRESET, OI_REBOOT, OI_BUILT,
  OI_W_STATUS, OI_W_NAME, OI_W_CHOOSE, OI_W_TYPE, OI_W_ADDR, OI_W_RETRY, OI_W_CH, OI_W_AP, OI_W_APPASS, OI_W_FORGET,
  OI_SC_NET, OI_SC_AGAIN,
  OI_S_ALL, OI_S_VIRTUAL, OI_S_RATE, OI_S_HOLE, OI_S_DEMO, OI_S_DEMOHOLES, OI_BUZZTEST, OI_BUZZTYPE,
  OI_DISPLAY, OI_D_UNITS, OI_D_SONDEPTH, OI_D_FOCDEPTH, OI_D_WEAK,
  OI_SONAR, OI_K_FILTER, OI_K_SNR, OI_K_CONFIRM, OI_K_DEAD, OI_K_GATE, OI_K_RESEND, OI_K_DEFAULTS, OI_NEARBEEP
};
struct OptRow { uint8_t item, idx; };
static const uint8_t OPT_MAX = 56;
static OptRow optRows[OPT_MAX];
static uint8_t optN = 0, optMenu = OM_MAIN, optSel = 0, optMainSel = 0, optWifiSel = 0;
static uint8_t optMode = SO_LIST, optConfirm = OI_NONE, optTextStep = 0;   // text step: 0 Wi-Fi name, 1 password
static char optSsid[33] = "", optText[65] = "";
static const uint8_t SCAN_MAX = 12;
static int8_t optScanN = -1;                 // -1 not yet, -2 scanning, -3 failed, >= 0 networks found
static char optScanSsid[SCAN_MAX][33];
static int8_t optScanRssi[SCAN_MAX];
static bool optScanLock[SCAN_MAX];
static const uint16_t HOLD_STEPS[] = {0, 1, 5, 15, 30};   // FISH ON alarm: 0 = until silenced, else minutes
static const uint8_t RATE_STEPS[] = {2, 6, 12, 30, 60};

// next value of a list (wraps), or the first one above the current value
static uint8_t nextStep(const uint8_t* steps, uint8_t n, uint8_t cur) {
  for (uint8_t i = 0; i < n; i++) if (steps[i] > cur) return steps[i];
  return steps[0];
}

uint16_t depthStepCm(uint8_t idx) {
  if (idx == 0 || idx > 8) return 0;
  return settings.unitsMetric ? (uint16_t)(DEPTH_M[idx - 1] * 100u) : (uint16_t)(DEPTH_FT[idx - 1] * 3048u / 100u);
}
void depthStepText(uint8_t idx, char* out, size_t n) {
  if (idx == 0 || idx > 8) snprintf(out, n, "auto");
  else if (settings.unitsMetric) snprintf(out, n, "0-%u m", DEPTH_M[idx - 1]);
  else snprintf(out, n, "0-%u ft", DEPTH_FT[idx - 1]);
}

static void optRow(ScrOptions* o, uint8_t item, uint8_t idx, const char* label, const char* value, bool sub = false) {
  if (optN >= OPT_MAX) return;
  optRows[optN].item = item; optRows[optN].idx = idx;
  if (o) {
    ScrRow& r = o->rows[optN];
    snprintf(r.label, sizeof(r.label), "%s", label); snprintf(r.value, sizeof(r.value), "%s", value); r.sub = sub;
  }
  optN++;
}

static const char* optHint(uint8_t item) {
  switch (item) {
    case OI_WIFI: case OI_SIM: case OI_W_CHOOSE: return "OK: open";
    case OI_W_TYPE: return "OK: type the name";
    case OI_W_RETRY: return "OK: try now";
    case OI_BUZZTEST: return "OK: 2 beeps";
    case OI_DISPLAY: case OI_SONAR: return "OK: open";
    case OI_K_RESEND: return "OK: send now";
    case OI_K_DEFAULTS: return "OK: ask to confirm";
    case OI_K_FILTER: return "OK: next filter";
    case OI_BUZZTYPE: return "OK: switch + test";
    case OI_SC_NET: return "OK: join";
    case OI_SC_AGAIN: return "OK: scan again";
    case OI_NETRESET: case OI_REBOOT: case OI_W_FORGET: return "OK: ask to confirm";
    case OI_S_HOLE: return "OK: next mode";
    case OI_S_DEMO: return "OK: off/1/2/3/4 hubs";
    case OI_NONE: case OI_BUILT: case OI_W_STATUS: case OI_W_NAME: case OI_W_ADDR: case OI_W_CH: case OI_W_AP: case OI_W_APPASS:
      return "Esc: back";
    default: return "OK: change";
  }
}

// Builds the rows of the current list (optRows: what each row does; o: the text, when drawing)
static void optBuild(ScrOptions* o) {
  const bool chalet = currentRole == ROLE_GATEWAY_OFFSHORE;
  char v[24];
  optN = 0;
  if (optMenu == OM_MAIN) {
    if (chalet) {
      optRow(o, OI_WIFI, 0, "Wi-Fi", wifiStaConnected ? "OK" : (storedSsid[0] ? "not conn." : "not set"), true);
    } else {
      if (wifiApActive && hubHotspotUntil) snprintf(v, sizeof(v), "ON %lum", (unsigned long)((hubHotspotUntil - millis()) / 60000UL + 1));
      else snprintf(v, sizeof(v), "%s", wifiApActive ? "ON" : "OFF");
      optRow(o, OI_HOTSPOT, 0, "Hotspot", v);
    }
    optRow(o, OI_BUZZER, 0, "Buzzer", settings.buzzerEnabled ? "ON" : "OFF");
    optRow(o, OI_BUZZTEST, 0, "Buzzer test", "");
    optRow(o, OI_BUZZTYPE, 0, "Buzzer type", settings.buzzerPassive ? "passive" : "active");
    if (chalet) optRow(o, OI_NEARBEEP, 0, "Near-bait beep", settings.nearBaitBeep ? "ON" : "OFF");
    if (settings.alarmHoldMin) snprintf(v, sizeof(v), "%u min", settings.alarmHoldMin); else snprintf(v, sizeof(v), "until silenced");
    optRow(o, OI_HOLD, 0, "Alarm", v);
    if (chalet) {
      const uint8_t t = settings.transportMode;
      optRow(o, OI_LINK, 0, "Link", t == 1 ? "LoRa only" : t == 2 ? "ESP-NOW" : "Auto");
      if (settings.loraChannel == MESH_CH_AUTO) snprintf(v, sizeof(v), "Auto (%u)", meshLoraChannel() + 1);
      else snprintf(v, sizeof(v), "%u fixed", settings.loraChannel + 1);
      optRow(o, OI_LORACH, 0, "LoRa channel", v);
      optRow(o, OI_SIM, 0, "Simulation", simAnyOn() ? "ON" : "off", true);
      static const char* const FN[] = {"low", "normal", "high", "custom"};
      optRow(o, OI_SONAR, 0, "Sonar", FN[icemesh::sonar::presetOf(sonarPrm)], true);
      optRow(o, OI_DISPLAY, 0, "Display", settings.unitsMetric ? "m" : "ft", true);
    } else {
      optRow(o, OI_RELAY, 0, "ESP-NOW relay", settings.ebRelay ? "ON" : "OFF");
      optRow(o, OI_REED, 0, "Reed polarity", settings.reedActiveHigh ? "HIGH" : "LOW");
    }
    optRow(o, OI_NETRESET, 0, "Network reset", "");
    optRow(o, OI_REBOOT, 0, "Reboot", "");
    // build date of this firmware (__DATE__ "Oct  6 2026", __TIME__ "14:02:33") -> "Oct  6 14:02"
    snprintf(v, sizeof(v), "%.6s %.5s", __DATE__, __TIME__);
    optRow(o, OI_BUILT, 0, "Built", v);
  } else if (optMenu == OM_WIFI) {
    char st[20]; wifiStateText(st, sizeof(st));
    optRow(o, OI_W_STATUS, 0, "Status", st);
    optRow(o, OI_W_NAME, 0, "Network", storedSsid[0] ? storedSsid : "-");
    optRow(o, OI_W_CHOOSE, 0, "Choose network", "", true);
    optRow(o, OI_W_TYPE, 0, "Type name", "", true);
    optRow(o, OI_W_ADDR, 0, "Address", wifiStaConnected ? staIpAddress.c_str() : "-");
    if (storedSsid[0] && !wifiStaConnected) optRow(o, OI_W_RETRY, 0, "Retry now", "");
    uint8_t primary = 0; wifi_second_chan_t sec; esp_wifi_get_channel(&primary, &sec);
    if (primary == ESPNOW_CHANNEL) snprintf(v, sizeof(v), "%u OK", primary);
    else snprintf(v, sizeof(v), "%u (hubs %u)", primary, ESPNOW_CHANNEL);
    optRow(o, OI_W_CH, 0, "Channel", v);
    char ap[32]; apName(ap, sizeof(ap));
    optRow(o, OI_W_AP, 0, "AP", ap);
    optRow(o, OI_W_APPASS, 0, "AP pass", WIFI_PASSWORD);
    if (storedSsid[0]) optRow(o, OI_W_FORGET, 0, "Forget network", "");
  } else if (optMenu == OM_SCAN) {
    if (optScanN == -2) optRow(o, OI_NONE, 0, "Scanning...", "");
    else if (optScanN == -3) optRow(o, OI_NONE, 0, "Scan failed", "");
    else if (optScanN == 0) optRow(o, OI_NONE, 0, "Nothing found", "");
    for (int8_t k = 0; k < optScanN; k++) {
      snprintf(v, sizeof(v), "%d%s", optScanRssi[k], optScanLock[k] ? "" : " open");
      optRow(o, OI_SC_NET, (uint8_t)k, optScanSsid[k], v);
    }
    if (optScanN != -2) optRow(o, OI_SC_AGAIN, 0, "Scan again", "");
  } else if (optMenu == OM_DISPLAY) {
    optRow(o, OI_D_UNITS, 0, "Units", settings.unitsMetric ? "metres" : "feet");
    depthStepText(settings.sonDepthIdx, v, sizeof(v)); optRow(o, OI_D_SONDEPTH, 0, "Sonar depth", v);
    depthStepText(settings.focusDepthIdx, v, sizeof(v)); optRow(o, OI_D_FOCDEPTH, 0, "Focus depth", v);
    optRow(o, OI_D_WEAK, 0, "Weak echoes", settings.hideWeak ? "hidden" : "shown");
  } else if (optMenu == OM_SONAR) {
    using namespace icemesh::sonar;
    static const char* const FN[] = {"low", "normal", "high", "custom"};
    optRow(o, OI_K_FILTER, 0, "Noise filter", FN[presetOf(sonarPrm)]);
    snprintf(v, sizeof(v), "%u dB", sonarPrm[P_SNR_DB]); optRow(o, OI_K_SNR, 0, "Detection", v);
    snprintf(v, sizeof(v), "%u pings", sonarPrm[P_CONFIRM]); optRow(o, OI_K_CONFIRM, 0, "Confirm", v);
    if (settings.unitsMetric) snprintf(v, sizeof(v), "%u.%u m", sonarPrm[P_DEADZONE_DM] / 10, sonarPrm[P_DEADZONE_DM] % 10);
    else snprintf(v, sizeof(v), "%.1f ft", sonarPrm[P_DEADZONE_DM] * 0.328084f);
    optRow(o, OI_K_DEAD, 0, "Dead zone", v);
    snprintf(v, sizeof(v), "%u cm", sonarPrm[P_GATE_CM]); optRow(o, OI_K_GATE, 0, "Max fish move", v);
    optRow(o, OI_K_RESEND, 0, "Send to holes again", "");
    optRow(o, OI_K_DEFAULTS, 0, "Defaults", "");
  } else if (optMenu == OM_SIM) {
    if (meshDemoHubs()) snprintf(v, sizeof(v), "%u hubs", meshDemoHubs()); else snprintf(v, sizeof(v), "off");
    optRow(o, OI_S_DEMO, 0, "Demo network", v);
    if (meshDemoHubs()) { snprintf(v, sizeof(v), "%u", meshDemoHoles()); optRow(o, OI_S_DEMOHOLES, 0, "Holes per hub", v); }
    optRow(o, OI_S_ALL, 0, "All holes", simAnyOn() ? "ON" : "OFF");
    optRow(o, OI_S_VIRTUAL, 0, "Test holes", settings.sonarSim ? "ON" : "OFF");
    snprintf(v, sizeof(v), "%u /h", simRate); optRow(o, OI_S_RATE, 0, "Fake fish rate", v);
    static const char* const SIMTXT[] = {"off", "sonar", "fish", "both"};
    for (int i = 0; i < network.node_count && optN < OPT_MAX; i++) {
      const NodeState& n = network.nodes[i];
      if (n.node_id == 0 || n.node_id == NODE_ID || !n.initialized) continue;
      const uint8_t dv = meshDemoSim(n.node_id);   // demo hole: what it runs (no confirmation needed)
      const uint8_t req = (dv != 0xFF ? dv : simRequested(n.node_id)) & 3;
      const bool on = HAS_FLAG(n.flags, FLAG_SIM);
      snprintf(v, sizeof(v), "%s%s", SIMTXT[req], dv == 0xFF && (req != 0) != on ? "~" : "");   // ~ = the hole has not confirmed yet
      char nm[20]; if (n.name[0]) snprintf(nm, sizeof(nm), "%s", n.name); else snprintf(nm, sizeof(nm), "Hole %u", n.node_id);
      optRow(o, OI_S_HOLE, (uint8_t)i, nm, v);
    }
  }
  if (optSel >= optN) optSel = optN ? optN - 1 : 0;
  if (!o) return;
  o->mode = optMode; o->n = optN; o->sel = optSel;
  static const char* const TITLES[] = {"Settings", "Wi-Fi", "Choose Wi-Fi", "Simulation", "Display", "Sonar (all holes)"};
  snprintf(o->title, sizeof(o->title), "%s", optMenu < 6 ? TITLES[optMenu] : "");
  snprintf(o->hint, sizeof(o->hint), "%s", !cardKbAvailable ? "CardKB or web page" : optN ? optHint(optRows[optSel].item) : "Esc: back");
  o->line[0] = 0; o->text[0] = 0;
  if (optMode == SO_TEXT) {
    snprintf(o->title, sizeof(o->title), "%s", optTextStep == 0 ? "Wi-Fi name" : "Wi-Fi password");
    if (optTextStep == 0) snprintf(o->line, sizeof(o->line), "Type the network name");
    else snprintf(o->line, sizeof(o->line), "for %s", optSsid);
    snprintf(o->text, sizeof(o->text), "%s", optText);
    snprintf(o->hint, sizeof(o->hint), "%s", optTextStep == 0 ? "Enter: next (password)" : "Enter: connect");
  } else if (optMode == SO_CONFIRM) {
    if (optConfirm == OI_NETRESET) { snprintf(o->title, sizeof(o->title), "Reset network?"); snprintf(o->line, sizeof(o->line), "Link + channel to Auto"); }
    else if (optConfirm == OI_REBOOT) { snprintf(o->title, sizeof(o->title), "Reboot?"); snprintf(o->line, sizeof(o->line), "Back in about 10 s"); }
    else if (optConfirm == OI_K_DEFAULTS) { snprintf(o->title, sizeof(o->title), "Sonar defaults?"); snprintf(o->line, sizeof(o->line), "for every sonar hole"); }
    else { snprintf(o->title, sizeof(o->title), "Forget Wi-Fi?"); snprintf(o->line, sizeof(o->line), "%s", storedSsid); }
  }
}

static void optStartScan() {
  optScanN = -2; wifiScanBusy = true;
  if (WiFi.getMode() == WIFI_MODE_AP) WiFi.mode(WIFI_AP_STA);
  if (WiFi.scanNetworks(true, false) == WIFI_SCAN_FAILED) { optScanN = -3; wifiScanBusy = false; }
}

// polls the Wi-Fi scan started from Options > Wi-Fi > Choose network (async: the loop keeps running)
void loopOptions() {
  if (!wifiScanBusy) return;
  const int16_t r = WiFi.scanComplete();
  if (r == WIFI_SCAN_RUNNING) return;
  wifiScanBusy = false;
  if (r < 0) { optScanN = -3; return; }
  optScanN = 0;
  const size_t own = strlen(NETWORK_NAME);
  for (int16_t i = 0; i < r; i++) {
    const String ss = WiFi.SSID(i);
    if (!ss.length() || (strncmp(ss.c_str(), NETWORK_NAME, own) == 0 && ss[own] == '-')) continue;   // hidden, or our own hotspots
    const int8_t rs = (int8_t)WiFi.RSSI(i);
    int k = -1;
    for (int j = 0; j < optScanN; j++) if (strcmp(optScanSsid[j], ss.c_str()) == 0) k = j;
    if (k >= 0) { if (rs > optScanRssi[k]) optScanRssi[k] = rs; continue; }   // same name, several access points
    if (optScanN < (int8_t)SCAN_MAX) k = optScanN++;
    else {
      int w = 0; for (int j = 1; j < (int)SCAN_MAX; j++) if (optScanRssi[j] < optScanRssi[w]) w = j;
      if (rs <= optScanRssi[w]) continue;
      k = w;
    }
    snprintf(optScanSsid[k], sizeof(optScanSsid[k]), "%s", ss.c_str());
    optScanRssi[k] = rs; optScanLock[k] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
  }
  WiFi.scanDelete();
  for (int a = 1; a < optScanN; a++)   // strongest first
    for (int b = a; b > 0 && optScanRssi[b] > optScanRssi[b - 1]; b--) {
      char t[33]; memcpy(t, optScanSsid[b], 33); memcpy(optScanSsid[b], optScanSsid[b - 1], 33); memcpy(optScanSsid[b - 1], t, 33);
      const int8_t tr = optScanRssi[b]; optScanRssi[b] = optScanRssi[b - 1]; optScanRssi[b - 1] = tr;
      const bool tl = optScanLock[b]; optScanLock[b] = optScanLock[b - 1]; optScanLock[b - 1] = tl;
    }
}

static void optOpen(uint8_t menu) {
  if (optMenu == OM_MAIN) optMainSel = optSel;
  if (optMenu == OM_WIFI) optWifiSel = optSel;
  optMenu = menu; optSel = 0; optMode = SO_LIST;
}

static void optBack() {
  if (optMenu == OM_SCAN) { optMenu = OM_WIFI; optSel = optWifiSel; }
  else { optMenu = OM_MAIN; optSel = optMainSel; }
}

static void optAfterJoin() {
  optMode = SO_LIST; optMenu = OM_WIFI; optSel = 0;   // on "Status": connecting... -> connected / not found / bad password?
  memset(optText, 0, sizeof(optText));
}

static void optTextEnter() {
  if (optTextStep == 0) {
    if (!optText[0]) return;
    snprintf(optSsid, sizeof(optSsid), "%s", optText);
    optTextStep = 1;
    snprintf(optText, sizeof(optText), "%s", strcmp(optSsid, storedSsid) == 0 ? storedPassword : "");
  } else {
    wifiJoin(optSsid, optText);
    optAfterJoin();
  }
}

static void optDo(uint8_t item) {
  switch (item) {
    case OI_NETRESET: resetNetworkSettings(); break;
    case OI_REBOOT: display.clearBuffer(); display.setFont(u8g2_font_6x10_tr); display.drawStr(30, 36, "Rebooting..."); display.sendBuffer(); delay(500); ESP.restart(); break;
    case OI_W_FORGET: wifiForget(); break;
    case OI_K_DEFAULTS: for (uint8_t i = 0; i < icemesh::sonar::P_COUNT; i++) sonarKnobSet(i, icemesh::sonar::paramInfo(i).def); break;
    default: break;
  }
}

static void optActivate(const OptRow& row) {
  switch (row.item) {
    case OI_WIFI: optOpen(OM_WIFI); break;
    case OI_SIM: optOpen(OM_SIM); break;
    case OI_BUZZER: settings.buzzerEnabled = !settings.buzzerEnabled; saveSettings(); if (!settings.buzzerEnabled) buzzerStop(); break;
    case OI_BUZZTEST: buzzerTest(); break;
    case OI_NEARBEEP: settings.nearBaitBeep = !settings.nearBaitBeep; saveSettings(); break;
    case OI_BUZZTYPE: settings.buzzerPassive = !settings.buzzerPassive; saveSettings(); buzzerApplyType(); buzzerTest(); break;
    case OI_HOLD: {
      uint8_t k = 0; while (k < 5 && HOLD_STEPS[k] != settings.alarmHoldMin) k++;
      settings.alarmHoldMin = (uint8_t)HOLD_STEPS[(k + 1) % 5]; saveSettings(); break;
    }
    case OI_LINK: {
      const uint8_t t = (uint8_t)((settings.transportMode + 1) % 3);
      settings.transportMode = t; saveSettings(); meshSetTransport(t); break;
    }
    case OI_LORACH: {
      const uint8_t c = settings.loraChannel;
      const uint8_t nx = c == MESH_CH_AUTO ? 0 : (c >= 7 ? MESH_CH_AUTO : (uint8_t)(c + 1));
      settings.loraChannel = nx; saveSettings(); meshSetLoraChannel(nx); break;
    }
    case OI_HOTSPOT: if (wifiApActive) hubHotspotOff(); else hubHotspotOn(); break;
    case OI_RELAY: settings.ebRelay = !settings.ebRelay; saveSettings(); meshSetEbRelay(settings.ebRelay); break;
    case OI_REED: settings.reedActiveHigh = !settings.reedActiveHigh; saveSettings(); break;
    case OI_NETRESET: case OI_REBOOT: case OI_W_FORGET: optConfirm = row.item; optMode = SO_CONFIRM; break;
    case OI_W_CHOOSE: optOpen(OM_SCAN); optStartScan(); break;
    case OI_W_TYPE: optTextStep = 0; snprintf(optText, sizeof(optText), "%s", storedSsid); optMode = SO_TEXT; break;
    case OI_W_RETRY: wifiRetryNow(); break;
    case OI_SC_AGAIN: optSel = 0; optStartScan(); break;
    case OI_SC_NET:
      snprintf(optSsid, sizeof(optSsid), "%s", optScanSsid[row.idx]);
      if (!optScanLock[row.idx]) { wifiJoin(optSsid, ""); optAfterJoin(); }
      else { optTextStep = 1; snprintf(optText, sizeof(optText), "%s", strcmp(optSsid, storedSsid) == 0 ? storedPassword : ""); optMode = SO_TEXT; }
      break;
    case OI_S_ALL: simAll(!simAnyOn()); break;
    case OI_DISPLAY: optOpen(OM_DISPLAY); break;
    case OI_SONAR: optOpen(OM_SONAR); break;
    case OI_D_UNITS: settings.unitsMetric = !settings.unitsMetric; saveSettings(); break;
    case OI_D_SONDEPTH: settings.sonDepthIdx = (uint8_t)((settings.sonDepthIdx + 1) % 9); saveSettings(); break;
    case OI_D_FOCDEPTH: settings.focusDepthIdx = (uint8_t)((settings.focusDepthIdx + 1) % 9); saveSettings(); break;
    case OI_D_WEAK: settings.hideWeak = !settings.hideWeak; saveSettings(); break;
    case OI_K_FILTER: {
      using namespace icemesh::sonar;
      const uint8_t cur = presetOf(sonarPrm), nx = cur >= FILT_HIGH ? FILT_LOW : (uint8_t)(cur + 1);
      Params q = sonarPrm; applyPreset(q, nx);
      for (uint8_t i = 0; i < P_COUNT; i++) if (q[i] != sonarPrm[i]) sonarKnobSet(i, q[i]);
      break;
    }
    case OI_K_SNR: { static const uint8_t S[] = {6, 8, 10, 12, 14, 18, 24}; sonarKnobSet(icemesh::sonar::P_SNR_DB, nextStep(S, 7, sonarPrm[icemesh::sonar::P_SNR_DB])); break; }
    case OI_K_CONFIRM: { static const uint8_t S[] = {1, 2, 3, 4, 5, 6, 8}; sonarKnobSet(icemesh::sonar::P_CONFIRM, nextStep(S, 7, sonarPrm[icemesh::sonar::P_CONFIRM])); break; }
    case OI_K_DEAD: { static const uint8_t S[] = {3, 5, 9, 12, 15, 20, 30}; sonarKnobSet(icemesh::sonar::P_DEADZONE_DM, nextStep(S, 7, sonarPrm[icemesh::sonar::P_DEADZONE_DM])); break; }
    case OI_K_GATE: { static const uint8_t S[] = {16, 24, 32, 48, 64, 96}; sonarKnobSet(icemesh::sonar::P_GATE_CM, nextStep(S, 6, sonarPrm[icemesh::sonar::P_GATE_CM])); break; }
    case OI_K_RESEND: sonarKnobsResend(); break;
    case OI_K_DEFAULTS: optConfirm = row.item; optMode = SO_CONFIRM; break;
    case OI_S_DEMO: demoSet((uint8_t)((meshDemoHubs() + 1) % 5), meshDemoHoles()); break;   // off -> 1 -> 2 -> 3 -> 4 -> off
    case OI_S_DEMOHOLES: demoSet(meshDemoHubs(), (uint8_t)(meshDemoHoles() % 4 + 1)); break;
    case OI_S_VIRTUAL: settings.sonarSim = !settings.sonarSim; meshSetSonarSim(settings.sonarSim); saveSettings(); break;
    case OI_S_RATE: {
      uint8_t k = 0; while (k < 5 && RATE_STEPS[k] <= simRate) k++;
      simRate = RATE_STEPS[k % 5]; break;
    }
    case OI_S_HOLE: {
      if (row.idx >= network.node_count) break;
      const uint8_t node = network.nodes[row.idx].node_id;
      const uint8_t dv = meshDemoSim(node);
      const uint8_t nx = (uint8_t)(((dv != 0xFF ? dv : simRequested(node)) + 1) & 3);   // off -> sonar -> fish -> both -> off
      simRequest(node, (nx & MESH_SIM_SONAR) != 0, (nx & MESH_SIM_HALL) != 0);
      break;
    }
    default: break;
  }
}

// CardKB key on the Options page. true = used here; false = the normal page keys (left/right/Esc on the main list)
static bool optKey(uint8_t k) {
  if (optMode == SO_TEXT) {
    const size_t n = strlen(optText);
    if (k == KEY_ESC) optMode = SO_LIST;
    else if (k == KEY_ENTER) optTextEnter();
    else if (k == KEY_BACKSP || k == 0x7F) { if (n) optText[n - 1] = 0; }
    else if (k >= 32 && k < 127 && n < (optTextStep == 0 ? 32u : 63u)) { optText[n] = (char)k; optText[n + 1] = 0; }
    return true;   // arrows are ignored while typing
  }
  if (optMode == SO_CONFIRM) {
    if (k == KEY_ENTER) optDo(optConfirm);
    optMode = SO_LIST; optConfirm = OI_NONE;
    return true;
  }
  optBuild(nullptr);
  switch (k) {
    case KEY_UP: if (optSel > 0) optSel--; return true;
    case KEY_DOWN: if (optSel + 1 < optN) optSel++; return true;
    case KEY_ENTER: if (optN) optActivate(optRows[optSel]); return true;
    case KEY_ESC: case KEY_LEFT: if (optMenu != OM_MAIN) { optBack(); return true; } return false;
    case KEY_RIGHT: return optMenu != OM_MAIN;   // inside Wi-Fi / Simulation: stay
    default: return false;
  }
}

static bool scrPageAvailable(uint8_t p) {
  const bool chalet = currentRole == ROLE_GATEWAY_OFFSHORE;
  if (!chalet && (p == PG_SONAR || p == PG_FOCUS || p == PG_TEST)) return false;   // the hub has no sonar store
  if (p == PG_SONAR || p == PG_FOCUS) { for (uint8_t i = 0; i < scr.n_holes; i++) if (scr.holes[i].son.valid) return true; return false; }
  return true;
}

static void scrStep(int dir) {
  scrMenu = false;   // the button (or left/right) shows a page: the CardKB menu closes
  for (uint8_t k = 0; k < PG_COUNT; k++) {
    scrPage = (uint8_t)((scrPage + PG_COUNT + dir) % PG_COUNT);
    if (scrPageAvailable(scrPage)) break;
  }
  scrSub = 0;
  scrLastInput = millis();
}

// ---- CardKB menu: Esc opens the list of pages, up/down + Enter opens one, Esc in a page comes back here ----
static const uint8_t MENU_ORDER[] = {PG_HOME, PG_HOLES, PG_SONAR, PG_FOCUS, PG_NETWORK, PG_CONNECT, PG_TEST, PG_OPTIONS};
static uint8_t menuPages[PG_COUNT], menuN = 0;

static void menuBuild(ScrOptions* o) {
  static const char* const NAMES[PG_COUNT] = {"Home", "Sonar", "Focus", "Holes", "Network", "Test / simulation", "Connect phone", "Settings"};
  menuN = 0;
  for (uint8_t k = 0; k < sizeof(MENU_ORDER); k++) {
    const uint8_t p = MENU_ORDER[k];
    if (!scrPageAvailable(p)) continue;
    if (o) {
      ScrRow& r = o->rows[menuN];
      snprintf(r.label, sizeof(r.label), "%s", NAMES[p]); r.value[0] = 0; r.sub = false;
      if (p == PG_HOLES) snprintf(r.value, sizeof(r.value), "%u", scr.n_holes);
      else if (p == PG_NETWORK) snprintf(r.value, sizeof(r.value), "ch%u", scr.lora_ch);
      else if (p == PG_CONNECT && scr.chalet) snprintf(r.value, sizeof(r.value), "%s", scr.sta ? "Wi-Fi OK" : "hotspot");
      else if (p == PG_TEST && simAnyOn()) snprintf(r.value, sizeof(r.value), "SIM ON");
    }
    menuPages[menuN++] = p;
  }
  if (scrMenuSel >= menuN) scrMenuSel = menuN ? menuN - 1 : 0;
  if (!o) return;
  o->mode = SO_LIST; o->n = menuN; o->sel = scrMenuSel;
  snprintf(o->title, sizeof(o->title), "Menu");
  snprintf(o->hint, sizeof(o->hint), "OK: open   Esc: home");
}

static void menuOpen() {
  menuBuild(nullptr);
  scrMenuSel = 0;
  for (uint8_t i = 0; i < menuN; i++) if (menuPages[i] == scrPage) scrMenuSel = i;
  scrMenu = true;
  scrLastInput = millis();
}

// key while the menu is open
static void menuKey(uint8_t k) {
  menuBuild(nullptr);
  switch (k) {
    case KEY_UP: if (scrMenuSel > 0) scrMenuSel--; break;
    case KEY_DOWN: if (scrMenuSel + 1 < menuN) scrMenuSel++; break;
    case KEY_ENTER: case KEY_RIGHT:
      if (!menuN) break;
      scrPage = menuPages[scrMenuSel]; scrSub = 0; scrMenu = false;
      if (scrPage == PG_OPTIONS) { optMenu = OM_MAIN; optSel = 0; optMode = SO_LIST; }
      break;
    case KEY_ESC: case KEY_LEFT: scrMenu = false; scrPage = PG_HOME; scrSub = 0; break;
    default: break;
  }
}

// FOCUS: the next / previous hole that has a sonar
static void focusStep(int dir) {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) return;
  int ids[48], n = 0, cur = -1;
  for (uint8_t i = 0; i < scr.n_holes; i++) {
    if (!scr.holes[i].son.valid) continue;
    if (scr.holes[i].id == scr.focus_node) cur = n;
    ids[n++] = scr.holes[i].id;
  }
  if (!n) return;
  const int pick = cur < 0 ? 0 : (cur + n + dir) % n;
  meshSetFocusNode((uint8_t)ids[pick]);
}

// CardKB up/down: move inside the page
static void scrScroll(int dir) {
  scrLastInput = millis();
  switch (scrPage) {
    case PG_HOLES: { const uint8_t n = screenHolesPages(scr); scrSub = (uint8_t)((scrSub + n + dir) % n); break; }
    case PG_SONAR: { const uint8_t n = screenSonarPages(scr); scrSub = (uint8_t)((scrSub + n + dir) % n); break; }
    case PG_FOCUS: focusStep(dir); break;
    default: break;
  }
}

// double press / Enter: the action written at the bottom of the page
static void scrAction(bool kb) {
  scrLastInput = millis();
  switch (scrPage) {
    case PG_HOME: if (alertsSilenced) { silenceAlerts(); showOverlayMessage("Unsilenced", 800); } else if (kb) menuOpen(); break;
    case PG_HOLES: scrSub = (uint8_t)((scrSub + 1) % screenHolesPages(scr)); break;
    case PG_SONAR: scrSub = (uint8_t)((scrSub + 1) % screenSonarPages(scr)); break;
    case PG_FOCUS: focusStep(+1); break;
    case PG_TEST: if (currentRole == ROLE_GATEWAY_OFFSHORE) { const bool on = !simAnyOn(); simAll(on); showOverlayMessage(on ? "Simulation ON" : "Simulation OFF", 1000); } break;
    case PG_CONNECT: if (kb && currentRole == ROLE_GATEWAY_OFFSHORE) { scrPage = PG_OPTIONS; optMenu = OM_MAIN; optOpen(OM_WIFI); } break;
    default: break;
  }
}

static void scrDraw() {
  const uint32_t idle = scrPage == PG_OPTIONS ? 180000UL : 60000UL;   // typing a password takes time
  if ((scrPage != PG_HOME || scrMenu) && millis() - scrLastInput > idle) { scrPage = PG_HOME; scrSub = 0; optMode = SO_LIST; scrMenu = false; }
  buildScreenModel();
  if (!scrPageAvailable(scrPage)) scrPage = PG_HOME;
  if (scrPage == PG_NETWORK && scr.radio_test) { drawRadioTest(); return; }   // radio test: detailed per-hub stats
  screenDraw(display.getU8g2(), scr, scrPage, scrSub, ((millis() / 500) % 2) == 0);
  // v2: send only when the picture changed (or every 2 s, or after another screen used the display)
  uint32_t h = 2166136261u;
  const uint8_t* b = display.getBufferPtr();
  for (int i = 0; i < 1024; i++) { h ^= b[i]; h *= 16777619u; }
  static uint32_t lastSent = 0, lastHash = 0;
  if (h != lastHash || scrDirty || millis() - lastSent > 2000UL) {
    display.sendBuffer();
    lastHash = h; lastSent = millis(); scrDirty = false;
  }
}

// =============================================================================================
// v2 PRG BUTTON (one button, same on chalet and hubs; docs/FIELD_GUIDE_NETWORK.md)
//   short press        : during an alert (or while silenced) silence on/off, otherwise wake / next page
//   hold 3 s, release  : hub = hotspot on/off (30 min, kept while a phone is connected); chalet = show
//                        how to connect (network, password, address)
//   hold 10 s, release : reset the NETWORK settings to defaults (link Auto, LoRa channel Auto, relay
//                        default, chalet ESP-NOW normal rate). Alerts, names and Wi-Fi credentials are kept.
// While the button is held the screen shows a bar with both marks, so the user sees what release does.
// =============================================================================================
static const uint32_t HOLD_CONNECT_MS = 3000, HOLD_RESET_MS = 10000;
static const uint32_t HUB_HOTSPOT_MS = 30UL * 60UL * 1000UL;
static void apName(char* out, size_t n) {
  if (currentRole == ROLE_GATEWAY_OFFSHORE) snprintf(out, n, "%s-Remote", NETWORK_NAME);
  else snprintf(out, n, "%s-Hub%u", NETWORK_NAME, (unsigned)NODE_ID);
}

// Hub hotspot on demand: AP on the ESP-NOW channel next to the station interface that ESP-NOW uses.
// UNTESTED on hardware: that ESP-NOW keeps working through the mode change (to check on the bench).
void hubHotspotOn() {
  if (currentRole == ROLE_GATEWAY_OFFSHORE) return;
  char ssid[32]; apName(ssid, sizeof(ssid));
  if (!wifiApActive) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ssid, WIFI_PASSWORD, ESPNOW_CHANNEL, 0, AP_MAX_CONNECTIONS);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    apIpAddress = WiFi.softAPIP().toString();
    wifiApActive = true;
    Serial.printf("Hotspot ON: %s / %s, http://%s\n", ssid, WIFI_PASSWORD, apIpAddress.c_str());
  }
  hubHotspotUntil = millis() + HUB_HOTSPOT_MS;
  if (hubHotspotUntil == 0) hubHotspotUntil = 1;
}

void hubHotspotOff() {
  if (currentRole == ROLE_GATEWAY_OFFSHORE || !wifiApActive) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  wifiApActive = false;
  hubHotspotUntil = 0;
  Serial.println(F("Hotspot OFF"));
}

static void loopHubHotspot() {
  if (currentRole == ROLE_GATEWAY_OFFSHORE || !wifiApActive || hubHotspotUntil == 0) return;
  if (static_cast<int32_t>(millis() - hubHotspotUntil) < 0) return;
  if (WiFi.softAPgetStationNum() > 0) { hubHotspotUntil = millis() + 5UL * 60UL * 1000UL; return; }   // phone still on it
  hubHotspotOff();
}

void resetNetworkSettings() {
  settings.transportMode = MESH_TR_AUTO;
  settings.loraChannel = MESH_CH_AUTO;
  settings.ebRelay = (currentRole != ROLE_GATEWAY_OFFSHORE);
  settings.ebChaletLr = false;
  saveSettings();
  if (currentRole == ROLE_GATEWAY_OFFSHORE) { meshSetTransport(MESH_TR_AUTO); meshSetLoraChannel(MESH_CH_AUTO); }
  meshSetEbRelay(settings.ebRelay);
  Serial.println(F("Network settings reset to defaults (link Auto, LoRa channel Auto, relay default)"));
}

// Hold bar: 0..10 s, marks at 3 s and 10 s, label of what releasing now does.
static void drawHoldBar(uint32_t held) {
  scrDirty = true;
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  const bool hub = currentRole != ROLE_GATEWAY_OFFSHORE;
  const char* now_lbl = held >= HOLD_RESET_MS ? "Release: RESET network" :
                        held >= HOLD_CONNECT_MS ? (hub ? (wifiApActive ? "Release: hotspot OFF" : "Release: hotspot ON") : "Release: connect info")
                                                : "Keep holding...";
  display.drawStr(0, 12, now_lbl);
  const int x0 = 4, w = 120, y = 26;
  display.drawFrame(x0, y, w, 10);
  const uint32_t h = held > HOLD_RESET_MS ? HOLD_RESET_MS : held;
  display.drawBox(x0 + 1, y + 1, static_cast<int>((w - 2) * h / HOLD_RESET_MS), 8);
  const int m3 = x0 + static_cast<int>(w * HOLD_CONNECT_MS / HOLD_RESET_MS);
  display.drawVLine(m3, y - 3, 16);
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(m3 - 10, y + 22, hub ? "3s hotspot" : "3s connect");
  display.drawStr(84, y + 22, "10s reset");
  display.sendBuffer();
}

// "How to connect" screen (chalet: always; hub: while its hotspot is on)
static void drawConnectInfo() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tr);
  char ssid[32]; apName(ssid, sizeof(ssid));
  char line[40];
  if (currentRole != ROLE_GATEWAY_OFFSHORE && !wifiApActive) {
    display.drawStr(0, 12, "Hotspot is OFF");
    display.drawStr(0, 28, "Hold button 3 s");
    display.drawStr(0, 40, "to turn it on");
    display.sendBuffer();
    return;
  }
  display.drawStr(0, 10, "Connect your phone:");
  snprintf(line, sizeof(line), "WiFi %s", wifiApActive ? ssid : "-");
  display.drawStr(0, 24, line);
  snprintf(line, sizeof(line), "Pass %s", WIFI_PASSWORD);
  display.drawStr(0, 36, line);
  snprintf(line, sizeof(line), "http://%s", wifiStaConnected ? staIpAddress.c_str() : apIpAddress.c_str());
  display.drawStr(0, 48, line);
  if (currentRole != ROLE_GATEWAY_OFFSHORE && hubHotspotUntil != 0) {
    snprintf(line, sizeof(line), "off in %lu min", (unsigned long)((hubHotspotUntil - millis()) / 60000UL + 1));
    display.setFont(u8g2_font_5x7_tr);
    display.drawStr(0, 62, line);
  }
  display.sendBuffer();
}

void loopButton() {
  static unsigned long buttonPressStart = 0;
  static uint8_t buttonPressCount = 0;
  static unsigned long lastButtonRelease = 0;
  static unsigned long lastBarDraw = 0;
  static bool wasAsleep = false, screenWasAsleep = false;

  loopHubHotspot();
  bool pressed = (digitalRead(USER_BUTTON) == LOW);
  unsigned long now = millis();

  if (pressed && buttonPressStart == 0) {
    buttonPressStart = now;
    wasAsleep = displaySleeping;
  } else if (pressed) {
    const uint32_t held = now - buttonPressStart;
    if (held >= 1000) {                       // show the hold bar from 1 s on
      if (buttonHeldMs == 0) registerActivity();
      buttonHeldMs = held;
      if (now - lastBarDraw >= 100) { lastBarDraw = now; drawHoldBar(held); }
    }
  } else if (buttonPressStart > 0) {
    const unsigned long pressDuration = now - buttonPressStart;
    buttonPressStart = 0;
    buttonHeldMs = 0;
    registerActivity();
    if (pressDuration >= HOLD_RESET_MS) {
      resetNetworkSettings();
      showOverlayMessage("Network reset", 1500);
    } else if (pressDuration >= HOLD_CONNECT_MS) {
      if (currentRole != ROLE_GATEWAY_OFFSHORE) { if (wifiApActive) hubHotspotOff(); else hubHotspotOn(); }
      scrPage = PG_CONNECT; scrSub = 0; scrLastInput = millis(); scrMenu = false;
    } else if (pressDuration >= 1000) {
      // released between 1 and 3 s: nothing (the bar told the user to keep holding)
    } else if (pressDuration > BUTTON_DEBOUNCE_MS) {
      if (buttonPressCount == 0) screenWasAsleep = wasAsleep;
      if (now - lastButtonRelease < 500) buttonPressCount++;
      else buttonPressCount = 1;
      lastButtonRelease = now;
    }
  }

  // Process button count after settle time (500ms after last release)
  if (buttonPressCount > 0 && (now - lastButtonRelease > 500)) {
    if (buttonPressCount == 1) {
      if (activeAlerts && !alertsSilenced) {   // alert on screen: the button is the silence button
        DEBUG_PRINTLN(F("PRG single press - silence"));
        silenceAlerts();
        showOverlayMessage("Silenced", 1000);
      } else if (!screenWasAsleep) {
        scrStep(+1);                           // next page (a press that woke the screen only wakes it)
      }
    } else if (buttonPressCount >= 2) {
      scrAction();                             // the page's action (written at the bottom)
    }
    buttonPressCount = 0;
  }
}

void handleKeyPress(char key) {
  // v2 CardKB: Esc = menu (list of pages: up/down, Enter = open, Esc = home); in a page up/down = inside
  // the page, Enter = OK (the action in the footer), left/right = previous/next page, s = silence.
  // On Settings the keys go to the list / text entry first (Esc there = up one level, then the menu).
  const bool wasAsleep = displaySleeping;
  registerActivity();
  if (wasAsleep) return;   // the first key only wakes the screen
  const uint8_t k = (uint8_t)key;
  DEBUG_PRINTF("Key: 0x%02X\n", k);
  scrLastInput = millis();
  if (activeAlerts && !alertsSilenced) {   // FISH ON screen: any key silences (like the button)
    silenceAlerts(); showOverlayMessage("Silenced", 800);
    return;
  }
  if (scrMenu) { menuKey(k); return; }
  if (scrPage == PG_OPTIONS && optKey(k)) return;
  switch (k) {
    case KEY_RIGHT: case KEY_TAB: case ' ': case 'n': case 'N': scrStep(+1); break;
    case KEY_LEFT: case 'p': case 'P': scrStep(-1); break;
    case KEY_DOWN: scrScroll(+1); break;
    case KEY_UP: scrScroll(-1); break;
    case KEY_ENTER: scrAction(true); break;
    case 's': case 'S': silenceAlerts(); showOverlayMessage(alertsSilenced ? "Silenced" : "Unsilenced", 800); break;
    case KEY_ESC: menuOpen(); break;   // back to the menu, pick another page
    case '+': case '=': case '-': case '_': case 'a': case 'A':   // v2: depth scale of the Sonar / Focus page
      if (scrPage == PG_SONAR || scrPage == PG_FOCUS) {
        uint8_t& idx = scrPage == PG_SONAR ? settings.sonDepthIdx : settings.focusDepthIdx;
        if (k == 'a' || k == 'A') idx = 0;
        else {
          const bool in = (k == '+' || k == '=');
          if (idx == 0) {   // from auto: the fixed scale just around what auto shows now
            const uint16_t cur = scrPage == PG_SONAR ? screenSonarAutoRange(scr) : screenFocusAutoRange(scr);
            uint8_t j = 1; while (j < 8 && depthStepCm(j) < cur) j++;
            idx = in ? (j > 1 ? j - 1 : 1) : j;
          } else if (in) { if (idx > 1) idx--; }
          else if (idx < 8) idx++;
        }
        saveSettings();
      }
      break;
    default: break;
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
        case 6:  // Reset All Nodes - hub: its own nodes; chalet: whole network (v2 beacon command)
          if (currentRole == ROLE_GATEWAY_ONSHORE || currentRole == ROLE_GATEWAY_OFFSHORE) {
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

// =============================================================================================
// v2 SONAR KNOBS (D42, lib/IceMesh/src/sonar_params.h): one set for every sonar hole.
// Chalet: set from the OLED (Settings > Sonar) or the web page, saved, sent in the beacon one knob per
// command (CMD_SONAR_PARAM), and the whole set again when a hub shows up after boot. Hub: keeps them
// (NVS), uses them for its own fake sonars and repeats them to its tip-ups in the sonar control message.
// =============================================================================================
void sonarKnobsLoad() {
  Preferences p;
  p.begin("sonar", true);
  uint8_t v[icemesh::sonar::P_COUNT];
  const size_t n = p.getBytes("p", v, sizeof(v));
  p.end();
  for (uint8_t i = 0; i < n && i < icemesh::sonar::P_COUNT; i++) sonarPrm.set(i, v[i]);   // clamped; newer knobs keep their default
  sonarPrmGen++;
  if (currentRole == ROLE_GATEWAY_OFFSHORE) meshDemoSetParams(sonarPrm.v, icemesh::sonar::P_COUNT);
}

static void sonarKnobsSave() {
  Preferences p;
  p.begin("sonar", false);
  p.putBytes("p", sonarPrm.v, icemesh::sonar::P_COUNT);
  p.end();
}

void sonarKnobApply(uint8_t id, uint8_t v) {
  if (!sonarPrm.set(id, v)) return;
  sonarKnobsSave();
  sonarPrmGen++;
  sonarPrmChangedMs = millis(); if (sonarPrmChangedMs == 0) sonarPrmChangedMs = 1;
  if (currentRole == ROLE_GATEWAY_OFFSHORE) meshDemoSetParams(sonarPrm.v, icemesh::sonar::P_COUNT);
  Serial.printf("Sonar knob %s = %u\n", icemesh::sonar::paramInfo(id).key, sonarPrm[id]);
}

void sonarKnobSet(uint8_t id, uint8_t v) {
  if (id >= icemesh::sonar::P_COUNT) return;
  sonarKnobApply(id, v);
  if (currentRole == ROLE_GATEWAY_OFFSHORE) meshSetSonarParam(id, sonarPrm[id]);
}

void sonarKnobsResend() {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) return;
  for (uint8_t i = 0; i < icemesh::sonar::P_COUNT; i++) meshSetSonarParam(i, sonarPrm[i]);
}

// chalet loop: a real hub heard for the first time since boot gets the whole set (it may have missed changes)
void sonarKnobsNewHubs() {
  static uint32_t last = 0;
  static uint8_t seen[16]; static uint8_t nseen = 0;
  if (millis() - last < 2000) return;
  last = millis();
  MeshHubLink hl[10];
  const uint8_t n = meshHubLinks(hl, 10);
  bool fresh = false;
  for (uint8_t k = 0; k < n; k++) {
    if (hl[k].demo) continue;
    bool known = false;
    for (uint8_t j = 0; j < nseen; j++) known |= seen[j] == hl[k].id;
    if (!known && nseen < sizeof(seen)) { seen[nseen++] = hl[k].id; fresh = true; }
  }
  if (fresh) { sonarKnobsResend(); baitResendAll(); Serial.println(F("Sonar knobs and bait depths sent again (new hub)")); }
}

// ---- v2 (D43) bait depth per hole -----------------------------------------------------------
void baitLoad() {
  Preferences p; p.begin("bait", true);
  memset(baitCm5, 0, sizeof(baitCm5));
  p.getBytes("b", baitCm5, sizeof(baitCm5));
  p.end();
  baitGen++;
}

static bool myTestHole(uint8_t id) {   // hub: its own hole or one of its virtual test holes
  if (id == NODE_ID) return true;
  for (uint8_t k = 0; k < 8; k++) if (meshSonarVirtualId(NODE_ID, k) == id) return true;
  return false;
}

void baitApply(uint8_t hole, uint8_t v5) {
  if (hole == 0 || hole == 255) return;
  if (baitCm5[hole] != v5) {
    baitCm5[hole] = v5;
    Preferences p; p.begin("bait", false); p.putBytes("b", baitCm5, sizeof(baitCm5)); p.end();
  }
  baitGen++;
  if (currentRole == ROLE_GATEWAY_OFFSHORE) meshDemoSetBait(hole, v5);
  else if (!myTestHole(hole) && v5) devCmdQueue(hole, DEVCMD_BAIT, v5);   // the tip-up gets it after its next message
  Serial.printf("Bait of hole %u: %u cm\n", hole, (unsigned)v5 * 5u);
}

void baitSet(uint8_t hole, uint8_t v5) {
  baitApply(hole, v5);
  if (currentRole == ROLE_GATEWAY_OFFSHORE && v5) meshSetBait(hole, v5);
}

void baitResendAll() {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) return;
  for (int i = 1; i < 255; i++) if (baitCm5[i]) meshSetBait((uint8_t)i, baitCm5[i]);
}

static void baitToSource(icemesh::sonar::SonarSource* src, uint8_t id) {
  if (src == nullptr || id == 0 || baitCm5[id] == 0) return;
  const float m = baitCm5[id] * 0.05f;
  src->proc.bait_m = m; src->scene.setBait(m);   // fake sonar: the lure follows too
}

// =============================================================================================
// v2 (D43) TRIP RECORDINGS: the last 60 s of sonar summaries of a hole, saved when its flag trips
// (labelled data for later: what the fish did before the flag). RAM only (8 newest), web download.
// NEAR-BAIT WATCH: a hole's sonar says "fish near the bait" (ST_NEAR_BAIT) -> note on the OLED / page,
// optional short beep (Settings > Near-bait beep). Never replaces FISH ON.
// =============================================================================================
uint8_t nearBaitNode = 0;                   // latest hole with a fish near its bait (OLED footer, page)
uint32_t nearBaitAt = 0;

void tripRecord(uint8_t node) {
  if (currentRole != ROLE_GATEWAY_OFFSHORE) return;
  TripRec& t = tripRecs[tripHead];
  t.node = node; t.at_s = millis() / 1000;
  t.n = meshSonarHistory(node, t.r, 30);
  if (t.n == 0) return;   // no sonar on that hole: nothing to keep
  tripHead = (uint8_t)((tripHead + 1) % 8); if (tripCount < 8) tripCount++;
  Serial.printf("Trip recording: hole %u, %u s of sonar before the flag\n", node, (unsigned)t.n * 2u);
}

void nearBaitWatch() {
  static uint32_t last = 0;
  static uint8_t was[256];
  if (currentRole != ROLE_GATEWAY_OFFSHORE || millis() - last < 1000) return;
  last = millis();
  for (int i = 1; i < network.node_count; i++) {
    const uint8_t id = network.nodes[i].node_id;
    MeshSonarLite sl;
    const bool near = meshSonarSummary(id, sl) && sl.age_frames < 10 && (sl.status & 0x02);
    if (near && !was[id]) {
      nearBaitNode = id; nearBaitAt = millis();
      if (settings.nearBaitBeep && settings.buzzerEnabled && !activeAlerts) triggerBuzzer(1);
      registerActivity();
    }
    was[id] = near ? 1 : 0;
  }
}

void loadSettings() {
  preferences.begin("settings", true);  // Read-only

  settings.buzzerEnabled = preferences.getBool("buzzer", true);
  settings.alertHoldSec = preferences.getUShort("alertHold", 30);
  settings.heartbeatSec = preferences.getUShort("heartbeat", 60);
  settings.displayBrightness = preferences.getUChar("brightness", 255);
  settings.webServerEnabled = preferences.getBool("webServer", true);
  settings.wifiModeSetting = preferences.getUChar("wifiMode", 2);  // Default: AP+STA
  settings.reedActiveHigh = preferences.getBool("reedHigh", true);  // Default: trigger on HIGH
  settings.radioTestMode = preferences.getUChar("radioTest", 0);
  settings.adaptiveRadio = preferences.getBool("adaptRadio", false);
  settings.sonarSim = preferences.getBool("sonarSim", false);
  settings.sonarVirtualNodes = preferences.getUChar("simNodes", 2);
  if (settings.sonarVirtualNodes > 4) settings.sonarVirtualNodes = 4;
  settings.transportMode = preferences.getUChar("transport", 0);
  if (settings.transportMode > 2) settings.transportMode = 0;
  settings.loraChannel = preferences.getUChar("loraCh", 255);
  if (settings.loraChannel > 7) settings.loraChannel = 255;
  settings.ebRelay = preferences.getBool("ebRelay", currentRole != ROLE_GATEWAY_OFFSHORE);   // v2: every hub relays by default
  settings.buzzerPassive = preferences.getBool("buzPas", false);
  settings.alarmHoldMin = preferences.getUChar("alarmHold", 0);
  settings.unitsMetric = preferences.getBool("metric", false);
  settings.sonDepthIdx = preferences.getUChar("sonDepth", 0); if (settings.sonDepthIdx > 8) settings.sonDepthIdx = 0;
  settings.focusDepthIdx = preferences.getUChar("focDepth", 0); if (settings.focusDepthIdx > 8) settings.focusDepthIdx = 0;
  settings.hideWeak = preferences.getBool("hideWeak", false);
  settings.nearBaitBeep = preferences.getBool("nearBeep", false);
  settings.ebChaletLr = preferences.getBool("ebChLr", false);
  settings.lastLoraCh = preferences.getUChar("lastLoraCh", 0);
  if (settings.lastLoraCh > 7) settings.lastLoraCh = 0;

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
  preferences.putUChar("radioTest", settings.radioTestMode);
  preferences.putBool("adaptRadio", settings.adaptiveRadio);
  preferences.putBool("sonarSim", settings.sonarSim);
  preferences.putUChar("simNodes", settings.sonarVirtualNodes);
  preferences.putUChar("transport", settings.transportMode);
  preferences.putUChar("loraCh", settings.loraChannel);
  preferences.putBool("ebRelay", settings.ebRelay);
  preferences.putBool("buzPas", settings.buzzerPassive);
  preferences.putUChar("alarmHold", settings.alarmHoldMin);
  preferences.putBool("metric", settings.unitsMetric);
  preferences.putUChar("sonDepth", settings.sonDepthIdx);
  preferences.putUChar("focDepth", settings.focusDepthIdx);
  preferences.putBool("hideWeak", settings.hideWeak);
  preferences.putBool("nearBeep", settings.nearBaitBeep);
  preferences.putBool("ebChLr", settings.ebChaletLr);
  preferences.putUChar("lastLoraCh", settings.lastLoraCh);

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
  
  // Clear silence when new alert comes in — and tell the network (v2: beacon / hub request)
  const bool wasSilenced = alertsSilenced;
  alertsSilenced = false;
  if (wasSilenced) { silenceTime = 0; silenceExpireTime = 0; sendSilenceSync(); }
  
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
    buzzerStop();   // v2: the beep being played stops at once
    alarmAck();     // v2: holes back to normal are cleared, tripped ones stay shown
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

void sendSilenceSyncEspNow() {
  if (!espNowReady) return;
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

void sendSilenceSync() {
  DEBUG_PRINTF("Broadcasting silence state: %d\n", alertsSilenced);
  sendSilenceSyncEspNow();                              // local tip-up nodes
  if (loraReady) meshRequestSilence(alertsSilenced);    // v2: chalet beacon carries the network state
}

bool sendRemoteConfig(uint8_t targetNodeId) {
  // v2: remote configuration over the TDMA mesh is not implemented yet (planned with the beacon
  // command field). The web page reports the failure.
  (void)targetNodeId;
  DEBUG_PRINTLN(F("Remote config: not available in mesh v2 yet"));
  return false;
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

  // v2: only the chalet can reset the network (beacon command, repeated for 5 s)
  if (loraReady && currentRole == ROLE_GATEWAY_OFFSHORE) {
    meshSendResetAll();
    DEBUG_PRINTLN("LoRa: Reset command in beacon");
    delay(6000);   // let hubs receive it (the radio task keeps running)
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

// ═══════════════════════════════════════════════════════════════════════════
// v2: TDMA MESH INTEGRATION (loop side). The radio runs in its own task (mesh_radio.cpp).
//   Hub:    network.nodes[] (ESP-NOW nodes + own tip-up) -> line-state table of the mesh
//   Chalet: owner-resolved node states from the mesh -> network.nodes[] (UI, web, buzzer)
// ═══════════════════════════════════════════════════════════════════════════

static volatile bool meshSyncPending = false;
void meshSyncNodesNow() { meshSyncPending = true; }

static uint8_t lineStateOf(const NodeState& n, bool self) {
  if (!self && !n.online) return MESH_LS_OFFLINE;
  if (HAS_FLAG(n.flags, FLAG_FISH_ON)) return MESH_LS_TRIPPED;
  if (HAS_FLAG(n.flags, FLAG_SENSOR_ERROR)) return MESH_LS_FAULT;
  return MESH_LS_IDLE;
}

static void applyMeshNodeUpdate(const MeshNodeUpdate& u) {
  if (u.node == 0 || u.node == NODE_ID) return;
  if (meshDemoNode(u.node) && meshDemoSim(u.node) == 0xFF) return;   // queued from a demo hole just removed
  const bool isNew = (findNodeIndexForMesh(u.node) < 0);
  NodeState* n = findOrCreateNode(u.node);
  if (n == nullptr) return;
  if (isNew) {
    String saved = loadNodeName(u.node);
    if (saved.length() > 0) { strncpy(n->name, saved.c_str(), sizeof(n->name) - 1); n->name[sizeof(n->name) - 1] = '\0'; }
    else if (meshDemoNode(u.node)) meshDemoName(u.node, n->name, sizeof(n->name));
  }
  const bool wasFish = HAS_FLAG(n->flags, FLAG_FISH_ON);
  const bool fish = (u.new_state == MESH_LS_TRIPPED || u.new_state == MESH_LS_RUNNING);
  n->initialized = true;
  n->via_lora = true;
  n->role = ROLE_SENSOR_ONLY;
  if (u.new_state == MESH_LS_OFFLINE) {
    n->online = false;
  } else {
    if (!n->online) registerActivity();
    n->online = true;
    n->last_seen = millis();
  }
  if (u.flags & MESH_LF_LOWBAT) SET_FLAG(n->flags, FLAG_LOW_BATTERY); else CLEAR_FLAG(n->flags, FLAG_LOW_BATTERY);
  if (u.flags & MESH_LF_SIM) SET_FLAG(n->flags, FLAG_SIM); else CLEAR_FLAG(n->flags, FLAG_SIM);
  if (u.new_state == MESH_LS_FAULT) SET_FLAG(n->flags, FLAG_SENSOR_ERROR); else CLEAR_FLAG(n->flags, FLAG_SENSOR_ERROR);
  if (fish && !wasFish) {
    SET_FLAG(n->flags, FLAG_FISH_ON);
    n->fish_on_time = millis();
    alarmStart(n);
    recordAlert(u.node);
    triggerBuzzer(3);
    registerActivity();
  } else if (!fish && wasFish) {
    CLEAR_FLAG(n->flags, FLAG_FISH_ON);
    n->fish_on_time = 0;
    alarmLineReset(n);
    for (int i = 0; i < ALERT_HISTORY_SIZE; i++)
      if (alertHistory[i].nodeId == u.node && alertHistory[i].active) alertHistory[i].active = false;
  }
  updateAlertState();
  network.last_update = millis();
  DEBUG_PRINTF("mesh: node %u state %u -> %u (hub %u)\n", u.node, u.old_state, u.new_state, u.owner);
}

static void networkResetFromChalet() {
  DEBUG_PRINTLN(F("RESET ALL from chalet: forwarding to nodes, then rebooting"));
  if (espNowReady) {
    ResetCmdMessage msg;
    msg.network_id = NETWORK_ID;
    msg.sender_id = NODE_ID;
    msg.msg_type = MSG_RESET_CMD;
    msg.reset_delay_sec = 0;
    msg.timestamp = millis() / 1000;
    esp_now_send(ESPNOW_BROADCAST, (uint8_t*)&msg, sizeof(msg));
  }
  showOverlayMessage("Network reset", 1500);
  delay(100 * (NODE_ID % 10));   // stagger hub reboots
  ESP.restart();
}

void meshLoop() {
  const bool chalet = (currentRole == ROLE_GATEWAY_OFFSHORE);
  static unsigned long lastSync = 0, lastSnap = 0, lastCounters = 0;

  if (!chalet) {
    if (meshSyncPending || millis() - lastSync >= 250) {
      meshSyncPending = false;
      lastSync = millis();
      meshHubSetBattery(localBatteryMv);
      for (int i = 0; i < network.node_count; i++) {
        const NodeState& n = network.nodes[i];
        const bool self = (i == 0);
        const uint8_t own_sim = self ? hubHoleSim(NODE_ID) : 0;
        if (self && !HAS_LOCAL_SENSOR && !own_sim) continue;
        if (!self && !n.initialized) continue;
        const uint16_t mv = self ? localBatteryMv : n.battery_mv;
        uint8_t st = lineStateOf(n, self);
        uint8_t fl = HAS_FLAG(n.flags, FLAG_LOW_BATTERY) ? MESH_LF_LOWBAT : 0;
        if (self) { if (own_sim) fl |= MESH_LF_SIM; if (hubSimTripped(NODE_ID)) st = MESH_LS_TRIPPED; }
        else if (HAS_FLAG(n.flags, FLAG_SIM)) fl |= MESH_LF_SIM;
        meshHubObserveNode(n.node_id, st, 0, fl, batteryMvToPercent(mv));
      }
    }
    if (meshPollReset()) networkResetFromChalet();
    sonarHubLoop();
    meshEbTick();   // v2: hub packet on the ESP-NOW backbone when the transport policy says so
    uint8_t cmd, dev, val;
    while (meshPollDevCmd(cmd, dev, val)) {   // from the chalet beacon: this hub, its holes, or its tip-ups
      if (cmd == MESH_CMD_SET_RELAY) {
        const bool on = val != 0;
        if (dev == NODE_ID) {
          settings.ebRelay = on; saveSettings(); meshSetEbRelay(on);
          Serial.printf("Backbone relay %s (set by the chalet)\n", on ? "ON" : "off");
        } else {
          devCmdQueue(dev, DEVCMD_RELAY, on ? 1 : 0);
        }
      } else if (cmd == MESH_CMD_SET_SIM) {
        hubSimApply(dev, val);
      } else if (cmd == MESH_CMD_SONAR_PARAM) {
        sonarKnobApply(dev, val);   // dev = knob index
      } else if (cmd == MESH_CMD_SET_BAIT) {
        baitApply(dev, val);        // own / test hole here, tip-up after its next message
      }
    }
  } else {
    MeshNodeUpdate u;
    uint8_t budget = 16;
    while (budget-- && meshPollNodeUpdate(u)) applyMeshNodeUpdate(u);
    if (millis() - lastSnap >= 1000) {
      lastSnap = millis();
      static MeshNodeSnapshot snap[48];
      const uint8_t k = meshNodeSnapshot(snap, 48);
      for (uint8_t i = 0; i < k; i++) {
        if (snap[i].node == NODE_ID || snap[i].node == 0) continue;
        // Reconcile: if an update was lost (queue full while loop() was busy), rebuild it from the
        // mesh's current view so a trip can never be missed.
        const int idx0 = findNodeIndexForMesh(snap[i].node);
        bool mismatch = (idx0 < 0);
        if (!mismatch) {
          const NodeState& n0 = network.nodes[idx0];
          const bool fish = (snap[i].state == MESH_LS_TRIPPED || snap[i].state == MESH_LS_RUNNING);
          mismatch = (fish != (bool)HAS_FLAG(n0.flags, FLAG_FISH_ON)) ||
                     ((snap[i].state == MESH_LS_OFFLINE) == n0.online) ||
                     ((snap[i].state == MESH_LS_FAULT) != (bool)HAS_FLAG(n0.flags, FLAG_SENSOR_ERROR)) ||
                     (((snap[i].flags & MESH_LF_SIM) != 0) != (bool)HAS_FLAG(n0.flags, FLAG_SIM));
        }
        if (mismatch) {
          MeshNodeUpdate fix = {snap[i].node, snap[i].owner, 0xFF, snap[i].state, snap[i].turns, snap[i].flags};
          applyMeshNodeUpdate(fix);
        }
        if (snap[i].state == MESH_LS_OFFLINE) continue;
        const int idx = findNodeIndexForMesh(snap[i].node);
        if (idx < 0) continue;
        NodeState& n = network.nodes[idx];
        n.last_seen = millis();      // keeps loopNodeTimeout quiet while the mesh reports the node
        n.online = true;
        if (snap[i].battery != 255) n.battery_mv = batteryPercentToMv(snap[i].battery);
      }
    }
  }

  uint8_t ch;   // v2: LoRa channel moved (chalet decision / hub followed): start there after a reboot
  if (meshPollChannelChanged(ch) && ch != settings.lastLoraCh) { settings.lastLoraCh = ch; saveSettings(); }

  bool s;
  if (meshPollSilence(s) && s != alertsSilenced) {
    alertsSilenced = s;
    if (s) { silenceTime = millis(); silenceExpireTime = millis() + SILENCE_AUTO_CLEAR_MS; buzzerStop(); alarmAck(); }
    else { silenceTime = 0; silenceExpireTime = 0; }
    if (!chalet) sendSilenceSyncEspNow();   // forward to this hub's tip-up nodes
    registerActivity();
  }

  if (millis() - lastCounters >= 1000) {
    lastCounters = millis();
    MeshCounters c;
    meshCounters(c);
    loraReady = c.radio_ok;
    loraRxCount = c.rx_ok;
    loraTxCount = c.tx;
    loraChecksumFails = c.rx_crc;     // hardware CRC failures
    if (c.last_rx_ms) lastLoRaRxTime = c.last_rx_ms;
  }
}

// OLED screen shown instead of the live status while the radio test mode is on.
void drawRadioTest() {
  scrDirty = true;
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tr);
  char line[32];
  snprintf(line, sizeof(line), "RADIO TEST: %s", meshTestModeName(meshTestMode()));
  display.drawStr(0, 7, line);
  display.drawHLine(0, 9, 128);
  if (currentRole == ROLE_GATEWAY_OFFSHORE) {
    MeshHubSummary h[5];
    const uint8_t n = meshHubSummaries(h, 5);
    if (n == 0) display.drawStr(0, 20, "No hub heard yet");
    for (uint8_t i = 0; i < n; i++) {
      snprintf(line, sizeof(line), "H%u%s %s %lu/%lu %ddB", h[i].id, h[i].via ? "*" : " ", meshModeName(h[i].mode),
               (unsigned long)h[i].rx, (unsigned long)h[i].sched, h[i].rssi);
      display.drawStr(0, 18 + i * 9, line);
    }
  } else {
    MeshHubView v;
    meshHubView(v);
    snprintf(line, sizeof(line), "%s frame %u", v.synced ? (v.from_echo ? "SYNC(echo)" : "SYNC") : "SEARCHING", v.frame);
    display.drawStr(0, 18, line);
    snprintf(line, sizeof(line), "Beacon %d dBm SNR %.1f", v.beacon_rssi, v.beacon_snr);
    display.drawStr(0, 27, line);
    snprintf(line, sizeof(line), "Lost %u/64  err %ld us", v.lost64, (long)v.sync_err_us);
    display.drawStr(0, 36, line);
    snprintf(line, sizeof(line), "Slot %s  %uB", v.own_slot_mode >= 0 ? meshModeName((uint8_t)v.own_slot_mode) : "none", v.allowance);
    display.drawStr(0, 45, line);
    snprintf(line, sizeof(line), "B%lu E%lu TX%lu", (unsigned long)v.beacons, (unsigned long)v.echoes, (unsigned long)v.tx);
    display.drawStr(0, 54, line);
  }
  display.sendBuffer();
}

// ---- web: /radio page and API (chalet; hubs in LR mode have no web server) ----
// meshRadioJson + what only main.cpp knows (relay requests, backbone receiver mode)
static String radioJsonFull() {
  String s = meshRadioJson();
  s.remove(s.length() - 1);   // closing brace
  s += ",\"eb_ready\":"; s += espNowReady ? "true" : "false";
  s += ",\"eb_chalet_lr\":"; s += settings.ebChaletLr ? "true" : "false";
  s += ",\"relay_req\":[";
  bool first = true;
  for (const auto& r : relayReqs) {
    if (r.dev == 0) continue;
    if (!first) s += ",";
    first = false;
    s += "{\"dev\":"; s += r.dev; s += ",\"on\":"; s += r.on ? "true" : "false"; s += "}";
  }
  s += "]}";
  return s;
}

void handleWebApiRadio() {
  server.send(200, "application/json", radioJsonFull());
}

void handleWebApiRadioPost() {
  StaticJsonDocument<320> doc;
  if (!server.hasArg("plain") || deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  if (currentRole != ROLE_GATEWAY_OFFSHORE) {
    server.send(400, "application/json", "{\"error\":\"radio settings are set on the chalet\"}");
    return;
  }
  if (doc.containsKey("test")) {
    const int m = doc["test"].as<int>();
    if (m >= 0 && m <= 4) { settings.radioTestMode = (uint8_t)m; meshSetTestMode((uint8_t)m); meshResetStats(); }
  }
  if (doc.containsKey("adaptive")) { settings.adaptiveRadio = doc["adaptive"].as<bool>(); meshSetAdaptive(settings.adaptiveRadio); }
  if (doc["resetStats"] | false) meshResetStats();
  // v2 network: transport, LoRa channel, relays, backbone receiver mode
  if (doc.containsKey("transport")) {
    const int t = doc["transport"].as<int>();
    if (t >= 0 && t <= 2) { settings.transportMode = (uint8_t)t; meshSetTransport((uint8_t)t); }
  }
  if (doc.containsKey("channel")) {   // "auto" or 1-8
    if (doc["channel"].is<const char*>()) { settings.loraChannel = MESH_CH_AUTO; meshSetLoraChannel(MESH_CH_AUTO); }
    else {
      const int c = doc["channel"].as<int>();
      if (c >= 1 && c <= 8) { settings.loraChannel = (uint8_t)(c - 1); meshSetLoraChannel((uint8_t)(c - 1)); }
    }
  }
  if (doc["rescan"] | false) meshRescanChannels();
  if (doc.containsKey("relay")) {
    const int dev = doc["relay"]["dev"] | 0;
    if (dev > 0 && dev < 255) relayRequest((uint8_t)dev, doc["relay"]["on"] | false);
  }
  saveSettings();
  server.send(200, "application/json", radioJsonFull());
}


// =============================================================================================
// v2 SONAR TEST MODE + FOCUS (see docs/SONAR_SIM.md)
//   Hub:    virtual sonar nodes (fake data through the real codec + TDMA transport), and the
//           sonar control broadcast to the tip-up nodes (sim switch + FOCUS node).
//   Chalet: /sonar page (hole grid, waterfall + flasher of the FOCUS hole) and its API.
// =============================================================================================
static icemesh::sonar::SonarSource* sonarVirt = nullptr;   // up to 4, allocated on first use (~15 KB each)

static void sendSonarCtrl(bool sim, uint8_t focus) {
  SonarCtrlMessage m;
  memset(&m, 0, sizeof(m));
  m.network_id = NETWORK_ID; m.sender_id = NODE_ID; m.msg_type = MSG_SONAR_CTRL;
  m.focus_node = focus; m.sim_on = sim ? 1 : 0;
  m.n_params = icemesh::sonar::P_COUNT;   // v2 (D42): the sonar knobs ride along
  memcpy(m.params, sonarPrm.v, icemesh::sonar::P_COUNT);
  esp_now_send(ESPNOW_BROADCAST, (uint8_t*)&m, sizeof(m));
}

// =============================================================================================
// v2 SIMULATION PER HOLE (hub side, docs/SONAR_SIM.md)
// The chalet sends CMD_SET_SIM (hole, value) in the beacon. The hub applies it to its own hole and to its
// virtual holes, and forwards it to a real tip-up (MSG_DEV_CMD DEVCMD_SIM) after that tip-up's next message.
// value: bit0 fake sonar, bit1 fake Hall trips, bits 2-7 trips per hour (0 = 6). Fake holes report LF_SIM.
// =============================================================================================
struct HoleSim { uint8_t node; uint8_t value; bool tripped; uint32_t until_ms, next_ms; };
static HoleSim hubSims[8];                   // own hole + virtual holes
static bool simAllSet = false;
static uint8_t simAllValue = 0;              // last "every hole" value (applies to holes that appear later)
static bool realSonarSim = false;            // a real tip-up of this pocket was asked for fake sonar

static HoleSim* hubSimSlot(uint8_t node, bool create) {
  for (auto& s : hubSims) if (s.node == node) return &s;
  if (!create) return nullptr;
  for (auto& s : hubSims) if (s.node == 0) { memset(&s, 0, sizeof(s)); s.node = node; s.value = simAllSet ? simAllValue : 0; return &s; }
  return nullptr;
}

static uint8_t hubHoleSim(uint8_t node) {
  const HoleSim* s = hubSimSlot(node, false);
  return s ? s->value : (simAllSet ? simAllValue : 0);
}

static bool isMyVirtualHole(uint8_t node) {
  for (uint8_t k = 0; k < 8; k++) if (meshSonarVirtualId(NODE_ID, k) == node) return true;
  return false;
}

static void hubSimApply(uint8_t target, uint8_t value) {
  if (target == 255) {
    simAllSet = true; simAllValue = value;
    for (auto& s : hubSims) if (s.node) { s.value = value; s.tripped = false; s.next_ms = 0; }
    for (int i = 1; i < network.node_count; i++) {
      const NodeState& n = network.nodes[i];
      if (n.initialized && n.node_id != 0 && n.node_id < 128) devCmdQueue(n.node_id, DEVCMD_SIM, value);
    }
    realSonarSim = (value & MESH_SIM_SONAR) != 0;
  } else if (target == NODE_ID || isMyVirtualHole(target)) {
    HoleSim* s = hubSimSlot(target, true);
    if (s) { s->value = value; s->tripped = false; s->next_ms = 0; }
  } else {
    devCmdQueue(target, DEVCMD_SIM, value);
    if (value & MESH_SIM_SONAR) realSonarSim = true;
  }
  Serial.printf("Simulation: hole %u -> sonar %s, Hall trips %s (%u/h)\n", target, (value & 1) ? "on" : "off",
                (value & 2) ? "on" : "off", (unsigned)((value >> 2) ? (value >> 2) : 6));
}

// Simulated Hall trip of the hub's own hole or a virtual hole: random start, flag up 20-90 s.
static bool hubSimTripped(uint8_t node) {
  const uint8_t v = hubHoleSim(node);
  HoleSim* s = hubSimSlot(node, (v & MESH_SIM_HALL) != 0);
  if (!(v & MESH_SIM_HALL) || s == nullptr) { if (s) s->tripped = false; return false; }
  const uint32_t now = millis();
  const uint32_t mean = 3600000UL / ((v >> 2) ? (v >> 2) : 6);
  if (s->tripped && static_cast<int32_t>(now - s->until_ms) >= 0) { s->tripped = false; s->next_ms = 0; }
  if (!s->tripped) {
    if (s->next_ms == 0) { s->next_ms = now + mean / 2 + esp_random() % (mean + 1); if (s->next_ms == 0) s->next_ms = 1; }
    if (static_cast<int32_t>(now - s->next_ms) >= 0) { s->tripped = true; s->until_ms = now + 20000UL + esp_random() % 70001UL; }
  }
  return s->tripped;
}

void sonarHubLoop() {
  static unsigned long lastTick = 0, lastCtrl = 0, lastObserve = 0, simOffSince = 0;
  static uint16_t prmGen = 0xFFFF;
  static bool wasSim = false;
  static uint8_t activeVirt = 0;
  static icemesh::sonar::SonarSource* ownSonar = nullptr;   // fake sonar of the hub's own hole
  if (prmGen != sonarPrmGen) {   // v2 (D42): knobs changed: the hub's own fake sonars use them too
    prmGen = sonarPrmGen;
    if (ownSonar) ownSonar->proc.prm = sonarPrm;
    if (sonarVirt) for (uint8_t k = 0; k < 4; k++) sonarVirt[k].proc.prm = sonarPrm;
  }
  static uint16_t bGen = 0xFFFF;
  if (bGen != baitGen) {         // v2 (D43): bait depth per hole
    bGen = baitGen;
    baitToSource(ownSonar, NODE_ID);
    if (sonarVirt) for (uint8_t k = 0; k < 4; k++) baitToSource(&sonarVirt[k], sonarVirt[k].node());
  }
  const bool global = meshSonarSim();                       // chalet: test holes (virtual) on
  const bool sim = global || realSonarSim;                  // tip-ups may run fake sonar
  const uint8_t focus = meshFocusNode();

  // own hole: fake sonar when asked (independent of the virtual holes)
  if (hubHoleSim(NODE_ID) & MESH_SIM_SONAR) {
    static unsigned long ownTick = 0;
    if (ownSonar == nullptr) { ownSonar = new icemesh::sonar::SonarSource(); ownSonar->proc.prm = sonarPrm; ownSonar->begin(NODE_ID, 4241UL + NODE_ID, (uint16_t)esp_random()); baitToSource(ownSonar, NODE_ID); ownTick = millis(); }
    if (millis() - ownTick > 1000UL) ownTick = millis() - 250UL;
    while (millis() - ownTick >= 250UL) {
      ownTick += 250UL;
      icemesh::sonar::Block out[2];
      const uint8_t n = ownSonar->tick(focus == NODE_ID, out, 2);
      for (uint8_t i = 0; i < n; i++) meshHubPushSonar(out[i].data, out[i].len);
    }
  }

  // control for the tip-up nodes: every second while on, at once after a node transmitted,
  // and for 10 s after the test mode is turned off (so awake nodes go back to deep sleep)
  static bool wasGlobal = false;
  if (global != wasGlobal) {   // test holes off: report them gone
    if (!global) {
      for (uint8_t k = 0; k < activeVirt; k++) meshHubObserveNode(meshSonarVirtualId(NODE_ID, k), MESH_LS_OFFLINE, 0, 0, 0);
      activeVirt = 0;
    }
    wasGlobal = global;
  }
  if (sim != wasSim) {
    if (!sim) simOffSince = millis();
    wasSim = sim;
  }
  const bool recentOff = !sim && simOffSince != 0 && millis() - simOffSince < 10000UL;
  const bool knobsFresh = sonarPrmChangedMs != 0 && millis() - sonarPrmChangedMs < 60000UL;   // new knobs: tell the tip-ups for 1 min
  if (espNowReady && (((sim || recentOff || knobsFresh) && millis() - lastCtrl >= 1000UL) ||
                      (sonarCtrlKick && millis() - lastCtrl >= 50UL))) {   // a tip-up just sent: it listens now
    sendSonarCtrl(sim, focus);
    lastCtrl = millis();
  }
  sonarCtrlKick = false;
  if (!global) return;

  // virtual holes (test holes generated by this hub): fake sonar by default, fake trips when asked
  const uint8_t want = settings.sonarVirtualNodes > 4 ? 4 : settings.sonarVirtualNodes;
  if (want > 0 && sonarVirt == nullptr) { sonarVirt = new icemesh::sonar::SonarSource[4]; for (uint8_t k = 0; k < 4; k++) sonarVirt[k].proc.prm = sonarPrm; }
  while (activeVirt > want) { activeVirt--; meshHubObserveNode(meshSonarVirtualId(NODE_ID, activeVirt), MESH_LS_OFFLINE, 0, 0, 0); }
  while (activeVirt < want) {
    const uint8_t id = meshSonarVirtualId(NODE_ID, activeVirt);
    sonarVirt[activeVirt].begin(id, (uint32_t)id * 7919UL + NODE_ID, (uint16_t)esp_random());
    baitToSource(&sonarVirt[activeVirt], id);
    HoleSim* hs = hubSimSlot(id, true);
    if (hs && !simAllSet) hs->value = MESH_SIM_SONAR;   // a test hole has fake sonar unless told otherwise
    meshHubObserveNode(id, MESH_LS_IDLE, 0, MESH_LF_SIM, 100);
    activeVirt++;
  }
  if (activeVirt == 0) return;
  if (millis() - lastObserve >= 1000UL) {
    lastObserve = millis();
    for (uint8_t k = 0; k < activeVirt; k++) {
      const uint8_t id = meshSonarVirtualId(NODE_ID, k);
      meshHubObserveNode(id, hubSimTripped(id) ? MESH_LS_TRIPPED : MESH_LS_IDLE, 0, MESH_LF_SIM, 100);
    }
  }
  if (millis() - lastTick > 1000UL) lastTick = millis() - 250UL;   // loop() was busy: don't burst
  uint8_t budget = 2;
  while (budget-- && millis() - lastTick >= 250UL) {               // 4 pings/s per virtual node
    lastTick += 250UL;
    for (uint8_t k = 0; k < activeVirt; k++) {
      icemesh::sonar::Block out[2];
      const uint32_t t0 = micros();
      if (!(hubHoleSim(sonarVirt[k].node()) & MESH_SIM_SONAR)) continue;
      const uint8_t n = sonarVirt[k].tick(focus == sonarVirt[k].node(), out, 2);
      const uint32_t dt = micros() - t0;
      if (dt > sonarTickUsMax) sonarTickUsMax = dt;
      sonarTickUsSum += dt; sonarTickCount++;
      for (uint8_t i = 0; i < n; i++) meshHubPushSonar(out[i].data, out[i].len);
    }
  }
}

void handleWebApiSonar() {
  server.send(200, "application/json", meshSonarListJson());
}

// v2 simulation per hole. GET: {"virtual":bool,"rate":n,"all":v|-1,"holes":[{"id","name","req","on"}]}
// POST: {"hole":id|255,"sonar":bool,"hall":bool} | {"all":bool} | {"virtual":bool} | {"rate":1-63}
static String simJson() {
  String s;
  s.reserve(160 + network.node_count * 48);
  s = "{\"virtual\":"; s += settings.sonarSim ? "true" : "false";
  s += ",\"rate\":"; s += simRate;
  s += ",\"all\":"; s += chSimAllSet ? String(chSimAllValue) : String(-1);
  s += ",\"demo_hubs\":"; s += meshDemoHubs();
  s += ",\"demo_holes\":"; s += meshDemoHoles();
  s += ",\"holes\":[";
  bool first = true;
  for (int i = 0; i < network.node_count; i++) {
    const NodeState& n = network.nodes[i];
    if (n.node_id == NODE_ID || n.node_id == 0 || !n.initialized) continue;
    if (!first) s += ",";
    first = false;
    s += "{\"id\":"; s += n.node_id;
    s += ",\"name\":\""; for (const char* c = n.name; *c; c++) if (*c != '"' && *c != '\\') s += *c; s += "\"";
    const uint8_t dv = meshDemoSim(n.node_id);   // demo hole: what it runs (256 = explicitly off, for the page)
    s += ",\"req\":"; s += dv != 0xFF ? String(dv ? dv : 256) : String(simRequested(n.node_id));
    s += ",\"on\":"; s += HAS_FLAG(n.flags, FLAG_SIM) ? "true" : "false";
    s += ",\"virtual\":"; s += n.node_id >= 128 ? "true" : "false";
    s += "}";
  }
  s += "]}";
  return s;
}

void handleWebApiSim() { server.send(200, "application/json", simJson()); }

void handleWebApiSimPost() {
  StaticJsonDocument<192> doc;
  if (!server.hasArg("plain") || deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}"); return; }
  if (currentRole != ROLE_GATEWAY_OFFSHORE) { server.send(400, "application/json", "{\"error\":\"simulation is set on the chalet\"}"); return; }
  if (doc.containsKey("rate")) { const int r = doc["rate"].as<int>(); if (r >= 1 && r <= 63) simRate = (uint8_t)r; }
  if (doc.containsKey("virtual")) { settings.sonarSim = doc["virtual"].as<bool>(); meshSetSonarSim(settings.sonarSim); saveSettings(); }
  if (doc.containsKey("all")) simAll(doc["all"].as<bool>());
  if (doc.containsKey("demo")) demoSet((uint8_t)(doc["demo"] | 0), (uint8_t)(doc["demo_holes"] | 3));
  if (doc.containsKey("hole")) {
    const int h = doc["hole"].as<int>();
    if (h > 0 && h <= 255) simRequest((uint8_t)h, doc["sonar"] | false, doc["hall"] | false);
  }
  server.send(200, "application/json", simJson());
}

// v2 (D42) GET: {"preset":0-3,"knobs":[{"key","name","lo","hi","def","unit","v"}]}
// POST: {"key":"snr","v":12} | {"preset":0-2} | {"defaults":true} | {"resend":true}
static String knobsJson() {
  using namespace icemesh::sonar;
  String s; s.reserve(200 + P_COUNT * 110);
  s = "{\"preset\":"; s += presetOf(sonarPrm); s += ",\"knobs\":[";
  for (uint8_t i = 0; i < P_COUNT; i++) {
    const ParamInfo& f = paramInfo(i);
    if (i) s += ",";
    s += "{\"key\":\""; s += f.key; s += "\",\"name\":\""; s += f.name; s += "\",\"lo\":"; s += f.lo;
    s += ",\"hi\":"; s += f.hi; s += ",\"def\":"; s += f.def; s += ",\"unit\":\""; s += f.unit; s += "\",\"v\":"; s += sonarPrm[i]; s += "}";
  }
  s += "]}";
  return s;
}

void handleWebApiKnobs() { server.send(200, "application/json", knobsJson()); }

void handleWebApiKnobsPost() {
  using namespace icemesh::sonar;
  StaticJsonDocument<192> doc;
  if (!server.hasArg("plain") || deserializeJson(doc, server.arg("plain"))) { server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}"); return; }
  if (currentRole != ROLE_GATEWAY_OFFSHORE) { server.send(400, "application/json", "{\"error\":\"the sonar knobs are set on the chalet\"}"); return; }
  if (doc.containsKey("key") && doc.containsKey("v")) {
    const char* k = doc["key"] | "";
    for (uint8_t i = 0; i < P_COUNT; i++) if (strcmp(k, paramInfo(i).key) == 0) sonarKnobSet(i, (uint8_t)constrain(doc["v"].as<int>(), 0, 255));
  }
  if (doc.containsKey("preset")) {
    Params q = sonarPrm; applyPreset(q, (uint8_t)(doc["preset"] | 1));
    for (uint8_t i = 0; i < P_COUNT; i++) if (q[i] != sonarPrm[i]) sonarKnobSet(i, q[i]);
  }
  if (doc["defaults"] | false) for (uint8_t i = 0; i < P_COUNT; i++) sonarKnobSet(i, paramInfo(i).def);
  if (doc["resend"] | false) sonarKnobsResend();
  server.send(200, "application/json", knobsJson());
}

void handleWebApiSonarPost() {
  StaticJsonDocument<128> doc;
  if (!server.hasArg("plain") || deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }
  if (currentRole != ROLE_GATEWAY_OFFSHORE) {
    server.send(400, "application/json", "{\"error\":\"sonar settings are set on the chalet\"}");
    return;
  }
  if (doc.containsKey("sim")) { settings.sonarSim = doc["sim"].as<bool>(); meshSetSonarSim(settings.sonarSim); saveSettings(); }
  if (doc.containsKey("focus")) {
    const int f = doc["focus"].as<int>();
    if (f >= 0 && f <= 255) meshSetFocusNode((uint8_t)f);
  }
  server.send(200, "application/json", meshSonarListJson());
}

void handleWebApiSonarPings() {
  const int node = server.arg("node").toInt();
  const uint32_t since = strtoul(server.arg("since").c_str(), nullptr, 10);
  const int mx = server.hasArg("max") ? server.arg("max").toInt() : 40;
  if (node <= 0 || node > 255) { server.send(400, "application/json", "{\"error\":\"node\"}"); return; }
  server.send(200, "application/json", meshSonarPingsJson((uint8_t)node, since, (uint8_t)(mx < 1 ? 1 : (mx > 64 ? 64 : mx))));
}

void handleWebApiSonarBg() {
  const int node = server.arg("node").toInt();
  if (node <= 0 || node > 255) { server.send(400, "application/json", "{\"error\":\"node\"}"); return; }
  server.send(200, "application/json", meshSonarBgJson((uint8_t)node));
}

