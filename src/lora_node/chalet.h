// Shared declarations of the chalet / hub firmware (src/lora_node): types, globals (defined in main.cpp)
// and the functions the other .cpp files of this firmware call. Split out of main.cpp (D49).
#pragma once

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


// ---- CardKB key codes, menu sizes ----
#define KEY_UP      0xB5
#define KEY_DOWN    0xB6
#define KEY_LEFT    0xB4
#define KEY_RIGHT   0xB7
#define KEY_ENTER   0x0D
#define KEY_ESC     0x1B
#define KEY_TAB     0x09
#define KEY_BACKSP  0x08
#define KEY_SPACE   0x20
#define MAIN_MENU_COUNT 7
#define SETTINGS_MENU_COUNT 8

// ---- types ----
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
#else
#endif
// LoRa radio (SX1262) is owned by mesh_radio.cpp (TDMA radio task)

// WiFi credentials (loaded from NVS or config.h defaults)

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
  uint8_t beamDeg;            // v2 (D52): transducer beam angle, degrees (full cone). Display: cone width per depth. est. 20
};


// Remote config state

// Config message deduplication (prevents relay loops)
#define CONFIG_DEDUP_SIZE 8
struct ConfigDedupEntry {
  uint8_t origin_id;
  uint8_t config_seq;
  uint32_t received_at;
};

// Alert history
#define ALERT_HISTORY_SIZE 10
struct AlertRecord {
  uint8_t nodeId;
  uint32_t timestamp;     // millis() when occurred
  bool active;            // Still ongoing?
};

// Menu state

// Silence state

// CardKB state

// Button silencing (for devices without CardKB)

// OLED sleep mode

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════════════


// M1 FIX: Mutex for network.nodes[] access (ESP-NOW callback vs main loop)
// Using minimal critical sections to avoid stack overflow issues

// Note: Thread safety between ESP-NOW callback and main loop was attempted with
// portMUX spinlock, but caused stack overflow on IDLE task. ESP-NOW callbacks
// are serialized by WiFi driver and race conditions are unlikely in practice.
// M1 FIX: Re-enabled with minimal critical sections around state mutations only.




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


// Legacy name for compatibility with existing code
#define loraReceived loraInterrupt

// BUG FIX #17b: LoRaAggregateMessage is now 107 bytes (6 header + 10*10 nodes + 1 checksum)
// after optimizing NodeStatusCompact from 14 to 10 bytes per node

// Diagnostic counter: Packets saved from loss by processPendingLoRaRx()



// ═══════════════════════════════════════════════════════════════════════════
// LORA DIAGNOSTIC COUNTERS & WATCHDOG (IMPROVEMENT 1, BUG FIX #1, #2)
// ═══════════════════════════════════════════════════════════════════════════



// v2: relay dedup caches removed — duplicates are handled by the TDMA mesh (lib/IceMesh)


// ---- globals (defined in main.cpp) ----
#if OLED_HW_I2C
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C display;
#else
extern U8G2_SSD1306_128X64_NONAME_F_SW_I2C display;
#endif
extern WebServer server;
extern Preferences preferences;
extern TwoWire CardKBWire;
extern char storedSsid[33];
extern char storedPassword[65];
extern DeviceSettings settings;
extern uint8_t pendingConfigSeq;
extern uint8_t pendingConfigTarget;
extern unsigned long pendingConfigTime;
extern bool pendingConfigWaiting;
extern const unsigned long CONFIG_ACK_TIMEOUT_MS;
extern ConfigDedupEntry configDedup[CONFIG_DEDUP_SIZE];
extern uint8_t configDedupIdx;
extern AlertRecord alertHistory[ALERT_HISTORY_SIZE];
extern uint8_t alertHistoryIdx;
extern uint8_t alertHistoryCount;
extern MenuScreen currentScreen;
extern MenuScreen previousScreen;
extern uint8_t menuSelection;
extern uint8_t menuScrollOffset;
extern uint8_t selectedNodeIdx;
extern uint8_t editingSettingIdx;
extern int32_t editingValue;
extern uint8_t liveStatusPage;
extern bool alertsSilenced;
extern uint32_t silenceTime;
extern uint32_t silenceExpireTime;
extern bool cardKbAvailable;
extern char inputBuffer[65];
extern uint8_t inputPos;
extern uint8_t inputField;
extern char inputSsid[33];
extern char inputPassword[65];
extern unsigned long lastButtonPress;
extern const unsigned long BUTTON_DEBOUNCE_MS;
extern bool displaySleeping;
extern unsigned long lastActivityTime;
extern const unsigned long DISPLAY_SLEEP_MS;
extern NodeRole currentRole;
extern NetworkState network;
extern portMUX_TYPE networkMux;
extern bool localReedState;
extern bool localFishOn;
extern uint16_t localBatteryMv;
extern uint16_t localSequence;
extern uint32_t localUptimeSec;
extern bool espNowReady;
extern bool loraReady;
extern bool wifiApActive;
extern bool wifiStaConnected;
extern String staIpAddress;
extern String apIpAddress;
extern unsigned long lastLoRaTx;
extern unsigned long lastDisplayUpdate;
extern unsigned long lastReedCheck;
extern unsigned long lastHeartbeatTime;
extern volatile bool loraInterrupt;
extern volatile bool loraTxComplete;
extern volatile RadioState radioState;
extern uint8_t loraBuffer[128];
extern uint32_t loraRxSavedFromLoss;
extern bool activeAlerts;
extern unsigned long lastBuzzerTime;
extern uint8_t buzzerState;
extern uint16_t txSequence;
extern uint32_t loraRxCount;
extern uint32_t loraTxCount;
extern uint32_t loraRxErrors;
extern uint32_t loraTxErrors;
extern uint32_t loraChecksumFails;
extern uint32_t loraWatchdogResets;
extern uint32_t loraStartRxRetries;
extern unsigned long lastLoRaRxTime;

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
void scrDraw();
void apName(char* out, size_t n);
void drawRadioTest();
void scrStep(int dir);
void scrAction(bool kb = false);
void loopOptions();
void optBuild(ScrOptions* o);
void menuBuild(ScrOptions* o);
void loopWebServer();
void loopBuzzer();
void loopNodeTimeout();
void checkWiFiStatus();
void verifyWiFiChannel();          // BUG FIX #6: WiFi channel verification

void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len);
void loopEspNowRx();
extern volatile bool sonarCtrlKick;
extern uint32_t sonarTickUsMax, sonarTickUsSum, sonarTickCount;
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
bool ebSend(const uint8_t* frame, size_t len);
void devCmdOnNodeHeard(uint8_t node);
void devCmdQueue(uint8_t node, uint8_t cmd, uint8_t value);
void relayRequest(uint8_t dev, bool on);
void simRequest(uint8_t node, bool sonar, bool hall);
void simAll(bool on);
void demoSet(uint8_t hubs, uint8_t holes);
bool simAnyOn();
inline uint8_t simValueOf(bool sonar, bool hall, uint8_t tph) {
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
const uint8_t DEPTH_FT[8] = {5, 10, 15, 20, 25, 30, 40, 60};
const uint8_t DEPTH_M[8] = {2, 3, 4, 5, 6, 8, 10, 12};
uint16_t depthStepCm(uint8_t idx);
void depthStepText(uint8_t idx, char* out, size_t n);
// v2 (D42) sonar knobs: kept in NVS ("sonar"), chalet sends them in the beacon, hubs pass them to tip-ups
extern icemesh::sonar::Params sonarPrm;
extern uint32_t sonarPrmChangedMs;
extern uint16_t sonarPrmGen;
void sonarKnobsLoad();
void sonarKnobApply(uint8_t id, uint8_t v);   // set + save + apply here (hub: from the beacon)
void sonarKnobSet(uint8_t id, uint8_t v);     // chalet: set + send to every sonar hole
void sonarKnobsResend();                      // chalet: send the whole set again
void sonarKnobsNewHubs();
// v2 (D43) bait depth per hole, 5 cm steps (0 = not set: the processing keeps its default)
extern uint8_t baitCm5[256];
extern uint16_t baitGen;
// v2 (D51) hole positions on the pocket map (chalet, NVS "pos"): metres from the chalet, x east, y north,
// in decimetres; POS_UNSET = not placed yet
static const int16_t POS_UNSET = 0x7FFF;
extern int16_t holePosDm[256][2];
void posLoad();
void posSet(uint8_t hole, int16_t x_dm, int16_t y_dm);   // POS_UNSET, POS_UNSET = remove from the map
void handleWebApiNodePos();
void baitLoad();
void baitToSource(icemesh::sonar::SonarSource* src, uint8_t id);   // hub: this hole's bait depth into its fake sonar
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
uint8_t hubHoleSim(uint8_t node);
bool hubSimTripped(uint8_t node);
void hubSimApply(uint8_t target, uint8_t value);
void handleWebWifiConfig();
void handleWebApiSettingsGet();
void handleWebApiSettingsPost();

// WiFi credential management
void loadWifiCredentials();
void saveWifiCredentials(const char* ssid, const char* password);
void checkSerialWifiConfig();
void wifiRetryNow();
void wifiJoin(const char* ssid, const char* pass);
void wifiForget();
void wifiStateText(char* out, size_t n);
extern bool wifiScanBusy;
void perfPrint();

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
extern uint32_t buttonHeldMs;
extern uint32_t connectInfoUntil;
extern uint32_t hubHotspotUntil;
void drawConnectInfo();
// Chalet: relay settings requested from the web page (hubs report theirs; tip-ups do not)
struct RelayReq { uint8_t dev; bool on; };
extern RelayReq relayReqs[16];
// Chalet: simulation requests (fake sonar / fake trips per hole)
extern bool chSimAllSet; extern uint8_t chSimAllValue, simRate;
uint8_t simRequested(uint8_t node);
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
extern TripRec tripRecs[8];
extern uint8_t tripHead, tripCount;
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

