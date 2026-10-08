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
#include "chalet.h"

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALS (declared in chalet.h)
// ═══════════════════════════════════════════════════════════════════════════
volatile bool sonarCtrlKick = false;
icemesh::sonar::Params sonarPrm;
TripRec tripRecs[8];
uint32_t sonarTickUsMax = 0, sonarTickUsSum = 0, sonarTickCount = 0;   // hub: CPU cost of one fake ping (scene + processing + codec)
uint32_t sonarPrmChangedMs = 0;   // hub: when the knobs last changed (sonar control repeats them for 1 min)
uint16_t sonarPrmGen = 0;         // bumps on every change: local fake sonars re-apply
uint8_t baitCm5[256];
int16_t holePosDm[256][2];
uint16_t baitGen = 0;
uint32_t buttonHeldMs = 0;            // > 0 while the button is held (display shows the hold bar)
uint32_t connectInfoUntil = 0;        // show the "how to connect" screen until then
uint32_t hubHotspotUntil = 0;         // hub hotspot auto-off time (0 = off)
uint8_t tripHead = 0, tripCount = 0;
#if OLED_HW_I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
#else
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, OLED_RST);
#endif
WebServer server(WEB_SERVER_PORT);
Preferences preferences;
TwoWire CardKBWire = TwoWire(1);  // Second I2C bus for CardKB
char storedSsid[33] = "";
char storedPassword[65] = "";
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
  .nearBaitBeep = false,
  .beamDeg = 20
};
uint8_t pendingConfigSeq = 0;
uint8_t pendingConfigTarget = 0;
unsigned long pendingConfigTime = 0;
bool pendingConfigWaiting = false;
const unsigned long CONFIG_ACK_TIMEOUT_MS = 15000;  // 15 seconds (allows for hops)
ConfigDedupEntry configDedup[CONFIG_DEDUP_SIZE];
uint8_t configDedupIdx = 0;
AlertRecord alertHistory[ALERT_HISTORY_SIZE];
uint8_t alertHistoryIdx = 0;
uint8_t alertHistoryCount = 0;
MenuScreen currentScreen = SCREEN_LIVE_STATUS;
MenuScreen previousScreen = SCREEN_LIVE_STATUS;
uint8_t menuSelection = 0;
uint8_t menuScrollOffset = 0;
uint8_t selectedNodeIdx = 0;
uint8_t editingSettingIdx = 0;
int32_t editingValue = 0;
uint8_t liveStatusPage = 0;  // Current page on live status screen
bool alertsSilenced = false;
uint32_t silenceTime = 0;
uint32_t silenceExpireTime = 0;  // When silence auto-expires
bool cardKbAvailable = false;
char inputBuffer[65] = "";
uint8_t inputPos = 0;
uint8_t inputField = 0;  // 0=SSID, 1=Password
char inputSsid[33] = "";
char inputPassword[65] = "";
unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 300;
bool displaySleeping = false;
unsigned long lastActivityTime = 0;
const unsigned long DISPLAY_SLEEP_MS = 5UL * 60UL * 1000UL;  // 5 minutes
NodeRole currentRole = NODE_ROLE;
NetworkState network;
portMUX_TYPE networkMux = portMUX_INITIALIZER_UNLOCKED;
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
volatile bool loraInterrupt = false;     // DIO1 interrupt occurred (RX case)
volatile bool loraTxComplete = false;    // C1 FIX: Separate flag for TX complete
volatile RadioState radioState = RADIO_STATE_RX;
uint8_t loraBuffer[128];  // Sufficient for 107-byte optimized message
uint32_t loraRxSavedFromLoss = 0;
bool activeAlerts = false;
unsigned long lastBuzzerTime = 0;
uint8_t buzzerState = 0;
uint16_t txSequence = 0;  // H1 FIX: Use uint16_t to prevent faster-than-expected wraparound
uint32_t loraRxCount = 0;           // Packets received successfully
uint32_t loraTxCount = 0;           // Packets transmitted successfully
uint32_t loraRxErrors = 0;          // Receive errors
uint32_t loraTxErrors = 0;          // Transmit errors
uint32_t loraChecksumFails = 0;     // Checksum verification failures
uint32_t loraWatchdogResets = 0;    // Watchdog-triggered radio reinitializations
uint32_t loraStartRxRetries = 0;    // startReceive retry count
unsigned long lastLoRaRxTime = 0;   // When last valid packet was received

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
  posLoad();
  
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

bool ebSend(const uint8_t* frame, size_t len) {
  return espNowReady && esp_now_send(ESPNOW_BROADCAST, frame, len) == ESP_OK;
}

// Hub: commands for tip-up nodes (relay on/off). A sleeping node only listens right after it
// transmits, so the command goes out after each of its next 3 messages (it is idempotent).
struct PendingDevCmd { uint8_t node, cmd, value, sends; uint32_t since; };
static PendingDevCmd devCmds[8];

void devCmdQueue(uint8_t node, uint8_t cmd, uint8_t value) {
  PendingDevCmd* slot = nullptr;
  for (auto& d : devCmds) if (d.node == node && d.cmd == cmd) slot = &d;
  for (auto& d : devCmds) if (slot == nullptr && d.node == 0) slot = &d;
  if (slot == nullptr) slot = &devCmds[0];
  slot->node = node; slot->cmd = cmd; slot->value = value; slot->sends = 0; slot->since = millis();
}

void devCmdOnNodeHeard(uint8_t node) {
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

RelayReq relayReqs[16];   // (chalet.h)

void relayRequest(uint8_t dev, bool on) {
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
bool chSimAllSet = false;
uint8_t chSimAllValue = 0;
uint8_t simRate = 6;                  // fake trips per hour (Hall simulation)

uint8_t simRequested(uint8_t node) {
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

uint8_t hubHoleSim(uint8_t node) {
  const HoleSim* s = hubSimSlot(node, false);
  return s ? s->value : (simAllSet ? simAllValue : 0);
}

static bool isMyVirtualHole(uint8_t node) {
  for (uint8_t k = 0; k < 8; k++) if (meshSonarVirtualId(NODE_ID, k) == node) return true;
  return false;
}

void hubSimApply(uint8_t target, uint8_t value) {
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
bool hubSimTripped(uint8_t node) {
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



