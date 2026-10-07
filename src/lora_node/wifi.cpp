// Chalet / hub Wi-Fi: hotspot, cabin network, credentials, retry (split out of main.cpp, D49). Shared declarations: chalet.h
#include "chalet.h"

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
bool wifiScanBusy = false;

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

