// Chalet / hub web server: routes, JSON API, remote config (split out of main.cpp, D49).
#include "chalet.h"

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
