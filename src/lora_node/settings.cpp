// Settings and hole names in NVS (split out of main.cpp, D49). Shared declarations: chalet.h
#include "chalet.h"

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

// v2 (D51) positions on the pocket map
void posLoad() {
  for (int i = 0; i < 256; i++) holePosDm[i][0] = holePosDm[i][1] = POS_UNSET;
  Preferences p; p.begin("pos", true);
  p.getBytes("p", holePosDm, sizeof(holePosDm));
  p.end();
}
void posSet(uint8_t hole, int16_t x_dm, int16_t y_dm) {
  if (hole == 0 || hole == 255) return;
  if (holePosDm[hole][0] == x_dm && holePosDm[hole][1] == y_dm) return;
  holePosDm[hole][0] = x_dm; holePosDm[hole][1] = y_dm;
  Preferences p; p.begin("pos", false); p.putBytes("p", holePosDm, sizeof(holePosDm)); p.end();
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

void baitToSource(icemesh::sonar::SonarSource* src, uint8_t id) {
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

