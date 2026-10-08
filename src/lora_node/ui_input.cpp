// OLED input: CardKB keys, menu, Settings pages, text entry, navigation (split out of main.cpp, D49). Shared declarations: chalet.h
#include "chalet.h"

// ═══════════════════════════════════════════════════════════════════════════
// CARDKB KEYBOARD & MENU INPUT
// ═══════════════════════════════════════════════════════════════════════════

// CardKB key codes

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
    h.since_s = h.state == SH_FISH ? (now - (n.alarm.since_ms ? n.alarm.since_ms : (n.fish_on_time ? n.fish_on_time : now))) / 1000 : 0;
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
void optBuild(ScrOptions* o) {
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

void scrStep(int dir) {
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

void menuBuild(ScrOptions* o) {
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
void scrAction(bool kb) {
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

void scrDraw() {
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
void apName(char* out, size_t n) {
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
void drawConnectInfo() {
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

