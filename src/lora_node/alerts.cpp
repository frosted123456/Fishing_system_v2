// FISH ON alarms: per-hole latch loops, alert history, silence (local + network), buzzer (split out of main.cpp, D49).
#include "chalet.h"

// v2 FISH ON alarm latch (D41 / D47): the rules are in lib/IceMesh/src/alarm_latch.h (host-tested);
// here only the loops over the holes. An offline hole keeps its alarm (tip-up dragged?).
bool nodeAlarm(const NodeState& n) { return n.alarm.active(HAS_FLAG(n.flags, FLAG_FISH_ON)); }

void alarmStart(NodeState* n) {
  n->alarm.start(millis());   // (recordAlert turns the sound back on)
  tripRecord(n->node_id);     // v2 (D43): keep the sonar of the minute before
}

void alarmAck() {
  for (int i = 0; i < network.node_count; i++) network.nodes[i].alarm.ack(HAS_FLAG(network.nodes[i].flags, FLAG_FISH_ON));
  updateAlertState();
}
// the line of a hole went back to normal: an alarm silenced during the trip ends here (D47)
void alarmLineReset(NodeState* n) { n->alarm.lineReset(); }

void loopAlarmHold() {
  static uint32_t last = 0;
  if (settings.alarmHoldMin == 0 || millis() - last < 1000) return;
  last = millis();
  bool changed = false;
  for (int i = 0; i < network.node_count; i++)
    changed |= network.nodes[i].alarm.hold(HAS_FLAG(network.nodes[i].flags, FLAG_FISH_ON), millis(), settings.alarmHoldMin * 60000UL);
  if (changed) updateAlertState();
}

void updateAlertState() {
  activeAlerts = false;
  for (int i = 0; i < network.node_count; i++) {
    const NodeState& n = network.nodes[i];
    if ((n.online && HAS_FLAG(n.flags, FLAG_FISH_ON)) || n.alarm.since_ms != 0) { activeAlerts = true; break; }
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

