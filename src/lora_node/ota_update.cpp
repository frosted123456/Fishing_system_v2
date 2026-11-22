// D148: update mode (see ota_update.h). Not compiled or run in the cloud: first flash over USB, then try it on the bench.
#include "chalet.h"
#include "ota_update.h"
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

#define OTA_STR2(x) #x
#define OTA_STR(x) OTA_STR2(x)
// The tag the updater looks for in an uploaded file: a file built for another board (another node ID or role) is refused.
static const char OTA_TAG[] __attribute__((used)) = "ICEFW1:R" OTA_STR(NODE_ROLE) ":N" OTA_STR(NODE_ID) ":";
const char* otaBoardTag() { return OTA_TAG; }

static const uint32_t OTA_MAGIC = 0x07A0FEEDUL;
RTC_NOINIT_ATTR static uint32_t g_otaAsk;          // survives a software restart, not a power cycle
static const uint32_t OTA_IDLE_MS = 600000UL;      // update mode ends by itself after 10 min without an upload
static const uint32_t OTA_CONFIRM_MS = 90000UL;    // a new firmware is kept once it has run this long
static const uint8_t OTA_MAX_TRIES = 3;            // boots a new firmware gets to reach that


bool otaSupported() { return esp_ota_get_next_update_partition(nullptr) != nullptr; }

void otaRequest() {
  Serial.println(F("OTA: restarting into the update mode"));
  Serial.flush();
  g_otaAsk = OTA_MAGIC;
  delay(100);
  ESP.restart();
}

// ---- new firmware on trial -------------------------------------------------------------------------------------------
static bool g_trial = false;
static void trialBoot() {
  Preferences p; if (!p.begin("ota", false)) return;
  const uint8_t pend = p.getUChar("pend", 0);
  if (!pend) { p.end(); return; }
  const uint8_t tries = static_cast<uint8_t>(p.getUChar("tries", 0) + 1);
  if (tries > OTA_MAX_TRIES) {
    p.putUChar("pend", 0); p.putUChar("tries", 0); p.putUChar("back", 1); p.end();
    const esp_partition_t* other = esp_ota_get_next_update_partition(nullptr);   // with two slots: the one this firmware replaced
    Serial.printf("OTA: the new firmware did not stay up %lu s in %u boots: back to the previous one (%s)\n",
                  static_cast<unsigned long>(OTA_CONFIRM_MS / 1000), static_cast<unsigned>(OTA_MAX_TRIES), other ? other->label : "none");
    Serial.flush();
    if (other && esp_ota_set_boot_partition(other) == ESP_OK) { delay(200); ESP.restart(); }
    return;   // nothing to go back to: keep running
  }
  p.putUChar("tries", tries); p.end();
  g_trial = true;
  Serial.printf("OTA: new firmware on trial, boot %u of %u (kept after %lu s of running)\n", static_cast<unsigned>(tries),
                static_cast<unsigned>(OTA_MAX_TRIES), static_cast<unsigned long>(OTA_CONFIRM_MS / 1000));
}
void otaLoop() {
  if (!g_trial || millis() < OTA_CONFIRM_MS) return;
  g_trial = false;
  Preferences p; if (p.begin("ota", false)) { p.putUChar("pend", 0); p.putUChar("tries", 0); p.end(); }
  esp_ota_mark_app_valid_cancel_rollback();   // only matters to a bootloader built with rollback (the core already confirms at start there); harmless otherwise
  Serial.println(F("OTA: new firmware confirmed"));
}

// ---- update mode -----------------------------------------------------------------------------------------------------
static WebServer* g_srv = nullptr;
static uint32_t g_lastAct = 0;
static size_t g_done = 0;
static bool g_tagSeen = false, g_force = false, g_failed = false, g_reboot = false;
static uint8_t g_tagPos = 0;
static char g_err[96] = "";

static void oled(const char* l1, const char* l2, const char* l3, const char* l4) {
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tr);
  display.drawStr(0, 10, "FIRMWARE UPDATE");
  display.drawHLine(0, 13, 128);
  if (l1) display.drawStr(0, 26, l1);
  if (l2) display.drawStr(0, 38, l2);
  if (l3) display.drawStr(0, 50, l3);
  if (l4) display.drawStr(0, 62, l4);
  display.sendBuffer();
}

static const char OTA_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Firmware update</title><style>
body{margin:0;background:#EEF3F4;color:#13262D;font:16px/1.4 "Arial Narrow",system-ui,sans-serif}
main{max-width:520px;margin:0 auto;padding:18px 14px}h1{font-size:1.7rem;margin:0 0 4px}
.card{background:#fff;border:1px solid #D2DEE2;border-radius:14px;padding:14px;margin:12px 0}
.muted{color:#55707A;font-size:.94rem}dl{display:grid;grid-template-columns:auto 1fr;gap:4px 14px;margin:0}dt{color:#55707A}dd{margin:0;font-weight:600}
input[type=file]{display:block;width:100%;margin:8px 0;font:inherit}
button{font:inherit;font-weight:600;min-height:48px;padding:10px 18px;border-radius:10px;border:0;background:#1C6C77;color:#fff;cursor:pointer}
button[disabled]{background:#D2DEE2;color:#55707A}button.plain{background:#fff;color:#13262D;border:1px solid #D2DEE2}
.bar{height:14px;border-radius:7px;background:#E1EAEC;overflow:hidden;margin:10px 0}.bar i{display:block;height:100%;width:0;background:#1C6C77}
#msg{font-weight:600;min-height:1.4em}.bad{color:#C2412D}.ok{color:#1C6C77}label.t{display:flex;gap:8px;align-items:center;margin:8px 0}
</style></head><body><main>
<h1>Firmware update</h1><p class="muted">This board is in update mode: nothing else runs until it restarts.</p>
<div class="card"><dl><dt>Board</dt><dd id="b">-</dd><dt>Running</dt><dd id="r">-</dd><dt>Spare slot</dt><dd id="s">-</dd></dl></div>
<div class="card"><p style="margin:0 0 4px"><b>Firmware file for this board</b></p>
<p class="muted" style="margin:0">Built on the PC with <code>tools/flash.py ... --ota-out</code>: a <code>.bin</code> named after this board.</p>
<input type="file" id="f" accept=".bin">
<label class="t muted"><input type="checkbox" id="force"> Send it even if the file is not marked for this board (it can change the board's ID and role)</label>
<button id="go" disabled>Send and restart</button> <button class="plain" id="leave">Leave update mode</button>
<div class="bar"><i id="p"></i></div><p id="msg"></p></div>
<p class="muted">The running firmware is kept in the other slot. If the new one does not stay up 90 s in three starts, the board goes back to it by itself.</p>
</main><script>
const $=i=>document.getElementById(i);let tag='';
fetch('/info').then(r=>r.json()).then(j=>{tag=j.tag;$('b').textContent=j.name+' (node '+j.node+', '+j.role+')';$('r').textContent='built '+j.build+', slot '+j.slot;$('s').textContent=j.next+', '+(j.room/1048576).toFixed(1)+' MB';});
$('f').onchange=()=>{const f=$('f').files[0];$('go').disabled=!f;$('msg').className='';$('msg').textContent=f?f.name+', '+(f.size/1048576).toFixed(2)+' MB':'';};
$('leave').onclick=()=>fetch('/leave',{method:'POST'}).then(()=>{$('msg').textContent='Restarting into normal running.';});
$('go').onclick=()=>{const f=$('f').files[0];if(!f)return;$('go').disabled=true;const x=new XMLHttpRequest();
 x.open('POST','/update?force='+($('force').checked?1:0)+'&size='+f.size);
 x.upload.onprogress=e=>{if(e.lengthComputable){$('p').style.width=(100*e.loaded/e.total)+'%';$('msg').textContent='Sending '+Math.round(100*e.loaded/e.total)+' %';}};
 x.onload=()=>{let j={};try{j=JSON.parse(x.responseText)}catch(e){}if(x.status==200&&j.ok){$('msg').className='ok';$('msg').textContent='Written ('+j.bytes+' bytes). The board restarts now; this hotspot disappears.';}
  else{$('msg').className='bad';$('msg').textContent='Refused: '+(j.error||('HTTP '+x.status));$('go').disabled=false;$('p').style.width='0';}};
 x.onerror=()=>{$('msg').className='bad';$('msg').textContent='Connection lost. Reload this page: if it still opens, nothing was changed.';$('go').disabled=false;};
 const d=new FormData();d.append('fw',f,f.name);x.send(d);};
</script></body></html>)HTML";

static void hInfo() {
  g_lastAct = millis();
  const esp_partition_t* run = esp_ota_get_running_partition(); const esp_partition_t* nx = esp_ota_get_next_update_partition(nullptr);
  char b[320];
  snprintf(b, sizeof(b), "{\"node\":%d,\"name\":\"%s\",\"role\":\"%s\",\"build\":\"%s %s\",\"slot\":\"%s\",\"next\":\"%s\",\"room\":%lu,\"tag\":\"%s\"}",
           NODE_ID, NODE_NAME, getRoleName(currentRole), __DATE__, __TIME__, run ? run->label : "?", nx ? nx->label : "none",
           static_cast<unsigned long>(nx ? nx->size : 0), OTA_TAG);
  g_srv->send(200, "application/json", b);
}
static void fail(const char* why) { if (!g_failed) { g_failed = true; strncpy(g_err, why, sizeof(g_err) - 1); g_err[sizeof(g_err) - 1] = 0; Update.abort(); Serial.printf("OTA: refused: %s\n", g_err); } }
// the board tag anywhere in the stream, across chunk borders (the tag has no repeated prefix: a plain restart is enough)
static void scanTag(const uint8_t* d, size_t n) {
  const size_t L = sizeof(OTA_TAG) - 1;
  for (size_t i = 0; i < n && !g_tagSeen; i++) {
    if (d[i] == static_cast<uint8_t>(OTA_TAG[g_tagPos])) { if (++g_tagPos == L) g_tagSeen = true; }
    else g_tagPos = d[i] == static_cast<uint8_t>(OTA_TAG[0]) ? 1 : 0;
  }
}
static void hUpload() {
  HTTPUpload& up = g_srv->upload();
  g_lastAct = millis();
  if (up.status == UPLOAD_FILE_START) {
    g_done = 0; g_tagSeen = false; g_tagPos = 0; g_failed = false; g_err[0] = 0; g_force = g_srv->arg("force") == "1";
    const esp_partition_t* nx = esp_ota_get_next_update_partition(nullptr);
    const size_t size = static_cast<size_t>(g_srv->arg("size").toInt());
    Serial.printf("OTA: receiving %s (%u bytes) into %s\n", up.filename.c_str(), static_cast<unsigned>(size), nx ? nx->label : "none");
    oled("Receiving...", up.filename.c_str(), nullptr, nullptr);
    if (!nx) fail("this board has a single firmware slot");
    else if (size && size > nx->size) fail("the file is larger than the firmware slot");
    else if (!Update.begin(size ? size : UPDATE_SIZE_UNKNOWN, U_FLASH)) fail("the spare slot could not be opened");
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_failed) return;
    if (g_done == 0 && up.currentSize > 0 && up.buf[0] != 0xE9) { fail("not an ESP32 firmware file (.bin from the build)"); return; }
    scanTag(up.buf, up.currentSize);
    if (Update.write(up.buf, up.currentSize) != up.currentSize) { fail("writing the flash failed"); return; }
    g_done += up.currentSize;
    static uint32_t last = 0; if (millis() - last > 500) { last = millis(); char l[24]; snprintf(l, sizeof(l), "%u kB", static_cast<unsigned>(g_done / 1024)); oled("Receiving...", l, nullptr, nullptr); }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_failed) return;
    if (!g_tagSeen && !g_force) { fail("this file is not built for this board (other node ID or role)"); return; }
    if (!Update.end(true)) { fail("the file did not check out (cut short or damaged)"); return; }
    Preferences p; if (p.begin("ota", false)) { p.putUChar("pend", 1); p.putUChar("tries", 0); p.putUChar("back", 0); p.end(); }
    Serial.printf("OTA: %u bytes written, tag %s: restarting on the new firmware\n", static_cast<unsigned>(g_done), g_tagSeen ? "matched" : "NOT matched (forced)");
    g_reboot = true;
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    fail("the upload was cut");
  }
}
static void hUpdateDone() {
  char b[160];
  if (g_reboot && !g_failed) { snprintf(b, sizeof(b), "{\"ok\":true,\"bytes\":%u}", static_cast<unsigned>(g_done)); g_srv->send(200, "application/json", b); }
  else { snprintf(b, sizeof(b), "{\"ok\":false,\"error\":\"%s\"}", g_err[0] ? g_err : "nothing received"); g_srv->send(400, "application/json", b); oled("Refused:", g_err, "Nothing was changed", nullptr); }
}

static void otaRun() {
  // the OLED (same start as setupDisplay())
  pinMode(VEXT_PIN, OUTPUT); digitalWrite(VEXT_PIN, LOW); delay(100);
#if OLED_HW_I2C
  display.setBusClock(OLED_I2C_HZ);
#endif
  display.begin();
  char ssid[40]; snprintf(ssid, sizeof(ssid), "%s-%s%u-UPDATE", NETWORK_NAME, currentRole == ROLE_GATEWAY_OFFSHORE ? "Chalet" : "Hub", static_cast<unsigned>(NODE_ID));
  Serial.println(F("════════ FIRMWARE UPDATE MODE ════════"));
  if (!otaSupported()) {
    Serial.println(F("OTA: this board has a single firmware slot (partition table): update over USB. Restarting."));
    oled("One firmware slot", "only: use USB.", "Restarting...", nullptr); delay(4000); ESP.restart();
  }
  WiFi.persistent(false); WiFi.mode(WIFI_AP);
  const bool ap = WiFi.softAP(ssid, WIFI_PASSWORD, 1, 0, 2);
  delay(200);
  Serial.printf("OTA: hotspot \"%s\" %s, password as the other hotspots, page http://%s/  (tag %s)\n", ssid, ap ? "on" : "FAILED", WiFi.softAPIP().toString().c_str(), OTA_TAG);
  static WebServer srv(80); g_srv = &srv;
  srv.on("/", HTTP_GET, []() { g_lastAct = millis(); g_srv->send_P(200, "text/html", OTA_PAGE); });
  srv.on("/info", HTTP_GET, hInfo);
  srv.on("/leave", HTTP_POST, []() { g_srv->send(200, "application/json", "{\"ok\":true}"); delay(300); ESP.restart(); });
  srv.on("/update", HTTP_POST, hUpdateDone, hUpload);
  srv.onNotFound([]() { g_srv->sendHeader("Location", "/"); g_srv->send(302, "text/plain", ""); });
  srv.begin();
  g_lastAct = millis();
  uint32_t lastDraw = 0;
  for (;;) {
    srv.handleClient();
    if (g_reboot && !g_failed) { oled("Written.", "Restarting on the", "new firmware...", nullptr); delay(1500); ESP.restart(); }
    if (Serial.available()) { const String s = Serial.readStringUntil('\n'); if (s.indexOf("EXIT") >= 0 || s.indexOf("REBOOT") >= 0) ESP.restart(); }
    const uint32_t idle = millis() - g_lastAct;
    if (idle > OTA_IDLE_MS) { Serial.println(F("OTA: 10 min without an upload: back to normal running")); oled("No upload:", "back to normal", nullptr, nullptr); delay(1500); ESP.restart(); }
    if (millis() - lastDraw > 1000 && g_done == 0 && !g_failed) {
      lastDraw = millis();
      char l3[28]; snprintf(l3, sizeof(l3), "ends in %lu min", static_cast<unsigned long>((OTA_IDLE_MS - idle) / 60000UL + 1));
      oled(ssid, "pass: " WIFI_PASSWORD, "http://192.168.4.1", l3);
    }
    delay(2);
  }
}

void otaBootCheck() {
  trialBoot();
  bool ask = g_otaAsk == OTA_MAGIC;
  g_otaAsk = 0;
#if BTN_PIN >= 0
  if (!ask) {   // the arming button held from power-on for 3 s
    pinMode(BTN_PIN, INPUT_PULLUP); delay(20);
    if (digitalRead(BTN_PIN) == LOW) {
      Serial.println(F("OTA: button down at start: keep it down 3 s for the update mode"));
      const uint32_t t0 = millis(); bool held = true;
      while (millis() - t0 < 3000) { if (digitalRead(BTN_PIN) != LOW) { held = false; break; } delay(20); }
      ask = held;
    }
  }
#endif
  if (ask) otaRun();
}
