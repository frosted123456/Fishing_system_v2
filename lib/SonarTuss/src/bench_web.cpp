// Phone test page for the sonar bench builds (D56, D87). See bench_web.h.
#include "bench_web.h"
#if defined(SONAR_BENCH) && SONAR_BENCH
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

namespace bench {

static WebServer* g_srv = nullptr;   // own server (begin) or the owner's (attach)
static bool g_own = false;
static CommandFn g_cmd = nullptr;
static bool g_on = false;

// recording ring: whole lines with a sequence number, oldest dropped past RING_BYTES / RING_LINES
#ifndef BENCH_RING_BYTES
#define BENCH_RING_BYTES (32 * 1024)   // WROOM tip-up (no mesh server, heap to spare); the hub builds set 12 kB (D88)
#endif
// D92: a static byte ring, no heap at all (the deque of Strings made two copies of every 5 kB RAW line and its node
// allocation aborted the hub on OOM during a sweep). Records: [seq 4 B][len 2 B][text], oldest overwritten; a record
// never straddles the end (a record that does not fit at the end starts at 0 and the tail is left unused).
static const size_t RING_BYTES = BENCH_RING_BYTES;
static uint8_t g_ringBuf[RING_BYTES];
static size_t g_head = 0, g_tail = 0, g_wrapAt = RING_BYTES;   // tail = oldest record, head = write position, wrapAt = end of valid data before the wrap
static uint32_t g_ringCount = 0;
static bool g_wrapped = false;                                  // head is below tail: valid data is tail..wrapAt then 0..head
static const size_t REC_HDR = 6;
static inline uint32_t rd32(size_t at) { return (uint32_t)g_ringBuf[at] | ((uint32_t)g_ringBuf[at + 1] << 8) | ((uint32_t)g_ringBuf[at + 2] << 16) | ((uint32_t)g_ringBuf[at + 3] << 24); }
static inline uint16_t rd16(size_t at) { return (uint16_t)(g_ringBuf[at] | (g_ringBuf[at + 1] << 8)); }
static void ringPopOldest() {
  if (!g_ringCount) return;
  g_tail += REC_HDR + rd16(g_tail + 4);
  if (--g_ringCount == 0) { g_head = g_tail = 0; g_wrapAt = RING_BYTES; g_wrapped = false; }
  else if (g_wrapped && g_tail >= g_wrapAt) { g_tail = 0; g_wrapAt = RING_BYTES; g_wrapped = false; }
}
// the record at `pos` (a record start, first = g_tail), then pos moves to the next one; the caller walks exactly
// g_ringCount records (a position test cannot tell a full wrapped ring's head from its tail)
static void ringRead(size_t& pos, uint32_t& seq, const char*& text, uint16_t& len) {
  if (g_wrapped && pos >= g_wrapAt) pos = 0;
  seq = rd32(pos); len = rd16(pos + 4); text = (const char*)g_ringBuf + pos + REC_HDR;
  pos += REC_HDR + len;
}
static uint32_t g_seq = 0;

static String g_status = "{}";

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Sonar bench</title>
<style>
:root{--bg:#fff;--fg:#111;--mut:#666;--card:#f2f2f2;--acc:#0a63c9;--bad:#c62828;--ok:#2e7d32}
@media(prefers-color-scheme:dark){:root{--bg:#111;--fg:#eee;--mut:#999;--card:#1e1e1e;--acc:#5aa0ff;--bad:#ff6b6b;--ok:#6ccf72}}
body{font:15px system-ui,sans-serif;margin:0;padding:12px;background:var(--bg);color:var(--fg)}
h1{font-size:18px;margin:0 0 8px}.card{background:var(--card);border-radius:10px;padding:10px;margin:0 0 10px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}button{font:inherit;padding:10px 14px;border-radius:8px;border:0;
background:var(--acc);color:#fff}button.off{background:var(--bad)}input{font:inherit;padding:8px;border-radius:6px;border:1px solid var(--mut);
background:var(--bg);color:var(--fg);min-width:0}.g{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:4px 12px}
.k{color:var(--mut);font-size:13px}.v{font-weight:600}canvas{width:100%;height:180px;display:block}
table{width:100%;border-collapse:collapse}td{padding:3px 2px;font-size:14px}td input{width:64px}
#msg{white-space:pre-wrap;font-size:13px;color:var(--mut);overflow-wrap:anywhere}.bad{color:var(--bad)}.ok{color:var(--ok)}
</style></head><body>
<h1>Sonar bench <span id="node" class="k"></span></h1>
<div class="card"><div class="g" id="vals"></div></div>
<div class="card"><canvas id="cv" width="600" height="180"></canvas><div class="k">Last ping, 200 kHz: depth left to right (m), echo strength up. Red line = bottom.</div></div>
<div class="card">
 <div class="row"><button id="rec">Start recording</button><button id="dl">Download</button><button id="raw">RAW now</button><button id="sw" title="automated preset run: gain, cycles, frequency, threshold, averaging, drive (recording must be on)">Sweep</button></div>
 <p id="rinfo" class="k">Not recording.</p>
 <div class="row"><input id="note" placeholder="Note: ruler depth, target, temp..." style="flex:1"><button id="nb">Note</button></div>
 <div class="row" style="margin-top:8px"><span class="k">Record every</span><input id="every" type="number" min="1" max="20" value="1" style="width:56px"><span class="k">ping(s), RAW every</span><input id="rawe" type="number" min="0" max="600" value="30" style="width:64px"><span class="k">s</span><button id="ap">Apply</button></div>
</div>
<div class="card"><b>Knobs</b> <span class="k">(this tip-up only, not saved)</span><table id="kn"></table></div>
<div class="card"><div id="msg"></div></div>
<script>
const $=id=>document.getElementById(id);let lines=[],last=0,lost=0,recOn=false,busy=false,st={};
function msg(t){$('msg').textContent=t}
async function cmd(c){try{const r=await fetch('cmd?c='+encodeURIComponent(c));const t=await r.text();msg(t);return t}catch(e){msg('No answer from the tip-up: '+e)}}
function stamp(){const d=new Date(),p=n=>String(n).padStart(2,'0');return d.getFullYear()+'-'+p(d.getMonth()+1)+'-'+p(d.getDate())+'T'+p(d.getHours())+':'+p(d.getMinutes())+':'+p(d.getSeconds())}
function draw(){const c=$('cv'),x=c.getContext('2d'),w=c.width,h=c.height,hex=st.codes||'';x.clearRect(0,0,w,h);
 const n=hex.length/2;if(!n)return;const fg=getComputedStyle(document.body).color;x.strokeStyle=fg;x.lineWidth=1;x.beginPath();
 for(let i=0;i<n;i++){const v=parseInt(hex.substr(2*i,2),16),X=i*w/n,Y=h-v*h/255;i?x.lineTo(X,Y):x.moveTo(X,Y)}x.stroke();
 const m=n*st.bin_m;x.fillStyle=fg;x.font='11px system-ui';for(let d=0;d<=m;d+=Math.max(0.5,Math.ceil(m/8))){x.fillText(d+'m',d/m*w+2,11)}
 if(st.bottom>0){x.strokeStyle='#e53935';const X=st.bottom/m*w;x.beginPath();x.moveTo(X,0);x.lineTo(X,h);x.stroke()}}
function show(){const L=st.line||'';const f={};L.replace(/(\w+)=([^\s|]+)/g,(a,k,v)=>{f[k]=v});
 const items=[['Bottom',f.bottom],['Bottom SNR',f.bot_snr],['Noise floor',f.noise],['Ring-down',f.ring],['ADC rate',(f.adc||'').replace('(want','')],['Time zero',f.t0],['Raw max',f.raw_max],['Ping time',f.ping]];
 const tg=(L.split('|')[2]||'').trim();
 $('vals').innerHTML=items.map(([k,v])=>'<div><div class="k">'+k+'</div><div class="v">'+(v||'-')+'</div></div>').join('')+'<div style="grid-column:1/-1"><div class="k">Targets</div><div class="v">'+(tg||'none')+'</div></div>'+
 ((/CLIPPED|FAULT|FALLBACK|ALARM/.test(L))?'<div style="grid-column:1/-1" class="bad">'+(L.match(/\((CLIPPED|FALLBACK|RING ALARM)\)|BURST FAULT\([^)]*\)/g)||[]).join(' ')+'</div>':'');
 $('node').textContent=st.node?('tip-up '+st.node+' · battery '+st.vbat+' mV · free RAM '+st.heap):'';draw()}
async function poll(){try{const r=await fetch('s');st=await r.json();show();
 if(st.rec!==recOn&&!busy){recOn=st.rec;ui()}}catch(e){$('vals').innerHTML='<div class="bad">No connection to the tip-up</div>'}}
async function pull(){if(!recOn||busy)return;busy=true;try{const r=await fetch('log?since='+last);const t=await r.text();
 for(const ln of t.split('\n')){if(!ln)continue;if(ln.startsWith('#first ')){const f=+ln.slice(7);if(last&&f>last+1)lost+=f-last-1;continue}
 const i=ln.indexOf('\t');if(i<0)continue;const s=+ln.slice(0,i);if(s<=last)continue;last=s;lines.push(ln.slice(i+1))}}catch(e){}busy=false;ui()}
function ui(){$('rec').textContent=recOn?'Stop recording':'Start recording';$('rec').className=recOn?'off':'';
 const kb=Math.round(lines.reduce((a,l)=>a+l.length+1,0)/1024);
 $('rinfo').innerHTML=(recOn?'<span class="ok">Recording</span> · ':'')+lines.length+' lines, '+kb+' kB kept on this phone'+(lost?' · <span class="bad">'+lost+' lines lost (keep this page open, screen on)</span>':'')}
$('rec').onclick=async()=>{busy=true;if(!recOn){if(lines.length&&!confirm('Start a new recording? The current one ('+lines.length+' lines) is cleared. Download it first if you need it.')){busy=false;return}
 lines=[];lost=0;last=+(await (await fetch('s')).json()).seq||0;await cmd('REC WEB ON '+stamp());recOn=true}else{await cmd('REC WEB OFF');recOn=false;await pull()}busy=false;ui()};
$('dl').onclick=()=>{if(!lines.length){msg('Nothing recorded yet.');return}const b=new Blob([lines.join('\n')+'\n'],{type:'text/plain'});
 const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download='sonar_'+(st.node||'')+'_'+stamp().replace(/[:T]/g,'-')+'.txt';document.body.appendChild(a);a.click();a.remove()};
$('raw').onclick=()=>cmd('RAW');
$('sw').onclick=async()=>{if(!recOn){msg('Start recording first, then Sweep.');return}if(confirm('Run the preset sweep? ~15 steps, about 3 minutes. Keep the transducer still.'))await cmd('SWEEP')};
$('nb').onclick=async()=>{const t=$('note').value.trim();if(!t)return;await cmd('NOTE ['+stamp()+'] '+t);$('note').value=''};
$('ap').onclick=async()=>{await cmd('RECEVERY '+$('every').value);await cmd('RAWEVERY '+$('rawe').value)};
async function knobs(){try{const k=await (await fetch('k')).json();$('kn').innerHTML=k.map(o=>'<tr><td>'+o.label+'<div class="k">'+o.key+' · '+o.min+'-'+o.max+' '+o.unit+'</div></td><td><input id="k_'+o.key+'" type="number" value="'+o.val+'"></td><td><button data-k="'+o.key+'">Set</button></td></tr>').join('');
 document.querySelectorAll('#kn button').forEach(b=>b.onclick=async()=>{await cmd('KNOB '+b.dataset.k+' '+$('k_'+b.dataset.k).value)})}catch(e){}}
window.onbeforeunload=()=>lines.length?'Recording not downloaded':undefined;
knobs();poll();setInterval(poll,1500);setInterval(pull,800);
</script></body></html>)HTML";

static unsigned long g_served = 0;
static uint8_t g_node = 0;
unsigned long lastServedMs() { return g_served; }
static void handleRoot() { g_srv->send_P(200, "text/html", PAGE); g_served = millis(); }
static void handleStatus() { g_srv->sendHeader("Cache-Control", "no-store"); g_srv->send(200, "application/json", g_status); g_served = millis(); }
static void handleCmd() {
  const String c = g_srv->arg("c");
  const String r = g_cmd ? g_cmd(c) : String("no handler");
  g_srv->send(200, "text/plain", r);
  g_served = millis();
}
static void handleLog() {
  const uint32_t since = (uint32_t)strtoul(g_srv->arg("since").c_str(), nullptr, 10);
  g_srv->sendHeader("Cache-Control", "no-store");
  g_srv->setContentLength(CONTENT_LENGTH_UNKNOWN);
  g_srv->send(200, "text/plain", "");
  char hb[32];
  snprintf(hb, sizeof(hb), "#first %lu\n", (unsigned long)(g_ringCount ? rd32(g_tail) : g_seq + 1));
  g_srv->sendContent(hb);
  size_t sent = 0, pos = g_tail; uint32_t seq; const char* text; uint16_t len;
  for (uint32_t left = g_ringCount; left; left--) {
    ringRead(pos, seq, text, len);
    if (seq <= since) continue;
    snprintf(hb, sizeof(hb), "%lu\t", (unsigned long)seq);
    g_srv->sendContent(hb);
    g_srv->sendContent(text, len);   // D92: straight from the ring, no String copy
    g_srv->sendContent("\n");
    sent += len;
    if (sent > 40 * 1024) break;   // the page pulls the rest next time
  }
  g_srv->sendContent("");
  g_served = millis();
}
// knobs JSON is built by the command handler ("KNOBS?") so this file does not depend on the sonar library
static void handleKnobs() { g_srv->sendHeader("Cache-Control", "no-store"); g_srv->send(200, "application/json", g_cmd ? g_cmd("KNOBS?") : String("[]")); }

static void routes(const String& p) {   // the page fetches "s", "cmd", "log", "k" relative to its own URL (<prefix>/)
  g_srv->on(p + "/", handleRoot);
  g_srv->on(p + "/s", handleStatus);
  g_srv->on(p + "/cmd", handleCmd);
  g_srv->on(p + "/log", handleLog);
  g_srv->on(p + "/k", handleKnobs);
}

bool begin(uint8_t node_id, const char* password, uint8_t channel, CommandFn fn) {
  if (g_on) return true;
  g_cmd = fn; g_node = node_id;
  WiFi.mode(WIFI_AP_STA);   // STA stays for ESP-NOW (same channel as the AP)
  char ssid[32]; snprintf(ssid, sizeof(ssid), "IceFish-bench-%u", node_id);
  if (!WiFi.softAP(ssid, password, channel, 0, 2)) return false;
  // fewer beacons = less radio activity while the sonar listens (1024 TU = ~1 s; phones accept it)
  wifi_config_t c;
  if (esp_wifi_get_config(WIFI_IF_AP, &c) == ESP_OK) { c.ap.beacon_interval = 1000; esp_wifi_set_config(WIFI_IF_AP, &c); }
  g_srv = new WebServer(80); g_own = true;
  routes("");
  g_srv->onNotFound([]() { g_srv->send(404, "text/plain", "not found"); });
  g_srv->begin();
  g_on = true;
  return true;
}

// D87: the page under <prefix>/ of a server that already exists (the hub's: hotspot + /, the suite page). The owner
// keeps calling its own handleClient(); nothing else to serve. "/bench" -> http://<ip>/bench/
bool attach(WebServer& srv, const char* prefix, uint8_t node_id, CommandFn fn) {
  if (g_on) return true;
  g_cmd = fn; g_node = node_id; g_srv = &srv; g_own = false;
  String p = prefix ? prefix : ""; while (p.endsWith("/")) p.remove(p.length() - 1);
  routes(p);
  if (p.length()) g_srv->on(p, []() { g_srv->sendHeader("Location", String(g_srv->uri()) + "/"); g_srv->send(302, "text/plain", ""); });   // /bench -> /bench/
  g_on = true;
  return true;
}

void loop() { if (g_on && g_own) g_srv->handleClient(); }
bool active() { return g_on; }
uint32_t lastSeq() { return g_seq; }

void push(const char* line) {
  if (!g_on) return;
  size_t len = strlen(line);
  if (len > 0xFFFF) len = 0xFFFF;
  const size_t need = REC_HDR + len;
  if (need > RING_BYTES) { ++g_seq; return; }     // never fits (the page counts the gap)
  const uint32_t seq = ++g_seq;
  // make room: the record goes at head if it fits before the end, else at 0 (wrap); evict the oldest until free
  for (;;) {
    if (!g_ringCount) { g_head = g_tail = 0; g_wrapAt = RING_BYTES; g_wrapped = false; break; }
    if (!g_wrapped) {                             // valid data is tail..head, free is head..end and 0..tail
      if (g_head + need <= RING_BYTES) break;     // fits at the end
      if (need <= g_tail) { g_wrapAt = g_head; g_head = 0; g_wrapped = true; break; }   // wrap: fits before tail
      ringPopOldest();                            // (tail moves up, or everything is gone)
    } else {                                      // wrapped: valid data is tail..wrapAt and 0..head, free is head..tail
      if (g_head + need <= g_tail) break;
      ringPopOldest();
    }
  }
  uint8_t* w = g_ringBuf + g_head;
  w[0] = (uint8_t)seq; w[1] = (uint8_t)(seq >> 8); w[2] = (uint8_t)(seq >> 16); w[3] = (uint8_t)(seq >> 24);
  w[4] = (uint8_t)len; w[5] = (uint8_t)(len >> 8);
  memcpy(w + REC_HDR, line, len);
  g_head += need; g_ringCount++;
}

void setStatus(const char* bench_line, const uint8_t* codes, uint16_t nbins, float bin_m, bool recording,
               uint32_t rec_lines, uint16_t rec_every, uint16_t raw_every_s, uint16_t vbat_mv) {
  if (!g_on) return;
  static const char H[] = "0123456789ABCDEF";
  String s; s.reserve(2 * nbins + 600);
  s = "{\"node\":"; s += String(g_node);
  s += ",\"seq\":"; s += String(g_seq);
  s += ",\"rec\":"; s += recording ? "true" : "false";
  s += ",\"rec_lines\":"; s += String(rec_lines);
  s += ",\"every\":"; s += String(rec_every);
  s += ",\"raw_every\":"; s += String(raw_every_s);
  s += ",\"heap\":"; s += String(ESP.getFreeHeap());
  s += ",\"vbat\":"; s += String(vbat_mv);
  s += ",\"bin_m\":"; s += String(bin_m, 4);
  // bottom depth for the red line: parsed from the bench line ("bottom=1.23m")
  const char* b = strstr(bench_line, "bottom=");
  s += ",\"bottom\":"; s += b ? String(atof(b + 7), 3) : String("0");
  s += ",\"line\":\"";
  for (const char* p = bench_line; *p; p++) { if (*p == '"' || *p == '\\') s += '\\'; s += *p; }
  s += "\",\"codes\":\"";
  for (uint16_t i = 0; i < nbins; i++) { s += H[codes[i] >> 4]; s += H[codes[i] & 15]; }
  s += "\"}";
  g_status = s;
}

}  // namespace bench
#endif  // SONAR_BENCH
