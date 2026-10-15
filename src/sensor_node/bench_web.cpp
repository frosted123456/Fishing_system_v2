// Phone test page for the sonar bench build (D56). See bench_web.h.
#include "bench_web.h"
#if defined(SONAR_BENCH) && SONAR_BENCH
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <deque>

namespace bench {

static WebServer* g_srv = nullptr;
static CommandFn g_cmd = nullptr;
static bool g_on = false;

// recording ring: whole lines with a sequence number, oldest dropped past RING_BYTES / RING_LINES
static const size_t RING_BYTES = 32 * 1024, RING_LINES = 96;
struct Line { uint32_t seq; String text; };
static std::deque<Line> g_ring;
static size_t g_ringBytes = 0;
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
 <div class="row"><button id="rec">Start recording</button><button id="dl">Download</button><button id="raw">RAW now</button></div>
 <p id="rinfo" class="k">Not recording.</p>
 <div class="row"><input id="note" placeholder="Note: ruler depth, target, temp..." style="flex:1"><button id="nb">Note</button></div>
 <div class="row" style="margin-top:8px"><span class="k">Record every</span><input id="every" type="number" min="1" max="20" value="1" style="width:56px"><span class="k">ping(s), RAW every</span><input id="rawe" type="number" min="0" max="600" value="30" style="width:64px"><span class="k">s</span><button id="ap">Apply</button></div>
</div>
<div class="card"><b>Knobs</b> <span class="k">(this tip-up only, not saved)</span><table id="kn"></table></div>
<div class="card"><div id="msg"></div></div>
<script>
const $=id=>document.getElementById(id);let lines=[],last=0,lost=0,recOn=false,busy=false,st={};
function msg(t){$('msg').textContent=t}
async function cmd(c){try{const r=await fetch('/cmd?c='+encodeURIComponent(c));const t=await r.text();msg(t);return t}catch(e){msg('No answer from the tip-up: '+e)}}
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
async function poll(){try{const r=await fetch('/s');st=await r.json();show();
 if(st.rec!==recOn&&!busy){recOn=st.rec;ui()}}catch(e){$('vals').innerHTML='<div class="bad">No connection to the tip-up</div>'}}
async function pull(){if(!recOn||busy)return;busy=true;try{const r=await fetch('/log?since='+last);const t=await r.text();
 for(const ln of t.split('\n')){if(!ln)continue;if(ln.startsWith('#first ')){const f=+ln.slice(7);if(last&&f>last+1)lost+=f-last-1;continue}
 const i=ln.indexOf('\t');if(i<0)continue;const s=+ln.slice(0,i);if(s<=last)continue;last=s;lines.push(ln.slice(i+1))}}catch(e){}busy=false;ui()}
function ui(){$('rec').textContent=recOn?'Stop recording':'Start recording';$('rec').className=recOn?'off':'';
 const kb=Math.round(lines.reduce((a,l)=>a+l.length+1,0)/1024);
 $('rinfo').innerHTML=(recOn?'<span class="ok">Recording</span> · ':'')+lines.length+' lines, '+kb+' kB kept on this phone'+(lost?' · <span class="bad">'+lost+' lines lost (keep this page open, screen on)</span>':'')}
$('rec').onclick=async()=>{busy=true;if(!recOn){if(lines.length&&!confirm('Start a new recording? The current one ('+lines.length+' lines) is cleared. Download it first if you need it.')){busy=false;return}
 lines=[];lost=0;last=+(await (await fetch('/s')).json()).seq||0;await cmd('REC WEB ON '+stamp());recOn=true}else{await cmd('REC WEB OFF');recOn=false;await pull()}busy=false;ui()};
$('dl').onclick=()=>{if(!lines.length){msg('Nothing recorded yet.');return}const b=new Blob([lines.join('\n')+'\n'],{type:'text/plain'});
 const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download='sonar_'+(st.node||'')+'_'+stamp().replace(/[:T]/g,'-')+'.txt';document.body.appendChild(a);a.click();a.remove()};
$('raw').onclick=()=>cmd('RAW');
$('nb').onclick=async()=>{const t=$('note').value.trim();if(!t)return;await cmd('NOTE ['+stamp()+'] '+t);$('note').value=''};
$('ap').onclick=async()=>{await cmd('RECEVERY '+$('every').value);await cmd('RAWEVERY '+$('rawe').value)};
async function knobs(){try{const k=await (await fetch('/k')).json();$('kn').innerHTML=k.map(o=>'<tr><td>'+o.label+'<div class="k">'+o.key+' · '+o.min+'-'+o.max+' '+o.unit+'</div></td><td><input id="k_'+o.key+'" type="number" value="'+o.val+'"></td><td><button data-k="'+o.key+'">Set</button></td></tr>').join('');
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
  String head = "#first " + String(g_ring.empty() ? g_seq + 1 : g_ring.front().seq) + "\n";
  g_srv->sendContent(head);
  size_t sent = 0;
  for (const Line& l : g_ring) {
    if (l.seq <= since) continue;
    String s = String(l.seq) + "\t";
    g_srv->sendContent(s);
    g_srv->sendContent(l.text);
    g_srv->sendContent("\n");
    sent += l.text.length();
    if (sent > 40 * 1024) break;   // the page pulls the rest next time
  }
  g_srv->sendContent("");
  g_served = millis();
}
// knobs JSON is built by the command handler ("KNOBS?") so this file does not depend on the sonar library
static void handleKnobs() { g_srv->sendHeader("Cache-Control", "no-store"); g_srv->send(200, "application/json", g_cmd ? g_cmd("KNOBS?") : String("[]")); }

bool begin(uint8_t node_id, const char* password, uint8_t channel, CommandFn fn) {
  if (g_on) return true;
  g_cmd = fn; g_node = node_id;
  WiFi.mode(WIFI_AP_STA);   // STA stays for ESP-NOW (same channel as the AP)
  char ssid[32]; snprintf(ssid, sizeof(ssid), "IceFish-bench-%u", node_id);
  if (!WiFi.softAP(ssid, password, channel, 0, 2)) return false;
  // fewer beacons = less radio activity while the sonar listens (1024 TU = ~1 s; phones accept it)
  wifi_config_t c;
  if (esp_wifi_get_config(WIFI_IF_AP, &c) == ESP_OK) { c.ap.beacon_interval = 1000; esp_wifi_set_config(WIFI_IF_AP, &c); }
  g_srv = new WebServer(80);
  g_srv->on("/", handleRoot);
  g_srv->on("/s", handleStatus);
  g_srv->on("/cmd", handleCmd);
  g_srv->on("/log", handleLog);
  g_srv->on("/k", handleKnobs);
  g_srv->onNotFound([]() { g_srv->send(404, "text/plain", "not found"); });
  g_srv->begin();
  g_on = true;
  return true;
}

void loop() { if (g_on) g_srv->handleClient(); }
bool active() { return g_on; }
uint32_t lastSeq() { return g_seq; }

void push(const char* line) {
  if (!g_on) return;
  const size_t len = strlen(line);
  while (!g_ring.empty() && (g_ringBytes + len > RING_BYTES || g_ring.size() + 1 > RING_LINES)) {   // make room first
    g_ringBytes -= g_ring.front().text.length();
    g_ring.pop_front();
  }
  if (ESP.getMaxAllocHeap() < 2 * len + 256) { ++g_seq; return; }   // heap too fragmented: drop (the page counts the gap)
  Line l; l.seq = ++g_seq; l.text = line;
  if (l.text.length() != len) return;             // out of memory: drop the line
  g_ringBytes += len;
  g_ring.push_back(l);
}

void setStatus(const char* bench_line, const uint8_t* codes, uint16_t nbins, float bin_m, bool recording,
               uint32_t rec_lines, uint16_t rec_every, uint16_t raw_every_s) {
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
  extern uint16_t batteryMv;
  s += ",\"vbat\":"; s += String(batteryMv);
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
