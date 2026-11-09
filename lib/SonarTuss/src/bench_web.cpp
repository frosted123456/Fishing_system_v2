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

static char g_statusBuf[2 * 488 + 700] = "{}";   // D100: the status JSON, written in place by setStatus()

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
<div class="card"><canvas id="cv" width="600" height="180"></canvas><div class="k">Last ping, 200 kHz: depth left to right (m), echo strength up. Red line = bottom. Orange = deconvolved (experimental). Purple dashed = baseline (lower envelope): each peak is measured over it.</div></div>
<div class="card"><div class="row" style="gap:8px;align-items:flex-start"><div style="flex:1;min-width:0"><div class="k">Echogram, raw</div><canvas id="e0" width="240" height="220" style="height:220px"></canvas></div><div style="flex:1;min-width:0"><div class="k">Deconvolved (experimental, D125)</div><canvas id="e1" width="240" height="220" style="height:220px"></canvas></div></div>
<label class="k"><input type="checkbox" id="ear"> colour the deconvolved view by echo area instead of height</label> <label class="k"><input type="checkbox" id="ebg"> background removed (D126)</label><div class="k">Time left to right (one column per page update), depth down to the bottom + 0.5 m. Raw and deconvolved: dark = noise floor, full colour 40 dB over it; background removed: from 2 dB over the baseline, full colour at 20 dB. Experimental: peaks narrowed with the hump measured on the ball (docs/SONAR_DECONV.md), heights kept.</div></div>
<div class="card">
 <div class="row"><button id="rec">Start recording</button><button id="dl">Download</button><button id="raw">RAW now</button><button id="sw" title="automated preset run: gain, cycles, frequency, threshold, averaging, drive (recording must be on)">Sweep</button><button id="swr" title="D123: receiver sweep: band-pass Q 2/3/4/5, quiet listening (vmode 1), gain 3 (~3 min, recording on, transducer still)">RX sweep</button><button id="tn" title="D111: finds the best burst frequency and band-pass code for this transducer (~1 min, bottom in view, keep still)">Tune</button></div>
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
 if(st.bottom>0){x.strokeStyle='#e53935';const X=st.bottom/m*w;x.beginPath();x.moveTo(X,0);x.lineTo(X,h);x.stroke()}
 const L=EH[EH.length-1];if(L&&L.o.length===n){const ln=(v,c,dash)=>{x.strokeStyle=c;x.setLineDash(dash);x.beginPath();for(let i=0;i<n;i++){const X=i*w/n,Y=h-(v[i]+100)*255/95*h/255;i?x.lineTo(X,Y):x.moveTo(X,Y)}x.stroke();x.setLineDash([])};ln(L.o,'#fb8c00',[]);if(L.b)ln(L.b,'#8e24aa',[4,3])}}
// D125 experimental sharpened echogram (docs/SONAR_DECONV.md). Input: dB per bin (array), noise floor dB, bottom m,
// bin size m. Output: {out: sharpened dB, area: area-equivalent dB} (same length; outside the window = input).
// Steps: baseline = floor + ring-down decay through the lowest points (D126e) >= nf; linear
// excess; Richardson-Lucy with the hump measured on the ball (07nov, 32 cycles); 5 cm re-blur; each lobe height-matched
// to the raw excess at its peak (keeps peak height and small bumps, only narrows); area = lobe flux / hump area.
// o.noDeconv (D126): skip the deconvolution (line already sharpened on the hub): baseline, lobes and area only.
// Returns { out, area, base } (base = the baseline in dB inside the window, the input elsewhere).
function sharpenEcho(db, nf, bottom, bin, o) {
  o = o || {};
  const dead = o.dead != null ? o.dead : 0.30, buf = o.buf != null ? o.buf : 0.35, IT = o.it || 100, P = 0.02;
  const N = db.length, a = Math.max(1, Math.round(dead / bin)), b = Math.min(N, Math.round(((bottom > 0 ? bottom : N * bin) + buf) / bin));
  const out = db.slice(), area = db.slice(), base = db.slice();
  const n = b - a; if (n < 8) return { out, area, base };
  // hump (2.5 cm bins, ref = index 3), resampled to this bin size
  const H0 = [0, 0.58, 0.86, 1.0, 0.88, 0.56, 0.28, 0.10, 0.04], h0 = i => (i < 0 || i > 8) ? 0 : (H0[Math.floor(i)] * (1 - (i % 1)) + (H0[Math.min(8, Math.floor(i) + 1)] * (i % 1)));
  const r = bin / 0.025, L0 = Math.floor(3 / r), H = []; for (let k = -L0; k * r <= 5; k++) H.push(h0(3 + k * r));
  const C = L0, L = H.length, HS = H.reduce((s, v) => s + v, 0), HN = H.map(v => v / HS);
  const y = db.slice(a, b);
  const k0 = Math.min(n, Math.round(0.15 / bin));
  const lin = v => Math.pow(10, v / 20), dbv = v => 20 * Math.log10(Math.max(v, 1e-9));
  // D126e baseline: power = F + A exp(-d / tau) (floor + ring-down tail), F, A >= 0, fitted through the lowest points
  // (points above the curve weigh 2 %, 8 passes, tau 2 cm .. 2 m): one continuous, never-rising line, >= nf
  const Pw = y.map(v => Math.pow(10, v / 10)), pr = Math.min(...Pw), Q = Pw.map(v => v / pr);   // normalised (>= 1)
  let w = new Array(n).fill(1), M = Q.slice();
  for (let pass = 0; pass < 8; pass++) {
    let best = null;
    for (let t = 0; t < 30; t++) {
      const tau = 0.02 * Math.pow(100, t / 29), rr = Math.exp(-bin / tau);
      let s11 = 0, s12 = 0, s22 = 0, r1 = 0, r2 = 0, E = 1;
      for (let i = 0; i < n; i++) { const sw = w[i] / (Q[i] * Q[i]); s11 += sw; s12 += sw * E; s22 += sw * E * E; r1 += sw * Q[i]; r2 += sw * E * Q[i]; E *= rr; }
      const cands = [], det = s11 * s22 - s12 * s12;
      if (det > 0) { const F = (r1 * s22 - r2 * s12) / det, A = (r2 * s11 - r1 * s12) / det; if (F >= 0 && A >= 0) cands.push([F, A]); }
      if (s11 > 0) cands.push([Math.max(r1 / s11, 0), 0]); if (s22 > 0) cands.push([0, Math.max(r2 / s22, 0)]);
      for (const [F, A] of cands) { let r = 0; E = 1; for (let i = 0; i < n; i++) { const m = F + A * E; r += w[i] * (m - Q[i]) * (m - Q[i]) / Math.max(m * Q[i], 1e-30); E *= rr; }
        if (!best || r < best[0]) best = [r, F, A, rr]; }
    }
    let E = 1; for (let i = 0; i < n; i++) { M[i] = best[1] + best[2] * E; E *= best[3]; w[i] = Q[i] > M[i] ? P : 1; }
  }
  const B = M.map(v => Math.max(10 * Math.log10(Math.max(v * pr, 1e-30)), nf));
  const e = y.map((v, i) => Math.max(lin(v) - lin(B[i]), 0));
  // Richardson-Lucy
  const conv = (v, k, ref) => { const res = new Array(n).fill(0); for (let i = 0; i < n; i++) { let s = 0; for (let j = 0; j < k.length; j++) { const m = i + ref - j; if (m >= 0 && m < n) s += v[m] * k[j]; } res[i] = s; } return res; };
  const HF = HN.slice().reverse(); let x = new Array(n).fill(Math.max(e.reduce((s, v) => s + v, 0) / n, 1e-9));
  const ee = e.map(v => v + 1e-12);
  if (o.noDeconv) x = e.slice();
  else for (let it = 0; it < IT; it++) { const est = conv(x, HN, C); const q = ee.map((v, i) => v / (est[i] + 1e-12)); const c = conv(q, HF, L - 1 - C); x = x.map((v, i) => v * c[i]); }
  const G = [-3, -2, -1, 0, 1, 2, 3].map(k => Math.exp(-0.5 * Math.pow(k * bin / 0.025 / 0.85, 2)));
  const s = o.noDeconv ? e.slice() : conv(x.map(v => v / HS), G, 3);
  const so = new Array(n).fill(0), ar = new Array(n).fill(0);
  // D126e lobes (after the ring zone) between minima of s, merged into one echo unless the deconvolved dip is >= 6 dB AND
  // the raw ping itself dips >= 3 dB between them (a flat plateau - barrel wall, weeds, a school - is not broken into
  // fish); everything from 6 cm above the bottom on is one echo. Each echo comes down to the baseline valley to valley.
  const smax = Math.max(...s, 1e-30), cc = Math.max(...e) / smax, lvl = i => dbv(s[i] * cc + lin(B[i]));
  const kb = bottom > 0 ? Math.round((bottom - 0.06) / bin) - a : n, am = (i0, i1) => { let j = i0; for (let i = i0; i < i1; i++) if (s[i] > s[j]) j = i; return j; };
  const mins = [k0]; for (let i = k0 + 1; i < n - 1; i++) if (s[i] <= s[i - 1] && s[i] <= s[i + 1]) mins.push(i); mins.push(n);
  const groups = [];
  for (let m = 0; m + 1 < mins.length; m++) { const i0 = mins[m], i1 = mins[m + 1]; if (i1 <= i0) continue; const jr = am(i0, i1);
    if (groups.length) { const g = groups[groups.length - 1], jl = am(g[0], g[1]);
      let pl = -1e9, pq = -1e9; for (let i = g[0]; i < g[1]; i++) pl = Math.max(pl, lvl(i)); for (let i = i0; i < i1; i++) pq = Math.max(pq, lvl(i));
      let rmin = 1e9; for (let i = jl; i <= jr; i++) rmin = Math.min(rmin, y[i]);
      if ((!(jr >= kb && jl < kb) && (Math.min(pl, pq) - lvl(i0) < 6 || Math.min(y[jl], y[jr]) - rmin < 3)) || (jr >= kb && jl >= kb)) { g[1] = i1; g[3] = jr; continue; } }
    groups.push([i0, i1, jr, jr]); }
  for (const g of groups) { const i0 = g[0], i1 = g[1], Lg = i1 - i0, l0 = i0 > k0 ? s[i0] : 0, l1 = i1 < n ? s[i1 - 1] : 0;
    const v = []; for (let i = i0; i < i1; i++) v.push(Math.max(s[i] - (Lg > 1 ? l0 + (l1 - l0) * (i - i0) / (Lg - 1) : l0), 0));
    let j = 0; for (let i = 0; i < Lg; i++) if (v[i] > v[j]) j = i; const pv = v[j]; if (pv <= 0) continue;
    let raw = 0; for (let i = Math.max(0, i0 + j - 1); i < Math.min(n, i0 + j + 2); i++) raw = Math.max(raw, e[i]);
    let sx = 0, sv = 0, ss = 0; for (let i = i0; i < i1; i++) { so[i] = v[i - i0] * raw / pv; sx += x[i]; sv += v[i - i0]; ss += s[i]; }
    const jf = g[2], jl = i1 === n ? n - 1 : g[3]; if (jl > jf) for (let i = jf; i <= jl; i++) so[i] = Math.max(so[i], e[i] * 0.5);   // merged echo: no dip deeper than 6 dB under the raw ping
    const fl = sx * sv / Math.max(ss, 1e-30); for (let i = i0; i < i1; i++) ar[i] = v[i - i0] >= 0.5 * pv ? fl : 0; }   // area: whole echo, painted on its core
  for (let i = 0; i < k0; i++) { so[i] = e[i]; ar[i] = 0; }   // ring zone left raw
  for (let i = 0; i < n; i++) { out[a + i] = Math.max(dbv(so[i] + lin(B[i])), B[i]); area[a + i] = ar[i] > 0 ? dbv(ar[i] / HS + lin(B[i])) : out[a + i]; base[a + i] = B[i]; }
  return { out, area, base };
}

let EH=[],lastHex='';const PALS=[[0,10,30,38],[.16,17,65,76],[.32,27,119,120],[.48,88,174,124],[.62,211,192,74],[.76,238,139,47],[.9,216,67,47],[1,255,230,210]];
function palc(v){v=Math.max(0,Math.min(1,v));for(let i=1;i<PALS.length;i++)if(v<=PALS[i][0]){const a=PALS[i-1],b=PALS[i],t=(v-a[0])/(b[0]-a[0]);return[0,1,2].map(k=>a[k+1]+(b[k+1]-a[k+1])*t)}return PALS[7].slice(1)}
function addEcho(){const hex=st.codes||'';if(!hex||hex===lastHex||!st.bin_m)return;lastHex=hex;const n=hex.length/2,d=[];for(let i=0;i<n;i++)d.push(-100+parseInt(hex.substr(2*i,2),16)*95/255);
 const f={};(st.line||'').replace(/(\w+)=([^\s|]+)/g,(a,k,v)=>{f[k]=v});let nf=parseFloat(f.noise);if(!(nf<0)){const t=d.slice(n>>1).sort((p,q)=>p-q);nf=t[t.length>>1]}
 const bt=+st.bottom||0;let r;try{r=sharpenEcho(d,nf,bt,st.bin_m,{dead:0.3})}catch(e){r={out:d,area:d}}EH.push({d,o:r.out,a:r.area,b:r.base,nf,bt});if(EH.length>240)EH.shift();drawE()}
function drawE(){if(!EH.length)return;const bin=st.bin_m,n=EH[EH.length-1].d.length;let mb=0;EH.forEach(e=>{if(e.bt>mb)mb=e.bt});const D=Math.min(n*bin,mb>0?mb+0.5:n*bin);
 [['e0','d'],['e1',$('ear').checked?'a':'o']].forEach(([id,key])=>{const c=$(id),x=c.getContext('2d'),W=c.width,Hh=c.height,img=x.createImageData(W,Hh),c0=W-EH.length;
  for(let col=0;col<W;col++){const e=EH[col-c0];for(let row=0;row<Hh;row++){const p=(row*W+col)*4;let rgb=[10,30,38];if(e){const i=Math.min(n-1,Math.floor(row/Hh*D/bin));rgb=($('ebg').checked&&key!=='d')?palc((e[key][i]-e.b[i]-2)/18):palc((e[key][i]-e.nf-4)/36)}img.data[p]=rgb[0];img.data[p+1]=rgb[1];img.data[p+2]=rgb[2];img.data[p+3]=255}}
  x.putImageData(img,0,0);x.fillStyle='#fff';x.font='10px system-ui';x.fillText(D.toFixed(1)+' m',2,Hh-3)})}
function show(){const L=st.line||'';const f={};L.replace(/(\w+)=([^\s|]+)/g,(a,k,v)=>{f[k]=v});
 const items=[['Bottom',f.bottom],['Bottom SNR',f.bot_snr],['Noise floor',f.noise],['Ring-down',f.ring],['ADC rate',(f.adc||'').replace('(want','')],['Time zero',f.t0],['Raw max',f.raw_max],['Ping time',f.ping]];
 const tg=(L.split('|')[2]||'').trim();
 $('vals').innerHTML=items.map(([k,v])=>'<div><div class="k">'+k+'</div><div class="v">'+(v||'-')+'</div></div>').join('')+'<div style="grid-column:1/-1"><div class="k">Targets</div><div class="v">'+(tg||'none')+'</div></div>'+
 ((/CLIPPED|FAULT|FALLBACK|ALARM/.test(L))?'<div style="grid-column:1/-1" class="bad">'+(L.match(/\((CLIPPED|FALLBACK|RING ALARM)\)|BURST FAULT\([^)]*\)/g)||[]).join(' ')+'</div>':'');
 $('node').textContent=st.node?('tip-up '+st.node+' · battery '+st.vbat+' mV · free RAM '+st.heap):'';draw()}
async function poll(){try{const r=await fetch('s');st=await r.json();addEcho();show();
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
$('tn').onclick=async()=>{if(confirm('Tune the burst frequency and band-pass? About 1 minute. Keep the transducer still with the bottom in view.')){const r=await cmd('TUNE');msg(r)}};$('swr').onclick=async()=>{if(!recOn){msg('Start recording first, then RX sweep.');return}if(confirm('Run the receiver sweep? 8 steps, about 3 minutes. Keep the transducer still, bottom in view.'))msg(await cmd('SWEEP RX'))};$('sw').onclick=async()=>{if(!recOn){msg('Start recording first, then Sweep.');return}if(confirm('Run the preset sweep? ~15 steps, about 3 minutes. Keep the transducer still.'))await cmd('SWEEP')};
$('nb').onclick=async()=>{const t=$('note').value.trim();if(!t)return;await cmd('NOTE ['+stamp()+'] '+t);$('note').value=''};
$('ap').onclick=async()=>{await cmd('RECEVERY '+$('every').value);await cmd('RAWEVERY '+$('rawe').value)};
async function knobs(){try{const k=await (await fetch('k')).json();$('kn').innerHTML=k.map(o=>'<tr><td>'+o.label+'<div class="k">'+o.key+' · '+o.min+'-'+o.max+' '+o.unit+'</div></td><td><input id="k_'+o.key+'" type="number" value="'+o.val+'"></td><td><button data-k="'+o.key+'">Set</button></td></tr>').join('');
 document.querySelectorAll('#kn button').forEach(b=>b.onclick=async()=>{const k=b.dataset.k;await cmd('KNOB '+k+' '+$('k_'+k).value);await knobs();const r=$('k_'+k);if(r){r.style.outline='2px solid var(--ok)';setTimeout(()=>r.style.outline='',2500)}})}catch(e){}}
window.onbeforeunload=()=>lines.length?'Recording not downloaded':undefined;
$('ear').onchange=drawE;$('ebg').onchange=drawE;knobs();poll();setInterval(poll,1500);setInterval(pull,400);
</script></body></html>)HTML";

static unsigned long g_served = 0;
static uint8_t g_node = 0;
unsigned long lastServedMs() { return g_served; }
static void handleRoot() { g_srv->send_P(200, "text/html", PAGE); g_served = millis(); }
static void handleStatus() { g_srv->sendHeader("Cache-Control", "no-store"); g_srv->send_P(200, "application/json", g_statusBuf); g_served = millis(); }   // send_P streams the buffer; send() would copy it into a String
static void handleCmd() {
  const String c = g_srv->arg("c");
  const String r = g_cmd ? g_cmd(c) : String("no handler");
  g_srv->send(200, "text/plain", r);
  g_served = millis();
}
static void handleLog() {
  // D110: the log pull was the hub's biggest heap swing: up to 8 kB of TCP buffers in ~3 small writes per line while
  // a slow phone drains them (heap_min 2.5-10 kB and watchdog resets on 07nov, every one during a recording). Now:
  // at most 4 kB per pull (every 0.4 s), written in ~1.4 kB chunks from a static buffer, and nothing at all under 16 kB free heap
  // (the page simply pulls again 0.4 s later; the ring keeps the lines).
  const uint32_t since = (uint32_t)strtoul(g_srv->arg("since").c_str(), nullptr, 10);
  g_srv->sendHeader("Cache-Control", "no-store");
  if (ESP.getFreeHeap() < 16 * 1024) { g_srv->send(503, "text/plain", "busy"); g_served = millis(); return; }
  g_srv->setContentLength(CONTENT_LENGTH_UNKNOWN);
  g_srv->send(200, "text/plain", "");
  static char chunk[1460]; size_t cn = 0;
  auto flush = [&]() { if (cn) { g_srv->sendContent(chunk, cn); cn = 0; } };
  auto put = [&](const char* t, size_t n) {
    while (n) { const size_t room = sizeof(chunk) - cn, m = n < room ? n : room; memcpy(chunk + cn, t, m); cn += m; t += m; n -= m; if (cn == sizeof(chunk)) flush(); }
  };
  char hb[32];
  const int hn = snprintf(hb, sizeof(hb), "#first %lu\n", (unsigned long)(g_ringCount ? rd32(g_tail) : g_seq + 1));
  put(hb, (size_t)hn);
  size_t sent = 0, pos = g_tail; uint32_t seq; const char* text; uint16_t len;
  for (uint32_t left = g_ringCount; left; left--) {
    ringRead(pos, seq, text, len);
    if (seq <= since) continue;
    const int sn = snprintf(hb, sizeof(hb), "%lu\t", (unsigned long)seq);
    put(hb, (size_t)sn); put(text, len); put("\n", 1);
    sent += len;
    if (sent > 4 * 1024) break;    // D110: 4 kB per pull (was 8), the page pulls every 0.4 s (was 0.8): 10 kB/s, a 4-ping/s recording makes ~5
  }
  flush();
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

// D96: reserve a record of `len` text bytes and return where to write them (the caller fills exactly len bytes, no
// terminator); nullptr when it can never fit (the page counts the gap). A RAW line is formatted straight into the
// ring this way: no 5 kB buffer on the heap at all.
char* pushReserve(size_t len) {
  if (!g_on) return nullptr;
  if (len > 0xFFFF) len = 0xFFFF;
  const size_t need = REC_HDR + len;
  if (need > RING_BYTES) { ++g_seq; return nullptr; }     // never fits (the page counts the gap)
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
  g_head += need; g_ringCount++;
  return (char*)(w + REC_HDR);
}
void push(const char* line) {
  const size_t len = strlen(line);
  char* dst = pushReserve(len);
  if (dst) memcpy(dst, line, len > 0xFFFF ? 0xFFFF : len);
}

void setStatus(const char* bench_line, const uint8_t* codes, uint16_t nbins, float bin_m, bool recording,
               uint32_t rec_lines, uint16_t rec_every, uint16_t raw_every_s, uint16_t vbat_mv) {
  if (!g_on) return;
  static const char H[] = "0123456789ABCDEF";
  // D100: no String: one static buffer, written in place (was a 1.6 kB String built and copied on every ping)
  char* o = g_statusBuf; const char* end = g_statusBuf + sizeof(g_statusBuf) - 8;
  const char* b = strstr(bench_line, "bottom=");
  { const int r = snprintf(o, end - o, "{\"node\":%u,\"seq\":%lu,\"rec\":%s,\"rec_lines\":%lu,\"every\":%u,\"raw_every\":%u,\"heap\":%lu,\"vbat\":%u,\"bin_m\":%.4f,\"bottom\":%.3f,\"line\":\"",
                g_node, (unsigned long)g_seq, recording ? "true" : "false", (unsigned long)rec_lines, rec_every, raw_every_s, (unsigned long)ESP.getFreeHeap(), vbat_mv, (double)bin_m, b ? atof(b + 7) : 0.0);
    o += r < end - o ? r : end - o; }
  for (const char* p = bench_line; *p && o < end - 2; p++) { if (*p == '"' || *p == '\\') *o++ = '\\'; *o++ = *p; }
  { const int r = snprintf(o, end - o, "\",\"codes\":\""); o += r < end - o ? r : end - o; }
  for (uint16_t i = 0; i < nbins && o < end - 2; i++) { *o++ = H[codes[i] >> 4]; *o++ = H[codes[i] & 15]; }
  { const int r = snprintf(o, end - o, "\"}"); o += r < end - o ? r : end - o; }   // D100: snprintf returns the would-be length: clamp
  *o = 0;
}

}  // namespace bench
#endif  // SONAR_BENCH
