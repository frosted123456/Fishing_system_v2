// Runs the simulation + processing part of Frank's prototype (docs/prototype/sonar_display_prototype.html)
// headless in node and writes a reference trace for the C++ port (test/test_sonar_proc).
//   node tools/proto_reference.js [pings] > test/fixtures/proto_ref.txt
// Line formats (space separated):
//   D <ping> <f> <488 dB codes>          first 12 pings only, code = round((dB + 100) / (95/255))
//   P <ping> <t> <bottom> <nf> <hard> <ratio|nan> <ntracks> {<id> <depth> <vel> <label> <v> <s> <miss>}...
//   S <ping> <sum static> <sum sig> <count dyn>0>
const fs = require('fs');
const path = require('path');
const html = fs.readFileSync(path.join(__dirname, '..', 'docs', 'prototype', 'sonar_display_prototype.html'), 'utf8');
const a = html.indexOf("'use strict';");
const b = html.indexOf('/* ---------- State ---------- */');
if (a < 0 || b < 0) throw new Error('prototype layout changed');
const core = html.slice(a + "'use strict';".length, b);
const pings = +(process.argv[2] || 560);
const harness = `
  const out = [];
  const L = { Fish: 0, Bait: 1, 'Near bottom': 2, Cover: 3 };
  const HC = { hard: 0, medium: 1, soft: 2 };
  // restart() of the prototype: same seed, scene time 18 s, 160 pings, then live ticks
  seed = 20261005; sim.t = LOOP - 30; sim.jig = -99; sim.lastLt = ((sim.t % LOOP) + LOOP) % LOOP; sim.phase = 0;
  const proc = new Proc(opts);
  for (let k = 0; k < ${pings}; k++) {
    const t = sim.t;
    const raw = genPing(); sim.t += DT; sim.phase += 0.9;
    if (k < 12) for (let f = 0; f < 3; f++) out.push('D ' + k + ' ' + f + ' ' + Array.from(raw.dB[f], x => Math.round((x + 100) / (95 / 255))).join(' '));
    const r = proc.step(raw);
    const all = proc.tracks;
    let line = 'P ' + k + ' ' + t.toFixed(2) + ' ' + r.bottom.toFixed(6) + ' ' + r.nf.toFixed(6) + ' ' + HC[r.hard] + ' ' + (r.ratio == null ? 'nan' : r.ratio.toFixed(6)) + ' ' + r.tracks.length;
    for (const T of r.tracks) line += ' ' + T.id + ' ' + T.depth.toFixed(6) + ' ' + T.vel.toFixed(6) + ' ' + L[T.label] + ' ' + T.v.toFixed(6) + ' ' + T.s.toFixed(6) + ' ' + T.miss;
    out.push(line);
    let ss = 0, sg = 0, nd = 0;
    for (let i = 0; i < N; i++) { ss += proc.static[i]; sg += proc.sig[i]; }
    out.push('S ' + k + ' ' + ss.toFixed(3) + ' ' + sg.toFixed(3));
  }
  return out.join('\\n');
`;
process.stdout.write(new Function(core + harness)() + '\n');
