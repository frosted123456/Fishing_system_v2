# Sonar pod board v0.2 — tight variant (38 x 32 mm), generated, NOT ordered, NOT human-reviewed

Same circuit as v0.1 (TUSS4470 front end from Open Echo shield_003, MT3608 22 V boost with EN from the head,
TPS63020 3.3 V, sense dividers, Hall pull-up, 16-wire head cable). Changes vs v0.1:

| | v0.1 | v0.2 |
|---|---|---|
| Size | 52 x 43 | **38 x 32** (same 2-layer, top-side-only JLC process) |
| Head connector | JST-PH 1x16 (34 mm) | **JST-PHD 2x8** (B16B-PHDSS, 2.0 mm, 19 mm) |
| Signal R/C | 0603 | **0402** (LCSC numbers marked `verify-0402`: pick basic parts in the JLC BOM tool) |
| Boost inductor | NR6028 6x6 | **4020 22 uH** (SWPA4020S220MT or XFL4020-223; Isat >= 1 A) |
| TVS | SMA | **SMF** (SMF24A / SMF24CA) |
| 50 V caps | 1206 | C8/C9 10 uF 50 V 1206 kept; C12 2.2 uF 50 V -> 0805 |
| Holes | 4 x M3 | 3 x M2 |
| Test points | 5 | 2 (VOUT, VBOOST) |
| Placement | by hand | fixed ICs/connectors + greedy packer (`src/pack.py`); routed by freerouting (220 passes) |

DRC: 0 unconnected, 0 clearance, 0 copper-edge (rule 0.2 mm). Remaining: silkscreen reference text over pads
(cosmetic — let JLC strip or move in KiCad), 2 thermal-spoke warnings.

Files: `sonar_pod_v02.kicad_pro/.kicad_sch/.kicad_pcb`, `sonar_pod_v02_schematic.pdf`, `routed_f.png`, `routed_b.png`,
`sonar_pod_v02_gerbers.zip`, `sonar_pod_v02_bom_jlcpcb.csv`, `sonar_pod_v02_cpl_jlcpcb.csv`, `sonar_pod_v02.step`,
`drc.txt`, `src/` (design.py = single source; make_pcb.py, pack.py, make_sch.py, route.sh, import_ses.py, fixroute.py).

Pinouts, design notes and the VERIFY-BEFORE-ORDER list are the same as v0.1's README (hardware/sonar_pod_v0.1),
plus: (8) 0402 and the new inductor/TVS/PHD LCSC numbers are placeholders — assign in the JLC BOM tool;
(9) this layout is denser: check the boost loop (L1-U3-D1-C8/C9 is spread along the bottom edge — acceptable at
1.2 MHz with 10 uF x2, but a hand pass tightening it is worth 20 minutes); (10) 2 x 8 PHD header: pin 1 is the
square pad, odd pins in row 1 — document the cable both ends.

Cost: same as v0.1 (~$100-120 per 5 assembled, ~$22 each); the PCB itself is cheaper by a dollar, nothing else changes.
