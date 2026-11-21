# Sonar pod board v0.2.1 — JLCPCB-ready (Basic parts, fixed placement file)

Same circuit as v0.2, made cheap and boring to assemble at JLCPCB. 40 x 34 mm, 2 layers, all SMD on top.

## What was wrong with the v0.2 order and what changed

| Problem you saw | Cause | Fix in v0.2.1 |
|---|---|---|
| ENIG forced (~$60) | 21 BOM lines had placeholder LCSC numbers; JLC auto-matched them to **01005** parts (0.4 x 0.2 mm). 01005 is below the Economic PCBA minimum (0402), which pushes the order to Standard PCBA / ENIG | Every line now has a real, checked LCSC number; smallest part is **0603**. Order as **Economic PCBA, HASL (lead-free)** — nothing on this board needs ENIG (finest pitch 0.5 mm, Economic allows 0.4) |
| Parts expensive | ~20 Extended lines x setup fee each, plus wrong-size parts | **5 Extended parts only** (TUSS4470, TPS63020, MT3608, two 4x4 inductors). Every resistor, capacitor, the diode, the ferrite and the MOSFET are **JLC Basic** (no fee) |
| Parts offset on import | my CPL flipped Y a second time (KiCad already writes Y-up) | origin moved to the board's bottom-left; gerbers, drill and CPL all use it; checked: every CPL centre sits on its pads (0 rows off by > 0.3 mm). Board spans 0..40 x 0..34 in all files |
| Rotations | KiCad vs JLC reference orientation | community correction table applied (SOT-23 / SOT-23-6 -90, QFN-20 +270). **Still check the 3D preview** |
| THT connectors missing in CPL | intentional (hand-soldered) | J1-J4 are now also out of the BOM: no THT fee; solder the 4 JST headers yourself (14 pins) |
| L2 matched to a resistor | wrong LCSC number | YJYCOIN YNR4020-1R5N, C340376, 1.5 uH 4x4, Isat 4.45 A |

## Value changes (to land on Basic parts)
- TPS63020 feedback: 100k / 18k (was 1M / 180k) -> 3.28 V, same output.
- MT3608 feedback: 200k / 5.1k (was 360k / 10k) -> **24.1 V** (was 22.2). VDRV knob 20 still has margin; TUSS4470 VPWR is rated 36 V.
- VPWR sense: 100k / 10k (was 75k / 10k): `VPWR_ADC = VPWR x 10/110` (2.2 V at 24 V). Firmware does not read it yet; when it does, the factor is 11.0.
- RINP 200 R in 0603 (burst current ~8 mA, ~11 mW peak).
- C12 2.2 uF 50 V back to 1206 (Basic). FB1 ferrite is the 0805 GZ2012D601TF (C1017 is 0805, not 0603 as v0.2 had it).
- D1 Schottky: B5819W (SOD-123, Basic) instead of PMEG6010.
- Removed: D2 (TVS on the 24 V rail — would sit at its stand-off), test points TP1/TP3 (VOUT is on J4.10, VBOOST on C8). D3 (transducer TVS) and C18/C19/R9 stay as empty footprints.
- TUSS4470 front-end caps are exactly the Open Echo shield's parts (330 pF C1664, 22 nF C21122, 3.3 nF C1613).

## BOM (28 lines, 46 placed parts) — library type checked on jlcpcb.com 2026-10-11
Extended: U1 TUSS4470TRTJR C1855776 · U2 TPS63020DSJR C15483 · U3 MT3608 C84817 · L1 SRP4020TA-220M C1847949 · L2 Sunlord SWPA4018S1R5NT C80662 (alt: YJYCOIN YNR4020-1R5N C340376, Isat 4.45 A).
Basic: all R (0603WAF series), all C (Samsung CL10/CL21/CL31, FH), Q1 AO3401A C15127, D1 B5819W C8598, FB1 GZ2012D601TF C1017.

## Ordering at JLCPCB
1. PCB: upload `sonar_pod_v021_gerbers.zip`, 2 layers, 1.6 mm, **HASL lead-free**, green. Do not tick ENIG.
2. Assembly: **Economic**, top side, 5 boards (or 2 assembled of 5).
3. BOM + CPL: `sonar_pod_v021_bom_jlcpcb.csv`, `sonar_pod_v021_cpl_jlcpcb.csv`. All 28 lines should match; if one shows "shortfall", pick the Basic alternative of the same value/size.
4. In the placement preview check: U1 pin-1 dot, U2 pin-1 (top-left of the 3x4 package, pins 1-7 on the left), U3 and Q1 orientation, D1 bar (cathode) toward VBOOST, L1/L2 centred on their pads (both inductors use the 4020 land; the SRP4020TA body is 4.1 mm).

Rough cost (estimate, your quote decides): PCB ~$2-4, Economic setup + stencil ~$10, 5 extended x ~$3 = ~$15, parts ~$27 for 5 boards (TUSS4470 alone is $15.50), shipping to Canada $10-30 -> **about 75-110 CAD for 5 boards before duties**, vs 200 CAD for the v0.2 quote.

## Still open (not checkable from here)
- U2 footprint `Texas_DSJ0014A` is generated: pads 0.85 x 0.28 at +-1.45 mm (same rows as KiCad's DFN-14 3x4), EP 1.6 x 2.85 (TI ~1.58 x 2.85). Compare against the DSJ0014A drawing in the TPS63020 datasheet (last pages) before ordering.
- Autorouted layout: correct and DRC-clean (0 unconnected, 0 clearance), not hand-optimized.
- Silkscreen reference text overlaps pads (cosmetic; JLC clips it).

## Files
`sonar_pod_v021.kicad_pro/.kicad_sch/.kicad_pcb`, `sonar_pod_v021_schematic.pdf`, `routed_f.png`, `routed_b.png`,
`sonar_pod_v021_gerbers.zip`, `sonar_pod_v021_bom_jlcpcb.csv`, `sonar_pod_v021_cpl_jlcpcb.csv`, `sonar_pod_v021.step`, `drc.txt`,
`src/` (design.py = single source; make_pcb.py, pack.py, dsn.py, import_ses.py, fixroute.py, export_jlc.py, make_sch.py).
