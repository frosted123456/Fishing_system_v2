# Sonar pod board v0.1 — generated design (NOT ordered, NOT reviewed by a human yet)

TUSS4470 front end (net-for-net from Open Echo `shield_003`, 200 kHz values fixed) + MT3608 boost 22 V for VPWR
(EN from the head, pulled up) + TPS63020 3.3 V + sense dividers + one 16-wire cable to the ESP32 in the head.
52 x 43 mm, 2 layers, all SMD on top, 4 x M3. Made with KiCad 7 tools (opens in KiCad 10).

| File | What |
|---|---|
| `sonar_pod.kicad_pro/.kicad_sch` | schematic, label style (every pin ends in a global label, no wires) |
| `sonar_pod.kicad_pcb` | routed board (freerouting), zones filled, DRC: 0 unconnected, 0 clearance |
| `sonar_pod_schematic.pdf`, `routed_f.png`, `routed_b.png` | previews |
| `sonar_pod_gerbers.zip` | gerbers + drill for JLCPCB |
| `sonar_pod_bom_jlcpcb.csv`, `sonar_pod_cpl_jlcpcb.csv` | assembly BOM (LCSC numbers) and pick-and-place |
| `sonar_pod.step` | 3D (library models only) |
| `src/design.py` | the single source: parts, pins, nets, positions. `make_pcb.py`, `make_sch.py`, `route.sh` regenerate everything |

## Head cable J4 (JST-PH 2.0 mm, 16)
1 3V3 · 2 GND · 3 GND · 4 NCS · 5 SCLK · 6 SDI · 7 SDO · 8 IO1 · 9 IO2 · 10 VOUT · 11 OUT4 · 12 HALL · 13 BOOST_EN · 14 VBAT_ADC · 15 NTC_ADC · 16 VPWR_ADC
ESP32 DevKit V1 side (proposal): NCS 5, SCLK 18, SDI 23 (MOSI), SDO 19 (MISO), IO1 26, IO2 25, VOUT 36, OUT4 32, HALL 27, BOOST_EN 33, VBAT_ADC 35, NTC_ADC 39, VPWR_ADC 34.

## Other connectors
J1 battery (XH 2): + −. J2 transducer (XH 5): A, B, shield(SGND), NTC, NTC-gnd. J3 hall (XH 3): 3V3, GND, OUT.

## Design notes
- VBOOST = 0.6 V x (1 + 360k/10k) = 22.2 V. Knob `vdrv` 20 (VPWR > VDRV + 0.3 V). TVS SMAJ24A on the rail.
- MT3608 EN: 100k pull-up to VB, J4.13 pulls it low to cut the sonar rail (sleep). Boost off = TUSS4470 unpowered: re-init after wake.
- TPS63020: EN = VIN (always on), PS/SYNC = GND (power save on), VINA via 10 R + 100 nF, 1.5 uH XFL4020, 2 x 10 uF in, 3 x 22 uF out, FB 1M/180k.
- Receive: R_INP 200 R 1206 + C_INP 330 pF -> INP; C_INN 22 nF to SGND; C_FLT 3.3 nF to SGND; spare DNP footprints C18/C19 for tuning.
  SGND island (F.Cu) joined to GND through FB1 ferrite (R9 0 R DNP alternative). Transducer shield pin on SGND.
- VOUT to the cable through 100 R + 1 nF. VBAT/2 and VPWR x 10/85 dividers, NTC 10k pull-up.
- Reverse battery: AO3401A P-FET high side, gate to GND through 100k.

## VERIFY BEFORE ORDERING (things I could not check from here)
1. U2 footprint `Texas_DSJ0014A` is GENERATED (pads +-1.45 mm, 0.85 x 0.28, EP 1.6 x 2.6, pins 1-7 left top->bottom,
   8-14 right bottom->top). Compare with the TI DSJ0014A drawing / KiCad 10 library and replace if KiCad 10 has it.
2. D1 (K=1 to VBOOST, A=2 to SW) and D2 (K=1 to VBOOST, A=2 to GND) polarity against the SMA / SOD-123W footprint pad 1.
3. JST pin-1 orientation on J1-J4 against the housings you crimp.
4. L1 saturation current (>1.5 A at 22 V boost from 3.6 V), L2 >4 A. LCSC numbers for L1/L2/FB1/D2/D3/J1-J4 were typed from memory: check each on lcsc.com.
5. Routing is freerouting's: look at the boost loop (L1-U3-D1-C8) and the INP/INN traces in KiCad; shorten by hand if they wander.
6. Silkscreen: 20 overlap warnings (reference text over pads) — cosmetic, clean up or let JLC strip them.
7. ESP32 ADC pins: 34/35/36/39 are input only (fine for ADC), 32/33 are output capable.

## Rough cost (JLCPCB, 5 boards, estimate — verify in the JLC quote)
PCB 5 x 52x43 2-layer ~ $4 · assembly setup + stencil ~ $10 · extended-part fees ~ 8 x $3 = $24 ·
parts ~ $9/board (TUSS4470 $3.1, TPS63020 ~$2.5, inductors ~$0.8, connectors ~$0.7, rest ~$2) x 5 = $45 · shipping ~ $20
=> about $100-120 for 5 assembled boards, ~$22 each. The shield + 2 modules route is ~$40-45 per tip-up.
