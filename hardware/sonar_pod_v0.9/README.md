# Sonar pod board v0.9: v0.8 + JLC-assembled 200 kHz transformer drive (2026-11-07)

Status: generated, **not ordered**. Same 58 x 53 mm, 4 layers (F signal / In1 GND / In2 GND / B signal), 1.6 mm, HASL.
DRC: **0 unconnected, 0 clearance, 0 shorts, 0 keep-out** (silkscreen, library, one starved thermal on U9 pad 2 and
the four 0.1 mm courtyard overlaps inherited from v0.8 only). Netlist check: 353 pads, 0 mismatches against design.py.
XA (the ±110 V secondary) keeps ≥ 1.0 mm to every other net and to the GND pours on every layer.

Why: the barrel tests showed the drive is the missing part (today 40 V p-p from the TUSS4470 bridge). D119: the TDK
B78416A2386A003 (EP6 SMD, **300 kHz**, 1:1:9, 0.23 mH) gives ~216 V p-p on the transducer at a 9.4 V centre tap,
**+12.6 dB theoretical** (1-3 dB losses est.). Frank: "lets just do the best option for v0.9".

## Variants (one board, four BOMs; folder `assembly/<design>/`)

| Variant | Drive | R4 | Fitted only here | Not fitted |
|---|---|---|---|---|
| `tipup_direct` | direct (TUSS bridge, 40 V p-p) | 200k → 24.1 V | R40 / R41 0R links | T1, Q1, R42-R45, C44, C49, C43, D7, R48, U5 |
| `lora_direct` | direct + Ra-01SH LoRa | 200k → 24.1 V | R40 / R41, U5 | T1, Q1, R42-R45, C44, C49, C43, D7, R48 |
| **`tipup_xfmr`** | **transformer, push-pull** | **75k → 9.4 V** | T1, Q1, R42-R45, R46 10R, C44, C49, C43, D7, R48, R8B/R8C 1.1k | R40, R41, U5 |
| **`lora_xfmr`** | **transformer + Ra-01SH LoRa** | **75k → 9.4 V** | same as tipup_xfmr + U5 (Ra-01SH) | R40, R41 |

Same PCB for all four (one gerber zip). Each folder holds the JLC BOM + CPL (.csv, upload these) and the same in Excel.
R8B / R8C (receive series, 2 x 1206): 0R + 0R direct drive, 1.1k + 1.1k xfmr.

## What changed vs v0.8

| # | v0.8 | v0.9 xfmr | Why |
|---|---|---|---|
| 1 | P1 2x3 header for a hand-wired transformer | **T1 TDK B78416A2386A003**, EP6 SMD footprint drawn from TDK IND1867-Q | JLC places it (Global Sourcing, MPN in the BOM) |
| 2 | Q1/Q2 AP1002 SOT-23, 2 A | **Q1 AOSD62666E** dual N-FET SO-8, 60 V, 14.5 mΩ @10 V, Vth 1.2 V, ESD gate (LCSC C541719, Extended) | primary peaks 3-13 A (unknown Rm) |
| 3 | centre tap from 24 V | VBOOST **9.4 V** (R4 75k) | keeps the core near the 300 kHz volt-seconds (est.) |
| 4 | 2 x 10 µF on VTX | C44 10 µF at the FET sources + **C49 1000 µF 16 V** (Honor RVT 10x10, C3359) behind **R46 10 Ω** | a 32-cycle burst takes 4-18 mJ; the 10 Ω keeps the burst off VPWR |
| 5 | C43 DNP | **1.2 nF 630 V C0G 1206** (Walsin, C3851387) across the secondary | 0.23 mH resonates with the 1.5 nF transducer at 271 kHz; +1.2 nF → ~199 kHz |
| 6 | R8B one 0R | **R8B + R8C, 2 x 1.1k 1206** in series | ≥ 250 V across the pair on transmit |
| 7 | D7 DNP | **BAV99** | clamps RINP on transmit (~50 mA peak) |
| 8 | XB driven by OUTB | **R48 0R XB → GND** (xfmr only; TDK: pin 6 to GND) | secondary low side grounded |
| 9 | L3 shunt inductor footprint | **removed** | not needed with the TDK part; its space holds R48 |
| 10 | - | F.Cu keep-out over the hand-fitted U.FL J6 | its mask opening had exposed a routed track |
| 11 | - | **R50 DNP 0805 across the secondary (XA → GND)**, fit 4.7k-22k at the bench | damps the tuned tank if the ring-down grows (D122) |

Unchanged: everything else from v0.8 (receiver layout, ADC, power, USB, LoRa, connectors).

## T1 pin map (checked on the TDK drawing, data sheet p.3, 10/21)

| Pin | Winding | Net |
|---|---|---|
| 1 | N1a end | DB (Q1 drain B) |
| 2 | primary centre tap (dot) | VTX (9.4 V) |
| 3 | N1b end (dot) | DA (Q1 drain A) |
| 4 | N2 hot (dot) | XA → J2.1 + receive path |
| 6 | N2 low, "(GND)" | GND |

Land pattern (top view): pads 1.2 x 2.5 mm, row 1-2-3 at 2.2 mm pitch, pin 6 above pin 1, pin 4 above pin 3, 9.2 mm
overall (row centres 6.7 mm). Body 7.6 x 8.8 mm max. Pin 1 mark on the silk below pad 1.

## Firmware for the xfmr build

`drive 1` (PRE_DRIVER_MODE, reg 0x1A bit 6; caps VDRV at 10 V) and **`vdrv 8`** (VDRV must stay below VPWR ≈ 9.3 V).
Every flash resets the knobs to defaults (D114): set both on the chalet after each flash. Do NOT run `drive 0` on an
xfmr board (the TUSS bridge would drive the FET gates as a bridge).

## Bench order (first board)

1. Before power: T1 orientation (pin 1 mark), C49 polarity, Q1 pin 1.
2. `drive 1`, `vdrv 8`, **`cycles 8`**: scope VTX (sag), a drain (spikes < 50 V), the transducer (expect ~200 V p-p).
3. 16 then 32 cycles; TUNE (the tuned load moves the peak), SWEEP.
4. Floor SNR vs a direct-drive board in the barrel: expect +10-12 dB.
5. Only if clean: 13 V build (R4 105k, `vdrv 10`), +2.7 dB more.

## Risks / verify before ordering

| Item | Risk | Check |
|---|---|---|
| T1 stock | **EOL** at TDK; Mouser ~1,850, Digi-Key ~1,550 | Order via JLC Global Sourcing now; buy spares |
| JLC rotations | Q1 (SOIC-8), T1 (EP6), C49 (polarised) rotations are not confirmed | Check each in JLC's placement preview before paying |
| Transducer Rm | unknown: sets FET peak current (3-13 A) and burst energy | seller, or signal generator + 100 Ω + scope |
| Volt-seconds | no V·µs rating published; 199 kHz is 1.5x longer per half-cycle than 300 kHz | start at 9.4 V, scope the primary current for saturation (sharp rise at the end of each half-cycle) |
| Receive loss | 2.2 kΩ series before C15 / INP attenuates the echo (est. 2-6 dB, depends on the INP impedance) | compare floor SNR; if too much, 2 x 470 Ω (D7 then sees ~120 mA peak, BAV99 OK for a 160 µs burst) |
| Drain A path | DA reaches Q1 on B.Cu, ~16 mm (DB ~10 mm), over the In1 plane | scope both drains; ringing would show as unequal spikes |
| AOSD62666E | Extended part; 1,759 in stock | alternatives on LCSC: STN4828, AGM612MN (check pinout) |
| Starved thermal | U9 pad 2 (GND) has one pour spoke | electrically connected; cosmetic |

## Interference review (layout checks done; bench measurements still needed)

| Path | What the layout shows | Verdict / how to confirm |
|---|---|---|
| Transmit loop VTX → C44 → Q1 → T1 | Confined to the south-east corner: ≥ 14 mm from INP/INN, ≥ 16 mm from RINP, ≥ 6 mm from the ADC input. Active only during the burst, when the receiver is blanked anyway | Low risk |
| T1 stray field | EP core is a closed ferrite shape (low leakage field, est.) | Low risk |
| Gate lines OUTA/OUTB | Cross RIN0/RIN1 on the other layer; they switch only during the burst and sit at 0 V (10k pull-downs) while listening | Low risk |
| C49 refill while listening | After each burst C49 refills through R46 (50-190 mA decaying, τ 10 ms): extra MT3608 load during the echo window. VPWR reaches the TUSS through R7 10 Ω + C48 2.2 µF (corner ~7 kHz), so the 200 kHz band sees it heavily filtered | Medium-low: compare the bench `nf` (noise floor) of an xfmr board with a direct-drive board; if higher, R46 → 33-47 Ω (slower refill) |
| XA ±110 V | ≥ 1.0 mm to every other net and to the pours on all 4 layers (R50's own pads 0.6 mm, IPC-2221 B2 for 101-150 V) | Low risk |
| Ring-down | +12 dB drive into a tuned tank pushes the ring-down deeper (blind zone longer, est.) | Medium: measure `ring=` at 8/16/32 cycles; fit R50 (start 10k), raise `dead`, or use fewer cycles |
| Neighbour tip-ups | Pings 12 dB louder are heard farther away by other pods (D57) | Medium: two pods on the ice at the usual spacing, check for crossed echoes |
| Radios | 200 kHz harmonics are far from 868/915 MHz and 2.4 GHz; the switchers are unchanged from v0.8 | Low risk |

Nothing on paper guarantees zero interference: the two "medium" rows are bench / ice checks.

## How to close each risk

| # | Risk | Action | When | Pass |
|---|---|---|---|---|
| 1 | JLC rotations (Q1, T1, C49) | In JLC's placement preview: Q1 dot on pin 1, T1 pin-1 corner on the silk mark (pad 1 = bottom-left of the 3-pad row), C49 minus stripe on the GND pad. Screenshot the preview if unsure | at order | all three match |
| 2 | T1 end of life | Buy 10 now (Digi-Key / Mouser, ~3-5 $) or through JLC Global Sourcing into your JLC parts stock | now | parts in hand |
| 3 | Transducer Rm unknown | Scope VTX at C44 during a `cycles 8` burst: energy = C · V · ΔV, Rm ≈ (V_rms secondary)² / power. Or ask the seller | first board | Q1 peak < 10 A (est. from Rm) |
| 4 | Core saturation at 199 kHz | Start at 9.4 V and `cycles 8`; scope a drain: a drop of the drain voltage late in each half-cycle = saturation | first board | flat drain plateau |
| 5 | Receive loss (2.2 kΩ) | Compare floor / ball SNR against a direct board; if low, R8B/R8C → 470 Ω each (same 1206 pads) | first board | ≤ 2 dB loss |
| 6 | Drain spikes / DA vs DB | Scope both drains | first board | spikes < 45 V (60 V FET); else lower centre tap |
| 7 | Ring-down | `ring=` on the bench line at 8/16/32 cycles | first board | blind zone ≤ 0.6 m, else fit R50 |
| 8 | Noise floor | bench `nf` xfmr vs direct board | first board | within 1 dB |
| 9 | Neighbour crosstalk | two pods, normal spacing | first ice | no crossed echoes |

Suggested order: 5 bare boards, assemble 2 xfmr + 2 tipup (direct drive as the reference for rows 5 and 8).

## Files

| File | Use |
|---|---|
| sonar_pod_v09_gerbers.zip | JLC PCB order (4 layers, 1.6 mm) |
| assembly/<design>/sonar_pod_v09_<design>_bom_jlcpcb.csv + _cpl_jlcpcb.csv | JLC assembly upload, one folder per design (xfmr BOMs have the T1 Global Sourcing line) |
| assembly/<design>/*_bom.xlsx, *_centroid.xlsx | same, Excel |
| sonar_pod_v09.kicad_pcb / .kicad_sch / .kicad_pro | KiCad 7 board and label-style schematic |
| routed_f/b.png, fab_f/b.png, noise_v09.png | renders and the noise audit |
| src/ | the generator pipeline (design.py → build_v09.sh → post_v09.sh → export_jlc.py) |
| TRANSFORMER_200K.md | the drive calculation (D119) |

## Note from the shield build (2026-11-14, D168 to D181): converter noise

On the shield + XL6009 build (README 4.4.6) the boost converter was the receiver's noise floor: -75 dB with it
running, -86 dB with it switched off while each ping listens (hub 1, in water). The XL6009 switches at 400 kHz and its
noise sat at half of that, inside the 200 kHz band. This board is not changed by that work (other converter, other
drive), but check the same thing at bring-up: listen-only pings with the boost running and with its enable pin (GPIO33)
held LOW, before trusting the receive floor. The firmware can gate it per burst (`SONAR_BOOST_GATE=1`); here it is off
by default because VPWR feeds the TUSS4470 regulator (vmode 0) and the rail must hold through the burst.
