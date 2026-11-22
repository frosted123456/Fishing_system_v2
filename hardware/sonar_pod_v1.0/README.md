# Sonar pod board v1.0: one board, three drive options (2026-11-21; review round and parts review the same day)

Status: generated, **not ordered, not built**. Same 58 x 53 mm, 4 layers (F signal / In1 GND / In2 GND / B signal),
1.6 mm, HASL. DRC: **0 unconnected, 0 clearance, 0 shorts, 0 keep-out**. Netlist check: 382 pads, 0 mismatches against
design.py. Left in the DRC report: silkscreen, library warnings, 4 starved thermals, 14 courtyard overlaps (4 from v0.8;
the 10 new ones checked: no pads of different nets closer than 0.43 mm, no bodies closer than 0.14 mm).

Frank (21 Nov): "work all three options with a way to switch it", "software and link", "don't forget the enable pin",
"make sure the noise and interference is minimised".

## The three options and how to switch

| Option | On the transducer (theoretical) | Switched by | How |
|---|---|---|---|
| **20 V direct** | 40 V p-p | firmware | knob `vmode 0` (the TUSS4470's own regulator). IO10 low |
| **33 V direct** | ~65 V p-p (+4.3 dB) | firmware | knob `vmode 2`. The driver charges VDRV to 20 V, then raises IO10: Q3 feeds VDRV from the 33.9 V rail |
| **Transformer** | ~216 V p-p (+12.6 dB before losses) | 0R links (solder) | fit the XFMR set, remove the DIRECT set (table below). The firmware reads the strap R59 and sets `drive 1` itself |

20 V ↔ 33 V needs no soldering and can change from the chalet page. Direct ↔ transformer is a soldering job (the
transducer wiring changes and the secondary carries ±110 V, so no small FET can switch it).

### Links (the only difference between a direct and a transformer board)

| Build | Fit | Leave empty |
|---|---|---|
| **DIRECT** | R40, R41, R52 (0R), R56 (200k) | R42, R43, R46, R48, R51, R53, R54, R59 |
| **XFMR** | R42, R43 (10R), R46 (10R), R48, R51, R53, R59 (0R), R54 (100k) | R40, R41, R52, R56 |

Never both sets. R46 and R54 always together (R54 lowers the boost to 9.6 V; without it the centre tap sits at 17.0 V on
a 16 V capacitor). Everything else is on every board: T1, Q1, C44, C49, C43, D7, R8B / R8C, R44 / R45.

| Link | Size | Job |
|---|---|---|
| R40 / R41 | 1206 / 0603 | DIRECT: OUTA / OUTB to the transducer |
| R52 | 1206 | DIRECT: bypasses the receive series resistors R8B + R8C |
| R56 | 0603 | DIRECT: gate divider of the 33 V switch Q3. Without it (XFMR) the switch cannot turn on, whatever the firmware does |
| R42 / R43 | 0603 | XFMR: OUTA / OUTB to the FET gates |
| R46 | 0603 | XFMR: boost output to the transformer centre tap |
| R54 | 0603 | XFMR: boost 17.0 V → 9.6 V |
| R48 | 0603 | XFMR: transducer low side XB to GND |
| R51 | 1206 | XFMR: transformer secondary to the transducer (new: in direct drive the 0.23 mH winding must not sit across the bridge) |
| R53 | 0603 | XFMR: connects the receive clamp D7 (it would short a direct drive) |
| R59 | 0603 | XFMR: board-type strap, IO11 to GND |

Assembly folders: `tipup_direct`, `lora_direct` (DIRECT set), `tipup_xfmr`, `lora_xfmr` (XFMR set). To save the
transformer parts on a board that will stay direct, delete T1, Q1, C49, C44, C43 from its BOM.

## What changed vs v0.9

| # | v0.9 | v1.0 | Why |
|---|---|---|---|
| 1 | MT3608 at 24.1 V (direct) or 9.4 V (xfmr) | MT3608 at **17.0 V** (R4 82k / R5 3k) + **voltage doubler** on its switch node (C50 1 µF, D9A + D9B B5819W, C47 10 µF) = **VDBL ≈ 33.9 V** | The MT3608 stops at 28 V; 33.9 V nominal keeps VPWR ≤ 36 V over tolerance with ordinary 1 % parts |
| 2 | VPWR from the boost | VPWR from VDBL through the same R7 10 Ω + C48 + C11 | VPWR must stay above VDRV |
| 3 | - | **Q3 BSS84 + Q4 2N7002 + D10 BAV99 + R55-R58** on IO10 | 33 V feed to VDRV, switched by firmware. The two diodes of D10 keep VDRV ~1.2 V under VPWR (as the two-diode drop of today's hand-wired 30 V build); R58 47k bleeds VDRV |
| 4 | variants by BOM values (R4, R8B/C) | variants by **links** only; R8B / R8C 1k on every board, bypassed by R52 | one board, both drives |
| 5 | T1 secondary on XA | T1 secondary on its own net **XS**, joined by R51; C43 on XS | isolates the winding in direct drive |
| 6 | R50 (DNP damping) | removed; a damping resistor goes on top of C43 if needed | space |
| 7 | D7 always on RINP | D7 through link R53 | D7 would clamp a direct drive |
| 8 | IO10 / IO11 spare pads | **IO10 = VDRV_EXT** (R57 100k pull-down), **IO11 = XFMR_ID** (strap R59); TP7 / TP8 stay on them | switch and board type |
| 9 | EN_B on IO12 | unchanged; it now stops the boost **and** the doubler | enable pin |

Unchanged: receiver, ADC, 3.3 V supply, USB, LoRa, connectors, mounting holes, transformer stage values.

## Voltages (calculated, nothing measured)

TUSS4470 data sheet (6.1, 6.3): VPWR recommended 5-36 V, absolute maximum 40 V. VDRV absolute maximum = VPWR + 0.3 V, and
"always VPWR > VDRV + 0.3 V". So the order is fixed: **VPWR is the higher rail, VDRV sits under it.** Frank's request
(21 Nov) was "34.5 V VPWR, 36 V VDRV": the chip does not allow VDRV above VPWR, so this board does VPWR ~33.9 V, VDRV ~32.7 V.

| Rail | DIRECT | XFMR |
|---|---|---|
| VBOOST (MT3608, 0.6 V × (1 + Rtop / 3k)) | 17.0 V (82k) | 9.6 V (82k ∥ 100k = 45.1k) |
| VDBL = 2 × VBOOST + D1 drop − 2 Schottky drops (est.) | **33.9 V** | ~19 V |
| VPWR (TUSS4470) | 33.9 V | ~19 V |
| VDRV, IO10 low | 20 V (regulator, knob `vdrv`) | ≤ 10 V (gate drive, capped by the driver) |
| VDRV, IO10 high | **~32.7 V** (VDBL − 2 diodes) | cannot happen: R56 not fitted, and the firmware keeps IO10 low |
| VPWR ADC (R14 100k / R15 5.1k, × 20.6) | 1.64 V | 0.93 V |

Tolerance of the 33.9 V: MT3608 feedback ±2 % (0.588-0.612 V, data sheet), R4 and R5 1 % each: ±4 % worst case = 32.5 to
35.2 V, plus about ±0.3 V for the diode drops: **35.6 V at the very worst** (limit 36 V). The parts review traded the
0.1 % feedback resistors (34.5 V nominal) for Basic 1 % ones: 0.15 dB less drive, two extended-part fees less.

Standing load of the high rail at 33.9 V: R14 0.32 mA + TUSS4470 0.24 mA (data sheet) + R55 / R56 0.12 mA + R58 0.7 mA
(the last two only in 33 V mode) ≈ 47 mW, about 14 mA from a 4.2 V pack while the converter runs (est., 80 %
efficiency); zero while EN_B is low.

## Firmware (lib/SonarTuss, D210; not compiled, no board exists yet)

Build flags for this board (template `sensor_pod_v10`, commented out in platformio.ini):

| Flag | Pin | Job |
|---|---|---|
| `SONAR_PIN_SCK=7 MISO=16 MOSI=15 CS=6 IO1=17 IO2=18 VOUT=8` | - | TUSS4470 (from design.py) |
| `SONAR_PIN_BOOST=12`, `SONAR_BOOST_GATE=1` | IO12 EN_B | converter + doubler off while each burst listens, in the idle rest and in sleep (existing code, D170 / D207) |
| `SONAR_PIN_VDRV_EXT=10` | IO10 | high while the effective vmode is 2; low in vmode 0 / 1, in a rest with the converter off, in the wiring check and in sleep |
| `SONAR_PIN_XFMR_ID=11` | IO11 | read once at start (pull-up, then released). Strap = transformer board: `drive 1` whatever the knob says and IO10 locked low. No strap: `drive 0` |

The VPWR reading of this board is × 20.6 (100k / 5.1k), not × 11 as v0.9: set it in the pod build when it exists.

Switch-on sequence for 33 V: regulator on at 20 V with the 20 mA charge (8 ms) → regulator Hi-Z → IO10 high → 3 ms.
Q3 then only tops up 13 V on the 2.2 µF VDRV capacitor (a 33 V step from empty would exceed its 0.5 A pulse rating, est.).
Boards without these pins (shield builds, v0.9): both flags default to -1, nothing changes.

## Noise and interference: what the layout does

| Source | Measure | Layout figure |
|---|---|---|
| Doubler pump node DBX (17 V square wave, 1.2 MHz) | kept at the boost, hand-routed | 3.8 mm of track; 29 mm from RINP, 31 mm from INP |
| Boost switch node SW_B | hot loop unchanged from v0.9 (4.9 mm²); C50 sits on the node | 8.0 mm of copper (5.6 in v0.9), 22 mm from RINP |
| VDBL ripple into the chip | only through R7 10 Ω + 2.3 µF (corner ~7 kHz at no bias, higher at 33.9 V) into VPWR and through Q3 + D10 into the 2.2 µF VDRV cap | trunk on the east edge as the v0.9 boost trunk; 7.6 mm from VOUT_I at its end |
| Converter while listening | EN_B gates boost and doubler together during each capture (`SONAR_BOOST_GATE`) | - |
| Receiver area | nothing active added: only the passive links R52 / R53 | INP 3.2 mm, INN 4.1 mm, RINP 3.0 mm, 0 vias, full ground reference |
| Envelope to the ESP32 ADC (VOUT) | hand-routed as in v0.9 (the autorouter made it 88 mm with 6 vias) | 18.0 mm, 1 via |
| Switch control IO10, strap IO11 | static levels, no edges while pinging | - |
| ±110 V nets XA and XS | ≥ 1.0 mm to every other net and pour, all layers (DRC rule) | - |
| Coupling audit (aggressor / victim pairs within 1 mm) | no pair involves DBX, SW_B or XS; worst pair is FLT / OUTB 0.8 mm at 0.75 mm, as v0.9 | audit/noise_v10.json |

Not shown by a layout check: the doubler's ripple on VDBL, the converter's burst-mode frequency at this light load, and
the real noise floor. Those are bench measurements.

## Review round (21 Nov): what was found and what was done

| # | Finding | Severity | Done |
|---|---|---|---|
| 1 | A transformer board with the strap R59 forgotten would get 33 V on the FET gates (VGS max 20 V) as soon as the knob is vmode 2 | high | **Hardware interlock:** R56 is now a DIRECT-only part. Without it Q3 cannot turn on |
| 2 | Requested "VDRV 36 V above VPWR 34.5 V" is outside the chip's absolute maximum | high | Order kept as the data sheet requires; nominal set from the tolerance sum (see Voltages) |
| 3 | VPWR at the limit: 34.5 V nominal with 1 % feedback resistors could reach 36.2 V | medium | nominal lowered to 33.9 V (82k / 3k): worst case 35.6 V |
| 4 | Q3 gate at VDBL / 2 = 17.3 V against a 20 V limit | medium | R56 200k: VGS = VDBL / 3 = 11.5 V |
| 5 | VPWR sense: 34.5 V × 10 / 110 = 3.14 V, above the ESP32-S3 ADC range and close to the pin limit | medium | R15 5.1k: 1.64 V (1.75 V at 36 V) |
| 6 | Power-off: VDRV must fall before VPWR. With 100k the two time constants were close (0.22 s against ~0.35 s est.) | medium | R58 47k: 0.10 s. If VDRV still lags, the chip's own VDRV → VPWR path carries about 1 mA for a fraction of a second (est.): not a damage case, but outside the data sheet line |
| 7 | Secondary link R51 overlapped the transformer body by 0.5 mm | high | moved 1 mm (first pass) |
| 8 | Envelope line VOUT to the ESP32 ADC routed 88 mm with 6 vias by the autorouter | medium | hand-routed, 18 mm, 1 via (first pass) |
| 9 | 178k part number from memory was a 124 Ω part | high | every new number looked up (first pass and again for the five new values) |
| 10 | Excel BOM would have named unknown parts as the TDK transformer | low | export fixed |
| 11 | Doubler diodes: while VDBL charges from empty they carry the inductor current (up to the MT3608 limit, amps). The BAT54S is a 0.2 A part with a 0.6 A surge rating | high | **D9A + D9B B5819W** (1 A, 9 A surge, 40 V; the boost diode's part) |
| 11b | MT3608 switch pin (30 V) at the higher boost | low | sees ~17.4 V + ringing: no change |
| 12 | Boost duty cycle | low | 17.0 V needs 82 % at a 3.0 V pack (24.1 V in v0.9 needed 88 %): easier than before |

## Parts and cost (21 Nov, every BOM line looked up on jlcpcb.com)

JLC economic assembly charges **$3.07 per Extended part type** per order (JLC price page); Basic parts carry no such fee.
Part prices and stock are not on the pages I could read, so "cheapest" here means: fewest Extended lines, and Basic
parts that are already on the board wherever one does the job.

| Change | Was | Now | Saves |
|---|---|---|---|
| Doubler diodes D9 | BAT54S, Extended | 2 × B5819W, Basic, already on the BOM (also the stress fix, review row 11) | 1 fee |
| Feedback R4 / R5 | 143k 0.1 % Extended + 5.1k | 82k (C23254) + 3k (C4211), both Basic | 1 fee |
| xfmr link R54 | 158k 0.1 % Extended | 100k, Basic, already on the BOM | 1 fee (xfmr) |
| Sense divider R14 / R15 | 130k Extended + 10k | 100k + 5.1k, both already on the BOM | 1 fee |
| Receive series R8B / R8C | 1.1k 1206 Extended | 1k 1206 (C4410), Basic | 1 fee |

Extended lines per order: direct 15 → **11**, transformer 16 → **11** (+1 for the LoRa module).

| Still Extended | Why it stays |
|---|---|
| TUSS4470, TPS63020, MT3608, ADS7042 | no Basic equivalent with the same footprint |
| USB-C connector, USBLC6-2SC6 | footprint-specific; the USBLC6 (USB ESD) can be left off if USB is only used on the bench |
| L1 10 µH, L2 1.5 µH (Sunlord 4 x 4 mm) | footprint-specific |
| Q1 AOSD62666E, C49 1000 µF, C43 1.2 nF 630 V | transformer stage |
| T1 TDK | not at LCSC: Global Sourcing |
| Q3 LBSS84LT1G | listed as "Preferred" in the parts database (fee normally waived) but "Extended" on its page: count it as a possible 12th |

**Two more savings that are your call:**

- `assembly/tipup_direct_lean`, `lora_direct_lean`: a board that will stay direct, without T1, Q1, C49, C44, C43, D7,
  R44, R45. 8 Extended lines instead of 11, no Global Sourcing order, and the transformer parts saved. It can be
  converted later only by hand-soldering those parts.
- The ADS7042 (U8) is not used by the firmware today (the envelope goes to the ESP32 ADC on IO8). Deleting U8 from a
  BOM saves one more fee and the part.

## JLC placement preview check (21 Nov, Frank's screenshot of the lora_xfmr upload)

Compared part by part with the pads (pin-1 dot, lead sides, polarity marks):

| Part | Shown in the preview | Verdict | Done |
|---|---|---|---|
| Q1 (SO-8 dual FET) | leads on top / bottom, pads are left / right; pin 1 bottom-left | **wrong, 90°** | CPL rotation −90 |
| U2 (TPS63020) | body lying 4 x 3, pads need 3 x 4; pin 1 bottom-left | **wrong, 90°** | CPL rotation −90 |
| Q3, Q4, D10, D7 (SOT-23) | single lead on the two-pad side; pin 1 bottom-right | **wrong, 180°** | CPL rotation +180 |
| U8 (ADS7042) | pin 1 bottom-right, silkscreen mark top-left | **wrong, 180°** | CPL rotation +180 |
| U1 (TUSS4470), U3, U9, U5 | pin-1 dot on the pin-1 corner | correct | - |
| D1, D4, D5, D9A, D9B | cathode mark on the cathode pad | correct | - |
| C49 | + on the VTX pad | correct | - |
| D6 (LED) | + at the anode pad | looks correct; small in the picture, look again | - |
| T1 | empty | expected: Global Sourcing part, not placed until it is in your JLC parts stock. **Its rotation has never been seen in a preview** | check when it shows |
| R40, R41, R52, R56 | empty | expected on a transformer build | - |
| C43 | small checkered square | JLC has no 3-D model for it; position is right | - |

The two parts JLC did not match by itself and Frank picked by hand are the right ones: **C8598** (B5819W SL, for D1, D4,
D5, D9A, D9B) and **C4410** (1206W4F1001T5E, 1 kΩ 200 V, for R8B, R8C). Both Basic.

**USB-C connector J5:** shown too close to the board edge; Frank moved it down by hand. JLC centres this part on its
pads, the KiCad footprint origin is 1.46 mm nearer the edge: the CPL now carries that offset (`OFF` in
`src/export_jlc.py`, computed from the pads). The two BOM lines above now carry the full part number in the Comment and
Manufacturer Part # columns; why JLC did not match them from the LCSC number alone is not known.

**Antennas.** Wi-Fi: the ESP32-S3-WROOM-1**U** has its own U.FL socket (Espressif data sheet). LoRa: JLC and Digi-Key list
C2764087 with an "IPEX interface", but Frank reads the JLC photo as a pad without the socket, and Ai-Thinker sells both
kinds. The board no longer depends on it: **J6 (Hirose U.FL-R-SMT-1(80), C88374, Extended) is now fitted by JLC on every
LoRa build**, on the module's antenna pin (4 mm of trace). Plug the LoRa antenna into J6. Its rotation in the JLC
preview is unseen: the single small pad must point at the module, the two long pads sit left and right of it.

All CPL files in `assembly/` carry the corrected rotations: **upload the new CPL and look at the seven corrected parts
once more before paying.** The corrections live in `src/export_jlc.py` (ROT).

Silkscreen: the designators overlapped each other and the pads (83 overlaps). `src/silk.py` now puts each one in a
free spot next to its part and hides the ones with no room (39 hidden, 18 overlaps left, all with part outlines).
Hidden link designators: R43 (right of R42), R53. `links.png` shows where every link sits.

## Risks left / verify before ordering

| Item | Risk | Check |
|---|---|---|
| Feedback fault (R4 open, R5 short, bad joint) | the MT3608 has no over-voltage stop; the doubler would then pass 40 V and kill the TUSS4470. v0.9 had the same fault at half the voltage | first power-up of every board: bench supply limited to 100 mA, meter on TP4 **before** anything else. A clamp was looked at: a 39 V zener (37-41 V) does not protect a 40 V pin, a 36 V one conducts in normal use |
| Doubler never built | VDBL level and ripple are calculated | TP4 = 32.5-35.5 V (direct), ~19 V (xfmr). **Above 36 V: do not use the board, change R4** |
| VDRV sag in a burst | C12 2.2 µF 50 V keeps well under half its value at 33 V (X7R, est.); a long burst may sag VDRV | scope VDRV at `cycles 16`; if it sags more than 2 V fit a 10 µF 50 V 1206 (same pads, C13585) and raise `SONAR_VDRV_EXT_PRE_MS` to 15 |
| Receive input in direct drive | 330 pF into INP passes ~14 mA peak at 33 V (12.5 mA in today's 30 V build, which works) | noise floor and ring-down against the 20 V mode |
| VDRV vs VPWR at power-off | see review row 6 | scope both once |
| Strap forgotten on a transformer board | drive 0 would switch the gates at up to 20 V as a bridge. The 33 V case is now blocked by R56; this one is not | serial log must say "transformer board (strap on)" before the first ping |
| SOT-23 rotations | Q3, Q4, D10 (and D7) have no confirmed JLC rotation; D9A / D9B cathode side | JLC placement preview; pin 1 per the schematic |
| LCSC numbers | every line looked up on jlcpcb.com on 21 Nov (part and Basic / Extended label, see Parts and cost). Stock and prices are not shown on those pages: not checked | at order |
| Routing repeatability | freerouting gives a different result per run; this output is the checked one | use the files here; a value change needs only `post_v10.sh` (no new routing) |
| D1 / C50 pads | same net (SW_B), 0.10 mm apart: may solder together | harmless |
| Everything in the v0.9 README | T1 end of life, Q1 / T1 / C49 rotations, transducer Rm, saturation, receive loss through 2 kΩ, ring-down | unchanged, see hardware/sonar_pod_v0.9/README.md |

## First board, in this order

1. DIRECT links. Bench supply 4.5 V limited to 100 mA, meter on TP4: 32.5-35.5 V. Then 3V3.
2. `vmode 0`: pings as a v0.8 board.
3. `vmode 2`: VDRV ≈ 33 V (C12), echo about +4.3 dB against step 2; noise floor `nf` within 1 dB of step 2.
4. Wiring check on the page: the converter-enable line must show the pin as present (IO12).
5. Second board with the XFMR links: follow the bench order of the v0.9 README (`cycles 8` first). The serial log must
   say "transformer board (strap on)".

## Files

| File | Use |
|---|---|
| sonar_pod_v10_gerbers.zip | PCB order (4 layers, 1.6 mm) |
| assembly/<design>/*_bom_jlcpcb.csv + *_cpl_jlcpcb.csv | JLC assembly upload |
| assembly/<design>/*_bom.xlsx, *_centroid.xlsx | same, Excel |
| sonar_pod_v10.kicad_pcb / .kicad_sch / .kicad_pro, sonar_pod_v10_schematic.pdf | KiCad 7 board and label-style schematic |
| routed_f.png, routed_b.png, noise_v10.png | pictures |
| audit/drc.txt, audit/noise_v10.json | DRC report, coupling audit |
| src/ | generator (design.py = the netlist and placement; build_v10.sh) |
| TRANSFORMER_200K.md | transformer note, from v0.9 |
