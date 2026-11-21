# Sonar pod board v0.8: v0.7 after the design review (2026-10-15)

Status: generated, not ordered. Three review passes on v0.7 (every pin map against the datasheets, the TUSS4470 passives
against TI's formulas, the routed layout): the schematic was right, the autorouted analog layout and a few parts were
not. v0.8 fixes all of it. DRC: 0 unconnected, 0 clearance, 0 shorts (silkscreen, library and four 0.1 mm courtyard warnings
only). Netlist check: every pad matches design.py.
Board: 58 x 53 mm (unchanged), 4 layers (F signal / In1 GND / In2 GND / B signal), 1.6 mm, HASL lead-free.

## What changed vs v0.7, and why

| # | v0.7 finding | v0.8 |
|---|---|---|
| 1 | **Receiver reference (SGND) was a 40 mm loop**: the router sent the return of the band-pass caps C16/C17 out to the transducer shield pin and back on the bottom layer | SGND dropped. U1 pin 13, C16 and C17 return to the GND planes through their own vias right at the pads (TI: the ferrite split is "best practice, not required"). The cable shield (J2.3, net SH) still goes to GND through the ferrite FB1 |
| 2 | **XA, the receive line, detoured 43 mm** through the DNP transformer header, partly on the bottom, 4 mm from the 24 V trace | XA and XB are straight 0.4 mm lines from J2 to the 0R links and the input network (`preroute_v08.py`); the tuning parts C43/L3 sit on the lines; the transformer-port stubs hang off the ends |
| 3 | **ADS7042 CS floated** at reset and in deep sleep (IO3 is a strapping pin): the ADC could drive the SPI bus it shares with the TUSS4470 | R33 10k pull-up. R34 10k pull-up on the TUSS4470 NCS as well |
| 4 | **MT3608 inductor** 22 µH / 1.4 A saturation / 0.5 Ω against a 4 A switch limit and no soft-start | Sunlord SWPA4030S100MT 10 µH, Isat 2.4 A, 0.13 Ω (same 4x4 footprint, LCSC C38117) |
| 5 | 40 Vpp drive stubs (to the DNP FET gate resistors) ran between the pads of the ADC reference resistor and 1.8 mm from the ADC input | Transformer parts moved east/south (Q1/Q2, R42–R45, P1, C44, R46); the OUTA/OUTB stubs run on B.Cu, nowhere near U8 |
| 6 | 24 V output caps: 2 x 10 µF X5R at 24 V bias ~ 7 µF effective (22 µF recommended) | 4 x 10 µF (C46, C47 added; Basic part, no new extended type) |
| 7 | VPWR had no bulk cap behind the 10 Ω (TI Table 8-1 CPWR2) | C48 2.2 µF 50 V |
| 8 | ADS7042 DVDD cap 100 nF below the 220 nF minimum when AVDD/DVDD share a source | C42 1 µF |
| 9 | 24 V trace crossed the receiver/ADC quadrant | VBOOST is a 0.5 mm trunk down the east edge (y 8 → 40); R7/C11 moved to the VPWR corner of U1, R14 (VPWR sense) up by the trunk |
| 10 | No pull-downs on TUSS IO1/IO2 (a floating ESP32 pin at boot could start a burst) | R35/R36 100k |
| 11 | VOUT_I left the QFN bundled with IO1/IO2/SDO; VDRV cap 3.8 mm from the pin, CPWR1 without its own via | VOUT_I turns south after 3 mm (R11 moved next to R31); VDRV straight 0.4 mm trace; every GND pad fanned out |
| 12 | No USB ESD | U9 USBLC6-2SC6 at the connector |
| 13 | All three mounting holes on the right edge | H2 moved to the bottom-left, J3 shifted right |
| 14 | LoRa antenna only through the module's own IPEX socket; a pad-only Ra-01SH would have had no antenna | Module ANT castellation (pin 1) wired to a spare U.FL **J6** (DNP, hand-fit, 4 mm 0.3 mm trace). Fit J6 only with a pad-only module; the BOM part C2764087 is the IPEX version and needs nothing |

Kept on purpose (cost): TPS63020 divider 100k/18k (28 µA = 0.7 mAh/day, nothing next to the Hall sensor's mA); C40
X7R (a log envelope does not care); X5R 10 µF 50 V (Basic) instead of an Extended X7R. The TUSS4470 VDD still comes
straight from the 3.3 V switcher: if the noise floor on the bench is high, a 10 Ω + 1 µF π-filter into pin 10 is the
first thing to try (no pads for it: cut the 3V3 trace at C13).

## Review findings carried as notes (no board change)

| Item | Note |
|---|---|
| LoRa antenna | The module's IPEX socket sits at its pin-1 corner (by TP7/TP8). Its pigtail lies over C36/C37; keep it away from the ESP32's U.FL on the other side |
| LoRa TXEN/RXEN | The Ra-01SH switches its RF path from the SX1262 DIO2 (RadioLib default). Either leave IO14/IO42 as inputs or disable the DIO2 switch in firmware; never drive both. Never fit the Ra-01SH-**P** on this footprint (its pin 5 is a 5 V PA supply) |
| VOUT_SCALE_SEL | Must be 0 at VDD 3.3 V (scale 1 = 5 V range) |
| VDRV | Regulator level 0xF = 20 V typ / 21.4 V max, needs VPWR ≥ 20.3 V (24.1 V here, 23.9 V after R7). VDRV_HI_Z must be cleared before the regulator charges C12 |
| TUSS exposed pad | Tied to GND (standard TI practice; the datasheet text does not state it) |
| BTN (IO9) | No external pull-up: enable the RTC pull-up before deep sleep |
| ESP32 centre pad | Hot air from the bottom reflows the LoRa castellations 1.6 mm above it: skip the pad (pins 1 and 40 are GND) |

## Noise audit (`audit/noise_v08.json`)

| Check | v0.7 | v0.8 |
|---|---|---|
| INP / INN / RINP / FLT | 3.2 / 4.1 / 6.3 / 3.9 mm, 0 vias | 3.2 / 4.1 / 6.3 / 3.3 mm, 0 vias |
| Receiver reference | SGND 40 mm, 2 vias, via J2 | GND planes, via at each pad |
| XA (J2 → R8B) | 43 mm, 14 mm on B.Cu | 21 mm straight F.Cu (+ 11 mm B.Cu stub to the DNP port) |
| VOUT_I | 15.6 mm, bundled with IO1/IO2 | 14.9 mm, turns away after 3 mm |
| AVDD | 28 mm loop, 2 vias | 8.5 mm, 0 vias |
| VDRV cap | 3.8 mm | 5.0 mm straight (clearance to C17 cost the extra mm; ~4 nH, irrelevant for a 25 mA average burst draw) |
| OUTA / OUTB stubs | F.Cu through the ADC cluster | B.Cu, 2 vias each, ≥ 3 mm from U8 |
| GND vias / ground > 8 mm from a stitch | 151 / 0 % | 138 / 0 % |
| MT3608 loop / switch node | 4.9 mm² / 5.7 mm | 4.9 mm² / 5.7 mm |
| USB D+ / D− | 25 / 22 mm | 49 / 43 mm (the ESD array sits east of the connector; fine at 12 Mbit/s) |

Worth knowing: NTC_ADC runs 11 mm beside SCLK on B.Cu (slow DC net with 100 nF at the pin), FLT and the OUTB stub share
0.8 mm leaving the QFN (adjacent pins; OUTB is idle during receive).

## Orders

| Variant | Assembled parts | BOM lines | Extended types (fee each) |
|---|---|---|---|
| tipup | 73 | 35 | 8: TUSS4470, TPS63020, MT3608, L1, L2, USB-C, ADS7042, USBLC6 |
| lora | 77 | 36 | 9: the same + Ra-01SH |

## ESP32-S3 pin map (unchanged from v0.7)

| Function | GPIO |
|---|---|
| TUSS NCS / SCLK / SDI / SDO | 6 / 7 / 15 / 16 |
| TUSS IO1 / IO2 | 17 / 18 |
| Sonar VOUT (ADC1_CH7) | 8 |
| ADS7042 CS (SCLK/SDO shared) | 3 |
| VPWR / NTC / VBAT | 4 / 5 / 1 |
| HALL / BTN / LED | 2 / 9 / 43 |
| Boost EN | 12 |
| LoRa NSS / MOSI / MISO / SCK / RXEN / BUSY | 38 / 39 / 40 / 41 / 42 / 44 |
| LoRa DIO1 / TXEN / RST | 13 / 14 / 21 |
| USB D− / D+ | 19 / 20 |
| Free | IO10 / IO11 on TP7 / TP8 |

Module: ESP32-S3-WROOM-1U N4 / N8 / N16 / N8R2 (the octal-PSRAM R8 variants would also fit: IO35–37 are unused).

## Ordering

1. PCB: `sonar_pod_v08_gerbers.zip`, 4 layers, 1.6 mm, HASL lead-free, **via covering: tented** (vias under both modules).
2. Assembly: Economic, **top side only**. Upload the BOM + CPL of the variant (`tipup` or `lora`).
3. In the JLC preview, check: U5 Ra-01SH orientation (antenna pad bottom-right), pin 1 of U1 / U2 / U3 / U8 / U9, the
   cathode bars of D1, D4, D5, D6.
4. Hand-solder U4 ESP32-S3 on the bottom (edge pads with an iron, skip the centre pad), flash over USB-C, fit J1–J4.
5. Both variants from one order: 10 tipup boards, then hand-solder a Ra-01SH + C36/C37/R29 where LoRa is needed.

## Option fitting

| Mode | Fit | Remove / change |
|---|---|---|
| Direct drive (as ordered) | — | — (L3, C43 optional) |
| Transformer drive | Q1, Q2 (AP1002), R42/R43 10R, R44/R45 10k, R46 0R, C44 10 µF 50 V, transformer on P1, D7 BAV99 | remove R40, R41; R8B 0R → 1k–2.2k 1206; bridge P1 pins 4–6 |

## Risks and open items

| # | Item | Severity |
|---|---|---|
| 1 | Firmware for the S3 pod does not exist yet (pin map above; ADS7042 driver; DIO2 RF switch decision) | must do |
| 2 | Transformer mode: pre-driver mode with VDRV from the internal regulator undocumented; transformer not sourced | high (option only) |
| 3 | Vias under both modules must be tented | medium |
| 4 | U2 TPS63020 footprint self-drawn; AP1002 pinout from LCSC | medium |
| 5 | TUSS VDD from a switcher (see above) | low, bench will tell |
| 6 | Autorouted digital nets, not hand-polished (analog nets are hand-routed) | low |

## Files

- `sonar_pod_v08.kicad_pro/.kicad_sch/.kicad_pcb`, `sonar_pod_v08_schematic.pdf`, `sonar_pod_v08.step`
- `routed_f.png`, `routed_b.png` (bottom, mirrored view), `audit/noise_v08.png`, `audit/noise_v08.json`
- `sonar_pod_v08_gerbers.zip`, JLC CSVs, `.xlsx` BOM and centroid per variant
- `src/`: full pipeline (`build_v08.sh`: place → `preroute_v08.py` hand routes (receiver, XA/XB, VPWR/VDRV/FLT, VOUT_I, 24 V trunk, MT3608 switch node, LoRa ANT) → GND fan-out → freerouting for the rest → import → fixes → widen → stitch → DRC → audit; `v08.ses`)
