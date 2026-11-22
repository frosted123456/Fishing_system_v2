# Sonar pod board v0.7: v0.6 + ADS7042 external ADC fitted

Status: generated, not ordered, not reviewed by a human. DRC: 0 unconnected, 0 clearance (silkscreen and library warnings only).
Netlist check: every pad matches design.py (0 mismatches).
Board: 58 x 53 mm (same as v0.5), 4 layers (F signal / In1 GND / In2 GND / B signal), 1.6 mm, HASL lead-free.

## What changed vs v0.6

| Area | v0.6 | v0.7 | Why |
|---|---|---|---|
| External ADC | none | **ADS7042IDCUR (U8, C701641) fitted**, JLC-placed, with R31 100R / C40 1 nF input, R32 10R / C41 1 µF on AVDD (= reference), C42 100 nF | cleaner, more linear envelope data (12-bit, up to 1 MSPS) for averaging and E1/E2 bottom classification |
| ADC bus | — | shares SCLK (IO7) / SDO (IO16) with the TUSS4470; own CS on **IO3** | both chips tri-state SDO when not selected |
| Layout | — | ADC group right under the TUSS VOUT pin; R11/C21 and C20/R10 flipped; LED, AINP and AVDD pre-routed | short analog paths |
| Kept | internal ESP32 ADC on IO8 still wired (R11/C21) | | fallback / comparison |

## v0.6 changes vs v0.5 (kept)

| Area | v0.5 | v0.6 | Why |
|---|---|---|---|
| LoRa Ra-01SH (+ C36, C37, R29) | bottom, hand-soldered | **top, JLC-assembled** (lora variant) | one machine-assembled side for everything JLC can place |
| ESP32-S3 module | top, hand-soldered | **bottom, hand-soldered**, under the LoRa module | the only SMD part you solder yourself |
| External ADC (ADS7042 + 5 parts) | DNP option | **removed** | not worth it: the 80 µs ping limits resolution more than the 83 kHz internal ADC; it also lengthened VOUT_I |
| VOUT_I (TUSS → filter) | 26.8 mm, 2 vias | **9.9 mm, 0 vias** | ADC branch and TP3 stub gone |
| TP3 (VOUT test pad) | yes | removed | probe the envelope on R11 |
| Connector pin names | none | **on the silkscreen**: J1 BAT+/BAT−, J2 XA/XB/SH/NTC/NG, J3 3V3/GND/HALL, J4 3V3/GND/BTN/LED, P1 DA/DB/VTX/GND/XA/XB | what to solder where |
| Bottom paste layer | in the gerbers | **removed** | nothing on the bottom is machine-soldered, so no bottom stencil is needed |
| TUSS SPI / IO pin map | see v0.5 | **changed** (see below) | the bottom-mounted module is mirrored, so the bus order flips |

## Orders

| Variant | Assembled parts | BOM lines | Extended types (fee each) |
|---|---|---|---|
| tipup | 65 | 34 | 7: TUSS4470, TPS63020, MT3608, L1, L2, USB-C, ADS7042 |
| lora | 69 | 35 | 8: the same + Ra-01SH |

## Noise audit (`src/noise_audit.py`)

| Check | v0.6 | v0.7 |
|---|---|---|
| INP / INN / RINP / FLT | 3.2 / 4.1 / 6.0 / 3.9 mm, 0 vias | 3.2 / 4.1 / 6.3 / 3.9 mm, 0 vias |
| VOUT_I (TUSS → R11 and R31) | 9.9 mm, 0 vias | 15.6 mm, 0 vias, 0 % without reference |
| AINP (R31 → C40 → ADS7042) | — | 6.6 mm, 0 vias |
| VOUT (to the ESP32 ADC) | 10.2 mm, 1 via | 8.1 mm, 1 via |
| GND vias / ground >8 mm from a stitch | 154 / 0 % | 151 / 0 % |
| MT3608 loop / switch node | 4.9 mm² / 5.7 mm | 4.9 mm² / 5.7 mm |
| USB D+ / D− | 32 / 36 mm | 25 / 22 mm |

Worth knowing:

| Victim | Aggressor | Run | Assessment |
|---|---|---|---|
| AINP | SCLK | 2.0 mm at 0.42 mm | SCLK clocks the ADS7042 itself; the coupling is synchronous with the conversion (offset rather than random noise). Check on the bench. |
| VOUT_I | SCLK | 4.6 mm crossing on another layer | GND planes in between |
| NTC_ADC | VBOOST / USB | ≤8.8 mm | slow DC signal with a 100 nF filter |
| SGND | OUTA | 3.4 mm at 0.52 mm | OUTA is idle during receive |

## ESP32-S3 pin map (v0.7): firmware must be updated

| Function | GPIO | v0.5 |
|---|---|---|
| TUSS NCS | **6** | 8 |
| TUSS SCLK | **7** | 18 |
| TUSS SDI (MOSI) | **15** | 17 |
| TUSS SDO (MISO) | 16 | 16 |
| TUSS IO1 / IO2 | **17 / 18** | 15 / 7 |
| Sonar VOUT (ADC1_CH7) | **8** | 6 |
| VPWR / NTC / VBAT | 4 / 5 / 1 | same |
| HALL / BTN / LED | 2 / 9 / 43 | same |
| Boost EN | 12 | same |
| LoRa NSS / MOSI / MISO / SCK / RXEN / BUSY | 38 / 39 / 40 / 41 / 42 / 44 | same |
| LoRa DIO1 / TXEN / RST | 13 / 14 / 21 | same |
| USB D− / D+ | 19 / 20 | same |
| ADS7042 CS | **3** | — |
| ADS7042 SCLK / SDO | 7 / 16 (shared with the TUSS) | — |
| Free | IO10 / IO11 on TP7 / TP8 | |

Module: ESP32-S3-WROOM-1U **N4 / N8 / N16 / N8R2**, not the octal-PSRAM variants.

## Ordering

1. PCB: `sonar_pod_v07_gerbers.zip`, 4 layers, 1.6 mm, HASL lead-free, **via covering: tented**. Vias sit under both modules, so they must be covered by solder mask.
2. Assembly: Economic, **top side only**. Upload the BOM + CPL of the variant (`tipup` or `lora`).
3. In the JLC preview, check:
   - **U5 Ra-01SH orientation**: pin 1 (antenna pad) bottom-right, U.FL/IPEX side as on the module drawing
   - pin 1 of U1, U2 and U3
   - the cathode bars of D1, D4, D5 and D6
4. Hand-solder:
   1. **U4 ESP32-S3** on the bottom: edge pads with an iron. For the centre GND pad, use hot air from the bottom or skip it: pins 1/40 are also GND. Hot air on that pad also heats the LoRa module just above it on the top side.
   2. Flash over USB-C.
   3. Fit J1–J4.
5. **Cheaper way to get both variants:** order **10 tipup boards** in one order, then hand-solder a Ra-01SH (castellated 2 mm pads, easy with an iron) + C36/C37/R29 on the boards that need LoRa. A second order costs the fixed fees again (about C$52 for PCB + setup + stencil + extended fees).

## Option fitting (unchanged from v0.5, minus the ADC)

| Mode | Fit | Remove / change |
|---|---|---|
| Direct drive (as ordered) | — | — (L3, C43 optional) |
| Transformer drive | Q1, Q2 (AP1002), R42/R43 10R, R44/R45 10k, R46 0R, C44 10 µF 50 V (designator hidden: the 1206 pad pair right of P1), transformer on P1, D7 BAV99 | remove R40, R41; R8B 0R → 1k–2.2k 1206; bridge P1 pins 4–6 |

## Risks and open items

| # | Item | Severity |
|---|---|---|
| 1 | Firmware pin map changed (TUSS bus and VOUT) + ADS7042 driver (CS on IO3; frame format and offset calibration per the TI datasheet). Throughput over SPI on the S3 is not measured yet. | must do |
| 2 | Transformer mode: TUSS4470 pre-driver mode with VDRV from the internal regulator is undocumented. R8B pulse power ~6 W for ~80 µs. Transformer not sourced. | high (option only) |
| 3 | Vias under both modules: they must be tented. | medium |
| 4 | U5 orientation in the JLC preview not yet confirmed. Only the SOT-23-6 rotation correction is applied. | medium |
| 5 | U2 TPS63020 footprint self-drawn; AP1002 pinout from LCSC. | medium |
| 6 | No ESD on USB; autorouted, not hand-polished. | low |

## Files

- `sonar_pod_v07.kicad_pro/.kicad_sch/.kicad_pcb`, `sonar_pod_v07_schematic.pdf`, `sonar_pod_v07.step`
- `routed_f.png`, `routed_b.png` (bottom, mirrored view), `audit/noise_v07.png`, `audit/noise_v07.json`
- `sonar_pod_v07_gerbers.zip`, JLC CSVs, `.xlsx` BOM and centroid per variant
- `src/`: full pipeline (`build.sh`, `post.sh`, `v07.ses`)
