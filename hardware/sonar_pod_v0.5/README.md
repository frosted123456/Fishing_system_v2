# Sonar pod board v0.5: smaller (58 x 53 mm), transformer-drive and external-ADC options

Status: generated, not ordered, not reviewed by a human. DRC: 0 unconnected, 0 clearance (silkscreen and library warnings only).
Board: 58 x 53 mm, 4 layers (F signal / In1 GND / In2 GND / B signal), 1.6 mm, HASL lead-free.
Machine-assembled parts all on top. The LoRa module and its 3 support parts are on the bottom and hand-soldered.

## One assembly order for both variants

The LoRa parts are now all hand-soldered, so the JLC BOM/CPL is **identical** for `tipup` and `lora`.
Both file sets are kept for clarity.

| Item | v0.4 | v0.5 |
|---|---|---|
| Machine-placed parts / BOM lines | 56–60 / 30–31 | 59 / 32 |
| Hand-soldered | U4 ESP32-S3, J1–J4 | U4, J1–J4, P1 (option); lora: U5 Ra-01SH + C36, C37, R29 on the bottom |

## What changed vs v0.4, and why

| Area | v0.4 | v0.5 | Why |
|---|---|---|---|
| Size | 74 x 56 mm (4144 mm²) | **58 x 53 mm (3074 mm², −26 %)** | cheaper PCB, smaller pod |
| LoRa Ra-01SH | top side, assembled | **bottom side under the ESP32, hand-soldered** | frees about 25 x 17 mm on top |
| LoRa spare U.FL (J6) | footprint, not fitted | **removed** | use the module's IPEX |
| Transducer drive | direct only | **direct (default, R40/R41 0R)** or **transformer** via P1 + Q1/Q2 (DNP) | about +16 dB of transmit level with ~250 Vpp |
| Receive protection | — | **R8B** (1206, 0R by default) + **D7 BAV99 clamp** (DNP) | required in transformer mode |
| Sonar ADC | ESP32-S3 internal (83.3 kHz max) | same by default; **ADS7042 option** (U8, 12-bit 1 MSPS, DNP) on the TUSS SPI bus, CS = IO3 | headroom for better range resolution |
| OUT4 comparator line | IO3 | **dropped** (IO3 is now ADC_CS) | pin and space |
| Expansion header J7 | 4-pin | **dropped**; IO10/IO11 on test pads TP7/TP8 | space |
| Boost enable / button / LED | IO43 / IO12 / IO11 | **IO12 / IO9 / IO43** | routing on the smaller board |

## Option fitting

| Mode | Fit | Remove / change |
|---|---|---|
| **Direct drive (as ordered)** | — | — (L3 ~420 µH shunt tuning optional, C43 optional) |
| **Transformer drive** | Q1, Q2 (AP1002, C2995324), R42/R43 10R, R44/R45 10k, R46 0R (VTX = 24 V boost), C44 10 µF 50 V, transformer on P1, D7 BAV99 | remove R40, R41; change R8B 0R → 1k–2.2k 1206; bridge P1 pins 4–6 (grounds XB); C43 only if ≥500 V C0G |
| **External ADC** | U8 ADS7042 (C701641), R31 100R, C40 1 nF, R32 10R, C41 1 µF, C42 100 nF | none (R11/C21 stay for the internal ADC) |

P1 pinout: 1 DA (drain A), 2 DB (drain B), 3 VTX (centre tap), 4 GND, 5 XA, 6 XB.

The transformer is **not sourced**. The candidate is TDK B78416A2386A003 (230 µH, 1:1:9): check its pinout and its rating at 200 kHz before buying.

## Noise audit (`src/noise_audit.py`, same script as v0.3/v0.4)

| Check | v0.4 | v0.5 |
|---|---|---|
| INP / INN / RINP / FLT: length, vias, no-reference | short, 0, 0 % | 3.2 / 4.1 / 6.3 / 3.9 mm, 0 vias, 0 % |
| VOUT_I (TUSS → filter/ADC) | 16 mm, 0 vias | **26.8 mm, 2 vias, 11 % without reference** (branches to the ADS7042 and TP3) |
| VOUT (to the ESP32 ADC) | 7 mm | 9.5 mm, 0 vias |
| GND vias / area >8 mm from a stitch | 224 / 1 % | 141 / 2 % |
| MT3608 hot loop / switch-node copper | ~5 mm² / 5.7 mm | 4.9 mm² / 6.9 mm |
| TPS63020 FB trace | 5.7 mm | 5.7 mm |
| TUSS VPWR / VDRV / VDD capacitor distance | 2.0 / 2.2 / 1.6 mm | 2.6 / 3.8 / 1.5 mm |

Parallel runs worth knowing about:

| Victim | Aggressor | Run / gap | Assessment |
|---|---|---|---|
| VOUT_I | SDO | 11.7 mm at 0.24 mm, B | SDO is idle during the echo capture with the internal ADC. With the ADS7042 fitted it toggles while sampling → test it. |
| VOUT_I | SCLK | 6.8 mm at 0.78 mm, B | same: quiet with the internal ADC, active with the ADS7042 |
| NTC_ADC | SCLK | 2.3 mm at 0.43 mm | slow DC signal, harmless |
| FLT | VDRV | 1.5 mm at 0.25 mm | VDRV is DC during receive |

Honest net result: v0.5 is a bit noisier on paper than v0.4 because of the longer VOUT_I and the tighter VDRV cap. It is still far better than v0.3, and every receive-side net still has a solid plane under it.
This is a geometric risk check, not an EMI simulation. Bench test: noise floor with the boost on/off, Wi-Fi TX and LoRa TX.

## ESP32-S3 pin map (v0.5)

| Function | GPIO |
|---|---|
| TUSS NCS / SCLK / SDI(MOSI) / SDO(MISO) | 8 / 18 / 17 / 16 (shared with the ADS7042) |
| ADS7042 CS | 3 |
| TUSS IO1 / IO2 | 15 / 7 |
| Sonar VOUT (internal ADC) | 6 |
| VPWR / NTC / VBAT | 4 / 5 / 1 |
| HALL / BTN / LED | 2 / 9 / 43 |
| Boost EN (100k pull-down) | 12 |
| LoRa NSS–RXEN | 38–42 |
| LoRa DIO1 / TXEN / RST / BUSY | 13 / 14 / 21 / 44 |
| USB D− / D+ | 19 / 20 |
| Test pads | IO10 (TP7), IO11 (TP8) |

Module: ESP32-S3-WROOM-1U **N4 / N8 / N16 / N8R2** only. Not the octal-PSRAM R8/R16V variants: those use IO35–37, and the 1.8 V versions change IO47/48.

## Ordering

1. PCB: `sonar_pod_v05_gerbers.zip`, **4 layers**, 1.6 mm, HASL lead-free.
2. Assembly: **Economic, top side only**. JLC: `*_bom_jlcpcb.csv` + `*_cpl_jlcpcb.csv`. NextPCB/PCBWay: `.xlsx`.
   The "Not assembled" sheet lists the hand-solder and DNP parts.
3. Hand-solder: U4 (ESP32-S3-WROOM-1U-N8, C2980297), J1–J4. For lora: U5 Ra-01SH + C36, C37, R29 on the **bottom**.
   - Solder U4 first, flash it over USB-C, then do the bottom LoRa.
   - The two modules overlap through the board, so heat-gun the U4 thermal pad **before** U5 is fitted.
4. In the JLC preview, check pin 1 of U1, U2, U3 and J5, the D1/D4/D5 cathode bars and D6. Only SOT-23-6 has a rotation correction (−90°).

## Risks and open items

| # | Item | Severity |
|---|---|---|
| 1 | TUSS4470 pre-driver mode with VDRV from the **internal** regulator is not documented by TI (the datasheet describes VDRV_HI_Z=1 with external 5–15 V). The external VDRV parts were removed for space. Bench-test at ~10 V and check the AP1002 Vgs limit before relying on the transformer mode. | high (transformer mode only) |
| 2 | R8B in transformer mode sees ~125 V peak during the burst (≈6 W instantaneous into 1.2k for ~80 µs). Check the 1206 pulse rating, or use 2.2k. | medium (transformer mode) |
| 3 | Transformer part not sourced. P1 is a 2.54 mm header port. | open |
| 4 | ADS7042 throughput on the ESP32-S3 SPI is unproven, and SCLK/SDO run next to VOUT_I. | medium (option) |
| 5 | AP1002 pinout (1 G, 2 S, 3 D) taken from the LCSC listing: verify against the datasheet. | medium (option) |
| 6 | U2 TPS63020 footprint is self-drawn: compare it with TI DSJ0014A. | medium |
| 7 | LoRa module on the bottom: hand-soldered, sits under U4; the enclosure needs bottom clearance (module height from the Ra-01SH drawing, plus the IPEX cable). | low |
| 8 | No ESD protection on USB D+/D−. | low |
| 9 | Autorouted (freerouting) with hand pre-routes; not hand-polished. | — |
| 10 | No firmware for the S3 board yet. | — |

## Files

- `sonar_pod_v05.kicad_pro/.kicad_sch/.kicad_pcb`, `sonar_pod_v05_schematic.pdf` (A1, label-style), `sonar_pod_v05.step`
- `routed_f.png`, `routed_b.png`, `noise_v05.png`, `noise_v05.json`
- `sonar_pod_v05_gerbers.zip`, JLC CSVs, `.xlsx` BOM and centroid per variant
- `src/`: the full pipeline (`build.sh` = everything; `post.sh` = after routing)
