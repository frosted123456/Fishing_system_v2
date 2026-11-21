# Sonar pod board v0.4: ESP32-S3, 4-layer, layout built against the v0.3 noise audit

Status: generated, not ordered, not reviewed by a human. DRC: 0 unconnected, 0 clearance (silkscreen warnings only).
Board: 74 x 56 mm, 4 layers, 1.6 mm, HASL lead-free, all SMD parts on top.

One PCB with two parts lists (each is a separate JLC order):

| Variant | Files | Placed parts | BOM lines |
|---|---|---|---|
| tipup | `sonar_pod_v04_tipup_*` | 56 (+U4 by hand) | 30 |
| lora | `sonar_pod_v04_lora_*` | 60 (+U4 by hand) | 31 |

## What changed vs v0.3, and why

| Area | v0.3 | v0.4 | Reason |
|---|---|---|---|
| MCU | ESP32-WROOM-32U | **ESP32-S3-WROOM-1U** (U.FL), placed rotated 180° | SIMD (vector) instructions for sonar DSP, native USB, more GPIO |
| USB | CH340C + LDO + 2 transistors + 9 passives | **native USB** (GPIO19/20) | 11 fewer parts, one fewer extended-part fee (CH340C) |
| Layers | 2 | **4**: F signals / In1 GND / In2 GND / B signals | every trace has a solid ground plane right under it (fixes v0.3 audit #1 and #2) |
| Ground | 5 GND vias | **224 GND vias**: one per GND pad + 3 mm stitching grid | short return paths everywhere |
| SPI | TUSS and LoRa share one bus | **separate buses** | no LoRa SPI traffic near the sonar lines; fixes the HALL/SCLK coupling |
| Sonar output | 63 mm + 55 mm, 8 vias, filter at the TUSS end | **16 mm + 7 mm, 0 vias**, 1k + 3.3 nF filter at the ADC pin | anti-alias for the S3 ADC (83.3 kHz max) |
| MT3608 boost | switch node 21.8 mm, output cap 15 mm away | **switch node 5.7 mm**, loop area ~5 mm² | compact hot loop |
| TPS63020 feedback | 16.5 mm under L2 | **5.7 mm**, 0 vias | quiet 3.3 V rail |
| Bottom layer | crossed by a diagonal bus | B.Cu track keepouts under the receiver and both switchers | |
| SGND | F.Cu island along the XB trace | **B.Cu zone** under the receiver, In2 GND above it, FB1 single-point tie | drive burst no longer runs beside the receiver ground |
| MT3608 EN | 100k pull-up to VB (back-feeds a 3.3 V GPIO) | **100k pull-down** | boost off unless the ESP32 drives EN |
| Extras | | 4-pin expansion header J7 (IO9/IO10, I2C-ready, not fitted); test pads for VOUT, VBOOST, 3V3, GND and BOOT | room for future sensors; bench probing |
| LoRa antenna | J6 U.FL removed | **J6 U.FL footprint, not fitted**, on the ANT pad | insurance in case a Ra-01SH arrives without its IPEX connector |

## Noise audit (`src/noise_audit.py`, same script as v0.3)

| Check | v0.3 | v0.4 |
|---|---|---|
| INP / INN / FLT without a reference plane | 100 % | 0 % |
| VOUT path length / vias | 118 mm / 8 | 23 mm / 0 |
| ADC lines without a reference plane | 43–75 % | 0–8 % |
| GND vias | 5 | 224 |
| Ground area more than 8 mm from a stitch | 44 % | 1 % |
| MT3608 switch-node copper | 21.8 mm | 5.7 mm |
| TPS63020 FB trace | 16.5 mm, under L2 | 5.7 mm |
| TUSS VPWR / VDRV / VDD capacitor distance | 4.1 / 3.4 / 2.0 mm | 2.0 / 2.2 / 1.6 mm |
| Worst parallel run against a noisy net | XB along SGND: 21 mm at 0.24 mm | VPWR_ADC along VBOOST: 12.6 mm (it is the divider of that same rail, so harmless) |

This is a geometric risk check, not an EMI simulation. Real numbers come from the bench test (see v0.3 notes): measure the sonar noise floor with the boost on/off, Wi-Fi TX and LoRa TX.

## ESP32-S3 pin map (firmware port: env still to create)

| Function | GPIO | Notes |
|---|---|---|
| TUSS NCS / SCLK / MOSI / MISO | 8 / 18 / 17 / 16 | own SPI bus |
| TUSS IO1 / IO2 (burst, RMT) | 15 / 7 | |
| Sonar VOUT | 6 | ADC1_CH5, continuous ADC |
| OUT4 (comparator capture) | 3 | strap pin, only matters with a JTAG eFuse: safe |
| NTC / VPWR / VBAT | 5 / 4 / 1 | ADC1_CH4 / CH3 / CH0 |
| HALL / head button | 2 / 12 | both RTC pins: deep-sleep wake |
| LED / boost enable | 11 / 43 | IO43 = U0TXD: boot log only pulses the boost |
| LoRa NSS / MOSI / MISO / SCK | 38 / 39 / 40 / 41 | own SPI bus |
| LoRa RXEN / BUSY / RST / TXEN / DIO1 | 42 / 44 / 21 / 14 / 13 | checked against the Ra-01SH spec v1.1 pin table |
| Expansion SDA / SCL | 9 / 10 | J7, not fitted |
| USB D- / D+ | 19 / 20 | native USB-Serial-JTAG: flash and log over USB-C |
| Not used | 35, 36, 37, 45, 46, 47, 48 | PSRAM pins and 1.8 V pins on the N16R16VA variant; 45/46 are strap pins |

## Firmware port: what changes

- The I2S built-in ADC mode does not exist on the S3. Use the `adc_continuous` driver; its maximum is 83.3 kHz (ESP-IDF `soc_caps.h`). That is about 3 samples per 2.5 cm bin; the 16-cycle ping is about 7 samples long.
- RMT and MCPWM capture both exist on the S3. LoRa uses RadioLib with the pins above and `setRfSwitchPins(RXEN, TXEN)`.
- The S3 needs Arduino-ESP32 2.0.17 or later (already the project's version).

## Ordering

1. PCB: `sonar_pod_v04_gerbers.zip`, **4 layers**, 1.6 mm, HASL lead-free. The 4-layer surcharge at this size is my estimate: check the quote.
2. Assembly: **Economic**, top side. Upload the BOM and CPL of the variant you want (CSV for JLC, `.xlsx` for NextPCB/PCBWay).
3. U4 (ESP32-S3 module) is **not in the JLC BOM**: every ESP32-S3-WROOM-1U you can buy at JLC is "Standard Only", and the one
   Economic listing (N16R16VA, C50033030) is not available. Solder an **ESP32-S3-WROOM-1U-N8 (LCSC C2980297, ~US$4.85)** yourself:
   edge pads at 1.27 mm with an iron; the bottom GND/thermal pad wants hot air or a hot plate (pins 1 and 40 are GND too, so an
   unsoldered pad still works electrically, but solder it if you can). Any WROOM-1U variant fits.
   Flash it over the USB-C right after soldering, before closing the box.
4. In the placement preview, check pin 1 of U1, U2 and U3, the cathode bars of D1/D4/D5, and the U5 orientation. The only rotation correction applied is SOT-23-6 −90°.

## Still open / honest risks

- U2 TPS63020 footprint is self-drawn (as in v0.2.1/v0.3): compare it with the TI DSJ0014A drawing.
- Ra-01SH: LCSC lists an "IPEX interface", but the Ai-Thinker spec only names the ANT pad. J6 (not fitted) covers the case where there is no IPEX.
- The ANT trace to J6 is 6.7 mm long, widened to about 50 Ω. With J6 not fitted it is a short open stub, which should have a negligible effect at 915 MHz.
- No ESD protection on USB D+/D- (USB is only used on the bench).
- Autorouted (freerouting) with placement and keepouts chosen for noise; not hand-polished.
- The firmware for the S3 board does not exist yet.

## Files

- `sonar_pod_v04.kicad_pro/.kicad_sch/.kicad_pcb`, `sonar_pod_v04_schematic.pdf`, `sonar_pod_v04.step`
- `routed_f.png`, `routed_b.png`, `noise_v04.png`
- `sonar_pod_v04_gerbers.zip`
- JLC CSVs (BOM + CPL per variant), `.xlsx` BOM and centroid per variant
- `src/` holds the whole pipeline: design.py, make_pcb.py, pack_board.py, fanout.py, dsn.py, import_ses.py, widen.py, stitch.py, noise_audit.py, export_jlc.py, export_xlsx.py, make_sch.py and `build.sh`.
