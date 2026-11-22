# Sonar pod board v0.3 — ESP32 + USB on board, optional LoRa (generated, NOT ordered, NOT human-reviewed)

One PCB (66 x 50 mm, 2 layers, all SMD on top), two parts lists:

| Variant | BOM / CPL | What gets assembled | Use |
|---|---|---|---|
| **tipup** | `sonar_pod_v03_tipup_*` | ESP32 + USB + sonar + power (68 parts) | tip-up: ESP-NOW to the hub |
| **lora** | `sonar_pod_v03_lora_*` | everything above + Ra-01SH LoRa + U.FL (73 parts) | hub / relay with its own sonar |

Same gerbers for both (`sonar_pod_v03_gerbers.zip`). Each variant is its own JLC order (one BOM per order).

## What's on it
| Block | Parts | Notes |
|---|---|---|
| ESP32 | ESP32-WROOM-32UE-N4 (C701344) | classic ESP32 = the chip the firmware already runs on. **U.FL on the module** -> pigtail to the SMA in the head (WiFi/ESP-NOW antenna stays 20 cm up). Status LED on GPIO2 |
| USB | USB-C (HRO TYPE-C-31-M-12), CH340C, 2 x S8050 auto-reset | plug and flash from PlatformIO like a DevKit. CH340 is powered **from USB only** (own XC6206 3.3 V LDO): zero battery drain when unplugged |
| Power | battery and USB each through a B5819W into VB | USB powers the board on the bench; USB can never charge the L91 primaries |
| 3.3 V | TPS63020 (as v0.2.1) | 3 A: ESP32 Wi-Fi peaks + LoRa TX |
| Sonar | TUSS4470 + MT3608 24 V (as v0.2.1) | MT3608 EN on GPIO33 now: boost **off in deep sleep** (firmware done, env `sensor_pod_v03`) |
| LoRa (variant) | Ai-Thinker Ra-01SH SX1262 915 MHz (C2764087) + U.FL (C88374) | shares SPI with the TUSS4470 (separate CS). U.FL -> pigtail to a 2nd SMA |
| Connectors (hand-solder) | J1 battery XH2 (right edge), J2 transducer XH5 (right edge), J3 Hall XH3, J4 head PH4 (3V3, GND, button, LED) | head harness reduced from 16 wires to 4 + coax |

## ESP32 pin map (keeps the existing WROOM firmware)
SPI 18/19/23 (shared) · TUSS CS 5 · IO1 26 · IO2 25 · VOUT 36 · OUT4 32 · BOOST_EN 33 · VBAT 34 (/2) · VPWR 35 (x10/110) · NTC 39 ·
Hall 27 · LED 2 · head button 15 · LoRa NSS 4, DIO1 16, RXEN 17, RST 21, BUSY 22, TXEN 13 · UART 1/3 to the CH340 · EN/IO0 auto-reset.

## Cost-cutting applied
- Every passive, the diodes, transistors, LED and LDO are **JLC Basic** (no fee). Extended parts: 8 (tipup) / 10 (lora), all ICs/modules/inductors/USB-C with no Basic equivalent.
- No 0.2 mm vias: the ESP32 footprint's thermal vias and the TUSS4470's were removed (JLC small-drill surcharge).
- No ENIG needed (finest pitch 0.5 mm, smallest part 0603): order **Economic PCBA + HASL lead-free**.
- THT JSTs left out of the assembly (hand-soldered, no THT fee).
- USB-UART only powered from USB (no extra sleep current, so no power switch needed).

Rough cost (estimate, the JLC quote decides): tipup x 10 about **+60-80 CAD over v0.2.1** (ESP32 module ~$3.1 each, CH340C $0.40, USB-C $0.17, 3 more extended fees, bigger PCB) -> **~190-210 CAD for 10 incl. shipping**, i.e. ~20 CAD per finished tip-up board. LoRa x 5 as a separate order: ~120-150 CAD.

## Check in JLC's preview (rotations I could not confirm)
U1 TUSS4470, U2 TPS63020, Q2/Q3, U7 (SOT-23) and U4/U5/U6 pin-1: correct them in the preview and send me the angles,
I'll bake them into `export_jlc.py`. U3 (SOT-23-6) uses -90 (looked right in v0.2.1).

## Still open / honest risks
- U2 TPS63020 footprint is self-drawn (same as v0.2.1).
- Ra-01SH placed on KiCad's **Ra-01** footprint (same SMD16 17x16 package, same pin numbering; Ra-01SH pin functions differ and are wired as Ra-01SH). Not verified against Ai-Thinker's drawing: compare before ordering the lora variant.
- Ra-01SH: TCXO and RF-switch details not confirmed from the datasheet (RXEN/TXEN are wired to GPIO17/13; RadioLib `setRfSwitchPins`).
- **Firmware for the lora variant does not exist yet**: the LoRa roles run on the Heltec V3 (ESP32-S3); they need a port to the classic ESP32 + Ra-01SH (pins, no OLED). The tipup variant runs today's tip-up firmware (`pio run -e sensor_pod_v03`).
- WiFi antenna: if you'd rather have a built-in antenna, the ESP32-WROOM-32E (C701341, PCB antenna, $0.30 less) uses the same pads but its antenna would hang over the board's top edge and sit in the box at ice level -> shorter range. The U.FL + coax keeps the antenna 20 cm up. Not recommended.
- Autorouted (freerouting), DRC clean on copper (0 unconnected, 0 clearance); silkscreen text overlaps are cosmetic.
- Antenna pigtails (U.FL -> SMA bulkhead, ~15 cm) are not on the board BOM: buy 2 per lora board, 1 per tipup.
