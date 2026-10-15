# Ice fishing tip-up mesh with per-hole sonar

Firmware for a network of ice-fishing tip-ups (brimbales) around a chalet: each hole reports a bite over
ESP-NOW to a hub on the ice, hubs talk LoRa (TDMA mesh, SF9/500 kHz) to the chalet box, and the chalet shows
everything on its OLED and on a phone web page over its own hotspot. Holes fitted with a sonar (TUSS4470 +
200 kHz transducer) stream depth, bottom type and fish to the chalet.

## Boards

| Board | Role | Firmware |
|---|---|---|
| Heltec WiFi LoRa 32 V3 | Chalet (master, web page, buzzer), hub, relay, LoRa tip-up | `src/lora_node/` |
| ESP32 WROOM | Tip-up: Hall latch on the spool, optional sonar shield | `src/sensor_node/` |
| ESP32-C3 Super Mini | Tip-up with a reed switch (older head) | `src/sensor_node/` |

## Winter 2026 sonar tip-up: wiring (ESP32 DevKit V1 30-pin + Open Echo shield 002, hand-wired)

Build env: `sensor_wroom_sonar` (or `sensor_wroom_sonar_bench` for the bucket test). Pins = `src/sensor_node/sonar_tuss4470.h`.
Bench build on a phone: join the Wi-Fi **IceFish-bench-<node id>**, open **http://192.168.4.1** (live values, A-scan,
knobs, notes, RAW captures, record and download the test file).

| # | ESP32 DevKit V1 | Shield 002 header | Signal |
|---|---|---|---|
| 1 | **3V3** (TPS63020 3.3 V) | **5V** pin | TUSS4470 VDD. **Never 5 V**: the shield's "5V" pin is the TUSS4470 logic supply; at 5 V its SDO and VOUT would exceed the ESP32's 3.6 V limit and the ESP32's 3.3 V signals would be below its VIH (0.7 x VDD). The shield's "3.3V" pin is not connected to the TUSS4470 |
| 2 | GND | GND | common ground (pack, TPS63020, MT3608, ESP32, shield) |
| 3 | D18 | D13 | SCLK |
| 4 | D19 | D12 | SDO -> MISO |
| 5 | D23 | D11 | MOSI -> SDI |
| 6 | D5 | D10 | NCS |
| 7 | D26 | D8 | IO1 (held high) |
| 8 | D25 | D9 | IO2 (burst) |
| 9 | VP (GPIO36) | A0 | VOUT echo envelope (twist with its own GND wire) |
| 10 | - | **J4 middle pin (VCC)**, no jumper on J4 | MT3608 output + (MT3608 GND to shield GND) |
| - | not wired | D2, D3, RESET, AREF, 3.3V, VIN | OUT_4 / OUT_3 comparators unused (`SONAR_PIN_O4 = -1`) |

Also on the ESP32: Hall latch -> GPIO27, battery divider (2:1) -> GPIO34, LED GPIO2.

Shield set-up:
- Frequency jumpers: **JP3 + JP4 (200 kHz) only**; JP1/JP2 (40 kHz) and JP5/JP6 (custom) open.
- Transducer header: cable shield / ground -> **top** pin, signal -> bottom pin (Open Echo README).
- Do not feed the MT3608 into the shield VIN pin or the XT30: their indicator LEDs (R4 1 k, R5 2.2 k, 0603) would
  dissipate 0.16-0.36 W at 21 V (rated 0.1 W) and waste 10-20 mA. J4's middle pin bypasses both.
  Optional: remove R3 (power LED, ~4 mA on 3.3 V).
- MT3608 at **21.5 V** for the default `vdrv` 20: the shield's 1N4001 drops ~0.7-0.9 V and the TUSS4470 needs
  VPWR > VDRV + 0.3 V. Rule: `vdrv` = MT3608 setting - 1.5 V. Open Echo: keep the MT3608 at or below ~22 V.

## Build and flash

PlatformIO (VS Code extension or `pio` on the command line). Open `tools/flash.html` in a browser: pick the
board and what it will do, it gives the one command to run, e.g.

```
python tools/flash.py chalet --id 100 --name "Chalet" --port COM7 --monitor
```

Each board you flash is recorded in `devices.ini`. The cabin Wi-Fi of the chalet goes in
`src/lora_node/secrets.h` (copy `secrets.example.h`; the file is not in git). Without it the chalet
runs on its own hotspot.

## Layout

```
platformio.ini      build configurations (one per role) + devices.ini (your boards)
src/lora_node/      Heltec firmware: chalet / hub / relay (chalet.h, main.cpp, web_server.cpp, wifi.cpp,
                    ui_input.cpp, settings.cpp, alerts.cpp, screens.cpp, mesh_radio.cpp)
src/sensor_node/    tip-up firmware (Hall latch, TUSS4470 sonar driver)
include/messages.h  frames shared by both firmwares
lib/IceMesh/        mesh, sonar codec and processing, alarm and latch rules (plain C++)
web/suite.html      the chalet web page (embedded at build time by tools/gen_web.py)
tools/              flash.html / flash.py, gen_web.py, build_check.sh, web_preview.py
```

The design notes, host tests and development tools are kept outside this repository.
