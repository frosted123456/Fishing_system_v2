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
