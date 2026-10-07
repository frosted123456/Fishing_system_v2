# Code map (after the D49 split, 2026-10-07)

| Where | What | Tested how |
|---|---|---|
| `lib/IceMesh/src/` | Everything that does not need Arduino: TDMA mesh, planner, relay, ESP-NOW backbone, sonar codec / processing / fake scene / knobs, Hall latch rules, FISH ON alarm rules | `make -C test all` (20 suites), `make -C test sanitize` |
| `src/lora_node/` | Heltec firmware: chalet, hub, relay, sensor_lora (one code, `NODE_ROLE`) | `tools/build_check.sh` (arduino-cli) or PlatformIO |
| `  chalet.h` | Shared declarations: NODE_* defines, includes, types, **every global** (`extern`, defined in main.cpp), prototypes | |
| `  main.cpp` | Globals, setup / loop, OLED pages (v1 screens), LoRa, ESP-NOW, node state, mesh integration, sonar hub side, serial commands | |
| `  web_server.cpp` | Routes, JSON API (`/api/...`), remote config | `tools/web_preview.py` fakes the API for the page |
| `  wifi.cpp` | Hotspot, cabin network, credentials (config.h wins once), retry back-off, scan | |
| `  ui_input.cpp` | CardKB keys, Esc menu, Settings pages, text entry, v1 menu handlers | `tools/screens_mock.cpp` renders the v2 pages |
| `  settings.cpp` | Settings, hole names, sonar knobs, bait depths in NVS | |
| `  alerts.cpp` | Alarm loops over the holes, alert history, silence (local + network), buzzer (esp_timer) | rules: `test_alarm_latch` |
| `  screens.cpp/.h` | v2 OLED pages drawn from a `ScreenModel` (no globals: the mock renders them on the PC) | `tools/screens_mock.cpp` + `screens_sheet.py` |
| `  mesh_radio.cpp/.h` | The radio task: TDMA on the SX1262, ESP-NOW backbone, demo network | lib tests + `tools/sim` |
| `  web_suite.h` | GENERATED from `web/suite.html` by `tools/gen_web.py` (PlatformIO runs it; build_check.sh too). Never edit | |
| `web/suite.html` | The chalet web page (Holes / Sonar / Radio / Settings). Open with `python3 tools/web_preview.py` | Chromium screenshots |
| `src/sensor_node/` | Tip-up firmware (ESP32-C3 reed, WROOM Hall latch + TUSS4470 sonar) | build_check.sh; driver: bucket test (`docs/SONAR_DRIVER.md`) |
| `include/messages.h` | Frames and `NodeState` shared by the two firmwares | |

Rules kept from v1: `config.h` holds the Wi-Fi credentials and is never committed; every global of the Heltec
firmware is defined in `main.cpp` and declared in `chalet.h`; a `static` in one file is invisible to the others
(the compiler says so: "was declared 'extern' and later 'static'" means remove the `static`).
