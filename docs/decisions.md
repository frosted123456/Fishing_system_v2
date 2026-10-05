# Decisions log

| Date | # | Decision | Notes |
|---|---|---|---|
| 2026-10-05 | D1 | **Flag day**: every LoRa board is reflashed to v2; no v1↔v2 LoRa compatibility | Tip-up nodes (ESP-NOW) keep their v1 frames in phase 1 and need no reflash |
| 2026-10-05 | D2 | **Build system = PlatformIO**, one project, one env per role/board | Replaces Arduino IDE; shared code in `lib/`, host tests in `test/` |
| 2026-10-05 | D3 | **Arduino-ESP32 core 2.0.17** pinned via official `espressif32 @ 7.0.1` | Same API family as the v1 code; core 3.x (pioarduino) to be re-evaluated at the start of phase 2 (ESP-NOW RSSI) |
| 2026-10-05 | D4 | **Stay at SF9** (worst-case range); size the stream for SF9; fall back to SF7 only if bench tests prove SF9 impossible | See protocol_v2.md §6 |
| 2026-10-05 | D5 | **Aggregates: no ACK, no retry** (the next one supersedes) | Alerts, config, track events keep ACK + retry |
| 2026-10-05 | D6 | **The cabin does not relay** | It still originates (silence, config, ACKs) |
| 2026-10-05 | D7 | Library pins: RadioLib 7.7.1, U8g2 2.36.18, ArduinoJson 6.21.6 | ArduinoJson 6 because the code uses the v6 API (`StaticJsonDocument`); RadioLib 7.8.x was a week old, 7.7.1 is the previous stable |
