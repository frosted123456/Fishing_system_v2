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
| 2026-10-05 | D8 | **Radio access = TDMA superframe** (1 s, chalet beacon, slots), no CSMA; **500 kHz only** (SF9/SF8/SF7) — replaces D4's SF9/125 kHz | 125 kHz single channel would have to hop under RSS-247; v1 did not (my earlier miss) |
| 2026-10-05 | D9 | **Global 8-bit node IDs**; event ack = (hub, node, seq) in the beacon, repeated 3× | Replaces the proposal's 4-bit pocket index and ack bitmap (both flawed) |
| 2026-10-05 | D10 | **Hubs ("master LoRa hubs") and chalet = Heltec V3**; tip-up nodes = ESP32 WROOM (C3 env kept for existing nodes) | |
| 2026-10-05 | D11 | No separate range-test firmware: **radio test setting in the real firmware** (rotate / fixed modes, stats on web, serial, OLED) | Values in the code marked "est." are to be tuned from these tests |
| 2026-10-05 | D12 | One relay hop max; relay chosen by the chalet from hub neighbour reports; echo slot repeats the beacon for hubs that cannot hear the chalet | |
| 2026-10-05 | D13 | **Sonar test mode** in the real firmware: fake sonar from virtual nodes on hubs AND from real nodes (sim switch), through the real codec + TDMA transport; switched from the chalet (`/sonar`, `SONAR ON|OFF`) | Fake scene is my own (the prototype from Frank's other conversation was not available to me) |
| 2026-10-05 | D14 | Sonar blocks are **never truncated**: one block per SEC_SONAR section / ESP-NOW frame; hubs and relays drop whole blocks | Review fix #3 |
| 2026-10-05 | D15 | Virtual sonar node IDs = 128 + (hub ID & 0x0F) × 8 + k; **real node IDs stay below 128** | Two hubs with the same low 4 bits of ID would collide |
| 2026-10-06 | D16 | **Frank's display prototype is the reference** (docs/prototype): its simulator and processing are ported to C++ bit-exactly and run on the nodes/hubs in test mode; /sonar uses its layout | Replaces my own fake scene and page (D13) |
| 2026-10-06 | D17 | Sonar block format v2: ≤ 5 targets with 5-bit strength, hardness, noise floor, echo character every 2nd block; the 3-frequency echo trace stays on the node | Needed by the prototype display; still ≈ 35 B/s per FOCUS hole |
| 2026-10-06 | D18 | /sonar defaults to feet (ft/m toggle) | Frank |
