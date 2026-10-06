# OLED screens v2 (proposal, 2026-10-06) — chalet and hubs, Heltec V3 128x64

Mockups: `docs/screens/v2_proposal.png`, rendered on the PC by the **same drawing code** that runs on the board
(`src/lora_node/screens.cpp`, real U8g2 library, `tools/screens_mock.cpp` + `tools/screens_sheet.py`).
Status: approved by Frank and **wired** in the firmware (2026-10-06); not yet seen on a real screen.

## Why rework (current v1 screen)
| Problem | Detail |
|---|---|
| Built for v1 | grid of node IDs (no names, no pockets), "OK/!!/--", header "REMOTE [n/48]"; nothing about the v2 network (setup phase, channel, link, master), no sonar |
| Needs a keyboard | every menu (node list, details, alerts, settings, Wi-Fi typing) needs the CardKB; with the single PRG button only silence and paging work |
| Duplicates the phone page | settings and Wi-Fi typing on a 128x64 screen; the phone suite does it better |
| Hard to read | 5x7 and 4x6 fonts, 10 cells per page; FISH ON is a 9 px footer |
| No "how to connect" | only the IP, only when everything fits on one page |

## Proposed
| Page | Chalet | Hub |
|---|---|---|
| **Alert** (overrides all, blinks, until silenced) | FISH ON, hole name big, pocket + time since trip, "+N more", "Button: silence" | same |
| Home | one-line summary (holes OK / N online / SETUP n min) + one row per pocket (state symbols + OK / N off / ESP-NOW / LOST), 2 columns above 4 pockets | "HUB n", link to the chalet (OK + RSSI / via relay / via ESP-NOW / waiting), its holes with battery |
| Holes | 4 per page: symbol, name, sonar fish count, battery | same |
| Network | link, LoRa channel + MHz + Auto/fixed + activity %, each hub: LoRa RSSI or E+hops or -- | link, channel, ESP-NOW backup in use, uptime |
| Connect | Wi-Fi name, password, address (or cabin Wi-Fi address) | same while the hotspot is on, else "hold 3 s" |
Symbols: filled = OK, hollow = offline, X = sensor fault, half = low battery, blinking big = fish.
Footer: silenced countdown, else Wi-Fi name / address (chalet) or hotspot state (hub); page dots bottom right.

Button (one, same everywhere): short = next page (silence during an alert); hold 3 s = hotspot (hub) /
connect page (chalet); hold 10 s = network reset; a bar shows what releasing does. Pages return to Home after 60 s.
Screen sleeps after 5 min, wakes on alert and on the button (as v1).

## Added after review (2026-10-06)
| Page | Content |
|---|---|
| Sonar | one depth column per sonar hole (common scale): dithered bottom, bars = fish (wider = stronger), tick = bait; 9 per page |
| Focus | mono fish finder of the FOCUS hole (≈30 s, newest right), bottom, fish trails, dotted bait depth; double press = next hole |
| Test | holes running fake sonar / fake trips, radio test; double press = everything on / off |
CardKB (kept as extra keys): → ↓ Tab Space = next page, ← ↑ = previous, Enter = page action, S = silence, Esc = home.
