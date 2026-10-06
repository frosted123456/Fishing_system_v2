# OLED screens v2 (proposal, 2026-10-06) — chalet and hubs, Heltec V3 128x64

Mockups: `docs/screens/v2_proposal.png`, rendered on the PC by the **same drawing code** that runs on the board
(`src/lora_node/screens.cpp`, real U8g2 library, `tools/screens_mock.cpp` + `tools/screens_sheet.py`).
Status: approved by Frank and **wired** in the firmware (2026-10-06); first seen on the board 2026-10-06 → navigation/Options rework below.

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
CardKB (first version, replaced below): → ↓ Tab Space = next page, ← ↑ = previous, Enter = page action, S = silence, Esc = home.

## Navigation + Options (after the first bench test, 2026-10-06)
Mockups: `docs/screens/v2_options_nav.png` (same drawing code as the board). Frank's feedback on the board:
arrows not used as expected, not intuitive, settings missing, no address shown.

| CardKB key | Pages | Options page |
|---|---|---|
| ← / → | previous / next page | ← = back (inside Wi-Fi / Simulation); on the main list it changes page |
| ↑ / ↓ | inside the page: Holes / Sonar = more holes, Focus = previous / next hole | move in the list |
| Enter (OK) | the action written in the footer ("OK: ...") | change the value / open / join |
| Esc | Home | back, then Home |
| S | silence on/off | (types an S while typing a password) |
| any key during FISH ON | silence (like the button) | |
Without CardKB the footers say "2x:" (double press) as before; the Options page then only shows values.

Options page (last page, values change at once and are saved like the web page does):
| Chalet | Hub |
|---|---|
| Wi-Fi > (status, network, **choose network** = scan list, type name, address, retry now, channel, hotspot name/pass, forget) | Hotspot ON/OFF |
| Buzzer, Alert hold (10/30/60/120/300 s) | Buzzer, Alert hold |
| Link Auto / LoRa only / ESP-NOW, LoRa channel Auto / 1-8 | ESP-NOW relay, Reed polarity |
| Simulation > (all holes, test holes, fake fish rate, **each hole: off / sonar / fish / both**, ~ = not confirmed yet) | |
| Network reset, Reboot (both ask OK = yes / Esc = no), Built (firmware date) | same |

Cabin Wi-Fi made visible: home footer shows the address when joined, else alternates "No cabin Wi-Fi" / hotspot name;
Connect page shows "<network>: not found / bad password? / connecting..." (reason from the Wi-Fi driver, D36).
