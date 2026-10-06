# Field guide — LoRa channel, ESP-NOW backup, relays (firmware v2, 2026-10-06)

Everything here is set **on the chalet** (phone → Settings → Network, or serial). Hubs and tip-ups
follow by radio: nothing to reflash on the ice. Protocol details: protocol_v2.md §6c.
**Not yet tested on hardware** — ranges below are simulation estimates.

## Setting up on the ice
- Switch the devices on **in any order**, as you prepare the holes. Nothing changes mode during this
  **setup phase**: everyone stays on the LoRa channel used last time.
- The page says "setup phase (N min)" until the network has run complete for 5 min; then the automatic
  fallbacks (channel change, ESP-NOW backup) are armed.
- Mount every tip-up board **at least 20 cm above the ice** (v1 range problems: boards at ~5 cm).
- Every hub already relays ESP-NOW; nothing to set.

## The settings (Settings → Network → Advanced; normally nothing to change)
| Setting | Choices | Default | What it does |
|---|---|---|---|
| Hub ↔ chalet link | Auto / LoRa / ESP-NOW | Auto | Auto = LoRa, and a hub that loses the LoRa beacon for 10 s also sends over ESP-NOW until LoRa is back |
| LoRa channel | Auto / 1-8 | Auto | Auto = start on the last channel (915.0 MHz first), move when it gets > 25 % busy and another is clearly quieter |
| Relay (per device) | on / off | hubs on, tip-ups off | Device rebroadcasts ESP-NOW backbone frames; max 2 relays in a row. A tip-up relay stays awake (battery) |

## On the ice: what to do when…
| Situation | You see | Do |
|---|---|---|
| Other LoRa users on our channel | Settings: activity % high on the channel "in use" | Nothing in Auto (moves within ~1 min). Or pick the quietest channel by hand |
| Every LoRa channel busy | All channels > 30 %, alerts slow | Link → **ESP-NOW**; make sure every hub has an ESP-NOW path (relays) |
| One pocket too far for LoRa | Radio → Links: hub "LoRa heard" grows, "ESP-NOW heard" recent | Nothing in Auto; if "ESP-NOW heard" is also old → add a relay between |
| Hub not heard at all | Holes offline | Check power; move a relay closer; the hub keeps searching all channels by itself |

## Placing relays (estimates, flat ice, sim v2)
| Link | Path loss exponent 2.5 (clear) | 3.0 | 3.5 (pessimistic) |
|---|---|---|---|
| ESP-NOW normal rate (1 Mbps), one hop between raised boards (6 dB margin) | ~745 m | ~250 m | ~110 m |
| LoRa SF9/500 hub ↔ hub | | | ~1.4 km |
- Plan **one relay per ~150-200 m** of ESP-NOW distance until the field test gives real numbers; at most 2 relays in a row.
- A relay can be: a hub (no battery cost difference, it is awake anyway), a spare board flashed as a tip-up with
  `-DEB_RELAY_FORCE=1`, or a tip-up (stays awake: its battery lasts much less).
- Height helps more than anything: raise the board/antenna a little above the ice.

## Serial commands (chalet unless noted)
| Command | Effect |
|---|---|
| `TRANSPORT AUTO\|LORA\|ESPNOW` | link setting |
| `CHANNEL AUTO\|1-8\|SCAN` | LoRa channel; SCAN = re-evaluate now (on a hub: shows the channel) |
| `RELAY <id> ON\|OFF` | relay on a hub or tip-up (beacon command) |
| `RELAY ON\|OFF` | (any LoRa board) relay on this device |
| `EBMODE NORMAL\|LR` | chalet ESP-NOW rate, reboot to apply. **LR removes the phone hotspot** (bench only) |
| `RADIO` | status incl. `[net]` lines: transport, channel, activity %, backbone counters |

## Bench tests before the first outing (in this order)
1. **Chalet hears hubs on ESP-NOW**: `TRANSPORT ESPNOW`; Radio → Links: "ESP-NOW heard" must count up for each hub.
   Needs hubs/tip-ups built with `ESPNOW_LONG_RANGE_MODE false` (LR and a phone hotspot cannot share a board, D27).
2. **Phone hotspot still works** while the chalet runs ESP-NOW (connect, open the suite, trip a tip-up).
3. **Back to LoRa**: `TRANSPORT AUTO`; hubs back on LoRa within ~1 min (Links: "backbone no").
4. **Channel move**: `CHANNEL 3` → every hub follows within ~6-12 s (`RADIO` on a hub shows the channel); `CHANNEL AUTO` after.
5. **Fallback**: unplug a hub's LoRa antenna (or shield it) in Auto → its holes keep updating through ESP-NOW after ~10 s.
6. **Relay**: put hub B out of the chalet's Wi-Fi reach, `RELAY <hub A> ON` → Links shows hub B with "relays passed 1".
7. **Tip-up relay**: `RELAY <tip-up id> ON`, trip that tip-up once → its serial prints "Backbone relay ON (set by the chalet)" and it stays awake.
8. Range walk with one relay: note distances where "ESP-NOW heard" stops updating; replace the estimates above.
