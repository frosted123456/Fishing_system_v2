# Protocol v2 — TDMA LoRa mesh (as built, 2026-10-05)

Decisions: docs/decisions.md (D1-D12). Review that led here: docs/review_protocol_proposal.md.
Code: `lib/IceMesh/src` (pure C++11, host-tested, 81 tests) + `src/lora_node/mesh_radio.cpp`
(radio task). **Not yet tested on hardware.** Every value marked (est.) is a starting point
for the radio test mode.

## 1. Roles
| Role (env) | Board | Does |
|---|---|---|
| chalet (`cabin`, GATEWAY_OFFSHORE) | Heltec V3 | beacon master, never relays, web page, merges node states |
| hub (`hub`, `relay`, `sensor_lora`) | Heltec V3 | ESP-NOW hub for its pocket, sends line states in its slot, relays ONE remote hub when told |
| node (`sensor_wroom`, `sensor_c3`) | ESP32 WROOM (C3 for existing) | unchanged v1 ESP-NOW tip-up firmware for now |

## 2. Radio modes (lib/IceMesh/src/radio_modes.h)
All 500 kHz, CR 4/5, preamble 8, explicit header, hardware CRC on, 915 MHz, 20 dBm.
| Mode | Sensitivity (theory, est.) | 255 B airtime |
|---|---|---|
| SF9/500 | −123.5 dBm | ≈ 313 ms |
| SF8/500 | −121.0 dBm | ≈ 177 ms |
| SF7/500 | −118.5 dBm | ≈ 100 ms |
Beacon, echo, join and relay links always use SF9/500. Direct hub slots: SF9/500 unless
adaptive (off by default) or a test mode says otherwise.

## 3. Superframe (lib/IceMesh/src/tdma_schedule.h)
```
REF ─ beacon (SF9/500) ─ 6 ms ─ [ECHO slots] [JOIN slot] [remote hub slots] [direct hub slots] ─ ≥40 ms ─ next REF (+1 s)
slot = LEAD 3 ms + airtime(allowance, mode) + TAIL 3 ms            (all est.)
```
- REF = beacon START, computed as TxDone − airtime (chalet) or RxDone − airtime (hub): exactly
  periodic, same formula on both sides. Hubs that only hear an ECHO compute REF from its slot.
- Hubs listen for the next beacon from REF − 25 ms, then in the old plan's echo slots; no beacon
  → free-run on the last plan (transmit only after ≤ 1 miss, resync search after 3).
- Unsynced hubs send a JOIN at random every 1.5-4.5 s (est.); synced hubs without a slot use the JOIN slot.

## 4. Packets (lib/IceMesh/src/tdma_proto.h)
Header 6 B: `network_id, 0xF2, type, src, superframe u16`.
| Type | Body |
|---|---|
| BEACON / ECHO | flags (test, adaptive, silenced), cmd + cmd_seq (RESET ALL), silence ×10 s, focus node, frame ×10 ms, test mode, slots n × {kind, owner, mode, allowance, via}, acks n × {hub, node, seq} |
| HUB | flags (silence on/off request, pending), then sections {type, len, value} |
| JOIN | firmware version |

Hub sections, in this order, within the slot allowance:
| Section | Content |
|---|---|
| LINE | 3 B per node: node ID, state(3) / event seq(4) / pending(1), quarter turns(5) / flags(3). Pending first. Always first. |
| RELAY | remote hub's packet re-packed: hub, frame lo, flags, its sections (LINE only if it doesn't fit) |
| HEALTH | every 8 frames: battery, uptime, fw, beacon RSSI/SNR as heard, beacons lost /64, timing error, neighbour hubs + RSSI |
| JOINS | hub IDs heard joining (relay duty) |
| NODEINFO | rotating {node, battery %, flags} |
| TEST | filler up to the allowance in test mode (counter + pattern) |
| SONAR | reserved (phase 4/5) |
Line states: 0 idle, 1 tripped, 2 running, 3 fault, 4 offline (hub lost the node).

## 5. Reliability rules
- **Events**: a state change gives seq+1 and "pending"; repeated in every hub packet until the
  beacon carries ack (this hub, node, seq) — an older ack never clears a newer event.
- **Aggregates**: none. Line state is resent every frame anyway (no ACK on bulk data, D5).
- **Duplicates**: a node heard by two hubs is reported by both; the chalet keeps one OWNER hub
  per node and switches when the owner reports it offline or stops reporting it for 10 frames (est.).
- **Self-healing**: a hub not heard directly for 5 frames is reached through the direct hub
  that carried its packet / reported it (neighbour list, joins). It returns to direct after 3 direct
  receptions. Hubs silent for 30 frames are dropped. The chalet never relays.
- **Adaptive (off by default)**: step to a faster mode after 8/8 packets whose weakest RSSI keeps
  12 dB (est.) over the faster mode's sensitivity; step back after 2 misses; holds 20/30 frames.

## 6. Radio test setting (chalet decides, hubs follow the beacon)
| Setting | Effect |
|---|---|
| off | normal |
| rotate | each direct hub's slot cycles SF9/500 → SF8/500 → SF7/500 every frame; hubs fill their slot with TEST bytes (160 B allowance) |
| fixed SF9 / SF8 / SF7 | all direct hub slots in that mode, TEST filler |
Set from the chalet web page `/radio`, `POST /api/radio {"test":0-4,"adaptive":bool,"resetStats":true}`,
or serial on the chalet: `TEST OFF|ROTATE|SF9|SF8|SF7`, `ADAPT ON|OFF`, `RADIO`, `RADIO RESET`.
Read: `/radio` (per hub per mode: rx/scheduled, CRC, RSSI/SNR avg/min; hub's view of the beacon,
loss /64, timing error), hub OLED test screen, serial every 5 s.

## 7. Open points
| # | Point |
|---|---|
| O1 | Guard / window values (LEAD, TAIL, gaps, beacon window) are estimates — check timing error and slot loss in test mode. |
| O2 | Remote configuration over LoRa removed for now (web page returns an error); to add as a beacon command. |
| O3 | Node firmware still v1 (reed, 2 states). Hall quarter-turn counting, line "running" state and global-ID provisioning come with the WROOM node rewrite. |
| O4 | Sonar codec (phase 4/5) goes in the SONAR section; budget at SF9/500 ≈ 3× the FOCUS + BASE need (est.). |
| O5 | 125 kHz modes not offered (would need hopping). |
| O6 | ESP-NOW RSSI is not available with core 2.x callbacks — node-to-hub link quality is not used yet. |
| O7 | Capacity (calc., est.): a 96 B slot at SF9/500 reserves ≈ 139 ms → about 6 direct hubs per 1 s frame (beacon ≈ 72 ms, join ≈ 30 ms, margin 40 ms). Allowances are fixed per hub today; for sonar they must follow each hub's need (FOCUS hub large, others small). |
