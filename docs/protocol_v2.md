# Protocol v2 — TDMA LoRa mesh (as built, 2026-10-05)

Decisions: docs/decisions.md (D1-D26). Review that led here: docs/review_protocol_proposal.md.
Code: `lib/IceMesh/src` (pure C++11, host-tested, 110 tests in 15 suites) + `src/lora_node/mesh_radio.cpp`
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
| BEACON / ECHO | flags (test, adaptive, silenced, sonar sim), cmd + cmd_seq, silence ×10 s, focus node, frame ×10 ms (1 / 1.5 / 2 s), test mode, cmd target + value, net_cfg (§6c), slots n ≤ 16 × {kind, owner, mode, allowance, via}, acks n × {hub, node, seq} |
| HUB | flags (silence on/off request, pending, backbone relay / path §6c), then sections {type, len, value} |
| JOIN | firmware version |

Hub sections, in this order, within the slot allowance:
| Section | Content |
|---|---|
| LINE | 3 B per node: node ID, state(3) / event seq(4) / pending(1), quarter turns(5) / flags(3). Pending first. Always first. |
| RELAY | remote hub's packet re-packed: hub, frame lo, flags, its sections. If it doesn't fit: LINE trimmed to whole records, other sections kept whole while they fit, TEST dropped |
| HEALTH | every 8 frames: battery, uptime, fw, beacon RSSI/SNR as heard, beacons lost /64, timing error, neighbour hubs + RSSI |
| JOINS | hub IDs heard joining (relay duty) |
| SONAR | one sonar block per section (§6b), whole blocks only: BASE, then DATA (oldest first), then BG |
| NODEINFO | rotating {node, battery %, flags} |
| TEST | filler up to the allowance in test mode (counter + pattern) |
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
Set from the Radio tab of the chalet web suite (`/radio`), `POST /api/radio {"test":0-4,"adaptive":bool,"resetStats":true}`,
or serial on the chalet: `TEST OFF|ROTATE|SF9|SF8|SF7`, `ADAPT ON|OFF`, `RADIO`, `RADIO RESET`.
Read: `/radio` (per hub per mode: rx/scheduled, CRC, RSSI/SNR avg/min; hub's view of the beacon,
loss /64, timing error), hub OLED test screen, serial every 5 s.

## 6b. Sonar blocks (lib/IceMesh/src/sonar_codec.h, sonar_link.h)
Every block decodes on its own (review fixes): node ID in each block, length = its SEC_SONAR section
(or its ESP-NOW frame), first ping index (16 bit), 3-bit track slot on every target (0 = bait).
| Block | Content | Size (measured on fake data) |
|---|---|---|
| BASE v3 (every 2 s, + at once when a fish shows up) | node, activity 0-15, ping index, bottom (cm, 11 b), hardness (2 b), fish count (3 b), static-scene version, **every target of the latest ping** (≤ 5: depth 11 b, level 2 b, bait 1 b); nearest fish derived on decode | 7-16 B |
| DATA (FOCUS, 4 pings/block, 1 block/s) | header 6 B (node, type, N, first ping, noise floor −dB, scene version, hardness, info flag); per ping: bottom (abs 11 b, then ±7 cm / escape), counts (3 + 3 b), targets (slot 3 b; first in block: depth 11 b + strength 5 b + width 3 b; then ±31 cm / escape + strength 5 b), changed cells (Rice gaps k=6 + level 2 b); every 2nd block: echo character per target (slot, flicker 0.25 dB, frequency spread 0.5 dB, echo length bins, mature) | ≈ 7.3 B/ping |
| BG (FOCUS, 1 of 8 segments every 2 s) | 4-level static scene (prototype's grey layer), RLE + Rice, new version when > 12 bins changed | ≈ 6 B/segment |
FOCUS total ≈ 35 B/s incl. section headers. Strength = the prototype's display value × 31.
- Hub: `SonarOutbox` (12 blocks, stale after 4 frames, eviction BG → DATA → BASE). No ACK.
- Chalet: `SonarStore` (16 nodes, ring of 128 pings, duplicate/late pings dropped, node restart detected, **~3 min of summaries per hole** for the at-a-glance views: `/api/sonar/glance`).
- Planner: the hub reporting the FOCUS node gets 180 B (est.) instead of 96 B; a remote FOCUS hub
  gets at most 255 − 96 so its relay can carry it; FOCUS shrinks first when the plan is too long.
- Node ↔ hub (ESP-NOW): `MSG_SONAR` 0x60 = [net][node][0x60] + one block;
  `MSG_SONAR_CTRL` 0x61 = {net, hub, 0x61, focus node, sim on, 0}. Test mode: docs/SONAR_SIM.md.

## 6c. Network config: LoRa channel, transport, ESP-NOW backbone (2026-10-06)
Set on the chalet only (Settings → Network, `/api/radio`, serial); hubs follow the beacon and the
backbone beacon, so **nothing is reprogrammed on the ice**. Code: `tdma_proto.h`, `eb_link.h`,
`hub_role.h`, `chalet_role.h`, firmware `mesh_radio.cpp`. Simulated (docs/SIM_RESULTS.md, sim v2), **not tested on hardware**.

**Beacon v2** (17 B fixed + slots + acks): byte 13 `cmd_target`, 14 `cmd_value`, 15 `net_cfg`, 16 `n_slots`.
`net_cfg` = transport (bits 0-1: 0 Auto, 1 LoRa only, 2 ESP-NOW only) | LR (bit 2) | LoRa channel (bits 4-7).
Commands (one at a time, each in 6 beacons, new `cmd_seq` each; hubs act once per seq):
| cmd | target | value |
|---|---|---|
| 1 RESET ALL | – | – |
| 2 SET CHANNEL | beacons left before the switch (5 → 0) | channel 0-7 |
| 3 SET RELAY | device ID (hub or tip-up) | 1 on / 0 off |
| 4 SET SIM | hole ID (255 = every hole) | bit0 fake sonar, bit1 fake Hall trips, bits 2-7 trips/hour (0 = 6) |
Hub flags (HUB packet byte 6) gain `HF_EB_RELAY` 0x08 (this hub relays the backbone) and `HF_EB_PATH` 0x10
(this hub also sends on the backbone now) — reports for the Radio tab only.

**LoRa channels** (all 500 kHz, 20 dBm): 1 = 915.0, 2 = 904.0, 3 = 907.0, 4 = 910.0, 5 = 913.0, 6 = 918.0, 7 = 921.0, 8 = 924.0 MHz
(index 0-7 in code). The chalet measures activity per channel: instantaneous RSSI ~ every ms above −100 dBm (est.),
on its own channel between frames (network silent then) and 60 ms of one other channel per frame (round robin);
% kept as a 60 s sliding value. **Auto**: at boot sample every channel 250 ms; every 60 s move when the channel in
use is > 25 % busy and another is quieter by ≥ 15 points (915.0 MHz gets a 5-point preference) — same rule as the sim.
A move is announced with SET CHANNEL in 6 beacons (also on the backbone); everyone retunes after the target-0 beacon.
A hub without beacon hops to the next channel every 2.5 s unless it heard the network (or a backbone beacon naming
the channel) in the last 10 s. Every device saves the channel it last used (NVS) and starts there after a reboot.

**ESP-NOW backbone (EB)**: ESP-NOW broadcast, Wi-Fi channel `ESPNOW_CHANNEL`, **normal rate (802.11b 1 Mbps)**: a device with a phone hotspot
cannot enable LR (D27), so the backbone only reaches hubs built with `ESPNOW_LONG_RANGE_MODE false`. Frame = 7 B header {net, sender (this hop), type, hops, origin, origin seq u16} + payload ≤ 243 B.
| Type | Payload | Sent by / when |
|---|---|---|
| 0x70 EB beacon | the chalet beacon without slots (acks, flags, commands, net_cfg, frame no.) | chalet, every frame, in Auto / ESP-NOW (in LoRa only: 2 min after a change and while a hub talks on the backbone) |
| 0x71 EB hub | the hub packet built without slot (`buildHubFree`: line states first, sonar, node info, health) | hub, every 1 s while the policy says so, 250 ms after a new line event |
Relays: any device with the relay setting on (hub, tip-up kept awake, spare board) rebroadcasts frames not seen
before (dedup on origin/type/seq, 48 entries), up to **2 relays in a row**; its own frames are never relayed back.
**Setup phase (D30):** after power-on every device starts on the saved LoRa channel. Automatic channel moves
wait until a hub has been in the network 5 min (or 10 min uptime); a hub's automatic fallback waits for 5 min of
stable LoRa beacons. A hub that has not heard any beacon yet also sends its status on the backbone (searching).
All hubs relay the backbone by default (D29).
Transport policy per hub (`TransportPolicy`, once armed):
| Setting | Hub sends on | Safety net |
|---|---|---|
| Auto | LoRa; + backbone after 10 s without LoRa beacon, until 5 LoRa beacons in a row | – |
| LoRa only | LoRa | + backbone after 60 s without LoRa beacon |
| ESP-NOW only | backbone (LoRa radio idle) | LoRa again after 60 s without backbone beacon |
Sending on both is safe: the chalet keeps one state per node and drops repeated sonar pings.
Tip-up relay command: the hub sends `MSG_DEV_CMD` 0x62 {net, hub, 0x62, node, 1 = relay, on/off} right after each of the
node's next 3 messages (a sleeping node only listens ~120 ms after it transmits). A relay tip-up stays awake (battery!).

## 7. Open points
| # | Point |
|---|---|
| O1 | Guard / window values (LEAD, TAIL, gaps, beacon window) are estimates — check timing error and slot loss in test mode. |
| O2 | Remote configuration: only the network settings (transport, channel, relays) and RESET ALL travel as beacon commands (§6c); other hub settings are set on the hub. |
| O3 | Node firmware still v1 (reed, 2 states). Hall quarter-turn counting, line "running" state and global-ID provisioning come with the WROOM node rewrite. |
| O4 | Sonar chain built on FAKE data only (§6b): processing v0 = port of the prototype (thresholds tuned on its simulation). Real sizes and thresholds need TUSS4470 recordings; per-hole bait depth setting still missing. |
| O5 | 125 kHz modes not offered (would need hopping). |
| O6 | ESP-NOW RSSI is not available with core 2.x callbacks — node-to-hub link quality is not used yet. |
| O7 | Capacity (calc., est.): a 96 B slot at SF9/500 reserves ≈ 139 ms → about 6 direct hubs per 1 s frame (beacon ≈ 72 ms, join ≈ 30 ms, margin 40 ms). Allowances: 96 B per hub, 180 B for the FOCUS hub (est.); not yet sized from each hub's queue. |
| O8 | To verify on hardware: TX start latency (log nowUs() around startTransmit), RadioLib 7.7.1 RX IRQ defaults (HeaderValid enabled, DIO1 = RxDone), TCXO delay. A settings save (NVS write) can shift one frame's timing — harmless, only when settings change. |
| O9 | LR vs hotspot: confirmed incompatible on one ESP32 (D27). Open: switch the ice side to the normal rate (range cost to measure: walk test LR vs 1 Mbps, tip-up ↔ hub), real ESP-NOW range on ice (sim: est. only). |
| O10 | Channel activity threshold (−100 dBm) and the 25 / 15 % rule are sim values; check the activity % on the Settings page against a known busy channel. |
