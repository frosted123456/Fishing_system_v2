# Protocol v2 — LoRa mesh core (DRAFT for review)

Status: draft, 2026-10-05. Scope: what phase 1 implements, plus the rules phase 2 will build on
(marked **[P2]**). Nothing here is coded yet. Items marked **DECISION** are Frank's to make.

## 1. Terms
| Term | Meaning |
|---|---|
| node | battery tip-up (ESP-NOW only, deep sleep). A sonar node is a node with a TUSS4470. |
| hub | always-on board with ESP-NOW + LoRa (today: Heltec V3, `ROLE_GATEWAY_ONSHORE`) |
| cabin | LoRa sink with the display / web page (`ROLE_GATEWAY_OFFSHORE`) |
| origin | the device that **created** a payload. Never changes along the path. Can be a node (a hub injected its frame) or a hub. |
| sender | the device that transmitted **this copy** (changes every hop) |
| serving hub **[P2]** | the one hub responsible for forwarding a given node's traffic |
| parent **[P2]** | the next hub toward the cabin chosen by a hub |

## 2. Common LoRa header (every v2 packet, 8 bytes, little-endian)

| Off | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | `network_id` | same position as v1 → filtered the same way |
| 1 | 1 | `origin` | 1-254; 0 and 255 reserved |
| 2 | 1 | `ver` | `0xF0 \| version` → `0xF2`. v1 code dispatches on byte 2 (`processLoRaMessage` L2221-2225) with no `default:` case, so a v1 board should silently ignore v2 packets (**verify on the bench**) |
| 3 | 1 | `ctl` | bits 7-6 `class` (0 = RELIABLE, 1 = STREAM, 2 = LINK, 3 = reserved) · bits 5-4 `hops` (0-3) · bit 3 `ack_req` · bits 2-0 reserved = 0 |
| 4 | 2 | `seq` | per (`origin`, `class`), 16-bit, wraps; compared with serial arithmetic `(int16_t)(a - b)` |
| 6 | 1 | `sender` | rewritten at each hop |
| 7 | 1 | `type` | payload type (v2 type table, phase 1 reuses v1 payload layouts where possible) |
| 8 | n | payload | ≤ 247 bytes (SX1262 max 255) |

- Max hops = 3 fits the 2-bit field. A relay increments `hops` and drops the packet if it would exceed 3.
- Integrity: rely on the LoRa PHY CRC (phase 1 sets it explicitly with RadioLib) + `network_id` +
  `ver`. No XOR byte. See open question Q9.
- When a hub injects a node's ESP-NOW frame (e.g. an alert), `origin` = node ID and `seq` = the
  node's own ESP-NOW sequence. **Every hub that heard the same alert produces the same identity**,
  so all duplicates collapse network-wide (answers issue 6 for alerts without any coordination).
- Header cost: 8 bytes (v1 headers are 3-7 bytes). Aggregate payload becomes 8 + 1 + 10·n bytes.

## 3. Traffic classes

### 3.1 RELIABLE — alerts, alert clears, config, config ACK, silence, reset, aggregates, track born/lost
- Identity = (`origin`, `seq`) in the RELIABLE sequence space of that origin.
- **Dedup window per origin** (replaces the 32-entry ring for this class):
  `highest` (u16) + `bitmap` (u32, bit *i* = `highest - i` seen) + `last_heard` (ms).
  Let `d = (int16_t)(seq - highest)`:

  | Case | Action |
  |---|---|
  | first frame from this origin | create entry, accept |
  | `d > 0` | shift bitmap left by `d` (clear if `d ≥ 32`), set bit 0, `highest = seq`, accept |
  | `d == 0` | duplicate → drop |
  | `-32 < d < 0` | accept once if bit `-d` clear, then set it; else duplicate → drop |
  | `d ≤ -32` | **origin restart** (reboot reset its counter) → re-init window at `seq`, accept |

  The last row is safe only if a copy can never be older than the window. Guaranteed by two rules:
  every queue drops RELIABLE packets older than **TTL = 30 s**, and no origin sends more than 32
  RELIABLE packets per 30 s (hub today: ~5 aggregates + alerts). Sonar track events could break
  that budget → open question Q6.
- Table: 32 origins × 12 B ≈ 384 B, LRU eviction, entries idle for 10 min are freed.
- Same structure is reused on the hub for ESP-NOW frames (per node).
- ACK / retry (see Q4 for the open part):
  - `ALERT`, alert clear, `CONFIG`, `TRACK_EVENT`: originator sets `ack_req`. The cabin answers
    with an `ACK` packet (RELIABLE class, origin = cabin, payload = list of (origin, seq),
    batched ≤ 8 per packet, flooded back like any reliable packet). Originator retries at
    ~2 s / 4 s / 8 s (± 25 % jitter) until ACKed or 3 retries, each retry with the **same seq**.
  - Aggregates: **no ACK, no retry** (proposal) — the next one 6 s later supersedes it.
  - Relays never retry in phase 1. **[P2]** a relay that does not overhear its parent forwarding
    within one window retries once.

### 3.2 STREAM — sonar focus data (vector frames)
- Identity = (`origin`, `seq`) in the STREAM sequence space. **No ACK, no dedup cache.**
- Every hub keeps `last_fwd_seq` per origin (small table, ~16 × 7 B):
  forward iff `d = (int16_t)(seq - last_fwd_seq) > 0`, then `last_fwd_seq = seq`;
  `d ≤ 0` → drop (older or duplicate); `d ≤ -16` → origin restart, accept and reset.
  Safe because STREAM TTL in any queue is 1 s and a stream sends ≤ ~8 packets/s.
- In the TX queue a newer STREAM packet from the same origin **replaces** the queued one.
- Phase 1: plumbing + a synthetic test source only; multi-hop streaming is only meaningful after
  serving hub + parent selection **[P2]**, because flooding a stream through every hub is exactly
  what issue 2 forbids.

### 3.3 LINK — never relayed (`hops` must be 0)
- Hello / cost advertisement **[P2]**, cabin beacon (later). No dedup, no ACK.

## 4. Relay rules (phase 1 = still flooding, but fixed)
1. Drop if `network_id` or `ver` mismatch, or `origin == self`.
2. Pass the class filter (3.1 dedup window / 3.2 newest-wins). Duplicates are dropped **before** any
   processing that has side effects (buzzer, alert history).
3. Process locally (state update, display).
4. Forward only if `hops < 3` and the role forwards (**DECISION Q8**: should the cabin forward?).
5. Forwarded copy: `sender = self`, `hops + 1`, **pushed to the TX queue** with
   `not_before = now + random(0 … 2 × airtime(packet))` — never `delay()`.
6. **[P2]** only the serving hub forwards a node's traffic; other hubs that heard it hold the copy
   and cancel it if they overhear the serving hub (or anyone closer to the cabin) forwarding the
   same identity before `not_before`; parents chosen by advertised cost (hops + link quality) with
   hysteresis; loop prevention = strictly decreasing cost, never pick a parent whose parent is
   you, hop limit 3.

## 5. Non-blocking TX queue
- Called from `loop()` only. ISRs and the ESP-NOW callback **only copy into an RX ring buffer**;
  all parsing, state changes and TX decisions happen in `loop()` (fixes the WiFi-task race).

| Prio | Content | Depth | TTL | Supersede key |
|---|---|---|---|---|
| P0 | alert, alert clear, ACK | 8 | 30 s | – |
| P1 | config, config ACK, silence, reset | 8 | 30 s | – |
| P2 | aggregate, sonar static scene, track born/lost | 4 | 10 s (aggregates) / 30 s | (type, origin) for aggregate & static scene |
| P3 | stream | 2 | 1 s | (type, origin) |

- Scheduler (each `loop()`): if the radio is idle in RX and `now ≥ next_allowed`, take the
  highest-priority entry whose `not_before ≤ now`, run CAD; busy → `not_before += random backoff`
  (exponential per entry, capped), free → `startTransmit()` (non-blocking); TX-done IRQ → back
  to `startReceive()`. No `delay()` anywhere in the path.
- LBT always runs (v1 skips it after a recent RX, L1914 — removed).
- Full level: drop the oldest entry of that level, count it in diagnostics (`tx_dropped[prio]`).
- Airtime guard: token bucket on STREAM (default ≤ 30 % of airtime, configurable) so the stream
  can never starve aggregates; diagnostics expose airtime used per class.
- ESP-NOW sends stay direct (they are already non-blocking) but move out of the callback into `loop()`.

## 6. Airtime reference (SX1262, BW 125 kHz, CR 4/5, preamble 8, CRC on, explicit header)

| Payload | SF7 | SF8 | SF9 (today) | SF10 |
|---|---|---|---|---|
| 18 B (v1 alert) | 51 ms | 93 ms | 185 ms | 330 ms |
| 52 B (8 B hdr + 4 pings batched) | 103 ms | 185 ms | 329 ms | 616 ms |
| 108 B (v1 aggregate) | 185 ms | 328 ms | 595 ms | 1067 ms |

One focused hole at 50 B/s payload ≈ one 52-byte packet per second ≈ **33 % airtime at SF9,
10 % at SF7**, doubled per relay hop. This is the data behind DECISION Q2.

## 7. Backward compatibility
- ESP-NOW frames (`SensorMessage`, `AlertMessage`, `AckMessage`, …) are **unchanged in phase 1**:
  already-built tip-up nodes keep working without reflashing.
- LoRa: v1 and v2 boards cannot exchange aggregates/alerts unless a v2 board also speaks v1
  (**DECISION Q1**).

## 8. Open questions
| # | Question | Owner | My recommendation / note |
|---|---|---|---|
| Q1 | Backward compatibility with LoRa boards already built: flag day (reflash all Heltecs) or a v2 build that also parses/sends v1 for a transition? | **Frank** | Flag day is far simpler; tip-up nodes need no reflash in phase 1 either way |
| Q2 | Switch the network to SF7 during focus (or permanently)? | **Frank** | Numbers in §6. SF7 has ~5 dB less link budget than SF9 (demod SNR limit −7.5 dB vs −12.5 dB) |
| Q3 | Hub board for sonar hubs: Heltec V3 vs WROOM + LoRa module | **Frank** | Not needed for phase 1 |
| Q4 | Can every hub reach the cabin directly? | **Frank** | Decides how much phase 2 relaying matters |
| Q5 | Exact versions: Arduino-ESP32 core (or Heltec package) and RadioLib / U8g2 / ArduinoJson | Frank (to tell me) | Code uses core-2.x APIs. Non-blocking TX needs RadioLib `startTransmit()` + `finishTransmit()` (RadioLib 6.x); I need the version before coding |
| Q6 | Track born/lost as RELIABLE could exceed 32 packets / 30 s per origin. Widen the window to 64 bits, or rate-limit track events? | Frank | 64-bit bitmap costs 4 B per origin; I'd do it |
| Q7 | Aggregates without ACK/retry (superseded every 6 s) — OK? Your brief lists them under "ACK and retry" | Frank | Retrying a periodic snapshot only adds airtime |
| Q8 | Should the cabin relay? (v1: yes, everything) | Frank | No — a sink that relays doubles airtime near the cabin |
| Q9 | Integrity: PHY CRC only, or add an app CRC-16 (+2 B/packet)? | Frank | PHY CRC + `ver` + `network_id` is enough; the private sync word 0x34 already filters most other LoRa traffic (not all) |
| Q10 | Build system for shared code (`common/` used by both sketches + host tests): stay in Arduino IDE (copy step / library) or move to PlatformIO? | Frank | PlatformIO makes shared code + native tests trivial; Arduino IDE works with a small sync script |
| Q11 | Phase 5 wants a full sonar page on the onshore hub, but in LR mode it has no AP (L3805-3813) | Frank | Revisit in phase 5 (AP on a second radio, or page on the cabin only) |
| Q12 | ESP-NOW link quality for serving-hub choice: core 2.x gives no RSSI in the receive callback | phase 2 | promiscuous RX callback or core 3.x |
| Q13 | Node ID plan: are IDs 1-254 unique across all hubs, nodes, sonar nodes and the cabin? Max nodes per network (v1 `MAX_NODES` 16)? | Frank | |
| Q14 | `sonar-display-prototype.html` is not in the IceFishing folder | Frank | Needed from phase 4 only |
