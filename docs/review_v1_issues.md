# Review of the 6 suspected issues (v1 baseline)

All line numbers: `lora_node/lora_node.ino` unless another file is named. Verdicts come from
reading the code; none of them was reproduced on hardware.

| # | Suspected issue | Verdict |
|---|---|---|
| 1 | Aggregate dedup key ignores the originating hub | **Confirmed** (worse than suspected) |
| 2 | Pure flooding | **Confirmed** (and relays are not collision-protected) |
| 3 | 32-entry / 30 s cache flushed by a 4 Hz stream | **Confirmed** by arithmetic (stream not built yet) |
| 4 | Blocking `delay()` in relay paths and retry | **Confirmed** (and some run inside the ESP-NOW callback) |
| 5 | Sensor keeps the first hub that ACKs | **Confirmed**, with nuances |
| 6 | Node heard by several hubs appears in several aggregates | **Confirmed**, can cause offline/online flapping at the cabin |

## 1. Aggregate relay dedup key — CONFIRMED
- Relay side, L2369-2373: `dedupKey = (network_id << 8) | lora_sequence` and the cache is called
  with `node_id = msg->network_id`. Own TX, L2815-2816: same key.
- So for every hub the cache key is `(0x42, 0x42xx)`: the only varying part is the **8-bit**
  `lora_sequence` (`messages.h` L140), fed from a 16-bit counter truncated at L2699.
- Each hub starts its counter at 0 at boot and increments it every ~6 s, so hubs powered on
  together advance almost in lockstep. Hub B records its own seq *k* (L2816); when hub A's
  aggregate with the same *k* arrives within 30 s, B refuses to relay it. A → B → cabin
  multi-hop then fails silently for long stretches.
- Only relaying is affected; B still *processes* A's contents (L2266-2363 run before the relay test).
- Related: the relay overwrites `sender_id` (L2375), so after one hop **the origin hub is lost**.
  The receiver then refreshes the *relay* as "sender" (L2251-2257) and gives it the origin's role
  (`sender_role` is not rewritten). The v1 frame has no origin field at all.

## 2. Pure flooding — CONFIRMED
- Aggregates: every LoRa node of **any role, including the cabin (OFFSHORE)**, relays any
  aggregate with hop < 3 (L2365-2384; the comment at L2366 says so).
- Alerts: RELAY and SENSOR_LORA roles relay (L2419); silence: all roles (L2485);
  config: all roles (L2612); config ACK: all roles **with no dedup** (L2662-2677).
- Relays go through `loRaTransmitWithWait` (L1815), which has **no CAD**, after only
  `delay(random(20, 50))` (L2380). Every node that heard the same frame retransmits within
  ~30 ms of the others while one aggregate lasts ~595 ms at SF9 → the relays collide with each other.
- Airtime: one aggregate ≈ 595 ms. Worst case (3 hubs + cabin all hearing each other): each
  aggregate is sent once and relayed by the 3 other nodes = 4 × 595 ms every 6 s per hub →
  offered load > 100 %. In practice issue 1 suppresses many relays at random, so **issue 1 may
  currently be hiding this airtime problem**. A 4 Hz sonar stream on top is impossible this way.

## 3. 32-entry / 30 s cache vs a 4 Hz stream — CONFIRMED (projection)
- One global ring, `DEDUP_CACHE_SIZE 32`, `DEDUP_WINDOW_MS 30000` (`messages.h` L268-269);
  `recordForDedup` overwrites the oldest slot whatever its age (L3398-3403).
- 4 frames/s wrap the ring in 8 s < 30 s, so an entry is evicted before its window ends and a
  late copy (3 hops, CAD backoff) would be relayed again.
- Already today the ring is shared by unrelated traffic: **every sensor status/heartbeat heard
  over ESP-NOW takes a slot (L3049) though it is never relayed**, plus alerts (L3100, L2423),
  aggregates (L2373, L2816) and silence (L2488). Namespaces also collide: aggregates use
  `node_id = 0x42 (66)`, so a real node 66 would share keys with them; silence stores a 32-bit
  timestamp truncated to 16 bits.

## 4. Blocking delays — CONFIRMED
Relay paths: L2380 (aggregate), L2427 (alert), L2493 (silence), L2605 / L2625 / L2674 (config,
50-300 ms). Retry: `sendLoRaAggregateWithRetry` L2828-2850 = 3 × (~595 ms airtime + LBT) +
2 × 150-250 ms ≈ **2-3 s blocked**. Also: LBT backoff `delay(200-500)` ×2 (L1925-1928), TX
retry
backoff (L1988-1993), BUSY polling after `radio.transmit()` (which already blocks) (L1835-1838,
L1954-1957), local reed alert 3× with delays (L3639-3642), `sendRemoteConfig` (L5627-5630),
reset delays up to 3 s (L2533, L3187, L5678), `showOverlayMessage` up to 1.5 s (L837).

**More serious than the delays themselves:** `processEspNowMessage` runs in the WiFi task
(L2962-2966) and calls `sendLoRaAggregateWithRetry(3)` (L3037), `relayLoRaAlert` (L3101) and
`loRaTransmitWithWait` (L3151). That (a) blocks the WiFi task for up to ~3 s, during which
other ESP-NOW frames are delayed or lost, and (b) drives the SX1262 from a second task with no
lock while `loop()` may be in the middle of its own TX/RX — a real race on SPI and on
`radioState`. A non-blocking queue fixes both if the callback only enqueues.

Also noted: `ensureLoRaReceiveMode` (L1686-1697) resets the timer after a forced reset, so a
permanently stuck BUSY pin loops until the 30 s task watchdog panics (reboot = recovery, but slow).

## 5. Sensor keeps the first hub that ACKs — CONFIRMED (nuances)
- `sensor_node.ino` L796: the MAC is learned only `if (!gatewayMacKnown)`; there is no link
  quality comparison (and none is available: core-2.x receive callbacks give no RSSI, the hub
  passes `rssi = 0`, L2966).
- Every hub that accepts the alert answers with a **broadcast** ACK (L3083-3090), so "first"
  means whichever ACK the sensor processes first, not the nearest hub.
- Hubs ACK **only `MSG_ALERT`** (L3080). A node that never alerts never learns a hub and always
  broadcasts (which, with several hubs, is actually the more robust behaviour).
- Failover exists: 3 failed unicasts → MAC cleared → broadcast (L570-575); next ACK re-learns.
  The learned MAC survives deep sleep (RTC, L92-93), so a sensor stays on a weak hub until it
  fails 3 times in a row.

## 6. Same node in several aggregates — CONFIRMED (+ flapping)
- Every hub that hears a sensor directly sets `via_espnow = true` (L3011, L3073), and the
  aggregate includes every node with `via_espnow` (filter L2765 only skips "LoRa-only" nodes).
  → the node is reported by every hub that hears it, costing 10 bytes per extra copy.
- Side effect: when the weaker hub B times the node out (L3487-3507) it clears both
  `via_espnow` and `via_lora`, so the filter no longer skips it and B **advertises it with
  `AGG_FLAG_NODE_OFFLINE`** (L2782-2784) until B hears it again by LoRa. The cabin has no direct
  contact, so it marks the node offline and **clears FISH_ON** (L2273-2305); hub A's next
  aggregate brings it back online and, if the flag is up, RULE 1 (L3247) re-accepts FISH_ON,
  restarts the hold timer and records a second alert (L3414-3418). Expected only when B cannot
  hear A's aggregates (separate pockets), which is exactly the multi-pocket layout.

## Other findings (not in the list)
| # | Finding | Where | Matters for |
|---|---|---|---|
| A | Radio driven from the ESP-NOW callback (WiFi task) — see §4 | L3037, L3101, L3151 | phase 1 (queue) |
| B | LBT is skipped when a frame was received < 2 s ago — the moment others are most likely active | L1914 | phase 1 |
| C | Cabin (OFFSHORE) relays everything it hears; a sink normally should not | L2368 | phase 1/2 — your call |
| D | `MSG_CONFIG_ACK` relayed with no dedup | L2662-2677 | phase 1 |
| E | SENSOR_LORA / RELAY roles never send their own periodic status; they look "online" only because relayed frames carry their ID as sender | L677-682, L2375 | phase 2 |
| F | ONSHORE hub in LR mode has no AP / web server, but phase 5 wants a full display page on the onshore hub | L3805-3813 | phase 5 — open question |
| G | No ESP-NOW RSSI with core 2.x callbacks → serving-hub choice by link quality needs promiscuous RX or core 3.x | L2966 | phase 2 |
| H | `settings.heartbeatSec` is stored/edited/sent remotely but never used (loop uses `HEARTBEAT_INTERVAL_SEC`); `loadAllNodeNames()` is never called | L607, L5499 | cosmetic, out of scope |
| I | Sensor heartbeat is 60 s (`sensor_node/config.h`), hub `NODE_TIMEOUT_SEC` is 90 s → a single lost heartbeat (120 s gap) is enough to flag a node offline | config.h files | phase 2 tuning |
