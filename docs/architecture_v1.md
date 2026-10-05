# Architecture map — v1 code (baseline commit)

Line numbers refer to the baseline commit on `main`. "L" = line.

## 1. Files

| File | Lines | Role |
|---|---|---|
| `sensor_node/sensor_node.ino` | 1064 | Tip-up node: reed switch, ESP-NOW (LR mode, ch 1), deep sleep |
| `sensor_node/config.h` | 182 | Board pins (C3 Super Mini / ESP32-CAM / WROOM), timings, reed polarity |
| `sensor_node/messages.h` | 337 | **Byte-identical copy** of `lora_node/messages.h` (checked with `diff`) |
| `lora_node/lora_node.ino` | 5684 | All LoRa roles + OLED menu + CardKB + web server (~45 % of the file is UI/HTML) |
| `lora_node/config.h` | 182 | Heltec V3 pins, LoRa PHY settings, timings, WiFi mode |
| `lora_node/messages.h` | 337 | Shared packed structs, flags, dedup cache type, helpers |
| `BUG_FIXES_SUMMARY.md` | 197 | History of earlier fixes |

Hardware confirmed from code: **Heltec WiFi LoRa 32 V3 = ESP32-S3 + SX1262**
(`lora_node/config.h` L4, L102; `lora_node.ino` L4, L111 `SX1262 radio = new Module(...)`).
Board package referenced: Heltec ESP32 Series, index URL `.../0.0.9/...` (`lora_node.ino` L8).
RadioLib / U8g2 / ArduinoJson versions are **not pinned anywhere** in the repo.

Toolchain clues (to be confirmed by Frank): both sketches use **Arduino-ESP32 core 2.x
(ESP-IDF 4.x) APIs** — `esp_task_wdt_init(30, true)` (`lora_node.ino` L457) and the
`onEspNowRecv(const uint8_t* mac, ...)` callback signature (`lora_node.ino` L2962,
`sensor_node.ino` L778). Both signatures changed in core 3.x, so as written the code does not
build on core 3.x. Consequence: no ESP-IDF 5 APIs, **no ESP-NOW v2 (> 250-byte payloads)**.

## 2. sensor_node (battery tip-up)

Everything happens in `setup()`; `loop()` (L324) only runs if `SLEEP_ENABLED` is false.

```
wake ─► validateAndInitRtcData (L982) ─► read battery + reed (L198-199)
     ─► setupEspNow (L404: STA, LR protocol L417-438, channel 1, broadcast peer, restore hub peer)
     ─► reed open ? sendAlert (L726) : sendStatus(HEARTBEAT) (L691)
     ─► if alert: stay awake ≤ 30 s, re-send every 5 s, poll reed every 500 ms,
        exit on FLAG_ACKNOWLEDGED or flag reset (L246-287)
     ─► deep sleep: 60 s + GPIO wake (enterDeepSleep L860)
                    or 5 s timer-only while FISH_ON (enterDeepSleepFast L934)
```

- TX: `espNowSendReliable` (L536): unicast ×3 to the learned hub MAC (HW ACK);
  on total failure clears the MAC (L570-575) and broadcasts ×2 (L579-606).
- RX (`onEspNowRecv` L778): `MSG_ACK` for this node → set ACKNOWLEDGED, learn hub MAC **only if
  none known** (L796); `MSG_SILENCE_SYNC` (L818); `MSG_RESET_CMD` → restart (L843).
- Sequence: `messageSeq` in RTC memory (L73), +1 on every frame sent (L698, L742), reset to 0
  on power-on (L180), `FLAG_FIRST_BOOT` set on power-on.
- Heartbeat: 60 s (`sensor_node/config.h` `HEARTBEAT_INTERVAL_SEC`), 5 s while FISH_ON.

## 3. lora_node (Heltec V3) — roles chosen by `NODE_ROLE` (L29)

| Role | setup (L494-535) | Periodic work in `loop()` (L593-683) |
|---|---|---|
| `ROLE_GATEWAY_ONSHORE` (hub on ice) | ESP-NOW (LR, STA-only → **no AP/web**, L3805-3813), local sensor | local reed, self heartbeat 30 s, **LoRa aggregate every 6 s + 0-500 ms jitter**, skipped if a frame was received < 1.5 s ago (L623-634) |
| `ROLE_GATEWAY_OFFSHORE` (cabin) | WiFi AP/STA + web | web server, self heartbeat, aggregate every ~11 s (6 s + 5 s offset, L663-674) |
| `ROLE_SENSOR_LORA` | ESP-NOW + local sensor | local reed only (no periodic aggregate) |
| `ROLE_RELAY_LORA` | ESP-NOW | nothing periodic |

Every role, every loop: `loopCardKB`, `loopButton`, `loopLoRa` (L2121), `loopLoRaWatchdog`
(L2014), `loopAutoUnsilence`, `loopDisplay`, `loopBuzzer`, `loopNodeTimeout` (L3463).

### 3.1 LoRa radio layer
- PHY (`lora_node/config.h` L22-29): 915 MHz, BW 125 kHz, **SF9**, CR 4/5, sync 0x34, 20 dBm, preamble 8.
- RX: DIO1 ISR `onLoRaReceive` (L2063) sets `loraInterrupt`; `loopLoRa` (L2121) reads the
  packet and calls `processLoRaMessage` (L2212). `processPendingLoRaRx` (L1766) drains a
  pending RX before any TX.
- TX — two helpers, both **blocking** (`radio.transmit()` + BUSY polling):
  - `loRaTransmitPacket` (L1889): CAD listen-before-talk ×2 with `delay(200-500 ms)` backoff
    (L1919-1929) — **skipped if anything was received in the last 2 s** (L1914) — then up to
    3 tries with doubling `delay()` (L1988-1993). Used by own originations.
  - `loRaTransmitWithWait` (L1815): **no CAD at all**. Used by every relay path and by the
    ESP-NOW→LoRa silence forward.
- Recovery: `ensureLoRaReceiveMode` (L1680), watchdog full reset after 120 s without RX (L2030).
- Airtime of one 108-byte aggregate at SF9/125 kHz/CR4-5: **≈ 595 ms** (≈ 185 ms at SF7).

### 3.2 ESP-NOW layer (hub side)
- `setupEspNow` (L2884): ONSHORE = STA + LR protocol + 21 dBm; others AP+STA. Broadcast peer only.
- `onEspNowRecv` (L2962) → `processEspNowMessage` (L2969) **runs inside the WiFi task
  callback** and from there can start LoRa transmissions: `sendLoRaAggregateWithRetry(3)`
  (L3037), `relayLoRaAlert` (L3101), `loRaTransmitWithWait` (L3151). RSSI is passed as 0
  (L2966): the core-2.x receive callback gives no RSSI.
- Hub → node ACK: only for `MSG_ALERT`, sent as **broadcast** by every hub that accepts it (L3080-3092).

### 3.3 State and dedup
- `NetworkState network` (L226), `NodeState` (`messages.h` L233-253), `MAX_NODES` 16;
  slot 0 = self.
- **State dedup** `shouldAcceptMessage` (L3230): per node `last_seq` + `last_uptime`;
  FISH_ON always accepted, clears need a newer seq + hold time or 2 confirmations, reboot
  detection by lower uptime / `FLAG_FIRST_BOOT`.
- **Relay dedup** `isDuplicateForRelay` / `recordForDedup` (L3385 / L3398): one global ring of
  32 `(node_id, seq, time)` entries, 30 s validity (`messages.h` L268-275).
- **Config dedup**: separate 8-entry ring `(origin_id, config_seq)`, 60 s (L2193-2210).
- Timeouts: `loopNodeTimeout` (L3463), `NODE_TIMEOUT_SEC` 90 s; for nodes heard directly only
  the direct ESP-NOW timestamp counts (`last_direct_seen`, L3487-3495).

## 4. Message types (`messages.h` L55-69)

| Type | Struct (bytes) | Radio | Originated by | Relayed by | Dedup used |
|---|---|---|---|---|---|
| `MSG_STATUS` 0x01 / `MSG_HEARTBEAT` 0x03 | `SensorMessage` (12) | ESP-NOW | sensor (L691) | not individually — goes into the aggregate | `shouldAcceptMessage`; also written to relay cache (L3049) though never relayed |
| `MSG_ALERT` 0x02 | `AlertMessage` (16) | ESP-NOW | sensor (L726) | hub → `MSG_LORA_ALERT` (L3099-3101) | state + relay cache `(node, seq)` |
| `MSG_ACK` 0x04 | `AckMessage` (6) | ESP-NOW bcast | hub (L3083-3090) | – | – |
| `MSG_SILENCE_SYNC` 0x05 | `SilenceSyncMessage` (12) / `LoRaSilenceSyncMessage` (16) | both | `sendSilenceSync` (L5560) | LoRa hop<3 (L2485), ESP-NOW re-broadcast by gateways (L2470) | relay cache `(origin, timestamp→u16)` |
| `MSG_LORA_AGGREGATE` 0x40 | `LoRaAggregateMessage` (108) | LoRa | ONSHORE/OFFSHORE `sendLoRaAggregate` (L2685) | **every LoRa node, any role**, hop<3 (L2368) | relay cache `(NETWORK_ID, NETWORK_ID<<8 \| lora_sequence)` |
| `MSG_LORA_ALERT` 0x41 | `LoRaAlertMessage` (18) | LoRa | `relayLoRaAlert` (L2852) ← ESP-NOW alert / local reed (L3640) | RELAY + SENSOR_LORA only (L2419) | relay cache `(origin, seq)` |
| `MSG_LORA_DISCOVERY` 0x42 | (8) | – | defined, never sent | | |
| `MSG_CONFIG_UPDATE` 0x50 | `ConfigUpdateMessage` (17; comment says 24) | LoRa | `sendRemoteConfig` (L5596) ← web (L4831) | all, hop<3 (L2612) | config ring `(origin, config_seq)` |
| `MSG_CONFIG_ACK` 0x51 | `ConfigAckMessage` (12) | LoRa | target (L2592) | all except origin, hop<3 (L2664) | **none** |
| `MSG_PING`/`MSG_PONG` 0x80/0x81 | – | – | defined, unused | | |
| `MSG_RESET_CMD` 0x90 | `ResetCmdMessage` (8) | both | `sendResetAllCommand` (L5639) | not relayed | – |

## 5. Where TX / RX / relay / dedup happen (quick index)

| Concern | Location (lora_node.ino unless noted) |
|---|---|
| LoRa RX | ISR L2063 → `loopLoRa` L2121 → `processLoRaMessage` L2212; pre-TX drain L1766 |
| LoRa TX (own) | `loRaTransmitPacket` L1889 — aggregates L2819, alerts L2877, silence L5591, config L5628, config ACK L2607, reset L5665 |
| LoRa TX (relay) | `loRaTransmitWithWait` L1815 — aggregate L2382, alert L2429, silence L2495 (+ config relay L2627 / ACK relay L2676 via `loRaTransmitPacket`) |
| ESP-NOW RX | hub L2962/L2969 (WiFi task); sensor `sensor_node.ino` L778 |
| ESP-NOW TX | sensor `espNowSendReliable` L536; hub ACK L3090, silence L2480/L5573, reset L5658 |
| Relay dedup | `isDuplicateForRelay` L3385, `recordForDedup` L3398; config L2193/L2205 |
| State dedup | `shouldAcceptMessage` L3230, `updateNodeState` L3406 |
| Aggregate build | `sendLoRaAggregate` L2685 (self slot 0, rotation, "authoritative only" filter L2765) |
