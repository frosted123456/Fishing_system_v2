# Phase 1 — proposed commits (WAITING FOR APPROVAL — nothing below is coded yet)

Branch: `v2/phase1-mesh-core`. Each commit is small, keeps **both sketches compiling** and,
from P3 on, keeps the host tests green. Sensor-node ESP-NOW behaviour and deep sleep are
untouched in phase 1.

| # | Commit | What changes | Fixes | Depends on |
|---|---|---|---|---|
| P1 | `build: compile check + shared-header check` | `tools/build.*` (arduino-cli, pinned core + lib versions), `tools/check_shared.*` (the two `messages.h` stay identical). Baseline compile recorded. No firmware change. | – | Q5 (versions) |
| P2 | `fix(lora): aggregate relay dedup keyed by origin hub` | v1 wire format kept. Relays stop overwriting `sender_id` (it stays = origin hub); relay key = (origin hub, `lora_sequence`) in its own namespace; own TX recorded with own ID. Flashable on its own before v2. Side effect (wanted): BUG FIX #13 refreshes the real origin, not the relay. | issue 1 | – |
| P3 | `test: host test harness + common/ folder` | `common/` (pure C++, no Arduino headers), `test/` with a tiny assert header and a Makefile (`g++ -std=c++17 -Wall -Wextra -Werror`). | – | Q10 (where shared code lives) |
| P4 | `feat(common): v2 header encode/decode` | `mesh_hdr.h`: 8-byte header, ctl bit fields, serial-number seq compare + tests (round-trip, bit packing, hop limit, wrap). | – | P3 |
| P5 | `feat(common): per-origin dedup window` | `dedup_window.h` (highest + bitmap, LRU table, idle expiry) + tests: in-order, duplicate, reorder inside window, `d ≤ −32` restart, 65535→0 wrap, jump ≥ 32, table full, idle expiry. | issue 3 (reliable) | P4 |
| P6 | `feat(common): stream newest-wins filter` | `stream_filter.h` + tests: duplicate, older, wrap, restart, many origins. | issue 3 (stream) | P4 |
| P7 | `feat(common): priority TX queue` | `tx_queue.h` fixed memory, 4 priorities, supersede key, TTL, `not_before`, drop-oldest, stream airtime bucket + tests: priority order, supersede, TTL expiry, backoff, no starvation of P2 by P3. | issue 4 (design) | P4 |
| P8 | `refactor(lora): ESP-NOW callback only enqueues` | RX ring buffer filled by `onEspNowRecv`; `processEspNowMessage` runs from `loop()`. Same behaviour, no LoRa TX from the WiFi task any more. | finding A | P1 |
| P9 | `refactor(lora): non-blocking radio driver` | state machine RX → CAD → TX → RX with `startTransmit()` / TX-done IRQ; CAD always (drop the "skip LBT after RX" rule); every TX call site pushes to the queue; `delay()` removed from relay paths and from `sendLoRaAggregateWithRetry` (becomes queued copies with spacing). Reset/reboot and UI overlay delays left as they are. | issue 4, finding B | P7, P8, Q5 |
| P10 | `feat(lora): v2 header + reliable class on LoRa` | all LoRa packets carry the v2 header; dispatcher on `ver`; reliable relays use the dedup window; config-ACK deduped; alerts injected with origin = node and the node's seq (duplicates from several hubs collapse); cabin ACKs alerts/config; originator retry 2/4/8 s. v1 compatibility per Q1. | issues 1-3, 6 (alerts), finding D | P5, P9, Q1, Q7, Q8 |
| P11 | `feat(lora): stream class plumbing` | STREAM type, newest-wins relay, P3 queue level, `#define TEST_STREAM_SOURCE` synthetic 4 Hz source for bench tests, airtime-per-class counters in serial + `/api/status`. No sonar code. | issue 2/3 for stream | P6, P10 |
| P12 | `docs: protocol_v2 as built + bench checklist` | update the doc to match the code; bench test list below. | – | all |

## Not in phase 1 (on purpose)
Serving hub, overhear suppression, parent selection and failover (phase 2) — so issue 2
(flooding) and issue 6 for aggregates are only *mitigated* in phase 1 (dedup, jitter, CAD), not
solved. Issue 5 (sensor hub choice) is phase 2 because it touches the tip-up firmware.

## Bench checklist (for you, after P9 / P10 / P11)
1. 1 tip-up + 1 hub + cabin: alert → cabin buzzer, ACK back, no duplicate entry in alert history.
2. 2 hubs hearing the same tip-up: one alert at the cabin, not two.
3. Hub A out of cabin range (antenna off / attenuator), hub B in range: A's aggregates reach the cabin via B.
4. Hub reboot during a session: its packets are accepted again immediately (restart rule).
5. `TEST_STREAM_SOURCE` at 4 Hz: aggregates and alerts still arrive on time; airtime counters match §6 of protocol_v2.md.
6. (If Q1 = mixed fleet) a v1 board next to v2 boards ignores v2 packets and does not crash.

## How I will verify each commit
- Host tests: `make -C test` with g++ (available in your Cowork shell).
- Firmware: **I cannot compile the sketches right now.** On 2026-10-05 the network policy blocked
  the ESP32 core downloads (downloads.arduino.cc, espressif.github.io, PlatformIO registry) from both
  my cloud workspace and your Cowork shell. Options: (a) you compile each commit in your Arduino IDE
  and send me the error log, or (b) an org admin allows those domains (Admin settings → Capabilities)
  and I compile with arduino-cli myself. I cannot flash or radio-test — the bench checklist is yours.
