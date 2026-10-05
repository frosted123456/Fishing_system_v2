# Phase 1 — commit plan (APPROVED 2026-10-05, revised with decisions D1-D7)

Progress 2026-10-05: C1-C8 committed. 47 host tests green (g++ -std=c++11 -pedantic -Werror, and under ASan/UBSan).
C2/C3 not compiled for ESP32 yet.

Branch: `v2/phase1-mesh-core`. Each commit is small; from C4 on the host tests stay green.
Tip-up ESP-NOW behaviour and deep sleep are untouched in phase 1.

| # | Commit | What changes | Fixes | Status |
|---|---|---|---|---|
| C1 | `docs: decisions + revised plan` | decisions.md, this plan, protocol updates | – | done |
| C2 | `build: migrate to a PlatformIO project` | `platformio.ini` (envs hub / cabin / relay / sensor_lora on Heltec V3, sensor_c3, sensor_wroom, native); sketches moved to `src/lora_node/main.cpp` and `src/sensor_node/main.cpp` (`.ino` → `.cpp` + function prototypes, no logic change); one `include/messages.h`; `NODE_ID` / `NODE_NAME` / `NODE_ROLE` overridable from `build_flags`. | – | done, **needs your compile** |
| C3 | `fix(lora): aggregate relay dedup keyed by origin hub` | relays keep `sender_id` (= origin hub); key = (origin hub, `lora_sequence`) in its own namespace. | issue 1 | done, **needs your compile** |
| C4 | `test: host test harness` | `[env:native]` for `pio test -e native` (Unity) + a Unity-compatible shim and Makefile so tests also run with plain g++ (my verification path). | – | done, host tests green |
| C5 | `feat(mesh): v2 header encode/decode` | `lib/IceMesh/mesh_hdr.h` + tests | – | done, host tests green |
| C6 | `feat(mesh): per-origin dedup window` | `dedup_window.h` + tests | issue 3 | done, host tests green |
| C7 | `feat(mesh): stream newest-wins filter` | `stream_filter.h` + tests | issue 3 | done, host tests green |
| C8 | `feat(mesh): priority TX queue + airtime` | `tx_queue.h`, `airtime.h` + tests | issue 4 (design) | done, host tests green |
| C9 | `refactor(lora): ESP-NOW callback only enqueues` | RX ring; processing from `loop()` | finding A | next, after C2/C3 compile OK |
| C10 | `refactor(lora): non-blocking radio driver` | RX→CAD→TX→RX state machine on RadioLib `startTransmit()`; all TX through the queue; no `delay()` in relay/retry paths; LBT always | issue 4, finding B | next |
| C11 | `feat(lora): v2 header + reliable class` | v2 header on all LoRa packets, v1 LoRa formats dropped (D1); variable-length aggregates; dedup window on relays; alerts with origin = node; cabin ACK for alerts/config, originator retries 2/4/8 s; aggregates no ACK (D5); cabin never relays (D6) | issues 1-3, 6 (alerts), finding D | next |
| C12 | `feat(lora): stream class plumbing` | STREAM type, newest-wins relay, P3 queue level, 20 % airtime cap at SF9, `TEST_STREAM_SOURCE`, airtime counters | stream | next |
| C13 | `docs: as built + bench checklist` | | | next |

Why the split: C2/C3 change firmware that I cannot compile from here (network policy blocks the
ESP32 toolchain download). C9-C12 are large firmware changes, so they start once the compile
loop works (your PlatformIO, or the domains allowed for me).

## Not in phase 1 (on purpose)
Serving hub, overhear suppression, parent selection and failover (phase 2). Issue 2 (flooding)
and issue 6 for aggregates are mitigated in phase 1 (dedup, jitter, CAD, cabin silent), not
solved. Issue 5 (sensor hub choice) is phase 2 because it touches the tip-up firmware.

## Bench checklist (after C10 / C11 / C12)
1. 1 tip-up + 1 hub + cabin: alert → cabin buzzer, ACK back, no duplicate in alert history.
2. 2 hubs hearing the same tip-up: one alert at the cabin, not two.
3. Hub A out of cabin range (antenna off / attenuator), hub B in range: A's aggregates reach the cabin via B.
4. Hub reboot during a session: its packets are accepted again immediately (restart rule).
5. `TEST_STREAM_SOURCE` at SF9: aggregates and alerts still arrive on time; airtime counters ≈ protocol_v2.md §6/§7b.
6. A board still on v1 (if any) ignores v2 packets and does not crash.
