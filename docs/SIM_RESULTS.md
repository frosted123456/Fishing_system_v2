# Mesh simulation: 1 chalet + 10 hubs + 30 tip-ups + 4 sonars (2026-10-06)

`tools/sim/mesh_sim.cpp` runs the **real protocol code** (lib/IceMesh: ChaletRole, HubRole, planner,
line table, sonar chain, codec, store) for 1 chalet, 10 hubs, 3 tip-up nodes per hub and 4 sonar holes,
30 min per run, 5 seeds per row (≈ 570 flag trips per row). Only the world around it is simulated.

**What this tells you:** whether the protocol logic and the code hold together with 10 hubs, losses,
relays, failures and load. **What it does not tell you:** real range or real packet loss — every radio
number below is an estimate until the range test.

## Model (all estimates)
| Part | Model |
|---|---|
| Path loss | 31.7 dB at 1 m + 35·log10(d) (near-ground, people, snow), shadowing σ 4 dB per link, chalet antenna +6 dB, 20 dBm, 2 dBi antennas |
| Obstacles | hub 8 behind a ridge (+32 dB to the chalet), hub 9 behind an island (+30 dB), hub 10 at 1.4 km (+6 dB) |
| Per packet | fading σ 3 dB; success from 10 % to 95 % over ~3 dB around the theoretical sensitivity; burst outages 25 dB (person/snowmobile at an antenna); collisions with 6 dB capture; half duplex |
| Timing | hub clocks ±20 ppm (hostile ±50), RxDone jitter ~10 µs, TX start latency 150-400 µs; a packet counts only inside the receiver's window (3-symbol preamble tolerance, 2 ms RX extension) |
| Tip-ups (ESP-NOW) | v1 sensor behaviour: alert at once + every 5 s while tripped, flag reset seen at the next 5 s wake, heartbeat 60 s, 5 tries per send; hub marks a node offline after 90 s (lora_node config); trips every ~6 min per hole, flag up 20-90 s |
| Sonar | real SonarSource on 4 holes (one behind the ridge, relayed), FOCUS switched every 2 min, 1 ESP-NOW try per block |

Link margins at SF9/500 (mean, no fading; chalet = C):
| hub | dist to C (m) | margin to C (dB) | best hub link (dB) |
|---|---|---|---|
| 1 | 206 | 41.7 | 33.4 (hub 2) |
| 2 | 370 | 31.5 | 38.1 (hub 6) |
| 3 | 506 | 31.5 | 37.6 (hub 4) |
| 4 | 653 | 19.4 | 37.6 (hub 3) |
| 5 | 391 | 30.7 | 32.2 (hub 1) |
| 6 | 541 | 27.3 | 38.1 (hub 2) |
| 7 | 806 | 16.6 | 37.1 (hub 8) |
| 8 | 949 | -13.6 | 37.1 (hub 7) |
| 9 | 446 | -7.1 | 21.4 (hub 5) |
| 10 | 1401 | 7.6 | 20.7 (hub 7) |

## Scenarios
| Scenario | Conditions |
|---|---|
| baseline | as above |
| degraded | +10 dB loss on every link, 5× more burst outages, ESP-NOW 50 % per try |
| storm | 10 holes trip within 5 s at 12 min, FOCUS rotating incl. the relayed sonar |
| failures | hub 3 cut from the chalet 10-13 min, hub 7 (a relay) reboots at 16:40, hub 7 carried out of range from 25 min |
| hostile | 20 % extra packet loss everywhere, ESP-NOW 50 %, clocks ±50 ppm |
| small6 / small6_degr | only hubs 1-6 powered (all direct), baseline / degraded |
| no_tx_miss | hostile, but a hub never transmits after missing a beacon |

| Variant | What |
|---|---|
| current | firmware as committed (12 slots per beacon, SF9/500, 1 s frame, 96 B slots, adaptive off) |
| fixA | 16 slots + adaptive SF (SF8/SF7 where the margin allows) + node offline after 150 s + 2 ESP-NOW tries per sonar block |
| fixB | 16 slots + **2 s frame, SF9 only** + node offline after 150 s + 2 ESP-NOW tries per sonar block |

## Results
"missed" = the chalet never showed FISH ON during the trip; "alert" = flag up → FISH ON at the chalet;
"clear" = flag down → back to normal; "frames with all hubs" = beacons that gave every powered hub a slot;
"hub delivery" = scheduled hub slots that reached the chalet (direct or relayed).

| scenario | variant | trips | missed | alert p50/p95/max (s) | clear p95 (s) | false offline (node-min) | frames with all 10 hubs | hub delivery | FOCUS pings | sonar delay p95 (s) | BASE age avg/max (s) | timing losses | max sync err (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline | current | 572 | 3 | 1 / 12 / 44 | 17 | 3.2 | 0 % | 94 % | 22 % | 3 | 6.2 / 135 | 3 | 62 |
| degraded | current | 567 | 10 | 2 / 19 / 58 | 20 | 84.1 | 0 % | 81 % | 11 % | 3 | 18.2 / 358 | 10 | 66 |
| storm | current | 578 | 1 | 1 / 14 / 37 | 17 | 5.2 | 0 % | 95 % | 23 % | 3 | 6.2 / 135 | 3 | 62 |
| failures | current | 560 | 3 | 1 / 13 / 60 | 17 | 32.4 | 0 % | 94 % | 23 % | 3 | 6.1 / 135 | 3 | 62 |
| hostile | current | 575 | 0 | 2 / 16 / 49 | 15 | 61.6 | 0 % | 69 % | 12 % | 3 | 13.6 / 141 | 695 | 63 |
| small6 | current | 349 | 0 | 1 / 1 / 1 | 6 | 0.0 | 99 % | 100 % | 83 % | 1 | 2.0 / 19 | 0 | 61 |
| small6_degr | current | 336 | 0 | 1 / 1 / 10 | 5 | 36.8 | 100 % | 98 % | 48 % | 1 | 4.9 / 39 | 0 | 63 |
| no_tx_miss | current | 561 | 5 | 2 / 16 / 46 | 21 | 41.1 | 0 % | 66 % | 10 % | 3 | 14.9 / 134 | 0 | 63 |
| baseline | fixA | 564 | 0 | 1 / 1 / 5 | 6 | 0.0 | 97 % | 94 % | 83 % | 2 | 1.8 / 35 | 33 | 68 |
| degraded | fixA | 562 | 0 | 1 / 6 / 56 | 9 | 0.5 | 60 % | 90 % | 34 % | 2 | 4.9 / 95 | 31 | 62 |
| storm | fixA | 575 | 0 | 1 / 1 / 5 | 6 | 0.0 | 97 % | 94 % | 82 % | 2 | 1.8 / 35 | 48 | 68 |
| failures | fixA | 570 | 1 | 1 / 1 / 16 | 6 | 17.1 | 88 % | 94 % | 79 % | 2 | 1.9 / 58 | 19 | 68 |
| hostile | fixA | 570 | 2 | 1 / 12 / 39 | 15 | 0.8 | 7 % | 70 % | 18 % | 3 | 7.5 / 100 | 863 | 71 |
| small6 | fixA | 342 | 0 | 1 / 1 / 1 | 5 | 0.0 | 99 % | 100 % | 97 % | 0 | 1.5 / 15 | 0 | 63 |
| small6_degr | fixA | 347 | 0 | 1 / 2 / 11 | 6 | 0.5 | 100 % | 97 % | 72 % | 0 | 2.6 / 23 | 0 | 64 |
| no_tx_miss | fixA | 566 | 3 | 2 / 14 / 42 | 15 | 0.6 | 6 % | 65 % | 16 % | 3 | 8.4 / 129 | 0 | 66 |
| baseline | fixB | 577 | 0 | 1 / 3 / 10 | 7 | 0.0 | 99 % | 94 % | 92 % | 0 | 3.7 / 88 | 73 | 61 |
| degraded | fixB | 554 | 0 | 1 / 6 / 51 | 7 | 2.0 | 98 % | 92 % | 64 % | 2 | 7.7 / 102 | 20 | 66 |
| storm | fixB | 583 | 0 | 1 / 3 / 28 | 7 | 0.0 | 99 % | 94 % | 92 % | 0 | 4.0 / 104 | 63 | 61 |
| failures | fixB | 559 | 2 | 1 / 3 / 12 | 6 | 3.5 | 92 % | 94 % | 91 % | 1 | 3.7 / 88 | 49 | 64 |
| hostile | fixB | 577 | 0 | 2 / 6 / 19 | 8 | 1.5 | 99 % | 73 % | 54 % | 1 | 8.9 / 72 | 401 | 61 |
| small6 | fixB | 335 | 0 | 1 / 2 / 2 | 6 | 0.0 | 99 % | 100 % | 96 % | 0 | 3.0 / 22 | 0 | 58 |
| small6_degr | fixB | 348 | 0 | 1 / 2 / 10 | 6 | 0.0 | 99 % | 98 % | 71 % | 0 | 5.6 / 44 | 1 | 66 |
| no_tx_miss | fixB | 574 | 0 | 2 / 6 / 21 | 10 | 0.5 | 99 % | 67 % | 55 % | 2 | 9.1 / 126 | 0 | 71 |

## Findings
| # | Finding | Evidence | Severity |
|---|---|---|---|
| S1 | **10 hubs do not fit** in the current plan: 12 slots per beacon < 10 hubs + JOIN + echo slots, and at SF9/500 a 1 s frame holds ~6 hubs of 96 B (beacon ~120 ms, each echo ~125 ms, each hub slot ~140 ms). The planner drops the last slots every frame → those hubs starve, fall back to JOIN, get misclassified as remote → cascade | current: 0 % frames with all hubs, alert p95 12-19 s, max ~1 min, up to 10 missed alerts / 570, FOCUS 11-23 % | **blocking for 10 hubs** |
| S2 | With **6 direct hubs the current firmware works**: alert p95 1 s, max 1 s (10 s degraded), no misses | small6 rows | ok |
| S3 | **Node shown offline after one lost heartbeat**: node heartbeat 60 s vs hub timeout 90 s | false offline 37-84 node-min per 30 min in degraded rows; 0-2 with 150 s | fix: timeout ≥ 2.5 heartbeats |
| S4 | **Sonar FOCUS limited by ESP-NOW** (1 try per block): ≤ 85 % even on perfect LoRa | small6 current 83 % → 96-97 % with 2 tries | fix: 1 retry |
| S5 | Timing has **ample margin**: worst sync error 60-70 µs vs 3 ms guards; TX starts ~3.1 ms into the window | max sync err column | ok (to confirm on hardware, O8) |
| S6 | A hub that **missed a beacon still sends on its old plan**; when the plan changed it lands in someone else's slot ("timing losses" 400-860 in hostile). Forbidding it (no_tx_miss) removes those but gives no better alerts | hostile vs no_tx_miss rows | keep as is (low impact) |
| S7 | A hub **at the edge of range** (hub 10 in degraded, margin ≈ −2 dB) or flipping between direct and relay: its alerts take 10-50 s in ~2 % of trips | degraded max 51-56 s | physics + placement; relay stickiness tried: no gain |
| S8 | Adaptive SF alone (fixA) is excellent in good conditions (p95 1 s, FOCUS 83 %) but **collapses with uniform loss** (hostile: 7 % frames with all hubs) — faster modes don't help when every link is bad | fixA hostile row | fixA is not robust alone |
| S9 | **fixB (2 s frames, SF9 only) is robust everywhere**: no missed alert except hubs that are off/out of range, p95 3-6 s, max ≤ 51 s (edge hub), FOCUS 54-92 % | fixB rows | recommended direction |
| S10 | Silence (beacon flag) reached every hub within 1 frame; relay echo timing within 70 µs | detail output | ok |
| S11 | Chalet firmware node list holds **16 nodes** (v1 `MAX_NODES`): 10 hubs × 3 tip-ups = 30 → the web page would drop nodes (the library's 32-node table is fine) | code inspection, not simulated | to fix with 10 hubs |

## Recommendation (to decide)
1. MAX_SLOTS 16 (beacon +20 B) — needed for 10 hubs in any variant.
2. Frame length chosen by the chalet: **1 s while the plan fits, 2 s when it does not** (beacon already carries the frame length; hubs follow it). Gives small6 latency (p95 1 s) and fixB robustness at 10 hubs. Adaptive SF stays optional.
3. Hub node timeout 150 s (2 missed heartbeats + margin), or node heartbeat 30 s.
4. One ESP-NOW retry for sonar blocks.
5. Chalet node list 16 → 32.

Re-run: `g++ -std=c++11 -O2 -Ilib/IceMesh/src tools/sim/mesh_sim.cpp -o mesh_sim && ./mesh_sim all 5 30 current`
(fixA/fixB: add `-DICEMESH_MAX_SLOTS=16` and pass `fixA` / `fixB`). `SIM_DETAIL=1` per-hub delivery, `SIM_LATE=1` slow alerts, `SIM_MARGINS=1` link table.
