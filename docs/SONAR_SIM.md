# Sonar test mode (fake sonar data)

Lets you test the whole sonar chain — codec, ESP-NOW, hub outbox, TDMA slots, relay, chalet
decoding, phone display — without a TUSS4470. **All data is invented** (my own scene: the
prototype from the other conversation was not available to me). It does not model real fish,
real TUSS4470 output, or real water.

## Switch it on
| Where | How |
|---|---|
| Chalet web | `/sonar` → "sonar test mode" checkbox (also linked from the main page footer) |
| Chalet serial | `SONAR ON` / `SONAR OFF`, `FOCUS <node>` / `FOCUS OFF` |
| Chalet API | `POST /api/sonar {"sim":true,"focus":144}` |
| Hub serial | `SIMNODES 0-4` virtual sonar nodes on this hub (default 2, saved in NVS) |
| Node build flag | `SONAR_SIM_ALLOWED=0` → node ignores the test mode; `SONAR_SIM_FORCE=1` → fake sonar from boot (bench, no chalet) |
The chalet setting is saved (NVS key `sonarSim`) and survives a reboot — turn it off after testing.

## What happens
| Module | In test mode |
|---|---|
| Chalet | sets `BF_SONAR_SIM` in every beacon; FOCUS node in the beacon; FOCUS hub gets a 180 B slot |
| Hub | runs N virtual sonar nodes (IDs 128 + (hub & 0x0F) × 8 + k, shown as holes on the chalet); queues blocks from real nodes (`MSG_SONAR`); broadcasts `MSG_SONAR_CTRL` every 1 s and right after a node transmits |
| Node (WROOM / C3) | listens 120 ms after each wake-up TX; if the control says "on", **stays awake** (no deep sleep) and sends a summary every 4 s, or DATA + background when it is the FOCUS; reed/alerts/heartbeat keep working; back to deep sleep 10 s after "off" (or 30 s without control) |
| Phone `/sonar` | grid of holes (fish count, nearest fish, bottom, activity, line state from `/api/status`); tap = FOCUS; waterfall + flasher of the FOCUS hole |

Battery: a node in test mode never sleeps (tens of mA, est.) — use it on the bench or for short field tests.

## Fake scene (lib/IceMesh/src/sonar_sim.h, seeded per node)
Bottom 2.5-9.5 m with a ±4 cm swell, weeds up to 45 cm, bait (track 0) 30-120 cm above bottom
jigged every 5-10 s, up to 2 fish (tracks 1-7) arriving about every 30 s per free slot, hovering
10-40 s near the bait then leaving, noise cells, rare loss of bottom lock.

## Display
- Waterfall: newest ping on the right, 2 px per ping (4 pings/s); dim colours = background profile
  (shifted to each ping's bottom), green/orange/red = weak/medium/strong, white = bait, pink line = bottom.
- Flasher: dial 0 at the top, clockwise, one turn = display range (auto: bottom + 15 %, whole metres).
- Delay: ≈ 1-2 s (1 block per frame + 700 ms polling + playback queue).

## Desktop preview (no hardware)
```
mkdir -p .preview
g++ -std=c++11 -O1 -Ilib/IceMesh/src tools/sonar_trace.cpp -o .preview/sonar_trace
.preview/sonar_trace 300 > .preview/trace.json
python3 tools/sonar_preview.py          # http://localhost:8000/sonar
```
The trace is made by the real codec and store; the page is taken from `main.cpp`.

## Measured in the PC simulation (test/test_tdma_sim, fake data, est. radio model)
| Loss per link | FOCUS pings received from a REMOTE hub (via relay) | Line alert while streaming |
|---|---|---|
| 0 % | 98 % (the last block is still in flight) | 1 frame |
| 10 % | 83 % | 2 frames |
| 20 % | 50 % (no ACK on sonar; 3-4 links in series) | 5 frames |

## Not done / to check on hardware
- Real node-side processing (bottom lock, tracker, background separation) for the TUSS4470.
- Pixel layer (review fix: first ping vs background, later pings vs previous ping in the block).
- ESP-NOW load: a FOCUS node sends ~2 frames/s; several nodes in BASE ~1 frame/4 s each — fine on paper, not measured.
- Chalet node list holds 16 nodes (v1 `MAX_NODES`): virtual nodes count against it.
