# Sonar test mode (fake sonar data)

Tests the whole sonar chain without a TUSS4470: fake raw pings -> node processing -> codec ->
ESP-NOW -> hub outbox -> TDMA slots / relay -> chalet store -> phone page.
**All data is simulated.** Scene and processing come from Frank's display prototype
(`docs/prototype/sonar_display_prototype.html`): "Real recordings will look different. Use this to
tune the display, not to trust the numbers."

## Chain
| Stage | Code | Notes |
|---|---|---|
| Raw pings | `sonar_scene.h` (port of the prototype `genPing`) | 488 bins × 3 frequencies (190/200/210 kHz), 8-bit log amplitude. Prototype hole: 18.5 ft, lure at 15 ft, weeds, perch, walleye strike, crossing fish, 48 s loop. Other holes: same story mapped to bottom 3.5-6.5 m, bait 0.6-1.6 m above it, sand or mud, own time offset |
| Processing v0 | `sonar_proc.h` (port of the prototype `Proc`) | bottom lock, noise floor, range compensation, auto contrast, peak fit, tracker + labels (Fish / Bait / Near bottom / Cover), static scene, bottom hardness, flicker / frequency spread / echo length. **Bit-exact with the prototype** in double precision (test_sonar_proc); the float build used on the ESP32 matches at print precision |
| Blocks | `sonar_sim.h` (SonarSource) + `sonar_codec.h` | targets (≤ 5, slot 0 = bait), changed cells (≤ 7), 4-level static scene, hardness, noise floor, echo character every 2 s |
| Display | `/sonar` (main.cpp `SONAR_PAGE`) | prototype layout: flasher, 60 s waterfall + overlays, bottom lock, readouts, "Under the hole now", echo character; hole grid on top |

On a real sonar node only the first stage changes (TUSS4470 capture instead of SonarScene).

## Switch it on
| Where | How |
|---|---|
| Chalet web | `/sonar` → "Sonar test mode" switch; tap a hole to stream it (also linked from the main page footer) |
| Chalet serial | `SONAR ON` / `SONAR OFF`, `FOCUS <node>` / `FOCUS OFF` |
| Chalet API | `POST /api/sonar {"sim":true,"focus":144}` |
| Hub serial | `SIMNODES 0-4` virtual sonar holes on this hub (default 2, saved); `SIMNODES` alone = CPU cost per fake ping |
| Node build flag | `SONAR_SIM_ALLOWED=0` → node ignores the test mode; `SONAR_SIM_FORCE=1` → fake sonar from boot (bench, no chalet) |
The chalet setting is saved (NVS key `sonarSim`) and survives a reboot — turn it off after testing.

## What each module does in test mode
| Module | Behaviour |
|---|---|
| Chalet | `BF_SONAR_SIM` in every beacon; FOCUS hole in the beacon; the hub reporting it gets a 180 B slot |
| Hub | runs N virtual holes (IDs 128 + (hub & 0x0F) × 8 + k); queues blocks from real nodes (`MSG_SONAR`); broadcasts `MSG_SONAR_CTRL` every 1 s and right after a node transmits |
| Node (WROOM / C3) | listens 120 ms after each wake-up TX; test mode on = **stays awake** and runs the chain at 4 pings/s; reed/alerts/heartbeat keep working; back to deep sleep 10 s after "off" (or 30 s without control). Debug serial prints the CPU cost per ping every minute |
| Phone `/sonar` | ft by default (ft/m toggle remembered on the phone), light/dark from the phone setting |

## Differences from the prototype page (on purpose)
| Prototype | /sonar | Why |
|---|---|---|
| Full pixel column per ping (raw or processed) | static scene (4 grey levels) + targets + changed cells, rebuilt on the phone | the radio carries ~35 B/s per FOCUS hole, not 1952 B/s |
| Echo shape at 190/200/210 kHz (trace) | not shown; flicker / frequency change / echo length come from the hole every 2 s | ~110 B per update would exceed the FOCUS budget |
| "dB vs bait" from range-compensated dB | from the 5-bit display value (inverse colour curve, 46 dB span) | approximation, ±1-2 dB (est.) |
| Processing switches, Raw/Processed, Quiet receive, Jig, Sand/Mud, Restart | not on /sonar | they act on the hole's processing / the simulator, not on the display |
| Bait line = fixed 15 ft | median depth of the bait track (slot 0) | the chalet does not know the lure depth |
| Google font | system fonts | the chalet Wi-Fi has no internet |

## Measured (fake data, PC)
| Item | Value |
|---|---|
| FOCUS stream | ≈ 7.3 B/ping, ≈ 35 B/s incl. static scene, echo character and section headers (budget 60) |
| BASE summary | 8-9 B every 4 s ≈ 2.5 B/s per hole |
| Remote hole through a relay (test_tdma_sim) | 98 % / 83 % / 50 % of pings at 0 / 10 / 20 % loss per link; line alert 1 / 2 / 5 frames |
| Processing cost | 0.07 ms/ping on the PC; **ESP32 not measured** (estimate a few ms per ping on S3/WROOM with FPU; the C3 has no FPU, slower) — check with `SIMNODES` on a hub and the node debug output |
| RAM | ~15 KB per fake hole + ~25 KB shared scratch (heap, first use) |

## Found while porting (simulation, not real water)
- Deeper than ~6.5 m the simulated lure stays under the prototype's 10 dB detection threshold and is
  never labelled Bait; fish are still tracked. The real threshold must come from recordings.
- The bottom is reported ~5 cm shallower than the scene (the -10 dB onset rule) — same in the prototype.

## Desktop preview (no hardware)
```
mkdir -p .preview
g++ -std=c++11 -O2 -Ilib/IceMesh/src tools/sonar_trace.cpp -o .preview/sonar_trace
.preview/sonar_trace 300 > .preview/trace.json
python3 tools/sonar_preview.py          # http://localhost:8000/sonar
```
The trace uses the real chain, codec and store; the page is read from main.cpp (`SONAR_PAGE=file.html` to try a page file).
Reference trace of the prototype: `node tools/proto_reference.js > test/fixtures/proto_ref.txt`.

## Not done / to check on hardware
- TUSS4470 capture into the chain (3-frequency compounding needs 3 pulses per ping, or run with 1 frequency).
- Per-hole bait depth setting (for the Bait label) — today the scene's bait depth is used.
- ESP-NOW load, CPU and heap on the boards; chalet node list holds 16 nodes (virtual holes count).
