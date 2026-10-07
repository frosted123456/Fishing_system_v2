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
| Display | Sonar tab of the chalet web suite (main.cpp `SUITE_PAGE`, docs/WEB_SUITE.md) | prototype layout: flasher, 60 s waterfall + overlays, bottom lock, readouts, "Under the hole now", echo character; hole grid on top |

On a real sonar node only the first stage changes (TUSS4470 capture instead of SonarScene).

## Switch it on
| Where | How |
|---|---|
| Chalet web | Sonar tab (or Settings tab) → "Sonar test mode" switch; tap a hole to stream it, or Watch on the Holes tab |
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
| Phone, Sonar tab | ft by default (ft/m toggle remembered on the phone), light/dark from the phone setting |

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
python3 tools/web_preview.py            # http://localhost:8000/sonar
```
The trace uses the real chain, codec and store; the page is read from main.cpp (`SUITE_PAGE=file.html` to try a page file).
Reference trace of the prototype: `node tools/proto_reference.js > test/fixtures/proto_ref.txt`.

## Not done / to check on hardware
- TUSS4470 capture into the chain (3-frequency compounding needs 3 pulses per ping, or run with 1 frequency).
- Per-hole bait depth setting (for the Bait label) — today the scene's bait depth is used.
- ESP-NOW load, CPU and heap on the boards; chalet node list holds 16 nodes (virtual holes count).

## Simulation per hole (2026-10-06, D35)
Each hole can run **fake sonar** and/or **fake fish** (Hall-sensor trips: flag up 20-90 s at random, about N per hour),
set one by one or all at once: Settings → Test & simulation, serial `SIM <hole> SONAR|HALL|BOTH|OFF`, `SIM ALL ON|OFF`,
`SIM RATE n`, or the OLED Test page (double press = everything on/off).
- The chalet sends `CMD_SET_SIM` in the beacon; the hub applies it to its own hole and its test holes, and passes it to a
  real tip-up (`MSG_DEV_CMD` / `DEVCMD_SIM`) right after the tip-up's next message. The tip-up keeps it in NVS.
- Fake trips on a real tip-up go through its normal wake / alert / deep-sleep cycle: the whole alert path is tested.
- Every hole running a simulation reports it (tip-up `FLAG_SIM` → hub `LF_SIM` → chalet) and is marked **SIM** on the
  page, the phone alert and the OLED. Test holes (virtual, IDs ≥ 128) are always SIM.
- "Test holes on the hubs" (the former sonar test mode switch) creates the virtual holes; they run fake sonar by default.

## Demo network (chalet only, no other hardware) — 2026-10-06
Frank: "I thought I could fake entire modules, but I need to program and power a module to activate test mode."
The test holes above are made by a hub, so a powered hub was needed. The demo network runs **inside the chalet box**.

| | |
|---|---|
| Turn on | OLED: Menu › Settings › Simulation › **Demo network** (off/1/2/3/4 hubs) and **Holes per hub** (1-4) · web: Settings › Test & simulation › "Demo network on this box" · serial: `DEMO 3 3`, `DEMO OFF` |
| Default | 3 hubs × 3 holes (Frank's layout), fake sonar on every hole, fake fish (Hall) **off** (turn on per hole or "All holes") |
| How | each fake hub is a real `HubRole` (line table, sonar outbox, hub packet); its packet enters the chalet through the backbone entry `onEbHubPacket`, and the chalet's backbone beacon is fed back to it (acks, FOCUS, `CMD_SET_SIM`). Pages, alerts, web, glance and FOCUS see it like a real pocket |
| Sonar | non-focus holes: light BASE summary every 2 s (bottom, bait, 0-3 wandering fish); the FOCUS hole: the full fake fish finder (`SonarSource`, ~15 KB, only one) |
| Marks | hubs: **DEMO** on the home page, "H121 SIM" on Network; holes: SIM flag, names "Demo A1"… |
| IDs | hubs 121-124 (own hole = hub ID), other holes 101-112 (never a real hub's test-hole ID, 128-255). Keep these IDs free in a real network |
| Not saved | off after every reboot (never left running on the ice). Turning it off removes the demo holes from the pages |
| Not simulated | LoRa airtime / slots (the demo hubs never join the LoRa plan), radio range, battery |
| Host test | `test/test_demo_net` (holes appear with pocket + SIM, acks clear, BASE reaches the store, SET_SIM reaches the hubs, forget) |

## Knobs (2026-10-07, D42)
Real-water tests come late and may happen on the ice first, so the processing has knobs instead of fixed constants.
**Processing knobs** (run on every sonar hole, one set for all; `lib/IceMesh/src/sonar_params.h`):

| Key | Knob | Default | What it does |
|---|---|---|---|
| snr | Detection threshold | 10 dB | echo above the noise floor to count as a target (higher = less noise, misses faint fish) |
| prom | Peak contrast | 5 dB | peak above its neighbourhood |
| confirm | Confirm pings | 3 | pings before a target is reported (single-ping noise filter) |
| keep | Keep without echo | 5 | gaps allowed in a fish trace |
| gate | Max target move | 32 cm/ping | larger = fast fish stay one track, more mixing |
| dead | Dead zone | 0.9 m | nothing reported closer (ring-down, ice, bubbles) |
| bsmooth | Bottom smoothing | 25 % | lower = steadier bottom line |
| learn | Static scene learning | 0.012 | how fast bottom / weeds become grey background |
| range | Colour range | 46 dB | smaller = more contrast |
| tvg | Range compensation | on | deeper echoes boosted for spreading/absorption |
| bmin | Bottom search from | 0.6 m | bottom is searched deeper than this (prototype 0.6 m). **Bucket**: just past the ring-down (`ring=` on the BENCH line) and below the water depth, or the 2nd bottom echo is taken (0.40 m bucket reads 0.80 m). Too low in a lake locks onto the ring-down (replay: 0.20 m) |

**Driver knobs** (D43, used by the TUSS4470 driver `src/sensor_node/sonar_tuss4470.cpp`; ignored by the fake sonar except the ping rates):

| Key | Knob | Default | What it does |
|---|---|---|---|
| cycles | Burst cycles, base | 16 | longer = more energy, longer ring-down and blind zone |
| fcycles | Burst cycles, focus | 8 | shorter = sharper depth resolution in focus |
| gain | Receiver gain code | 1 | TUSS4470 LNA (reg 0x13), 0-3. Lower if `raw_max` clips |
| pinghz | Ping rate, active | 4/s | when targets are present / in focus |
| idlehz | Ping rate, idle | 1/s | nothing in the water column: saves battery |
| avg | Bursts averaged | 2 | base mode only (power average) |
| freq | Frequency mode | 0 | 0 = 190/200/210 kHz each ping, 1 = one per ping rotating, 2 = 200 kHz only |
| sound | Sound speed | 1403 m/s | 1350 + value; fresh water near 0 °C ≈ 1403 |
| bpf | Band-pass code | 0x1E | TUSS4470 reg 0x10 (200 kHz per open_echo) |
| thresh | Edge threshold | 31 | TUSS4470 reg 0x17, OUT_4 comparator for edge timing |

Defaults = the prototype ("Processed"): `test_sonar_proc` stays bit-exact. **Noise filter** presets (OLED): Low 7/4/2/6, Normal 10/5/3/5, High 14/7/5/4 (snr/prom/confirm/keep).
Path: chalet (OLED Settings › Sonar, web Sonar › Display › Processing, serial `KNOBS` / `KNOB <key> <v>`; bench: `KNOB <key> <v>` on the tip-up's own serial, this node only, not saved) → beacon `CMD_SONAR_PARAM` (one knob per command; whole set again when a hub appears after boot, or "Send to holes again") → hub (NVS) → tip-ups in the sonar control message (NVS). Demo network: the knobs act on the FOCUS hole (full fake sonar); the light demo summaries ignore them.

**Display knobs** (no radio): OLED Settings › Display (units, Sonar / Focus depth Auto or fixed, hide weak echoes; CardKB `+` `-` `a` on the Sonar / Focus pages); web Sonar › Display (depth scale Auto/fixed, colour gain, weak echoes, per phone); web Holes cards: scale "same for all" / "each hole" / fixed.
