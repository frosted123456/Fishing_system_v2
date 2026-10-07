# Sonar improvement list vs what exists (updated 2026-10-07, D43)

Frank's list (from the sonar design conversation), checked against this repo. Priorities dropped on purpose.
Status:

- **Done**: in the node processing (`lib/IceMesh/src/`), host-tested on fake data (`make -C test all`).
- **Blind**: in the TUSS4470 driver (`src/sensor_node/sonar_tuss4470.cpp`); compiles, never run on hardware. See `SONAR_DRIVER.md`.
- **Data**: needs real recordings first (`REC ON` + `tools/sonar_replay.cpp`).

| # | Item | Status | Where / notes |
|---|---|---|---|
| 1 | Sound speed 1403 m/s | Blind | `sound` knob (1350 + v, default 1403). Sample time → 2.5 cm bins in the driver |
| 2 | Converter off while listening | Dropped (hardware) | No enable wire in the design: MT3608 always on, TUSS4470 sleep mode instead. Driver still supports an enable GPIO if added |
| 3 | Drive voltage | Knob + hardware | `vdrv` knob (5-20 V, default 11 V for the MT3608 at 12.0 V). Up to 40 V p-p (+5.2 dB) with the MT3608 at 21 V |
| 4 | I2S-DMA ADC + calibration curve | Blind | 150 kHz, rate measured each ping (`adc=`); 17-point curve, serial `CAL`, NVS |
| 5 | Band-pass / log-amp registers | Blind (datasheet-checked) | `bpf` knob ±1 per frequency (datasheet table), `gain` 10-20 V/V, `thresh` 0-15. Log-amp 29.7 mV/dB typ. |
| 6 | Burst 16 base / 6-8 focus | Blind | `cycles` / `fcycles` knobs, RMT burst |
| 7 | Rotate 190/200/210 kHz | Done (processing) + Blind | `freq` knob: 0 = 3 per ping, 1 = rotate (processing compounds the last 3 pings), 2 = 200 only |
| 8 | OUT_4 edge timing | Blind | MCPWM capture, 12.5 ns; `edge=` on the BENCH line, `edge_um` in recordings. Not used by the processing yet |
| 9 | Adaptive ping rate | Done | `pinghz` / `idlehz` knobs; real `dt` used by tracking |
| 10 | Average 2-4 pings (base) | Blind | `avg` knob: power average of bursts inside one ping |
| 11 | Pulse coding + correlation | Data | Only if range is short |
| 12 | Range compensation | Done | `tvg` |
| 13 | Adaptive noise floor | Done | |
| 14 | Bottom tracking, second echo | Done | 2nd echo validates the bottom (D47): a louder school / bait over a soft bottom no longer steals it. `bmin` knob for shallow water. Noise floor in deep holes (bottom near 12 m) from the quietest water above it |
| 15 | Sub-sample peak fit | Done | |
| 16 | Static scene | Done | `learn` |
| 17 | Tracking and trend | Done | |
| 18 | Deconvolution | Data | Needs the recorded burst shape |
| 19 | Bait as reference | Done | Bait depth per hole (chalet OLED / web "Bait:", `CMD_SET_BAIT`, tip-up NVS) |
| 20 | Flicker | Done | |
| 21 | 190 vs 210 kHz decorrelation | Done | Needs #7 on hardware |
| 22 | Bottom hardness | Done | |
| 23 | Weeds / cover | Done (first version) | "Bait in cover" flag (`ST_BAIT_COVER`); thresholds are guesses |
| 24 | Cone-crossing profile | Data | |
| 25 | Size class | Data | |
| 26 | Ring-down monitoring | Done | Ring-down length vs learned normal → `ST_RING` → "ICE?" on the chalet. Thresholds are guesses |
| 27 | Level check at setup | Done | Bottom echo dB over noise (`bottom_snr`, BASE v4) on OLED Holes / web card |
| 28 | Strike-imminent | Done | Fish near the bait (`ST_NEAR_BAIT`) → "Near bait: X" note, optional beep (Settings) |
| 29 | Log 60 s before each flag | Done | Chalet keeps the last 8 trips (`/api/trips`, download). Raw recordings: tip-up serial `REC ON` |

## Backlog (not sonar, parked)

| Item | Status | Notes |
|---|---|---|
| Pin holes with the chalet box's GPS at installation | Idea (Frank, 2026-10-07) | Carry the box to each hole, pick it on the OLED, OK = store the fix; positions relative to where the box ends up. Needs: GPS module type and UART pins on the offshore box. Accuracy 3-5 m: fine for holes 10 m+ apart. Writes the same `/api/node/pos` store as the web map (D51). ~1 h once the module is known |

