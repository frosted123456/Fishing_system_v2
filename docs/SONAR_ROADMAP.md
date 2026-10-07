# Sonar improvement list vs what exists (2026-10-07)

Frank's list (from the sonar design conversation), checked against this repo. Priorities dropped on purpose.
Status: **Done** = in the node processing (`lib/IceMesh/src/sonar_proc.h`, port of the prototype, host-tested);
**Now** = can be written and tested on fake data before any hardware; **Driver** = part of the TUSS4470 driver
(write now, verify in the bucket); **Data** = needs real recordings first.

| # | Item | Status | Notes |
|---|---|---|---|
| 1 | Sound speed 1403 m/s | Driver | Processing works in 2.5 cm depth bins; the driver converts sample time to depth: 1 bin = 35.6 µs round trip at 1403 m/s. One constant (knob). |
| 2 | Converter off while listening | Driver | Needs a GPIO on the converter's enable pin. Note: 3 AA (≈4.5 V) to 24-28 V needs a **boost**; a buck only steps down. |
| 3 | Drive 24-28 V | Hardware | Converter output; firmware only reports battery. |
| 4 | I2S-DMA ADC 100-200 ksps + calibration curve | Driver | Classic ESP32 I2S-ADC mode (IDF 4.4). 4-7 samples per 2.5 cm bin → averaged into the bin (extra SNR). Calibration table in NVS. |
| 5 | Band-pass + log-amp low-pass registers | Driver | TUSS4470 reg 0x10 BPF (0x1E = 200 kHz per open_echo settings.h), LNA gain 0x13; log-amp LPF per datasheet. Knobs. |
| 6 | Burst 16 cycles base / 6-8 focus | Now (knob) + Driver | Reg 0x1A. Split the `cycles` knob into base / focus. |
| 7 | Rotate 190/200/210 kHz per ping | Now (processing) + Driver | Processing today expects the 3 frequencies **in the same ping** (3 bursts). Rotating 1 per ping needs compounding over the last 3 pings: change in `sonar_proc.h`, testable on fake data. BPF code per frequency: datasheet. |
| 8 | OUT_4 comparator edge timing | Driver | ESP32 GPIO interrupt jitter is a few µs (≈1-3 mm); hardware capture (MCPWM capture, 12.5 ns) needed for sub-mm. Threshold reg 0x17. |
| 9 | Adaptive ping rate 1 Hz idle / 4 Hz on target | Now | Node logic + `pinghz` knob; testable with the fake sonar (processing assumes 0.25 s between pings: needs the real interval). |
| 10 | Average 2-4 pings in base mode | Now | Base mode only (summary every 2 s); fish move between pings → average the echo envelope, not the targets. |
| 11 | Pulse coding + correlation | Data | Only if range is short. |
| 12 | Range compensation 40 log r + 2αr | **Done** | α = 0.04 dB/m; `tvg` knob. |
| 13 | Adaptive noise floor below the bottom | **Done** | |
| 14 | Bottom: energy window, onset, hysteresis, second echo | **Done** (second echo used for hardness, not to validate the bottom) | Second-echo check against a fish → Now. |
| 15 | Sub-sample peak fit | **Done** | |
| 16 | Static scene, frozen around targets | **Done** | `learn` knob. |
| 17 | Tracking, velocity, trend | **Done** | Trend arrows on the web Sonar tab. |
| 18 | Deconvolution with a recorded burst | Data | Needs the real burst shape (bucket). |
| 19 | Bait as reference | Partial | "vs bait" shown; bait depth is a constant (4.57 m). Per-hole bait depth → Now. |
| 20 | Flicker | **Done** | |
| 21 | 190 vs 210 kHz decorrelation | **Done** (needs #7 on hardware) | |
| 22 | Bottom hardness | **Done** | |
| 23 | Weeds / cover | Partial | "Cover" label exists (static near bottom); "is the bait in cover" → Now. |
| 24 | Cone-crossing profile | Data | |
| 25 | Size class | Data | |
| 26 | Ringdown monitoring (frazil / slush) | Now | Ring-down energy vs its normal level, alert per hole. |
| 27 | Level check at setup | Now | Bottom-echo strength per hole on the phone / OLED during setup (adds bottom strength to the BASE block). |
| 28 | Strike-imminent alert | Now | Fish rising toward the bait within N cm → early warning (separate from FISH ON). |
| 29 | Log 60 s of sonar before each flag | Now | Chalet keeps ~3 min per hole already: save the 60 s before each trip, download from the web. |
