# TUSS4470 sonar driver: bucket-test guide (D43, 2026-10-07)

`src/sensor_node/sonar_tuss4470.h/.cpp`, ESP32 WROOM tip-up + Open Echo TUSS4470 shield, 200 kHz transducer.

**Status: WRITTEN BLIND.** It compiles (envs `sensor_wroom_sonar`, `sensor_wroom_sonar_bench`, no warnings in these
files) and the processing it feeds is host-tested, but **not one line of the driver has run on hardware**. Expect
the first bucket session to find wiring / register / timing mistakes. The steps below are ordered so each one
checks one thing.

## What is verified and what is not

| Part | Verified? | How |
|---|---|---|
| Processing (bottom, noise floor, targets, ring-down, flags, rotation, adaptive rate) | Host only | `make -C test all` (fake sonar data), ASan/UBSan clean |
| Recording line format → PC replay | Host only | `tools/sonar_replay.cpp` on a fake recording; file, stdin and Windows monitor logs (`12:00:01.123 > SONAR ...`, CRLF) read the same |
| Driver compiles for the WROOM (IDF 4.4 legacy APIs: SPI, RMT, I2S-ADC, MCPWM capture) | Yes | arduino-cli, core 2.0.17 |
| SPI register protocol (odd parity, mode 1, 1 MHz) | **No** | Datasheet 7.5. Presence check = DEVICE_ID (0x1D) reads 0xB9. Every reply carries the status bits (VDRV_READY, burst faults) |
| Register fields (IO_MODE 0, LNA codes, threshold bits, BPF table, VDRV, sleep) | Datasheet | Re-read 2026-10-07 against the TI datasheet: 4 fields were wrong before (D47) |
| Burst on IO2 (RMT, 12.5 ns) | **No** | |
| I2S-ADC real sample rate | **No** | Measured by the driver itself (`adc=` on the BENCH line) |
| Time zero (transmit leakage onset) | **No** | Rule: first sample above 80 % of the max of the first 3 ms, if that max stands ~27 dB above the level after it; else `SONAR_T0_FALLBACK` (30 samples, est.) and `(FALLBACK)` on the BENCH line |
| dB scale of the ADC | Datasheet typ. | 29.7 mV/dB (25-33 over parts) at the 3.3 V VOUT map, 3.3 V ADC full scale: 0.0271 dB/count. `CAL` corrects the ADC curve; the slope stays ±10 % until a known-level test |
| OUT_4 edge timing (MCPWM capture) | **No** | IO2 is both RMT output and capture input (see "Risks") |
| TUSS4470 sleep mode between sessions / before deep sleep | **No** | Reg 0x1B bit 7 (datasheet, 220 µA typ.); NCS held high in deep sleep |

## Wiring: PROPOSED pins, Frank to confirm

None of these are confirmed. If the mast-head wiring differs, define **all** of them in the env's `build_flags`
(they are one `#ifndef SONAR_PIN_SCK` block: defining only some fails to compile).

| Signal | TUSS4470 / shield | WROOM GPIO | Why this pin |
|---|---|---|---|
| SPI SCK | SCLK | 18 | VSPI default |
| SPI MISO | SDO | 19 | VSPI default |
| SPI MOSI | SDI | 23 | VSPI default |
| SPI CS | NCS | 5 | VSPI default; strapping pin, fine as CS (idles high) |
| IO1 | IO1 | 26 | Held HIGH (as open_echo) |
| IO2 burst | IO2 | 25 | RMT output + MCPWM CAP0 input |
| OUT_4 comparator | OUT_4 | -1 | Not in the 12-core cable (the one echo wire is VOUT). Spare core + a free GPIO = edge timing |
| VOUT log-amp | VOUT | 36 (VP, ADC1_CH0) | I2S-ADC works on ADC1 only (GPIO 32-39); ADC1 works with the radio on |
| Converter enable | none | -1 | Not in Frank's design (MT3608 always on). Optional GPIO = converter off while listening |
| Already used | | 27 Hall latch, 2 LED, 34 battery | Hall moved from 15 to 27 (D46) |

Check before powering: **VOUT must stay below about 3.1 V** at the ESP32 pin (ADC at 12 dB attenuation). If the
shield's VOUT can go higher, add a divider and tell Claude (the dB scale changes).

## Power (items 2-3): Frank's design, 2026-10-07

| Rail | From | Feeds |
|---|---|---|
| Pack | 3 x L91 lithium AA, holder switch, 3.6-5.4 V | both converters, straight (no load switch) |
| 3.3 V | TPS63020 buck-boost | ESP32, Hall latch, shield logic (VDD) |
| 12.0 V (up to ~21 V useful) | MT3608 boost, trimpot locked | shield VIN (VPWR, transducer drive) |

What the firmware does with it:

- **No enable wire**: `SONAR_PIN_BOOST` = -1, the MT3608 runs whenever the holder switch is on. Idle = the
  TUSS4470 **sleep mode** over SPI (reg 0x1B bit 7, datasheet), set before every deep sleep and when the sonar
  stops; the next ping wakes it (`SONAR_WAKE_MS` 5 ms, est.). NCS is held high through deep sleep so a floating
  line cannot clock in a frame (a 10 k pull-up on NCS at the shield is the hardware equivalent).
- **Drive voltage = knob `vdrv`** (volts, 5-20, default 11). The TUSS4470 charges VDRV from VPWR (the MT3608)
  and needs VPWR > VDRV + 0.3 V (datasheet), so **knob = MT3608 setting − 1 V**. Full bridge: 2 × VDRV
  peak-to-peak across the transducer. Set from the chalet like any knob, or `KNOB vdrv 14` on the bench.
  If pings go silent after raising it, the knob is above what the MT3608 gives.

| MT3608 setting | `vdrv` knob | Across the transducer | Echo vs 12 V |
|---|---|---|---|
| 12.0 V (current plan) | 11 | 22 V p-p | reference |
| 15 V | 14 | 28 V p-p | +2.1 dB |
| 18 V | 17 | 34 V p-p | +3.8 dB |
| 21 V | 20 (chip max) | 40 V p-p | +5.2 dB |
| above 21 V | 20 | 40 V p-p | nothing more: VDRV tops out at 20 V |

  Before going above 12 V: the transducer's voltage rating (spec sheet), MT3608 still reaching the setting at
  3.6 V in (end of pack, bench supply), and the freezer re-check. Higher drive also lengthens the ring-down
  (`ring=`; raise `dead` / `bmin` to match).
- Quiet receive, chip side (roadmap #2, datasheet 7.3.1): DIS_VDRV_REG_LSTN = 1, so the TUSS4470 charges VDRV only
  between VDRV_TRIGGER and the burst (the driver waits for VDRV_READY, 4 ms max), not while it listens. The MT3608
  itself still switches during listening. Check in the bucket: `noise=` with the
  pack vs with the shield VIN from a bench supply / fresh 9 V battery. A few dB worse = acceptable; much worse =
  LC filter on shield VIN (hardware), or add an enable wire later (the driver supports one).
- Battery: the TUSS4470 sleeps, but the MT3608's own idle draw stays while the holder switch is on. Worth one
  multimeter reading in series with the pack (deep sleep, sonar asleep).
- `SONAR_CHARGE_MS` (4 ms, est.): pause before each burst to refill VDRV. Too short → weaker bursts later in an
  average (watch `raw_max` drop across pings).

## Bucket test, step by step

Build and flash the bench firmware (stays awake, real sonar from boot, no hub needed, one `BENCH` line every 2 s):

```
pio run -e sensor_wroom_sonar_bench -t upload
pio device monitor -b 115200
```

A BENCH line:
```
BENCH adc=149800Hz(want 150000) t0=37 raw_max=3010 edge=0.412m ping=61ms | bottom=0.41m bot_snr=38dB noise=-84dB ring=0.12m | 0.25m:fish
```

| Step | Do | Expect | If not |
|---|---|---|---|
| 0 Shallow water | **First thing in a bucket**: `KNOB dead 1` (0.1 m) and `KNOB bmin <v>` (x0.1 m) just past the ring-down (`ring=`) and below the water depth. Defaults (dead 0.9 m, bottom search from 0.6 m) hide a bucket bottom or take its 2nd echo (0.40 m reads 0.80 m) | `knob dead = 1`, `knob bmin = …` | `bottom=` jumps to `ring=`: bmin too low. Put both back to defaults (9 and 6) before the lake |
| 1 SPI | Boot message `Sonar TUSS4470: found`; type `REG 17` then `REG 17 5A` | Reads back what was written | `NOT answering` → SPI wiring, CS pin, shield power. Try `REG 10` (BPF) to see any non-zero answer |
| 2 ADC rate | Read `adc=` | Within ~2 % of 150000 | ≈75000 or ≈300000 = I2S-ADC rate quirk: **every depth is off by that factor**. Note the value and send it (fix = `SONAR_ADC_HZ` or the I2S config) |
| 3 Clipping | `raw_max` | Below 4095, no `(CLIPPED)` | Lower the LNA gain: `KNOB gain 0` (serial, this node, no hub needed) or the chalet knob `gain` once the value is known |
| 4 Time zero | `t0` | Small and stable (a few tens of samples; < 450 = 3 ms), no `(FALLBACK)` | `(FALLBACK)` on every line = no transmit leakage visible on VOUT: measure the real offset (ruler depth vs `bottom=`) and set `SONAR_T0_FALLBACK`. Jumping around = send a recording (step 8) |
| 5 Known depth | Transducer facing down, measure the water depth with a ruler | `bottom=` = ruler ± 2.5 cm | Error proportional to depth → sound speed (`KNOB sound <v>`, speed = 1350 + v m/s, 53 = 1403; fresh water near 0 °C is ~1403) or the ADC rate (step 2). Constant offset → time zero (step 4) |
| 6 Level | Tilt the transducer a few degrees | `bot_snr` drops when tilted | This is the setup level check (item 27) shown on the chalet |
| 7 Edge | `edge=` | Close to `bottom=` (first echo above the threshold after the blind zone) | `-` = no edge: threshold too high (`KNOB thresh <lower>`) or the IO2 capture does not see the burst (see "Risks") |
| 8 Record | `REC ON`, a minute or two, `REC OFF` (BENCH lines stop while recording) | Long `SONAR ...` lines | Monitor to a file: `pio device monitor -b 115200 --filter log2file` |
| 9 Ring-down | Note `ring=` with clean water; then stir in crushed ice | Larger `ring=`; after a few pings `(RING ALARM)` | Item 26 thresholds are guesses: send a recording |
| 10 Frequencies | `KNOB freq 0` (3 per ping), `1` (rotate), `2` (200 kHz only) | All give the same bottom | 190/210 kHz weaker = expected until per-frequency BPF codes are set (open question 3) |

## Serial commands (sonar builds only)

| Command | Does |
|---|---|
| `STAT` | One BENCH line after the next ping (any sonar build, not only bench) |
| `KNOB <name> <value>` | Set one processing/driver knob on this node only, for the bench (not saved; the chalet's set replaces it when a hub sends one). `KNOB` alone lists the names |
| `REC ON` / `REC OFF` | Recording lines: `SONAR <ms> <nfreq> <rot_f> <t0> <edge_um> <raw_max> <us> <hex codes freq0> [freq1 freq2]` (488 bins × 2 hex each) |
| `REG <addr hex>` / `REG <addr hex> <value hex>` | Read / write a TUSS4470 register (the next ping rewrites 0x10 0x13 0x17 0x1A from the knobs) |
| `CAL` / `CAL <17 numbers>` | Show / set the ADC calibration curve: corrected value for raw 0, 256, … 4096. Kept in NVS `sonarcal`. Identity until set |

## Replay a recording on the PC (tune knobs without water)

```
g++ -std=c++11 -O2 -Ilib/IceMesh/src tools/sonar_replay.cpp -o .preview/sonar_replay
.preview/sonar_replay log.txt                     # CSV: ms, nfreq, bottom_m, noise_db, bottom_snr_db, ring_m, ring_alarm, status, targets
.preview/sonar_replay snr=8 learn=20 log.txt      # same with knobs changed (names: see SONAR_SIM.md "Knobs")
.preview/sonar_replay --trace log.txt > t.json    # JSON trace for tools/web_preview.py
.preview/sonar_replay --make-fake 200 > fake.txt  # fake recording, to test the tool
```
No compiler on the Windows PC? Send the log file to Claude: the replay runs in its workspace.

## Constants that are estimates (search `est.` in the code)

| Constant | Value | Where it matters | Fix from |
|---|---|---|---|
| `SONAR_ADC_HZ` | 150000 | Depth scale | Step 2 (`adc=`) |
| dB scale | -100 dB at 0, -5 dB at 4095 (95 dB over the ADC range) | Noise floor / SNR numbers, `snr` knob meaning | TUSS4470 datasheet log-amp slope (mV/dB) + VOUT divider, then `CAL` |
| Time-zero rule / `SONAR_T0_FALLBACK` | 80 % of the max in the first 3 ms / 30 samples | Constant depth offset | Step 4 / recordings |
| `SONAR_CHARGE_MS` | 4 ms | Burst strength in averages | Scope on VDRV, or `raw_max` over an average |
| BPF codes | knob `bpf` (0x1D = 196.8 kHz) ±1 for 190 / 210 kHz | 190/210 kHz sensitivity | Datasheet table, bench: `bot_snr` per `freq` mode |
| Burst count register 0x1A | `cycles` (1-32), **never 0 = continuous burst** (datasheet) | Probably unused in IO mode: the RMT sends the real count | Datasheet: IO mode |
| Wake time | 10 ms (datasheet power-up time; sleep exit is probably faster) | First ping after sleep | Bench |
| Capture length | 2800 samples = 12.2 m at 1403 m/s | Max depth | Raise `MAX_SAMPLES` for deeper lakes (RAM: 2 B/sample) |
| Ping time | ~20 ms per capture + 4 ms charge; base = 3 freq × avg 2 → ~140 ms | Must stay under the 250 ms tick | `ping=` on the BENCH line |

## Risks to check first (for the reviewer)

1. **IO2 shared by RMT (output) and MCPWM CAP0 (input).** Order in `begin()`: MCPWM GPIO init → RMT config →
   `PIN_INPUT_ENABLE` on IO2. If `edge=` is always `-`, this is the first suspect; fallback: jumper IO2 to a
   second free GPIO and capture there.
2. **I2S built-in ADC on the classic ESP32**: sample-rate and swapped-pair quirks are handled but unproven;
   `adc=` measures the real rate on every ping.
3. **OUT_4 edge rate**: if OUT_4 toggles at the carrier frequency during echoes, the capture ISR runs at up to
   200 kHz for the echo length. Only the first edge is used; the ISR is short, but watch for watchdog resets.
4. **ADC during radio TX**: ESP-NOW sends happen after the ping in the same loop (never during a capture).
5. **Analog echo up the mast**: VOUT travels in the same cable as SCLK. No SPI traffic happens during a capture
   (registers are written before the burst), but the IO2 burst at 200 kHz does: watch for a burst-shaped bump
   right after `t0` that is not in the water. Twisting VOUT with a GND wire helps.
6. SPI timing: mode 1 at 1 MHz per open_echo; the datasheet (7.5) says SDO is sampled on the falling SCLK edge
   and SDI shifted on the rising edge. If `REG 1D` does not read 0xB9, try SPI_MODE0 first.

## Open questions

1. GPIO numbers (table above). The cable carries SCLK/SDI/SDO/NCS, IO1, IO2, "echo": is "echo" VOUT (analog,
   needed) or OUT_4 (edge timing, optional), or both?
2. Transducer voltage rating, before raising the MT3608 above 12 V (`vdrv` table in "Power").
3. Shield VOUT maximum vs the ESP32 ADC range: with the shield logic on 3.3 V it should stay under ~3.1 V; check.
4. Log-amp slope of this part (datasheet 25-33 mV/dB): a known-level test, or accept ±10 % on all dB numbers.
5. Transducer: beam angle, Q / bandwidth, ring-down time (sets the `dead` knob and burst cycles).
