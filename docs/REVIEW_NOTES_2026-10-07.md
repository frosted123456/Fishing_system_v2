# Review notes: commits dcebd23..fd63e36 (2026-10-07)

For the next reviewer. The last full review round was 1264584; everything after it is unreviewed
(29 files, ~1900 lines). `git diff 1264584..HEAD`.

## What changed

| Commit | Area | Main files |
|---|---|---|
| dcebd23 | FISH ON latched until silenced (D41); depth Auto/fixed scaling; sonar + display knobs (D42) | `src/lora_node/main.cpp`, `screens.cpp`, `lib/IceMesh/src/sonar_params.h` |
| d676a0a | Sonar improvement list mapped | `docs/SONAR_ROADMAP.md` |
| cf4ff0f | BASE v4 status bits, ring-down alarm, level check, near-bait / cover, bait per hole, rotation, adaptive rate, trip recordings (D43) | `lib/IceMesh/src/sonar_*.h`, `src/lora_node/main.cpp`, `mesh_radio.cpp`, `src/sensor_node/main.cpp` |
| fd63e36 | TUSS4470 driver (blind), bench firmware, PC replay, `bmin` knob | `src/sensor_node/sonar_tuss4470.*`, `tools/sonar_replay.cpp` |

## Where the risk is (review these first)

1. `src/sensor_node/sonar_tuss4470.cpp`: never run on hardware. IDF 4.4 legacy APIs (RMT, I2S built-in ADC,
   MCPWM capture). IO2 is both RMT output and capture input. The TUSS4470 SPI frame and read semantics come from open_echo.
   `docs/SONAR_DRIVER.md` › "Risks" lists the specific doubts.
2. Proposed WROOM pins (header) are **not confirmed by Frank**; strapping pin 5 used as CS.
3. Chalet alarm latch (D41): `alarmStart` / `alarmAck` / `loopAlarmHold` and the buzzer esp_timer. The
   FISH ON alarm must never be lost or stuck.
4. Protocol: `CMD_SONAR_PARAM` (5), `CMD_SET_BAIT` (6), `DEVCMD_BAIT` (3), `SonarCtrlMessage` params[24],
   BASE v4 (17 B max). Hub forwards via `g_devq`. Old v3 decoding still accepted?
5. NVS blobs that grow with new knobs (`sonar`/"p" on chalet, hub and tip-up): shorter old blobs keep
   defaults for new knobs (intended).

## How it was checked

- Builds: hub, cabin, relay, sensor_lora, sensor_c3, sensor_wroom, sensor_wroom_sonar, sensor_wroom_sonar_bench
  (`tools/build_check.sh`, arduino-cli, core 2.0.17). New files are free of warnings at `WARNINGS=all`; the 4 left are old v1 code.
- Host: `make -C test all` 18/18, `make -C test sanitize` 18/18 (ASan + UBSan). Prototype bit-exact test unchanged.
- Replay tool: fake recording, stdin = file, Windows monitor prefix + CRLF.
- **Not checked**: anything on hardware since the buzzer fix; the web pages only by `node --check` on the scripts.

## Known open items (not bugs)

- 2nd bottom echo used for hardness only, not to validate the bottom (roadmap #14).
- One BPF code for 190/200/210 kHz; per-frequency codes need the datasheet table.
- Edge timing (`edge_um`) is recorded but not used by the processing.
- Near-bait / cover / ring-down thresholds are guesses until real recordings exist.
- Never commit `src/lora_node/config.h` (real Wi-Fi credentials, locally modified). Stage explicit paths only.

## Added later the same day (db27995..363a4e1)

| Commit | Area | Review focus |
|---|---|---|
| db27995 | Sonar matched to Frank's power design: no converter enable, TUSS4470 sleep (0x1B bit 7) before deep sleep, NCS held high in deep sleep, burst-count register never 0 (= continuous burst) | `tuss::sleep()` / `wake()` / `xfer()` hold handling |
| 95e7d7e | Drive voltage knob `vdrv` (VDRV = V − 5 in reg 0x16, keep below the MT3608 output), OUT_4 off by default | Silent ping if `vdrv` > supply? (datasheet does not say) |
| 363a4e1 | **WROOM alert path rewritten for the spool-shaft Hall latch** (flip counting, ext0 wake on the opposite level, RTC pull-up), Hall on GPIO 27 | Highest risk: the FISH ON path. `hall_latch.h` + `setupPins` / `readReedHw` / `hallArmWake` in `src/sensor_node/main.cpp`. C3 reed path must be unchanged (`HALL_LATCH` 0) |

## Review round 2 (Fable, D47) — what was found and fixed

| # | Where | Finding | Fix |
|---|---|---|---|
| 1 | driver `begin()` | Presence check wrote 0x5A to reg 0x17 whose bits 7:5 are read-only: `g_ok` could never be true on real hardware | Two-value write/read on 0x10 (all bits R/W) |
| 2 | driver `ping()` | LNA gain codes are not in order (0=15, 1=10, 2=20, 3=12.5 V/V): knob was not monotonic | Mapped: knob 0..3 = 10/12.5/15/20 V/V |
| 3 | driver | Threshold reg 0x17 is 4 bits + enable bit; knob 0-255 wrote junk into the enable | Knob 0-15, enable only when OUT_4 is wired |
| 4 | driver | One BPF code for 3 frequencies; datasheet table = one code per ~10 kHz | knob−1 / knob / knob+1, default 0x1D |
| 5 | driver | dB/count from the prototype (95 dB/4096), 15 % off the datasheet slope | 29.7 mV/dB typ. (est. ±10 %) |
| 6 | driver | Time zero from a leakage that may not exist on VOUT → random offsets | Fallback constant + `(FALLBACK)` flag on the BENCH line |
| 7 | driver | IO_MODE never written (relied on reset value) | 0x14 = 0 written in `begin()` |
| 8 | processing | A louder school / bait over a soft bottom steals the bottom (strongest window rule) | 2nd-echo validation (#14); test fails on old code |
| 9 | processing | Bottom near the end of the range: noise floor measured ON the bottom tail → no targets at all | Quietest 40 bins above the bottom; test: old −42 dB / 0 fish, new −83 dB / 18 fish |
| 10 | sensor node | No bait set → prototype 4.57 m used: false "Bait" labels and near-bait beeps | bait_m = −1 on a real hole until set |
| 11 | chalet | Silence while a hole is still tripped: alarm came back after the silence expired, once the line reset | `alarm_acked` → cleared on line reset |
| 12 | tip-up | Hall state not reset with the RTC data → possible false trip after RTC corruption | Reset with the magic check |

Still open (needs hardware or data): log-amp slope of this part, time-zero fallback value, ring-down / cover / near-bait thresholds, deconvolution, pulse coding, size class.
