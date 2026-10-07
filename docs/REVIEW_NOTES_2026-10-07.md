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
