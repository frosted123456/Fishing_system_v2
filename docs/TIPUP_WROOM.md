# WROOM tip-up: Hall latch on the spool shaft (D46, 2026-10-07)

Hardware (Frank's design): US1881 bipolar latch (TO-92) in the arm, 4 × N52 magnets in the shaft clamp at
45/135/225/315°, poles alternating. VCC from the pack (US1881 needs ≥ 3.5 V), open-drain output up the
mast to **GPIO 27** (RTC GPIO, internal pull-up = 3.3 V at the pin).

**Status: host-tested logic only** (`test/test_hall_latch`), compiles for `sensor_wroom*`. Not run on hardware.

## How the firmware reads it

There is no "flag up" level: the output flips every quarter turn of the shaft.

| Event | Firmware |
|---|---|
| Asleep | Wakes on the **opposite** level of the last one (ext0), so the next flip wakes it, never a wake loop. RTC pull-up on GPIO 27 kept through deep sleep |
| Wake by the pin, level changed | 1 flip |
| Wake by the pin, level the same | Flipped and back before the boot read it: 2 flips |
| Awake | Interrupt counts every edge (spool spinning fast is counted) |
| Trip (FISH ON) | `TRIGGER_FLIPS` flips (1 = quarter turn, 2 = half turn) with less than `HALL_WINDOW_MS` (10 s) between them |
| Trip ends | Shaft still for `HALL_CLEAR_SEC` (30 s): line state back to normal. The chalet alarm stays latched until silenced (D41) |

Settings in `src/sensor_node/config.h`. `HALL_LATCH` is 1 for the WROOM, 0 for the C3 (reed logic unchanged).

## Bench test (by hand, serial monitor 115200)

| Step | Do | Expect |
|---|---|---|
| 1 | Power on | `Hall latch: level 0/1, 0 flips since power-on, trip off` |
| 2 | Wait for deep sleep, then turn the shaft a quarter turn | Wakes at once, `>>> Sending ALERT` (TRIGGER_FLIPS 1) |
| 3 | Keep turning | Stays FISH ON; the flip count grows on each boot line |
| 4 | Stop for 30 s | `Flag reset - FISH OFF` / clear status sent |
| 5 | TRIGGER_FLIPS 2: one quarter turn | No alert; a second quarter turn within 10 s alerts |
| 6 | Idle current | Multimeter in series with the pack, everything asleep (sonar asleep too) |

## Known limits / open

- Resetting the flag by hand turns the shaft: with `TRIGGER_FLIPS 1` that may raise a short alert while you
  stand there. If it is annoying, use 2.
- Line-out length (flips × spool circumference / 4) is counted (`hall.total`) but not sent to the chalet yet.
