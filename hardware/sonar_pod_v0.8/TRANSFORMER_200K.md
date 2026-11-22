# Transformer drive for the sonar pod: JLC-assembled (D118 / D119)

**Correction (D119).** The first version of this note (D118) said the TDK B78416A2386A003 named in `src/design.py` was
a ~50 kHz part and proposed a hand-wound toroid. That was wrong: it rested on a distributor listing. TDK's own EP6
family data sheet lists the **B78416A2386A003 at 300 kHz**, 1:1:9, 0.23 mH ±8 % (pins 4–6), the only off-the-shelf
part found that is specified above 200 kHz. The other B78416 types (A2232 / A2360 / A2430 / A2433) are 50–52 kHz.
The original choice in design.py was right. No hand-wound part is needed.

Sources: TDK EP6 family data sheet
https://product.tdk.com/system/files/dam/doc/product/transformer/transformer/ultrasonic/data_sheet/85/ds/b78416ax003_ep6.pdf ;
stock: Mouser ~1,850, Digi-Key ~1,550, about 3–5 $ (both: **end of life**). Not at LCSC: order it through **JLC Global
Sourcing** (Digi-Key / Mouser) so JLC assembles it. Buy a buffer stock now (EOL).

## The part

| Item | B78416A2386A003 (EP6, SMD) |
|---|---|
| Rated frequency | 300 kHz |
| Turns | N1a : N1b : N2 = 1 : 1 : 9 (centre-tapped primary, push-pull) |
| Inductance | 0.23 mH ±8 %, pins 4–6 (measured at 50 kHz); pin 6 to GND recommended |
| DC resistance | 0.23 Ω per primary half, 8.7 Ω secondary |
| Insulation | 200 Vrms, 50 Hz, 1 s, primary to secondary |
| Pin map | from the TDK drawing (not in the text version): **confirm before the footprint** |

## Operating point at 199 kHz (calc unless marked)

| Item | Value |
|---|---|
| Secondary 0.23 mH + transducer 1.5 nF | resonates at 271 kHz |
| Tuning to 199 kHz | 2.78 nF in total: **add 1.2–1.3 nF C0G, ≥ 630 V** across the secondary (the C43 position) |
| L3 shunt inductor | not needed with this part |
| Volt-seconds | 199 kHz is 1.5x longer per half-cycle than 300 kHz. No V·µs rating is published, so keep the centre tap low (est.) |

| Centre tap | Secondary square | Fundamental on the transducer | Gain vs today (40 V p-p) |
|---|---|---|---|
| **9.4 V (start here)** | ±85 V | ~216 V p-p | **+12.6 dB** |
| 13 V | ±117 V | ~297 V p-p | +15.3 dB |

Theoretical: expect 1–3 dB of losses (est.). 9.4 V keeps the core near the volt-seconds of a 12 V / 300 kHz design (est.).

Currents and energy depend on the transducer's unknown motional resistance Rm (tuned load = Rm):

| Centre tap | Rm | Primary per FET | Burst power | Energy per 32-cycle burst |
|---|---|---|---|---|
| 9.4 V | 200 Ω | ~4.9 A pk | ~29 W | ~4.7 mJ |
| 13 V | 100 Ω | 13.4 A pk | 110 W | 17.7 mJ |
| 13 V | 200 Ω | 6.7 A pk | 55 W | 8.9 mJ |
| 13 V | 400 Ω | 3.3 A pk | 28 W | 4.4 mJ |

## Board changes for a JLC-assembled transformer build (pod v0.9, "xfmr" variant)

| # | v0.8 | v0.9 xfmr | Why |
|---|---|---|---|
| 1 | P1 2x3 header (hand-wired transformer) | **EP6 SMD footprint for B78416A2386A003** | JLC places it (Global Sourcing) |
| 2 | Q1 / Q2 AP1002, SOT-23, 2 A | **dual 60 V logic-level N-FET, SO-8, IDM ≥ 20 A, RDS(on) ≤ 30 mΩ at 4.5 V** (TI suggests CSD88537ND on E2E; check LCSC stock) | primary peaks 3–13 A; drains see 2 x centre tap + spikes |
| 3 | VBOOST 24.1 V (R4 200k / R5 5.1k) | **R4 75.0k → 9.4 V** (or 105k → 13 V) | centre tap from VBOOST; VPWR / VDRV follow (VDRV is only the gate drive in this mode) |
| 4 | C44 / C45 2 x 10 µF on VTX | **+ 1000 µF 16 V SMD electrolytic** on VTX, R46 0R → **10 Ω** | a burst needs 4–18 mJ; 2 x 10 µF hold ~1 mJ; the 10 Ω keeps the burst off VPWR |
| 5 | C43 DNP | **1.2–1.3 nF C0G 630 V** (1206 / 1210) | tunes the secondary to 199 kHz |
| 6 | R8B 0R | **2.2 kΩ, 2 x 1206 in series** (≥ 250 V across it) | receive series resistor |
| 7 | D7 DNP | **BAV99** | clamps the receiver input |
| 8 | R40 / R41 0R | not fitted | direct drive off |
| 9 | XB / P1 4–6 | secondary low side (pin 6) to GND | TDK recommendation |

Firmware (done): knob **`drive 1`** sets PRE_DRIVER_MODE (reg 0x1A bit 6) and caps the gate drive (VDRV) at 10 V;
with the 9.4 V build set **`vdrv 8`** (VDRV must stay below VPWR). Every flash resets `drive` to 0 (D114).

## Bench order

1. v0.9 board, transformer build, `drive 1`, `vdrv 8`, **`cycles 8`**: scope VTX (sag), a drain (spikes < 50 V), the
   transducer.
2. 16, then 32 cycles. TUNE (the tuned load shifts the peak), SWEEP.
3. Compare the floor SNR with a direct-drive pod (or the shield): expect +10–12 dB at 9.4 V.
4. Only if clean: 13 V build (R4 105k, `vdrv 10`), +2.7 dB more.

## Open questions

1. TDK drawing: pin map of the EP6 part (and the recommended land pattern).
2. Transducer Rm at 199 kHz (sets the FET current and the reservoir size): seller, or signal generator + 100 Ω + scope.
3. Dual FET choice in LCSC stock (JLC Basic / Extended).
