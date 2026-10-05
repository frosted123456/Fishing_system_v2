# Review — radio + data protocol proposal (2026-10-05)

"(est.)" = estimate, not measured. Regulatory notes are not legal advice.

## Verdict
Direction is right: 500 kHz instead of 125 kHz, scheduled slots instead of CSMA, latching
events, BASE/FOCUS budgets. Wrong or missing as written:
1. EVENT-ACK as a bitmap can drop a newer event (no sequence in the ack).
2. "Node index in pocket" breaks dedup/self-healing when a node is heard or served by another pocket.
3. Sonar blocks have no node ID and no length, but hubs concatenate and truncate them → not parsable.
4. Pixel layer "changes vs last ping" breaks the packet-independence rule across blocks.
5. "ping index = seq × N" breaks when N changes; later-ping targets are ambiguous when a target appears/disappears mid-block.
6. Sensitivity numbers: the 125→500 kHz cost is 6 dB in theory, not 3.75 dB.
7. A 2-3 ms guard needs an interrupt/timer-driven radio task; the current loop() blocks for up to 1.5 s.
Correction of my own earlier work: v1 runs SF9/125 kHz on one channel at 20 dBm, and my decision
D4 kept it. Under the RSS-247 reading in this proposal that is not compliant; I should have flagged it.

## 1. Regulatory (RSS-247 §5, mirrors FCC 15.247 — check the current RSS-247 issue)
| Claim | Check |
|---|---|
| 125 kHz single channel must hop | Right. Hopping with 20 dB BW < 250 kHz needs ≥ 50 channels and ≤ 0.4 s average occupancy per channel in 20 s. |
| ≥ 500 kHz for single-channel digital modulation | Right: DTS, 6 dB bandwidth ≥ 500 kHz, PSD ≤ 8 dBm / 3 kHz, ≤ 1 W conducted, antenna > 6 dBi reduces power. |
| LoRa 500 kHz qualifies | Plausible: LoRaWAN US915/CA915 runs 500 kHz channels as DTS. The measured 6 dB bandwidth of a 500 kHz chirp sits close to the limit (est.). |
| PSD at 20 dBm / 500 kHz | ≈ −2 dBm / 3 kHz (calc.), under 8 dBm. |
| ≤ 400 ms airtime | Hopping dwell rule, not needed for DTS. Fine to keep as a design limit (short slots). |

## 2. Sensitivity and link budget
Sensitivity = −174 + 10·log10(BW) + NF + SNRlimit, NF ≈ 6 dB (est.).
| Mode | SNR limit | Sensitivity (theory) | Proposal |
|---|---|---|---|
| SF9/125 | −12.5 dB | ≈ −129.5 dBm | −131.25 |
| SF7/125 | −7.5 dB | ≈ −124.5 dBm | – |
| SF9/500 | −12.5 dB | ≈ −123.5 dBm | −127.5 |
| SF8/500 | −10 dB | ≈ −121.0 dBm | – |
| SF7/500 | −7.5 dB | ≈ −118.5 dBm | – |
125 → 500 kHz = 10·log10(4) = 6.0 dB. Plan with 6 dB.

Over ice both antennas are near a flat reflecting surface → two-ray (plane-earth) path loss
PL ≈ 40·log10(d) − 20·log10(h_hub·h_chalet) beyond ~50-150 m (est. model).
TX 20 dBm, hub antenna 0 dBi, chalet external 3 dBi (est.), chalet antenna 4 m high (est.).
| Hub antenna height | PL @ 1 km | Margin SF9/500 | Margin SF7/500 | Margin SF9/125 |
|---|---|---|---|---|
| 0.3 m | 118 dB | ≈ 28 dB | ≈ 23 dB | ≈ 34 dB |
| 1 m | 108 dB | ≈ 38 dB | ≈ 33 dB | ≈ 44 dB |
| 2 m | 102 dB | ≈ 45 dB | ≈ 40 dB | ≈ 51 dB |
At 500 m add ≈ 12 dB. Unmodelled losses (people, huts, snow, antenna detuned near ice/body,
cold, cable) could take 10-20 dB (est.). SF9/500 at 1 km looks comfortable on paper; the range test decides.
Correction: the hub antenna height counts exactly as much as the chalet's (the product h1·h2).
Raising a hub antenna from 0.3 m to 1 m ≈ +10 dB, like raising the chalet antenna ×3.3.

## 3. Capacity (my recalculation, same 80 % use)
Assumptions (est.): 1 s superframe, beacon 30 B in the same mode, 4 hub packets + 1 relay
packet, 16 nodes of line state (2 B), 8 B per-packet header, 3 ms guard, 10 sonars, 4 relayed (count twice).
| Mode | Max payload ≤ 400 ms | Proposal B/s/sonar | Mine B/s/sonar |
|---|---|---|---|
| SF9/125 | 66 B | ~8.6 | **≈ 0 — beacon + 5 minimal packets ≈ 1050 ms > 800 ms** |
| SF7/125 | 255 B | ~35 | ≈ 22 |
| SF9/500 | 255 B | ~45 | ≈ 33 |
| SF8/500 | 255 B | ~80 | ≈ 69 |
| SF7/500 | 255 B | ~141 | ≈ 126 |
The proposal is 10-35 % optimistic under my assumptions; SF9/125 does not fit a 1 s superframe at all.
This table also answers a question the design does not ask: the real load is 1 FOCUS + BASE.
1 FOCUS at 60 B/s (relayed = 120) + 9 BASE at ~3 B/s (×1.4 for relays) ≈ 160 B/s needed vs
≈ 460 B/s available at SF9/500 → ~3× margin (est.).

## 4. Superframe / slots
| Item | Comment |
|---|---|
| Guard 2-3 ms | OK only if the beacon RxDone is timestamped in the DIO1 interrupt and slot TX is started from a hardware timer / high-priority task. Crystal drift ~20 µs/s (est.) is negligible. The v1 loop() blocks up to 1.5 s (UI overlays, web server, software-I2C OLED) → cannot hold a slot. |
| Mode switch per slot | SF/BW reconfiguration is a few SPI commands with BUSY waits (~1-3 ms, est.): do it before the slot, not inside the guard. |
| Beacon mode | Always the most robust mode (SF9/500) so every hub hears it. |
| Relay hub | Must listen to the remote hub's slot in that hub's mode → the beacon must say so. |
| SNR-based adaptation | LoRa packet SNR flattens for strong signals (ceiling to verify on your boards). Use RSSI − sensitivity(mode) as the margin, SNR as cross-check; hysteresis (step up after N good superframes, down at the first miss). |
| Missing: neighbour table | The chalet knows who it hears, not who can relay whom. Hubs must report the hubs they hear (+RSSI) in the health record. |
| Missing: join | A hub that does not hear the beacon has no slot. Needs a small contention/join slot that relay hubs listen to. |
| Missing: chalet lost | Free-run on the last timing for N superframes, then what? Local hub buzzers must not depend on the chalet. |
| Beacon size | Superframe 2 + slot map ~5 + mode/allowance ~10 + focus 1 + heard bitmap 2 + event acks 0-5 + header ≈ 25-35 B (est.) ≈ 50 ms at SF9/500. |

## 5. Line state
| Item | Comment |
|---|---|
| 4-bit index in pocket | Identity tied to topology. A node heard by two hubs gets two identities, and a node moving to another hub changes ID → dedup and self-healing break. Use a global 8-bit node ID: 8+3+4+5 = 20 bits, two records in 5 bytes. |
| EVENT-ACK bitmap | **Wrong as written.** A bit cannot say which seq was received: event k is pending, event k+1 occurs, the ack for k arrives, the hub clears k+1. Ack must carry (node ID, seq): 12 bits per pending event, usually 0-3 entries (≈ 0-5 B). |
| Latching instead of ACK | Good: implicit ack, no extra packets. |
| 4-bit event seq | Fine with one outstanding event; a seq jump tells the chalet it missed intermediate states. |
| Latency ≤ 1 superframe + relay | Plus node wake: deep-sleep boot + Wi-Fi init ≈ 0.2-0.4 s (est.) → ≈ 1.5-2.5 s worst case (est.). |

## 6. Sonar codec
| Item | Comment |
|---|---|
| No node ID, no length per block | **Wrong as written.** Hub concatenates blocks and may truncate one: without a length (or self-delimiting parse that survives truncation) the decoder loses every following block. Add node ID 8 b + length 8 b per block, or only ever truncate the last block. |
| seq × N | **Breaks when N changes** (BASE ↔ FOCUS, allowance change). Send the first ping index (12-16 b). 12 bits at 1 block/s wraps every ~68 min — fine. |
| Later-ping targets 6+3 b | Ambiguous when a target appears or disappears mid-block. Add the 3-bit track ID (9 → 12 b) or a per-ping presence bitmap over the block's tracks. |
| Pixel layer "changes vs last ping" | **Contradicts packet independence** across blocks. First ping of a block = changes vs background (version-tagged), later pings = vs previous ping in the same block. |
| Background CDF 5/3 + Rice | Wavelet on a 3-4-level quantized profile is probably the wrong tool; run-length + Rice on run lengths is likely smaller and simpler (est.) — measure both on recordings. |
| 5-bit target amplitude | Only worth it if used (target strength / fish size); otherwise 2 bits like the display. |
| 1 cm sub-bin depth | Fine for movement. Absolute depth is limited by sound speed: ~1402 m/s at 0 °C vs ~1421 m/s at 4 °C (bottom water) → up to ~1.3 % ≈ 13 cm at 10 m (est.). |
| 2.5 cm bins | At 1403 m/s = 35.6 µs per bin = 28.1 kHz ADC rate. A 10-cycle 200 kHz burst ≈ 3.5 cm resolution (est.) — bins slightly finer than the pulse, OK. |
| No LZ / no whole-ping DCT | Agree at these packet sizes. |
| Sizes | My count (N=4, 1 target, track ID added): ≈ 4.4 B/ping; N=8: ≈ 3.5 B/ping; 2 targets + a few residual cells: ≈ 5-8 B/ping (all est.). "Tuned 2-4 B/ping" needs N=8 and ≤ 1 target. FOCUS 40-60 B/s = 10-15 B/ping → vector fits with room for some pixel layer. |

## 7. Nodes (WROOM, Hall, 3×AA)
| Item | Comment |
|---|---|
| Energy budget | 3×AA lithium in series ≈ 3000 mAh (est., L91 at moderate drain; less with cold + high pulses). 4 sessions × 10 h = 40 h → ≈ 75 mA average allowed. Power is not tight: a devkit's LDO + USB-UART in deep sleep (≈ 5-15 mA, est.) still fits; sonar nodes could stay in light sleep instead of deep sleep, which removes boot latency. |
| Hall wake | Magnet parked in front of the sensor = output stuck active → a level-triggered wake loops (same class as the v1 reed bug). Wake on change and re-arm on the opposite level. |
| Hall part | Micropower Hall switches sample at a few Hz-tens of Hz (est.) and can miss quarter turns when the spool runs fast; always-on ones draw mA (est.). Need the max spool speed (rev/s) to choose. |

## 8. Impact on the phase-1 code
| Already done | Status under this proposal |
|---|---|
| PlatformIO + arduino-cli build check | Valid. First real compile: hub, sensor_wroom, sensor_c3 build, 0 errors. |
| C3 aggregate dedup fix | Moot (v1 aggregates go away), harmless. |
| dedup_window, seq, airtime | Valid (ESP-NOW duplicates, slot planning). |
| tx_queue | Valid as "slot packer": fill a slot allowance by priority. |
| mesh_hdr (8-byte per-message header) | Replace by a hub-packet header + records. |
| Planned C10 CAD/CSMA driver, C11, C12 | Not valid as designed → stopped before coding. Replace with a TDMA driver (ISR timestamp, esp_timer, FreeRTOS task, per-slot mode) and the hub packet / beacon formats. |
