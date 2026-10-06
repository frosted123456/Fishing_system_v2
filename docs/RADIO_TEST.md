# Radio test — how to run it (hardware: chalet + 1-3 hubs, Heltec V3)

1. Flash: `pio run -e cabin -t upload` on the chalet board, `pio run -e hub -t upload` on each hub
   (give every hub its own NODE_ID in platformio.ini — copy the `[env:hub]` block).
2. Power the chalet, open its Wi-Fi AP (IceFish-Remote / fishon123) → http://192.168.4.1/radio
3. Power the hubs. Within ~5 s each hub's OLED should go from SEARCHING to SYNC and the hub
   appears on /radio (JOIN). If a hub shows SYNC(echo) it is reached through another hub.
4. On /radio choose **rotate**. Every hub slot now cycles SF9/500, SF8/500, SF7/500 with 160-byte
   packets. Press "Reset stats" each time you move a hub.
5. Per position, wait ≥ 60 frames (1 min) and write down for each mode: rx/sched (loss %),
   RSSI avg/min, SNR avg/min at the chalet, and the hub's beacon RSSI/SNR, lost/64, sync err.
6. Walk test: set **fixed SF7** (most fragile) and walk a hub away; the OLED shows beacon RSSI/SNR and
   lost/64 live. Repeat with fixed SF9.
7. Relay test: put hub B where the chalet cannot hear it but hub A can: B's row shows `via A`.

What to look at:
| Signal | Healthy (est.) | If not |
|---|---|---|
| Hub "sync err" | within ±1000 µs | increase LEAD/TAIL in tdma_schedule.h |
| loss % in SF9/500 at the far position | < 5 % | antenna height (hub antenna matters as much as the chalet's) |
| SF7/500 loss vs SF9/500 | similar loss with ≥ 10 dB SNR margin | keep SF9 for that hub |
| CRC errors | ≈ 0 | collisions → check two hubs don't share an ID; timing |
Send me the /radio JSON (`/api/radio`) of each position and I'll tune the thresholds.
