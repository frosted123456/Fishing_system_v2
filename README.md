# Fishing_system v2 — ice fishing tip-up mesh with per-hole sonar

This folder is the **new version** of the firmware. The original download stays untouched
next to it in `../Fishing_system-main/` for reference; `../open_echo-main/` is the Open Echo
(TUSS4470) reference repo.

## Git
- `main` = baseline import of Fishing_system-main, unchanged (commit "Baseline import").
- `v2/phase1-mesh-core` = phase 1 work (mesh core). Nothing is merged to `main` without review.

## Layout (PlatformIO project, decision D2)
```
Fishing_system_v2/
├── platformio.ini       envs: hub, cabin, relay, sensor_lora (Heltec V3), sensor_c3, sensor_wroom, native
├── src/
│   ├── lora_node/       main.cpp + config.h — all LoRa roles (was lora_node.ino)
│   └── sensor_node/     main.cpp + config.h — tip-up node (was sensor_node.ino)
├── include/messages.h   single shared copy of the message structs (was duplicated)
├── lib/IceMesh/         shared mesh core, pure C++ (header, dedup window, stream filter, TX queue)
├── test/                host unit tests: `pio test -e native` or `make -C test` (plain g++)
├── docs/                protocol_v2 (as built), RADIO_TEST (how to test), decisions, reviews
├── tools/               helper scripts
└── reference/           sonar-display-prototype.html (missing for now)
```

## First-time setup (Windows)
1. Install VS Code, then the "PlatformIO IDE" extension.
2. File → Open Folder → `Fishing_system_v2`. PlatformIO downloads the pinned toolchain and libraries.
3. Bottom bar: pick an env (e.g. `env:hub`), then Build (✓) / Upload (→) / Monitor (plug).

## Roadmap
1. Mesh core (this branch) · 2. Self-healing · 3. Sonar driver · 4. Sonar processing + modes ·
5. Vector encoder/decoder + web pages · 6. PC mesh simulator.
