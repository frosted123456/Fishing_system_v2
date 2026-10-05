# Fishing_system v2 — ice fishing tip-up mesh with per-hole sonar

This folder is the **new version** of the firmware. The original download stays untouched
next to it in `../Fishing_system-main/` for reference; `../open_echo-main/` is the Open Echo
(TUSS4470) reference repo.

## Git
- `main` = baseline import of Fishing_system-main, unchanged (commit "Baseline import").
- `v2/phase1-mesh-core` = phase 1 work (mesh core). Nothing is merged to `main` without review.

## Layout
```
Fishing_system_v2/
├── sensor_node/        tip-up node (ESP32-C3 / WROOM), ESP-NOW + deep sleep   [v1 code]
├── lora_node/          Heltec WiFi LoRa 32 V3 (SX1262): hub / cabin / relay roles [v1 code]
├── docs/
│   ├── architecture_v1.md   map of the existing code (files, loops, TX/RX/relay/dedup)
│   ├── review_v1_issues.md  the 6 suspected issues + other findings, with line refs
│   ├── protocol_v2.md       v2 header, traffic classes, dedup rules, queue, open questions
│   └── phase1_plan.md       proposed phase 1 commits (waiting for approval)
├── test/               host-side unit tests (plain C++ / g++) — phase 1
├── tools/              build + consistency scripts — phase 1
└── reference/          sonar-display-prototype.html (JS reference for the sonar pipeline)
```

## Roadmap
1. Mesh core (this branch) · 2. Self-healing · 3. Sonar driver · 4. Sonar processing + modes ·
5. Vector encoder/decoder + web pages · 6. PC mesh simulator.
