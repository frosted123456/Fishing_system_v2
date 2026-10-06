# Chalet web suite

One page served by the chalet (`/`), tabs **Holes / Sonar / Radio / Settings**. Theme, colours and the
sonar views come from Frank's prototype (docs/prototype). Light/dark follows the phone.
`/sonar`, `/radio` and `/settings` open the same page on that tab (old bookmarks keep working).

| Tab | Content | APIs |
|---|---|---|
| Holes | every hole: state (FISH ON / OK / LOW BATT / OFFLINE), battery, role, path (ESP-NOW / LoRa), last seen, rename; sonar holes show their last summary + **Watch** | `/api/status`, `/api/node/name`, `/api/sonar` |
| Sonar | sonar holes, FOCUS stream: flasher, 60 s waterfall, bottom lock, targets, echo character | `/api/sonar`, `/api/sonar/pings`, `/api/sonar/bg` |
| Radio | radio / range test mode, adaptive SF, per-hub per-mode stats | `/api/radio` |
| Settings | chalet alert settings (buzzer, hold, heartbeat, flag polarity), alert sound + units on this phone, sonar test mode, about | `/api/settings`, `/api/sonar` |

On **every tab**: header line (holes online, LoRa, Wi-Fi, uptime), TEST MODE badge, Sound on/off,
Silence button (state and minutes left come from the chalet), and a **FISH ON bar fixed at the bottom
of the screen** with the hole names and a Silence button. A new FISH ON beeps 3 times on the phone
(not while silenced). Phones only allow sound after a first tap on the page: the page says so.

| Change vs v1 pages | Why |
|---|---|
| One page instead of 5 | alert, sound and silence keep working whatever you look at |
| Silence state read from the chalet (`/api/status`: `silenced`, `silence_left_sec`) | v1 lost it on reload |
| No beep while silenced | v1 beeped anyway |
| Remote config not linked | not available over the v2 mesh yet (`/remote-config` still answers) |
| Sonar/radio polling only while their tab is open | less load on the chalet |

Size: ~58 KB in flash, sent with `send_P` (no RAM copy). No web fonts (the chalet hotspot has no internet).

Preview on a PC (fake data, no hardware): see `tools/web_preview.py` (node 3 trips 20 s every minute).
