// D148: firmware update without the cable ("update mode"), hub first.
//   - The board restarts into a small mode of its own: nothing else runs (no sonar, no LoRa, no ESP-NOW), its hotspot
//     "<network>-Hub<N>-UPDATE" (same password as the other hotspots) and one page at http://192.168.4.1/ that takes a
//     firmware file built for THIS board (tools/flash.py --ota-out <folder>) and writes it to the spare firmware slot.
//   - Entered by: serial `OTA`; the arming button held 3 s at power-on; the chalet page (Settings -> Advanced ->
//     "Firmware update", sent as CMD_SELFTEST value 3). Left by itself after 10 min without an upload.
//   - Safety: the running firmware is never overwritten (two slots). The file must carry this board's own tag (role +
//     node ID) or it is refused. A new firmware that does not stay up 90 s three boots in a row is abandoned: the board
//     goes back to the slot it came from. A board with a single slot (the chalet, D74 partition table) refuses the mode.
#ifndef LORA_NODE_OTA_UPDATE_H
#define LORA_NODE_OTA_UPDATE_H
#include <stdint.h>

void otaBootCheck();        // first thing in setup(): rollback bookkeeping, then the update mode when asked (does not return)
void otaLoop();             // from loop(): confirms a new firmware once it has run 90 s
void otaRequest();          // restart into the update mode
bool otaSupported();        // false = one firmware slot only
const char* otaBoardTag();  // "ICEFW1:R<role>:N<node>:" as embedded in this firmware
#endif
