# Ice Fishing Mesh Monitor - Bug Fixes Summary

## Overview

This document summarizes all bugs identified and fixed in the ESP32-based ice fishing tip-up monitoring system.

---

## FIXED BUGS

### BUG #1: Deep Sleep GPIO Wakeup Compilation Error (sensor_node.ino)

**Location:** `sensor_node.ino` lines 538-543

**Error:**
```
sensor_node.ino:540:3: error: 'esp_deep_sleep_enable_gpio_wakeup' was not declared in this scope
sensor_node.ino:540:3: error: 'ESP_GPIO_WAKEUP_GPIO_HIGH' was not declared in this scope
```

**Root Cause:**
The functions `esp_deep_sleep_enable_gpio_wakeup()` and `ESP_GPIO_WAKEUP_GPIO_HIGH` are ESP-IDF 5.x APIs not available in Arduino ESP32 core < 3.0. The ESP32-C3 doesn't support EXT0/EXT1 wakeup like the ESP32-WROOM.

**Fix Applied:**
1. Added proper includes for ESP-IDF version detection and GPIO driver
2. Implemented cross-compatible deep sleep GPIO wakeup code:
   - ESP-IDF 5.x (Arduino ESP32 3.x): Uses `esp_deep_sleep_enable_gpio_wakeup()`
   - ESP-IDF 4.x (Arduino ESP32 2.x): Uses `gpio_wakeup_enable()` + `esp_sleep_enable_gpio_wakeup()`
   - ESP32-WROOM/CAM: Uses `esp_sleep_enable_ext0_wakeup()`

**Files Modified:** `sensor_node/sensor_node.ino`

---

### BUG #2: Onshore Gateway Shows Offline After 1-2 Minutes (lora_node.ino)

**Symptoms:**
- Onshore module status goes offline on the offshore gateway's display/web interface
- Status never recovers to online even after silence timeout expires

**Root Cause:**
When the offshore gateway receives a LoRa aggregate from the onshore gateway, it processes the node statuses in the aggregate but does NOT explicitly refresh the **sender's** (onshore gateway's) `last_seen` timestamp. If the sender's own data in the aggregate gets rejected by deduplication logic (e.g., same sequence number), the sender node's `last_seen` is never updated, causing it to timeout and appear offline.

**Fixes Applied:**

1. **BUG FIX #13:** Added explicit sender status refresh after successful LoRa aggregate receipt (`lora_node.ino` lines 1284-1297):
   - After validating checksum, immediately update sender node's `last_seen`, `online`, `via_lora`, and `rssi`
   - This refresh happens regardless of whether individual node entries pass deduplication

2. **BUG FIX #14:** Added 5-second self-status refresh in main loop (`lora_node.ino` lines 378-387):
   - Refreshes self node's `last_seen`, `online`, and `last_uptime` every 5 seconds
   - Ensures self node data is always current for LoRa aggregate broadcasts
   - Independent of the 30-second heartbeat interval

**Files Modified:** `lora_node/lora_node.ino`

---

### BUG #3: LoRa Buffer Overflow (CRITICAL)

**Location:** `lora_node.ino` line 167

**Root Cause:**
The `loraBuffer` was only 64 bytes, but `LoRaAggregateMessage` is 107 bytes:
- Header: 6 bytes (network_id, sender_id, msg_type, hop_count, node_count, lora_sequence)
- NodeStatusCompact[10]: 10 * 10 = 100 bytes
- Checksum: 1 byte
- **Total: 107 bytes**

This caused buffer overflow corruption when receiving large aggregate messages.

**Fix Applied (BUG FIX #15):**
Increased `loraBuffer` from 64 to 128 bytes to safely accommodate all LoRa message types.

**Files Modified:** `lora_node/lora_node.ino`

---

## AUDIT RESULTS

### 1. Timing & Timeout Issues

| Check | Status | Notes |
|-------|--------|-------|
| `last_seen` updates | OK | All message handlers update `last_seen` properly |
| NODE_TIMEOUT_SEC (90s) | OK | Appropriate given HEARTBEAT_INTERVAL_SEC (30s) and LORA_TX_INTERVAL_MS (3000ms) |
| millis() rollover | OK | All timing comparisons use unsigned arithmetic which handles rollover correctly |
| `hasElapsed()` usage | OK | Used in critical timeout checks; raw comparison also safe |

### 2. Deduplication Logic

| Check | Status | Notes |
|-------|--------|-------|
| `shouldAcceptMessage()` | OK | Properly handles FISH_ON priority, sequence wraparound, and reboot detection |
| `isSequenceNewer()` | OK | Correctly handles uint16_t wraparound at boundaries |
| Reboot detection | OK | Uses uptime comparison to detect node reboots |
| Same-sequence heartbeats | OK | Returns `true` to allow `last_seen` refresh |

### 3. LoRa Communication

| Check | Status | Notes |
|-------|--------|-------|
| TX → RX mode recovery | OK | All `radio.transmit()` calls have `ensureLoRaReceiveMode()` afterward |
| LoRa watchdog | OK | 60s threshold detects and recovers stuck radio |
| `loraReceived` volatile flag | OK | Properly declared volatile for ISR safety |
| Buffer size | FIXED | Increased from 64 to 128 bytes |

### 4. ESP-NOW Communication

| Check | Status | Notes |
|-------|--------|-------|
| WiFi channel alignment | OK | Channel verification with retry on mismatch |
| `esp_wifi_set_channel()` | OK | Called after AP setup with verification |
| ESP-NOW peer configuration | OK | Broadcast peer added correctly |

### 5. State Machine & Flags

| Check | Status | Notes |
|-------|--------|-------|
| volatile for shared variables | OK | `loraReceived` properly marked volatile |
| flags field access | OK | Uses HAS_FLAG/SET_FLAG/CLEAR_FLAG macros |
| Self-node timeout protection | OK | `loopNodeTimeout()` skips NODE_ID |

### 6. Memory & Buffer Issues

| Check | Status | Notes |
|-------|--------|-------|
| LoRaAggregateMessage size | FIXED | Buffer increased to 128 bytes |
| MAX_NODES vs aggregate capacity | OK | Rotation logic handles >10 nodes |
| JSON buffer size | OK | StaticJsonDocument<2048> sufficient for 16 nodes |

---

## TEST RECOMMENDATIONS

### 1. Basic Functionality Tests

- [ ] **Compile Test:** Verify sensor_node.ino compiles for ESP32-C3 with Arduino ESP32 core 2.x and 3.x
- [ ] **Deep Sleep Wakeup:** Verify ESP32-C3 wakes from deep sleep on reed switch trigger
- [ ] **Timer Wakeup:** Verify ESP32-C3 wakes from deep sleep after heartbeat interval

### 2. Gateway Offline Bug Tests

- [ ] **Long Duration Test:** Run onshore and offshore gateways for 30+ minutes, verify both stay online
- [ ] **Network Stress Test:** Add multiple sensor nodes, verify all nodes including gateways stay online
- [ ] **Sequence Rollover Test:** Set initial sequence near 65535, verify nodes stay online after rollover

### 3. Buffer Overflow Tests

- [ ] **Full Aggregate Test:** Add 10+ nodes to network, verify large aggregates are received without corruption
- [ ] **Checksum Verification:** Monitor debug output for checksum failures after fix

### 4. Communication Tests

- [ ] **ESP-NOW Range:** Verify sensor nodes communicate reliably at expected distances
- [ ] **LoRa Range:** Verify gateway-to-gateway communication at expected distances
- [ ] **Channel Alignment:** In APSTA mode with different STA channel, verify ESP-NOW still works

### 5. Edge Cases

- [ ] **Power Cycle Recovery:** Reboot nodes and verify they rejoin network correctly
- [ ] **Rapid Alerts:** Trigger multiple alerts in quick succession, verify all are registered
- [ ] **Silence Propagation:** Verify silence state propagates to all nodes via ESP-NOW and LoRa

---

## REMAINING CONCERNS / RECOMMENDATIONS

### 1. WiFi Channel in APSTA Mode
When connecting to an external WiFi network in APSTA mode, the ESP32 is forced to use that network's channel. If this differs from ESPNOW_CHANNEL, ESP-NOW communication with sensor nodes may fail. The code warns about this but cannot fix it automatically.

**Recommendation:** Document that the external WiFi network should be on the same channel as ESPNOW_CHANNEL, or avoid STA mode in production.

### 2. LoRa Watchdog Threshold
The 60-second LoRa watchdog threshold may be too aggressive in some environments with low LoRa traffic.

**Recommendation:** Consider making this configurable or increasing to 120 seconds.

### 3. Aggregate Node Rotation
The aggregate rotation logic handles >10 nodes, but nodes not included in a particular aggregate won't have their status refreshed on the remote gateway until they appear in a subsequent aggregate.

**Recommendation:** For networks with >10 nodes, consider increasing LORA_TX_INTERVAL_MS to ensure all nodes are transmitted within NODE_TIMEOUT_SEC.

---

## FILES MODIFIED

1. `sensor_node/sensor_node.ino` - Deep sleep GPIO wakeup fix
2. `lora_node/lora_node.ino` - Gateway offline fix, buffer overflow fix

## VERSION HISTORY

- **2026-01-03:** Initial bug fixes applied
  - Fixed deep sleep GPIO wakeup compilation error
  - Fixed gateway offline bug
  - Fixed LoRa buffer overflow
  - Completed systematic audit
