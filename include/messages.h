/*
 * ICE FISHING MESH MONITOR - Message Definitions
 * 
 * Data structures for ESP-NOW and LoRa communication.
 * Includes deduplication and state tracking for mesh reliability.
 */

#ifndef MESSAGES_H
#define MESSAGES_H

#include <Arduino.h>
#include <alarm_latch.h>   // v2 (D41/D47) FISH ON alarm latch rules (lib/IceMesh, host-tested)

// ═══════════════════════════════════════════════════════════════════════════
// NODE ROLES
// ═══════════════════════════════════════════════════════════════════════════

enum NodeRole : uint8_t {
  ROLE_SENSOR_ONLY      = 0x01,
  ROLE_SENSOR_LORA      = 0x02,
  ROLE_RELAY_LORA       = 0x03,
  ROLE_GATEWAY_ONSHORE  = 0x04,
  ROLE_GATEWAY_OFFSHORE = 0x05
};

// ═══════════════════════════════════════════════════════════════════════════
// STATUS FLAGS
// ═══════════════════════════════════════════════════════════════════════════

#define FLAG_FISH_ON        (1 << 0)
#define FLAG_LOW_BATTERY    (1 << 1)
#define FLAG_FIRST_BOOT     (1 << 2)
#define FLAG_SIM            (1 << 3)   // v2: this node runs a simulation (fake sonar / fake trips); was FLAG_CONFIG_MODE, never used
#define FLAG_CONFIG_MODE    FLAG_SIM   // legacy name
#define FLAG_DISARMED       (1 << 4)   // v2 (D61): the hole's button / the chalet disarmed it: spool movement ignored, no FISH ON
#define FLAG_LORA_OK        FLAG_DISARMED   // legacy name (never used)
#define FLAG_ESPNOW_OK      (1 << 5)
#define FLAG_SENSOR_ERROR   (1 << 6)
#define FLAG_ACKNOWLEDGED   (1 << 7)

// BUG FIX #2: Aggregate-only flags use separate agg_flags byte in NodeStatusCompact
// This avoids collision with FLAG_CONFIG_MODE (bit 3) which sensors use
#define AGG_FLAG_NODE_OFFLINE   (1 << 0)  // Bit 0 of agg_flags: source reports node offline
#define AGG_FLAG_HOLD_EXPIRED   (1 << 1)  // Bit 1 of agg_flags: FISH_ON hold time has expired

// Legacy define for backwards compatibility - DO NOT USE IN NEW CODE
// This is kept only to prevent build errors if any old code references it
#define FLAG_NODE_OFFLINE   AGG_FLAG_NODE_OFFLINE

#define HAS_FLAG(flags, flag)   ((flags) & (flag))
#define SET_FLAG(flags, flag)   ((flags) |= (flag))
#define CLEAR_FLAG(flags, flag) ((flags) &= ~(flag))

// ═══════════════════════════════════════════════════════════════════════════
// MESSAGE TYPES
// ═══════════════════════════════════════════════════════════════════════════

enum MessageType : uint8_t {
  MSG_STATUS          = 0x01,
  MSG_ALERT           = 0x02,
  MSG_HEARTBEAT       = 0x03,
  MSG_ACK             = 0x04,
  MSG_SILENCE_SYNC    = 0x05,   // Silence alerts network-wide
  MSG_LORA_AGGREGATE  = 0x40,
  MSG_LORA_ALERT      = 0x41,
  MSG_LORA_DISCOVERY  = 0x42,
  MSG_CONFIG_UPDATE   = 0x50,   // Remote config update (supports hops)
  MSG_CONFIG_ACK      = 0x51,   // Config update acknowledgment (supports hops)
  MSG_SONAR           = 0x60,   // v2: node -> hub, [net][node][type] + one sonar block (sonar_codec.h)
  MSG_SONAR_CTRL      = 0x61,   // v2: hub -> nodes, SonarCtrlMessage (sim switch + FOCUS node)
  MSG_DEV_CMD         = 0x62,   // v2: hub -> one node, DevCmdMessage (backbone relay on/off)
  MSG_SELFTEST        = 0x63,   // D70: node -> hub, SelfTestMessage (wiring check result)
  MSG_EB_BEACON       = 0x70,   // v2: ESP-NOW backbone, chalet beacon (lib/IceMesh/src/eb_link.h)
  MSG_EB_HUB          = 0x71,   // v2: ESP-NOW backbone, hub packet (eb_link.h)
  MSG_PING            = 0x80,
  MSG_PONG            = 0x81,
  MSG_RESET_CMD       = 0x90,   // Reset all nodes command
};

// ═══════════════════════════════════════════════════════════════════════════
// ESP-NOW MESSAGES (with sequence for deduplication)
// ═══════════════════════════════════════════════════════════════════════════

// Basic sensor status message (12 bytes)
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  node_id;
  uint8_t  msg_type;
  uint8_t  flags;
  uint16_t battery_mv;
  uint16_t sequence;        // Monotonic counter for deduplication
  uint32_t uptime_sec;      // Monotonic uptime
  uint16_t turns;           // D69: quarter turns of the spool since armed / trip cleared (0 on reed heads)
} SensorMessage;
#define SENSOR_MSG_MIN_LEN 12   // D69: frames from a head without `turns` are still accepted (turns = 0)

// Alert message (16 bytes)
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  node_id;
  uint8_t  msg_type;
  uint8_t  flags;
  uint16_t battery_mv;
  uint16_t sequence;
  uint32_t uptime_sec;
  uint32_t alert_time;
  uint16_t turns;           // D69: as SensorMessage
} AlertMessage;
#define ALERT_MSG_MIN_LEN 16

// Silence sync message (12 bytes)
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  sender_id;
  uint8_t  msg_type;        // MSG_SILENCE_SYNC
  uint8_t  silence_state;   // 1=silence, 0=unsilence
  uint32_t timestamp;       // When silence was triggered
  uint32_t expire_time;     // When silence will auto-expire
} SilenceSyncMessage;

typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  node_id;
  uint8_t  msg_type;
  uint8_t  ack_node_id;
  uint8_t  ack_msg_type;
  uint8_t  reserved;
} AckMessage;

// ═══════════════════════════════════════════════════════════════════════════
// LORA MESSAGES
// ═══════════════════════════════════════════════════════════════════════════

// BUG FIX #17a: Optimized struct (10 bytes vs 14 bytes = 27% reduction per node)
// Message size: 6 header + (10 × 10 bytes) + 1 checksum = 107 bytes (was 147)
// Estimated airtime: ~950ms (down from ~1.3s at SF10/125kHz)
typedef struct __attribute__((packed)) {
  uint8_t  node_id;
  uint8_t  flags;           // Core node flags (FISH_ON, LOW_BATTERY, etc.)
  uint8_t  agg_flags;       // BUG FIX #2: Aggregate-specific flags (AGG_FLAG_NODE_OFFLINE, etc.)
  uint8_t  battery_pct;     // Battery percentage 0-100% (was battery_mv, saves 1 byte)
  uint16_t sequence;        // Include sequence in aggregate
  uint16_t uptime_min;      // Uptime in minutes (max ~45 days, saves 2 bytes vs uptime_sec)
  uint16_t fish_on_elapsed_sec;  // BUG FIX #1: Seconds since FISH_ON started (0 if not active)
} NodeStatusCompact;

typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  sender_id;
  uint8_t  msg_type;
  uint8_t  hop_count;
  uint8_t  node_count;
  uint8_t  lora_sequence;   // LoRa packet sequence (for LoRa dedup)
  uint8_t  sender_role;     // Sender's NodeRole (so receivers know gateway vs sensor)
  NodeStatusCompact nodes[10];  // Reduced to fit with larger NodeStatusCompact
  uint8_t  checksum;
} LoRaAggregateMessage;

typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  sender_id;
  uint8_t  msg_type;
  uint8_t  hop_count;
  uint8_t  origin_id;       // Original alerting node
  uint8_t  flags;
  uint16_t battery_mv;
  uint16_t sequence;        // Original node's sequence
  uint32_t uptime_sec;      // BUG FIX: Include uptime for reboot detection
  uint32_t alert_time;
} LoRaAlertMessage;

typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  node_id;
  uint8_t  msg_type;
  uint8_t  role;
  uint8_t  hop_count;
  int8_t   rssi;
  uint16_t reserved;
} LoRaDiscoveryMessage;

// LoRa silence propagation (16 bytes)
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  sender_id;
  uint8_t  msg_type;        // MSG_SILENCE_SYNC
  uint8_t  hop_count;
  uint8_t  silence_state;   // 1=silence, 0=unsilence
  uint8_t  origin_id;       // Who initiated silence
  uint16_t reserved;
  uint32_t timestamp;
  uint32_t expire_time;     // When silence will auto-expire
} LoRaSilenceSyncMessage;

// Remote configuration update (gateway to gateway via LoRa)
// Supports multi-hop relay to reach distant gateways
// 24 bytes total
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  origin_id;       // Original sender (for ACK routing)
  uint8_t  msg_type;        // MSG_CONFIG_UPDATE
  uint8_t  hop_count;       // Current hop count (increment on relay)
  uint8_t  target_id;       // Target node ID (0 = all gateways)
  uint8_t  config_seq;      // Config sequence for ACK matching & dedup
  // Config payload
  uint8_t  buzzerEnabled;   // 0 or 1
  uint16_t alertHoldSec;    // 5-300
  uint16_t heartbeatSec;    // 10-600
  uint8_t  reedActiveHigh;  // 0 or 1
  uint8_t  reserved[4];     // Future expansion
  uint8_t  checksum;        // XOR checksum
} ConfigUpdateMessage;

// Config update acknowledgment (relayed back to origin)
// 12 bytes total
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  sender_id;       // Node sending this ACK (target that applied config)
  uint8_t  msg_type;        // MSG_CONFIG_ACK
  uint8_t  hop_count;       // Current hop count (increment on relay)
  uint8_t  origin_id;       // Original config sender (ACK destination)
  uint8_t  config_seq;      // Matching sequence from config update
  uint8_t  success;         // 0 = failed, 1 = success
  uint8_t  target_id;       // Who applied the config (for dedup)
  uint8_t  reserved[3];     // Padding
  uint8_t  checksum;        // XOR checksum
} ConfigAckMessage;

// v2 sonar control - hub broadcast every second while the sonar test mode is on (or the knobs just
// changed), and right after any message from a node (the node listens briefly after it transmits).
#define SONAR_CTRL_PARAMS 32   // >= icemesh::sonar::P_COUNT (26 since D57) (lib/IceMesh/src/sonar_params.h)
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  sender_id;
  uint8_t  msg_type;        // MSG_SONAR_CTRL
  uint8_t  focus_node;      // 0 = none
  uint8_t  sim_on;          // 1 = sonar nodes generate fake data
  uint8_t  n_params;        // v2 (D42): sonar knobs that follow (0 = none)
  uint8_t  params[SONAR_CTRL_PARAMS];
} SonarCtrlMessage;

// v2 device command (6 bytes) - hub -> one node, sent right after a message from that node
// (the node listens briefly after it transmits). Relay = stay awake and rebroadcast backbone frames.
enum : uint8_t { DEVCMD_RELAY = 1, DEVCMD_SIM = 2, DEVCMD_BAIT = 3, DEVCMD_ARM = 4, DEVCMD_SELFTEST = 5 };   // DEVCMD_SELFTEST: run the wiring check, answer with MSG_SELFTEST (D70)   // DEVCMD_BAIT value: bait depth, 5 cm steps   // DEVCMD_SIM value: bit0 sonar, bit1 Hall trips, bits 2-7 trips/hour (0 = 6)   // DEVCMD_ARM value: 1 armed, 0 disarmed (D61)
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  sender_id;
  uint8_t  msg_type;        // MSG_DEV_CMD
  uint8_t  target;          // node ID
  uint8_t  cmd;             // DEVCMD_*
  uint8_t  value;           // relay: 1 on, 0 off; sim: see DEVCMD_SIM
} DevCmdMessage;

// D70 wiring self-test result (12 bytes), the same bytes on ESP-NOW (SelfTestMessage) and in the LoRa SEC_SELFTEST
// section ([node] + these 12). Built by the hole (WROOM / C3 head, or the Heltec's own hole), decoded by the chalet page.
enum : uint8_t {   // SelfTestResult.sensor
  ST_SENS_LEVEL = 0x01, ST_SENS_FLIPS = 0x02, ST_SENS_HALL = 0x04, ST_SENS_PINGED = 0x08 };   // PINGED: the shield test ping ran (the shield byte is full)
enum : uint8_t {   // SelfTestResult.button
  ST_BTN_RELEASED = 0x01, ST_BTN_PRESSED_SINCE_BOOT = 0x02, ST_BTN_NONE = 0x80 };
enum : uint8_t {   // SelfTestResult.shield
  ST_SH_SPI_OK = 0x01, ST_SH_PINS_SWAPPED = 0x02, ST_SH_VDRV_READY = 0x04, ST_SH_PULSE_FLT = 0x08,
  ST_SH_DRV_FLT = 0x10, ST_SH_EE_FLT = 0x20, ST_SH_LEAKAGE = 0x40, ST_SH_NOT_BUILT = 0x80 };
typedef struct __attribute__((packed)) {
  uint8_t  ver;             // 1
  uint8_t  sensor;          // ST_SENS_*: bit0 pin level now, bit1 flips / edges seen since boot, bit2 Hall latch logic (else reed)
  uint8_t  button;          // ST_BTN_*
  uint16_t battery_mv;      // 0 = no reading
  uint8_t  shield;          // ST_SH_*
  uint8_t  raw_max_16;      // largest raw ADC sample of the test ping / 16 (255 = clipped)
  uint8_t  ring_bins;       // bins (2.5 cm) above the noise floor after time zero: transducer ring-down (heuristic)
  uint8_t  noise_code;      // noise floor, 8-bit log code (prototype scale)
  uint8_t  flips;           // sensor edges since boot, saturates at 255
  uint8_t  presses;         // button presses since boot, saturates at 255
  uint8_t  rssi_abs;        // |RSSI| dBm of the last hub / chalet message heard, 0 = none
} SelfTestResult;
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  node_id;
  uint8_t  msg_type;        // MSG_SELFTEST
  SelfTestResult r;
} SelfTestMessage;

// Reset command message (8 bytes) - broadcast to all nodes
typedef struct __attribute__((packed)) {
  uint8_t  network_id;
  uint8_t  sender_id;
  uint8_t  msg_type;        // MSG_RESET_CMD
  uint8_t  reset_delay_sec; // Delay before reset (stagger reboots)
  uint32_t timestamp;       // When command was issued
} ResetCmdMessage;

// ═══════════════════════════════════════════════════════════════════════════
// NODE STATE TRACKING (for deduplication and state management)
// ═══════════════════════════════════════════════════════════════════════════

#define MAX_NODES 48       // v2: 10 hubs x (hub hole + 3 tip-ups) + spares (was 16)
#define ALERT_MIN_HOLD_MS 1000    // 1 second debounce - prevents false clears from reed bounce

// Per-node tracking for sequence and state
typedef struct {
  uint8_t  node_id;
  uint8_t  role;
  uint8_t  flags;
  uint16_t battery_mv;
  uint16_t last_seq;          // Highest sequence seen
  uint32_t last_uptime;       // Last uptime value (detect reboot)
  uint32_t last_seen;         // millis() when last heard (ANY source)
  uint32_t last_direct_seen;  // millis() when last heard via ESP-NOW directly
  uint32_t fish_on_time;      // millis() when FISH_ON started (0 if not)
  int8_t   rssi;
  bool     online;
  bool     initialized;       // Have we seen any message from this node?
  bool     via_espnow;        // Currently being received via ESP-NOW
  bool     via_lora;          // Currently being received via LoRa
  bool     received_direct;   // Has EVER been received via ESP-NOW (for timeout logic)
  uint8_t  clear_confirm_count;  // Consecutive FISH_ON clear confirmations
  uint16_t turns;             // D69: quarter turns since armed / trip cleared, as the hole reported
  char     name[16];
  uint8_t  grid_row;
  uint8_t  grid_col;
  icemesh::AlarmLatch alarm;  // v2 (D41/D47): the FISH ON alarm shown / sounded for this hole (alarm_latch.h)
} NodeState;

// Network state
typedef struct {
  uint8_t    network_id;
  uint8_t    node_count;
  uint8_t    alert_count;
  uint32_t   last_update;
  NodeState  nodes[MAX_NODES];
} NetworkState;

// ═══════════════════════════════════════════════════════════════════════════
// RELAY DEDUPLICATION CACHE
// ═══════════════════════════════════════════════════════════════════════════

#define DEDUP_CACHE_SIZE 32
#define DEDUP_WINDOW_MS  30000   // 30 seconds

typedef struct {
  uint8_t  node_id;
  uint16_t sequence;
  uint32_t received_at;       // millis() when cached
} DedupeEntry;

// ═══════════════════════════════════════════════════════════════════════════
// HELPERS
// ═══════════════════════════════════════════════════════════════════════════

// CRITICAL: Handle sequence number wraparound (uint16_t wraps at 65535)
inline bool isSequenceNewer(uint16_t incoming, uint16_t current) {
  // Calculate signed difference
  int32_t diff = (int32_t)incoming - (int32_t)current;

  // If difference is huge, one has wrapped around
  if (diff > 32767) return false;   // incoming wrapped, is older
  if (diff < -32767) return true;   // current wrapped, incoming is newer

  // Normal comparison
  return diff > 0;
}

// CRITICAL: Handle millis() rollover (wraps after 49.7 days)
// Always use this instead of (millis() - lastTime) to handle rollover
inline unsigned long elapsedMillis(unsigned long startTime) {
  return millis() - startTime;  // Unsigned arithmetic handles rollover automatically
}

inline bool hasElapsed(unsigned long startTime, unsigned long interval) {
  return elapsedMillis(startTime) >= interval;
}

inline uint8_t calculateChecksum(const uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

inline const char* getMessageTypeName(uint8_t type) {
  switch (type) {
    case MSG_STATUS:          return "STATUS";
    case MSG_ALERT:           return "ALERT";
    case MSG_HEARTBEAT:       return "HEARTBEAT";
    case MSG_ACK:             return "ACK";
    case MSG_SILENCE_SYNC:    return "SILENCE_SYNC";
    case MSG_LORA_AGGREGATE:  return "LORA_AGG";
    case MSG_LORA_ALERT:      return "LORA_ALERT";
    case MSG_LORA_DISCOVERY:  return "LORA_DISC";
    default:                  return "UNKNOWN";
  }
}

inline const char* getRoleName(uint8_t role) {
  switch (role) {
    case ROLE_SENSOR_ONLY:      return "SENSOR";
    case ROLE_SENSOR_LORA:      return "SENSOR+LORA";
    case ROLE_RELAY_LORA:       return "RELAY";
    case ROLE_GATEWAY_ONSHORE:  return "GW_ICE";
    case ROLE_GATEWAY_OFFSHORE: return "GW_REMOTE";
    default:                    return "UNKNOWN";
  }
}

#endif // MESSAGES_H
