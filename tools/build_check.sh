#!/bin/bash
# Compile check of the PlatformIO envs with arduino-cli (same Arduino-ESP32 core 2.0.17 and
# library versions as platformio.ini). Used where PlatformIO cannot reach its registry.
#
#   tools/build_check.sh [env ...]      default: hub cabin relay sensor_lora sensor_c3 sensor_wroom
#
# Needs: ESP_HOME with bin/arduino-cli, arduino-cli.yaml, sketchbook/hardware/espressif/esp32
# (2.0.17) and sketchbook/libraries (RadioLib 7.7.1, U8g2 2.36.18, ArduinoJson 6.21.6).
# Mirrors build_flags of platformio.ini by hand — keep the two in sync.
# The .ino is empty (all code is in .cpp files), so ctags is replaced by a no-op script.
# NODE_NAME is not passed (arduino-cli mangles quoted string defines); the code default is used.
# Builds are incremental: if a run is cut short, run it again and it continues.
set -u
REPO="$(cd "$(dirname "$0")/.." && pwd)"
ESP_HOME="${ESP_HOME:-$HOME/esp}"
CLI="$ESP_HOME/bin/arduino-cli --config-file $ESP_HOME/arduino-cli.yaml"
OUT="${BUILD_OUT:-$HOME/esp/build}"
ENVS=("$@"); [ ${#ENVS[@]} -eq 0 ] && ENVS=(hub cabin relay sensor_lora sensor_c3 sensor_wroom)

env_cfg() {  # sets FW FQBN FLAGS for an env name
  case "$1" in
    hub)          FW=lora_node;   FQBN=espressif:esp32:heltec_wifi_lora_32_V3
                  FLAGS='-DNODE_ROLE=ROLE_GATEWAY_ONSHORE -DNODE_ID=1 -DHAS_LOCAL_SENSOR=true' ;;
    cabin)        FW=lora_node;   FQBN=espressif:esp32:heltec_wifi_lora_32_V3
                  FLAGS='-DNODE_ROLE=ROLE_GATEWAY_OFFSHORE -DNODE_ID=100 -DHAS_LOCAL_SENSOR=false' ;;
    relay)        FW=lora_node;   FQBN=espressif:esp32:heltec_wifi_lora_32_V3
                  FLAGS='-DNODE_ROLE=ROLE_RELAY_LORA -DNODE_ID=90 -DHAS_LOCAL_SENSOR=false' ;;
    sensor_lora)  FW=lora_node;   FQBN=espressif:esp32:heltec_wifi_lora_32_V3
                  FLAGS='-DNODE_ROLE=ROLE_SENSOR_LORA -DNODE_ID=50 -DHAS_LOCAL_SENSOR=true' ;;
    sensor_c3)    FW=sensor_node; FQBN=espressif:esp32:esp32c3:CDCOnBoot=cdc
                  FLAGS='-DBOARD_ESP32C3 -DNODE_ID=2' ;;
    sensor_wroom) FW=sensor_node; FQBN=espressif:esp32:esp32
                  FLAGS='-DBOARD_ESP32WROOM -DNODE_ID=3' ;;
    *) echo "unknown env $1"; return 1 ;;
  esac
  [ -n "${EXTRA_FLAGS:-}" ] && FLAGS="$FLAGS $EXTRA_FLAGS"
  return 0
}

rc=0
for e in "${ENVS[@]}"; do
  env_cfg "$e" || { rc=1; continue; }
  SK="$OUT/sketch/$e"; rm -rf "$SK"; mkdir -p "$SK"
  : > "$SK/$e.ino"                                   # arduino-cli needs <folder>.ino
  cp "$REPO"/src/$FW/* "$SK/"; cp "$REPO"/include/*.h "$SK/"
  echo "=== $e ($FQBN)"
  if $CLI compile --fqbn "$FQBN" --library "$REPO/lib/IceMesh" \
       --build-path "$OUT/build/$e" --warnings "${WARNINGS:-default}" \
       --build-property "runtime.tools.ctags.path=$ESP_HOME/tools/fakectags" \
       --build-property "compiler.cpp.extra_flags=$FLAGS" "$SK" > "$OUT/$e.log" 2>&1; then
    grep -E 'Sketch uses|Global variables' "$OUT/$e.log"
  else
    rc=1; echo "FAILED — first errors:"; grep -E 'error|Error' "$OUT/$e.log" | head -20
  fi
done
exit $rc
