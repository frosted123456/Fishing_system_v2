#!/bin/bash
# post-routing half of build.sh (import -> SGND fan-out -> widen -> stitch -> fill -> DRC -> audit)
set -e
cd /home/claude/pcb6
python3 make_pcb.py final >/dev/null
python3 pack_board.py out/sonar_pod_v05_final.kicad_pcb out/sonar_pod_v05_final.kicad_pcb >/dev/null
python3 import_ses.py out/sonar_pod_v05_final.kicad_pcb out/v05.ses out/sonar_pod_v05_routed.kicad_pcb
python3 fanout.py out/sonar_pod_v05_routed.kicad_pcb out/sonar_pod_v05_routed.kicad_pcb GND Bonly
python3 fixups.py out/sonar_pod_v05_routed.kicad_pcb out/sonar_pod_v05_routed.kicad_pcb fix_v05.py
python3 widen.py out/sonar_pod_v05_routed.kicad_pcb out/sonar_pod_v05_routed.kicad_pcb | tail -1
python3 stitch.py out/sonar_pod_v05_routed.kicad_pcb out/sonar_pod_v05_routed.kicad_pcb
python3 fill.py out/sonar_pod_v05_routed.kicad_pcb >/dev/null
python3 drc.py out/sonar_pod_v05_routed.kicad_pcb out/drc.txt
grep -A6 "^\[unconnected\|^\[clearance\|^\[shorting\|^\[hole_clearance" out/drc.txt | grep "@" || true
python3 noise_audit.py out/sonar_pod_v05_routed.kicad_pcb out/noise_v05.png > /dev/null 2>&1 && echo audit ok
