#!/bin/bash
# post-routing half (import -> GND fan-out -> fixes -> widen -> stitch -> fill -> DRC -> audit)
set -e
cd "$(dirname "$0")"
./mkpro.sh out/sonar_pod_v10_final.kicad_pcb out/sonar_pod_v10_routed.kicad_pcb /tmp/_w.kicad_pcb
python3 make_pcb.py final >/dev/null
python3 pack_board.py out/sonar_pod_v10_final.kicad_pcb out/sonar_pod_v10_final.kicad_pcb >/dev/null
python3 import_ses.py out/sonar_pod_v10_final.kicad_pcb out/v10.ses out/sonar_pod_v10_routed.kicad_pcb
python3 fanout.py out/sonar_pod_v10_routed.kicad_pcb out/sonar_pod_v10_routed.kicad_pcb GND Bonly
python3 fixups.py out/sonar_pod_v10_routed.kicad_pcb out/sonar_pod_v10_routed.kicad_pcb fix_v10.py
python3 widen.py out/sonar_pod_v10_routed.kicad_pcb out/sonar_pod_v10_routed.kicad_pcb | tail -1
python3 stitch.py out/sonar_pod_v10_routed.kicad_pcb out/sonar_pod_v10_routed.kicad_pcb
python3 fill.py out/sonar_pod_v10_routed.kicad_pcb >/dev/null
python3 silk.py out/sonar_pod_v10_routed.kicad_pcb out/sonar_pod_v10_routed.kicad_pcb
python3 drc.py out/sonar_pod_v10_routed.kicad_pcb out/drc.txt
grep -A6 "^\[unconnected\|^\[clearance\|^\[shorting\|^\[hole_clearance" out/drc.txt | grep "@" || true
python3 noise_audit.py out/sonar_pod_v10_routed.kicad_pcb out/noise_v10.png > /dev/null 2>&1 && echo audit ok
