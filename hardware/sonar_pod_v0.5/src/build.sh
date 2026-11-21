#!/bin/bash
# full v0.4 pipeline: board -> pack -> GND fanout -> DSN -> freerouting -> import -> DRC
set -e
cd /home/claude/pcb6
FR=/tmp/claude-0/-home-claude/50de1734-7460-5479-87e7-cad70922642e/scratchpad/fr/freerouting.jar
python3 make_pcb.py >/dev/null
python3 pack_board.py out/sonar_pod_v05_route.kicad_pcb out/sonar_pod_v05_route.kicad_pcb | grep -v "^  " 
python3 preroute.py out/sonar_pod_v05_route.kicad_pcb out/sonar_pod_v05_route.kicad_pcb
python3 fanout.py out/sonar_pod_v05_route.kicad_pcb out/sonar_pod_v05_fan.kicad_pcb
python3 fill.py out/sonar_pod_v05_fan.kicad_pcb >/dev/null
python3 dsn.py out/sonar_pod_v05_fan.kicad_pcb out/v05.dsn
(cd out && timeout ${FRT:-420} xvfb-run -a java -jar $FR -de v05.dsn -do v05.ses -mp ${MP:-20} -mt 0 > fr.log 2>&1) || true
grep -E "completed" out/fr.log | tail -1
python3 make_pcb.py final >/dev/null
python3 pack_board.py out/sonar_pod_v05_final.kicad_pcb out/sonar_pod_v05_final.kicad_pcb >/dev/null
python3 import_ses.py out/sonar_pod_v05_final.kicad_pcb out/v05.ses out/sonar_pod_v05_routed.kicad_pcb
python3 fanout.py out/sonar_pod_v05_routed.kicad_pcb out/sonar_pod_v05_routed.kicad_pcb GND Bonly
python3 fixups.py out/sonar_pod_v05_routed.kicad_pcb out/sonar_pod_v05_routed.kicad_pcb fix_v05.py
python3 widen.py out/sonar_pod_v05_routed.kicad_pcb out/sonar_pod_v05_routed.kicad_pcb | tail -1
python3 stitch.py out/sonar_pod_v05_routed.kicad_pcb out/sonar_pod_v05_routed.kicad_pcb
python3 fill.py out/sonar_pod_v05_routed.kicad_pcb >/dev/null
python3 drc.py out/sonar_pod_v05_routed.kicad_pcb out/drc.txt
grep -A6 "^\[unconnected\|^\[clearance\|^\[shorting" out/drc.txt | grep "@" || true
python3 noise_audit.py out/sonar_pod_v05_routed.kicad_pcb out/noise_v05.png > /dev/null 2>&1 && echo audit ok
