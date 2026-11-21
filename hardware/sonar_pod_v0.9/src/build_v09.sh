#!/bin/bash
# full v0.9 pipeline: board -> pack -> hand routes -> GND fanout -> DSN -> freerouting -> import -> fixes -> DRC -> audit
set -e
cd "$(dirname "$0")"
FR=${FR:-/home/claude/pcb9/tools/freerouting.jar}
mkdir -p out
./mkpro.sh out/sonar_pod_v09_route.kicad_pcb out/sonar_pod_v09_fan.kicad_pcb out/sonar_pod_v09_final.kicad_pcb out/sonar_pod_v09_routed.kicad_pcb /tmp/_w.kicad_pcb
python3 make_pcb.py >/dev/null
python3 pack_board.py out/sonar_pod_v09_route.kicad_pcb out/sonar_pod_v09_route.kicad_pcb | grep -v "^  "
python3 preroute_v09.py out/sonar_pod_v09_route.kicad_pcb out/sonar_pod_v09_route.kicad_pcb | tail -3
python3 fanout.py out/sonar_pod_v09_route.kicad_pcb out/sonar_pod_v09_fan.kicad_pcb
python3 fill.py out/sonar_pod_v09_fan.kicad_pcb >/dev/null
python3 dsn.py out/sonar_pod_v09_fan.kicad_pcb out/v09.dsn
(cd out && timeout ${FRT:-600} xvfb-run -a java -jar $FR -de v09.dsn -do v09.ses -mp ${MP:-20} -mt 0 > fr.log 2>&1) || true
grep -E "completed|unrouted|incomplete" out/fr.log | tail -2
bash post_v09.sh
