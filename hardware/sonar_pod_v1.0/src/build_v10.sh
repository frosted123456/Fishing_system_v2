#!/bin/bash
# full v1.0 pipeline: board -> pack -> hand routes -> GND fanout -> DSN -> freerouting -> import -> fixes -> DRC -> audit
set -e
cd "$(dirname "$0")"
FR=${FR:-../tools/freerouting.jar}   # freerouting 1.9.0 jar; the routing is not fully repeatable: rerun (MP=100) until DRC shows 0 unconnected
mkdir -p out
./mkpro.sh out/sonar_pod_v10_route.kicad_pcb out/sonar_pod_v10_fan.kicad_pcb out/sonar_pod_v10_final.kicad_pcb out/sonar_pod_v10_routed.kicad_pcb /tmp/_w.kicad_pcb
python3 make_pcb.py >/dev/null
python3 pack_board.py out/sonar_pod_v10_route.kicad_pcb out/sonar_pod_v10_route.kicad_pcb | grep -v "^  "
python3 preroute_v10.py out/sonar_pod_v10_route.kicad_pcb out/sonar_pod_v10_route.kicad_pcb | tail -3
python3 fanout.py out/sonar_pod_v10_route.kicad_pcb out/sonar_pod_v10_fan.kicad_pcb
python3 fill.py out/sonar_pod_v10_fan.kicad_pcb >/dev/null
python3 dsn.py out/sonar_pod_v10_fan.kicad_pcb out/v10.dsn
(cd out && timeout ${FRT:-600} xvfb-run -a java -jar $FR -de v10.dsn -do v10.ses -mp ${MP:-20} -mt 0 > fr.log 2>&1) || true
grep -E "completed|unrouted|incomplete" out/fr.log | tail -2
bash post_v10.sh
