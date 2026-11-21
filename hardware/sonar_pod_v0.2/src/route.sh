#!/bin/bash
# generate board, export DSN, add a power net class (0.5 mm), freerouting, import, DRC
set -e
cd /home/claude/pcb2
python3 make_pcb.py --nozones | tail -1
python3 make_pcb.py | tail -1
python3 - <<'EOF'
import pcbnew, re
b = pcbnew.LoadBoard('out/sonar_pod_v02_nozones.kicad_pcb')
assert pcbnew.ExportSpecctraDSN(b, 'out/sonar_pod_v02.dsn')
s = open('out/sonar_pod_v02.dsn').read()
POWER = ["VBOOST", "SW_B", "VBAT"]
m = re.search(r'\(class kicad_default ("" )?([^(]*)\(circuit', s, re.S)
nets = m.group(2).split()
rest = [n for n in nets if n not in POWER]
cls = '(class kicad_default "" %s\n      (circuit\n        (use_via Via[0-1]_700:350_um)\n      )\n      (rule\n        (width 250)\n        (clearance 200.1)\n      )\n    )\n    (class power %s\n      (circuit\n        (use_via Via[0-1]_700:350_um)\n      )\n      (rule\n        (width 500)\n        (clearance 200.1)\n      )\n    )' % (" ".join(rest), " ".join(POWER))
s2 = re.sub(r'\(class kicad_default .*?\n    \)', cls, s, count=1, flags=re.S)
open('out/sonar_pod_v02.dsn', 'w').write(s2)
print("dsn with power class")
EOF
cd out && rm -f sonar_pod_v02.ses
timeout 900 xvfb-run -a java -jar /tmp/claude-0/-home-claude/50de1734-7460-5479-87e7-cad70922642e/scratchpad/fr/freerouting.jar -de sonar_pod_v02.dsn -do sonar_pod_v02.ses -mp ${MP:-100} -dr 2>&1 | grep "INFO" | grep -i "completed" | tail -2
cd ..
python3 import_ses.py out/sonar_pod_v02.kicad_pcb out/sonar_pod_v02.ses out/sonar_pod_v02_routed.kicad_pcb
python3 - <<'EOF'
import pcbnew
b = pcbnew.LoadBoard('out/sonar_pod_v02_routed.kicad_pcb'); b.BuildConnectivity()
print("unconnected", b.GetConnectivity().GetUnconnectedCount(True))
pcbnew.WriteDRCReport(b, 'out/drc.txt', pcbnew.EDA_UNITS_MILLIMETRES, True)
EOF
grep -o "^\[[a-z_]*\]" out/drc.txt | sort | uniq -c | sort -rn
grep -A3 "^\[unconnected_items\]" out/drc.txt | grep "@" | head
