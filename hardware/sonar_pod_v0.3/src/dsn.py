import pcbnew, re, sys
b = pcbnew.LoadBoard('out/sonar_pod_v03_nozones.kicad_pcb'); assert pcbnew.ExportSpecctraDSN(b, 'out/sonar_pod_v03.dsn')
s = open('out/sonar_pod_v03.dsn').read()
POWER = ["VBOOST", "SW_B", "VBAT"]
m = re.search(r'\(class kicad_default ("" )?([^(]*)\(circuit', s, re.S); nets = m.group(2).split(); rest = [n for n in nets if n not in POWER]
cls = '(class kicad_default "" %s\n      (circuit\n        (use_via Via[0-1]_700:350_um)\n      )\n      (rule\n        (width 250)\n        (clearance 200.1)\n      )\n    )\n    (class power %s\n      (circuit\n        (use_via Via[0-1]_700:350_um)\n      )\n      (rule\n        (width 400)\n        (clearance 200.1)\n      )\n    )' % (" ".join(rest), " ".join(POWER))
open('out/sonar_pod_v03.dsn', 'w').write(re.sub(r'\(class kicad_default .*?\n    \)', cls, s, count=1, flags=re.S)); print("dsn ok")
