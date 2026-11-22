"""Export the routing board to Specctra DSN with net classes (wide class for the 24 V / switch / battery nets)."""
import pcbnew, re, sys
src, dst = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(src); assert pcbnew.ExportSpecctraDSN(b, dst)
s = open(dst).read()
WIDE = ["VBOOST", "SW_B", "VBAT"]
m = re.search(r'\(class kicad_default ("" )?(.*?)\(circuit', s, re.S); nets = m.group(2).split()
rest = [n for n in nets if n not in WIDE]
cls = ('(class kicad_default "" %s\n      (circuit\n        (use_via Via[0-3]_600:300_um)\n      )\n      (rule\n        (width 250)\n        (clearance 200.1)\n      )\n    )\n'
       '    (class wide %s\n      (circuit\n        (use_via Via[0-3]_600:300_um)\n      )\n      (rule\n        (width 500)\n        (clearance 200.1)\n      )\n    )') % (" ".join(rest), " ".join(WIDE))
s = re.sub(r'\(class kicad_default .*?\n    \)', cls, s, count=1, flags=re.S)
# SGND is joined by the B.Cu zone + vias (fan-out): keep the router away from it
s = re.sub(r'(\(net SGND\n\s*\(pins)[^)]*\)', r'\1 J2-3)', s)   # single-pin net: SGND is done by B.Cu zone + vias after routing
open(dst, "w").write(s); print("dsn ok", len(nets), "nets")
