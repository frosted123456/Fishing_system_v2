"""v1.0 fabrication export (3 variants; per-variant values; Global Sourcing line for the TDK transformer): 4-layer gerbers + drill (origin = board bottom-left), JLC BOM/CPL per variant, raw pos."""
import pcbnew, csv, subprocess, sys, os, collections, shutil
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design as D
SRC, OUT = sys.argv[1], sys.argv[2]; os.makedirs(OUT, exist_ok=True)
b = pcbnew.LoadBoard(SRC); bb = b.GetBoardEdgesBoundingBox()
b.GetDesignSettings().SetAuxOrigin(pcbnew.VECTOR2I(bb.GetLeft(), bb.GetBottom()))
fixed = os.path.join(OUT, "board_jlc.kicad_pcb"); pcbnew.SaveBoard(fixed, b)
g = os.path.join(OUT, "gerbers"); shutil.rmtree(g, ignore_errors=True); os.makedirs(g)
run = lambda *a: subprocess.run(["kicad-cli", "pcb", "export", *a], check=True, capture_output=True)
run("gerbers", "-o", g + "/", "--layers", "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts",
    "--subtract-soldermask", "--no-x2", "--use-drill-file-origin", fixed)
run("drill", "-o", g + "/", "--format", "excellon", "--excellon-units", "mm", "--drill-origin", "plot", fixed)
raw = os.path.join(OUT, "pos_raw.csv")
run("pos", "-o", raw, "--format", "csv", "--units", "mm", "--side", "front", "--use-drill-file-origin", fixed)
shutil.make_archive(os.path.join(OUT, D.BOARD_NAME + "_gerbers"), "zip", g)
ROT = {"Package_TO_SOT_SMD:SOT-23-6": -90,      # confirmed in a JLC preview (v0.2.1) and again on v1.0 (U3, U9)
       # v1.0 JLC placement preview of 21 Nov 2026 (Frank's screenshot): pin-1 dot against the pads
       "Package_TO_SOT_SMD:SOT-23": 180,          # Q3, Q4, D10, D7 were shown turned half a turn
       "Package_SO:VSSOP-8_2.3x2mm_P0.5mm": 180,  # U8 pin 1 was shown bottom-right
       "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm": -90,  # Q1 was shown with its leads on the top / bottom, pin 1 bottom-left
       "SonarPod:Texas_DSJ0014A": -90}            # U2 was shown lying down (4 x 3), pin 1 bottom-left
# correct in that preview without a correction: U1 (QFN-20), U5, D1 / D4 / D5 / D9A / D9B, D6, C49. T1 was not shown (Global Sourcing).
rows = list(csv.DictReader(open(raw)))
# Position corrections: JLC centres some parts on their pads, KiCad's footprint origin is elsewhere. v1.0 preview (21 Nov): the USB-C
# connector sat too high (toward the board edge) and Frank moved it down by hand. Offset = centre of all pads - footprint origin.
OFF = {}
for _f in b.GetFootprints():
    if D.P.get(_f.GetReference(), {}).get("fp", "").endswith("USB_C_Receptacle_HRO_TYPE-C-31-M-12"):
        _x = []; _y = []
        for _p in _f.Pads():
            _q = _p.GetBoundingBox(); _x += [_q.GetLeft(), _q.GetRight()]; _y += [_q.GetTop(), _q.GetBottom()]
        _c = _f.GetPosition()
        OFF[_f.GetReference()] = (round(((min(_x) + max(_x)) / 2 - _c.x) / 1e6, 3), round(-((min(_y) + max(_y)) / 2 - _c.y) / 1e6, 3))   # CPL y points up
        print("offset", _f.GetReference(), OFF[_f.GetReference()])
GS = "JLC Global Sourcing (Digi-Key/Mouser)"
for var in D.VARIANTS:
    inc = lambda r: r in D.P and D.fitted(r, var) and not D.P[r].get("handsolder") and (D.val(r, var)[1] or D.P[r].get("mpn"))
    with open(os.path.join(OUT, "%s_%s_cpl_jlcpcb.csv" % (D.BOARD_NAME, var)), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in rows:
            if not inc(r["Ref"]): continue
            rot = (float(r["Rot"]) + ROT.get(D.P[r["Ref"]]["fp"], 0)) % 360
            ox, oy = OFF.get(r["Ref"], (0.0, 0.0))
            w.writerow([r["Ref"], "%.3fmm" % (float(r["PosX"]) + ox), "%.3fmm" % (float(r["PosY"]) + oy), "Top", "%.0f" % rot])
    grp = collections.OrderedDict()
    for ref, p in D.P.items():
        if inc(ref):
            v, l = D.val(ref, var)
            grp.setdefault((l, p.get("jlc_comment", v), p["fp"].split(":")[1], p.get("mfr", ""), p.get("mpn", "")), []).append(ref)
    with open(os.path.join(OUT, "%s_%s_bom_jlcpcb.csv" % (D.BOARD_NAME, var)), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #", "Manufacturer", "Manufacturer Part #", "Sourcing"])
        for (l, v, fp, mfr, mpn), refs in grp.items():
            w.writerow([v, ",".join(refs), fp, l, mfr, mpn, "" if l else GS])
    print(var, "BOM lines", len(grp), "placed", sum(len(v) for v in grp.values()))
xs = [float(r["PosX"]) for r in rows]; ys = [float(r["PosY"]) for r in rows]
print("CPL range x %.1f..%.1f y %.1f..%.1f (board %.1f x %.1f)" % (min(xs), max(xs), min(ys), max(ys), bb.GetWidth() / 1e6, bb.GetHeight() / 1e6))
