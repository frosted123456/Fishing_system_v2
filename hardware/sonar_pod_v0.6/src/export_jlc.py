"""v0.5 fabrication export: 4-layer gerbers + drill (origin = board bottom-left), JLC BOM/CPL per variant, raw pos."""
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
ROT = {"Package_TO_SOT_SMD:SOT-23-6": -90}     # the one correction confirmed in a JLC preview (v0.2.1); check the rest there
rows = list(csv.DictReader(open(raw)))
for var in ("tipup", "lora"):
    inc = lambda r: r in D.P and not D.P[r].get("dnp") and not D.P[r].get("handsolder") and D.P[r].get("variant") in (None, var) and D.P[r].get("lcsc")
    with open(os.path.join(OUT, "%s_%s_cpl_jlcpcb.csv" % (D.BOARD_NAME, var)), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in rows:
            if not inc(r["Ref"]): continue
            rot = (float(r["Rot"]) + ROT.get(D.P[r["Ref"]]["fp"], 0)) % 360
            w.writerow([r["Ref"], "%.3fmm" % float(r["PosX"]), "%.3fmm" % float(r["PosY"]), "Top", "%.0f" % rot])
    grp = collections.OrderedDict()
    for ref, p in D.P.items():
        if inc(ref): grp.setdefault((p["lcsc"], p["value"], p["fp"].split(":")[1]), []).append(ref)
    with open(os.path.join(OUT, "%s_%s_bom_jlcpcb.csv" % (D.BOARD_NAME, var)), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for (l, v, fp), refs in grp.items(): w.writerow([v, ",".join(refs), fp, l])
    print(var, "BOM lines", len(grp), "placed", sum(len(v) for v in grp.values()))
xs = [float(r["PosX"]) for r in rows]; ys = [float(r["PosY"]) for r in rows]
print("CPL range x %.1f..%.1f y %.1f..%.1f (board %.1f x %.1f)" % (min(xs), max(xs), min(ys), max(ys), bb.GetWidth() / 1e6, bb.GetHeight() / 1e6))
