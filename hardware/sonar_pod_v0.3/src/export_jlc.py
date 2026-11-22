"""JLCPCB fabrication export (v0.2.1).
Fixes vs v0.2: (1) aux/drill origin at the board's bottom-left corner, gerbers + CPL both use it, CPL Y is taken
as KiCad writes it (already Y-up) -> no mirrored/offset parts; (2) rotation corrections from the community JLC
database (matthewlai/JLCKicadTools cpl_rotations_db.csv) for SOT-23 / QFN; (3) hand-soldered THT connectors and
DNP parts are left out of BOM and CPL; (4) BOM grouped by LCSC number."""
import pcbnew, csv, re, subprocess, sys, os, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design as D

SRC, OUT = sys.argv[1], sys.argv[2]
VARIANT = sys.argv[3] if len(sys.argv) > 3 else "all"   # tipup = no LoRa parts, lora = everything
os.makedirs(OUT, exist_ok=True)
b = pcbnew.LoadBoard(SRC)
bb = b.GetBoardEdgesBoundingBox()
origin = pcbnew.VECTOR2I(bb.GetLeft(), bb.GetBottom())
ds = b.GetDesignSettings(); ds.SetAuxOrigin(origin)
fixed = os.path.join(OUT, "board_jlc.kicad_pcb"); pcbnew.SaveBoard(fixed, b)

gdir = os.path.join(OUT, "gerbers"); os.makedirs(gdir, exist_ok=True)
run = lambda *a: subprocess.run(["kicad-cli", "pcb", "export", *a], check=True, capture_output=True)
run("gerbers", "-o", gdir + "/", "--layers", "F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts",
    "--subtract-soldermask", "--no-x2", "--use-drill-file-origin", fixed)
run("drill", "-o", gdir + "/", "--format", "excellon", "--excellon-units", "mm", "--drill-origin", "plot", fixed)
raw = os.path.join(OUT, "pos_raw.csv")
run("pos", "-o", raw, "--format", "csv", "--units", "mm", "--side", "front", "--use-drill-file-origin", fixed)

ROT = [(r"^SOT-23-6", -90)]   # v0.3: only SOT-23-6 (U3) confirmed in the v0.2.1 JLC preview; others 0 -> check preview
def corr(pkg):
    for pat, d in ROT:
        if re.search(pat, pkg): return d
    return 0
skip = lambda ref: ref.startswith(("TP", "H")) or D.P.get(ref, {}).get("dnp") or D.P.get(ref, {}).get("handsolder") or (VARIANT == "tipup" and D.P.get(ref, {}).get("variant") == "lora")
rows = list(csv.DictReader(open(raw)))
with open(os.path.join(OUT, D.BOARD_NAME + "_" + VARIANT + "_cpl_jlcpcb.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
    for r in rows:
        if skip(r["Ref"]): continue
        rot = (float(r["Rot"]) + corr(r["Package"])) % 360
        w.writerow([r["Ref"], "%.3fmm" % float(r["PosX"]), "%.3fmm" % float(r["PosY"]), "Top", "%.0f" % rot])
groups = collections.OrderedDict()
for ref, p in D.P.items():
    if skip(ref): continue
    key = (p["lcsc"], p["value"], p["fp"].split(":")[1])
    groups.setdefault(key, []).append(ref)
with open(os.path.join(OUT, D.BOARD_NAME + "_" + VARIANT + "_bom_jlcpcb.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
    for (lcsc, val, fp), refs in groups.items(): w.writerow([val, ",".join(refs), fp, lcsc])
xs = [float(r["PosX"]) for r in rows]; ys = [float(r["PosY"]) for r in rows]
print("origin bottom-left; CPL x %.1f..%.1f y %.1f..%.1f (board %.1f x %.1f)" % (min(xs), max(xs), min(ys), max(ys), bb.GetWidth() / 1e6, bb.GetHeight() / 1e6))
print("BOM lines", len(groups), "| placed parts", sum(len(v) for v in groups.values()))
