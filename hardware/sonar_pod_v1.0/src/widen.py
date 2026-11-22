"""Widen supply traces (VB, 3V3, VBUS, VBAT, VBOOST, VPWR) to 0.5 mm wherever that keeps DRC clean (fine-pitch pins keep
the routed 0.25 mm neck). Iterates: widen all -> DRC -> revert every segment near a violation."""
import pcbnew, sys, re
src, dst = sys.argv[1], sys.argv[2]
NETS = {"LORA_ANT": 0.36, "VB": 0.5, "3V3": 0.5, "VBUS": 0.5, "VBAT": 0.6, "VBOOST": 0.5, "VDBL": 0.5, "VDRV": 0.4, "VPWR": 0.4, "XA": 0.4, "XB": 0.4}
b = pcbnew.LoadBoard(src)
segs = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK" and t.GetNetname() in NETS]
orig = {id(t): t.GetWidth() for t in segs}
for t in segs: t.SetWidth(max(t.GetWidth(), pcbnew.FromMM(NETS[t.GetNetname()])))
def violations():
    pcbnew.SaveBoard("/tmp/_w.kicad_pcb", b); bb = pcbnew.LoadBoard("/tmp/_w.kicad_pcb")
    pcbnew.ZONE_FILLER(bb).Fill(bb.Zones())
    pcbnew.WriteDRCReport(bb, "/tmp/_w.txt", pcbnew.EDA_UNITS_MILLIMETRES, False)
    s = open("/tmp/_w.txt").read(); pts = []
    for blk in re.split(r'\n\[', s):
        if blk.startswith(("clearance", "copper_edge", "hole_clearance", "shorting", "tracks_crossing")):
            pts += [(float(x), float(y)) for x, y in re.findall(r'@\((-?[\d.]+) mm, (-?[\d.]+) mm\)', blk)]
    return pts
for it in range(6):
    v = violations()
    if not v: break
    n = 0
    for t in segs:
        if t.GetWidth() == orig[id(t)]: continue
        a, c = t.GetStart(), t.GetEnd()
        for (x, y) in v:
            # distance from violation marker to the segment
            ax, ay, cx, cy = a.x / 1e6, a.y / 1e6, c.x / 1e6, c.y / 1e6
            dx, dy = cx - ax, cy - ay; L2 = dx * dx + dy * dy or 1e-9
            u = max(0, min(1, ((x - ax) * dx + (y - ay) * dy) / L2)); px, py = ax + u * dx, ay + u * dy
            if (px - x) ** 2 + (py - y) ** 2 < 1.0:
                t.SetWidth(orig[id(t)]); n += 1; break
    print("pass", it, "violations", len(v), "reverted", n)
w = sum(1 for t in segs if t.GetWidth() != orig[id(t)])
pcbnew.SaveBoard(dst, b); print("widened", w, "of", len(segs), "supply segments")
