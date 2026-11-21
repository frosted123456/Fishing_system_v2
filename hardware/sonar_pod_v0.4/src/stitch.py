"""Stitching vias: on a 3 mm grid, wherever F.Cu AND B.Cu GND fills both cover a via disk + clearance (so the via
touches nothing else), add a GND via. Ties the F/B pours to In1/In2 every few mm."""
import pcbnew, sys
from pcbnew import FromMM as mm, VECTOR2I
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
src, dst = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(src); pcbnew.ZONE_FILLER(b).Fill(b.Zones())
def polys(ps):
    out = []
    for i in range(ps.OutlineCount()):
        o = ps.Outline(i); pts = [(o.CPoint(k).x / 1e6, o.CPoint(k).y / 1e6) for k in range(o.PointCount())]
        holes = [[(ps.Hole(i, h).CPoint(k).x / 1e6, ps.Hole(i, h).CPoint(k).y / 1e6) for k in range(ps.Hole(i, h).PointCount())] for h in range(ps.HoleCount(i))]
        out.append(Polygon(pts, holes).buffer(0))
    return unary_union(out)
FG = {}
for z in b.Zones():
    if z.GetNetname() != "GND" or z.GetIsRuleArea(): continue
    for l in z.GetLayerSet().Seq(): FG[l] = polys(z.GetFilledPolysList(l))
both = FG[pcbnew.F_Cu].intersection(FG[pcbnew.B_Cu]).intersection(FG[pcbnew.In1_Cu]).intersection(FG[pcbnew.In2_Cu])
crt = []
for f in b.GetFootprints():
    if f.GetReference()[0] in "UJL":     # not under modules, ICs, inductors, connectors
        cy = f.GetCourtyard(pcbnew.F_CrtYd)
        if cy.OutlineCount(): bb = cy.BBox(); crt.append(Polygon([(bb.GetLeft()/1e6, bb.GetTop()/1e6), (bb.GetRight()/1e6, bb.GetTop()/1e6), (bb.GetRight()/1e6, bb.GetBottom()/1e6), (bb.GetLeft()/1e6, bb.GetBottom()/1e6)]))
crt = unary_union(crt)
existing = [Point(t.GetPosition().x / 1e6, t.GetPosition().y / 1e6) for t in b.GetTracks() if t.GetClass() == "PCB_VIA"]
holes = [Point(p.GetPosition().x / 1e6, p.GetPosition().y / 1e6) for p in b.GetPads() if p.HasHole()]
gnd = b.GetNetInfo().GetNetItem("GND"); bb = b.GetBoardEdgesBoundingBox(); W, H = bb.GetWidth() / 1e6, bb.GetHeight() / 1e6
n = 0; STEP = 3.0
y = 1.5
while y < H - 1.0:
    x = 1.5
    while x < W - 1.0:
        p = Point(x, y)
        if both.contains(p.buffer(0.3 + 0.05)) and not crt.contains(p) and all(p.distance(q) > 1.2 for q in existing) and all(p.distance(q) > 2.0 for q in holes):
            v = pcbnew.PCB_VIA(b); v.SetPosition(VECTOR2I(mm(x), mm(y))); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3))
            v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetNet(gnd); b.Add(v); existing.append(p); n += 1
        x += STEP
    y += STEP
pcbnew.SaveBoard(dst, b); print("stitching vias", n)
