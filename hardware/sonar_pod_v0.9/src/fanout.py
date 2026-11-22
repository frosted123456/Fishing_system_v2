"""GND fan-out for the 4-layer board: every GND SMD pad gets its own via to the In1/In2 planes (short F.Cu stub);
exposed pads get a grid of vias inside the pad. Run on the routing board BEFORE autorouting."""
import pcbnew, sys, math
from pcbnew import FromMM as mm, VECTOR2I
from shapely.geometry import Point, LineString, box
from shapely.ops import unary_union
b = pcbnew.LoadBoard(sys.argv[1]); out = sys.argv[2]
NETS = sys.argv[3].split(",") if len(sys.argv) > 3 else ["GND"]
BONLY = "Bonly" in sys.argv
VR, VD, CL, TW = 0.3, 0.3, 0.22, 0.3          # via radius (0.6 dia), drill, clearance (+margin), stub width
gnd = b.GetNetInfo().GetNetItem("GND"); sgnd = b.GetNetInfo().GetNetItem("SGND")
bb = b.GetBoardEdgesBoundingBox(); W, H = bb.GetWidth() / 1e6, bb.GetHeight() / 1e6
def padpoly(p):
    r = p.GetBoundingBox(); return box(r.GetLeft() / 1e6, r.GetTop() / 1e6, r.GetRight() / 1e6, r.GetBottom() / 1e6)
pads = [(p, padpoly(p), p.GetNetname(), p.GetParent().GetReference()) for p in b.GetPads()]
def _tracks(net):
    out = []
    for t in b.GetTracks():
        if t.GetNetname() == net: continue
        if t.GetClass() == "PCB_VIA":
            q = t.GetPosition(); out.append(Point(q.x / 1e6, q.y / 1e6).buffer(t.GetWidth() / 2e6 + (0.8 if t.GetNetname() == "XA" else 0)))
        else:   # all layers: a via goes through every layer
            out.append(LineString([(t.GetStart().x / 1e6, t.GetStart().y / 1e6), (t.GetEnd().x / 1e6, t.GetEnd().y / 1e6)]).buffer(t.GetWidth() / 2e6 + (0.8 if t.GetNetname() == "XA" else 0)))
    return out
HVX = 0.8   # extra clearance around XA (HV net class 1.0 mm vs 0.2 mm)
others = lambda net: unary_union([g.buffer(HVX) if n == "XA" else g for (p, g, n, r) in pads if n != net] + _tracks(net))
vias_bad = []      # non-GND obstacles added later (none yet)
placed = []        # our GND vias (points)
crt = {}
for f in b.GetFootprints():
    cy = f.GetCourtyard(pcbnew.F_CrtYd)
    if cy.OutlineCount(): crt[f.GetReference()] = box(*(v / 1e6 for v in (cy.BBox().GetLeft(), cy.BBox().GetTop(), cy.BBox().GetRight(), cy.BBox().GetBottom())))
OBSTS = {"GND": others("GND"), "SGND": others("SGND")}; OBST = OBSTS["GND"]
def ok(pt, stub, ref):
    c = Point(pt)
    if pt[0] < 0.8 or pt[1] < 0.8 or pt[0] > W - 0.8 or pt[1] > H - 0.8: return False
    if c.buffer(VR + CL).intersects(OBST): return False
    if stub is not None and stub.buffer(TW / 2 + CL).intersects(OBST): return False
    if any(c.distance(Point(q)) < 2 * VR + 0.25 for q in placed): return False
    # v0.9: no stub or via inside an F.Cu rule area (KEEPOUT_F, hand-fitted U.FL J6)
    if any(c.buffer(VR).intersects(k) or (stub is not None and stub.intersects(k)) for k in KO): return False
    # not under another footprint's body (keeps vias out from under modules/connectors)
    if any(g.contains(c) for r2, g in crt.items() if r2 != ref and not r2.startswith(("H", "TP"))): return False
    return True
KO = []
for z in b.Zones():
    if z.GetIsRuleArea() and z.GetDoNotAllowTracks() and z.IsOnLayer(pcbnew.F_Cu):
        r = z.GetBoundingBox(); w = r.GetWidth() / 1e6
        if w < 20: KO.append(box(r.GetLeft() / 1e6, r.GetTop() / 1e6, r.GetRight() / 1e6, r.GetBottom() / 1e6))
CUR = [gnd]
def add_via(x, y):
    v = pcbnew.PCB_VIA(b); v.SetPosition(VECTOR2I(mm(x), mm(y))); v.SetWidth(mm(2 * VR)); v.SetDrill(mm(VD))
    v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetNet(CUR[0]); b.Add(v); placed.append((x, y))
LAYER = [pcbnew.F_Cu]
def add_track(a, c):
    t = pcbnew.PCB_TRACK(b); t.SetStart(VECTOR2I(mm(a[0]), mm(a[1]))); t.SetEnd(VECTOR2I(mm(c[0]), mm(c[1]))); t.SetWidth(mm(TW)); t.SetLayer(LAYER[0]); t.SetNet(CUR[0]); b.Add(t)
EP = {("U1", "21"): (2, 0.85), ("U2", "15"): (1, 0.0), ("U4", "41"): (3, 1.1)}   # n x n grid, pitch
n_ok = n_fail = 0
for (p, g, n, ref) in pads:
    if n not in NETS or p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD: continue
    LAYER[0] = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    if BONLY and LAYER[0] == pcbnew.F_Cu: continue
    if BONLY:
        _c = p.GetPosition()
        _g = g.buffer(0.05)
        if any(t.GetNetname() == n and t.GetClass() == "PCB_TRACK" and (_g.contains(Point(t.GetStart().x / 1e6, t.GetStart().y / 1e6)) or _g.contains(Point(t.GetEnd().x / 1e6, t.GetEnd().y / 1e6))) for t in b.GetTracks()):
            continue
    CUR[0] = gnd if n == "GND" else sgnd; OBST = OBSTS[n]
    c = p.GetPosition(); cx, cy = c.x / 1e6, c.y / 1e6
    key = (ref, p.GetNumber())
    if key in EP:
        if BONLY: continue
        k, pitch = EP[key]
        if n not in NETS: continue
        if ref == "U2":
            for dy in (-0.8, 0.8): add_via(cx, cy + dy)
        else:
            for i in range(k):
                for j in range(k):
                    add_via(cx + (i - (k - 1) / 2) * pitch, cy + (j - (k - 1) / 2) * pitch)
        n_ok += 1; continue
    if ref == "U4" and p.GetNumber() == "41": continue
    if ref == "U2" and p.GetNumber() in ("2", "13"):        # GND pins next to the EP: tie to the EP
        fpp = p.GetParent().GetPosition(); ex = fpp.x / 1e6 + (-0.6 if p.GetNumber() == "2" else 0.6)
        add_track((cx, cy), (ex, cy)); n_ok += 1; continue
    if ref == "U1" and p.GetNumber() == "13" and False:               # SGND pin between PGND and INP: via 1.7 mm out
        pt = (cx + 1.66, cy - 0.25); add_via(*pt); add_track((cx, cy), (cx + 0.7, cy)); add_track((cx + 0.7, cy), pt); n_ok += 1; continue
    if ref == "U1" and p.GetNumber() in ("2", "14"):        # TUSS GND/PGND next to the EP: stub straight into the EP
        fpp = p.GetParent().GetPosition(); fx, fy = fpp.x / 1e6, fpp.y / 1e6
        ex, ey = (cx + (fx - cx) * 0.55, cy + (fy - cy) * 0.55) if abs(cx - fx) < 0.6 or abs(cy - fy) < 0.6 else (cx, cy)
        if abs(cx - fx) > abs(cy - fy): ex, ey = fx + (cx - fx) * 0.5, cy
        else: ex, ey = cx, fy + (cy - fy) * 0.5
        add_track((cx, cy), (ex, ey)); n_ok += 1; continue
    if ref == "U3" and p.GetNumber() == "2":                # via under the SOT-23-6 body
        fpp = p.GetParent().GetPosition(); pt = (fpp.x / 1e6 + 0.15, fpp.y / 1e6)
        add_via(*pt); add_track((cx, cy), pt); n_ok += 1; continue
    w, h = (g.bounds[2] - g.bounds[0]) / 2, (g.bounds[3] - g.bounds[1]) / 2
    fc = crt.get(ref).centroid if ref in crt else Point(cx, cy)
    best = None
    for d in (0.0, 0.25, 0.5, 0.8, 1.2, 1.6):
        for k in range(16):
            a = 2 * math.pi * k / 16; ux, uy = math.cos(a), math.sin(a)
            reach = abs(ux) * w + abs(uy) * h
            pt = (round(cx + ux * (reach + VR + 0.15 + d), 3), round(cy + uy * (reach + VR + 0.15 + d), 3))
            stub = LineString([(cx, cy), pt])
            if not ok(pt, stub.difference(g), ref): continue
            away = Point(pt).distance(fc)        # prefer outward
            score = d * 10 - away * 0.1
            if best is None or score < best[0]: best = (score, pt)
        if best: break
    if best:
        add_via(*best[1]); add_track((cx, cy), best[1]); n_ok += 1
    else:
        n_fail += 1; print("no fanout for", ref, p.GetNumber())
pcbnew.SaveBoard(out, b); print("GND fanout: pads", n_ok, "failed", n_fail, "vias", len(placed))
