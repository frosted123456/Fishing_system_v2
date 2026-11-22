# post-route: connect any SMD pad still unconnected to the nearest same-net track on the other layer with one via
import math
from shapely.geometry import Point, LineString, box
from shapely.ops import unary_union
def copper_except(netname, layer):
    g = []
    for t in b.GetTracks():
        if t.GetNetname() == netname: continue
        if t.GetClass() == "PCB_VIA":
            g.append(Point(t.GetPosition().x / 1e6, t.GetPosition().y / 1e6).buffer(t.GetWidth() / 2e6))
        elif t.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu):
            g.append(LineString([(t.GetStart().x / 1e6, t.GetStart().y / 1e6), (t.GetEnd().x / 1e6, t.GetEnd().y / 1e6)]).buffer(t.GetWidth() / 2e6))
    for p in b.GetPads():
        if p.GetNetname() == netname: continue
        r = p.GetBoundingBox(); g.append(box(r.GetLeft() / 1e6, r.GetTop() / 1e6, r.GetRight() / 1e6, r.GetBottom() / 1e6))
    return unary_union(g)
def drop_via(ref, num, netname, maxd=2.0):
    pd_ = [p for p in b.FindFootprintByReference(ref).Pads() if p.GetNumber() == str(num)][0]
    P0 = Point(pd_.GetPosition().x / 1e6, pd_.GetPosition().y / 1e6)
    obst = copper_except(netname, None)
    best = None
    for t in b.GetTracks():
        if t.GetNetname() != netname or t.GetClass() != "PCB_TRACK" or t.GetLayer() != pcbnew.B_Cu: continue
        L = LineString([(t.GetStart().x / 1e6, t.GetStart().y / 1e6), (t.GetEnd().x / 1e6, t.GetEnd().y / 1e6)])
        for k in range(41):
            q = L.interpolate(k / 40, normalized=True)
            d = q.distance(P0)
            if d > maxd: continue
            _r = pd_.GetBoundingBox()
            if box(_r.GetLeft() / 1e6, _r.GetTop() / 1e6, _r.GetRight() / 1e6, _r.GetBottom() / 1e6).buffer(0.35).contains(q): continue
            if q.buffer(0.3 + 0.22).intersects(obst): continue
            stub = LineString([P0, q]).buffer(0.125 + 0.21)
            if stub.difference(P0.buffer(0.5)).intersects(obst): continue
            if best is None or d < best[0]: best = (d, q)
    if best:
        q = best[1]; via(q.x, q.y, netname); seg((P0.x, P0.y), (q.x, q.y), netname, pcbnew.F_Cu)
        print("fix: via for", ref, num, netname, "at", round(q.x, 2), round(q.y, 2))
    else:
        print("fix: no via spot for", ref, num, netname)
drop_via("R21", 1, "3V3")
# v0.7: connect two same-net SMD pads through vias and a straight B.Cu segment (used when the router leaves one open)
def _obst(netname, layer):
    g = []
    for t in b.GetTracks():
        if t.GetNetname() == netname: continue
        if t.GetClass() == "PCB_VIA":
            g.append(Point(t.GetPosition().x / 1e6, t.GetPosition().y / 1e6).buffer(t.GetWidth() / 2e6))
        elif t.GetLayer() == layer:
            g.append(LineString([(t.GetStart().x / 1e6, t.GetStart().y / 1e6), (t.GetEnd().x / 1e6, t.GetEnd().y / 1e6)]).buffer(t.GetWidth() / 2e6))
    for p in b.GetPads():
        if p.GetNetname() == netname or not p.IsOnLayer(layer): continue
        r = p.GetBoundingBox(); g.append(box(r.GetLeft() / 1e6, r.GetTop() / 1e6, r.GetRight() / 1e6, r.GetBottom() / 1e6))
    return unary_union(g)
def bridge(a, c, netname, w=0.25):
    pa = [p for p in b.FindFootprintByReference(a[0]).Pads() if p.GetNumber() == str(a[1])][0]
    pc = [p for p in b.FindFootprintByReference(c[0]).Pads() if p.GetNumber() == str(c[1])][0]
    OF, OB = _obst(netname, pcbnew.F_Cu), _obst(netname, pcbnew.B_Cu)
    def cands(p):
        P0 = Point(p.GetPosition().x / 1e6, p.GetPosition().y / 1e6); r = p.GetBoundingBox()
        pb = box(r.GetLeft() / 1e6, r.GetTop() / 1e6, r.GetRight() / 1e6, r.GetBottom() / 1e6)
        out = []
        for k in range(1, 9):
            for i in range(16):
                ang = 2 * math.pi * i / 16; q = Point(P0.x + 0.25 * k * math.cos(ang), P0.y + 0.25 * k * math.sin(ang))
                if pb.buffer(0.3).contains(q): continue
                if q.buffer(0.3 + 0.22).intersects(OF) or q.buffer(0.3 + 0.22).intersects(OB): continue
                stub = LineString([P0, q]).buffer(w / 2 + 0.21).difference(pb.buffer(0.05))
                if stub.intersects(OF): continue
                out.append(q)
        return P0, out
    A, ca = cands(pa); Cc, cc = cands(pc)
    best = None
    for qa in ca:
        for qc in cc:
            if LineString([qa, qc]).buffer(w / 2 + 0.21).intersects(OB): continue
            L = A.distance(qa) + qa.distance(qc) + qc.distance(Cc)
            if best is None or L < best[0]: best = (L, qa, qc)
    if not best: print("bridge: no path for", a, c); return
    _, qa, qc = best
    for P0, q in ((A, qa), (Cc, qc)):
        via(q.x, q.y, netname); seg((P0.x, P0.y), (q.x, q.y), netname, pcbnew.F_Cu, w)
    seg((qa.x, qa.y), (qc.x, qc.y), netname, pcbnew.B_Cu, w)
    print("bridge:", netname, a, "->", c, "%.1f mm" % best[0])
def bridge_to_b(a, netname, w=0.25, maxlen=8.0):
    pa = [p for p in b.FindFootprintByReference(a[0]).Pads() if p.GetNumber() == str(a[1])][0]
    OF, OB = _obst(netname, pcbnew.F_Cu), _obst(netname, pcbnew.B_Cu)
    A = Point(pa.GetPosition().x / 1e6, pa.GetPosition().y / 1e6); r = pa.GetBoundingBox()
    pb = box(r.GetLeft() / 1e6, r.GetTop() / 1e6, r.GetRight() / 1e6, r.GetBottom() / 1e6)
    tgt = []
    for t in b.GetTracks():
        if t.GetNetname() != netname or t.GetClass() != "PCB_TRACK" or t.GetLayer() != pcbnew.B_Cu: continue
        L = LineString([(t.GetStart().x / 1e6, t.GetStart().y / 1e6), (t.GetEnd().x / 1e6, t.GetEnd().y / 1e6)])
        tgt += [L.interpolate(k / 20, normalized=True) for k in range(21)]
    best = None
    for k in range(1, 9):
        for i in range(16):
            ang = 2 * math.pi * i / 16; q = Point(A.x + 0.25 * k * math.cos(ang), A.y + 0.25 * k * math.sin(ang))
            if pb.buffer(0.3).contains(q): continue
            if q.buffer(0.52).intersects(OF) or q.buffer(0.52).intersects(OB): continue
            if LineString([A, q]).buffer(w / 2 + 0.21).difference(pb.buffer(0.05)).intersects(OF): continue
            for z in tgt:
                if q.distance(z) > maxlen: continue
                if LineString([q, z]).buffer(w / 2 + 0.21).intersects(OB): continue
                L = A.distance(q) + q.distance(z)
                if best is None or L < best[0]: best = (L, q, z)
    if not best: print("bridge_to_b: no path for", a); return
    _, q, z = best
    via(q.x, q.y, netname); seg((A.x, A.y), (q.x, q.y), netname, pcbnew.F_Cu, w); seg((q.x, q.y), (z.x, z.y), netname, pcbnew.B_Cu, w)
    print("bridge_to_b:", netname, a, "%.1f mm" % best[0])


# J6 (hand-fitted U.FL) keep-out: drop router GND stubs from its shell pads; they join the F.Cu GND pour instead
_j6 = b.FindFootprintByReference("J6")
if _j6:
    _ends = [(p.GetPosition().x, p.GetPosition().y) for p in _j6.Pads() if p.GetNetname() == "GND"]
    for t in list(b.GetTracks()):
        if t.GetClass() == "PCB_TRACK" and t.GetNetname() == "GND" and t.GetLayer() == pcbnew.F_Cu and \
           any(abs(t.GetStart().x - x) < 2000 and abs(t.GetStart().y - y) < 2000 or abs(t.GetEnd().x - x) < 2000 and abs(t.GetEnd().y - y) < 2000 for (x, y) in _ends):
            b.Remove(t); print("J6 keep-out: removed GND stub")
