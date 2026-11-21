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
