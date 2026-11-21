"""Hand-route the receive network (critical, sub-5 mm) before autorouting so freerouting keeps them as fixed wiring."""
import pcbnew, sys
from pcbnew import FromMM as mm, VECTOR2I
b = pcbnew.LoadBoard(sys.argv[1])
def pad(ref, num):
    for p in b.FindFootprintByReference(ref).Pads():
        if p.GetNumber() == str(num): return p
def seg(p0, p1, net, w=0.25):
    t = pcbnew.PCB_TRACK(b); t.SetStart(VECTOR2I(mm(p0[0]), mm(p0[1]))); t.SetEnd(VECTOR2I(mm(p1[0]), mm(p1[1])))
    t.SetWidth(mm(w)); t.SetLayer(pcbnew.F_Cu); t.SetNet(net); b.Add(t)
def link(a, c, mid=None):
    pa, pc = pad(*a), pad(*c); net = pa.GetNet()
    A = (pa.GetPosition().x / 1e6, pa.GetPosition().y / 1e6); Cc = (pc.GetPosition().x / 1e6, pc.GetPosition().y / 1e6)
    pts = [A] + (mid or []) + [Cc]
    for i in range(len(pts) - 1): seg(pts[i], pts[i + 1], net)
    print("routed", pa.GetNetname(), a, "->", c)
pin = lambda ref, n: (pad(ref, n).GetPosition().x / 1e6, pad(ref, n).GetPosition().y / 1e6)
u12, u11 = pin("U1", 12), pin("U1", 11)
link(("U1", 12), ("C15", 2), [(u12[0], u12[1] + 0.9)])                       # INP: down, then diagonal to CINP
link(("U1", 11), ("C16", 1), [(u11[0], u11[1] + 0.6), (pin("C16", 1)[0] + 0.6, u11[1] + 0.6 + 0.0)])   # INN: down, left
link(("C15", 1), ("R8", 2))                                                    # RINP
r81 = pin("R8", 1)
link(("R8", 1), ("R8B", 2), [(r81[0] + 0.8, pin("R8B", 2)[1])])               # RIN1
u13 = pin("U1", 13)
link(("U1", 13), ("FB1", 1), [(u13[0], u13[1] + 0.9), (u13[0] + 0.4, u13[1] + 1.3), (u13[0] + 0.4, pin("FB1", 1)[1])])   # SGND -> ferrite
# C16 SGND side: via next to its pad, hop on B.Cu to a via on the SGND stub (crosses under INP only on another layer)
c162 = pin("C16", 2); sg = b.FindFootprintByReference("FB1").Pads()[0].GetNet() if False else pad("U1", 13).GetNet()
def via(x, y, net):
    v = pcbnew.PCB_VIA(b); v.SetPosition(VECTOR2I(mm(x), mm(y))); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3))
    v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetNet(net); b.Add(v)
v1 = (c162[0], c162[1] + 0.95); v2 = (u13[0] + 0.4, pin("FB1", 1)[1] - 1.0)
via(*v1, sg); via(*v2, sg); seg(c162, v1, sg)
t = pcbnew.PCB_TRACK(b); t.SetStart(VECTOR2I(mm(v1[0]), mm(v1[1]))); t.SetEnd(VECTOR2I(mm(v2[0]), mm(v2[1])))
t.SetWidth(mm(0.3)); t.SetLayer(pcbnew.B_Cu); t.SetNet(sg); b.Add(t); print("SGND hop", v1, "->", v2)
# USB-C: the ESP->TUSS bus runs right under the connector, so VBUS / CC1 / CC2 leave through vias and run on B.Cu
def bseg(p0, p1, net, w=0.25):
    t = pcbnew.PCB_TRACK(b); t.SetStart(VECTOR2I(mm(p0[0]), mm(p0[1]))); t.SetEnd(VECTOR2I(mm(p1[0]), mm(p1[1])))
    t.SetWidth(mm(w)); t.SetLayer(pcbnew.B_Cu); t.SetNet(net); b.Add(t)
P_ = lambda r, n: (pad(r, n).GetPosition().x / 1e6, pad(r, n).GetPosition().y / 1e6)
vb = pad("J5", "A4").GetNet(); cc1 = pad("J5", "A5").GetNet(); cc2 = pad("J5", "B5").GetNet()
vA, vB = P_("J5", "A4"), P_("J5", "B4"); c1, c2 = P_("J5", "A5"), P_("J5", "B5")
xl, xr = min(vA[0], vB[0]), max(vA[0], vB[0]); yv = vA[1] + 1.35
for x in (xl, xr): seg((x, vA[1]), (x, yv), vb, 0.3); via(x, yv, vb)
d4 = P_("D4", 2); r23 = P_("R23", 1); r24 = P_("R24", 1)
bseg((xl, yv), (xr, yv), vb, 0.4); bseg((xr, yv), (36.2, yv), vb, 0.4); bseg((36.2, yv), (36.2, 3.0), vb, 0.4)
bseg((36.2, 3.0), (37.6, 3.0), vb, 0.4); via(37.6, 3.0, vb); seg((37.6, 3.0), d4, vb, 0.4)
y1 = c1[1] + 2.45; seg(c1, (c1[0], y1), cc1, 0.2); via(c1[0], y1, cc1)
bseg((c1[0], y1), (40.8, y1), cc1, 0.2); bseg((40.8, y1), (40.8, r23[1]), cc1, 0.2); bseg((40.8, r23[1]), (r23[0] - 1.2, r23[1]), cc1, 0.2)
via(r23[0] - 1.2, r23[1], cc1); seg((r23[0] - 1.2, r23[1]), r23, cc1, 0.2)
y2 = c2[1] + 2.45; seg(c2, (c2[0], y2), cc2, 0.2); via(c2[0], y2, cc2)
bseg((c2[0], y2), (c2[0], 11.0), cc2, 0.2); bseg((c2[0], 11.0), (41.6, 11.0), cc2, 0.2); bseg((41.6, 11.0), (41.6, r24[1]), cc2, 0.2)
bseg((41.6, r24[1]), (r24[0] - 1.2, r24[1]), cc2, 0.2); via(r24[0] - 1.2, r24[1], cc2); seg((r24[0] - 1.2, r24[1]), r24, cc2, 0.2)
print("USB VBUS/CC1/CC2 pre-routed on B.Cu")
# v0.7: left-edge channel on B.Cu for the two ESP32 pins on the left side that must go to the bottom connectors:
# LED (U4.37 -> J4.4) at x 1.95, HALL (U4.38 -> down to y 30.6, then east; the router finishes it to R16/J3) at x 1.2
led = pad("U4", 37).GetNet(); hall = pad("U4", 38).GetNet()
p37, p38, j44 = P_("U4", 37), P_("U4", 38), P_("J4", 4)
for a_, c_ in ((p37, (1.1, p37[1])), ((1.1, p37[1]), (1.1, 46.9)), ((1.1, 46.9), (j44[0], 46.9)), ((j44[0], 46.9), j44)):
    bseg(a_, c_, led)      # outermost lane; HALL (U4.38, just above) is left to the router on the inner lane
ain = pad("R31", 2).GetNet(); r312 = P_("R31", 2); u86 = P_("U8", 6)
for a_, c_ in ((r312, (32.05, r312[1])), ((32.05, r312[1]), (32.05, u86[1])), ((32.05, u86[1]), u86)): seg(a_, c_, ain)
avd = pad("U8", 7).GetNet(); u87 = P_("U8", 7); c411 = P_("C41", 1)
for a_, c_ in ((u87, (36.3, u87[1])), ((36.3, u87[1]), c411)): seg(a_, c_, avd)
print("LED / HALL pre-routed down the left edge on B.Cu")
pcbnew.SaveBoard(sys.argv[2], b)
