"""v0.8 hand routing before freerouting: receiver, drive links, XA/XB, VPWR/VDRV/FLT, VOUT_I, ADC input, 24 V trunk,
USB-C escapes, LED lane. Everything here is what the v0.7 autorouter got wrong (review 2026-10-15)."""
import pcbnew, sys
from pcbnew import FromMM as mm, VECTOR2I
b = pcbnew.LoadBoard(sys.argv[1])
def pad(ref, num):
    for p in b.FindFootprintByReference(ref).Pads():
        if p.GetNumber() == str(num): return p
P_ = lambda r, n: (pad(r, n).GetPosition().x / 1e6, pad(r, n).GetPosition().y / 1e6)
def seg(p0, p1, net, w=0.25, layer=pcbnew.F_Cu):
    t = pcbnew.PCB_TRACK(b); t.SetStart(VECTOR2I(mm(p0[0]), mm(p0[1]))); t.SetEnd(VECTOR2I(mm(p1[0]), mm(p1[1])))
    t.SetWidth(mm(w)); t.SetLayer(layer); t.SetNet(net); b.Add(t)
def bseg(p0, p1, net, w=0.25): seg(p0, p1, net, w, pcbnew.B_Cu)
def via(x, y, net):
    v = pcbnew.PCB_VIA(b); v.SetPosition(VECTOR2I(mm(x), mm(y))); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3))
    v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetNet(net); b.Add(v)
def path(pts, net, w=0.25, layer=pcbnew.F_Cu):
    for i in range(len(pts) - 1): seg(pts[i], pts[i + 1], net, w, layer)
def link(a, c, mid=None, w=0.25, layer=pcbnew.F_Cu):
    pa, pc = pad(*a), pad(*c); net = pa.GetNet()
    path([P_(*a)] + (mid or []) + [P_(*c)], net, w, layer); print("routed", pa.GetNetname(), a, "->", c)
N = lambda r, n: pad(r, n).GetNet()

# ---- receiver (unchanged from v0.7: 3-4 mm, no vias) ----
u12, u11 = P_("U1", 12), P_("U1", 11)
link(("U1", 12), ("C15", 2), [(u12[0], u12[1] + 0.9)])                                       # INP
link(("U1", 11), ("C16", 1), [(u11[0], u11[1] + 0.6), (P_("C16", 1)[0] + 0.6, u11[1] + 0.6)])  # INN
link(("C15", 1), ("R8", 2))                                                                   # RINP
r81 = P_("R8", 1); r8b2 = P_("R8B", 2)
link(("R8", 1), ("R8B", 2), [(r81[0] + 1.0, 23.1), (r8b2[0] - 0.9, 23.1)])                   # RIN1, above C41

# ---- drive: OUTA / OUTB to the 0R links, stubs to the DNP gate resistors on B.Cu ----
link(("U1", 15), ("R40", 1))                                                                  # OUTA straight down
link(("U1", 16), ("R41", 1))                                                                  # OUTB
oa, ob = N("U1", 15), N("U1", 16)
r401, r411 = P_("R40", 1), P_("R41", 1)
seg(r401, (39.7, 18.3), oa); via(39.7, 18.3, oa); bseg((39.7, 18.3), (40.2, 25.5), oa); via(40.2, 25.5, oa); seg((40.2, 25.5), P_("R42", 1), oa)
seg(r411, (41.2, 18.0), ob); via(41.2, 18.0, ob); bseg((41.2, 18.0), (43.3, 25.95), ob); via(43.3, 25.95, ob); seg((43.3, 25.95), P_("R43", 1), ob)
print("OUTA/OUTB stubs on B.Cu")

# ---- XA / XB: straight lines from the links to J2 (tuning parts sit on the lines), branches to R8B and P1 ----
xa, xb = N("J2", 1), N("J2", 2)
j21, j22 = P_("J2", 1), P_("J2", 2)
path([P_("R40", 2), (39.6, 21.0), (j21[0], 21.0)], xa, 0.4)
path([(P_("R8B", 1)[0], 21.0), P_("R8B", 1)], xa, 0.4)
path([P_("R41", 2), (45.0, 18.5), (j22[0], 18.5)], xb, 0.4)
p15, p16 = P_("P1", 5), P_("P1", 6)
path([j21, (52.8, 22.3), (52.8, 30.2), p15], xa, 0.4, pcbnew.B_Cu)                           # XA to the transformer port
path([j22, (55.2, 19.6), (55.2, 27.3), p16], xb, 0.4)                                        # XB to the transformer port
print("XA/XB direct")

# ---- TUSS supply corner: VPWR, VDRV, FLT, shield ferrite ----
vp = N("U1", 20); u20 = P_("U1", 20); c111 = P_("C11", 1)
path([u20, (c111[0], 13.4), c111], vp, 0.3); path([c111, (41.0, 12.3), (43.0, 11.8), P_("R7", 2)], vp, 0.3); path([P_("R7", 2), P_("C48", 1)], vp, 0.3)
link(("U1", 17), ("C12", 1), w=0.3)                                                           # VDRV
u18 = P_("U1", 18); c171 = P_("C17", 1)
path([u18, (40.2, u18[1]), (c171[0], 13.75), c171], N("U1", 18))                             # FLT
link(("J2", 3), ("FB1", 1), w=0.3)                                                            # shield -> ferrite
link(("R34", 2), ("U1", 3))                                                                   # NCS pull-up
print("VPWR/VDRV/FLT/SH")

# ---- 24 V trunk along the east edge: C9 -> R7 (VPWR) and R14 (sense), with C46 / R46 stubs ----
vb = N("C9", 1); c91 = P_("C9", 1); r71 = P_("R7", 1); r141 = P_("R14", 1)
path([c91, (54.3, 40.0), (56.4, 40.0), (56.4, 8.0), (r71[0], 8.0), r71], vb, 0.5)
path([(r71[0], 8.0), (r141[0], 8.0), r141], vb, 0.5)
c461 = P_("C46", 1); seg(c461, (c461[0], 40.0), vb, 0.5)
r461 = P_("R46", 1); seg(r461, (56.4, r461[1]), vb, 0.5)
link(("U3", 1), ("L1", 2), w=0.6); link(("L1", 2), ("D1", 2), w=0.6)   # MT3608 switch node: hot loop, hand-routed
print("VBOOST trunk")

# ---- envelope: VOUT_I out of pin 9, west then south to R31 / R11; ADS7042 input and reference ----
vo = N("U1", 9); u9 = P_("U1", 9); r311 = P_("R31", 1); r111 = P_("R11", 1)
path([u9, (32.2, u9[1]), (30.0, 17.7), (30.0, 23.4), r311], vo); path([(30.0, 23.4), r111], vo)
ain = N("R31", 2); r312 = P_("R31", 2); u86 = P_("U8", 6)
path([r312, (32.05, r312[1]), (32.05, u86[1]), u86], ain)
avd = N("U8", 7); u87 = P_("U8", 7); c411 = P_("C41", 1)
path([u87, (36.3, u87[1]), c411], avd)
print("VOUT_I / AINP / AVDD")

# ---- USB-C: VBUS / CC1 / CC2 escape on B.Cu (the ESP->TUSS bus runs under the connector), as v0.7 ----
vbus = pad("J5", "A4").GetNet(); cc1 = pad("J5", "A5").GetNet(); cc2 = pad("J5", "B5").GetNet()
vA, vB = P_("J5", "A4"), P_("J5", "B4"); c1, c2 = P_("J5", "A5"), P_("J5", "B5")
xl, xr = min(vA[0], vB[0]), max(vA[0], vB[0]); yv = vA[1] + 1.35
for x in (xl, xr): seg((x, vA[1]), (x, yv), vbus, 0.3); via(x, yv, vbus)
d4 = P_("D4", 2); r23 = P_("R23", 1); r24 = P_("R24", 1)
bseg((xl, yv), (xr, yv), vbus, 0.4); bseg((xr, yv), (36.2, yv), vbus, 0.4); bseg((36.2, yv), (36.2, 3.0), vbus, 0.4)
bseg((36.2, 3.0), (37.6, 3.0), vbus, 0.4); via(37.6, 3.0, vbus); seg((37.6, 3.0), d4, vbus, 0.4)
y1 = c1[1] + 2.45; seg(c1, (c1[0], y1), cc1, 0.2); via(c1[0], y1, cc1)
bseg((c1[0], y1), (40.8, y1), cc1, 0.2); bseg((40.8, y1), (40.8, r23[1]), cc1, 0.2); bseg((40.8, r23[1]), (r23[0] - 1.2, r23[1]), cc1, 0.2)
via(r23[0] - 1.2, r23[1], cc1); seg((r23[0] - 1.2, r23[1]), r23, cc1, 0.2)
y2 = c2[1] + 2.45; seg(c2, (c2[0], y2), cc2, 0.2); via(c2[0], y2, cc2)
bseg((c2[0], y2), (c2[0], 11.0), cc2, 0.2); bseg((c2[0], 11.0), (41.6, 11.0), cc2, 0.2); bseg((41.6, 11.0), (41.6, r24[1]), cc2, 0.2)
bseg((41.6, r24[1]), (r24[0] - 1.2, r24[1]), cc2, 0.2); via(r24[0] - 1.2, r24[1], cc2); seg((r24[0] - 1.2, r24[1]), r24, cc2, 0.2)
print("USB VBUS/CC1/CC2 pre-routed on B.Cu")
# ---- LED lane down the left edge on B.Cu (as v0.7) ----
led = N("U4", 37); p37, j44 = P_("U4", 37), P_("J4", 4)
for a_, c_ in ((p37, (1.1, p37[1])), ((1.1, p37[1]), (1.1, 46.9)), ((1.1, 46.9), (j44[0], 46.9)), ((j44[0], 46.9), j44)): bseg(a_, c_, led)
print("LED lane")
if b.FindFootprintByReference("J6"): link(("U5", 1), ("J6", 1), w=0.3)   # v0.8.1 LoRa ANT pad -> spare U.FL
pcbnew.SaveBoard(sys.argv[2], b)
