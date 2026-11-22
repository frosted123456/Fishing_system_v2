"""v0.9 hand routing (v0.8 + transformer stage) before freerouting: receiver, drive links, XA/XB, VPWR/VDRV/FLT, VOUT_I, ADC input, 24 V trunk,
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
# v0.9 receive series pair (1206 + 1206): XA -> R8B -> RIN0 -> R8C -> RIN1 -> R8; RIN1 runs 2.2 mm below the XA line
r81 = P_("R8", 1); r8c2 = P_("R8C", 2)
link(("R8C", 2), ("R8", 1), [(38.9, r8c2[1] - 0.74), (38.9, 23.45), (r81[0] + 0.8, 23.45)])   # RIN1, 2 mm below XA
link(("R8B", 2), ("R8C", 1))                                                                  # RIN0

# ---- drive: OUTA / OUTB to the 0R links (R40 is a 1206 now), stubs to the xfmr gate resistors R42/R43 on B.Cu ----
u15, u16 = P_("U1", 15), P_("U1", 16)
link(("U1", 15), ("R40", 1), [(u15[0], u15[1] + 0.8)])                                        # OUTA
link(("U1", 16), ("R41", 1))                                                                  # OUTB
oa, ob = N("U1", 15), N("U1", 16)
r401, r411 = P_("R40", 1), P_("R41", 1)
r421, r431 = P_("R42", 1), P_("R43", 1)
seg(r401, (41.05, 19.0), oa); via(41.05, 19.0, oa); bseg((41.05, 19.0), (39.0, 27.4), oa); via(39.0, 27.4, oa); seg((39.0, 27.4), r421, oa)
seg(r411, (42.1, 18.2), ob); via(42.1, 18.2, ob); bseg((42.1, 18.2), (41.5, 27.6), ob); via(41.5, 27.6, ob); seg((41.5, 27.6), r431, ob)
link(("R42", 2), ("R44", 1)); link(("R43", 2), ("R45", 1))                                   # gate R -> pull-down
print("OUTA/OUTB stubs on B.Cu")

# ---- XA / XB: straight lines from the links to J2 (tuning parts sit on the lines); XA branches to R8B and T1.4 ----
xa, xb = N("J2", 1), N("J2", 2)
j21, j22 = P_("J2", 1), P_("J2", 2)
r402 = P_("R40", 2)
path([r402, (r402[0] + 0.6, 21.0), (j21[0], 21.0)], xa, 0.4)
path([(P_("R8B", 1)[0], 21.0), P_("R8B", 1)], xa, 0.4)
# v1.0: the secondary (net XS) reaches the XA line through the link R51 (1206, xfmr only); C43 hangs on XS under T1
xs = N("T1", 4); t14 = P_("T1", 4); r511, r512 = P_("R51", 1), P_("R51", 2)
seg((r511[0], 21.0), r511, xa, 0.5); seg(r512, t14, xs, 0.5)
c431 = P_("C43", 1); path([(46.75, t14[1]), (46.75, c431[1]), c431], xs, 0.4)
path([P_("R41", 2), (45.0, 18.5), (j22[0], 18.5)], xb, 0.4)
r481 = P_("R48", 1); path([r481, (r481[0], 18.5)], xb, 0.4)                                   # XB -> GND link (xfmr)
print("XA/XB direct")

# ---- transformer primary: DA straight down to the FET drains, DB on B.Cu along the east edge, VTX under the T1 body ----
db, da, vtx = N("T1", 1), N("T1", 3), N("T1", 2)          # T1.1 = DB (to Q1 D2, pins 7-8), T1.3 = DA (Q1 D1, pins 5-6)
t11, t12, t13 = P_("T1", 1), P_("T1", 2), P_("T1", 3)
q8, q7, q6, q5 = P_("Q1", 8), P_("Q1", 7), P_("Q1", 6), P_("Q1", 5)
path([t11, (q8[0], t11[1] + 1.2), q8], db, 1.0); seg(q8, q7, db, 0.6)                          # DB: 5 mm, F.Cu
for x in (t13[0] - 0.45, t13[0] + 0.45): via(x, 24.45, da)
seg(t13, (t13[0], 24.45), da, 1.0); seg((t13[0] - 0.45, 24.45), (t13[0] + 0.45, 24.45), da, 0.6)
bseg((t13[0] - 0.45, 24.45), (t13[0] + 0.45, 24.45), da, 0.6)
bseg((t13[0] + 0.45, 24.45), (55.55, 25.4), da, 0.8); bseg((55.55, 25.4), (55.55, 40.05), da, 0.8)   # DA: B.Cu, east edge
via(55.55, 40.05, da); seg((55.55, 40.05), (q5[0], 39.75), da, 0.6); seg((q5[0], 39.75), q5, da, 0.6); seg(q5, q6, da, 0.6)
c441 = P_("C44", 1); xv = 52.1
path([t12, (xv, t12[1]), (xv, 32.4), (c441[0], 32.4), c441], vtx, 0.8)                        # VTX to C44 (F.Cu, under T1)
for y in (30.0, 31.0): via(xv, y, vtx)
bseg((xv, 30.0), (xv, 31.0), vtx, 0.8)
c491 = P_("C49", 1); r462 = P_("R46", 2)
bseg((xv, 31.0), (46.3, 34.45), vtx, 0.8); bseg((46.3, 34.45), (38.6, 34.45), vtx, 0.8)        # VTX to R46 / 1000 uF (B.Cu)
bseg((38.6, 34.45), (38.0, 34.3), vtx, 0.8); via(38.0, 34.3, vtx)
c491 = P_("C49", 1); r462 = P_("R46", 2)
path([(38.0, 34.3), r462], vtx, 0.6); path([r462, (c491[0] + 1.7, c491[1] - 0.6), c491], vtx, 0.8)
ga, gb = N("Q1", 4), N("Q1", 2)
r441, r451, q4, q2 = P_("R44", 1), P_("R45", 1), P_("Q1", 4), P_("Q1", 2)
path([r441, (39.35, r441[1] + 0.4), (39.35, 34.0), (47.75, 34.0), (47.75, q4[1]), q4], ga)       # GA -> G1 (pin 4)
path([r451, (43.7, r451[1] + 0.4), (43.7, 33.4), (48.2, 33.4), (48.2, q2[1]), q2], gb)         # GB -> G2 (pin 2)
c442 = P_("C44", 2); q1 = P_("Q1", 1); path([c442, (c442[0], q1[1] - 0.6), q1], N("Q1", 1), 0.6)   # C44 GND -> S2
print("T1 primary: DA / DB / VTX")

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
# v1.0: the trunk is VDBL (doubler output). It leaves D9 / C47 on B.Cu (under the VBOOST feed), comes up at (53.0, 46.7)
# and runs up the east edge on F.Cu as before.
vb = N("R7", 1); r71 = P_("R7", 1); r141 = P_("R14", 1); dak, daa = P_("D9A", 1), P_("D9A", 2); dbk, dba = P_("D9B", 1), P_("D9B", 2); c472 = P_("C47", 2)
path([(56.4, 46.9), (56.4, 8.0), (r71[0], 8.0), r71], vb, 0.5)
via(56.4, 46.9, vb); bseg((56.4, 46.9), (54.8, 48.5), vb, 0.5); bseg((54.8, 48.5), (46.8, 48.5), vb, 0.5); bseg((46.8, 48.5), (46.4, 48.9), vb, 0.5)
via(46.4, 48.9, vb); path([(46.4, 48.9), (dbk[0], 49.7), dbk, c472], vb, 0.4)
vbo = N("C8", 1); c81 = P_("C8", 1)
path([c81, (51.5, c81[1]), (51.5, 46.9), daa], vbo, 0.3)
path([P_("R54", 2), (47.17, 37.2), (47.17, 40.4), (47.8, 41.2), P_("D1", 1)], vbo, 0.25)   # feedback top / R46 to the boost output, 5 mm, west of the gate lines             # VBOOST to the doubler, between C8 and C9
path([(r71[0], 8.0), (r141[0], 8.0), r141], vb, 0.5)
tp4 = P_("TP4", 1); seg(tp4, (tp4[0], 8.0), vb, 0.4)
link(("U3", 1), ("L1", 2), w=0.6); link(("L1", 2), ("D1", 2), w=0.6)   # MT3608 switch node: hot loop, hand-routed
link(("L1", 2), ("C50", 1), w=0.6)                                    # v1.0 doubler: flying cap at the switch node
link(("C50", 2), ("D9A", 1), w=0.4); link(("D9A", 1), ("D9B", 2), w=0.4)                                    # DBX: the only other switched node, 3 mm
link(("R8", 2), ("R53", 1)); link(("R53", 2), ("D7", 3))              # v1.0 clamp link at the receiver input (xfmr)
link(("Q3", 3), ("D10", 1), [(48.9, 2.5), (48.9, 4.0)], w=0.4)   # VDX: switch -> series diodes
q31 = P_("Q3", 1); r551 = P_("R55", 1); pg = N("Q3", 1)
path([q31, (49.3, q31[1]), r551, P_("R56", 1)], pg, 0.25)             # PG: gate - R55 - R56, one layer
path([P_("TP4", 1), (53.7, 5.8), (53.7, r551[1]), P_("R55", 2)], vb, 0.3)   # VDBL to the gate pull-up, east of D10
path([(45.675, 8.0), (45.675, 3.9), P_("Q3", 2)], vb, 0.25)              # VDBL to the switch source (corridor R24 / Q4)
ng = N("Q4", 3); r562 = P_("R56", 2)
seg(r562, (r562[0], 3.85), ng, 0.25); via(r562[0], 3.85, ng); bseg((r562[0], 3.85), (48.3, 4.6), ng, 0.25); via(48.3, 4.6, ng); seg((48.3, 4.6), P_("Q4", 3), ng, 0.25)
ve = N("Q4", 1); via(47.6, 3.6, ve); seg((47.6, 3.6), P_("Q4", 1), ve, 0.25)        # VDRV_EXT comes up on B.Cu
print("VDBL trunk, doubler, clamp link")

# ---- envelope: VOUT_I out of pin 9, west then south to R31 / R11; ADS7042 input and reference ----
vo = N("U1", 9); u9 = P_("U1", 9); r311 = P_("R31", 1); r111 = P_("R11", 1)
path([u9, (32.2, u9[1]), (30.0, 17.7), (30.0, 23.4), r311], vo); path([(30.0, 23.4), r111], vo)
ain = N("R31", 2); r312 = P_("R31", 2); u86 = P_("U8", 6)
path([r312, (32.05, r312[1]), (32.05, u86[1]), u86], ain)
avd = N("U8", 7); u87 = P_("U8", 7); c411 = P_("C41", 1)
path([u87, (36.3, u87[1]), c411], avd)
# v1.0: VOUT (envelope after the RC, to the ESP32 ADC on IO8) by hand, the v0.9 route (18 mm, 1 via): the autorouter made it 88 mm
vo = N("R11", 2); r112 = P_("R11", 2); c211 = P_("C21", 1); u412 = P_("U4", 12)
path([r112, (25.364, 24.811), (25.364, 26.989), c211, (25.237, 28.338), (25.206, 28.338)], vo, 0.25)
via(25.206, 28.338, vo); bseg((25.206, 28.338), (22.027, 25.159), vo, 0.25); bseg((22.027, 25.159), (22.027, u412[1]), vo, 0.25); bseg((22.027, u412[1]), u412, vo, 0.25)
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
