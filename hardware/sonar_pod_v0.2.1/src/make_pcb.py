"""Build sonar_pod.kicad_pcb (KiCad 7 format) from design.py with the pcbnew API: footprints from the
KiCad 7 library (+ a generated DSJ0014A), nets, placement, board outline, GND/SGND zones."""
import pcbnew, os, sys
from pcbnew import FromMM as mm, VECTOR2I, wxPoint
sys.path.insert(0, os.path.dirname(__file__))
import design as D
import pack   # v0.2: packs the non-fixed parts around their anchors (mutates D.P)

LIB = "/usr/share/kicad/footprints"
OUT = os.path.join(os.path.dirname(__file__), "out")
os.makedirs(OUT, exist_ok=True)

board = pcbnew.BOARD()
ds = board.GetDesignSettings()
ds.m_TrackMinWidth = mm(0.2); ds.m_ViasMinSize = mm(0.6); ds.m_MinThroughDrill = mm(0.2)
ds.m_MinClearance = mm(0.2); ds.m_CopperEdgeClearance = mm(0.3)
# net classes: default 0.25 track / 0.2 clearance; power wider (set later by freerouting rules)
try:
    nc = ds.m_NetSettings.m_DefaultNetClass
    nc.SetTrackWidth(mm(0.25)); nc.SetClearance(mm(0.2)); nc.SetViaDiameter(mm(0.7)); nc.SetViaDrill(mm(0.35))
except Exception as e: print("netclass:", e)

# ---- nets
netinfo = {}
def net(name):
    if name not in netinfo:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni); ni.thisown = 0
        netinfo[name] = board.GetNetInfo().GetNetItem(name)
    return netinfo[name]
for name in D.nets(): net(name)

# ---- custom footprint: TI DSJ0014A (VSON-14, 3.0 x 4.0 mm, 0.5 pitch, 7 pins per long side)
# ASSUMPTIONS TO VERIFY against TI drawing DSJ0014A: pad centres +-1.45 mm (x), pads 0.85 x 0.28, EP 1.6 x 2.6
def make_dsj0014a():
    fp = pcbnew.FOOTPRINT(board)
    fp.SetFPID(pcbnew.LIB_ID("SonarPod", "Texas_DSJ0014A"))
    fp.SetAttributes(pcbnew.FP_SMD)
    def pad(num, x, y, w, h):
        p = pcbnew.PAD(fp); p.SetNumber(str(num)); p.SetShape(pcbnew.PAD_SHAPE_ROUNDRECT); p.SetRoundRectRadiusRatio(0.2)
        p.SetAttribute(pcbnew.PAD_ATTRIB_SMD); p.SetSize(VECTOR2I(mm(w), mm(h))); p.SetPos0(VECTOR2I(mm(x), mm(y))); p.SetPosition(VECTOR2I(mm(x), mm(y)))
        p.SetLayerSet(pcbnew.PAD.SMDMask()); fp.Add(p); return p
    for i in range(7):
        y = (i - 3) * 0.5
        pad(1 + i, -1.45, y, 0.85, 0.28)        # pins 1-7 left side, top to bottom
        pad(14 - i, 1.45, y, 0.85, 0.28)        # pins 8-14 right side, bottom to top
    pad(15, 0, 0, 1.6, 2.85)                    # exposed pad = PGND (TI DSJ0014A EP ~1.58 x 2.85)
    for (x0, y0, x1, y1) in [(-1.5, -2.0, 1.5, -2.0), (-1.5, 2.0, 1.5, 2.0)]:
        s = pcbnew.FP_SHAPE(fp); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetStart0(VECTOR2I(mm(x0), mm(y0))); s.SetEnd0(VECTOR2I(mm(x1), mm(y1))); s.SetStart(VECTOR2I(mm(x0), mm(y0))); s.SetEnd(VECTOR2I(mm(x1), mm(y1)))
        s.SetLayer(pcbnew.F_SilkS); s.SetWidth(mm(0.12)); fp.Add(s)
    c = pcbnew.FP_SHAPE(fp); c.SetShape(pcbnew.SHAPE_T_CIRCLE); c.SetStart0(VECTOR2I(mm(-2.3), mm(-1.5))); c.SetEnd0(VECTOR2I(mm(-2.1), mm(-1.5))); c.SetStart(VECTOR2I(mm(-2.3), mm(-1.5))); c.SetEnd(VECTOR2I(mm(-2.1), mm(-1.5))); c.SetLayer(pcbnew.F_SilkS); c.SetWidth(mm(0.15)); fp.Add(c)
    for ly, (w, h) in ((pcbnew.F_CrtYd, (4.2, 5.2)),):
        r = pcbnew.FP_SHAPE(fp); r.SetShape(pcbnew.SHAPE_T_RECT); r.SetStart0(VECTOR2I(mm(-w / 2), mm(-h / 2))); r.SetEnd0(VECTOR2I(mm(w / 2), mm(h / 2))); r.SetStart(VECTOR2I(mm(-w / 2), mm(-h / 2))); r.SetEnd(VECTOR2I(mm(w / 2), mm(h / 2))); r.SetLayer(ly); r.SetWidth(mm(0.05)); fp.Add(r)
    return fp

# ---- place parts
for ref, p in D.P.items():
    lib, name = p["fp"].split(":")
    if lib == "SonarPod" and name == "Texas_DSJ0014A":
        fp = make_dsj0014a()
    else:
        fp = pcbnew.FootprintLoad(os.path.join(LIB, lib + ".pretty"), name)
        if fp is None: raise SystemExit(f"footprint not found {p['fp']}")
    fp.SetReference(ref); fp.SetValue(p["value"])
    x, y = p["pos"]
    fp.SetPosition(VECTOR2I(mm(x), mm(y)))
    if p["side"] == "B": fp.Flip(fp.GetPosition(), False)
    fp.SetOrientationDegrees(p["rot"])
    if p.get("dnp"): fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_EXCLUDE_FROM_BOM | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
    fp.SetProperty("LCSC", p.get("lcsc", ""))
    for pad in fp.Pads():
        num = pad.GetNumber()
        try: n = p["pins"].get(int(num), "")
        except ValueError: n = ""
        if n: pad.SetNet(net(n))
    fp.Reference().SetTextSize(VECTOR2I(mm(0.8), mm(0.8))); fp.Reference().SetTextThickness(mm(0.12))
    fp.Value().SetVisible(False)
    board.Add(fp)

# ---- outline (rounded rect)
W, H = D.BOARD_W, D.BOARD_H
def seg(x0, y0, x1, y1, layer=pcbnew.Edge_Cuts, w=0.1):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetStart(VECTOR2I(mm(x0), mm(y0))); s.SetEnd(VECTOR2I(mm(x1), mm(y1))); s.SetLayer(layer); s.SetWidth(mm(w)); board.Add(s)
r = 1.0
seg(r, 0, W - r, 0); seg(W, r, W, H - r); seg(W - r, H, r, H); seg(0, H - r, 0, r)
for (cx, cy, sx, sy) in [(r, r, -1, -1), (W - r, r, 1, -1), (W - r, H - r, 1, 1), (r, H - r, -1, 1)]:
    a = pcbnew.PCB_SHAPE(board); a.SetShape(pcbnew.SHAPE_T_ARC)
    a.SetCenter(VECTOR2I(mm(cx), mm(cy)))
    # arc from the horizontal tangent point to the vertical one, 90 deg
    if (sx, sy) == (-1, -1): a.SetStart(VECTOR2I(mm(cx), mm(cy - r))); a.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(-90, pcbnew.DEGREES_T), False)
    elif (sx, sy) == (1, -1): a.SetStart(VECTOR2I(mm(cx + r), mm(cy))); a.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(-90, pcbnew.DEGREES_T), False)
    elif (sx, sy) == (1, 1): a.SetStart(VECTOR2I(mm(cx), mm(cy + r))); a.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(-90, pcbnew.DEGREES_T), False)
    else: a.SetStart(VECTOR2I(mm(cx - r), mm(cy))); a.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(-90, pcbnew.DEGREES_T), False)
    a.SetLayer(pcbnew.Edge_Cuts); a.SetWidth(mm(0.1)); board.Add(a)

# ---- silkscreen title
t = pcbnew.PCB_TEXT(board); t.SetText("SONAR POD v0.2.1"); t.SetPosition(VECTOR2I(mm(W / 2), mm(H - 1.6))); t.SetLayer(pcbnew.F_SilkS); t.SetTextSize(VECTOR2I(mm(1.0), mm(1.0))); t.SetTextThickness(mm(0.15)); board.Add(t)

# ---- zones: GND both layers (whole board), SGND island on F.Cu around the receive pins / J2
def zone(netname, layer, pts, prio=0, clearance=0.25):
    z = pcbnew.ZONE(board); z.SetNet(net(netname)); z.SetLayerSet(pcbnew.LSET(layer))
    z.Outline().NewOutline()
    for (x, y) in pts: z.Outline().Append(mm(x), mm(y))
    z.SetAssignedPriority(prio); z.SetLocalClearance(mm(clearance)); z.SetMinThickness(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL); z.SetThermalReliefGap(mm(0.3)); z.SetThermalReliefSpokeWidth(mm(0.4))
    board.Add(z); return z
margin = 0.5
full = [(margin, margin), (W - margin, margin), (W - margin, H - margin), (margin, H - margin)]
if "--nozones" not in sys.argv:
    zone("GND", pcbnew.B_Cu, full, 0)
    zone("GND", pcbnew.F_Cu, full, 0)
# SGND island (F.Cu): INN cap, FLT cap, J2 shield pin, U1 pin 13; higher priority than GND
if "--nozones" not in sys.argv:
    zone("SGND", pcbnew.F_Cu, [(26.5, 8.0), (37.5, 8.0), (37.5, 25.0), (26.5, 25.0)], 1)

out = os.path.join(OUT, D.BOARD_NAME + ("_nozones" if "--nozones" in sys.argv else "") + ".kicad_pcb")
board.SetFileName(out)
pcbnew.SaveBoard(out, board)
print("saved", out, "footprints", len(board.GetFootprints()), "nets", board.GetNetCount())
