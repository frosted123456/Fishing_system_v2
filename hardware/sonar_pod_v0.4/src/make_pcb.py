"""Build the v0.4 board (4 layers) from design.py with the pcbnew API.
Stage 'route': In1/In2 GND planes + B.Cu keepouts only (for freerouting). Stage 'final' adds F/B GND pours + SGND island."""
import pcbnew, os, sys
from pcbnew import FromMM as mm, VECTOR2I
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design as D
LIB = "/usr/share/kicad/footprints"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out"); os.makedirs(OUT, exist_ok=True)

board = pcbnew.BOARD()
board.SetCopperLayerCount(4)
board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER); board.SetLayerType(pcbnew.In2_Cu, pcbnew.LT_POWER)
board.SetLayerName(pcbnew.In1_Cu, "In1.Cu"); board.SetLayerName(pcbnew.In2_Cu, "In2.Cu")
ds = board.GetDesignSettings()
ds.m_TrackMinWidth = mm(0.2); ds.m_ViasMinSize = mm(0.6); ds.m_MinThroughDrill = mm(0.3)
ds.m_MinClearance = mm(0.2); ds.m_CopperEdgeClearance = mm(0.3)
nc = ds.m_NetSettings.m_DefaultNetClass
nc.SetTrackWidth(mm(0.25)); nc.SetClearance(mm(0.2)); nc.SetViaDiameter(mm(0.6)); nc.SetViaDrill(mm(0.3))

netinfo = {}
def net(name):
    if name not in netinfo:
        ni = pcbnew.NETINFO_ITEM(board, name); board.Add(ni); ni.thisown = 0
        netinfo[name] = board.GetNetInfo().GetNetItem(name)
    return netinfo[name]
for name in D.nets(): net(name)

def make_dsj0014a():
    fp = pcbnew.FOOTPRINT(board); fp.SetFPID(pcbnew.LIB_ID("SonarPod", "Texas_DSJ0014A")); fp.SetAttributes(pcbnew.FP_SMD)
    def pad(num, x, y, w, h):
        p = pcbnew.PAD(fp); p.SetNumber(str(num)); p.SetShape(pcbnew.PAD_SHAPE_ROUNDRECT); p.SetRoundRectRadiusRatio(0.2)
        p.SetAttribute(pcbnew.PAD_ATTRIB_SMD); p.SetSize(VECTOR2I(mm(w), mm(h))); p.SetPos0(VECTOR2I(mm(x), mm(y))); p.SetPosition(VECTOR2I(mm(x), mm(y)))
        p.SetLayerSet(pcbnew.PAD.SMDMask()); fp.Add(p)
    for i in range(7):
        y = (i - 3) * 0.5
        pad(1 + i, -1.45, y, 0.85, 0.28); pad(14 - i, 1.45, y, 0.85, 0.28)
    pad(15, 0, 0, 1.6, 2.85)
    def line(x0, y0, x1, y1, layer, w):
        s = pcbnew.FP_SHAPE(fp); s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        for f in (s.SetStart0, s.SetStart): f(VECTOR2I(mm(x0), mm(y0)))
        for f in (s.SetEnd0, s.SetEnd): f(VECTOR2I(mm(x1), mm(y1)))
        s.SetLayer(layer); s.SetWidth(mm(w)); fp.Add(s)
    line(-1.5, -2.0, 1.5, -2.0, pcbnew.F_SilkS, 0.12); line(-1.5, 2.0, 1.5, 2.0, pcbnew.F_SilkS, 0.12)
    line(-2.4, -1.9, -2.0, -1.9, pcbnew.F_SilkS, 0.15)                        # pin-1 mark
    for (a, b, c, d) in ((-2.1, -2.6, 2.1, -2.6), (2.1, -2.6, 2.1, 2.6), (2.1, 2.6, -2.1, 2.6), (-2.1, 2.6, -2.1, -2.6)):
        line(a, b, c, d, pcbnew.F_CrtYd, 0.05)
    return fp

for ref, p in D.P.items():
    lib, name = p["fp"].split(":")
    fp = make_dsj0014a() if lib == "SonarPod" else pcbnew.FootprintLoad(os.path.join(LIB, lib + ".pretty"), name)
    if fp is None: raise SystemExit("footprint not found " + p["fp"])
    for pd in list(fp.Pads()):   # module thermal vias (0.2 mm drill) -> JLC small-drill surcharge: drop
        if pd.GetAttribute() == pcbnew.PAD_ATTRIB_PTH and pd.GetDrillSize().x < 300000: fp.Remove(pd)
    fp.SetReference(ref); fp.SetValue(p["value"])
    fp.SetPosition(VECTOR2I(mm(p["pos"][0]), mm(p["pos"][1]))); fp.SetOrientationDegrees(p["rot"])
    attrs = fp.GetAttributes()
    if p.get("dnp"): attrs |= pcbnew.FP_EXCLUDE_FROM_BOM | pcbnew.FP_EXCLUDE_FROM_POS_FILES
    fp.SetAttributes(attrs)
    fp.SetProperty("LCSC", p.get("lcsc", ""))
    for pad in fp.Pads():
        num = pad.GetNumber(); n = p["pins"].get(num, "")
        try: n = n or p["pins"].get(int(num), "")
        except ValueError: pass
        if n: pad.SetNet(net(n))
    fp.Reference().SetTextSize(VECTOR2I(mm(0.8), mm(0.8))); fp.Reference().SetTextThickness(mm(0.15)); fp.Value().SetVisible(False)
    board.Add(fp)

W, H = D.BOARD_W, D.BOARD_H
def seg(x0, y0, x1, y1, layer=pcbnew.Edge_Cuts, w=0.1):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetStart(VECTOR2I(mm(x0), mm(y0))); s.SetEnd(VECTOR2I(mm(x1), mm(y1))); s.SetLayer(layer); s.SetWidth(mm(w)); board.Add(s)
seg(0, 0, W, 0); seg(W, 0, W, H); seg(W, H, 0, H); seg(0, H, 0, 0)
t = pcbnew.PCB_TEXT(board); t.SetText("SONAR POD v0.4"); t.SetPosition(VECTOR2I(mm(33), mm(35.2))); t.SetLayer(pcbnew.F_SilkS)
t.SetTextSize(VECTOR2I(mm(1.0), mm(1.0))); t.SetTextThickness(mm(0.15)); board.Add(t)

def zone(netname, layer, pts, prio=0, clearance=0.25):
    z = pcbnew.ZONE(board); z.SetNet(net(netname)); z.SetLayerSet(pcbnew.LSET(layer)); z.Outline().NewOutline()
    for (x, y) in pts: z.Outline().Append(mm(x), mm(y))
    z.SetAssignedPriority(prio); z.SetLocalClearance(mm(clearance)); z.SetMinThickness(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL); z.SetThermalReliefGap(mm(0.3)); z.SetThermalReliefSpokeWidth(mm(0.4))
    board.Add(z); return z
def keepout(layer, pts):
    z = pcbnew.ZONE(board); z.SetIsRuleArea(True); z.SetLayerSet(pcbnew.LSET(layer)); z.Outline().NewOutline()
    for (x, y) in pts: z.Outline().Append(mm(x), mm(y))
    z.SetDoNotAllowTracks(True); z.SetDoNotAllowVias(False); z.SetDoNotAllowPads(False); z.SetDoNotAllowFootprints(False)
    z.SetDoNotAllowCopperPour(False); board.Add(z)
full = [(0.4, 0.4), (W - 0.4, 0.4), (W - 0.4, H - 0.4), (0.4, H - 0.4)]
zone("GND", pcbnew.In1_Cu, full); zone("GND", pcbnew.In2_Cu, full)
for k in D.KEEPOUT_B: keepout(pcbnew.B_Cu, k)
if "final" in sys.argv:
    zone("GND", pcbnew.F_Cu, full); zone("GND", pcbnew.B_Cu, full)
    zs = zone("SGND", pcbnew.B_Cu, D.SGND_ZONE_B, prio=1); zs.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
out = os.path.join(OUT, D.BOARD_NAME + ("_final" if "final" in sys.argv else "_route") + ".kicad_pcb")
# (fill after reload: filling an unsaved board segfaults)
board.SetFileName(out); pcbnew.SaveBoard(out, board)
print("saved", out, "fps", len(board.GetFootprints()), "nets", board.GetNetCount())
