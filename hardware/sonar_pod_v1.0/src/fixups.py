"""Post-route hand fixes (each is checked by DRC afterwards): CC2 hop on B.Cu, dangling escape vias removed."""
import pcbnew, sys
from pcbnew import FromMM as mm, VECTOR2I
b = pcbnew.LoadBoard(sys.argv[1])
def net(n): return b.GetNetInfo().GetNetItem(n)
def seg(p0, p1, n, layer, w=0.2):
    t = pcbnew.PCB_TRACK(b); t.SetStart(VECTOR2I(mm(p0[0]), mm(p0[1]))); t.SetEnd(VECTOR2I(mm(p1[0]), mm(p1[1])))
    t.SetWidth(mm(w)); t.SetLayer(layer); t.SetNet(net(n)); b.Add(t)
def via(x, y, n):
    v = pcbnew.PCB_VIA(b); v.SetPosition(VECTOR2I(mm(x), mm(y))); v.SetWidth(mm(0.6)); v.SetDrill(mm(0.3))
    v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); v.SetNet(net(n)); b.Add(v)
exec(open(sys.argv[3]).read())
pcbnew.SaveBoard(sys.argv[2], b)
