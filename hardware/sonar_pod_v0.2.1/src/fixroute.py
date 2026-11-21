"""Hand-route leftovers: add a track (optionally via B.Cu) between two pads, then refill zones."""
import pcbnew, sys
from pcbnew import VECTOR2I, FromMM as mm
def pad(b, ref, num):
    fp=b.FindFootprintByReference(ref)
    for p in fp.Pads():
        if p.GetNumber()==str(num): return p
def track(b, net, pts, layer, w=0.25):
    for i in range(len(pts)-1):
        t=pcbnew.PCB_TRACK(b); t.SetStart(VECTOR2I(mm(pts[i][0]),mm(pts[i][1]))); t.SetEnd(VECTOR2I(mm(pts[i+1][0]),mm(pts[i+1][1]))); t.SetWidth(mm(w)); t.SetLayer(layer); t.SetNet(net); b.Add(t)
def via(b, net, x, y):
    v=pcbnew.PCB_VIA(b); v.SetPosition(VECTOR2I(mm(x),mm(y))); v.SetWidth(mm(0.7)); v.SetDrill(mm(0.35)); v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu,pcbnew.B_Cu); v.SetNet(net); b.Add(v)
if __name__=="__main__":
    b=pcbnew.LoadBoard(sys.argv[1])
    exec(open(sys.argv[2]).read())
    pcbnew.ZONE_FILLER(b).Fill(b.Zones()); pcbnew.SaveBoard(sys.argv[3], b); print("saved")
