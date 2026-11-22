b.GetDesignSettings().m_CopperEdgeClearance = mm(0.2)
def k(t):
    s,e=t.GetStart(),t.GetEnd(); return "%.2f,%.2f->%.2f,%.2f"%(s.x/1e6,s.y/1e6,e.x/1e6,e.y/1e6)
kill={"25.27,17.25->26.48,17.25","25.27,17.25->24.88,17.64","24.88,17.64->24.88,18.44"}
rm=[t for t in b.GetTracks() if t.GetNetname()=="GND" and k(t) in kill]
print("removing", len(rm))
for t in rm: b.Remove(t)
c13=b.FindFootprintByReference("C13"); c13.SetOrientationDegrees((c13.GetOrientationDegrees()+180)%360)
for p in c13.Pads(): print("C13", p.GetNumber(), p.GetNetname(), p.GetPosition().x/1e6, p.GetPosition().y/1e6)
n=b.GetNetInfo().GetNetItem("3V3")
track(b, n, [(25.27, 17.25), (25.19, 16.95)], pcbnew.F_Cu, 0.25)
via(b, n, 25.19, 16.95)
