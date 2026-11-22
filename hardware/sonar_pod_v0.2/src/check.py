import pcbnew, sys
b=pcbnew.LoadBoard(sys.argv[1]); W=float(sys.argv[2]); H=float(sys.argv[3])
boxes={}
for fp in b.GetFootprints():
    cy=fp.GetCourtyard(pcbnew.F_CrtYd if not fp.IsFlipped() else pcbnew.B_CrtYd)
    bb=cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False,False)
    x0,y0,x1,y1=[v/1e6 for v in (bb.GetLeft(),bb.GetTop(),bb.GetRight(),bb.GetBottom())]
    boxes[fp.GetReference()]=(x0,y0,x1,y1)
    if fp.GetReference().startswith("H"): continue
    if x0<0.3 or y0<0.3 or x1>W-0.3 or y1>H-0.3: print("OUT", fp.GetReference(), "%.1f %.1f %.1f %.1f"%(x0,y0,x1,y1))
refs=list(boxes); n=0
for i in range(len(refs)):
    for j in range(i+1,len(refs)):
        a=boxes[refs[i]]; c=boxes[refs[j]]
        if a[0]<c[2] and c[0]<a[2] and a[1]<c[3] and c[1]<a[3]: print("OVERLAP",refs[i],refs[j], "%.1f..%.1f/%.1f..%.1f vs %.1f..%.1f/%.1f..%.1f"%(a[0],a[2],a[1],a[3],c[0],c[2],c[1],c[3])); n+=1
print("overlaps", n)
