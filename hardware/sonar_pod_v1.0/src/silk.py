"""v1.0 silkscreen pass (after routing, copper untouched): every top-side designator is moved to a free spot next to its part
(no pad, no other part's body, no other text, inside the board) or hidden when there is none. Link and polarised parts first."""
import pcbnew, sys
from shapely.geometry import box
src, dst = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(src); mm = lambda v: v / 1e6
def bb(o, g=0.0):
    r = o.GetBoundingBox(); return box(mm(r.GetLeft()) - g, mm(r.GetTop()) - g, mm(r.GetRight()) + g, mm(r.GetBottom()) + g)
e = b.GetBoardEdgesBoundingBox(); edge = box(mm(e.GetLeft()) + 0.5, mm(e.GetTop()) + 0.5, mm(e.GetRight()) - 0.5, mm(e.GetBottom()) - 0.5)
obst = []; court = {}; silkg = {}
for f in b.GetFootprints():
    if f.GetLayer() != pcbnew.F_Cu:
        for p in f.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH: obst.append(bb(p, 0.15))
        continue
    for p in f.Pads(): obst.append(bb(p, 0.15))
    body = None                                   # body = fab outline + pads (the courtyard margin may carry text)
    for g in list(f.GraphicalItems()) + list(f.Pads()):
        if isinstance(g, pcbnew.PAD) or (g.GetLayer() == pcbnew.F_Fab and "TEXT" not in g.GetClass()):
            q = bb(g); body = q if body is None else box(*body.union(q).bounds)
    court[f.GetReference()] = body if body is not None else bb(f)
    silkg[f.GetReference()] = [bb(g, 0.02) for g in f.GraphicalItems() if g.GetLayer() == pcbnew.F_SilkS and "TEXT" not in g.GetClass()]
for d in b.GetDrawings():
    if d.GetLayer() == pcbnew.F_SilkS: obst.append(bb(d, 0.1))
FIRST = ["R40", "R41", "R52", "R56", "R42", "R43", "R46", "R48", "R51", "R53", "R54", "R59", "T1", "Q1", "C49", "C43", "D1", "D4", "D5", "D9A", "D9B",
         "D6", "D7", "D10", "Q3", "Q4", "U1", "U2", "U3", "U5", "U8", "U9", "TP1", "TP2", "TP4", "TP5", "TP6", "TP7", "TP8", "R4", "R5", "R58", "C12", "C50", "C47"]
fps = {f.GetReference(): f for f in b.GetFootprints() if f.GetLayer() == pcbnew.F_Cu and f.Reference().IsVisible()}
order = [r for r in FIRST if r in fps] + sorted((r for r in fps if r not in FIRST), key=lambda r: -court[r].area)
placed = []; hidden = []; moved = 0
for ref in order:
    f = fps[ref]; t = f.Reference(); c = court[ref]; x0, y0, x1, y1 = c.bounds; cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    t.SetLayer(pcbnew.F_SilkS); t.SetKeepUpright(True); ok = False
    if ref.startswith("H"): t.SetVisible(False); continue
    for sz, th, dist in ((0.8, 0.15, (0.2, 0.5, 0.9)), (0.65, 0.12, (0.15, 0.4, 0.8) + ((1.3,) if ref in FIRST else ()))):
      if ok: break
      t.SetTextSize(pcbnew.VECTOR2I(int(sz * 1e6), int(sz * 1e6))); t.SetTextThickness(int(th * 1e6))
      w = 0.9 * sz * len(ref) + 0.2; h = sz + 0.2
      for d in dist:
          cand = []
          for s_ in (0.0, -0.6, 0.6, -1.2, 1.2):
              cand += [(cx + s_, y0 - h / 2 - d, 0), (cx + s_, y1 + h / 2 + d, 0), (x1 + w / 2 + d, cy + s_, 0), (x0 - w / 2 - d, cy + s_, 0),
                       (x1 + h / 2 + d, cy + s_, 90), (x0 - h / 2 - d, cy + s_, 90), (cx + s_, y0 - w / 2 - d, 90), (cx + s_, y1 + w / 2 + d, 90)]
          for (x, y, a) in cand:
              t.SetTextAngle(pcbnew.EDA_ANGLE(a, pcbnew.DEGREES_T)); t.SetPosition(pcbnew.VECTOR2I(int(x * 1e6), int(y * 1e6)))
              tb = bb(t, 0.05)
              if not edge.contains(tb): continue
              if any(tb.intersects(o) for o in obst) or any(tb.intersects(o) for o in placed): continue
              if any(tb.intersects(k) for r2, k in court.items()): continue
              if any(tb.intersects(k) for r2, ks in silkg.items() for k in ks): continue
              ok = True; break
          if ok: break
    if ok: placed.append(bb(t, 0.1)); moved += 1
    else: t.SetVisible(False); hidden.append(ref)
pcbnew.SaveBoard(dst, b)
print("silk: %d designators placed, %d hidden (no free spot): %s" % (moved, len(hidden), " ".join(hidden)))
