"""Resolve courtyard overlaps on the generated board: fixed parts stay; others move to the nearest free 0.25 mm grid
point around their design position (same rotation first, then +90). Writes the board and prints moves."""
import pcbnew, sys, math, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design as D
b = pcbnew.LoadBoard(sys.argv[1]); W, H = D.BOARD_W, D.BOARD_H; EDGE = 0.3
fps = {f.GetReference(): f for f in b.GetFootprints()}
def box(f):
    cy = f.GetCourtyard(pcbnew.F_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else f.GetBoundingBox(False, False)
    return (bb.GetLeft() / 1e6, bb.GetTop() / 1e6, bb.GetRight() / 1e6, bb.GetBottom() / 1e6)
def hit(a, c, g=0.02): return a[0] < c[2] - g and c[0] < a[2] - g and a[1] < c[3] - g and c[1] < a[3] - g
def inside(a, ref):
    if ref.startswith("H") or ref == "J5": return True
    return a[0] >= EDGE and a[1] >= EDGE and a[2] <= W - EDGE and a[3] <= H - EDGE
placed = {r: box(f) for r, f in fps.items() if D.P[r].get("fixed")}
order = [r for r in D.P if not D.P[r].get("fixed")]
for r in order:
    f = fps[r]; x0, y0 = D.P[r]["pos"]; rot0 = D.P[r]["rot"]
    best = None
    for k in range(0, 160):
        rad = k * 0.25; n = 1 if k == 0 else max(8, int(2 * math.pi * rad / 0.25))
        for i in range(n):
            a = 2 * math.pi * i / n
            x = round((x0 + rad * math.cos(a)) / 0.25) * 0.25; y = round((y0 + rad * math.sin(a)) / 0.25) * 0.25
            for rot in (rot0, (rot0 + 90) % 360):
                f.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))); f.SetOrientationDegrees(rot)
                bx = box(f)
                if inside(bx, r) and not any(hit(bx, o) for o in placed.values()):
                    best = (x, y, rot, bx); break
            if best: break
        if best: break
    if not best: print("COULD NOT PLACE", r); continue
    x, y, rot, bx = best; placed[r] = bx
    d = math.hypot(x - x0, y - y0)
    if d > 0.01 or rot != rot0: print("  %-4s moved %.2f mm%s -> (%.2f, %.2f)" % (r, d, " rot %d" % rot if rot != rot0 else "", x, y))
pcbnew.SaveBoard(sys.argv[2], b); print("packed", len(placed))
