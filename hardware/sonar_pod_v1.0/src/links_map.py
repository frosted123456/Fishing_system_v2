"""links.png: where the DIRECT / XFMR link parts sit (top view), for soldering. usage: links_map.py board.kicad_pcb out.png"""
import pcbnew, sys, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import design as D
b = pcbnew.LoadBoard(sys.argv[1]); e = b.GetBoardEdgesBoundingBox(); mm = lambda v: v / 1e6
fig, ax = plt.subplots(figsize=(11, 10)); x0, y0 = mm(e.GetLeft()), mm(e.GetTop())
ax.add_patch(Rectangle((0, 0), mm(e.GetWidth()), mm(e.GetHeight()), fill=False, lw=1.5))
BIG = ("U1", "U2", "U3", "U5", "U8", "Q1", "T1", "C49", "J1", "J2", "J3", "J4", "J5", "L1", "L2", "Q3", "Q4", "D10", "D7", "TP4")
for f in b.GetFootprints():
    if f.GetLayer() != pcbnew.F_Cu: continue
    r = f.GetReference(); col = "#3b6fd4" if r in D.LINKS_DIRECT else "#e07b00" if r in D.LINKS_XFMR else "#cfcfcf"
    for p in f.Pads():
        q = p.GetBoundingBox(); ax.add_patch(Rectangle((mm(q.GetLeft()) - x0, mm(q.GetTop()) - y0), mm(q.GetWidth()), mm(q.GetHeight()), color=col, lw=0))
    c = f.GetPosition(); cx, cy = mm(c.x) - x0, mm(c.y) - y0
    if r in D.LINKS_DIRECT or r in D.LINKS_XFMR:
        ax.annotate(r, (cx, cy), xytext=(cx + (2.6 if cx < 46 else -2.6), cy - 2.2), fontsize=11, fontweight="bold", color=col, ha="center",
                    arrowprops=dict(arrowstyle="-", color=col, lw=1))
    elif r in BIG: ax.text(cx, cy, r, fontsize=8, color="#777777", ha="center", va="center")
ax.set_xlim(-1, mm(e.GetWidth()) + 1); ax.set_ylim(mm(e.GetHeight()) + 1, -1); ax.set_aspect("equal"); ax.axis("off")
ax.set_title("Pod v1.0 links, top view.  Blue = DIRECT set (%s).  Orange = XFMR set (%s).  Never both." % (", ".join(D.LINKS_DIRECT), ", ".join(D.LINKS_XFMR)), fontsize=9)
fig.savefig(sys.argv[2], dpi=130, bbox_inches="tight")
