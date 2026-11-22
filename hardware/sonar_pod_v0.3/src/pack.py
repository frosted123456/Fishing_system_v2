"""Greedy courtyard packer for v0.2: FIXED parts keep their position; the others are placed at the nearest free
0.25 mm grid point around an anchor (their design position), trying rot 0 and 90. Writes positions back into design.P."""
import pcbnew, os, sys, math
sys.path.insert(0, os.path.dirname(__file__))
import design as D
LIB = "/usr/share/kicad/footprints"

FIXED = {"J1", "J2", "J3", "J4", "J5", "J6", "U1", "U2", "U3", "U4", "U5", "U6", "L1", "L2", "H1", "H2", "H3"}
ORDER = [  # pack order: closest-to-IC things first
    "C16", "C15", "C17", "C12", "C11", "C13", "C14", "R8", "C18", "C19", "FB1", "R9", "D3", "R11", "C21", "R10", "C20", "R7",
    "C10", "D1", "C8", "C9", "R4", "R5", "R6", "D2", "R14", "R15", "C23",
    "C3", "C4", "C5", "C6", "C7", "C2", "R17", "R2", "R3",
    "Q1", "R1", "C1", "R12", "R13", "C22", "R16", "C24", "TP1", "TP3",
    "D5", "D4", "C30", "C31", "R20", "C32", "R21", "R22", "D6", "C36", "C37", "R29", "U7", "C33", "C34", "C35",
    "R23", "R24", "R25", "R26", "Q2", "Q3", "R27", "R28"]

def cy_box(fp, rot):
    fp.SetOrientationDegrees(rot)
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    bb = cy.BBox() if cy.OutlineCount() else fp.GetBoundingBox(False, False)
    c = fp.GetPosition()
    return (bb.GetLeft() - c.x) / 1e6, (bb.GetTop() - c.y) / 1e6, (bb.GetRight() - c.x) / 1e6, (bb.GetBottom() - c.y) / 1e6

def load(ref):
    lib, name = D.P[ref]["fp"].split(":")
    if lib == "SonarPod":   # generated DSJ0014A: courtyard 4.2 x 5.2
        return None
    return pcbnew.FootprintLoad(os.path.join(LIB, lib + ".pretty"), name)

boxes = {}   # ref -> absolute (x0,y0,x1,y1)
rel = {}
for ref in D.P:
    fp = load(ref)
    if fp is None:
        rel[ref] = {0: (-2.1, -2.6, 2.1, 2.6), 90: (-2.6, -2.1, 2.6, 2.1)}
    else:
        rel[ref] = {r: cy_box(fp, r) for r in (0, 90, 180, 270)}
W, H = D.BOARD_W, D.BOARD_H
EDGE = 0.6
def place_box(ref, x, y, rot):
    r = rel[ref][rot]; return (x + r[0], y + r[1], x + r[2], y + r[3])
def free(b, ignore=None):
    if b[0] < EDGE or b[1] < EDGE or b[2] > W - EDGE or b[3] > H - EDGE: return False
    for k, o in boxes.items():
        if k == ignore: continue
        if b[0] < o[2] - 0.001 and o[0] < b[2] - 0.001 and b[1] < o[3] - 0.001 and o[1] < b[3] - 0.001: return False
    return True

for ref in FIXED:
    p = D.P[ref]; boxes[ref] = place_box(ref, p["pos"][0], p["pos"][1], p["rot"] % 360)
    if not free(boxes[ref], ignore=ref): print("WARNING fixed part out of board or overlapping:", ref, boxes[ref])

GRID = 0.25
for ref in ORDER:
    if ref not in D.P: continue
    p = D.P[ref]; ax, ay = p["pos"]
    best = None
    for radius_i in range(0, 200):
        rad = radius_i * GRID
        cands = []
        n = max(8, int(2 * math.pi * rad / GRID)) if rad > 0 else 1
        for k in range(n):
            a = 2 * math.pi * k / n
            cands.append((round((ax + rad * math.cos(a)) / GRID) * GRID, round((ay + rad * math.sin(a)) / GRID) * GRID))
        for (x, y) in cands:
            for rot in ((p["rot"] % 360, (p["rot"] + 90) % 360)):
                b = place_box(ref, x, y, rot)
                if free(b):
                    best = (x, y, rot); break
            if best: break
        if best: break
    if not best: print("COULD NOT PLACE", ref); continue
    x, y, rot = best
    boxes[ref] = place_box(ref, x, y, rot)
    p["pos"] = (round(x, 2), round(y, 2)); p["rot"] = rot
    d = math.hypot(x - ax, y - ay)
    if d > 3: print("  %s moved %.1f mm from its anchor" % (ref, d))
print("packed", len(boxes), "parts")
