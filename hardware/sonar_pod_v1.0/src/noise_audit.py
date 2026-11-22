"""Noise / signal-integrity audit of the routed board (v0.3 onward) (pcbnew + shapely).
Coordinates reported in mm from the board's TOP-LEFT corner (as seen in KiCad, Y down)."""
import sys, math, collections, json
import pcbnew
from shapely.geometry import LineString, Point, Polygon, MultiPolygon, box
from shapely.ops import unary_union

PCB = sys.argv[1]; OUTPNG = sys.argv[2]
b = pcbnew.LoadBoard(PCB)
bb = b.GetBoardEdgesBoundingBox(); X0, Y0 = bb.GetLeft() / 1e6, bb.GetTop() / 1e6
W, H = bb.GetWidth() / 1e6, bb.GetHeight() / 1e6
mm = lambda v: v / 1e6
def P(v): return (mm(v.x) - X0, mm(v.y) - Y0)
F, B = pcbnew.F_Cu, pcbnew.B_Cu

AGG = {  # net: (description, severity weight)
    "SW_B": ("MT3608 switch node, 0-18 V square wave 1.2 MHz", 3),
    "DBX": ("doubler pump node, 17-35 V square wave 1.2 MHz", 3), "VDBL": ("34.5 V doubler output (ripple)", 1),
    "XS": ("transformer secondary, ~200 V burst 200 kHz", 3), "VDX": ("switched 34.5 V to VDRV", 1),
    "TL1": ("TPS63020 switch node L1, 2.4 MHz", 3), "TL2": ("TPS63020 switch node L2, 2.4 MHz", 3),
    "XA": ("transducer drive A, ~20 V burst 200 kHz", 3), "XB": ("transducer drive B, ~20 V burst 200 kHz", 3),
    "VBOOST": ("17.4 V boost output (ripple)", 1), "VDRV": ("TUSS VDRV supply (burst current)", 1),
    "SCLK": ("SPI clock (TUSS + LoRa)", 2), "SDI": ("SPI MOSI", 1), "SDO": ("SPI MISO", 1),
    "TXD0": ("UART TX", 1), "RXD0": ("UART RX", 1), "USB_DP": ("USB D+", 1), "USB_DN": ("USB D-", 1),
    "LORA_ANT": ("LoRa RF 915 MHz, 22 dBm", 2),
    "OUTA": ("TUSS OUTA (drive)", 3), "OUTB": ("TUSS OUTB (drive)", 3), "DA": ("FET drain A (transformer)", 3),
    "DB": ("FET drain B (transformer)", 3), "GA": ("gate A", 2), "GB": ("gate B", 2), "VTX": ("center tap", 1),
    "LORA_SCK": ("LoRa SPI clock", 2), "LORA_MOSI": ("LoRa MOSI", 1), "LORA_MISO": ("LoRa MISO", 1),
}
VIC = {
    "INP": "TUSS receive input (uV echoes)", "INN": "TUSS receive ref", "RINP": "receive path R->C", "FLT": "TUSS filter cap",
    "VOUT_I": "TUSS analog envelope out (to ADC)", "VOUT": "envelope after RC, ESP32-S3 ADC IO6",
    "VBAT_ADC": "battery ADC", "VPWR_ADC": "boost ADC", "NTC_ADC": "water temp ADC", "HALL": "hall sensor (digital)",
    "FB_B": "MT3608 feedback (high impedance)", "FB_T": "TPS63020 feedback (high impedance)", "VINA": "TPS63020 analog supply",
    "SGND": "sonar analog ground island", "RIN1": "receive path (after R8B)", "AINP": "ADS7042 input",
}

# ---------- geometry
tracks = collections.defaultdict(list)   # net -> [(LineString, width, layer)]
vias = collections.defaultdict(list)
for t in b.GetTracks():
    n = t.GetNetname()
    if t.GetClass() == "PCB_VIA":
        vias[n].append(Point(P(t.GetPosition())))
    elif t.GetClass() == "PCB_TRACK":
        tracks[n].append((LineString([P(t.GetStart()), P(t.GetEnd())]), mm(t.GetWidth()), t.GetLayer()))
    elif t.GetClass() == "PCB_ARC":
        tracks[n].append((LineString([P(t.GetStart()), P(t.GetMid()), P(t.GetEnd())]), mm(t.GetWidth()), t.GetLayer()))

def polyset(ps):
    out = []
    for i in range(ps.OutlineCount()):
        o = ps.Outline(i); pts = [P(o.CPoint(k)) for k in range(o.PointCount())]
        holes = []
        for h in range(ps.HoleCount(i)):
            hh = ps.Hole(i, h); holes.append([P(hh.CPoint(k)) for k in range(hh.PointCount())])
        if len(pts) >= 3: out.append(Polygon(pts, holes).buffer(0))
    return out
fill = {(F, "GND"): [], (B, "GND"): [], (F, "SGND"): [], (B, "SGND"): []}
for z in b.Zones():
    for l in z.GetLayerSet().Seq():
        fill.setdefault((l, z.GetNetname()), []).extend(polyset(z.GetFilledPolysList(l)))
NCU = b.GetCopperLayerCount()
if NCU >= 4:   # 4-layer: F.Cu references In1, B.Cu references In2 (the planes right underneath)
    In1, In2 = pcbnew.In1_Cu, pcbnew.In2_Cu
    for l in (In1, In2):
        fill.setdefault((l, "GND"), [])
    REF = {F: unary_union(fill[(In1, "GND")]), B: unary_union(fill[(In2, "GND")])}
    REF_OTHER = {F: B, B: F}
else:
    REF = {B: unary_union(fill[(F, "GND")] + fill[(F, "SGND")]), F: unary_union(fill[(B, "GND")] + fill[(B, "SGND")])}

pads = collections.defaultdict(list); fpbox = {}
for f in b.GetFootprints():
    r = f.GetReference()
    cy = f.GetCourtyard(pcbnew.F_CrtYd)
    if cy.OutlineCount(): fpbox[r] = unary_union(polyset(cy))
    for pd in f.Pads():
        pads[pd.GetNetname()].append((r, pd.GetNumber(), Point(P(pd.GetPosition())), pd.HasHole(), pd))

def net_geom(n, layer=None):
    return unary_union([ls.buffer(w / 2) for ls, w, l in tracks[n] if layer is None or l == layer] or [Polygon()])

R = {"summary": {}, "victims": [], "pairs": [], "inductor": [], "loops": [], "gnd": {}}
# ---------- victims: reference plane + coupling
for v, vdesc in VIC.items():
    segs = tracks.get(v, [])
    tot = sum(ls.length for ls, _, _ in segs)
    if tot == 0: continue
    noref = 0.0
    for ls, w, l in segs:
        n = max(2, int(ls.length / 0.25))
        miss = sum(1 for k in range(n + 1) if not REF[l].contains(ls.interpolate(k / n, normalized=True)))
        noref += ls.length * miss / (n + 1)
    lay = collections.Counter(("F" if l == F else "B") for _, _, l in segs for _ in [0])
    R["victims"].append(dict(net=v, desc=vdesc, length=round(tot, 1), vias=len(vias[v]),
                             layers="/".join(sorted(lay)), no_ref_mm=round(noref, 1), no_ref_pct=round(100 * noref / tot)))
    for a, (adesc, sev) in AGG.items():
        if a not in tracks: continue
        for lay_ in (F, B):
            ag = net_geom(a, lay_)
            if ag.is_empty: continue
            for gap in (0.5, 1.0):
                par = sum(ls.intersection(ag.buffer(gap + w / 2)).length for ls, w, l in segs if l == lay_)
                if par > 0.3:
                    mind = min(ls.distance(ag) - w / 2 for ls, w, l in segs if l == lay_)
                    R["pairs"].append(dict(victim=v, aggressor=a, layer="F" if lay_ == F else "B", gap=gap,
                                           parallel_mm=round(par, 1), min_gap_mm=round(max(mind, 0), 2), sev=sev))
        # broadside crossings (opposite layer overlap)
        cross = 0.0
        for ls, w, l in segs:
            ag = net_geom(a, B if l == F else F)
            if not ag.is_empty: cross += ls.intersection(ag).length
        if cross > 2.0:
            R["pairs"].append(dict(victim=v, aggressor=a, layer="cross", gap=0, parallel_mm=round(cross, 1), min_gap_mm=1.5, sev=sev))
    for ind in ("L1", "L2", "U3", "U2"):
        if ind in fpbox:
            ln = sum(ls.intersection(fpbox[ind].buffer(0.5)).length for ls, w, l in segs)
            if ln > 0.3: R["inductor"].append(dict(victim=v, part=ind, length=round(ln, 1)))
# keep only the closest gap row per (victim, aggressor, layer)
best = {}
for p in R["pairs"]:
    k = (p["victim"], p["aggressor"], p["layer"])
    if k not in best or (p["gap"] > best[k]["gap"]): best[k] = p   # 1.0 mm window includes 0.5
R["pairs"] = sorted(best.values(), key=lambda p: -p["parallel_mm"] * p["sev"])

# ---------- switching loops
def padpt(ref, num):
    f = b.FindFootprintByReference(ref)
    for pd in f.Pads():
        if pd.GetNumber() == str(num): return Point(P(pd.GetPosition()))
def dist(a, c): return round(a.distance(c), 1)
sw = padpt("U3", 1); gnd3 = padpt("U3", 2); dA = padpt("D1", 2); dK = padpt("D1", 1)
couts = [("C8", padpt("C8", 1), padpt("C8", 2)), ("C9", padpt("C9", 1), padpt("C9", 2))]
cn = min(couts, key=lambda c: c[1].distance(dK))
loop = Polygon([sw.coords[0], dA.coords[0], dK.coords[0], cn[1].coords[0], cn[2].coords[0], gnd3.coords[0]]).buffer(0)
R["loops"].append(dict(name="MT3608 hot loop SW->D1->%s->GND->U3" % cn[0], area_mm2=round(loop.area, 1),
                       segs="U3.SW-D1 %.1f, D1-%s %.1f, %s.GND-U3.GND %.1f mm" % (sw.distance(dA), cn[0], dK.distance(cn[1]), cn[0], cn[2].distance(gnd3)),
                       sw_node_copper_mm=round(sum(ls.length for ls, _, _ in tracks["SW_B"]), 1)))
# TPS63020 caps: VIN (VB pins 10-12) and VOUT (3V3 pins 4-5)
for label, net, pins in (("TPS63020 input", "VB", (10, 11, 12)), ("TPS63020 output", "3V3", (4, 5))):
    ic = [padpt("U2", p) for p in pins]
    caps = [(r, pt) for (r, num, pt, hole, pd) in pads[net] if r.startswith("C")]
    near = sorted(((min(pt.distance(i) for i in ic), r) for r, pt in caps))[:3]
    R["loops"].append(dict(name=label + " caps", nearest=", ".join("%s %.1f mm" % (r, d) for d, r in near)))
R["loops"].append(dict(name="TPS63020 switch nodes", tl1_mm=round(sum(ls.length for ls, _, _ in tracks["TL1"]), 1),
                       tl2_mm=round(sum(ls.length for ls, _, _ in tracks["TL2"]), 1)))
# TUSS VPWR / VDRV decoupling distance
for net, pin in (("VPWR", 20), ("VDRV", 17), ("3V3", 10)):
    ic = padpt("U1", pin)
    caps = sorted((pt.distance(ic), r) for (r, num, pt, hole, pd) in pads[net] if r.startswith("C"))[:2]
    R["loops"].append(dict(name="TUSS4470 %s decoupling" % net, nearest=", ".join("%s %.1f mm" % (r, d) for d, r in caps)))

# ---------- ground
stitch = vias["GND"] + [pt for (r, num, pt, hole, pd) in pads["GND"] if hole]
R["gnd"]["gnd_vias"] = len(vias["GND"]); R["gnd"]["gnd_tht_pads"] = len(stitch) - len(vias["GND"])
R["gnd"]["F_fill_pct"] = round(100 * unary_union(fill[(F, "GND")]).area / (W * H))
R["gnd"]["B_fill_pct"] = round(100 * unary_union(fill[(B, "GND")]).area / (W * H))
# both-layer GND overlap that is NOT stitched within 5 mm
both = unary_union(fill[(F, "GND")]).intersection(unary_union(fill[(B, "GND")]))
far = []
for xi in range(0, int(W * 2)):
    for yi in range(0, int(H * 2)):
        q = Point(xi / 2, yi / 2)
        if both.contains(q):
            d = min((q.distance(s) for s in stitch), default=99)
            if d > 8: far.append((q, d))
R["gnd"]["overlap_pts_far_from_stitch_gt8mm_pct"] = round(100 * len(far) / max(1, sum(1 for xi in range(int(W*2)) for yi in range(int(H*2)) if both.contains(Point(xi/2, yi/2)))))
# floating / weakly-connected F.Cu islands
isl = []
for l in (F, B):
    for poly in fill[(l, "GND")]:
        parts = [poly] if poly.geom_type == "Polygon" else list(poly.geoms)
        for pg in parts:
            npad = sum(1 for (r, num, pt, hole, pd) in pads["GND"] if pg.buffer(0.05).contains(pt) and (hole or l == F))
            nvia = sum(1 for s in vias["GND"] if pg.buffer(0.05).contains(s))
            if pg.area > 1 and npad + nvia <= 1:
                c = pg.centroid; isl.append(dict(layer="F" if l == F else "B", area=round(pg.area, 1), at="(%.1f, %.1f)" % (c.x, c.y), connections=npad + nvia))
R["gnd"]["weak_islands"] = sorted(isl, key=lambda d: -d["area"])
# LoRa RF trace
if "LORA_ANT" in tracks:
    R["summary"]["lora_ant"] = dict(length=round(sum(ls.length for ls, _, _ in tracks["LORA_ANT"]), 1),
                                    widths=sorted(set(round(w, 2) for _, w, _ in tracks["LORA_ANT"])),
                                    layers=sorted(set("F" if l == F else "B" for _, _, l in tracks["LORA_ANT"])))
if "USB_DP" in tracks:
    R["summary"]["usb"] = dict(dp=round(sum(ls.length for ls, _, _ in tracks["USB_DP"]), 1), dn=round(sum(ls.length for ls, _, _ in tracks["USB_DN"]), 1))
json.dump(R, open(OUTPNG.replace(".png", ".json"), "w"), indent=1)

# ---------- picture
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, axs = plt.subplots(1, 2, figsize=(16, 6.6))
for ax, l, nm in ((axs[0], F, "Top (F.Cu)"), (axs[1], B, "Bottom (B.Cu)")):
    for poly in fill[(l, "GND")]:
        for pg in ([poly] if poly.geom_type == "Polygon" else poly.geoms):
            ax.fill(*pg.exterior.xy, color="#d9d9d9", lw=0)
            for h in pg.interiors: ax.fill(*h.xy, color="white", lw=0)
    for poly in fill[(l, "SGND")]:
        for pg in ([poly] if poly.geom_type == "Polygon" else poly.geoms): ax.fill(*pg.exterior.xy, color="#c6e5c6", lw=0)
    for n, segs in tracks.items():
        col, lw, z = "#9a9a9a", 0.6, 1
        if n in AGG: col, lw, z = "#d62728", 1.6, 3
        if n in VIC: col, lw, z = "#1f5fbf", 1.8, 4
        for ls, w, ll in segs:
            if ll == l: ax.plot(*ls.xy, color=col, lw=lw, solid_capstyle="round", zorder=z)
    for r in ("U1", "U2", "U3", "L1", "L2", "U4", "U5", "J2", "D1", "C8", "C9", "FB1"):
        if r in fpbox:
            for pg in ([fpbox[r]] if fpbox[r].geom_type == "Polygon" else fpbox[r].geoms):
                ax.plot(*pg.exterior.xy, color="k", lw=0.5, zorder=5)
            c = fpbox[r].centroid; ax.text(c.x, c.y, r, fontsize=7, ha="center", va="center", zorder=6, weight="bold")
    for s in vias["GND"]: ax.plot(s.x, s.y, "o", ms=3, color="k", zorder=6)
    ax.plot(*loop.exterior.xy, color="orange", lw=1.2, ls="--", zorder=6)
    ax.set_xlim(-1, W + 1); ax.set_ylim(H + 1, -1); ax.set_aspect("equal"); ax.set_title(nm)
    ax.plot([0, W, W, 0, 0], [0, 0, H, H, 0], color="k", lw=1)
fig.suptitle(b.GetFileName().split("/")[-1].replace(".kicad_pcb", "") + " noise audit — red: noisy nets, blue: sensitive nets, grey fill: GND pour, green: SGND, black dots: GND vias, orange dashed: MT3608 hot loop")
plt.tight_layout(); plt.savefig(OUTPNG, dpi=130)
print(json.dumps(R, indent=1))
