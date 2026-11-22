#!/usr/bin/env python3
"""Before / after picture of 14 Nov 2026 (D189): what the bin recordings show, and what the same three builds
should show on a lake.

    python tools/echogram_before_after.py [LOG_FOLDER] [OUT.png]

Top row = measured (three recordings of the compost bin, one colour scale in absolute dB).
Bottom row = simulated lake, 6 m deep, drawn the way the chalet page draws it: dark = that build's own receiver
floor, full colour 40 dB above it. Levels: L = K + TS - 40 log R. K = +10.7 dB for the fixed builds (middle of the
weight's +3.5 and the centred ball's +17.8, D188: +-7 dB), -30.3 dB for the build with the burst fault at its own
settings (gain 2, 32 cycles): +10.7 minus the 48.5 dB measured between 4 Nov and 14 Nov at the same gain and burst
length (D192), plus 7.5 dB for that gain and burst (the 8 Nov ball alone gave -17 to -27, D188). Floors: -90 dB (fault: the chip's resting level), -75 dB (converter running), -86 dB (gated), D179.
Needs numpy + matplotlib.
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "DejaVu Sans"
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "log")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "docs", "echogram_before_after_2026-11-14.png")
BIN = 0.025
# the pages' own palette (bench_web.cpp PALS)
PAL = [(0, 10, 30, 38), (.16, 17, 65, 76), (.32, 27, 119, 120), (.48, 88, 174, 124), (.62, 211, 192, 74), (.76, 238, 139, 47), (.9, 216, 67, 47), (1, 255, 230, 210)]
CMAP = LinearSegmentedColormap.from_list("page", [(p, (r / 255, g / 255, b / 255)) for p, r, g, b in PAL])
INK, MUTED = "#1c2a30", "#55646b"


def recording(name):
    rows = []
    for l in open(os.path.join(LOG, name), errors="ignore"):
        if l.startswith("SONAR"):
            h = l.split()[-1]
            if len(h) >= 976: rows.append(np.frombuffer(bytes.fromhex(h[:976]), dtype=np.uint8).astype(float))
    return np.array(rows) * 95.0 / 255.0 - 100.0          # pings x bins, dB


def lake(K, floor, sigma_m, seed, ring_extra=0.0, depth=6.0, secs=60, rate=4):
    """Simulated echogram: pings x bins in dB. Targets: jig head, a 25 cm fish coming to it, a 15 cm fish passing."""
    rng = np.random.default_rng(seed)
    nb, nt = int(7.0 / BIN), secs * rate
    r = (np.arange(nb) + 0.5) * BIN
    out = np.zeros((nt, nb))
    for t in range(nt):
        s = t / rate
        p = np.full(nb, 10 ** (floor / 10))                                   # receiver floor, power
        ring = np.where(r < 0.12 + ring_extra, -7.0, -7.0 - 110.0 * (r - 0.12 - ring_extra)) + (K - 10.7)
        p += 10 ** (np.minimum(ring, -7.0) / 10)
        tg = [(4.50 + 0.05 * np.sin(2 * np.pi * s / 2.5) * (1.0 if (s % 15) < 9 else 0.15), -51.5, 0.0)]      # jig head
        if 18 <= s < 46:                                                     # 25 cm fish: up from the bottom to the jig, stays, leaves
            d = 5.70 - 1.0 * min(1.0, (s - 18) / 8) if s < 38 else 4.70 + 1.0 * (s - 38) / 8
            tg.append((d + 0.02 * np.sin(s * 1.7), -40.0, 2.0 * np.sin(s * 0.9)))
        if 5 <= s < 15:                                                      # 15 cm fish crossing the beam at 2.5 m
            x = (s - 10) / 5
            tg.append((2.5 + 0.03 * x * x, -46.0, -9.0 * x * x))
        for d, ts, fluct in tg:
            L = K + ts - 40 * np.log10(d) + fluct
            p += 10 ** (L / 10) * np.exp(-0.5 * ((r - d) / sigma_m) ** 2)
        Lb = K - 40.7 + rng.normal(0, 0.8)                                   # bottom: -30 dB at 6 m with K = +10.7 (est., firm bottom)
        p += 10 ** (Lb / 10) * np.where(r >= depth, np.exp(-(r - depth) / 0.12), np.exp(-0.5 * ((r - depth) / sigma_m) ** 2))
        out[t] = 10 * np.log10(p) + rng.normal(0, 1.2, nb)
    return np.clip(out, -100, -6.5)


def panel(ax, data, vmin, vmax, ymax, title, sub, xlabel):
    im = ax.imshow(data.T, aspect="auto", cmap=CMAP, vmin=vmin, vmax=vmax, extent=[0, data.shape[0], data.shape[1] * BIN, 0], interpolation="nearest")
    ax.set_ylim(ymax, 0)
    ax.set_title(title, fontsize=12.5, color=INK, loc="left", fontweight="bold", pad=22)
    ax.text(0, 1.012, sub, transform=ax.transAxes, fontsize=9.6, color=MUTED, va="bottom")
    ax.set_xlabel(xlabel, fontsize=9.5, color=MUTED)
    ax.tick_params(labelsize=9, colors=MUTED)
    for s in ax.spines.values(): s.set_color("#c9d1d4")
    return im


def note(ax, x, y, text):
    ax.annotate(text, xy=(x, y), xytext=(8, 0), textcoords="offset points", fontsize=9.2, color="white", va="center",
                bbox=dict(boxstyle="round,pad=0.25", fc="#0a1e26", ec="none", alpha=0.78))


fig, axs = plt.subplots(2, 3, figsize=(16.5, 11.2), gridspec_kw=dict(hspace=0.40, wspace=0.13, left=0.05, right=0.93, top=0.82, bottom=0.065))
fig.suptitle("Before and after 14 Nov: measured in the bin, expected on a lake", fontsize=17, color=INK, fontweight="bold", x=0.05, ha="left", y=0.978)
fig.text(0.05, 0.94, "Top: your recordings, one colour scale. The background rose because the fix lifted every echo by 30 dB or more, the bin's own wall echoes included.\n"
                      "Bottom: simulation, no walls. Each panel is drawn as the chalet page draws it: dark = that build's receiver floor, full colour 40 dB above it.", fontsize=10.6, color=MUTED, va="top")

BINS = [("sonar_1_2026-11-08-11-58-51.txt", "8 Nov: 20 V, burst fault", "gain 2, 32 cycles. Ball at 0.6 m, bottom 0.8 m"),
        ("sonar_1_2026-11-14-11-35-24.txt", "14 Nov 11:35: fault fixed, converter running", "gain 0, 8 cycles. Ball at 0.5 m, bottom 0.72 m"),
        ("sonar_1_2026-11-14-19-09-05.txt", "14 Nov 19:09: fault fixed, converter gated", "gain 0, 8 cycles. Ball at 0.47 m, bottom 0.72 m")]
for i, (fn, title, sub) in enumerate(BINS):
    d = recording(fn)[:, :int(1.2 / BIN)]
    im1 = panel(axs[0, i], d, -92, -6, 1.2, title, sub, "ping number")
    if i == 0: axs[0, i].set_ylabel("depth in the bin, m", fontsize=10, color=MUTED)
    mid = np.median(d, 0)
    axs[0, i].text(0.985, 0.04, f"water between the echoes: {np.percentile(mid[8:26], 25):.0f} dB", transform=axs[0, i].transAxes, fontsize=9.4, color="white", ha="right",
                   bbox=dict(boxstyle="round,pad=0.25", fc="#0a1e26", ec="none", alpha=0.78))
cb = fig.colorbar(im1, cax=fig.add_axes([0.94, 0.50, 0.009, 0.29])); cb.set_label("echo level, dB (absolute)", fontsize=9.5, color=MUTED); cb.ax.tick_params(labelsize=8.5, colors=MUTED)

LAKES = [("Before today: 20 V, burst fault", -30.3, -90.0, 0.09, 0.10, "32 cycles, gain 2. Floor -90 dB (the chip's resting level)"),
         ("This morning: fault fixed, converter running", 10.7, -75.0, 0.035, 0.0, "30 V, 8 cycles, gain 0. Floor -75 dB"),
         ("Now: fault fixed, converter gated", 10.7, -86.0, 0.035, 0.0, "30 V, 8 cycles, gain 0. Floor -86 dB")]
for i, (title, K, floor, sig, rx, sub) in enumerate(LAKES):
    d = lake(K, floor, sig, 11 + i, rx)
    im2 = panel(axs[1, i], d - floor, 0, 40, 7.0, title, sub, "seconds (4 pings per second)")
    axs[1, i].set_xticks(np.arange(0, 241, 40)); axs[1, i].set_xticklabels([str(v) for v in range(0, 61, 10)])
    if i == 0: axs[1, i].set_ylabel("depth, m", fontsize=10, color=MUTED)
    m = lambda ts, r: K + ts - 40 * np.log10(r) - floor
    fmt = lambda v: (f"+{v:.0f} dB" if v >= 3 else "under the floor")
    note(axs[1, i], 60, 2.5, f"15 cm fish at 2.5 m: {fmt(m(-46, 2.5))}")
    note(axs[1, i], 96, 4.05, f"25 cm fish at 4.7 m: {fmt(m(-40, 4.7))}")
    note(axs[1, i], 4, 4.5, f"jig at 4.5 m: {fmt(m(-51.5, 4.5))}")
    note(axs[1, i], 4, 6.45, f"bottom at 6 m: +{K - 40.7 - floor:.0f} dB")
cb = fig.colorbar(im2, cax=fig.add_axes([0.94, 0.075, 0.009, 0.29])); cb.set_label("dB above that build's own floor", fontsize=9.5, color=MUTED); cb.ax.tick_params(labelsize=8.5, colors=MUTED)
fig.text(0.05, 0.012, "Simulation: echo level = K + target strength - 40 log(range). Calibration K uncertain by about +-7 dB (D188); fish 25 cm = -40 dB, 15 cm = -46 dB, jig head = -51.5 dB (lead sphere). "
                      "Bottom level and the blind zone under the ice are estimates.", fontsize=8.8, color=MUTED)
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
fig.savefig(OUT, dpi=130, facecolor="white")
print("wrote", OUT)


# ---- second picture: the same comparison as traces, like the bench page's "Last ping" (D190) ----
def lake_ping(K, floor, sigma_m, seed, ring_extra=0.0, depth=6.0):
    """One simulated ping out to 12.2 m with the three targets present together."""
    rng = np.random.default_rng(seed)
    r = (np.arange(488) + 0.5) * BIN
    p = np.full(488, 10 ** (floor / 10))
    ring = np.where(r < 0.12 + ring_extra, -7.0, -7.0 - 110.0 * (r - 0.12 - ring_extra)) + (K - 10.7)
    p += 10 ** (np.minimum(ring, -7.0) / 10)
    for d, ts in ((2.5, -46.0), (4.5, -51.5), (4.7, -40.0)):
        p += 10 ** ((K + ts - 40 * np.log10(d)) / 10) * np.exp(-0.5 * ((r - d) / sigma_m) ** 2)
    for dd, Lb in ((depth, K - 40.7), (2 * depth, K - 40.7 - 22.0)):      # bottom, and its second bounce off the ice (est.)
        p += 10 ** (Lb / 10) * np.where(r >= dd, np.exp(-(r - dd) / 0.12), np.exp(-0.5 * ((r - dd) / sigma_m) ** 2))
    return r, np.clip(10 * np.log10(p) + rng.normal(0, 1.2, 488), -100, -6.5)


def trace_axes(ax, title, sub):
    ax.set_xlim(0, 12.2); ax.set_ylim(-100, 0)
    ax.set_title(title, fontsize=12, color=INK, loc="left", fontweight="bold", pad=20)
    ax.text(0, 1.02, sub, transform=ax.transAxes, fontsize=9.4, color=MUTED, va="bottom")
    ax.grid(True, color="#e3e8ea", lw=0.8); ax.set_axisbelow(True)
    ax.tick_params(labelsize=9, colors=MUTED)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    for s in ("left", "bottom"): ax.spines[s].set_color("#c9d1d4")


TRACE, BOTTOM_RED, FLOORC = "#12475a", "#d8432f", "#7a8a90"
fig, axs = plt.subplots(3, 2, figsize=(16.5, 11.6), gridspec_kw=dict(hspace=0.55, wspace=0.10, left=0.055, right=0.985, top=0.855, bottom=0.09))
fig.suptitle("The same comparison as traces, like \"Last ping\" on the bench page", fontsize=17, color=INK, fontweight="bold", x=0.055, ha="left", y=0.978)
fig.text(0.055, 0.94, "Depth left to right, echo strength up, same scale everywhere. Red line = bottom. Dashed = receiver floor of that build.\n"
                      "Left: your bin recordings (median ping). Right: one simulated ping on a 6 m lake with a 15 cm fish, the jig and a 25 cm fish 20 cm under it.", fontsize=10.6, color=MUTED, va="top")
FLOORS = (-90.0, -75.0, -86.0)
for i, (fn, title, sub) in enumerate(BINS):
    ax = axs[i, 0]; d = np.median(recording(fn), 0); r = (np.arange(488) + 0.5) * BIN
    trace_axes(ax, "Bin, " + title[0].lower() + title[1:] if not title[0].isdigit() else "Bin, " + title, sub)
    ax.plot(r, d, color=TRACE, lw=1.6)
    b = int(np.argmax(d[20:44])) + 20
    ax.axvline(r[b], color=BOTTOM_RED, lw=1.6)
    ax.axhline(FLOORS[i], color=FLOORC, lw=1.2, ls=(0, (5, 4)))
    ax.text(12.1, FLOORS[i] - 2.0, f"receiver floor {FLOORS[i]:.0f} dB", fontsize=9.2, color=FLOORC, ha="right", va="top")
    far = float(np.median(d[360:]))
    if far > FLOORS[i] + 6: ax.text(10.4, far + 26, f"bin echoes still {far - FLOORS[i]:.0f} dB\nover the floor at 9-12 m", fontsize=9.2, color=INK, ha="center", va="bottom")
    ax.set_ylabel("echo level, dB", fontsize=9.5, color=MUTED)
axs[2, 0].set_xlabel("depth, m (the bin is 0.7 m deep: everything past the red line is echoes bouncing around)", fontsize=9.5, color=MUTED)
for i, (title, K, floor, sig, rx, sub) in enumerate(LAKES):
    ax = axs[i, 1]; r, d = lake_ping(K, floor, sig, 31 + i, rx)
    trace_axes(ax, "Lake, " + title[0].lower() + title[1:], sub)
    ax.plot(r, d, color=TRACE, lw=1.6)
    ax.axvline(6.0, color=BOTTOM_RED, lw=1.6)
    ax.axhline(floor, color=FLOORC, lw=1.2, ls=(0, (5, 4)))
    ax.text(11.2, floor - 2.0, f"receiver floor {floor:.0f} dB", fontsize=9.2, color=FLOORC, ha="right", va="top")
    lab = lambda ts, dist: K + ts - 40 * np.log10(dist)
    for name, ts, dist, dx, ha in (("15 cm fish", -46.0, 2.5, 0.0, "center"), ("jig", -51.5, 4.5, -0.12, "right"), ("25 cm fish", -40.0, 4.7, 0.12, "left")):
        L = lab(ts, dist); over = L - floor
        txt = f"{name}\n+{over:.0f} dB" if over >= 3 else f"{name}\nnot visible"
        ax.text(dist + dx, max(L, floor) + 6, txt, fontsize=9.2, color=INK, ha=ha, va="bottom", linespacing=1.15)
    ax.text(6.15, -12, "bottom", fontsize=9.2, color=BOTTOM_RED, va="center")
    ax.text(12.0, max(K - 62.7, floor) + 6, "bottom,\nsecond\nbounce", fontsize=8.6, color=MUTED, ha="right", va="bottom", linespacing=1.1)
axs[2, 1].set_xlabel("depth, m", fontsize=9.5, color=MUTED)
fig.text(0.055, 0.006, "Simulation: echo level = K + target strength - 40 log(range); K uncertain by about +-7 dB (D188). Bottom, its second bounce and the blind zone under the ice are estimates.\n"
                       "With the 32-cycle burst of the old build the jig and the fish 20 cm apart merge into one bump.", fontsize=8.8, color=MUTED)
OUT2 = os.path.join(os.path.dirname(os.path.abspath(OUT)), "echogram_traces_2026-11-14.png")
fig.savefig(OUT2, dpi=130, facecolor="white")
print("wrote", OUT2)


# ---- third picture: the burst fix alone, same day, same bin, same settings (D191) ----
AB = ("sonar_1_2026-11-14-09-53-51.txt", "sonar_1_2026-11-14-10-21-13.txt")
if all(os.path.exists(os.path.join(LOG, f)) for f in AB):
    a, b = (np.median(recording(f), 0) for f in AB)
    r = (np.arange(488) + 0.5) * BIN
    CB, CA = "#b5622e", "#12475a"
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(13.5, 9.2), sharex=True, gridspec_kw=dict(height_ratios=[3, 1.5], hspace=0.12, left=0.07, right=0.975, top=0.86, bottom=0.075))
    fig.suptitle("The burst fix alone: same morning, same bin, same settings", fontsize=16.5, color=INK, fontweight="bold", x=0.07, ha="left", y=0.975)
    fig.text(0.07, 0.935, "14 Nov, 09:53 (before the fix) and 10:21 (after). Both at 30 V, gain 3, 32 cycles; median ping. Converter running in both: floor about -75 dB.", fontsize=10.6, color=MUTED, va="top")
    for s in ("top", "right"): ax.spines[s].set_visible(False); ax2.spines[s].set_visible(False)
    for x in (ax, ax2):
        x.grid(True, color="#e3e8ea", lw=0.8); x.set_axisbelow(True); x.tick_params(labelsize=9.5, colors=MUTED)
        for s in ("left", "bottom"): x.spines[s].set_color("#c9d1d4")
    ax.axhspan(-6.9, 0, color="#f3e3df", lw=0); ax.text(7.9, -3.4, "receiver clipped above -7 dB", fontsize=9.2, color="#9a4a3c", ha="right", va="center")
    ax.plot(r, a, color=CB, lw=2, label="before the fix (chip stuck in transmit)")
    ax.plot(r, b, color=CA, lw=2, label="after the fix")
    ax.axhline(-75, color=FLOORC, lw=1.2, ls=(0, (5, 4))); ax.text(7.9, -77, "receiver floor that morning, -75 dB", fontsize=9.2, color=FLOORC, ha="right", va="top")
    ax.set_xlim(0, 8); ax.set_ylim(-100, 0); ax.set_ylabel("echo level, dB", fontsize=10, color=MUTED)
    ax.legend(loc="upper right", bbox_to_anchor=(1.0, 0.92), frameon=False, fontsize=10.5)
    ax.text(0.08, -14, "transmit:\nclipped\nin both", fontsize=9.2, color=INK, va="top")
    ax.annotate("bottom: -38 dB before,\nclipped after", xy=(0.80, -37.8), xytext=(1.25, -34), fontsize=9.2, color=INK, va="center", arrowprops=dict(arrowstyle="-", color=MUTED, lw=1))
    ax.text(3.2, -30, "bin echoes, now visible", fontsize=9.2, color=CA)
    ax.text(3.0, -84, "before the fix they were at or under the floor", fontsize=9.2, color=CB)
    d = b - a
    lower = (np.median(recording(AB[1]), 0) >= -7.0) | (a < -70)      # after clipped, or before sitting on the floor: the true difference is larger
    both_clip = (a > -7.0) & (np.median(recording(AB[1]), 0) > -7.0)
    ax2.plot(r, np.where(lower | both_clip, np.nan, d), color=INK, lw=2.2)
    ax2.plot(r, np.where(lower & ~both_clip, d, np.nan), color=INK, lw=1.4, ls=(0, (2, 2)))
    ax2.set_ylim(0, 70); ax2.set_ylabel("after minus before, dB", fontsize=10, color=MUTED); ax2.set_xlabel("depth, m", fontsize=10, color=MUTED)
    ax2.text(7.9, 62, "solid = both readable.  dotted = a lower bound (after clipped, or before lost in the floor)", fontsize=9.2, color=MUTED, ha="right", va="center")
    ax2.text(0.42, 55, "45-51 dB", fontsize=10, color=INK, fontweight="bold")
    ax2.text(1.75, 55, "39-61 dB, lower bounds", fontsize=10, color=INK, fontweight="bold")
    OUT3 = os.path.join(os.path.dirname(os.path.abspath(OUT)), "burst_fix_same_day_2026-11-14.png")
    fig.savefig(OUT3, dpi=130, facecolor="white")
    print("wrote", OUT3)


# ---- fourth picture: first bench logs (4 Nov) against today, same gain, same burst length (D192) ----
PAIRS = (("sonar_1_2026-11-04-16-02-33.txt", None, "sonar_1_2026-11-14-17-54-15.txt", "gain 2 (15 V/V)", "8 cycles, gain 2"),
         ("sonar_1_2026-11-04-15-52-34.txt", "gain 0 (10 V/V)", "sonar_1_2026-11-14-17-54-15.txt", "cycles 32", "32 cycles, gain 0"))


def sweep_step(fn, step):
    out, cur = {}, None
    for l in open(os.path.join(LOG, fn), errors="ignore"):
        if l.startswith("NOTE") and "SWEEP step" in l: cur = l.split(": ", 1)[1].split(" (0x11")[0].strip()
        elif l.startswith("NOTE") and "SWEEP done" in l: cur = "(after)"
        elif l.startswith("SONAR"):
            h = l.split()[-1]
            if len(h) >= 976: out.setdefault(cur, []).append(np.frombuffer(bytes.fromhex(h[:976]), dtype=np.uint8).astype(float))
    v = out[step]
    return np.median(np.array(v[1:] if len(v) > 2 else v), 0)


if all(os.path.exists(os.path.join(LOG, p[0])) and os.path.exists(os.path.join(LOG, p[2])) for p in PAIRS):
    fig, axs = plt.subplots(2, 2, figsize=(16, 9.6), gridspec_kw=dict(height_ratios=[3, 1.4], hspace=0.14, wspace=0.12, left=0.06, right=0.985, top=0.83, bottom=0.075))
    fig.suptitle("First bench logs (4 Nov) against today (14 Nov): same gain, same burst length", fontsize=16.5, color=INK, fontweight="bold", x=0.06, ha="left", y=0.975)
    fig.text(0.06, 0.935, "4 Nov: 20 V (MT3608), burst fault. 14 Nov 17:54 sweep: 30 V (XL6009, gated), fault fixed. Bottom at 0.78-0.80 m in both. Median of each step.\n"
                          "Still different: drive voltage (+3.5 dB to 14 Nov), averaging and frequency mode (level unchanged), and the bin's water and walls after ten days.", fontsize=10.6, color=MUTED, va="top")
    CB, CA = "#b5622e", "#12475a"
    for c, (fo, so, fn, sn, lab) in enumerate(PAIRS):
        a = sweep_step(fo, so) if so else np.median((recording(fo)[1:] + 100.0) * 255.0 / 95.0, 0)   # raw codes
        b = sweep_step(fn, sn)
        da, dbb = a * 95.0 / 255.0 - 100.0, b * 95.0 / 255.0 - 100.0
        r = (np.arange(488) + 0.5) * BIN
        ax, ax2 = axs[0, c], axs[1, c]
        for x in (ax, ax2):
            x.grid(True, color="#e3e8ea", lw=0.8); x.set_axisbelow(True); x.tick_params(labelsize=9.5, colors=MUTED)
            for sp in ("top", "right"): x.spines[sp].set_visible(False)
            for sp in ("left", "bottom"): x.spines[sp].set_color("#c9d1d4")
        ax.axhspan(-6.9, 0, color="#f3e3df", lw=0)
        ax.plot(r, da, color=CB, lw=2, label="4 Nov, burst fault")
        ax.plot(r, dbb, color=CA, lw=2, label="14 Nov, fixed")
        ax.set_xlim(0, 3); ax.set_ylim(-100, 0)
        ax.set_title(lab, fontsize=12.5, color=INK, loc="left", fontweight="bold")
        if c == 0: ax.set_ylabel("echo level, dB", fontsize=10, color=MUTED); ax2.set_ylabel("14 Nov minus 4 Nov, dB", fontsize=10, color=MUTED)
        ax.legend(loc="lower left", frameon=False, fontsize=10.5)
        pa = 14 + int(np.argmax(a[14:44])); pb = 14 + int(np.argmax(b[14:44]))
        ax.annotate(f"bottom {da[pa]:.0f} dB", xy=(r[pa], da[pa]), xytext=(r[pa] + 0.25, da[pa] - 8), fontsize=9.6, color=CB, arrowprops=dict(arrowstyle="-", color=CB, lw=1))
        ax.annotate("bottom clipped", xy=(r[pb], dbb[pb]), xytext=(r[pb] + 0.25, -14), fontsize=9.6, color=CA, arrowprops=dict(arrowstyle="-", color=CA, lw=1))
        ax.text(2.95, -3.4, "receiver clipped above -7 dB", fontsize=9, color="#9a4a3c", ha="right", va="center")
        d = dbb - da
        lower = (b >= 249)
        ax2.plot(r, np.where(lower, np.nan, d), color=INK, lw=2.2)
        ax2.plot(r, np.where(lower, d, np.nan), color=INK, lw=1.4, ls=(0, (2, 2)))
        ax2.set_ylim(0, 100); ax2.set_xlim(0, 3); ax2.set_xlabel("depth, m", fontsize=10, color=MUTED)
        w = slice(12, 28)
        ax2.text(0.05, 88, f"water 0.3-0.7 m: +{np.median(d[w]):.0f} dB", fontsize=10, color=INK, fontweight="bold")
        ax2.text(1.25, 88, f"bottom: at least +{dbb[pb] - da[pa]:.0f} dB", fontsize=10, color=INK, fontweight="bold")
        if c == 1: ax2.text(2.95, 6, "solid = both readable.  dotted = 14 Nov clipped: a lower bound", fontsize=9.2, color=MUTED, ha="right")
    fig.text(0.06, 0.012, "Burst fix alone = these differences minus 3.5 dB for 30 V against 20 V. The water figures also carry the bin's reverberation and the freed transducer's ringing.", fontsize=9, color=MUTED)
    OUT4 = os.path.join(os.path.dirname(os.path.abspath(OUT)), "first_logs_vs_today_2026-11-14.png")
    fig.savefig(OUT4, dpi=130, facecolor="white")
    print("wrote", OUT4)
