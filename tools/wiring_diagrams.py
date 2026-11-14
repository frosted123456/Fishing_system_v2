#!/usr/bin/env python3
"""Draws the wiring pictures of the README (D181): the 30 V drive build with the converter enable wire.

    python tools/wiring_diagrams.py            -> images/wiring_heltec_v3_shield002.png
                                                  images/wiring_wroom_shield002.png
                                                  images/wiring_high_drive.png
    python tools/wiring_diagrams.py OUT_FOLDER -> the same three files in OUT_FOLDER

Needs matplotlib. Coordinates are pixels of a 100 dpi canvas, y down. Change a pin here, run, commit the PNGs.
"""
import os, sys
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "DejaVu Sans"   # ships with matplotlib: the same picture on every PC
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Rectangle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RED, ORANGE, PURPLE, BLUE, GREEN, BLACK, GREY, PINK = "#c62828", "#e65100", "#6a1b9a", "#1565c0", "#2e7d32", "#222222", "#555555", "#ad1457"


class Sheet:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.fig = plt.figure(figsize=(w / 100.0, h / 100.0), dpi=100)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, w); self.ax.set_ylim(h, 0); self.ax.axis("off")

    def text(self, x, y, s, color=BLACK, size=13, ha="left", va="center", weight="normal", z=6):
        self.ax.text(x, y, s, color=color, fontsize=size, ha=ha, va=va, fontweight=weight, zorder=z)

    def box(self, x, y, w, h, color, fill, title=None, sub=None, tsize=20, lw=3, dashed=False):
        self.ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=12", ec=color, fc=fill, lw=lw,
                                         ls=(0, (6, 4)) if dashed else "-", zorder=2))
        if title: self.text(x + w / 2, y + 25, title, color, tsize, "center", weight="bold")
        if sub: self.text(x + w / 2, y + 54, sub, "#333333", 13.5, "center")

    def wire(self, pts, color, lw=3.5, z=3):
        self.ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, lw=lw, solid_capstyle="round", solid_joinstyle="round", zorder=z)

    def dot(self, x, y, color, r=5.5):
        self.ax.plot([x], [y], "o", color=color, ms=r * 1.45, zorder=5)

    def pin(self, x, y, label, color, side, size=13.5):
        """A pin on a box edge: side 'l' = dot on the left edge, label inside to the right of it; 'r' = the mirror."""
        self.dot(x, y, color)
        self.text(x + (14 if side == "l" else -14), y, label, color, size, "left" if side == "l" else "right")

    def chip(self, x, y, w, h, label, color, sub=None):
        """A small part drawn in a wire (label inside, optional note on the right)."""
        self.ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0,rounding_size=6", ec=color, fc="white", lw=2.4, zorder=4))
        self.text(x, y, label, color, 11.5, "center", weight="bold")
        if sub: self.text(x + w / 2 + 8, y, sub, GREY, 10.5)

    def save(self, path):
        self.fig.savefig(path, dpi=140, facecolor="white")
        plt.close(self.fig)
        print("wrote", path)


BOARDS = {
    "heltec": dict(
        file="wiring_heltec_v3_shield002.png",
        title="Heltec V3 hub / LoRa tip-up with sonar: wiring",
        sub="Heltec WiFi LoRa 32 V3 + TUSS4470 shield 002, 30 V drive build with the converter enable wire (README 4.4.6)",
        mcu=("Heltec WiFi LoRa 32 V3", "ESP32-S3, LoRa SX1262, OLED, 36 pins"),
        left=[("3V3 pin (fed)", ORANGE), ("GND", BLACK), ("GPIO4 hole", GREEN), ("GPIO2 VBAT", RED), ("GPIO5 button", GREEN)],
        notes=["GPIO0 PRG (screen)", "JST: nothing (no LiPo)", "LED 35, buzzer 7", "LoRa 8-14, OLED 17/18/21", "USB-C: flash / charge"],
        en="GPIO40 conv. enable",
        right=["GPIO47 SCLK", "GPIO33 MISO", "GPIO48 MOSI", "GPIO34 NCS", "GPIO38 IO1", "GPIO39 IO2 burst", "GPIO6 echo"],
        hall=("Own hole sensor", "reed or Hall latch (REED_PIN 4)", "VDD ← VBAT (Hall only; 3.5 V min)", "OUT → GPIO4",
              "reed: GPIO4 to GND. US1881: 1 VDD, 2 GND, 3 OUT"),
        sense=("two 100 k, tap on GPIO2", "GPIO2"),
        button=("GPIO5", "short = arm / disarm, press while FISH ON = clear, hold 3 s = report"),
        foot=[(RED, "Never 5 V on the shield's \"5V\" pin (TUSS4470 logic supply): feed it from the 3.3 V rail."),
              (RED, "Pack switch OFF while USB is plugged: the Heltec's own regulator drives the same 3V3 rail. Nothing on the JST (no LiPo, no charger in use)."),
              (GREY, "A0 / VOUT must stay below ~3.1 V at GPIO6. Twist the A0 wire with its own GND wire. Battery read = the 2:1 divider on GPIO2, not the onboard VBAT_Read."),
              (GREY, "XL6009: set 31.5 V with a meter before wiring anything. Wire A into the J4 middle pin only. Wire B never without the DB107 + 1N4148 (VDRV must stay below VPWR)."),
              (GREY, "Enable wire: the module ties chip pin 2 (EN) to VIN. Cut that pin free of its pad, lift it, solder a thin wire (30 AWG) from the pin to GPIO40. No resistor."),
              (GREY, "Flash: python tools/flash.py hub --id N --sonar --gate (or sensor_lora). Check: web page, Settings, Advanced, Wiring check: \"Converter enable wire OK\"."),
              (GREY, "20 V build instead: MT3608 at 21.5 V into the J4 middle pin, no wire B, no enable wire, flash without --gate. Same pins otherwise.")]),
    "wroom": dict(
        file="wiring_wroom_shield002.png",
        title="WROOM sonar tip-up: wiring",
        sub="ESP32 DevKit V1 (30 pin) + TUSS4470 shield 002, 30 V drive build with the converter enable wire (README 4.4.6)",
        mcu=("ESP32 DevKit V1", "ESP32-WROOM-32, 30 pins"),
        left=[("3V3 pin (input)", ORANGE), ("GND", BLACK), ("GPIO27 Hall", GREEN), ("GPIO34 VBAT sense", RED), ("GPIO33 arming button", GREEN)],
        notes=["GPIO2 LED (on board)", "USB: flashing only"],
        en="D13 conv. enable",
        right=["D18 SCLK", "D19 MISO", "D23 MOSI", "D5 NCS", "D26 IO1", "D25 IO2 burst", "VP = GPIO36 echo"],
        hall=("Hall latch US1881", "spool shaft, 4 alternating magnets", "VDD ← VBAT (3.5-24 V spec)", "OUT → GPIO27",
              "US1881: 1 VDD (pack), 2 GND, 3 OUT open drain"),
        sense=("two 100 k, max 2.7 V at the pin", "GPIO34"),
        button=("GPIO33", "short = arm / disarm, press while tripped = clear, hold 3 s = report"),
        foot=[(RED, "Never 5 V on the shield's \"5V\" pin: it is the TUSS4470 logic supply; 5 V there puts more than 3.6 V on ESP32 pins."),
              (GREY, "A0 / VOUT must stay below ~3.1 V at GPIO36. Twist the A0 wire with its own GND wire."),
              (GREY, "ESP32 fed on its 3V3 pin. Pack switch OFF while USB is plugged: the devkit's own regulator drives the same 3V3 rail."),
              (GREY, "XL6009: set 31.5 V with a meter before wiring anything. Wire A into the J4 middle pin only. Wire B never without the DB107 + 1N4148 (VDRV must stay below VPWR)."),
              (GREY, "Enable wire: the module ties chip pin 2 (EN) to VIN. Cut that pin free of its pad, lift it, solder a thin wire (30 AWG) from the pin to D13 (GPIO13). No resistor."),
              (GREY, "Flash: python tools/flash.py tipup_wroom --id N --sonar --gate. The 30 V build and the enable wire are proven on the Heltec hub; not yet tried on a WROOM."),
              (GREY, "20 V build instead: MT3608 at 21.5 V into the J4 middle pin, no wire B, no enable wire, flash without --gate. Same pins otherwise.")]),
}


def board_sheet(cfg, out_dir):
    s = Sheet(1600, 2180)
    s.text(800, 34, cfg["title"], "#111111", 25, "center", weight="bold")
    s.text(800, 70, cfg["sub"], GREY, 12.5, "center")
    s.text(800, 93, "Red = pack 3.6-5.4 V    orange = 3.3 V    purple = 31.5 V drive    pink = converter enable    black = GND", GREY, 12.5, "center")

    # ---- supply ----
    s.box(60, 125, 300, 140, RED, "#faf0ee", "3 x L91 lithium AA", "holder switch in series", 20)
    s.pin(360, 200, "+", RED, "r"); s.pin(360, 240, "−", BLACK, "r")
    s.box(560, 125, 320, 140, ORANGE, "#fff3ee", "TPS63020 module", "buck-boost → 3.3 V", 20)
    s.pin(560, 200, "VIN", RED, "l"); s.pin(560, 240, "GND", BLACK, "l"); s.pin(880, 200, "VOUT", ORANGE, "r"); s.pin(880, 240, "GND", BLACK, "r")
    s.box(560, 305, 320, 185, PURPLE, "#f4eef7", "XL6009 module", "boost, trimpot set to 31.5 V", 20)
    s.pin(560, 395, "VIN", RED, "l"); s.pin(560, 435, "GND", BLACK, "l"); s.pin(880, 395, "VOUT", PURPLE, "r"); s.pin(880, 435, "GND", BLACK, "r")
    s.dot(720, 490, PINK); s.text(720, 468, "EN = chip pin 2, cut free", PINK, 12.5, "center")

    # ---- boards ----
    MY = 560                                    # top of the MCU and shield boxes
    row = lambda i: MY + 80 + 40 * i
    s.box(260, MY, 400, 600, BLUE, "#eef3fb", cfg["mcu"][0], cfg["mcu"][1], 20)
    for i, (lab, col) in enumerate(cfg["left"]): s.pin(260, row(i), lab, col, "l")
    for i, lab in enumerate(cfg["notes"]): s.pin(260, row(len(cfg["left"]) + i), lab, GREY, "l")
    s.pin(660, row(0), cfg["en"], PINK, "r")
    for i, lab in enumerate(cfg["right"]): s.pin(660, row(i + 1), lab, GREEN if "echo" in lab else BLUE, "r")
    s.box(820, MY, 320, 680, GREEN, "#f0f5ef", "Shield 002", "JP3 + JP4 = 200 kHz", 20)
    for i, lab in enumerate(["D13 SCLK", "D12 SDO", "D11 SDI", "D10 NCS", "D8 IO1", "D9 IO2", "A0 VOUT"]):
        col = GREEN if lab.startswith("A0") else BLUE
        s.pin(820, row(i + 1), lab, col, "l"); s.wire([(660, row(i + 1)), (820, row(i + 1))], col)
    Y5, YG, YJ, YC = row(9), row(10), row(11), row(12)
    s.pin(1140, Y5, "\"5V\" pin ← 3.3 V", ORANGE, "r"); s.pin(1140, YG, "GND", BLACK, "r")
    s.pin(1140, YJ, "J4 middle pin ← wire A", PURPLE, "r"); s.pin(1140, YC, "C6 pad nearer the chip ← wire B", PURPLE, "r", 12.5)
    s.text(832, YC + 38, "no jumper on J4", GREY, 11.5)
    s.text(832, YC + 64, "bleed 47-100 kΩ across C6", PURPLE, 11.5)
    s.text(832, YC + 90, "D2 D3 VIN XT30: unused", GREY, 11.5)
    SB = MY + 680                                # shield bottom
    s.dot(980, SB, GREEN); s.text(994, SB + 24, "XDCR header", GREEN, 12)
    s.box(820, SB + 60, 320, 110, GREEN, "#f0f5ef", "Transducer", "200 kHz", 20)
    s.wire([(980, SB), (980, SB + 60)], GREEN)
    s.text(822, SB + 192, "cable shield → TOP pin of XDCR", GREEN, 11.5); s.text(822, SB + 216, "signal → BOTTOM pin", GREEN, 11.5)

    # ---- hole sensor, battery sense, button ----
    HY = 1300
    s.box(260, HY, 360, 150, GREEN, "#f0f5ef", cfg["hall"][0], cfg["hall"][1], 20)
    s.pin(260, HY + 80, cfg["hall"][2], RED, "l"); s.pin(260, HY + 120, "GND", BLACK, "l"); s.pin(620, HY + 100, cfg["hall"][3], GREEN, "r")
    s.text(262, HY + 169, cfg["hall"][4], GREY, 10)
    s.box(260, HY + 190, 360, 150, RED, "#faf0ee", "Battery sense 2:1", cfg["sense"][0], 20)
    s.text(290, HY + 278, "VBAT ─ 100k ─┬─ 100k ─ GND", BLACK, 15); s.text(290, HY + 311, "             └─ " + cfg["sense"][1], BLACK, 15)
    s.box(260, HY + 380, 360, 130, GREEN, "#f0f5ef", "Arming button", "momentary, no resistor", 20)
    s.text(290, HY + 468, cfg["button"][0] + " ─ [button] ─ GND", BLACK, 15)
    s.text(262, HY + 529, cfg["button"][1], GREY, 10)

    # ---- wires: pack, 3.3 V, GND ----
    s.wire([(360, 200), (560, 200)], RED, 4.5); s.text(470, 185, "VBAT", RED, 11.5)
    s.wire([(460, 200), (460, 395), (560, 395)], RED, 4.5); s.dot(460, 200, RED)
    s.wire([(460, 395), (460, 525), (40, 525), (40, HY + 270), (270, HY + 270)], RED, 3); s.dot(460, 395, RED)
    s.wire([(40, HY + 80), (260, HY + 80)], RED, 3.5); s.text(50, 510, "VBAT", RED, 11.5)
    s.wire([(360, 240), (560, 240)], BLACK, 4.5)
    s.wire([(500, 240), (500, 435), (560, 435)], BLACK, 4.5); s.dot(500, 240, BLACK)
    RG, RO, RA, RB = 1560, 1200, 1262, 1370      # right-hand lanes: GND, 3.3 V, wire A, wire B
    BO, BG = HY + 560, HY + 600                  # bottom lanes: 3.3 V, GND
    s.wire([(880, 240), (RG, 240), (RG, BG), (150, BG), (150, row(1)), (260, row(1))], BLACK, 4.5)
    s.wire([(880, 435), (960, 435), (960, 240)], BLACK, 4.5); s.dot(960, 240, BLACK)
    s.wire([(RG, YG), (1140, YG)], BLACK, 4.5); s.dot(RG, YG, BLACK)
    s.wire([(150, HY + 120), (260, HY + 120)], BLACK, 3.5); s.dot(150, HY + 120, BLACK)
    s.wire([(880, 200), (RO, 200), (RO, BO), (120, BO), (120, row(0)), (260, row(0))], ORANGE, 4.5)
    s.wire([(RO, Y5), (1140, Y5)], ORANGE, 4.5); s.dot(RO, Y5, ORANGE)
    s.text(RO - 8, BO + 16, "3.3 V", ORANGE, 12, "right"); s.text(RG - 8, BG + 16, "GND", BLACK, 12, "right")

    # ---- 31.5 V: wire A (VPWR through J4) and wire B (VDRV through DB107 + 1N4148) ----
    s.wire([(880, 395), (RB, 395), (RB, YC), (1140, YC)], PURPLE, 4.5)
    s.wire([(RA, 395), (RA, YJ), (1140, YJ)], PURPLE, 4.5); s.dot(RA, 395, PURPLE)
    s.text(1070, 380, "31.5 V", PURPLE, 12, "center")
    s.text(RA - 8, 520, "wire A", PURPLE, 12, "right"); s.text(RB + 10, 520, "wire B", PURPLE, 12)
    s.chip(RB, 640, 96, 46, "DB107", PURPLE); s.text(RB + 58, 628, "in on −, out on +", GREY, 10.5); s.text(RB + 58, 650, "~ ~ legs cut off", GREY, 10.5)
    s.chip(RB, 760, 96, 46, "1N4148", PURPLE); s.text(RB + 58, 760, "stripe toward C6", GREY, 10.5)

    # ---- converter enable wire ----
    s.wire([(720, 490), (720, row(0)), (660, row(0))], PINK, 4.5)
    s.text(732, 528, "enable wire, 30 AWG", PINK, 12)

    # ---- sensor wires (three lanes under the MCU box) ----
    U = MY + 600
    s.wire([(260, row(2)), (200, row(2)), (200, U + 60), (700, U + 60), (700, HY + 100), (620, HY + 100)], GREEN, 3)
    s.wire([(260, row(3)), (220, row(3)), (220, U + 40), (680, U + 40), (680, HY + 317), (620, HY + 317)], RED, 3)
    s.wire([(260, row(4)), (240, row(4)), (240, U + 20), (720, U + 20), (720, HY + 465), (620, HY + 465)], GREEN, 3)

    for i, (col, t) in enumerate(cfg["foot"]): s.text(60, BG + 62 + 30 * i, t, col, 11.6)
    s.save(os.path.join(out_dir, cfg["file"]))


def high_drive_sheet(out_dir):
    s = Sheet(2000, 1420)
    INK, NEW, PALE = "#17242b", "#c0392b", "#fdecea"
    s.text(46, 46, "30 V drive (vmode 2) and converter enable wire", INK, 29, weight="bold")
    s.text(46, 94, "The XL6009 replaces the MT3608. Red = wires and parts to add. Pink = enable wire. Black = already on the shield. All grounds common.", "#3d4b52", 16.5)

    # pack, converter, MCU
    s.box(46, 212, 257, 136, INK, "#eef2f2", lw=3)
    s.text(174, 261, "3 x AA L91 pack", INK, 17.5, "center", weight="bold"); s.text(174, 298, "3.6 - 5.4 V", INK, 17, "center")
    s.wire([(303, 280), (440, 280)], INK, 3.5)
    s.box(440, 212, 318, 136, NEW, PALE, lw=4)
    s.text(599, 252, "XL6009 boost", NEW, 17.5, "center", weight="bold"); s.text(599, 288, "(replaces the MT3608)", INK, 16.5, "center")
    s.text(599, 322, "set 31.5 V first", INK, 17, "center", weight="bold")
    s.text(840, 318, "Still 29 V on a half-used pack?", "#3d4b52", 12.5)
    s.text(840, 342, "If not: MT3608 (12 V) in front of it.", "#3d4b52", 12.5)
    s.text(840, 366, "Minimum input 3.6 V. Not an XL6019", "#3d4b52", 12.5)
    s.text(840, 390, "(180 kHz: inside the sonar band).", "#3d4b52", 12.5)

    # 31.5 V rail, wire A, wire B
    s.wire([(758, 280), (1590, 280), (1590, 372)], NEW, 5); s.dot(1182, 280, NEW, 8)
    s.text(1182, 242, "31.5 V", NEW, 15, "center", weight="bold")
    s.wire([(1182, 280), (1182, 752)], NEW, 5); s.text(1198, 598, "wire A", NEW, 15, weight="bold")
    s.box(1515, 372, 152, 118, NEW, PALE, lw=4)
    s.text(1591, 398, "DB107", NEW, 16.5, "center", weight="bold"); s.text(1540, 432, "−", INK, 17, "center", weight="bold"); s.text(1643, 432, "+", INK, 17, "center", weight="bold")
    s.text(1591, 469, "~ ~ cut off", "#3d4b52", 12.5, "center")
    s.text(1682, 384, "IN on −, OUT on +", INK, 14.5); s.text(1682, 411, "(2 diodes inside)", INK, 14.5)
    s.text(1682, 438, "meter diode mode − → +:", "#3d4b52", 13.5); s.text(1682, 463, "1.0-1.3 V; reversed: OL", "#3d4b52", 13.5)
    s.wire([(1590, 490), (1590, 530)], NEW, 5)
    s.ax.add_patch(Polygon([(1566, 530), (1614, 530), (1590, 568)], closed=True, ec=NEW, fc=PALE, lw=3.5, zorder=4))
    s.wire([(1564, 569), (1616, 569)], NEW, 4); s.wire([(1590, 569), (1590, 665)], NEW, 5)
    s.text(1682, 549, "1N4148", NEW, 16, weight="bold"); s.text(1682, 578, "stripe toward C6", "#3d4b52", 13.5)
    s.text(1606, 646, "wire B", NEW, 15, weight="bold")

    # shield
    s.box(850, 667, 1120, 485, INK, "#f7f8fa", lw=2.5, dashed=True)
    s.text(1946, 703, "Open Echo shield 002", INK, 19.5, "right", weight="bold")
    s.text(894, 735, "J4", INK, 15); s.text(923, 735, "middle pin", INK, 15, weight="bold"); s.text(894, 762, "no jumper on J4", "#3d4b52", 12.5)
    s.box(1121, 727, 122, 52, INK, "white", lw=3)
    for x, c, r in ((1148, INK, 6), (1182, NEW, 8), (1215, INK, 6)): s.dot(x, 753, c, r)
    s.wire([(1182, 779), (1182, 850)], INK, 3.5); s.box(1128, 850, 107, 40, INK, "white", lw=2.5); s.text(1182, 870, "1N4001", INK, 14, "center")
    s.wire([(1182, 890), (1182, 925)], INK, 3.5); s.box(1136, 925, 91, 38, INK, "white", lw=2.5); s.text(1182, 944, "10 Ω", INK, 14, "center")
    s.wire([(1182, 963), (1182, 1030)], INK, 3.5)
    s.wire([(1590, 665), (1590, 742)], NEW, 5)
    s.box(1522, 742, 138, 70, NEW, "white", lw=4); s.text(1591, 765, "C6", INK, 15.5, "center", weight="bold"); s.text(1591, 793, "2 µF, 40 V", INK, 13, "center")
    s.text(1682, 758, "pad closer to the chip", INK, 14); s.text(1682, 783, "(the other pad is GND:", INK, 14); s.text(1682, 808, "it beeps to ground)", INK, 14)
    s.wire([(1590, 812), (1590, 1030)], INK, 3.5)
    s.wire([(1590, 905), (1690, 905)], NEW, 3.5); s.dot(1590, 905, NEW, 6.5)
    s.box(1690, 882, 128, 46, NEW, PALE, lw=2.5); s.text(1754, 905, "100 kΩ", INK, 13.5, "center", weight="bold")
    s.wire([(1818, 905), (1890, 905), (1890, 930)], NEW, 3.5)
    for i, hw in enumerate((27, 17, 8)): s.wire([(1890 - hw, 930 + 8 * i), (1890 + hw, 930 + 8 * i)], INK, 2.5)
    s.text(1682, 972, "bleed resistor", NEW, 12.5, weight="bold"); s.text(1682, 995, "C6 pad to GND, 47-100 kΩ:", "#3d4b52", 11.5); s.text(1682, 1015, "drains C6 at power-off", "#3d4b52", 11.5)
    s.ax.add_patch(FancyBboxPatch((1091, 1030), 575, 84, boxstyle="round,pad=0,rounding_size=10", ec=INK, fc=INK, lw=2, zorder=3))
    s.text(1378, 1058, "TUSS4470", "white", 18, "center", weight="bold")
    s.text(1121, 1094, "VPWR ≈ 31 V", "white", 14.5); s.text(1657, 1094, "VDRV ≈ 29.6 V", "white", 14.5, "right")

    # enable wire: XL6009 chip pin 2 -> GPIO
    s.box(935, 470, 225, 112, "#1565c0", "#eef3fb", lw=3)
    s.text(1047, 496, "ESP32 board", "#1565c0", 15.5, "center", weight="bold")
    s.text(1047, 528, "Heltec V3: GPIO40", INK, 13.5, "center"); s.text(1047, 556, "WROOM DevKit: D13", INK, 13.5, "center")
    s.wire([(620, 348), (620, 526), (935, 526)], PINK, 5); s.dot(620, 348, PINK, 7)
    s.text(634, 376, "EN = chip pin 2", PINK, 14, weight="bold")
    s.text(634, 502, "enable wire, 30 AWG", PINK, 14, weight="bold")
    s.text(634, 552, "HIGH or floating = converter on", "#3d4b52", 11.5); s.text(634, 574, "LOW = off, while each ping listens", "#3d4b52", 11.5)

    # XL6009 package detail
    PX, PY = 46, 1060
    s.text(PX, PY - 150, "The enable pin on the module", INK, 16.5, weight="bold")
    s.text(PX, PY - 121, "XL6009E1, seen from the top, legs toward you. The board", "#3d4b52", 13)
    s.text(PX, PY - 99, "ties pin 2 to VIN: cut it free of its pad, lift it, solder the wire.", "#3d4b52", 13)
    s.ax.add_patch(Rectangle((PX + 30, PY - 70), 250, 46, ec=INK, fc="#cfd6d8", lw=2.5, zorder=3))        # metal tab
    s.ax.add_patch(Rectangle((PX + 30, PY - 24), 250, 96, ec=INK, fc="#2b3a40", lw=2.5, zorder=3))        # body
    s.text(PX + 155, PY + 24, "XL6009", "white", 15, "center", weight="bold")
    for i, nm in enumerate(("GND", "EN", "SW", "VIN", "FB")):
        x = PX + 55 + 50 * i
        if i == 1:
            s.ax.add_patch(Rectangle((x - 8, PY + 72), 16, 30, ec=PINK, fc=PINK, lw=2, zorder=4))
            s.wire([(x, PY + 102), (x, PY + 214)], PINK, 4.5)
            s.text(x + 14, PY + 212, "wire to the GPIO", PINK, 13, weight="bold")
        else:
            s.ax.add_patch(Rectangle((x - 8, PY + 72), 16, 62, ec=INK, fc="#cfd6d8", lw=2, zorder=4))
        s.text(x + (0 if i != 1 else 0), PY + 152, str(i + 1) if i != 1 else "", INK, 13, "center", weight="bold")
        s.text(x, PY + 176, nm if i != 1 else "", "#3d4b52", 11.5, "center")
    s.text(PX + 130, PY + 90, "2", PINK, 12, "center", weight="bold"); s.text(PX + 130, PY + 110, "EN", PINK, 10.5, "center", weight="bold")
    s.text(PX + 300, PY - 6, "Meter check before cutting:", INK, 13, weight="bold")
    s.text(PX + 300, PY + 18, "pin 2 beeps to pin 4 (VIN) = tied on the board.", "#3d4b52", 12.5)
    s.text(PX + 300, PY + 42, "After the cut: no beep, and the output", "#3d4b52", 12.5)
    s.text(PX + 300, PY + 64, "falls when pin 2 is held to GND.", "#3d4b52", 12.5)

    # text column
    y = 478
    for head, lines in (
        ("Rule: VDRV always BELOW VPWR", ["Wire A loses ~0.5 V (1N4001), wire B ~1.2-1.4 V", "(DB107 + 1N4148): gap ~0.7-1 V.", "Gating keeps it: both follow the same rail."]),
        ("Order", ["1. XL6009 alone: 31.5 V on the meter.", "2. Wire A, then wire B: J4 middle ≈ 31 V,", "    C6 pad ≈ 29.6-30.3 V (lower!).", "3. Enable wire, then flash with --gate.", "4. Wiring check: \"Converter enable wire OK\"."]),
    ):
        s.text(46, y, head, INK, 16.5, weight="bold"); y += 34
        for l in lines: s.text(46, y, l, INK, 15); y += 29
        y += 16
    s.text(850, 1200, "What it gives", INK, 16.5, weight="bold")
    s.text(850, 1234, "Drive: 40 → ~60 V p-p on the transducer, +3 to +4.6 dB (est.).", INK, 15)
    s.text(850, 1263, "Receiver floor with the converter gated: −75 → −86 dB, no transmit loss (measured on hub 1, D179).", INK, 15)
    s.text(46, 1352, "Limits: chip 36 V max (40 V absolute), C6 40 V, converter capacitors often 35 V: stay at 31.5 V.  Back to 20 V: vmode 0 AND unplug wire B.", INK, 14)
    s.text(46, 1384, "No enable wire fitted: flash without --gate. The converter then runs all the time and the receiver floor stays about 11 dB higher.", INK, 14)
    s.save(os.path.join(out_dir, "wiring_high_drive.png"))


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "images")
    os.makedirs(out, exist_ok=True)
    for b in BOARDS.values(): board_sheet(b, out)
    high_drive_sheet(out)
