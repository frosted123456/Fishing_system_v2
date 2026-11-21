"""Label-style KiCad 7 schematic from design.py: every part is a box symbol, every pin ends in a global label
with the net name (no wires). Readable for review, editable in KiCad 7/8/9/10."""
import os, sys, uuid
sys.path.insert(0, os.path.dirname(__file__))
import design as D

PIN_NAMES = {
    "U1": {1: "OUT3", 2: "DGND", 3: "NCS", 4: "SCLK", 5: "SDI", 6: "SDO", 7: "IO1", 8: "IO2", 9: "VOUT", 10: "VDD", 11: "INN", 12: "INP", 13: "SGND", 14: "GND", 15: "OUTA", 16: "OUTB", 17: "VDRV", 18: "FLT", 19: "OUT4", 20: "VPWR", 21: "EP"},
    "U2": {1: "VINA", 2: "GND", 3: "FB", 4: "VOUT", 5: "VOUT", 6: "L2", 7: "L2", 8: "L1", 9: "L1", 10: "VIN", 11: "VIN", 12: "EN", 13: "PS/SYNC", 14: "PG", 15: "PGND"},
    "U3": {1: "SW", 2: "GND", 3: "FB", 4: "EN", 5: "IN", 6: "NC"},
    "Q1": {1: "G", 2: "S", 3: "D"}, "Q2": {1: "G", 2: "S", 3: "D"},
    "U8": {1: "DVDD", 2: "SCLK", 3: "SDO", 4: "CS", 5: "AINM", 6: "AINP", 7: "AVDD", 8: "GND"},
    "U9": {1: "IO1", 2: "GND", 3: "IO2", 4: "IO2", 5: "VBUS", 6: "IO1"},
    "P1": {1: "DA", 2: "DB", 3: "VTX", 4: "GND", 5: "XA", 6: "XB"},
    "D7": {1: "A", 2: "K", 3: "K/A"},
    "J4": {1: "3V3", 2: "GND", 3: "BTN", 4: "LED"},
    "U4": {1: "GND", 2: "3V3", 3: "EN", 4: "IO4", 5: "IO5", 6: "IO6", 7: "IO7", 8: "IO15", 9: "IO16", 10: "IO17", 11: "IO18", 12: "IO8",
           13: "IO19/D-", 14: "IO20/D+", 15: "IO3", 16: "IO46", 17: "IO9", 18: "IO10", 19: "IO11", 20: "IO12", 21: "IO13", 22: "IO14",
           23: "IO21", 24: "IO47", 25: "IO48", 26: "IO45", 27: "IO0", 28: "IO35", 29: "IO36", 30: "IO37", 31: "IO38", 32: "IO39",
           33: "IO40", 34: "IO41", 35: "IO42", 36: "RXD0/IO44", 37: "TXD0/IO43", 38: "IO2", 39: "IO1", 40: "GND", 41: "EPAD"},
    "U5": {1: "ANT", 2: "GND", 3: "3V3", 4: "NRST", 5: "TXEN", 6: "DIO1", 9: "GND", 10: "BUSY", 11: "RXEN", 12: "SCK", 13: "MISO",
           14: "MOSI", 15: "NSS", 16: "GND"},
    "J2": {1: "A", 2: "B", 3: "SHIELD", 4: "NTC", 5: "NTC_GND"}, "J3": {1: "3V3", 2: "GND", 3: "OUT"}, "J1": {1: "+", 2: "-"},
}
def pin_name(ref, n):
    if ref in PIN_NAMES: return PIN_NAMES[ref].get(n, str(n))
    if ref.startswith("D"): return {1: "K", 2: "A"}.get(n, str(n))
    return str(n)

def U(): return str(uuid.uuid4())
ROOT = U()
F = lambda v: "%.2f" % v

lib_syms, syms, labels, texts = [], [], [], []

def symbol(ref):
    p = D.P[ref]
    import re as _re
    pins = sorted(p["pins"].keys(), key=lambda k: (str(k).rstrip("0123456789"), int(_re.sub(r"\D", "", str(k)) or 0)))
    n = len(pins)
    name = f"S_{ref}"
    two = n == 2 and not ref.startswith("J")
    if two:
        # horizontal passive: pin 1 left, pin 2 right
        body = (-2.54, -1.27, 2.54, 1.27)
        ppos = {pins[0]: (-5.08, 0, 0), pins[1]: (5.08, 0, 180)}
    else:
        left = pins[: (n + 1) // 2]; right = pins[(n + 1) // 2:]
        h = max(len(left), len(right)) * 2.54 + 2.54
        w = 15.24 if ref.startswith("U") else 10.16
        body = (-w / 2, -h / 2, w / 2, h / 2)
        ppos = {}
        for i, pn in enumerate(left): ppos[pn] = (-w / 2 - 2.54, h / 2 - 2.54 * (i + 1), 0)
        for i, pn in enumerate(right): ppos[pn] = (w / 2 + 2.54, h / 2 - 2.54 * (i + 1), 180)
    pin_sx = "".join(f'''
      (pin passive line (at {F(x)} {F(y)} {a}) (length 2.54)
        (name "{pin_name(ref, pn)}" (effects (font (size 1.0 1.0))))
        (number "{pn}" (effects (font (size 1.0 1.0)))))''' for pn, (x, y, a) in ppos.items())
    lib_syms.append(f'''
    (symbol "SonarPod:{name}" (pin_names (offset 0.6)) (in_bom yes) (on_board yes)
      (property "Reference" "{ref}" (at 0 {F(body[3] + 1.5)} 0) (effects (font (size 1.27 1.27))))
      (property "Value" "{p['value']}" (at 0 {F(body[1] - 1.5)} 0) (effects (font (size 1.0 1.0))))
      (property "Footprint" "{p['fp']}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
      (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
      (symbol "{name}_0_1"
        (rectangle (start {F(body[0])} {F(body[1])}) (end {F(body[2])} {F(body[3])}) (stroke (width 0.254) (type default)) (fill (type background))))
      (symbol "{name}_1_1"{pin_sx}
      ))''')
    return name, ppos, body

def place(ref, x, y):
    p = D.P[ref]
    name, ppos, body = symbol(ref)
    pin_sx = "".join(f'\n      (pin "{pn}" (uuid "{U()}"))' for pn in ppos)
    desc = (p.get("desc") or "").replace('"', "'")
    syms.append(f'''
  (symbol (lib_id "SonarPod:{name}") (at {F(x)} {F(y)} 0) (unit 1) (in_bom {"no" if p.get("dnp") else "yes"}) (on_board yes) (dnp {"yes" if p.get("dnp") else "no"}) (uuid "{U()}")
    (property "Reference" "{ref}" (at {F(x)} {F(y - body[3] - 1.5)} 0) (effects (font (size 1.27 1.27))))
    (property "Value" "{p['value']}" (at {F(x)} {F(y - body[1] + 1.5)} 0) (effects (font (size 1.0 1.0))))
    (property "Footprint" "{p['fp']}" (at {F(x)} {F(y)} 0) (effects (font (size 1.27 1.27)) hide))
    (property "Datasheet" "" (at {F(x)} {F(y)} 0) (effects (font (size 1.27 1.27)) hide))
    (property "LCSC" "{p.get('lcsc', '')}" (at {F(x)} {F(y)} 0) (effects (font (size 1.27 1.27)) hide))
    (property "Note" "{desc}" (at {F(x)} {F(y)} 0) (effects (font (size 1.27 1.27)) hide)){pin_sx}
    (instances (project "{D.BOARD_NAME}" (path "/{ROOT}" (reference "{ref}") (unit 1)))))''')
    for pn, (px, py, a) in ppos.items():
        net = p["pins"][pn]
        if not net: continue
        lx, ly = x + px, y - py
        ang = 180 if a == 0 else 0      # label text points away from the body
        labels.append(f'''
  (global_label "{net}" (shape input) (at {F(lx)} {F(ly)} {ang}) (fields_autoplaced)
    (effects (font (size 1.0 1.0)) (justify {"right" if ang == 180 else "left"}))
    (uuid "{U()}")
    (property "Intersheet References" "${{INTERSHEET_REFS}}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide)))''')
    return body

def text(s, x, y, size=2.0):
    texts.append(f'\n  (text "{s}" (at {F(x)} {F(y)} 0) (effects (font (size {size} {size}) bold) (justify left bottom)) (uuid "{U()}"))')

# ---- layout: blocks in columns (A3 = 420 x 297 mm)
blocks = [
    ("POWER IN + VBAT", ["J1", "D5", "D4", "C1", "R12", "R13", "C22"]),
    ("3.3 V TPS63020", ["U2", "L2", "C2", "R17", "C3", "C4", "C5", "C6", "C7", "R2", "R3"]),
    ("24 V BOOST MT3608", ["U3", "L1", "D1", "C10", "C8", "C9", "C46", "C47", "R4", "R5", "R6", "R14", "R15", "C23"]),
    ("TUSS4470 FRONT END", ["U1", "R7", "C11", "C48", "C12", "C13", "C14", "R8", "R8B", "D7", "C15", "C16", "C17", "FB1", "R34", "R35", "R36", "R11", "C21"]),
    ("DRIVE: R40/R41 or XFMR (DNP)", ["R40", "R41", "Q1", "Q2", "R42", "R43", "R44", "R45", "R46", "C44", "P1", "C43", "L3"]),
    ("ADS7042 12-bit 1 MSPS ADC", ["U8", "R31", "C40", "R32", "C41", "C42", "R33"]),
    ("ESP32-S3 (bottom, hand) + LED", ["U4", "C30", "C31", "R20", "C32", "R21", "R22", "D6", "TP1", "TP7", "TP8"]),
    ("USB-C", ["J5", "U9", "R23", "R24"]),
    ("LoRa Ra-01SH (lora variant)", ["U5", "C36", "C37", "R29"]),
    ("CONNECTORS", ["J2", "R10", "C20", "J3", "R16", "C24", "J4", "TP2", "TP4", "TP5", "TP6"]),
]
x0 = 20.0
for title, refs in blocks:
    y = 30.0
    text(title, x0, 22.0, 1.5)
    colw = 0
    for ref in refs:
        if ref not in D.P: continue
        n = len(D.P[ref]["pins"])
        body = place(ref, x0 + 20.0, y + 10.0)
        hgt = body[3] - body[1]
        y += hgt + 12.0
        colw = max(colw, body[2] - body[0] + 40)
        if y > 270: y = 30.0; x0 += colw; colw = 0
    x0 += colw + 10

notes = ["SONAR POD v0.8 (58x53 mm, review fixes of v0.7: SGND dropped, XA/XB direct, pull-ups, USB ESD, 10 uH boost inductor; top = all JLC parts incl. LoRa, bottom = ESP32-S3 module hand-soldered) - label-style schematic generated from design.py; nets connect by global label name.",
         "Default fit: direct drive OUTA/OUTB -> R40/R41 0R -> transducer. Transformer option: remove R40/R41, fit Q1/Q2/R42-R45/R46/C44 + transformer on P1, swap R8B for 1k-2.2k, fit D7, bridge P1 4-6.",
         "VERIFY BEFORE ORDER: U2 footprint DSJ0014A (self-drawn), AP1002 pinout (1 G 2 S 3 D), JST pin 1 orientation, D1/D4/D5 polarity."]
for i, n in enumerate(notes): text(n, 20.0, 285.0 + 4 * i, 1.6)

out = f'''(kicad_sch (version 20230121) (generator sonarpod)
  (uuid "{ROOT}")
  (paper "A1")
  (title_block (title "Sonar pod board") (rev "v0.7") (comment 1 "TUSS4470 + MT3608 24 V + TPS63020 3.3 V + ESP32-S3"))
  (lib_symbols{"".join(lib_syms)}
  ){"".join(texts)}{"".join(labels)}{"".join(syms)}
  (sheet_instances (path "/" (page "1")))
)
'''
os.makedirs("out", exist_ok=True)
open(os.path.join(os.path.dirname(__file__), "out", D.BOARD_NAME + ".kicad_sch"), "w").write(out)
print("schematic written", len(syms), "symbols", len(labels), "labels")
