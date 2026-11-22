"""Sonar pod board v0.2 (tight: 36 x 30, 0402, PHD 2x8 head connector, 4020 inductors, SMF TVS) - single source of truth: parts, pins, nets, placement.
TUSS4470 front end copied net-for-net from Open Echo shield_003 (200 kHz values fixed),
MT3608 boost (22 V, EN from the head), TPS63020 3.3 V, sense dividers, one 16-pin head cable.
"""
# ---- parts: ref -> dict(value, fp (lib:name), lcsc, pins {num: netname}, pos (x, y, rot), side, desc)
P = {}
def part(ref, value, fp, pins, pos, lcsc="", rot=0, side="F", desc="", dnp=False, mfr=""):
    P[ref] = dict(value=value, fp=fp, pins=pins, pos=pos, rot=rot, side=side, lcsc=lcsc, desc=desc, dnp=dnp, mfr=mfr)

R0603 = "Resistor_SMD:R_0402_1005Metric"   # v0.2: signal resistors 0402
R1206 = "Resistor_SMD:R_1206_3216Metric"
C0603 = "Capacitor_SMD:C_0402_1005Metric"   # v0.2: signal caps 0402
C0805 = "Capacitor_SMD:C_0805_2012Metric"; C1206 = "Capacitor_SMD:C_1206_3216Metric"

# ---------------- power input ----------------
part("J1", "BAT 3xL91", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", {1: "VBAT", 2: "GND"}, (4.5, 13.0), rot=90, lcsc="C158012", desc="JST XH 2p, battery holder", mfr="B2B-XH-A")
part("Q1", "AO3401A", "Package_TO_SOT_SMD:SOT-23", {1: "GND", 2: "VB", 3: "VBAT"}, (11.5, 4.0), rot=0, lcsc="C15127", desc="P-FET reverse-battery (G=1 S=2 D=3)")
part("R1", "100k", R0603, {1: "GND", 2: "VBAT"}, (16.5, 3.0), lcsc="verify-0402", desc="Q1 gate")   # gate to GND (Vgs = -Vbat); R only limits inrush on the gate
part("C1", "10uF 10V", C0805, {1: "VB", 2: "GND"}, (16.5, 5.5), rot=0, lcsc="C15850", desc="input bulk")

# ---------------- 3.3 V buck-boost TPS63020 ----------------
# DSJ0014A pins: 1 VINA 2 GND 3 FB 4,5 VOUT 6,7 L2 8,9 L1 10,11 VIN 12 EN 13 PS/SYNC 14 PG, 15 = PGND pad
part("U2", "TPS63020DSJR", "SonarPod:Texas_DSJ0014A", {1: "VINA", 2: "GND", 3: "FB_T", 4: "3V3", 5: "3V3", 6: "TL2", 7: "TL2", 8: "TL1", 9: "TL1", 10: "VB", 11: "VB", 12: "VB", 13: "GND", 14: "", 15: "GND"},
     (15.5, 13.5), rot=0, lcsc="C15483", desc="buck-boost 3.3 V 3 A; EN=VIN, PS/SYNC=GND (power save)")
part("L2", "1.5uH XFL4020", "Inductor_SMD:L_Coilcraft_XxL4020", {1: "TL1", 2: "TL2"}, (15.5, 19.5), rot=0, lcsc="C150393", desc="Coilcraft XFL4020-152ME or equivalent 4x4 1.5uH >4 A", mfr="XFL4020-152MEB")
part("R2", "1M", R0603, {1: "3V3", 2: "FB_T"}, (11.0, 8.5), lcsc="verify-0402", desc="FB top (0.5 V ref -> 3.28 V)")
part("R3", "180k", R0603, {1: "FB_T", 2: "GND"}, (11.0, 18.5), lcsc="verify-0402", desc="FB bottom")
part("C2", "100nF", C0603, {1: "VINA", 2: "GND"}, (19.5, 9.5), rot=0, lcsc="verify-0402", desc="VINA bypass (max 0.22 uF)")
part("R17", "10R", R0603, {1: "VB", 2: "VINA"}, (19.5, 7.5), rot=0, lcsc="verify-0402", desc="VINA RC from VB")
part("C3", "10uF 10V", C0805, {1: "VB", 2: "GND"}, (20.0, 12.5), rot=0, lcsc="C15850", desc="VIN")
part("C4", "10uF 10V", C0805, {1: "VB", 2: "GND"}, (20.0, 14.5), rot=0, lcsc="C15850", desc="VIN")
part("C5", "22uF 6.3V", C0805, {1: "3V3", 2: "GND"}, (10.5, 11.5), rot=0, lcsc="C45783", desc="VOUT")
part("C6", "22uF 6.3V", C0805, {1: "3V3", 2: "GND"}, (10.5, 13.5), rot=0, lcsc="C45783", desc="VOUT")
part("C7", "22uF 6.3V", C0805, {1: "3V3", 2: "GND"}, (10.5, 15.5), rot=0, lcsc="C45783", desc="VOUT")

# ---------------- 22 V boost MT3608 (VPWR) ----------------
# SOT-23-6: 1 SW 2 GND 3 FB 4 EN 5 IN 6 NC
part("U3", "MT3608", "Package_TO_SOT_SMD:SOT-23-6", {1: "SW_B", 2: "GND", 3: "FB_B", 4: "EN_B", 5: "VB", 6: ""}, (25.5, 28.0), rot=0, lcsc="C84817", desc="boost 22 V, EN pulled up, head can switch it")
part("L1", "22uH 4020", "Inductor_SMD:L_Sunlord_SWPA4020S", {1: "VB", 2: "SW_B"}, (19.0, 29.0), rot=0, lcsc="verify", desc="4x4 22 uH, Isat >= 1.0 A (Sunlord SWPA4020S220MT or Coilcraft XFL4020-223)", mfr="SWPA4020S220MT")
part("D1", "PMEG6010ELR", "Diode_SMD:Nexperia_CFP3_SOD-123W", {1: "VBOOST", 2: "SW_B"}, (30.5, 25.5), rot=0, lcsc="C179432", desc="60 V 1 A Schottky (shield_003 part); K=1 A=2")
part("C8", "10uF 50V", C1206, {1: "VBOOST", 2: "GND"}, (35.0, 29.5), rot=90, lcsc="C13585", desc="boost out")
part("C9", "10uF 50V", C1206, {1: "VBOOST", 2: "GND"}, (38.5, 30.0), rot=90, lcsc="C13585", desc="boost out")
part("C10", "10uF 10V", C0805, {1: "VB", 2: "GND"}, (13.5, 27.0), rot=90, lcsc="C15850", desc="boost in")
part("R4", "360k", R0603, {1: "VBOOST", 2: "FB_B"}, (31.5, 31.5), lcsc="verify-0402", desc="FB top: 0.6*(1+360/10) = 22.2 V")
part("R5", "10k", R0603, {1: "FB_B", 2: "GND"}, (28.5, 31.5), lcsc="verify-0402", desc="FB bottom")
part("R6", "100k", R0603, {1: "VB", 2: "EN_B"}, (25.5, 31.5), lcsc="verify-0402", desc="EN pull-up: boost on by default")
part("D2", "SMF24A", "Diode_SMD:D_SMF", {1: "VBOOST", 2: "GND"}, (42.5, 31.0), rot=90, lcsc="verify", desc="TVS 24 V standoff VBOOST (K=1) to GND (A=2)", mfr="SMF24A")

# ---------------- TUSS4470 ----------------
part("U1", "TUSS4470TRTJR", "Package_DFN_QFN:QFN-20-1EP_4x4mm_P0.5mm_EP2.5x2.5mm_ThermalVias",
     {1: "", 2: "GND", 3: "NCS", 4: "SCLK", 5: "SDI", 6: "SDO", 7: "IO1", 8: "IO2", 9: "VOUT_I", 10: "3V3", 11: "INN", 12: "INP", 13: "SGND", 14: "GND", 15: "XA", 16: "XB", 17: "VDRV", 18: "FLT", 19: "OUT4", 20: "VPWR", 21: "GND"},
     (31.5, 14.5), rot=0, lcsc="C1855776", desc="ultrasonic driver + log-amp receiver")
part("R7", "10R", "Resistor_SMD:R_0805_2012Metric", {1: "VBOOST", 2: "VPWR"}, (25.0, 7.5), rot=0, lcsc="verify", desc="RPWR (datasheet 8-1), 0805")
part("C11", "100nF 50V", "Capacitor_SMD:C_0603_1608Metric", {1: "VPWR", 2: "GND"}, (28.5, 9.5), rot=0, lcsc="C14663", desc="CPWR1 at pin 20")
part("C12", "2.2uF 50V", C0805, {1: "VDRV", 2: "GND"}, (37.0, 11.0), rot=0, lcsc="verify", desc="CPWR2 VDRV storage 0805 X7R (shield: 2 uF 40 V 1206)")
part("C13", "10nF", C0603, {1: "3V3", 2: "GND"}, (29.0, 19.5), rot=0, lcsc="verify-0402", desc="CVDD at pin 10")
part("C14", "1uF", C0603, {1: "3V3", 2: "GND"}, (32.0, 19.5), rot=0, lcsc="verify-0402", desc="VDD bulk")
part("R8", "200R 1206", R1206, {1: "XA", 2: "RINP"}, (40.5, 15.5), rot=0, lcsc="C25361", desc="RINP (1/4 W)")
part("C15", "330pF 50V", C0603, {1: "RINP", 2: "INP"}, (36.5, 15.5), rot=0, lcsc="verify-0402", desc="CINP")
part("C16", "22nF", C0603, {1: "INN", 2: "SGND"}, (36.5, 17.5), rot=0, lcsc="verify-0402", desc="CINN 200 kHz (1/(2*pi*150*f/4))")
part("C17", "3.3nF", C0603, {1: "FLT", 2: "SGND"}, (32.5, 9.0), rot=90, lcsc="verify-0402", desc="CFLT 200 kHz (25/(2*pi*6250*f))")
part("C18", "DNP", C0603, {1: "INN", 2: "SGND"}, (36.5, 19.5), rot=0, desc="spare CINN for tuning", dnp=True)
part("C19", "DNP", C0603, {1: "FLT", 2: "SGND"}, (31.0, 8.5), rot=90, desc="spare CFLT for tuning", dnp=True)
part("FB1", "600R@100MHz", "Inductor_SMD:L_0603_1608Metric", {1: "SGND", 2: "GND"}, (39.5, 20.0), rot=0, lcsc="C1017", desc="SGND-GND ferrite (datasheet layout 10.1)", mfr="BLM18PG601SN1D")
part("R9", "DNP 0R", R0603, {1: "SGND", 2: "GND"}, (39.5, 22.0), rot=0, desc="SGND-GND short, alternative to FB1", dnp=True)
part("D3", "SMF24CA", "Diode_SMD:D_SMF", {1: "XA", 2: "XB"}, (42.0, 5.0), rot=90, lcsc="verify", desc="bidirectional TVS across the transducer", mfr="SMF24CA")
part("R11", "100R", R0603, {1: "VOUT_I", 2: "VOUT"}, (26.0, 13.0), rot=0, lcsc="verify-0402", desc="VOUT series for the cable")
part("C21", "1nF", C0603, {1: "VOUT", 2: "GND"}, (26.0, 15.0), rot=0, lcsc="verify-0402", desc="VOUT cable filter")

# ---------------- sense + hall ----------------
part("R12", "100k", R0603, {1: "VB", 2: "VBAT_ADC"}, (4.5, 31.5), rot=0, lcsc="verify-0402", desc="VBAT/2")
part("R13", "100k", R0603, {1: "VBAT_ADC", 2: "GND"}, (4.5, 33.5), rot=0, lcsc="verify-0402")
part("C22", "100nF", C0603, {1: "VBAT_ADC", 2: "GND"}, (4.5, 35.5), rot=0, lcsc="verify-0402")
part("R14", "75k", R0603, {1: "VBOOST", 2: "VPWR_ADC"}, (46.5, 28.5), rot=90, lcsc="verify-0402", desc="VPWR x10/85 (shield_003)")
part("R15", "10k", R0603, {1: "VPWR_ADC", 2: "GND"}, (46.5, 31.5), rot=90, lcsc="verify-0402")
part("C23", "100nF", C0603, {1: "VPWR_ADC", 2: "GND"}, (46.5, 34.5), rot=90, lcsc="verify-0402")
part("R10", "10k", R0603, {1: "3V3", 2: "NTC_ADC"}, (42.5, 24.5), rot=0, lcsc="verify-0402", desc="NTC pull-up (10k NTC on the transducer cable)")
part("C20", "100nF", C0603, {1: "NTC_ADC", 2: "GND"}, (42.5, 26.5), rot=0, lcsc="verify-0402")
part("R16", "10k", R0603, {1: "3V3", 2: "HALL"}, (8.5, 31.5), rot=0, lcsc="verify-0402", desc="Hall latch pull-up (open drain)")
part("C24", "1nF", C0603, {1: "HALL", 2: "GND"}, (8.5, 33.5), rot=0, lcsc="verify-0402")

# ---------------- connectors ----------------
part("J2", "XDCR", "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical", {1: "XA", 2: "XB", 3: "SGND", 4: "NTC_ADC", 5: "GND"}, (47.5, 21.0), rot=90, lcsc="C144401", desc="transducer A, B, shield; NTC, NTC gnd", mfr="B5B-XH-A")
part("J3", "HALL", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", {1: "3V3", 2: "GND", 3: "HALL"}, (4.5, 25.5), rot=90, lcsc="C144394", desc="US1881 on the arm", mfr="B3B-XH-A")
part("J4", "HEAD", "Connector_JST:JST_PHD_B16B-PHDSS_2x08_P2.00mm_Vertical",
     {1: "3V3", 2: "GND", 3: "GND", 4: "NCS", 5: "SCLK", 6: "SDI", 7: "SDO", 8: "IO1", 9: "IO2", 10: "VOUT", 11: "OUT4", 12: "HALL", 13: "EN_B", 14: "VBAT_ADC", 15: "NTC_ADC", 16: "VPWR_ADC"},
     (10.0, 38.0), rot=0, lcsc="verify", desc="to the ESP32 in the head, 16 wires, 2 x 8 at 2.0 mm (PHD)", mfr="B16B-PHDSS")
part("TP1", "VOUT", "TestPoint:TestPoint_Pad_1.5x1.5mm", {1: "VOUT_I"}, (23.0, 14.0), desc="test point")
part("TP3", "VBOOST", "TestPoint:TestPoint_Pad_1.5x1.5mm", {1: "VBOOST"}, (35.0, 25.5), desc="test point")
for i, (x, y) in enumerate([(2.3, 2.3), (33.7, 2.3), (2.3, 27.7)], 1):
    part(f"H{i}", "M2", "MountingHole:MountingHole_2.2mm_M2", {}, (x, y), desc="mounting hole")

BOARD_W, BOARD_H = 38.0, 32.0
BOARD_NAME = "sonar_pod_v02"

def nets():
    n = {}
    for ref, p in P.items():
        for pin, net in p["pins"].items():
            if net: n.setdefault(net, []).append((ref, pin))
    return n

if __name__ == "__main__":
    n = nets()
    for k in sorted(n, key=lambda k: -len(n[k])): print("%-9s %s" % (k, " ".join(f"{r}.{p}" for r, p in n[k])))
    single = [k for k in n if len(n[k]) == 1]
    print("single-pin nets:", single)

# ---------------- v0.2 placement: fixed parts + anchors for the packer (pack.py) ----------------
V02 = {
    "J1": ((4.0, 10.8), 90), "J3": ((4.0, 22.3), 90), "J2": ((33.4, 22.0), 90), "J4": ((11.0, 27.0), 0),
    "H1": ((2.3, 2.3), 0), "H2": ((35.7, 2.3), 0), "H3": ((2.3, 29.7), 0),
    "U1": ((23.5, 13.5), 0), "U2": ((12.5, 12.0), 0), "L2": ((12.5, 17.5), 0), "U3": ((19.0, 21.5), 0), "L1": ((29.9, 27.8), 0),
    # anchors (packer moves them to the nearest free spot)
    "C16": ((28.2, 15.0), 0), "C18": ((28.2, 16.2), 0), "C15": ((28.2, 12.5), 90), "R8": ((28.5, 9.0), 90), "C17": ((24.5, 9.5), 0), "C19": ((24.5, 8.0), 0),
    "C12": ((21.0, 8.5), 0), "C11": ((18.5, 10.0), 0), "R7": ((18.5, 8.0), 0), "C13": ((24.5, 17.2), 0), "C14": ((26.5, 17.5), 0),
    "FB1": ((27.0, 18.5), 90), "R9": ((27.0, 20.5), 90), "D3": ((27.0, 5.5), 90), "R11": ((17.5, 13.0), 0), "C21": ((17.5, 14.5), 0),
    "R10": ((29.0, 23.5), 0), "C20": ((29.0, 24.7), 0), "TP1": ((17.0, 11.0), 0),
    "C10": ((15.5, 21.5), 90), "D1": ((23.5, 22.5), 0), "C8": ((33.4, 27.4), 90), "C9": ((35.9, 27.4), 90), "R4": ((17.0, 23.0), 0), "R5": ((19.0, 23.0), 0), "R6": ((15.0, 20.5), 90),
    "D2": ((32.5, 3.5), 0), "TP3": ((26.0, 23.5), 0), "R14": ((35.0, 6.0), 90), "R15": ((35.0, 8.0), 90), "C23": ((32.5, 6.0), 0),
    "C3": ((15.5, 10.5), 0), "C4": ((15.5, 12.5), 0), "C5": ((8.5, 11.0), 90), "C6": ((8.5, 13.0), 90), "C7": ((8.5, 15.0), 90),
    "C2": ((9.5, 8.0), 0), "R17": ((12.0, 8.0), 0), "R2": ((9.5, 14.5), 0), "R3": ((9.5, 16.0), 0),
    "Q1": ((9.5, 4.0), 0), "R1": ((12.0, 4.0), 0), "C1": ((15.0, 4.0), 0),
    "R12": ((9.0, 19.0), 0), "R13": ((9.0, 20.5), 0), "C22": ((9.0, 22.0), 0), "R16": ((16.0, 17.5), 0), "C24": ((16.0, 18.7), 0),
}
for _r, (_pos, _rot) in V02.items():
    P[_r]["pos"] = _pos; P[_r]["rot"] = _rot

# ---------------- v0.2.1: JLC-standardized BOM (Basic parts wherever possible, 0603 passives) ----------------
# Every LCSC number below was checked on jlcpcb.com/partdetail on 2026-10-11 (library type in the comment).
R06 = "Resistor_SMD:R_0603_1608Metric"; C06 = "Capacitor_SMD:C_0603_1608Metric"
def _set(ref, value=None, fp=None, lcsc=None, desc=None, dnp=None):
    p = P[ref]
    if value is not None: p["value"] = value
    if fp is not None: p["fp"] = fp
    if lcsc is not None: p["lcsc"] = lcsc
    if desc is not None: p["desc"] = desc
    if dnp is not None: p["dnp"] = dnp
# resistors 0603 (UNI-ROYAL 0603WAF, all Basic)
for r in ("R1", "R6", "R12", "R13"): _set(r, "100k", R06, "C25803")
_set("R14", "100k", R06, "C25803", "VPWR sense top: VPWR x 10/110 (2.2 V at 24 V)")
for r in ("R10", "R15", "R16"): _set(r, "10k", R06, "C25804")
_set("R2", "100k", R06, "C25803", "TPS63020 FB top: 0.5 x (1 + 100/18) = 3.28 V")
_set("R3", "18k", R06, "C25810", "TPS63020 FB bottom")
_set("R4", "200k", R06, "C25811", "MT3608 FB top: 0.6 x (1 + 200/5.1) = 24.1 V")
_set("R5", "5.1k", R06, "C23186", "MT3608 FB bottom")
_set("R11", "100R", R06, "C22775"); _set("R17", "10R", R06, "C22859")
_set("R8", "200R", R06, "C8218", "RINP (burst only, ~11 mW peak: 0603 is plenty)")
_set("R7", "10R", "Resistor_SMD:R_0805_2012Metric", "C17415", "RPWR (datasheet 8-1)")
_set("R9", "DNP 0R", R06, "", dnp=True)
# capacitors (Samsung / FH, all Basic) - TUSS4470 values are exactly the Open Echo shield parts
for c in ("C2", "C11", "C20", "C22", "C23"): _set(c, "100nF 50V", C06, "C14663")
_set("C13", "10nF", C06, "C57112", "CVDD at pin 10"); _set("C14", "1uF", C06, "C15849", "VDD bulk")
for c in ("C21", "C24"): _set(c, "1nF", C06, "C1588")
_set("C15", "330pF C0G", C06, "C1664", "CINP (shield_003 part)")
_set("C16", "22nF", C06, "C21122", "CINN 200 kHz (shield_003 part)")
_set("C17", "3.3nF", C06, "C1613", "CFLT 200 kHz (shield: 3.2 nF)")
for c in ("C18", "C19"): _set(c, "DNP", C06, "", dnp=True)
for c in ("C1", "C3", "C4", "C10"): _set(c, "10uF 25V", "Capacitor_SMD:C_0805_2012Metric", "C15850")
for c in ("C5", "C6", "C7"): _set(c, "22uF 25V", "Capacitor_SMD:C_0805_2012Metric", "C45783")
for c in ("C8", "C9"): _set(c, "10uF 50V", "Capacitor_SMD:C_1206_3216Metric", "C13585")
_set("C12", "2.2uF 50V", "Capacitor_SMD:C_1206_3216Metric", "C50254", "CPWR2 VDRV storage (Basic, 1206)")
# magnetics, diodes
_set("FB1", "600R@100MHz", "Inductor_SMD:L_0805_2012Metric", "C1017", "SGND-GND ferrite GZ2012D601TF (0805, Basic)")
_set("D1", "B5819W", "Diode_SMD:D_SOD-123", "C8598", "40 V 1 A Schottky (Basic); K=1 A=2")
_set("D3", "DNP SMF24CA", None, "", "optional TVS across the transducer (hand-fit if wanted)", dnp=True)
_set("L1", "22uH SRP4020TA-220M", "Inductor_SMD:L_Sunlord_SWPA4020S", "C1847949", "22 uH 4x4 (Extended)")
_set("L2", "1.5uH YNR4020-1R5N", "Inductor_SMD:L_Sunlord_SWPA4020S", "C340376", "1.5 uH 4x4 Isat 4.45 A (Extended)")
# ICs (Extended, no alternative)
_set("U1", None, "Package_DFN_QFN:QFN-20-1EP_4x4mm_P0.5mm_EP2.5x2.5mm")   # no 0.2 mm thermal vias (JLC surcharge)
_set("Q1", None, None, "C15127")
# D2 (TVS on VBOOST) removed: 24.1 V boost sits at the SMF24A stand-off; the TUSS4470 VPWR is rated 36 V
del P["D2"]
# connectors: through-hole, hand-soldered -> not in the JLC BOM/CPL
for j in ("J1", "J2", "J3", "J4"): P[j]["handsolder"] = True
BOARD_NAME = "sonar_pod_v021"
# v0.2.1 board: 0603 passives need ~2 mm more each way
BOARD_W, BOARD_H = 40.0, 34.0
for _r, _dx, _dy in (("J2", 2, 0), ("J4", 0, 2), ("H2", 2, 0), ("H3", 0, 2), ("L1", 2, 2), ("C8", 2, 2), ("C9", 2, 2)):
    _x, _y = P[_r]["pos"]; P[_r]["pos"] = (_x + _dx, _y + _dy)
for _i, (_x, _y) in enumerate([(2.3, 2.3), (37.7, 2.3), (2.3, 31.7)], 1):
    P[f"H{_i}"]["pos"] = (_x, _y)
# routing room around U1: test points dropped (VOUT is on J4.10, VBOOST on C8/C9 pads)
del P["TP1"]; del P["TP3"]
# v0.2.1b: L2 -> Sunlord SWPA4018S1R5NT (C80662): same maker/land as the KiCad footprint (L_Sunlord_SWPA4020S),
# Isat 3.35 A (load < 1 A). YJYCOIN YNR4020 (C340376) remains the higher-Isat alternative.
P["L2"]["value"] = "1.5uH SWPA4018S1R5NT"; P["L2"]["lcsc"] = "C80662"; P["L2"]["desc"] = "1.5 uH 4x4 Sunlord, Isat 3.35 A (Extended)"

# =====================================================================================================================
# v0.3: ESP32 integrated (ESP32-WROOM-32UE, U.FL -> coax to the head SMA), USB-C + CH340C with auto-reset,
#       optional LoRa (Ai-Thinker Ra-01SH, SX1262) + U.FL. One PCB, two BOMs: "tipup" (no LoRa) and "lora" (all).
#       Pin map keeps the existing sensor_wroom firmware: SPI 18/19/23, TUSS CS 5, IO1 26, IO2 25, VOUT 36,
#       HALL 27, LED 2, VBAT 34.  New: OUT4 32, BOOST_EN 33, VPWR_ADC 35, NTC 39, BTN 15,
#       LoRa NSS 4, DIO1 16, RXEN 17, RST 21, BUSY 22, TXEN 13 (SPI shared with the TUSS4470).
# =====================================================================================================================
BOARD_NAME = "sonar_pod_v03"
BOARD_W, BOARD_H = 62.0, 44.0
DX = 22.0                                   # sonar section moves right; the left side takes the radios
for _r in list(P):
    if _r.startswith("H") or _r in ("J1", "J2", "J3", "J4"): continue
    _x, _y = P[_r]["pos"]; P[_r]["pos"] = (_x + DX, _y)
for _r in ("J4", "Q1", "R1"): del P[_r]       # head harness gone; battery reverse FET replaced by an OR-ing diode
B5819 = "Diode_SMD:D_SOD-123"
# --- power OR-ing: battery and USB both feed VB through a Schottky (no charging of the L91 primaries from USB)
part("D5", "B5819W", B5819, {1: "VB", 2: "VBAT"}, (DX + 9.5, 4.0), lcsc="C8598", desc="battery OR diode (K=VB)")
part("D4", "B5819W", B5819, {1: "VB", 2: "VBUS"}, (22.0, 36.0), lcsc="C8598", desc="USB OR diode (K=VB)")
# --- connectors (hand-soldered JSTs on the bottom edge; transducer on the right edge)
part("J1", "BAT 3xL91", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", {1: "VBAT", 2: "GND"}, (51.0, 40.5), rot=180)
part("J3", "HALL", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", {1: "3V3", 2: "GND", 3: "HALL"}, (42.0, 40.5), rot=180)
part("J4", "HEAD", "Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical", {1: "3V3", 2: "GND", 3: "BTN", 4: "LED"}, (33.0, 40.8), rot=180,
     desc="head: button + LED (antenna arrives by coax)")
P["J2"]["pos"] = (58.6, 20.0); P["J2"]["rot"] = 90
for _r in ("J1", "J3", "J4"): P[_r]["handsolder"] = True
# --- ESP32
part("U4", "ESP32-WROOM-32U-N16", "RF_Module:ESP32-WROOM-32UE",
     {1: "GND", 2: "3V3", 3: "EN", 4: "VOUT", 5: "NTC_ADC", 6: "VBAT_ADC", 7: "VPWR_ADC", 8: "OUT4", 9: "EN_B", 10: "IO2",
      11: "IO1", 12: "HALL", 13: "", 14: "", 15: "GND", 16: "LORA_TXEN", 23: "BTN", 24: "LED", 25: "IO0", 26: "LORA_NSS",
      27: "LORA_DIO1", 28: "LORA_RXEN", 29: "NCS", 30: "SCLK", 31: "SDO", 33: "LORA_RST", 34: "RXD0", 35: "TXD0",
      36: "LORA_BUSY", 37: "SDI", 38: "GND", 39: "GND"},
     (11.0, 11.5), rot=0, lcsc="C529580", desc="ESP32 classic WROOM-32U-N16, U.FL antenna (JLC Economic OK; 32UE-N4 C701344 is Standard Only)")
part("C30", "10uF 25V", "Capacitor_SMD:C_0805_2012Metric", {1: "3V3", 2: "GND"}, (22.0, 3.0), lcsc="C15850", desc="ESP32 bulk")
part("C31", "100nF 50V", R06.replace("Resistor", "Capacitor").replace("R_0603", "C_0603"), {1: "3V3", 2: "GND"}, (22.0, 5.5), lcsc="C14663")
part("R20", "10k", R06, {1: "3V3", 2: "EN"}, (22.0, 8.0), lcsc="C25804", desc="EN pull-up")
part("C32", "1uF", C06, {1: "EN", 2: "GND"}, (22.0, 10.0), lcsc="C15849", desc="EN delay")
part("R21", "10k", R06, {1: "3V3", 2: "IO0"}, (22.0, 12.0), lcsc="C25804", desc="IO0 pull-up")
part("R22", "2.2k", R06, {1: "LED", 2: "LED_A"}, (22.0, 14.0), lcsc="C4190")
part("D6", "red", "LED_SMD:LED_0603_1608Metric", {1: "GND", 2: "LED_A"}, (22.0, 16.0), lcsc="C2286", desc="status LED (GPIO2, low at boot)")
# --- USB-C + CH340C + auto-reset (CH340 powered from VBUS through its own 3.3 V LDO: zero drain on the battery)
part("J5", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
     {"A1": "GND", "B12": "GND", "A12": "GND", "B1": "GND", "S1": "GND", "A4": "VBUS", "B9": "VBUS", "A9": "VBUS", "B4": "VBUS",
      "A5": "CC1", "B5": "CC2", "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DN", "B7": "USB_DN"},
     (13.0, 39.7), rot=0, lcsc="C165948", desc="USB-C 16P (HRO TYPE-C-31-M-12)")
part("R23", "5.1k", R06, {1: "CC1", 2: "GND"}, (6.0, 36.0), lcsc="C23186", desc="CC pull-down (5 V sink)")
part("R24", "5.1k", R06, {1: "CC2", 2: "GND"}, (20.0, 36.0), lcsc="C23186")
part("U6", "CH340C", "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm",
     {1: "GND", 2: "CH_TX", 3: "CH_RX", 4: "V3USB", 5: "USB_DP", 6: "USB_DN", 13: "DTR", 14: "RTS", 16: "V3USB"},
     (13.0, 29.0), rot=90, lcsc="C84681", desc="USB-UART, internal oscillator")
part("U7", "XC6206P332MR", "Package_TO_SOT_SMD:SOT-23", {1: "GND", 2: "V3USB", 3: "VBUS"}, (5.0, 31.0), lcsc="C5446", desc="3.3 V for the CH340 only, from VBUS")
part("C33", "1uF", C06, {1: "VBUS", 2: "GND"}, (5.0, 34.0), lcsc="C15849")
part("C34", "1uF", C06, {1: "V3USB", 2: "GND"}, (5.0, 28.0), lcsc="C15849")
part("C35", "100nF 50V", C06, {1: "V3USB", 2: "GND"}, (20.0, 28.0), lcsc="C14663")
part("R25", "4.7k", R06, {1: "CH_TX", 2: "RXD0"}, (20.0, 25.0), lcsc="C23162", desc="UART series (limits back-feed when USB is unplugged)")
part("R26", "4.7k", R06, {1: "TXD0", 2: "CH_RX"}, (20.0, 23.0), lcsc="C23162")
part("Q2", "S8050", "Package_TO_SOT_SMD:SOT-23", {1: "Q2B", 2: "RTS", 3: "EN"}, (6.0, 23.0), lcsc="C2146", desc="auto-reset (B=1 E=2 C=3)")
part("Q3", "S8050", "Package_TO_SOT_SMD:SOT-23", {1: "Q3B", 2: "DTR", 3: "IO0"}, (6.0, 26.5), lcsc="C2146")
part("R27", "10k", R06, {1: "DTR", 2: "Q2B"}, (9.5, 23.0), lcsc="C25804")
part("R28", "10k", R06, {1: "RTS", 2: "Q3B"}, (9.5, 26.5), lcsc="C25804")
# --- LoRa (variant "lora" only)
part("U5", "Ra-01SH", "RF_Module:Ai-Thinker-Ra-01-LoRa",
     {1: "LORA_ANT", 2: "GND", 3: "3V3", 4: "LORA_RST", 5: "LORA_TXEN", 6: "LORA_DIO1", 9: "GND", 10: "LORA_BUSY",
      11: "LORA_RXEN", 12: "SCLK", 13: "SDO", 14: "SDI", 15: "LORA_NSS", 16: "GND"},
     (11.0, 31.5), rot=0, lcsc="C2764087", desc="SX1262 915 MHz module (Ra-01 compatible SMD16)")
part("J6", "U.FL", "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical", {1: "LORA_ANT", 2: "GND"}, (1.8, 24.5), lcsc="C88374", desc="LoRa antenna")
part("C36", "10uF 25V", "Capacitor_SMD:C_0805_2012Metric", {1: "3V3", 2: "GND"}, (21.0, 31.0), lcsc="C15850")
part("C37", "100nF 50V", C06, {1: "3V3", 2: "GND"}, (21.0, 33.5), lcsc="C14663")
part("R29", "10k", R06, {1: "3V3", 2: "LORA_NSS"}, (21.0, 35.5), lcsc="C25804", desc="NSS pull-up")
for _r in ("U5", "J6", "C36", "C37", "R29"): P[_r]["variant"] = "lora"
# --- holes
for _i, (_x, _y) in enumerate([(59.7, 2.3), (2.3, 41.7), (59.7, 41.7)], 1):
    P[f"H{_i}"]["pos"] = (_x, _y)
# --- v0.3 layout (fixed parts; small parts are anchors for pack.py)
BOARD_W, BOARD_H = 62.0, 50.0
_pos = {
    "U4": ((11.0, 11.5), 0), "J6": ((3.4, 23.4), 0), "U5": ((11.0, 34.0), 0), "J5": ((11.0, 46.6), 0),
    "C30": ((21.6, 3.5), 90), "C31": ((21.6, 6.5), 90), "R20": ((21.6, 9.0), 90), "C32": ((21.6, 11.5), 90), "R21": ((21.6, 14.0), 90),
    "R22": ((21.6, 16.5), 90), "D6": ((21.6, 19.0), 90),
    "C36": ((21.4, 28.0), 90), "C37": ((21.4, 31.0), 90), "R29": ((21.4, 33.5), 90),
    "U6": ((27.0, 42.0), 90), "U7": ((22.0, 46.5), 0), "C33": ((19.0, 47.5), 90), "C34": ((24.5, 47.5), 90), "C35": ((31.5, 42.0), 90),
    "R23": ((4.0, 43.0), 90), "R24": ((18.0, 43.0), 90), "D4": ((22.0, 39.5), 0),
    "R25": ((31.5, 38.5), 90), "R26": ((33.5, 38.5), 90), "Q2": ((23.0, 36.0), 0), "Q3": ((27.0, 36.0), 0), "R27": ((30.0, 35.5), 0), "R28": ((30.0, 33.8), 0),
    "J4": ((39.0, 47.0), 180), "J3": ((47.5, 46.6), 180), "J1": ((56.5, 46.6), 180), "J2": ((58.6, 20.0), 90),
}
for _r, (_p, _rot) in _pos.items():
    P[_r]["pos"] = _p; P[_r]["rot"] = _rot
# --- v0.3 layout, round 2: 66 x 50, left column ESP32 / U.FL / Ra-01SH, bottom strip USB + CH340 + JSTs
BOARD_W, BOARD_H = 66.0, 50.0
_pos2 = {
    "U4": ((11.0, 11.5), 0), "J6": ((3.4, 24.4), 0), "U5": ((11.0, 36.0), 0),
    "J5": ((27.0, 46.6), 0), "U6": ((39.5, 38.0), 90), "U7": ((25.0, 39.5), 0), "D4": ((30.5, 40.0), 0),
    "C33": ((22.5, 41.5), 90), "C34": ((27.5, 41.5), 90), "C35": ((45.5, 38.0), 90),
    "R23": ((21.8, 45.5), 90), "R24": ((32.4, 45.5), 90), "R25": ((34.5, 35.0), 0), "R26": ((34.5, 36.8), 0),
    "Q2": ((47.5, 36.0), 0), "Q3": ((51.0, 36.0), 0), "R27": ((47.5, 39.0), 0), "R28": ((51.0, 39.0), 0),
    "J4": ((38.5, 47.0), 180), "J3": ((50.0, 46.6), 180), "J1": ((60.3, 46.6), 180), "J2": ((62.6, 20.0), 90),
    "H1": ((63.7, 2.3), 0), "H2": ((2.3, 47.7), 0), "H3": ((63.7, 36.0), 0),
}
for _r, (_p, _rot) in _pos2.items():
    P[_r]["pos"] = _p; P[_r]["rot"] = _rot
for _r, (_p, _rot) in {"J4": ((41.0, 47.0), 180), "J3": ((53.0, 46.6), 180), "J1": ((62.1, 31.0), 90), "J2": ((62.1, 18.5), 90),
                      "H3": ((63.7, 40.5), 0)}.items():
    P[_r]["pos"] = _p; P[_r]["rot"] = _rot
