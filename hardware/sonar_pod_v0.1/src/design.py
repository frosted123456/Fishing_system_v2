"""Sonar pod board v0.1 - single source of truth: parts, pins, nets, placement.
TUSS4470 front end copied net-for-net from Open Echo shield_003 (200 kHz values fixed),
MT3608 boost (22 V, EN from the head), TPS63020 3.3 V, sense dividers, one 16-pin head cable.
"""
# ---- parts: ref -> dict(value, fp (lib:name), lcsc, pins {num: netname}, pos (x, y, rot), side, desc)
P = {}
def part(ref, value, fp, pins, pos, lcsc="", rot=0, side="F", desc="", dnp=False, mfr=""):
    P[ref] = dict(value=value, fp=fp, pins=pins, pos=pos, rot=rot, side=side, lcsc=lcsc, desc=desc, dnp=dnp, mfr=mfr)

R0603 = "Resistor_SMD:R_0603_1608Metric"; R1206 = "Resistor_SMD:R_1206_3216Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"; C0805 = "Capacitor_SMD:C_0805_2012Metric"; C1206 = "Capacitor_SMD:C_1206_3216Metric"

# ---------------- power input ----------------
part("J1", "BAT 3xL91", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", {1: "VBAT", 2: "GND"}, (4.5, 13.0), rot=90, lcsc="C158012", desc="JST XH 2p, battery holder", mfr="B2B-XH-A")
part("Q1", "AO3401A", "Package_TO_SOT_SMD:SOT-23", {1: "GND", 2: "VB", 3: "VBAT"}, (11.5, 4.0), rot=0, lcsc="C15127", desc="P-FET reverse-battery (G=1 S=2 D=3)")
part("R1", "100k", R0603, {1: "GND", 2: "VBAT"}, (16.5, 3.0), lcsc="C25803", desc="Q1 gate")   # gate to GND (Vgs = -Vbat); R only limits inrush on the gate
part("C1", "10uF 10V", C0805, {1: "VB", 2: "GND"}, (16.5, 5.5), rot=0, lcsc="C15850", desc="input bulk")

# ---------------- 3.3 V buck-boost TPS63020 ----------------
# DSJ0014A pins: 1 VINA 2 GND 3 FB 4,5 VOUT 6,7 L2 8,9 L1 10,11 VIN 12 EN 13 PS/SYNC 14 PG, 15 = PGND pad
part("U2", "TPS63020DSJR", "SonarPod:Texas_DSJ0014A", {1: "VINA", 2: "GND", 3: "FB_T", 4: "3V3", 5: "3V3", 6: "TL2", 7: "TL2", 8: "TL1", 9: "TL1", 10: "VB", 11: "VB", 12: "VB", 13: "GND", 14: "", 15: "GND"},
     (15.5, 13.5), rot=0, lcsc="C15483", desc="buck-boost 3.3 V 3 A; EN=VIN, PS/SYNC=GND (power save)")
part("L2", "1.5uH XFL4020", "Inductor_SMD:L_Coilcraft_XxL4020", {1: "TL1", 2: "TL2"}, (15.5, 19.5), rot=0, lcsc="C150393", desc="Coilcraft XFL4020-152ME or equivalent 4x4 1.5uH >4 A", mfr="XFL4020-152MEB")
part("R2", "1M", R0603, {1: "3V3", 2: "FB_T"}, (11.0, 8.5), lcsc="C22935", desc="FB top (0.5 V ref -> 3.28 V)")
part("R3", "180k", R0603, {1: "FB_T", 2: "GND"}, (11.0, 18.5), lcsc="C25270", desc="FB bottom")
part("C2", "100nF", C0603, {1: "VINA", 2: "GND"}, (19.5, 9.5), rot=0, lcsc="C14663", desc="VINA bypass (max 0.22 uF)")
part("R17", "10R", R0603, {1: "VB", 2: "VINA"}, (19.5, 7.5), rot=0, lcsc="C25092", desc="VINA RC from VB")
part("C3", "10uF 10V", C0805, {1: "VB", 2: "GND"}, (20.0, 12.5), rot=0, lcsc="C15850", desc="VIN")
part("C4", "10uF 10V", C0805, {1: "VB", 2: "GND"}, (20.0, 14.5), rot=0, lcsc="C15850", desc="VIN")
part("C5", "22uF 6.3V", C0805, {1: "3V3", 2: "GND"}, (10.5, 11.5), rot=0, lcsc="C45783", desc="VOUT")
part("C6", "22uF 6.3V", C0805, {1: "3V3", 2: "GND"}, (10.5, 13.5), rot=0, lcsc="C45783", desc="VOUT")
part("C7", "22uF 6.3V", C0805, {1: "3V3", 2: "GND"}, (10.5, 15.5), rot=0, lcsc="C45783", desc="VOUT")

# ---------------- 22 V boost MT3608 (VPWR) ----------------
# SOT-23-6: 1 SW 2 GND 3 FB 4 EN 5 IN 6 NC
part("U3", "MT3608", "Package_TO_SOT_SMD:SOT-23-6", {1: "SW_B", 2: "GND", 3: "FB_B", 4: "EN_B", 5: "VB", 6: ""}, (25.5, 28.0), rot=0, lcsc="C84817", desc="boost 22 V, EN pulled up, head can switch it")
part("L1", "22uH NR6028", "Inductor_SMD:L_Taiyo-Yuden_NR-60xx", {1: "VB", 2: "SW_B"}, (19.0, 29.0), rot=0, lcsc="C16218", desc="6x6 22 uH >1.5 A sat (same family as shield_003)", mfr="NR6028T220M")
part("D1", "PMEG6010ELR", "Diode_SMD:Nexperia_CFP3_SOD-123W", {1: "VBOOST", 2: "SW_B"}, (30.5, 25.5), rot=0, lcsc="C179432", desc="60 V 1 A Schottky (shield_003 part); K=1 A=2")
part("C8", "10uF 50V", C1206, {1: "VBOOST", 2: "GND"}, (35.0, 29.5), rot=90, lcsc="C13585", desc="boost out")
part("C9", "10uF 50V", C1206, {1: "VBOOST", 2: "GND"}, (38.5, 30.0), rot=90, lcsc="C13585", desc="boost out")
part("C10", "10uF 10V", C0805, {1: "VB", 2: "GND"}, (13.5, 27.0), rot=90, lcsc="C15850", desc="boost in")
part("R4", "360k", R0603, {1: "VBOOST", 2: "FB_B"}, (31.5, 31.5), lcsc="C23245", desc="FB top: 0.6*(1+360/10) = 22.2 V")
part("R5", "10k", R0603, {1: "FB_B", 2: "GND"}, (28.5, 31.5), lcsc="C25804", desc="FB bottom")
part("R6", "100k", R0603, {1: "VB", 2: "EN_B"}, (25.5, 31.5), lcsc="C25803", desc="EN pull-up: boost on by default")
part("D2", "SMAJ24A", "Diode_SMD:D_SMA", {1: "VBOOST", 2: "GND"}, (42.5, 31.0), rot=90, lcsc="C129537", desc="TVS 24 V standoff VBOOST (K=1) to GND (A=2)", mfr="SMAJ24A")

# ---------------- TUSS4470 ----------------
part("U1", "TUSS4470TRTJR", "Package_DFN_QFN:QFN-20-1EP_4x4mm_P0.5mm_EP2.5x2.5mm_ThermalVias",
     {1: "", 2: "GND", 3: "NCS", 4: "SCLK", 5: "SDI", 6: "SDO", 7: "IO1", 8: "IO2", 9: "VOUT_I", 10: "3V3", 11: "INN", 12: "INP", 13: "SGND", 14: "GND", 15: "XA", 16: "XB", 17: "VDRV", 18: "FLT", 19: "OUT4", 20: "VPWR", 21: "GND"},
     (31.5, 14.5), rot=0, lcsc="C1855776", desc="ultrasonic driver + log-amp receiver")
part("R7", "10R", R1206, {1: "VBOOST", 2: "VPWR"}, (25.0, 7.5), rot=0, lcsc="C17903", desc="RPWR (datasheet 8-1)")
part("C11", "100nF 50V", C0603, {1: "VPWR", 2: "GND"}, (28.5, 9.5), rot=0, lcsc="C14663", desc="CPWR1 at pin 20")
part("C12", "2.2uF 50V", C1206, {1: "VDRV", 2: "GND"}, (37.0, 11.0), rot=0, lcsc="C50254", desc="CPWR2 VDRV storage (shield: 2 uF 40 V)")
part("C13", "10nF", C0603, {1: "3V3", 2: "GND"}, (29.0, 19.5), rot=0, lcsc="C57112", desc="CVDD at pin 10")
part("C14", "1uF", C0603, {1: "3V3", 2: "GND"}, (32.0, 19.5), rot=0, lcsc="C15849", desc="VDD bulk")
part("R8", "200R 1206", R1206, {1: "XA", 2: "RINP"}, (40.5, 15.5), rot=0, lcsc="C25361", desc="RINP (1/4 W)")
part("C15", "330pF 50V", C0603, {1: "RINP", 2: "INP"}, (36.5, 15.5), rot=0, lcsc="C1664", desc="CINP")
part("C16", "22nF", C0603, {1: "INN", 2: "SGND"}, (36.5, 17.5), rot=0, lcsc="C21122", desc="CINN 200 kHz (1/(2*pi*150*f/4))")
part("C17", "3.3nF", C0603, {1: "FLT", 2: "SGND"}, (32.5, 9.0), rot=90, lcsc="C1613", desc="CFLT 200 kHz (25/(2*pi*6250*f))")
part("C18", "DNP", C0603, {1: "INN", 2: "SGND"}, (36.5, 19.5), rot=0, desc="spare CINN for tuning", dnp=True)
part("C19", "DNP", C0603, {1: "FLT", 2: "SGND"}, (31.0, 8.5), rot=90, desc="spare CFLT for tuning", dnp=True)
part("FB1", "600R@100MHz", "Inductor_SMD:L_0603_1608Metric", {1: "SGND", 2: "GND"}, (39.5, 20.0), rot=0, lcsc="C1017", desc="SGND-GND ferrite (datasheet layout 10.1)", mfr="BLM18PG601SN1D")
part("R9", "DNP 0R", R0603, {1: "SGND", 2: "GND"}, (39.5, 22.0), rot=0, desc="SGND-GND short, alternative to FB1", dnp=True)
part("D3", "SMAJ24CA", "Diode_SMD:D_SMA", {1: "XA", 2: "XB"}, (42.0, 5.0), rot=90, lcsc="C129545", desc="bidirectional TVS across the transducer", mfr="SMAJ24CA")
part("R11", "100R", R0603, {1: "VOUT_I", 2: "VOUT"}, (26.0, 13.0), rot=0, lcsc="C22775", desc="VOUT series for the cable")
part("C21", "1nF", C0603, {1: "VOUT", 2: "GND"}, (26.0, 15.0), rot=0, lcsc="C1588", desc="VOUT cable filter")

# ---------------- sense + hall ----------------
part("R12", "100k", R0603, {1: "VB", 2: "VBAT_ADC"}, (4.5, 31.5), rot=0, lcsc="C25803", desc="VBAT/2")
part("R13", "100k", R0603, {1: "VBAT_ADC", 2: "GND"}, (4.5, 33.5), rot=0, lcsc="C25803")
part("C22", "100nF", C0603, {1: "VBAT_ADC", 2: "GND"}, (4.5, 35.5), rot=0, lcsc="C14663")
part("R14", "75k", R0603, {1: "VBOOST", 2: "VPWR_ADC"}, (46.5, 28.5), rot=90, lcsc="C23242", desc="VPWR x10/85 (shield_003)")
part("R15", "10k", R0603, {1: "VPWR_ADC", 2: "GND"}, (46.5, 31.5), rot=90, lcsc="C25804")
part("C23", "100nF", C0603, {1: "VPWR_ADC", 2: "GND"}, (46.5, 34.5), rot=90, lcsc="C14663")
part("R10", "10k", R0603, {1: "3V3", 2: "NTC_ADC"}, (42.5, 24.5), rot=0, lcsc="C25804", desc="NTC pull-up (10k NTC on the transducer cable)")
part("C20", "100nF", C0603, {1: "NTC_ADC", 2: "GND"}, (42.5, 26.5), rot=0, lcsc="C14663")
part("R16", "10k", R0603, {1: "3V3", 2: "HALL"}, (8.5, 31.5), rot=0, lcsc="C25804", desc="Hall latch pull-up (open drain)")
part("C24", "1nF", C0603, {1: "HALL", 2: "GND"}, (8.5, 33.5), rot=0, lcsc="C1588")

# ---------------- connectors ----------------
part("J2", "XDCR", "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical", {1: "XA", 2: "XB", 3: "SGND", 4: "NTC_ADC", 5: "GND"}, (47.5, 21.0), rot=90, lcsc="C144401", desc="transducer A, B, shield; NTC, NTC gnd", mfr="B5B-XH-A")
part("J3", "HALL", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", {1: "3V3", 2: "GND", 3: "HALL"}, (4.5, 25.5), rot=90, lcsc="C144394", desc="US1881 on the arm", mfr="B3B-XH-A")
part("J4", "HEAD", "Connector_JST:JST_PH_B16B-PH-K_1x16_P2.00mm_Vertical",
     {1: "3V3", 2: "GND", 3: "GND", 4: "NCS", 5: "SCLK", 6: "SDI", 7: "SDO", 8: "IO1", 9: "IO2", 10: "VOUT", 11: "OUT4", 12: "HALL", 13: "EN_B", 14: "VBAT_ADC", 15: "NTC_ADC", 16: "VPWR_ADC"},
     (10.0, 38.0), rot=0, lcsc="C131338", desc="to the ESP32 in the head, 16 wires", mfr="B16B-PH-K-S")
part("TP1", "VOUT", "TestPoint:TestPoint_Pad_1.5x1.5mm", {1: "VOUT_I"}, (23.0, 14.0), desc="test point")
part("TP2", "VDRV", "TestPoint:TestPoint_Pad_1.5x1.5mm", {1: "VDRV"}, (41.5, 11.0), desc="test point")
part("TP3", "VBOOST", "TestPoint:TestPoint_Pad_1.5x1.5mm", {1: "VBOOST"}, (35.0, 25.5), desc="test point")
part("TP4", "3V3", "TestPoint:TestPoint_Pad_1.5x1.5mm", {1: "3V3"}, (11.0, 22.0), rot=0, desc="test point")
part("TP5", "GND", "TestPoint:TestPoint_Pad_1.5x1.5mm", {1: "GND"}, (11.0, 24.5), rot=0, desc="test point")
for i, (x, y) in enumerate([(3.0, 3.0), (49.0, 3.0), (3.0, 40.0), (49.0, 40.0)], 1):
    part(f"H{i}", "M3", "MountingHole:MountingHole_3.2mm_M3", {}, (x, y), desc="mounting hole")

BOARD_W, BOARD_H = 52.0, 43.0
BOARD_NAME = "sonar_pod"

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
