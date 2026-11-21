"""Universal BOM + centroid (.xlsx) for NextPCB / PCBWay. Rotations = raw KiCad (IPC-7351 zero), no JLC correction."""
import sys, os, csv, collections
sys.path.insert(0, "/home/claude/pcb8"); import design as D
from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment
OUT = sys.argv[1]
MPN = {  # lcsc: (manufacturer, MPN, description, substitutes ok)
 "C25803": ("UNI-ROYAL", "0603WAF1003T5E", "Resistor 100k 1% 0603", "Yes"),
 "C25804": ("UNI-ROYAL", "0603WAF1002T5E", "Resistor 10k 1% 0603", "Yes"),
 "C25810": ("UNI-ROYAL", "0603WAF1802T5E", "Resistor 18k 1% 0603", "Yes"),
 "C25811": ("UNI-ROYAL", "0603WAF2003T5E", "Resistor 200k 1% 0603", "Yes"),
 "C23186": ("UNI-ROYAL", "0603WAF5101T5E", "Resistor 5.1k 1% 0603", "Yes"),
 "C22859": ("UNI-ROYAL", "0603WAF100JT5E", "Resistor 10R 0603", "Yes"),
 "C8218":  ("UNI-ROYAL", "0603WAF2000T5E", "Resistor 200R 1% 0603", "Yes"),
 "C22775": ("UNI-ROYAL", "0603WAF1000T5E", "Resistor 100R 1% 0603", "Yes"),
 "C4190":  ("UNI-ROYAL", "0603WAF2201T5E", "Resistor 2.2k 1% 0603", "Yes"),
 "C23162": ("UNI-ROYAL", "0603WAF4701T5E", "Resistor 4.7k 1% 0603", "Yes"),
 "C17415": ("UNI-ROYAL", "0805W8F100JT5E", "Resistor 10R 0805", "Yes"),
 "C15850": ("Samsung", "CL21A106KAYNNNE", "MLCC 10uF 25V X5R 0805", "Yes (>=25V X5R/X7R)"),
 "C45783": ("Samsung", "CL21A226MAQNNNE", "MLCC 22uF 25V X5R 0805", "Yes (>=25V X5R/X7R)"),
 "C13585": ("Samsung", "CL31A106KBHNNNE", "MLCC 10uF 50V X5R 1206", "Yes (>=50V)"),
 "C50254": ("Samsung", "CL31B225KBHNNNE", "MLCC 2.2uF 50V X7R 1206", "Yes (>=50V)"),
 "C14663": ("Yageo", "CC0603KRX7R9BB104", "MLCC 100nF 50V X7R 0603", "Yes"),
 "C57112": ("Fenghua (FH)", "0603B103K500NT", "MLCC 10nF 50V X7R 0603", "Yes"),
 "C15849": ("Samsung", "CL10A105KB8NNNC", "MLCC 1uF 50V X5R 0603", "Yes"),
 "C1664":  ("Samsung", "CL10C331JB8NNNC", "MLCC 330pF 50V C0G 0603", "C0G only"),
 "C21122": ("Samsung", "CL10B223KB8NNNC", "MLCC 22nF 50V X7R 0603", "Yes"),
 "C1613":  ("Samsung", "CL10B332KB8NNNC", "MLCC 3.3nF 50V X7R 0603", "Yes"),
 "C1588":  ("Samsung", "CL10B102KB8NNNC", "MLCC 1nF 50V X7R 0603", "Yes"),
 "C1017":  ("Sunlord", "GZ2012D601TF", "Ferrite bead 600R@100MHz 0805", "Yes (600R, >=1A)"),
 "C8598":  ("Changjiang (CJ)", "B5819W SL", "Schottky 40V 1A SOD-123", "Yes (40V 1A SOD-123)"),
 "C2286":  ("KENTO", "KT-0603R", "LED red 0603", "Yes"),
 "C2146":  ("JSCJ", "S8050 J3Y", "NPN transistor SOT-23", "Yes (S8050/MMBT2222 SOT-23)"),
 "C5446":  ("Torex", "XC6206P332MR-G", "LDO 3.3V 200mA SOT-23", "Yes (XC6206P332MR pin-compatible)"),
 "C1855776": ("Texas Instruments", "TUSS4470TRTJR", "Ultrasonic driver/receiver QFN-20 4x4", "No"),
 "C15483": ("Texas Instruments", "TPS63020DSJR", "Buck-boost 3.3V VSON-14 3x4 (DSJ)", "No"),
 "C84817": ("Aerosemi", "MT3608", "Boost converter SOT-23-6", "No"),
 "C1847949": ("Bourns", "SRP4020TA-220M", "Inductor 22uH shielded 4x4mm", "Yes (22uH 4x4, Isat>=1A)"),
 "C80662": ("Sunlord", "SWPA4018S1R5NT", "Inductor 1.5uH 4x4mm", "Yes (1.5uH 4x4, Isat>=3A)"),
 "C50033030": ("Espressif", "ESP32-S3-WROOM-1U-N16R16VA", "ESP32-S3 module, 16MB flash, 16MB PSRAM, U.FL antenna", "Any ESP32-S3-WROOM-1U (N4/N8/N16/R2/R8)"),
 "C21190": ("UNI-ROYAL", "0603WAF1001T5E", "Resistor 1k 1% 0603", "Yes"),
 "C84681": ("WCH", "CH340C", "USB-UART, internal osc, SOIC-16", "No (CH340C only, not CH340G)"),
 "C165948": ("Korean Hroparts (HRO)", "TYPE-C-31-M-12", "USB-C receptacle 16-pin SMD", "No (footprint specific)"),
 "C2764087": ("Ai-Thinker", "Ra-01SH", "LoRa module SX1262 915MHz SMD-16", "No"),
 "C17888": ("UNI-ROYAL", "1206W4F0000T5E", "Resistor 0R 1206 (R8B, swap for 1k-2.2k in transformer mode)", "Yes"),
 "C21189": ("UNI-ROYAL", "0603WAF0000T5E", "Resistor 0R 0603", "Yes"),
 "C2500": ("Changjiang (CJ)", "BAV99", "Dual switching diode SOT-23 (DNP)", "Yes"),
 "C2995324": ("Diodes/various", "AP1002", "100V N-FET SOT-23 (DNP, pinout to verify)", "Check pinout"),
 "C701641": ("Texas Instruments", "ADS7042IDCUR", "12-bit 1MSPS SAR ADC VSSOP-8", "No"),
 "C88374": ("Hirose", "U.FL-R-SMT-1(80)", "U.FL coax receptacle", "Yes (U.FL compatible)"),
}
HAND = {"U4": ("Espressif", "ESP32-S3-WROOM-1U-N8", "ESP32-S3 module, 8MB flash, U.FL (LCSC C2980297), BOTTOM side; N4/N16/N8R2 also fit"),
        "J7": ("JST", "B4B-PH-K-S(LF)(SN)", "Header 4p 2.0mm vertical THT (expansion, optional)"),
        "P1": ("generic", "2x3 pin header 2.54 mm", "Transformer port, only for the transformer option"),
        "J1": ("JST", "B2B-XH-A(LF)(SN)", "Header 2p 2.5mm vertical THT"),
        "J2": ("JST", "B5B-XH-A(LF)(SN)", "Header 5p 2.5mm vertical THT"),
        "J3": ("JST", "B3B-XH-A(LF)(SN)", "Header 3p 2.5mm vertical THT"),
        "J4": ("JST", "B4B-PH-K-S(LF)(SN)", "Header 4p 2.0mm vertical THT")}
pos = {r["Ref"]: r for r in csv.DictReader(open("/home/claude/pcb8/out/jlc/pos_raw.csv"))}
B = Font(bold=True); HDR = PatternFill("solid", fgColor="DDEBF7")
def sheet(ws, head, rows, widths):
    ws.append(head)
    for c in ws[1]: c.font = B; c.fill = HDR; c.alignment = Alignment(wrap_text=True)
    for r in rows: ws.append(r)
    for i, w in enumerate(widths): ws.column_dimensions[chr(65 + i)].width = w
    ws.freeze_panes = "A2"
nat = lambda s: (s.rstrip("0123456789"), int(s[len(s.rstrip("0123456789")):] or 0))
for var in ("tipup", "lora"):
    inc = lambda p: p.get("variant") in (None, var)
    asm = {r: p for r, p in D.P.items() if inc(p) and not p.get("dnp") and not p.get("handsolder") and p.get("lcsc")}
    g = collections.OrderedDict()
    for r in sorted(asm, key=nat): g.setdefault(asm[r]["lcsc"], []).append(r)
    rows = []
    for i, (l, refs) in enumerate(sorted(g.items(), key=lambda kv: nat(kv[1][0])), 1):
        m, mpn, desc, sub = MPN[l]; p = asm[refs[0]]
        rows.append([i, len(refs), ", ".join(refs), p["value"], p["fp"].split(":")[1], m, mpn, desc, l, sub, "SMD"])
    wb = Workbook(); ws = wb.active; ws.title = "BOM (assemble)"
    sheet(ws, ["Item", "Qty per board", "Designators", "Value", "Footprint", "Manufacturer", "Manufacturer P/N", "Description", "LCSC #", "Substitute allowed", "Type"],
          rows, [6, 8, 34, 24, 34, 20, 22, 38, 11, 26, 6])
    na = [[r, "Hand-solder (not assembled)", D.P[r]["value"], *HAND[r]] for r in ("U4", "J1", "J2", "J3", "J4", "P1")]
    na += [[r, "DNP (leave empty)", D.P[r]["value"], "", "", D.P[r]["fp"].split(":")[1]] for r in sorted(D.P, key=nat) if D.P[r].get("dnp")]
    if var == "tipup": na += [[r, "DNP in this variant (lora only)", D.P[r]["value"], "", "", D.P[r]["fp"].split(":")[1]] for r in sorted(D.P, key=nat) if D.P[r].get("variant") == "lora" and not D.P[r].get("dnp")]
    sheet(wb.create_sheet("Not assembled"), ["Designator", "Status", "Value", "Manufacturer", "Manufacturer P/N", "Description / footprint"], na, [11, 30, 22, 14, 22, 40])
    nt = wb.create_sheet("Notes")
    for line in ["Sonar pod v0.7 (%s variant), 58 x 53 mm, 4 layers (F sig / In1 GND / In2 GND / B sig), 1.6 mm, HASL lead-free. Assembled parts all on TOP (incl. LoRa in the lora variant); ESP32 module on the bottom is hand-soldered." % var,
                 "%d BOM lines, %d placed parts per board. Through-hole headers and the ESP32-S3 module (bottom side) are hand-soldered (not assembled)." % (len(rows), sum(map(len, g.values()))),
                 "LCSC # given as a cross-reference; Manufacturer P/N is the part to buy.",
                 "Smallest package 0603, finest pitch 0.5 mm (TUSS4470 QFN, TPS63020 VSON). Min drill 0.3 mm. No ENIG required.",
                 "Pin-1 check: U1 TUSS4470, U2 TPS63020, U3 MT3608, U5 Ra-01SH (lora, rotated 180), D1/D4/D5 cathode, D6 LED. Vias: tented."]:
        nt.append([line])
    nt.column_dimensions["A"].width = 120
    wb.save(os.path.join(OUT, "sonar_pod_v07_%s_bom.xlsx" % var))
    # centroid
    crow = []
    for r in sorted(asm, key=nat):
        q = pos[r]; crow.append([r, D.P[r]["value"], q["Package"], round(float(q["PosX"]), 3), round(float(q["PosY"]), 3), "Top", round(float(q["Rot"]) % 360, 1)])
    wb = Workbook(); ws = wb.active; ws.title = "Centroid"
    sheet(ws, ["Designator", "Comment", "Footprint", "Mid X (mm)", "Mid Y (mm)", "Layer", "Rotation (deg)"], crow, [11, 24, 38, 11, 11, 7, 13])
    nt = wb.create_sheet("Notes")
    for line in ["Units mm. Origin (0,0) = bottom-left corner of the board outline, X right, Y up (same origin as the gerbers/drill).",
                 "Board 58 x 53 mm; all assembled parts top side. Rotation = KiCad / IPC-7351 zero orientation, counter-clockwise, no fab-specific correction.",
                 "Hand-soldered parts (U4 bottom, J1-J4, P1), DNP parts and mounting holes are not listed."]:
        nt.append([line])
    nt.column_dimensions["A"].width = 120
    wb.save(os.path.join(OUT, "sonar_pod_v07_%s_centroid.xlsx" % var))
    print(var, len(rows), "lines", sum(map(len, g.values())), "parts", len(crow), "centroid rows")
