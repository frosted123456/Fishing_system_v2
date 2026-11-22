"""Sonar pod board v0.4 - single source of truth (parts, pins, nets, placement).

Changes vs v0.3 (see README):
  * ESP32-S3-WROOM-1U (U.FL). Pin map avoids GPIO35-37 (octal PSRAM) and GPIO47/48 (1.8 V on the R16V part), so ANY
    WROOM-1U variant fits: N16R16VA (JLC Economic) or N8 (hand-solder).
  * Native USB on GPIO19/20: CH340C, its LDO, the 2 auto-reset transistors and 9 passives are gone.
  * 4-layer board: F.Cu signals / In1 GND / In2 GND / B.Cu signals. Every trace has a solid reference plane.
  * TUSS4470 on its own SPI bus (LoRa no longer shares it); LoRa on its own pins.
  * Functional placement: radios left, sonar right, power bottom; hot loops compact; filters at the ADC pins.
  * Sonar output: 1k + 3.3 nF anti-alias filter at the ESP32 pin (S3 ADC continuous max 83.3 kHz).
  * MT3608 EN: pull-DOWN (boost off unless the ESP32 drives it high) - no 5 V back-feed into a 3.3 V GPIO.
Coordinates: mm, origin top-left, Y down (KiCad).
"""
P = {}
def part(ref, value, fp, pins, pos, rot=0, lcsc="", desc="", dnp=False, hand=False, variant=None, fixed=False):
    P[ref] = dict(value=value, fp=fp, pins=pins, pos=pos, rot=rot, side="F", lcsc=lcsc, desc=desc, dnp=dnp,
                  handsolder=hand, variant=variant, fixed=fixed)

BOARD_NAME = "sonar_pod_v04"
BOARD_W, BOARD_H = 74.0, 56.0
R06 = "Resistor_SMD:R_0603_1608Metric"; C06 = "Capacitor_SMD:C_0603_1608Metric"
C08 = "Capacitor_SMD:C_0805_2012Metric"; C12 = "Capacitor_SMD:C_1206_3216Metric"
SOD = "Diode_SMD:D_SOD-123"; IND = "Inductor_SMD:L_Sunlord_SWPA4020S"; TP = "TestPoint:TestPoint_Pad_1.0x1.0mm"
def R(ref, val, a, b, pos, rot=0, desc=""):
    lc = {"100k": "C25803", "10k": "C25804", "18k": "C25810", "200k": "C25811", "5.1k": "C23186", "10R": "C22859",
          "200R": "C8218", "1k": "C21190", "2.2k": "C4190"}[val]
    part(ref, val, R06, {1: a, 2: b}, pos, rot, lc, desc)
def C(ref, val, a, b, pos, rot=0, desc=""):
    fp, lc = {"100nF": (C06, "C14663"), "10nF": (C06, "C57112"), "1uF": (C06, "C15849"), "330pF": (C06, "C1664"),
              "22nF": (C06, "C21122"), "3.3nF": (C06, "C1613"), "1nF": (C06, "C1588"), "10uF 25V": (C08, "C15850"),
              "22uF 25V": (C08, "C45783"), "10uF 50V": (C12, "C13585"), "2.2uF 50V": (C12, "C50254")}[val]
    v = {"100nF": "100nF 50V", "330pF": "330pF C0G"}.get(val, val)
    part(ref, v, fp, {1: a, 2: b}, pos, rot, lc, desc)

# ======================= ESP32-S3-WROOM-1U (rot 180: U.FL at the bottom, pins 1-14 on the RIGHT side) ===============
part("U4", "ESP32-S3-WROOM-1U", "RF_Module:ESP32-S3-WROOM-1U",
     {1: "GND", 2: "3V3", 3: "EN",
      4: "VPWR_ADC",   # IO4  ADC1_CH3
      5: "NTC_ADC",    # IO5  ADC1_CH4
      6: "VOUT",       # IO6  ADC1_CH5  sonar envelope (continuous ADC)
      7: "TS_IO2",     # IO7  TUSS IO2 = burst (RMT)
      8: "TS_IO1",     # IO15 TUSS IO1
      9: "SDO",        # IO16 TUSS SDO -> MISO
      10: "SDI",       # IO17 MOSI
      11: "SCLK",      # IO18
      12: "NCS",       # IO8
      13: "USB_DN", 14: "USB_DP",          # IO19 / IO20 native USB
      15: "OUT4",      # IO3  TUSS comparator out (MCPWM capture)
      17: "EXP_SDA",   # IO9  expansion header
      18: "EXP_SCL",   # IO10
      19: "LED",       # IO11
      20: "BTN",       # IO12 (RTC: deep-sleep wake)
      21: "LORA_DIO1", # IO13
      22: "LORA_TXEN", # IO14
      23: "LORA_RST",  # IO21
      27: "IO0",       # boot strap
      31: "LORA_NSS",  # IO38
      32: "LORA_MOSI", # IO39
      33: "LORA_MISO", # IO40
      34: "LORA_SCK",  # IO41
      35: "LORA_RXEN", # IO42
      36: "LORA_BUSY", # IO44 (U0RXD)
      37: "EN_B",      # IO43 (U0TXD) boost enable (boot-time toggling only pulses the boost)
      38: "HALL",      # IO2  (RTC: deep-sleep wake) ADC1_CH1
      39: "VBAT_ADC",  # IO1  ADC1_CH0
      40: "GND", 41: "GND"},
     (33.0, 22.0), 180, "C2980297",
     "ESP32-S3-WROOM-1U-N8 (LCSC C2980297, JLC 'Standard Only'): HAND-SOLDERED so the board stays Economic. Any WROOM-1U fits.",
     hand=True, fixed=True)
C("C30", "10uF 25V", "3V3", "GND", (47.4, 30.2), 0, "ESP32 bulk")
C("C31", "100nF", "3V3", "GND", (44.3, 30.0), 0)
R("R20", "10k", "3V3", "EN", (47.2, 32.2), 0, "EN pull-up")
C("C32", "1uF", "EN", "GND", (44.3, 32.0), 0, "EN delay")
R("R21", "10k", "3V3", "IO0", (26.5, 9.5), 0, "IO0 pull-up")
part("TP1", "BOOT", TP, {1: "IO0"}, (24.0, 9.0), desc="short to TP2 (GND) at power-up = download mode")
part("TP2", "GND", TP, {1: "GND"}, (21.8, 9.0))
R("R22", "2.2k", "LED", "LED_A", (34.5, 9.0), 0)
part("D6", "red", "LED_SMD:LED_0603_1608Metric", {1: "GND", 2: "LED_A"}, (37.0, 9.0), 0, "C2286", "status LED")

# ---- sonar-side filters AT the ESP32 pins (right column, IO6/IO5/IO4)
R("R11", "1k", "VOUT_I", "VOUT", (47.0, 23.4), 0, "anti-alias 1k + 3.3 nF = 48 kHz (S3 ADC 83 kHz max)")
C("C21", "3.3nF", "VOUT", "GND", (44.3, 24.4), 0, "anti-alias cap at the ADC pin")
R("R10", "10k", "3V3", "NTC_ADC", (47.0, 26.2), 0, "NTC pull-up")
C("C20", "100nF", "NTC_ADC", "GND", (44.3, 26.2), 0)
R("R14", "100k", "VBOOST", "VPWR_ADC", (50.1, 12.0), 270, "VPWR x 10/110")
R("R15", "10k", "VPWR_ADC", "GND", (44.3, 28.0), 0)
C("C23", "100nF", "VPWR_ADC", "GND", (47.0, 28.0), 0)
# ---- left-bottom pins: VBAT (IO1), HALL (IO2)
R("R12", "100k", "VB", "VBAT_ADC", (22.5, 35.0), 90, "VBAT/2")
R("R13", "100k", "VBAT_ADC", "GND", (24.5, 35.0), 90)
C("C22", "100nF", "VBAT_ADC", "GND", (26.5, 35.0), 90)
R("R16", "10k", "3V3", "HALL", (18.5, 35.0), 90, "Hall pull-up (open drain)")
C("C24", "1nF", "HALL", "GND", (20.5, 35.0), 90)

# ======================= USB-C (top edge), native USB ===================================================================
part("J5", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
     {"A1": "GND", "B12": "GND", "A12": "GND", "B1": "GND", "S1": "GND", "A4": "VBUS", "B9": "VBUS", "A9": "VBUS", "B4": "VBUS",
      "A5": "CC1", "B5": "CC2", "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DN", "B7": "USB_DN"},
     (45.0, 3.4), 180, "C165948", "USB-C 16P", fixed=True)
R("R23", "5.1k", "CC1", "GND", (38.5, 5.5), 90, "CC pull-down (5 V sink)")
R("R24", "5.1k", "CC2", "GND", (51.5, 5.5), 90)
part("D4", "B5819W", SOD, {1: "VB", 2: "VBUS"}, (55.0, 4.0), 0, "C8598", "USB OR diode (K=VB)")

# ======================= LoRa Ra-01SH (left), variant "lora" ===========================================================
part("U5", "Ra-01SH", "RF_Module:Ai-Thinker-Ra-01-LoRa",
     {1: "LORA_ANT", 2: "GND", 3: "3V3", 4: "LORA_RST", 5: "LORA_TXEN", 6: "LORA_DIO1", 9: "GND", 10: "LORA_BUSY",
      11: "LORA_RXEN", 12: "LORA_SCK", 13: "LORA_MISO", 14: "LORA_MOSI", 15: "LORA_NSS", 16: "GND"},
     (13.0, 23.5), 0, "C2764087", "SX1262 915 MHz (antenna: module IPEX, or J6 if fitted)", variant="lora", fixed=True)
C("C36", "10uF 25V", "3V3", "GND", (6.0, 12.0), 90)
part("J6", "DNP U.FL", "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical", {1: "LORA_ANT", 2: "GND"}, (2.6, 12.6), 0, "C88374",
     "spare LoRa antenna connector on the ANT pad - fit only if the module has no IPEX", dnp=True, variant="lora")
C("C37", "100nF", "3V3", "GND", (8.0, 12.0), 90)
R("R29", "10k", "3V3", "LORA_NSS", (21.0, 12.5), 90, "NSS pull-up")
for _r in ("C36", "C37", "R29"): P[_r]["variant"] = "lora"

# ======================= TUSS4470 sonar front end (right) ===============================================================
part("U1", "TUSS4470TRTJR", "Package_DFN_QFN:QFN-20-1EP_4x4mm_P0.5mm_EP2.5x2.5mm",
     {1: "", 2: "GND", 3: "NCS", 4: "SCLK", 5: "SDI", 6: "SDO", 7: "TS_IO1", 8: "TS_IO2", 9: "VOUT_I", 10: "3V3", 11: "INN",
      12: "INP", 13: "SGND", 14: "GND", 15: "XA", 16: "XB", 17: "VDRV", 18: "FLT", 19: "OUT4", 20: "VPWR", 21: "GND"},
     (56.0, 16.0), 0, "C1855776", "ultrasonic driver + log-amp receiver", fixed=True)
part("R7", "10R", "Resistor_SMD:R_0805_2012Metric", {1: "VBOOST", 2: "VPWR"}, (51.9, 12.0), 270, "C17415", "RPWR")
C("C11", "100nF", "VPWR", "GND", (53.6, 12.0), 90, "CPWR1 at pin 20")
C("C12", "2.2uF 50V", "VDRV", "GND", (57.9, 11.0), 90, "CPWR2 VDRV storage at pin 17")
C("C17", "3.3nF", "FLT", "SGND", (56.0, 11.0), 90, "CFLT 200 kHz, straight above pin 18")
C("C13", "10nF", "3V3", "GND", (57.8, 19.6), 270, "CVDD at pin 10")
C("C14", "1uF", "3V3", "GND", (58.0, 22.6), 270, "VDD bulk")
R("R8", "200R", "XA", "RINP", (62.0, 15.0), 270, "RINP")
C("C15", "330pF", "RINP", "INP", (61.2, 16.9), 180, "CINP")
C("C16", "22nF", "INN", "SGND", (60.8, 18.6), 0, "CINN 200 kHz")
part("FB1", "600R@100MHz", "Inductor_SMD:L_0805_2012Metric", {1: "SGND", 2: "GND"}, (61.0, 20.6), 0, "C1017", "SGND-GND single point")
part("R9", "DNP 0R", R06, {1: "SGND", 2: "GND"}, (61.0, 22.4), 0, desc="alternative to FB1", dnp=True)
part("D3", "DNP SMF24CA", "Diode_SMD:D_SMF", {1: "XA", 2: "XB"}, (65.6, 11.2), 0, desc="optional TVS across transducer", dnp=True)
part("J2", "XDCR", "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical", {1: "XA", 2: "XB", 3: "SGND", 4: "NTC_ADC", 5: "GND"},
     (69.8, 16.0), 90, "", "transducer A, B, shield; NTC, NTC gnd (pin 1 at the bottom)", hand=True, fixed=True)
part("TP3", "VOUT", TP, {1: "VOUT_I"}, (51.0, 24.2), desc="scope: sonar envelope")
part("TP4", "VBOOST", TP, {1: "VBOOST"}, (64.5, 40.5), desc="scope: 24 V rail")

# ======================= Power (bottom) ==================================================================================
part("J1", "BAT 3xL91", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", {1: "VBAT", 2: "GND"}, (44.0, 51.6), 0, "",
     "battery", hand=True, fixed=True)
part("D5", "B5819W", SOD, {1: "VB", 2: "VBAT"}, (45.2, 45.8), 0, "C8598", "battery OR diode (K=VB)")
C("C1", "10uF 25V", "VB", "GND", (41.0, 46.5), 90, "VB bulk")
# TPS63020 3.3 V buck-boost: U2 pins 1 VINA 2 GND 3 FB 4,5 VOUT 6,7 L2 | 8,9 L1 10,11 VIN 12 EN 13 PS 14 PG, 15 EP
part("U2", "TPS63020DSJR", "SonarPod:Texas_DSJ0014A",
     {1: "VINA", 2: "GND", 3: "FB_T", 4: "3V3", 5: "3V3", 6: "TL2", 7: "TL2", 8: "TL1", 9: "TL1", 10: "VB", 11: "VB", 12: "VB",
      13: "GND", 14: "", 15: "GND"}, (32.0, 43.0), 0, "C15483", "buck-boost 3.3 V; EN=VIN, PS=GND", fixed=True)
part("L2", "1.5uH SWPA4018S1R5NT", IND, {1: "TL1", 2: "TL2"}, (32.0, 48.4), 180, "C80662", "1.5 uH 4x4", fixed=True)
C("C3", "10uF 25V", "VB", "GND", (35.6, 42.8), 90, "VIN at pins 10-12")
C("C4", "10uF 25V", "VB", "GND", (37.6, 42.8), 90)
C("C5", "22uF 25V", "3V3", "GND", (28.4, 43.4), 90, "VOUT at pins 4-5")
C("C6", "22uF 25V", "3V3", "GND", (26.4, 43.4), 90)
C("C7", "22uF 25V", "3V3", "GND", (24.4, 43.4), 90)
R("R2", "100k", "3V3", "FB_T", (28.2, 40.0), 0, "FB top 0.5 x (1+100/18) = 3.28 V")
R("R3", "18k", "FB_T", "GND", (30.0, 38.6), 90, "FB bottom (at the FB pin)")
R("R17", "10R", "VB", "VINA", (34.0, 38.6), 90, "VINA RC")
C("C2", "100nF", "VINA", "GND", (32.2, 38.6), 90)
# MT3608 24 V boost: U3 SOT-23-6 1 SW 2 GND 3 FB 4 EN 5 IN 6 NC
part("U3", "MT3608", "Package_TO_SOT_SMD:SOT-23-6", {1: "SW_B", 2: "GND", 3: "FB_B", 4: "EN_B", 5: "VB", 6: ""}, (57.0, 44.5), 180,
     "C84817", "boost 24 V (rot 180: SW/GND/FB on the right, EN/IN on the left)", fixed=True)
part("L1", "22uH SRP4020TA-220M", IND, {1: "VB", 2: "SW_B"}, (57.0, 48.7), 0, "C1847949", "22 uH 4x4", fixed=True)
part("D1", "B5819W", SOD, {1: "VBOOST", 2: "SW_B"}, (60.8, 46.8), 270, "C8598", "boost diode, A at SW", fixed=True)
C("C8", "10uF 50V", "VBOOST", "GND", (63.3, 46.3), 270, "boost out, next to D1-K / U3-GND")
C("C9", "10uF 50V", "VBOOST", "GND", (65.6, 46.3), 270)
C("C10", "10uF 25V", "VB", "GND", (53.4, 45.4), 90, "boost in")
R("R4", "200k", "VBOOST", "FB_B", (59.5, 40.2), 180, "FB top 0.6 x (1+200/5.1) = 24.1 V")
R("R5", "5.1k", "FB_B", "GND", (59.5, 41.9), 0, "FB bottom (at the FB pin)")
R("R6", "100k", "EN_B", "GND", (54.4, 42.2), 0, "EN pull-DOWN: boost off unless the ESP32 drives it")

# ======================= connectors (hand-soldered) ======================================================================
part("J3", "HALL", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", {1: "3V3", 2: "GND", 3: "HALL"}, (14.0, 51.6), 0,
     "", "US1881 on the flag arm", hand=True, fixed=True)
part("J4", "HEAD", "Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical", {1: "3V3", 2: "GND", 3: "BTN", 4: "LED"}, (8.0, 2.9), 0,
     "", "head: button + LED", hand=True, fixed=True)
part("J7", "EXP", "Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical", {1: "3V3", 2: "GND", 3: "EXP_SDA", 4: "EXP_SCL"},
     (19.5, 2.9), 0, "", "expansion (I2C / 2 GPIO), not fitted", hand=True, fixed=True)
part("TP5", "3V3", TP, {1: "3V3"}, (22.0, 40.0))
part("TP6", "GND", TP, {1: "GND"}, (22.0, 42.2))
for _i, (_x, _y) in enumerate([(2.6, 53.4), (71.4, 53.4), (71.4, 26.0)], 1):
    part(f"H{_i}", "M2", "MountingHole:MountingHole_2.2mm_M2", {}, (_x, _y), fixed=True)

# copper zones (F.Cu SGND island; GND everywhere else on all 4 layers)
# SGND: B.Cu zone under the receive front end (In2 GND plane above it), reached by vias from each SGND pad
SGND_ZONE_B = [(55.2, 8.4), (66.0, 8.4), (66.0, 9.9), (71.6, 9.9), (71.6, 12.1), (68.6, 12.1), (68.6, 23.6), (58.6, 23.6),
               (58.6, 10.7), (55.2, 10.7)]
KEEPOUT_B = [  # no B.Cu tracks: under the receive front end and the two switchers
    [(52.0, 8.2), (71.8, 8.2), (71.8, 24.0), (52.0, 24.0)],
    [(52.0, 39.0), (67.5, 39.0), (67.5, 51.5), (52.0, 51.5)],
    [(23.0, 36.8), (39.5, 36.8), (39.5, 51.0), (23.0, 51.0)],
]

def nets():
    n = {}
    for ref, p in P.items():
        for pin, net in p["pins"].items():
            if net: n.setdefault(net, []).append((ref, pin))
    return n

if __name__ == "__main__":
    n = nets()
    for k in sorted(n, key=lambda k: -len(n[k])): print("%-10s %s" % (k, " ".join(f"{r}.{p}" for r, p in n[k])))
    print("single-pin nets:", [k for k in n if len(n[k]) == 1])
    print("parts", len(P))
