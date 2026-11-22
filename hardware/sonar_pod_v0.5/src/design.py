"""Sonar pod board v0.5 - single source of truth (parts, pins, nets, placement).

v0.5 = v0.4 circuit + three things, on a smaller board (54 x 50 mm vs 74 x 56):
  1. Optional TRANSFORMER DRIVE (all DNP): TUSS4470 pre-driver mode -> 2 x 100 V N-FETs -> center-tapped transformer on a
     6-pin port (P1, 2.54 mm) -> transducer. Direct drive stays the default through two 0R links (R40/R41).
     Receive protection for the transformer mode: series 1206 resistor (R8B) + BAV99 clamp (D7) before CINP.
  2. Optional ADS7042 12-bit 1 MSPS ADC on the sonar envelope (DNP): shares SCLK/SDO with the TUSS4470 (both tri-state
     their SDO when deselected - checked in both datasheets), own CS on IO3. Gets around the S3 ADC 83 kHz limit.
  3. Ra-01SH LoRa moved to the BOTTOM side under the ESP32 module (hand-soldered, like the ESP32 module itself).
Layer stack, ground strategy, filters at the ADC pins, compact switcher loops: unchanged from v0.4 (see its README).
Module: ESP32-S3-WROOM-1U-N8 / N4 / N16 / N8R2 (hand-soldered). NOT the octal-PSRAM R8/R16V variants (IO47/48 used here
are free on those, but the pin budget assumes quad or no PSRAM).
Coordinates: mm, origin top-left, Y down (KiCad).
"""
P = {}
def part(ref, value, fp, pins, pos, rot=0, lcsc="", desc="", dnp=False, hand=False, variant=None, fixed=False, side="F"):
    P[ref] = dict(value=value, fp=fp, pins=pins, pos=pos, rot=rot, side=side, lcsc=lcsc, desc=desc, dnp=dnp,
                  handsolder=hand, variant=variant, fixed=fixed)

BOARD_NAME = "sonar_pod_v05"
BOARD_W, BOARD_H = 54.0, 50.0
R06 = "Resistor_SMD:R_0603_1608Metric"; C06 = "Capacitor_SMD:C_0603_1608Metric"
C08 = "Capacitor_SMD:C_0805_2012Metric"; C12 = "Capacitor_SMD:C_1206_3216Metric"
R08 = "Resistor_SMD:R_0805_2012Metric"; R12 = "Resistor_SMD:R_1206_3216Metric"
SOD = "Diode_SMD:D_SOD-123"; IND = "Inductor_SMD:L_Sunlord_SWPA4020S"; TP = "TestPoint:TestPoint_Pad_1.0x1.0mm"
SOT = "Package_TO_SOT_SMD:SOT-23"
RLC = {"100k": "C25803", "10k": "C25804", "18k": "C25810", "200k": "C25811", "5.1k": "C23186", "10R": "C22859",
       "200R": "C8218", "1k": "C21190", "2.2k": "C4190", "100R": "C22775", "0R": "C21189"}
def R(ref, val, a, b, pos, rot=0, desc="", **kw):
    part(ref, val, R06, {1: a, 2: b}, pos, rot, RLC[val], desc, **kw)
def C(ref, val, a, b, pos, rot=0, desc="", **kw):
    fp, lc = {"100nF": (C06, "C14663"), "10nF": (C06, "C57112"), "1uF": (C06, "C15849"), "330pF": (C06, "C1664"),
              "22nF": (C06, "C21122"), "3.3nF": (C06, "C1613"), "1nF": (C06, "C1588"), "10uF 25V": (C08, "C15850"),
              "22uF 25V": (C08, "C45783"), "10uF 50V": (C12, "C13585"), "2.2uF 50V": (C12, "C50254")}[val]
    v = {"100nF": "100nF 50V", "330pF": "330pF C0G"}.get(val, val)
    part(ref, v, fp, {1: a, 2: b}, pos, rot, lc, desc, **kw)

# ======================= ESP32-S3-WROOM-1U, rot 180 (U.FL at the bottom; pins 1-14 on the RIGHT) =======================
part("U4", "ESP32-S3-WROOM-1U", "RF_Module:ESP32-S3-WROOM-1U",
     {1: "GND", 2: "3V3", 3: "EN",
      4: "VPWR_ADC",  # IO4  ADC1_CH3
      5: "NTC_ADC",   # IO5  ADC1_CH4
      6: "VOUT",      # IO6  ADC1_CH5 sonar envelope
      7: "TS_IO2",    # IO7  TUSS IO2 (burst)
      8: "TS_IO1",    # IO15 TUSS IO1
      9: "SDO",       # IO16 MISO (TUSS + ADS7042)
      10: "SDI",      # IO17 MOSI
      11: "SCLK",     # IO18 (TUSS + ADS7042)
      12: "NCS",      # IO8  TUSS CS
      13: "USB_DN", 14: "USB_DP",
      15: "ADC_CS",   # IO3  ADS7042 CS (strap only matters with a JTAG eFuse)
      16: "OUT4_R",   # IO46 TUSS OUT4 via R30 (input only)
      17: "BTN",      # IO9  RTC: wake
      18: "EXP1",     # IO10 test pad
      19: "EXP2",     # IO11 test pad
      20: "EN_B",     # IO12 boost enable
      21: "LORA_DIO1", 22: "LORA_TXEN", 23: "LORA_RST",   # IO13 / IO14 / IO21
      27: "IO0",
      31: "LORA_NSS", 32: "LORA_MOSI", 33: "LORA_MISO", 34: "LORA_SCK", 35: "LORA_RXEN", 36: "LORA_BUSY",  # IO38-42, IO44
      37: "LED",      # IO43 (U0TXD: blinks during the boot log - harmless)
      38: "HALL",     # IO2  RTC: wake
      39: "VBAT_ADC", # IO1  ADC1_CH0
      40: "GND", 41: "GND"},
     (12.2, 13.0), 180, "C2980297",
     "ESP32-S3-WROOM-1U-N8 (LCSC C2980297, JLC 'Standard Only'): HAND-SOLDERED. N4/N16/N8R2 also fit.", hand=True, fixed=True)
C("C30", "10uF 25V", "3V3", "GND", (26.6, 21.4), 0, "ESP32 bulk")
C("C31", "100nF", "3V3", "GND", (23.8, 20.9), 0)
R("R20", "10k", "3V3", "EN", (26.4, 19.5), 0, "EN pull-up")
C("C32", "1uF", "EN", "GND", (23.8, 19.4), 0, "EN delay")
R("R21", "10k", "3V3", "IO0", (8.0, 25.2), 0, "IO0 pull-up")
part("TP1", "BOOT", TP, {1: "IO0"}, (3.2, 25.0), desc="short to TP2 at power-up = download mode")
part("TP2", "GND", TP, {1: "GND"}, (5.4, 25.0))
R("R22", "2.2k", "LED", "LED_A", (11.0, 25.2), 0)
part("D6", "red", "LED_SMD:LED_0603_1608Metric", {1: "GND", 2: "LED_A"}, (13.6, 25.2), 0, "C2286", "status LED")
part("TP7", "IO10", TP, {1: "EXP1"}, (16.2, 25.0), desc="spare GPIO")
part("TP8", "IO11", TP, {1: "EXP2"}, (18.4, 25.0), desc="spare GPIO")
# filters AT the ESP32 pins
R("R11", "1k", "VOUT_I", "VOUT", (26.3, 14.6), 0, "anti-alias 1k + 3.3 nF")
C("C21", "3.3nF", "VOUT", "GND", (23.8, 15.2), 0)
R("R10", "10k", "3V3", "NTC_ADC", (26.3, 16.6), 0, "NTC pull-up")
C("C20", "100nF", "NTC_ADC", "GND", (23.8, 16.6), 0)
R("R15", "10k", "VPWR_ADC", "GND", (23.8, 18.0), 0)
C("C23", "100nF", "VPWR_ADC", "GND", (26.3, 18.0), 0)
R("R14", "100k", "VBOOST", "VPWR_ADC", (27.6, 11.5), 270, "VPWR x 10/110")
R("R12", "100k", "VB", "VBAT_ADC", (6.0, 28.0), 90, "VBAT/2")
R("R13", "100k", "VBAT_ADC", "GND", (8.0, 28.0), 90)
C("C22", "100nF", "VBAT_ADC", "GND", (10.0, 28.0), 90)
R("R16", "10k", "3V3", "HALL", (12.0, 28.0), 90, "Hall pull-up")
C("C24", "1nF", "HALL", "GND", (14.0, 28.0), 90)
R("R30", "1k", "OUT4", "OUT4_R", (30.6, 12.6), 0, "OUT4 series (IO46 is a strap pin)")

# ======================= USB-C (top edge) ==============================================================================
part("J5", "USB-C", "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
     {"A1": "GND", "B12": "GND", "A12": "GND", "B1": "GND", "S1": "GND", "A4": "VBUS", "B9": "VBUS", "A9": "VBUS", "B4": "VBUS",
      "A5": "CC1", "B5": "CC2", "A6": "USB_DP", "B6": "USB_DP", "A7": "USB_DN", "B7": "USB_DN"},
     (28.2, 3.4), 180, "C165948", "USB-C 16P", fixed=True)
R("R23", "5.1k", "CC1", "GND", (35.2, 5.4), 90, "CC pull-down")
R("R24", "5.1k", "CC2", "GND", (35.2, 8.2), 90)
part("D4", "B5819W", SOD, {1: "VB", 2: "VBUS"}, (36.6, 1.8), 0, "C8598", "USB OR diode (K=VB)")

# ======================= LoRa Ra-01SH on the BOTTOM, under the ESP32 (hand-soldered) ===================================
part("U5", "Ra-01SH", "RF_Module:Ai-Thinker-Ra-01-LoRa",
     {1: "LORA_ANT", 2: "GND", 3: "3V3", 4: "LORA_RST", 5: "LORA_TXEN", 6: "LORA_DIO1", 9: "GND", 10: "LORA_BUSY",
      11: "LORA_RXEN", 12: "LORA_SCK", 13: "LORA_MISO", 14: "LORA_MOSI", 15: "LORA_NSS", 16: "GND"},
     (15.2, 13.0), 0, "C2764087", "SX1262 915 MHz, BOTTOM side, hand-soldered (module IPEX, or J6)", hand=True,
     variant="lora", fixed=True, side="B")
C("C36", "10uF 25V", "3V3", "GND", (7.4, 24.4), 0, "LoRa bulk (bottom)", side="B", hand=True, variant="lora")
C("C37", "100nF", "3V3", "GND", (11.4, 24.4), 0, side="B", hand=True, variant="lora")
R("R29", "10k", "3V3", "LORA_NSS", (14.8, 24.4), 0, "NSS pull-up", side="B", hand=True, variant="lora")
part("J6", "DNP U.FL", "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical", {1: "LORA_ANT", 2: "GND"}, (27.0, 4.6), 0,
     "C88374", "spare LoRa antenna connector (bottom)", dnp=True, variant="lora", side="B")

# ======================= TUSS4470 sonar front end ======================================================================
part("U1", "TUSS4470TRTJR", "Package_DFN_QFN:QFN-20-1EP_4x4mm_P0.5mm_EP2.5x2.5mm",
     {1: "", 2: "GND", 3: "NCS", 4: "SCLK", 5: "SDI", 6: "SDO", 7: "TS_IO1", 8: "TS_IO2", 9: "VOUT_I", 10: "3V3", 11: "INN",
      12: "INP", 13: "SGND", 14: "GND", 15: "OUTA", 16: "OUTB", 17: "VDRV", 18: "FLT", 19: "OUT4", 20: "VPWR", 21: "GND"},
     (33.5, 18.5), 0, "C1855776", "ultrasonic driver + log-amp receiver", fixed=True)
part("R7", "10R", R08, {1: "VBOOST", 2: "VPWR"}, (29.4, 14.5), 270, "C17415", "RPWR")
C("C11", "100nF", "VPWR", "GND", (31.1, 14.5), 90, "CPWR1 at pin 20")
C("C12", "2.2uF 50V", "VDRV", "GND", (35.6, 13.4), 90, "VDRV storage at pin 17")
C("C17", "3.3nF", "FLT", "SGND", (33.5, 13.9), 90, "CFLT 200 kHz")
C("C13", "10nF", "3V3", "GND", (35.3, 22.1), 270, "CVDD at pin 10")
C("C14", "1uF", "3V3", "GND", (35.5, 25.1), 270, "VDD bulk")
# direct drive (default): OUTA/OUTB -> transducer through 0R links. Transformer mode: remove R40/R41, fit the DNP parts.
R("R40", "0R", "OUTA", "XA", (38.2, 16.4), 0, "direct drive link A (remove for transformer mode)")
R("R41", "0R", "OUTB", "XB", (37.0, 14.6), 0, "direct drive link B (remove for transformer mode)")
# receive path: XA -> R8B (1206, 0R; 1k-2.2k in transformer mode) -> R8 200R -> C15 330 pF -> INP; D7 clamp (transformer mode)
part("R8B", "0R", R12, {1: "XA", 2: "RIN1"}, (41.8, 18.6), 90, "C17888", "RX series: 0R direct; 1k-2.2k 1206 with the transformer")
R("R8", "200R", "RIN1", "RINP", (39.6, 18.0), 270, "RINP")
C("C15", "330pF", "RINP", "INP", (38.7, 19.6), 180, "CINP")
part("D7", "DNP BAV99", SOT, {1: "GND", 2: "GND", 3: "RINP"}, (41.6, 22.0), 0, "C2500",
     "RX clamp +-0.7 V (fit with the transformer)", dnp=True)
C("C16", "22nF", "INN", "SGND", (38.3, 21.3), 0, "CINN 200 kHz")
part("FB1", "600R@100MHz", "Inductor_SMD:L_0805_2012Metric", {1: "SGND", 2: "GND"}, (38.6, 23.4), 0, "C1017", "SGND-GND")
part("R9", "DNP 0R", R06, {1: "SGND", 2: "GND"}, (38.6, 25.2), 0, desc="alternative to FB1", dnp=True)
part("D3", "DNP SMF24CA", "Diode_SMD:D_SMF", {1: "XA", 2: "XB"}, (44.6, 24.8), 90, desc="optional TVS (direct drive only)", dnp=True)
part("C43", "DNP tune", C12, {1: "XA", 2: "XB"}, (44.6, 19.0), 90, desc="tuning cap across the transducer (C0G, >=500 V in transformer mode)", dnp=True)
part("L3", "DNP 420uH", IND, {1: "XA", 2: "XB"}, (45.2, 29.8), 0, desc="shunt tuning inductor ~420 uH for 1500 pF @ 200 kHz (direct drive)", dnp=True)
part("J2", "XDCR", "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical", {1: "XA", 2: "XB", 3: "SGND", 4: "NTC_ADC", 5: "GND"},
     (49.8, 26.0), 90, "", "transducer A, B, shield; NTC, NTC gnd (pin 1 at the bottom)", hand=True, fixed=True)
part("TP3", "VOUT", TP, {1: "VOUT_I"}, (28.6, 26.6), desc="scope: sonar envelope")

# ---- transformer drive (all DNP) ----
part("Q1", "DNP AP1002", SOT, {1: "GA", 2: "GND", 3: "DA"}, (41.2, 3.0), 0, "C2995324", "100 V 2 A N-FET (G=1 S=2 D=3)", dnp=True)
part("Q2", "DNP AP1002", SOT, {1: "GB", 2: "GND", 3: "DB"}, (41.2, 6.8), 0, "C2995324", "100 V 2 A N-FET", dnp=True)
part("R42", "DNP 10R", R06, {1: "OUTA", 2: "GA"}, (38.6, 3.4), 0, desc="gate R A", dnp=True)
part("R43", "DNP 10R", R06, {1: "OUTB", 2: "GB"}, (38.6, 6.4), 0, desc="gate R B", dnp=True)
part("R44", "DNP 10k", R06, {1: "GA", 2: "GND"}, (39.0, 1.6), 0, desc="gate pull-down A", dnp=True)
part("R45", "DNP 10k", R06, {1: "GB", 2: "GND"}, (38.6, 8.2), 0, desc="gate pull-down B", dnp=True)
part("P1", "XFMR PORT", "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical",
     {1: "DA", 2: "DB", 3: "VTX", 4: "GND", 5: "XA", 6: "XB"}, (45.2, 2.2), 0, "",
     "transformer port: 1 drain A, 2 drain B, 3 center tap, 4 GND, 5/6 secondary (TDK B78416A2386A003 1:1:9 or toroid)",
     hand=True, fixed=True)
part("R46", "DNP 0R", R08, {1: "VBOOST", 2: "VTX"}, (52.0, 10.6), 90, desc="center tap from 24 V boost", dnp=True)
part("R47", "DNP 0R", R08, {1: "VB", 2: "VTX"}, (49.4, 10.8), 90, desc="center tap from battery (lower power)", dnp=True)
part("R48", "DNP 0R", R06, {1: "XB", 2: "GND"}, (47.0, 15.4), 0, desc="ground the secondary low side (transformer mode)", dnp=True)
part("C44", "DNP 10uF 50V", C12, {1: "VTX", 2: "GND"}, (52.2, 3.6), 90, desc="center-tap burst storage", dnp=True)
part("C45", "DNP 10uF 50V", C12, {1: "VTX", 2: "GND"}, (52.2, 7.0), 90, desc="center-tap burst storage", dnp=True)
part("R49", "DNP 1k", R08, {1: "VBOOST", 2: "VDRV"}, (31.6, 10.4), 0, desc="external VDRV feed (VDRV_HI_Z=1 pre-driver mode)", dnp=True)
part("D8", "DNP 12V zener", "Diode_SMD:D_SOD-323", {1: "VDRV", 2: "GND"}, (36.8, 11.0), 0, desc="VDRV clamp 12 V (BZT52C12 class, K=VDRV)", dnp=True)

# ---- ADS7042 12-bit 1 MSPS ADC on the envelope (DNP) ----
part("U8", "DNP ADS7042", "Package_SO:VSSOP-8_2.3x2mm_P0.5mm",
     {1: "3V3", 2: "SCLK", 3: "SDO", 4: "ADC_CS", 5: "GND", 6: "AINP", 7: "AVDD", 8: "GND"}, (31.4, 23.6), 0, "C701641",
     "ADS7042IDCUR 12-bit 1 MSPS, ref = AVDD", dnp=True)
part("R31", "DNP 100R", R06, {1: "VOUT_I", 2: "AINP"}, (31.0, 26.0), 0, desc="ADC input R", dnp=True)
part("C40", "DNP 1nF", C06, {1: "AINP", 2: "GND"}, (33.6, 26.4), 90, desc="ADC charge bucket", dnp=True)
part("R32", "DNP 10R", R06, {1: "3V3", 2: "AVDD"}, (28.8, 22.6), 90, desc="AVDD (= reference) RC", dnp=True)
part("C41", "DNP 1uF", C06, {1: "AVDD", 2: "GND"}, (28.6, 24.8), 0, desc="AVDD reference cap", dnp=True)
part("C42", "DNP 100nF", C06, {1: "3V3", 2: "GND"}, (33.4, 24.4), 0, desc="DVDD bypass", dnp=True)

# ======================= Power (bottom) ================================================================================
part("J1", "BAT 3xL91", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", {1: "VBAT", 2: "GND"}, (30.0, 46.0), 0, "",
     "battery", hand=True, fixed=True)
part("D5", "B5819W", SOD, {1: "VB", 2: "VBAT"}, (30.6, 41.4), 0, "C8598", "battery OR diode (K=VB)")
C("C1", "10uF 25V", "VB", "GND", (26.4, 41.2), 90, "VB bulk")
part("U2", "TPS63020DSJR", "SonarPod:Texas_DSJ0014A",
     {1: "VINA", 2: "GND", 3: "FB_T", 4: "3V3", 5: "3V3", 6: "TL2", 7: "TL2", 8: "TL1", 9: "TL1", 10: "VB", 11: "VB", 12: "VB",
      13: "GND", 14: "", 15: "GND"}, (12.0, 34.6), 0, "C15483", "buck-boost 3.3 V", fixed=True)
part("L2", "1.5uH SWPA4018S1R5NT", IND, {1: "TL1", 2: "TL2"}, (12.0, 40.0), 180, "C80662", "1.5 uH 4x4", fixed=True)
C("C3", "10uF 25V", "VB", "GND", (15.6, 34.4), 90, "VIN at pins 10-12")
C("C4", "10uF 25V", "VB", "GND", (17.6, 34.4), 90)
C("C5", "22uF 25V", "3V3", "GND", (8.4, 35.0), 90, "VOUT at pins 4-5")
C("C6", "22uF 25V", "3V3", "GND", (6.4, 35.0), 90)
C("C7", "22uF 25V", "3V3", "GND", (4.4, 35.0), 90)
R("R2", "100k", "3V3", "FB_T", (8.2, 31.6), 0, "FB top")
R("R3", "18k", "FB_T", "GND", (10.0, 30.2), 90, "FB bottom")
R("R17", "10R", "VB", "VINA", (14.0, 30.2), 90, "VINA RC")
C("C2", "100nF", "VINA", "GND", (12.2, 30.2), 90)
part("U3", "MT3608", "Package_TO_SOT_SMD:SOT-23-6", {1: "SW_B", 2: "GND", 3: "FB_B", 4: "EN_B", 5: "VB", 6: ""}, (40.0, 38.0), 180,
     "C84817", "boost 24 V", fixed=True)
part("L1", "22uH SRP4020TA-220M", IND, {1: "VB", 2: "SW_B"}, (40.0, 42.2), 0, "C1847949", "22 uH 4x4", fixed=True)
part("D1", "B5819W", SOD, {1: "VBOOST", 2: "SW_B"}, (43.8, 40.3), 270, "C8598", "boost diode", fixed=True)
C("C8", "10uF 50V", "VBOOST", "GND", (46.3, 39.8), 270, "boost out")
C("C9", "10uF 50V", "VBOOST", "GND", (48.6, 39.8), 270)
C("C10", "10uF 25V", "VB", "GND", (36.4, 38.9), 90, "boost in")
R("R4", "200k", "VBOOST", "FB_B", (42.5, 33.7), 180, "FB top 24.1 V")
R("R5", "5.1k", "FB_B", "GND", (42.5, 35.4), 0, "FB bottom")
R("R6", "100k", "EN_B", "GND", (37.4, 35.7), 0, "EN pull-down")
part("TP4", "VBOOST", TP, {1: "VBOOST"}, (47.6, 35.6), desc="scope: 24 V")
part("TP5", "3V3", TP, {1: "3V3"}, (2.6, 31.0))
part("TP6", "GND", TP, {1: "GND"}, (2.6, 33.2))

# ======================= connectors (hand-soldered) ======================================================================
part("J3", "HALL", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", {1: "3V3", 2: "GND", 3: "HALL"}, (17.0, 46.0), 0,
     "", "Hall on the flag arm", hand=True, fixed=True)
part("J4", "HEAD", "Connector_JST:JST_PH_B4B-PH-K_1x04_P2.00mm_Vertical", {1: "3V3", 2: "GND", 3: "BTN", 4: "LED"}, (4.6, 46.6), 0,
     "", "head: button + LED", hand=True, fixed=True)
for _i, (_x, _y) in enumerate([(51.4, 47.4), (51.4, 32.0), (2.6, 41.6)], 1):
    part(f"H{_i}", "M2", "MountingHole:MountingHole_2.2mm_M2", {}, (_x, _y), fixed=True)

# SGND: B.Cu zone under the receiver (In2 GND above), joined by vias from each SGND pad
SGND_ZONE_B = [(32.4, 11.4), (37.2, 11.4), (37.2, 15.2), (48.2, 15.2), (48.2, 18.9), (51.6, 18.9), (51.6, 21.1), (48.2, 21.1),
               (48.2, 27.0), (36.6, 27.0), (36.6, 13.4), (32.4, 13.4)]
KEEPOUT_B = [  # no B.Cu tracks under the receiver and the two switchers
    [(29.0, 11.2), (48.4, 11.2), (48.4, 27.4), (29.0, 27.4)],
    [(34.4, 32.0), (50.6, 32.0), (50.6, 45.0), (34.4, 45.0)],
    [(3.0, 29.2), (19.4, 29.2), (19.4, 42.6), (3.0, 42.6)],
]

# ---------------- v0.5 round 2: 58 mm wide (TUSS section +4 mm, filters +1.5 mm) to relieve the receiver area -------
BOARD_W = 58.0
for _r in ("D8", "R49"): del P[_r]      # external-VDRV option dropped: test pre-driver with the internal VDRV regulator first
def _sx(x): return x + 4.0 if x > 29.0 else (x + 1.5 if x > 22.5 else x)
for _r, _p in P.items():
    if _r == "J5": _p["pos"] = (_p["pos"][0] + 2.0, _p["pos"][1]); continue
    _p["pos"] = (_sx(_p["pos"][0]), _p["pos"][1])
P["R30"]["pos"] = (26.4, 9.8)
# TUSS section back up 3 mm (the extra width gives it room under the USB-C): the ESP->TUSS bus runs straight across
# above the filter column instead of through it
for _r in ("U1", "R7", "C11", "C12", "C17", "C13", "C14", "R40", "R41", "R8B", "R8", "C15", "D7", "C16", "FB1", "R9", "D3",
           "C43", "L3", "J2", "TP3", "U8", "R31", "C40", "R32", "C41", "C42", "R30", "R48"):
    P[_r]["pos"] = (P[_r]["pos"][0], P[_r]["pos"][1] - 3.0)
SGND_ZONE_B = [(x, y - 3.0) for x, y in SGND_ZONE_B]
KEEPOUT_B[0] = [(x, y - 3.0) for x, y in KEEPOUT_B[0]]
P["H2"]["pos"] = (55.4, 29.8)
# ADS7042 group out of the TUSS fan-out area, into the free space below the ESP->TUSS bus
for _r, _xy in {"U8": (28.6, 28.2), "R31": (31.8, 27.4), "C40": (31.8, 29.2), "R32": (25.4, 27.2), "C41": (25.4, 29.0),
                "C42": (28.6, 30.6)}.items():
    P[_r]["pos"] = _xy; P[_r]["rot"] = 0
P["J6"]["pos"] = (31.0, 11.0)
# keep the ESP->TUSS bus corridor (x 22-36, y 7-16) free: filter column moves below it, still next to the ADC pins
for _r, _xy in {"C21": (24.0, 17.0), "R11": (27.0, 17.0), "C20": (24.0, 18.6), "R10": (27.0, 18.6),
                "C23": (24.0, 20.2), "R15": (27.0, 20.2), "R14": (30.0, 20.2), "C32": (24.0, 21.8), "R20": (27.0, 21.8),
                "C31": (24.0, 23.4), "C30": (27.2, 23.4), "TP3": (30.2, 17.4)}.items():
    P[_r]["pos"] = _xy; P[_r]["rot"] = 0
P["R30"]["pos"] = (23.6, 1.8); P["R30"]["rot"] = 90
P["C17"]["pos"] = (37.5, 10.9); P["C17"]["fixed"] = True        # CFLT straight above pin 18
del P["C45"]          # one burst-storage cap on the center tap is enough for a test fit
SGND_ZONE_B = [(_sx(x), y) for x, y in SGND_ZONE_B]
KEEPOUT_B = [[(_sx(x), y) for x, y in k] for k in KEEPOUT_B]

# ---------------- v0.5 round 3: TUSS4470 rotated 270 so drive (OUTA/OUTB) and receive (INN/INP/SGND) both exit at the
# bottom/right, digital at the top/left facing the ESP32; transformer stage below J2; simplified option set ----------
for _r in ("D3", "R9", "R47", "R48"): del P[_r]   # TVS, FB1 alternative, battery center-tap, XB-GND link (= bridge P1 pins 4-6)
P["U1"]["pos"] = (37.5, 15.0); P["U1"]["rot"] = 270
_new = {  # ref: ((x, y), rot)
    "C11": ((40.6, 14.0), 90), "R7": ((42.4, 13.4), 270), "C17": ((43.0, 16.0), 0), "C12": ((45.0, 18.0), 0),
    "R41": ((41.2, 19.7), 0), "R40": ((39.4, 22.0), 270),
    "C16": ((35.6, 22.0), 270), "C15": ((37.0, 22.2), 90), "R8": ((37.0, 24.6), 90), "R8B": ((39.4, 26.0), 180),
    "D7": ((34.6, 24.6), 0), "FB1": ((42.6, 22.6), 0), "C13": ((33.8, 21.4), 90), "C14": ((32.2, 21.4), 90),
    "TP3": ((30.2, 20.4), 0),
    "J2": ((53.8, 21.0), 90), "C43": ((50.2, 19.6), 90), "L3": ((47.6, 22.6), 0),
    "Q2": ((41.0, 29.4), 0), "Q1": ((41.0, 32.4), 0), "R43": ((38.4, 29.0), 0), "R42": ((38.4, 32.0), 0),
    "R45": ((38.4, 30.5), 0), "R44": ((38.4, 33.5), 0),
    "P1": ((44.5, 32.3), 90), "C44": ((50.4, 36.4), 0), "R46": ((54.6, 36.4), 90),
    "R30": ((23.6, 1.8), 90),
}
for _r, (_xy, _rot) in _new.items():
    P[_r]["pos"] = _xy; P[_r]["rot"] = _rot
P["J2"]["fixed"] = True; P["P1"]["fixed"] = True
# power section and the bottom connectors down 3 mm to make room for the transformer stage; board 58 x 53
BOARD_H = 53.0
for _r in ("U3", "L1", "D1", "C8", "C9", "C10", "R4", "R5", "R6", "TP4", "J1", "J3", "J4", "D5", "C1", "U2", "L2", "C3", "C4",
           "C5", "C6", "C7", "R2", "R3", "R17", "C2", "TP5", "TP6", "H1", "H3"):
    P[_r]["pos"] = (P[_r]["pos"][0], P[_r]["pos"][1] + 3.0)
P["H2"]["pos"] = (55.4, 27.6)
SGND_ZONE_B = None                      # SGND is now a routed net (C16, C17, U1.13, FB1, J2.3): no B-side island
KEEPOUT_B = [
    [(33.0, 19.6), (40.0, 19.6), (40.0, 27.6), (33.0, 27.6)],              # under the receive network only
    [(KEEPOUT_B[1][0][0], 35.0), (KEEPOUT_B[1][1][0], 35.0), (KEEPOUT_B[1][2][0], 48.0), (KEEPOUT_B[1][3][0], 48.0)],
    [(3.0, 32.2), (19.4, 32.2), (19.4, 45.6), (3.0, 45.6)],
]

# ---------------- v0.5 round 4: hand placement of the TUSS4470 neighbourhood (computed pin-by-pin, no crossings) --------
del P["R30"]; P["U1"]["pins"][19] = ""; P["U4"]["pins"][16] = ""     # OUT4 comparator output dropped (firmware: O4 = -1)
_fix = {
    "C16": ((33.6, 19.4), 270), "C15": ((36.6, 19.4), 180), "R8": ((39.0, 21.8), 90), "R8B": ((41.8, 24.5), 180),
    "D7": ((36.0, 22.6), 0), "R40": ((40.9, 18.4), 0), "R41": ((41.7, 16.4), 0), "C12": ((44.6, 14.4), 0),
    "C17": ((42.8, 12.4), 0), "C11": ((40.4, 10.8), 90), "R7": ((38.6, 10.0), 270), "FB1": ((45.4, 18.2), 0),
    "C43": ((49.4, 19.8), 90), "L3": ((47.0, 26.4), 0), "P1": ((45.0, 33.0), 90),
    "Q2": ((40.9, 29.8), 0), "Q1": ((40.9, 33.4), 0), "R43": ((37.0, 29.4), 0), "R45": ((37.0, 30.9), 0),
    "R42": ((37.0, 32.4), 0), "R44": ((37.0, 33.9), 0), "C44": ((50.6, 36.6), 0), "R46": ((53.6, 33.4), 90),
    "U8": ((26.9, 31.2), 0), "R31": ((30.8, 30.4), 0), "C40": ((30.8, 32.0), 0), "R32": ((22.8, 30.4), 0),
    "C41": ((22.8, 32.0), 0), "C42": ((26.9, 33.8), 0),
}
for _r, (_xy, _rot) in _fix.items():
    P[_r]["pos"] = _xy; P[_r]["rot"] = _rot; P[_r]["fixed"] = True
for _r, (_xy, _rot) in {"C15": ((36.1, 19.4), 180), "R8": ((36.9, 21.8), 90), "R8B": ((40.5, 23.8), 180),
                        "D7": ((34.2, 23.9), 0), "FB1": ((39.6, 20.4), 0), "J6": ((22.0, 2.2), 0)}.items():
    P[_r]["pos"] = _xy; P[_r]["rot"] = _rot; P[_r]["fixed"] = True
KEEPOUT_B = KEEPOUT_B[1:]
P["C13"]["pos"] = (33.4, 16.6); P["C13"]["rot"] = 180; P["C13"]["fixed"] = True     # CVDD right at pin 10
P["C14"]["pos"] = (30.9, 18.9); P["C14"]["rot"] = 90; P["C14"]["fixed"] = True
P["H3"]["pos"] = (55.4, 2.6); P["TP5"]["pos"] = (21.0, 37.0); P["TP6"]["pos"] = (23.2, 37.0)   # free the left-edge channel   # receive area: F traces sit over In1 GND and B traces under In2 GND -> no B keepout needed there
del P["J6"]; P["U5"]["pins"][1] = ""                       # LoRa antenna: the module's IPEX (LCSC: "IPEX interface")
for _r, (_xy, _rot) in {"D4": ((40.4, 1.8), 180), "R23": ((44.0, 5.0), 0), "R24": ((44.0, 6.6), 0)}.items():
    P[_r]["pos"] = _xy; P[_r]["rot"] = _rot; P[_r]["fixed"] = True
for _r, _xy in {"C21": (25.1, 17.0), "R11": (28.1, 17.0), "C20": (25.1, 19.2), "R10": (28.1, 19.2), "C23": (25.1, 21.4),
                "R15": (28.1, 21.4), "R14": (30.4, 22.0), "C32": (25.1, 23.6), "R20": (28.1, 23.6), "C31": (25.1, 25.8),
                "C30": (28.3, 25.8), "TP3": (31.6, 27.4)}.items():
    P[_r]["pos"] = _xy; P[_r]["rot"] = 90 if _r == "R14" else 0; P[_r]["fixed"] = True

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
