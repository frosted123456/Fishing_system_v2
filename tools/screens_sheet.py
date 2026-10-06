#!/usr/bin/env python3
"""PBM screens (tools/screens_mock.cpp) -> one PNG sheet, OLED look (white on black, 4x)."""
import glob, os, sys
from PIL import Image, ImageDraw
d = sys.argv[1]; out = sys.argv[2]
files = sorted(glob.glob(os.path.join(d, "*.pbm")))
S, PAD, LAB = 4, 16, 22
cols = 2
rows = (len(files) + cols - 1) // cols
W, H = 128 * S, 64 * S
sheet = Image.new("RGB", (cols * (W + PAD) + PAD, rows * (H + PAD + LAB) + PAD), (235, 238, 240))
dr = ImageDraw.Draw(sheet)
for i, f in enumerate(files):
    lines = open(f).read().split("\n")[2:66]
    img = Image.new("RGB", (128, 64), (8, 10, 14))
    px = img.load()
    for y, row in enumerate(lines):
        for x, c in enumerate(row):
            if c == "1": px[x, y] = (235, 245, 255)
    img = img.resize((W, H), Image.NEAREST)
    x = PAD + (i % cols) * (W + PAD); y = PAD + (i // cols) * (H + PAD + LAB)
    dr.text((x, y), os.path.basename(f)[3:-4].replace("_", " "), fill=(40, 40, 40))
    sheet.paste(img, (x, y + LAB))
sheet.save(out)
print(out, sheet.size)
