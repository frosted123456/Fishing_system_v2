import sys, subprocess, cairosvg
pcb, outp = sys.argv[1], sys.argv[2]
for side, layers in (("f", "F.Cu,F.SilkS,Edge.Cuts"), ("b", "B.Cu,Edge.Cuts")):
    svg = outp + "_" + side + ".svg"
    subprocess.run(["kicad-cli", "pcb", "export", "svg", "-o", svg, "--layers", layers, "--page-size-mode", "2", "--exclude-drawing-sheet"] + (["--mirror"] if side == "b" else []) + [pcb], capture_output=True)
    cairosvg.svg2png(url=svg, write_to=outp + "_" + side + ".png", output_width=1500, background_color="white")
print("ok")
