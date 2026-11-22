#!/bin/bash
# project file (HV net class: XA clearance 1.0 mm) next to every board the pipeline loads
D=$(cd "$(dirname "$0")" && pwd)
for f in "$@"; do cp "$D/template.kicad_pro" "${f%.kicad_pcb}.kicad_pro"; cp "$D/template.kicad_dru" "${f%.kicad_pcb}.kicad_dru"; done
