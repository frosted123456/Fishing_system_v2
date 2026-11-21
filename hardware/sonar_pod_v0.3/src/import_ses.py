"""Import a Specctra .ses (freerouting output) into the board with the pcbnew API (KiCad 7 CLI has no importer).
Units: (resolution um 10) -> 1 unit = 0.1 um; y is negated."""
import pcbnew, re, sys
from pcbnew import VECTOR2I

def parse(s):
    """minimal s-expression parser"""
    tokens = re.findall(r'\(|\)|"[^"]*"|[^\s()]+', s)
    def rd(i):
        out = []
        while i < len(tokens):
            t = tokens[i]
            if t == "(":
                sub, i = rd(i + 1); out.append(sub)
            elif t == ")":
                return out, i + 1
            else:
                out.append(t.strip('"')); i += 1
        return out, i
    return rd(0)[0]

def find(tree, key):
    for n in tree:
        if isinstance(n, list) and n and n[0] == key: yield n

def main(board_in, ses, board_out):
    b = pcbnew.LoadBoard(board_in)
    tree = parse(open(ses).read())
    sess = tree[0]
    routes = next(find(sess, "routes"))
    res = next(find(routes, "resolution")); scale = 1000.0 / float(res[2])   # nm per unit: um/10 -> 100 nm
    vias = {}
    for lib in find(routes, "library_out"):
        for ps in find(lib, "padstack"):
            name = ps[1]
            sizes = []
            for sh in find(ps, "shape"):
                c = sh[1]  # (circle layer dia x y)
                if c[0] == "circle": sizes.append(float(c[2]))
            m = re.search(r'_(\d+):(\d+)_um', name)
            dia, drill = (int(m.group(1)), int(m.group(2))) if m else (int(max(sizes) * scale / 1000), 350)
            vias[name] = (dia, drill)
    layers = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu}
    nt, nv = 0, 0
    for net in find(next(find(routes, "network_out")), "net"):
        name = net[1]
        ni = b.GetNetInfo().GetNetItem(name)
        if ni is None: print("unknown net", name); continue
        for w in find(net, "wire"):
            path = next(find(w, "path"))
            layer = layers[path[1]]; width = int(float(path[2]) * scale)
            pts = [float(v) for v in path[3:] if re.match(r'^-?[\d.]+$', v)]
            for i in range(0, len(pts) - 2, 2):
                t = pcbnew.PCB_TRACK(b)
                t.SetStart(VECTOR2I(int(pts[i] * scale), int(-pts[i + 1] * scale)))
                t.SetEnd(VECTOR2I(int(pts[i + 2] * scale), int(-pts[i + 3] * scale)))
                t.SetWidth(width); t.SetLayer(layer); t.SetNet(ni); b.Add(t); nt += 1
        for v in find(net, "via"):
            dia, drill = vias.get(v[1], (700, 350))
            via = pcbnew.PCB_VIA(b)
            via.SetPosition(VECTOR2I(int(float(v[2]) * scale), int(-float(v[3]) * scale)))
            via.SetWidth(int(dia * 1000)); via.SetDrill(int(drill * 1000)); via.SetViaType(pcbnew.VIATYPE_THROUGH)
            via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu); via.SetNet(ni); b.Add(via); nv += 1
    filler = pcbnew.ZONE_FILLER(b); filler.Fill(b.Zones())
    pcbnew.SaveBoard(board_out, b)
    print("tracks", nt, "vias", nv, "->", board_out)

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3])
