import pcbnew, sys, re, collections
b = pcbnew.LoadBoard(sys.argv[1]); out = sys.argv[2]
pcbnew.WriteDRCReport(b, out, pcbnew.EDA_UNITS_MILLIMETRES, True)
s = open(out).read()
cnt = collections.Counter(re.findall(r'^\[(\w+)\]', s, re.M))
print({k: v for k, v in cnt.items()})
m = re.search(r'Found (\d+) unconnected', s); print(m.group(0) if m else "no unconnected line")
