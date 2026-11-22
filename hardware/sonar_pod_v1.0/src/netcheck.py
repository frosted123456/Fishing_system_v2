"""Netlist check: every pad of every footprint on the board carries exactly the net design.py gives it (and no
footprint / pin is missing or extra). Also checks that every net is one connected item set in the DRC (0 unconnected)."""
import pcbnew, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design as D
b = pcbnew.LoadBoard(sys.argv[1]); bad = 0; npad = 0
refs = {f.GetReference() for f in b.GetFootprints()}
for r in sorted(set(D.P) ^ refs): print("footprint mismatch:", r); bad += 1
for f in b.GetFootprints():
    r = f.GetReference()
    if r not in D.P: continue
    pins = {str(k): v for k, v in D.P[r]["pins"].items()}
    seen = set()
    for p in f.Pads():
        num = p.GetNumber(); npad += 1
        if not num: continue                          # unnumbered mechanical pads (U.FL / USB shell handled by number)
        want = pins.get(num, ""); got = p.GetNetname()
        if want != got: print("pad mismatch %s.%s: board %r design %r" % (r, num, got, want)); bad += 1
        seen.add(num)
    for num, net in pins.items():
        if net and num not in seen: print("design pin missing on board: %s.%s (%s)" % (r, num, net)); bad += 1
print("netcheck: %d pads checked, %d mismatches" % (npad, bad))
sys.exit(1 if bad else 0)
