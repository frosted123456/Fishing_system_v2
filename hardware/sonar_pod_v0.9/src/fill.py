import pcbnew, sys
b = pcbnew.LoadBoard(sys.argv[1]); pcbnew.ZONE_FILLER(b).Fill(b.Zones()); pcbnew.SaveBoard(sys.argv[2] if len(sys.argv) > 2 else sys.argv[1], b); print("filled")
