#!/usr/bin/env python3
"""lake_prep.py - D74: build the offline contour "map pack" the chalet stores (Settings -> Map & PIN -> Store on chalet).

The pack is depth contours around a point, as metres east / north of that point, so the chalet web page can draw
them under the holes without internet. Pick the point where the chalet box will sit (its GPS origin) or anywhere
near: the page shifts the pack to the chalet origin when the chalet has a GPS fix.

Sources (pick one):
  --nonna            CHS NONNA-10 bathymetry (10 m grid, open.canada.ca), fetched from the CHS WCS server around
                     the point. Rivers and big lakes surveyed by the Hydrographic Service (Ottawa River is covered
                     in parts). Depth in metres below chart datum. Built from the WCS capabilities, not tested
                     against the live server from this side: if it fails, download the GeoTIFF tile from
                     open.canada.ca ("NONNA-10") and pass --geotiff.
  --geotiff FILE     any depth raster (GeoTIFF; NONNA tile, your own). Depth positive down, metres (or --unit ft).
  --geojson FILE     contour lines already drawn (GBLQ "Guide des bathymetries des lacs du Quebec" isobaths,
                     Donnees Quebec; or any LineString / MultiLineString GeoJSON with a depth property).

Needs: numpy, and for rasters rasterio + matplotlib; for GeoJSON only numpy (and pyproj when the file is not in
lon/lat).   pip install numpy rasterio matplotlib pyproj

Examples:
  python tools/lake_prep.py --lat 45.6214 --lon -75.9811 --radius 1500 --nonna --name "Baie de Quyon" --out quyon.json
  python tools/lake_prep.py --lat 46.80 --lon -71.95 --radius 800 --geojson lac_xyz_isobathes.geojson --out xyz.json
  python tools/lake_prep.py --lat 45.62 --lon -75.98 --radius 1200 --geotiff NONNA10_4550N07600W.tiff --out ottawa.json

ONE PACK FOR THE WHOLE SEASON: run it once per spot with --add on the same --out; each run adds a region (its own
origin) and the page draws the regions near the chalet. A river stretch: --bbox S W N E (degrees) instead of
--radius. Store the file once on the chalet (4.8 MB with the chalet partition table); nothing to redo on the ice.

Pack format (v2): {"v":2,"regions":[{"name":..,"src":..,"lat0":..,"lon0":..,"radius":..,"interval":..,
  "lines":[{"d":<depth m>,"p":[x0,y0,x1,y1,...]}]}]}  p = decimetres east / north of (lat0, lon0), integers.

Examples:
  python tools/lake_prep.py --bbox 45.55 -76.10 45.70 -75.85 --nonna --name "Quyon-Arnprior" --out season.json
  python tools/lake_prep.py --lat 46.80 --lon -71.95 --radius 800 --geojson <file from GBLQ>.geojson --name "Lac X" --add --out season.json
"""
import argparse, json, math, sys, os, urllib.request, urllib.parse, tempfile

M_PER_DEG = 111320.0

def ll_to_m(lat0, lon0, lat, lon):
    c = math.cos(math.radians(lat0))
    return (lon - lon0) * M_PER_DEG * c, (lat - lat0) * M_PER_DEG

def m_to_ll(lat0, lon0, x, y):
    c = math.cos(math.radians(lat0))
    return lat0 + y / M_PER_DEG, lon0 + x / (M_PER_DEG * c)

def merc(lat, lon):   # EPSG:3857
    R = 6378137.0
    return R * math.radians(lon), R * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))

def simplify(pts, tol):
    """Douglas-Peucker on [(x, y), ...] metres."""
    if len(pts) < 3 or tol <= 0:
        return pts
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    imax, dmax = 0, -1.0
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        if L2 == 0:
            d = math.hypot(px - x0, py - y0)
        else:
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
            d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        if d > dmax:
            imax, dmax = i, d
    if dmax > tol:
        return simplify(pts[:imax + 1], tol)[:-1] + simplify(pts[imax:], tol)
    return [pts[0], pts[-1]]

def clip_radius(pts, radius, box=None):
    """Split a line into the runs inside the circle (or the box (xmin, ymin, xmax, ymax) metres)."""
    out, cur = [], []
    for p in pts:
        if (box[0] <= p[0] <= box[2] and box[1] <= p[1] <= box[3]) if box else (math.hypot(p[0], p[1]) <= radius):
            cur.append(p)
        elif cur:
            out.append(cur); cur = []
    if cur:
        out.append(cur)
    return out

# ---------------------------------------------------------------- raster -> contours
def contours_from_raster(path, lat0, lon0, radius, interval, unit_scale, max_depth):
    try:
        import numpy as np, rasterio
        from rasterio.warp import transform
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as e:
        sys.exit(f"missing module: {e}.  pip install numpy rasterio matplotlib")
    with rasterio.open(path) as ds:
        # window around the point: metres -> raster CRS
        lat_s, lon_w = m_to_ll(lat0, lon0, -radius * 1.1, -radius * 1.1)
        lat_n, lon_e = m_to_ll(lat0, lon0, radius * 1.1, radius * 1.1)
        xs, ys = transform("EPSG:4326", ds.crs, [lon_w, lon_e], [lat_s, lat_n])
        win = ds.window(min(xs), min(ys), max(xs), max(ys))
        win = win.round_offsets().round_lengths()
        if win.width < 2 or win.height < 2:
            sys.exit("the point is outside this raster")
        a = ds.read(1, window=win).astype("float64")
        raw_min, raw_max = (np.nanmin(a), np.nanmax(a)) if a.size else (float("nan"), float("nan"))
        nod = ds.nodata
        if nod is not None:
            a[a == nod] = np.nan
        a[~np.isfinite(a)] = np.nan
        a[np.abs(a) > 1e20] = np.nan          # NONNA nil 3.4e38
        valid = int(np.count_nonzero(~np.isnan(a)))
        print(f"raster: {ds.width}x{ds.height} px, {ds.crs}, {ds.dtypes[0]}, nodata {nod}; window {int(win.width)}x{int(win.height)} px, "
              f"raw {raw_min:.3g}..{raw_max:.3g}, {valid} valid pixels ({100.0 * valid / max(1, a.size):.1f} %)")
        a *= unit_scale
        if max_depth:
            a[a > max_depth] = np.nan
        if valid == 0:
            sys.exit("no depth data in that window: this source does not cover the spot (NONNA = CHS surveys only; try --nonna100, "
                     "or GBLQ for a lake, or a smaller/other area). The raster is kept with --keep-tiff if you want to look at it.")
        if np.nanmedian(a) < 0:
            print("NOTE: depths are negative in this raster (elevation convention): flipping the sign")
            a = -a
        tr = ds.window_transform(win)
        rows, cols = a.shape
        # pixel centres -> lon/lat -> metres from the point (one transform call for all)
        cc, rr = np.meshgrid(np.arange(cols) + 0.5, np.arange(rows) + 0.5)
        px, py = tr * (cc.ravel(), rr.ravel())
        lons, lats = transform(ds.crs, "EPSG:4326", list(px), list(py))
        lons, lats = np.array(lons).reshape(rows, cols), np.array(lats).reshape(rows, cols)
        c = math.cos(math.radians(lat0))
        X = (lons - lon0) * M_PER_DEG * c
        Y = (lats - lat0) * M_PER_DEG
        dmin, dmax = np.nanmin(a), np.nanmax(a)
        levels = np.arange(math.floor(dmin / interval) * interval, dmax + interval, interval)
        levels = [l for l in levels if l > 0]
        fig = plt.figure()
        cs = plt.contour(X, Y, np.ma.masked_invalid(a), levels=levels)
        lines = []
        for lev, segs in zip(cs.levels, cs.allsegs):
            for seg in segs:
                lines.append((float(lev), [(float(p[0]), float(p[1])) for p in seg]))
        plt.close(fig)
        return lines, f"raster {os.path.basename(path)} ({rows}x{cols} px, {dmin:.1f}-{dmax:.1f} m)"

# ---------------------------------------------------------------- GeoJSON isobaths
def contours_from_geojson(path, lat0, lon0, depth_field, epsg, unit_scale):
    with open(path, "r", encoding="utf-8") as f:
        gj = json.load(f)
    feats = gj.get("features", [gj] if gj.get("type") != "FeatureCollection" else [])
    proj = None
    if epsg and epsg != 4326:
        try:
            from pyproj import Transformer
        except ImportError:
            sys.exit("pip install pyproj (file is not in lon/lat)")
        proj = Transformer.from_crs(f"EPSG:{epsg}", "EPSG:4326", always_xy=True)
    lines, field = [], depth_field
    for ft in feats:
        props = ft.get("properties") or {}
        if field is None:
            for k in props:
                kl = k.lower()
                if "prof" in kl or "depth" in kl or kl in ("z", "elev", "niveau", "cote"):
                    field = k; break
            if field is None:
                sys.exit(f"no depth property found; keys: {list(props)}. Use --depth-field")
        try:
            d = abs(float(props.get(field))) * unit_scale
        except (TypeError, ValueError):
            continue
        geom = ft.get("geometry") or {}
        coords = geom.get("coordinates", [])
        if geom.get("type") == "LineString":
            parts = [coords]
        elif geom.get("type") == "MultiLineString":
            parts = coords
        elif geom.get("type") == "Polygon":
            parts = coords
        elif geom.get("type") == "MultiPolygon":
            parts = [r for poly in coords for r in poly]
        else:
            continue
        for part in parts:
            pts = []
            for c in part:
                x, y = c[0], c[1]
                if proj is None and (abs(x) > 180 or abs(y) > 90):
                    sys.exit("coordinates are not lon/lat: pass --epsg <code of the file's CRS>")
                lon, lat = proj.transform(x, y) if proj else (x, y)
                pts.append(ll_to_m(lat0, lon0, lat, lon))
            lines.append((d, pts))
    return lines, f"geojson {os.path.basename(path)} (field {field})"

# ---------------------------------------------------------------- NONNA-10 over WCS
NONNA_WCS = "https://nonna-geoserver.data.chs-shc.ca/geoserver/wcs"
def fetch_nonna(lat0, lon0, radius, out_path, coverage="nonna__NONNA 10 Coverage"):
    lat_s, lon_w = m_to_ll(lat0, lon0, -radius * 1.1, -radius * 1.1)
    lat_n, lon_e = m_to_ll(lat0, lon0, radius * 1.1, radius * 1.1)
    x0, y0 = merc(lat_s, lon_w); x1, y1 = merc(lat_n, lon_e)
    q = [("service", "WCS"), ("version", "2.0.1"), ("request", "GetCoverage"), ("coverageId", coverage),
         ("subset", f"X({x0:.1f},{x1:.1f})"), ("subset", f"Y({y0:.1f},{y1:.1f})"), ("format", "image/tiff")]
    url = NONNA_WCS + "?" + urllib.parse.urlencode(q)
    print("NONNA WCS:", url)
    req = urllib.request.Request(url, headers={"User-Agent": "lake_prep.py (ice fishing map pack)"})
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read()
    if data[:4] not in (b"II*\x00", b"MM\x00*"):
        sys.exit("the server did not return a GeoTIFF:\n" + data[:600].decode("utf-8", "replace"))
    with open(out_path, "wb") as f:
        f.write(data)
    print(f"  {len(data) // 1024} KB saved to {out_path}")
    return out_path

def probe_nonna(bbox, radius, lat, lon):
    """Coarse coverage map of NONNA-100 over the box: where CHS has data at all (10 m product has the same footprint)."""
    try:
        import numpy as np, rasterio
    except ImportError as e:
        sys.exit(f"missing module: {e}.  pip install numpy rasterio")
    S, W, N, E = bbox
    tiff = os.path.join(tempfile.gettempdir(), "lake_prep_probe.tiff")
    fetch_nonna(lat, lon, radius, tiff, "nonna__NONNA 100 Coverage")
    with rasterio.open(tiff) as ds:
        a = ds.read(1).astype("float64")
        a[np.abs(a) > 1e20] = np.nan
        if ds.nodata is not None:
            a[a == ds.nodata] = np.nan
        ok = ~np.isnan(a)
        pct = 100.0 * ok.sum() / ok.size
        print(f"NONNA-100 over the box: {pct:.1f} % of the pixels have data" + (" -> nothing here, CHS did not survey it" if ok.sum() == 0 else ""))
        if ok.sum() == 0:
            return
        rows, cols = np.where(ok)
        tr = ds.transform
        from rasterio.warp import transform as wtr
        xs, ys = tr * (np.array([cols.min(), cols.max() + 1]), np.array([rows.max() + 1, rows.min()]))
        lons, lats = wtr(ds.crs, "EPSG:4326", list(xs), list(ys))
        print(f"data footprint: lat {min(lats):.4f}..{max(lats):.4f}, lon {min(lons):.4f}..{max(lons):.4f}, depths {np.nanmin(np.abs(a)):.1f}..{np.nanmax(np.abs(a)):.1f} m")
        # coarse ASCII map, north up, '#' = data in that cell
        H, Wc = 20, 60
        print(f"coverage map (north up, each char ~{(E - W) / Wc * M_PER_DEG * math.cos(math.radians(lat)) / 1000:.1f} km wide):")
        for j in range(H):
            r0, r1 = int(j * ok.shape[0] / H), int((j + 1) * ok.shape[0] / H)
            line = ""
            for i in range(Wc):
                c0, c1 = int(i * ok.shape[1] / Wc), int((i + 1) * ok.shape[1] / Wc)
                line += "#" if ok[r0:max(r1, r0 + 1), c0:max(c1, c0 + 1)].any() else "."
            print("  " + line)
        print(f"  W {W}  ..  E {E}   (top = N {N}, bottom = S {S})")

# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lat", type=float, help="centre latitude (chalet spot), decimal degrees")
    ap.add_argument("--lon", type=float, help="centre longitude (negative west)")
    ap.add_argument("--radius", type=float, default=1000, help="metres around the centre (default 1000)")
    ap.add_argument("--bbox", type=float, nargs=4, metavar=("S", "W", "N", "E"), help="area as lat/lon corners instead of --lat/--lon/--radius (a river stretch)")
    ap.add_argument("--add", action="store_true", help="add this region to an existing --out pack (one pack for the season)")
    ap.add_argument("--probe", action="store_true", help="with --nonna and --bbox: only show WHERE the box has data (coarse map, NONNA-100), no pack")
    ap.add_argument("--name", default="", help="shown on the page")
    ap.add_argument("--out", help="pack file to write (.json)")
    src = ap.add_mutually_exclusive_group(required=False)
    src.add_argument("--nonna", action="store_true", help="fetch CHS NONNA-10 around the point")
    src.add_argument("--geotiff", help="depth raster file")
    src.add_argument("--geojson", help="contour lines file (GBLQ isobaths or any)")
    ap.add_argument("--nonna100", action="store_true", help="with --nonna: the 100 m product (wider coverage, coarser)")
    ap.add_argument("--interval", type=float, default=1.0, help="contour interval, metres (rasters; default 1)")
    ap.add_argument("--max-depth", type=float, default=0, help="ignore deeper than this, metres (0 = all)")
    ap.add_argument("--unit", choices=["m", "ft"], default="m", help="unit of the SOURCE depths (default m)")
    ap.add_argument("--depth-field", help="GeoJSON property holding the depth (auto: *prof*, *depth*)")
    ap.add_argument("--epsg", type=int, default=4326, help="GeoJSON CRS when not lon/lat (GBLQ GPKG exports: often 32198)")
    ap.add_argument("--simplify", type=float, default=1.5, help="line simplification tolerance, metres (default 1.5)")
    ap.add_argument("--keep-tiff", action="store_true", help="with --nonna: keep the downloaded GeoTIFF next to --out")
    a = ap.parse_args()
    unit_scale = 0.3048 if a.unit == "ft" else 1.0
    box = None
    if a.bbox:
        S, W, N, E = a.bbox
        a.lat, a.lon = (S + N) / 2, (W + E) / 2
        x0, y0 = ll_to_m(a.lat, a.lon, S, W); x1, y1 = ll_to_m(a.lat, a.lon, N, E)
        box = (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))
        a.radius = math.hypot(box[2], box[3])   # window / clip radius that contains the box
    elif a.lat is None or a.lon is None:
        sys.exit("--lat and --lon (or --bbox) are required")

    if a.probe:
        if not a.bbox:
            sys.exit("--probe needs --bbox S W N E")
        probe_nonna(a.bbox, a.radius, a.lat, a.lon)
        return
    if not (a.nonna or a.geotiff or a.geojson) or not a.out:
        sys.exit("need a source (--nonna / --geotiff / --geojson) and --out")
    if a.nonna:
        tiff = (os.path.splitext(a.out)[0] + "_nonna.tiff") if a.keep_tiff else os.path.join(tempfile.gettempdir(), "lake_prep_nonna.tiff")
        fetch_nonna(a.lat, a.lon, a.radius, tiff, "nonna__NONNA 100 Coverage" if a.nonna100 else "nonna__NONNA 10 Coverage")
        lines, src_txt = contours_from_raster(tiff, a.lat, a.lon, a.radius, a.interval, unit_scale, a.max_depth)
        src_txt = "CHS NONNA-" + ("100" if a.nonna100 else "10") + " (Open Government Licence - Canada)"
    elif a.geotiff:
        lines, src_txt = contours_from_raster(a.geotiff, a.lat, a.lon, a.radius, a.interval, unit_scale, a.max_depth)
    else:
        lines, src_txt = contours_from_geojson(a.geojson, a.lat, a.lon, a.depth_field, a.epsg, unit_scale)

    # clip to the radius, simplify, pack as decimetre integers
    out, npts = [], 0
    tol = a.simplify
    for _ in range(6):
        out, npts = [], 0
        for d, pts in lines:
            for run in clip_radius(pts, a.radius, box):
                run = simplify(run, tol)
                if len(run) < 2:
                    continue
                flat = []
                for x, y in run:
                    flat += [int(round(x * 10)), int(round(y * 10))]
                out.append({"d": round(d, 2), "p": flat}); npts += len(run)
        if npts * 12 < 4000 * 1024:   # ~12 bytes per point once written; the chalet holds ~4.7 MB in all
            break
        tol *= 2
        print(f"too many points ({npts}): simplifying harder (tolerance {tol} m)")
    if not out:
        sys.exit("no contour line inside the radius")
    out.sort(key=lambda L: L["d"])
    region = {"name": a.name, "src": src_txt, "lat0": round(a.lat, 7), "lon0": round(a.lon, 7), "radius": round(a.radius),
              "interval": a.interval, "lines": out}
    if a.bbox:
        region["bbox"] = [round(v, 7) for v in a.bbox]
    pack = {"v": 2, "regions": []}
    if a.add and os.path.exists(a.out):
        with open(a.out, "r", encoding="utf-8") as f:
            old = json.load(f)
        pack["regions"] = old.get("regions") if old.get("v") == 2 else [old]   # v1 single-region file: kept as region 1
        pack["regions"] = [r for r in pack["regions"] if not (a.name and r.get("name") == a.name)]   # same name = replaced
    pack["regions"].append(region)
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(pack, f, separators=(",", ":"))
    size = os.path.getsize(a.out)
    depths = sorted(set(L["d"] for L in out))
    print(f"region '{a.name}': {len(out)} lines, {npts} points, depths {depths[0]}-{depths[-1]} m ({len(depths)} levels)")
    print(f"{a.out}: {size // 1024} KB, {len(pack['regions'])} region(s): " + ", ".join(r.get("name") or "?" for r in pack["regions"]))
    if size > 4700 * 1024:
        print("WARNING: over 4.7 MB, the chalet will refuse it: larger --interval / --simplify, or fewer regions")
    elif size > 1200 * 1024:
        print("NOTE: over 1.2 MB: needs the chalet partition table (tools/partitions_chalet_8MB.csv, set in platformio.ini env:cabin)")
    print("Next: chalet page -> Settings -> Map & PIN -> pick this file -> Store on chalet.")

if __name__ == "__main__":
    main()
