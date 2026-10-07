#!/usr/bin/env python3
"""Desktop preview of the chalet web suite (Holes / Sonar / Radio / Settings) with fake data.

    mkdir -p .preview
    g++ -std=c++11 -O2 -Ilib/IceMesh/src tools/sonar_trace.cpp -o .preview/sonar_trace
    .preview/sonar_trace 300 > .preview/trace.json
    python3 tools/web_preview.py            # then open http://localhost:8000/

The page is read from web/suite.html; set SUITE_PAGE=file.html to try another file.
Sonar data is the trace recorded with the real codec and store (replayed in real time, looped).
Status, radio and settings are invented: node 3 trips for 20 s every minute to show the alert.
"""
import json, os, sys, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRACE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, ".preview", "trace.json")
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8000

def page_html():
    if os.environ.get("SUITE_PAGE"):
        return open(os.environ["SUITE_PAGE"], encoding="utf-8").read()
    return open(os.path.join(ROOT, "web", "suite.html"), encoding="utf-8").read()

frames = json.load(open(TRACE))
t0 = time.time()
state = {"focus": frames[0]["list"]["focus"], "sim": True, "silenced": False, "sil_until": 0,
         "names": {3: "Pointe", 4: "Baie"}, "test": 0, "adaptive": False,
         "simreq": {}, "simrate": 6,
         "transport": 0, "ch_setting": "auto", "ch": 0, "moves": 0, "relay_req": {}, "lr_only": False,
         "settings": {"buzzerEnabled": True, "alertHoldSec": 30, "heartbeatSec": 60, "reedActiveHigh": True,
                      "alarmHoldMin": 0, "nearBaitBeep": False, "unitsMetric": False, "buzzerPassive": False},
         "bait": {3: 457}, "knobs": None, "trips": []}
# the same knob table as lib/IceMesh/src/sonar_params.h (key, name, lo, hi, def, unit)
KNOBS = [("snr", "Detection threshold", 4, 30, 10, "dB"), ("prom", "Peak contrast", 1, 15, 5, "dB"),
         ("confirm", "Confirm pings", 1, 10, 3, ""), ("keep", "Keep without echo", 1, 20, 5, "pings"),
         ("gate", "Max target move", 5, 100, 32, "cm/ping"), ("dead", "Dead zone", 0, 50, 9, "x0.1 m"),
         ("bsmooth", "Bottom smoothing", 5, 100, 25, "%"), ("learn", "Static scene learning", 1, 100, 12, "x0.001"),
         ("range", "Colour range", 20, 80, 46, "dB"), ("tvg", "Range compensation", 0, 1, 1, ""),
         ("cycles", "Burst cycles, base", 1, 32, 16, "cycles"), ("gain", "Receiver gain", 0, 3, 2, "10/12.5/15/20 V/V"),
         ("pinghz", "Ping rate, active", 1, 16, 16, "x0.25/s"), ("fcycles", "Burst cycles, focus", 1, 32, 8, "cycles"),
         ("idlehz", "Ping rate, idle", 1, 16, 4, "x0.25/s"), ("avg", "Bursts averaged", 1, 4, 2, "per ping"),
         ("freq", "Frequency mode", 0, 2, 0, "0 3/ping 1 rot 2 200k"), ("sound", "Sound speed", 0, 255, 53, "+1350 m/s"),
         ("bpf", "Band-pass code (200k)", 1, 62, 29, "0x1D=196.8kHz"), ("thresh", "Edge threshold", 0, 15, 7, "reg 0x17"),
         ("bmin", "Bottom search from", 1, 50, 6, "x0.1 m"), ("vdrv", "Drive voltage", 5, 20, 11, "V (MT3608 - 1)")]
state["knobs"] = {k[0]: k[4] for k in KNOBS}
def knobs_json():
    v = state["knobs"]
    preset = {(7, 4, 2, 6): 0, (10, 5, 3, 5): 1, (14, 7, 5, 4): 2}.get((v["snr"], v["prom"], v["confirm"], v["keep"]), 3)
    return {"preset": preset, "knobs": [{"key": k, "name": nm, "lo": lo, "hi": hi, "def": d, "unit": u, "v": v[k]} for k, nm, lo, hi, d, u in KNOBS]}
TESTS = ["off", "rotate", "SF9/500", "SF8/500", "SF7/500"]

def now_index():
    return int(time.time() - t0) % len(frames)

import math, random
_glance = {}
def glance(since):
    """Fake per-hole sonar summaries (every 2 s, ~3 min kept), shaped like the firmware's /api/sonar/glance."""
    el = time.time() - t0; frame = int(el) + 1000
    out = []
    ids = [n["node"] for n in frames[0]["list"]["nodes"]] + [2, 3, 4]
    for i, nid in enumerate(ids):
        g = _glance.setdefault(nid, {"recs": [], "last": 0, "bottom": 300 + 90 * i, "fish": []})
        while g["last"] < frame:
            g["last"] = (g["last"] or frame - 180) + 2
            f = g["last"]; g["bottom"] = max(150, min(1100, g["bottom"] + random.randint(-4, 4)))
            g["fish"] = [d + random.randint(-15, 15) for d in g["fish"] if random.random() > 0.05 and 50 < d < g["bottom"] - 20]
            if random.random() < 0.08 * (1 + i % 3): g["fish"].append(random.randint(80, g["bottom"] - 40))
            bait = int(g["bottom"] * 0.6)
            t = [bait | (2 << 11) | (1 << 13)] + [min(d, 2046) | (random.choice([1, 2, 2, 3]) << 11) for d in g["fish"][:4]]
            g["recs"] = (g["recs"] + [[f, g["bottom"]] + t])[-90:]
        out.append({"node": nid, "hub": 1 + i // 3, "hard": 1 + i % 3, "act": len(g["fish"]), "recs": [r for r in g["recs"] if r[0] > since]})
    return {"frame": frame, "nodes": out}

def status():
    el = time.time() - t0
    fish3 = 20 <= (el % 60) < 40
    nodes = [{"id": 2, "role": "Sensor", "online": True, "fish": False, "lowbat": False, "battery": 4300, "via_espnow": True, "last_seen_sec": 12},
             {"id": 3, "role": "Sensor", "online": True, "fish": fish3, "lowbat": False, "battery": 3900, "via_lora": True, "last_seen_sec": 1},
             {"id": 4, "role": "Sensor", "online": True, "fish": False, "lowbat": True, "battery": 3350, "via_lora": True, "last_seen_sec": 40},
             {"id": 5, "role": "Sensor", "online": False, "fish": False, "lowbat": False, "battery": 0, "via_lora": True, "last_seen_sec": 900}]
    for n in frames[0]["list"]["nodes"]:
        nodes.append({"id": n["node"], "role": "Sensor", "online": True, "fish": False, "lowbat": False, "battery": 4500, "via_lora": True, "last_seen_sec": 1})
    for n in nodes:
        n["name"] = state["names"].get(n["id"], "")
        n["line"] = n["fish"]; n["bait"] = state["bait"].get(n["id"], 0)
        n["sim"] = state["simreq"].get(n["id"], 0) != 0 or n["id"] >= 128
    if state["silenced"] and time.time() > state["sil_until"]:
        state["silenced"] = False
    return {"network_id": 66, "node_count": len(nodes), "uptime": int(el) + 7380, "role": "Chalet", "node_id": 100, "fw": "v2",
            "silenced": state["silenced"], "silence_left_sec": max(0, int(state["sil_until"] - time.time())) if state["silenced"] else 0,
            "sonar_sim": state["sim"], "radio_test": TESTS[state["test"]], "trips": len(state["trips"]),
            "setup_arm_s": max(0, 260 - int(el)), "transport": ["auto", "lora", "espnow"][state["transport"]],
            "lora_ch": state["ch"] + 1, "master": "chalet",
            "wifi": {"ap_active": True, "ap_ip": "192.168.4.1", "sta_connected": False, "sta_ip": ""},
            "lora": {"ready": True, "rx_count": 1200 + int(el) * 3, "tx_count": 400 + int(el)}, "nodes": nodes}

def radio():
    el = int(time.time() - t0)
    hubs = []
    for hid, via, rssi in ((1, 0, -96), (2, 0, -104), (3, 2, -112)):
        sched = 300 + el
        modes = [{"mode": "SF9/500", "sched": sched, "rx": sched - (3 if via == 0 else 22), "crc": 1, "rssi_avg": rssi, "rssi_min": rssi - 6, "snr_avg": 6.5, "snr_min": 2.0}]
        hubs.append({"id": hid, "via": via, "mode": "SF9/500", "modes": modes,
                     "health": {"beacon_rssi": rssi + 2, "beacon_snr": 7.0, "beacon_lost64": 1 if via == 0 else 6, "sync_err_us": 40}})
    mhz = [915.0, 904.0, 907.0, 910.0, 913.0, 918.0, 921.0, 924.0]
    busy = [4, 31, 2, None, 7, 12, 0, 3]
    paths = [{"id": 1, "lora_age": 1, "eb_age": -1, "eb_hops": 0, "relay": True, "eb_path": False},
             {"id": 2, "lora_age": 0, "eb_age": 4, "eb_hops": 1, "relay": False, "eb_path": state["transport"] != 1},
             {"id": 3, "lora_age": 75, "eb_age": 1, "eb_hops": 2, "relay": False, "eb_path": True}]
    return {"role": "chalet", "radio_ok": True, "frame": el, "rx_ok": 2000 + el * 3, "rx_crc": 12, "tx": 300 + el,
            "test_mode": TESTS[state["test"]], "adaptive": state["adaptive"],
            "setup": el < 260, "arm_in_s": max(0, 260 - el),
            "transport": ["auto", "lora", "espnow"][state["transport"]], "channel": state["ch"], "channel_mhz": mhz[state["ch"]],
            "eb": {"lr": True, "relay": False, "rx": 800 + el, "tx": 400 + el, "relayed": 0, "dup": 35},
            "channel_setting": state["ch_setting"], "channel_moves": state["moves"],
            "channels": [{"ch": i, "mhz": mhz[i], "busy": busy[i]} for i in range(8)], "paths": paths,
            "allowance": 96, "dropped_slots": 0, "hubs": hubs,
            "eb_ready": True, "eb_chalet_lr": state["lr_only"],
            "relay_req": [{"dev": k, "on": v} for k, v in state["relay_req"].items()]}

class H(BaseHTTPRequestHandler):
    def send(self, code, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(b)))
        self.end_headers(); self.wfile.write(b)

    def log_message(self, *a):
        pass

    def listing(self):
        d = dict(frames[now_index()]["list"]); d["focus"] = state["focus"]; d["sim"] = state["sim"]
        return json.dumps(d)

    def simj(self):
        hs = [{"id": n["id"], "name": n.get("name", ""), "req": state["simreq"].get(n["id"], 0),
               "on": n["sim"], "virtual": n["id"] >= 128} for n in status()["nodes"]]
        return {"virtual": state["sim"], "rate": state["simrate"], "all": -1, "holes": hs}

    def body(self):
        n = int(self.headers.get("Content-Length", "0") or 0)
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return {}

    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path in ("/", "/sonar", "/radio", "/settings"):
            return self.send(200, page_html(), "text/html; charset=utf-8")
        if u.path == "/api/status":
            return self.send(200, json.dumps(status()))
        if u.path == "/api/sonar":
            return self.send(200, self.listing())
        if u.path == "/api/sonar/pings":
            node = int(q.get("node", ["0"])[0]); since = int(q.get("since", ["0"])[0]); mx = int(q.get("max", ["40"])[0])
            k = now_index(); out = []
            if node == frames[0]["list"]["focus"]:
                for f in frames[: k + 1]:
                    out += [p for p in f["pings"] if p[0] > since]
                if k < 2: out = list(frames[k]["pings"])
            out = out[-mx:]
            return self.send(200, json.dumps({"node": node, "last": out[-1][0] if out else since, "pings": out}))
        if u.path == "/api/sonar/glance":
            return self.send(200, json.dumps(glance(int(q.get("since", ["0"])[0]))))
        if u.path == "/api/sim":
            return self.send(200, json.dumps(self.simj()))
        if u.path == "/api/sonar/bg":
            return self.send(200, json.dumps(frames[now_index()]["bg"]))
        if u.path == "/api/radio":
            return self.send(200, json.dumps(radio()))
        if u.path == "/api/settings":
            d = dict(state["settings"]); d.update({"nodeId": 100, "role": 3}); return self.send(200, json.dumps(d))
        if u.path == "/api/sonar/knobs":
            return self.send(200, json.dumps(knobs_json()))
        if u.path == "/api/trips":
            return self.send(200, json.dumps({"uptime": int(time.time() - t0), "trips": state["trips"]}))
        self.send(404, "{}")

    def do_POST(self):
        u = urlparse(self.path); b = self.body()
        if u.path == "/api/sonar":
            if "focus" in b: state["focus"] = int(b["focus"])
            if "sim" in b: state["sim"] = bool(b["sim"])
            return self.send(200, self.listing())
        if u.path == "/api/sim":
            if "rate" in b: state["simrate"] = int(b["rate"])
            if "virtual" in b: state["sim"] = bool(b["virtual"])
            if "all" in b:
                state["sim"] = bool(b["all"]); v = (3 | (state["simrate"] << 2)) if b["all"] else 0
                for n in status()["nodes"]: state["simreq"][n["id"]] = v
            if "hole" in b:
                state["simreq"][int(b["hole"])] = ((1 if b.get("sonar") else 0) | (2 if b.get("hall") else 0)) | ((state["simrate"] << 2) if (b.get("sonar") or b.get("hall")) else 0)
            return self.send(200, json.dumps(self.simj()))
        if u.path == "/api/silence":
            state["silenced"] = not state["silenced"]; state["sil_until"] = time.time() + 300
            return self.send(200, json.dumps({"silenced": state["silenced"]}))
        if u.path == "/api/node/name":
            state["names"][int(b.get("nodeId", 0))] = str(b.get("name", ""))[:15]; return self.send(200, '{"success":true}')
        if u.path == "/api/radio":
            if "test" in b: state["test"] = int(b["test"])
            if "adaptive" in b: state["adaptive"] = bool(b["adaptive"])
            if "transport" in b: state["transport"] = int(b["transport"])
            if "channel" in b:
                if b["channel"] == "auto": state["ch_setting"] = "auto"
                else:
                    state["ch_setting"] = int(b["channel"]) - 1
                    if state["ch"] != state["ch_setting"]: state["ch"] = state["ch_setting"]; state["moves"] += 1
            if "relay" in b: state["relay_req"][int(b["relay"]["dev"])] = bool(b["relay"]["on"])
            return self.send(200, json.dumps(radio()))
        if u.path == "/api/settings":
            state["settings"].update({k: v for k, v in b.items() if k in state["settings"]}); return self.send(200, '{"success":true}')
        if u.path == "/api/sonar/knobs":
            if "key" in b and b["key"] in state["knobs"]: state["knobs"][b["key"]] = int(b["v"])
            if "preset" in b:
                p = {0: (7, 4, 2, 6), 1: (10, 5, 3, 5), 2: (14, 7, 5, 4)}.get(int(b["preset"]))
                if p: state["knobs"].update(dict(zip(("snr", "prom", "confirm", "keep"), p)))
            if b.get("defaults"): state["knobs"] = {k[0]: k[4] for k in KNOBS}
            return self.send(200, json.dumps(knobs_json()))
        if u.path == "/api/node/bait":
            state["bait"][int(b.get("nodeId", 0))] = int(b.get("cm", 0)); return self.send(200, '{"success":true}')
        self.send(404, "{}")

print("Web suite preview on http://localhost:%d/" % PORT)
ThreadingHTTPServer(("", PORT), H).serve_forever()
