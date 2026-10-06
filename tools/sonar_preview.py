#!/usr/bin/env python3
"""Desktop preview of the chalet /sonar page with fake data, no hardware needed.

    g++ -std=c++11 -O1 -Ilib/IceMesh/src tools/sonar_trace.cpp -o .preview/sonar_trace
    .preview/sonar_trace 300 > .preview/trace.json
    python3 tools/sonar_preview.py            # then open http://localhost:8000/sonar

The page HTML is taken from src/lora_node/main.cpp (handleWebSonar), the data from the trace
recorded with the real codec and store (one entry per second, replayed in real time, looped).
"""
import json, os, sys, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRACE = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, ".preview", "trace.json")
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8000

def page_html():
    src = open(os.path.join(ROOT, "src", "lora_node", "main.cpp"), encoding="utf-8").read()
    i = src.index("void handleWebSonar() {")
    a = src.index('R"rawliteral(', i) + len('R"rawliteral(')
    return src[a:src.index(')rawliteral"', a)]

frames = json.load(open(TRACE))
t0 = time.time()
state = {"focus": frames[0]["list"]["focus"], "sim": True}

def now_index():
    return int(time.time() - t0) % len(frames)

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

    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path in ("/", "/sonar"):
            return self.send(200, page_html(), "text/html; charset=utf-8")
        if u.path == "/api/sonar":
            return self.send(200, self.listing())
        if u.path == "/api/status":
            nodes = [{"id": n["node"], "name": "Hole %d" % n["node"], "online": True, "fish": False} for n in frames[0]["list"]["nodes"]]
            return self.send(200, json.dumps({"nodes": nodes}))
        if u.path == "/api/sonar/pings":
            node = int(q.get("node", ["0"])[0]); since = int(q.get("since", ["0"])[0]); mx = int(q.get("max", ["40"])[0])
            k = now_index(); out = []
            if node == frames[0]["list"]["focus"]:
                for f in frames[: k + 1]:
                    out += [p for p in f["pings"] if p[0] > since]
                if k < 2: out = [p for p in frames[k]["pings"]]   # loop restarted
            out = out[-mx:]
            last = out[-1][0] if out else since
            return self.send(200, json.dumps({"node": node, "last": last, "pings": out}))
        if u.path == "/api/sonar/bg":
            return self.send(200, json.dumps(frames[now_index()]["bg"]))
        self.send(404, "{}")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0")); body = json.loads(self.rfile.read(n) or b"{}")
        if "focus" in body: state["focus"] = int(body["focus"])
        if "sim" in body: state["sim"] = bool(body["sim"])
        self.send(200, self.listing())

print("Sonar preview on http://localhost:%d/sonar (only the recorded FOCUS node streams)" % PORT)
ThreadingHTTPServer(("", PORT), H).serve_forever()
