#!/usr/bin/env python3
"""Flash one board of the ice-fishing system with one command (tools/flash.html builds the command for you).

    python tools/flash.py <purpose> --id N [--name "Pointe"] [--port COM7] [--sonar] [--gate] [--bench] [--build-only] [--ota-out FOLDER]

purpose: chalet | hub | relay | sensor_lora | tipup_c3 | tipup_wroom
The board is written to devices.ini (your fleet: one [env:...] per physical board, kept in git) and
PlatformIO builds + uploads that env. Re-flash the same board later: python tools/flash.py --env <name> --port COM7
"""
import argparse, datetime, os, re, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INI = os.path.join(ROOT, "devices.ini")
PURPOSE = {   # purpose: (base env, role, extra build flags)
    "chalet":      ("cabin",        "ROLE_GATEWAY_OFFSHORE", ["-D HAS_LOCAL_SENSOR=false"]),
    "hub":         ("hub",          "ROLE_GATEWAY_ONSHORE",  ["-D HAS_LOCAL_SENSOR=true"]),
    "relay":       ("relay",        "ROLE_RELAY_LORA",       ["-D HAS_LOCAL_SENSOR=false"]),
    "sensor_lora": ("sensor_lora",  "ROLE_SENSOR_LORA",      ["-D HAS_LOCAL_SENSOR=true"]),
    "tipup_c3":    ("sensor_c3",    None, ["-D BOARD_ESP32C3", "-D ARDUINO_USB_MODE=1", "-D ARDUINO_USB_CDC_ON_BOOT=1"]),
    "tipup_wroom": ("sensor_wroom", None, ["-D BOARD_ESP32WROOM"]),
}

def env_name(purpose, node_id, sonar, bench):
    base = {"chalet": "chalet", "hub": "hub", "relay": "relay", "sensor_lora": "lora", "tipup_c3": "hole", "tipup_wroom": "hole"}[purpose]
    return "%s%d%s%s" % (base, node_id, "_sonar" if sonar else "", "_bench" if bench else "")

def block(purpose, node_id, name, sonar, bench, gate=False):
    base, role, extra = PURPOSE[purpose]
    flags = ([("-D NODE_ROLE=%s" % role)] if role else []) + ["-D NODE_ID=%d" % node_id, "'-D NODE_NAME=\"%s\"'" % name] + extra
    if sonar: flags.append("-D SONAR_REAL=1")
    if sonar and gate: flags.append("-D SONAR_EN_WIRE=1")   # D181: 30 V build with the enable wire: converter off while each ping listens
    if bench: flags.append("-D SONAR_BENCH=1")
    if bench and role: flags.append("-D BENCH_RING_BYTES=10240")   # D152: LoRa boards share the heap with the mesh and the web server
    env = env_name(purpose, node_id, sonar, bench)
    return env, "[env:%s]\nextends = env:%s\nbuild_flags =\n%s\n" % (env, base, "\n".join("    " + f for f in flags))

def write_ini(env, text):
    old = open(INI, encoding="utf-8").read() if os.path.exists(INI) else "; Your boards, one env each (written by tools/flash.py; edit by hand if you like).\n; platformio.ini loads this file (extra_configs).\n"
    pat = re.compile(r"\[env:%s\]\n(?:(?!\[env:).*\n?)*" % re.escape(env))
    # D170: flags added by hand to this board (e.g. -D SONAR_PIN_BOOST=40) and comment lines survive a re-flash: every
    # line of the old block whose -D key the tool does not write itself is kept at the end of the new block
    m = pat.search(old)
    if m:
        key = lambda l: (re.match(r"\s*'?-D\s*([A-Za-z0-9_]+)", l) or [None, None])[1]
        mine = {key(l) for l in text.splitlines()}
        keep = [l for l in m.group(0).splitlines()[1:] if l.strip() and not l.startswith(("extends", "build_flags")) and (l.lstrip().startswith(";") or (key(l) and key(l) not in mine))]
        if keep: text = text.rstrip("\n") + "\n" + "\n".join(keep) + "\n"
    new = pat.sub(text, old) if pat.search(old) else old.rstrip("\n") + "\n\n" + text
    if new != old:
        open(INI, "w", encoding="utf-8").write(new)
        print("devices.ini: [env:%s] %s" % (env, "updated" if pat.search(old) else "added"))

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("purpose", nargs="?", choices=sorted(PURPOSE))
    ap.add_argument("--id", type=int, help="node ID, unique on the network: 1-100 (101-124 = demo network, 128-255 = test holes)")
    ap.add_argument("--name", default=None, help="name shown on the screens (default: Hole N / role)")
    ap.add_argument("--port", help="serial port (COM7, /dev/ttyUSB0); omit = PlatformIO guesses")
    ap.add_argument("--sonar", action="store_true", help="with the TUSS4470 shield 002 (tipup_wroom, hub, sensor_lora)")
    ap.add_argument("--gate", action="store_true", help="30 V build with the converter enable wire (XL6009 pin 2 to GPIO40 on a Heltec, GPIO13 on a WROOM): the converter is switched off while each ping listens. Kept in devices.ini for later re-flashes of this board")
    ap.add_argument("--bench", action="store_true", help="bucket test build: BENCH line every 2 s (tipup_wroom: real sonar from boot, no hub; hub: chalet optional)")
    ap.add_argument("--env", help="re-flash an env already in devices.ini")
    ap.add_argument("--build-only", action="store_true")
    ap.add_argument("--ota-out", metavar="FOLDER", help="build only, and copy the firmware file for the cable-free update (D148) into FOLDER as <env>_<date>_<time>.bin")
    ap.add_argument("--monitor", action="store_true", help="open the serial monitor after the upload")
    a = ap.parse_args()
    if a.ota_out: a.build_only = True
    if a.env:
        env = a.env
    else:
        if not a.purpose or a.id is None: ap.error("purpose and --id are needed (or --env)")
        if not (1 <= a.id <= 100): ap.error("--id must be 1-100 (101-124 demo network, 128-255 test holes)")
        if a.sonar and a.purpose not in ("tipup_wroom", "hub", "sensor_lora"): ap.error("--sonar is for tipup_wroom, hub, sensor_lora")
        if a.bench and a.purpose not in ("tipup_wroom", "hub"): ap.error("--bench is for tipup_wroom or hub")
        if a.bench: a.sonar = True
        if a.gate and not a.sonar: ap.error("--gate needs --sonar (or --bench)")
        name = a.name or {"chalet": "Chalet", "hub": "Hub %d" % a.id, "relay": "Relay %d" % a.id, "sensor_lora": "Hole %d" % a.id}.get(a.purpose, "Hole %d" % a.id)
        env, text = block(a.purpose, a.id, name, a.sonar, a.bench, a.gate)
        write_ini(env, text)
    cmd = ["pio", "run", "-e", env] + ([] if a.build_only else ["-t", "upload"]) + (["--upload-port", a.port] if a.port and not a.build_only else [])
    print("> " + " ".join(cmd))
    rc = subprocess.call(cmd, cwd=ROOT)
    if rc == 0 and a.ota_out:   # D148: the file the board's update page takes
        src = os.path.join(ROOT, ".pio", "build", env, "firmware.bin")
        if not os.path.isfile(src): print("! build finished but %s is missing" % src); sys.exit(1)
        out_dir = a.ota_out if os.path.isabs(a.ota_out) else os.path.join(ROOT, a.ota_out)
        os.makedirs(out_dir, exist_ok=True)
        dst = os.path.join(out_dir, "%s_%s.bin" % (env, datetime.datetime.now().strftime("%Y-%m-%d_%H%M")))
        shutil.copyfile(src, dst)
        data = open(dst, "rb").read()
        tags = sorted(set(m.group(0).decode() for m in re.finditer(rb"ICEFW1:R[A-Z_0-9]+:N[0-9]+:", data)))
        print("\nUpdate file: %s (%.2f MB)" % (dst, len(data) / 1048576.0))
        print("Board tag in the file: %s" % (", ".join(tags) if tags else "NONE: this firmware predates the update mode, the board will refuse it"))
        print("On the board: update mode (chalet page Settings > Advanced > Firmware update, serial OTA, or the arming button held 3 s at power-on),")
        print("then join its hotspot <network>-Hub<N>-UPDATE, open http://192.168.4.1/ and send this file.")
    if rc == 0 and a.monitor and not a.build_only:
        subprocess.call(["pio", "device", "monitor", "-b", "115200"] + (["-p", a.port] if a.port else []), cwd=ROOT)
    sys.exit(rc)

if __name__ == "__main__":
    main()
