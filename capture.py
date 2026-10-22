"""Single-camera capture tool for the calibration session (plan §13), for the Jetson Orin (Linux V4L2).

Needs v4l2-ctl (sudo apt install v4l-utils). Stable device links: ls -l /dev/v4l/by-path /dev/v4l/by-id
Control names differ between kernels; copy them from: v4l2-ctl -d DEV --list-ctrls-menus
List auto-mode controls before the manual values they unlock (controls are applied in file order).

  python capture.py --cameras cameras.json --camera CAM2 --session sessions/S01 --kind intrinsic-fit --spacing 0.0741
  python capture.py ... --kind extrinsic --placement P03

Keys: space = capture, q / Esc = quit. Images land in <session>/<camera>/<kind>/, each with a JSON sidecar.
"""
import argparse
import datetime
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

from bevcal import PATTERN

KINDS = {"intrinsic-fit": 30, "intrinsic-check": 10, "extrinsic": None, "ground": None, "focus-check": None}
PREVIEW_W = 1280


def v4l2(dev, *args):
    r = subprocess.run(["v4l2-ctl", "-d", dev, *args], capture_output=True, text=True)
    if r.returncode:
        sys.exit(f"v4l2-ctl {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout


def check_controls(dev, wanted):
    """Read controls back; exit if any differs from the mapping file (§13.4)."""
    out = v4l2(dev, "--get-ctrl=" + ",".join(wanted))
    got = {k.strip(): int(v) for k, v in (line.split(":", 1) for line in out.splitlines() if ":" in line)}
    bad = {k: (v, got.get(k)) for k, v in wanted.items() if got.get(k) != v}
    if bad:
        sys.exit(f"{dev}: controls not as configured (wanted, got): {bad}. Power-cycled? Restart the tool.")
    return got


def open_camera(cfg):
    dev = cfg["device"]
    if not dev.startswith(("/dev/v4l/by-path/", "/dev/v4l/by-id/")):
        sys.exit(f"{dev}: use a /dev/v4l/by-path or by-id link, never discovery-order /dev/videoN (§13.3)")
    fourcc = {"YUY2": "YUYV"}.get(cfg["fourcc"], cfg["fourcc"])
    if fourcc not in ("YUYV", "MJPG"):
        sys.exit(f"{fourcc}: only YUYV and MJPG are supported")
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f"cannot open {dev}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["height"])
    cap.set(cv2.CAP_PROP_FPS, cfg.get("fps", 5))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # raw buffers: MJPG is saved byte-exact
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
           int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(errors="replace"))
    if got != (cfg["width"], cfg["height"], fourcc):
        sys.exit(f"{dev}: camera gave mode {got}, configured {(cfg['width'], cfg['height'], fourcc)} (§2.1)")
    for k, v in cfg["controls"].items():  # re-applied on every start: cameras forget them on power-up
        v4l2(dev, f"--set-ctrl={k}={v}")
    check_controls(dev, cfg["controls"])
    return cap, fourcc


def decode(raw, fourcc, w, h):
    """Raw V4L2 buffer -> BGR image. Raises if the frame is not the configured size (§5)."""
    raw = np.asarray(raw, np.uint8).ravel()
    img = None
    if fourcc == "MJPG":
        img = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    elif raw.size == w * h * 2:
        img = cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUYV)
    if img is None or img.shape[:2] != (h, w):
        raise ValueError(f"bad {fourcc} frame: {raw.size} bytes, expected {w}x{h}")
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cameras", required=True, help="mapping file: camera ID -> device, mode, controls")
    ap.add_argument("--camera", required=True)
    ap.add_argument("--session", required=True, help="session folder; files are never overwritten")
    ap.add_argument("--kind", required=True, choices=list(KINDS))
    ap.add_argument("--placement", help="placement ID, required for extrinsic")
    ap.add_argument("--spacing", type=float, nargs="+", required=True, help="measured square spacing SX [SY], m")
    a = ap.parse_args()
    if a.kind == "extrinsic" and not a.placement:
        ap.error("--placement is required for extrinsic captures")

    cfg = json.loads(Path(a.cameras).read_text())[a.camera]
    w, h = cfg["width"], cfg["height"]
    cap, fourcc = open_camera(cfg)
    dev = cfg["device"]
    info = v4l2(dev, "--info").strip().splitlines()
    out = Path(a.session) / a.camera / a.kind
    out.mkdir(parents=True, exist_ok=True)
    n = len(list(out.glob("*.json")))
    target = KINDS[a.kind]
    s = min(1.0, PREVIEW_W / w)
    win = f"{a.camera} {a.kind}"

    while True:
        ok, raw = cap.read()
        if not ok:
            sys.exit("frame read failed (USB bandwidth? capture one camera at a time, §13.7)")
        small = cv2.resize(decode(raw, fourcc, w, h), None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        # ponytail: board detection on every preview frame; skip frames if the preview lags on the Orin
        found, c = cv2.findChessboardCornersSB(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), PATTERN)
        if found:
            cv2.drawChessboardCorners(small, PATTERN, c, True)
        status = f"{a.camera} {a.kind} {a.placement or ''} saved {n}{f'/{target}' if target else ''}  " \
                 f"board {'7x10' if found else '--'}"
        cv2.putText(small, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.imshow(win, small)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key != ord(" "):
            continue

        for _ in range(2):  # drop buffered frames so the saved image is what the camera sees now
            cap.grab()
        ok, raw = cap.retrieve()
        if not ok:
            sys.exit("frame read failed")
        img = decode(raw, fourcc, w, h)
        controls = check_controls(dev, cfg["controls"])  # a USB reset silently restores auto modes
        data = np.asarray(raw, np.uint8).tobytes() if fourcc == "MJPG" else cv2.imencode(".png", img)[1].tobytes()
        stamp = datetime.datetime.now().astimezone()
        stem = f"{a.camera}_{a.kind}{'_' + a.placement if a.placement else ''}_{stamp:%Y%m%dT%H%M%S}_{n:03d}"
        path = out / f"{stem}.{'jpg' if fourcc == 'MJPG' else 'png'}"
        with open(path, "xb") as fh:
            fh.write(data)
        meta = {"camera": a.camera, "device": dev, "device_resolved": os.path.realpath(dev), "v4l2_info": info,
                "width": w, "height": h, "fourcc": fourcc, "fps": cap.get(cv2.CAP_PROP_FPS), "controls": controls,
                "kind": a.kind, "placement": a.placement, "spacing_m": a.spacing, "session": a.session,
                "timestamp": stamp.isoformat(), "file": path.name, "sha256": hashlib.sha256(data).hexdigest()}
        with open(out / f"{stem}.json", "x") as fh:
            json.dump(meta, fh, indent=2)
        n += 1
        print(f"saved {path}")


if __name__ == "__main__":
    main()
