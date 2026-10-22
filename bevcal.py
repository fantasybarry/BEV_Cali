"""Camera calibration for vehicle_camera_calibration_measurement_plan.md (section refs § point there).

Frames: survey (leveled total station, +Z up), vehicle (§6.2), board (§4.1), camera (x right, y down, z forward).
T_a_from_b is a 4x4 mapping b coordinates into a. All lengths in meters.

CLI:
  bevcal.py feasibility --height 1.5 --range 10 --hfov 70 --width 4656
  bevcal.py detect IMG... --out DIR
  bevcal.py intrinsics --fit DIR --check DIR --spacing SX [SY] --models standard rational --out result.json
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

PATTERN = (7, 10)  # inner corners (cols, rows) in the portrait reference orientation §4
OABC = [0, 6, 63, 69]  # indices of O, A, B, C in board order
LIMIT = 0.05  # horizontal ground error, m §1
BANDS = [(0, 2), (2, 5), (5, 8), (8, 10)]  # range from body outline, m §10.1
MIN_VALIDATION = 24  # per camera §10.1
FLAGS = {
    "k1k2": cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST,
    "standard": 0,
    "rational": cv2.CALIB_RATIONAL_MODEL,
    "fisheye": cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
}
CRIT = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-12)


# ---------- board §4 ----------

def board_points(sx, sy=None):
    """Inner corners in board frame, row-major from O. sy for a measured-anisotropic print."""
    sy = sx if sy is None else sy
    r, c = np.mgrid[0:PATTERN[1], 0:PATTERN[0]]
    return np.stack([c.ravel() * sx, r.ravel() * sy, np.zeros(r.size)], 1)


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _patch(gray, p, k):
    u, v = int(round(p[0])), int(round(p[1]))
    return gray[max(v - k, 0):v + k + 1, max(u - k, 0):u + k + 1].mean()


def detect(gray):
    """7x10 inner corners in board order (O first, row-major) as (70, 2), or None.

    OpenCV may return the grid mirrored or rotated 180°; both are resolved here (§4.1).
    """
    ok, c = cv2.findChessboardCornersSB(gray, PATTERN, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        return None
    g = c.reshape(PATTERN[1], PATTERN[0], 2).astype(np.float64)
    if _cross(g[0, 1] - g[0, 0], g[1, 0] - g[0, 0]) < 0:  # seen from the printed face, board x/y keep image u/v handedness
        g = g[:, ::-1]
    dx, dy = g[0, 1] - g[0, 0], g[1, 0] - g[0, 0]
    # The corner square diagonally outside O is black: darker than the square beside it.
    k = max(1, int(0.15 * min(np.linalg.norm(dx), np.linalg.norm(dy))))
    if _patch(gray, g[0, 0] - (dx + dy) / 2, k) > _patch(gray, g[0, 0] + (dx - dy) / 2, k):
        g = g[::-1, ::-1]
    return g.reshape(-1, 2)


def annotate(img, corners):
    """Image with detected corners and O/A/B/C labels, for the ordering check §9.7."""
    out = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.drawChessboardCorners(out, PATTERN, corners.reshape(-1, 1, 2).astype(np.float32), True)
    s = max(out.shape[:2]) / 800
    for name, i in zip("OABC", OABC):
        p = tuple(int(v) for v in corners[i])
        cv2.putText(out, name, p, cv2.FONT_HERSHEY_SIMPLEX, s, (0, 0, 255), max(1, int(2 * s)))
    return out


# ---------- camera models §8 ----------

def _as3(X):
    return np.asarray(X, np.float64).reshape(-1, 1, 3)


def _as2(uv):
    return np.asarray(uv, np.float64).reshape(-1, 1, 2)


def calibrate(obj, pts, size, model):
    """Fit intrinsics. obj: list of (N,3), pts: list of (N,2), size: (w, h). Returns K, D, rms px."""
    if model == "fisheye":
        # fisheye's own initialization fails on wide-angle edge views; seed K from a rational pinhole fit
        K0 = calibrate(obj, pts, size, "rational")[0]
        rms, K, D, _, _ = cv2.fisheye.calibrate([_as3(o) for o in obj], [_as2(p) for p in pts], size,
                                                K0, np.zeros(4), flags=FLAGS[model] | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS,
                                                criteria=CRIT)
    else:
        rms, K, D, _, _ = cv2.calibrateCamera([np.float32(o) for o in obj], [np.float32(p) for p in pts], size,
                                              None, None, flags=FLAGS[model], criteria=CRIT)
    return K, D.ravel(), rms


def project(X, rvec, tvec, K, D, model):
    f = cv2.fisheye.projectPoints if model == "fisheye" else cv2.projectPoints
    return f(_as3(X), np.float64(rvec), np.float64(tvec), K, D)[0].reshape(-1, 2)


def undistort(uv, K, D, model):
    """Pixels -> normalized image coordinates (x, y) on the z=1 plane."""
    if model == "fisheye":
        return cv2.fisheye.undistortPoints(_as2(uv), K, D, criteria=CRIT).reshape(-1, 2)
    return cv2.undistortPointsIter(_as2(uv), K, D, None, None, CRIT).reshape(-1, 2)


def solve_pose(X, uv, K, D, model):
    """rvec, tvec of T_camera_from_X."""
    if model == "fisheye":
        # ponytail: fisheye.solvePnP minimizes in normalized coords, not pixels; edge points weigh
        # differently on a ~126° lens. Add a pixel-space LM refine if CAM2 residuals look structured.
        _, r, t = cv2.fisheye.solvePnP(_as3(X), _as2(uv), K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        return r, t
    _, r, t = cv2.solvePnP(_as3(X), _as2(uv), K, D, flags=cv2.SOLVEPNP_ITERATIVE)
    return cv2.solvePnPRefineLM(_as3(X), _as2(uv), K, D, r, t, CRIT)


def fov_deg(K, D, model, size):
    """Measured (HFOV, VFOV) through the principal point §8."""
    (w, h), (cx, cy) = size, K[:2, 2]
    x = undistort([[0, cy], [w - 1, cy], [cx, 0], [cx, h - 1]], K, D, model)
    a = np.degrees(np.arctan(np.abs([x[0, 0], x[1, 0], x[2, 1], x[3, 1]])))
    return float(a[0] + a[1]), float(a[2] + a[3])


def intrinsics(fit, check, size, models):
    """Fit each candidate model on `fit`, score on held-out `check` (lists of (obj, uv)). §8"""
    out = {}
    for m in models:
        try:
            K, D, rms = calibrate([o for o, _ in fit], [p for _, p in fit], size, m)
        except cv2.error as e:
            out[m] = {"error": str(e).strip().splitlines()[-1]}
            continue
        uv = np.concatenate([p for _, p in check])
        res = uv - np.concatenate([project(o, *solve_pose(o, p, K, D, m), K, D, m) for o, p in check])
        # 3x3 image grid of held-out RMS, row-major from top-left, to expose structured residuals
        cell = (np.minimum(uv[:, 1] * 3 // size[1], 2) * 3 + np.minimum(uv[:, 0] * 3 // size[0], 2)).astype(int)
        grid = [float(np.sqrt((res[cell == i] ** 2).sum(1).mean())) if (cell == i).any() else None for i in range(9)]
        hfov, vfov = fov_deg(K, D, m, size)
        out[m] = {"K": K.tolist(), "D": D.tolist(), "fit_rms_px": rms,
                  "check_rms_px": float(np.sqrt((res ** 2).sum(1).mean())),
                  "check_max_px": float(np.linalg.norm(res, axis=1).max()),
                  "check_rms_grid_px": grid, "hfov_deg": hfov, "vfov_deg": vfov}
    ok = [m for m in out if "error" not in out[m]]
    return out, min(ok, key=lambda m: out[m]["check_rms_px"]) if ok else None


# ---------- survey and vehicle frame §6, §7 ----------

def apply(T, P):
    return np.asarray(P) @ T[:3, :3].T + T[:3, 3]


def fit_plane(P):
    """Least-squares plane: centroid, unit normal (+z up), signed residuals."""
    c = P.mean(0)
    n = np.linalg.svd(P - c)[2][2]
    n = n if n[2] > 0 else -n
    return c, n, (P - c) @ n


def fit_circle_3d(P):
    """Rim-lip circle §6.2: center, radius, residual per point (radial and out-of-plane combined)."""
    c, n, off = fit_plane(P)
    u = np.linalg.svd(P - c)[2][0]
    v = np.cross(n, u)
    x, y = (P - c) @ u, (P - c) @ v
    a, b, k = np.linalg.lstsq(np.c_[2 * x, 2 * y, np.ones_like(x)], x * x + y * y, rcond=None)[0]
    r = np.sqrt(k + a * a + b * b)
    return c + a * u + b * v, r, np.hypot(np.hypot(x - a, y - b) - r, off)


def rigid_transform(A, B):
    """T with B ≈ T·A (Kabsch), and per-point residual norms."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cb - R @ ca
    return T, np.linalg.norm(apply(T, A) - B, axis=1)


def vehicle_frame(wheels, ground):
    """T_vehicle_from_survey §6.2. wheels: {FL, FR, RL, RR: center XYZ}, ground: (N,3), survey frame."""
    c, z, _ = fit_plane(ground)
    rear, front = (wheels["RL"] + wheels["RR"]) / 2, (wheels["FL"] + wheels["FR"]) / 2
    x = front - rear
    x -= (x @ z) * z
    x /= np.linalg.norm(x)
    R = np.array([x, np.cross(z, x), z])
    o = rear - ((rear - c) @ z) * z
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, -R @ o
    return T


def outline_range(xy, outline):
    """Horizontal distance from the body-outline polygon (vehicle XY), 0 inside §1."""
    poly = np.float32(outline).reshape(-1, 1, 2)
    return np.array([max(0.0, -cv2.pointPolygonTest(poly, (float(x), float(y)), True)) for x, y in xy])


def board_in_vehicle(B, offsets, targets):
    """Rigid board pose from the four border targets §9.4.

    B: board_points(); offsets: (4,3) target centers relative to O, A, B, C in board frame;
    targets: (4,3) surveyed target centers in vehicle frame. Returns corners in vehicle frame, target residuals.
    """
    T, res = rigid_transform(B[OABC] + offsets, np.asarray(targets))
    return apply(T, B), res


# ---------- extrinsics and ground validation §9–§11 ----------

def extrinsics(X, uv, K, D, model):
    """T_vehicle_from_camera from vehicle-frame points (board corners + fit-set ground targets), and px residuals."""
    r, t = solve_pose(X, uv, K, D, model)
    res = np.linalg.norm(project(X, r, t, K, D, model) - uv, axis=1)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(r)[0], np.ravel(t)
    return np.linalg.inv(T), res


def ground_xy(uv, K, D, model, T_vc, z=0.0, min_deg=2.0):
    """Intersect pixel rays with the horizontal plane Z=z (scalar or per point) §10.2.

    Rays pointing away from the plane or within min_deg of parallel come back as NaN.
    """
    d = np.c_[undistort(uv, K, D, model), np.ones(len(uv))] @ T_vc[:3, :3].T
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    with np.errstate(divide="ignore", invalid="ignore"):
        lam = (z - T_vc[2, 3]) / d[:, 2]
    xy = T_vc[:2, 3] + lam[:, None] * d[:, :2]
    xy[(lam <= 0) | (np.abs(d[:, 2]) < np.sin(np.radians(min_deg)))] = np.nan
    return xy


def evaluate(rows):
    """Per-camera acceptance summary §11.

    rows: dicts with camera, point, est (XY), survey (XY), range (m from outline), unc (reference uncertainty, m).
    """
    summary = {}
    for cam in sorted({r["camera"] for r in rows}):
        rs = [r for r in rows if r["camera"] == cam]
        e = np.array([np.linalg.norm(np.subtract(r["est"], r["survey"])) for r in rs])
        rng = np.array([r["range"] for r in rs])
        unc = np.array([r["unc"] for r in rs])
        ok = np.isfinite(e) & (rng <= BANDS[-1][1])
        e, rng, unc = e[ok], rng[ok], unc[ok]
        bands = {f"{lo}-{hi}": {"n": int(m.sum()), "max": float(e[m].max()) if m.any() else None}
                 for lo, hi in BANDS for m in [((rng > lo) | (lo == 0)) & (rng <= hi)]}
        if cam == "CAM1":
            status = "NOT APPLICABLE (planned)"
        elif (e > LIMIT).any():
            status = "FAIL"
        elif len(e) < MIN_VALIDATION or not bands["8-10"]["n"]:
            status = "NOT VALIDATED"
        elif (e + unc > LIMIT).any():
            status = "BORDERLINE (refer to requirement owner)"
        else:
            status = "PASS at sampled locations"
        summary[cam] = {"status": status, "n": len(e), "rejected_or_out_of_range": int((~ok).sum()),
                        "median": float(np.median(e)) if len(e) else None,
                        "rms": float(np.sqrt((e ** 2).mean())) if len(e) else None,
                        "p95": float(np.percentile(e, 95)) if len(e) else None,
                        "max": float(e.max()) if len(e) else None, "bands": bands}
    return summary


def overlap(rows):
    """Horizontal disagreement between cameras that see the same validation point §11 (diagnostic only)."""
    by_point = {}
    for r in rows:
        if np.all(np.isfinite(r["est"])):
            by_point.setdefault(r["point"], []).append(r)
    return [{"point": p, "cameras": (a["camera"], b["camera"]),
             "diff": float(np.linalg.norm(np.subtract(a["est"], b["est"])))}
            for p, rs in by_point.items() for i, a in enumerate(rs) for b in rs[i + 1:]]


def footprint(h, r, hfov_deg, width_px):
    """Ground range change per pixel at horizontal camera distance r, feasibility gate §2.3."""
    return (h + r * r / h) * np.radians(hfov_deg) / width_px


# ---------- CLI ----------

IMAGE_EXT = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def _images(d):
    return sorted(p for p in Path(d).iterdir() if p.suffix.lower() in IMAGE_EXT)


def _observations(paths, B, size):
    """Detect corners in each image. Returns accepted [(obj, uv)], rejected [(path, reason)], size."""
    obs, rejected = [], []
    for p in paths:
        gray = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            rejected.append((str(p), "unreadable"))
            continue
        size = size or gray.shape[::-1]
        if gray.shape[::-1] != size:
            sys.exit(f"{p}: {gray.shape[::-1]} differs from {size}; one capture mode per calibration (§2.1)")
        uv = detect(gray)
        if uv is None:
            rejected.append((str(p), "no 7x10 detection"))
        else:
            obs.append((B, uv))
    return obs, rejected, size


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("feasibility", help="ground footprint per pixel §2.3")
    f.add_argument("--height", type=float, required=True, help="camera height above ground, m")
    f.add_argument("--range", type=float, required=True, help="horizontal camera-to-point distance, m")
    f.add_argument("--hfov", type=float, required=True, help="horizontal FOV, deg")
    f.add_argument("--width", type=int, required=True, help="image width at the capture mode, px")
    d = sub.add_parser("detect", help="annotated O/A/B/C corner images for the ordering check §9.7")
    d.add_argument("images", nargs="+")
    d.add_argument("--out", required=True)
    i = sub.add_parser("intrinsics", help="fit and compare lens models §8")
    i.add_argument("--fit", required=True, help="folder of fitting images")
    i.add_argument("--check", required=True, help="folder of held-out images")
    i.add_argument("--spacing", type=float, nargs="+", required=True, help="measured square spacing SX [SY], m")
    i.add_argument("--models", nargs="+", default=["standard", "rational"], choices=list(FLAGS))
    i.add_argument("--out", required=True, help="result JSON (must not exist)")
    a = ap.parse_args(argv)

    if a.cmd == "feasibility":
        fp = footprint(a.height, a.range, a.hfov, a.width)
        print(f"{fp * 100:.2f} cm/px -> {'OK' if fp <= LIMIT / 2 else 'EXCEEDS 2.5 cm: refer to requirement owner'}")
    elif a.cmd == "detect":
        Path(a.out).mkdir(parents=True, exist_ok=True)
        for p in map(Path, a.images):
            img = cv2.imread(str(p))
            uv = None if img is None else detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
            if uv is None:
                print(f"{p}: no detection")
                continue
            cv2.imwrite(str(Path(a.out) / f"{p.stem}_annot.png"), annotate(img, uv))
            np.savetxt(Path(a.out) / f"{p.stem}_corners.csv", uv, fmt="%.4f", delimiter=",", header="u,v")
            print(f"{p}: ok")
    elif a.cmd == "intrinsics":
        B = board_points(*a.spacing[:2])
        fit, rej_fit, size = _observations(_images(a.fit), B, None)
        check, rej_check, size = _observations(_images(a.check), B, size)
        print(f"fit {len(fit)} accepted / {len(rej_fit)} rejected; check {len(check)} / {len(rej_check)}")
        if len(fit) < 30 or len(check) < 10:
            print("warning: below 30 fit + 10 held-out views (§8)")
        models, best = intrinsics(fit, check, size, a.models)
        for m, r in models.items():
            print(m, r.get("error") or f"fit {r['fit_rms_px']:.3f} px, held-out {r['check_rms_px']:.3f} px, "
                                       f"HFOV {r['hfov_deg']:.1f}°, VFOV {r['vfov_deg']:.1f}°")
        print("lowest held-out RMS:", best)
        with open(a.out, "x") as fh:
            json.dump({"image_size": size, "pattern": PATTERN, "spacing": a.spacing, "models": models,
                       "lowest_check_rms": best, "rejected": rej_fit + rej_check}, fh, indent=2)


if __name__ == "__main__":
    main()
