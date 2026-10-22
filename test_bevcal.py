"""Synthetic end-to-end check of bevcal: board ordering, intrinsics, survey frame, extrinsics, ground validation.

Run: uv run python test_bevcal.py
"""
import cv2
import numpy as np

import bevcal as bc

rng = np.random.default_rng(0)
SIZE = (1920, 1200)  # CAM2-like: ~126° HFOV equidistant fisheye
K = np.array([[870.0, 0, 955], [0, 870, 603], [0, 0, 1]])
D = np.array([0.04, -0.01, 0.003, -0.001])
B = bc.board_points(0.074)


def rot(axis_angle):
    return cv2.Rodrigues(np.float64(axis_angle))[0]


def T_of(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def in_image(uv, margin=5):
    return np.all((uv > margin) & (uv < np.subtract(SIZE, margin)), axis=1)


def test_board_ordering():
    img = cv2.imread("Screenshot 2026-10-05 at 11-34-39 .png", cv2.IMREAD_GRAYSCALE)
    img = cv2.copyMakeBorder(img, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
    h, w = img.shape
    uv = bc.detect(img)
    assert uv is not None and uv.shape == (70, 2)
    assert uv[0].sum() < uv[69].sum() and uv[6][0] > uv[0][0] and uv[63][1] > uv[0][1], "O top-left, A right, B down"
    # 180° rotation: O is now physically at bottom-right
    uv180 = bc.detect(cv2.rotate(img, cv2.ROTATE_180))
    assert np.allclose(uv180, [w - 1, h - 1] - uv, atol=1.0)
    # perspective view keeps the same physical correspondence
    H = np.array([[0.8, 0.15, 60], [-0.1, 0.9, 90], [1e-4, 5e-5, 1]])
    uvp = bc.detect(cv2.warpPerspective(img, H, (w + 200, h + 200), borderValue=255))
    assert np.allclose(uvp, cv2.perspectiveTransform(uv.reshape(-1, 1, 2), H).reshape(-1, 2), atol=1.5)


def views(n, dist):
    """Random board poses in front of the camera; returns [(obj, uv)] fully inside the image."""
    out = []
    while len(out) < n:
        R = rot(rng.normal(0, 0.45, 3))
        t = R @ -B.mean(0) + [rng.uniform(-0.8, 0.8), rng.uniform(-0.5, 0.5), rng.uniform(*dist)]
        uv = bc.project(B, cv2.Rodrigues(R)[0], t, K, D, "fisheye")
        if (R @ [0, 0, 1])[2] > 0.3 and in_image(uv).all():
            out.append((B, uv + rng.normal(0, 0.15, uv.shape)))
    return out


def test_intrinsics():
    models, best = bc.intrinsics(views(30, (0.5, 1.6)), views(10, (0.5, 1.6)), SIZE, ["rational", "fisheye"])
    f = models["fisheye"]
    assert best == "fisheye", models
    assert np.allclose(np.array(f["K"])[[0, 1, 0, 1], [0, 1, 2, 2]], K[[0, 1, 0, 1], [0, 1, 2, 2]], atol=1.5), f["K"]
    assert f["check_rms_px"] < 0.3


def test_pipeline():
    # survey frame: leveled instrument, arbitrary yaw/offset, ground slightly sloped
    T_sv = T_of(rot([0.001, -0.002, 0.7]), [100.0, 200.0, 50.0])
    noise = lambda P, s=0.0007: P + rng.normal(0, s, np.shape(P))

    # §6.2 wheel rim lips (vertical circles) and ground points
    wheels = {"RL": [0, 0.8, 0.33], "RR": [0, -0.8, 0.33], "FL": [2.8, 0.8, 0.33], "FR": [2.8, -0.8, 0.33]}
    a = np.linspace(0, 2 * np.pi, 10, endpoint=False)
    centers = {}
    for k, c in wheels.items():
        lip = np.array(c) + 0.22 * np.c_[np.cos(a), np.zeros_like(a), np.sin(a)]
        centers[k], r, res = bc.fit_circle_3d(noise(bc.apply(T_sv, lip)))
        assert abs(r - 0.22) < 0.002 and res.max() < 0.003
    ground = np.c_[rng.uniform(-10, 13, (30, 2)), np.zeros(30)]
    T_vs = bc.vehicle_frame(centers, noise(bc.apply(T_sv, ground)))
    err = T_vs @ T_sv
    assert np.linalg.norm(err[:3, 3]) < 0.002 and np.degrees(np.arccos((np.trace(err[:3, :3]) - 1) / 2)) < 0.05
    survey = lambda P: bc.apply(T_vs, noise(bc.apply(T_sv, P)))  # surveyed points, expressed in vehicle frame

    # front-left camera, 45° left, 20° down
    yaw, pitch = np.radians(45), np.radians(20)
    fwd = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), -np.sin(pitch)])
    right = np.cross(fwd, [0, 0, 1])
    right /= np.linalg.norm(right)
    T_vc = T_of(np.c_[right, np.cross(fwd, right), fwd], [3.6, 0.85, 1.5])
    T_cv = np.linalg.inv(T_vc)
    cam = lambda P: bc.project(bc.apply(T_cv, P), np.zeros(3), np.zeros(3), K, D, "fisheye")

    # §9 raised board placements: survey the four border targets, rigid-fit the corners
    offsets = np.array([[-0.1, -0.1, -0.003], [0.1, -0.1, -0.003], [-0.1, 0.1, -0.003], [0.1, 0.1, -0.003]])
    X, uv = [], []
    while len(X) < 6 * 70:
        ray = T_vc[:3, :3] @ np.r_[rng.uniform(-0.8, 0.8, 2), 1]
        ray /= np.linalg.norm(ray)
        center = T_vc[:3, 3] + rng.uniform(2, 5) * ray
        if center[2] < 0.4:
            continue
        zb = ray  # board +z into the board, facing the camera
        xb = np.cross([0, 0, -1], zb)
        xb /= np.linalg.norm(xb)
        Rb = np.c_[xb, np.cross(zb, xb), zb] @ rot(rng.normal(0, 0.3, 3))
        T_vb = T_of(Rb, center - Rb @ B.mean(0))
        corners, res = bc.board_in_vehicle(B, offsets, survey(bc.apply(T_vb, B[bc.OABC] + offsets)))
        assert res.max() < 0.003
        px = cam(bc.apply(T_vb, B))
        if in_image(px).all():
            X.append(corners)
            uv.append(px + rng.normal(0, 0.15, px.shape))

    # §10 ground grid, interleaved fit/validation split
    g = np.mgrid[-2:16:0.75, -2:16:0.75].reshape(2, -1).T
    g = np.c_[g, np.zeros(len(g))]
    gpx = cam(g)
    seen = in_image(gpx) & (bc.apply(T_cv, g)[:, 2] > 0)
    g, gpx = g[seen], gpx[seen] + rng.normal(0, 0.1, (seen.sum(), 2))
    is_fit = np.arange(len(g)) % 2 == 0
    g_surveyed = survey(g)

    T_est, res = bc.extrinsics(np.r_[np.concatenate(X), g_surveyed[is_fit]],
                               np.r_[np.concatenate(uv), gpx[is_fit]], K, D, "fisheye")
    assert np.median(res) < 0.5, np.median(res)
    dT = np.linalg.inv(T_vc) @ T_est
    assert np.linalg.norm(dT[:3, 3]) < 0.005 and np.degrees(np.arccos(min(1, (np.trace(dT[:3, :3]) - 1) / 2))) < 0.03

    # §11 independent validation
    outline = [[-0.9, -0.95], [3.8, -0.95], [3.8, 0.95], [-0.9, 0.95]]
    val = ~is_fit
    sv = g_surveyed[val]
    ranges = bc.outline_range(sv[:, :2], outline)

    def rows(T, cam_id):
        est = bc.ground_xy(gpx[val], K, D, "fisheye", T)
        return [{"camera": cam_id, "point": i, "est": e, "survey": s[:2], "range": r, "unc": 0.003}
                for i, e, s, r in zip(np.flatnonzero(val), est, sv, ranges)]

    good = bc.evaluate(rows(T_est, "CAM3"))["CAM3"]
    assert good["status"] == "PASS at sampled locations", good
    assert good["bands"]["8-10"]["n"] > 0
    tilted = T_est @ T_of(rot([np.radians(0.2), 0, 0]), [0, 0, 0])  # 0.2° about camera x: pitch error
    assert bc.evaluate(rows(tilted, "CAM3"))["CAM3"]["status"] == "FAIL"
    d = bc.overlap(rows(T_est, "CAM3") + rows(tilted, "CAM5"))
    assert d and max(x["diff"] for x in d) > bc.LIMIT


def test_capture_decode():
    import capture
    w, h = 64, 48
    gray = capture.decode(np.full(w * h * 2, 128, np.uint8), "YUYV", w, h)
    assert gray.shape == (h, w, 3) and abs(int(gray[0, 0, 0]) - 128) <= 2
    jpg = cv2.imencode(".jpg", np.zeros((h, w, 3), np.uint8))[1]
    assert capture.decode(jpg, "MJPG", w, h).shape == (h, w, 3)
    for bad in [(np.zeros(10, np.uint8), "YUYV"), (jpg, "MJPG")]:
        try:
            capture.decode(bad[0], bad[1], w + 1, h)
            raise AssertionError("size mismatch not caught")
        except ValueError:
            pass


if __name__ == "__main__":
    test_capture_decode()
    assert abs(bc.footprint(1.5, 10, 70, 1920) - 0.0434) < 5e-4  # plan §2.3 example
    assert np.allclose(bc.outline_range([[0, 0], [5, 0], [0, -3]], [[-1, -1], [1, -1], [1, 1], [-1, 1]]), [0, 4, 2])
    test_board_ordering()
    test_intrinsics()
    test_pipeline()
    print("ok")
