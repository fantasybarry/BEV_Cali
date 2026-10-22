# Handoff — 2026-10-22

Project: survey-assisted intrinsic + extrinsic calibration of six vehicle cameras for BEV ground projection.
Spec: `vehicle_camera_calibration_measurement_plan.md` (§ refs below). Work order: `calibration_runbook.md`.
Vehicle computer: Jetson Orin (Linux, V4L2). Bench work so far: this Mac.

## Completed

- **`bevcal.py`** — calibration library + CLI (`feasibility`, `detect`, `intrinsics`).
  - Board: 7 × 10 inner corners, O/A/B/C ordering with mirror and 180° disambiguation (§4.1), annotated images.
  - Intrinsics: k1k2 / standard / rational / fisheye models, held-out scoring, 3 × 3 residual grid, HFOV/VFOV.
    Fisheye is seeded from a rational fit (OpenCV's own fisheye init fails on wide views).
  - Survey math: plane fit, 3D rim circle fit, Kabsch rigid fit, vehicle frame (§6.2), body-outline range.
  - Extrinsics from vehicle-frame points; ray–ground intersection; acceptance summary (§11); overlap diagnostics.
- **`capture.py`** — capture tool (§13). Linux: V4L2 + `v4l2-ctl` controls, by-path IDs, read-back checks.
  macOS bench mode: frames via pip-packaged ffmpeg (`imageio-ffmpeg`), native modes only, `--list-mac`,
  `--mac-device NAME`. Lossless saves + JSON sidecar with sha256, never overwrites. Refuses repeat poses in
  intrinsic modes.
- **`test_bevcal.py`** — synthetic end-to-end check (board ordering on the real pattern image, fisheye intrinsics,
  survey frame, extrinsics, validation PASS/FAIL, capture decode). `uv run python test_bevcal.py` → `ok`.
- **`cameras.example.json` / `cameras.json`** — six-camera mapping template (device paths are placeholders).
- **Practice intrinsics** on a spare AR0234 (NOT a vehicle camera): `sessions/practice01-unmounted-AR0234/`
  (see its README). 43 fit + 10 held-out images. Rational model best. Flat-board fit: held-out 0.35 px.
  Board-shape-corrected fit (done by hand, not in code): held-out 0.15 px, f ≈ 599.5 ± 0.16 px, ray uncertainty
  ≈ 0.03° (1σ). HFOV ≈ 123°.
- **`calibration_runbook.md`** — 37-step plan in 9 phases for the real calibration.
- Repo committed by the user (`e27f3b2 Calibrating Practice`); working tree was clean at handoff time.

## Current tasks (in progress / not started)

| Task | Status | Where |
|---|---|---|
| Board-shape correction (`cv2.calibrateCameraRO`) in `bevcal.py intrinsics`, flat vs corrected side by side | not started; method proven by hand | runbook step 8 |
| Intrinsics uncertainty in the report (OpenCV σ + bootstrap ray spread) | not started; method proven by hand | step 9 |
| `capture.py` on the Jetson | never run on Linux hardware | step 10 |
| Survey import + vehicle-frame/gate report CLI | blocked on surveyor export format | step 29 |
| Ground-target picker (sub-pixel), extrinsics CLI, validation report CLI | library functions exist, no CLI | steps 33–35 |

## Stuck / waiting on someone

1. **[requirement owner] 5 cm at 10 m is likely infeasible for CAM2.** CAM2-type: ≈ 11 cm ground per pixel at 10 m
   (f ≈ 600 px, 1.5 m height assumed), gate is 2.5 cm (§2.3). Lens calibration alone ≈ 3.3 cm (1σ), ~7 cm (95%) at
   10 m. Proposed: ≤ 5 cm within 0–5 m, ≤ 1% of range beyond, plus camera-overlap agreement. Not yet decided.
2. **[user] Board is twisted ≈ 4.7 mm peak to peak** (A and B forward, O and C back). Must be fixed (rigid flat
   backing) before extrinsics (3 mm gate, §4/§7). Confirm with straightedge on both diagonals.
3. **[surveyor] Export format unknown** → survey import code cannot be written yet.
4. **[user] Capture modes per camera not chosen** (§2.1); Mac and Jetson must use identical modes.
5. **[user] CAM3–6 autofocus status unknown.** If autofocus, intrinsics must be done on the Jetson with fixed
   `focus_absolute`; the Mac cannot set UVC controls.

## Next steps (recommended order)

1. Phase 0 decisions (requirement owner, capture modes, book survey) — longest lead time.
2. Fix and measure the board (spacing in x and y, flatness, survey targets).
3. Implement board-shape correction + uncertainty in `bevcal.py intrinsics`; extend `test_bevcal.py`.
4. Intrinsics for each real vehicle camera, one at a time (runbook Phase 4); CAM1 lens locked first.
5. Bring up `capture.py` on the Jetson; obtain surveyor export format; write survey import before the field session.

## Pitfalls (learned the hard way)

- **OpenCV segfaults on macOS via OpenCL** (crash in `clBuildProgram` loading cached kernels; affects
  `findChessboardCornersSB`). Fix in both scripts: `os.environ.setdefault("OPENCV_OPENCL_RUNTIME", "disabled")`
  **before** `import cv2`. `cv2.ocl.setUseOpenCL(False)` does NOT work (per-thread; workers still use OpenCL).
  Any new script importing cv2 needs the same line (or import `bevcal` first).
- **Don't use OpenCV's AVFoundation capture on the Mac** — it can rescale to any requested size; ffmpeg only
  accepts native modes. Query modes: `ffmpeg -f avfoundation -video_size 1x1 -framerate 5 -i "NAME:none"`
  (error lists supported modes).
- **ffmpeg duplicates frames by default** (CFR padding → frozen preview, identical saved images).
  `-fps_mode passthrough` is required (already in `MacCapture`).
- **Orphan ffmpeg keeps the camera open** if Python dies. `atexit` kill is in place, but a segfault/SIGKILL bypasses
  it: check `pgrep -fl ffmpeg-macos` and kill leftovers. macOS shows a camera-in-use preview naming the parent app.
- **Running `capture.py` from Claude's background shell works** (window appears on screen), but keys only reach
  the window when it is focused. If the window "never appears", suspect a crash, not a hidden window.
- **`cmd | grep` masks the exit code** (status is grep's); use `${pipestatus[1]}` in zsh. `os._exit()` skips
  stdout flush — use `print(..., flush=True)` in probe scripts.
- **Mac bench mode sets no UVC controls**: exposure/WB auto (geometry unaffected), focus uncontrolled → only
  fixed-focus or mechanically locked lenses may be calibrated on the Mac.
- **USB serials are not unique** (AR0234 reports `01.00.00`); identify cameras by `/dev/v4l/by-path` on the Jetson
  and by connecting one vehicle camera at a time on the Mac (the tool refuses ambiguous names).
- **Close-range blur**: views nearer than ~0.6 m were soft on the AR0234 unit; keep CAM2–6 poses at 0.6–1.5 m.
- **Operators tend to press space repeatedly on one pose** and keep the board mid-frame/far. Coverage of image
  corners and the ground band matters most; the tool now blocks repeats, but check coverage after each run.
- **Comparing two lens models pixel-by-pixel is misleading** (focal length trades off against pose); compare by
  held-out error and bootstrap ray spread after removing a common rotation.
- **Practice `intrinsics.json` is the flat-board fit**, not the corrected one, and is for a spare unit — never
  use it for the vehicle CAM2.
- **Plan §2.3 example is off**: 120° at 1920 px gives ~7.4 cm/px at 10 m by its own formula, not 6.9 cm.
- **Environment**: Python 3.10 via `uv` (system Python 3.9 lacks deps); OpenCV pinned `<5`; `imageio-ffmpeg` is
  macOS-only. JetPack 5 ships Python 3.8 — would need the pins revisited (JetPack 6 is fine).
- **Repo hygiene**: no `.gitignore`; `__pycache__/` and `.DS_Store` are committed. Add a `.gitignore`.
- **Claude Code setup changed today (/doctor)**: 36 unused skills turned off, placeholder `claude` agent disabled,
  4 claude.ai connectors disabled for this project, `~/.claude/CLAUDE.md` (graphify pointer) removed.
  Backups: `~/.claude/doctor-backup-2026-10-22/`.
