# Calibration Runbook — Real Vehicle Cameras

Step-by-step order of work for `vehicle_camera_calibration_measurement_plan.md` (the spec; § refers to it).
Items marked **[code]** are software still to write; items marked **[decision]** belong to the requirement owner (§3.3).
Lessons from the 2026-10-22 practice session (`sessions/practice01-unmounted-AR0234/README.md`) are built in.

## Phase 0 — Decisions with lead time (start now)

1. **[decision] CAM2 range and the 5 cm requirement.** A CAM2-type camera covers about 11 cm of ground per pixel at
   10 m (practice unit: f ≈ 600 px, assuming 1.5 m mount height), far beyond the 2.5 cm gate (§2.3). Proposed
   alternative: ≤ 5 cm within 0–5 m of the body outline, ≤ 1% of range beyond (10 cm at 10 m), plus an overlap
   agreement check between adjacent cameras. Decide before the field session; it changes the pass lines in §1/§11.
2. **[decision] Capture mode per camera** (§2.1), identical on the Mac and the Jetson: CAM2 1920 × 1200; CAM1 and
   CAM3–6 the largest 4:3 mode that USB bandwidth and the BEV frame rate allow. List modes on the Mac with
   `capture.py --list-mac` plus an ffmpeg mode query, on the Jetson with `v4l2-ctl --list-formats-ext`.
   Any later mode change means recalibrating (§12).
3. **Book the survey**: total station, operator, site with flat level ground to 10 m beyond the body outline (§3.1, §5).
4. **Camera inventory** (§2, §14): physical label, lens ID, USB port, chosen mode for each camera. USB serials are
   not unique (the AR0234 reports `01.00.00`), so identity on the Jetson comes from `/dev/v4l/by-path`.

## Phase 1 — Fix and measure the board

5. **Remove the twist.** The 74 mm board is twisted about 4.7 mm peak to peak. Bond the print flat to a rigid backing
   (aluminium composite panel or glass), or make a new board. Gate: ≤ 3 mm deviation (§4, §7).
6. **Measure it** (§4): square spacing along x and y separately (multi-square spans, several places, across any
   print seams); flatness with straightedge and feeler gauges on both diagonals and midlines. Record both.
7. **Survey targets**: four retro-reflective targets in the border beside O, A, B, C; caliper offsets to the inner
   corners; O/A/B/C labels; reference photo (§4, §4.1).

## Phase 2 — Software before any real capture

8. **[code] Board-shape correction in `bevcal.py intrinsics`** (`cv2.calibrateCameraRO`, rational/standard models).
   Report flat vs. corrected results side by side: on a good board they agree; disagreement flags a board problem.
9. **[code] Uncertainty in the intrinsics report**: OpenCV parameter σ and bootstrap ray-direction spread
   (as done by hand in practice), so every camera gets an accuracy figure, not just pixel RMS.
10. Test `capture.py` on the Jetson with one camera (control names, mode, preview, saving), then fix `cameras.json`.

## Phase 3 — Lens preparation, per camera (§2.2)

11. **CAM1**: set zoom and focus at the operational setting, lock the rings, paint witness marks.
12. **CAM3–6**: find out whether they autofocus (watch the preview while moving the board near and far).
    - Fixed-focus → may be calibrated on the Mac.
    - Autofocus → calibrate **on the Jetson only**, with `focus_absolute` fixed in `cameras.json`, after the
      three-power-cycle focal-length stability check. The Mac cannot lock focus.
13. **CAM2**: confirm fixed focus.
14. Cameras that will sit behind housing glass or a windshield are calibrated **through that glass** (§5).

## Phase 4 — Intrinsic calibration, one camera at a time (§8)

15. New session folder per day, e.g. `sessions/2026-MM-DD-intrinsics/`. Run `capture.py` with the camera's real ID.
16. Capture **40–60 fit poses** (minimum 30) and **10 held-out**, one save per pose:
    - CAM2–6: hand-held, 0.6–1.5 m (nearer was blurry), a few at 1.5–2 m.
    - CAM1: 4–8 m, many placements to reach every image corner, strong tilts (§8.3).
    - Cover corners and edges first, then the centre; tilt up to about 45°; whole board in frame (green 7x10).
    - Extra poses in the image region that will see the ground once mounted.
17. Fit with `bevcal.py intrinsics` using the plan's candidate models (CAM1 `k1k2 standard`, CAM2 `rational fisheye`,
    CAM3–6 `standard rational`), with board-shape correction.
18. **Proposed acceptance per camera** (project screening values, confirm with the reviewer):
    held-out RMS ≤ 0.3 px; no 3 × 3 image cell above twice the median cell; no single held-out image above 1 px RMS
    without a found cause; ray-direction uncertainty ≤ 0.03° (1σ) in the ground region. Otherwise add poses.
19. Record model choice, K, D, measured HFOV/VFOV, uncertainty (§11, §14).

## Phase 5 — Mount and set up on the vehicle

20. Mount and tighten all cameras; final optical path in place; witness marks inspected (§5).
21. Jetson: `cameras.json` with by-path devices, chosen modes and control values; check every camera's ID by
    showing it a distinctive object (§2). Confirm each runs in the mode it was calibrated in.
22. **Feasibility gate with real mount heights** (§2.3): footprint per pixel at the claimed range, using the
    *measured* focal lengths. Exceptions go to the requirement owner before the field session.

## Phase 6 — Ground targets and site layout

23. Make the ground targets: 0.6 × 0.6 m 2 × 2 checker, matte, < 2 mm thick, large painted ID (§10.2).
24. Plan the grid to give each of CAM2–6 at least 24 validation points over the bands 0–2, 2–5, 5–8, 8–10 m from
    the body outline, left/centre/right of each image (§10.1). Roughly double the count for the fit set.
25. **Assign fit vs. validation before any capture**, interleaved about 50/50, written to the manifest; never change it (§10.1).

## Phase 7 — Field session: survey (§6, §7)

26. Vehicle parked in its calibration state; record tyre pressures, load, occupants (§5).
27. Control network (≥ 4 references, redundant observations); rim-lip points (≥ 8 per wheel) and circle fits;
    ≥ 6 ground points plus the target grid; body outline; vehicle frame; sign checks.
28. Run the §7 gates on site. A failed gate means remeasure, not relax.
29. **[code] Survey import**: read the surveyor's export (format to obtain in advance), build the vehicle frame
    (`vehicle_frame`), body outline (`outline_range`) and gate report. Needed on site, so write it before the session.

## Phase 8 — Field session: extrinsic capture (§9)

30. Per camera, ≥ 6 raised board placements at two or more distances and several heights/orientations:
    survey the four board targets → capture several images (`--kind extrinsic --placement Pnn`; repeats are
    fine here) → re-check one target → check O/A/B/C order with `bevcal.py detect`.
31. Per camera, images of the ground-target field (`--kind ground`).
32. Re-inspect witness marks at the end of the session (§2.2).

## Phase 9 — Processing and acceptance

33. **[code] Ground-target picker**: click near each target's centre, refine to sub-pixel, store pixel + target ID.
34. **[code] Extrinsics CLI**: board corners (rigid fit from surveyed targets) + fit-set ground targets →
    `T_vehicle_from_camera` per camera (library: `board_in_vehicle`, `extrinsics`).
35. **[code] Validation report**: validation-set rays → ground (`ground_xy`), per-camera and per-band statistics,
    PASS / FAIL / NOT VALIDATED / BORDERLINE, CAM1 not applicable, overlap diagnostics (`evaluate`, `overlap`; §11).
36. Report per §11 and §14; archive the session unchanged. A calibration change informed by validation data
    needs a fresh validation set (§11).

## After acceptance

37. Recalibrate on any §12 trigger (remount, broken witness mark, mode/focus change, glass, suspension, impact).

## Status (2026-10-22)

- Done: plan; `bevcal.py` library (board, intrinsics, survey math, extrinsics, ground validation); `capture.py`
  (Jetson V4L2 and Mac bench mode); synthetic end-to-end test; practice intrinsics on a spare AR0234.
- Next: Phase 0 decisions, board fix (step 5), board-shape correction in code (step 8).
