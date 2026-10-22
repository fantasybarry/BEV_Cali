# Vehicle Camera Calibration Measurement Plan

**Status:** Revision 2. It follows the agreed survey-assisted workflow and includes the decisions from the 2026-10-22 plan review. Equipment availability, camera capture modes, and field measurements remain to be verified. This document does not include Python implementation or completed calibration results. Revision 1 is retained as `vehicle_camera_calibration_measurement_plan.v1.md`.

## 1 Purpose and acceptance criterion

Measure the data needed to estimate intrinsic parameters and camera-to-vehicle extrinsic parameters for the vehicle's six current cameras. The extrinsics are full 6-DoF poses, used for BEV ground projection and for other vehicle-frame uses. Use a precision survey to connect checkerboard and ground-target observations to a common vehicle coordinate frame. Validate the resulting geometry with separate surveyed ground points.

The agreed accuracy requirement is **horizontal ground-position error within 0.05 m over each camera's usable ground view out to 10 m**. For acceptance, **range is the horizontal distance from the vehicle body-outline polygon** (§6.2), so every camera's bands cover the same physical stretch of ground. The vehicle coordinate origin remains at the rear axle (§6.2). Also retain camera-to-point distances in the records.

The requirement applies only to the **recorded static vehicle state on the surveyed site**. Pitch, roll, and ride-height changes from braking, load, or road camber are tenths of a degree, far larger than the ~0.04° budget at 10 m (§7). Dynamic accuracy is out of scope. A PASS here makes no claim about accuracy while driving.

Acceptance applies to measured validation points and their documented coverage. It does not establish accuracy between all sampled points, in blind areas, above the ground, or on surfaces that depart from the surveyed ground. A camera without a visible ground region within the required range must be reported as not validated for that requirement. CAM1 is expected to have no such region and is planned as not applicable (§11).

## 2 Agreed camera configuration

The inventory comes from the slide labeled **Current Setup** in the supplied vehicle deck, plus the hardware models identified in review. Directions describe the schematic, not measured rotations. The camera-to-label pairing on the slide is inferred from label positions. Hardware specifications below come from retailer listings and a third-party datasheet, not ELP documentation. Verify them on the actual units.

| Camera ID | Viewing direction | Deck FOV | Hardware model | Sensor / shutter | Native resolution | Lens notes |
|---|---|---:|---|---|---|---|
| CAM1 | Forward, narrow | 40° | ELP-USB8MP02G-SFV | Sony IMX179, rolling | 3264 × 2448 (4:3) | Manual varifocal (listed 2.8–12 mm), set near the long end |
| CAM2 | Forward, wide | 110° | ELP-USBGS1200P01-BH120 | AR0234, global | 1920 × 1200 (16:10) | Listed "120° low distortion", ~126° HFOV |
| CAM3 | Front-left | 70° | ELP 16 MP IMX298 board, 75° lens (Amazon B0BGH2LDMP) | Sony IMX298, rolling | 4656 × 3496 (4:3) | Listed "75° no distortion"; may be autofocus |
| CAM4 | Rear-left | 70° | As CAM3 | As CAM3 | As CAM3 | As CAM3 |
| CAM5 | Front-right | 70° | As CAM3 | As CAM3 | As CAM3 | As CAM3 |
| CAM6 | Rear-right | 70° | As CAM3 | As CAM3 | As CAM3 | As CAM3 |

CAM7 belongs to alternative configurations and is outside this plan. The schematic supplies no metric camera positions. Neither the deck nor the listings say whether a FOV figure is horizontal, vertical, or diagonal, and the deck's 110° disagrees with CAM2's listed 120°. **Do not trust any FOV label.** Report measured HFOV and VFOV from the fitted intrinsics.

For each camera, record its physical label, serial number, lens identifier, USB port path, actual image dimensions, capture mode, pixel format, focus setting, exposure settings, and any image rotation or mirroring. Verify the physical-to-file mapping by observing a distinctive object in each view. Never assign permanent camera IDs from discovery order.

### 2.1 Capture mode and image format

Calibrate each camera at the **exact capture mode the BEV software will run**: the same resolution, crop, scaling, and binning. Intrinsics are valid only for that mode. Neither sensor family has a native 1920 × 1080 mode; the guide's 1080p would be a crop or rescale.

If the runtime mode is not yet decided, default to full-sensor modes without crop: 1920 × 1200 for CAM2, and a 4:3 mode for CAM1 and CAM3–6. For CAM3–6, prefer the largest 4:3 mode that USB bandwidth and the BEV frame rate allow, because the feasibility gate (§2.3) depends on it. Record the chosen mode per camera here before the session. Any later mode change requires a new calibration.

Capture calibration and validation frames as uncompressed YUY2 wherever the camera offers it at the operating mode. Still scenes tolerate a low frame rate. Where YUY2 is unavailable at that mode, which is likely for CAM1 and CAM3–6 at high resolution, use the highest-quality MJPEG setting and record it in the camera inventory.

### 2.2 Lens locking and focus stability

For every camera with a manually adjusted lens (CAM1 certainly, the others if they have threaded or varifocal lenses):

1. Set zoom and focus at the operational setting.
2. Fix the rings with adhesive or thread locker.
3. Paint a witness mark across each ring and its housing.
4. Inspect the marks at the start and end of every session. A broken mark invalidates the intrinsics.

If CAM3–6 turn out to be autofocus (voice-coil) modules, a mechanical lock is impossible. Then:

1. Disable autofocus through UVC and set a fixed `focus_absolute` value; record it.
2. Re-apply that setting after every power-up; do not rely on the camera to remember it.
3. Before calibration, capture a few board images after each of at least three power cycles and confirm the fitted focal length stays within a recorded tolerance.

A failed stability check is a hardware problem, not a calibration problem. Switch to a fixed-focus lens variant.

### 2.3 Pre-session feasibility gate

Before the field session, estimate each camera's ground footprint per pixel at the far edge of its claimed range, using its actual mount height and chosen capture mode. For a camera at height `h` and horizontal camera-to-point distance `r`, range changes per pixel approximately as `(h + r²/h) × IFOV`, where IFOV is the angle per pixel in radians.

As an illustration only, at h = 1.5 m and r = 10 m, a 70° camera 1920 px wide gives ~4.3 cm per pixel in the range direction, and a 120° camera 1920 px wide gives ~6.9 cm. At 4656 px the 70° camera improves to ~1.8 cm.

If any camera exceeds **2.5 cm per pixel** (half the 5 cm budget) within its claimed range, refer it to the requirement owner (§3.3) before the session. Possible outcomes are a different capture mode, a different mount, or an approved, documented reduction of that camera's claimed range. Do not discover the problem in the acceptance results.

## 3 Equipment and responsibilities

### 3.1 Baseline survey equipment

Arrange access to a calibrated precision total station, suitable accessories, and an experienced operator. Borrowing, rental, or an existing metrology service is sufficient; ownership is not required.

| Item | Required capability or purpose |
|---|---|
| Precision total station | Prefer a 1–2 arcsecond angular specification and approximately 1 mm + 2 ppm prism distance specification; retain the actual model, serial number, calibration evidence, and operating limitations. Reflectorless capability is needed for rim-lip points (§6.2). |
| Stable instrument support | Tripod, leveling equipment, and fixed survey control references that remain undisturbed throughout capture. |
| Matched prisms or survey targets | Known prism constant, repeatable centering, and verified offsets to the physical point being measured. |
| Board survey targets | Four adhesive retro-reflective targets on the board border (§4), with caliper-measured offsets to the inner corners. |
| Wheel-center fixtures (fallback only) | Needed only if a rim-lip circle fit fails its gate (§6.2). Include fixture offsets and uncertainty. |
| Checkerboard and stand | The single 74 mm board (§4) on a stand that holds each placement stationary at several heights and tilts. Weight or clamp it against wind. |
| Ground targets | 0.6 × 0.6 m 2 × 2 checker targets, matte, under 2 mm thick, each with a large painted ID outside the pattern (§10.2). |
| Dimensional measuring tools | Calibrated rule or caliper for multi-square spans and target offsets; straightedge and feeler gauges for board flatness. |
| Session records | Point IDs, photographs, instrument observations, timestamps, camera settings, target drawings, and uncertainty estimates. |

As one specification reference, Trimble publishes S7 variants with 1–2 arcsecond angular accuracy and a prism distance specification of 1.0 mm + 2 ppm [3]. These instrument figures do not establish the accuracy of a complete camera calibration. The operator must account for targeting, setup, target offsets, control-network geometry, and environmental effects.

A handheld range measurement alone does not determine a point's complete 3D coordinates. A laser distance meter may supplement dimensional checks, but it does not replace the survey network in this baseline workflow.

### 3.2 Conditions for using a custom jig

A custom jig may replace part of the survey only after an independent survey or equivalent metrology verifies:

1. Reference-point coordinates and offsets to the actual checkerboard corners or wheel centers.
2. Rigidity under the expected mounting load and temperature conditions.
3. Placement repeatability over at least three remove-and-reseat cycles.
4. Orientation and positional accuracy over the intended working volume.

Retain the verification report. A jig's small internal dimensional error does not establish its orientation relative to the vehicle. An unverified jig is suitable for preliminary experiments, not acceptance testing.

### 3.3 Responsibilities

The **requirement owner** is the project lead who commissioned this plan. They approve feasibility-gate exceptions (§2.3), any change to claimed range or requirement, and decisions on borderline results (§11).

The survey operator establishes the reference frame, estimates measurement uncertainty, and approves survey checks. The capture operator verifies camera identity, settings, lens witness marks, target visibility, and image sharpness. The calibration reviewer confirms target correspondence, diagnoses residuals, and evaluates independent ground validation. One person may perform multiple roles if the checks and records remain explicit.

## 4 Target preparation and identification

Use one physical checkerboard for **all intrinsic and extrinsic captures of all six cameras**. Its pattern is the supplied image, originally named `Screenshot 2026-10-05 at 11-34-39 .png`: **8 columns × 11 rows of squares**, giving **7 columns × 10 rows of inner corners** in portrait orientation. The guide's 8 × 6 inner-corner configuration does not apply.

The nominal square size is **74 mm × 74 mm**. The pattern is therefore about 0.592 × 0.814 m, and the extreme inner corners span `6s ≈ 0.444 m` by `9s ≈ 0.666 m`. Use the measured spacing, not the nominal value, in all calculations.

The board is a paper print taped onto white foam board, with a clear white border. Check that the paper lies flat and has not lifted or bubbled; re-tape it if it has, without stretching the print. If the print is assembled from several sheets, include spans that cross every seam in the measurements below.

Measure spans across several square intervals in both directions, divide by the interval count, and repeat at different places. Record measured horizontal and vertical spacing, instrument uncertainty, total board dimensions, and flatness. Reprint or correct a board with materially unequal spacing; do not silently force an anisotropic print into one square-size value. A working preparation goal is spacing agreement within 0.1%, subject to the available dimensional measurement and downstream sensitivity assessment. This is a project preparation goal, not evidence of final camera accuracy.

**Flatness.** Measure bow with a straightedge and feeler gauges along both diagonals and both midlines, before and after the session, and again if the board is used outdoors in different humidity. Minor deviation is accepted for this board and is recorded as a known error source. Any deviation exceeding 3 mm, the board rigid-fit gate in §7, must be corrected before extrinsic use. Expect a bowed board to show up as structured residuals in intrinsic fits; the reviewer must check for it.

**Survey targets.** Fix four adhesive retro-reflective survey targets in the white border, one beside each of O, A, B, and C, without covering the pattern. Measure each target center's offset to its nearest inner corner with a caliper and record it in board coordinates. The surveyor shoots these four targets for each raised placement; the inner corners follow from the measured offsets.

For CAM1's narrow view, the board covers only about 15–20% of the frame at 4–6 m, so CAM1 needs more placements to cover the image (§8). Confirm sharp, well-separated corners in each camera's preview at its operating mode before the session.

### 4.1 Board coordinate convention

The **reference orientation** is the one in which **the corner square next to O is black**, matching the source image. The pattern is not symmetric under a 180° rotation, so this orientation is identifiable from square colors alone. A photograph with a white top-left corner square shows the board rotated 180° from the reference.

Viewed from the printed face in the reference portrait orientation:

- `O = (0, 0, 0)` is the top-left inner corner.
- Board `+x` follows the top row of inner corners to the right.
- Board `+y` follows the left column of inner corners downward.
- Board `+z = +x × +y` points into the board in this viewing convention.

Let measured square spacing be `s` meters (nominal 0.074). Name the four extreme inner corners:

| Point ID | Board coordinate in meters |
|---|---|
| O | `(0, 0, 0)` |
| A | `(6s, 0, 0)` |
| B | `(0, 9s, 0)` |
| C | `(6s, 9s, 0)` |

These are extreme **inner corners**, not the outside corners of the printed paper. Their rectangular side lengths are `6s` and `9s`, with diagonal `sqrt(117) × s`.

Write O, A, B, C and axis labels on the border, matching the reference orientation, without covering the pattern. Keep a reference photograph. Every extrinsic image must have its detected corner ordering checked against this physical convention. Reject an unresolved orientation or correspondence mistake before calibration.

## 5 Site preparation and vehicle state

Use a stable, flat, level surface with space for visible targets and ground checks out to 10 m beyond the vehicle body outline. Survey the surface rather than assuming that a workshop floor or parking area is planar. NVIDIA's referenced procedure also requires a flat, level floor and ties the vehicle frame to wheel references [2].

Park the vehicle in its intended calibration state. Record tire pressures, payload, suspension/ride-height state, and whether occupants are present. Finish mounting and tightening the cameras before capture. Keep the vehicle, mounts, and suspension state unchanged between extrinsic measurements and validation.

Keep the final optical path in place, including housing glass or windshield where applicable. Clean the lenses. Warm up the cameras, set the capture mode (§2.1), apply the focus settings (§2.2), and inspect the witness marks. Record any setting that cannot be locked. Auto exposure, auto white balance, autofocus, stabilization, electronic warping, cropping, and automatic image rotation must be disabled or fixed and documented.

Use original camera images with a known processing history. Record whether images have already been rectified or corrected by the camera; do not treat corrected images as raw distorted images. Verify actual resolution and pixel format on saved files.

## 6 Establish the survey and vehicle coordinate frames

### 6.1 Establish survey control

1. Set up and check the instrument according to its operating procedure. Record prism constants, target offsets, atmospheric corrections where applicable, and units.
2. Establish at least four stable control references distributed around the vehicle, with a redundant reference beyond the minimum needed for a setup solution. Use additional control when sight lines are blocked.
3. Observe controls redundantly and preserve original observations. Where supported, use face-I/face-II observations and independent instrument setups to expose setup or aiming errors.
4. If the instrument must move, re-establish its pose from the common network and check unused control points. Never combine local coordinate systems without a verified transformation.

### 6.2 Derive the vehicle frame and body outline

**Wheel centers.** For each of the four wheels, survey at least eight reflectorless points spread around the rim lip. Fit a 3D circle; its center is the wheel-center reference. If a rim's shape does not support a clean circle, or the fit residual exceeds 2 mm, fall back to verified locating fixtures for that wheel and correct their offsets. Record the method used per wheel.

**Ground.** Survey at least six ground points distributed around the vehicle and across the measurement region, in addition to the ground-target grid (§10).

Use the surveyed level ground plane to define upward `+Z`. Project the vector from the rear wheel-center midpoint toward the front wheel-center midpoint onto that plane to define forward `+X`. Define left `+Y = +Z × +X`, and orthonormalize the axes. The origin is the rear wheel-center midpoint projected onto the ground plane.

This defines a geometric vehicle centerline from the axle references. Document axle offset, damage, unusual wheel geometry, or an existing manufacturer frame that would require a different physical reference.

**Body outline.** Survey the outermost fixed points of the body: bumpers, sills, body corners, and any fixed sensors. Exclude mirrors, which fold. Project them onto the ground plane and record the resulting polygon in vehicle coordinates. Acceptance range (§1) is the horizontal distance from this polygon.

Retain both the survey-frame coordinates and the survey-to-vehicle transformation. Check direction signs with known points: a point ahead should have positive X and a point to the left positive Y. A point on the reference plane should have Z near zero.

The camera optical frame will use `x` right, `y` down, `z` forward. Name transformations explicitly: `T_vehicle_from_camera` maps camera coordinates to vehicle coordinates, and `T_camera_from_vehicle` is its inverse. Use meters for all translations.

## 7 Verify the measurement system before camera calibration

The following are project screening gates, not manufacturer guarantees or a statistical proof of total uncertainty. The survey operator must assess correlated errors and provide an uncertainty estimate for the reference data.

| Check | Working gate and action |
|---|---|
| Repeated fixed-point observations | Investigate coordinate differences exceeding 3 mm in any component. |
| Check points from a second setup | Investigate 3D discrepancies exceeding 5 mm. |
| Known target spans and diagonals | Investigate disagreement exceeding 3 mm or the independently measured dimensional tolerance, whichever requires closer examination. |
| Rigid fit of surveyed board targets | Investigate any target residual exceeding 3 mm; do not average away a bad target. |
| Rim-lip circle fits | Fall back to fixtures for any wheel whose fit residual exceeds 2 mm. |
| Ground-plane checks | Investigate deviations exceeding 3 mm over the intended validation area; use additional independent points to check the plane. If the floor genuinely fails this gate, follow §10.2. |
| Independent validation references | Aim for horizontal expanded uncertainty no greater than 5 mm at approximately 95% coverage, including frame and targeting contributions. Record the actual estimate and method. |

A failed gate requires diagnosis and remeasurement. Do not relax a gate silently to accept an inconsistent dataset. A small residual or good repeatability can coexist with a systematic offset; known-length checks, independent setups, and target verification address different failure modes.

**Orientation sensitivity matters.** As an illustration, for a camera 1.5 m above a planar ground surface, ground range changes with a small pitch error approximately as `Δr ≈ (h + r²/h) × Δθ`, where the angle is in radians. For this local illustration, r is horizontal camera-to-point distance, distinct from the body-outline range used for acceptance. At r = 10 m, about 0.042° of pitch error alone consumes 5 cm. This calculation assumes fixed camera position and a planar ground surface; 1.5 m is an example, not a measured vehicle height. It explains why widely distributed measurements and independent ground checks are necessary.

## 8 Capture intrinsic calibration images

Perform this separately for each camera at its operating mode (§2.1) and locked focus (§2.2). Board-to-vehicle surveying is unnecessary for these images because the board may move freely between exposures.

1. Aim for at least 30 sharp, varied images for fitting, plus at least 10 additional views reserved for checking intrinsic-model behavior. Increase the count if coverage is poor; count alone is not an acceptance test.
2. Cover the center, all edges and corners, varied board scales, and both horizontal and vertical tilts. Keep the full 7 × 10 inner-corner grid visible for the standard checkerboard workflow.
3. Use distances compatible with focus and readable corner spacing. For CAM2–6, about 0.5–2 m is a starting range. CAM1 is focused far (hyperfocal distance roughly 7 m, sharp from about 3.5 m), so capture it at about 4–8 m. It needs more placements to reach every image corner, and strong tilts to constrain its principal point.
4. Stop board motion for each exposure. Reject blur, glare, clipped highlights, severe shadow, cropping, or ambiguous detections. Preserve rejected files and the rejection reason separately from accepted observations.
5. Save original-resolution images and retain exposure, focus, mode, and pixel-format metadata. The camera may remain mounted while the board moves, provided the needed image regions are accessible.

NVIDIA's reference procedure recommends at least 30 intrinsic views with broad image coverage and varied tilts and distances [2]. The extra held-out views above are this project's validation provision.

Fit these candidate models and choose per camera by held-out image behavior [4]:

| Camera | Candidate models | Watch for |
|---|---|---|
| CAM1 (40°) | Standard model with k1, k2 only; standard 5-coefficient | Overfitting and principal-point drift from the long focal length |
| CAM2 (120°) | Rational model; fisheye (Kannala-Brandt) | Edge behavior at wide angles |
| CAM3–6 (70–75°) | Standard 5-coefficient; rational model | Structured residuals from board bow (§4) |

Report reprojection RMS, the spatial distribution of residuals, and measured HFOV/VFOV. Good pixel RMS alone does not certify the 5 cm vehicle-frame requirement.

## 9 Measure and capture extrinsic data

Keep all intrinsic settings and mounts fixed. Each camera's 6-DoF pose is fitted from two kinds of surveyed observations:

- **Raised board placements.** At least six stationary placements per camera, with at least two distances and several view directions and board orientations. Include multiple heights and target depths so the observed 3D corners are well distributed. Keep CAM2's raised placements within about 6 m, where its squares remain about 10 px or larger. Raised points help separate pitch from mounting height.
- **Ground targets from the fit set** (§10), which constrain exactly the ground mapping that acceptance tests, including the far band.

These counts are a starting acquisition minimum; inadequate geometry requires additional placements.

No camera-to-camera overlap is required: each camera is independently tied to the same surveyed vehicle frame. Shared observations are reported as overlap diagnostics (§11).

For every raised placement:

1. Assign a unique placement ID. Photograph the arrangement and visible labels.
2. Hold the board rigidly in place. Survey the four retro-reflective border targets (§4).
3. Correct the instrument's prism or target constant and each target's measured offset to its inner corner. A target center is not automatically a printed corner. Do not assume a reflectorless reading on a black/white boundary is an accurate corner observation.
4. Derive the board-to-vehicle rigid pose from the measured board dimensions and surveyed targets. Retain target residuals and the original measurements. Infer other corner coordinates from this rigid geometry only after its checks pass.
5. Capture several sharp images while the board and vehicle remain stationary. Repeated images of one placement help assess repeatability but do not count as new spatial placements.
6. Re-check at least one target after capture. If the board moved, invalidate that placement and remeasure it.
7. Verify detected O/A/B/C ordering in an annotated image before accepting the placement.

The board's inner-corner span is only 0.44 × 0.67 m, so each individual board orientation is weak: about 0.1° per millimeter of survey noise. That does not break the method. The camera pose is estimated from all corners and ground targets together, spread over meters, rather than from any single board pose.

Hold focus, exposure timing, and the scene static for each capture. Hardware synchronization is not required for separate static observations, but record timestamps and confirm there was no motion. CAM1 and CAM3–6 have rolling shutters, which is harmless for still scenes. This does not establish suitability for synchronized moving-scene BEV operation.

## 10 Ground targets: fitting and independent validation

### 10.1 Grid and fit/validation split

Survey all ground targets as one grid. **Before any capture**, assign each target to either the **fit set** (used in §9) or the **validation set**, and record the assignment in the validation manifest. Interleave the two sets roughly 50/50 across the whole area rather than giving them separate regions. Do not change the assignment after fitting begins. Validation points can be shared across cameras when each camera sees them, but must remain excluded from all calibration fits.

Aim for at least 24 validation observations per camera for CAM2–6. CAM1 is excluded from this count (§11). Use the following planning bands, measured from the body-outline polygon, with approximately six validation points per observable band and coverage across left, center, and right portions of the image:

| Horizontal range from body outline | Intended coverage |
|---|---|
| 0–2 m | Near ground where visible |
| Greater than 2–5 m | Intermediate ground |
| Greater than 5–8 m | Farther ground |
| Greater than 8–10 m | Boundary region, including visible points near 10 m |

Respect the actual camera's visible region. Record unavailable bands, masks, occlusions, and the actual tested range; do not invent near-ground coverage for a camera that cannot see it. A claim extending to 10 m needs visible checks in the far band, including near the outer boundary. A camera with insufficient coverage is **NOT VALIDATED** for the full claimed region even if its available points have small errors.

### 10.2 Ground target design and observations

Each ground target is a flat 0.6 × 0.6 m 2 × 2 checker on thin matte vinyl or mat, under 2 mm thick, with a large painted ID beside it outside the pattern. The surveyed point is the center saddle point, marked for the prism pole or a reflectorless shot. The saddle point can be located to sub-pixel accuracy; even at 10 m on CAM2 the target is still about 9 px across in the range direction.

Account for target thickness; the image feature must correspond to the surveyed point. Treat points on ramps, uneven ground, or raised objects separately from planar-ground acceptance.

Record each observed pixel location on the original image and its localization uncertainty. Use independent review or repeated localization for difficult features. A 5 cm physical criterion cannot be resolved reliably if the target is blurred or only a few indistinct pixels wide.

Later evaluation will undistort each pixel with the selected lens model, form its camera ray, transform the ray into the vehicle frame, and intersect it with the ground. Reject rays directed away from the ground or too close to parallel for stable evaluation. Record these rejections and coverage consequences.

**Non-flat floor.** If the floor fails the ground-plane gate (§7), report two results:

- **Primary (calibration accuracy):** intersect each ray at that validation point's own surveyed height.
- **Secondary (BEV planar assumption):** intersect with the single fitted plane, and report the additional error it introduces.

This keeps floor shape from being counted as calibration error, while still showing what a planar BEV will see.

## 11 Acceptance and reporting

For each valid independent ground point, calculate:

`e_XY = sqrt((X_estimated - X_surveyed)² + (Y_estimated - Y_surveyed)²)`

Report each point's ID, camera ID, range from body outline, camera-to-point distance, surveyed coordinates, pixel coordinates, estimated ground coordinates, error, and reference uncertainty. Summarize count, median, RMS, 95th percentile, and maximum horizontal error per camera and per distance band. The percentile is descriptive; it does not replace the maximum-error requirement.

Use these result categories:

- **PASS at sampled locations:** all valid independent points have `e_XY ≤ 0.05 m`, the planned usable-region coverage is adequately represented, and reference uncertainty is reported and sufficiently small to make the result meaningful.
- **FAIL:** one or more valid independent points exceed 0.05 m. Preserve the failure and diagnose it.
- **NOT VALIDATED:** insufficient coverage, unresolved target correspondence, failed survey checks, excessive reference uncertainty, or no suitable visible ground region prevents an accuracy conclusion.
- **NOT APPLICABLE (planned):** CAM1. Its narrow forward view is not expected to see ground within 10 m of the body outline. Report its intrinsic and extrinsic diagnostics. If it does see ground targets, report their errors as diagnostics only, not acceptance.

A result close to the 5 cm boundary must be reviewed against combined measurement uncertainty and referred to the requirement owner. Do not call a borderline result a confident pass when its uncertainty crosses the limit. Report the observed result and uncertainty; collect better measurements if a defensible decision is needed.

**Overlap diagnostics.** For validation points seen by two adjacent cameras, report the horizontal difference between the two cameras' estimates. Two cameras can each be within 5 cm and still disagree by up to 10 cm, which shows up as BEV seams. These are diagnostics, not an acceptance gate, unless the BEV consumer later requires one.

If validation data informs a calibration change, it has become development data. Collect a fresh independent validation set before claiming acceptance. Preserve the previous result and explain the change.

A six-camera acceptance report must include all six camera IDs, including any failed, unvalidated, or not-applicable camera. Do not imply complete 360° ground coverage from the schematic or from successful individual camera checks.

## 12 Recalibration triggers

A calibration is invalid, and the affected camera must be recalibrated, after any of:

- A camera removed, remounted, or its mount adjusted.
- A broken lens witness mark.
- A change of capture mode, pixel format affecting geometry, or focus setting.
- A failed autofocus stability check (§2.2).
- Replacement of camera glass, housing, or windshield in the optical path.
- A suspension, ride-height, or tire-size change (extrinsics of all cameras).
- An impact near a camera mount.

No periodic recalibration schedule is set. Add one only if field data shows drift.

## 13 Capture tooling requirements

Do **not** use the guide's `camera_calibration.py` as-is. Its `(8, 6)` corners, 25 mm squares, 20-image capture, and discovery-order camera numbering (`FIRST_OUTPUT_CAMERA_NUMBER = 4`) all conflict with this plan. Any capture tool used for the session must:

1. Detect 7 × 10 inner corners and take the measured square spacing as input, not a hard-coded value.
2. Support at least 40 intrinsic images per camera (30 fit + 10 held out), with manual trigger per pose rather than a fixed timer.
3. Assign camera IDs from a fixed mapping file keyed by serial number or USB port path, never discovery order.
4. Set and verify the capture mode and pixel format (§2.1), and disable auto exposure, auto white balance, and autofocus through UVC, re-applying them after every power-up.
5. Save frames losslessly at original resolution, with a metadata sidecar per image: camera ID, mode, format, focus and exposure values, timestamp, and placement ID.
6. Never delete or overwrite earlier images; write each session to a new folder.
7. Capture one camera at a time if USB bandwidth causes dropped or corrupted frames.

This section states requirements only. Implementation is outside this document.

## 14 Measurement records to retain

| Record | Minimum contents |
|---|---|
| Session record | Session ID, operators, requirement owner, date/time/timezone, site, vehicle state, equipment IDs, calibration evidence, and conditions. |
| Camera inventory | Physical ID, model, serial/USB-port mapping, lens, capture mode, pixel format and compression setting, focus/exposure, autofocus status and stability check, witness-mark inspections, processing history, and mounting photograph. |
| Feasibility record | Per camera: mount height, mode, IFOV, ground footprint per pixel at claimed range, and any approved exception. |
| Board record | Board ID, source pattern, 7 × 10 inner corners, measured spacing, flatness before and after, survey-target offsets, reference labels, dimensions, and uncertainty. |
| Survey network | Raw observations, units, station IDs, control/check classification, prism constants, target offsets, residuals, and uncertainty estimates. |
| Vehicle references | Rim-lip points and circle fits (or fixture data), floor observations, plane fit/checks, body-outline polygon, frame construction, and survey-to-vehicle transformation. |
| Intrinsic image manifest | Camera ID, file path, capture time, accepted/rejected status, held-out status, and image settings. |
| Extrinsic placement manifest | Placement ID, images, surveyed targets, rigid fit, pre/post movement check, and corner-order approval. |
| Ground target manifest | Target ID, surveyed XYZ, fit/validation assignment (locked before capture), camera/image IDs, pixel coordinates, range from body outline, and uncertainty. |
| Results report | Intrinsic diagnostics and model choice, measured FOV, both transform directions, ground errors and coverage, overlap diagnostics, non-flat-floor results if applicable, failures/rejections, and final status per camera. |

Keep original files and survey observations immutable within the session archive. Use separate folders for intrinsic fitting, intrinsic checks, extrinsic fitting, independent ground validation, and results. A new session or changed camera setting must not silently reuse prior images.

## 15 Field readiness checklist

- [ ] Total station and experienced operator are available; instrument calibration and suitable accessories are confirmed.
- [ ] Camera hardware is verified against §2; capture mode and pixel format are fixed per camera.
- [ ] Feasibility gate (§2.3) is computed for every camera; exceptions are approved by the requirement owner.
- [ ] Lenses are locked with witness marks; CAM3–6 autofocus status is known and, if AF, the power-cycle stability check passes.
- [ ] The 74 mm board is measured, flatness recorded, labeled in the reference orientation, and fitted with four offset-measured survey targets.
- [ ] Ground targets are made, laid out, surveyed, and assigned to fit or validation sets before capture.
- [ ] Site ground and survey-control checks pass; rim-lip fits pass; the vehicle frame is established and sign-checked; the body outline is surveyed.
- [ ] All six physical cameras map to the correct saved-file identities.
- [ ] A capture tool meeting §13 is available and tested.
- [ ] Intrinsic fitting and held-out views have adequate image coverage.
- [ ] Extrinsic placements have verified survey targets, corner ordering, and movement checks.
- [ ] Validation ground points cover the usable regions of CAM2–6 and include checks near 10 m where visible.
- [ ] Survey uncertainty and acceptance-report responsibilities are agreed before evaluating results.

The plan is finalized, but the field session is not ready until these prerequisites are verified. Measured square spacing, equipment access, survey checks, capture modes, mount heights, and captured data are deliberately recorded as field inputs rather than invented values. No Python implementation is authorized by this document.

## 16 Sources and basis

1. **User-supplied materials:** `MiLa_CAM_setup(1).pptx` (Current Setup slide; local counterpart originally named `MiLa_CAM_setup.pptx`), `Camera_Calibration_Guide_Google_Docs.docx`, `Screenshot 2026-10-05 at 11-34-39 .png`, and a photograph of the physical board. These establish the camera schematic, initial guide, selected checkerboard pattern, and board construction.
2. **NVIDIA DriveWorks Static Camera Calibration Tutorial:** [Official tutorial](https://developer.nvidia.com/docs/drive/drive-os/6.0.5/public/driveworks-nvsdk/dwx_camera_calibration.html). Reference for rigid targets, vehicle wheel references, a level floor, and diverse intrinsic images. Its AprilTag/external-camera workflow is background; this plan uses the user's checkerboard with surveyed geometry.
3. **Trimble S7 manufacturer specifications:** [Official product specifications](https://geospatial.trimble.com/en/products/hardware/trimble-s7). Equipment-class reference, not a claim that a particular instrument has been obtained or field-verified.
4. **OpenCV camera calibration documentation:** [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html) and [Fisheye Camera Model](https://docs.opencv.org/4.13.0/db/d58/group__calib3d__fisheye.html). Reference for camera geometry and separate distortion-model conventions.
5. **Camera hardware listings (secondhand, unverified):** [IPX USB8MP02G-SFV datasheet](https://global-uploads.webflow.com/606b5fcccfa449439a69ba6f/628f8d1fe7687cca7c5f228e_IPX-%20USB8MP02G-SFV.pdf), [ELP global-shutter 120° listing](https://www.newegg.com/p/3C6-01BP-003J1), [Amazon B0BGH2LDMP](https://www.amazon.com/dp/B0BGH2LDMP), and [IMX298 module listing](https://www.globalsources.com/product/usb-camera-module_1201809935f.htm).

The 5 cm requirement, static-state scope, six-camera scope, vehicle origin, body-outline range, camera hardware, selected board and its 74 mm squares, survey-assisted approach, and independent validation were agreed with the user. Acquisition counts, survey screening gates, the feasibility threshold, validation layout, and uncertainty handling are the engineering procedure specified by this plan; they are not quoted manufacturer acceptance limits.
