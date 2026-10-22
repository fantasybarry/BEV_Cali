# Vehicle Camera Calibration Measurement Plan

**Status:** Final measurement plan following the agreed survey-assisted workflow. Equipment availability and field measurements remain to be verified. This document does not include Python implementation or completed calibration results.

## 1 Purpose and acceptance criterion

Measure the data needed to estimate intrinsic parameters and camera-to-vehicle extrinsic parameters for the vehicle's six current cameras. Use a precision survey to connect checkerboard observations to a common vehicle coordinate frame. Validate the resulting geometry with separate surveyed ground points.

The agreed accuracy requirement is **horizontal ground-position error within 0.05 m over each camera's usable ground view out to 10 m**. For this plan, range is the horizontal distance from the vehicle origin, `r = sqrt(X² + Y²)`. Also retain camera-to-point distances in the records.

Acceptance applies to measured validation points and their documented coverage. It does not establish accuracy between all sampled points, in blind areas, above the ground, or on surfaces that depart from the surveyed ground plane. A camera without a visible ground region within the required range must be reported as not validated for that requirement.

## 2 Agreed camera configuration

The following inventory comes from the slide labeled **Current Setup** in the supplied vehicle deck. Directions describe the schematic, not measured rotations.

| Camera ID | Viewing direction | Labeled field of view |
|---|---|---:|
| CAM1 | Forward, narrow view | 40° |
| CAM2 | Forward, wide view | 110° |
| CAM3 | Front-left | 70° |
| CAM4 | Rear-left | 70° |
| CAM5 | Front-right | 70° |
| CAM6 | Rear-right | 70° |

CAM7 belongs to alternative configurations and is outside this plan. The current schematic suggests paired mounting areas, but it supplies no metric camera positions. It does not identify whether the labeled FOV is horizontal, vertical, or diagonal.

For each camera, record its physical label, serial number, lens identifier, capture channel, actual image dimensions, focus setting, exposure settings, and any image rotation or mirroring. Verify the physical-to-file mapping by observing a distinctive object in each view. Never assign permanent camera IDs from discovery order.

Camera/lens models and actual operating resolutions are not yet established. The guide's requested 1920 × 1080 is not proof of the cameras' delivered resolution. FOV alone is insufficient to choose a pinhole or fisheye model.

## 3 Equipment and responsibilities

### 3.1 Baseline survey equipment

Arrange access to a calibrated precision total station, suitable accessories, and an experienced operator. Borrowing, rental, or an existing metrology service is sufficient; ownership is not required.

| Item | Required capability or purpose |
|---|---|
| Precision total station | Prefer a 1–2 arcsecond angular specification and approximately 1 mm + 2 ppm prism distance specification; retain the actual model, serial number, calibration evidence, and operating limitations. |
| Stable instrument support | Tripod, leveling equipment, and fixed survey control references that remain undisturbed throughout capture. |
| Matched prisms or survey targets | Known prism constant, repeatable centering, and verified offsets to the physical point being measured. |
| Wheel-center locating fixtures | Repeatably identify hub/axle centers and relate them to survey targets. Include fixture offsets and uncertainty. |
| Rigid checkerboard and support | Flat, matte, dimensionally stable board on a stand that can hold each placement stationary. |
| Dimensional measuring tools | Suitable calibrated rule or caliper to measure multi-square spans and reference offsets. |
| Independent check targets | Clearly identifiable points reserved for survey checks and camera validation. |
| Session records | Point IDs, photographs, instrument observations, timestamps, camera settings, fixture drawings, and uncertainty estimates. |

As one specification reference, Trimble publishes S7 variants with 1–2 arcsecond angular accuracy and a prism distance specification of 1.0 mm + 2 ppm [3]. These instrument figures do not establish the accuracy of a complete camera calibration. The operator must account for targeting, setup, fixture offsets, control-network geometry, and environmental effects.

A handheld range measurement alone does not determine a point's complete 3D coordinates. A laser distance meter may supplement dimensional checks, but it does not replace the survey network in this baseline workflow.

### 3.2 Conditions for using a custom jig

A custom jig may replace part of the survey only after an independent survey or equivalent metrology verifies:

1. Reference-point coordinates and offsets to the actual checkerboard corners or wheel centers.
2. Rigidity under the expected mounting load and temperature conditions.
3. Placement repeatability over at least three remove-and-reseat cycles.
4. Orientation and positional accuracy over the intended working volume.

Retain the verification report. A jig's small internal dimensional error does not establish its orientation relative to the vehicle. An unverified jig is suitable for preliminary experiments, not acceptance testing.

### 3.3 Responsibilities

The survey operator establishes the reference frame, estimates measurement uncertainty, and approves survey checks. The capture operator verifies camera identity, settings, target visibility, and image sharpness. The calibration reviewer confirms target correspondence, diagnoses residuals, and evaluates independent ground validation. One person may perform multiple roles if the checks and records remain explicit.

## 4 Target preparation and identification

Use the supplied checkerboard image, originally named `Screenshot 2026-10-05 at 11-34-39 .png`. It has **8 columns × 11 rows of squares**, giving **7 columns × 10 rows of inner corners** in portrait orientation. The guide's 8 × 6 inner-corner configuration does not apply to this target.

Print at a size that produces sharp, well-separated corners in the actual camera images. The same pattern may be printed at multiple physical sizes; give every physical board its own ID and measured dimensions. Select the size by a camera preview at the intended focus distances, including CAM1's narrow view and CAM2's wide view. No physical square size can be recovered from the screenshot alone.

Mount the print on a flat, rigid, matte backing. Preserve the aspect ratio, include a clear surrounding border, and check the mounted board for bowing. Do not stretch the print during mounting.

Measure spans across several square intervals in both directions, divide by the interval count, and repeat at different places. Record measured horizontal and vertical spacing, instrument uncertainty, total board dimensions, and flatness. Reprint or correct a board with materially unequal spacing; do not silently force an anisotropic print into one square-size value. A working preparation goal is spacing agreement within 0.1%, subject to the available dimensional measurement and downstream sensitivity assessment. This is a project preparation goal, not evidence of final camera accuracy.

### 4.1 Board coordinate convention

Viewed from the printed face in the reference portrait orientation:

- `O = (0, 0, 0)` is the top-left inner corner.
- Board `+x` follows the top row of inner corners to the right.
- Board `+y` follows the left column of inner corners downward.
- Board `+z = +x × +y` points into the board in this viewing convention.

Let measured square spacing be `s` meters. Name the four extreme inner corners:

| Point ID | Board coordinate in meters |
|---|---|
| O | `(0, 0, 0)` |
| A | `(6s, 0, 0)` |
| B | `(0, 9s, 0)` |
| C | `(6s, 9s, 0)` |

These are extreme **inner corners**, not the outside corners of the printed paper. Their rectangular side lengths are `6s` and `9s`, with diagonal `sqrt(117) × s`.

Put O, A, B, C and axis labels on the surrounding border without covering the pattern. Keep a reference photograph. Every extrinsic image must have its detected corner ordering checked against this physical convention. Reject an unresolved orientation or correspondence mistake before calibration.

## 5 Site preparation and vehicle state

Use a stable, flat, level surface with space for visible targets and ground checks out to 10 m from the vehicle origin. Survey the surface rather than assuming that a workshop floor or parking area is planar. NVIDIA's referenced procedure also requires a flat, level floor and ties the vehicle frame to wheel references [2].

Park the vehicle in its intended calibration state. Record tire pressures, payload, suspension/ride-height state, and whether occupants are present. Finish mounting and tightening the cameras before capture. Keep the vehicle, mounts, and suspension state unchanged between extrinsic measurements and validation.

Keep the final optical path in place, including housing glass or windshield where applicable. Clean the lenses. Warm up the cameras, focus at the operational setting, and lock focus/zoom where possible. Record any setting that cannot be locked. Stabilization, electronic warping, cropping, and automatic image rotation must be disabled or fixed and documented.

Use original camera images with a known processing history. Record whether images have already been rectified or corrected by the camera; do not treat corrected images as raw distorted images. Verify actual resolution on saved files.

## 6 Establish the survey and vehicle coordinate frames

### 6.1 Establish survey control

1. Set up and check the instrument according to its operating procedure. Record prism constants, target/fixture offsets, atmospheric corrections where applicable, and units.
2. Establish at least four stable control references distributed around the vehicle, with a redundant reference beyond the minimum needed for a setup solution. Use additional control when sight lines are blocked.
3. Observe controls redundantly and preserve original observations. Where supported, use face-I/face-II observations and independent instrument setups to expose setup or aiming errors.
4. If the instrument must move, re-establish its pose from the common network and check unused control points. Never combine local coordinate systems without a verified transformation.

### 6.2 Derive the vehicle frame

Survey the front-left, front-right, rear-left, and rear-right wheel-center references using verified locating fixtures. Correct fixture offsets to obtain the actual intended hub/axle reference points. Also survey at least six ground points distributed around the vehicle and across the measurement region.

Use the surveyed level ground plane to define upward `+Z`. Project the vector from the rear wheel-center midpoint toward the front wheel-center midpoint onto that plane to define forward `+X`. Define left `+Y = +Z × +X`, and orthonormalize the axes. The origin is the rear wheel-center midpoint projected onto the ground plane.

This defines a geometric vehicle centerline from the axle references. Document axle offset, damage, unusual wheel geometry, or an existing manufacturer frame that would require a different physical reference.

Retain both the survey-frame coordinates and the survey-to-vehicle transformation. Check direction signs with known points: a point ahead should have positive X and a point to the left positive Y. A point on the reference plane should have Z near zero.

The camera optical frame will use `x` right, `y` down, `z` forward. Name transformations explicitly: `T_vehicle_from_camera` maps camera coordinates to vehicle coordinates, and `T_camera_from_vehicle` is its inverse. Use meters for all translations.

## 7 Verify the measurement system before camera calibration

The following are project screening gates, not manufacturer guarantees or a statistical proof of total uncertainty. The survey operator must assess correlated errors and provide an uncertainty estimate for the reference data.

| Check | Working gate and action |
|---|---|
| Repeated fixed-point observations | Investigate coordinate differences exceeding 3 mm in any component. |
| Check points from a second setup | Investigate 3D discrepancies exceeding 5 mm. |
| Known target spans and diagonals | Investigate disagreement exceeding 3 mm or the independently measured dimensional tolerance, whichever requires closer examination. |
| Rigid fit of surveyed board controls | Investigate any control residual exceeding 3 mm; do not average away a bad control point. |
| Ground-plane checks | Investigate deviations exceeding 3 mm over the intended validation area; use additional independent points to check the plane. |
| Independent validation references | Aim for horizontal expanded uncertainty no greater than 5 mm at approximately 95% coverage, including frame and targeting contributions. Record the actual estimate and method. |

A failed gate requires diagnosis and remeasurement. Do not relax a gate silently to accept an inconsistent dataset. A small residual or good repeatability can coexist with a systematic offset; known-length checks, independent setups, and fixture verification address different failure modes.

**Orientation sensitivity matters.** As an illustration, for a camera 1.5 m above a planar ground surface, ground range changes with a small pitch error approximately as `Δr ≈ (h + r²/h) × Δθ`, where the angle is in radians. For this local illustration, r is horizontal camera-to-point distance, distinct from the vehicle-origin range used for acceptance. At r = 10 m, about 0.042° of pitch error alone consumes 5 cm. This calculation assumes fixed camera position and a planar ground surface; 1.5 m is an example, not a measured vehicle height. It explains why widely distributed measurements and independent ground checks are necessary.

## 8 Capture intrinsic calibration images

Perform this separately for each camera using its final operational optical settings. Board-to-vehicle surveying is unnecessary for these images because the board may move freely between exposures.

1. Aim for at least 30 sharp, varied images for fitting, plus at least 10 additional views reserved for checking intrinsic-model behavior. Increase the count if coverage is poor; count alone is not an acceptance test.
2. Cover the center, all edges and corners, varied board scales, and both horizontal and vertical tilts. Keep the full 7 × 10 inner-corner grid visible for the standard checkerboard workflow.
3. Use distances compatible with focus and readable corner spacing. If the board is too small at the required focus distance, use a larger physical print with its own measured spacing.
4. Stop board motion for each exposure. Reject blur, glare, clipped highlights, severe shadow, cropping, or ambiguous detections. Preserve rejected files and the rejection reason separately from accepted observations.
5. Save original-resolution images and retain exposure/focus metadata. The camera may remain mounted while the board moves, provided the needed image regions are accessible.

NVIDIA's reference procedure recommends at least 30 intrinsic views with broad image coverage and varied tilts and distances [2]. The extra held-out views above are this project's validation provision.

Later analysis will select the camera model per camera using lens information and held-out image behavior. Report reprojection RMS and the spatial distribution of residuals. Good pixel RMS alone does not certify the 5 cm vehicle-frame requirement.

## 9 Measure and capture extrinsic target placements

Keep all camera intrinsics settings and mounts fixed. For each camera, plan at least six stationary checkerboard placements, with at least two distances and several view directions and board orientations. Include multiple heights and target depths so the complete set of observed 3D corners is well distributed. This is a starting acquisition minimum; inadequate geometry requires additional placements.

No camera-to-camera overlap is required: each camera is independently tied to the same surveyed vehicle frame. Shared observations are useful checks where overlap exists.

For every placement:

1. Assign a unique placement ID and physical board ID. Photograph the arrangement and visible labels.
2. Hold the board rigidly in place. Survey O, A, B, C, or survey rigid fiducials whose offsets to these corners are independently known.
3. When using prisms, correct both the instrument's prism constant and the fixture's geometric offset. A prism center is not automatically a printed corner. Reflectorless observations require verified spot placement and surface behavior; do not assume a reading on a black/white boundary is an accurate corner observation.
4. Derive the board-to-vehicle rigid pose using its measured dimensions and surveyed controls. Retain control residuals and the original measurements. Infer other corner coordinates from this rigid geometry only after its checks pass.
5. Capture several sharp images while the board and vehicle remain stationary. Repeated images of one placement help assess repeatability but do not count as new spatial placements.
6. Check at least one control observation after capture. If the board moved, invalidate that placement and remeasure it.
7. Verify detected O/A/B/C ordering in an annotated image before accepting the placement.

For a small board, measurement noise in a short baseline can produce a large orientation error. Increase the target size or use wider-spaced rigid reference fiducials when necessary, verifying their offsets and rigidity. Combine observations from the distributed placements to estimate one fixed camera-to-vehicle pose rather than relying on a single small board pose.

Hold focus, exposure timing, and the scene static for each capture. Hardware synchronization is not required for separate static observations, but record timestamps and confirm there was no motion. This does not establish suitability for synchronized moving-scene BEV operation.

## 10 Capture independent ground validation data

Reserve validation targets, placements, and surveyed point IDs before fitting the extrinsics. Do not use them to choose or refine the calibration being evaluated. Points can be shared across cameras when each camera sees them, but must remain excluded from all calibration fits.

### 10.1 Coverage layout

Aim for at least 24 validation observations per camera, distributed across the physically visible ground region. Use the following planning bands, with approximately six points per observable band and coverage across left, center, and right portions of the image:

| Horizontal range from vehicle origin | Intended coverage |
|---|---|
| 0–2 m | Near ground where visible and outside the vehicle footprint |
| Greater than 2–5 m | Intermediate ground |
| Greater than 5–8 m | Farther ground |
| Greater than 8–10 m | Boundary region, including visible points near 10 m |

Respect the actual camera's visible region. Record unavailable bands, masks, occlusions, and the actual tested range; do not invent near-ground coverage for a camera that cannot see it. A claim extending to 10 m needs visible checks in the far band, including near the outer boundary. A camera with insufficient coverage is **NOT VALIDATED** for the full claimed region even if its available points have small errors.

### 10.2 Ground reference and image observations

Survey the physical centers or selected corners of clearly identifiable, thin ground targets. Account for target thickness or marker offsets; the selected image feature must correspond to the surveyed point. Verify actual ground height against the reference plane. Treat points on ramps, uneven ground, or raised objects separately from planar-ground acceptance.

Record each observed pixel location on the original image and its localization uncertainty. Use independent review or repeated localization for difficult features. A 5 cm physical criterion cannot be resolved reliably if the target is blurred or only a few indistinct pixels wide.

Later evaluation will undistort each pixel with the correct lens model, form its camera ray, transform the ray into the vehicle frame, and intersect it with the fixed surveyed reference plane. Reject rays directed away from the plane or too close to parallel for stable evaluation. Record these rejections and coverage consequences.

## 11 Acceptance and reporting

For each valid independent ground point, calculate:

`e_XY = sqrt((X_estimated - X_surveyed)² + (Y_estimated - Y_surveyed)²)`

Report each point's ID, camera ID, range, surveyed coordinates, pixel coordinates, estimated ground coordinates, error, and reference uncertainty. Summarize count, median, RMS, 95th percentile, and maximum horizontal error per camera and per distance band. The percentile is descriptive; it does not replace the maximum-error requirement.

Use these result categories:

- **PASS at sampled locations:** all valid independent points have `e_XY ≤ 0.05 m`, the planned usable-region coverage is adequately represented, and reference uncertainty is reported and sufficiently small to make the result meaningful.
- **FAIL:** one or more valid independent points exceed 0.05 m. Preserve the failure and diagnose it.
- **NOT VALIDATED:** insufficient coverage, unresolved target correspondence, failed survey checks, excessive reference uncertainty, or no suitable visible ground region prevents an accuracy conclusion.

A result close to the 5 cm boundary must be reviewed against combined measurement uncertainty. Do not call a borderline result a confident pass when its uncertainty crosses the limit. Report the observed result and uncertainty; collect better measurements if a defensible decision is needed.

If validation data informs a calibration change, it has become development data. Collect a fresh independent validation set before claiming acceptance. Preserve the previous result and explain the change.

A six-camera acceptance report must include all six camera IDs, including any failure or unvalidated camera. Do not imply complete 360° ground coverage from the schematic or from successful individual camera checks.

## 12 Measurement records to retain

| Record | Minimum contents |
|---|---|
| Session record | Session ID, operators, date/time/timezone, site, vehicle state, equipment IDs, calibration evidence, and conditions. |
| Camera inventory | Physical ID, serial/channel mapping, lens, actual image size, focus/exposure, processing history, and mounting photograph. |
| Board record | Physical board ID, source pattern, 7 × 10 inner corners, measured spacing, flatness, reference labels, dimensions, and uncertainty. |
| Survey network | Raw observations, units, station IDs, control/check classification, prism constants, fixture offsets, residuals, and uncertainty estimates. |
| Vehicle references | Hub-center and floor observations, locating-fixture definitions, plane fit/checks, frame construction, and survey-to-vehicle transformation. |
| Intrinsic image manifest | Camera ID, file path, capture time, accepted/rejected status, held-out status, and image settings. |
| Extrinsic placement manifest | Placement and board IDs, images, surveyed controls, rigid fit, pre/post movement check, and corner-order approval. |
| Validation manifest | Ground-point ID, camera/image IDs, surveyed XYZ, pixel coordinates, range, uncertainty, and independent-data designation. |
| Results report | Intrinsic diagnostics, both transform directions, ground errors and coverage, failures/rejections, and final status. |

Keep original files and survey observations immutable within the session archive. Use separate folders for intrinsic fitting, intrinsic checks, extrinsic fitting, independent ground validation, and results. A new session or changed camera setting must not silently reuse prior images.

## 13 Field readiness checklist

- [ ] Total station and experienced operator are available; instrument calibration and suitable accessories are confirmed.
- [ ] Prism constants, wheel fixtures, and checkerboard-reference offsets are documented and verified.
- [ ] The 7 × 10 inner-corner board is rigid, flat, correctly labeled, and physically measured.
- [ ] Site ground and survey-control checks pass; the vehicle frame is established and sign-checked.
- [ ] All six physical cameras map to the correct saved-file identities.
- [ ] Final focus, optical path, resolution, and image processing are documented.
- [ ] Intrinsic fitting and held-out views have adequate image coverage.
- [ ] Extrinsic placements have verified survey controls, corner ordering, and movement checks.
- [ ] Independent ground points cover the usable regions and include checks near 10 m where visible.
- [ ] Survey uncertainty and acceptance-report responsibilities are agreed before evaluating results.

The plan is finalized, but the field session is not ready until these prerequisites are verified. Physical square spacing, equipment access, survey checks, camera specifications, and captured data are deliberately recorded as field inputs rather than invented values. No Python implementation is authorized by this document.

## 14 Sources and basis

1. **User-supplied materials:** `MiLa_CAM_setup(1).pptx` (Current Setup slide; local counterpart originally named `MiLa_CAM_setup.pptx`), `Camera_Calibration_Guide_Google_Docs.docx`, and `Screenshot 2026-10-05 at 11-34-39 .png`. These establish the camera schematic, initial guide, and selected checkerboard pattern.
2. **NVIDIA DriveWorks Static Camera Calibration Tutorial:** [Official tutorial](https://developer.nvidia.com/docs/drive/drive-os/6.0.5/public/driveworks-nvsdk/dwx_camera_calibration.html). Reference for rigid targets, vehicle wheel references, a level floor, and diverse intrinsic images. Its AprilTag/external-camera workflow is background; this plan uses the user's checkerboard with surveyed geometry.
3. **Trimble S7 manufacturer specifications:** [Official product specifications](https://geospatial.trimble.com/en/products/hardware/trimble-s7). Equipment-class reference, not a claim that a particular instrument has been obtained or field-verified.
4. **OpenCV camera calibration documentation:** [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html) and [Fisheye Camera Model](https://docs.opencv.org/4.13.0/db/d58/group__calib3d__fisheye.html). Reference for camera geometry and separate distortion-model conventions.

The 5 cm requirement, six-camera scope, vehicle origin, selected board, survey-assisted approach, and independent validation were agreed with the user. Acquisition counts, survey screening gates, validation layout, and uncertainty handling are the engineering procedure specified by this plan; they are not quoted manufacturer acceptance limits.
