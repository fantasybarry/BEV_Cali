# Practice session — NOT a vehicle camera

Date: 2026-10-22. Captured on the Mac (ffmpeg/AVFoundation) with `capture.py`.

The camera was a **spare AR0234 global-shutter unit ("Global Shutter Camera", USB 32e4:0234), not mounted on the
vehicle**. File names and JSON sidecars say `CAM2` because the tool was run with `--camera CAM2`; they are left
unchanged so the original records stay intact. **Never use these images or `intrinsics.json` as the vehicle CAM2
calibration** — lens parameters differ from unit to unit.

What this session established (applies to the real calibration):

- The 74 mm board is twisted about 4.7 mm peak to peak (corners A and B forward of the best-fit plane, O and C
  behind), found with `cv2.calibrateCameraRO` and confirmed on two disjoint image halves.
- Estimating the board shape cut held-out error from 0.35 px to 0.15 px and moved floor-band rays by up to 0.37°.
- Corrected result for this unit: f ≈ 599.5 px (±0.16 px), principal point ≈ (954.5, 561.8) (±0.1 px),
  HFOV ≈ 123°, ray uncertainty ≈ 0.03° (1σ). `intrinsics.json` holds the earlier flat-board fit only.
- Close-range limit: about 0.6 m; nearer views were blurry.

`rejected/` holds images set aside during practice, with reasons in `rejected/intrinsic-fit/REJECTED.txt`.
