# VibeCoaster2 2.0.0-highlands.1 — fallback evidence

This is an immutable report for the earlier Highlands fallback, not the active redesign contract. The standalone rewrite is in D:\Coding\Codex\Vibecoaster2. Open [Play VibeCoaster2](<D:/Coding/Codex/Vibecoaster2/Play VibeCoaster2.lnk>). Press F9 to load the included verified seed-42 ride, then Space to ride. Enter generates the selected settings; Tab opens setup; 1/2/3 select seats; M toggles overview; R restarts; F5 saves. Saves use V2's independent UserData directory. The original project and its Play shortcut are preserved. Current requirements are in [NEXT_SESSION.md](../NEXT_SESSION.md).

## Completed work

1. Continuous hybrid FVD and spline authorship now carries live pitch, heading, curvature and higher derivatives between adjoining motion. Global physical banking replaces forced neutral joins; hardware alignment is retained where required. The booster-boundary interpolation bug was fixed at its source.
2. The planar asymmetric camelback uses the accepted full-photo fit, with the marked flank bow reduced and the crown 2.217 m / 0.0442 s earlier in the source motion. Whole-contour RMS is 1.917 px at 2048-pixel width; marked-region RMS is 0.911 px. These are image-fit measurements, not calibrated as-built FF angles.
3. A substantial twisted opening precedes the multi-element winding ridge, loop/S/booster chain and giant hill. The ridge has spiral and weaving compositions. The return has flowing and twin-airtime compositions; required recovery room is planned before placing the major route. The aperture release/diving passage adapts to the actual giant-hill corridor.
4. The original green/slate bent ridge is one persisted surface shared by generation, clearance, supports and rendering. Native and Unreal use the same 8 m terrain triangles. Flat ground remains a comparison mode.
5. Segmented LSM and brake hardware follows the actual track, including graded propulsion. Optional ride-wide trims use upstream sensing, latched partial deployment, finite train occupancy and speed-dependent eddy-current force. Their visible fins follow measured deployment; no target speed is substituted into replay.
6. Native verification, a representative corpus, packaging and two actual GPU traversals were completed. Compatible earlier V2 saves retain their geometry and provenance. Superseded authoring helpers and the unused V2 palette path were removed.

## Measured baseline

| Quantity | Measured |
| --- | ---: |
| Maximum speed | 300.540 km/h |
| First attainment of 180 km/h | 1.397000 s |
| Track length / complete stop | 7322.29 m / 156.179 s |
| Maximum height above ground | 248.203 m |
| Representative-seat Gz | -0.98180 to +4.71281 g |
| Maximum absolute Gy / Gx | 1.24894 / 4.29630 g |
| Minimum swept ground clearance | 6.27723 m |
| Longest unpowered flat interval | 0.633 s |
| Maximum modeled energy residual | 9.73e-10 J/kg |
| Peak modeled propulsion power | 19.919 MW |

The unchanged limits are Gz -1.5 to +5, absolute Gy 1.5, absolute Gx 4.5, each body-axis force rate 20 g/s and configured minimum clearance 2 m. Final canonical geometry and physical orientation pass C3 join checks, source replay and interior-shape checks. Complete train/rider/ground/support/station envelopes pass. Native acceptance uses 960/1920 Hz temporal replay and spatial half-spacing refinement. The largest baseline position and physical-frame join residuals are 4.34e-16 and 3.54e-16; these are numerical derivative residuals, not comfort measures.

| Seat | Gz rate g/s | Gy rate g/s | Gx rate g/s | Inertial jerk m/s³ | Angular jerk rad/s³ |
| --- | ---: | ---: | ---: | ---: | ---: |
| Front | 8.318 | 5.121 | 19.607 | 192.275 | 59.167 |
| Middle | 8.247 | 4.862 | 19.607 | 192.275 | 55.641 |
| Rear | 8.119 | 4.833 | 19.607 | 192.275 | 50.350 |

[Front motion chart](checkpoints/03-motion-front.svg), [middle](checkpoints/03-motion-middle.svg), [rear](checkpoints/03-motion-rear.svg). Each shows synchronized roll, pitch, yaw, speed, Gx/Gy/Gz, force rates and jerk. Plots are 60 Hz display traces; native peaks above are not inferred from these plots.

## Representative corpus

| Case | Composition | Candidate | Length m | Stop s | Speed km/h |
| --- | --- | ---: | ---: | ---: | ---: |
| baseline-42 | spiral-ridge / flowing-return | 0 | 7322 | 156.2 | 300.54 |
| seed-1 | spiral-ridge / twin-airtime-return | 1 | 7692 | 165.1 | 300.54 |
| seed-5 | spiral-ridge / flowing-return | 0 | 7312 | 156.3 | 300.54 |
| seed-77 | weaving-ridge / twin-airtime-return | 0 | 7801 | 169.9 | 300.54 |
| twin-2718 | spiral-ridge / twin-airtime-return | 1 | 7846 | 168.4 | 300.54 |
| flowing-7 | weaving-ridge / flowing-return | 0 | 7676 | 165.9 | 300.54 |
| higher-314 | weaving-ridge / flowing-return | 0 | 7532 | 161.1 | 310.54 |
| lower-42 | spiral-ridge / flowing-return | 0 | 7100 | 153.1 | 290.54 |

All eight cases pass with the original limits. They cover both terrain modes, both ridge/return families, 290/300/310 km/h, airtime and signature-roll changes. The separate trim study passes trims disabled, full deployment and 20% lower drag. The low-drag result approaches the vertical cap and is called out below.

## Build and runtime evidence

- 21 native suites pass across the full run and targeted corrections. Earlier failures and their reruns remain in native/build/evidence. Stale flat-only/trim fixtures were corrected; limits were not widened.
- Six Unreal automation tests pass, including exact ground-triangle parity, canonical rail/support geometry, imported train art and setup controls. One unrelated engine connectivity probe emitted an HTTP timeout warning.
- Packaged Win64 Development build uses Unreal 5.8.2. Front and rear 2560×1440 GPU traversals pass; pause/restart, all seat/overview camera poses, save, reload and cancellation preserve accepted state.
- Fresh generation and independent cross-process reload retain geometry identity 550EB86087AC697D74AB6FC18506286A766D2DC2 and saved-file identity 09325B6DF0E9D22B0F3451FC2378B1B4FA1F19D7. Packaged baseline metrics are exactly equal to the native baseline metrics.
- The shortcut was read back after creation and targets the exact tested executable. Executable SHA256: 1BBE12A0F51D634F3D9FCB42ED522C7A4DA7520C705CFBCE5A5A98D9B518AF93.

Executable: D:\Coding\Codex\Vibecoaster2\dist\2.0.0-highlands.1\run-20260920-120133-302\Windows\VibeCoaster\Binaries\Win64\VibeCoaster.exe

[Machine-readable manifest](verification.json) includes source and package hashes, test paths, shortcut readback and both runtime results. [Visual review](checkpoints/03-delivery.html) collects selected untouched screenshots and the seat charts. Original image and traced-reference hashes are retained in the manifest.

## Remaining limitations

- Immelmann remains approximately 194 m above ground in the baseline. Its high-altitude feel is not resolved by numerical acceptance.
- Signature excitement, comparative pacing and perceived roll quality still need the user to ride the build. No claim of worldwide novelty.
- Terrain currently offers one bent-ridge family and flat comparison mode; detailed scenery, imported landscapes and further terrain families are future work.
- Mass, drag, wheel/suspension behaviour and hardware force curves are prototype assumptions. LSM/magnet gap, thermal capacity, restraint and structural engineering are not calibrated; no F2291 certification is claimed.
- The 20%-lower-drag case reaches 4.97332 Gz under the unchanged +5 cap. This is a small tested margin, not a validated full operating envelope.
- No calibrated FF/TRR/I305 force comparison or full angular-jerk comfort limit is established. The legacy reference mode remains unavailable without reference data.
- Eight representative cases do not prove every seed or parameter combination. Infeasible requests remain bounded, explicit rejections.
- GPU smoke covers full front/rear rides and all three paused camera poses. Physical keyboard input and a dedicated FPS benchmark were not performed.

The package is a verified stepping stone for further ride and product development. Numerical acceptance does not establish the unresolved subjective qualities above.
