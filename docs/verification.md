# VibeCoaster2 — fallback evidence: verified Escarpment prototype

Version **2.0.0-escarpment.1** is packaged and verified as the current playable fallback. Open **Play VibeCoaster2.lnk**, press **F9** to load the included verified ride, then **Space**. It uses the separate `UserData-Escarpment` profile. The previous Highlands package, profile and fallback shortcut are preserved. This report is an immutable snapshot of that shipped route; the active replacement contract is [NEXT_SESSION.md](../NEXT_SESSION.md).

[Screenshots and synchronized seat charts](checkpoints/04-escarpment.html) · [Machine-readable evidence and source hashes](verification.json) · [Previous Highlands report](verification-highlands.md).

## What changed

1. Replaced raised ordinary running with a 2 m general rail datum and a contoured summit beneath the complete winding act. Nearby lower passages constrain the landform. The same rendered terrain triangles are checked against the entire swept rider/train box; no extra blanket clearance floor is applied.
2. Added the slow trimmed cliff lip, 88° force-authored descent and negative-grade LSM. The LSMs use 16 m/s² rated motor force per mass. Brake ratings were not strengthened.
3. Replaced the distorted spline wave with a continuous FVD rising/falling reversal. Gravity-relative bank has analytic derivatives through third order; the loaded exit shares its complete geometry jet directly with the loop. Tangent-twist mode remains available for inversions.
4. Restored the actual loop-aperture signature and full return. The final turn uses the space and arc length its speed requires. Fixed near-180° chord and heading-branch errors, with independent analytic regressions. The approved planar asymmetric camelback fit is retained.
5. Added a banked-descent trim before the loop, selected by the generated geometry. It uses the existing eddy-current force model and an upstream latched command. All fins remain retracted in nominal motion; visible hardware follows the real track frame.

## Measured baseline

- Peak speed **300.54 km/h**; first 180 km/h **1.397000 s**. Propulsion continues past that threshold.
- **8.757 km**, **201.27 s** to rest; 188.99 s to the terminal brake section.
- Gz **-1.078…4.711**, |Gy| **1.310**, |Gx| **4.296** across representative seats.
- Caps remain Gz −1.5…+5, Gy ±1.5, Gx ±4.5, each component rate 20 g/s.
- Winding sections average **3.06 m rail-to-ground**. The ridge act including its initial approach and LSM averages **3.61 m**. Inclined LSM corridors average about 2 m.
- Minimum certified clearance **outside the full modeled envelope: 0.513 m**. This is different from rail-to-ground height.
- The first climbing approach still averages **9.16 m**. Whole-circuit rail-to-ground mean is **39.70 m**, including the giant hill, cliff, inversions and elevated waves. A 5 m whole-circuit average is not claimed.
- Largest C3 position/frame join residuals: **4.61e-16 / 3.58e-16**. Half-spacing position error **1.4e-10 m**; force relative difference **3.84e-06**.
- Energy balance residual **2.2e-09 J/kg**. Four meaningful transverse encounters are verified, including the original signature through the generated loop. Longest level interval outside operation/alignment corridors **1.433 s**.

## Propulsion evidence

Speeds below use the whole authored corridor, including train alignment and exit fade. Peak speed can occur before its end. The gravity-only value is a lossless upper bound from actual finite-train height change; motor work is measured separately.

| Corridor | Entry → exit km/h | Gravity-only exit upper bound km/h | Peak motor m/s² | Motor work J/kg |
|---|---:|---:|---:|---:|
| ridge-LSM-ascent | 182.1 → 203.4 | 37.1 | 16.00 | 1781.0 |
| descending-cliff-LSM | 199.7 → 257.0 | 227.4 | 16.00 | 718.9 |
| main-speed-launch | 200.4 → 298.0 | 200.4 | 16.00 | 2203.6 |

The [public FF telemetry viewed during research](https://www.youtube.com/watch?v=0UaOSBGSx20) showed roughly 0.85–0.91 g longitudinal peaks on the relevant launch passages; that is an uncalibrated observation, not an Intamin actuator rating. Our explicit actuator model and measured gains establish the implemented stronger-LSM intent. Official context: [Intamin Falcon’s Flight](https://www.intamin.com/project/falcons-flight/).

## Verification

All **21 native suites**, **6 Unreal contract tests**, **8 representative cases** and an independent saved-design reload pass. Native acceptance uses 960/1920 Hz time replay and spatial half-spacing refinement. Every force/rate limit remains unchanged. The native suite includes analytic FVD/helix oracles, full-frame C3 checks, exact swept-solid/terrain checks, finite support caps, energy balance, shape checks, deterministic regeneration and three-seat dynamics.

| Case | Selected candidate | Peak km/h | Seconds to rest | Minimum swept gap m | Result |
|---|---:|---:|---:|---:|---|
| baseline-42 | 0 | 300.54 | 201.27 | 0.513 | pass |
| seed-1 | 1 | 300.54 | 203.53 | 0.561 | pass |
| seed-5 | 0 | 300.54 | 203.77 | 0.523 | pass |
| seed-77 | 4 | 300.54 | 223.18 | 0.489 | pass |
| twin-2718 | 0 | 300.54 | 201.98 | 0.480 | pass |
| flowing-7 | 2 | 300.54 | 226.23 | 0.639 | pass |
| higher-314 | 5 | 310.54 | 228.06 | 0.608 | pass |
| lower-42 | 0 | 290.54 | 199.64 | 1.112 | pass |

The retained operating regression passes with trims removed, all trim banks fully deployed, and 20% lower drag. The lower-drag case peaks at **4.831 Gz / 1.397 |Gy|**. A prior run reached 5.186 Gz at the loop; the new upstream regulator fixes that conflict instead of relaxing the cap. Default nominal trims dissipate zero energy.

| Seat | Gz rate g/s | Gy rate g/s | Gx rate g/s | Inertial jerk m/s³ | Angular jerk rad/s³ |
|---|---:|---:|---:|---:|---:|
| Front | 9.648 | 4.136 | 19.607 | 192.275 | 54.108 |
| Middle | 9.592 | 4.007 | 19.607 | 192.275 | 52.309 |
| Rear | 9.490 | 3.816 | 19.607 | 192.275 | 49.141 |

[Front synchronized chart](checkpoints/04-motion-front.svg), [middle](checkpoints/04-motion-middle.svg), [rear](checkpoints/04-motion-rear.svg). Plots display 60 Hz traces; acceptance peaks come from the native simulation, not the plotted samples.

Both front and rear packaged **2560×1440 D3D12** traversals pass on the RTX 5070 Ti. They check full motion, pause/restart, camera poses, save/reload and save cancellation. Rear reload is a separate process. Native and packaged baseline metrics are exactly equal. Screenshots were inspected for ground proximity, cliff/drop, visible LSM/trim hardware, camelback, return and terminal approach. Automated keyboard input and a formal FPS benchmark are not claimed.

## Build identity and preservation

Executable: `D:\Coding\Codex\Vibecoaster2\dist\2.0.0-escarpment.1\run-20260921-005352-688\Windows\VibeCoaster\Binaries\Win64\VibeCoaster.exe`

SHA-256: `397DF917F27129F2A33ADD25552669D3303F457D563182ABFFBD10EACA204408`

Geometry SHA-1: `5D91E21834F674A6FB5C174D508F9BC2C06D64BB`. Save SHA-1: `02CF8C99018F85D5A337B151FE0FDF1A67DDA4E8`. Both processes agree. The Play shortcut’s resolved target and hash are recorded in `native/build/evidence/escarpment-shortcut.json`.

Native logs: `native/build/evidence/escarpment-native-tests-2.log`, `escarpment-corpus-3/summary.json`; engine logs: `native/unreal/Saved/BuildRuns/20260921-005352-688`; GPU logs/screenshots: `native/build/evidence/gpu-escarpment-front` and `gpu-escarpment-rear`.

The original Vibecoasterjs Git state, reference images, approved camelback trace, original Play shortcut and old V2 save are preserved. The sibling VibeCoaster2 folder is independent of the original Git repository.

## Assumptions and remaining review

F2291 is an envelope/containment guideline here, not a universal 5 m rail-height rule. [ASTM lists F2291-26 as current](https://store.astm.org/standards/f2291); its complete text was unavailable. The implemented box is documented from the accessible preceding edition and NASA 95th-percentile data in [clearance-model.md](clearance-model.md). Seat/restraint dimensions, movement allowances and the reduced brake/aerodynamic models need manufacturer calibration. This is not F2291 certification or a structural stress analysis.

The cliff brake and slow aligned approach take12.51s in the baseline; that pacing remains a user-review item. The Immelmann remains a very large **176.8 m** inversion. Terrain and supporting art remain prototype quality. The generator has multiple winding/return compositions and useful parameters, but it is a bounded ride family, not an unrestricted final-product layout designer. Perceived pacing, signature excitement and roll feel still need your ride review; numerical smoothness cannot prove them. Reference intensity comparison remains disabled until a calibrated FF benchmark exists.

Automatic approval review rejected an earlier proposed 3 m global track datum as an additional clearance margin. It was not applied; support geometry and terrain shaping were corrected while the rail datum stayed 2 m. A separate extra analytic-test write timed out in review; its independent scratch oracle passed, and the production tests above are retained in the project.

