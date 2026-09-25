# Riftwake force-authored redesign — default.4

The user rejected completion-07's clifftop, inversion geometry, return flats, terrain fit, conservative LSM placement and trim-before-boost hardware. Candidate11 replaces those parts with connected force-authored sources. **Native acceptance and exact-save reload are complete. Standalone front/rear and third-person review, packaging and activation are still pending at this implementation checkpoint.** Completion-07 remains the active rollback until those steps pass.

## Measured replacement

`out/riftwake-fvd-redesign-11.vcdesign`, seed42, candidate0, generator `2.0.0-default.4`.

- Save SHA256: `a88318506bd4d4f8028770b724a40cec199a6c75d60677f8e79739f4b1e47f5a`.
- Length **7327.458 m**; full ride **176.207 s**; retained0–180 km/h **1.397 s**; seven two-seat rows and80 station parts.
- Before the clifftop: **58.509 s**, previously47.877 s. After the cliff-drop body: **82.109 s**, previously96.643 s. These comparisons use the same train-centre event convention.
- The slow shelf is **621.8 m /26.0 s**, with shallow live grade, an outward edge bank, an opposing counterturn and nested low traverse. It does not contain repeated airtime crests. Rail gap averages6.02 m across the shelf, including the deliberate exposed bay; ordinary parts remain close to the plateau.
- The cliff lip occupies **1.532 s** for the front seat, down from2.690 s. Actual brake-work activity across the complete seven-car passage is approximately **2.30 s**, measured on the60 Hz trace; this is a different quantity from lip traversal time.
- The post-camelback180° Wave has84° bank and **50.316 m** actual vertical extent. The following rising brake physically removes approximately874.06 J/kg and delivers **47.996 m/s** to the loop, against its48 m/s target.
- Loop **74 m**, with one monotone8 m displacement toward its exit and parallel arms; planar Immelmann **66 m**. Front-seat apex spacing is **8.046 s**, versus the roughly6.5 s Tormenta reference. This follows the user's allowance for modest extra duration. The1.10 s connector stays loaded above1.12 g across all three measured seats and has no filler airtime crest.
- Two opposing45° signature crests continue through the return hill with live curvature. The longest native inactive flat coast is1.30 s; the final station-alignment turn remains purposeful. Independent continuous-window evidence is in `out/riftwake-fvd-redesign-11-closure-audit.json`.
- All three LSMs start **2.000 m** into their real constant-grade corridors:20°,32° and−8°. Their exits retain the same local2 m guard. Actual exit rail gaps are2.250,2.219 and2.250 m. The uphill exit/arrival stays below4.70 m gap over the next150 m; the downhill bed follows the actual low pullout before the protected camelback rises.
- The redundant trim5.39 s before the uphill boost is removed. The nearest remaining preboost trim is16.90 s upstream. Retained trim controllers remain retracted during the nominal accepted replay.

## Force and size acceptance

| Actual ascent peak | Front | Middle | Rear | Retained counterpart floor |
| --- | ---: | ---: | ---: | ---: |
| Loop | 5.113 g | 4.652 g | 4.414 g |4.344 g |
| Immelmann | 5.234 g | 4.778 g | 4.506 g |4.326 g |

Generation, time/spatial refinement and new-version reload check actual seat ascent phases. Source point-mass peaks alone cannot satisfy the floor. The existing project allowance and F2291-25 acceleration, duration, onset and combined-axis checks are unchanged. All three assessed seat histories pass. The observed reference floors come from consumer recordings; they do not establish exact pointwise force dominance for every feature. See the [reference audit](../references/2026-09-redesign-reference-audit.md).

The canonical degree-nine extrema checks use actual element spans. Loop74<81.8388 m; Immelmann66<99.6696 m; cliff drop221.5<247.5 m. Camelback240.0<247.5 m and Wave50.316<70 m use the explicitly provisional same-type benchmarks. Overall highest rail above the **lowest embedded footing** is **292.261<292.5 m**. Both generation and reload enforce the same size checks. The summit was lowered0.5 m to fit even this conservative datum. No unsupported Opening-specific world-record category is claimed.

## Implementation and checks

Eighteen retained FVD sources cover the curved ride; there are zero geometric spline programmes. Constant-grade hardware and final alignment remain straight geometry. The new sources inherit position, direction, curvature, physical up and derivatives. The final approach solves height and pitch through smooth normal-force controls as well as its plan position and heading; it does not snap a tiny incoming grade to zero. The signature's solved negative-load metadata and inherited inversion entry pitch are not exposed as silently ignored recipe edits.

Native acceptance includes source reconstruction, analytic rider dynamics, full-train/station/support/terrain clearance,960→1920 Hz replay, and independent half-spacing geometry/replay. Minimum swept-ground clearance is **0.470 m**. All accepted errors are empty. Focused regressions:125528 inherited-port checks,66279 FVD checks,1234316 drive checks,100124 dimension checks,140 recipe checks and45 convergence checks.

Exact native hardware and source inventory: `out/riftwake-fvd-redesign-11-native-audit.json`. Consolidated metrics: `out/riftwake-fvd-redesign-11-evidence.json`. Ordinary terrain: `out/riftwake-fvd-redesign-11-terrain-audit/`. True-scale geometry: `out/riftwake-fvd-redesign-11-views/`. These are local artifacts retained under the repository's ignored output directory.

Graphify's native/core index is refreshed:2135 nodes,5424 edges,120 communities from81 code files, zero LLM extraction tokens. Two headers are partially parsed (`clearance.hpp`, `acceptance_internal.hpp`); their actual source was inspected and compiled independently. Existing graph inference warnings remain navigation caveats, not acceptance evidence.

## Delivery gate still open

Build a clean committed Development standalone with the existing V3 assets and `-SkipEditorPreparation`. Run the exact candidate11 save in fresh isolated front/rear profiles, including the opt-in nine third-person review captures. Inspect the outward cliff bay, every boost exit/pullout, inversion spacing and closure. Verify save/package/geometry identities,14 riders and80 station parts before updating `dist/current.json`, shortcuts or the active accepted profile. Preserve the user's already-running old game and completion-07 rollback.
