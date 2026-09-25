# Riftwake force-authored redesign — default.4

The user rejected completion-07's clifftop, inversion geometry, return flats, terrain fit, conservative LSM placement and trim-before-boost hardware. Candidate11 replaces those parts with connected force-authored sources. **Native acceptance, exact-save reload, final-package front/rear traversal, agent visual review and playable activation are complete.** Both Play shortcuts now load the exact candidate11 save in `UserData-Riftwake-V4`. Completion-07 is preserved for rollback. Work continued solo after every subagent reached its stopping point.

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

Native acceptance includes source reconstruction, analytic rider dynamics, full-train/station/support/terrain clearance,960→1920 Hz replay, and independent half-spacing geometry/replay. Minimum swept-ground clearance is **0.470 m**. All accepted errors are empty. Focused regressions:125528 inherited-port checks,66279 FVD checks,1234316 drive checks,100128 dimension checks, including the exact saved candidate,140 recipe checks and45 convergence checks.

Exact native hardware and source inventory: `out/riftwake-fvd-redesign-11-native-audit.json`. Consolidated metrics: `out/riftwake-fvd-redesign-11-evidence.json`. Ordinary terrain: `out/riftwake-fvd-redesign-11-terrain-audit/`. True-scale geometry: `out/riftwake-fvd-redesign-11-views/`. These are local artifacts retained under the repository's ignored output directory.

Graphify's native/core index is refreshed:2135 nodes,5424 edges,120 communities from81 code files, zero LLM extraction tokens. Two headers are partially parsed (`clearance.hpp`, `acceptance_internal.hpp`); their actual source was inspected and compiled independently. Existing graph inference warnings remain navigation caveats, not acceptance evidence.

## Packaged proof and visual review

The Development standalone was built with UE 5.8.2 from source commit `f110b1b2b62ec632f0a2366939d9c564312972f9`, using the existing verified V3 assets and `-SkipEditorPreparation`. Package: `native/unreal/Packaged/run-20260925-124313-111/Windows/VibeCoaster/Binaries/Win64/VibeCoaster.exe`.

| Identity | Value |
| --- | --- |
| Game executable SHA256 | `198100288A1302A9D5EAA60F00F3BDD6097B3FDAAFB300F257AF19C601D70237` |
| Package manifest SHA256 | `BAF6C9F4E02D92324E44F799DAAB8E287E8AB636DA2E3BE7C8B77B8C7F71E172` |
| Loaded save SHA1 | `B66E6BEA7A91D88B79F96BF6F84FB1CB3DC9587A` |
| Runtime geometry SHA1 | `FCEE7D0E4771B6965142FF2396A2FEE28C396322` |

Both fresh isolated runtime profiles completed the full 176.207 s ride with 14 riders and 80 station parts. The final front run retained 74 screenshots, including all ten planned review views; the final rear run retained 70 screenshots. Both passed load, pause, restart, paused-pose stability and cancellation of save, generation, mesh preparation and scene commit. Both verifier processes exited successfully. Their loaded save and geometry identities match, and the executable hash was checked before and after each run.

- Front: `out/riftwake-default4-runtime-front-v2/result.json` and adjacent `.identity.json`.
- Rear: `out/riftwake-default4-runtime-rear-v2/result.json` and adjacent `.identity.json`.
- Agent visual observations and exact image hashes: `out/riftwake-default4-visual-review.json`.

Root inspected all ten final-package third-person images at larger size, all 64 rear POV images in timestamped contact sheets, and earlier front POV frames from the identical saved geometry. The [outward-bank view](../../out/riftwake-default4-runtime-front-v2/30-review-cliff-outward-bank.png) shows the sustained 50-degree outward bank over the exposed bay; the [whole shelf](../../out/riftwake-default4-runtime-front-v2/31-review-cliff-winding.png) shows opposing and nested shallow turns. The [actual low pullout](../../out/riftwake-default4-runtime-front-v2/40-review-boost-pullout.png) has a measured 2.262 m rail gap. The first uphill camera is shadowed but shows its rail and terrain bed; the other boost views are visible. Dedicated [Wave](../../out/riftwake-default4-runtime-front-v2/45-review-wave-turnaround.png), [inversion](../../out/riftwake-default4-runtime-front-v2/51-review-loop-immel-spacing.png), [closure](../../out/riftwake-default4-runtime-front-v2/60-review-post-inversion-closure.png) and [final return](../../out/riftwake-default4-runtime-front-v2/70-review-final-return.png) views cover the remaining requested geometry. Continuous force, curvature and flatness conclusions also rely on native trace audits, not isolated screenshots.

The first package's review exposed two terrain-obscured boost cameras and a pullout camera that had selected the camelback ascent. The final package fixes the observer targets and terrain sightlines and adds the dedicated Wave view. Those camera-only changes did not alter candidate11's geometry. Earlier nine-view runs remain diagnostic history; the final evidence above uses ten views and the final executable.

## Activation and retained limits

`dist/current.json`, the repository Play shortcut and desktop VibeCoaster2 shortcut now target the verified final standalone and `UserData-Riftwake-V4`, with `-CoasterLoad`. Both shortcut targets and arguments were read back. The playable save still matches candidate11's SHA256. `out/riftwake-default4-activation.json` records those identities and the rollback paths.

The previous manifest, root/desktop shortcuts and default.3 accepted save are backed up in `out/default4-activation-backup/`. `UserData-Riftwake-V3`, its package and the original `UserData-Development` profile remain unchanged. The original development save still hashes to `DB3D4B55F055036A3D18ABDD54D3D1ADF5EC8EC8988F41F760ECB42DEF03A8CA`. The user's old running game was not closed; it must be relaunched through the updated shortcut to use this build.

Runtime evidence records agent inspection, not user styling approval or physical rider intensity testing. Direct keyboard input and FPS/GPU acceptance remain unclaimed. The older 3 s startup target remains unmet in the previous 100-run benchmark; this redesign does not claim a new startup percentile result. The force-reference and provisional category limits described above remain explicit. No force or clearance acceptance gate was relaxed.
