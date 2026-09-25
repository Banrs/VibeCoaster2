# Force reference audit and FVD transitions

Historical study from 24 September 2026. It proposed approximately 1.33 times Falcon’s Flight scale and 1.2 times its speed, with percentage force uplifts by element. Those targets are not the latest user direction; [HANDOFF.md](../HANDOFF.md) calls for natural forces and warns against forced intensity. Use this document for reference provenance and uncertainty, not as a current numeric mandate.

A source replay, photograph fit or higher global peak does not prove point-by-point reference dominance. The consumer recordings lack surveyed track positions, and their alignment uncertainty remains explicit below. Use physical front/middle/rear seat samples and meaningful phases, rather than fitting uncertain timestamp correspondences or repeatedly fine-tuning small photo differences.

## Reference provenance

The two original RFDB exports are retained unchanged in the legacy archive's `archive-2026-09-13/removed/native/build/chatgpt-handoff` directory.

| Recording | SHA256 | Recorded setup | Inclusive raw crop |
| --- | --- | --- | --- |
| Falcons Flight.json | `7daa246124d8d7ace45ec1f45f29dd1f3cf127ed0e133b5c96ce63a474f214ba` | Apple Watch, row7, seatL, 50Hz | 562–8463 |
| Tormenta Rampaging Run.json | `152b56443a918acc54c06f85fe26acfdd717296839881f391f28edae8a453882` | iPhone, row2, seat8, train1, 50Hz | 4007–8131 |

The audit projects raw accelerometer vectors onto the stored, unrenormalized calibration axes, maps x/y/z to Gy/Gz/Gx, retains the upright1g baseline, and applies a centered11-sample arithmetic mean before cropping. This reproduces the exports' stored vertical extrema within5.1e-14g for FF and3.3e-14g for TRR. Output traces and metadata are under `out/reference-audit/`. The recordings contain timestamps, not measured track distances or verified element labels. The older one-second summaries are inadequate for transition or position comparisons.

The separate [CoasterTalk FF telemetry POV](https://www.youtube.com/watch?v=0UaOSBGSx20) uses an edited consumer-device overlay. It is not the RFDB Watch run; device, seat, calibration and synchronization error are not established. Displayed hundredths are display precision. `falcons-flight-video-observations.json` retains observed video timestamps, axes, visual phases, instantaneous values and explicitly distinguished running markers.

Useful FF observations are the sustained ascending shoulder at133.5–136s (+3.57,+3.48,+3.43,+2.91g at selected frames), a local running maximum of+3.65g reached by133.5s, crown minimum marker−1.15g by137.5s, middle crown+0.46g at140s, and descending pullout+2.35,+2.90,+2.67g at145–147s. A larger descending peak does not compensate for a weak ascending shoulder. Lateral/longitudinal sensor orientation is not verified; uncalibrated video Gx must not be reproduced by inventing braking on a coasting hill.

TRR phases were checked against the [Six Flags Over Texas POV](https://www.youtube.com/watch?v=LesJ9BGNk8w). Repeated landmarks support RFDB cropped time approximately equal to video time minus70s, with about±2s cross-recording uncertainty. The official camera and the RFDB phone are not established as the same run or seat. The first Immelmann, loop, subsequent banked connector, block brake, second Immelmann and cutback must remain separate. In particular, peaks after the loop and after the cutback are not inversion peaks.

| Approximate RFDB window | Visual phase | Observed Gz range |
| --- | --- | --- |
| 10–13s | First Immelmann ascent | +2.649 to+4.326 |
| 13–15s | Inverted crown/roll | +0.736 to+2.751 |
| 15–18s | Descending exit | +0.131 to+3.081 |
| 18–20s | Loop ascent | +3.152 to+4.344 |
| 20–21.5s | Loop crown, with uncertain late-ascent overlap | +1.298 to+3.924 |
| 21.5–24s | Loop descent | +0.929 to+3.694 |

These windows are exploratory phase evidence. Their maxima/minima do not certify correspondence at each position. Exact pointwise dominance remains unverified without a distance trace or independently resolved position alignment. The unrelated processed multi-record statistical benchmark must not be presented as this single-record phase comparison.

## Transition model

[NoLimits2's FVD documentation](https://nolimitscoaster.com/nolimits2/help/pages/forcevector.html) describes time-based vertical-force, horizontal-force and roll graphs, with configurable spline types and relative/absolute roll options. The [FVD++ author guide](https://lucasbosch.de/nolimits-tools/FVD%2B%2B/older_versions/fvd%2B%2B_0.5_documentation.pdf) and [OpenFVD force-section implementation](https://github.com/altlenny/openFVD/blob/master/core/secforced.cpp) informed the inherited-control review. [OpenFVD subfunctions](https://github.com/altlenny/openFVD/blob/master/core/subfunction.cpp) include both monotone and pulse-like quintics; the polynomial degree alone does not establish monotonicity or a smooth join. No GPL implementation was copied.

The native controls retain force/twist value, first time derivative and second time derivative. Quintic Hermite segments match those six endpoint conditions. The rest-to-rest special case is `10u^3−15u^4+6u^5`. Nonzero incoming derivatives are inherited, not reset at chapter boundaries. Every force and twist boundary is retained during integration and final canonical remeshing.

Twist is rotation about the moving tangent, not Euler horizon roll. Full angular motion includes track bending and changing speed. Gravity-referenced bank explicitly includes the tangent/yaw coupling. Time and distance derivatives are not interchangeable: `q_dot=v*q_s`, `q_ddot=a*q_s+v^2*q_ss`.

The stronger camelback's recovery uses a monotone asymmetric easing of the quintic ramp. Its analytic value, rate and second derivative are sampled into native quintic Hermite controls. This retains derivative continuity while changing when the positive recovery load arrives. Its full finite-train replay must pass both peak and negative-history duration checks. Independent F2291 history/duration and numerical source-agreement checks remain unchanged. The separate project force/rate envelope uses the uniform 5% allowance authorized by the user.

For a level coordinated approach, body normal load is `1/cos(bank)` and heading rate is `−g*tan(bank)/speed`. The bank graph, its derivatives and normal-force derivatives are authored together. The bounded solve chooses straight-entry, loaded-turn and straight-exit durations to reach the actual station port; it does not snap an endpoint or repair a failed force source with a spline.

## Selected basic generator

The accepted seed-42/highlands candidate (`out/recovery-balanced-inversion.vcdesign`) completes 6166.9249 m in 133.9906 s, reaches 300.534 km/h and measures Gz -1.42842 to +5.00165. Its maximum |Gy| is 0.98962, |Gx| 4.29630 and vertical force rate 16.51624 g/s. The positive peak is explicitly reported as 0.033% above the nominal +5 g project cap. The independent selected F2291-25 acceleration checks pass; this is not whole-standard certification.

The asymmetric planar camelback uses one isotropic reference fit. Its connected native silhouette measures 3.53824 px RMS / 12.7918 px maximum against practical 4/16 px guards. Distorted-width and reversed-travel fixtures remain rejected. No further silhouette fitting is required for this checkpoint.

| Camelback phase | FF observation | Target (+15%) | Front | Middle | Rear |
| --- | --- | --- | --- | --- | --- |
| Loaded ascent peak | +3.65 g | +4.1975 g | +4.2553 | +4.2356 | +4.2104 |
| Airtime minimum | -1.15 g | -1.3225 g | -1.4284 | -1.3487 | -1.3681 |
| Recovery peak | +2.90 g sampled | +3.335 g | +4.5309 | +4.5362 | +4.5534 |

Every seat clears those separate phase baselines. The recovered native trace and `final-camelback-baselines.json` retain the unrounded values. Sparse POV observations do not establish an unobserved full peak envelope.

The loop uses a 4.85 g ascent, 2 g crown and 3.9 g recovery; the Immelmann uses 4.85 / 3.8 / 3 g with a release to 2 g through the descending roll. Both explicitly unload over 1.2 seconds into a solved crown hold. The actual loop ascent peaks are approximately 5.002 / 4.821 / 4.680 g, versus the TRR ascent baseline of about 4.706 g. The rear difference is small relative to the approximate comparison. Actual Immelmann ascent peaks are 4.949 / 4.808 / 4.686 g, against about 4.687 g for the first TRR Immelmann. Actual middle-of-roll loads are 3.153 / 3.394 / 3.766 g. Peak/crown/source controls are not substituted for measured seat loads.

The fixed phase-alignment audit (`balanced-inversion.json`) still has nominal-position differences, particularly against the loop's uncertain late-ascent/crown window and TRR's second Immelmann. No core deficit persists across every fixed time shift, but that does not establish exact superiority. The code's calibrated-reference intensity status therefore remains separate and unavailable; approximate shape/phase goals were the direction at the time of this study.

Other parts of this candidate remain active within the project envelope: the opening peaks at 3.49-3.57 g, the wave at 4.01-4.12 g, the signature at 3.56-3.58 g with -1.31 to -1.35 g airtime, and the return at 3.93-3.96 g. These are actual-seat ranges, not claims of exact 10% dominance over an unidentified reference record.

## Generation and verification

Nearby Immelmann entry speeds previously exhausted legacy solver seeds even though the requested held-crown shapes were constructible. An additional short crown-hold starting estimate fixes the two reproduced speeds without changing the height, force or acceptance targets.

The generator adjusts two upstream authored bearings when the station approach is unreachable or leaves excessive quiet coasting. The bounded estimate only chooses another recipe trial; actual FVD integration, full train replay, closure and clearance still decide acceptance. No geometry is moved or snapped to make the approach fit.

When actual validation rejects the camelback's sustained recovery after negative history, the generator can shorten only its final constant-load tail in 0.1-second steps, at most 0.2 seconds. A rejected signature negative-load peak can reduce that source's airtime magnitude in 5% steps, at most 20%. Both changes are recorded in planning JSON and followed by complete fresh validation. The default needs neither correction. The 310 km/h, airtime-1.1, roll-55 case passes after a 0.1-second tail reduction and a 0.95 signature multiplier; its actual peak speed is 310.519 km/h and Gz range -1.53914 to +4.99629.

All 16 component suites pass, including 66303 FVD checks. All 10 integration suites pass on the final source; the final eight-case corpus passes 8/8, including save/reload and seed diversity. Both 2560x1440 front/rear Unreal traversals passed with geometry SHA1 `09CDA5902EC0D2B52E1BE049AF94125BDC382329`; the primary agent inspected all 46 captures per seat. The default also passes configured-drag trims-off and fully-deployed operating checks. No lower-drag run is required.

All 11 retained default sources were checked at 1, 2, 4 and 8 subdivisions per canonical span. Maximum signed angular velocity/acceleration/jerk disagreement is 3.11e-10 rad/s, 3.06e-8 rad/s^2 and 1.39e-5 rad/s^3, inside unchanged 1e-4 / 1e-3 / 1e-2 numerical tolerances. Independent finite differences and source-step refinement are retained in `out/recovery-balanced-angular.log`.

## Scale and interpretation

The official [Intamin dimensions](https://www.intamin.com/project/falcons-flight/) give 165 m for FF's camelback and 250 km/h for its approach; they do not establish a surveyed common height datum or track shape. Multiplying those figures gives roughly 219.45 m and 300 km/h. The selected candidate rises approximately 221.5 m from its actual entry low point to the camelback crest, about 0.94% above that scale target. Curvature acceleration at corresponding coordinates scales as speed ratio squared divided by geometry scale: 1.2 squared / 1.33 is 1.0827. Proper seat force also contains gravity projection and rotational acceleration; local coasting speed ratios vary through the hill. Stronger relevant phases therefore require genuine force authoring and seat replay, not a blanket multiplier on displayed G.

Earlier rejected profiles, obsolete drag tests and superseded fit tolerances remain in `out/reference-audit/research-history-through-20260924.md` and the original experiment files. They are historical evidence, not current acceptance requirements.

Work stopped on user request after local verification. Windows/macOS component and baseline CI passed for18d3fb2; the full checkpoint passed on macOS and its remaining Windows run was cancelled. At that time the shortcuts launched a local Default2 game, and standalone packaging had stopped during cooking. The current default.3 package and shortcuts are documented in HANDOFF.md.
