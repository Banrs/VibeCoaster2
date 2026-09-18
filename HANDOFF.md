# Bounded maintenance of the restored Immelmann baseline

The user stopped and archived the broad milestone on 16 September 2026, then authorized a bounded maintenance pass on the later `0.8.0-immelmann.2` baseline. Archived proposals remain evidence, not instructions to resume that milestone.

## Active correction — visible flatness

The user rejected the remaining flatness in the packaged 0.8.1 ride and authorized another focused correction using real-world design evidence, explicitly allowing inclined braking. The previous zero-work/straight-only checks missed long near-level curved approaches and active hardware. They are not evidence that visible flatness is acceptable. The current audit includes all operations and horizontal curves, with pitch, height relief and time measured together.

The user clarified the micro-issues with an image: small wrong-way motions and hesitations at joins, across pitch, heading and roll. Independent diagnosis reproduces two FVD valley pitch-rate dips of about 80% despite C3 continuity, and separate small heading/roll reversals. These require explicit shape checks; mathematical continuity alone is insufficient.

The user rejected trial D's extra elevation. Preserve the good Immelmann shape and elevation and fix its approaches. Do not ship the experimental +18 m inversion lift or +12 m whole-flyover lift. Modest element entry/exit-height asymmetry is allowed where it solves a real elevation transition; changing every element or adding top hats is not requested. Falcon's Flight POV informs purposeful grade and hardware placement; observations and limits are retained in the task evidence.

Candidate source is `0.8.2-graded.1`; it is unfinished. Non-signature variety remains a separate TODO. The verified 0.8.1 package and shortcut stay available until corrected source and package pass validation. Current source, saved rides and other inputs were copied and SHA256-verified in `D:/Coding/Codex/vibecoasterlegacy/task-20260917-graded-flats/baseline` before this pass. Read that task's `ARCHITECTURE-REVIEW.md` together with its latest-user correction; initial trial proposals are not current requirements.

## Previous maintenance — verified 0.8.1 baseline

- Remove unnecessary level transport while preserving the later Immelmann and useful hills and macro elements.
- Move the final braking work onto the ending straight and shorten the physical stop, preserving normal coasting speed through the final bank.
- Restrict generated propulsion to upright, essentially straight corridors with constant or nearly constant grade. The grade should be level or visibly inclined/declined, not an incidental shallow slope.
- Use the original launch plus two substantial mid-ride boosters; the held-helix mode may use a third. Count separated physical powered runs separately. Do not add short recovery motors to hide energy or layout problems.
- Audit loading improvements from the archived milestone and retain only justified, verified changes. The restored baseline already contains the substantial loading improvements; the small measured trial was not retained.
- This baseline's source and verified package are `0.8.1-linear.1 / COASTER5`. `Play VibeCoaster.lnk` opens its inner packaged executable directly with no arguments, bypassing Epic, the Editor and command wrappers.
- Working evidence and immutable baseline: `D:/Coding/Codex/vibecoasterlegacy/task-20260916-flats-brakes`.

## Next-turn TODO

**Add more variety to non-signature elements.** The user explicitly deferred this to the next turn; it is outside the current maintenance pass.

**Broader direct-transition tool deferred.** This pass uses the existing C3 joins, trims section reservations, and distributes the final descent through its return window. The held-helix variant reuses descending crest flanks for two of its motors. Complete crest neighborhoods and whole-train propulsion/braking clearances take precedence over trimming the last few meters.

The measured ordinary routes have no long nonterminal neutral coasting runs; the longest in the independent audit is 5.75 m. Total level distance nevertheless increases because propulsion and the final brakes now need real straight corridors. Do not describe this as removing every flat or reducing all flat-distance measures. The detailed audit separates level geometry, unoccupied track, actual zero-work travel and motor maintenance.

## Changes and measured results

- Ordinary rides have the original launch plus **two** continuous mid-ride boosters, approximately 196–216 m of hardware each. Held-helix rides have **three** mid-ride boosters, including two descending crest flanks. No final recovery motor remains.
- Propulsion is checked after final banking across the complete train footprint: hardware plus the full 17 m car-centre span at each end. Checks include heading, pitch variation, curvature, upright frame and roll derivatives. Visible grades are approximately 5° or steeper, with a documented 0.1° fitting allowance; level means within 0.05°. These are geometry criteria, not new force limits.
- The final bank coasts. Actual braking begins after the rear car clears the curve, on the straight approach. The station controller uses 6 m/s² nominal deceleration, 7 m/s² per-car capacity and a 0.5 s ramp. The physical stop reference remains unchanged; speed reaches zero without a position snap.
- Seed 42 final-bank exit speed changes from **14.17 to 49.56 m/s**. Comparing the same final-bank-entry anchor, time to rest improves from **21.27 to 19.31 s**. The new straight exit-to-rest interval is 9.73 s; it starts with much more speed than the old one.
- The complete loop, later Immelmann and all four FVD hills remain. The two audited connecting crests retain their original prominence within the bounded regression allowance. All 13 baseline visible crests match in ordinary seeds 42, 17 and 38. C3 joins and frame continuity pass the independent audit.
- Existing loading optimizations were already present in the restored baseline. A marginal trial was discarded. A new accepted-search exit prevents redundant attempts once a fully validated, converged ride stops within 200 s. The real default seed-42 budget now uses one attempt instead of eight; acceptance gates remain unchanged. This avoids a regression caused by moving the final-brake boundary, and is not a claim of a measured overall speedup against 0.8.0.

| Fixture | Original physical stop | Current physical stop |
| --- | ---: | ---: |
| Seed 42 | 192.506 s | 196.302 s |
| Seed 17 | 180.041 s | 184.663 s |
| Seed 38 | 189.603 s | 194.119 s |
| Seed 5 | 194.798 s | 198.682 s |
| Seed 42, 180 m / 65 m/s targets | 194.816 s | 199.446 s |
| Seed 42, 240 m / 80 m/s targets | 190.513 s | 193.850 s |
| Synthetic held-helix 20, seed 1 | 200.416 s | 207.947 s |
| Synthetic held-helix 30, seed 1 | 199.145 s | 204.546 s |

### Explicit limits

- Held-helix variants still exceed 200 s. This bounded pass does not establish an all-menu duration pass or implement the cancelled broad milestone's force/reference policy. The synthetic held fixtures are not real-world telemetry.
- Continuous terminal elevation release substantially flattens the held variants' small final-bank crest: original prominence about 6.4 / 6.2 m becomes about 1.2 / below 1 m. The bank remains curved and descending with valid force/clearance checks. Do not claim every original held crest survives.
- Seed 42 at 220 m / 95 m/s with a one-candidate budget still rejects on the existing vertical-force gates, as did the original baseline. This is a bounded rejection example, not proof that every 95 m/s request is impossible.
- Original `0.8.0-immelmann.2` saves keep their geometry, operations and provenance. **Generate a fresh ride to see the maintenance changes.** Other numerical versions remain unsupported, and all historical files remain preserved.

## Verification and retained evidence

Evidence root: `D:/Coding/Codex/vibecoasterlegacy/task-20260916-flats-brakes`.

- Final-source Release build: **23/23 native CTest suites pass**, including ordinary/held menu fixtures, original-save compatibility, rejection guards and 960/1920 Hz convergence. Log: `v080-flats-final-brake-20260916-a/final_guard_ctest.log`.
- Independent final audit: `independent-validation-20260916-a/FINAL-AUDIT-98B2F8.md`. It verifies all 72 source/export manifest entries, reuses ordinary evidence only after byte-identical comparison, and independently replays the changed held rides. Motor footprints pass 0.5 / 0.25 / 0.125 m grids plus boundaries and knots. Spatial sampling is evidence, not an interval proof.
- Implementation details, multi-seed measurements and actual-work exports: `v080-flats-final-brake-20260916-a/FINAL_IMPLEMENTATION_REPORT.md` and `FINAL_MATRIX.json`.
- Frozen `generation.cpp` SHA256: `98B2F8B788C17F5F341556BEDD2EBFCC236C6418483AC74186F706ABB723236E`. All 134 packaged native source files match the workspace; all 28 protected inputs match their original hashes. Records: `packaged-source-final-check.json` and `protected-inputs-final-check.json`.
- Unreal package: `D:/Coding/Codex/vibecoasterlegacy/linear-081-game/run-20260916-155013-026/Windows`. The Editor build, **six Unreal automations**, game packaging and both packaged full-traversal checks pass. The new generated ride stops at 196.302 s; the original app-authored 0.8.0 save still stops at 192.506 s with identical geometry and save hashes. New save/load, cross-process old-save load, pause/restart, seat/overview poses and save cancellation pass. Both runs use actual offscreen rendering at 2560×1440. Overview, loop, Immelmann and station-stop screenshots were inspected; this does not replace the user's ride review.
- Runtime evidence: `unreal-final-generation-20260917`, `unreal-final-legacy-load-20260917` and `runtime-final-check.json`. All 36 recorded screenshot hashes verify. Executable SHA256: `BD0FC1B702AD728186F30C0BEC00594F3A87150C1360C3BEBEEBB93FF3A23C14`. Package and shortcut records: `package-final-manifest.json` and `shortcut-final-check.json`.
- Single observed request-to-commit times: new generation 2.382 s, original-save loading in the new executable 1.691 s; the old package's generation observation was 2.247 s. These are not controlled benchmark or FPS/GPU acceptance results. Keyboard input and cancellation outside saving were not retested. A same-scale route/elevation/ending comparison is retained in `final42-review.png`.
- Builds, profiles, screenshots and experiment outputs remain outside the working folder. No commits or history rewrites were made.

## Preserved baseline and fallback

- **Version:** `0.8.0-immelmann.2`, restored from commit `0d2c994126f6a7d1fb35df0bd944c5fd172d07e1`. This keeps the later Immelmann revision, subsequent test organization and concise source comments. It is the flat-ground baseline.
- **Fallback:** the original package remains at `D:/Coding/Codex/vibecoasterlegacy/current-game`; its inner `VibeCoaster/Binaries/Win64/VibeCoaster.exe` can still be opened directly. The working-folder shortcut now targets the verified maintenance package described above.
- **Validation:** fresh Release build and all **23 native CTest tests passed**. All 49 packaged runtime files match their retained SHA256 manifest for `0.8.0-immelmann.2`. This restores a baseline; it does not implement the abandoned milestone's newer requirements.
- **Game check:** the direct packaged executable passed one offscreen startup check, generated the accepted seed-42 ride and entered moving playback. Evidence is in the archive's `verification/packaged-launch` folder; no editor rebuild or new packaged build was needed.
- **Preserved:** `AGENTS.md` verbatim, both RFDB recordings, the baseline's full historical test fixtures, and unchanged pre-rollback ride/fixture inputs in `Saved Rides`.

## Archive

`D:/Coding/Codex/vibecoasterlegacy/paused-milestone-20260916`

The archive contains the complete dirty workspace, Git metadata, later tests, planning/research documents, and 46 retained experiment directories. All 6,312 copied files were SHA256-verified before cleanup. Verification logs and restoration scripts are also there. No commits or remote history were rewritten.

## Last design feedback, for future discussion

The rejected milestone prolonged the Immelmann's inverted second half, introduced repeated elevated plateaus, lost useful macro elements, and braked through intense banked sections. The user wants almost no level transport outside propulsion/braking runs and a useful sequence after the fastest section before the final brakes. Existing force ceilings were acceptable; realized profiles, especially negative Gz, were too weak. The current maintenance scope above supersedes the paused status; broader force-profile and layout redesign remains future work.
