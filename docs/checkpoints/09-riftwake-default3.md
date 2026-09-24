# Riftwake Default 3 geometry checkpoint

**Reopened by user review on 25 September 2026:** This is a measured numerical checkpoint. Its clifftop flow/density, brake pacing, inversion lateral shape, and terrain fit were rejected; those requirements remain active in the completion audit.

The native default ride is accepted with seven two-seat row modules (14 riders), a 181.961458 s duration, and the retained 0–180 km/h launch in 1.397 s. Seed 42 accepts on candidate 0; nearby seed 43 accepts on candidate 1 at 181.390625 s using normal defaults.

## Geometry and dynamics

- Added meaningful low opening hills and a 1000 m hooked clifftop sequence. Active clifftop travel is 27.0529 s, followed by a 5.7469 s lip.
- Riftwake's signature keeps opposing 45-degree wings in one descending silhouette. Closure corrections act on the setup and approach, preserving the signature's internal shape.
- Replaced endpoint-clustered pitch control edits with two broad exact polynomial phases. Fixed total length, authored height, complete endpoint jets, original semantic section, and crest bank ownership are retained. The sub-metre curvature-rate spike is removed.
- The Immelmann uses a 1.0 G crest and broad half-roll ramps. The loop completes its existing 15-degree plane yaw during ascent, separating its legs without raising forces or moving the finished track afterwards.
- Adjusted broad semantic terrain benches and ravines. Native minimum swept-ground clearance is 2.5616 m. Distance-weighted mean rail-ground height is 47.40975 m, down 13.49% from 54.80261 m; time-weighted mean is 48.38214 m versus 55.43613 m, using 60 Hz display traces.
- Peak speed is 300.504 km/h; rider vertical loads are -1.4424 to 4.5567 G. Native source, acceleration-history, force-rate, clearance, support, 960/1920 Hz temporal, and half-spacing spatial checks all pass. These remain the project's game-model checks.

## Verification

- `out/riftwake-resume-06.vcdesign` and matching report, trace, plan, and recipe: accepted default.
- `out/riftwake-nearby-43.vcdesign` and matching diagnostics: accepted nearby seed.
- `organic_generation_tests --baseline-only`: 805317 checks, including continuous rendered forces, trim operating scenarios, and save/load.
- Inherited-port regression: 51791 checks. Motion spline regression: 16893 checks. Core analytical/capacity: 2516 checks. FVD, recipe, drive, hardware, and clearance component tests pass.
- Existing six-row accepted save still independently revalidates as six rows / 12 riders. No save schema migration changes were needed.
- Organic supports reproduce exactly on the prior accepted track: 175 frames, 1623 members, 124 inter-frame members, and verified canonical mesh containment/winding. Station regression: 1129 checks.
- Unreal Editor target build succeeded. First full runtime traversal and interaction/cancellation smoke passed, but visual inspection found material fallbacks: instanced glass was opaque and terrain shaders were unprepared. Added the instanced-material usage flag to the importer and real-RHI content preparation; material recompilation passed. Corrected runtime verification also passed at `out/riftwake-runtime-review-fixed/result.json`. The assistant inspected actual overview, station, opening, clifftop, inversion and signature captures; the opaque-visor and checkerboard fallbacks are resolved. Raw verifier reports retain human design approval as pending, and keyboard input is untested.

## Retained work

Train styling and the original grand station remain unapproved drafts. An isolated functional seven-row station study is at `scratch/station-resume/`; it includes aligned holding lanes, covered queue/merge, dispatch cabin, separate unload/exit, and lifts. It has not been integrated into canonical station geometry or runtime assets.

The verified save is now active at `UserData-Development/Saved/VibeCoaster2/Designs/Accepted.vcdesign`, used by the existing editor-game shortcuts. Prior save and editor binaries are backed up at `out/pre-riftwake-runtime/`; activation identities are in `out/riftwake-activation.json`. No standalone package or `dist/current.json` promotion has occurred.

This checkpoint commits the accepted native geometry, capacity, support/hardware integration, matching project version and seat assertion. Existing Unreal renderer/artwork/palette/import/package changes remain a preserved local draft, including the material preparation repair. They are not visual-design approval and must not be discarded by a clean/reset.
