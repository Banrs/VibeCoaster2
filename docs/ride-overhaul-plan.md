# Default 3 ride overhaul

**Paused:** The user redirected work to a grand premium station and the other environment/hardware models. The unfinished ride/physics changes are retained in `checkpoints/08-riftwake-paused.patch`; the working generator is restored to Default2. Train refinement is also paused.

The 24 September 2026 request resumes implementation after the stopped Default 2 checkpoint and supersedes its solo-work and force-uplift instructions. The target is a coherent, believable, terrain-integrated ride around 180 seconds, with visible improvements in the running game. FF means Falcon's Flight; TRR means Tormenta Rampaging Run.

## Baseline and decisions

The retained seed-42 ride is 6,166.9 m and 133.99 s. It reaches the plateau after about 31.6 s and spends only 7.61 s on active clifftop track versus 9.24 s on the brake/lip approach. Closure feedback changes signature heading. Several short roll ramps create large real lateral accelerations; the G calculation is not an artificial multiplier. Fix the authored motion rather than suppressing telemetry.

The user explicitly reconfirmed the extreme 0-180 km/h in 1.4 s launch; preserve it. Realism improvements target the shape, physical force history and progressive transitions around that intentional exception.

The new layout targets 175-190 s, approximately 180 s. Add meaningful hills and directional changes before the climb, give the clifftop around 25-30 s of active winding track, and keep its brake/lip approach below 6 s. The old 20.5 s cap on the entire clifftop interval conflicts with the request and is retained only for older saves. New default rides have explicit active-track and brake-duration checks. Intamin describes FF as approximately 205 s; our 180 s is the user's design target.

Preserve deliberate inversion and signature yaw against overlap/closure correction. Place or adjust connecting track to accommodate authored elements. Create a recognisable original multi-phase signature with a clear crest, reversal and terrain interaction. Judge forces through actual front/middle/rear seat histories, not isolated peaks or increased roll jerk.

Reduce distance-weighted mean rail-to-terrain height from the retained baseline, including meaningful lower running in connectors and return sections. Keep the giant record-scale elements, but avoid elevating the entire ride to make it fit. Report the before/after metric and inspect rider views. Preserve actual clearance checks.

Use actual Blender MCP for the visible model redesign: train, rail web, station and operating hardware. The first low-bonnet preview was rejected: fully redesign the train with a higher bonnet and a creative late-2020s/2030s silhouette. Make tall support frames sparse and open while retaining canonical member clearance and connected foundations. Verify imported assets in the game, not just Blender.

Measure request-to-ready and process-to-ready separately. Target p99 3 s; do not assert the percentile from a few runs. Preserve fresh validation of arbitrary loaded designs, avoid blocking the game thread, and reduce repeated work. The current saved-ride load is about 2.67 s; ~2.47 s is validation. Longer new rides make this a real performance concern.

## Work and verification

1. Research and implementation plan: inspect retained FF telemetry/POV and TRR/reference data; identify historical 0.8.0 geometry regressions. Keep source and measurement uncertainty visible.
2. Implement in separate lanes: ride authoring, physical roll transitions, Blender models, support frames and startup validation. Root integrates and reviews. Use Astra for the hardest three lanes, Sol for performance/research/supports and Luna for bounded historical inspection.
3. Verify once the relevant changes settle: focused existing physics/support checks, native seed-42 generation and save/load, section timings and force/height comparison, then an integrated Unreal build and front/rear rider-view inspection. Add checks only where they detect a real new failure. Do not run broad CI or repeated suites without a reason.
4. Commit and push meaningful checkpoints with CI skipped. Update the current playable profile only after the new ride passes validation. Preserve the previous accepted ride and working shortcuts until then. Record measured improvements and unmet targets honestly.

The existing acceleration assessment remains an independent numerical check. Its presence is not a claim of physical ride certification. No new force or clearance limits are relaxed merely to obtain acceptance.
