# Inversion path research and correction

This is a prior geometry study and default.3 numerical checkpoint, not approval of the loop or Immelmann. Scratch paths below now refer to D:/Coding/Codex/vibecoasterlegacy/workspace-deflation-20260925/old-probes/scratch. The latest user reports both remain unfaithful, especially in yaw; see [HANDOFF.md](../../HANDOFF.md).

The user clarified that the defect is unnecessary left/right centerline modulation and the wrong loop hand, not merely rotation rate. The new default must satisfy those path constraints as well as the existing physical checks.

## Real element reference

[Efteling describes Baron 1898's Immelmann](https://www.efteling.com/en/press/-track-baron-1898-) as a half-loop with a half-roll. The vertical half-loop reverses travel direction; a separate horizontal yaw manoeuvre is not inherent to that definition. For this ride, the design target is a near-planar half-loop and roll-out without an imposed yaw weave. Real implementations can depart from an ideal plane; this source is not a surveyed centerline or a universal zero-offset tolerance.

[B&M's Baron 1898 page](https://www.bolliger-mabillard.com/coasters/baron-1898) and [SheiKra page](https://www.bolliger-mabillard.com/coasters/sheikra) identify built examples. Their official photographs were actually inspected and retained for research under `out/reference-audit/real-inversion-shapes/`. Baron's inversion is visible in the background; the SheiKra photograph shows the first drop, so it is not evidence of Immelmann lateral shape. No numerical lateral limit is inferred from perspective photographs.

For the vertical loop here, use one coherent offset toward the required exit side and a nearly parallel exit. Do not impose a global yaw curve that sends the path across both sides of its entry plane.

## Historical and current geometry

Historical source commit `a3a02c9` in the local VibeCoasterjs repository is 0.8.3. It constrains the descending low arm to `(0,+18,0)` relative to the ascending arm in local core coordinates. Core is right-handed with Z up; for +X entry, +Y is rider-left. The Unreal conversion `(100*x,-100*y,100*z)` preserves this rider-relative side.

The 0.9.0-flight.1 reference is an accepted saved artifact; no corresponding source revision was found in local history. Its sampled loop shifts monotonically about17.93m left and exits within0.57deg of the entering heading. The rejected resume-06 loop instead reverses lateral direction twice and has its descending low arm approximately10m right of the ascending arm. Its Immelmann also changes sides; the historical artifact rolls out consistently left.

Use only the corrected comparison files under `scratch/historical-inversion-completion/` (the filenames include `corrected`). The initial scratch comparison used an incorrect frame transform and an unaccepted baseline and was rejected during review.

## Physical implementation intent

The loop source must independently prove the signed left crossing, monotone lateral displacement, near-parallel exit, inherited derivatives and physical forces. Absolute separation or an endpoint yaw scalar alone is inadequate.

For the Immelmann, forcing rider lateral force to zero during a loaded roll rotates the complete normal force sideways and steers the centerline. The proposed planar roll projects its support envelope into the actual rider axes, allowing the real lateral component while keeping the world resultant in the vertical plane. That is a physical force law, not a telemetry clamp or coordinate correction. Source reconstruction, full-train forces and the final path must establish the result.

## Current source implementation and proof

The default recipe requests no separate loop-plane yaw and uses a signed 18 m crossing on the rider-left side of the inherited entry plane. At its retained isolated fixture port, the source export contains 4,256 points over 538.538 m / 10.571 s; its entry-relative left displacement rises monotonically to +18.00004 m, then exits with near-zero lateral tangent. The default Immelmann requests zero yaw and enables the planar half-roll. Its isolated fixture (before final connector/exit placement) has a 4,765-point export spanning 547.650 m / 11.793 s, with maximum lateral centerline deviation 1.22e-8 m, actual lateral load from -0.300 to +0.300 G, and normal load up to 3.8 G. Source assertions in native/core/tests/inherited_port_tests.cpp and native/core/tests/fvd_tests.cpp also cover inherited jets, saved-source replay, clearance, both roll hands and rejection of conflicting yaw. The path exports are scratch/loop-crossing/points.csv and scratch/loop-crossing/immelmann-0.350000.csv.

The completed route now passes native acceptance in `out/riftwake-completion-07-report.json`:181.817708s with all track, terrain, support, station, source, force, temporal and spatial checks passing. A180m low rollover separates the two inversion bodies, and the Immelmann recovery exits20m above its own entry so the short signature approach can bridge above the loop. Neither spacing change adds lateral weaving inside an inversion. Final section comparisons are retained in `scratch/terrain-fit-completion/final-clearance-audit.md`. Both actual packaged front/rear traversals pass, and the assistant inspected their inversion captures; see `docs/checkpoints/10-riftwake-completion.md`.
