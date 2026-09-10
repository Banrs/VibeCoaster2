# Terrain routing and code simplification checkpoint

Source checkpoint verified; packaged traversal and distribution are being checked separately. This is not a completed-foundation claim.

## Changes

The existing folded generator jointly ranks actual source footprints, independent ascent/return angles and exact closure against the unchanged seeded canyon. The final rigid hill chain exits on the real rim. One active septic window places each climb/descent against terrain; it no longer stretches the height transition across the whole available corridor. Crossing clearance finds the smallest common rigid tail lift that clears every crossing, preserving existing underpasses. A real 100 m straight arrival occupies the station reservation and is included once in closure and stopping room.

The powered climb has a separate 150 km/h design target, based on [Intamin's Falcon's Flight description](https://www.intamin.com/2026/01/15/six-flags-qiddiya-city-falcons-flight/). Its force/power-limited motor, gravity and losses determine actual speed. Signature targets are unchanged. All ordinary turns use measured maximum any-car occupancy speed, including the formerly fixed-speed terminal turn. Loop brake capacity accounts for actual passive approach work. A reached loop stall may correct only the existing upstream motor, within the existing eight-rebuild budget. The 86-degree bank clamp was removed; canonical rider forces still determine acceptance.

Two test-only reversing-module adapters and their private integration helpers were retired. Active force-authored reversal/pullout and full-loop implementations remain. The unused terrain-jet path and its state were also removed. This intentionally retires obsolete pre-1.0 source APIs; exact-version saved geometry is not migrated. Root AGENTS.md records the user's maintenance requirements for all code and future commits. Useful behavior checks remain on active code; deletions do not remove force, terrain, target or persistence gates.

## Evidence and limits

Intermediate variants and failures remain in this directory, including v9/v13 panel failures and their causal energy, banking and station investigations. The frozen v16 panel accepts all six requests on candidate zero, including hills9 with only one candidate allowed. All six save after independent replay and pass960/1920Hz convergence. Moving times are140.027–172.440s; full stops164.755–207.575s. Canyon42 ascent/return max sampled centerline AGL improves from165/205m to approximately30/47m; canyon9 still reaches91m locally on its ascent. These5m samples are not continuous extrema or structural approval. See panel-v16/REPORT.md and its raw local evidence.

All26 native suites passed across ctest-v16.log and the targeted ctest-core-fixture.log rerun (354.17s). The initial full run's sole failure was a historical hills1 candidate-index expectation; complete acceptance, swept clearance, determinism and terrain-variation checks remain. The hills9 one-candidate station regression passes unchanged after correcting feedback damping. Python discovered178 tests:172 passed and six actual-CLI cases run separately in CTest. Unreal built and passed all seven contracts in Saved/BuildRuns/20260910-072107-258. No skips or force/target limits were added.

The production/API diff is53 lines smaller and code including tests106 lines smaller, before documentation and evidence. This includes new routing/energy mathematics and retires unused implementations rather than compressing formatting. Eight frames from two actual intermediate editor-game traversals were inspected; their reviews explicitly identify v13/v15 predecessors, not final packaged generation. Source version is 0.8.2-terrain.1 / COASTER5; the published ae09f02 package is preserved.

The [ASTM publisher page](https://store.astm.org/f2291-24.html) identifies clearance/containment as part of F2291. Current geometry checks are explicit train/hardware/terrain/station sweeps. They are not a complete patron reach, restraint, structural, control-system or full current F2291 assessment. No arbitrary smaller clearance was introduced. No eligible real I305/Pantherian reference has been supplied by this change; synthetic feasibility tests cannot qualify ALL RECORDS.

Terrain alignment improves but the restricted folded grammar still produces long approaches and longer canyon rides. Scientific geometry plots and selected rendered views are evidence of what was inspected, not approval of every seed or a complete POV study. No new packaged FPS or Mac/Metal runtime claim follows.
