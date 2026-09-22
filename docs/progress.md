# Rewrite progress

This is a development checkpoint, not a final acceptance or certification claim.
The original ride, loading-tail, runtime, packaging and Play-identity requirements
remain the goal. Windows and macOS are the current CI targets; Linux is deferred.

## Authorship and station

The station is in the lower park, like Falcon's Flight. A powered ascent leads
to the clifftop. The fixed site has one 210 m escarpment and modest relief; its
height is never fitted to the rail samples.

Force-critical hills, inversions, the cliff and protected camelback use FVD.
Spatial splines handle placement, grounded turns, the descending roll and the
station return. Both compile into the same position/orientation representation
and are assessed using the same independent finite-train simulation. OpenFVD
was consulted as a design reference; its GPL source was not copied.

The recipe retains seed, style and real height/speed parameters. Solved source
controls, semantic roles and inherited entry states survive the mixed-source
save format. Seed 77 changes the layout; height edits change measured geometry.

## Current native evidence

The current nine-case corpus passes: seeds 42/77, balanced/flow/intense,
290/300/305 km/h, 65/75/85 m opening hills, and a combined seed-77 edit with an
80 m opening, 140 m loop and 100 m Immelmann. This is representative coverage,
not proof that every combination in the authoring ranges is feasible.

The default result is 179.9958 s active, 3.8 s lip braking and 8.1438 s terminal
braking, with a closed 8,312.67 m circuit. It reaches 180 km/h in 1.3833 s and
300.0002 km/h at the main boost. The terminal descent stays above the valley
crossing before descending to the station. The return includes two lower
height elements and a broad crossing crest.

Front, middle and rear seats pass the retained scoped F2291-25 assessment,
nominal component envelopes and 20 g/s rate caps. The default also passes
full/trims and 20% lower-drag scenarios. Lower drag shortens the active ride to
approximately 174.2–174.7 s; these are operating scenarios, not the nominal
180-second authoring target. No scoped ASTM rule was relaxed.

Continuous occupied-box bounds certify terrain and nonlocal track clearance.
The former 25 m local exclusion has been reduced to 6 m, with a regression for
a crossing only 20 m away along the route. This does not yet certify support,
station, full vehicle articulation or all hardware clearances.

Independent source replay is within about 0.021 mm on the default. Halving
spatial and temporal steps gives about 0.0019 mm replay error; the finite-train
work-energy residual falls from 0.000260 to 0.0000648 J/kg. The reported source
speed interpolation discrepancy is an authoring-reference quantity, not an
energy reset in the separate train simulation.

The protected default camelback profile is retained. At 290 km/h a bounded
study found that moving the existing crown relief slightly later resolves a
front-seat post-negative-history failure. Its duration and peak were retained;
the history evaluator was unchanged. Full-route checks subsequently passed.

The return solver now caches its affine heading weights, deduplicates roots
and re-solves the previous route family during train-speed calibration. Every
candidate still undergoes physical and clearance checks. Recent local runs
were roughly 5–7 s; these are development observations, not the final matched
Escarpment performance comparison.

Mixed-source persistence tests cover round trips, fresh nominal assessment,
cancellation after writing a temporary replacement, byte corruption and a
rechecksummed design buried in the terrain. Failed/cancelled replacement keeps
the existing save. The 235 retained acceleration fixtures also pass.

## Unreal and performance

Unreal 5.8.2 and MSVC are installed locally. Builds use one compiler worker.
The fresh module provides editable controls, asynchronous generation/loading,
cancellation, retained previous scenes, save/load, front/rear views and a GPU
fence following actual back-buffer rendering. Scene resources are explicitly
released during replacement, and the material has strong UObject ownership.

Earlier preview fences reported roughly 1.5–1.8 s, but visual inspection showed
fallback colours and unfinished material compilation. Those timings are not
accepted load evidence. The local runtime now checks material resources before
starting its final rendered-frame fence. The standalone editor material-cache
state is still being repaired and retested; these runtime edits are not part
of this native checkpoint. No p99 or final GPU-readiness claim exists.
A preserved default package loaded the archived fixture in about 3.992 s to its
old scene-commit event. That experiment did not reproduce the reported 20–30 s
saved-load delay and did not establish old GPU readiness. The archived 25.204 s
log is labelled generation, not saved loading. Startup and file-cache state
must remain separate in subsequent measurements.

## Remaining acceptance work

Complete activation validation must include operating and refinement checks,
recipe/source semantics, explicit peak-allowance reporting, and physical motor
coverage. Support/station clearance, rendered-terrain agreement, ravine terrain
and visual composition require further work. Full front/rear GPU traversals,
save/load/cancel retention, sufficient cold/warm load-tail samples, the matched
generation baseline, packaged builds and exact Play executable identity remain
open. A native pass is never labelled final ride acceptance.

The parent repository, VibeCoasterjs, archived implementation and playable
fallbacks remain untouched. The independent rewrite branch is
`codex/fresh-rewrite` in `Banrs/VibeCoaster2`.

The computer-use helper remains unavailable because sandbox setup cannot add
its protection to `.git`, which is owned by `CodexSandboxOnline`. A narrowly
scoped owner-only repair was prepared, but has not been run; it awaits the
specific approval requested after automatic review rejected the administrative
ownership change. No sandbox or filesystem security controls were disabled.
