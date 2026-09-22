# Rewrite progress

Development checkpoint. The original ride, scene-clearance, loading-tail,
packaging and Play-identity requirements remain the goal. Windows and macOS are
the CI targets; Linux is deferred.

## Authorship and site

The station is in the lower park, like Falcon's Flight. A purposeful powered
ascent reaches the 210 m plateau. Terrain is independent of the rail samples:
one escarpment, modest relief and a fixed 14 m ravine. The descending full roll
is routed to (790, -660, -8) m within that ravine; an FVD hill climbs back out.

Force-critical hills, inversions, the cliff and protected camelback use FVD.
Spatial splines handle placement, grounded turns, the roll and station return.
Both compile into the same position/orientation representation. Editable recipe
parameters, stable element IDs, solved controls and inherited states survive
persistence. Seed 77 changes the layout and height edits change measured geometry.
OpenFVD was consulted as a design reference; its GPL code was not copied.

## Native evidence

All nine representative cases pass: seeds 42/77, balanced/flow/intense,
290/300/305 km/h, 65/75/85 m openings, and a combined seed-77 edit with an 80 m
opening, 140 m loop and 100 m Immelmann. Every case passes nominal, full-mode,
20% lower-drag trims/full, temporal-only, spatial-only and combined refinement.
This corpus does not prove every combination in the authoring ranges feasible.

The default is 179.9990 s active, 3.8 s lip braking and 8.1417 s terminal braking
on a closed 8,310.97 m circuit. It reaches 180 km/h in 1.3833 s and 300.0002 km/h
at the main boost. Lower-drag operating scenarios shorten the active duration
to approximately 174–175 s; 180 s is the nominal authoring target.

Continuous front/middle/rear force histories pass the retained scoped F2291-25
assessment, project peak envelopes and 20 g/s rate caps. Nominal compliance,
strictly sub-1% peak allowance and maximum excess are separately reported.
Exact decimal 1% boundaries are rejected. No scoped ASTM threshold was relaxed.
The default needs no peak allowance.

Expanded operating checks exposed failures in taller opening hills, the faster
Immelmann/ravine sequence and the lower-speed terminal descent. Bounded changes
to force targets, roll progression, routing and terminal curvature resolved them.
The default protected camelback remains intact. Its established 290 km/h crown
adjustment retains the same relief duration and peak.

Independent source replay is within about 0.021 mm on the default, and refined
replay is about 0.0019 mm. Halving the time step reduces the finite-train
work-energy residual from about 0.000260 to 0.0000648 J/kg. Source reference
velocities never reset the independent finite-train energy evolution.

Continuous occupied-box bounds certify terrain and nonlocal track clearance.
Rendering and clearance share the same indexed terrain triangles, including a
1 mm rendering precision allowance. Tests cover ravine shoulders, cell edges,
nearby nonadjacent crossings and cancellation. Support, station, complete
vehicle articulation and hardware clearance are still separate open checks.

Saved format 3 records fixed-site revision 2. Earlier development saves are
explicitly rejected instead of being reinterpreted against changed terrain.
Loading rebuilds its operating reference and all seven dynamics assessments,
independent replay and terrain/track clearance. Evidence is published only after
every check passes; no saved approval is trusted. Independent checks use up to
eight workers, while cancellation callbacks are serialized.

Regressions cover integrity-valid height/speed/role mismatches, buried geometry,
unsupported save/site versions, corruption, late save cancellation, worker-thread
cancellation and stale-evidence removal. A failing test exposed numerical fitting
swallowing cancellation; cancellation now bypasses trial-recovery catches.
The 235 retained acceleration fixtures pass. The audit command reports the fresh
validation results without recomputing them.

## Runtime evidence and timing

Unreal 5.8.2 builds with one compiler worker. The preview supports asynchronous
generation/loading, editable recipe controls, save/load, cancellation, retained
previous scenes and front/rear views. Material completeness, actual back-buffer
frames and a GPU fence determine readiness. Material and GPU timeouts restore
the prior scene. Resources are explicitly released on replacement and shutdown.

The current terrain/route/cameras completed full front and rear traversals of
191.94 s each, with about 45,000 rendered frames per view and process exit 0.
The automated flow also passed seed-77/intense generation, save/reload, restored
authoring controls, CPU/upload/GPU cancellation and corrupt-load rejection.
Every cancelled or rejected replacement retained the playing, rendered prior
ride. These tests use the real runtime handlers; they are not manual mouse tests.

Three fully validated preview loads reached GPU readiness in 2.387, 2.532 and
2.547 s. Exact polynomial derivative evaluation reduced matched native loads
of one unchanged save from median 2.808 to 2.368 s in three paired samples.
These are small development samples, not p99 or release acceptance. An earlier
first request after material rebuilding took 5.77 s; that tail remains recorded.

A preserved default package loaded its archived fixture in about 3.992 s to the
old scene-commit event. This did not reproduce the reported 20–30 s saved-load
delay or establish old GPU readiness. An archived 25.204 s event is generation,
not loading. Startup, process-cold and warm measurements must remain separate.

See [runtime-verification.md](runtime-verification.md) and the retained evidence
for exact scope. The preceding `6296602` checkpoint passed Windows/macOS CI:
https://github.com/Banrs/VibeCoaster2/actions/runs/35769490814 .

## Remaining acceptance work

Full support/station/vehicle clearance, physical motor coverage, further visual
composition, sufficient cold/warm load-tail samples, the matched Escarpment
generation baseline, versioned packages and exact tested Play executable identity
remain open. Native and automated GPU passes alone are not final ride acceptance.

The parent repository, VibeCoasterjs, archived implementation and playable
fallbacks remain untouched. Work is on `codex/fresh-rewrite` in `Banrs/VibeCoaster2`.

Computer use remains blocked by sandbox setup's inability to protect `.git`,
whose owner is `CodexSandboxOnline`. An owner-only repair is prepared but awaits
the specific approval requested after automatic review rejected the administrative
ownership change. No sandbox or filesystem security controls were disabled.