# VibeCoaster

C++20 coaster generation, finite-train simulation and an Unreal 5.8 viewer.
The new default, **Faultline**, starts in a ravine, climbs to one cliff reveal,
then uses a floating camelback and an overbanked wall turn to head home. Its
recipe is editable; inversions and the historical element sequence are optional.

The generator constructs one forward route. Each section inherits geometry,
rider frame and finite-train energy. Adaptive polynomial spans retain C4
centerline and C3 orientation continuity without a dense control-point grid.
Local station closure finishes the draft; independent physics and clearance
checks assess the completed ride. See [the design brief](docs/references/creative-redesign.md)
and [measurements](docs/diagnostics/faultline/README.md). The three-seed Linux
checkpoint generates accepted rides in 3.01–3.17 seconds; original main took
28.74 seconds for seed 42 on this workspace.

## Build and run

```sh
cmake -S native -B native/build -DCMAKE_BUILD_TYPE=Release
cmake --build native/build --parallel 8
native/build/coaster_cli generate --seed 42 --terrain highlands --out ride.coaster --json ride.json --trace ride-trace.json --plan ride-plan.json
native/build/ride_feel_tests --design ride.coaster
python3 native/tools/ride_review.py ride --out ride-review.html
```

Visual Studio builds put executables under `native/build/Release`. Only complete
accepted rides can be saved; loading freshly validates geometry, physics and
clearance. Export editable intent with `coaster_cli recipe --out ride.vcrecipe`,
then pass `--recipe ride.vcrecipe` to generation. Distances are metres and the
launch target measures rest to 180 km/h. The default 1.4-second launch is intentional.
For the default recipe, `--height` controls cliff relief above the station,
`--speed-kmh` the floor boost, and `--turnaround-height` the nominal wall-turn
rise. An explicit recipe owns its element dimensions.
Brakes are planned with the recipe. Candidate and closure-iteration controls
have been retired because generation has no whole-route search loop. Recipe
approach lengths are preferred planning scales. Nominal force and rate values
express ride character; actual signed duration, combined axes and load history
are assessed separately, without a universal percentage allowance.

## Inspect the ride

Open `ride-review.html` in a browser for an offline replay of the generated
geometry, terrain, structures and measured forces. Orbit the layout, scrub the
timeline or ride in the front, middle and rear seats. The review uses sampled
native output; the full-resolution native checks remain authoritative. CI
includes this interactive artifact with each accepted baseline.

Launch Unreal with `-CoasterGenerate -CoasterSeed=42`. Development shortcuts
explicitly generate current source in an isolated profile. `-CoasterLoad` loads
a historical save. See [camera controls](docs/camera-controls.md) for rider,
orbit and free-flight inspection.

Reports include physical loads, energy, section pacing and ride-feel diagnostics.
A few regressions cover meaningful airtime, contrasting force beats, inactive
gaps and continuous roll. These complement independent physics checks; passing
numbers do not establish visual quality or rider enjoyment. This is a coaster
authoring game, not an engineering certification tool.

Start with [HANDOFF.md](HANDOFF.md), [the code map](docs/CODE_MAP.md) and
[design research](docs/references/generator-rewrite-research.md). Historical
default.3 saves and art remain under `docs/diagnostics` and `native/art`.
The earlier [branch consolidation](docs/BRANCH_CONSOLIDATION.md) is history,
not a second active implementation.
