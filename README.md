# VibeCoaster

C++20 coaster generation, finite-train simulation and an Unreal 5.8 viewer.
Version 3 builds a single route forward. Each section inherits geometry, rider
frame and finite-train energy; the compiler appends a station approach to that
same draft. Adaptive polynomial spans represent the authored curves without a
dense control-point grid. Independent validation measures the finished ride.
Observed native generation is 7.5–7.8 seconds for the three-seed checkpoint,
down from 28.7 seconds on the original main; see [measurements](docs/diagnostics/rewrite/README.md).

## Build and run

```sh
cmake -S native -B native/build -DCMAKE_BUILD_TYPE=Release
cmake --build native/build --parallel 8
native/build/coaster_cli generate --seed 42 --terrain highlands --out ride.coaster --json ride.json --trace ride-trace.json --plan ride-plan.json
native/build/ride_feel_tests --design ride.coaster
```

Visual Studio builds put executables under `native/build/Release`. Only complete
accepted rides can be saved; loading freshly validates geometry, physics and
clearance. Export editable intent with `coaster_cli recipe --out ride.vcrecipe`,
then pass `--recipe ride.vcrecipe` to generation. Distances are metres and the
launch target measures rest to 180 km/h. The default 1.4-second launch is intentional.
Brakes are planned with the recipe. Candidate and closure-iteration controls
have been retired because generation has no whole-route search loop. Recipe
approach lengths are preferred planning scales. Nominal force and rate values
express ride character; actual signed duration, combined axes and load history
are assessed separately, without a universal percentage allowance.

## Inspect the ride

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
