# VibeCoaster2

C++20 / Unreal 5.8 coaster authoring with editable mixed FVD and spatial-spline
sources, independent finite-train dynamics and scoped F2291-25 acceleration
assessment. The verified Windows preview is **3.0.0-preview.1**.

The lower-park station, powered ascent and fixed 210 m escarpment follow the
Falcon's Flight-style site decision. Force-critical hills, inversions, the cliff
and protected asymmetric camelback use FVD; grounded routing, placement, the
ravine roll and closure use spatial splines. The default provides 180 s active
riding, 3.8 s lip braking and 8.14 s terminal braking.

On the tested Ryzen 9 5900X / RTX 5070 Ti host, packaged saved-load p99 through
actual GPU readiness is **2.360 s for first loads and 2.364 s for repeat loads**
(300 each). Startup is separate; OS/driver caches were not reset. Generation
median is **7.675 s**, 75.7% lower than the matched Escarpment baseline.
See the [release report](docs/release-3.0.0-preview.1.md) for samples, scope and identity.

## Play and author

On this configured Windows workspace, run `Play.cmd` (PowerShell 7 required).
The launcher checks the promoted package manifest and executable/assets before
opening a 1600x900 window at 60 FPS. It loads the user's saved design when one
exists, otherwise the shipped default. Space toggles playback, R restarts and V
changes view. Recipe controls author a new ride; Save updates the next Load target.
`Play.cmd -Verify` tests the same launcher and records actual GPU readiness.

Build the portable core with CMake 3.24 or later:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 1
ctest --test-dir build -C Release --output-on-failure --parallel 1
```

Windows and macOS native CI pass. On this development Windows machine,
`scripts/build.ps1 -Test` uses its configured MSVC toolchain. All native and UE
builds use one compiler worker. A Windows game package is GPU-tested; a macOS
Unreal package has not been built or GPU-tested.

To create a package from clean committed source, generate a validated fixture
with `build/vibe.exe save recipes/default.vcr`, then run
`scripts/package.ps1 -Design out/candidate.vcd`. This requires Unreal 5.8.
Packaging produces a new versioned archive and manifest; promotion follows runtime
verification, not a successful build alone.

## Verification and scope

Every saved load repeats source replay, nominal/operating/refinement dynamics,
modeled drive coverage and continuous clearance of shared vehicle, occupant,
terrain, track, support and station geometry. A saved checksum never substitutes
for physical validation. The previous ride remains available during preparation;
cancellation and rejected replacements preserve it.

Nine representative recipes, 235 acceleration fixtures, persistence regressions,
both complete packaged traversals, the generation/save/load/cancel/rejection flow,
600 paired timing loads and a 40-load session pass. The full authoring parameter
space is not claimed feasible. Detailed art, full GUI layout editing and VR are
later work. Equipment capacity, detailed actuators, structural strength and
whole-standard certification are outside this explicit simulation model.

See [authoring decisions](docs/rewrite.md), [validation details](docs/progress.md),
[acceleration scope](docs/acceleration-standard.md),
[modeled drive demand](docs/hardware-coverage.md) and
[runtime reproduction](docs/runtime-verification.md).
The parent repository, VibeCoasterjs, archived implementation and playable fallbacks
remain preserved outside this independent repository.
