# Code map

## Generation and review

| Area | Entry | Responsibility |
| --- | --- | --- |
| Public native boundary | `native/core/include/coaster/coaster.hpp` | Canonical track, design, train, operations and validation API shared by CLI and Unreal. |
| Orchestration | `native/core/src/generation.cpp` | Request checks, compilation, frozen-route structures and independent acceptance. |
| Route plan and composition | `route_plan.hpp`, `route_composer.cpp` | Resolved typed intent, source ports, per-role phrases, terrain and hardware corridors. |
| Physical solves | `recipe_compiler.cpp` | Bounded finite-train energy fitting and station closure; finalizes frames before validation. |
| Geometry primitives | `authoring.cpp`, `motion_program.cpp`, `fvd.cpp` | Inherited geometry jets, direction splines and force-authored elements. |
| Physical frame | `banking.cpp` | Source-owned bank intervals; immutable FVD and hardware boundaries. |
| Acceptance | `acceptance_replay.hpp` | Independent coarse/fine train replay, authored-source checks and spatial refinement. |
| Experience | `ride_feel.cpp`, `motion_audit.cpp` | Front/middle/rear elapsed-time experience, continuity, pacing and recipe semantics. |
| Simulation | `simulation.cpp`, `acceleration.cpp` | Finite-train motion, forces, energy, acceleration histories. |
| Spatial checks | `clearance.cpp`, `track.cpp`, `structures.cpp` | Occupied train envelope, geometry and structure clearance. |
| Persistence | `persistence.cpp` | Atomic saves, fresh load validation, original generation identity. |
| Command line | `main.cpp` | Generate, validate and recipe commands; report, trace and plan export. |

Paths above without a prefix are under `native/core/src`.

## Viewer and assets

Unreal starts at `native/unreal/Source/VibeCoaster/Private/VibeCoasterWorld.cpp`.
Its asynchronous job invokes the native API and publishes only accepted scenes.
`VibeCoasterGame.cpp` owns input and startup mode. `CoasterCamera.cpp` owns rider,
orbit and free cameras; `CoasterMesh.cpp` builds scene geometry. Every core `.cpp`
is compiled into Unreal through a corresponding `Private/Core*.cpp` wrapper.

Development startup uses `-CoasterGenerate`; `-CoasterLoad` is an explicit saved
profile review. See [camera controls](camera-controls.md). Windows packaging is
`native/unreal/scripts/package.ps1`; macOS uses `package_macos.sh`.

Support generation is in `supports.cpp` and `support_*.hpp`; fabrication is in
`support_fabrication.cpp`. Existing art authoring and editable train assets are
under `native/art`. The train proposal is not automatically adopted by v3.

## Verification

```sh
cmake -S native -B native/build -DCMAKE_BUILD_TYPE=Release
cmake --build native/build --parallel 8
ctest --test-dir native/build --output-on-failure -R '^(ride_feel|motion_spline|frame_force|recipe|cli_arguments)$'
native/build/coaster_cli generate --seed 42 --terrain highlands --out ride.coaster --json ride.json
native/build/ride_feel_tests --design ride.coaster
```

CI runs component checks and one complete generation/save/reload/experience path.
Two additional seeds and broader existing integration tests run at explicit
checkpoints. Keep regression scope focused on behavior rather than canonical
track snapshots. Research is in [the rewrite research record](references/generator-rewrite-research.md).
Historical default.3 artifacts and restoration instructions are in
[ONLINE_DIAGNOSIS.md](ONLINE_DIAGNOSIS.md); they are comparison fixtures, not new
verification receipts.
