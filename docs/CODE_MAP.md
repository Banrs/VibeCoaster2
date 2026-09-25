# Code map

This is the short route into the active default.3 project. The current design problems and next-task research are in [HANDOFF.md](../HANDOFF.md). Historical run data lives in [checkpoint 10](checkpoints/10-riftwake-completion.md); [checkpoint 11](checkpoints/11-riftwake-fvd-redesign.md) documents the rejected default.4 attempt.

## Runtime path

| Area | Start here | Responsibility |
| --- | --- | --- |
| Playable selector | dist/current.json; native/unreal/scripts/create_shortcuts.ps1 | Selects and verifies the packaged executable and accepted profile. |
| Unreal game | native/unreal/Source/VibeCoaster/Private/VibeCoasterWorld.cpp | Starts the load/generation job, commits the scene, and connects the native core to the game. |
| Unreal geometry/view | native/unreal/Source/VibeCoaster/Private/CoasterMesh.cpp; CoasterRuntimeVerification.cpp | Builds visible track/supports and drives packaged diagnostic captures. |
| Public core API | native/core/include/coaster/coaster.hpp | Defines the main design, validation and generation boundary used by CLI and Unreal. |
| CLI | native/core/src/main.cpp | Generate, validate, save and report from the command line. |
| Generation | native/core/src/generation.cpp | Orchestrates candidate creation, compilation, replay and acceptance. |
| Recipe/FVD | native/core/src/recipe_compiler.cpp; fvd.cpp; authoring.cpp | Compiles authored elements and motion into track geometry. |
| Physics and clearance | native/core/src/simulation.cpp; motion_audit.cpp; track.cpp; clearance.cpp | Replays trains and checks forces, geometry and swept clearances. |
| Persistence | native/core/src/persistence.cpp | Saves and loads designs with fresh validation. |
| Art import | native/art/README.md; native/unreal/scripts/import_v3_art.py | Authors the V3 Blender kit and imports its meshes/materials into Unreal. |

The native build graph is [native/CMakeLists.txt](../native/CMakeLists.txt). The maintained Windows/macOS configure and test commands are in [.github/workflows/native.yml](../.github/workflows/native.yml). A focused local check is to build coaster_cli, then run coaster_cli validate out/riftwake-completion-07.vcdesign. The cleanup already did that once against restored default.3 source. A source-changing task should run relevant component checks; the old local CTest log predates the rebuild and contains a motion_spline failure, so it is not a current pass.

Packaging uses native/unreal/scripts/package.ps1 on Windows or package_macos.sh on macOS. The default shortcut mode follows dist/current.json; -Development selects a local Unreal build and UserData-Development. The active packaged game and UserData-Riftwake-V3 profile are local ignored artifacts, not source files.

## Graph navigation

The root graphify-out graph covers native/core only: 81 files, 2,099 nodes and 4,933 edges after the default.3 refresh. Broad natural-language BFS queries are noisy. From the repo root, use source-qualified queries such as:

    graphify explain 'src/main.cpp::main()'
    graphify explain 'src/generation.cpp::generate()'
    graphify explain 'src/recipe_compiler.cpp::compileRecipe()'
    graphify explain 'src/clearance.cpp::buildClearanceSweepVerified()'

A separate focused Unreal graph lives under native/unreal/Source/VibeCoaster/graphify-out: 37 source files, 473 nodes and 632 edges. From native/unreal/Source/VibeCoaster, use graphify explain with a source-qualified symbol such as Private/VibeCoasterWorld.cpp::AVibeCoasterWorld::StartQueuedJob(). Its call graph does not show every cross-function call: the Generate to StartQueuedJob to coaster::generate chain is present in source but missing as graph edges. Confirm behaviour in source. Blender art scripts are outside both graphs. Generated graph, build, out, package and profile files are ignored by Git. Rejected default.4 and old workspace material were moved to D:\Coding\Codex\vibecoasterlegacy; see its rejected-default4-20260925 and workspace-deflation-20260925 manifests.

## Evidence boundary

The active save is out/riftwake-completion-07.vcdesign. dist/current.json, out/default3-restoration.json and the retained out/completion-runtime-front/result.json and rear result are the quickest way to check playable identity. The 100-warm startup baseline is out/startup-final-100-20260925/evidence.md. It misses the 3-second target; details and limits are in HANDOFF.md.