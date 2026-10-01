# Code map

This is the short route into the active default.3 project. The current design problems and next-task research are in [HANDOFF.md](../HANDOFF.md). Historical run data lives in [checkpoint 10](checkpoints/10-riftwake-completion.md); [checkpoint 11](checkpoints/11-riftwake-fvd-redesign.md) documents the rejected default.4 attempt.

## Runtime path

| Area | Start here | Responsibility |
| --- | --- | --- |
| Playable selector | dist/current.json; native/unreal/scripts/create_shortcuts.ps1 | Selects and verifies the packaged executable and accepted profile. |
| Unreal game | native/unreal/Source/VibeCoaster/Private/VibeCoasterWorld.cpp | Starts the load/generation job, commits the scene, and connects the native core to the game. |
| Inspection cameras | native/unreal/Source/VibeCoaster/Public/CoasterCamera.h; Private/CoasterCamera.cpp; Private/VibeCoasterGame.cpp | Orbit/pan/free-flight state, controls and HUD. See [controls](camera-controls.md). |
| Unreal geometry/view | native/unreal/Source/VibeCoaster/Private/CoasterMesh.cpp; CoasterRuntimeVerification.cpp | Builds visible track/supports and drives packaged diagnostic captures. |
| Public core API | native/core/include/coaster/coaster.hpp | Defines the main design, validation and generation boundary used by CLI and Unreal. |
| CLI | native/core/src/main.cpp | Generate, validate, save and report from the command line. |
| Generation | native/core/src/generation.cpp; acceptance_replay.hpp | Candidate orchestration and the shared generation/load acceptance pipeline. |
| Recipe/FVD | native/core/src/recipe_compiler.cpp; fvd.cpp; authoring.cpp | Compiles authored elements and motion into track geometry. |
| Physics and clearance | native/core/src/simulation.cpp; motion_audit.cpp; track.cpp; clearance.cpp | Replays trains and checks forces, geometry and swept clearances. |
| Adaptive supports | native/core/src/supports.cpp; support_assemblies.hpp; support_bridge.hpp; support_twisted.hpp; support_cliff.hpp; native/tools/support_review.cpp | Terrain-aware whole-element frames, shared swept girders and trestles, fitted cliff terrain and wall sockets; shared foundation graph validation and canonical Blender exports. See [support system](support-system.md). |
| Exa profile and fabrication | native/core/include/coaster/track_profile.hpp; track_mesh.hpp; native/core/src/support_fabrication.cpp; native/art/import_native_exa.py | Persisted section dimensions, fitted support meshes and full visible clearance. The same portable geometry feeds Blender MCP and Unreal. Legacy saves keep their section. See [integration](exa-runtime-integration.md). |
| Support visual review | native/art/import_native_exa.py; update_native_supports.py; style_native_review.py; render_native_ride.py; render_native_support_review.py; audit_native_support_joints.py; extract_native_joint_meshes.py; support_review_terrain.py | Complete saved ride imported as native triangles in live Blender MCP, verified support-only replacement, external ground/joint/rock-anchor cameras and triangulated tube-end checks. Current review: out/support-adapted/ride; out/support-direct is the preceding comparison. |
| Historical fabrication study | native/art/support_detail_geometry.py; author_support_study.py; verify_support_details.py; verify_support_scene.py | Original editable art and dimension checks. New native scenes sit alongside these retained studies. |
| Persistence | native/core/src/persistence.cpp; acceptance_replay.hpp | Atomic saves, fresh load validation, and worker cancellation/join ownership. |
| Train model | native/art/TRAIN_DESIGN.md; train_profile.json; author_train.py; verify_train_model.py; render_train.py; export_train.py | Editable seven-car Blender proposal and train-only GLB. Transverse compact wheel clamps follow the user's correction. No runtime adoption. |
| Art import | native/art/README.md; native/unreal/scripts/import_v3_art.py | Authors station/hardware only. New train art is a separate review proposal; dimensions are in native/art/DIMENSIONS.md. |
| Exa track study | native/art/TRACK_STUDY.md; native/art/track_study_profile.json; native/art/author_track_study.py; native/unreal/scripts/import_track_study.py | Editable Blender MCP track section, shared support datum, focused geometry checks and an isolated Unreal review map. Not yet adopted by the live procedural ride. |
| Diagnostic plots | native/tools/render_motion_report.py | Draws front/middle/rear plots from CLI reports and traces. |

The native build graph is [native/CMakeLists.txt](../native/CMakeLists.txt).
Windows/macOS configure and test commands are in
[.github/workflows/native.yml](../.github/workflows/native.yml). Use
`cmake --build native/build --parallel 8` with eight-CPU affinity, then
`ctest --test-dir native/build --output-on-failure --parallel 1`.
`-DBUILD_TESTING=OFF` builds only the core and CLI. To check the retained save,
run `coaster_cli validate out/riftwake-completion-07.vcdesign`.
Current verification evidence is described in HANDOFF.md.

Packaging uses native/unreal/scripts/package.ps1 on Windows or package_macos.sh on macOS. The current Play shortcuts use -GeometryReview: the local Unreal build with UserData-GeometryReview. Recreating shortcuts without a mode follows dist/current.json; -Development uses UserData-Development. The active packaged game and UserData-Riftwake-V3 profile are local ignored artifacts, not source files.

## Where to start for the current issues

| Problem | First files to inspect |
| --- | --- |
| Forces, stuck angles, banking and flat resets | native/core/src/fvd.cpp; authoring.cpp; motion_program.cpp; motion_audit.cpp |
| LSM placement, waits and trim brakes | native/core/src/recipe_compiler.cpp; native/core/src/drive_profile.cpp; native/core/src/trim_layout.hpp |
| Ground/track clearance and load time | native/core/src/clearance.cpp; track.cpp; generation.cpp; native/unreal/Source/VibeCoaster/Private/VibeCoasterWorld.cpp |
| Train view and diagnostic cameras | native/unreal/Source/VibeCoaster/Private/CoasterCamera.cpp; VibeCoasterGame.cpp; VibeCoasterWorld.cpp; native/art/DIMENSIONS.md |

## Optional Graphify inspection

The stale core and Unreal graph caches were deleted during this refactor.
Start with the paths above. Generate a focused index only if a particular symbol
relationship needs it, and confirm changing code directly. Blender art is not
covered by a C++ graph. Build, out, package and profile data are ignored by Git;
external archive manifests are historical evidence.

## Evidence boundary

The active save is out/riftwake-completion-07.vcdesign. dist/current.json, out/default3-restoration.json and the retained out/completion-runtime-front/result.json and rear result are the quickest way to check playable identity. The 100-warm startup baseline is out/startup-final-100-20260925/evidence.md. It misses the 3-second target; details and limits are in HANDOFF.md.
