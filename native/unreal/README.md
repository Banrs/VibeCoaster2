# VibeCoaster native Unreal game - review checkpoint

Recovery delivery **0.7.3-review.5** adds a 60fps default and a portable Windows EXE bundle; numerical/save identity remains **0.7.2-pacing.2 / COASTER5**. No new performance benchmarks were run while Cities: Skylines 2 was open. [Current download and status](../DELIVERY.md). The review.2 evidence below belongs to that previous package.

Application **0.7.3-review.2** has passed real Windows build/cook/package, five UE contracts and four final complete packaged runs. Numerical geometry and persistence remain **0.7.2-pacing.2 / COASTER5**, with existing p2 save compatibility verified. With Cities: Skylines 2 closed, three complete 1440p timing runs measured 152,697 traversal frames: pooled p95 **4.996ms**, maximum **12.862ms**, zero above 16.667 ms. This is focused Windows performance evidence, not completed foundation acceptance. [Review, exact scope and remaining limits](../artifacts/review-v073-20260908/REPORT.md).

Previous verified p2 delivery: [connected track webbing and package](../artifacts/track-web-v072p2-20260908/REPORT.md). Five UE contracts and build/cook/package passed. Its targeted canyon front-seat traversal/save/load, exact-save native replay and ten inspected packaged frames belong to p2. The [p1 pacing/train report](../artifacts/pacing-aero-v072-20260908/REPORT.md) separately records three terrain/seat runs and sixteen inspected frames. The p2 hardware change preserved those three full numerical traces byte-for-byte. These scopes must not be combined into a new application acceptance claim.

This continuing C++ UE5 implementation uses the portable G3/C2 numerical core in `../core` and is the single active game after the browser implementation was retired. UE5.8.2 is installed and has built the previous verified packages. Application identity is separate from numerical/save identity during this review. macOS remains a first-class target with prepared Apple Silicon packaging; an actual Mac host/build is still required.

**Preserved geometry.2 Windows checkpoint:** real UHT, editor and game compilation, asset creation, cooking and packaging pass. Three engine contracts and six scripted runs across flat/hills/canyon seed42 passed complete playback and accepted save/load. Actual2560×1440 viewport timing has two canyon hitches above 16.667 ms; it is not an every-frame60fps claim. See [integration report](../artifacts/ue58-integration-20260907/REPORT.md). The preserved, user-rejected 0.6 organic revision has [separate acceptance and scoped packaged review](../artifacts/organic-v060-20260907/REPORT.md), including six scripted runtime runs and measured frame times. Full continuous human POV and presentation quality remain unfinished. Strict-record comparison remains unavailable without eligible authentic force recordings.

The last verified archived executable is [run-20260907-222637-901/Windows/VibeCoaster.exe](Packaged/run-20260907-222637-901/Windows/VibeCoaster.exe). For the reviewed canyon example, explicitly select PHYSICS-PROOF, Canyon and seed42. Old packages remain available and must not be confused with this correction.

## Build and run

1. Complete UE5.8 installation through Epic Games Launcher, including Windows support; this installation is complete on the current host. The previously checked Epic setup table lists VS2022 17.14+, MSVC 14.38 minimum (14.50 recommended), and Windows SDK 10.0.22621.0 minimum (10.0.26100+ recommended). The installed Windows toolchain meets those documented C++ minima; UnrealBuildTool and the actual build must still validate the selected toolset and bundled .NET dependencies. See [Epic's VS setup table](https://dev.epicgames.com/documentation/en-us/unreal-engine/setting-up-visual-studio-development-environment-for-cplusplus-projects-in-unreal-engine).

2. Close any running editor using this project. From the repository root, run the following with your actual engine installation path.

```powershell
# Read-only UE5.8 engine/compiler/SDK inventory; does not certify download completion.
& .\native\unreal\scripts\check_prerequisites.ps1 -AsJson
# Fast validation that does not need Unreal.
& .\native\unreal\scripts\validate_source.ps1 -ZigPath .\native\.tools\zig-x86_64-windows-0.14.1\zig.exe
# Build the editor module, create map/material assets, run UE automation.
& .\native\unreal\scripts\package.ps1 -UnrealRoot 'D:\Games\Epic Games\UE_5.8' -PrepareOnly
# Build, cook, stage and archive a Development game after the same checks.
& .\native\unreal\scripts\package.ps1 -UnrealRoot 'D:\Games\Epic Games\UE_5.8'
# Optional distribution build after Development runtime checks succeed.
& .\native\unreal\scripts\package.ps1 -UnrealRoot 'D:\Games\Epic Games\UE_5.8' -Configuration Shipping
```

The packaging script creates a **fresh** `Packaged/run-<UTC timestamp>` archive and prints the real `VibeCoaster.exe` path only if UAT succeeds and that executable exists in the fresh archive. An old executable cannot satisfy this check. `-OutputDirectory` changes the archive parent. `-SkipAutomation` is an explicit diagnostic override; do not use it for an acceptance build.

To play in the editor after preparation, open `native/unreal/VibeCoaster.uproject`, open `/Game/Maps/Ride`, and select Play. All runtime actors, lighting and HUD are native C++; the map is deliberately minimal. A packaged game uses that same cooked map.

The bootstrap runs `scripts/create_content.py` through Epic's editor Python API. It creates seven real material assets (including the texture-free slope palette) and `Ride.umap`; those generated binaries are ignored by Git. Existing content is preserved. If an existing material needs redesign, edit it in the editor. Python and Editor Scripting Utilities are editor-only plugins. The runtime uses the built-in ProceduralMeshComponent plugin for canonical rails/terrain/supports, original Blender5.2.1 train/station/tie meshes under `/Game/Art/V072/Import1` (train/station) and `/Game/Art/V072/TrackWeb1` (tie/web), and exact cube fallbacks for unsupported station dimensions. The same source `.uasset` models and materials must accompany Mac builds. No paid plugin is required. See [art import and source workflow](../art/README.md); packaging requires all six runtime meshes and all five UE contracts.

Each Windows attempt retains logs, its bootstrap receipt and UE automation reports under `native/unreal/Saved/BuildRuns/<timestamp>`, including failed attempts. Build temporary files use `Saved/Temp`; DDC uses `Saved/DerivedDataCache`. A failed process, missing bootstrap receipt, missing asset, missing named test success, or missing fresh executable stops the packaging script.

For macOS preparation and packaging, use [MACOS.md](MACOS.md) and `scripts/package_macos.sh` on a Mac. Windows checks do not establish Metal rendering, macOS input/save behavior, packaging or performance. `check_prerequisites.ps1 -TargetVersion 5.6` remains available for an explicit historical Windows toolchain inventory; the active project association is 5.8.

## Riding and generating

The initial mode is **ALL RECORDS**. It is explicitly unavailable while the I305 exposure benchmark is missing; raw RFDB reference data is not bundled. There is no automatic fallback or preaccepted failed candidate.

Choose **PHYSICS-PROOF** yourself in the setup menu to disable only intensity comparison. Height, speed, inversion-height, launch, geometric and provisional force constraints still apply. Accepted proof rides remain visibly labeled `PHYSICS-PROOF; intensity untested`.

| Input | Action |
| --- | --- |
| Up/Down | Select seed, terrain, mode, record target or candidate budget |
| Left/Right | Change selected option; custom goals can be stricter than the baseline |
| Digits / Backspace / Delete | Edit the selected 64-bit seed |
| Enter or G | Generate from a snapshot of the current settings |
| Esc | Cancel a pending generation/load or render commit; otherwise show setup |
| Tab | Show/hide setup |
| Space | Start/pause the accepted ride |
| R | Restart its recorded simulation at the station |
| 1 / 2 / 3, setup hidden | Front / physical middle car / rear POV |
| M | Toggle overview camera |
| T | Toggle optional force/speed/time telemetry |
| F5 | Save the current accepted geometry |
| F9 | Load and revalidate saved geometry |

The production generator combines seeded geometric modules with four FVD-authored crest sections. Their smooth force profiles undergo independent source replay; the joined circuit then passes complete finite-train dynamics and 960/1920 Hz verification. It is not yet a fully force-profile-authored circuit generator. See [FVD scope](../core/FVD.md) and the current checkpoint report for exact evidence.

Generation starts paused after a complete successful commit. One train follows the accepted CPU simulation trace; visual interpolation never runs a separate physics model. The ride stops at the trace end and waits for Restart. A hitch longer than 100 ms slows playback rather than jumping the view forward.

The requested settings remain independent of the active ride. An invalid request, exhausted candidate budget, worker exception, load failure, cancellation or render-budget rejection leaves the previous accepted ride active. No unsuccessful candidate becomes rideable. Newly generated terrain also stays hidden until the replacement is complete.

## Binding a real I305 reference

Use the repository's reference processing workflow to obtain the **comparable maximum 10-second positive vertical specific-force exposure in g·s**, with a source identity that records the actual trace/provenance. Do not use a peak-g number, an invented value, or an uncalibrated accelerometer magnitude. This integration does not verify provenance from a numeric value.

After obtaining that processed result, add the following section to `Config/DefaultGame.ini` before packaging, or to the active build's `Saved/Config/Windows/Game.ini` (`WindowsEditor` for editor play). Replace both placeholders with the processed value and an identifiable source record. Restart the game/editor session.

```ini
[CoasterReference]
Exposure10Seconds=<processed numeric g-seconds value>
ReferenceId=<source identity and processing/version identity>
```

A processed reference file can instead be selected with `File=<absolute file path>` in `[CoasterReference]`; relative paths resolve under the project's `Saved` directory. An explicitly selected file that fails parsing does not fall back to the scalar settings. Typed recording metadata preserves observed spread and provenance without independently verifying calibration or authenticity.

The parser requires a positive finite value and a nonempty identity. The HUD labels it **user-configured**; it does not claim independent verification. The exact reference threshold and ID are persisted with the accepted design. All-record mode will then call the core acceptance gate with that reference; selecting PHYSICS-PROOF changes only `requireIntensity`.

## Geometry, threading and persistence

- The source adapters compile the canonical core translation units directly, including frame, drive-profile and convergence sources. The centerline has degree-seven G3 spans; raw up/bank have degree-five caches giving a C2 orthonormal frame in the certified domain. The renderer, car transforms, camera and core force trace all evaluate the same `Track::sample(distance)`; there is no Unreal spline reconstruction.

- The single adapter converts core metres/right-handed XY ground and Z up into Unreal centimetres/left-handed coordinates: `(x, y, z) -> (100*x, -100*y, 100*z)`. Directions reflect Y without scaling; winding follows the tested engine convention without an extra index reversal after reflection. Camera basis uses mapped forward/up, and all POV offsets use the core's `seatDistanceOffset`.

- One C++ thread-pool job generates, validates and prepares plain mesh buffers. Workers capture request values and shared non-UObject state. They never create or touch scene objects. A new request cancels the previous worker, queues only the newest request, and waits for that worker to finish before dispatch; monotonically increasing revisions reject stale results.

- Hidden staging assemblies commit **at most one section (including up to 6,144 vertices for batched cliff backdrop) and 64 tie/support/station instances per tick**, with a cooperative 2 ms budget. An individual engine call cannot be preempted, so this is not a measured frame-time guarantee. Old assemblies hide at the atomic swap and retire at most two mesh components per tick. Engine garbage collection and rendering still require profiling.

- Rails/spine use 80 m procedural chunks sampled every 2 m. Ties remain instanced meshes. New towers use explicit closed tapered steel and footing solids in bounded mesh chunks; their endpoints and radii are canonical saved geometry. Station platforms, canopy and posts use validated imported meshes within canonical oriented boxes; unsupported dimensions, piers and footings use matching cube instances. The core validates swept train and rail/spine/tie clearance, member terrain anchoring and station/support mutual clearance. These are provisional geometric checks, not structural certification. Terrain is sampled from unchanged `Terrain::height(x,y)` on a 20 m grid.

- Rendering has explicit chunk/vertex/instance budgets. Exceeding them fails the replacement. Procedural meshes use conventional runtime rendering; **no automatic Nanite or collision acceleration is assumed**. Rail/train motion uses the numerical core, so scene collision cooking and Chaos simulation are disabled.

- F5/F9 save the full accepted canonical design at `Saved/Designs/Accepted.vcdesign`. **COASTER5 is the only supported geometry format.** It stores the request, limits, train, canonical knots, operations with explicit entry ramps, exit fades and stop profiles, explicit tapered support members and bounded REFERENCE, AXIS_RATE_LIMITS and STATION extensions. The current geometry identity requires enabled station geometry and nonempty support members. Older schemas/identities (including `.1-work` operation rows without exit fades) are explicitly rejected; their knots are not reinterpreted under the new G3/C2 semantics. Unknown or duplicate extensions fail closed. The UE adapter delegates parsing, rebuilding and replay to the core.

- The default solver is 960 Hz with mandatory 1920 Hz half-step verification. Generation, save and load reapply the unchanged targets/force gates at both resolutions and require convergence. `Design::accepted()` requires that verification to be performed and passed. Rejected, unverified or cancelled geometry cannot become the active ride. The vertical specific-force rate is evaluated analytically at solver times; lateral/longitudinal rates remain discrete and are unassessed unless explicit optional limits are configured. This is not a proof of continuous extrema, calibrated rider safety or a complete ride-quality assessment. The measured matrix status is recorded in [release validation](../VALIDATION.md).

- Terrain checks cover the full body and interior footprint with analytic terrain bounds and the shared continuous frame sweep. The nonadjacent 6 m central-chord model now certifies each chord's true arc bound≤2.1 m. Its 12 m same-local-rail exclusion remains a documented modelling limitation; support/station checks do not inherit that exemption. See [the numerical model](../core/NUMERICS.md) and [clearance scope](../SELF_TRACK_ASSESSMENT.md).

- Save/load run off the game thread. World shutdown cancels and drains the outstanding worker before C++ module unloading. This shutdown wait is deliberate; ordinary gameplay never waits on an unfinished future.

Lighting uses native daylight, sky and fog with opaque steel, a Blender-authored aero train, translucent windshields and rough ground. The review adds a separate texture-free slope palette; its new packaged appearance is pending inspection. Art quality, culling, shadows and frame pacing must be judged in the actual matching UE build.

## Remaining acceptance checks on a UE workstation

Run all five contracts: `VibeCoaster.CoordinateContract`, `VibeCoaster.MeshContract`, `VibeCoaster.TerrainBackdropContract`, `VibeCoaster.StationArtContract` and `VibeCoaster.ImportedArtContract` (the packaging script does this). Synthetic mesh fixtures do not establish successful physics generation.

Then exercise the actual packaged Development executable:

1. Confirm the initial all-record request reports the missing reference and creates no ride. Choose PHYSICS-PROOF, flat terrain and seed 42; a ride may start only after the current core accepts it.

2. Ride from station through a complete circuit and stop. Check all three POVs, banking/inversions, forward direction, pause, restart, overview and telemetry. In particular, the middle POV and middle-seat telemetry must refer to the same physical car.

3. Generate flat, hills and canyon designs. Inspect terrain clearance, support placement, rail seams, tie orientation and front/rear visibility. Record any failed seeds as failures; do not substitute their geometry.

4. Keep a good ride active while generating another. Cancel during numerical search, during mesh preparation and during chunk commit. Spam Generate with different seeds. The visible seed/design must change only to the latest fully accepted request.

5. Save, save again to replace that slot, restart the process, and load. Compare accepted seed, stored geometry, terrain and metrics. Corrupt a copied save and test rejection in a test profile. A failed load must preserve the running accepted ride.

6. Run a request with intentionally hard custom goals and a small candidate budget. Confirm rejection, readable diagnostics and preserved active design. Test zero and maximum unsigned 64-bit seeds.

7. Use Unreal Insights and `stat unit`/`stat gpu` in Development. Measure generation responsiveness, chunk commit costs, GC, draw calls and frame time on the target GPU. No 60 FPS or hitch-free guarantee has been measured.

8. Close the game/editor while generating and while loading/saving. Verify cooperative cancellation and safe worker drain. Reopen to ensure the accepted save was not partially overwritten.

## Primary API references

- [Epic coordinate system and spaces](https://dev.epicgames.com/documentation/en-us/unreal-engine/coordinate-system-and-spaces-in-unreal-engine) documents Unreal's left-handed Z-up convention.

- [CreateMeshSection](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Plugins/ProceduralMeshComponent/UProceduralMeshComponent/CreateMeshSection) documents the C++ arrays and the extra cost of collision creation.

- [EAsyncExecution](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Core/EAsyncExecution) documents thread-pool execution.

- [Module properties](https://dev.epicgames.com/documentation/en-us/unreal-engine/module-properties-in-unreal-engine) documents module-level `bEnableExceptions`, C++ standard and external dependencies. Enabling exceptions here does not imply rebuilding the entire engine.

- [Editor Python scripting](https://dev.epicgames.com/documentation/en-us/unreal-engine/scripting-the-unreal-editor-using-python), [LevelEditorSubsystem](https://dev.epicgames.com/documentation/en-us/unreal-engine/python-api/class/LevelEditorSubsystem?application_version=5.6) and [MaterialEditingLibrary](https://dev.epicgames.com/documentation/en-us/unreal-engine/python-api/class/MaterialEditingLibrary?application_version=5.6) support the bootstrap workflow.

- [Build/cook/package operations](https://dev.epicgames.com/documentation/en-us/unreal-engine/build-operations-cooking-packaging-deploying-and-running-projects-in-unreal-engine) and [automation framework](https://dev.epicgames.com/documentation/en-us/unreal-engine/automation-test-framework-in-unreal-engine?application_version=5.6) describe the remaining engine validation gates.

Epic API documentation was checked on 2026-09-06; numerical/schema documentation follows the current release source. The API references above include the original UE5.6 integration sources. UE5.8.2 Windows compilation/UHT and packaging have subsequently passed for the archived revisions. Each changed application still requires its own build and validation; historical source checks do not establish Mac compatibility.
