# Riftwake completion work in progress

Updated 25 September 2026. The user rejected checkpoint `b39622c`: clifftop density/flow, unintended flat/upright resets, long cliff braking, inversion left/right modulation and wrong loop hand, and ordinary track standing too far above terrain. These are active requirements. Do not reapply the paused patch. The current implementation is frozen for a source checkpoint and standalone build; preserve it and the rollback files.

## Shape and verification contract

Keep roughly 180 seconds (175–190 while tuning), the explicitly retained 0–180 km/h in 1.4 seconds, seven two-seat rows, and the protected camelback/signature with opposing 45-degree crests. The loop needs one lateral offset toward its exit, with nearly parallel arms. The Immelmann should reverse through its half-loop and roll upright without a separate horizontal weave. This is a path-shape correction, not a request merely to reduce rotation rates. See `docs/references/inversion-shape-research.md` and `docs/requirements-completion-audit.md`.

Full native force, source replay, terrain/train/track/support/station clearance and temporal/spatial refinement remain mandatory. Do not relax gates or activate a rejected diagnostic. Global average height alone does not close terrain fit; inspect ordinary sections and actual front/rear runtime views.

## Accepted geometry; package review pending

`out/riftwake-completion-07.vcdesign` accepts seed42 on candidate0: 181.817708s, fourteen riders, retained0–180km/h launch1.397s, active cliff26.2473s and lip2.69047s. Its SHA256 is `d48ed31cf62621405ee43a859c3e6e1ea468e405b383217fc7fb3e84895a0cf7` (16,028,026bytes). All errors are empty. Front/middle/rear acceleration histories, source authorship, C3 motion, terrain/track/support/station clearance,960→1920Hz convergence and independent half-spacing replay pass. Minimum swept-ground clearance is+2.56160m; spatial force relative error5.04e-9.

The compact1000m six-arc shelf carries continuous pitch/yaw jets with meaningful hills and sustained banking. Previous checkpoint lip5.7469s is now2.69047s. The source-study near-level time drops77%→23.9%; final full-trace breakdown is being retained separately. Actual whole-train braking in the unchanged cliff section measured3.183→2.650s, and the slow coast before centre departure2.624→0.447s.

The loop uses a signed18m lateral offset toward its exit with no left/right reversal and a nearly parallel exit. The Immelmann half-loop/roll is planar with real, modest lateral rider force, not an imposed yaw weave. A180m/6m rollover spaces the inversion bodies; a higher+20m Immel recovery permits a real signature-approach bridge above the loop. The protected opposing45-degree signature remains. Source tests include both hands, inherited jets, ordinary saved-control replay and a67.27m/s regression for the former sub-millisecond knot gap.

Terrain now follows the opening hills, LSM/pullout and low return. The first signature wing terrace is shifted toward the actual wing and narrowed away from the lower wave/loop; two crowns sit below the actual wing crests. The source retains eight foothills and the original terrain/force/clearance domains. No failed design is promoted. Earlier completion01–06 outputs are rejected diagnostic history, not deliverables.

`/root/geometry` is doing final read-only metrics and one nearby seed43 request; source is frozen. Root may commit/package concurrently. `/root/terrain_fit_audit` is comparing final07 with the user-rejected `riftwake-resume-06` baseline. Native default maxCandidates is8; the accepted CLI experiment bounded it to4 and accepted immediately on0.

Final native metrics and the nearby check are pending, followed by actual packaged front/rear views, startup measurements and safe promotion.

## Art and station

Actual Blender MCP authored a coherent higher-bonnet train after viewing official Intamin/Ferrari World references. Seven physical row modules and the canonical eye/force position remain unchanged. The coupler is a parked prototype (`runtime:false`); rigid four-car grouping is explicitly deferred.

Functional station is integrated: 80 canonical boxes, covered queue/merge, seven aligned row lanes and gates, dispatch cabin, separate unload/exit, lifts/stairs and underpass. Native station tests passed 1455 checks on seven-row and explicit legacy six-row fixtures. Independent review caught and fixed a stair/roof obstruction and top landing mismatch; actual MCP exterior/cutaway previews are in `scratch/station-resume/mcp-preview/`.

A further independent review caught 129 mm entry-lift and 135 mm exit-underpass floor lips. Both model floors now meet the decks at -4.2 m within unchanged canonical boxes. Actual vertex checks pass. Root re-exported through real Blender MCP and completed a fresh real-RHI import: `out/completion-v3-floorfix-import-receipt.json` (21 assets) and matching engine log with success marker/no Python errors. No simultaneous shared Blender scene mutations.

Runtime uses **Art/V3**, preserving local V2 for the user's running old game. Fresh initial V3 import passed all21 runtime meshes, bounds/materials/instancing flags: `out/completion-v3-import-receipt.json` and `out/completion-v3-import-engine.log`. Use the newer floor-fix receipt above for final art provenance. `native/art/train_models.py`, the whole source/export kit and V3 assets are untracked and must be included in a checkpoint. Obsolete export files were archived with hashes in `out/pre-completion-art/superseded-exports/`.

Runtime adapter includes all12 new station roles, lead/passenger separation, correct hardware materials and verifier-only queue/unload observer frames. Coordinate/Mesh/StationArt contracts can run in a Development packaged game; ImportedArtContract remains editor-only. The manifest/import/source contract was checked read-only. Actual final packaged front/rear views remain pending.

## Performance, package and activation

`motion_audit.cpp` overlaps refined simulation with independent refined clearance and joins both before acceptance. Thirty focused persistence checks pass. Three-run native validation samples on the old182s save improve3.26–3.30 to2.74–2.85s; these are not p99 evidence.

Package scripts require clean committed source, current21-asset/nine-material V3 kit and an identified source commit. `VibeCoaster.Build.cs` now supplies process-local exact-repository safe.directory so elevated UBT does not silently embed `unversioned`. Use game packaging with `-SkipEditorPreparation`; UE5.8 `-skipbuildeditor` avoids overwriting the locked old editor DLL. The new package build is the next step after the frozen-source commit.

Benchmark final standalone with `benchmark_startup.py --executable ... --mode load --load-design ...`. It uses fresh marked isolated profiles, real rendering, separate request-to-ready/process-to-ready/motion metrics. At least100 valid warm samples are required before reporting empirical p99. The3s target is unproven.

Preserve the user's running old game (PID14328 at this update). An earlier request to close it was superseded by V3 assets; closing it is no longer required. Do not terminate it. Existing `UserData-Development/Saved/VibeCoaster2/Designs/Accepted.vcdesign` is rejected-by-user numerical checkpoint06, SHA256 `DB3D4B55F055036A3D18ABDD54D3D1ADF5EC8EC8988F41F760ECB42DEF03A8CA`. Check/backup its actual current contents before any promotion. Prior save/binaries are in `out/pre-riftwake-runtime/`. Existing shortcuts still use old editor-game; `dist/current.json` is the older standalone. Promote only after real packaged validation and front/rear inspection, preserving rollback files.

## Working rules and tooling

The user authorizes bounded parallel geometry, modeling and optimization work; root integrates/reviews. Up to3 Astra and6 Sol, extensive Luna; agents use the required efficient-subagent-waiting skill and event-driven waits. No unnecessary broad suites/CI or repeated photo tuning. Meaningful commits/pushes use `[skip ci]`. Repository is public and the signed-in GitHub user has ADMIN; established origin is `https://github.com/Banrs/VibeCoaster2.git`, branch `codex/legacy-authoring`, pushed HEAD `b39622c` before this work.

Normal exec/apply_patch/view_image helpers fail setup refresh. Use `exec_command` with `sandbox_permissions=require_escalated` and a concise task justification. PowerShell; explicit UTF-8 Python I/O. Git: `git -c safe.directory=D:/Coding/Codex/Vibecoaster2`. Do not change global trust.

Python: `C:/Users/danie/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe`; native VS14.38 at `D:/Toolchains/VS2022`; UE5.8 at `D:/Games/Epic Games/UE_5.8` uses14.44. `scratch/build-legacy-baseline.cmd` builds CLI. No stale scratch object substitutions in accepted builds.

Blender MCP wrapper: `native/art/run_blender_mcp.py`, Python `D:/Toolchains/BlenderMCP/venv/Scripts/python.exe`, server `D:/Toolchains/BlenderMCP/venv/Scripts/blender-mcp.exe`, loopback9876. Safe mode remains on; root serializes scene work. Use actual MCP for authoring. To inspect images, elevated Pillow thumbnail→JPEG base64 can be emitted through functions.image without printing the data. Distinguish Blender renders from runtime screenshots.

Graphify skill was read; no graph/runtime exists, so direct source inspection is used. Original full task requirements were recovered into `scratch/original-handoff-requirements.md`. User requested Fast OFF: global config is `service_tier="default"`, but active per-task tier is not exposed; do not claim confirmed off.
