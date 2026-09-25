# Riftwake default.4 — native accepted; standalone delivery pending

Updated25 September2026. **Work solo. All subagents are finished; do not spawn or resume any.** The user's active request is the redesign described in [checkpoint11](docs/checkpoints/11-riftwake-fvd-redesign.md). Completion-07 is only rollback/current active delivery, not visual acceptance of the new task.

## Current result

Final native candidate: `out/riftwake-fvd-redesign-11.vcdesign`, SHA256 `a88318506bd4d4f8028770b724a40cec199a6c75d60677f8e79739f4b1e47f5a`. Seed42 accepts on candidate0 with176.207 s,7327.458 m,0–180 km/h in1.397 s,14 riders/80 station parts. Exact-save reload through the current library passes. Both native resolutions and independent spatial refinement pass; errors are empty.

Current final shape: twelve-second smooth loaded precliff setup; two real uphill constant-grade LSM strips separated by FVD;26-second shallow winding cliff with an outward bay and opposing/nested turns;34 m lip;88-degree dive; preserved camelback; compact84-degree180° Wave; real rising brake to48 m/s;74 m Loop with monotone8 m exit-side offset;1.1-second loaded upright link;66 m planar Immelmann;12.5-second opposing45-degree signature;35 m return hill; short FVD station approach. Eighteen FVD programmes, zero splines. Before cliff58.509 s vs47.877; after drop82.109 s vs96.643. Front apex gap8.046 s matches the user's allowance for somewhat more duration than Tormenta's~6.5 s.

All measured inversion seats meet the counterpart floors: Loop5.113/4.652/4.414 g vs4.344; Immel5.234/4.778/4.506 vs4.326. Native limits and existing5% project allowance were not relaxed. Category caps are actual canonical extents. Overall rail above lowest embedded footing292.261 m<292.5; source summit284.5 m. New size helper runs on generation and reload; previous default.3 saves retain their original contract.

All LSM setbacks independently read from saved canonical geometry are2 m. The remaining preboost trim nearest a motor is16.90 s away; the5.39-second redundant trim was removed with a six-second actual-replay guard. Last drive regression1234316 passes. Other focused passes:125528 inherited,66279 FVD,100124 dimensions,140 recipes,45 convergence. An optional dimensions-test assertion has been updated for the now-smaller valid inversions and should be built/run against candidate11 with the next metadata rebuild.

Outputs under `out/riftwake-fvd-redesign-11-*` include report/trace/plan/recipe, independent audit, native hardware/source audit, closure audit, terrain audit, consolidated evidence and views. Root inspected candidate09 true-scale cliff/inversion/return plots; candidate10 was byte-identical geometry/frames with the redundant trim removed. Candidate11 only lowers the summit0.5 m to satisfy the most conservative foundation datum and is fully revalidated; root inspected its final true-scale cliff, Wave, inversion and return views. Audit bank angles now derive from physical forward/up vectors, avoiding equivalent Euler-branch artifacts after inversions.

## Next steps — no further redesign unless review finds a real defect

1. Commit the reviewed source plus checkpoint/docs with `[skip ci]`. Implementation is committed at `6e340061e79dcba5ca28220ea3c9cb69a9fa4ddd`. The first Unreal build found a local variable shadow in the new review camera, and the rollback test exposed the default.3 save taking the pre-default.3 total cliff pacing rule. Both are fixed in the following small compatibility commit; rebuild and recheck them before delivery. Branch `codex/legacy-authoring`, origin `https://github.com/Banrs/VibeCoaster2.git`; commit/push is already authorized by the earlier handoff.
2. Package using `native/unreal/scripts/package.ps1 -UnrealRoot 'D:/Games/Epic Games/UE_5.8' -Configuration Development -SkipEditorPreparation`. The script requires a clean committed tree. Keep tracked files unchanged while packaging. Use existing verified V3 content; do not overwrite the old user's Editor DLL or rerun Blender work.
3. Build/recheck native CLI after the commit so final reload reports carry the correct source identity. Run the optional saved-file dimensions test against candidate11.
4. Run `scratch/redesign-audits/delivery/run-candidate-runtime.ps1` against the new package and **candidate11**, with fresh non-overlapping output/profile paths, front and rear. Front uses `-ReviewCameras`; expected six fixed views plus every Boost exit (three), so nine review captures. This harness syntax was parsed, but the new runtime camera code has not yet been built/run. Inspect actual2560×1440 front/rear and third-person images. Require14 riders,80 station parts and matching save/package/geometry identities.
5. Only after actual runtime review passes, preserve rollback and promote a fresh default.4 profile, package manifest/current.json and root/desktop shortcuts. Update checkpoint/HANDOFF with package SHA/source commit and runtime evidence; commit/push final delivery docs. The old user game remains running and must not be killed.

## Tools and artifacts

- Normal exec/apply_patch/view_image helpers can fail setup refresh. Use elevated `exec_command` with concise task justification. Windows PowerShell. Python: `C:/Users/danie/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe`.
- Build native: `scratch/fvd-redesign/build-integration-tests.cmd` (VS14.38, single worker). New cap helper is in `dimensions.cpp`/`acceptance_internal.hpp`; saved derived Loop brake/link ownership and per-seat reference floors are already integrated/tested.
- Private probes and final evidence extractor live in `scratch/solo-layout/`; the old private compiler there uses frozen diagnostic inlet speeds and must NEVER replace production code. Production uses actual train energy calibration.
- Read images using elevated Pillow thumbnail→JPEG base64, then `functions.image`; do not print base64. Plots use `out/plotting-deps`.
- Graphify refreshed native/core successfully:2135 nodes/5424 edges/120 communities,81 code files,0 LLM extraction tokens. `graphify-out/.needs_update` was removed after validating fresh JSON. Two header parse warnings remain and are documented in checkpoint11. No subagents or extra semantic labeling are needed.
- Reference evidence: `docs/references/2026-09-redesign-reference-audit.md`, `docs/references/fvd-redesign-contract.md`, `docs/reference-force-audit.md`. Root and completed agents inspected sequential digital FF flyover and real POV frames. No standard-crest cliff prototype is allowed. User says after-inversion closure is the airtime/flatness concern; no filler hill between inversions.
- Exact save reload already passed via `scratch/solo-layout/native-audit.exe`, whose JSON records all hardware and per-seat ascent peaks. No new package or profile has been activated yet.

---

## Previous delivery record (superseded as visual acceptance)

Updated 25 September 2026. The user rejected checkpoint `b39622c`: clifftop density/flow, unintended flat/upright resets, long cliff braking, inversion left/right modulation and wrong loop hand, and ordinary track standing too far above terrain. Those corrections are implemented and verified in the delivered revision. Do not reapply the paused patch. The implementation is committed/pushed at `9631e488f93e71a1fa2056550264b7d1a31967ed`. Its standalone is built, verified and promoted. Preserve it and the rollback files. The final startup percentile experiment is complete; its measured3s target remains unmet.

## Shape and verification contract

Keep roughly 180 seconds (175–190 while tuning), the explicitly retained 0–180 km/h in 1.4 seconds, seven two-seat rows, and the protected camelback/signature with opposing 45-degree crests. The loop needs one lateral offset toward its exit, with nearly parallel arms. The Immelmann should reverse through its half-loop and roll upright without a separate horizontal weave. This is a path-shape correction, not a request merely to reduce rotation rates. See `docs/references/inversion-shape-research.md` and `docs/requirements-completion-audit.md`.

Full native force, source replay, terrain/train/track/support/station clearance and temporal/spatial refinement remain mandatory. Do not relax gates or activate a rejected diagnostic. Global average height alone does not close terrain fit; inspect ordinary sections and actual front/rear runtime views.

## Accepted geometry

`out/riftwake-completion-07.vcdesign` accepts seed42 on candidate0: 181.817708s, fourteen riders, retained0–180km/h launch1.397s, active cliff26.2473s and lip2.69047s. Its SHA256 is `d48ed31cf62621405ee43a859c3e6e1ea468e405b383217fc7fb3e84895a0cf7` (16,028,026bytes). All errors are empty. Front/middle/rear acceleration histories, source authorship, C3 motion, terrain/track/support/station clearance,960→1920Hz convergence and independent half-spacing replay pass. Minimum swept-ground clearance is+2.56160m; spatial force relative error5.04e-9.

The compact1000m six-arc shelf carries continuous pitch/yaw jets with meaningful hills and sustained banking. Previous checkpoint lip5.7469s is now2.69047s. The source-study near-level time drops77%→23.9%; final full-trace breakdown is being retained separately. Actual whole-train braking in the unchanged cliff section measured3.183→2.650s, and the slow coast before centre departure2.624→0.447s.

The loop uses a signed18m lateral offset toward its exit with no left/right reversal and a nearly parallel exit. The Immelmann half-loop/roll is planar with real, modest lateral rider force, not an imposed yaw weave. A180m/6m rollover spaces the inversion bodies; a higher+20m Immel recovery permits a real signature-approach bridge above the loop. The protected opposing45-degree signature remains. Source tests include both hands, inherited jets, ordinary saved-control replay and a67.27m/s regression for the former sub-millisecond knot gap.

Terrain now follows the opening hills, LSM/pullout and low return. The first signature wing terrace is shifted toward the actual wing and narrowed away from the lower wave/loop; two crowns sit below the actual wing crests. The source retains eight foothills and the original terrain/force/clearance domains. No failed design is promoted. Earlier completion01–06 outputs are rejected diagnostic history, not deliverables.

`/root/geometry` and `/root/terrain_fit_audit` are finished. Nearby seed43 accepts candidate1 at181.5625s with all checks. Final geometry evidence is `out/riftwake-completion-07-geometry-evidence.md`/`-geometry-summary.json`; section audit is `scratch/terrain-fit-completion/final-clearance-audit.md`. Spatial mean rail-ground is39.7594m versus47.4098m; within10m share40.02% versus30.6%. Native default maxCandidates is8; the accepted CLI experiment bounded it to4 and accepted immediately on0.

Final native checks, package and front/rear runtime review are complete. Final startup measurement and delivery documentation are complete; the3s target remains the known performance limitation.

## Art and station

Actual Blender MCP authored a coherent higher-bonnet train after viewing official Intamin/Ferrari World references. Seven physical row modules and the canonical eye/force position remain unchanged. The coupler is a parked prototype (`runtime:false`); rigid four-car grouping is explicitly deferred.

Functional station is integrated: 80 canonical boxes, covered queue/merge, seven aligned row lanes and gates, dispatch cabin, separate unload/exit, lifts/stairs and underpass. Native station tests passed 1455 checks on seven-row and explicit legacy six-row fixtures. Independent review caught and fixed a stair/roof obstruction and top landing mismatch; actual MCP exterior/cutaway previews are in `scratch/station-resume/mcp-preview/`.

A further independent review caught 129 mm entry-lift and 135 mm exit-underpass floor lips. Both model floors now meet the decks at -4.2 m within unchanged canonical boxes. Actual vertex checks pass. Root re-exported through real Blender MCP and completed a fresh real-RHI import: `out/completion-v3-floorfix-import-receipt.json` (21 assets) and matching engine log with success marker/no Python errors. No simultaneous shared Blender scene mutations.

Runtime uses **Art/V3**, preserving local V2 for the user's running old game. Fresh initial V3 import passed all21 runtime meshes, bounds/materials/instancing flags: `out/completion-v3-import-receipt.json` and `out/completion-v3-import-engine.log`. Use the newer floor-fix receipt above for final art provenance. `native/art/train_models.py`, the complete source/export kit and V3 assets are committed in9631e48. Obsolete export files were archived with hashes in `out/pre-completion-art/superseded-exports/`.

Runtime adapter includes all12 new station roles, lead/passenger separation, correct hardware materials and verifier-only queue/unload observer frames. Coordinate/Mesh/StationArt contracts can run in a Development packaged game; ImportedArtContract remains editor-only. The manifest/import/source contract was checked read-only. Actual final packaged front/rear views are complete: both runtime smoke reports pass with the same geometry SHA1 `00BA13C0A10848CA46C9F477E7E5A4F80E4CB83F`, and the save SHA256 is unchanged. Root inspected station circulation, train, cliff, inversion, bridge and low signature images. Packaged Coordinate/Mesh/StationArt contracts pass. Human styling approval and direct keyboard-input testing remain unclaimed.

## Package, activation and remaining performance measurement

The verified Development standalone is `native/unreal/Packaged/run-20260924-231703-433/Windows/VibeCoaster/Binaries/Win64/VibeCoaster.exe`. SHA256 `DBB65AF1F0F6E9B54D05E2DB83B48994237F4E845580B244B609650B42A29EF4`; source commit9631e488. UE5.8.2 Game build/cook/package passed. The package manifest is beside its Windows folder. Editor preparation was skipped using the already-verified V3 content; the old running editor DLL was not overwritten.

Both actual2560x1440 full runtime traversals pass: `out/completion-runtime-front/result.json` (62captures), `out/completion-runtime-rear/result.json` (63captures). Package contracts: `out/completion-game-contracts-engine.log`. The temporary runner is `scratch/run-completion-runtime.ps1`. All those processes are finished.

`dist/current.json`, the root Play shortcut and desktop VibeCoaster2 shortcut now use the verified standalone and fresh `UserData-Riftwake-V3` profile. The profile contains exact accepted completion07 bytes. Activation identities are in `out/completion-activation.json`. Prior current.json, both shortcuts and accepted save are copied to `out/completion-activation-backup/`; original UserData-Development remains unchanged, hash `DB3D4B55F055036A3D18ABDD54D3D1ADF5EC8EC8988F41F760ECB42DEF03A8CA`. Prior binaries and V2 assets remain. Do not close the user's old running game (PID14328 at last identification); the already-open old window does not change builds in place. Relaunch through the updated shortcut for the new version.

`motion_audit.cpp` overlaps refined simulation with independent refined clearance; both still complete before acceptance. The retained focused persistence/cancellation checks pass. The final startup pilot has10valid warm runs: request-to-ready median3.167s/max3.337s; process-to-ready median4.882s/max5.071s. Both3s targets are missed. Native validation median2.954s, mesh0.098s, scene commit0.102s. Do not claim the target met or infer p99 from this pilot.

The final100-warm benchmark is complete in `out/startup-final-100-20260925/`: request-to-ready median3.150476s/p993.253885s/max3.300493s; process-to-ready median4.849519s/p994.978452s/max5.009111s. All101total runs pass and match the exact accepted save and geometry identity,14riders/80stationparts. All100warm runs exceed3s on both measures. The first process is separate, not a cold-cache claim; OS/driver caches were not cleared and the old user game remained running. `evidence.md` records limits and native-stage breakdowns.

The benchmark harness now writes its profile marker as exact LF-terminated bytes via `Path.write_bytes(...)`; Windows text newline translation had caused the first pilot to be correctly refused. The failed pilot is preserved as diagnostic and excluded from the final samples. This harness-only change does not alter the game binary. No agent-owned build, import, verifier or timing process remains running.

The final delivery checkpoint includes docs, the LF harness fix and the updated tracked root shortcut. Keep the package's source identity9631e488; later harness/docs/shortcut commits are not a different game binary. The known remaining target is startup below3s with fresh validation. Do not claim it achieved or bypass validation to meet a number. See `docs/checkpoints/10-riftwake-completion.md` for the complete delivery/evidence record.

## Working rules and tooling

The user authorizes parallel work; root integrates/reviews. Latest limits are one Astra, two Sol and ten Luna; agents use the required efficient-subagent-waiting skill and event-driven waits. No unnecessary broad suites/CI or repeated photo tuning. Meaningful commits/pushes use `[skip ci]`. Repository is public and the signed-in GitHub user has ADMIN; established origin is `https://github.com/Banrs/VibeCoaster2.git`, branch `codex/legacy-authoring`, pushed implementation checkpoint `9631e48`.

Normal exec/apply_patch/view_image helpers fail setup refresh. Use `exec_command` with `sandbox_permissions=require_escalated` and a concise task justification. PowerShell; explicit UTF-8 Python I/O. Git: `git -c safe.directory=D:/Coding/Codex/Vibecoaster2`. Do not change global trust.

Python: `C:/Users/danie/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe`; native VS14.38 at `D:/Toolchains/VS2022`; UE5.8 at `D:/Games/Epic Games/UE_5.8` uses14.44. `scratch/build-legacy-baseline.cmd` builds CLI. No stale scratch object substitutions in accepted builds.

Blender MCP wrapper: `native/art/run_blender_mcp.py`, Python `D:/Toolchains/BlenderMCP/venv/Scripts/python.exe`, server `D:/Toolchains/BlenderMCP/venv/Scripts/blender-mcp.exe`, loopback9876. Safe mode remains on; root serializes scene work. Use actual MCP for authoring. To inspect images, elevated Pillow thumbnail→JPEG base64 can be emitted through functions.image without printing the data. Distinguish Blender renders from runtime screenshots.

Graphify indexed native/core; use the graph first and confirm changing source directly. The current graph may lag ongoing edits. Original full task requirements were recovered into `scratch/original-handoff-requirements.md`. User requested Fast OFF: global config is `service_tier="default"`, but active per-task tier is not exposed; do not claim confirmed off.
