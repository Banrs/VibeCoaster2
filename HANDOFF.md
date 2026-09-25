# Riftwake default.4 — verified and activated

Updated 25 September 2026. **Work solo. All subagents are finished; do not spawn or resume any.** The redesign requested in this conversation is implemented, packaged, reviewed and activated. Current evidence is in [checkpoint11](docs/checkpoints/11-riftwake-fvd-redesign.md). Completion-07 is retained as rollback, not as visual acceptance of the new work.

## Current playable delivery

- Package source: `f110b1b2b62ec632f0a2366939d9c564312972f9`. Later delivery/docs/shortcut commits do not change that binary identity.
- Standalone: `native/unreal/Packaged/run-20260925-124313-111/Windows/VibeCoaster/Binaries/Win64/VibeCoaster.exe`.
- Executable SHA256: `198100288A1302A9D5EAA60F00F3BDD6097B3FDAAFB300F257AF19C601D70237`.
- Manifest SHA256: `BAF6C9F4E02D92324E44F799DAAB8E287E8AB636DA2E3BE7C8B77B8C7F71E172`.
- Accepted save: `out/riftwake-fvd-redesign-11.vcdesign`, SHA256 `A88318506BD4D4F8028770B724A40CEC199A6C75D60677F8E79739F4B1E47F5A`.
- Runtime geometry SHA1: `FCEE7D0E4771B6965142FF2396A2FEE28C396322`.
- `dist/current.json`, root Play shortcut and desktop VibeCoaster2 shortcut use this package and fresh `UserData-Riftwake-V4`, whose accepted save matches the above bytes. Targets/arguments were read back.
- Activation proof: `out/riftwake-default4-activation.json`. Previous manifest, both shortcuts and default.3 save: `out/default4-activation-backup/`. Old V3 and original Development profiles remain unchanged. The user's old game was not closed; relaunch through Play to use default.4.

## Shape and native evidence

Seed42 accepts candidate0: 176.207 s, 7327.458 m, retained 0–180 km/h in 1.397 s, seven two-seat rows, 80 station parts. Eighteen FVD programmes cover curved geometry; zero splines. All native errors are empty. Exact-save reload, front/middle/rear F2291-25 histories, source reconstruction, train/terrain/track/support/station clearance, 960→1920 Hz refinement and independent half-spacing replay pass. Minimum swept-ground clearance is +0.470 m.

Current shape: twelve-second smooth loaded precliff setup; 20/32-degree uphill LSM strips separated by FVD; 26-second shallow winding cliff with an outward bay, opposing counterturn and nested low traverse; 34 m lip; 88-degree dive; protected camelback; compact 84-degree 180-degree Wave; real rising brake to 48 m/s; 74 m Loop with monotone 8 m exit-side offset; 1.1-second loaded upright connector; 66 m planar Immelmann; 12.5-second opposing 45-degree signature; 35 m return hill and short FVD station approach. No standard airtime crests on the shelf and no filler hill between inversions.

Before cliff 58.509 s versus 47.877; after drop 82.109 s versus 96.643. Front Loop-to-Immel apex gap 8.046 s follows the user's allowance for modest extra duration over Tormenta's roughly 6.5 s. All measured seats stay above 1.12 g on the connector. Cliff lip traversal is 1.532 s; actual whole-train brake-work activity is about 2.30 s, a different measure. Longest native inactive flat coast is 1.30 s.

All LSM entry/exit guards are independently measured at 2.000 m on the real constant-grade geometry. Actual boost-exit rail gaps are 2.250/2.219/2.250 m. The downhill terrain follows the real low pullout; the protected camelback's subsequent rise is intentional. Removed the redundant trim 5.39 s before a boost using actual replay-time spacing; the nearest retained preboost trim is 16.90 s away. All six retained trim controllers are inactive in the nominal ride.

Actual front/middle/rear ascent peaks: Loop 5.113/4.652/4.414 g against the 4.344 g reference floor; Immel 5.234/4.778/4.506 g against 4.326. Existing project force allowance and F2291 curves are unchanged. Canonical category caps pass, including highest rail above the lowest embedded footing 292.261 m < 292.5 m. Wave and camelback benchmarks are explicitly provisional where no formal comparable record category was verified; consumer G traces do not prove pointwise force dominance at every location.

Focused passes: 125528 inherited-port, 66279 FVD, 1234316 drive, 100128 dimensions including the exact save, 140 recipe and 45 convergence. Exact default.3 rollback reload also passes after preserving its separate active-cliff/lip pacing contract. Native source at cc96565 is unchanged by the final f110b1b camera-only update.

Outputs under `out/riftwake-fvd-redesign-11-*`: report, trace, plan, recipe, exact reload, native hardware/source audit, independent audit, closure audit, terrain audit, consolidated evidence and true-scale cliff/Wave/inversion/return plots. Physical bank is derived from actual forward/up vectors; equivalent Euler branches after inversions are not extra rail rotations.

## Final runtime review

Both final-package verifier runs pass with the exact save/geometry/executable identities above:

- `out/riftwake-default4-runtime-front-v2/result.json` and `out/riftwake-default4-runtime-front-v2.identity.json`: full traversal, 74 screenshots including all ten review cameras.
- `out/riftwake-default4-runtime-rear-v2/result.json` and `out/riftwake-default4-runtime-rear-v2.identity.json`: full traversal, 70 screenshots.
- Both retain 14 riders/80 station parts and pass load, pause, restart, paused-pose stability, and save/generation/mesh/scene cancellation. Both processes exited successfully.
- Root inspected all ten final third-person images at larger size and all 64 rear POV images in six timestamped contact sheets, plus earlier front POV images from the identical geometry. `out/riftwake-default4-visual-review.json` records observations and image hashes. No new geometry defect was found in these views.
- The sustained outward-bank target is -50 degrees over the exposed cliff bay. The real low-pullout target has a 2.262 m rail gap. Full-shelf, every boost exit, Wave, inversion spacing, closure and final-return images are retained.

The first final-geometry package passed front/rear but had two terrain-obscured observer views and the wrong pullout target. The final package corrects the cameras and adds the Wave view; its ten-view evidence supersedes the older nine-view captures. Do not confuse old package `run-20260925-121833-356` with the active final package.

This is agent visual inspection, not user styling approval or physical intensity testing. Direct keyboard testing, FPS/GPU acceptance and a new startup percentile claim are not made. The previous measured 3 s startup target remains unmet; see the historical record below. No further redesign or broad rerun is pending unless the user identifies a new issue.

## Working tools and references

- Branch `codex/legacy-authoring`, origin `https://github.com/Banrs/VibeCoaster2.git`. Commit/push is authorized. Use `[skip ci]` for these delivery checkpoints.
- Normal exec/apply_patch/view_image helpers can fail Windows setup refresh. Elevated `exec_command` works. PowerShell; Python `C:/Users/danie/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe`. Git uses `-c safe.directory=D:/Coding/Codex/Vibecoaster2`; do not alter global trust.
- Graphify native/core is refreshed: 2135 nodes, 5424 edges, 120 communities, 81 code files, zero LLM extraction tokens. Two partially parsed headers are documented and were inspected/compiled directly. No stale update marker remains.
- Native build: `scratch/fvd-redesign/build-integration-tests.cmd`. Runtime runner: `scratch/redesign-audits/delivery/run-candidate-runtime.ps1`. Saved probes in `scratch/solo-layout/` are diagnostic only; production uses actual train-energy calibration.
- References: `docs/references/2026-09-redesign-reference-audit.md`, `docs/references/fvd-redesign-contract.md`, `docs/reference-force-audit.md`. Sequential digital Falcon's Flight flyover, real POV and official Tormenta POV frames were inspected. The correct Tormenta pair is the Loop and second Immelmann, not the earlier lift/drop.
- Preserve V3 art, all old profiles/packages and the original user's running game. Do not rerun Blender or overwrite the old Editor DLL for this completed geometry task.

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

The latest user instruction is to work solo after subagents reach their stopping points. All subagents are finished. Do not spawn or resume agents; the earlier concurrency limits are historical. No unnecessary broad suites/CI or repeated photo tuning. Meaningful commits/pushes use `[skip ci]`. Repository is public and the signed-in GitHub user has ADMIN; established origin is `https://github.com/Banrs/VibeCoaster2.git`, branch `codex/legacy-authoring`, pushed implementation checkpoint `9631e48`.

Normal exec/apply_patch/view_image helpers fail setup refresh. Use `exec_command` with `sandbox_permissions=require_escalated` and a concise task justification. PowerShell; explicit UTF-8 Python I/O. Git: `git -c safe.directory=D:/Coding/Codex/Vibecoaster2`. Do not change global trust.

Python: `C:/Users/danie/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe`; native VS14.38 at `D:/Toolchains/VS2022`; UE5.8 at `D:/Games/Epic Games/UE_5.8` uses14.44. `scratch/build-legacy-baseline.cmd` builds CLI. No stale scratch object substitutions in accepted builds.

Blender MCP wrapper: `native/art/run_blender_mcp.py`, Python `D:/Toolchains/BlenderMCP/venv/Scripts/python.exe`, server `D:/Toolchains/BlenderMCP/venv/Scripts/blender-mcp.exe`, loopback9876. Safe mode remains on; root serializes scene work. Use actual MCP for authoring. To inspect images, elevated Pillow thumbnail→JPEG base64 can be emitted through functions.image without printing the data. Distinguish Blender renders from runtime screenshots.

Graphify indexed native/core; use the graph first and confirm changing source directly. The current graph may lag ongoing edits. Original full task requirements were recovered into `scratch/original-handoff-requirements.md`. User requested Fast OFF: global config is `service_tier="default"`, but active per-task tier is not exposed; do not claim confirmed off.
