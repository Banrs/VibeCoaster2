# VibeCoaster2 - stopped checkpoint for the next sessions

**User stopped work on 24 September 2026 and requested local cleanup with no more tests. Do not resume builds, tests or delivery automatically.** All agent-owned build/runtime processes are stopped. The remaining remote Windows checkpoint run was cancelled at the user's request. No standalone Default2 package was completed or promoted.

## Repository and playable state

- Branch: `codex/legacy-authoring`.
- Reviewed native recovery commit: `840644d0c2acf6bfc4af0eda7af1aa50d19843ca`.
- Latest code/shortcut commit: `18d3fb28a389a4878cea2afbed2fe412f8ef8f4a`.
- Both commits were pushed with explicit user approval. A final documentation/cleanup checkpoint is local only.
- Release identity: `2.0.0-default.2`. Default1 saves remain readable and receive fresh validation.
- Root `Play VibeCoaster2.lnk` and Desktop `VibeCoaster2.lnk` launch the **current local Default2 game**, automatically loading the corrected ride. They invoke UnrealEditor with the project, `-game`, `-UserDir="D:/Coding/Codex/Vibecoaster2/UserData-Development"` and `-CoasterLoad`. Press Space to ride.
- The user specifically objected when these links still targeted Default1. **Do not restore that stale target.** Preserve `UserData-Development` and `native/unreal/Binaries` so the working shortcut remains usable.
- Recreate the current links with `native/unreal/scripts/create_shortcuts.ps1 -Desktop -Development -UnrealRoot "D:/Games/Epic Games/UE_5.8"`.
- `dist/current.json` still points to the preserved Default1 standalone package. Running the shortcut helper without `-Development` before a new promotion would revert the links to that old package.

## User direction

The eventual game is like NoLimits 2 with fast automatic generation as a starting point for coaster design/refinement. Several coasters share one environment, no park management, and only optional/skippable queue-to-seat walking. This task is the basic generator. See docs/product-vision.md.

Work solo. Do not reuse old agents. Graphify and efficient-subagent-waiting were read earlier; no graph/runtime was available. User instructions override forced delegation.

Use approximately 1.33x FF scale and 1.2x speed (300+ km/h). Approximate element/phase force baselines: FF +15%, TRR +8 1/3%, others +10%; negative loads use magnitude. Check actual seats and phases. Up to 5% over project force/rate/pacing limits is allowed only alongside unchanged independent F2291 checks. Do not cherry-pick rounding. Stop fine-tuning tiny photo differences. Lower-drag stress acceptance is removed; configured drag and trims-off/fully-deployed hardware checks remain.

## Selected result

`out/recovery-balanced-inversion.vcdesign` and report/trace/plan: seed42 highlands, 6166.9249 m, 133.9906 s, 300.534 km/h, Gz -1.42842 to +5.00165, maximum |Gy| .98962. All physical/source/spatial/refinement gates pass. The +5 g nominal excess is explicitly reported under the project allowance.

Camelback: one isotropic planar asymmetric fit, 3.53824 px RMS / 12.7918 px max against practical 4/16 px guards. All-seat ascending/airtime/recovery phases exceed their FF +15% baselines. Do not fit it again. Loop: normal4.85/crest2/recovery3.9, rise145/yaw33. Immelmann: normal4.85/crest3.8/roll-exit2/recovery3, rise85/yaw45, rollReleaseFraction .45. Both explicitly unload over1.2s into a solved real crown hold.

The exact pointwise reference comparison remains uncertain because recordings have no surveyed distance. Approximate per-phase goals and known limitations are documented in docs/reference-force-audit.md. The optional calibrated-reference intensity flag stays separate; no whole-standard legal-certification claim.

## Completed verification (do not rerun merely for handoff)

- Final local component suites: **16/16**, `out/recovery-adaptive-components.log`; **66303 FVD checks**, `out/recovery-crown-seed-fvd-tests.log`.
- Final local integration suites: **10/10**, `out/default2-final-integration.log`.
- Final local corpus: **8/8**, save/reload and seed diversity, `out/recovery-final-generator-corpus/summary.json`.
- Source `18d3fb2` Windows/macOS component + baseline CI: **both passed**, https://github.com/Banrs/VibeCoaster2/actions/runs/35996407473.
- Full checkpoint CI for the identical native core at `840644d`: **macOS passed components, all integration tests and all8 corpus cases; Windows was cancelled on user request while its full suite was running**, https://github.com/Banrs/VibeCoaster2/actions/runs/35995677863. Downloaded macOS evidence: `out/default2-ci-macos`.
- Default1-provenance corrected save successfully revalidated under runtime Default2: `out/default2-compatible-baseline-report.json`.
- Both 2560x1440 front/rear Unreal traversals passed with geometry SHA1 `09CDA5902EC0D2B52E1BE049AF94125BDC382329`. Pause/restart/load/pose and generation/mesh/scene/save cancellation checks passed. Root inspected all46 captures per seat, retained in `out/recovery-final-generator-runtime-front` and `-rear`. Agent review: `out/recovery-final-generator-visual-review.json`. This is captured-frame review, not human approval, continuous-video review, keyboard testing or a GPU performance claim.
- The actual shortcut startup path also passed its full traversal and load/pose/cancellation checks under app2.0.0-default.2, source18d3fb2. `out/default2-shortcut-startup/result.json` and its events confirm automatic loading of the same geometry. The unavailable-reference test is explicitly skipped for this already-loading startup flow; the ordinary front/rear tests exercised it.
- All11 retained sources pass angular agreement at1/2/4/8 subdivisions: max velocity/acceleration/jerk disagreement3.11e-10/3.06e-8/1.39e-5 against unchanged1e-4/1e-3/1e-2 tolerances. `out/recovery-balanced-angular.log`.

## Important implementation details

A short crown-hold starting estimate fixes two reproducible nearby-speed Immelmann solve failures without changing requested forces or heights. Return composition adjusts bounded upstream authored bearings, then rebuilds the real FVD source and repeats every acceptance check; no points or endpoints are snapped.

The 310km/h, airtime1.1, roll55 case passes after shortening only the final camelback constant-load hold by0.1s and multiplying signature airtime by0.95. These coarse bounded corrections are recorded in planning JSON and followed by full new validation. The default needs neither force correction. No force or standard limits were relaxed to pass that case.

## Interrupted delivery and cleanup

Standalone packaging from18d3fb2 was stopped during cooking. Its log is `out/default2-package.log`; build evidence is `native/unreal/Saved/BuildRuns/20260924-120626-375`. No `dist/2.0.0-default.2/.../package-manifest.json` was produced, and `dist/current.json` was not changed. Completing and verifying a standalone package is optional future work only when a new user request resumes it.

Cleanup removed this session's three marked verification-profile copies, partial `native/unreal/Saved/Cooked/Windows`, one-use crown-probe files and regenerated contact sheets. Original screenshots, reports, accepted designs, source/reference inputs, working game binaries and build caches remain. Reusable runtime helpers moved to `out/default2-delivery-tools`. Receipt: `out/default2-cleanup-receipt.json`.

The original Default1 package/profile, rejected rewrite branch/archive, RFDB exports and historical recovery evidence remain untouched. `out/default2-delivery-state.json` records the stopped status. Older session-state notes and earlier handoffs are superseded by this file.

## Environment

Work solo; no agents are needed or active. One compiler worker if work is explicitly resumed. Normal exec/image tools encounter a Windows sandbox-helper failure; elevated exec was used. Git requires process-local `safe.directory=D:/Coding/Codex/Vibecoaster2`. Python: `C:/Users/danie/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe`. Unreal: `D:/Games/Epic Games/UE_5.8`. Native build helpers remain in scratch; package helper is `native/unreal/scripts/package.ps1`. No tests or builds remain running.
