# VibeCoaster2 basic generator checkpoint

Updated 24 September 2026. Source is reviewed and local verification is complete. Release 2.0.0-default.2 is being prepared; Windows/macOS CI, versioned packaging, packaged-runtime verification and promotion remain. No agents are active or needed.

## User direction

The eventual game is like NoLimits 2 with fast automatic generation as a starting point for coaster design/refinement. Several coasters share one environment, no park management, and only optional/skippable queue-to-seat walking. This task is the basic generator. See docs/product-vision.md.

Work solo. Do not reuse old agents. Graphify and efficient-subagent-waiting were read earlier; no graph/runtime was available. User instructions override forced delegation.

Use approximately 1.33x FF scale and 1.2x speed (300+ km/h). Approximate element/phase force baselines: FF +15%, TRR +8 1/3%, others +10%; negative loads use magnitude. Check actual seats and phases. Up to 5% over project force/rate/pacing limits is allowed only alongside unchanged independent F2291 checks. Do not cherry-pick rounding. Stop fine-tuning tiny photo differences. Lower-drag stress acceptance is removed; configured drag and trims-off/fully-deployed hardware checks remain.

## Selected result

`out/recovery-balanced-inversion.vcdesign` and report/trace/plan: seed42 highlands, 6166.9249 m, 133.9906 s, 300.534 km/h, Gz -1.42842 to +5.00165, maximum |Gy| .98962. All physical/source/spatial/refinement gates pass. The +5 g nominal excess is explicitly reported under the project allowance.

Camelback: one isotropic planar asymmetric fit, 3.53824 px RMS / 12.7918 px max against practical 4/16 px guards. All-seat ascending/airtime/recovery phases exceed their FF +15% baselines. Do not fit it again. Loop: normal4.85/crest2/recovery3.9, rise145/yaw33. Immelmann: normal4.85/crest3.8/roll-exit2/recovery3, rise85/yaw45, rollReleaseFraction .45. Both explicitly unload over1.2s into a solved real crown hold.

The exact pointwise reference comparison remains uncertain because recordings have no surveyed distance. Approximate per-phase goals and known limitations are documented in docs/reference-force-audit.md. The optional calibrated-reference intensity flag stays separate; no whole-standard legal-certification claim.

## Completed local verification

- 16/16 component suites: out/recovery-adaptive-components.log, including 66303 FVD checks.
- 10/10 integration suites: out/recovery-balanced-integration.log, before the bounded force-feedback addition; CI will run the final complete suite.
- 8/8 final corpus, save/reload and seed diversity: out/recovery-final-generator-corpus/summary.json.
- The 310 km/h high-airtime case now passes after shortening only the final camelback hold by .1s and multiplying signature airtime by .95. Both coarse bounded corrections are explicit in planning JSON and require full new validation. The default needs no force correction. No limits were relaxed for this case.
- A short crown-hold solver seed fixes two nearby-speed Immelmann construction failures, with regression coverage.
- All11 retained default sources pass angular agreement at1/2/4/8 subdivisions. Max velocity/acceleration/jerk error3.11e-10/3.06e-8/1.39e-5 vs unchanged1e-4/1e-3/1e-2 tolerances: out/recovery-balanced-angular.log.
- Both 2560x1440 front/rear Unreal traversals completed, matching geometrySHA1 09CDA5902EC0D2B52E1BE049AF94125BDC382329. Pause/restart/load/pose and generation/mesh/scene/save cancellation checks pass. All46 captures per seat were inspected by the primary agent: out/recovery-final-generator-visual-review.json. Human approval, continuous-video review, keyboard interaction and GPU performance are not claimed.

## Delivery and shortcut

Root `Play VibeCoaster2.lnk` and Desktop `VibeCoaster2.lnk` currently directly open the restored published Default1 package/profile. `native/unreal/scripts/create_shortcuts.ps1 -Desktop` verifies manifest/executable hashes and refreshes both links after promotion. Keep dist/current.json unchanged until the new versioned package is verified. Original Default1 package/profile and the rejected rewrite archive remain preserved.

Before packaging, commit reviewed source, pass Windows/macOS CI, and keep the package source tree clean. Version is now2.0.0-default.2 and Default1 remains an explicitly supported saved provenance. Package only the reviewed source; verify the packaged executable before updating current.json and both shortcuts. Update this handoff with actual commit, CI and package paths on completion.

## Environment

Branch codex/legacy-authoring, original HEAD a2149dfb0c91a0264ae70bd49ff74bba1073f930. Preserve staged AGENTS.md/NEXT_SESSION.md deletions and the inherited recovery edits. No compiler/runtime commands active at this snapshot; a release-identity native build follows.

Use one compiler worker. Elevated exec is needed because the normal Windows sandbox helper fails. Git needs process-local safe.directory=D:/Coding/Codex/Vibecoaster2. Build scripts: scratch/build-all-checks.cmd and build-angular-probe.cmd. Python: C:/Users/danie/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe. Unreal: D:/Games/Epic Games/UE_5.8. Package script: native/unreal/scripts/package.ps1. Runtime helper: scratch/run-balanced-runtime.ps1. Full local research history remains under out/reference-audit; original RFDB exports are untouched in the legacy archive.
