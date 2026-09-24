# Riftwake Default 3 accepted checkpoint

Updated 2026-09-25. The geometry continuation is complete and activated in the working editor-game profile. This file supersedes earlier paused/reapply directions. Do not reapply `docs/checkpoints/08-riftwake-paused.patch`; its work is integrated and further repaired.

## Current accepted result

- `out/riftwake-resume-06.vcdesign`, seed42, default candidate0: **181.961458 s**, **7 row modules / 14 riders**, 300.504 km/h peak, retained 0–180 km/h in1.397 s.
- Clifftop active27.0529 s, lip5.7469 s. Distance-weighted mean rail-ground47.40975 m versus old54.80261 m (13.49% lower); 60Hz time-weighted48.38214 m versus old55.43613 m. Native minimum swept-ground+2.5616 m.
- All native errors empty. Authorship, numerical acceleration histories, forces/rates, terrain/self/support/station clearances, temporal960/1920 Hz convergence, independent half-spacing spatial replay PASS. No force/clearance gates were relaxed.
- `out/riftwake-nearby-43.vcdesign`: nearby seed43, normal defaults, candidate1,181.390625 s, all acceptance/refinement checks pass.
- Live save **UserData-Development/Saved/VibeCoaster2/Designs/Accepted.vcdesign** is now the accepted14-rider ride. SHA256 DB3D4B55F055036A3D18ABDD54D3D1ADF5EC8EC8988F41F760ECB42DEF03A8CA.
- Prior12-rider save and editor binaries preserved at `out/pre-riftwake-runtime/`. Prior save SHA256 0F7F7ADDD98EFB447C4DB5D234EBD8620D1E9C0BF849638EC1546CC3DFAA97EB. `out/riftwake-activation.json` records the atomic replacement. Existing root/Desktop shortcuts keep their direct UnrealEditor -game -CoasterLoad arguments and working profile.
- No new standalone package; `dist/current.json` still points to the older standalone. Do not promote it without a real package and review.

## What changed

- Default train now7 current single-row modules, each2 seats; `riderCapacity()` and report JSON derive14. Explicit6-row saves retain6/12; the previous accepted save was independently revalidated. True rigid four-car grouping remains deferred.
- Added low early hills and fixed1000m hooked clifftop; retained protected camelback; gentler Loop/Immelmann; original outward-wing → ravine-transfer → counter-wing → low-carve signature with opposing45deg crests.
- Terrain curves now use two broad exact septic pitch phases, fixed combined length/height, full endpoint jets, one semantic section and original crest-bank ownership. The prior cosine-clustered control edit collapsed curvature over0.75m and caused~1145rad/s³ angular jerk. Corrected local join now~0.211rad/s³; full ride rates pass. Existing native regression covers physical behavior.
- Immelmann crest1.0g constructs the actual61.44m/s inherited port; broad40% roll-rate ramps reduce harsh roll. Changing only recovery or extra solver seeds did not solve the old0.6g request; this was not treated as impossibility proof.
- Loop retains15deg final plane yaw, completed during ascent. That separates its branches with real Gy/twist integration; native isolated source clearance passed with extra1.75m margin. Full current ride passes independently.
- Authored setup+.45rad and signature-approach-.45rad establish compact closure; feedback remains separate and signature internal shape is preserved. Default candidate0 accepts. Broad cliff-foot bench/ravine adjustments clear low track; lip braking13m/s² meets pacing.
- Native support system is the new tubular/raked/shared-frame design, with16-sided canonical meshes. Tie saddles and LSM bridge clearance are integrated. Source version and DefaultGame.ini are2.0.0-default.3.
- Native load retains independent revalidation with structures/station overlapped with geometry. No additional speculative performance refactor was adopted.

## Verified evidence

- `docs/checkpoints/09-riftwake-default3.md` is the readable checkpoint report.
- `out/riftwake-resume-06-{report,trace,plan}.json` and `out/riftwake-nearby-43-*` contain full numerical evidence.
- Clean CLI build completed before source iterations. Final `organic_generation_tests --baseline-only out/riftwake-resume-06.vcdesign`:805317 checks (C3, continuous rendered forces, trim scenarios, persistence).
- Inherited-port:51791 checks. Motion-spline:16893. Core analytic/capacity:2516. FVD/recipe/drive/track-web/clearance component suites pass. Station on old accepted baseline:1129 checks.
- Organic support audit on old accepted track after clean integration:175 supports,67one-foot/56wishbones/52A-backstays,1623members,124inter-frame links; exact deterministic regeneration and mesh containment/winding pass.
- UnrealEditor target rebuilt successfully with UE5.8/VS14.44. Two real2560x1440 runtime traversals passed automation. First visual review found opaque glass/material fallback. Fixed V2 material instancing flags and compiled with real RHI; importer now sets the usage flag and package content preparation uses real RHI.
- Corrected result: `out/riftwake-runtime-review-fixed/result.json`. Full traversal, load, pause/restart, paused pose, save/generation/mesh/scene cancellation pass. Save hash unchanged after runtime. Assistant reviewed actual overview/station/opening/cliff/inversion/signature PNGs and confirmed clear POV and terrain shading. Human styling approval and keyboard-input review remain unclaimed.
- Isolated runtime profile: `out/riftwake-runtime-profile`. No verifier/editor process remains running after final completion.

## Remaining art, performance and packaging work

The user rejected prior train styling and huge grand-station concept. Those are still drafts, not approved design. The live geometry/core station is the original physical station builder scaled to7 rows (stop centre40.2m). Existing Unreal render/art source and V2 assets from the incoming handoff remain locally modified/untracked. Do not discard them with a reset or claim the model design is approved. The active editor binary was built from this working tree before the geometry checkpoint commit.

An independent functional station study is complete in `scratch/station-resume/`: `station_functional.py`, `functional-station-preview.png`, `README.md`. Seven row centres30.0…50.4m match40.2m stop. Covered queue/merge, aligned holding lanes/gates, dispatch cabin, separate unload/exit, stairs and step-free lifts. It is isolated; roof hidden only in cutaway. No canonical/runtime integration yet. Do not let further modelling block accepted geometry.

The material bug was a missing InstancedStaticMeshes usage flag, making intended translucent visor opaque. The canonical eye/force point was correct and was not moved. Current generated V2 material assets were repaired by `scratch/repair-riftwake-materials.py`; receipt `out/riftwake-runtime-materials.json`. The persistent importer fix is in `native/unreal/scripts/import_v2_art.py`. Shader priming with the real RHI fixed the terrain fallback; no geometry or displacement changed.

Loading review found possible future station broad-phase squared-distance/spatial indexing and duplicate track rebuilding, but impact is unmeasured; no speculative changes made. Native old-save load was2.32s in one warm sample. Editor startup still adds substantial time. Standalone work is needed for the earlier3s total startup ambition; not achieved or newly packaged here.

## User priorities and working style

Geometry and realistic force/roll transitions remain first, preserving authored inversions/signature intent and lower average terrain-relative height. Approximately180s is wanted,175–190 acceptable; keep explicitly retained extreme launch. Models and independent Luna refactor/optimization discovery may proceed in parallel with bounded ownership. No imposed geometry-first completion gate for independent reviews.

User authorizes extensive Luna MAX work, Sol HIGH/XHIGH, Astra HIGH/XHIGH/MAX for hardest work; up to3Astra/6Sol, upwards of60Luna. Root integrates/reviews; delegate bounded work. Every agent reads `C:/Users/danie/.codex/skills/efficient-subagent-waiting/SKILL.md`, propagates it, and uses event-driven waits without status polling. Avoid needless broad suites/CI and photo micro-tuning. Commit/push meaningful checkpoints with `[skip ci]`. No agents/processes from this continuation need waiting after completion.

## Workspace and tools

Repo D:/Coding/Codex/Vibecoaster2, branch codex/legacy-authoring, origin https://github.com/Banrs/VibeCoaster2.git. All original and continued geometry work is applied. Use process-local `git -c safe.directory=D:/Coding/Codex/Vibecoaster2`; do not change global safe.directory.

Normal exec/apply_patch/Node/view_image helpers fail with helper_unknown_error setup refresh. Use `exec_command` with `sandbox_permissions=require_escalated` and concise authorized-work justification. Python text I/O explicitUTF8. Python: C:/Users/danie/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe. Native VS2022 CLI toolchain14.38 at D:/Toolchains/VS2022; UE5.8 at D:/Games/Epic Games/UE_5.8 uses14.44. `scratch/build-legacy-baseline.cmd` builds CLI; `scratch/build-riftwake-clean.cmd` clean rebuilds. Do not link stale scratch replacement objects into acceptance binaries.

Image inspection workaround: elevated Python/Pillow thumbnail JPEG into BytesIO, printbase64, then functions.image(dataURL + result.output.trim()) without text; keepJPEGsmall. Distinguish actual runtime captures from Blender.

No graphify graph/runtime exists; the skill was read and direct source inspection used. Blender5.2.1 at D:/Toolchains/Blender/blender-5.2.1-windows-x64/blender.exe; MCP loopback9876 and wrapper paths remain as previous handoff. The functional station render used a separate background Blender process, preserving shared scene.

User requested Fast OFF. Saved config still service_tier="default". This task's own log records gpt-6-astra/effortmax but omits active service tier; tools expose no speed switch. Do not claim Fast is confirmed off. Official guidance https://learn.chatgpt.com/docs/agent-configuration/speed .
