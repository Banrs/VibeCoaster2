# Windows recovery delivery: 0.7.3-review.5

[Download the Windows ZIP](https://github.com/Banrs/OpenVibeCoaster/releases/tag/native-v0.7.3-review.5).
Extract it completely, then open **Play VibeCoaster.cmd** or **VibeCoaster.exe**.
The launcher uses a 1600×900 window with a 60fps cap. The EXE keeps normal saved
settings; fresh profiles now also default to 60fps. Unreal Editor and Blender are
not needed to play. Keep the Engine and VibeCoaster folders beside the EXE.

Select **PHYSICS-PROOF**, **Canyon**, seed **42**, then press Enter to generate
and Space to ride. Up/Down chooses a setting; Left/Right changes it. Tab toggles
setup, 1/2/3 switches seats while riding, M shows overview, R restarts, F5 saves,
F9 loads, and Alt+F4 exits. If MSVC runtime DLLs are missing, run the included
`Engine/Extras/Redist/en-us/vc_redist.x64.exe`.

ALL RECORDS remains explicitly unavailable without authentic eligible reference
recordings. PHYSICS-PROOF checks the other selected targets and full numerical
acceptance. It is not an intensity-record claim.

## Recovery and changes

The pre-crash review.2 package survived intact: all 50 packaged files and all
211 files in its active source snapshot matched their saved SHA256 hashes.
The recovery adds a 60fps default, removes unused Android file-server settings,
supplies a verified player ZIP/launcher, restores historical test fixtures and
fixes concurrent-save temporary-file ownership. Review.5 also corrects track-brace
distance evaluation in optimized Windows builds. Canonical geometry, simulation,
force limits and COASTER5 save identity remain **0.7.2-pacing.2**; loaded saves
undergo full validation with the corrected clearance check.

The previous simulation optimization, batched train rendering, cached HUD geometry
and save-cancellation fix remain intact. The wider review reproduced and fixed a two-writer save race: each save now
exclusively owns its temporary file. Historical source, failed runs and frozen
evidence were preserved. At the user's request, the obsolete TypeScript/browser
implementation and its tooling were retired from the active repository after a
verified recovery archive captured all uncommitted browser work.

Build and startup results for this delivery are recorded in the GitHub release
notes. No performance benchmarks were run during recovery because Cities:
Skylines 2 was open. Previous review.2 timing results are historical, not new
measurements of this recovery. The review.4 draft was withheld after Windows CI
exposed the clearance failure; its package and failed evidence remain preserved.
Large local raw evidence and engine/build caches are
excluded from source control.

## Verified checks

- Final Windows editor/game build, five UE contracts, cooking and packaging passed.
- Extracted player EXE initialized the real Ride map and exited successfully in a
  brief startup check. A fresh profile reported `t.MaxFPS = 60`.
- The concurrent-save regression failed the old implementation and passed all 28
  checks after the fix. Eight additional targeted native suites passed.
- The corrected track-brace distance suite passes 140 checks in optimized Windows
  Precise and Strict modes, including an independent analytic regression.
- All 161 pre-existing/distribution Python tests passed; six additional real-CLI
  argument tests passed after rejecting malformed numeric input. CLI save roundtrip
  and independent convergence evidence are recorded in the local recovery report.
- All 48 distributed runtime files were copied with matching SHA256 hashes; ZIP
  CRC verification passed. The download includes a SHA256 checksum file.

These checks are not a new full-ride/POV or frame-time benchmark. Prior review.2
full-ride and performance evidence remains separately identified.

## Scope

This is an unsigned Windows development preview, not a completed foundation.
Authentic benchmark recordings, actual Mac/Metal compilation and testing, further
scenery work and continuous human POV/keyboard review remain outstanding.
No ASTM compliance or structural certification is claimed.

Source builds: [BUILDING.md](tools/BUILDING.md). Numerical scope:
[NUMERICS.md](core/NUMERICS.md). Mac preparation: [MACOS.md](unreal/MACOS.md).
