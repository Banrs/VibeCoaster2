# v083 checkpoint qualification

**Source59 is a development checkpoint, not final integration qualification.** Work stopped at the user's request on 12 September 2026. Main remains unmerged. Start the next session with [NEXT_CHAT.md](NEXT_CHAT.md).

Identity remains **0.8.3-flow.1 / COASTER5**. Public generation calls, CLI arguments, save representation, selected targets, force/power limits, eight whole-ride builds, 0.5 m/s feedback and 960/1920 Hz acceptance are retained.

Evidence below is under `artifacts/flow-intent-v083-20260910/integration-20260911/`. Failed attempts are preserved separately. The [handoff manifests](artifacts/flow-intent-v083-20260910/integration-20260911/handoff-20260912-source59/) bind current source, executables and the accepted regression save.

## Current source59

| Check | Actual result | Receipt |
|---|---|---|
| Fresh portable build | All 28 executables; 128.05 s | `validation-speed-59/portable-build-result.json` |
| Focused support suite | 30,259 checks pass | `validation-speed-59/support-focused.log` |
| Canonical source binding | 69/69 current core source/test files match the frozen build | `handoff-20260912-source59/core-source-parity.json` |
| Canyon0, 12 cars | Accepted candidate zero; genuine save; exact independent replay; 960/1920 Hz audit passes | `support-budget-59-case5/result.json` |
| Geometry preservation for that correction | All 4,969 canonical knot rows exactly match the rejected source57 witness | `support-budget-59-case5/canonical-parity.json` |

The regression ride completes in **165.30520833371247 s**. Save SHA256: `a74c395051f3eb3692eea1f4afc34b84dc40b51d8c8a34f2d67d17bf27183b4b`.

The support correction reconciles two conflicting owners: a 600 m tower with 16 m tiers emits up to 620 canonical members, while the former per-support cap was 512. The cap is now derived from the existing family. It retains all members, dimensions, tier spacing, the 60,000-member total budget and physical checks. Oversized count rejection tests follow the derived bounds. [Independent old/new witnesses](artifacts/flow-intent-v083-20260910/integration-20260911/support-diagnosis-58/README.md) reproduce the old failure and verify the complete corrected support layout without regenerating the track.

## Source57 controls — not final source59 qualification

All eight required rides accept candidate zero, exactly replay and pass independent convergence and organic audits in `itinerary-final-57/results.json`. The complete sequential panel takes 290.44 seconds, including generation, saving and audits.

| Ride | Complete-stop seconds | Saved replay / convergence / organic |
|---|---:|---|
| flat5 | 159.914583 | Pass |
| flat7 | 160.409375 | Pass |
| flat42 | 158.829167 | Pass |
| hills2, explicit crossover brief | 163.834375 | Pass |
| hills9 | 165.681250 | Pass |
| canyon1 | 161.557292 | Pass |
| canyon24 | 158.076042 | Pass |
| canyon42 | 158.807292 | Pass |

Each case directory retains its save hash, report, plan, trace, independent replay and audit. Hills2 has one transverse nonlocal crossing, **120.336 m** canonical vertical separation, four actual crests and **11.2179%** straight-flat share; its organic audit passes 6,317 checks.

The 44-request source57 matrix was stopped after **11 completed cases: 10 pass, one failure**. Case5, canyon0/12 cars, exposed the member-budget inconsistency. Two active requests were interrupted and remaining requests were not run. `architecture-matrix-57/interruption.json` is the authoritative ledger. The successful source59 regression does not convert this partial matrix into a pass.

Other source57 checks:
- 20/20 CMake component suites in 35.24 s; 20/20 equivalent portable checks.
- Route integration and requested-crossover integration both pass on the CMake build, 103.96 s combined.
- Python: 184 discovered, 176 passed, eight real-CLI tests skipped for their native suite.
- All 28 portable executables build using one core archive. Four existing aggregate-initializer warnings remain in unchanged tests; no new warning is introduced.
- Exact-input airtime reuse reduces a measured flat7 generation from 58.7138 to 39.9552 s, preserving complete report and plan hashes. Profiles are evidence-only code in `generation-profile-55/` and `airtime-sharing-56/`.

## Earlier retained controls

Source54 passes all 30 native suites on one frozen build: two support regressions in 215.62 s, then 28 remaining suites in 2,135.08 s. Its detailed receipts remain in `final-source-54/`. Source57's test registration splits the organic suite and adds focused entries, giving 36 CMake tests; acceptance coverage is retained.

Source48 / `3b63f161dc828f32f4011776e6b84ff58b871589` has complete local qualification and green [Windows/Linux/macOS CI](https://github.com/Banrs/VibeCoaster2/actions/runs/34662459847). Its indexed UE terrain fix passes ten real UE contracts and preserves exact expanded triangles. These results do not qualify later source or later saved payloads.

Detailed older qualification tables, control comparisons and dated handoff paragraphs are retained in [pre-cleanup documents](artifacts/flow-intent-v083-20260910/integration-20260911/handoff-20260912-source59/pre-cleanup/), previous commits and their original evidence directories.

## Remaining promotion gates

Final source still needs the complete native/CLI suite, all 44 predeclared requests, all eight required independent saved-ride audits, Python/portable-wrapper confirmation, relevant real UE tests, motor/quiet-tail/duration comparisons, and final diff/provenance review. Obtain green CI for the final integration commit, then merge and verify main CI.

Do not weaken acceptance or substitute seeds. The supplied Falcon angle scale is inaccurate and supplies no hard grade/quiet percentage. Packaging, benchmark qualification, POV review, measured FPS/load benchmarks and Mac/Metal runtime verification are outside this milestone.
