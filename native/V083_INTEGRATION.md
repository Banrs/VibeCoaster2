# v083 integration qualification

## Placed-track ownership and CI qualification (pending)

The follow-up to `e0dd5c2c60b8d58c7c594be43e52f7955a477f27` moves the final
placed track into its owning `Design` and migrates test readers to that owner.
Source/link interval metadata remains available. The before-change circuit
suite passed 5,432 checks; post-change native qualification is pending CI.
The preserved copy/move probe measured 2,395,248 bytes of duplicate vector
payload for its fixture, excluding allocator/capacity overhead. It establishes
the removed copy's storage cost, not a measured runtime speedup.

The [fixed CI qualification](tools/qualification/README.md) retains all 36 tests,
44 unchanged matrix requests and eight required rides. A single portable build
feeds four jobs with two workers each. Exact saves, replay, independent audits,
organic checks, raw outputs and source/binary hashes remain required.
[Current evidence](artifacts/native-recovery-20260912/) preserves the original
compilation-receipt failure, implementation review and tooling verification.
The stable tooling passed a fresh 196-test Python discovery run (188 passed,
eight real CLI tests skipped locally for CI execution). Twelve of those tests
exercise missing/mixed source identities, corrupted raw evidence, failed
commands, coverage and immutable output handling. The new raw reader also
validated all 52 preserved source64 bundles without launching native code;
that establishes reader compatibility, not new physical acceptance.

The user requested no local simulations and then paused local Unreal/high-load
work while gaming. Engine repair was authorized and delegated to the existing
recovery task, but is now paused. The current UE payload lacks the editor and
build tools; historical UE results below do not qualify this native change.
Finish code/CI checks and let the user verify before merging. This continuation
does not claim full milestone completion.

**Source64 has passed every required local gate on rebuilt, consistently hashed binaries.** [Promotion records](artifacts/flow-intent-v083-20260910/integration-20260911/promotion-64/) and [GitHub checks](https://github.com/Banrs/VibeCoaster2/actions/workflows/native.yml) record the integration and main commit status. [NEXT_CHAT.md](NEXT_CHAT.md) is the continuation entry point.

Identity remains **0.8.3-flow.1 / COASTER5**. Public APIs are retained against the approved source59 checkpoint, not every historical experimental FVD interface on main. This continuation retains CLI arguments, save representation, selected targets, force/power limits, eight whole-ride builds, 0.5 m/s feedback and 960/1920 Hz acceptance.

## Portable driver review after source64

The requested deflation/skeptic review found a coverage omission: portable `build.ps1 -Test` ran the native suites but never supplied its new CLI to `test_cli_arguments.py`. Ordinary Python discovery skipped all eight executable contracts. The driver now resolves Python 3 only for `-Test`, invokes that unchanged suite with its newly built CLI and propagates a failing exit. A reproduced Windows PowerShell 5.1 stderr-discovery failure was corrected and re-reviewed; configured interpreter, fallback, warning, missing-interpreter, launcher and build-only cases have bounded execution evidence.

Evidence is separate from source64 under [deflation-skeptic-20260912](artifacts/deflation-skeptic-20260912/). The [scoped re-review](artifacts/deflation-skeptic-20260912/task-1-rereview.md) and [verification scope](artifacts/deflation-skeptic-20260912/VERIFICATION_SCOPE.md) explain the contracts and limitations. All eight real CLI contracts passed against the fresh executable before the qualification checkpoint commit; ordinary Python discovery passed 176 tests with eight CLI skips. The full actual Release portable driver uses Windows PowerShell 5.1, Python 3.13.15 and Zig 0.14.1. Its command, frozen inputs, complete log, final status and output hashes are recorded in `portable-final-2/`. The driver covers the test bodies represented by all 36 CTest registrations through 26 native processes and the eight-test Python process; it does not provide CTest's scheduling or separate grouped processes.

`local-verified.json` is written only after complete local success and final hash checks. A reviewed checkpoint may enter CI while the full local suite continues. Merge requires both the full local driver and exact-commit Windows/macOS integration CI to pass, followed by main CI. The new `promotion/complete.json` is written only after every gate passes; its linked run receipts establish final status. No Linux gate is added.

The adjacent design review found no demonstrated numerical defect in 6,509 focused checks. A roughly 2.4 MB placed-track copy, repeated certification and duplicated import helpers were investigated. The copy has dependent test ownership; certification spans distinct public validation contracts; import consolidation needs an actual import witness. Crossing-scan invariance checks did not establish equivalence throughout placement. Those optional changes were not implemented solely to reduce line counts. Reports preserve the measured evidence and rejected approaches.

All 168 maintained qualification inputs were compared: 166 match source64, with only `build.ps1` and `BUILDING.md` changed. The historical qualification manifest also includes three separate evidence-driver sources. Native code, fixtures, criteria, compiler flags and Unreal source remain unchanged. The preserved 44-request, eight-ride, 960/1920 Hz and ten-UE results belong to their source64 binaries and saves; they are not new executions of this tooling correction. The local UE installation is now incomplete, as recorded in [the installation investigation](artifacts/deflation-skeptic-20260912/unreal-investigation.json). Any subsequent native correction requires full fresh qualification, including real UE checks.

Evidence below is under `artifacts/flow-intent-v083-20260910/integration-20260911/`. Failed attempts and historical snapshots remain separate from promoted results.

## Current source64

The continuation registers the existing Graphify installation, hoists site-independent terrain inputs, assigns passive-transfer feedback to its local energy discrepancy, corrects the source-port turn-capacity constraint, and uses the core reference validator for UE preflight/menu availability. Source64 also removes the unreachable post-generation missing-reference branch. Root-cause reproductions and review are recorded in [the continuation ledger](artifacts/flow-intent-v083-20260910/integration-20260911/continuation-20260912/REVIEW.md).

| Check | Current result | Receipt |
|---|---|---|
| Source and executable binding | 168 inputs and 65 binaries; external UE compiler dependencies included | `qualification-64/provenance.json` |
| Native/CLI suites | All 36 pass: 20 components before 16 integrations, including all eight real-CLI argument tests; Release, two workers, existing timeouts and stop-on-failure | `qualification-64/ctest-component-result.json`, `ctest-integration-result.json` |
| Portable build/components/wrappers | Build passes; 20 components pass; wrapper and separate generation-wrapper compilation pass | `qualification-64/portable-build-result.json`, `portable-component-result.json`, `wrappers-result.json`, `wrapper-generation-result.json` |
| Python | 176 pass; eight CLI tests deferred to the real executable in CTest | `qualification-64/python-result.json` |
| All 44 predeclared requests | All pass candidate zero, exact saved replay and independent 960/1920 Hz audits; 1,011.375 s; no substitutions or acceptance changes | `architecture-matrix-64/results.json`, `qualification-64/matrix-result.json` |
| Required eight rides | All pass candidate-zero generation, exact saved replay, independent 960/1920 Hz and organic audits; 317.407 s | `itinerary-final-64/results.json`, `qualification-64/panel-result.json` |
| UE build and contracts | Build passes; all ten real UE contracts pass, zero failed/not-run/in-process, including eight permanent contracts and terrain-parity/saved-render evidence | `qualification-64/ue-build-result.json`, `ue-automation-result.json`, `automation/index.json` |
| Motor and pacing controls | 25 recorded runs; all 40 positive motors matched to source57 retain force/power ratings; largest complete-stop duration increase is 0.39375 s | `control-comparison-64/input-manifest.json`, `source57-comparison.json`, `all-positive-motors.json` |
| Consolidated local verification | All declared tests, requests, raw replays/audits, source/binary/control hashes and 40 matched positive motors verified | `qualification-64/verified-local-gates.json`, `verify-final2.log` |
| Integration and main CI | Commit-specific remote results are recorded separately; integration CI must pass before merge, followed by main CI | `promotion-64/` and GitHub checks |

All eight source64 saves are byte-identical to source63. Motor hardware length, inlet/outlet speed, quiet-tail and duration measurements remain descriptive; they introduce no physical acceptance threshold. The eight cases remain **flat5, flat7, flat42, hills2, hills9, canyon1, canyon24 and canyon42**. Hills2 retains its explicit private crossover brief, full clearance and organic checks; all cases retain the under-35% straight-flat requirement.

All eight retain four audited airtime crests and 11.01–14.45% straight-flat share. Hills2's transverse crossover has 119.758 m canonical separation. Its organic audit passes 6,317 checks. The departure measurement begins at the recorded rest state inside launch hardware; no earlier inlet is fabricated. The rejected historical control remains in the 25-run comparison.

Graphify **0.9.51** is registered for Codex with eight references. The final code-only index covers 125 maintained inputs, including C++ headers and implementations, with 2,159 nodes and 5,167 edges. Five representative relationships were checked against source. Three declaration headers have parser limitations; graph findings guide inspection and do not establish correctness. Receipts are [setup verification](artifacts/flow-intent-v083-20260910/integration-20260911/continuation-20260912/setup-verification.json) and [graph verification](artifacts/flow-intent-v083-20260910/integration-20260911/continuation-20260912/graph-verification-64.json).

Rules 1, 2 and 4 remain text-identical to the community-maintained [Karpathy-inspired guidelines](https://github.com/multica-ai/andrej-karpathy-skills/blob/main/CLAUDE.md), which are not authored by Karpathy. Rule 3 explicitly permits adjacent agent-authored repairs. Relevant [Superpowers workflows](https://github.com/obra/superpowers) govern reproduction, review and fresh verification within the user's authorized scope.

## Source63 completed qualification — before the final cleanup

Source63 completed all 36 CTest suites, all 44 unchanged requests, all eight saved-ride audits and all ten real UE contracts. Its receipts remain in `qualification-63/`, `architecture-matrix-63/` and `itinerary-final-63/`. Source64 removes the dead reference branch and repeats qualification against rebuilt, consistently hashed executables; earlier green results are historical evidence, not a substitute.

## Source59 historical checkpoint

Source59 was the development checkpoint at the user's 12 September handoff. The [handoff manifests](artifacts/flow-intent-v083-20260910/integration-20260911/handoff-20260912-source59/) bind that source, its executables and accepted regression save.

| Check | Actual result | Receipt |
|---|---|---|
| Fresh portable build | All 28 executables; 128.05 s | `validation-speed-59/portable-build-result.json` |
| Focused support suite | 30,259 checks pass | `validation-speed-59/support-focused.log` |
| Canonical source binding | 69/69 source59 core source/test files match its frozen build | `handoff-20260912-source59/core-source-parity.json` |
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

## Promotion evidence

The complete integration diff and final provenance have been reviewed, including independent code and evidence reviews. Append-only records under `promotion-64/` bind the final integration commit to green Windows/macOS CI, then the merge and green main CI. The user's latest scope removes required Linux tests while retaining portability for possible future support. The resulting CI/documentation change leaves qualified native code unchanged. GitHub checks are authoritative for remote status. Any later source correction requires rebuilt final-source qualification.

Do not weaken acceptance or substitute seeds. The supplied Falcon angle scale is inaccurate and supplies no hard grade/quiet percentage. Packaging, benchmark qualification, POV review, measured FPS/load benchmarks and Mac/Metal runtime verification are outside this milestone.
