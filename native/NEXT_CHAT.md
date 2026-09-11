# Resume the v083 engineering checkpoint

Continue the same native C++/UE game in **D:\Coding\Codex\Vibecoasterjs**. Do not restart it. The user requested a reasonable stopping point followed by a new chat. The source and experiments below are deliberately distinguished: the shorter experimental rides have **not** replaced the published executable.

Read `AGENTS.md`, this file, `WORK_IN_PROGRESS.md`, `VALIDATION.md`, `core/FORCE_GUIDELINES.md` and `core/FVD.md`. AGENTS sections 1/2/4 are the user's verbatim wording; section 3 was explicitly excluded. Prefer one owner for each calculation, remove obsolete compensating paths, and do not force a line-count reduction or weaken acceptance to obtain a pass.

## Repository and delivery state

- Native runtime/source identity: **0.8.3-flow.1 / COASTER5**. Active build: `native/artifacts/generator-intent-v081/build/Release/`.
- The v083 work is on **`codex/flow-v083-checkpoint`**, an unfinished engineering checkpoint, not a green release. `main` remains the default branch. Inspect the final checkpoint receipt in `artifacts/flow-intent-v083-20260910/handoff-20260911/` before continuing.
- Published Windows package and launcher remain **0.8.2-terrain.1-6f23a9e**. See `DELIVERY.md`. Do not claim a new EXE or silently launch an old package as new work.
- Preserve all failed/frozen evidence and the unrelated untracked `native/unreal/Content/Art/V072/Conventional1/` directory. Do not commit that directory as part of this work.
- All three active workers eventually hit the account usage limit. Their files are retained; no live verification process remained at the stopping inventory. Do not silently switch to paid Go or send private repository material to Muse. The previously rejected private Muse transfer remains unauthorized. Public-only research may use the user's preferred free Muse model when available.

## Current production source, already integrated

- Explicit higher historical restraint-dependent force profile; signed duration, filtered onset, post-uplift and paired-axis histories at all rider positions. See the force guideline/model scope; this is not F2291-26 or physical-restraint certification.
- Loss-aware FVD sources, stronger phase intentions around +5/−1.5g with actual toleranced excursions, and distinct floater/positive/ejector crest roles. Do not clip the waveform or interpret those values as whole-ride means.
- A single coupled Immelmann/pullout source replaces the separate solvers and removes the artificial lateral/approach-crest overlays. Its true rolling descent was inspected in the actual UE renderer.
- Ordinary turn ramps derive from 80°/s twist and 150°/s² onset intentions. Final frame/force checks remain authoritative; these nominal values are not new ASTM caps.
- Keep the final bank smoothing. A direct force-alignment replacement passed flat terrain but failed violently near a nearly weightless canyon crest. Evidence: `artifacts/flow-intent-v083-20260910/fvd-chain/turn-phase-study/height-bank-owner/DECISION.md`.
- Terminal Station Operation now receives its computed `stopDeceleration`, matching the stopping-distance and hardware sizing calculation.
- The last core fix qualifies the higher negative-X onset exception **before** the short-event skip for sustained reversals. The new 960/1920 regression fails before and passes after. `artifacts/flow-intent-v083-20260910/core-review/REVIEW.md` describes the exact defect.

**The old route LP, baseline repair and post-placement crossing-lift machinery is still in production `generation.cpp`.** The common owner below is the intended replacement, not a second layer to stack on top.

## Best terrain/source-owner experiment — integrate this first

Evidence root: `native/artifacts/flow-intent-v083-20260910/`.

The latest preserved route-worker source is `route-angle-study/terrain-v27/common5-source/`:

- `generation-common.cpp`
- `terrain_baseline.hpp`
- `fixed_transfer_intent.hpp`
- `prepare.py` and `prepare_common.py` document how the artifact was assembled. Do not blindly rerun prototype scripts over later files.

The shared source-height/C3 baseline solve replaces nearest-grid repairs, timing warps and post-solve crossing translations. It preserves one seeded third-turn preference and solves the other closure angles instead of absorbing missing displacement in long passive corridors. Source positions stay rigid except for one vertical translation per source. Station/departure/first signature/arrival share a bounded scalar height, not a hardcoded minimum. Sources, motor ports, ordinary gaps and actual nonlocal crossings enter the same solve.

The canyon climb/return have a route-owned, motion-certified active window. Preserve their exact rise and active start/length when tying them to the height solve; a different conservative fixed-motor interval cannot veto or reshape that already certified domain. Actual loss-aware motion and full finite-train/body checks still run afterward.

`common5-flat42`, `common5-hills2`, `common5-hills9`, `common5-canyon42` each contain accepted reports/saves with mandatory convergence:

| Case | Full duration |
|---|---:|
| flat42 | 125.642708 s |
| hills2 | 125.736458 s |
| hills9 | 126.225000 s |
| canyon42 | 147.930208 s |

`common5-canyon1` has no completed report at handoff. Do not count it as tested. Hills2's earlier real turn1/turn2 crossing and excessive transition were corrected in this experiment, not ignored. The last change allocates the ordinary baseline's rate budget after accounting for the authored turn's bank/hump contribution. Review it and its portable behavior before promotion.

The independent baseline component note/tests are `passive-energy/clearance-v24/PROMOTION.md`, `terrain_baseline_tests.cpp`, and `terrain-tests-numerical-final.log`: 38 checks pass. The route snapshot contains subsequent caller/intent integration. Preserve the squared QR-rank correction, exact normalized cell endpoint and cancellation checks. Clean artifact-only file output/instrumentation from a production patch; remove stale locals and the obsolete owners/tests only when actually replaced.

## Physically sized booster proposal — combine after the common owner

`propulsion-sizing/proposal/` contains `propulsion-sizing.patch`, `drive-profile-planning-tests.patch`, `generation.cpp`, `boost_planning.hpp`, and the test source. The generation patch targets `propulsion-sizing/v2/generation-baseline.cpp` (the **common4-era** source), not current production or common5. Rebase the small physical-work change onto common5; do not overwrite its later station/crossing/rate corrections.

The ordinary booster rating is 0.8g nominal; length integrates the existing force/power/governor law over kinetic work plus finite-train/ramp/fade space. The late motor gets its own measured inlet through the existing bounded energy feedback, rather than borrowing a turn's maximum speed. No new retry loop, hidden motor or instantaneous velocity change is introduced. Primary manufacturer references and the exact limits of their evidence are in `propulsion-sizing/REFERENCES.md`.

Actual accepted experimental results:

| Case | Length | Full duration |
|---|---:|---:|
| `v1-sized-flat42` | 5759.955 m | 118.885417 s |
| `v1-sized-flat7` | 5699.754 m | 117.179167 s |
| `v2/canyon42` | 6605.387 m | 136.436458 s |

Their saved replay evidence is retained in their own folders. Sixteen independent force/power/6-12-car operation cases pass at both mandatory rates. The older combined hills9 proposal **rejects** at 21.94g/s in the first ordinary turn; it must be retested on common5's corrected rate allocation. Do not promote a flat-only result as terrain-wide success.

The flat42 two motors shrink from about 400/400 m to 248/171 m. Actual outlet speeds remain within about 0.15 km/h of the matched control; the late near-1g tail falls from 2.93 to 0.92 s. Detailed measurements are in `propulsion-sizing/results.json` and its actual viewed `review.png`. Later canyon evidence lives separately under `v2/`.

## Actual visuals and remaining ride-quality issues

- `editor-v27-load-retained-flat42-front/REVIEW.md`: actual full 130.944 s editor-game traversal; eight of 27 captures reviewed, including curved Immelmann rolling descent.
- `editor-v27-load-common4-canyon42-front/REVIEW.md`: actual full 147.930 s traversal; eight of 30 captures reviewed, including cliff climb and return.
- Both are **load-only** experiments: pause/restart/pose/load/refusal/save-cancel passed, ordinary save and keyboard input were not tested there. No new FPS or packaged claim.
- Sparse perimeter routing persists in the reviewed examples. Some seeds have genuine crossings, but do not weaken the geometry/crossover regression merely to make a sparse family green. Decide whether a selected fixture should exercise an actual crossing rather than requiring one on every seed; retain meaningful macro-variation coverage.
- Canyon common4 climb reaches 141.2 m above local ground in an inspected frame, and return 158.3 m as terrain falls away. Compare feasible site/window choices against actual easing requirements and supported heights; numerical acceptance alone is not convincing terrain use.
- Fixed motor flats are materially improved only in the newer propulsion proposal. Review the combined result after integration, not the older reviewed controls.

## Validation and next bounded workflow

Latest main-source component checks: FVD/layout suites pass, frame-force 3,569 checks pass, drive/source-port/station 6,055 checks pass, and the final force-envelope/convergence CTest pair passes 2/2 in 1.96 s. `git diff --check` passed. Earlier full native and UE integration failures remain documented in `VALIDATION.md`; no later all-green run is implied.

Obsolete source solvers/overlays were removed, but this complete checkpoint has a net line-count increase, including the new force assessment and tests. Do not describe it as overall code deflation. The proposed common owner removes further repairs only after it is actually integrated.

1. Inspect the checkpoint branch/CI receipt, then build a clean common5 + propulsion integration. Preserve selected targets, exact-version saves and mandatory 960/1920 acceptance.
2. Run meaningful baseline/force/drive/source regressions and representative flat/hills/canyon saved replays, including the older failed hills9 and unfinished canyon1. Resolve actual failures; do not run another unchanged 1,000-request audit.
3. Complete native/portable checks and GitHub CI before merging the engineering branch to main. Root source and experimental source are currently different; report exact hashes.
4. Compile UE/UHT, run all UE contracts, prepare/cook/package a fresh Windows build, actually review representative generated POV and ordinary save/load, then measure 1440p performance with engineering jobs stopped. Update launcher only after verifying the new executable and distribution hashes.
5. Continue genuine benchmark curation and Mac runtime work separately. Do not call the whole foundation complete on numerical tests alone.

## References and external prerequisites

Real RFDB traces/graphs and the phase atlas are under `references/element-profiles/ATLAS.md`; original supplied data is in the repository's `RFDB Data` directory. FF4804 has row7/left/Apple Watch metadata; TRR6383 row2/seat8/train1/iPhone. Additional Pantherian row16 recordings are preserved. They anchor phase character and descriptive comparisons; no matched speed channel or exact POV clock is established. Raw/calibration/mount/configuration qualification is still unresolved for the strict benchmark. Do not fabricate eligibility or substitute a synthetic number in ALL RECORDS.

UE5.8.2, VS2022 and Windows SDK are installed on D:. Mac core CI is available, but a real Mac/Metal package remains unverified. The current full F2291-26 clauses and physical restraint/structure/control qualification are not established by the historical force profile.

## Useful commands / paths

- Explicit workdir: `D:/Coding/Codex/Vibecoasterjs` (the ambient Codex C: workspace is unrelated).
- Git: `git -c safe.directory=D:/Coding/Codex/Vibecoasterjs -c core.safecrlf=false ...`.
- CMake/CTest: `native/.tools/cmake-package/cmake/data/bin/`; build `native/artifacts/generator-intent-v081/build`, Release. Targets include `coaster_cli`, `force_envelope_tests`, `drive_profile_tests`, `convergence_tests`, `fvd_tests`.
- Python: `D:/Games/Epic Games/UE_5.8/Engine/Binaries/ThirdParty/Python3/Win64/python.exe -B`; plotting libraries via `PYTHONPATH=D:/Coding/Codex/Vibecoasterjs/native/.tools/python311`.
- VS: `D:/Toolchains/VS2022`; SDK: `D:/Toolchains/WindowsKits`. Isolated CMake uses VS17 2022 x64, toolset14.38, SDK10.0.22621.0.
- UE packaging: `native/unreal/scripts/package.ps1 -UnrealRoot 'D:/Games/Epic Games/UE_5.8'`. Do not use SkipAutomation for acceptance. Latest renderer DLL was v27, SHA256 `0EBF9B1AA7DF115FCEA1244BC2500B93A7497B5EDA1247F2E01437632EA25A62`.
- Artifact patches need `git apply --check --ignore-space-change` because this repository's existing CRLF policy otherwise rejects matching context. Inspect dependencies before applying.
- Parse report JSON compactly. Do not print huge single-line run logs. Keep temporary files, builds, caches and snapshots in suitable project/D: locations, not D:'s root or the games directory.
