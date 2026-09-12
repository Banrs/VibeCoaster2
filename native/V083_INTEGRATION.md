# v083 integration qualification

The universal source-itinerary implementation is committed as `4906de08c7c46289e5615428450d8294dec43493` on `codex/flow-v083-checkpoint`. Its source48 follow-up removes unused transfer machinery, propagates the existing numeric policy to CMake consumers, and fixes duplicated UE backdrop vertices. Source48 passes local physical and rendering gates. Promotion remains held for the user's quiet-running design review and final-commit three-platform CI. This is not a merge or package receipt. [Live progress](V083_PROGRESS.md) records the current work.

Identity remains **0.8.3-flow.1 / COASTER5**. Public generation APIs, CLI arguments, save format, selected targets, force/power limits, eight complete-ride builds, 0.5 m/s feedback tolerance and mandatory 960/1920 Hz acceptance are retained. [Architecture and physical owners](GENERATOR_ARCHITECTURE.md), [whole-codebase audit](SIMPLICITY_AUDIT.md), [progress](V083_PROGRESS.md).

All evidence paths below are relative to `artifacts/flow-intent-v083-20260910/integration-20260911/`. Frozen sources, failed attempts, interrupted/cancelled ledgers and unrelated art remain preserved.

## Source48 follow-up qualification

- Complete fresh native suite: **30/30**, 1,069.16 s (`full-suite-48/ctest.log`). Active component contracts include 53 baseline, 5,384 circuit and 6,115 drive checks. Removed checks exercised the deleted alternate transfer/window-placement path; rigid climb/dive shape and active-window coverage remain.
- Required eight plus four regressions: **12/12** candidate-zero acceptance, exact replay and independent 960/1920 Hz audits (`itinerary-final-48/results.json`). Saves, plans and traces are all byte-identical to source47 (`source47-parity.json`). All eight independent organic audits pass with the warning-free strict-arithmetic audit executable (`organic-final-48/results.jsonl`). The earlier audit build missing exception-unwind flags is preserved separately.
- Fixed generality matrix: **44/44**, no substitution (`architecture-matrix-48/results.json`). Fresh portable contracts and changed canonical translation units pass (`portable-48/`). Python tooling remains unchanged from its complete source47 run; actual CLI tests pass again in source48 CTest.
- Final real UE 5.8 editor automation: **10/10**, comprising eight permanent contracts and two evidence contracts (`terrain-indexing/final-automation.stdout.log`). All eight saved rides prepare successfully. The new canyon regression independently reproduces both old budget failures; final canyon1/canyon24 payloads are 1,957,796/1,956,264 vertices under the unchanged 2,000,000 limit. Fifteen exact expanded-triangle comparisons preserve positions, normals, UVs and winding. Earlier failing controls and the superseded cancellation assertion remain preserved. NullRHI preparation timings are diagnostic, not FPS or packaged-performance measurements.
- `review-48/core-source-parity.json` binds all 70 manifested source/test/build/audit files to root. `ue-source-parity.json` binds all 35 maintained UE source and packaging-script files to the tested snapshot. Evidence-only comparison code is absent from production.

| Source48 artifact | SHA-256 |
|---|---|
| Panel manifest | `cef4386ff1f46eefd1c121055bbcf0b5bf653f7cbcad807412965735258e1d47` |
| Matrix manifest | `0c6de44bfa219b25313f8ca34716c4aec52df9be1f493b53d76c7c883034b52c` |
| Panel CLI | `eda3014882b3df0e0e75cd27c97f98d8e0ffadaffa5c9c4c56547ac6717a9bbb` |
| Convergence auditor | `2d71a82e7ff8201f168bc062f889b5eb71f739b5dd24b62400000279e72d15f8` |
| Organic auditor | `486b5a8063a5149919b677f896cb5c3e69b37f34a1d8396d76f8be7585634cea` |

The source47 CI run completed with Windows/Linux green and macOS 29/30. `review-47/MACOS_NUMERICS.md` isolates the failing turn-boundary arithmetic with contraction on/off; CMake now exports the already-required numeric flags rather than adjusting a geometry tolerance. Final-head CI remains required.

The original Falcon's Flight recording and the user's plots are being compared with actual saved geometry under `neutral-flow-review/`. Combined magnitude cannot identify signed quiet load by itself, and the user confirms the displayed angle scale is offset/inaccurate. This review has not introduced a new hard percentage or changed physical acceptance. The source47 tables below remain a separate historical checkpoint; the required save identities and ride measurements also apply to source48 because their bytes match.

## Local gates

| Gate | Actual result | Receipt |
|---|---|---|
| Complete native suite | 30/30 on one fresh frozen build: placement regression 1/1 in 152.41 s; remaining 29/29 in 972.65 s | `full-suite-47/ctest-placement.log`, `ctest-remaining.log` |
| Python tooling | 184 discovered; 176 passed, eight CLI skips; all eight actual CLI tests pass in CTest | `python-tooling-47.log`, native `cli_arguments` result |
| Portable wrappers | Project/PowerShell contracts, coordinates, canonical wrappers and changed generation/simulation/support translation units pass | `portable-47/` |
| Required saved rides | 8/8 candidate zero, selected targets, exact replay and independent full 160-metric convergence audit | `itinerary-feasible-site-47/results.json` and case receipts |
| Additional regression rides | flat11, hills11, canyon11, hills42: 4/4 with the same save/replay/audit gates | Same panel, separately named cases |
| Required organic audits | 8/8 independent saved-ride audits | `itinerary-feasible-site-47/organic-audit-results.json` |
| General request matrix | Running; all 44 predeclared requests must be reported without substitution | `architecture-matrix-47/` |
| Integration CI | macOS 29/30: circuit_layout fails; other jobs still running when recorded | [Windows/Linux/macOS run](https://github.com/Banrs/VibeCoaster2/actions/runs/34658595544) |

The independent baseline coverage is now 60 checks, including continuous clearance, rigid sources, C3 joins, shared station bounds, actual crossings, numerical rank, endpoint termination and cancellation. The 5,380 circuit checks include independent RK4 verification of source energy against actual closed lengths. That regression fails on the preserved old source and passes the current implementation (`source-route-regression-45`). Drive tests independently simulate six- and twelve-car trains, force- and power-limited motors, both mandatory rates, delivered outlet speed, bounded thrust and hardware clearance before passive sources.

The dedicated hills2 crossover has one transverse intersection between branches at least 1,000 m apart along the canonical route, with 48.6135 m vertical separation and full clearance acceptance. It retains four crest shapes and 16.4777% straight-flat share. Every required fixture passes the unchanged under-35% share and independent source/operation/crest checks; a crossover is required specifically for hills2.

## Required ride measurements and save identities

Moving time ends when the train centre reaches the terminal station operation; complete-stop time includes terminal stopping. Exact replay excludes only the nondeterministic `generationSeconds` field.

| Ride | Moving seconds | Complete-stop seconds | Save SHA-256 |
|---|---:|---:|---|
| flat5 | 173.468 | 184.704 | `68e29b500ecd1ca6c8990de97df6965b1fb4408c629ce6213d9d35aab7c482bb` |
| flat7 | 185.464 | 196.923 | `4346c8cde0616de42793dbf7b1471d72b3dc358a8276ad2f29f2da3cbb400d56` |
| flat42 | 185.226 | 196.825 | `736f31b4dbedfb95df2781aef7501452277167f720e1b1bb46bcd3d1890bfec3` |
| hills2 | 192.264 | 203.611 | `405234a1836dd01af6f10e4e14ce5e782dfb0334269bc9b77fa12063a2cc95f5` |
| hills9 | 186.071 | 197.805 | `062089ccdc46907c4f2c33d8bd1e5598f1e1ceb7fb0aa0861ff61d06dc7f7ad7` |
| canyon1 | 189.606 | 202.067 | `c1d2924dba4284e5b21c195e178fe3f5e01e797a52a13063d0a9f6e1ff53430e` |
| canyon24 | 170.059 | 181.875 | `6fe87c5505a387eb8b700c23b28645ab29ecafcfce5d488134e1e65af3e92880` |
| canyon42 | 182.530 | 194.048 | `eeeea472f00dc5684edd26bb17b91617c98380fb0929ed9d79a951a4178b7cc6` |

Six of eight exceed the existing soft 180-second moving-time goal. Their warnings remain visible. This milestone does not claim prototype-matched durations, whole-ride feel or a new performance result.

These eight rides converge in four or five complete-ride builds. Their largest reported feedback residual is 0.470781821664 m/s (hills2), below the unchanged 0.5 m/s threshold. Motor inlets, occupied ordinary-turn speeds and force-authored passive source phases are measured in the same bounded process; the certified powered climb/dive retain their separate operating windows and rise constraints.

## Generality panel

The fixed matrix comprises seeds 0, 3, 11, 19, 73 and 101 on flat/hills/canyon with both six- and twelve-car trains (36 requests); seed37 across all three terrains under two translated/rotated frames, with one frame also scaled (six requests); flat23 with height240/speed80 and hills31/twelve cars with height250/speed85 (two requests). The matrix runner and its exact request definitions are frozen in its manifest. No failed request is replaced or omitted.

The matrix is a deliberately varied regression panel, not proof that every supported input is physically feasible. Infeasible requests retain explicit rejection. The generator uses the same algorithm for every seed and constructs one candidate.

## Preserved control comparison

`control-comparison-47/results.json` includes all motors in common5 flat42/hills9/canyon42, the preserved sized-booster proposals, and the three current saves. The rejected historical sized hills9 proposal remains explicitly rejected. The earlier architectures have different element ordering, powered roles and target speeds; these are descriptive comparisons, not matched experiments.

| Ride | common5 complete-stop seconds | Sized proposal seconds | Current seconds |
|---|---:|---:|---:|
| flat42 | 125.643 | 118.885 | 196.825 |
| hills9 | 126.225 | Rejected | 197.805 |
| canyon42 | 147.930 | 136.436 | 194.048 |

Current flat42's installed ordinary motors are 78.929 m before the Immelmann, 167.616 m before the giant camelback and 83.086 m before the loop. Their measured inlet/outlet speeds are respectively 52.257/54.325, 67.768/74.907 and 43.286/48.063 m/s. The powered climb is separately 616.877 m, with 41.210/41.923 m/s at its full-train occupancy boundaries; it performs gravitational work despite similar inlet and outlet speeds.

The current late loop boosters measure 60.888–90.719 m across those three controls, with 0.850–0.883 s of longest quiet powered exposure, versus approximately 383 m and 0.583–3.000 s for the old late airtime boosters. Their physical roles and delivered speeds differ. Quiet exposure is a descriptive 60 Hz trace measurement using `|middle Gx| < 0.15` and `|middle Gz - 1| < 0.12` while any car occupies the motor. It does not replace 960/1920 Hz force acceptance.

## Source and executable binding

`review-47/source-parity.json` verifies every active production source/header against the frozen `full-suite-47/source/native` tree. The fixture and matrix manifests include individual source and executable hashes; each accepted save has its own result receipt and independent replay/audit output.

| Artifact | SHA-256 |
|---|---|
| Required panel manifest | `a8a5f95ddd540903b4bded3ba25dafe95e000ce75075b1318a2238de91288adf` |
| Matrix manifest | `b4dc7e4ae6f2b5cdd4c79358511851a34c782fa9f68347477fd5aa3a5a82304b` |
| Required panel CLI | `6682b025b9556a2097033de2dcef387b1f0e47de86c8a8c9f9feef0444622b1e` |
| Required panel convergence auditor | `1646c83b63ce4bd9afa82521e8b7da1eeea33f70337df25d4b2f2ad485145d62` |
| Saved organic auditor | `b070b5dd5dc3c64200c7dcce2d48915bc6e376d36b9339598bf9eb5e2e27be97` |

Source46's unconditional site reranking was rejected by canyon42. Source47 retains a feasible previous site while allowing the existing alternatives if current physical constraints invalidate it. The paired native regression covers that distinction. The earlier matrix33/44 interruptions and matrix46 supersession have explicit ledgers; none is relabelled as a complete pass.

## Promotion scope

Final documentation, the complete matrix, green CI for the final integration commit, merge to main and green main CI remain required. Windows packaging, benchmark qualification, Unreal packaging, POV review, performance measurement and Mac/Metal runtime verification remain later milestones.
