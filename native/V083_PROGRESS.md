# v083 integration progress

**Goal:** a simple organic generator that passes the unchanged physical and delivery gates, then merges to `main`.

**Current step: 4 of 7 — physical qualification passes; quiet-running design review.** Updated 12 September 2026, 10:43 Sydney. This is a sequence of gates, not a time estimate.

Source48 passes all 30 native suites in one fresh run (1,069.16 s), all 44 predeclared requests, the required eight plus four regression saves/replays, and all eight independent organic audits. All 12 saved rides, plans and traces are byte-identical to source47. The final real UE run passes eight permanent contracts plus two independent evidence contracts: all required saves fit the unchanged renderer budget, and indexed backdrop triangles exactly preserve the old positions, normals, UVs and winding. Fresh portable checks pass. Receipts and source binding are in `review-48/`.

The latest user-supplied Falcon's Flight plots motivate a separate design review before promotion. Their angle scale is offset/inaccurate and will not become a grade constraint. `neutral-flow-review/` measures signed loads, magnitude, actual canonical pitch/bank, operation windows and quiet intervals from the original recording and all eight saves. No screenshot-derived percentage has become an acceptance threshold, and no limit or geometry has been tuned in response. Some generated quiet intervals approach ten seconds; their physical ownership is under review.

The previous commit's Windows and Linux CI pass; macOS fails one header-arithmetic component. A minimal reproducer isolates floating-point contraction, and source48 propagates the existing numeric policy to every CMake consumer. Final-commit three-platform CI and main promotion remain pending. The table below preserves the earlier source47 checkpoint rather than relabelling those receipts as source48.

| Step | State | Evidence |
|---|---|---|
| 1. Establish requirements and audit | Complete | AGENTS.md governs implementation; Falcon signature progression agreed; entire maintained codebase audited in SIMPLICITY_AUDIT.md. |
| 2. Implement one coherent generator | Implemented; broad validation running | Root source47 has one itinerary, consistent source/closed-route energy, sparse shared heights/crossings, finite-train source qualification and physical motor sizing. No seed-specific rule or whole-ride retry. |
| 3. Verify independent components | Focused checks pass | Source45 passes60 baseline checks and5,380 circuit checks. The closed-route regression fails on the preserved old source and passes current source. Existing6,115 drive and183 force checks also pass. |
| 4. Validate complete rides and generality | Required panel passes; full matrix running | Source47 accepts all eight required rides and all four added regressions on candidate zero, with exact saved replay and independent 960/1920 Hz audits. The full 44-request matrix is running without substitutions. Its native placement regression passes both higher-target twelve-car reselection and canyon42 continuation. A previous site is a preference checked against current constraints. Source46's unconditional reranking is rejected and preserved; its panel was 11/12, and superseded bulk runs have explicit cancellation ledgers. |
| 5. Complete local checks | Complete | Source47 passes all 30 native suites on one frozen build: paired placement regression in 152.41s, remaining 29 in 972.65s. Python: 184 discovered, 176 passed, eight CLI skips; all eight actual CLI tests pass in CTest. Fresh portable coordinate/wrapper and changed translation-unit checks pass. |
| 6. Review, record and qualify commit | Implementation pushed; CI and requested reviews running | Reviewed implementation `4906de0` is pushed. Frozen sources, executable/save hashes, control comparisons and failed evidence are retained in V083_INTEGRATION.md and its receipts. Windows/Linux/macOS CI is running. The newly requested logic/skeptic, simplicity and UE performance agents are reviewing independently, with no routine coordination polling. Final documentation and final-commit qualification remain required. |
| 7. Merge and verify main | Pending | Merge only after every required gate passes, then verify main CI. |

The requested reviews found two corrections before promotion. The simplicity agent is removing unused transfer-domain/window-placement machinery while retaining the active physical contracts. The UE review proved that duplicated backdrop vertices make canyon1/canyon24 exceed the unchanged 2,000,000-vertex budget; an isolated real UE test reproduced both failures. Indexed strips are implemented and undergoing exact old/new triangle comparison and complete required-save preparation checks. The logic/skeptic review found no actionable numerical acceptance defect.

MacOS CI for `4906de0` is 29/30: `circuit_layout` reports "Turn bank ramps exceed the requested heading". Floating-point option propagation is under focused diagnosis. This failure blocks merge regardless of the passing local suite and matrix receipts. All findings and failed controls remain preserved under `review-47/` and `terrain-indexing/`.

## Current evidence

- `itinerary-feasible-site-47/results.json`: all 12 requests accept on candidate zero, with exact replay and independent audits. Required saves and their hashes are separately recorded. `full-suite-47/` and `architecture-matrix-47/` qualify the same frozen production source; `review-47/source-parity.json` verifies all 38 production source/header files against root.
- `portable-47/`: fresh coordinate, canonical wrapper and changed generation/simulation/support translation-unit checks pass. `control-comparison-47/results.json` reports every installed booster in the selected controls and current source, including the rejected historical hills9 proposal.
- `itinerary-feasible-site-47/organic-audit-results.json`: all eight required saves pass the independent organic audit. Hills2 has one distant transverse crossing, 48.6135 m canonical separation, four crest shapes and 16.4777% straight-flat share. `python-tooling-47.log` records the complete tooling run.

The following receipts remain useful earlier checkpoints and are not combined into a final source47 pass:

- `itinerary-closed-energy-44/results.json`: all12 requests accept, with exact replay and independent audits. The required eight are unchanged; the four extra regressions are separately identified.
- `itinerary-closed-energy-44/organic-audit-results.json`: eight independent organic passes. Hills2 has one distant transverse crossing,48.6135m minimum canonical separation, four crest shapes and16.4777% straight-flat share.
- `source-route-consistency-44/components-energy-contract.log`:5,378 independent circuit checks.
- `full-suite-44/`:29/29 pass. `architecture-matrix-44/interruption-ledger.json`:21 completed passes and every unreported case identified; no matching process remained at08:50. No partial matrix is a full-pass claim.
- `python-tooling-42.log`:184 discovered,176 passed,8 CLI skips. All eight actual CLI checks passed in the complete native snapshot42 run.
- `portable-44/`: fresh coordinate, canonical wrapper and changed translation-unit checks pass.
- `control-comparison-44/results.json`: every installed booster in the selected common5, propulsion and current controls is reported, including the rejected historical hills9 proposal. Roles and targets differ; durations are measured rather than matched to prototypes.

All paths above are under `artifacts/flow-intent-v083-20260910/integration-20260911/`. Implementation `4906de0` is pushed and [CI is running](https://github.com/Banrs/VibeCoaster2/actions/runs/34658595544). Final qualification and merge remain pending. Identity remains `0.8.3-flow.1 / COASTER5`.

## Resolved physical causes

Finite-train source energy replaces point-mass phase-speed references. Long-train inversion pulses are qualified by the shared simulator and unchanged force evaluator before placement. Signed connector-load bounds correctly distinguish horizontal turning from negative rider load. Support foundations use the existing depth and clearance limits, and blocked towers derive an outreach direction from the actual obstructing branch.

The remaining seed11 crossing failure was independently proven infeasible in all four over/under combinations. Its source speeds were selected before closure changed passive lengths, demanding about31m of compensating descent. Source44 resolves source intent and actual closed lengths together before freezing them; all three seed11 terrains now pass without changing limits. Exact equations, HiGHS witnesses and the rejected source remain in `solver-diagnosis-43/`.

Earlier sparse projection, physical sample boundaries and isolated force-peak time interpolation also have independent witnesses and regressions. Their failed attempts and original component receipts remain preserved. The obsolete dense matrix33 was explicitly cancelled after matrix35 completed all44 requests; its cancellation ledger reports completed, timed-out, cancelled and unrun cases without inference.

## Working rules

- Think before coding; correct conflicting physical owners instead of stacking clamps or fallback paths.
- Keep the same algorithm for every seed. No substitutions in the44-request matrix.
- Preserve eight complete-ride geometry builds,0.5m/s agreement, selected targets, force/power limits, full clearance and960/1920Hz acceptance.
- Use Codex's edit tool. Preserve failed evidence, frozen controls and unrelated art.
- Keep the completed audit agent autonomous; no repeated coordination polling.
- An unresolved required failure blocks merge. A wake-up deadline does not replace verification.

Packaging, benchmark qualification, POV review, performance benchmarking and Mac/Metal runtime verification remain later milestones.
