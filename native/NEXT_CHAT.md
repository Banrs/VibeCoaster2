# Start here: v083 checkpoint

Work stopped at the user's request on 12 September 2026. This is a tested development checkpoint, **not a completed integration or release**. Resume only in the next session.

- Repository: `D:\Coding\Codex\Vibecoasterjs`
- Branch: `codex/flow-v083-checkpoint`; the latest commit is the handoff checkpoint.
- Current source/build snapshot: **source59**, `validation-speed-59`.
- Identity: **0.8.3-flow.1 / COASTER5**. Main has not been merged.
- Read [AGENTS.md](../AGENTS.md), then [architecture](GENERATOR_ARCHITECTURE.md) and [current qualification](V083_INTEGRATION.md). [Progress](V083_PROGRESS.md) is the short checklist.
- The [handoff directory](artifacts/flow-intent-v083-20260910/integration-20260911/handoff-20260912-source59/) contains source/binary/save hashes, the checkpoint receipt and the preserved pre-cleanup documents.

## What is implemented

One source itinerary supplies routing, terrain placement and emission. Rigid source heights, C3 connectors, station datum, motor work and crossing separation share the terrain solve. Actual inlet-based booster sizing replaces unused zero-speed reservations; complete closed-route energy feasibility precedes order selection. Ordinary boosters retain their nominal 0.8g rating within existing force/power limits.

Only **cliff ascent → summit turns → dive → fastest launch → giant camelback** has fixed order. The loop, Immelmann and supporting airtime are placed around that block by physical feasibility. The same supported climb/dive applies on flat ground. No seed-specific generator branch is allowed.

The dedicated hills2 fixture requests a crossover through private `detail::generateRide(request, true)`; ordinary public generation may be uncrossed. Its independent audit requires a transverse crossing of nonlocal canonical branches and full clearance. This is an explicit fixture brief, not a hidden seed rule.

Source57 removes repeated airtime construction by reusing successful results for the exact chain/speed within one route solve. No order is omitted, speed rounded or global cache introduced. CMake now exposes focused component/organic checks. CI runs components before integration and stops scheduling after a failure. Portable builds compile the core once.

The latest support corrections:
- Continuous steel terrain bounds separate horizontal travel from vertical extent. This fixes false footing failures without changing geometry/clearance tolerances.
- The existing 600 m tower family can emit 620 members, contradicting the old independent 512 cap. Construction and the per-support resource bound now share the 600 m/16 m tier dimensions. The total 60,000-member budget and all physical limits remain unchanged.

## Exact stopping point

Source59 builds all 28 portable executables in 128.05 seconds. Its focused support suite passes **30,259 checks**.

The previously failing **canyon0 / 12-car** request now accepts candidate zero, saves, exactly replays and passes the independent 960/1920 Hz audit. Duration: 165.30520833371247 s. All **4,969 canonical knots are text-identical** to the rejected witness: the resource correction did not change its track.

Save SHA256: `a74c395051f3eb3692eea1f4afc34b84dc40b51d8c8a34f2d67d17bf27183b4b`.

Full source59 qualification has **not** run. Do not treat earlier green builds as qualifying it.

## Resume in this order

1. Inspect `git status`, the latest checkpoint commit and its GitHub CI. Preserve unrelated art. Do not merge this checkpoint.
2. Use focused checks for any failure; the current support reproducer takes seconds. Complete the final native/CLI suite after affected checks pass. CMake has 36 registered tests; use `-L component` before `-L integration`, two workers and `--stop-on-failure`.
3. Run the complete **44 predeclared requests** on final source in fresh evidence. Source57 stopped after 11 completed cases: 10 passed, canyon0/12 failed the resource cap. Two active cases were interrupted. Source59 fixes that failure but does not yet qualify the full matrix.
4. Generate and independently save/replay/audit **flat5, flat7, flat42, hills2, hills9, canyon1, canyon24, canyon42** on final source. All require candidate zero, selected targets and 960/1920 Hz agreement. Hills2 uses the explicit internal crossover driver. Retain organic crests, variation, terrain adaptation and <35% straight-flat share.
5. Complete Python tooling, portable wrapper checks and relevant real UE contracts on final code/saves. Compare motor lengths/speeds, quiet tails and duration with preserved controls. Update qualification with actual hashes/results.
6. Review the final diff, commit/push coherent changes and obtain green final-commit Windows/Linux/macOS CI. **Only then merge to main and verify main CI.** Any unresolved required failure blocks merge.

Do not restart architecture experiments or blindly replay the earlier prototype generators. The retained source59 snapshot is the starting point. The supplied Falcon angle scale is inaccurate; its shape is qualitative evidence, not a new grade or quiet-time threshold.

## Where things are

| Purpose | Location |
|---|---|
| Production C++ | `native/core/src`, `native/core/include` |
| Permanent tests | `native/core/tests` |
| Unreal integration | `native/unreal/Source` |
| Build/test instructions | [tools/BUILDING.md](tools/BUILDING.md) |
| Current frozen source and portable binaries | [validation-speed-59](artifacts/flow-intent-v083-20260910/integration-20260911/validation-speed-59/) |
| Fixed failure, accepted save and exact knot parity | [support-budget-59-case5](artifacts/flow-intent-v083-20260910/integration-20260911/support-budget-59-case5/) |
| Fast rejected-track witness and explanation | [support-diagnosis-58/README.md](artifacts/flow-intent-v083-20260910/integration-20260911/support-diagnosis-58/README.md) |
| Eight accepted source57 controls | [itinerary-final-57](artifacts/flow-intent-v083-20260910/integration-20260911/itinerary-final-57/) |
| Stopped source57 matrix | [architecture-matrix-57/interruption.json](artifacts/flow-intent-v083-20260910/integration-20260911/architecture-matrix-57/interruption.json) |
| Predeclared matrix definition | [architecture-matrix.md](artifacts/flow-intent-v083-20260910/integration-20260911/architecture-matrix.md) |

The existing evidence drivers are `run_panel_57.py` and `matrix-runner-57/run.py` under that integration evidence directory. Both accept a frozen native source/build path; use a fresh output name. The panel also needs the evidence build containing `crossover_fixture.exe` and `replay_organic.exe`. The matrix retains all 44 definitions and two workers.

## Practical constraints

- Follow AGENTS.md's **Think Before Coding / Simplicity First** verbatim. Resolve shared constraints; do not tune seeds, add compensating retries or weaken physical acceptance.
- Keep eight whole-ride builds, 0.5 m/s feedback, force/power limits, exact-version saves and mandatory rates. Use Codex's edit-file tool.
- Work alone. The requested logic/skeptic, simplicity and performance audits are complete or stopped; their reports remain preserved. Do not restart agents or poll them.
- Preserve failed/frozen evidence and `native/unreal/Content/Art/V072/Conventional1/`. Some evidence UE `Content` folders are junctions to real content. Do not recursively move/delete them.
- Installed tools: Zig `native/.tools/zig-x86_64-windows-0.14.1/zig.exe`; CMake/CTest `native/.tools/cmake-package/cmake/data/bin/`; UE Python `D:/Games/Epic Games/UE_5.8/Engine/Binaries/ThirdParty/Python3/Win64/python.exe`. Use `-B`; plotting packages are in `native/.tools/python311`.
- VS CMake configuration uses `Visual Studio 17 2022`, x64, `v143,version=14.38`, instance `D:/Toolchains/VS2022`, SDK `10.0.22621.0`. Installed toolchains may need sandbox escalation.
- A preserved invalid internal Codex git ref can break automatic maintenance. Use command-local `git -c maintenance.auto=false`. If fetch needs repair, the established non-destructive command uses `-c transfer.hideRefs=refs/codex -c fetch.negotiationAlgorithm=noop fetch --refetch --no-auto-maintenance` with explicit main/checkpoint refs.
- Packaging, benchmark qualification, POV review, measured FPS/load benchmarking and Mac/Metal runtime verification remain outside this milestone.
