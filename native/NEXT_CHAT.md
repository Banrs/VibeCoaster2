# Source64 integration record

**Every required local gate passes on source64.** Read [qualification](V083_INTEGRATION.md), [progress](V083_PROGRESS.md), and the [promotion record](artifacts/flow-intent-v083-20260910/integration-20260911/promotion-64/) for the integration/main commit status. `complete.json` is written only after final integration CI, merge and main CI succeed; GitHub checks remain authoritative.

- Repository: `D:\Coding\Codex\Vibecoasterjs`; integration branch: `codex/flow-v083-checkpoint`.
- Frozen final source/build: `native/artifacts/flow-intent-v083-20260910/integration-20260911/qualification-64/`.
- Identity: **0.8.3-flow.1 / COASTER5**. Public headers, CLI and test registration remain unchanged from source59. Earlier experimental FVD interfaces differ from historical main.
- Read [AGENTS.md](../AGENTS.md) and [architecture](GENERATOR_ARCHITECTURE.md). Normal independent subagent use is authorized.

## Verified behavior

One source itinerary owns routing, terrain placement and hardware. The fixed progression remains **cliff ascent → summit turns → dive → fastest launch → giant camelback**; other sources use physically feasible positions around it. Passive feedback corrects local work without counting upstream motor error twice. Turn capacity covers required and observed source speed. Site-independent placement calculations are shared within each build. UE reference availability uses the core validator; the obsolete post-generation missing-reference branch is removed.

Source64 passes all **36 CTest suites**, **44 unchanged requests**, the **eight required saved-ride/organic audits**, Python/portable-wrapper checks and **ten real UE contracts**. Each ride accepts candidate zero and exactly replays with independent **960/1920 Hz** audits. Hills2 retains its explicit private crossover and full clearance. All eight saves match source63 byte-for-byte.

The consolidated verifier checks raw results, requests, copied executables, 168 maintained inputs, 65 binaries, external UE compiler dependencies and the control comparison. Twenty-five runs retain the rejected historical control; all 40 matched positive motors keep their force/power ratings. Motor/quiet-tail/duration measurements are descriptive.

Graphify **0.9.51** is registered with eight references. The code-only index covers 125 files, including C++ headers/implementations. Five representative relationships were source-verified; three declaration-header parser limitations remain documented. The graph guides inspection and does not prove correctness.

## Evidence

Paths below are under `native/artifacts/flow-intent-v083-20260910/integration-20260911/`.

| Purpose | Location |
|---|---|
| Final source, builds and consolidated verification | `qualification-64/provenance.json`, `verified-local-gates.json` |
| Eight saved rides and fixed request matrix | `itinerary-final-64/`, `architecture-matrix-64/` |
| Every positive motor and preserved controls | `control-comparison-64/` |
| Workflow, graph and independent review | `continuation-20260912/` |
| Reproduced energy/turn failures | `case38-diagnosis-62/` |
| Integration/main SHAs and CI receipts | `promotion-64/` |
| Historical source59 checkpoint | `handoff-20260912-source59/` |

## Continuing safely

1. Inspect the working tree, promotion receipts and GitHub checks. Preserve unrelated art and frozen evidence. If promotion is unfinished, finish final-commit three-platform CI before merging, then verify main CI.
2. Any subsequent source correction needs a rebuilt, consistently hashed qualification. Retain all 36 suites, all 44 unchanged requests, the required eight rides, ten UE contracts and independent audits. Never substitute historical passes or seeds.
3. Preserve selected targets, physical limits, eight whole-ride builds, 0.5 m/s feedback and exact-version saves. The existing panel/matrix drivers accept frozen source/build paths and require fresh output names.
4. Follow the user's next scope. Packaging, benchmark qualification, POV review, FPS/load measurement and Mac/Metal runtime verification are outside this milestone.

Preserve `native/unreal/Content/Art/V072/Conventional1/`. Evidence UE `Content` directories can be junctions to real content; do not recursively move/delete them.

Tool paths and focused/full commands are in [BUILDING.md](tools/BUILDING.md). VS uses `Visual Studio 17 2022`, x64, `v143,version=14.38`, instance `D:/Toolchains/VS2022`, SDK `10.0.22621.0`. Use UE Python with `-B`; plotting dependencies are in `native/.tools/python311`.

Preserve the invalid internal Codex ref. Use command-local `git -c maintenance.auto=false`; if needed, the established fetch workaround adds `-c transfer.hideRefs=refs/codex -c fetch.negotiationAlgorithm=noop fetch --refetch --no-auto-maintenance` with explicit refs.
