# Fixed native qualification

The native workflow qualifies the unchanged 44 matrix requests and eight saved
rides for `0.8.3-flow.1 / COASTER5`. `baseline.json` retains each complete request
and accepted save SHA256 from source64 (`d68acb66f77016aa9531cac58175e8c380e24f7b`).
Its driver hashes bind the three unchanged C++ evidence drivers promoted from
the preserved matrix and panel runs. Changing the baseline to accommodate a
failure is not a fix.

Native simulations run on CI under the current user instruction. Windows and
macOS each run all 36 CTest suites in Release, components before integration,
with two workers and stop-on-failure. The additional Windows qualification build
uses Zig 0.14.1, builds the core archive once and links its tests and drivers to
that archive. The same 36 tests run against those portable executables, including
the eight real CLI contracts. Portable Unreal wrapper checks are also required.

Four jobs then consume that single binary artifact. Fixed case indices modulo
four assign 13 cases to each job, with two case workers per job. No request
filters, substitutions or acceptance overrides are offered. Generation keeps its
600-second timeout; matrix replay/audit keep 120 seconds and panel stages keep
600 seconds. Simulation rates, numerical flags and acceptance limits are unchanged.

Each case requires candidate zero, exact saved replay (excluding only timing),
the source64 save bytes, and independent 960/1920 Hz acceptance with all 160
metrics checked from raw data. Each panel ride additionally runs the permanent
organic geometry contracts. Hills2 uses its explicit private crossover fixture.
The authoring energy checks retain the eight-build bound and 0.5 m/s tolerance.

The workflow uploads each shard's raw files even on failure, plus copied source,
executables, command receipts and hashes. `qualification-verify` requires all 52
cases and four shards to share the same source, compiler and binaries. The
verifier checks copied files and recorded path bindings without launching native
processes or requiring the original CI directories to exist.

To independently verify downloaded artifacts offline:

```powershell
gh run download <run-id> --pattern 'qualification-shard-*' --dir <fresh-directory>
python -B native/tools/qualification/run.py verify --input <fresh-directory>
```

The summary records actual case/stage times; these are descriptive measurements,
not benchmark qualification. This workflow does not replace the ten real Unreal
contracts, motor/control comparison, final integration/main CI or the complete
milestone evidence review. Local Unreal work is currently paused at the user's
request. Packaging, POV, FPS/load and Mac/Metal runtime remain outside scope.

Compiler archive identity is pinned to the [official Zig download index](https://ziglang.org/download/index.json).
The build helper uses an installed compiler and never downloads a toolchain.
