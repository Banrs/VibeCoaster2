# Build the native game and numerical tools

Current source: application and geometry/save identity 0.8.3-flow.1 / COASTER5.
The published Windows package is an earlier release; this checkpoint has not
been packaged. Mac/Metal runtime verification still needs a suitable Mac.

## Current source build

Use CMake and a C++20 compiler from the repository root:

```sh
cmake -S native -B native/build-cmake -DCMAKE_BUILD_TYPE=Release
cmake --build native/build-cmake --config Release --parallel 2
ctest --test-dir native/build-cmake -C Release --output-on-failure --parallel 2
```

During development, build the changed target and select the relevant regression
instead of regenerating every fixture. For example:

```sh
cmake --build native/build-cmake --config Release --target support_family_tests --parallel 2
ctest --test-dir native/build-cmake -C Release -R '^support_terrain_footprints$' --output-on-failure
```

`circuit_components` and `station_terrain_footprints` also avoid complete ride
generation. Organic integration is split into `organic_hills`, `organic_crossing`,
`organic_canyon` and `organic_variety`; the last keeps repeatability and variation
together without generating duplicate fixtures in separate tests.

Run `ctest ... -L component` for the fast physical contracts, then
`ctest ... -L integration` for complete generation and saved replay. CI requires
both groups on Windows and macOS. Linux remains a possible future target; its
tests and platform-specific work are not currently required. An unfiltered CTest
run still runs everything;
`ctest ... --rerun-failed --output-on-failure` repeats only failures from the last
run in that build directory. Add `--stop-on-failure` to stop scheduling work after
the first failure; CI uses this in both groups. A successful qualification still
runs every check. Never use a focused pass as full qualification.

The portable `native/tools/build.ps1` likewise compiles the core once into a
static library before linking the CLI and tests. Every invocation rebuilds the
objects with the current headers and flags; `-Test` still runs every full suite.

CMake also registers real CLI argument tests when Python is available. Historical rejection fixtures under
`core/tests/fixtures/historical/artifacts` are required committed test inputs.
The separate convergence CLI is built but does not run a matrix automatically.

Packaging is outside the current checkpoint. For a later UE build/cook/package,
follow [Unreal setup](../unreal/README.md). Imported
runtime art is committed; Blender is not required. The editor bootstrap creates
the map and seven base materials. Compilation is limited to two parallel actions
to bound memory use. The following is the earlier 0.8.2 distribution example;
do not label a new checkpoint build with that old version:

```powershell
& .\native\unreal\scripts\package.ps1 -UnrealRoot 'D:\Games\Epic Games\UE_5.8'
python native/tools/distribute_windows.py --package native/unreal/Packaged/<new-run>/Windows --output native/releases --version 0.8.2-terrain.1
```

The distribution helper requires Python 3.11+, verifies copied runtime files and
ZIP CRCs, retains notices/MSVC redist, excludes debug symbols and refuses to
replace prior output. Keep the whole extracted game folder together.

[Player download and controls](../DELIVERY.md). [Mac preparation](../unreal/MACOS.md).
Consult the delivery report for that package's measured performance scope.

Python diagnostic tools/tests use the recorded plotting dependency:

```sh
python -m pip install -r native/tools/requirements.txt
python -B -m unittest discover -s native/tools -p "test_*.py" -v
```

## Historical 0.5.0 build and acceptance procedure

The following preserved instructions target their original exact version, not
the current CLI. Its earlier missing-UE blocker is now resolved. Do not run the
old matrix against current binaries or combine its results with current evidence.

# Preserved build procedure

Runtime `0.5.0-geometry.2`, COASTER5. See [release validation](../VALIDATION.md) for measured matrix results and frozen evidence. UE compilation, cooking and runtime checks remain external gates.

From the repository root:

```powershell
& .\native\tools\build.ps1 -Compiler .\native\.tools\zig-x86_64-windows-0.14.1\zig.exe -Test
python -B -m unittest discover -s native/tools -p "test_*.py" -v
& .\native\unreal\scripts\validate_source.ps1 -ZigPath .\native\.tools\zig-x86_64-windows-0.14.1\zig.exe
```

The build creates `build/coaster_cli.exe`, all permanent suite executables and `build/coaster_convergence.exe` inside `native/`. It never downloads a compiler. `-Clean` is restricted to that resolved build directory. Station tests default to new generated/saved/reloaded COASTER5 fixtures; old-schema fixtures are explicit rejection cases.

The acceptance runner requires exact report identity `0.5.0-geometry.2`, request echoes, completed/accepted/uncancelled status, no errors and performed/passed coarse/half-step convergence. Each accepted result must independently replay from its complete saved geometry. Evidence paths and hashes are checked on publication and resume. Contradictory reports are infrastructure failures; missing raw I305 reference remains an all-record rejection.

Run the frozen matrix explicitly, outside unit tests:

```powershell
python native/tools/acceptance.py --cli native/build/coaster_cli.exe --out-dir native/artifacts/acceptance-v050-reproduce --count 1000 --workers 2 --candidates 8 --step 0.0010416666666666667 --timeout 120 --validate-timeout 120 --samples 30 --min-success 0.95 --min-physics-proof 0.95 --min-all-records 0.95
```

Defaults are 1000 cases, 500 per preset, balanced terrains, seeds 1–1000, eight candidates, 960 Hz coarse integration, 120-second generation/replay timeouts and two workers. Half-step verification is mandatory in the core. Minimum-success flags are explicit: without them, a successful harness process does not establish a high generation success rate. The separate convergence runner can audit every accepted frozen save again; see `convergence/README.md`.

Earlier 0.3 results describe their archived version. Current claims require this source/executable manifest and its own completed evidence. UE build/cook/playback and GPU verification remain external gates.
