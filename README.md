# VibeCoaster

One active game: a single-player Unreal Engine 5 coaster game with an independent
C++ numerical core, under [`native/`](native/). The earlier TypeScript/browser
implementation has been retired; it is not a dependency of this game.

[Download the Windows engineering preview](https://github.com/Banrs/VibeCoaster2/releases/tag/native-v0.8.1-intent.1-ae09f02) · [Controls and delivery status](native/DELIVERY.md) · [Build from source](native/tools/BUILDING.md)

Extract the complete Windows ZIP and open **Play VibeCoaster.cmd** or
**VibeCoaster.exe**. No Unreal Editor or Blender installation is needed to play.
Choose **PHYSICS-PROOF**, **Canyon**, seed **42**, then Enter to generate and
Space to ride. Fresh profiles use desktop-native borderless resolution, 100%
internal resolution and uncapped rendering with VSync off; saved settings remain respected.

The native generator retains seeded variation, folded circuits and crossings,
force-authored signatures and inversions, adaptive supports and flat,
hills and canyon terrain. Full finite-train simulation and convergence checks
gate accepted rides and saves. Height above local terrain is distinct from
height above the station and landscape relief.

**Development preview, not a completed foundation.** Authentic eligible
I305/Pantherian recordings are still needed for strict intensity comparison;
ALL RECORDS stays unavailable. Whole-route flow, terrain-directed approaches,
drive/support engineering, actual Mac/Metal verification and continuous human
POV review remain unfinished. No ASTM compliance claim is made.

This engineering package binds source commit `ae09f02` and application/save identity
`0.8.1-intent.1`. Its loss-aware tall signature and continuous canyon climb passed
all 26 local CTest suites and Windows/Linux/macOS core/Python CI. Eleven targeted
circuits passed exact saved replay and convergence; three actual packaged terrain/seat
runs passed full traversal and save/load. Their complete-stop durations are 166.19–170.92 s;
time to final braking is reported separately. These are targeted checks, not broad
ride-quality or performance acceptance. Older releases and evidence remain preserved.

## Development

```sh
cmake -S native -B native/build-cmake
cmake --build native/build-cmake --config Release --parallel 2
ctest --test-dir native/build-cmake -C Release --output-on-failure --parallel 1
```

CI checks the independent C++ core and Python tools on Windows, Linux and macOS, without
benchmarks. UE packaging uses the engine/toolchain workflow in
[native/unreal/README.md](native/unreal/README.md). Mac preparation is documented
in [MACOS.md](native/unreal/MACOS.md); a Mac binary is not yet verified.

Original source uses the repository [license](LICENSE). Unreal and bundled
runtime components retain their own redistribution notices in the player ZIP.
See [license scope and runtime terms](LICENSE-SCOPE.md).
