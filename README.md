# VibeCoaster

One active game: a single-player Unreal Engine 5 coaster game with an independent
C++ numerical core, under [`native/`](native/). The earlier TypeScript/browser
implementation has been retired; it is not a dependency of this game.

[Download the Windows preview](https://github.com/Banrs/OpenVibeCoaster/releases/tag/native-v0.7.3-review.4) · [Controls and delivery status](native/DELIVERY.md) · [Build from source](native/tools/BUILDING.md)

Extract the complete Windows ZIP and open **Play VibeCoaster.cmd** or
**VibeCoaster.exe**. No Unreal Editor or Blender installation is needed to play.
Choose **PHYSICS-PROOF**, **Canyon**, seed **42**, then Enter to generate and
Space to ride. The launcher opens a 1600×900 window capped at 60fps.

The native generator retains seeded variation, folded circuits and crossings,
true Immelmann/dive elements, FVD-authored crests, adaptive supports and flat,
hills and canyon terrain. Full finite-train simulation and convergence checks
gate accepted rides and saves. Height above local terrain is distinct from
height above the station and landscape relief.

**Development preview, not a completed foundation.** Authentic eligible
I305/Pantherian recordings are still needed for strict intensity comparison;
ALL RECORDS stays unavailable. Actual Mac/Metal verification and further scenery
and human POV review remain unfinished. No ASTM compliance claim is made.

The recovery delivery was reviewed without running performance benchmarks while
Cities: Skylines 2 was open. The retired browser's uncommitted source and prior
outputs were archived locally before removal from the active repository. Git
history and frozen numerical evidence remain preserved.

## Development

```sh
cmake -S native -B native/build-cmake
cmake --build native/build-cmake --config Release --parallel 2
ctest --test-dir native/build-cmake -C Release --output-on-failure --parallel 1
```

CI checks the independent C++ core and Python tools on Windows and Linux, without
benchmarks. UE packaging uses the engine/toolchain workflow in
[native/unreal/README.md](native/unreal/README.md). Mac preparation is documented
in [MACOS.md](native/unreal/MACOS.md); a Mac binary is not yet verified.

Original source uses the repository [license](LICENSE). Unreal and bundled
runtime components retain their own redistribution notices in the player ZIP.
