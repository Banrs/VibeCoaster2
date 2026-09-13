# VibeCoaster

A roller-coaster prototype with a C++20 generation and simulation core and an Unreal Engine viewer. The current build generates rides on flat ground.

Current numerical save version: `0.8.0-immelmann.2 / COASTER5`. Saves require a matching numerical version; hills and canyon saves are unsupported.

## Source

- [`native/core`](native/core): generation, geometry, simulation, persistence and tests.
- [`native/unreal`](native/unreal): Unreal project, viewer and required assets.
- [`native/unreal/scripts`](native/unreal/scripts): packaging, asset creation and startup benchmarking.

## Build and test the core

Requires CMake 3.20+ and a C++20 compiler. Python 3 enables the CLI argument tests.

```sh
cmake -S native -B native/build-cmake -DCMAKE_BUILD_TYPE=Release
cmake --build native/build-cmake --config Release --parallel 2
ctest --test-dir native/build-cmake -C Release --output-on-failure --parallel 4 --timeout 600 --stop-on-failure -L component
ctest --test-dir native/build-cmake -C Release --output-on-failure --parallel 4 --timeout 600 --stop-on-failure -L integration
```

This builds the `coaster_cli` command-line application and the native tests independently of Unreal Engine.

## Build the game

The [Unreal project](native/unreal/VibeCoaster.uproject) uses UE 5.8. Windows packaging requires the Visual Studio C++ toolchain:

```powershell
& ./native/unreal/scripts/package.ps1 -UnrealRoot 'C:/path/to/UE_5.8'
```

Packages are written under `native/unreal/Packaged`; `-OutputDirectory <path>` selects another location. The [macOS packaging script](native/unreal/scripts/package_macos.sh) runs on a Mac with Unreal Engine and Xcode.

## Play

In a Windows package, open `VibeCoaster/Binaries/Win64/VibeCoaster.exe` directly. Keep the complete package folder together.

Choose PHYSICS-PROOF and a seed in the setup menu, then generate a ride. ALL RECORDS mode requires supplied reference data.

| Key | Action |
| --- | --- |
| Up/Down, Left/Right | Select and change setup settings |
| Enter in setup, or G | Generate |
| Space | Start or pause playback |
| Tab | Toggle setup |
| 1 / 2 / 3 outside setup | Select a seat |
| M | Toggle overview |
| R | Restart playback |
| F5 / F9 | Save / load |
| Escape | Cancel an active request or open setup |

## License

Original project code and assets use the [MIT license](LICENSE). Unreal Engine, third-party components and reference recordings have separate terms; see [license scope](LICENSE-SCOPE.md).
