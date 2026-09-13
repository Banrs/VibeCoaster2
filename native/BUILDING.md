# Build

C++20 core:

```sh
cmake -S native -B native/build-cmake -DCMAKE_BUILD_TYPE=Release
cmake --build native/build-cmake --config Release --parallel 2
```

Run all local checks, components before integration:

```sh
ctest --test-dir native/build-cmake -C Release --output-on-failure --parallel 4 --timeout 600 --stop-on-failure -L component
ctest --test-dir native/build-cmake -C Release --output-on-failure --parallel 4 --timeout 600 --stop-on-failure -L integration
```

Four workers bound suite concurrency; generation and replay also use internal workers. Use two on smaller machines or while other CPU work is running. Timings exclude compilation and require an idle machine for comparison. CI retains two workers.

Windows game, with UE 5.8+ and its Visual Studio toolchain:

```powershell
& ./native/unreal/scripts/package.ps1 -UnrealRoot 'D:/Games/Epic Games/UE_5.8'
```

The package is written under `native/unreal/Packaged`. The current playable installation is `native/game`.
