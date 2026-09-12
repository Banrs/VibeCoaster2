# Build

C++20 core:

```sh
cmake -S native -B native/build-cmake -DCMAKE_BUILD_TYPE=Release
cmake --build native/build-cmake --config Release --parallel 2
```

Windows game, with UE 5.8+ and its Visual Studio toolchain:

```powershell
& ./native/unreal/scripts/package.ps1 -UnrealRoot 'D:/Games/Epic Games/UE_5.8'
```

The package is written under `native/unreal/Packaged`. The current playable installation is `native/game`.