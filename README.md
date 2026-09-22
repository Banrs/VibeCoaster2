# VibeCoaster2

C++20 / Unreal 5.8 coaster authoring rewrite with mixed FVD and spatial-spline
sources, independent finite-train dynamics and scoped F2291-25 acceleration
assessment.

The current native corpus passes nine representative recipes and persistence
regressions, including fresh operating scenarios and separate spatial/temporal
refinement on every case. Saved loads run those checks again before scene
replacement. The default circuit delivers approximately 180 s active riding,
3.8 s lip braking and 8.14 s terminal braking. Runtime and release acceptance
remain open; this is a development preview.

Build the portable core with CMake 3.24 or later and run CTest:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel 1
ctest --test-dir build -C Release --output-on-failure --parallel 1
```

GitHub CI targets Windows and macOS. On the configured development Windows
machine, `scripts/build.ps1 -Test` uses the installed MSVC toolchain. Unreal
builds also use one compiler worker.

See [rewrite decisions](docs/rewrite.md), [current evidence and open work](docs/progress.md)
and [acceleration scope](docs/acceleration-standard.md). A checksum establishes
save integrity; it never establishes physical validity. Final acceptance also
requires full scene clearance, GPU traversal, saved-load p99 below 5 s, a tested
package and exact Play executable identity.

The archived implementation, launchers and fallback profiles are preserved
outside this repository. The parent workspace and VibeCoasterjs are untouched.
