# macOS native build and acceptance

macOS is a first-class intended platform for the same independent numerical core and UE game. This entry point prepares and packages the existing implementation; it does not replace the generator or weaken acceptance. **A real Mac build, packaged ride review and performance measurements are still required.** Windows source/shell checks do not establish Mac compatibility.

## Host setup

Use a Mac with the **macOS distribution** of Unreal Engine, full Xcode selected in `xcode-select`, its macOS SDK and Metal compiler, and completed Xcode first-launch setup. The Windows engine installation cannot produce this package. Close the editor using this project before running; do not run concurrent builds against the same checkout.

For UE 5.8, Epic's current [macOS requirements](https://dev.epicgames.com/documentation/unreal-engine/macos-development-requirements-for-unreal-engine?lang=en-US) list macOS Sonoma 14.5 or newer, Xcode 26.0 minimum and 26.1.1 recommended; Xcode 26.4 is incompatible. UE 5.6–5.7 list macOS 14.0 and Xcode 15.2 minimum. The 5.8 SDK JSON available on the Windows workstation contains broader limits than the published platform page; this script uses the stricter published prerequisites and leaves final SDK compatibility to UnrealBuildTool. Apple separately requires **macOS Sequoia 15.6 or later to run Xcode 26/26.1.1**, so the build script enforces that stricter host minimum; the engine's OS floor is not a promise that its selected Xcode runs there. See [Apple's Xcode 26.1.1 requirements](https://developer.apple.com/documentation/xcode-release-notes/xcode-26_1-release-notes). These requirements were checked on 2026-09-07.

The default game architecture is **Apple Silicon arm64**. Use a native terminal rather than a shell running through Rosetta. `--architecture universal` explicitly requests `arm64+x64`; the editor target uses the host architecture. That syntax was checked against UE 5.8's actual `UnrealArchitectures` parser and UAT `ProjectParams`; producing a working Intel slice and validating Intel gameplay still need real hardware evidence. No Intel support is inferred from an arm64 build.

The script does not accept licenses, change the selected Xcode, install components, or submit signing/notarization requests. Follow any prerequisite error on the Mac. UAT receives `-NoCodeSign`; local executable signing performed by the Apple toolchain may still be needed. This workflow does not claim a Developer ID signed, notarized or App Store-ready release.

## Build

Run from a clone containing both `native/core` and `native/unreal`. Keep their relative layout intact; UE compiles the canonical C++ sources directly.

```bash
# Adjust the engine location to the actual macOS installation.
bash native/unreal/scripts/package_macos.sh \
  --unreal-root '/Users/Shared/Epic Games/UE_5.8' --prepare-only

# Editor build, real asset bootstrap, UE automation, cook/stage/archive.
bash native/unreal/scripts/package_macos.sh \
  --unreal-root '/Users/Shared/Epic Games/UE_5.8'

# Optional, explicit universal package after arm64 Development validation.
bash native/unreal/scripts/package_macos.sh \
  --unreal-root '/Users/Shared/Epic Games/UE_5.8' --architecture universal
```

`--configuration Shipping` selects Shipping for the packaged game; editor preparation always uses Development. `--output-directory '/Volumes/Builds/VibeCoaster'` changes the archive parent. The script checks Darwin, engine version/files, full Xcode, SDK, clang and Metal availability before building. `VibeCoaster.CoordinateContract`, `VibeCoaster.MeshContract` and `VibeCoaster.TerrainBackdropContract` must all report success. There is no automation-skip acceptance path.

Each invocation on a Mac reserves `native/unreal/Saved/MacPackaging/run-<UTC>-<unique suffix>/` for its environment record, temp files, build/bootstrap/automation/UAT logs, reports, staging output and unique bootstrap receipt. Failure retains those files. DDC and Zen data stay under `native/unreal/Saved/DerivedDataCache/`; they are disposable shared caches, not frozen evidence. Ordinary intermediate/cooked files also remain mutable build products. Keep complete release evidence and packaged runs separately; do not infer success from cached content.

Default archives are fresh children of `native/unreal/Packaged/Mac/`. A successful UAT exit is insufficient: the script requires a new `VibeCoaster.app`, an executable identified by its own Info.plist, and the requested Mach-O architecture slices. Existing materials/maps are preserved by the content bootstrap, while a receipt unique to this run proves that the bootstrap reached completion.

## Acceptance on the Mac

Run the packaged app and complete the same [manual acceptance checks](README.md#remaining-acceptance-checks-on-a-ue-workstation): complete circuits on flat/hills/canyon; front/middle/rear POV and telemetry; pause/restart/overview; cancellation and replacement; save, process restart and load; rejection without destroying the accepted ride; shutdown during work. Record actual failing seeds and retain each failed build/review. Check physical function-key behavior on Mac keyboards for F5/F9 saves/loads.

The default ALL RECORDS mode must remain unavailable until an authentic eligible processed I305/Pantherian benchmark is supplied. Explicit PHYSICS-PROOF remains labeled as intensity untested. All selected geometry/dynamic targets and the mandatory 960/1920 Hz verification still apply. Run the portable C++ suites on the Mac as well; the three UE contracts do not establish numerical portability alone.

Measure the actual Metal build at **2560×1440 render resolution**, with the Mac model/chip, RAM, macOS, engine version, graphics settings and power mode recorded. Review sustained frame time, hitches, generation/commit responsiveness and representative complete POV rides; a Retina window's logical size is not evidence of 1440p rendering. No FPS claim follows from packaging, NullRHI automation or Windows results. Cross-platform saved-design replay and any universal Intel build also need their own verification before release claims.
