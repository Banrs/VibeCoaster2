# VibeCoaster2

C++20 hybrid FVD/spline coaster authoring and Unreal 5.8 viewer. Development has returned to the pre-rewrite **Default 1** source on `codex/legacy-authoring`. The rejected rewrite remains preserved on its own branch and in local packages.

Default 1 is the visual starting point, not an accepted final design. Its useful post-package edits are being reviewed individually. Current findings and outstanding design requirements are recorded in [Legacy authoring recovery](docs/legacy-authoring-recovery.md).

Start with [HANDOFF.md](HANDOFF.md) for current status, build commands, and unresolved issues. Double-click `Play VibeCoaster2.lnk` in the repository or `VibeCoaster2` on the desktop to launch the current Default 2 local game and automatically load its corrected ride; press Space to ride. Until the standalone package is promoted, the shortcuts use Unreal game mode and the isolated `UserData-Development` profile. Recreate that shortcut with `native/unreal/scripts/create_shortcuts.ps1 -Desktop -Development -UnrealRoot "D:/Games/Epic Games/UE_5.8"`. `dist/current.json` still identifies the preserved Default 1 standalone package; the helper without `-Development` follows that published pointer.

The original camelback image, trace, fit and clifftop reference remain in [docs/references](docs/references). Historical checkpoint reports document their original snapshots; their acceptance wording does not override the current design issues.

Build the native core with `native/CMakeLists.txt`. Windows and macOS are the CI platforms. Package Unreal with `native/unreal/scripts/package.ps1`. Tests cover authored sources, frame and force derivatives, finite-train replay, persistence, clearance and numerical refinement; these checks do not establish manufacturer approval or whole-standard certification.

Export a recipe with `coaster_cli recipe --out ride.vcrecipe`, then generate with `coaster_cli generate --recipe ride.vcrecipe --out ride.coaster`. Only fully accepted closed-circuit results can be saved. Loaded rides receive fresh validation.
