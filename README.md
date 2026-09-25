# VibeCoaster2

C++20 hybrid FVD/spline coaster authoring and Unreal 5.8 viewer. **Riftwake default.3** is the current source and playable baseline on codex/legacy-authoring. The rejected default.4 package and generated history were moved beside the legacy checkpoints at D:/Coding/Codex/vibecoasterlegacy.

The rollback followed feedback about constant roll, late LSM placement in section 1, and the clifftop. The next design task is specified in [HANDOFF.md](HANDOFF.md); use the [code map](docs/CODE_MAP.md) to find the relevant implementation paths.

Start with [HANDOFF.md](HANDOFF.md) for status and evidence. Double-click Play VibeCoaster2.lnk in the repository or VibeCoaster2 on the desktop to launch the verified default.3 standalone and automatically load the accepted ride from UserData-Riftwake-V3; press Space to ride. Both shortcuts and dist/current.json select the same default.3 package.

The original camelback image, trace, fit and clifftop reference remain in [docs/references](docs/references). Historical checkpoint reports document their original snapshots; their acceptance wording does not override the current design issues.

Build the native core with `native/CMakeLists.txt`. Windows and macOS are the CI platforms. Package Unreal with `native/unreal/scripts/package.ps1`. Tests cover authored sources, frame and force derivatives, finite-train replay, persistence, clearance and numerical refinement; these checks do not establish manufacturer approval or whole-standard certification.

Export a recipe with `coaster_cli recipe --out ride.vcrecipe`, then generate with `coaster_cli generate --recipe ride.vcrecipe --out ride.coaster`. Only fully accepted closed-circuit results can be saved. Loaded rides receive fresh validation.
