# VibeCoaster2

C++20 hybrid FVD/spline coaster authoring and Unreal 5.8 viewer. **Riftwake default.3** is the current source and playable baseline on `main`, including the latest source/evidence sync from the former `codex/legacy-authoring` branch. The rejected default.4 package and generated history were moved beside the legacy checkpoints at D:/Coding/Codex/vibecoasterlegacy.

See [the branch consolidation record](docs/BRANCH_CONSOLIDATION.md) before integrating older branch work. `codex/fresh-rewrite` is a preserved alternative implementation, not a drop-in update to this baseline.

The current local state is still buggy. For online diagnosis, start with [the portable saves, review evidence and issue guide](docs/ONLINE_DIAGNOSIS.md), then read the open requirements in HANDOFF.md. Passing numerical checks do not mean the design issues are resolved.

The rollback followed feedback about constant roll, late LSM placement in section 1, and the clifftop. The next design task is specified in [HANDOFF.md](HANDOFF.md); use the [code map](docs/CODE_MAP.md) to find the relevant implementation paths.

Start with [HANDOFF.md](HANDOFF.md) for status and evidence. Play VibeCoaster2.lnk launches the current Unreal development game and loads the default.3 baseline from the separate UserData-GeometryReview profile; press Space to ride. The desktop VibeCoaster2 shortcut uses the same review build. dist/current.json retains the frozen standalone package.

Development source now includes [third-person and free-view controls](docs/camera-controls.md). Rejected train art has been removed; the new [editable train proposal](native/art/TRAIN_DESIGN.md) follows the retained [1.40 m gauge and maximum 2.40 m width](native/art/DIMENSIONS.md). Development still uses car-position markers. The updated Play shortcuts use the camera changes; the retained standalone package does not.

Current ride-reference notes are in [docs/references](docs/references). Earlier checkpoint images and verification reports were moved beside the legacy checkpoints; their old acceptance wording does not override [HANDOFF.md](HANDOFF.md).

Build the native core with `native/CMakeLists.txt`. Windows and macOS are the CI platforms. Package Unreal with `native/unreal/scripts/package.ps1`. Tests cover authored sources, frame and force derivatives, finite-train replay, persistence, clearance and numerical refinement; these checks do not establish manufacturer approval or whole-standard certification.

Export a recipe with `coaster_cli recipe --out ride.vcrecipe`, then generate with `coaster_cli generate --recipe ride.vcrecipe --out ride.coaster`. Only fully accepted closed-circuit results can be saved. Loaded rides receive fresh validation.
