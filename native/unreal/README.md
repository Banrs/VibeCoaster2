# Unreal project

Open `VibeCoaster.uproject` with UE 5.8+. The portable core is compiled through the wrappers in `Source/VibeCoaster/Private`.

`./scripts/package.ps1 -UnrealRoot <engine-directory>` builds and packages Windows. `./scripts/package_macos.sh --unreal-root <engine-directory>` builds on a Mac. Runtime meshes and materials are in `Content`; `scripts/create_content.py` creates missing base content.