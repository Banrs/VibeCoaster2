# Unreal project

Open `VibeCoaster.uproject` with UE 5.8+. The portable core is compiled through the wrappers in `Source/VibeCoaster/Private`.

`./scripts/package.ps1 -UnrealRoot <engine-directory>` builds and packages Windows. Runtime meshes and materials are in `Content`; `scripts/create_content.py` creates missing base content.

`python scripts/benchmark_startup.py --before <exe> --after <exe> --output <new-directory>` compares Development builds from process launch to moving playback using real rendering and isolated profiles.
