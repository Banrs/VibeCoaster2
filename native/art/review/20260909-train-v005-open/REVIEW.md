# Conventional open train v005 — 2026-09-09

Derived from frozen v004 evaluated meshes with only the 13 reviewed windshield/frame and aero-accent objects omitted. All 63 retained parts preserve their evaluated vertices, faces, materials and smoothing. No seats, restraints, footwells, chassis or main shell reshaped. No track/web changes.

## Actual result

Blender 5.2.1 background build/export/render completed with exit 0. Front, side and 1.2m rider-eye images were opened and inspected. The prominent windscreens and bright shoulder/nose lines are gone, leaving an open cabin and clear peripheral rider view. The existing low pointed shell remains, as requested for this minimal revision; no additional body redesign was needed for the scoped simplification.

- Runtime parts: 76 -> 63.
- Vertices: 12,924 -> 9,652.
- Triangles: 24,584 -> 18,492 (6,092 fewer, about 24.8%).
- Material slots: 10 -> 9; no windscreen/translucent slot.
- Identical source bounds: X +/-1.275m, Y +/-0.85m, Z 0.10..1.51m (within source float precision).
- Rail-origin pivot, 1.3m gauge, 3.4m spacing and 1.2m camera datum preserved. Runtime still derives all six car transforms from the finite-train design.
- Frozen source SHA256 unchanged: 96b640534a6caa286eb75628e9c1c9a3c42ad2f5b5a69933c73c7dece06e1070.

These are controlled Blender asset comparisons, not packaged gameplay POV or measured game performance. Existing absence of validated moving bogies/reaction hardware is unchanged.

## Files and validation

`train-manifest.json` contains exact removed object names, export hashes and bounds. `train-front.png`, `train-side.png`, `train-eye-1p2m.png` are the three actual inspected renders. FBX/GLB exports and the two saved source/review .blend files live under the matching `20260909-train-v005-open` export/source directories. Build log: `native/artifacts/flow-v080-20260908/operation-hardware-20260909/open-train-build.log`.

Python AST syntax validation, import-spec/actual-manifest checks and git diff whitespace checks pass. Root owns UE import/build/execution. `ImportedArtContract` now expects Conventional1/SM_TrainCar, the unchanged bounds and nine opaque materials. No UE build was run by this asset worker.

## Exact commands (PowerShell)

The completed derivation was:

```powershell
& 'D:/Toolchains/Blender/blender-5.2.1-windows-x64/blender.exe' --background 'D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v004/VibeCoaster_HighSpeedTrain.blend' --python 'D:/Coding/Codex/Vibecoasterjs/native/art/blender/build_train_v005_open.py' --python-exit-code 1
```

Do not rerun over this frozen revision: the script deliberately refuses existing output directories. The following imports only the new train and its independent asymmetric coordinate witness. It preserves Import1 station assets and TrackWeb1, and requires a fresh Conventional1 asset destination and receipt directory:

```powershell
$env:VIBECOASTER_ART_IMPORT_SPEC = 'D:/Coding/Codex/Vibecoasterjs/native/art/import/import-spec-v072-trainv005-open.json'
& 'D:/Games/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealEditor-Cmd.exe' 'D:/Coding/Codex/Vibecoasterjs/native/unreal/VibeCoaster.uproject' -run=pythonscript '-script=D:/Coding/Codex/Vibecoasterjs/native/unreal/scripts/import_art_v072.py' -unattended -nop4 -nosplash -stdout -FullStdOutLogOutput
```

Inspect `native/unreal/Saved/ArtImport/v072_20260909_trainv005_open/receipt.json`; status must be `import-validated-not-integrated`, including the witness and actual section/material/bounds checks. Launch a fresh editor/game process after import so constructor asset references are resolved anew. Then run `VibeCoaster.ImportedArtContract`, `VibeCoaster.OperationHardware` and `VibeCoaster.MeshContract` as coordinated with the root.
