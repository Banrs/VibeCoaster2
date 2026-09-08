# UE5.8 art import handoff

`native/unreal/scripts/import_art_v071.py` is editor-only. It creates original solid-colour materials and static meshes under the fresh `/Game/Art/V071` namespace. It does not alter runtime source, levels, numerical collision geometry or historical content. Root executes it only after successful Blender exports and manifests exist.

Copy `import-spec-v071.example.json` to a concrete spec in `native/art/import/` and set its two manifest paths to the actual persisted JSON output from the train and environment build scripts. The train manifest must be the direct `VC_TRAIN_BUILD_RESULT` object; the environment manifest must be the direct JSON between `VC_ENVKIT_MANIFEST_BEGIN/END`, not an MCP response wrapper. Set the environment variable `VIBECOASTER_ART_IMPORT_SPEC` to that absolute spec path before launching the existing UE editor Python workflow. Use a fresh receipt run identity. No explicit GPU rendering is required for the import checks.

Initial mesh paths are `/Game/Art/V071/SM_TrainCar`, `SM_TrackTie`, `SM_StationPlatformPanel`, `SM_StationPlatformEndPanel`, `SM_StationRoofPanel`, `SM_StationPost` and `REVIEW_CoordinateWitness`. Materials live under `/Game/Art/V071/Materials`. The script refuses an existing destination directory or receipt directory; if an attempt fails, preserve it and use an explicit fresh child such as `/Game/Art/V071/Retry02`, then update integration paths only after that attempt succeeds. It never retries under altered coordinate settings or rescales failed geometry automatically.

The coordinate witness is imported first. Actual LOD0 section vertices are grouped by their imported material slot, and the one-metre cube, ochre +X nose and coral rider-right marker must land at their expected centimetre bounds. This checks 1 metre to 100 centimetres and exactly one source-Y reflection; aggregate dimensions alone cannot establish left/right correctness. The witness must pass before the train or station/tie imports begin. Every subsequent asset is compared against its evaluated Blender manifest bounds at its deliberate source pivot. Maximum allowed coordinate discrepancy is 0.02 cm (0.2 mm).

The importer requires exact authored material slot-name sets, verifies nonempty finite section vertices/normals, valid triangle indices, vertex bounds against asset bounds, and no generated simple collision. It assigns the authoring scripts’ actual linear base colours, metallic values and roughness, with instanced-static-mesh material support. CPU access is enabled temporarily for geometry inspection and disabled before the validated mesh is saved. Export/import evidence does not replace finite-train clearance, physics, actual gameplay POV review or packaging.

A receipt is written incrementally to `native/unreal/Saved/ArtImport/<run>/receipt.json`. It records engine identity, source spec/manifests/FBX hashes, imported paths, exact vertex bounds, material/section counts, coordinate witness results, import options and errors. A success status is `import-validated-not-integrated`; failures are retained as `failed-preserved`. Initial FBX imports are saved before validation so failed geometry remains inspectable. No bogie, rail-to-spine web, fixed saved tower, or radius-specific joint/cap reference is imported.

## Locally verified API basis

No generated Python stub was found in this checkout or installed PythonScriptPlugin. Calls are grounded in the reflected API declarations from the installed UE5.8 source:

- `Engine/Source/Editor/UnrealEd/Classes/Factories/FbxImportUI.h`, `FbxStaticMeshImportData.h`, `FbxAssetImportData.h`, and `FbxMeshImportData.h`: explicit static import, unit/axis conversion, no generated collision/Nanite, combine meshes and preserve assembly-origin coordinates.
- `Engine/Source/Developer/AssetTools/Private/AssetTools.cpp` lines 3566-3570: a supplied factory bypasses Interchange; the script uses `FbxFactory` rather than changing global feature flags.
- `Engine/Source/Editor/UnrealEd/Public/AssetImportTask.h`: synchronous task defaults, explicit factory/options, `get_objects()`.
- `Engine/Source/Editor/StaticMeshEditor/Public/StaticMeshEditorSubsystem.h`: `get_lod_material_slot`, `set_allow_cpu_access`, collision count.
- `Engine/Plugins/Runtime/ProceduralMeshComponent/Source/ProceduralMeshComponent/Public/KismetProceduralMeshLibrary.h`: reflected `get_section_from_static_mesh` with vertices/triangles/normals/UVs/tangents. This plugin is already enabled in the project.
- `Engine/Source/Runtime/Engine/Classes/Engine/StaticMesh.h`: reflected bounds, section count and material slots.

Python syntax compilation passed. The authoring agent has not launched UE or run the importer. Runtime property-name mapping, actual FBX basis conversion and rendering remain execution checks; any mismatch stops with a concrete receipt rather than being presented as an accepted import.
