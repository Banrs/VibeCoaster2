> Active result: Blender **5.2.1 LTS**, built successfully through the 12 separate `train_stages_v003/` MCP stages. Editable source and runtime exports are under `20260908-train-v003`. Actual Blender renders and UE import checks passed; original FBX uses explicit UE import yaw +90 degrees after the material-coded axis witness. Earlier combined v001/v002 attempts below remain failure history, not the active builder. See `../README.md` for current delivery status.

# Train art build handoff

`build_train.py` is authored for Blender 4.5 and the installed Blender MCP safe-mode validator. Python syntax and `validate_code` passed; the authoring agent has not executed Blender or MCP.

Root must precreate these two folders before sending the script contents to `execute_blender_code`:

- `native/art/source/train/20260907-train-v002/`
- `native/art/exports/train/20260907-train-v002/`

For a revised run, change the three literal RUN/SOURCE/EXPORT values to a fresh revision so previous exports and failed work remain available. The script replaces only `VCTrain_` in-memory data; unrelated scene contents remain intact. Its separate `VCTrain_AssetReview` scene contains the art. The saved `.blend` is a local copy of the shared session, retaining other scenes; the GLB/FBX exports contain only the selected assembled asset. Root should persist the printed `VC_TRAIN_BUILD_RESULT` JSON as the manifest and verify files exist.

The runtime assembly has 57 named source parts, an exact 2.55 × 1.70 m maximum footprint, rail-centre origin, and source vertices asserted inside the existing unpadded body envelope. The fixed eye witness is at 1.2 m, with +X forward, -Y rider-right and +Z up. Source parts include a continuous contoured orange hull, two shaped graphite seats and slim headrests, separate closed lap bars/hinges/handles, tread plates, ivory trim and metal chassis details. No driver, simulation or restraint interlock is invented.

The separate wheel study has 40 parts: four running/up-stop/side-guide arrangements around a 1.30 m rail gauge, with wheel radii 0.13/0.08/0.075 m. Their circular contact surfaces meet rail radius 0.085 m. This is review geometry, not a manufacturer drawing or validated moving assembly. The collection and export are explicitly marked UNVALIDATED and default hidden. Do not combine `bogie_review_unvalidated` with the runtime car until its moving envelope has been implemented and checked.

The assembled export objects default hidden after export, avoiding duplicate geometry over the visible source parts. Bogie and rail-reference collections also default hidden. Select the dedicated scene to review the car; unhide the review collection and its objects deliberately for the wheel/contact study.

Materials are original solid-colour Principled shaders: warm orange paint, burnt-orange secondary shell, graphite padding, dark grip/tyre rubber, brushed aluminium, dark chassis steel and ivory trim. No external assets or textures are used. Export filenames are `train_car_runtime.glb/.fbx` and `bogie_review_unvalidated.glb/.fbx`. Export exceptions are reported independently in the printed manifest.

Remaining root checks: Blender execution and bounds assertions; actual mesh silhouette/material review; exported UE units/axis witness and imported bounds; front/middle/rear POV at the unchanged camera datum. A source bounds assertion is not a substitute for imported-envelope or gameplay review.

## Revision v002: native crash mitigation

The v001 runtime GLB and FBX were written before Blender 4.5.13 crashed with an unsymbolicated native access violation across evaluation threads. No exact C++ fault is established. v001 exports and logs remain preserved.

v002 no longer replaces or frees source mesh/curve datablocks during dependency-graph evaluation. It creates independent evaluated mesh snapshots and assembles export meshes from those snapshots while leaving the editable source objects intact. It selects the actual dedicated window scene, disables redundant exporter modifier evaluation on already baked meshes, defers hiding until every exporter returns, and saves the source before exporting. A second `VibeCoaster_Train_Review.blend` stores final visibility. Begin/end markers identify each export boundary. Syntax and MCP safe-mode validation pass; a fresh actual Blender run is still required to establish whether the native crash is resolved.
