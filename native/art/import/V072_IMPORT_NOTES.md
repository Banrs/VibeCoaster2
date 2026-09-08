# V072 train v004 import preparation

Run `native/unreal/scripts/import_art_v072.py` in the UE 5.8 editor with `VIBECOASTER_ART_IMPORT_SPEC` pointing at `native/art/import/import-spec-v072-trainv004.json`. The spec selects fresh `/Game/Art/V072/Import1` and reads the actual completed v004 manifest at `native/art/review/20260908-4/train-manifest.json`. No dimensions are supplied from a guessed drawing. The importer preserves V071, the failed reflection trial and old source models.

The proven original-FBX pipeline remains unchanged: scene/unit conversion, force-front-X and explicit UE import yaw +90 degrees. The material-coded witness must pass before the new train or five contained environment modules can pass. A generic bounds-only check cannot replace that witness.

The ten authored v004 slots have a fresh explicit palette. `VCTrain4_WindDeflector_ClearCyan` is a dedicated thin-translucent material: Translucent blend, Thin Translucent shading, Surface Forward Shading lighting, linear transmittance (0.91,0.975,0.99), opacity 0.04 and surface coverage 1.0. These are deliberate game-art settings, not a claim of exact optical equivalence with Blender. The material retains depth testing. The source shield is a closed 4 mm shell, so backface culling remains enabled instead of rendering duplicate two-sided layers.

Refraction is explicitly `RM_None`, with the refraction input unconnected. This avoids screen-space distortion of fast-moving rail/terrain geometry; it is not a physical glazing simulation. Actual grazing-angle appearance, highlight strength, visibility through both shields, overlapping-car sorting and temporal behaviour still require UE visual/POV review. No alpha fallback to an opaque base-color material is permitted.

The receipt independently reads the material bound to every mesh slot, base-color/metallic/roughness graph constants, ISM usage, shield blend/shading/lighting/refraction/depth settings, opacity input, and the connected thin-translucent transmittance/coverage constants. This proves graph/configuration state only. Shader compilation/cooking and visual results remain separate evidence; the receipt explicitly labels that limit.

Installed primary implementation checked: UE `Runtime/Engine/Public/Materials/Material.h`, `MaterialExpressionThinTranslucentMaterialOutput.h`, `Editor/MaterialEditor/Private/MaterialEditingLibrary.cpp`, and `Shaders/Private/ThinTranslucentCommon.ush`. Epic also documents the three required thin-transparency settings and transmittance output: [Using Transparency in Unreal Engine Materials](https://dev.epicgames.com/documentation/unreal-engine/using-transparency-in-unreal-engine-materials).

Review files are in `native/art/blender/train_review_v004/`: preparation, separate activation, then one front/side/rear/three-car eye render per MCP call. They use Eevee at 1280×720 and request 64 samples where exposed. These are small actual art renders, never a game-performance benchmark. They preserve the 1.2 m eye height and 3.4 m spacing.

Authoring-agent checks: Python syntax passes; six Blender review scripts pass the installed MCP safe-mode validator. No Blender/MCP/UE execution was performed by this agent for these preparation files.
