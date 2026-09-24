"""Build the texture-free escarpment palette through Unreal editor Python.

World-space sediment beds, talus, and sparse dry scrub follow the exact terrain
surface. The shader never offsets the mesh or changes its collision normals.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import unreal

NAME = "M_Ground_Highlands"
ASSET = "/Game/Materials/" + NAME
CODE = r"""
float upright = saturate(abs(SurfaceNormal.z));
float2 metres = WorldPosition.xy * 0.01;
float heightMetres = WorldPosition.z * 0.01;

// Broad soil fields stay legible at overview distance. Small mineral variation
// only breaks up the flat portions; it does not cover the rock in visual noise.
float patch = 0.5 + 0.26 * sin(metres.x * 0.008 + sin(metres.y * 0.004))
                  + 0.24 * sin(metres.y * 0.011 - metres.x * 0.003);
float mineral = 0.5 + 0.25 * sin(metres.x * 0.37 + metres.y * 0.21)
                    + 0.25 * sin(metres.y * 0.43 - metres.x * 0.18);
float scrub = smoothstep(0.78, 0.93, patch) * smoothstep(0.91, 0.99, upright);
float3 soil = lerp(float3(0.22, 0.155, 0.097), float3(0.38, 0.275, 0.16), patch);
soil *= 0.96 + 0.08 * mineral;
soil = lerp(soil, float3(0.18, 0.20, 0.105), scrub * 0.5);

// The same world height defines bedding across every mesh tile and LOD ring.
// A gentle plan-view warp keeps the exposed layers from looking ruler straight.
float bedHeight = heightMetres + 1.2 * sin(metres.x * 0.023)
                               + 0.7 * sin(metres.y * 0.037);
float bed = frac(bedHeight / 9.4);
float seam = 1.0 - smoothstep(0.015, 0.095, bed);
float paleBed = smoothstep(0.24, 0.32, bed) * (1.0 - smoothstep(0.58, 0.66, bed));
float3 sandstone = lerp(float3(0.27, 0.145, 0.083), float3(0.43, 0.29, 0.17), paleBed);
sandstone *= (0.92 + 0.14 * patch) * (1.0 - 0.17 * seam);

float rock = 1.0 - smoothstep(0.53, 0.88, upright);
float talus = (1.0 - smoothstep(0.78, 0.98, upright)) * (1.0 - rock);
float3 surface = lerp(soil, float3(0.32, 0.235, 0.155), talus * 0.72);
return lerp(surface, sandstone, rock);
"""
SCALARS = {"MP_ROUGHNESS": 0.93, "MP_METALLIC": 0.0, "MP_SPECULAR": 0.15}


def main():
    library = unreal.EditorAssetLibrary
    edit = unreal.MaterialEditingLibrary
    material = library.load_asset(ASSET) if library.does_asset_exist(ASSET) else None
    if material is None:
        material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            NAME, "/Game/Materials", unreal.Material, unreal.MaterialFactoryNew())
    if not isinstance(material, unreal.Material):
        raise RuntimeError("Escarpment surface asset is not a material")
    previous = edit.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR)
    unchanged = isinstance(previous, unreal.MaterialExpressionCustom) and previous.get_editor_property("code") == CODE

    if not unchanged:
        edit.delete_all_material_expressions(material)
        material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)
        material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
        material.set_editor_property("two_sided", True)
        base = edit.create_material_expression(material, unreal.MaterialExpressionCustom, -200, 0)
        base.set_editor_property("code", CODE)
        base.set_editor_property("description", "Ochre escarpment: sediment seams, talus and sparse dry scrub")
        base.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT3)
        inputs = []
        for name in ("SurfaceNormal", "WorldPosition"):
            item = unreal.CustomInput()
            item.set_editor_property("input_name", name)
            inputs.append(item)
        base.set_editor_property("inputs", inputs)
        normal = edit.create_material_expression(material, unreal.MaterialExpressionPixelNormalWS, -500, 0)
        position = edit.create_material_expression(material, unreal.MaterialExpressionWorldPosition, -500, 160)
        if not (edit.connect_material_expressions(normal, "", base, "SurfaceNormal") and
                edit.connect_material_expressions(position, "", base, "WorldPosition") and
                edit.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR)):
            raise RuntimeError("Escarpment material input connection failed")
        for index, (name, value) in enumerate(SCALARS.items()):
            node = edit.create_material_expression(material, unreal.MaterialExpressionConstant, -200, 180+index*100)
            node.set_editor_property("r", value)
            if not edit.connect_material_property(node, "", getattr(unreal.MaterialProperty, name)):
                raise RuntimeError("Escarpment material scalar connection failed: " + name)
        errors = edit.recompile_material(material)
        if errors:
            raise RuntimeError("Escarpment shader compile failed: " + str(errors))
        if not library.save_loaded_asset(material, only_if_is_dirty=False):
            raise RuntimeError("Escarpment material save failed")
    asset_file = Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_content_dir())) / "Materials" / (NAME + ".uasset")
    if not asset_file.is_file():
        raise RuntimeError("Saved escarpment material asset is missing")
    saved = Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_saved_dir())).resolve()
    directory = saved / "TerrainPalette" / (datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S-%f") + "-escarpment-v3")
    directory.mkdir(parents=True, exist_ok=False)
    receipt = {"schema": "vibecoaster-terrain-palette-3", "asset": ASSET,
               "status": "unchanged" if unchanged else "created-and-compiled",
               "codeSha256": hashlib.sha256(CODE.encode("utf-8")).hexdigest(),
               "assetFile": str(asset_file), "assetSha256": hashlib.sha256(asset_file.read_bytes()).hexdigest(),
               "textures": 0, "worldPositionOffset": False,
               "normalInput": "PixelNormalWS", "scalars": SCALARS,
               "slopeTransitionDegrees": [28,58], "approximateStratumSpacingMeters": 9.4}
    (directory / "receipt.json").write_text(json.dumps(receipt,indent=2)+"\n",encoding="utf-8")
    unreal.log("VIBECOASTER_TERRAIN_PALETTE_OK " + str(directory / "receipt.json"))


if __name__ == "__main__":
    main()
