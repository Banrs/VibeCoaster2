"""Create/verify a separate texture-free relief palette. Never overwrite existing graphs.

No terrain sampling, displacement, textures, noise, emissive or runtime promotion.
Run through UE editor Python. Every invocation writes a fresh Saved receipt.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import traceback
import unreal

NAME = "M_Ground_Highlands"
ASSET = "/Game/Materials/" + NAME
# PixelNormalWS uses the renderer's canonical terrain normals. Core Y reflection
# leaves world Z unchanged. abs preserves the existing two-sided soil convention.
# cos(20 degrees), cos(65 degrees): broad restrained slope transition, no stripes.
CODE = r"""
float upright = saturate(abs(SurfaceNormal.z));
float rock = 1.0 - smoothstep(0.422618262, 0.939692621, upright);
return lerp(float3(0.055, 0.105, 0.035), float3(0.073, 0.081, 0.092), rock);
"""
SIGNATURE = hashlib.sha256(CODE.encode("utf-8")).hexdigest()
SCALARS = {"MP_ROUGHNESS": 0.95, "MP_METALLIC": 0.0, "MP_SPECULAR": 0.20}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def validate(material):
    edit = unreal.MaterialEditingLibrary
    require(isinstance(material, unreal.Material), "Unexpected asset type; preserving existing asset")
    required = {"blend_mode": unreal.BlendMode.BLEND_OPAQUE,
                "shading_model": unreal.MaterialShadingModel.MSM_DEFAULT_LIT,
                "two_sided": True}
    for key, expected in required.items():
        require(material.get_editor_property(key) == expected, "Unexpected material setting: " + key)
    nodes = list(edit.get_material_expressions(material))
    require(len(nodes) == 5, "Unexpected graph expression count; preserving existing graph")
    base = edit.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR)
    require(isinstance(base, unreal.MaterialExpressionCustom), "Expected exact relief expression")
    require(base.get_editor_property("code") == CODE, "Existing relief code differs; preserving user edits")
    require(base.get_editor_property("output_type") == unreal.CustomMaterialOutputType.CMOT_FLOAT3,
            "Unexpected relief output type")
    names = list(edit.get_material_expression_input_names(base))
    inputs = list(edit.get_inputs_for_material_expression(material, base))
    require(len(names) == len(inputs) == 1 and str(names[0]) == "SurfaceNormal",
            "Relief must have exactly one SurfaceNormal input")
    require(isinstance(inputs[0], unreal.MaterialExpressionPixelNormalWS), "Relief normal is not PixelNormalWS")
    actual_scalars = {}
    for name, expected in SCALARS.items():
        node = edit.get_material_property_input_node(material, getattr(unreal.MaterialProperty, name))
        require(isinstance(node, unreal.MaterialExpressionConstant), "Unexpected scalar graph: " + name)
        value = float(node.get_editor_property("r"))
        require(abs(value - expected) < 1e-6, "Unexpected scalar value: " + name)
        actual_scalars[name] = value
    # Enumerate only known valid descriptors; hidden material enum entries can
    # have null descriptors in the editor API.
    absent = ("MP_EMISSIVE_COLOR", "MP_OPACITY", "MP_OPACITY_MASK", "MP_NORMAL",
              "MP_WORLD_POSITION_OFFSET", "MP_ANISOTROPY", "MP_TANGENT",
              "MP_SUBSURFACE_COLOR", "MP_AMBIENT_OCCLUSION", "MP_REFRACTION",
              "MP_MATERIAL_ATTRIBUTES", "MP_FRONT_MATERIAL")
    for name in absent:
        require(edit.get_material_property_input_node(material, getattr(unreal.MaterialProperty, name)) is None,
                "Forbidden extra material connection: " + name)
    return {"expressionCount": len(nodes), "codeSha256": SIGNATURE,
            "normalInput": "PixelNormalWS", "scalars": actual_scalars,
            "blendMode": "BLEND_OPAQUE", "shadingModel": "MSM_DEFAULT_LIT", "twoSided": True,
            "grassLinearRGB": [0.055, 0.105, 0.035], "rockLinearRGB": [0.073, 0.081, 0.092],
            "slopeTransitionDegrees": [20, 65], "absentProperties": list(absent)}


def main():
    saved = Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_saved_dir())).resolve()
    directory = saved / "TerrainPalette" / (datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S-%f") + "-highlands")
    directory.mkdir(parents=True, exist_ok=False)
    receipt = {"schema": "vibecoaster-terrain-palette-1", "asset": ASSET,
               "status": "started", "runtimePromoted": False, "visualReview": "pending"}
    try:
        library = unreal.EditorAssetLibrary
        edit = unreal.MaterialEditingLibrary
        existed = library.does_asset_exist(ASSET)
        receipt["existingAsset"] = existed
        if existed:
            material = library.load_asset(ASSET)
            receipt["readback"] = validate(material)
            # Read-only repeat: no recompile/resave that could change existing bytes.
            receipt["status"] = "existing-graph-validated"
        else:
            material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
                NAME, "/Game/Materials", unreal.Material, unreal.MaterialFactoryNew())
            require(material, "Could not create relief material")
            material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)
            material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
            material.set_editor_property("two_sided", True)
            base = edit.create_material_expression(material, unreal.MaterialExpressionCustom, -200, 0)
            base.set_editor_property("code", CODE)
            base.set_editor_property("description", "Green highland-to-rock slope palette; no terrain displacement")
            base.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT3)
            item = unreal.CustomInput()
            item.set_editor_property("input_name", "SurfaceNormal")
            base.set_editor_property("inputs", [item])
            normal = edit.create_material_expression(material, unreal.MaterialExpressionPixelNormalWS, -500, 0)
            require(edit.connect_material_expressions(normal, "", base, "SurfaceNormal"), "Normal connection failed")
            require(edit.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR), "Base connection failed")
            for index, (name, value) in enumerate(SCALARS.items()):
                node = edit.create_material_expression(material, unreal.MaterialExpressionConstant, -200, 180 + index * 100)
                node.set_editor_property("r", value)
                require(edit.connect_material_property(node, "", getattr(unreal.MaterialProperty, name)),
                        "Scalar connection failed: " + name)
            errors = edit.recompile_material(material)
            require(not errors, "Shader compiler returned errors: " + str(errors))
            receipt["readback"] = validate(material)
            require(library.save_loaded_asset(material, only_if_is_dirty=False), "Relief material save failed")
            receipt["status"] = "created-and-graph-validated"
        asset_file = Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_content_dir())) / "Materials" / (NAME + ".uasset")
        require(asset_file.is_file(), "Saved material file is missing")
        receipt["assetSha256"] = hashlib.sha256(asset_file.read_bytes()).hexdigest()
        receipt["assetFile"] = str(asset_file)
        unreal.log("VIBECOASTER_TERRAIN_PALETTE_OK " + str(directory / "receipt.json"))
    except Exception:
        receipt["status"] = "failed"
        receipt["error"] = traceback.format_exc()
        raise
    finally:
        (directory / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
