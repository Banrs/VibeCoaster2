"""UE editor-only bootstrap, creating actual assets through Unreal APIs.
Existing map/material assets are preserved so local edits are not overwritten.
"""
import json
import os
import runpy
from pathlib import Path
import unreal


def make_material(name, colour, metallic, roughness):
    path = "/Game/Materials/" + name
    library = unreal.EditorAssetLibrary
    if library.does_asset_exist(path):
        material = library.load_asset(path)
        if not isinstance(material, unreal.Material):
            raise RuntimeError("Expected material asset: " + path)
        return
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name, "/Game/Materials", unreal.Material, unreal.MaterialFactoryNew()
    )
    if not material:
        raise RuntimeError("Could not create " + path)
    material.set_editor_property("two_sided", True)
    material.set_editor_property("used_with_instanced_static_meshes", True)
    edit = unreal.MaterialEditingLibrary
    base = edit.create_material_expression(material, unreal.MaterialExpressionConstant3Vector, -400, 0)
    base.set_editor_property("constant", unreal.LinearColor(*colour, 1.0))
    if not edit.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR):
        raise RuntimeError("Base colour connection failed: " + path)
    for value, property_, y in [
        (metallic, unreal.MaterialProperty.MP_METALLIC, 150),
        (roughness, unreal.MaterialProperty.MP_ROUGHNESS, 300),
    ]:
        expression = edit.create_material_expression(material, unreal.MaterialExpressionConstant, -400, y)
        expression.set_editor_property("r", value)
        if not edit.connect_material_property(expression, "", property_):
            raise RuntimeError("Material connection failed: " + path)
    edit.recompile_material(material)
    if not library.save_loaded_asset(material, only_if_is_dirty=False):
        raise RuntimeError("Save failed: " + path)


def main():
    make_material("M_Rail", (0.52, 0.055, 0.02), 0.45, 0.32)
    make_material("M_Structure", (0.13, 0.19, 0.23), 0.6, 0.48)
    make_material("M_Footing", (0.42, 0.40, 0.36), 0.0, 0.95)
    make_material("M_LSM", (0.48, 0.19, 0.055), 0.85, 0.32)
    make_material("M_Brake", (0.44, 0.50, 0.57), 0.8, 0.28)
    make_material("M_Train", (0.6, 0.08, 0.03), 0.4, 0.35)
    # Separate texture-free relief asset; the script validates existing graphs.
    runpy.run_path(str(Path(__file__).with_name("create_terrain_palette.py")), run_name="__main__")
    subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if not unreal.EditorAssetLibrary.does_asset_exist("/Game/Maps/Ride"):
        if not subsystem.new_level("/Game/Maps/Ride"):
            raise RuntimeError("Could not create /Game/Maps/Ride")
        if not subsystem.save_current_level():
            raise RuntimeError("Could not save /Game/Maps/Ride")
    # Completion receipt only; not an Unreal asset or a visual validation claim.
    receipt = Path(os.environ.get("VIBECOASTER_BOOTSTRAP_RECEIPT") or
                   (Path(unreal.Paths.project_saved_dir()) / "ContentBootstrap.json"))
    receipt.parent.mkdir(parents=True, exist_ok=True)
    receipt.write_text(json.dumps({"map": "/Game/Maps/Ride", "materials": 7, "created_through": "UE editor Python"}), encoding="utf-8")
    unreal.log("VIBECOASTER_CONTENT_BOOTSTRAP_OK")


main()

