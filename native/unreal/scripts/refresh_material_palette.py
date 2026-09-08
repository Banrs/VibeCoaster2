"""Explicit, guarded migration of the two original generated palette materials.

Run in a separate UE editor process after closing other asset editors/builds.
Normal create_content.py deliberately preserves every existing material.
This migration refuses changed input graphs/values and dirty packages; it does
not overwrite an already refreshed material. Backups and receipts are unique
under Saved/Presentation. A failed save may require restoring those backups
with the editor closed; no failed receipt is presented as success.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import unreal


# Linear material values, not display/sRGB swatches.
PALETTE = {
    "M_Rail": ((0.48, 0.57, 0.62), 0.85, 0.3,
               (0.52, 0.055, 0.02), 0.45, 0.32),
    "M_Ground": ((0.18, 0.23, 0.12), 0.0, 0.95,
                 (0.04, 0.07, 0.028), 0.0, 0.95),
}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def near(actual, expected):
    return math.isfinite(actual) and abs(actual - expected) <= 1e-6


def inspect_material(name, expected):
    path = "/Game/Materials/" + name
    material = unreal.EditorAssetLibrary.load_asset(path)
    require(isinstance(material, unreal.Material), "Expected existing material: " + path)
    edit = unreal.MaterialEditingLibrary
    require(edit.get_num_material_expressions(material) == 3,
            "Unexpected expression count: " + path)
    for flag in ("two_sided", "used_with_instanced_static_meshes"):
        require(material.get_editor_property(flag) is True, "Unexpected " + flag + ": " + path)
    properties = (unreal.MaterialProperty.MP_BASE_COLOR,
                  unreal.MaterialProperty.MP_METALLIC,
                  unreal.MaterialProperty.MP_ROUGHNESS)
    nodes = [edit.get_material_property_input_node(material, prop) for prop in properties]
    require(isinstance(nodes[0], unreal.MaterialExpressionConstant3Vector)
            and all(isinstance(node, unreal.MaterialExpressionConstant) for node in nodes[1:])
            and nodes[1] != nodes[2], "Unexpected constant input graph: " + path)
    require(all(edit.get_material_property_input_node_output_name(material, prop) == ""
                for prop in properties), "Unexpected channel selection: " + path)
    # These properties all have non-null input descriptors in UE 5.8 Material.cpp.
    # Do not blindly enumerate MP_MAX/hidden enum values: the editor API dereferences
    # their missing descriptors without a null check.
    for prop_name in ("MP_EMISSIVE_COLOR", "MP_OPACITY", "MP_OPACITY_MASK", "MP_SPECULAR",
                      "MP_ANISOTROPY", "MP_NORMAL", "MP_TANGENT", "MP_WORLD_POSITION_OFFSET",
                      "MP_SUBSURFACE_COLOR", "MP_AMBIENT_OCCLUSION", "MP_REFRACTION",
                      "MP_MATERIAL_ATTRIBUTES", "MP_FRONT_MATERIAL"):
        prop = getattr(unreal.MaterialProperty, prop_name)
        require(edit.get_material_property_input_node(material, prop) is None,
                "Unexpected extra input " + prop_name + ": " + path)
    colour = nodes[0].get_editor_property("constant")
    actual_colour = (colour.r, colour.g, colour.b, colour.a)
    actual = (actual_colour, nodes[1].get_editor_property("r"), nodes[2].get_editor_property("r"))
    require(all(near(a, b) for a, b in zip(actual_colour, (*expected[0], 1.0)))
            and near(actual[1], expected[1]) and near(actual[2], expected[2]),
            "Material differs from original bootstrap; refusing refresh: " + path)
    return material, nodes, actual


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    # Preflight BOTH materials before creating any backup or mutating either.
    inspected = {name: inspect_material(name, values[:3]) for name, values in PALETTE.items()}
    dirty = {package.get_path_name() for package in
             unreal.EditorLoadingAndSavingUtils.get_dirty_content_packages()}
    require(not dirty.intersection("/Game/Materials/" + name for name in PALETTE),
            "Palette materials have unsaved changes; refusing refresh")
    content = Path(unreal.Paths.project_content_dir()).resolve()
    sources = {name: content / "Materials" / (name + ".uasset") for name in PALETTE}
    require(all(path.is_file() for path in sources.values()), "Missing source .uasset")
    run = Path(unreal.Paths.project_saved_dir()).resolve() / "Presentation" / (
        datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S-%f") + "-palette")
    run.mkdir(parents=True, exist_ok=False)
    receipt = {"status": "backing_up", "materials": {}, "saved": []}
    receipt_path = run / "palette-refresh.json"
    try:
        for name, source in sources.items():
            backups = []
            # Normally editor assets are single files; preserve any external payload too.
            for suffix in (".uasset", ".uexp", ".ubulk", ".uptnl"):
                file = source.with_suffix(suffix)
                if file.is_file():
                    target = run / file.name
                    before = digest(file)
                    shutil.copy2(file, target)
                    require(before == digest(target) == digest(file), "Backup verification failed: " + str(file))
                    backups.append({"source": str(file), "backup": str(target), "sha256": before})
            receipt["materials"][name] = {
                "before": inspected[name][2], "after": PALETTE[name][3:], "backups": backups}
        receipt["status"] = "backed_up"
        receipt_path.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
        # Only the three verified constants change. Other material settings stay intact.
        for name, (material, nodes, _) in inspected.items():
            colour, metallic, roughness = PALETTE[name][3:]
            nodes[0].set_editor_property("constant", unreal.LinearColor(*colour, 1.0))
            nodes[1].set_editor_property("r", metallic)
            nodes[2].set_editor_property("r", roughness)
            unreal.MaterialEditingLibrary.recompile_material(material)
            require(unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False),
                    "Save failed: " + name)
            inspect_material(name, PALETTE[name][3:])
            receipt["saved"].append(name)
            receipt["materials"][name]["saved_sha256"] = digest(sources[name])
            receipt_path.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
        receipt["status"] = "refreshed"
        receipt_path.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
        unreal.log("VIBECOASTER_PALETTE_REFRESH_OK " + str(receipt_path))
    except Exception as error:
        receipt["status"] = "failed"
        receipt["error"] = str(error)
        receipt_path.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
        raise


if __name__ == "__main__":
    main()
