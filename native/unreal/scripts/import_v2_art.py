"""Import the Blender MCP kit with explicit metre-to-centimetre contracts.

Run inside UnrealEditor-Cmd with -ExecutePythonScript. FBX metre units are converted by the importer; convert_scene is disabled so
local +X stays forward.
"""
import json
from pathlib import Path
import unreal

source = Path(__file__).resolve().parents[2] / "art" / "exports"
manifest = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
required_runtime_assets = {
    "SM_LeadCar", "SM_TrainCar", "SM_TrackTieWeb", "SM_LSMStator", "SM_BrakeFin",
    "SM_StationPlatformPanel", "SM_StationPlatformEndPanel",
    "SM_StationRoofPanel", "SM_StationPost",
    "SM_StationQueueDeck", "SM_StationRouteRoof", "SM_StationMergeDeck",
    "SM_StationHoldingLane", "SM_StationBoardingGate", "SM_StationDispatchCabin",
    "SM_StationUnloadDeck", "SM_StationExitWalkway", "SM_StationLift",
    "SM_StationStair", "SM_StationUnderpass", "SM_StationQueueRail",
}
runtime_assets = {entry["name"] for entry in manifest["assets"] if entry.get("runtime", True)}
missing_assets = required_runtime_assets - runtime_assets
if missing_assets:
    raise RuntimeError("Functional runtime art is incomplete: " + ", ".join(sorted(missing_assets)))
root = "/Game/Art/V3"
library = unreal.EditorAssetLibrary
library.make_directory(root + "/Materials")
tools = unreal.AssetToolsHelpers.get_asset_tools()
edit = unreal.MaterialEditingLibrary
palette = {
    "VC2_Petrol": ((.018,.105,.13),.62,.29),
    "VC2_Pearl": ((.57,.64,.65),.4,.27),
    "VC2_Graphite": ((.022,.031,.035),.45,.42),
    "VC2_Copper": ((.83,.265,.045),.46,.34),
    "VC2_Steel": ((.31,.38,.41),.78,.3),
    "VC2_Padding": ((.016,.024,.029),0,.82),
    "VC2_Concrete": ((.38,.395,.36),0,.88),
    "VC2_Light": ((.59,.88,.93),0,.29),
    "VC2_Glass": ((.43,.69,.76),0,.08),
}
materials = {}
for name, (colour, metallic, roughness) in palette.items():
    path = root + "/Materials/M_" + name
    existing = library.does_asset_exist(path)
    material = library.load_asset(path) if existing else tools.create_asset(
        "M_" + name, root + "/Materials", unreal.Material, unreal.MaterialFactoryNew())
    if not isinstance(material, unreal.Material):
        raise RuntimeError("V3 material asset has the wrong type: " + name)
    blend = unreal.BlendMode.BLEND_TRANSLUCENT if name == "VC2_Glass" else unreal.BlendMode.BLEND_OPAQUE
    if existing:
        # UE 5.8's DeleteAllMaterialExpressions marks old expressions as garbage.
        # Some loaded expressions are rooted during editor startup. Update the
        # known constant graph in place, then compile its ISM shader usage.
        def input_node(prop, expected_type):
            node = edit.get_material_property_input_node(material, prop)
            if not isinstance(node, expected_type):
                raise RuntimeError("Existing V3 material input changed: " + name + " " + str(prop))
            return node
        # Extra unconnected expressions are legal in an existing editor graph.
        # The connected inputs below define the actual runtime surface.
        unreal.log("VIBECOASTER_MATERIAL_GRAPH " + name + " nodes=" +
                   str(edit.get_num_material_expressions(material)))
        base = input_node(unreal.MaterialProperty.MP_BASE_COLOR, unreal.MaterialExpressionConstant3Vector)
        unreal.log("VIBECOASTER_MATERIAL_INPUT " + name + " base=" +
                   str(base.get_editor_property("constant")))
        base.set_editor_property("constant", unreal.LinearColor(*colour, 1))
        for value, prop in ((metallic, unreal.MaterialProperty.MP_METALLIC),
                            (roughness, unreal.MaterialProperty.MP_ROUGHNESS)):
            scalar = input_node(prop, unreal.MaterialExpressionConstant)
            unreal.log("VIBECOASTER_MATERIAL_INPUT " + name + " " + str(prop) +
                       "=" + str(scalar.get_editor_property("r")))
            scalar.set_editor_property("r", value)
        if name == "VC2_Glass":
            input_node(unreal.MaterialProperty.MP_OPACITY, unreal.MaterialExpressionConstant).set_editor_property("r", .10)
        if name == "VC2_Light":
            input_node(unreal.MaterialProperty.MP_EMISSIVE_COLOR,
                       unreal.MaterialExpressionConstant3Vector).set_editor_property(
                           "constant", unreal.LinearColor(*colour, 1))
    else:
        base = edit.create_material_expression(material, unreal.MaterialExpressionConstant3Vector, -400, 0)
        base.set_editor_property("constant", unreal.LinearColor(*colour, 1))
        if not edit.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR):
            raise RuntimeError("Could not connect V3 material base colour: " + name)
        for value, prop, y in ((metallic, unreal.MaterialProperty.MP_METALLIC, 160),
                               (roughness, unreal.MaterialProperty.MP_ROUGHNESS, 320)):
            constant = edit.create_material_expression(material, unreal.MaterialExpressionConstant, -400, y)
            constant.set_editor_property("r", value)
            if not edit.connect_material_property(constant, "", prop):
                raise RuntimeError("Could not connect V3 material scalar: " + name + " " + str(prop))
        if name == "VC2_Glass":
            opacity = edit.create_material_expression(material, unreal.MaterialExpressionConstant, -400, 480)
            opacity.set_editor_property("r", .10)
            if not edit.connect_material_property(opacity, "", unreal.MaterialProperty.MP_OPACITY):
                raise RuntimeError("Could not connect V3 glass opacity")
        if name == "VC2_Light":
            emission = edit.create_material_expression(material, unreal.MaterialExpressionConstant3Vector, -400, 600)
            emission.set_editor_property("constant", unreal.LinearColor(*colour, 1))
            if not edit.connect_material_property(emission, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR):
                raise RuntimeError("Could not connect V3 material emission")
    material.set_editor_property("blend_mode", blend)
    material.set_editor_property("two_sided", name == "VC2_Glass")
    if name == "VC2_Glass":
        material.set_editor_property("translucency_lighting_mode", unreal.TranslucencyLightingMode.TLM_SURFACE_PER_PIXEL_LIGHTING)
    material.set_editor_property("used_with_instanced_static_meshes", True)
    errors = edit.recompile_material(material)
    if errors:
        raise RuntimeError("Material compile failed: " + name + " " + str(errors))
    if not library.save_loaded_asset(material, only_if_is_dirty=False):
        raise RuntimeError("Material save failed: " + name)
    materials[name] = material

receipt = []
for asset in manifest["assets"]:
    if not asset.get("runtime",True):
        continue
    options = unreal.FbxImportUI()
    options.set_editor_property("import_mesh",True)
    options.set_editor_property("import_materials",False)
    options.set_editor_property("import_textures",False)
    options.set_editor_property("import_as_skeletal",False)
    options.set_editor_property("mesh_type_to_import",unreal.FBXImportType.FBXIT_STATIC_MESH)
    data = options.static_mesh_import_data
    data.set_editor_property("combine_meshes",True)
    data.set_editor_property("auto_generate_collision",False)
    data.set_editor_property("generate_lightmap_u_vs",False)
    data.set_editor_property("convert_scene",False)
    data.set_editor_property("convert_scene_unit",False)
    data.set_editor_property("force_front_x_axis",False)
    data.set_editor_property("import_uniform_scale",1.0)
    data.set_editor_property("normal_import_method",unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS)
    task = unreal.AssetImportTask()
    task.set_editor_property("filename",str(source / (asset["name"] + ".fbx")))
    task.set_editor_property("destination_path",root)
    task.set_editor_property("destination_name",asset["name"])
    task.set_editor_property("replace_existing",True)
    task.set_editor_property("replace_existing_settings",True)
    task.set_editor_property("automated",True)
    task.set_editor_property("save",False)
    task.set_editor_property("options",options)
    tools.import_asset_tasks([task])
    mesh = library.load_asset(root + "/" + asset["name"])
    if not isinstance(mesh,unreal.StaticMesh):
        raise RuntimeError("Static mesh import missing: " + asset["name"])
    slots = mesh.get_editor_property("static_materials")
    actual_slots=[str(slot.get_editor_property("imported_material_slot_name")) for slot in slots]
    if sorted(actual_slots) != sorted(asset["materials"]):
        raise RuntimeError("Imported material slots differ from source manifest: " + asset["name"])
    for index, slot in enumerate(slots):
        name = str(slot.get_editor_property("imported_material_slot_name"))
        if name not in materials:
            raise RuntimeError("Unexpected source material slot: " + name)
        mesh.set_material(index,materials[name])
    mesh.set_editor_property("allow_cpu_access",False)
    bounds = mesh.get_bounding_box()
    actual = [[bounds.min.x,bounds.min.y,bounds.min.z],[bounds.max.x,bounds.max.y,bounds.max.z]]
    expected = [[value*100 for value in side] for side in asset["boundsMetres"]]
    error = max(abs(a-b) for side_a,side_b in zip(actual,expected) for a,b in zip(side_a,side_b))
    if error > .025:
        raise RuntimeError("Imported pivot/axes/units differ: " + asset["name"] + " " + str(actual) + " expected " + str(expected))
    if not library.save_loaded_asset(mesh,only_if_is_dirty=False):
        raise RuntimeError("Could not save " + asset["name"])
    receipt.append({"asset":root + "/" + asset["name"],"boundsCm":actual,
                    "materialSlots":actual_slots,"maximumBoundErrorCm":error})

destination = Path(unreal.Paths.project_saved_dir()) / "V3ArtImport.json"
destination.write_text(json.dumps({"source":manifest["createdThrough"],"assets":receipt},indent=2),encoding="utf-8")
unreal.log("VIBECOASTER_V3_ART_IMPORT_OK " + str(destination))
