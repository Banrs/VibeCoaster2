"""Import the Blender MCP Exa study and build its isolated Unreal review map.

Run in a separate Unreal editor process with -ExecutePythonScript. This writes
only /Game/Art/TrackStudy and an out/track-study receipt; it does not select a
playable profile or change the saved default.3 track/clearance contract.
"""
import json
import time
from pathlib import Path
import unreal

repo = Path(__file__).resolve().parents[3]
source = repo / "native/art/exports/track-study"
manifest = json.loads((source / "manifest.json").read_text(encoding="utf-8"))
root = "/Game/Art/TrackStudy"
library = unreal.EditorAssetLibrary
asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
edit = unreal.MaterialEditingLibrary
library.make_directory(root+"/Materials")
library.make_directory(root+"/Maps")
materials = {}
for name, spec in manifest["profile"]["palette"].items():
    path = root+"/Materials/M_"+name
    material = library.load_asset(path) if library.does_asset_exist(path) else asset_tools.create_asset(
        "M_"+name, root+"/Materials", unreal.Material, unreal.MaterialFactoryNew())
    if not isinstance(material, unreal.Material):
        raise RuntimeError("Study material missing: "+name)
    for prop, value, node_type, y in (
        (unreal.MaterialProperty.MP_BASE_COLOR, spec["colour"], unreal.MaterialExpressionConstant3Vector, 0),
        (unreal.MaterialProperty.MP_METALLIC, spec["metallic"], unreal.MaterialExpressionConstant, 160),
        (unreal.MaterialProperty.MP_ROUGHNESS, spec["roughness"], unreal.MaterialExpressionConstant, 320)):
        node = edit.get_material_property_input_node(material, prop)
        if node is None:
            node = edit.create_material_expression(material, node_type, -400, y)
            if not edit.connect_material_property(node, "", prop):
                raise RuntimeError("Could not connect study material")
        if not isinstance(node, node_type):
            raise RuntimeError("Unexpected study material graph: "+name)
        if isinstance(value, list):
            node.set_editor_property("constant", unreal.LinearColor(*value, 1))
        else:
            node.set_editor_property("r", value)
    material.set_editor_property("two_sided", False)
    material.set_editor_property("used_with_instanced_static_meshes", True)
    errors = edit.recompile_material(material)
    if errors:
        raise RuntimeError("Study material failed compilation: "+str(errors))
    library.save_loaded_asset(material, only_if_is_dirty=False)
    materials[name] = material

meshes, receipt = {}, []
for asset in manifest["assets"]:
    options = unreal.FbxImportUI()
    options.set_editor_property("import_mesh", True)
    options.set_editor_property("import_materials", False)
    options.set_editor_property("import_textures", False)
    options.set_editor_property("import_as_skeletal", False)
    options.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    data = options.static_mesh_import_data
    for key, value in {"combine_meshes": True, "auto_generate_collision": False,
                       "generate_lightmap_u_vs": False, "convert_scene": False,
                       "convert_scene_unit": False, "force_front_x_axis": False,
                       "import_uniform_scale": 1.0}.items():
        data.set_editor_property(key, value)
    data.set_editor_property("normal_import_method", unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS)
    task = unreal.AssetImportTask()
    for key, value in {"filename": str(source/(asset["name"]+".fbx")),
        "destination_path": root, "destination_name": asset["name"], "replace_existing": True,
        "replace_existing_settings": True, "automated": True, "save": False, "options": options}.items():
        task.set_editor_property(key, value)
    asset_tools.import_asset_tasks([task])
    mesh = library.load_asset(root+"/"+asset["name"])
    if not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError("Missing imported Exa asset: "+asset["name"])
    slots = mesh.get_editor_property("static_materials")
    actual_slots = [str(slot.get_editor_property("imported_material_slot_name")) for slot in slots]
    if sorted(actual_slots) != sorted(asset["materials"]):
        raise RuntimeError("Material slots differ: "+asset["name"])
    for index, name in enumerate(actual_slots):
        mesh.set_material(index, materials[name])
    bounds = mesh.get_bounding_box()
    actual = [[bounds.min.x, bounds.min.y, bounds.min.z], [bounds.max.x, bounds.max.y, bounds.max.z]]
    error = max(abs(a-100*b) for aa, bb in zip(actual, asset["boundsMetres"]) for a, b in zip(aa, bb))
    if error > .025:
        raise RuntimeError("Exa import units/axes mismatch: "+asset["name"]+" "+str(error))
    if not library.save_loaded_asset(mesh, only_if_is_dirty=False):
        raise RuntimeError("Cannot save study mesh")
    meshes[asset["name"]] = mesh
    receipt.append({"asset": root+"/"+asset["name"], "boundsCm": actual,
                    "maximumBoundErrorCm": error, "materialSlots": actual_slots})

level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
map_path = root+"/Maps/ExaTrackStudy"
if library.does_asset_exist(map_path):
    if not level.load_level(map_path):
        raise RuntimeError("Cannot load the existing study map")
    # Keep any manual review additions. Only replace this script's actors.
    for actor in actors.get_all_level_actors():
        if "ExaStudyGenerated" in [str(tag) for tag in actor.tags]:
            actors.destroy_actor(actor)
else:
    if not level.new_level(map_path):
        raise RuntimeError("Cannot create study map")
world = unreal.EditorLevelLibrary.get_editor_world()
world.get_world_settings().set_editor_property("default_game_mode", unreal.GameModeBase)


def spawn(cls, name, position, rotation=unreal.Rotator()):
    actor = actors.spawn_actor_from_class(cls, unreal.Vector(*(v*100 for v in position)), rotation)
    actor.set_actor_label(name)
    actor.tags = ["ExaStudyGenerated"]
    return actor


def mesh_actor(name, mesh, position, scale=(1, 1, 1), material=None):
    actor = spawn(unreal.StaticMeshActor, name, position)
    component = actor.static_mesh_component
    component.set_static_mesh(mesh)
    actor.set_actor_scale3d(unreal.Vector(*scale))
    if material is not None:
        component.set_material(0, material)
    return actor


for study in manifest["studies"]:
    mesh_actor("Exa / "+study["kind"], meshes[study["asset"]], study["offsetMetres"])
cube = library.load_asset("/Engine/BasicShapes/Cube")
cylinder = library.load_asset("/Engine/BasicShapes/Cylinder")
mount_z = manifest["checks"]["supportMountLocalMetres"][2]
base_size = manifest["profile"]["support_base_size"]
column_diameter = 2*manifest["profile"]["support_column_radius"]
column_bottom = base_size[2]-.01
for distance in (2.8, 8.4):
    x, y, z = distance-5.6, -10, 2.8
    mesh_actor("Exa / support head", meshes["SM_ExaSupportHead"], (x, y, z))
    height = z+mount_z+.005-column_bottom
    mesh_actor("Exa / short review column", cylinder, (x, y, column_bottom+height/2),
               (column_diameter, column_diameter, height), materials["TS_Support"])
    mesh_actor("Exa / review stand", cube, (x, y, base_size[2]/2), base_size, materials["TS_Support"])
mesh_actor("Exa / removable wheel fit", meshes["SM_ExaWheelFit"], (-3.5, -10, 2.8))
mesh_actor("Exa / studio floor", cube, (0, 0, -.15), (150, 150, .30), materials["TS_Floor"])

sun = spawn(unreal.DirectionalLight, "Exa / sunlight", (0, 0, 20), unreal.Rotator(-40, -35, 0))
sun.light_component.set_editor_property("intensity", 4.0)
sun.light_component.set_editor_property("light_source_angle", 5.0)
sun.light_component.set_editor_property("atmosphere_sun_light", True)
spawn(unreal.SkyAtmosphere, "Exa / atmosphere", (0, 0, 0))
sky = spawn(unreal.SkyLight, "Exa / sky", (0, 0, 10))
sky.light_component.set_editor_property("real_time_capture", True)
sky.light_component.set_editor_property("intensity", 1.0)
post = spawn(unreal.PostProcessVolume, "Exa / exposure", (0, 0, 0))
post.set_editor_property("unbound", True)
settings = post.get_editor_property("settings")
settings.set_editor_property("override_auto_exposure_min_brightness", True)
settings.set_editor_property("override_auto_exposure_max_brightness", True)
settings.set_editor_property("auto_exposure_min_brightness", 1.0)
settings.set_editor_property("auto_exposure_max_brightness", 1.0)
post.set_editor_property("settings", settings)

cameras = []
for name, position, target, fov in (
    ("Hero", (-14, -20, 8), (0, -10, 2.3), 50),
    ("Overview", (-33, -38, 31), (0, 2, 3), 60),
    ("Banked", (-22, -20, 15), (0, .6, 3), 55),
    ("Inverted", (-30, -17, 15), (0, 10, 3.6), 58),
    ("Rider", (-10, -2, 4.8), (7, 1.6, 3.6), 82)):
    rotation = unreal.MathLibrary.find_look_at_rotation(unreal.Vector(*position), unreal.Vector(*target))
    camera = spawn(unreal.CameraActor, "Exa / Camera / "+name, position, rotation)
    camera.camera_component.set_editor_property("field_of_view", float(fov))
    cameras.append(camera)
unreal.EditorLevelLibrary.set_level_viewport_camera_info(cameras[1].get_actor_location(), cameras[1].get_actor_rotation())
if not level.save_current_level():
    raise RuntimeError("Could not save Exa study map")
destination = repo / "out/track-study/unreal-import.json"
destination.write_text(json.dumps({"source": manifest["createdThrough"], "map": map_path,
    "profileId": manifest["profile"]["id"], "railRadiusMetres": manifest["profile"]["rail_radius"],
    "supportColumnDiameterMetres": column_diameter, "crossheadAxisHeightMetres": manifest["profile"]["crosshead_height"],
    "supportConnection": manifest["profile"]["support_connection"],
    "assets": receipt, "cameras": [a.get_actor_label() for a in cameras]}, indent=2)+"\n")
unreal.log("EXA_TRACK_STUDY_IMPORT_OK "+str(destination))
# Let the editor finish opening its viewport before shutdown. Quitting inside
# the startup script can tear down its mode manager while Slate creates it.
quit_at = time.monotonic()+12
def finish_import(delta):
    if time.monotonic() >= quit_at:
        unreal.unregister_slate_post_tick_callback(finish_handle)
        unreal.SystemLibrary.quit_editor()
finish_handle = unreal.register_slate_post_tick_callback(finish_import)
unreal.EditorPythonScripting.set_keep_python_script_alive(True)
