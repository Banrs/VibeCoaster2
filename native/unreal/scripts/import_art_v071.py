"""UE5.8 editor-only art import and geometry receipt. Never changes runtime code.
Set VIBECOASTER_ART_IMPORT_SPEC to a project-local JSON spec. Every run requires a
fresh /Game/Art/V071 destination (or fresh child for a preserved retry) and receipt directory. Explicit FbxFactory
uses the locally inspected legacy FBX API; no global Interchange settings change.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import re
import traceback
import unreal

PROJECT = Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())).resolve()
ART = (PROJECT.parent / "art").resolve()
ASSET_NAMES = (
    "SM_StationPlatformPanel", "SM_StationPlatformEndPanel_1m", "SM_StationRoofPanel",
    "SM_StationPost", "SM_TrackTie", "REVIEW_CoordinateWitness",
)
ENV_PALETTE = {
    "Graphite": ([.065, .085, .11], .45, .34),
    "BlueSteel": ([.13, .24, .28], .55, .31),
    "MachinedSteel": ([.42, .48, .53], .82, .28),
    "DarkRecess": ([.022, .028, .034], .22, .5),
    "Concrete": ([.43, .415, .38], 0, .81),
    "ConcreteEdge": ([.32, .315, .295], 0, .8),
    "SafetyOchre": ([.69, .40, .065], .2, .52),
    "LightDiffuser": ([.73, .79, .77], .05, .34),
    "ReviewCoral": ([.51, .13, .065], .45, .38),
}
TRAIN_PALETTE = {
    "Paint_WarmOrange": ("EC6928", .32, .27),
    "Paint_BurntOrange": ("AB431E", .25, .34),
    "Padding_Graphite": ("272D32", .03, .72),
    "Rubber_GripAndTyre": ("121619", 0, .82),
    "Metal_BrushedAluminium": ("ABB4BB", .85, .32),
    "Metal_DarkChassis": ("353F48", .72, .4),
    "Trim_Ivory": ("EEE5D4", .18, .32),
}
TOLERANCE_CM = .02  # 0.2 mm, covering FBX float precision without hiding scale errors.


def bounded_file(value, root):
    path = Path(value).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise RuntimeError("Missing or out-of-scope file: " + str(path))
    return path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(value):
    return [float(value.x), float(value.y), float(value.z)]


def bounds(points):
    if not points or not all(math.isfinite(v) for p in points for v in p):
        raise RuntimeError("Empty or non-finite imported geometry")
    return [[min(p[a] for p in points) for a in range(3)], [max(p[a] for p in points) for a in range(3)]]


def ue_bounds(source):
    lo, hi = source
    return [[100*lo[0], -100*hi[1], 100*lo[2]], [100*hi[0], -100*lo[1], 100*hi[2]]]


def match_bounds(actual, expected, context):
    error = max(abs(actual[e][a] - expected[e][a]) for e in range(2) for a in range(3))
    if error > TOLERANCE_CM:
        raise RuntimeError(context + ": expected cm " + str(expected) + ", got " + str(actual) + ", error " + str(error))
    return error


def linear_hex(text):
    values = [int(text[i:i+2], 16)/255 for i in (0, 2, 4)]
    return [v/12.92 if v <= .04045 else ((v+.055)/1.055)**2.4 for v in values]


def palette(name):
    if name.startswith("VC_ENVKIT_") and name[10:] in ENV_PALETTE:
        return ENV_PALETTE[name[10:]]
    if name.startswith("VCTrain_") and name[8:] in TRAIN_PALETTE:
        color, metallic, roughness = TRAIN_PALETTE[name[8:]]
        return linear_hex(color), metallic, roughness
    raise RuntimeError("Unrecognized authored material slot: " + name)


def make_material(slot_name, destination, cache):
    if slot_name in cache:
        return cache[slot_name]
    color, metallic, roughness = palette(slot_name)
    name = "M_" + slot_name
    path = destination + "/Materials/" + name
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        raise RuntimeError("Refusing material overwrite: " + path)
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(name, destination + "/Materials", unreal.Material, unreal.MaterialFactoryNew())
    if not material:
        raise RuntimeError("Material creation failed: " + path)
    material.set_editor_property("used_with_instanced_static_meshes", True)
    material.set_editor_property("two_sided", False)
    edit = unreal.MaterialEditingLibrary
    expression = edit.create_material_expression(material, unreal.MaterialExpressionConstant3Vector, -400, 0)
    expression.set_editor_property("constant", unreal.LinearColor(*color, 1))
    if not edit.connect_material_property(expression, "", unreal.MaterialProperty.MP_BASE_COLOR):
        raise RuntimeError("Could not connect base color")
    for value, property_, y in ((metallic, unreal.MaterialProperty.MP_METALLIC, 160), (roughness, unreal.MaterialProperty.MP_ROUGHNESS, 320)):
        expression = edit.create_material_expression(material, unreal.MaterialExpressionConstant, -400, y)
        expression.set_editor_property("r", value)
        if not edit.connect_material_property(expression, "", property_):
            raise RuntimeError("Could not connect material scalar")
    edit.recompile_material(material)
    if not unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False):
        raise RuntimeError("Material save failed: " + path)
    cache[slot_name] = material
    return material


def import_mesh(item, destination):
    options = unreal.FbxImportUI()
    for key, value in {
        "import_mesh": True, "import_as_skeletal": False, "import_animations": False,
        "import_materials": False, "import_textures": False, "create_physics_asset": False,
        "automated_import_should_detect_type": False, "override_full_name": True,
        "mesh_type_to_import": unreal.FBXImportType.FBXIT_STATIC_MESH,
    }.items():
        options.set_editor_property(key, value)
    data = options.get_editor_property("static_mesh_import_data")
    for key, value in {
        "combine_meshes": True, "auto_generate_collision": False, "build_nanite": False,
        "generate_lightmap_u_vs": False, "remove_degenerates": True,
        "convert_scene": True, "convert_scene_unit": True, "force_front_x_axis": True,
        "import_uniform_scale": 1.0, "import_translation": unreal.Vector(0, 0, 0),
        "import_rotation": unreal.Rotator(pitch=0, yaw=90, roll=0), "transform_vertex_to_absolute": True,
        "bake_pivot_in_vertex": False,
        "normal_import_method": unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS,
    }.items():
        data.set_editor_property(key, value)
    task = unreal.AssetImportTask()
    for key, value in {
        "filename": item["fbx"], "destination_path": destination,
        "destination_name": item["name"], "replace_existing": False,
        "replace_existing_settings": False, "automated": True, "save": True,
        "factory": unreal.FbxFactory(), "options": options,
    }.items():
        task.set_editor_property(key, value)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    objects = list(task.get_objects())
    meshes = [obj for obj in objects if isinstance(obj, unreal.StaticMesh)]
    if len(meshes) != 1 or any(not isinstance(obj, unreal.StaticMesh) for obj in objects):
        raise RuntimeError("Expected exactly one static mesh, got: " + str([obj.get_path_name() for obj in objects]))
    mesh = meshes[0]
    expected = destination + "/" + item["name"]
    if mesh.get_path_name().split(".", 1)[0] != expected:
        raise RuntimeError("Unexpected imported asset path: " + mesh.get_path_name())
    if not isinstance(mesh.get_editor_property("asset_import_data"), unreal.FbxStaticMeshImportData):
        raise RuntimeError("Import did not use the inspected legacy FBX path")
    return mesh


def inspect_mesh(mesh, item, subsystem):
    subsystem.set_allow_cpu_access(mesh, True)
    static_materials = list(mesh.get_editor_property("static_materials"))
    slots = [str(slot.get_editor_property("imported_material_slot_name")) for slot in static_materials]
    if any(name == "None" or not name for name in slots):
        raise RuntimeError("Imported material slot lost its source name: " + str(slots))
    if set(slots) != set(item["materials"]):
        raise RuntimeError("Material slot mismatch: expected " + str(item["materials"]) + ", got " + str(slots))
    all_points, section_receipts, by_material = [], [], {}
    for section in range(mesh.get_num_sections(0)):
        vertices, triangles, normals, uvs, tangents = unreal.ProceduralMeshLibrary.get_section_from_static_mesh(mesh, 0, section)
        points = [vector(v) for v in vertices]
        if not points or len(triangles) % 3 or len(normals) != len(points):
            raise RuntimeError("Malformed imported section " + str(section))
        if any(i < 0 or i >= len(points) for i in triangles):
            raise RuntimeError("Out-of-range imported triangle index")
        if any(not math.isfinite(c) for n in normals for c in vector(n)):
            raise RuntimeError("Non-finite imported normal")
        slot_index = subsystem.get_lod_material_slot(mesh, 0, section)
        if slot_index < 0 or slot_index >= len(slots):
            raise RuntimeError("Invalid section material mapping")
        slot = slots[slot_index]
        by_material.setdefault(slot, []).extend(points)
        all_points.extend(points)
        section_receipts.append({"section": section, "materialSlot": slot, "vertices": len(points), "triangles": len(triangles)//3, "boundsCm": bounds(points)})
    actual = bounds(all_points)
    expected = ue_bounds(item["sourceBoundsM"])
    error = match_bounds(actual, expected, item["name"] + " source-to-UE bounds")
    sphere_box = mesh.get_bounds()
    origin, extent = vector(sphere_box.origin), vector(sphere_box.box_extent)
    asset_bounds = [[origin[a]-extent[a] for a in range(3)], [origin[a]+extent[a] for a in range(3)]]
    match_bounds(asset_bounds, actual, "Render bounds versus actual LOD vertices")
    witness = None
    if item["name"] == "REVIEW_CoordinateWitness":
        expected_sections = {
            "VC_ENVKIT_Graphite": [[-50, -50, 0], [50, 50, 100]],
            "VC_ENVKIT_SafetyOchre": [[44, -7, 6], [160, 7, 24]],
            "VC_ENVKIT_ReviewCoral": [[-7, 44, 6], [7, 160, 24]],
        }
        witness = {}
        for name, expected_section in expected_sections.items():
            actual_section = bounds(by_material.get(name, []))
            witness[name] = {"boundsCm": actual_section, "errorCm": match_bounds(actual_section, expected_section, "Handedness witness " + name)}
    collision_count = subsystem.get_simple_collision_count(mesh)
    if collision_count != 0:
        raise RuntimeError("Unexpected generated collision; numerical geometry remains authoritative")
    return {"assetPath": mesh.get_path_name(), "boundsCm": actual, "sourceBoundsM": item["sourceBoundsM"],
            "maximumBoundsErrorCm": error, "materialSlots": slots, "sections": section_receipts,
            "vertexCountWithSectionDuplicates": len(all_points), "collisionCount": collision_count,
            "coordinateWitness": witness, "unitScale": "1 metre -> 100 Unreal centimetres",
            "basis": "Source +X/-Y/+Z -> UE +X/+Y/+Z, independently checked by the asymmetric witness"}


def main():
    spec_file = bounded_file(os.environ.get("VIBECOASTER_ART_IMPORT_SPEC", ""), ART)
    spec = json.loads(spec_file.read_text(encoding="utf-8"))
    run = spec["run"]
    if not re.fullmatch(r"[A-Za-z0-9_]+", run):
        raise RuntimeError("Run identity must contain only letters, numbers and underscores")
    destination = spec.get("destinationRoot", "/Game/Art/V071")
    if not re.fullmatch(r"/Game/Art/V071(?:/[A-Za-z0-9_]+)?", destination):
        raise RuntimeError("Destination must be V071 or one fresh bounded child")
    content_destination = PROJECT / "Content" / destination.removeprefix("/Game/")
    if unreal.EditorAssetLibrary.does_directory_exist(destination) or content_destination.exists():
        raise RuntimeError("Fresh destination required; preserving existing import: " + destination)
    receipt_dir = PROJECT / "Saved" / "ArtImport" / run
    if receipt_dir.exists():
        raise RuntimeError("Fresh receipt directory required: " + str(receipt_dir))
    train_path = bounded_file(spec["trainManifest"], ART)
    environment_path = bounded_file(spec["environmentManifest"], ART)
    train = json.loads(train_path.read_text(encoding="utf-8"))
    environment = json.loads(environment_path.read_text(encoding="utf-8"))
    environment_items = {item["name"]: item for item in environment["assets"]}
    overrides = spec.get("fbxOverrides", {})
    if overrides and set(overrides) != set(ASSET_NAMES) | {"SM_TrainCar"}:
        raise RuntimeError("Explicit UE export overrides must cover every imported asset")
    items = []
    for name in ASSET_NAMES:
        source = environment_items[name]
        fbx = bounded_file(overrides.get(name, source["fbx"]), ART / "exports")
        if name != "REVIEW_CoordinateWitness" and source["status"] != "ENVELOPE_FIT_REQUIRES_IMPORT_REVIEW":
            raise RuntimeError("Asset is not a contained import candidate: " + name)
        target_name = "SM_StationPlatformEndPanel" if name == "SM_StationPlatformEndPanel_1m" else name
        items.append({"name": target_name, "sourceAssetName": name, "fbx": str(fbx), "sha256": sha(fbx),
                      "sourceBoundsM": [source["boundsMinimumM"], source["boundsMaximumM"]], "materials": source["materialSlots"]})
    fbx = bounded_file(overrides.get("SM_TrainCar", Path(train["exports_directory"])/"train_car_runtime.fbx"), ART / "exports")
    train_item = {"name": "SM_TrainCar", "fbx": str(fbx), "sha256": sha(fbx),
                  "sourceBoundsM": train["runtime_bounds_m"], "materials": train["runtime_material_slots"]}
    lo, hi = train_item["sourceBoundsM"]
    if lo[0] < -1.275-1e-6 or hi[0] > 1.275+1e-6 or lo[1] < -.85-1e-6 or hi[1] > .85+1e-6 or lo[2] < 0 or hi[2] > 2.4:
        raise RuntimeError("Train source manifest violates the approved art body")
    if abs(hi[0]-lo[0]-2.55) > 1e-6 or abs(hi[1]-lo[1]-1.70) > 1e-6:
        raise RuntimeError("Train source footprint is not the specified full-size car")
    # The independent witness must pass before train/station meshes are imported.
    items.sort(key=lambda item: item["name"] != "REVIEW_CoordinateWitness")
    items.insert(1, train_item)
    for item in items:
        for slot in item["materials"]:
            palette(slot)
    receipt_dir.mkdir(parents=True, exist_ok=False)
    receipt_file = receipt_dir / "receipt.json"
    receipt = {"schema": "vibecoaster-art-import-1", "status": "running", "destination": destination,
               "engineVersion": unreal.SystemLibrary.get_engine_version(),
               "importOptions": {"factory": "FbxFactory", "convertScene": True, "convertSceneUnit": True, "forceFrontXAxis": True, "uniformScale": 1.0, "translation": [0,0,0], "rotation": [0,90,0], "combineMeshes": True, "transformVertexToAbsolute": True, "autoCollision": False, "nanite": False}, "sourceSpec": str(spec_file), "sourceSpecSha256": sha(spec_file), "exportCoordinatePreparation": spec.get("exportCoordinatePreparation", "Original source FBX"),
               "sourceManifests": [{"path": str(path), "sha256": sha(path)} for path in (train_path, environment_path)],
               "toleranceCm": TOLERANCE_CM, "assets": [], "runtimeIntegrated": False,
               "exclusions": ["Unvalidated bogie", "Rail-to-spine web outside tie envelope", "Saved tower review assembly", "Generic joint/footing reference dimensions"],
               "limits": ["No runtime or physics code changed", "No collision/body-envelope replacement", "Actual POV and packaging review remain required"]}
    material_cache = {}
    subsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    try:
        for item in items:
            entry = {"name": item["name"], "sourceFbx": item["fbx"], "sourceSha256": item["sha256"], "status": "importing"}
            receipt["assets"].append(entry)
            receipt_file.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
            mesh = import_mesh(item, destination)
            entry.update(inspect_mesh(mesh, item, subsystem))
            for index, slot in enumerate(entry["materialSlots"]):
                mesh.set_material(index, make_material(slot, destination, material_cache))
            subsystem.set_allow_cpu_access(mesh, False)
            unreal.EditorAssetLibrary.set_metadata_tag(mesh, "VibeCoaster.ImportStatus", "GeometryChecked_NotRuntimeIntegrated")
            unreal.EditorAssetLibrary.set_metadata_tag(mesh, "VibeCoaster.SourceSHA256", item["sha256"])
            if not unreal.EditorAssetLibrary.save_loaded_asset(mesh, only_if_is_dirty=False):
                raise RuntimeError("Mesh save failed: " + mesh.get_path_name())
            entry["status"] = "geometry-and-materials-checked"
        receipt["status"] = "import-validated-not-integrated"
        unreal.log("VIBECOASTER_ART_IMPORT_OK " + str(receipt_file))
    except Exception as error:
        receipt["status"] = "failed-preserved"
        receipt["error"] = str(error)
        receipt["traceback"] = traceback.format_exc()
        if receipt["assets"]:
            receipt["assets"][-1]["status"] = "failed-preserved"
        unreal.log_error("VIBECOASTER_ART_IMPORT_FAILED " + str(error))
        raise
    finally:
        receipt_file.write_text(json.dumps(receipt, indent=2), encoding="utf-8")


main()
