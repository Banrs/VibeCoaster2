"""Import one independently contained track-tie/web asset into a fresh V072 child.
No runtime promotion; original train/station imports and failed receipts are preserved.
Set VIBECOASTER_TRACK_TIE_IMPORT_SPEC to the project-local spec.
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
TOLERANCE_CM = .02
CONTAINMENT_TOLERANCE_M = 2e-6
ENV_PALETTE = {
    "Graphite": ([.065, .085, .11], .45, .34),
    "BlueSteel": ([.13, .24, .28], .55, .31),
    "MachinedSteel": ([.42, .48, .53], .82, .28),
    "DarkRecess": ([.022, .028, .034], .22, .5),
}
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


def palette(name):
    if not name.startswith("VC_ENVKIT_") or name[10:] not in ENV_PALETTE:
        raise RuntimeError("Unexpected tie material: " + name)
    return ENV_PALETTE[name[10:]]


def verify_certificates(manifest, core):
    certificates = manifest.get("webCertificates", [])
    if len(certificates) != 2 or sorted(c["coreIndex"] for c in certificates) != [0, 1]:
        raise RuntimeError("Exactly two distinct per-web certificates required")
    if manifest.get("railToTieLocalZ") != .19 or core["tiePivotZ"] != -.19:
        raise RuntimeError("Unexpected rail-to-tie pivot")
    for cert in certificates:
        solid = core["webs"][cert["coreIndex"]]
        if cert["coreOBB"] != solid or cert.get("checkedBeforeAssembly") is not True:
            raise RuntimeError("Per-part certificate is not tied to the exact compiled solid")
        if cert["vertexCount"] < 8:
            raise RuntimeError("Incomplete web mesh certificate")
        extents = cert["measuredHalfExtentsM"]
        if len(extents) != 3 or any(not math.isfinite(v) or v < 0 for v in extents):
            raise RuntimeError("Invalid per-part extents")
        outside = max(0, max(a-b for a,b in zip(extents,solid["half"])))
        cap = max(0, extents[1]-solid["half"][1])
        if outside > 1e-6 or cap > 1e-6:
            raise RuntimeError("Individual web protrudes beyond its exact OBB")
        if abs(cert["maximumOutsideM"]-outside)>1e-9 or abs(cert["capExtensionM"]-cap)>1e-9:
            raise RuntimeError("Per-web containment summary disagrees with extents")
    return certificates


def memberships(point_cm, core):
    # UE centimetres are core axes at the tie pivot; recover the rail datum.
    p = [v/100 for v in point_cm]
    found = set()
    if abs(p[0]) <= .07+CONTAINMENT_TOLERANCE_M and abs(p[1]) <= .825+CONTAINMENT_TOLERANCE_M and abs(p[2]) <= .08+CONTAINMENT_TOLERANCE_M:
        found.add("originalTie")
    p[2] -= .19
    for index, solid in enumerate(core["webs"]):
        d = [p[a]-solid["center"][a] for a in range(3)]
        if all(abs(sum(d[a]*solid[k][a] for a in range(3))) <= solid["half"][j]+CONTAINMENT_TOLERANCE_M for j,k in enumerate(("forward","right","up"))):
            found.add("web"+str(index))
    return found


def inspect_tie(mesh, item, core, subsystem):
    subsystem.set_allow_cpu_access(mesh, True)
    slots = [str(s.get_editor_property("imported_material_slot_name")) for s in mesh.get_editor_property("static_materials")]
    if len(slots) != len(item["materials"]) or set(slots) != set(item["materials"]):
        raise RuntimeError("Imported material slots differ from assembled source")
    all_points = []; sections = []; counts = {"originalTie":0,"web0":0,"web1":0}
    for section in range(mesh.get_num_sections(0)):
        vertices, triangles, normals, uvs, tangents = unreal.ProceduralMeshLibrary.get_section_from_static_mesh(mesh,0,section)
        points = [vector(v) for v in vertices]
        if not points or not triangles or len(triangles)%3 or len(normals)!=len(points):
            raise RuntimeError("Malformed imported section")
        assigned = []
        for point in points:
            if not all(math.isfinite(c) for c in point):
                raise RuntimeError("Non-finite imported point")
            solids = memberships(point,core)
            if not solids:
                raise RuntimeError("Imported vertex outside exact tie/web union: " + str(point))
            assigned.append(solids)
        for offset in range(0,len(triangles),3):
            ids = triangles[offset:offset+3]
            if any(i<0 or i>=len(points) for i in ids):
                raise RuntimeError("Invalid triangle index")
            common = set.intersection(*(assigned[i] for i in ids))
            if not common:
                raise RuntimeError("Triangle spans outside any one certified convex solid")
            for solid in common:
                counts[solid] += 1
        if any(not math.isfinite(v) for normal in normals for v in vector(normal)):
            raise RuntimeError("Non-finite imported normal")
        slot = subsystem.get_lod_material_slot(mesh,0,section)
        if slot<0 or slot>=len(slots):
            raise RuntimeError("Invalid material mapping")
        sections.append({"section":section,"materialSlot":slots[slot],"vertices":len(points),"triangles":len(triangles)//3,"boundsCm":bounds(points)})
        all_points.extend(points)
    if any(v==0 for v in counts.values()):
        raise RuntimeError("Imported tie or individual web is missing")
    actual = bounds(all_points)
    error = match_bounds(actual,ue_bounds(item["sourceBoundsM"]),"Source-to-UE tie bounds")
    box = mesh.get_bounds(); origin=vector(box.origin); extent=vector(box.box_extent)
    match_bounds([[origin[a]-extent[a] for a in range(3)],[origin[a]+extent[a] for a in range(3)]],actual,"Actual render bounds")
    if subsystem.get_simple_collision_count(mesh)!=0:
        raise RuntimeError("Unexpected imported collision")
    return {"assetPath":mesh.get_path_name(),"boundsCm":actual,"maximumBoundsErrorCm":error,"materialSlots":slots,
            "sections":sections,"triangleContainmentCounts":counts,"allTrianglesContainedInIndividualConvexSolid":True,
            "containmentToleranceM":CONTAINMENT_TOLERANCE_M,"collisionCount":0}


def inspect_material(material, name):
    edit=unreal.MaterialEditingLibrary
    rgb,metallic,roughness=palette(name)
    if material.get_editor_property("blend_mode")!=unreal.BlendMode.BLEND_OPAQUE or not material.get_editor_property("used_with_instanced_static_meshes"):
        raise RuntimeError("Tie material must be opaque and ISM enabled")
    node=edit.get_material_property_input_node(material,unreal.MaterialProperty.MP_BASE_COLOR)
    if not isinstance(node,unreal.MaterialExpressionConstant3Vector):
        raise RuntimeError("Missing base color")
    color=node.get_editor_property("constant"); actual=[color.r,color.g,color.b]
    if max(abs(a-b) for a,b in zip(actual,rgb))>1e-6:
        raise RuntimeError("Base color mismatch")
    for prop,expected in ((unreal.MaterialProperty.MP_METALLIC,metallic),(unreal.MaterialProperty.MP_ROUGHNESS,roughness)):
        node=edit.get_material_property_input_node(material,prop)
        if not isinstance(node,unreal.MaterialExpressionConstant) or abs(node.get_editor_property("r")-expected)>1e-6:
            raise RuntimeError("Structural material scalar mismatch")
    return {"slot":name,"materialPath":material.get_path_name(),"baseColorLinear":actual,"metallic":metallic,"roughness":roughness,"opaque":True,"instancedStaticMeshUsage":True}


def main():
    spec_file=bounded_file(os.environ.get("VIBECOASTER_TRACK_TIE_IMPORT_SPEC",""),ART)
    spec=json.loads(spec_file.read_text(encoding="utf-8"))
    run=spec["run"]; destination=spec["destinationRoot"]
    if not re.fullmatch(r"[A-Za-z0-9_]+",run) or not re.fullmatch(r"/Game/Art/V072/TrackWeb[A-Za-z0-9_]+",destination):
        raise RuntimeError("Out-of-scope run/destination")
    if unreal.EditorAssetLibrary.does_directory_exist(destination) or (PROJECT/"Content"/destination.removeprefix("/Game/")).exists():
        raise RuntimeError("Fresh destination required; existing assets preserved")
    receipt_dir=PROJECT/"Saved"/"ArtImport"/run
    if receipt_dir.exists():raise RuntimeError("Fresh receipt required")
    manifest_file=bounded_file(spec["tieManifest"],ART)
    core_file=bounded_file(spec["coreSpec"],PROJECT/"Saved"/"TrackWeb")
    witness_file=bounded_file(spec["coordinateWitnessReceipt"],PROJECT/"Saved"/"ArtImport")
    manifest=json.loads(manifest_file.read_text(encoding="utf-8"))
    core=json.loads(core_file.read_text(encoding="utf-8"))
    witness=json.loads(witness_file.read_text(encoding="utf-8"))
    if witness.get("status")!="import-validated-not-integrated" or not any(a.get("coordinateWitness") for a in witness.get("assets",[])):
        raise RuntimeError("Established yaw90 coordinate witness evidence missing")
    if manifest.get("build")!="20260908-tie-v002" or manifest.get("coreSpecSha256")!=sha(core_file):
        raise RuntimeError("Actual v002 manifest and exact compiled spec hash required")
    certificates=verify_certificates(manifest,core)
    fbx=bounded_file(manifest["fbx"],ART/"exports"/"track"/"20260908-tie-v002")
    item={"name":"SM_TrackTieWeb","fbx":str(fbx),"sha256":sha(fbx),"sourceBoundsM":[manifest["boundsMinimumM"],manifest["boundsMaximumM"]],"materials":manifest["materialSlots"]}
    for slot in item["materials"]:palette(slot)
    receipt_dir.mkdir(parents=True,exist_ok=False)
    receipt_file=receipt_dir/"receipt.json"
    receipt={"schema":"vibecoaster-track-web-import-1","status":"running","destination":destination,
      "engineVersion":unreal.SystemLibrary.get_engine_version(),"sourceSpec":str(spec_file),"sourceSpecSha256":sha(spec_file),
      "manifest":str(manifest_file),"manifestSha256":sha(manifest_file),"coreSpec":str(core_file),"coreSpecSha256":sha(core_file),
      "perWebPreAssemblyCertificates":certificates,"coordinateWitnessReceipt":str(witness_file),"coordinateWitnessSha256":sha(witness_file),
      "basis":"Original FBX + UE yaw90; Blender (x,y,z) -> UE metres (x,-y,z), previously witnessed independently",
      "importOptions":{"uniformScale":1,"rotation":[0,90,0],"convertScene":True,"convertSceneUnit":True,"forceFrontXAxis":True,"autoCollision":False,"nanite":False},
      "sourceFbx":str(fbx),"sourceFbxSha256":sha(fbx),"runtimeIntegrated":False,
      "limits":["Local art containment only; full-circuit core hardware proof and runtime promotion are separate","Existing train/station assets and source tie untouched"]}
    try:
        receipt_file.write_text(json.dumps(receipt,indent=2),encoding="utf-8")
        mesh=import_mesh(item,destination)
        subsystem=unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
        receipt["asset"]=inspect_tie(mesh,item,core,subsystem)
        cache={}; material_receipts=[]
        for index,slot in enumerate(receipt["asset"]["materialSlots"]):
            material=make_material(slot,destination,cache); mesh.set_material(index,material)
            if mesh.get_material(index)!=material:raise RuntimeError("Material binding failed")
            material_receipts.append(inspect_material(mesh.get_material(index),slot))
        receipt["asset"]["materialReceipts"]=material_receipts
        subsystem.set_allow_cpu_access(mesh,False)
        unreal.EditorAssetLibrary.set_metadata_tag(mesh,"VibeCoaster.ImportStatus","LocalHardwareContainmentChecked_NotRuntimeIntegrated")
        unreal.EditorAssetLibrary.set_metadata_tag(mesh,"VibeCoaster.CoreSpecSHA256",sha(core_file))
        if not unreal.EditorAssetLibrary.save_loaded_asset(mesh,only_if_is_dirty=False):raise RuntimeError("Mesh save failed")
        receipt["status"]="import-validated-not-integrated"
        unreal.log("VIBECOASTER_TRACK_WEB_IMPORT_OK "+str(receipt_file))
    except Exception as error:
        receipt["status"]="failed-preserved";receipt["error"]=str(error);receipt["traceback"]=traceback.format_exc()
        unreal.log_error("VIBECOASTER_TRACK_WEB_IMPORT_FAILED "+str(error))
        raise
    finally:
        receipt_file.write_text(json.dumps(receipt,indent=2),encoding="utf-8")


main()
