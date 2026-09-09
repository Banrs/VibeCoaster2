"""Derive an open train from frozen v004 snapshots; run with Blender 5.2 --background.

Removes only the 13 reviewed aero cosmetics. Preserves all retained mesh vertices,
materials, restraints, footprint, rail datum and original v004 files.
"""
import bpy
import hashlib
import json
from pathlib import Path
from mathutils import Matrix

ART = Path(__file__).resolve().parents[1]
BUILD = "20260909-train-v005-open"
SOURCE = ART / "source/train" / BUILD
EXPORT = ART / "exports/train" / BUILD
REVIEW = ART / "review" / BUILD
BASE = ART / "source/train/20260908-train-v004/VibeCoaster_HighSpeedTrain.blend"
assert Path(bpy.data.filepath).resolve() == BASE.resolve(), "Open the frozen v004 source explicitly"
assert not any(p.exists() for p in (SOURCE, EXPORT, REVIEW)), "Fresh revision directories required"
assert bpy.app.version[:2] == (5, 2), "Use the existing Blender 5.2 installation"
source_hash = hashlib.sha256(BASE.read_bytes()).hexdigest()

removed = {"NoseCentralCarbonBlade", "NoseSignature-1", "NoseSignature1"}
for side in ("Left", "Right"):
    removed.update(side + "_" + part for part in (
        "ClearWindDeflector", "ShieldEdgeSpar-1", "ShieldEdgeSpar1", "ShieldTopRim", "AeroShoulderAccent"))
parts = [m for m in bpy.data.meshes if m.name.startswith("VCTrain4_V004Snapshot_")]
assert len(parts) == 76 and len(removed) == 13
names = {m["source_name"].removeprefix("VCTrain4_") for m in parts}
assert removed <= names, "Every reviewed cosmetic must exist in the actual source"
kept = [m for m in parts if m["source_name"].removeprefix("VCTrain4_") not in removed]
assert len(kept) == 63 and all(m["runtime_part"] for m in kept)

scene = bpy.data.scenes.new("VCTrain5_AssetReview")
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1
bpy.context.window.scene = scene
vertices, faces, assignments, materials = [], [], [], []
for part in kept:
    flat = list(part["world_matrix"])
    world = Matrix((flat[0:4], flat[4:8], flat[8:12], flat[12:16]))
    base = len(vertices)
    vertices.extend(world @ v.co for v in part.vertices)
    slots = []
    for material in part.materials:
        if material not in materials:
            materials.append(material)
        slots.append(materials.index(material))
    for face in part.polygons:
        faces.append(tuple(base + i for i in face.vertices))
        assignments.append((slots[face.material_index], face.use_smooth))
mesh = bpy.data.meshes.new("VCTrain5_OpenCarMesh")
mesh.from_pydata(vertices, [], faces)
mesh.update()
for material in materials:
    mesh.materials.append(material)
for face, (slot, smooth) in zip(mesh.polygons, assignments):
    face.material_index = slot
    face.use_smooth = smooth
assert all((v.co - original).length < 1e-7 for v, original in zip(mesh.vertices, vertices))
assert not any("WindDeflector" in m.name for m in materials)
car = bpy.data.objects.new("VCTrain5_SM_TrainCar_Assembled", mesh)
scene.collection.objects.link(car)
car["units"] = "metres"
car["visual_only"] = True
lo = [min(v.co[i] for v in mesh.vertices) for i in range(3)]
hi = [max(v.co[i] for v in mesh.vertices) for i in range(3)]
manifest = json.loads((ART / "review/20260908-4/train-manifest.json").read_text())
assert all(abs(value - expected) < 1e-7 for bounds, original in
           zip((lo, hi), manifest["runtime_bounds_m"]) for value, expected in zip(bounds, original))
manifest.update(build=BUILD, blender=bpy.app.version_string,
                source_blend=str(SOURCE / "VibeCoaster_OpenTrain.blend"), exports_directory=str(EXPORT),
                runtime_parts=len(kept), runtime_vertices=len(mesh.vertices),
                runtime_triangles=sum(len(p.vertices) - 2 for p in mesh.polygons),
                runtime_material_slots=[m.name for m in materials], runtime_bounds_m=[lo, hi],
                design="Conventional open train; original seats, restraints and bounded shell retained",
                base_build="20260908-train-v004", base_source_sha256=source_hash,
                removed_cosmetics=sorted(removed), retained_geometry="Unchanged evaluated source vertices and faces",
                source_review_blend=str(SOURCE / "VibeCoaster_OpenTrain_Review.blend"), exports={})
for directory in (SOURCE, EXPORT, REVIEW):
    directory.mkdir(parents=True, exist_ok=False)
car.select_set(True)
bpy.context.view_layer.objects.active = car
for extension, exporter, options in (
    ("fbx", bpy.ops.export_scene.fbx, dict(use_selection=True, object_types={"MESH"},
        use_mesh_modifiers=False, apply_unit_scale=True, apply_scale_options="FBX_SCALE_UNITS",
        axis_forward="X", axis_up="Z", add_leaf_bones=False, bake_anim=False)),
    ("glb", bpy.ops.export_scene.gltf, dict(export_format="GLB", use_selection=True,
        export_apply=False, export_yup=True)),
):
    output = EXPORT / ("train_car_runtime." + extension)
    assert "FINISHED" in exporter(filepath=str(output), **options)
    manifest["exports"][output.name] = dict(status="written", path=str(output),
        sha256=hashlib.sha256(output.read_bytes()).hexdigest())
scene["VCTrain5_manifest_json"] = json.dumps(manifest)
assert "FINISHED" in bpy.ops.wm.save_as_mainfile(filepath=manifest["source_blend"], copy=True)
(REVIEW / "train-manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

# Reuse the existing review cameras/lighting exactly; this is an asset comparison,
# not game POV or a performance measurement. Only the source object/path changes.
review_scripts = ART / "blender/train_review_v004"
prepare = (review_scripts / "01_prepare.py").read_text().replace("VCReview4_", "VCReview5_")
prepare = prepare.replace("VCTrain4_SM_TrainCar_Assembled", car.name)
exec(compile(prepare, str(review_scripts / "01_prepare.py"), "exec"))
bpy.context.window.scene = bpy.data.scenes["VCReview5_Scene"]
for filename in ("03_front.py", "04_side.py", "06_eye.py"):
    code = (review_scripts / filename).read_text().replace("VCReview4_", "VCReview5_")
    code = code.replace("D:/Coding/Codex/Vibecoasterjs/native/art/review/20260908-4", REVIEW.as_posix())
    exec(compile(code, str(review_scripts / filename), "exec"))
assert "FINISHED" in bpy.ops.wm.save_as_mainfile(filepath=manifest["source_review_blend"], copy=True)
assert hashlib.sha256(BASE.read_bytes()).hexdigest() == source_hash, "Frozen v004 source changed"
print("VIBECOASTER_OPEN_TRAIN_COMPLETE=" + json.dumps(manifest))
