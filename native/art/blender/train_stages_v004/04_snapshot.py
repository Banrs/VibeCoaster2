import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain4_"
SCENE="VCTrain4_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v004"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v004"
scene=bpy.data.scenes[SCENE]
assert bpy.context.scene == scene, "Train review scene must remain active"
runtime=bpy.data.collections[PREFIX+"RuntimeParts_AboveRail"]
review=bpy.data.collections[PREFIX+"BogieReview_UNVALIDATED"]
# Each snapshot is an independent persistent mesh datablock. No source mesh,
# object, modifier or curve is replaced, freed or hidden.
depsgraph=bpy.context.evaluated_depsgraph_get()
created=[]
for obj in list(runtime.objects)+list(review.objects):
    if obj.type not in {"MESH","CURVE"}: continue
    name=PREFIX+"V004Snapshot_"+obj.name.removeprefix(PREFIX)
    assert name not in bpy.data.meshes, "Snapshot already exists; preserve this run"
    print("VC_TRAIN_SNAPSHOT_BEGIN="+obj.name)
    evaluated=obj.evaluated_get(depsgraph)
    mesh=bpy.data.meshes.new_from_object(evaluated,preserve_all_data_layers=True,depsgraph=depsgraph)
    mesh.name=name
    mesh.use_fake_user=True
    mesh["source_name"]=obj.name
    mesh["world_matrix"]=[v for row in obj.matrix_world for v in row]
    mesh["pivot_m"]=list(obj.location)
    mesh["runtime_part"]=obj.name in runtime.objects
    created.append({"name":name,"vertices":len(mesh.vertices)})
print("VC_TRAIN_STAGE_04_SNAPSHOTS="+json.dumps(created))
