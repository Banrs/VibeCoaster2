import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain_"
SCENE="VCTrain_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v003"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v003"
scene=bpy.data.scenes[SCENE]
assert bpy.context.scene == scene, "Train review scene must remain active"
layer=bpy.context.view_layer
obj=bpy.data.objects[PREFIX+"SM_Bogie_REVIEW_ONLY"]
for other in list(layer.objects): other.select_set(False)
obj.select_set(True)
layer.objects.active=obj
output=EXPORT+"/bogie_review_unvalidated.fbx"
result=bpy.ops.export_scene.fbx(filepath=output,use_selection=True,object_types={"MESH"},use_mesh_modifiers=False,apply_unit_scale=True,apply_scale_options="FBX_SCALE_UNITS",axis_forward="X",axis_up="Z",add_leaf_bones=False,bake_anim=False)
assert "FINISHED" in result, str(result)
manifest=json.loads(scene["VCTrain_manifest_json"])
manifest["exports"]["bogie_review_unvalidated.fbx"]={"status":"written","path":output}
scene["VCTrain_manifest_json"]=json.dumps(manifest)
print("VC_TRAIN_STAGE_10_EXPORTED="+json.dumps(manifest))
