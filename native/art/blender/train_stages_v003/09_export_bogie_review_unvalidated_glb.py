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
output=EXPORT+"/bogie_review_unvalidated.glb"
result=bpy.ops.export_scene.gltf(filepath=output,export_format="GLB",use_selection=True,export_apply=False,export_yup=True)
assert "FINISHED" in result, str(result)
manifest=json.loads(scene["VCTrain_manifest_json"])
manifest["exports"]["bogie_review_unvalidated.glb"]={"status":"written","path":output}
scene["VCTrain_manifest_json"]=json.dumps(manifest)
print("VC_TRAIN_STAGE_09_EXPORTED="+json.dumps(manifest))
