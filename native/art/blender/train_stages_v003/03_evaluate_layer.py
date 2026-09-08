import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain_"
SCENE="VCTrain_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v003"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v003"
scene=bpy.data.scenes[SCENE]
assert bpy.context.scene == scene, "Run activate stage and return to MCP first"
layer=bpy.context.view_layer
layer.update()
depsgraph=bpy.context.evaluated_depsgraph_get()
print("VC_TRAIN_STAGE_03_EVALUATED="+json.dumps({"objects":len(layer.objects),"evaluated_objects":len(depsgraph.objects),"layer":layer.name}))
