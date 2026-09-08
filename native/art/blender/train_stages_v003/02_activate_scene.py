import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain_"
SCENE="VCTrain_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v003"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v003"
scene=bpy.data.scenes[SCENE]
assert bpy.context.window is not None, "A real Blender window is required"
bpy.context.window.scene=scene
print("VC_TRAIN_STAGE_02_ACTIVE="+json.dumps({"scene":bpy.context.scene.name,"window_scene":bpy.context.window.scene.name,"layer":bpy.context.view_layer.name}))
