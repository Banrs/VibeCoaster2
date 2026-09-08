import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain4_"
SCENE="VCTrain4_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v004"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v004"
scene=bpy.data.scenes[SCENE]
assert bpy.context.scene == scene, "Train review scene must remain active"
result=bpy.ops.wm.save_as_mainfile(filepath=SOURCE+"/VibeCoaster_HighSpeedTrain.blend",copy=True)
print("VC_TRAIN_STAGE_06_SAVED="+str(result))
