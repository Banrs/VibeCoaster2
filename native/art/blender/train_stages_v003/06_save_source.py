import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain_"
SCENE="VCTrain_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v003"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v003"
scene=bpy.data.scenes[SCENE]
assert bpy.context.scene == scene, "Train review scene must remain active"
result=bpy.ops.wm.save_as_mainfile(filepath=SOURCE+"/VibeCoaster_Train.blend",copy=True)
print("VC_TRAIN_STAGE_06_SAVED="+str(result))
