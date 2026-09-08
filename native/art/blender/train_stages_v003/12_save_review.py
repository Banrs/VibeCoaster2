import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain_"
SCENE="VCTrain_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v003"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v003"
scene=bpy.data.scenes[SCENE]
result=bpy.ops.wm.save_as_mainfile(filepath=SOURCE+"/VibeCoaster_Train_Review.blend",copy=True)
manifest=json.loads(scene["VCTrain_manifest_json"])
manifest["source_review_blend"]=SOURCE+"/VibeCoaster_Train_Review.blend"
print("VC_TRAIN_STAGE_12_COMPLETE="+json.dumps(manifest))
