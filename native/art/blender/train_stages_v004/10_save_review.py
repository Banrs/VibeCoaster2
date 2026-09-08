import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain4_"
SCENE="VCTrain4_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v004"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v004"
scene=bpy.data.scenes[SCENE]
result=bpy.ops.wm.save_as_mainfile(filepath=SOURCE+"/VibeCoaster_HighSpeedTrain_Review.blend",copy=True)
manifest=json.loads(scene["VCTrain4_manifest_json"])
manifest["source_review_blend"]=SOURCE+"/VibeCoaster_HighSpeedTrain_Review.blend"
print("VC_TRAIN_STAGE_10_COMPLETE="+json.dumps(manifest))
