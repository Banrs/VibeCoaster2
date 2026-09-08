import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain4_"
SCENE="VCTrain4_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v004"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v004"
scene=bpy.data.scenes[SCENE]
assert bpy.context.scene == scene, "Train review scene must remain active"
layer=bpy.context.view_layer
layer.objects.active=None
for obj in list(layer.objects): obj.select_set(False)
review=bpy.data.collections[PREFIX+"BogieReview_UNVALIDATED"]
references=bpy.data.collections[PREFIX+"ReferenceGuides"]
for obj in [bpy.data.objects[PREFIX+"SM_TrainCar_Assembled"]]+list(review.objects)+list(references.objects):
    obj.hide_render=True
    obj.hide_set(True)
print("VC_TRAIN_STAGE_09_VISIBILITY_SET")
