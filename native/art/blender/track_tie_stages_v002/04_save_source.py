import bpy
import bmesh
import json
from mathutils import Vector
PREFIX="VCTie2_"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/track/20260908-tie-v002"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/track/20260908-tie-v002"
assert bpy.context.scene.name==PREFIX+"Scene"
result=bpy.ops.wm.save_as_mainfile(filepath=SOURCE+"/TrackTieWeb_v002.blend",copy=True)
assert "FINISHED" in result
print("VC_TIE2_SOURCE_SAVED")
