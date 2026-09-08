import bpy
import math
import json
from mathutils import Vector
scene=bpy.data.scenes["VCReview4_Scene"]
assert bpy.context.scene==scene, "Activate review in separate prior MCP call"
cam=scene.camera
cam.location=(0, 0, 1.2)
cam.rotation_euler=(Vector((20, 0, 1.2))-cam.location).to_track_quat("-Z","Y").to_euler()
cam.data.lens_unit="FOV"
cam.data.angle=math.radians(82)
for i in (1,2): bpy.data.objects["VCReview4_Car"+str(i)].hide_render=False
scene.render.filepath="D:/Coding/Codex/Vibecoasterjs/native/art/review/20260908-4/train-eye-1p2m.png"
result=bpy.ops.render.render(write_still=True,scene=scene.name)
assert "FINISHED" in result
print("VC_REVIEW4_RENDER="+json.dumps({"path":scene.render.filepath,"engine":scene.render.engine,"gamePOV":False,"gamePerformanceBenchmark":False,"eyeDatumM":1.2,"carSpacingM":3.4}))
