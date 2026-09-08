import bpy
import bmesh
import json
from mathutils import Vector
PREFIX="VCTie2_"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/track/20260908-tie-v002"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/track/20260908-tie-v002"
scene=bpy.data.scenes[PREFIX+"Scene"]
assert bpy.context.scene==scene
manifest=json.loads(scene[PREFIX+"manifest"])
for other in list(bpy.context.view_layer.objects):other.select_set(False)
obj=bpy.data.objects[manifest["assembledObject"]]
obj.select_set(True)
bpy.context.view_layer.objects.active=obj
result=bpy.ops.export_scene.fbx(filepath=manifest["fbx"],use_selection=True,object_types={"MESH"},
    global_scale=1,apply_unit_scale=True,apply_scale_options="FBX_SCALE_UNITS",axis_forward="X",axis_up="Z",
    use_mesh_modifiers=False,add_leaf_bones=False,bake_anim=False)
assert "FINISHED" in result
manifest["status"]="exported-local-part-containment-checked; no active promotion"
print("VC_TIE2_MANIFEST="+json.dumps(manifest))
