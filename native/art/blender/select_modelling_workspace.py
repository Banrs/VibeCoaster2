import bpy
import json
from mathutils import Vector
scene=bpy.data.scenes["VCTrain_AssetReview"]
bpy.context.window.scene=scene
for area in bpy.context.screen.areas:
    if area.type=="VIEW_3D":
        space=area.spaces.active
        space.shading.type="SOLID"
        space.shading.color_type="MATERIAL"
        space.overlay.show_floor=False
        space.region_3d.view_location=Vector((0,0,.7))
        space.region_3d.view_distance=4.8
        space.region_3d.view_rotation=(Vector((0,0,.7))-Vector((4,-5,3))).to_track_quat("-Z","Y")
        space.region_3d.view_perspective="PERSP"
print(json.dumps({"blender":bpy.app.version_string,"activeScene":scene.name,"editableRuntimeParts":len(bpy.data.collections["VCTrain_RuntimeParts_AboveRail"].objects)}))
