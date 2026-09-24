"""Review the actual rail web, LSM and brake geometry at operating dimensions."""
import bpy
import math
from mathutils import Vector

EXPORT_ROOT="__EXPORT_ROOT__"
scene=bpy.context.scene
for obj in scene.objects:
    obj.hide_render=True
scene.render.engine="BLENDER_EEVEE"
scene.render.resolution_x=1200
scene.render.resolution_y=760
scene.render.resolution_percentage=100
scene.render.image_settings.file_format="PNG"
scene.world.use_nodes=True
scene.world.node_tree.nodes["Background"].inputs[0].default_value=(.17,.20,.23,1)
scene.world.node_tree.nodes["Background"].inputs[1].default_value=.55
scene.view_settings.view_transform="AgX"

def copy_asset(name,position,scale=(1,1,1)):
    obj=bpy.data.objects[name].copy()
    scene.collection.objects.link(obj)
    obj.hide_render=False
    obj.hide_set(False)
    obj.location=position
    obj.scale=scale
    return obj

bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-1.1))
ground=bpy.context.object
ground.data.materials.append(bpy.data.materials["VC2_Concrete"])
for name,loc,power,size in (("Key",(0,-3,8),2200,6),("Rim",(-2,5,5),1500,5)):
    bpy.ops.object.light_add(type="AREA",location=loc)
    lamp=bpy.context.object
    lamp.name=name
    lamp.data.energy=power
    lamp.data.shape="DISK"
    lamp.data.size=size
    lamp.rotation_euler=(Vector((0,0,0))-lamp.location).to_track_quat("-Z","Y").to_euler()

for y,z,radius in ((-.65,0,.085),(.65,0,.085),(0,-.55,.16)):
    bpy.ops.mesh.primitive_cylinder_add(vertices=12,radius=radius,depth=10,location=(0,y,z))
    obj=bpy.context.object
    obj.rotation_euler.y=math.pi*.5
    obj.data.materials.append(bpy.data.materials["VC2_Petrol"])
    for face in obj.data.polygons:
        face.use_smooth=len(face.vertices)==4
for x in (-4.5,-1.5,1.5,4.5):
    copy_asset("SM_TrackTieWeb",(x,0,-.19))
for x in (-4.2,-2.7,-1.2):
    for side in (-1,1):
        copy_asset("SM_LSMStator",(x,side*.34,-.11),(1.2,.20,.18))
    copy_asset("SM_LSMStator",(x,0,-.295),(.84,.80,.21))
for x in (.6,1.5,2.4,3.3,4.2):
    copy_asset("SM_BrakeFin",(x,0,-.08),(.72,.02,.28))
    copy_asset("SM_BrakeFin",(x,0,-.28),(.432,.20,.26))
    copy_asset("SM_BrakeFin",(x,.24,-.22),(.288,.22,.14))
bpy.ops.object.camera_add(location=(6.2,-7.8,5.1))
camera=bpy.context.object
camera.data.lens=48
camera.rotation_euler=(Vector((-.2,0,-.18))-camera.location).to_track_quat("-Z","Y").to_euler()
scene.camera=camera
scene.render.filepath=EXPORT_ROOT+"/track-hardware.png"
bpy.ops.render.render(write_still=True)
print("Rendered LSMs and retractable brake hardware at the existing operation-box dimensions.")
