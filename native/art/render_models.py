"""Small visual review renders of the actual MCP-authored geometry."""
import bpy
import math
from mathutils import Vector

EXPORT_ROOT="__EXPORT_ROOT__"
scene=bpy.context.scene
scene.render.engine="BLENDER_EEVEE"
scene.render.resolution_x=1000
scene.render.resolution_y=760
scene.render.resolution_percentage=100
scene.render.image_settings.file_format="PNG"
scene.render.film_transparent=False
scene.world.use_nodes=True
scene.world.node_tree.nodes["Background"].inputs[0].default_value=(0.19,0.22,0.25,1)
scene.world.node_tree.nodes["Background"].inputs[1].default_value=.55
scene.view_settings.view_transform="AgX"

for obj in scene.objects:
    obj.hide_render=True

bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-.85))
ground=bpy.context.object
ground.name="Review ground"
ground.hide_render=False
mat=bpy.data.materials.new("Review ground")
mat.diffuse_color=(.18,.205,.225,1)
mat.use_nodes=True
mat.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value=(.18,.205,.225,1)
mat.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value=.78
ground.data.materials.append(mat)

for name,location,power,size in (("Key",(2,-3,6),1400,5),("Rim",(-3,4,5),2000,4),("Fill",(4,5,3),950,5)):
    bpy.ops.object.light_add(type="AREA",location=location)
    lamp=bpy.context.object
    lamp.name=name
    lamp.data.energy=power
    lamp.data.shape="DISK"
    lamp.data.size=size
    lamp.rotation_euler=(Vector((0,0,.7))-lamp.location).to_track_quat("-Z","Y").to_euler()

review_track=[]
for y,z,radius,material_name in ((-.65,0,.085,"VC2_Steel"),(.65,0,.085,"VC2_Steel"),(0,-.55,.16,"VC2_Petrol")):
    bpy.ops.mesh.primitive_cylinder_add(vertices=20,radius=radius,depth=7,location=(0,y,z))
    obj=bpy.context.object
    obj.name="Review rail"
    obj.rotation_euler.y=math.pi*.5
    obj.data.materials.append(bpy.data.materials[material_name])
    for face in obj.data.polygons:
        face.use_smooth=len(face.vertices)==4
    review_track.append(obj)
for x in (-2,0,2):
    obj=bpy.data.objects["SM_TrackTieWeb"].copy()
    obj.data=bpy.data.objects["SM_TrackTieWeb"].data.copy()
    scene.collection.objects.link(obj)
    obj.hide_render=False
    obj.hide_set(False)
    obj.location=(x,0,-.19)
    review_track.append(obj)

bpy.ops.object.camera_add(location=(3.6,-3.6,2.55))
camera=bpy.context.object
scene.camera=camera
camera.data.lens=53
camera.rotation_euler=(Vector((0,0,.7))-camera.location).to_track_quat("-Z","Y").to_euler()
train=bpy.data.objects["SM_LeadCar"]
train.hide_render=False
scene.render.filepath=EXPORT_ROOT+"/train-front.png"
bpy.ops.render.render(write_still=True)

camera.location=(-3.5,-3.6,2.4)
camera.rotation_euler=(Vector((0,0,.7))-camera.location).to_track_quat("-Z","Y").to_euler()
scene.render.filepath=EXPORT_ROOT+"/train-rear.png"
bpy.ops.render.render(write_still=True)

# Match the game's default seat centre and 82-degree horizontal field of view.
camera.location=(0,0,1.2)
camera.data.lens=36/(2*math.tan(math.radians(82)*.5))
camera.rotation_euler=(Vector((10,0,1.2))-camera.location).to_track_quat("-Z","Y").to_euler()
scene.render.filepath=EXPORT_ROOT+"/train-rider.png"
bpy.ops.render.render(write_still=True)

# Review the complete visual language as an articulated high-speed train.
train_copies=[]
for x in (-3.4,-6.8):
    obj=bpy.data.objects["SM_TrainCar"].copy()
    scene.collection.objects.link(obj)
    obj.location=(x,0,0)
    obj.hide_render=False
    obj.hide_set(False)
    train_copies.append(obj)
for x in (-1.7,-5.1):
    obj=bpy.data.objects["SM_TrainCoupler"].copy()
    scene.collection.objects.link(obj)
    obj.location=(x,0,.30)
    obj.scale.x=.85
    obj.hide_render=False
    obj.hide_set(False)
    train_copies.append(obj)
for obj in review_track:
    if obj.name.startswith("Review rail"):
        obj.location.x=-3.4
        obj.scale.z=2.15
camera.location=(7,-12,6)
camera.data.lens=49
camera.rotation_euler=(Vector((-3.2,0,.78))-camera.location).to_track_quat("-Z","Y").to_euler()
scene.render.filepath=EXPORT_ROOT+"/train-formation.png"
bpy.ops.render.render(write_still=True)
for obj in train_copies:
    obj.hide_render=True

train.hide_render=True
for obj in review_track:
    obj.hide_render=True
ground.location.z=-1.10
for name,location,scale in (
    ("SM_StationRoofPanel",(0,0,5.4),(1,1,1)),
    ("SM_StationPlatformPanel",(0,-3.2,0),(1,1,1)),
    ("SM_StationPost",(-1.15,-4.8,0),(1,1,1)),
    ("SM_StationHoldingLane",(0,4.9,.58),(.74,2.95,.58)),
    ("SM_StationBoardingGate",(0,1.95,.67),(.74,.10,.67)),
    ("SM_StationDispatchCabin",(4.7,4.9,1.65),(1.6,1.55,1.65)),
    ("SM_StationLift",(-5.4,8,-1.0),(1.35,1.45,3.65)),
    ("SM_TrackTieWeb",(0,0,.75),(1,1,1)),
    ("SM_LSMStator",(0,2,.2),(1,1,1))):
    obj=bpy.data.objects[name]
    obj.location=location
    obj.scale=scale
    obj.hide_render=False
camera.location=(12,-15,10)
camera.data.lens=48
camera.rotation_euler=(Vector((0,0,2.1))-camera.location).to_track_quat("-Z","Y").to_euler()
scene.render.filepath=EXPORT_ROOT+"/station-kit.png"
bpy.ops.render.render(write_still=True)
print("Rendered train-front.png, train-rear.png, train-rider.png and station-kit.png from the authored mesh kit.")
