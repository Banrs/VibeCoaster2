"""Small Eevee art review scene only; not a game or performance benchmark."""
import bpy
import math
import json
from mathutils import Vector

PREFIX="VCReview4_"
assert PREFIX+"Scene" not in bpy.data.scenes, "Preserve existing review; do not overwrite"
source=bpy.data.objects["VCTrain4_SM_TrainCar_Assembled"]
scene=bpy.data.scenes.new(PREFIX+"Scene")
scene.unit_settings.system="METRIC"
scene.unit_settings.scale_length=1.0
engines=[item.identifier for item in scene.render.bl_rna.properties["engine"].enum_items]
engine=next((name for name in ("BLENDER_EEVEE","BLENDER_EEVEE_NEXT") if name in engines),None)
assert engine is not None, "Actual alpha-aware Eevee engine required"
scene.render.engine=engine
scene.render.resolution_x=1280
scene.render.resolution_y=720
scene.render.resolution_percentage=100
scene.render.image_settings.file_format="PNG"
scene.render.film_transparent=False
if hasattr(scene,"eevee") and hasattr(scene.eevee,"taa_render_samples"):
    scene.eevee.taa_render_samples=64
scene.world=bpy.data.worlds.new(PREFIX+"World")
scene.world.use_nodes=True
scene.world.node_tree.nodes.get("Background").inputs[0].default_value=(.10,.14,.18,1)
scene.world.node_tree.nodes.get("Background").inputs[1].default_value=.30
scene.view_settings.view_transform="AgX"
for i in range(3):
    car=bpy.data.objects.new(PREFIX+"Car"+str(i),source.data)
    scene.collection.objects.link(car)
    car.location.x=i*3.4
    car.hide_render=i>0
camera_data=bpy.data.cameras.new(PREFIX+"CameraData")
camera=bpy.data.objects.new(PREFIX+"Camera",camera_data)
scene.collection.objects.link(camera)
scene.camera=camera
camera_data.clip_start=.01
camera_data.clip_end=200
for label,pos,energy,size in [("Key",(3,-4,6),1100,5),("Fill",(0,4,3),750,4),("Rim",(-4,-1,4),1300,3)]:
    data=bpy.data.lights.new(PREFIX+label,"AREA")
    data.energy=energy
    data.shape="DISK"
    data.size=size
    obj=bpy.data.objects.new(PREFIX+label,data)
    scene.collection.objects.link(obj)
    obj.location=pos
    obj.rotation_euler=(Vector((0,0,.7))-obj.location).to_track_quat("-Z","Y").to_euler()
# Small neutral review floor; no game terrain material is created or changed.
mat=bpy.data.materials.new(PREFIX+"ReviewFloor")
mat.use_nodes=True
bsdf=mat.node_tree.nodes.get("Principled BSDF")
bsdf.inputs["Base Color"].default_value=(.16,.18,.20,1)
bsdf.inputs["Roughness"].default_value=.72
mesh=bpy.data.meshes.new(PREFIX+"FloorMesh")
mesh.from_pydata([(-8,-8,-.06),(15,-8,-.06),(15,8,-.06),(-8,8,-.06)],[],[(0,1,2,3)])
mesh.materials.append(mat)
floor=bpy.data.objects.new(PREFIX+"Floor",mesh)
scene.collection.objects.link(floor)
print("VC_REVIEW4_READY="+json.dumps({"scene":scene.name,"engine":engine,"resolution":[1280,720],"samplesRequested":64,"eyeM":[0,0,1.2],"spacingM":3.4,"gamePerformanceBenchmark":False}))
