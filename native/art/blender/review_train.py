"""Lightweight actual Blender asset renders, independent of game performance."""
import bpy
import math
import json
from mathutils import Vector

OUTPUT = "D:/Coding/Codex/Vibecoasterjs/native/art/review/20260907-3/"
source = bpy.data.objects["VCTrain_SM_TrainCar_Assembled"]
scene = bpy.data.scenes.new("VCReview_Train_v003")
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
scene.render.engine = "BLENDER_WORKBENCH"
scene.render.resolution_x = 1440
scene.render.resolution_y = 900
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.film_transparent = False
scene.display.shading.light = "STUDIO"
scene.display.shading.color_type = "MATERIAL"
scene.display.shading.show_shadows = True
scene.display.shading.show_cavity = True
scene.display.shading.cavity_type = "BOTH"
scene.display.shading.background_type = "WORLD"
scene.world = bpy.data.worlds.new("VCReview_Train_World")
scene.world.color = (0.09, 0.105, 0.13)
scene.view_settings.view_transform = "Standard"
car = bpy.data.objects.new("VCReview_Car", source.data)
scene.collection.objects.link(car)
cam_data = bpy.data.cameras.new("VCReview_Camera")
cam = bpy.data.objects.new("VCReview_Camera", cam_data)
scene.collection.objects.link(cam)
scene.camera = cam
cam_data.clip_start = 0.01
cam_data.clip_end = 1000
bpy.context.window.scene = scene

def view(name, location, target, lens):
    cam.location = location
    cam.rotation_euler = (Vector(target)-cam.location).to_track_quat("-Z", "Y").to_euler()
    cam_data.lens = lens
    scene.render.filepath = OUTPUT + name + ".png"
    result = bpy.ops.render.render(write_still=True, scene=scene.name)
    if "FINISHED" not in result:
        raise RuntimeError("Render did not finish")
    print("VC_REVIEW_RENDER=" + scene.render.filepath)

view("train-front", (5.2,-5.1,3.15), (0,0,.7), 58)
view("train-rear", (-4.8,-4.5,2.6), (0,0,.8), 58)
for index in (1,2):
    ahead = bpy.data.objects.new("VCReview_CarAhead"+str(index), source.data)
    scene.collection.objects.link(ahead)
    ahead.location.x = index*3.4
cam_data.lens_unit = "FOV"
cam_data.angle = math.radians(82)
view("train-eye-1p2m", (0,0,1.2), (30,0,1.2), cam_data.lens)
print(json.dumps({"blender":bpy.app.version_string,"scene":scene.name,"cameraHeightM":1.2,"spacingM":3.4,"renderEngine":scene.render.engine,"gamePOV":False}))
