import bpy
from mathutils import Vector
OUTPUT="D:/Coding/Codex/Vibecoasterjs/native/art/review/20260907-3/"
scene=bpy.data.scenes.new("VCReview_TrackDetails")
for name,x in (("SM_TrackTie",-1.1),("REVIEW_TrackRailToSpineWeb",1.1)):
    obj=bpy.data.objects.new("VCReview_"+name,None)
    obj.instance_type="COLLECTION"
    obj.instance_collection=bpy.data.collections["VC_ENVKIT_"+name]
    scene.collection.objects.link(obj)
    obj.location=(x,0,.5)
data=bpy.data.cameras.new("VCReview_TrackCamera")
cam=bpy.data.objects.new("VCReview_TrackCamera",data)
scene.collection.objects.link(cam)
cam.location=(3,-5,3)
cam.rotation_euler=(Vector((0,0,.25))-cam.location).to_track_quat("-Z","Y").to_euler()
data.type="ORTHO"
data.ortho_scale=4.5
scene.camera=cam
scene.world=bpy.data.worlds.new("VCReview_DetailsWorld")
scene.world.color=(.09,.105,.13)
tower=bpy.data.scenes["VC_ENVKIT_AdaptiveTowerReview"]
tower.camera.data.lens=32
tower.camera.rotation_euler=(Vector((0,0,37))-tower.camera.location).to_track_quat("-Z","Y").to_euler()
for target,name in ((scene,"track-details-isolated"),(tower,"adaptive-tower-complete")):
    bpy.context.window.scene=target
    target.render.engine="BLENDER_WORKBENCH"
    target.render.resolution_x=1440
    target.render.resolution_y=1000
    target.render.resolution_percentage=100
    target.render.image_settings.file_format="PNG"
    target.display.shading.color_type="MATERIAL"
    target.display.shading.show_cavity=True
    target.display.shading.cavity_type="BOTH"
    target.display.shading.background_type="WORLD"
    target.view_settings.view_transform="Standard"
    target.render.filepath=OUTPUT+name+".png"
    bpy.ops.render.render(write_still=True,scene=target.name)
    print("VC_REVIEW_RENDER="+target.render.filepath)
