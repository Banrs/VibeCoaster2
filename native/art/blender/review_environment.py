import bpy
import json
OUTPUT="D:/Coding/Codex/Vibecoasterjs/native/art/review/20260907-2/"
views=[("VC_ENVKIT_StationAndModules","VC_ENVKIT_StationCamera","station-overview"),("VC_ENVKIT_StationAndModules","VC_ENVKIT_StationHumanScaleCamera","station-human-scale"),("VC_ENVKIT_StationAndModules","VC_ENVKIT_TieAndWebCamera","track-detail"),("VC_ENVKIT_AdaptiveTowerReview","VC_ENVKIT_TowerCamera","adaptive-tower")]
for scene_name,camera_name,output in views:
    scene=bpy.data.scenes[scene_name]
    bpy.context.window.scene=scene
    scene.camera=bpy.data.objects[camera_name]
    scene.render.engine="BLENDER_WORKBENCH"
    scene.render.resolution_x=1440
    scene.render.resolution_y=900
    scene.render.resolution_percentage=100
    scene.render.image_settings.file_format="PNG"
    scene.display.shading.light="STUDIO"
    scene.display.shading.color_type="MATERIAL"
    scene.display.shading.show_cavity=True
    scene.display.shading.cavity_type="BOTH"
    scene.display.shading.background_type="WORLD"
    scene.view_settings.view_transform="Standard"
    scene.render.filepath=OUTPUT+output+".png"
    result=bpy.ops.render.render(write_still=True,scene=scene_name)
    if "FINISHED" not in result: raise RuntimeError("Render failed")
    print("VC_REVIEW_RENDER="+scene.render.filepath)
print(json.dumps({"blender":bpy.app.version_string,"views":len(views),"renderEngine":"BLENDER_WORKBENCH","gamePOV":False}))
