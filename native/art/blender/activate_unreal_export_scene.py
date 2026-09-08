import bpy
import json
scene=bpy.data.scenes.get("VC_ENVKIT_ExportMasters")
if scene is None: scene=bpy.data.scenes.get("VC_ENVKIT_Export")
assert scene is not None and bpy.context.window is not None
assert "VC_UE_EXPORT_v001_manifest" in scene
bpy.context.window.scene=scene
print("VC_UNREAL_EXPORT_SCENE_ACTIVE="+scene.name)
