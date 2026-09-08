import bpy
assert bpy.context.window is not None
bpy.context.window.scene=bpy.data.scenes["VCTie2_Scene"]
print("VC_TIE2_ACTIVE")
