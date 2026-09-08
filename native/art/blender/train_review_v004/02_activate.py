import bpy
assert bpy.context.window is not None
bpy.context.window.scene=bpy.data.scenes["VCReview4_Scene"]
print("VC_REVIEW4_ACTIVATED")
