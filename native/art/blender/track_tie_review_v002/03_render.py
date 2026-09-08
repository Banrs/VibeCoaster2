import bpy
assert bpy.context.scene.name=="VCWebReview2_Scene"
bpy.ops.render.render(write_still=True)
print("VC_WEB_REVIEW_IMAGE="+bpy.context.scene.render.filepath)
