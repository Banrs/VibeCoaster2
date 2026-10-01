"""Render one review camera through Blender MCP, without rebuilding the model."""
import bpy
import math
from mathutils import Vector

EXPORT_ROOT = "__EXPORT_ROOT__"
view = "__STUDY_VIEW__"
scene = bpy.data.scenes["Exa Track Study"]
bpy.context.window.scene = scene
camera = scene.camera
views = {
    "hero": ({"straight"}, False, (-11.8, -18, 8), (0, -10, 2.0), 14.2),
    "joint": ({"straight"}, False, (-7.5, -15, 4.0), (-2.8, -10, 1.7), 4.4),
    "section": (set(), True, (-8, 0, 1.55), (0, 0, 1.55), 4.25),
    "banked": ({"banked"}, False, (-16, -16, 13), (0, .6, 3), 26),
    "inverted": ({"inverted"}, False, (-24, -15, 14), (0, 10, 3.6), 38),
    "overview": ({"straight", "banked", "inverted"}, False, (-31, -36, 29), (0, 2, 2), 47),
    "rider": ({"banked"}, False, (-10, -2, 4.8), (7, 1.6, 3.6), 25),
}
kinds, section, position, target, scale = views[view]
for collection in scene.collection.children:
    if collection.name.startswith("Study / "):
        kind = collection.name.removeprefix("Study / ")
        visible = section if kind == "cross-section and wheel fit" else kind in kinds
        for obj in collection.objects:
            obj.hide_render = not visible
camera.location = position
camera.rotation_euler = (Vector(target)-camera.location).to_track_quat("-Z", "Y").to_euler()
camera.data.type = "PERSP" if view == "rider" else "ORTHO"
camera.data.ortho_scale = scale
camera.data.lens = 20.7 if view == "rider" else 28
scene.render.engine = "CYCLES"
scene.cycles.samples = 16
scene.render.threads_mode = next(v.identifier for v in scene.render.bl_rna.properties['threads_mode'].enum_items if v.identifier == 'FIXED')
scene.render.threads = 4
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1600, 1000
scene.render.resolution_percentage = 100
scene.render.filepath = EXPORT_ROOT+"/exa-"+view+".png"
bpy.ops.render.render(write_still=True)
print("EXA_RENDER_COMPLETE "+scene.render.filepath)
