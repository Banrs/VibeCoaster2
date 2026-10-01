"""Render named train cameras through Blender MCP; keep the viewport solid."""
import bpy
from pathlib import Path

ROOT=Path(globals().get('ROOT','D:/Coding/Codex/Vibecoaster2'))
VIEWS=globals().get('VIEWS',['hero','full-train','front','side','wheel','clamp','seating','rider','rear'])
scene=bpy.data.scenes['Riftwake / Train design']
scene.cycles.samples=16
scene.render.threads=8
scene.render.resolution_x=1800
scene.render.resolution_y=1125
scene.render.resolution_percentage=100
for name in VIEWS:
    scene.camera=bpy.data.objects['RT / Camera / '+name]
    scene.render.filepath=str(ROOT/'out/train-model'/f'{name}.png')
    bpy.ops.render.render(write_still=True,scene=scene.name)
    print('TRAIN_RENDERED '+name)
scene.camera=bpy.data.objects['RT / Camera / hero']
for window in bpy.context.window_manager.windows:
    window.scene=scene
    for area in window.screen.areas:
        if area.type=='VIEW_3D':
            area.spaces.active.shading.type=next(i.identifier for i in area.spaces.active.shading.bl_rna.properties['type'].enum_items if i.identifier=='SOLID')
            area.spaces.active.region_3d.view_perspective=next(i.identifier for i in area.spaces.active.region_3d.bl_rna.properties['view_perspective'].enum_items if i.identifier=='CAMERA')
            area.spaces.active.region_3d.view_camera_zoom=18
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'native/art/exports/train/Riftwake-Train.blend'),compress=True)
