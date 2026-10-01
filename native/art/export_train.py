"""Export a train-only GLB from disposable merged car copies through Blender MCP.

The editable source retains named components, curves and linked car parts.
The six standard GLB cars share one mesh. No lighting or reference rail exports.
"""
import bpy
import bmesh
import json
from pathlib import Path

ROOT=Path(globals().get('ROOT','D:/Coding/Codex/Vibecoaster2'))
scene=bpy.data.scenes['Riftwake / Train design']
deps=bpy.context.evaluated_depsgraph_get()
roots=sorted((o for o in scene.objects if o.get('role')=='car_root'),key=lambda o:o['car_index'])
temporary=bpy.data.collections.new('RT export / temporary copies')
scene.collection.children.link(temporary)
clones=[];meshes=[]
for i,root in enumerate(roots):
    if i<2:
        vertices=[];faces=[];slots=[];smooth=[];materials=[]
        for obj in scene.objects:
            if obj.parent!=root or obj.type not in {'MESH','CURVE'}: continue
            evaluated=obj.evaluated_get(deps)
            data=evaluated.to_mesh()
            if obj.type=='CURVE':
                bm=bmesh.new();bm.from_mesh(data)
                bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-7)
                bm.to_mesh(data);bm.free()
            matrix=root.matrix_world.inverted()@obj.matrix_world
            offset=len(vertices)
            vertices.extend(matrix@v.co for v in data.vertices)
            mapping={}
            for j,mat in enumerate(data.materials):
                if mat not in materials: materials.append(mat)
                mapping[j]=materials.index(mat)
            for polygon in data.polygons:
                faces.append(tuple(offset+j for j in polygon.vertices))
                slots.append(mapping[polygon.material_index]);smooth.append(polygon.use_smooth)
            evaluated.to_mesh_clear()
        mesh=bpy.data.meshes.new('Riftwake lead car' if i==0 else 'Riftwake standard car')
        mesh.from_pydata(vertices,[],faces);mesh.update()
        for mat in materials:mesh.materials.append(mat)
        for polygon,index,shading in zip(mesh.polygons,slots,smooth):
            polygon.material_index=index;polygon.use_smooth=shading
        meshes.append(mesh)
    else:mesh=meshes[1]
    clone=bpy.data.objects.new(f'Riftwake car {i+1:02d}',mesh)
    temporary.objects.link(clone)
    clone.matrix_world=root.matrix_world
    clone['gauge_metres']=1.4
    clone['car_index']=i
    clones.append(clone)
bpy.context.view_layer.update()
for obj in scene.objects:obj.select_set(False)
for obj in clones:obj.select_set(True)
bpy.context.view_layer.objects.active=clones[0]
destination=ROOT/'native/art/exports/train/Riftwake-Train.glb'
try:
    # Blender's inspected exporter defaults to binary glTF; extension is .glb.
    bpy.ops.export_scene.gltf(filepath=str(destination),use_selection=True,
        export_apply=True,export_extras=True,export_animations=False)
finally:
    for obj in clones:bpy.data.objects.remove(obj,do_unlink=True)
    bpy.data.collections.remove(temporary)
    for mesh in meshes:
        if mesh.users==0:bpy.data.meshes.remove(mesh)
print(json.dumps({'exported':str(destination),'bytes':destination.stat().st_size,
                  'cars':7,'unique_car_meshes':2,'units':'metres','up_axis':'glTF +Y'}))
