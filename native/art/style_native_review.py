"""Set clear daylight review materials without changing any native geometry.

Run via Blender MCP with SCENE_NAME. Copies are local to the current review;
the retained historical scenes keep their own materials and lighting.
"""
import bpy

scene = bpy.data.scenes[SCENE_NAME]

def finish(source, name, colour, metallic, roughness):
    material = bpy.data.materials.get(name)
    if material is None:
        material = source.copy()
        material.name = name
    material.diffuse_color = (*colour, 1)
    shader = next(n for n in material.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    shader.inputs['Base Color'].default_value = (*colour, 1)
    shader.inputs['Metallic'].default_value = metallic
    shader.inputs['Roughness'].default_value = roughness
    return material

ground = finish(bpy.data.materials['SS Terrain'], 'Exa review sandstone', (.43, .30, .17), 0, .85)
for obj in scene.objects:
    if obj.type != 'MESH':
        continue
    for i, material in enumerate(obj.data.materials):
        if material.name.startswith('Exa architectural support finish'):
            obj.data.materials[i] = finish(material, 'Exa architectural support finish / daylight', (.58, .64, .68), .25, .32)
    if obj.name in ('Native terrain', 'Native context apron'):
        obj.data.materials[0] = ground
if not scene.world.name.startswith('Exa review daylight'):
    scene.world = scene.world.copy()
    scene.world.name = 'Exa review daylight'
background = next(n for n in scene.world.node_tree.nodes if n.type == 'BACKGROUND')
background.inputs['Color'].default_value = (.38, .55, .76, 1)
background.inputs['Strength'].default_value = .5
for obj in scene.objects:
    if obj.type == 'LIGHT' and obj.data.type == 'SUN':
        obj.data.color = (1, .94, .84)
        obj.data.energy = 3
print('Daylight review finish applied; native meshes unchanged')
