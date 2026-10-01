"""Import the exact native Exa review meshes into live Blender through MCP.

Caller sets CASE (camelback-0, camelback-3, loop-0 or immelmann-3).
Existing art scenes are retained; native review scenes use their external cameras.
"""
import bpy
import json
import hashlib
import sys
from pathlib import Path
from mathutils import Vector

root = Path('D:/Coding/Codex/Vibecoaster2')
kind, seed = CASE.rsplit('-', 1)
title = {'camelback':'Camelback','loop':'Loop','immelmann':'Immelmann','ride':'Riftwake generated ride'}[kind]
source_title = 'Camelback' if kind == 'ride' else title
source_name = 'Support System / '+source_title+' / terrain '+('0' if kind == 'ride' else seed)
source = bpy.data.scenes.get(source_name)
if source is None:
    # The original art study only has selected terrain cases. Reuse its
    # lighting/cameras for another seed; the native payload supplies terrain.
    source = next(s for s in bpy.data.scenes if s.name.startswith('Support System / '+source_title+' / terrain '))
name = globals().get('SCENE_NAME', globals().get('SCENE_PREFIX','Native Exa / ')+title+' / terrain '+seed)
old = bpy.data.scenes.get(name)
if old:
    old_objects = list(old.objects)
    bpy.data.scenes.remove(old)
    for obj in old_objects:
        if obj.users == 0:
            mesh = obj.data if obj.type == 'MESH' else None
            bpy.data.objects.remove(obj)
            if mesh and mesh.users == 0:
                bpy.data.meshes.remove(mesh)
scene = bpy.data.scenes.new(name)
bpy.context.window.scene = scene
review_root = root/globals().get('REVIEW_ROOT','out/exa-runtime')
data_file = globals().get('DATA_FILE', CASE)
mesh_path = review_root/'fixtures'/f'{data_file}-fabrication.json'
data = json.loads((review_root/'fixtures'/f'{data_file}.json').read_text())
payload = json.loads(mesh_path.read_text())
scene['source'] = 'Portable coaster_core Exa fabrication; identical geometry used by clearance and Unreal'
scene['profile'] = payload['profile']
scene['native_case'] = CASE
scene['native_part_count'] = len(payload['parts'])
scene['fabrication_sha256'] = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
scene['native_mesh_path'] = str(mesh_path)
scene['review_root'] = str(review_root)
scene['native_data_json'] = json.dumps(data, separators=(',',':'))
scene.unit_settings.system = source.unit_settings.system
scene.unit_settings.scale_length = 1
scene.world = source.world
scene.render.engine = source.render.engine
scene.cycles.device = source.cycles.device
scene.cycles.samples = 16
scene.cycles.use_denoising = True
scene.render.threads_mode = source.render.threads_mode
scene.render.threads = 8
scene.render.resolution_x, scene.render.resolution_y = 1260, 770
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = source.render.image_settings.file_format
scene.view_settings.view_transform = source.view_settings.view_transform
for obj in source.objects:
    if obj.type in ('CAMERA','LIGHT'):
        copy = obj.copy()
        copy.data = obj.data.copy()
        scene.collection.objects.link(copy)
        if obj == source.camera:
            scene.camera = copy

materials = [bpy.data.materials[n] for n in ('SS Steel','SS Concrete','SS Fasteners','SS Rail','SS Spine')]
if globals().get('SUPPORT_COLOR'):
    # A scene-local finish keeps earlier comparisons and the accepted track
    # materials intact while making the new primary/secondary hierarchy clear.
    paint = materials[0].copy()
    paint.name = 'Exa architectural support finish'
    paint.diffuse_color = tuple(SUPPORT_COLOR)
    node = next(n for n in paint.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    node.inputs['Base Color'].default_value = tuple(SUPPORT_COLOR)
    node.inputs['Metallic'].default_value = .25
    node.inputs['Roughness'].default_value = .34
    materials[0] = paint
    scene['support_finish'] = list(SUPPORT_COLOR)
groups = {}
for part in payload['parts']:
    label = part['name']
    track = label in ('Running rail','Track spine','Centred crosshead')
    material = 3 if label == 'Running rail' else 4 if track else part['material']
    key = (label if track else 'Support '+str(part['owner']), material)
    vertices, faces, smoothing = groups.setdefault(key, ([],[],[]))
    base = len(vertices)
    vertices.extend(part['vertices'])
    # Use the native adapter's exact diagonal. Letting Blender tessellate a
    # long nonplanar quad independently can move the mating surface by cm.
    for i, face in enumerate(part['faces']):
        smooth = part['smooth'] and (track or len(face)==4) and ('flange' not in label or i%4>=2)
        for j in range(1, len(face)-1):
            faces.append([base+face[0], base+face[j], base+face[j+1]])
            smoothing.append(smooth)
del payload
while groups:
    (label, material),(vertices,faces,smoothing) = groups.popitem()
    mesh = bpy.data.meshes.new(label)
    mesh.from_pydata(vertices,[],faces)
    mesh.update()
    obj = bpy.data.objects.new(label,mesh)
    scene.collection.objects.link(obj)
    obj.data.materials.append(materials[material])
    for poly, smooth in zip(mesh.polygons,smoothing):
        poly.use_smooth = smooth
    obj['geometry_source'] = 'coaster_core portable mesh'
terrain = data['terrain']
columns,rows = terrain['columns'],terrain['rows']
faces = []
for y in range(rows-1):
    for x in range(columns-1):
        i = y*columns+x
        faces.extend(((i,i+1,i+columns+1),(i,i+columns+1,i+columns)))
mesh = bpy.data.meshes.new('Native terrain')
mesh.from_pydata(terrain['points'],[],faces)
mesh.update()
obj = bpy.data.objects.new('Native terrain',mesh)
scene.collection.objects.link(obj)
obj.data.materials.append(bpy.data.materials['SS Terrain'])
art_path = str(root/'native/art')
if art_path not in sys.path:
    sys.path.insert(0, art_path)
from support_review_terrain import add_context_apron
add_context_apron(scene, terrain, bpy.data.materials['SS Terrain'])

# Aim close views at the actual new section's head, using the existing camera
# projections. Whole-structure, foundation and node views remain comparable.
chosen = (max(range(len(data['supports'])),key=lambda i:data['supports'][i]['attachment'][2]) if kind=='camelback'
          else min(range(len(data['supports'])),key=lambda i:data['supports'][i]['frame'][2][2]))
support = data['supports'][chosen]
origin,right,up = (Vector(v) for v in support['frame'])
forward = right.cross(up).normalized()
target = Vector(support['attachment'])-up*.45
for camera in (o for o in scene.objects if o.type=='CAMERA'):
    if camera.name.startswith('Connection detail camera'):
        camera.location = origin-forward*3.5-right*4.2+up*.1
        camera.rotation_euler = (target-camera.location).to_track_quat('-Z','Y').to_euler()
    elif camera.name.startswith('Connection reverse camera'):
        camera.location = origin+forward*3.5+right*4.2-up*1.5
        camera.rotation_euler = (target-camera.location).to_track_quat('-Z','Y').to_euler()
scene.camera = next(o for o in scene.objects if o.name.startswith('Support ground camera'))
scene.render.filepath = str(review_root/f'{CASE}-ground.png')
print(json.dumps({'scene':name,'parts':scene['native_part_count'],'objects':len(scene.objects),
    'vertices':sum(len(o.data.vertices) for o in scene.objects if o.type=='MESH')}))
del groups
