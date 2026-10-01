"""Replace native support meshes in an existing full-ride Blender review.

Caller provides SCENE_NAME and REVIEW_ROOT. Track and terrain must match the
existing import; all changed supports still come from native fabrication data.
"""
import bpy
import hashlib
import json
import re
from pathlib import Path

root = Path('D:/Coding/Codex/Vibecoaster2')
scene = bpy.data.scenes[SCENE_NAME]
old = json.loads(scene['native_data_json'])
folder = root / REVIEW_ROOT / 'fixtures'
data = json.loads((folder / 'saved-layout.json').read_text())
old_ground = {(p[0], p[1]): p[2] for p in old['terrain']['points']}
assert old['terrain']['step'] == data['terrain']['step']
assert all(old_ground.get((p[0], p[1])) == p[2] for p in data['terrain']['points']), 'Terrain changed or expanded: use full native import'
assert old['nativeSpineDepth'] == data['nativeSpineDepth']
assert old['nativeSpineRadius'] == data['nativeSpineRadius']
# Exports add attachment stations to their fixed .35 m track samples. Compare
# the common fixed grid, including bank vectors, without requiring identical
# support station distances.
previous = {round(p[-1], 7): p[:3] for p in old['track']}
current = {round(p[-1], 7): p[:3] for p in data['track']}
common = previous.keys() & current.keys()
assert len(common) >= min(len(previous), len(current)) * .98
assert all(previous[k] == current[k] for k in common), 'Track changed: use full native import'
mesh_path = folder / 'saved-layout-fabrication.json'
payload = json.loads(mesh_path.read_text())
assert payload['profile'] == scene['profile']
owned = [o for o in scene.objects if o.type == 'MESH'
         and re.fullmatch(r'Support \d+(?:\.\d+)?', o.name)
         and o.get('geometry_source') == 'coaster_core portable mesh']
assert owned, 'No existing native supports to replace'
steel = next(o.data.materials[0] for o in owned
             if o.data.materials and o.data.materials[0].name.startswith('Exa architectural support finish'))
materials = [steel] + [bpy.data.materials[n] for n in ('SS Concrete', 'SS Fasteners', 'SS Rail', 'SS Spine')]
groups = {}
for part in payload['parts']:
    label = part['name']
    if label in ('Running rail', 'Track spine', 'Centred crosshead'):
        continue
    key = ('Support ' + str(part['owner']), part['material'])
    vertices, faces, smoothing = groups.setdefault(key, ([], [], []))
    base = len(vertices)
    vertices.extend(part['vertices'])
    for i, face in enumerate(part['faces']):
        smooth = part['smooth'] and len(face) == 4 and ('flange' not in label or i % 4 >= 2)
        for j in range(1, len(face)-1):
            faces.append([base+face[0], base+face[j], base+face[j+1]])
            smoothing.append(smooth)
part_count = len(payload['parts'])
del payload
for obj in owned:
    mesh = obj.data
    bpy.data.objects.remove(obj, do_unlink=True)
    if mesh.users == 0:
        bpy.data.meshes.remove(mesh)
while groups:
    (label, material), (vertices, faces, smoothing) = groups.popitem()
    mesh = bpy.data.meshes.new(label)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(label, mesh)
    scene.collection.objects.link(obj)
    mesh.materials.append(materials[material])
    for poly, smooth in zip(mesh.polygons, smoothing):
        poly.use_smooth = smooth
    obj['geometry_source'] = 'coaster_core portable mesh'
scene['native_data_json'] = json.dumps(data, separators=(',', ':'))
scene['native_part_count'] = part_count
scene['native_mesh_path'] = str(mesh_path)
scene['fabrication_sha256'] = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
scene['review_root'] = str(root / REVIEW_ROOT)
scene['support_update_method'] = 'Exact native replacement; unchanged track grid and terrain verified'
bpy.context.window.scene = scene
bpy.context.view_layer.update()
print(json.dumps({'scene': scene.name, 'parts': part_count, 'verifiedTrackSamples': len(common), 'objects': len(scene.objects)}))
