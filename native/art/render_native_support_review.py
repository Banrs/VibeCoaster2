"""External review cameras for an imported native support scene, via Blender MCP.

Set CASE, REVIEW_ROOT, SCENE_PREFIX and VIEW. Schedules one small GPU render.
"""
import bpy
import json
from collections import defaultdict
from pathlib import Path
from mathutils import Vector

root = Path('D:/Coding/Codex/Vibecoaster2') / REVIEW_ROOT
kind, seed = CASE.rsplit('-', 1)
title = {'camelback': 'Camelback', 'loop': 'Loop', 'immelmann': 'Immelmann'}[kind]
scene = bpy.data.scenes[SCENE_PREFIX + title + ' / terrain ' + seed]
bpy.context.window.scene = scene
data = json.loads(scene['native_data_json'])
camera_prefix = {'ground': 'Support ground camera', 'oblique': 'Support camera',
                 'joint': 'Connection detail camera', 'reverse': 'Connection reverse camera',
                 'shoulder': 'Structural joint camera', 'foundation': 'Foundation detail camera'}[VIEW]
camera = next(o for o in scene.objects if o.name.startswith(camera_prefix))
scene.camera = camera
camera.data.type = next(o for o in scene.objects if o.name.startswith('Support ground camera')).data.type
camera.data.clip_end = 20000
members = [m for s in data['supports'] for m in s['members']]
crest = max(data['supports'], key=lambda s: s['frame'][0][2])
centre, right, up = (Vector(v) for v in crest['frame'])
across = Vector((right.x, right.y, 0)).normalized()
along = across.cross(Vector((0, 0, 1))).normalized()
target = camera.location + camera.rotation_euler.to_quaternion() @ Vector((0, 0, -10))
if VIEW in ('ground', 'oblique') and kind == 'camelback':
    target = Vector((centre.x, centre.y, centre.z * .44))
    camera.location = target + along * (-300 if VIEW == 'ground' else -430) + across * (650 if VIEW == 'ground' else 290)
    camera.location.z = min(m[1][2] for m in members if m[4] == 1) + 2
    camera.data.lens = 48 if VIEW == 'ground' else 34
elif VIEW in ('ground', 'oblique'):
    points = [Vector(p[0]) for p in data['track']]
    low = Vector(tuple(min(p[j] for p in points) for j in range(3)))
    high = Vector(tuple(max(p[j] for p in points) for j in range(3)))
    ground = min(m[1][2] for m in members if m[4] == 1)
    height = high.z-ground
    target = (low+high)*.5
    target.z = ground+height*.46
    distance = max(height*2.8, (high-low).length*1.8)
    camera.location = target+across*distance+along*(height*(.5 if VIEW=='ground' else -1.5))
    camera.location.z = ground+1.8
    camera.data.lens = 48
elif VIEW == 'shoulder':
    nodes = defaultdict(list)
    for m in members:
        if m[4] == 0 and not m[5]:
            for end in (0, 1):
                nodes[tuple(round(v, 5) for v in m[end])].append(m[2+end])
    candidates = [(p, rs) for p, rs in nodes.items()
                  if len(rs) >= 6 and centre.z * .55 < p[2] < centre.z * .80]
    position, _ = max(candidates, key=lambda item: item[0][2])
    target = Vector(position)
    sign = 1 if (target - centre).dot(across) > 0 else -1
    camera.location = target + along * 5 + across * (sign * 9) + Vector((0, 0, 3.5))
    camera.data.lens = 50
elif VIEW == 'foundation':
    m = max((m for m in members if m[4] == 1), key=lambda m: m[2])
    target = Vector(m[1])
    camera.location = target + along * 6 + across * 5 + Vector((0, 0, 3))
    camera.data.lens = 50

def seat_eye_above_terrain():
    # The review backdrop now joins the actual terrain boundary. Keep the eye
    # above that surface as well; the lowest footing can be far downhill from
    # the external camera on a rotated, sloping terrain seed.
    ground_hits = []
    for obj in scene.objects:
        if obj.type == 'MESH' and obj.name.startswith(('Native terrain', 'Native context apron')):
            origin = obj.matrix_world.inverted() @ Vector((camera.location.x, camera.location.y, 10000))
            hit, location, _, _ = obj.ray_cast(origin, Vector((0, 0, -1)))
            if hit:
                ground_hits.append((obj.matrix_world @ location).z)
    if ground_hits:
        camera.location.z = max(ground_hits)+1.8

if VIEW in ('ground', 'oblique'):
    seat_eye_above_terrain()

# Keep the inspection camera outside all steel, including the far side of a
# paired chord. A camera inside that chord produces a misleading black disc.
def camera_clear(point):
    for m in members:
        if m[4] != 0:
            continue
        a, b = Vector(m[0]), Vector(m[1])
        axis = b-a
        t = max(0., min(1., (point-a).dot(axis)/axis.length_squared))
        if (point-a-axis*t).length < max(m[2:4]) + .65:
            return False
    return True

for _ in range(8):
    if camera_clear(camera.location):
        break
    camera.location = target + (camera.location-target) * 1.35
if not camera_clear(camera.location):
    raise RuntimeError('No clear external review camera position')
camera.rotation_euler = (target-camera.location).to_track_quat('-Z', 'Y').to_euler()
if VIEW in ('ground', 'oblique'):
    from bpy_extras.object_utils import world_to_camera_view
    # Frame the full structure after terrain seating; a sloping seed can move
    # the ground camera enough to clip a shoulder or its foundations.
    points = [Vector(p) for m in members for p in m[:2]]
    points += [Vector(p[0]) for p in data['track']]
    for _ in range(12):
        bpy.context.view_layer.update()
        projected = [world_to_camera_view(scene, camera, p) for p in points]
        if all(.045 < p.x < .955 and .045 < p.y < .955 and p.z > 0 for p in projected):
            break
        camera.location.x = target.x+(camera.location.x-target.x)*1.10
        camera.location.y = target.y+(camera.location.y-target.y)*1.10
        seat_eye_above_terrain()
        camera.rotation_euler = (target-camera.location).to_track_quat('-Z', 'Y').to_euler()
scene.render.threads = 8
scene.cycles.samples = 16
scene.render.filepath = str(root / f'{CASE}-{VIEW}.png')
scene['last_review_camera_clear'] = True
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        area.spaces.active.shading.type = 'SOLID'
        area.spaces.active.region_3d.view_perspective = 'CAMERA'
        area.spaces.active.region_3d.view_camera_zoom = 0

def render_once():
    bpy.ops.render.render(write_still=True)
    return None
if globals().get('RENDER_NOW', False):
    render_once()
else:
    bpy.app.timers.register(render_once, first_interval=.2)
print(json.dumps({'scene': scene.name, 'view': VIEW, 'camera_clear': True, 'path': scene.render.filepath}))
