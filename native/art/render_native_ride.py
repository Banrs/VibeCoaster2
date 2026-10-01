"""Review the imported complete saved ride; camera work only, no model authoring.

Run through Blender MCP with SCENE_NAME, REGION_INDEX (0..n-1, 'cliff' or 'tallest'),
VIEW ('ground', 'oblique', 'joint' or 'anchor') and REVIEW_ROOT. All framing uses native export data.
"""
import bpy
import json
import math
from pathlib import Path
from mathutils import Vector

scene = bpy.data.scenes[SCENE_NAME]
bpy.context.window.scene = scene
data = json.loads(scene['native_data_json'])
terrain = next(o for o in scene.objects if o.name.startswith('Native terrain'))

def ground_at(x, y):
    hit, p, _, _ = terrain.ray_cast(Vector((x, y, 10000)), Vector((0, 0, -1)))
    return p.z if hit else min(v[2] for v in data['terrain']['points'])

if REGION_INDEX == 'cliff':
    selected = [s for s in data['supports'] if any(m[4] == 2 for m in s['members'])]
    if not selected:
        raise RuntimeError('No native rock anchors in this exported ride')
    apex = selected[len(selected)//2]
    label = 'cliff-wall'
    region = {'begin': min(s['distance'] for s in selected), 'end': max(s['distance'] for s in selected)}
elif REGION_INDEX == 'tallest':
    candidates = [s for s in data['supports'] if not any(
        r['begin'] <= s['distance'] <= r['end'] for r in data['regions'])]
    apex = max(candidates, key=lambda s: s['attachment'][2]-ground_at(*s['attachment'][:2]))
    selected = [s for s in data['supports'] if abs(s['distance']-apex['distance']) < 42]
    label = 'high-transition'
    region = {'begin': min(s['distance'] for s in selected), 'end': max(s['distance'] for s in selected)}
else:
    region = data['regions'][REGION_INDEX]
    selected = [s for s in data['supports'] if region['begin'] <= s['distance'] <= region['end']]
    apex = max(selected, key=lambda s: s['frame'][0][2])
    label = ('camelback', 'loop', 'immelmann', 'twisted-drop')[region['kind']]
    number = sum(r['kind'] == region['kind'] for r in data['regions'][:REGION_INDEX+1])
    if number > 1:
        label += '-'+str(number)

position, right, up = (Vector(p) for p in apex['frame'])
forward = up.cross(right).normalized()
camera_name = 'Native ride / '+label+' / '+VIEW
camera = scene.objects.get(camera_name)
if camera is None:
    camera = bpy.data.objects.new(camera_name, bpy.data.cameras.new(camera_name))
    scene.collection.objects.link(camera)
camera.data.clip_start = .05
camera.data.clip_end = 20000
camera.data.lens = 48 if VIEW in ('joint', 'anchor') else 42

if VIEW == 'anchor':
    anchor = next(m for m in apex['members'] if m[4] == 2)
    target = Vector(anchor[1])
    normal = (target-Vector(anchor[0])).normalized()
    side = normal.cross(Vector((0, 0, 1))).normalized()
    eye = target+normal*7+side*5+Vector((0, 0, 2))
elif VIEW == 'joint':
    target = Vector(apex['attachment'])-up*.3
    eye = position+right*4+forward*3.5-up*1.5
else:
    points = [Vector(p) for s in selected for m in s['members'] for p in m[:2]]
    points += [Vector(p[0]) for p in data['track'] if region['begin'] <= p[-1] <= region['end']]
    low = Vector(tuple(min(p[j] for p in points) for j in range(3)))
    high = Vector(tuple(max(p[j] for p in points) for j in range(3)))
    target = (low+high)*.5
    extent = max(high-low)
    across = Vector((right.x, right.y, 0))
    if across.length < .1:
        across = Vector((0, 1, 0))
    across.normalize()
    if REGION_INDEX == 'cliff':
        # Look towards the exposed face, not along the track's lateral frame.
        anchor = next(m for m in apex['members'] if m[4] == 2)
        across = Vector(anchor[1])-Vector(anchor[0])
        across.z = 0
        across.normalize()
    base_angle = math.atan2(across.y, across.x)
    if VIEW == 'oblique':
        base_angle += .42
    visibility_points = [Vector(s['attachment']) for s in selected]
    visibility_points += [Vector(m[1])+Vector((0, 0, .6))
                          for s in selected for m in s['members'] if m[4] in (1, 2)]
    # The complete ride must remain visible. Score actual terrain sightlines to
    # the foundations as well as the crest; a clear midpoint alone can conceal
    # half the structure behind a hill. Also avoid cameras beside unrelated rails.
    other_track = [Vector(p[0]) for p in data['track'][::8]
                   if p[-1] < region['begin']-30 or p[-1] > region['end']+30]
    options = []
    for scale in (2.2, 2.7):
        for step in (0, 1, -1, 2, -2, 4, -4, 6, -6, 8):
            angle = base_angle+step*math.pi/8
            candidate = target+Vector((math.cos(angle), math.sin(angle), 0))*extent*scale
            candidate.z = ground_at(candidate.x, candidate.y)+1.8
            ray = target-candidate
            hit, _, _, _ = terrain.ray_cast(candidate, ray.normalized(), distance=ray.length-.5)
            if not hit:
                obscured = 0
                for point in visibility_points:
                    sight = point-candidate
                    blocked, _, _, _ = terrain.ray_cast(candidate, sight.normalized(),
                                                        distance=max(.1, sight.length-1))
                    obscured += int(blocked)
                nearby = min(((p-candidate).length for p in other_track), default=1000)
                # An end-on view may expose every footing but collapse an
                # entire loop into a line. Keep the intended broadside/oblique
                # silhouette in the score, accepting small terrain occlusions.
                score = (obscured/max(1, len(visibility_points))*100
                         +(1-abs(math.cos(angle-base_angle)))*100
                         +max(0, 60-nearby)*5+abs(step)*.4+scale)
                options.append((score, candidate))
    if not options:
        raise RuntimeError('No visible external native-ride ground camera')
    eye = min(options, key=lambda item: item[0])[1]

camera.location = eye
camera.rotation_euler = (target-eye).to_track_quat('-Z', 'Y').to_euler()
# A wide element viewed obliquely can occupy very little of a fixed 42 mm frame.
# Fit the selected native bounds, with room around every extremity; do not hide
# neighbouring ride geometry to make a cleaner picture.
if VIEW not in ('joint', 'anchor'):
    facing = (target-eye).normalized()
    horizontal = facing.cross(Vector((0, 0, 1))).normalized()
    vertical = horizontal.cross(facing).normalized()
    sx = sy = 0.
    for point in points:
        delta = point-eye
        distance = delta.dot(facing)
        if distance > 1:
            sx = max(sx, abs(delta.dot(horizontal))/distance)
            sy = max(sy, abs(delta.dot(vertical))/distance)
    camera.data.lens = min(70., 15.0/max(sx, .01), (15.*770/1260)/max(sy, .01))
scene.camera = camera
scene.render.threads_mode = 'FIXED'
scene.render.threads = 8
scene.cycles.samples = 16
scene.render.resolution_x, scene.render.resolution_y = 1260, 770
scene.render.resolution_percentage = 100
path = Path('D:/Coding/Codex/Vibecoaster2')/REVIEW_ROOT/(label+'-'+VIEW+'.png')
scene.render.filepath = str(path)
for area in bpy.context.screen.areas:
    if area.type == 'VIEW_3D':
        area.spaces.active.shading.type = 'SOLID'
        area.spaces.active.region_3d.view_perspective = 'CAMERA'
        area.spaces.active.region_3d.view_camera_zoom = 15
bpy.context.view_layer.update()
if globals().get('RENDER_NOW', True):
    bpy.ops.render.render(write_still=True)
print(json.dumps({'scene': scene.name, 'source': scene['native_mesh_path'],
                  'region': region, 'view': VIEW, 'path': str(path)}))
