"""Read-only verification of the actual generated Blender support scenes via MCP."""
import bpy
import json
import math
from mathutils import Vector

PROFILE = json.loads(__TRACK_PROFILE__)
reports = []
for scene in bpy.data.scenes:
    if not scene.name.startswith(('Support System / ','Support Joint Review / ')):
        continue
    data = json.loads(scene['native_data_json'])
    expected = sorted(json.dumps(m) for s in data['supports'] for m in s['members'])
    actual = sorted(o['native_member'] for o in scene.objects if 'native_member' in o)
    assert expected == actual, 'Native member metadata mismatch: '+scene.name
    mounts = [o for o in scene.objects if o.name.startswith('Track mount / ')]
    assert len(mounts) == 2*len(data['supports']), 'Missing mount geometry'
    hidden = [o for o in scene.objects if o.name.startswith('Native envelope / ')]
    assert len(hidden) == 2*len(data['supports']) and all(o.hide_render for o in hidden)
    cameras = sum(o.type == 'CAMERA' for o in scene.objects)
    assert cameras == 7 if scene.name.startswith('Support System / ') else cameras >= 6
    stations = {q[3]:i for i,q in enumerate(data['track'])}
    indices = sorted(set([0,len(data['track'])-1]+list(range(0,len(data['track']),80))+
                         [stations[s['distance']] for s in data['supports']]))
    maximum_error = 0.
    for side,label in ((-1,'left'),(1,'right')):
        obj = next(o for o in scene.objects if o.name.startswith('Study rail / 105 mm radius / '+label))
        count = len(data['track'])
        assert len(obj.data.vertices) == count*36*2, 'Incomplete track sweep'
        for i in indices:
            position,right,up,station = data['track'][i]
            offset = PROFILE['spine_depth']+PROFILE['spine_radius']-data['nativeSpineDepth']-data['nativeSpineRadius']
            centre = Vector(position)+Vector(up)*offset+Vector(right)*(side*PROFILE['gauge']/2)
            for shell,radius in enumerate((PROFILE['rail_radius'],PROFILE['rail_radius']-PROFILE['rail_wall'])):
                for j in range(36):
                    v = obj.data.vertices[(shell*count+i)*36+j].co
                    maximum_error = max(maximum_error,abs((v-centre).length-radius))
    # Blender mesh buffers use float32 at scene coordinates of several hundred
    # metres; this tolerance is below a millimetre and detects a radius change.
    assert maximum_error < .0002, 'Authored rail radius differs from the profile'
    reports.append({'scene':scene.name,'nativeMembers':len(actual),'trackMounts':len(mounts)//2,'cameras':cameras,
                    'sampledRailRings':len(indices)*2,'maximumRailRadiusErrorMetres':maximum_error,
                    'nativeDataCharacters':len(scene['native_data_json']),
                    'nativeDataChecksum':sum((i+1)*ord(c) for i,c in enumerate(scene['native_data_json'])),
                    'fabrication':json.loads(scene['fabrication_audit'])})
assert sum(r['scene'].startswith('Support System / ') for r in reports) == 4, 'Expected four generated support scenes'
assert sum(r['scene'].startswith('Support Joint Review / ') for r in reports) == 1, 'Expected the retained angled-head check'
print('SUPPORT_SCENE_AUDIT '+json.dumps({'passed':True,'scenes':reports}))
