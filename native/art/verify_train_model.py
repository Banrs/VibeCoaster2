"""Read-only checks of the actual evaluated Blender train and straight rail fit.

Execute through Blender MCP after author_train.py. This verifies visual-art
geometry, not train dynamics, engineering certification or a full-ride sweep.
"""
import bpy
import bmesh
import json
import math
import runpy
from pathlib import Path
from collections import Counter
from mathutils import Vector
from mathutils.bvhtree import BVHTree

ROOT=Path(globals().get('ROOT','D:/Coding/Codex/Vibecoaster2'))
scene=bpy.data.scenes['Riftwake / Train design']
P=json.loads(scene['train_profile'])
tp=json.loads((ROOT/'native/art/track_study_profile.json').read_text())
geometry=runpy.run_path(str(ROOT/'native/art/track_study_geometry.py'))
deps=bpy.context.evaluated_depsgraph_get()
roots=sorted((o for o in scene.objects if o.get('role')=='car_root'),key=lambda o:o['car_index'])
assert len(roots)==P['cars']==7
assert all(abs(root.location.x+i*P['car_pitch'])<1e-5 for i,root in enumerate(roots))
objects=[o for o in scene.objects if o.parent in roots and o.type in {'MESH','CURVE','FONT'}]
counts=Counter(o.get('role') for o in objects)
bounds=[Vector((math.inf,math.inf,math.inf)),Vector((-math.inf,-math.inf,-math.inf))]
unique={}; evaluated={}; total_triangles=0; welded_curves=0
for obj in objects:
    key=obj.data.as_pointer()
    if key not in unique:
        e=obj.evaluated_get(deps)
        mesh=e.to_mesh()
        bm=bmesh.new(); bm.from_mesh(mesh)
        # Blender evaluates curve end caps with coincident, separate rings.
        # Test their geometric closure after welding only those cap seams.
        if obj.type=='CURVE':
            before=len(bm.verts)
            bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-7)
            welded_curves+=len(bm.verts)<before
        assert all(edge.is_manifold for edge in bm.edges), 'Open/nonmanifold mesh: '+obj.name
        volume=bm.calc_volume(signed=True)
        assert volume>1e-12, 'Nonpositive mesh volume: '+obj.name
        bm.free()
        vv=[v.co.copy() for v in mesh.vertices]
        assert all(math.isfinite(c) for v in vv for c in v), 'Non-finite vertex: '+obj.name
        mesh.calc_loop_triangles()
        ff=[tuple(t.vertices) for t in mesh.loop_triangles]
        unique[key]=(vv,ff)
        e.to_mesh_clear()
    vv,ff=unique[key]
    world=[obj.matrix_world@v for v in vv]
    total_triangles+=len(ff)
    for axis in range(3):
        bounds[0][axis]=min(bounds[0][axis],min(v[axis] for v in world))
        bounds[1][axis]=max(bounds[1][axis],max(v[axis] for v in world))
    if obj.parent==roots[0]: evaluated[obj.name]=(world,ff)
width=bounds[1].y-bounds[0].y
assert width<=P['overall_width']+2e-5, ('Width exceeds proposal',width)
assert width<=P['maximum_width']
clamps=[o for o in objects if 'transverse C clamp' in o.name]
assert len(clamps)==4*P['cars']
assert not any('outer fork' in o.name or 'bearing bridge' in o.name for o in objects)
for obj in clamps:
    vv,_=unique[obj.data.as_pointer()]
    assert max(v.x for v in vv)-min(v.x for v in vv)<=.10001
    assert obj['clamp_plane']=='YZ / across rail; open inboard'

# Full longitudinal projection of each lead-car part against the centred
# crosshead and gusset solids is conservative for a straight track at any phase.
obstacles=geometry['tie_cross_section_solids'](tp)
minimum=math.inf; closest=None
for name,(vv,_) in evaluated.items():
    lo=[min(v[a] for v in vv) for a in (1,2)]
    hi=[max(v[a] for v in vv) for a in (1,2)]
    rect=[lo,(hi[0],lo[1]),hi,(lo[0],hi[1])]
    if hi[0]-lo[0]<1e-9 or hi[1]-lo[1]<1e-9: continue
    for obstacle in obstacles:
        distance=geometry['polygons_distance'](rect,obstacle)
        if distance<minimum: minimum,closest=distance,name
assert minimum>.005, ('Crosshead/gusset sweep collision',minimum,closest)

# Exact triangle intersections against the displayed track (rails, spine,
# crossheads and gussets). Also derive the gauge from the actual rail bounds.
track_v=[]; track_f=[]; rail_centres=[]
for obj in scene.objects:
    if obj.get('role')!='reference' or obj.type!='MESH' or any(s in obj.name for s in ('support','base')): continue
    data=obj.data
    data.calc_loop_triangles()
    world=[obj.matrix_world@v.co for v in data.vertices]
    if 'running rail' in obj.name:
        rail_centres.append((min(v.y for v in world)+max(v.y for v in world))/2)
    start=len(track_v);track_v.extend(world)
    track_f.extend(tuple(start+i for i in t.vertices) for t in data.loop_triangles)
gauge=max(rail_centres)-min(rail_centres)
assert abs(gauge-1.4)<1e-6
track_bvh=BVHTree.FromPolygons(track_v,track_f,all_triangles=True)
intersections=[]
lead_v=[];lead_f=[]
for name,(vv,ff) in evaluated.items():
    bvh=BVHTree.FromPolygons(vv,ff,all_triangles=True)
    overlaps=bvh.overlap(track_bvh)
    if overlaps: intersections.append({'object':name,'triangle_pairs':len(overlaps)})
    offset=len(lead_v);lead_v.extend(vv)
    lead_f.extend(tuple(offset+i for i in f) for f in ff)
assert not intersections, intersections

# Forward sight fan around the horizon from both front seats. Transparent lip
# is included, making this stricter than visibility through its material.
lead_bvh=BVHTree.FromPolygons(lead_v,lead_f,all_triangles=True)
blocked=[];rays=0
for side in (-1,1):
    eye=Vector((P['rider_eye'][0],side*abs(P['rider_eye'][1]),P['rider_eye'][2]))
    for yaw in range(-25,26,5):
        for pitch in range(-10,16,5):
            ya,pa=math.radians(yaw),math.radians(pitch)
            direction=Vector((math.cos(pa)*math.cos(ya),math.cos(pa)*math.sin(ya),math.sin(pa)))
            rays+=1
            hit=lead_bvh.ray_cast(eye,direction,20)
            if hit[0] is not None: blocked.append({'seat':side,'yaw':yaw,'pitch':pitch,'distance':hit[3]})
assert not blocked, blocked
report={
    'status':'passed', 'scope':'editable Blender art and straight Exa track fit only',
    'profile':P['id'],'cars':len(roots),'seats':len([o for o in objects if 'sculpted shell' in o.name]),
    'gauge_metres':gauge,'width_metres':width,'bounds_metres':[list(v) for v in bounds],
    'car_pitch_metres':P['car_pitch'],'unique_closed_meshes':len(unique),
    'clamp_count':len(clamps),'clamp_orientation':'transverse YZ; 100 mm along travel',
    'curve_cap_seams_welded_for_closure_check':welded_curves,
    'train_triangles':total_triangles,'object_roles':dict(counts),
    'minimum_crosshead_gusset_swept_gap_metres':minimum,'nearest_part':closest,
    'track_triangle_intersections':intersections,
    'clear_forward_sight_rays':rays,'sight_yaw_degrees':[-25,25],'sight_pitch_degrees':[-10,15],
    'limitations':['No occupied-train sweep on the complete ride','No station/hardware fit or runtime integration',
                   'Art dimensions are proposals, not manufacturer measurements','No structural or restraint engineering assessment']}
(ROOT/'out/train-model/geometry-checks.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
