"""Probe actual tube-end surfaces against neighbouring native meshes in Blender.

Run through Blender MCP after setting CASE and REVIEW_ROOT. This catches exposed
end rings that a centreline connectivity check cannot see.
"""
import json
import math
from collections import defaultdict
from pathlib import Path
from mathutils import Vector
from mathutils.bvhtree import BVHTree

folder = Path('D:/Coding/Codex/Vibecoaster2') / REVIEW_ROOT / globals().get('DATA_FOLDER', 'fixtures')
graph = json.loads((folder / f'{CASE}.json').read_text())
mesh_path = folder / f'{CASE}-joints.json'
if not mesh_path.exists():
    mesh_path = folder / f'{CASE}-fabrication.json'
mesh_data = json.loads(mesh_path.read_text())
parts = {(p['owner'], p['member']): p for p in mesh_data['parts']
         if p['name'] == 'Fitted tube' and p['member'] >= 0}

def triangles(part):
    return [(part['vertices'][f[0]], part['vertices'][f[j]], part['vertices'][f[j+1]])
            for f in part['faces'] for j in range(1, len(f)-1)]

def tree_for(part):
    return BVHTree.FromPolygons(part['vertices'],
           [(f[0], f[j], f[j+1]) for f in part['faces'] for j in range(1, len(f)-1)], all_triangles=True)

trees = {key: tree_for(p) for key, p in parts.items()}
exact_triangles = {key: triangles(p) for key, p in parts.items()}

# Blender's single-precision BVH can select the wrong side of a narrow, long
# triangle (observed on a 24 m tapered chord). Confirm potential failures in
# double precision before reporting a hole. Keep the fast BVH broad phase.
def add(a, b): return tuple(x+y for x, y in zip(a, b))
def sub(a, b): return tuple(x-y for x, y in zip(a, b))
def mul(a, t): return tuple(x*t for x in a)
def dot(a, b): return sum(x*y for x, y in zip(a, b))
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def closest_triangle(p, a, b, c):
    ab, ac, ap = sub(b, a), sub(c, a), sub(p, a)
    d1, d2 = dot(ab, ap), dot(ac, ap)
    if d1 <= 0 and d2 <= 0: return a
    bp = sub(p, b)
    d3, d4 = dot(ab, bp), dot(ac, bp)
    if d3 >= 0 and d4 <= d3: return b
    vc = d1*d4-d3*d2
    if vc <= 0 and d1 >= 0 and d3 <= 0: return add(a, mul(ab, d1/(d1-d3)))
    cp = sub(p, c)
    d5, d6 = dot(ab, cp), dot(ac, cp)
    if d6 >= 0 and d5 <= d6: return c
    vb = d5*d2-d1*d6
    if vb <= 0 and d2 >= 0 and d6 <= 0: return add(a, mul(ac, d2/(d2-d6)))
    va = d3*d6-d5*d4
    if va <= 0 and d4-d3 >= 0 and d5-d6 >= 0:
        return add(b, mul(sub(c, b), (d4-d3)/((d4-d3)+(d5-d6))))
    inv = 1/(va+vb+vc)
    return add(a, add(mul(ab, vb*inv), mul(ac, vc*inv)))

def exact_distance(point, tris):
    best, sign = math.inf, 1
    for a, b, c in tris:
        normal = cross(sub(b, a), sub(c, a))
        if dot(normal, normal) < 1e-20:
            continue
        delta = sub(point, closest_triangle(point, a, b, c))
        distance = dot(delta, delta)
        if distance < best:
            best, sign = distance, 1 if dot(delta, normal) > 0 else -1
    return math.sqrt(best)*sign

cans = []
for p in mesh_data['parts']:
    if p['name'] == 'Welded node can':
        lo = tuple(min(v[k] for v in p['vertices']) for k in range(3))
        hi = tuple(max(v[k] for v in p['vertices']) for k in range(3))
        cans.append((lo, hi, tree_for(p), triangles(p)))
nodes = defaultdict(list)
foundations = set()
for i, support in enumerate(graph['supports']):
    for j, member in enumerate(support['members']):
        if member[4] in (1, 2):
            foundations.add(tuple(round(v, 5) for v in member[1]))
        if (i, j) in parts:
            for end in (0, 1):
                nodes[tuple(round(v, 5) for v in member[end])].append(((i, j), end))

failures = []
probes = 0
precision_refinements = 0
for position, entries in nodes.items():
    if len(entries) < 2 or position in foundations:
        continue
    local_cans = [(tree, tris) for lo, hi, tree, tris in cans if all(lo[k] <= position[k] <= hi[k] for k in range(3))]
    for key, end in entries:
        part = parts[key]
        face = part['faces'][-2 + end]
        ring = [tuple(part['vertices'][i]) for i in face]
        worst = 0.0
        for precise_point in ring + [mul(add(a, b), .5) for a, b in zip(ring, ring[1:] + ring[:1])]:
            point = Vector(precise_point)
            probes += 1
            distances = []
            for neighbour, _ in entries:
                if neighbour == key:
                    continue
                nearest, normal, _, distance = trees[neighbour].find_nearest(point)
                distances.append(distance if (point - nearest).dot(normal) > 0 else -distance)
            for tree, _ in local_cans:
                nearest, normal, _, distance = tree.find_nearest(point)
                distances.append(distance if (point-nearest).dot(normal) > 0 else -distance)
            distance = min(distances)
            if distance > .008:
                precision_refinements += 1
                distance = min([exact_distance(precise_point, exact_triangles[neighbour])
                                for neighbour, _ in entries if neighbour != key]
                               + [exact_distance(precise_point, tris) for _, tris in local_cans])
            worst = max(worst, distance)
        if worst > .012:
            failures.append({'owner': key[0], 'member': key[1], 'end': end,
                             'node': position, 'exposed_mm': round(worst * 1000, 3)})
report = {'case': CASE, 'probes': probes, 'exposed_ends': len(failures),
          'tolerance_mm': 12, 'precision_refinements': precision_refinements,
          'failures': sorted(failures, key=lambda f: -f['exposed_mm'])}
(Path('D:/Coding/Codex/Vibecoaster2') / REVIEW_ROOT / f'{CASE}-joint-audit.json').write_text(json.dumps(report, indent=2))
print(json.dumps({**report, 'failures': report['failures'][:8]}))
if failures and not globals().get('ALLOW_FAILURES', False):
    raise RuntimeError(f"{CASE}: {len(failures)} exposed tube ends")
