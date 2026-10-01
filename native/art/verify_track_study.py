"""Focused checks on the actual study mesh buffers and interface dimensions."""
import copy
import json
import math
from collections import Counter
from pathlib import Path

from track_study_geometry import (cross, dot, sub, study_frame, support_mount, support_head_mesh, support_bearing_parts,
                                  support_bearing_web_mesh, swept_tube, transform, validate_profile)

root = Path(__file__).resolve().parent
profile = json.loads((root/"track_study_profile.json").read_text())
report = validate_profile(profile)
samples = 0
worst_gauge = 0.0
worst_radius = 0.0
for kind, length in (("straight", 11.2), ("banked", 22.4), ("inverted", 33.6)):
    rails = []
    for sign in (-1, 1):
        vertices, faces, bands = swept_tube(profile, kind, length,
            sign*profile["gauge"]/2, 0, profile["rail_radius"], profile["rail_wall"])
        assert all(math.isfinite(v) for point in vertices for v in point)
        assert len(bands) == len(faces)
        # Closed pipes must have exactly two incident faces at every edge,
        # including inner walls and both cut-end annuli.
        edges = Counter(tuple(sorted((face[i], face[(i+1)%len(face)])))
                        for face in faces for i in range(len(face)))
        assert set(edges.values()) == {2}, (kind, "pipe boundary/nonmanifold edge")
        sides = profile["ring_sides"]
        rings = len(vertices)//(sides*2)
        centroids = [tuple(sum(vertices[i*sides+j][a] for j in range(sides))/sides
                           for a in range(3)) for i in range(rings)]
        for shell, expected in enumerate((profile["rail_radius"],
                                           profile["rail_radius"]-profile["rail_wall"])):
            for i, centre in enumerate(centroids):
                for j in range(sides):
                    vertex = vertices[(shell*rings+i)*sides+j]
                    error = abs(math.dist(vertex, centre)-expected)
                    worst_radius = max(worst_radius, error)
                    assert error < 1e-10, "Rail radius or wall changed along the sweep"
        rails.append(centroids)
    for i, (left, right) in enumerate(zip(*rails)):
        distance = math.dist(left, right)
        worst_gauge = max(worst_gauge, abs(distance-1.4))
        assert abs(distance-1.4) < 1e-10
        frame = study_frame(kind, length*i/(len(rails[0])-1))
        _, f, r, u = frame
        assert abs(dot(cross(f, r), u)-1) < 1e-12, "Reflected or skew frame"
        rail_centre = tuple((a+b)/2 for a, b in zip(left, right))
        mount = transform(frame, support_mount(profile))
        expected_distance = profile["spine_depth"]+profile["spine_radius"]+profile["support_mount_drop"]
        assert abs(math.dist(rail_centre, mount)-expected_distance) < 1e-10
        samples += 1
# The replacement support must be a closed head without a neck contraction.
head_vertices, head_faces = support_head_mesh(profile)
head_edges = Counter(tuple(sorted((face[i], face[(i+1)%len(face)])))
                     for face in head_faces for i in range(len(face)))
assert set(head_edges.values()) == {2}, "Open or nonmanifold support head"
head_radii = [math.hypot(x, y) for x, y, z in head_vertices]
assert all(abs(r-profile["support_column_radius"]) < 1e-10 for r in head_radii)
assert all(math.isfinite(v) for point in head_vertices for v in point)
# The column is wider than the spine, so it must stop below it; extrapolating
# the smaller pipe's cope would create exposed horns outside the spine width.
assert max(z for x, y, z in head_vertices) < -profile["spine_depth"]-profile["spine_radius"]-.20
for web_x in (-.26, .26):
    vertices, faces = support_bearing_web_mesh(profile, web_x)
    edges = Counter(tuple(sorted((face[i], face[(i+1)%len(face)]))) for face in faces for i in range(len(face)))
    assert set(edges.values()) == {2}, "Bearing web is open"
    half = len(vertices)//2
    for x, y, z in vertices[2:half]:
        assert abs(y) < profile["spine_radius"], "Web extends beyond the pipe"
        penetration = profile["spine_radius"]-math.hypot(y, z+profile["spine_depth"])
        assert .003 < penetration < .009, "Bearing web does not seat into the spine"
    assert min(z for x, y, z in vertices) < max(z for x, y, z in head_vertices)+.05, "Web misses the load plate"
for name,(vertices,faces) in support_bearing_parts(profile):
    edges = Counter(tuple(sorted((face[i],face[(i+1)%len(face)]))) for face in faces for i in range(len(face)))
    assert set(edges.values()) == {2}, ('Open saddle part',name)
    assert all(math.isfinite(v) for point in vertices for v in point)
# Negative fixtures ensure rejection isn't merely a happy-path assertion.
for changes in ({"gauge": 1.3}, {"train_width": 2.41},
                {"crosshead_end_inset": -.04, "crosshead_radius": .20},
                {"rail_wall": .2}, {"rail_radius": .12}, {"crosshead_height": -.04}):
    broken = copy.deepcopy(profile)
    broken.update(changes)
    try:
        validate_profile(broken)
    except ValueError:
        continue
    raise AssertionError("Invalid profile was accepted: "+str(changes))
report.update(frameSamples=samples, maximumMeshGaugeErrorMetres=worst_gauge,
              railOutsideRadiusMetres=profile["rail_radius"],
              railWallMetres=profile["rail_wall"], maximumMeshRadiusErrorMetres=worst_radius,
              closedSupportHeads=1, supportHeadMinimumDiameterMetres=2*min(head_radii),
              closedRailShells=6, invalidProfilesRejected=6)
destination = root.parents[1]/"out/track-study/geometry-checks.json"
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(json.dumps(report, indent=2)+"\n")
print(json.dumps(report, indent=2))
