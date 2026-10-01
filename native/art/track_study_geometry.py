"""Pure geometry shared by the MCP authoring script and its offline checks.

The profile is a study contract, separate from the saved default.3 envelope.
Every repeated part uses the same orthonormal rail frame. A support generator
can consume support_mount() directly when this profile is adopted by the ride.
"""
import math


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def mul(a, value):
    return tuple(x * value for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(a):
    return mul(a, 1 / math.sqrt(dot(a, a)))


def ease(t):
    t = max(0.0, min(1.0, t))
    return t*t*t*(10+t*(-15+6*t))


def study_frame(kind, distance):
    if kind == "banked":
        radius = 48.0
        theta = distance / radius
        position = (radius*math.sin(theta), radius*(1-math.cos(theta)), 0.0)
        forward = (math.cos(theta), math.sin(theta), 0.0)
        transverse = (-math.sin(theta), math.cos(theta), 0.0)
        bank = math.radians(65)*ease(distance/22.4)
    else:
        position = (distance, 0.0, 0.0)
        forward, transverse = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
        bank = math.pi*ease(distance/33.6) if kind == "inverted" else 0.0
    up = (0.0, 0.0, 1.0)
    right = add(mul(transverse, math.cos(bank)), mul(up, math.sin(bank)))
    local_up = add(mul(up, math.cos(bank)), mul(transverse, -math.sin(bank)))
    return position, forward, right, local_up


def transform(frame, local):
    p, f, r, u = frame
    return add(p, add(mul(f, local[0]), add(mul(r, local[1]), mul(u, local[2]))))


def support_mount(profile):
    return (0.0, 0.0, -profile["spine_depth"]-profile["spine_radius"]-profile["support_mount_drop"])


def support_head_mesh(profile, sides=48):
    """Constant-diameter, flat-capped column below a fabricated bearing head."""
    radius = profile["support_column_radius"]
    bottom = support_mount(profile)[2]+profile["support_flange_size"][2]-.002
    top = -profile["spine_depth"]-profile["spine_radius"]-profile.get("support_cap_drop", .26)
    ring = [(radius*math.cos(2*math.pi*i/sides), radius*math.sin(2*math.pi*i/sides))
            for i in range(sides)]
    vertices = [(x, y, bottom) for x, y in ring]
    vertices += [(x, y, top) for x, y in ring]
    faces = [(i, (i+1)%sides, (i+1)%sides+sides, i+sides) for i in range(sides)]
    faces.append(tuple(reversed(range(sides))))
    faces.append(tuple(range(sides, 2*sides)))
    return vertices, faces


def support_bearing_web_mesh(profile, x, segments=24):
    """Thick transverse web welded to the spine, wholly inside its width.

    The wider column never wraps around a narrower spine. Two separate plate
    webs distribute the load from a full-width cap into the pipe's underside.
    """
    radius, depth = profile["spine_radius"], profile["spine_depth"]
    half_width = radius*.86
    bottom = -depth-radius-profile.get("support_cap_drop", .26)+profile.get("support_load_plate_thickness", .05)*.6
    outline = [(-half_width, bottom), (half_width, bottom)]
    for i in range(segments+1):
        y = half_width*(1-2*i/segments)
        outline.append((y, -depth-math.sqrt(radius*radius-y*y)+.008))
    n = len(outline)
    half_thickness = profile.get("support_web_thickness", .045)/2
    vertices = [(x+dx, y, z) for dx in (-half_thickness, half_thickness) for y, z in outline]
    faces = [tuple(reversed(range(n))), tuple(range(n, 2*n))]
    faces += [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)]
    return vertices, faces


def support_bearing_parts(profile):
    """Stiffened spine saddle, shared by the component kit and generated heads.

    Longitudinal cheeks spread the visible connection along the spine; two
    transverse diaphragms close the load path onto a full-width seat. All weld
    edges stay below the spine centreline, clear of rail/crosshead hardware.
    Dimensions are an art proposal informed by Intamin's external photographs.
    """
    radius, depth = profile['spine_radius'], profile['spine_depth']
    cap = -depth-radius-profile.get('support_cap_drop', .26)
    plate = profile.get('support_load_plate_thickness', .06)
    length = profile.get('support_saddle_length', 1.22)
    width = max(.86, 2*profile['support_column_radius']+.11)
    hx, hy, bevel = length/2, width/2, .075
    outline = [(-hx+bevel,-hy),(hx-bevel,-hy),(hx,-hy+bevel),
               (hx,hy-bevel),(hx-bevel,hy),(-hx+bevel,hy),
               (-hx,hy-bevel),(-hx,-hy+bevel)]
    vertices = [(x,y,z) for z in (cap-.002,cap+plate-.002) for x,y in outline]
    faces = [tuple(reversed(range(8))),tuple(range(8,16))]
    faces += [(i,(i+1)%8,(i+1)%8+8,i+8) for i in range(8)]
    parts = [('Connection / load plate', (vertices,faces))]
    xmin = max(-hx+.025,profile.get('support_weld_x_min',-hx))
    xmax = min(hx-.025,profile.get('support_weld_x_max',hx))
    for t in (.15,.85):
        x = xmin+(xmax-xmin)*t
        parts.append(('Connection / bearing web',support_bearing_web_mesh(profile,x)))
    half_thickness = profile.get('support_cheek_thickness', .055)/2
    bottom = cap+plate*.65
    # Each face meets the actual circular pipe, including across plate thickness.
    for side in (-1,1):
        vertices = []
        for y in (side*radius*.65-half_thickness,side*radius*.65+half_thickness):
            top = -depth-math.sqrt(radius*radius-y*y)+.008
            outline = [(xmin,bottom+.065),(xmin+.095,bottom),
                       (xmax-.095,bottom),(xmax,bottom+.065)]
            outline += [(xmax-(xmax-xmin)*i/8,top) for i in range(9)]
            vertices += [(x,y,z) for x,z in outline]
        n = len(outline)
        faces = [tuple(reversed(range(n))),tuple(range(n,2*n))]
        faces += [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
        parts.append(('Connection / longitudinal cheek', (vertices,faces)))
    return parts


def gusset_outline(profile):
    # XZ plane: broad welded roots on the spine, short crest under crosshead.
    root = -profile["spine_depth"]+math.sqrt(profile["spine_radius"]**2-profile["gusset_side"]**2)-.012
    top = profile["crosshead_height"]-profile["crosshead_radius"]+.008
    a, b = profile["gusset_half_length"], profile["gusset_top_half_length"]
    return [(-a, root), (-b, top), (b, top), (a, root)]


def gusset_mesh(profile, side):
    outline = gusset_outline(profile)
    half = profile["tie_plate_thickness"]/2
    y = side*profile["gusset_side"]
    vertices = [(x, y+dy, z) for dy in (-half, half) for x, z in outline]
    faces = [(3, 2, 1, 0), (4, 5, 6, 7)]
    faces += [(i, (i+1)%4, (i+1)%4+4, i+4) for i in range(4)]
    return vertices, faces


def tie_cross_section_solids(profile):
    # Conservative YZ projections of the crosshead and the two gussets.
    half = profile["gauge"]*.5-profile["crosshead_end_inset"]
    z, radius = profile["crosshead_height"], profile["crosshead_radius"]
    solids = [[(-half, z-radius), (half, z-radius), (half, z+radius), (-half, z+radius)]]
    outline = gusset_outline(profile)
    bottom, top = min(v[1] for v in outline), max(v[1] for v in outline)
    for side in (-1, 1):
        y, h = side*profile["gusset_side"], profile["tie_plate_thickness"]*.5
        solids.append([(y-h, bottom), (y+h, bottom), (y+h, top), (y-h, top)])
    return solids


def swept_tube(profile, kind, length, side, height, radius, wall):
    """Closed pipe shell, with visible wall thickness at both cut ends."""
    rings = max(2, math.ceil(length/profile["sweep_step"])+1)
    sides = profile["ring_sides"]
    vertices, faces, bands = [], [], []
    for shell_radius in (radius, radius-wall):
        for i in range(rings):
            frame = study_frame(kind, length*i/(rings-1))
            for j in range(sides):
                a = 2*math.pi*j/sides
                vertices.append(transform(frame, (0, side+shell_radius*math.cos(a),
                                                    height+shell_radius*math.sin(a))))
    layer = rings*sides
    for shell in (0, 1):
        offset = shell*layer
        for i in range(rings-1):
            for j in range(sides):
                k = (j+1) % sides
                face = (offset+i*sides+j, offset+i*sides+k,
                        offset+(i+1)*sides+k, offset+(i+1)*sides+j)
                faces.append(face if shell == 0 else tuple(reversed(face)))
                a = 2*math.pi*(j+.5)/sides
                contact = abs(math.sin(a)) > .96 or (math.cos(a)*(1 if side > 0 else -1) > .96)
                bands.append(1 if side and not shell and contact else 0)
    for i in (0, rings-1):
        for j in range(sides):
            k = (j+1) % sides
            faces.append((i*sides+j, layer+i*sides+j, layer+i*sides+k, i*sides+k))
            bands.append(1)
    return vertices, faces, bands


def wheel_envelopes(profile):
    """Full longitudinally swept YZ rectangles for the provisional wheels."""
    result = []
    rail, half, gap = profile["rail_radius"], profile["gauge"]*.5, profile["wheel_rail_gap"]
    for side in (-1, 1):
        for role in ("running", "upstop", "guide"):
            radius, width = profile[role+"_wheel_radius"], profile[role+"_wheel_width"]
            if role == "guide":
                y, z = side*(half+rail+radius+gap), 0
                hy, hz = radius, width/2
            else:
                y, z = side*half, (1 if role == "running" else -1)*(rail+radius+gap)
                hy, hz = width/2, radius
            result.append({"role": role, "side": side, "centre": (0, y, z),
                           "radius": radius, "width": width,
                           "yz_min": (y-hy, z-hz), "yz_max": (y+hy, z+hz)})
    return result


def segment_distance(a, b, p):
    d = sub(b, a)
    t = max(0.0, min(1.0, dot(sub(p, a), d)/dot(d, d)))
    v = sub(p, add(a, mul(d, t)))
    return math.sqrt(dot(v, v))


def polygons_distance(a, b):
    # Separating-axis intersection, then exact edge/vertex separation in 2D.
    separated = False
    for poly in (a, b):
        for i, point in enumerate(poly):
            edge = sub(poly[(i+1) % len(poly)], point)
            axis = (-edge[1], edge[0])
            aa, bb = [dot(v, axis) for v in a], [dot(v, axis) for v in b]
            if max(aa) < min(bb) or max(bb) < min(aa):
                separated = True
    if not separated:
        return 0.0
    return min(segment_distance(poly[i], poly[(i+1) % len(poly)], v)
               for poly, other in ((a, b), (b, a)) for i in range(len(poly)) for v in other)


def validate_profile(profile):
    if profile["units"] != "metres" or abs(profile["gauge"]-1.4) > 1e-9:
        raise ValueError("The track study requires 1.40 m rail-centre gauge in metres")
    if not 0 < profile["train_width"] <= profile["maximum_train_width"] <= 2.4:
        raise ValueError("Train study must fit the retained 2.40 m maximum")
    for key in ("rail_radius", "spine_radius", "tie_spacing", "sweep_step"):
        if not math.isfinite(profile[key]) or profile[key] <= 0:
            raise ValueError("Invalid dimension: " + key)
    if abs(profile["rail_radius"]-.105) > 1e-9:
        raise ValueError("The requested running-rail radius is 105 mm")
    if not math.isfinite(profile["crosshead_height"]) or abs(profile["crosshead_height"]) > 1e-9:
        raise ValueError("Crosshead axis must pass through both rail centres")
    for tube in ("rail", "spine"):
        if not 0 < profile[tube+"_wall"] < profile[tube+"_radius"]:
            raise ValueError("Invalid pipe wall")
    gap = math.inf
    for wheel in wheel_envelopes(profile):
        lo, hi = wheel["yz_min"], wheel["yz_max"]
        rectangle = [lo, (hi[0], lo[1]), hi, (lo[0], hi[1])]
        for poly in tie_cross_section_solids(profile):
            gap = min(gap, polygons_distance(poly, rectangle))
        if max(abs(lo[0]), abs(hi[0])) > profile["train_width"]/2:
            raise ValueError("Wheel placeholders exceed the train width")
    if gap < .005:
        raise ValueError("Tie-to-wheel sweep gap under 5 mm: " + str(gap))
    return {"gaugeMetres": profile["gauge"], "minimumTieWheelSweepGapMetres": gap,
            "crossheadAxisHeightMetres": profile["crosshead_height"],
            "supportColumnDiameterMetres": 2*profile["support_column_radius"],
            "supportMountLocalMetres": support_mount(profile),
            "scope": "local cross-section and provisional wheel barrels; excludes a bogie or occupied-train sweep"}
