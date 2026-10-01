"""Pure fabrication geometry for the generated Blender support study.

These are proposed art fittings. The native collision members remain separately
available; fitting radii and fabrication hardware have not migrated into them.
"""
import math
from bisect import bisect_right
from track_study_geometry import support_head_mesh, support_bearing_parts


def sd_add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def sd_sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def sd_mul(a, k):
    return tuple(x*k for x in a)


def sd_dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def sd_cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def sd_unit(v):
    length = math.sqrt(sd_dot(v, v))
    if length < 1e-9:
        raise ValueError('Degenerate fabrication axis')
    return sd_mul(v, 1/length)


def sd_basis(a, b):
    axis = sd_unit(sd_sub(b, a))
    right = sd_unit(sd_cross(axis, (0, 0, 1) if abs(axis[2]) < .9 else (0, 1, 0)))
    return axis, right, sd_cross(axis, right)


def sd_tube(a, b, r0, r1=None, sides=32):
    r1 = r0 if r1 is None else r1
    axis, right, up = sd_basis(a, b)
    vertices = [sd_add(p, sd_mul(sd_add(sd_mul(right, math.cos(2*math.pi*j/sides)), sd_mul(up, math.sin(2*math.pi*j/sides))), radius))
                for p, radius in ((a, r0), (b, r1)) for j in range(sides)]
    faces = [(j, (j+1)%sides, (j+1)%sides+sides, j+sides) for j in range(sides)]
    faces += [tuple(reversed(range(sides))), tuple(range(sides, 2*sides))]
    return vertices, faces


def sd_seated_tube(a, b, r0, r1, height, sides=32):
    """Intersect a tapered inclined pipe with a horizontal bearing surface."""
    axis, right, up = sd_basis(a,b)
    if axis[2] <= .05:
        raise ValueError('Foundation member must rise away from its bearing')
    length = math.dist(a,b)
    taper = (r1-r0)/length
    vertices = []
    for j in range(sides):
        angle = 2*math.pi*j/sides
        radial = sd_add(sd_mul(right,math.cos(angle)),sd_mul(up,math.sin(angle)))
        t = (height-a[2]-radial[2]*r0)/(axis[2]+radial[2]*taper)
        vertices.append(sd_add(a,sd_add(sd_mul(axis,t),sd_mul(radial,r0+taper*t))))
    vertices += [sd_add(b,sd_mul(sd_add(sd_mul(right,math.cos(2*math.pi*j/sides)),
                           sd_mul(up,math.sin(2*math.pi*j/sides))),r1)) for j in range(sides)]
    faces = [(j,(j+1)%sides,(j+1)%sides+sides,j+sides) for j in range(sides)]
    faces += [tuple(reversed(range(sides))),tuple(range(sides,2*sides))]
    return vertices,faces


def foundation_plate_thickness(member):
    return min(.09,max(.045,member[3]*.065))


def foundation_art_member(member, adjoining):
    """Fit the pedestal top inside its retained ground footprint."""
    result = list(member)
    for step in range(4):
        height = result[1][2]+foundation_plate_thickness(result)-.002
        for m in adjoining:
            vertices,_ = sd_seated_tube(m[0],m[1],m[2],m[3],height)
            reach = max(math.hypot(v[0]-result[1][0],v[1]-result[1][1]) for v in vertices[:32])
            result[3] = max(result[3],(reach+.14)/.94)
    if result[3] > result[2]-.05:
        raise ValueError('Fabricated bearing exceeds the retained foundation footprint')
    return result


def sd_ring(a, b, inner, outer, sides=40):
    axis, right, up = sd_basis(a, b)
    vertices = [sd_add(p, sd_mul(sd_add(sd_mul(right, math.cos(2*math.pi*j/sides)), sd_mul(up, math.sin(2*math.pi*j/sides))), radius))
                for p in (a, b) for radius in (inner, outer) for j in range(sides)]
    faces = []
    for j in range(sides):
        k = (j+1)%sides
        faces += [(j, k, k+sides, j+sides), (j+2*sides, j+3*sides, k+3*sides, k+2*sides),
                  (j, j+2*sides, k+2*sides, k), (j+sides, k+sides, k+3*sides, j+3*sides)]
    return vertices, faces


def sd_box(centre, size):
    vertices = [tuple(centre[i]+sign[i]*size[i]/2 for i in range(3))
                for sign in ((-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1))]
    return vertices, [(3,2,1,0),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]


def sd_part(name, mesh, material='steel'):
    return {'name':name, 'vertices':mesh[0], 'faces':mesh[1], 'material':material}


def sd_bolt(parts, point, axis, length, radius, label='Bolt'):
    axis = sd_unit(axis)
    parts.append(sd_part(label+' / shank', sd_tube(point, sd_add(point, sd_mul(axis, length)), radius*.64, sides=12), 'fastener'))
    for sign, tip in ((-1, point), (1, sd_add(point, sd_mul(axis, length)))):
        washer = sd_add(tip, sd_mul(axis, sign*.010))
        nut = sd_add(washer, sd_mul(axis, sign*.030))
        parts.append(sd_part(label+' / washer', sd_tube(tip, washer, radius*1.5, sides=20), 'fastener'))
        parts.append(sd_part(label+' / hex', sd_tube(washer, nut, radius, sides=6), 'fastener'))


def support_connection_detail(profile, support, track_data=None):
    """Orient a full-width fabricated head at an exact generated attachment."""
    if 'frame' not in support:
        raise ValueError('Regenerate support_review format 2 for exact joint frames')
    members = support['members']
    contacts = [i for i, m in enumerate(members) if m[5]]
    if len(contacts) != 1:
        raise ValueError('Each attachment requires one canonical spine contact')
    contact_index = contacts[0]
    contact = members[contact_index]
    parents = [i for i, m in enumerate(members) if not m[4] and not m[5] and math.dist(m[1], contact[0]) < 1e-6]
    if len(parents) != 1:
        raise ValueError('Fabricated head requires one incoming native stem')
    available = math.dist(contact[0], contact[1])
    p = dict(profile)
    radius = max([profile['support_column_radius']]+[max(members[i][2:4]) for i in parents])
    p['support_column_radius'] = radius
    parent = members[parents[0]]
    incoming_direction = sd_unit(sd_sub(parent[1],parent[0]))
    alignment = max(-1.,min(1.,sd_dot(incoming_direction,sd_unit(support['frame'][2]))))
    inline_stem = alignment > 1-1e-8
    p['support_mount_drop'] = min(profile['support_mount_drop'], available*.60)
    if not inline_stem:
        elbow_length = radius*math.tan(math.acos(alignment)/2)+.065
        p['support_mount_drop'] = min(p['support_mount_drop'],available-elbow_length)
        if p['support_mount_drop'] < .16:
            raise ValueError('Insufficient native contact length for a full-width inclined head')
    p['support_cap_drop'] = min(.26, p['support_mount_drop']*.50)
    p['support_load_plate_thickness'] = min(profile.get('support_load_plate_thickness',.06), p['support_cap_drop']*.4)
    thickness = min(profile['support_flange_size'][2], available*.15)
    lower_thickness = min(.045, thickness)
    flange_radius = max(profile['support_flange_size'][0]/2, radius+.15)
    p['support_flange_size'] = [flange_radius*2, flange_radius*2, thickness]
    underside = -p['spine_depth']-p['spine_radius']
    mount = underside-p['support_mount_drop']
    cap = underside-p['support_cap_drop']
    right, up = support['frame'][1:3]
    right, up = sd_unit(right), sd_unit(up)
    forward = sd_unit(sd_cross(right, up))
    stations = [q[3] for q in track_data['track']] if track_data else []
    station_direction = 1.
    if stations:
        i = min(max(1,bisect_right(stations,support['distance'])),len(stations)-1)
        tangent = sd_sub(track_data['track'][i][0],track_data['track'][i-1][0])
        station_direction = 1. if sd_dot(tangent,forward) > 0 else -1.
        limits = sorted(station_direction*(s-support['distance']) for s in (stations[0],stations[-1]))
        p['support_weld_x_min'],p['support_weld_x_max'] = limits
    parts = [sd_part('Column / constant diameter head', support_head_mesh(p))]
    parts.append(sd_part('Connection / upper flange', sd_ring((0,0,mount), (0,0,mount+thickness), radius-.003, flange_radius)))
    parts.append(sd_part('Connection / lower flange', sd_ring((0,0,mount-lower_thickness), (0,0,mount-.001), radius-.003, flange_radius)))
    parts += [sd_part(name,mesh) for name,mesh in support_bearing_parts(p)]
    bolt_circle = (radius+flange_radius)/2
    bolt_count = max(8, 2*math.ceil(math.pi*bolt_circle/.25))
    for j in range(bolt_count):
        a = 2*math.pi*j/bolt_count
        sd_bolt(parts, (bolt_circle*math.cos(a),bolt_circle*math.sin(a),mount-lower_thickness),
                (0,0,1), lower_thickness+thickness, .024, 'Connection bolt')
    origin = sd_add(support['attachment'], sd_mul(up, -underside))
    def world(v):
        return sd_add(origin, sd_add(sd_mul(forward, v[0]), sd_add(sd_mul(right, v[1]), sd_mul(up, v[2]))))
    def weld_surface(v):
        # Sample the native curve around the attachment, rather than leaving a
        # tangent-plane saddle floating above a curved or rolling spine.
        station = max(stations[0],min(stations[-1],support['distance']+station_direction*v[0]))
        index = min(max(1,bisect_right(stations,station)),len(stations)-1)
        a,b = track_data['track'][index-1:index+1]
        t = (station-a[3])/(b[3]-a[3])
        def lerp(x,y):
            return sd_add(sd_mul(x,1-t),sd_mul(y,t))
        position = lerp(a[0],b[0])
        r,u = sd_unit(lerp(a[1],b[1])),sd_unit(lerp(a[2],b[2]))
        centre_offset = p['spine_radius']-track_data['nativeSpineDepth']-track_data['nativeSpineRadius']
        z = centre_offset-math.sqrt(p['spine_radius']**2-v[1]**2)+.008
        return sd_add(position,sd_add(sd_mul(r,v[1]),sd_mul(u,z)))
    for part in parts:
        part['weld_vertices'] = [i for i,v in enumerate(part['vertices'])
                                if ('web' in part['name'] or 'cheek' in part['name']) and v[2] > underside]
        welded = set(part['weld_vertices'])
        part['vertices'] = [weld_surface(v) if stations and i in welded else world(v)
                            for i,v in enumerate(part['vertices'])]
    # Continue the incoming column at the same diameter into this head. The
    # original tapered contact members are kept as hidden reference envelopes.
    replacement_members = {}
    for i in parents:
        m = members[i]
        replacement_members[i] = [m[0], m[1], max(m[2],radius), radius, m[4], m[5]]
        # One continuous surface across the native stem/contact boundary avoids
        # a visible seam between separately phased polygon rings on that axis.
        if inline_stem:
            parts.append(sd_part('Column / incoming member and continuation',
                                 sd_tube(m[0], world((0,0,mount+.002)), radius)))
        else:
            parts.append(sd_part('Column / incoming member',sd_tube(m[0],m[1],radius)))
            parts.append(sd_part('Column / continuation',sd_tube(m[1],world((0,0,mount+.002)),radius)))
    return {'parts':parts, 'replaced':[contact_index]+parents, 'radius':radius,
            'replacement_members':replacement_members,
            'origin':origin, 'up':up, 'right':right, 'forward':forward,
            'mount':world((0,0,mount)), 'continuation_end':world((0,0,mount+.002)),
            'available':available, 'profile':p, 'inline_stem':inline_stem}


def support_fabrication_plan(profile, data):
    details = [support_connection_detail(profile,s,data) for s in data['supports']]
    members = [[d['replacement_members'].get(j,m) for j,m in enumerate(s['members'])]
               for s,d in zip(data['supports'],details)]
    all_members = [m for group in members for m in group]
    foundations = {}
    for group in members:
        for j,m in enumerate(group):
            if m[4]:
                adjoining = [a for a in all_members if not a[4] and math.dist(a[0],m[1]) < 1e-6]
                group[j] = foundation_art_member(m,adjoining)
                foundations[tuple(m[1])] = group[j]
    for detail in details:
        incoming = next(iter(detail['replacement_members'].values()))
        foundation = foundations.get(tuple(incoming[0]))
        if foundation is not None:
            height = foundation[1][2]+foundation_plate_thickness(foundation)-.002
            end = detail['continuation_end'] if detail['inline_stem'] else incoming[1]
            mesh = sd_seated_tube(incoming[0],end,detail['radius'],detail['radius'],height)
            for part in detail['parts']:
                if part['name'] in ('Column / incoming member and continuation','Column / incoming member'):
                    part['vertices'],part['faces'] = mesh
    return {'details':details,'members':members,'foundations':foundations}


def fit_support_fabrication(fabrication):
    fitted = fitted_support_members(fabrication)
    for i,detail in enumerate(fabrication['details']):
        parent = next(iter(detail['replacement_members']))
        for part in detail['parts']:
            if part['name'] in ('Column / incoming member and continuation','Column / incoming member'):
                part['vertices'],part['faces'] = fitted['meshes'][(i,parent)]
            elif part['name'] == 'Column / continuation':
                part['vertices'],part['faces'] = fitted['meshes'][(i,-1)]
    return fitted


def member_splice_details(member, maximum_section=12.):
    a, b, r0, r1, concrete, contact = member
    length = math.dist(a, b)
    if concrete or contact or length < maximum_section or max(r0,r1) < .28:
        return []
    axis, right, up = sd_basis(a, b)
    parts = []
    count = math.ceil(length/maximum_section)
    for j in range(1,count):
        u = j/count
        centre = sd_add(a, sd_mul(sd_sub(b,a),u))
        radius = r0+(r1-r0)*u
        outer = radius+max(.12,radius*.18)
        thickness = min(.075,max(.028,radius*.07))
        for lo, hi in ((-thickness,-.001),(.001,thickness)):
            parts.append(sd_part('Pipe splice / flange', sd_ring(sd_add(centre,sd_mul(axis,lo)),sd_add(centre,sd_mul(axis,hi)),radius-.003,outer)))
        circle = (radius+outer)/2
        bolts = max(8, 2*math.ceil(math.pi*circle/.28))
        for k in range(bolts):
            angle = 2*math.pi*k/bolts
            point = sd_add(centre,sd_add(sd_mul(axis,-thickness),sd_mul(sd_add(sd_mul(right,math.cos(angle)),sd_mul(up,math.sin(angle))),circle)))
            sd_bolt(parts,point,axis,2*thickness,.022,'Pipe splice bolt')
    return parts


def fitted_support_members(fabrication):
    """Fit tube ends to the actual shared graph, retaining every member axis.

    Dominant equal-section members share one exact mitre ellipse. Secondary
    branches are fishmouthed against those finite primary cylinders. This is
    fabrication geometry, not a visual collar placed over intersecting caps.
    """
    edges, nodes, meshes = {}, {}, {}
    def edge(key,a,b,ra,rb):
        edges[key] = (a,b,ra,rb)
        for end in (0,1):
            p,other = (a,b) if end == 0 else (b,a)
            node = tuple(round(x,5) for x in p)
            nodes.setdefault(node,[]).append((key,end,sd_unit(sd_sub(other,p)),ra if end == 0 else rb))
    for i, group in enumerate(fabrication['members']):
        detail = fabrication['details'][i]
        for j, m in enumerate(group):
            if m[4] or m[5]:
                continue
            a, b, ra, rb = m[:4]
            if j in detail['replacement_members'] and detail['inline_stem']:
                b, ra, rb = detail['continuation_end'], detail['radius'], detail['radius']
            key = (i,j)
            edge(key,a,b,ra,rb)
        if not detail['inline_stem']:
            incoming = next(iter(detail['replacement_members'].values()))
            edge((i,-1),incoming[1],detail['continuation_end'],detail['radius'],detail['radius'])
    rings, masters, planes = {}, {}, {}
    mitres, copes = 0, 0
    sides = 32
    for node, entries in nodes.items():
        options = []
        for i, a in enumerate(entries):
            for j in range(i+1,len(entries)):
                b = entries[j]
                cosine = sd_dot(a[2],b[2])
                if cosine < .65 and (min(a[3],b[3])/max(a[3],b[3]) >= .65 or len(entries) == 2):
                    options.append((min(a[3],b[3]),-cosine,i,j))
        if not options:
            continue
        _,_,ia,ib = max(options)
        a,b = entries[ia],entries[ib]
        normal = sd_unit(sd_sub(a[2],b[2]))
        cross_axis = sd_cross(a[2],b[2])
        if sd_dot(cross_axis,cross_axis) < 1e-12:
            _, right, up = sd_basis((0,0,0),normal)
        else:
            right = sd_unit(cross_axis)
            up = sd_cross(normal,right)
        radius = max(a[3],b[3])
        centre = edges[a[0]][a[1]]
        stretch = radius/abs(sd_dot(normal,a[2]))
        ring = [sd_add(centre,sd_add(sd_mul(right,radius*math.cos(2*math.pi*k/sides)),
                                   sd_mul(up,stretch*math.sin(2*math.pi*k/sides)))) for k in range(sides)]
        for entry, n in ((a,normal),(b,sd_mul(normal,-1))):
            rings[(entry[0],entry[1])] = ring
            planes[(entry[0],entry[1])] = n
        masters[node] = (a,b)
        mitres += 1

    def cylinder_exit(v, ray, axis, radius, plane, length):
        vp = sd_sub(v,sd_mul(axis,sd_dot(v,axis)))
        rp = sd_sub(ray,sd_mul(axis,sd_dot(ray,axis)))
        aa, bb, cc = sd_dot(rp,rp),2*sd_dot(vp,rp),sd_dot(vp,vp)-radius*radius
        if aa < 1e-12:
            return None
        discriminant = bb*bb-4*aa*cc
        if discriminant < 0:
            return None
        lo,hi = (-bb-math.sqrt(discriminant))/(2*aa),(-bb+math.sqrt(discriminant))/(2*aa)
        for n, limit in ((plane,0.),(sd_mul(axis,-1),-length)):
            value,slope = sd_dot(n,v)-limit,sd_dot(n,ray)
            if abs(slope) < 1e-12:
                if value < -1e-9:
                    return None
            elif slope > 0:
                lo = max(lo,-value/slope)
            else:
                hi = min(hi,-value/slope)
        return hi if hi >= max(0.,lo) else None

    for key, edge in edges.items():
        a,b,ra,rb = edge
        axis,right,up = sd_basis(a,b)
        end_rings = []
        for end,centre,radius in ((0,a,ra),(1,b,rb)):
            endpoint = (key,end)
            if endpoint in rings:
                end_rings.append(rings[endpoint])
                continue
            outward = axis if end == 0 else sd_mul(axis,-1)
            node = tuple(round(x,5) for x in centre)
            host = masters.get(node,())
            ring = []
            foundation = fabrication['foundations'].get(tuple(centre)) if end == 0 else None
            for k in range(sides):
                radial = sd_add(sd_mul(right,math.cos(2*math.pi*k/sides)),sd_mul(up,math.sin(2*math.pi*k/sides)))
                v = sd_mul(radial,radius)
                distance = 0.
                if foundation is not None:
                    height = foundation[1][2]+foundation_plate_thickness(foundation)-.002
                    taper = (rb-ra)/math.dist(a,b)
                    distance = (height-centre[2]-v[2])/(axis[2]+radial[2]*taper)
                    v = sd_mul(radial,radius+distance*taper)
                else:
                    exits = []
                    for h in host:
                        he = edges[h[0]]
                        value = cylinder_exit(v,outward,h[2],h[3],planes[(h[0],h[1])],math.dist(he[0],he[1]))
                        if value is not None:
                            exits.append(value)
                    if exits:
                        distance = max(exits)-.003
                        if distance > math.dist(a,b)*.95:
                            raise ValueError('Near-parallel branch has insufficient clear fabrication length: '+str((key,end,distance,math.dist(a,b),host)))
                ring.append(sd_add(centre,sd_add(v,sd_mul(outward,distance))))
            end_rings.append(ring)
            copes += bool(host)
        # Match ring phases along the member; paired joints still retain their
        # exact common world-space ellipse on both adjoining objects.
        first,second = end_rings
        def radial_direction(p,centre):
            delta = sd_sub(p,centre)
            return sd_unit(sd_sub(delta,sd_mul(axis,sd_dot(delta,axis))))
        first_radial = [radial_direction(p,a) for p in first]
        second_radial = [radial_direction(p,b) for p in second]
        options = []
        for reverse in (False,True):
            candidate = list(reversed(second_radial)) if reverse else second_radial
            for shift in range(sides):
                cost = sum(1-sd_dot(candidate[(k+shift)%sides],first_radial[k]) for k in range(sides))
                options.append((cost,reverse,shift))
        _,reverse,shift = min(options)
        second = list(reversed(second)) if reverse else second
        second = [second[(k+shift)%sides] for k in range(sides)]
        faces = [(k,(k+1)%sides,(k+1)%sides+sides,k+sides) for k in range(sides)]
        faces += [tuple(reversed(range(sides))),tuple(range(sides,2*sides))]
        meshes[key] = (first+second,faces)
    return {'meshes':meshes,'mitres':mitres,'copedEnds':copes,'rings':rings}


def footing_fixing_details(member, adjoining):
    a, top, base_radius, top_radius, concrete, contact = member
    if not concrete:
        return []
    radius = top_radius*.94
    thickness = foundation_plate_thickness(member)
    parts = [sd_part('Foundation / bearing plate',sd_tube(sd_add(top,(0,0,-.005)),sd_add(top,(0,0,thickness)),radius,sides=48))]
    circle = radius-.050
    count = max(12, 2*math.ceil(math.pi*circle/.24))
    for i in range(count):
        angle = 2*math.pi*i/count
        point = sd_add(top,(circle*math.cos(angle),circle*math.sin(angle),thickness))
        # Avoid burying anchor hardware inside an inclined incoming tube.
        obstructed = False
        for m in adjoining:
            axis = sd_unit(sd_sub(m[1],m[0]))
            for height in (0., .04):
                delta = sd_sub(sd_add(point,(0,0,height)),m[0])
                radial = sd_sub(delta,sd_mul(axis,sd_dot(delta,axis)))
                obstructed |= math.sqrt(sd_dot(radial,radial)) < m[2]+.045
        if not obstructed:
            sd_bolt(parts,sd_add(point,(0,0,-thickness)),(0,0,1),thickness,.027,'Foundation anchor')
    return parts
