"""Build the editable Riftwake train through live Blender MCP.

Invoke with runpy.run_path(path, init_globals={'ROOT': repo_path}). Parts are
named, closed meshes, with linked duplicates for the seven car assemblies.
Only the dedicated train scene is rebuilt; existing ride scenes are untouched.
"""
import bpy
import bmesh
import json
import math
from pathlib import Path
from mathutils import Matrix, Vector

ROOT = Path(globals().get('ROOT', 'D:/Coding/Codex/Vibecoaster2'))
P = json.loads((ROOT / 'native/art/train_profile.json').read_text())
EXPORT = ROOT / 'native/art/exports/train'
OUT = ROOT / 'out/train-model'
EXPORT.mkdir(parents=True, exist_ok=True)
OUT.mkdir(parents=True, exist_ok=True)
SCENE_NAME = 'Riftwake / Train design'
old_scene = bpy.context.scene
scene = bpy.data.scenes.get(SCENE_NAME)
if scene:
    for obj in list(scene.objects):
        if obj.get('riftwake_train_generated'):
            bpy.data.objects.remove(obj, do_unlink=True)
    for collection in list(scene.collection.children):
        if collection.name.startswith('RT / ') and not collection.all_objects:
            bpy.data.collections.remove(collection)
else:
    scene = bpy.data.scenes.new(SCENE_NAME)
scene.world = bpy.data.worlds.new('RT / studio world')
bpy.context.window.scene = scene
scene.unit_settings.system = next(i.identifier for i in scene.unit_settings.bl_rna.properties['system'].enum_items if i.identifier == 'METRIC')
scene.unit_settings.scale_length = 1
scene['train_profile'] = json.dumps(P)
scene['design_status'] = 'Original art proposal. No default.3 runtime or clearance envelope changes.'


def group(name):
    collection = bpy.data.collections.new('RT / ' + name)
    scene.collection.children.link(collection)
    return collection


def tag(obj, collection, role):
    collection.objects.link(obj)
    obj['riftwake_train_generated'] = True
    obj['role'] = role
    return obj


def material(name, colour, metal=0, rough=.35, coat=0, emission=0, transmission=0):
    mat = bpy.data.materials.new('RT / '+name)
    mat.diffuse_color = (*colour, 1)
    mat.use_nodes = True
    shader = next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    for socket, value in [('Base Color',(*colour,1)), ('Metallic',metal),
                          ('Roughness',rough), ('Coat Weight',coat),
                          ('Coat Roughness',.2), ('Transmission Weight',transmission)]:
        shader.inputs[socket].default_value = value
    if emission:
        shader.inputs['Emission Color'].default_value = (*colour, 1)
        shader.inputs['Emission Strength'].default_value = emission
    return mat


paint = material('deep petrol enamel', (.018,.175,.215), .52,.27,.5)
lightpaint = material('sea glass shoulder', (.045,.32,.36), .45,.27,.4)
gold = material('warm titanium', (.58,.30,.105), .72,.29)
ivory = material('ceramic seat shell', (.39,.49,.47), .24,.32,.25)
carbon = material('graphite composite', (.025,.033,.041), .25,.37)
rubber = material('cast polyurethane tyre', (.034,.037,.042), .02,.54)
cushion = material('charcoal seat pads', (.027,.041,.047), .02,.65)
aluminium = material('satin machined aluminium', (.42,.49,.52), .82,.28)
steel = material('polished steel', (.45,.51,.54), .92,.22)
black = material('recesses', (.009,.014,.018), .25,.38)
lamp = material('warm running light', (1,.56,.19), .15,.22,emission=3)
tailmat = material('tail marker', (.52,.016,.008), .2,.27,emission=2)
glass = material('clear aero lip', (.78,.91,.92), .0,.12,transmission=1)
floor_mat = material('studio floor', (.11,.145,.16), .0,.75)
track_mat = material('Exa warm grey coating', (.49,.49,.41), .35,.38)
parts = group('Car 01 / lead')
track_group = group('Exa rail fit reference')
studio = group('Studio and cameras')
car_root = tag(bpy.data.objects.new('RT / Car 01 / root', None), parts, 'car_root')
car_root['car_index'] = 0


def mesh(name, vertices, faces, mat, collection=parts, role='body', smooth=False, bevel=0):
    data = bpy.data.meshes.new('RT / '+name)
    data.from_pydata(vertices, [], faces)
    data.update()
    bm = bmesh.new()
    bm.from_mesh(data)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    if bevel:
        bevelled=bmesh.ops.bevel(bm, geom=list(bm.edges), offset=bevel, segments=6,
                        clamp_overlap=True,
                        affect=next(i.identifier for i in bpy.types.BevelModifier.bl_rna.properties['affect'].enum_items if i.identifier == 'EDGES'))
        for face in bevelled['faces']: face.smooth=True
        bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(data)
    bm.free()
    data.update()
    obj = tag(bpy.data.objects.new('RT / '+name, data), collection, role)
    data.materials.append(mat)
    if smooth:
        for face in data.polygons: face.use_smooth=True
    if collection == parts:
        obj.parent = car_root
    return obj


def box(name, centre, size, mat, collection=parts, role='chassis', bevel=.01):
    x,y,z = (s/2 for s in size)
    vertices = [(-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),
                (-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z)]
    faces = [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
    obj = mesh(name, vertices, faces, mat, collection, role, bevel=bevel)
    obj.location = centre
    return obj


def cylinder(name, a, b, radius, mat, collection=parts, role='chassis', sides=40, radius2=None):
    a,b=Vector(a),Vector(b)
    n=(b-a).normalized()
    u=n.cross(Vector((0,0,1)) if abs(n.z)<.9 else Vector((0,1,0))).normalized()
    v=n.cross(u)
    vertices = [tuple(p+r*(u*math.cos(2*math.pi*j/sides)+v*math.sin(2*math.pi*j/sides)))
                for p,r in ((a,radius),(b,radius if radius2 is None else radius2)) for j in range(sides)]
    faces=[(j,(j+1)%sides,(j+1)%sides+sides,j+sides) for j in range(sides)]
    faces += [tuple(reversed(range(sides))),tuple(range(sides,2*sides))]
    obj=mesh(name,vertices,faces,mat,collection,role)
    for face in list(obj.data.polygons)[:sides]: face.use_smooth=True
    return obj


def ring(name, centre, axis, inner, outer, width, mat, collection=parts, role='wheel', sides=64):
    centre,axis=Vector(centre),Vector(axis).normalized()
    u=axis.cross(Vector((0,0,1)) if abs(axis.z)<.9 else Vector((0,1,0))).normalized()
    v=axis.cross(u)
    vertices=[tuple(centre+axis*d+r*(u*math.cos(j*2*math.pi/sides)+v*math.sin(j*2*math.pi/sides)))
              for d,r in ((-width/2,outer),(width/2,outer),(-width/2,inner),(width/2,inner)) for j in range(sides)]
    faces=[]
    for j in range(sides):
        k=(j+1)%sides
        faces += [(j,k,k+sides,j+sides),(j+2*sides,j+3*sides,k+3*sides,k+2*sides),
                  (j,j+2*sides,k+2*sides,k),(j+sides,k+sides,k+3*sides,j+3*sides)]
    obj=mesh(name,vertices,faces,mat,collection,role)
    for i,face in enumerate(obj.data.polygons): face.use_smooth=i%4<2
    return obj


def tube(name, points, radius, mat, collection=parts, role='detail'):
    data=bpy.data.curves.new('RT / '+name,'CURVE')
    data.dimensions=next(i.identifier for i in data.bl_rna.properties['dimensions'].enum_items if i.identifier=='3D')
    data.bevel_depth=radius
    data.bevel_resolution=3
    data.resolution_u=12
    data.use_fill_caps=True
    spline=data.splines.new(next(i.identifier for i in bpy.types.Spline.bl_rna.properties['type'].enum_items if i.identifier=='BEZIER'))
    spline.bezier_points.add(len(points)-1)
    for point,co in zip(spline.bezier_points,points):
        point.co=co
        point.handle_left_type=point.handle_right_type=next(i.identifier for i in point.bl_rna.properties['handle_left_type'].enum_items if i.identifier=='AUTO')
    data.materials.append(mat)
    obj=tag(bpy.data.objects.new('RT / '+name,data),collection,role)
    if collection==parts: obj.parent=car_root
    return obj


def loft(name, stations, mat, role='body', power=.6, sides=32):
    # Rounded rectangular closed sections: x, lateral centre, half-width, zmid, half-height.
    vertices=[]
    for x,y,w,z,h in stations:
        for j in range(sides):
            a=2*math.pi*j/sides
            ca,sa=math.cos(a),math.sin(a)
            vertices.append((x,y+w*math.copysign(abs(ca)**power,ca),z+h*math.copysign(abs(sa)**power,sa)))
    faces=[(i*sides+j,i*sides+(j+1)%sides,(i+1)*sides+(j+1)%sides,(i+1)*sides+j)
           for i in range(len(stations)-1) for j in range(sides)]
    faces += [tuple(reversed(range(sides))), tuple(range((len(stations)-1)*sides,len(stations)*sides))]
    return mesh(name,vertices,faces,mat,collection=parts,role=role,smooth=True)


def bolt(name, centre, axis, radius=.012, length=.012, mat=steel):
    centre,axis=Vector(centre),Vector(axis)
    return cylinder(name,centre,centre+axis*length,radius,mat,role='fastener',sides=6)


def seat_sweep(name, cy, controls, mat, edge_rise, role='seat'):
    """Padded bucket following a continuous curved pan/lumbar/head profile.

    Each control holds X, Z, half width and half thickness. Catmull-Rom samples
    produce a continuous silhouette; the closed section curves up at its edges.
    """
    path=[]
    for i in range(len(controls)-1):
        a,b,c,d=[Vector(controls[max(0,min(len(controls)-1,j))]) for j in (i-1,i,i+1,i+2)]
        for j in range(6):
            t=j/6
            path.append(.5*((2*b)+(-a+c)*t+(2*a-5*b+4*c-d)*t*t+(-a+3*b-3*c+d)*t*t*t))
    path.append(Vector(controls[-1]))
    vertices=[]; sides=48
    for i,point in enumerate(path):
        tangent=path[min(i+1,len(path)-1)]-path[max(0,i-1)]
        normal=Vector((tangent[1],0,-tangent[0])).normalized()
        for j in range(sides):
            a=2*math.pi*j/sides
            ca,sa=math.cos(a),math.sin(a)
            lateral=math.copysign(abs(ca)**.7,ca)
            n=point[3]*math.copysign(abs(sa)**.7,sa)+edge_rise*lateral**4
            vertices.append(tuple(Vector((point[0],cy+point[2]*lateral,point[1]))+normal*n))
    faces=[(i*sides+j,i*sides+(j+1)%sides,(i+1)*sides+(j+1)%sides,(i+1)*sides+j)
           for i in range(len(path)-1) for j in range(sides)]
    faces += [tuple(reversed(range(sides))),tuple(range((len(path)-1)*sides,len(path)*sides))]
    return mesh(name,vertices,faces,mat,role=role,smooth=True)


def wheel(name, centre, radius, width, axis):
    centre,axis=Vector(centre),Vector(axis)
    ring(name+' / polyurethane',centre,axis,radius*.79,radius,width,rubber)
    ring(name+' / machined rim',centre,axis,radius*.66,radius*.81,width*.92,aluminium)
    cylinder(name+' / bearing hub',centre-axis*width*.60,centre+axis*width*.60,radius*.24,aluminium,role='wheel')
    u=axis.cross(Vector((0,0,1)) if abs(axis.z)<.9 else Vector((0,1,0))).normalized()
    v=axis.cross(u)
    for j in range(12):
        a=2*math.pi*j/12
        radial=u*math.cos(a)+v*math.sin(a)
        tangent=-u*math.sin(a)+v*math.cos(a)
        vv=[]
        for d in (-width*.30,width*.30):
            for r,t in ((radius*.22,-radius*.072),(radius*.72,-radius*.043),
                        (radius*.72,radius*.043),(radius*.22,radius*.072)):
                vv.append(tuple(centre+axis*d+radial*r+tangent*t))
        mesh(name+' / cooling spoke',vv,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],aluminium,role='wheel')
    for direction in (-1,1):
        cap=centre+axis*direction*width*.62
        cylinder(name+' / bearing cap',cap,cap+axis*direction*.012,radius*.16,carbon,role='wheel')
        for j in range(5):
            a=2*math.pi*j/5
            point=cap+radius*.18*(u*math.cos(a)+v*math.sin(a))
            bolt(name+' / hub fastener',point,axis*direction,.006,.008)


# Two rail-clasping bogies. Guide wheels are outboard so the centred crossheads
# and their webs remain unobstructed across a full longitudinal sweep.
rail=P['rail_radius']
for x in P['axle_stations']:
    box('bogie / upper crossbeam',(x,0,.69),(.16,1.99,.09),aluminium,bevel=.014)
    for side in (-1,1):
        y=side*.7
        wheel('road wheel', (x,y,rail+P['road_wheel_radius']+P['wheel_gap']), P['road_wheel_radius'],P['road_wheel_width'],(0,1,0))
        wheel('upstop wheel',(x,y,-rail-P['upstop_wheel_radius']-P['wheel_gap']),P['upstop_wheel_radius'],P['upstop_wheel_width'],(0,1,0))
        wheel('side guide wheel',(x,side*(.7+rail+P['guide_wheel_radius']+P['wheel_gap']),0),P['guide_wheel_radius'],P['guide_wheel_width'],(0,0,1))
        # The user corrected the clamp orientation. Its C lies in the transverse
        # YZ section, open toward the rail centre; only 100 mm thick along X.
        # A single short neck meets the crossbeam, with no longitudinal ladder.
        outline=[(.800,.425),(.89,.425),(.89,.655),(.99,.655),(.99,.425),
                 (1.095,.425),(1.165,.36),(1.165,-.27),(1.13,-.315),
                 (.797,-.315),(.797,-.218),(1.08,-.218),(1.102,-.197),
                 (1.102,.305),(1.075,.328),(.800,.328)]
        n=len(outline)
        vertices=[(x+dx,side*yy,zz) for dx in (-.05,.05) for yy,zz in outline]
        faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]
        faces += [(j,(j+1)%n,(j+1)%n+n,j+n) for j in range(n)]
        carrier=mesh('bogie / transverse C clamp',vertices,faces,aluminium,role='chassis',bevel=.008)
        carrier['clamp_plane']='YZ / across rail; open inboard'
        cylinder('bogie / road axle',(x,side*.685,.367),(x,side*.859,.367),.035,steel)
        cylinder('bogie / lower axle',(x,side*.685,-.257),(x,side*.845,-.257),.027,steel)
        cylinder('bogie / road bearing housing',(x,side*.802,.367),(x,side*.852,.367),.062,aluminium)
        cylinder('bogie / captive road axle cap',(x,side*.852,.367),(x,side*.864,.367),.046,carbon)
        cylinder('bogie / upstop bearing housing',(x,side*.794,-.257),(x,side*.848,-.257),.05,aluminium)
        box('bogie / guide spindle arm',(x,side*1.028,.127),(.09,.206,.064),aluminium,bevel=.012)
        cylinder('bogie / guide spindle',(x,side*.947,-.067),(x,side*.947,.157),.022,steel)
        for zz in (-.257,.367):
            bolt('bogie / captive axle fastener',(x,side*.866,zz),(0,side,0),.019,.012)

# Twin longitudinal chassis members and accessible floor with separate foot trays.
for side in (-1,1):
    box('chassis / main beam',(.06,side*.48,.697),(2.65,.15,.145),carbon,bevel=.018)
    box('floor / foot tray',(.45,side*.51,.79),(1.61,.72,.105),carbon,bevel=.035)
    for i in range(9):
        box('floor / traction rib',(.24+i*.092,side*.51,.849),(.026,.58,.009),rubber,bevel=.004)
    # Machined seat support brackets visible beneath the shell.
    for dx in (-.44,.18):
        box('seat / pedestal',(dx,side*.51,.927),(.13,.49,.21),aluminium,bevel=.018)
        for yy in (-.2,.2): bolt('seat / mounting bolt',(dx,side*.51+yy,1.035),(0,0,1))
    loft('body / swept side pod', [(-1.17,side*1.018,.08,.805,.056),(-1.03,side*1.025,.15,.84,.10),
          (-.63,side*1.025,.15,.858,.15),(.15,side*1.025,.15,.83,.135),
          (.86,side*.99,.14,.806,.108),(1.32,side*.90,.07,.78,.044),(1.38,side*.885,.018,.78,.016)],paint)
    tube('body / titanium coach line',[(-1.06,side*1.15,.88),(-.6,side*1.158,.92),
         (.23,side*1.153,.875),(.87,side*1.103,.845),(1.29,side*.945,.80)],.012,gold)
    for i in range(6):
        vent=box('body / cooling louvre',(-.94+i*.087,side*1.17,.829),(.035,.008,.071),black,role='body',bevel=.004)
        vent.rotation_euler.y=math.radians(-22)
    tube('body / inner edge',[(-.87,side*.862,.94),(-.39,side*.86,1.0),(.42,side*.875,.92),(.98,side*.845,.85)],.011,lightpaint)

# The seat is a contoured continuous shell, not a stack of box cushions.
for cy in P['seat_centres_y']:
    seat_sweep('seat / sculpted shell',cy,[(.37,1.07,.20,.012),(.30,1.077,.292,.022),
        (.02,1.044,.30,.028),(-.23,1.046,.292,.028),(-.385,1.17,.304,.025),
        (-.46,1.42,.32,.025),(-.53,1.67,.326,.025),(-.584,1.86,.246,.022),
        (-.591,1.985,.19,.018),(-.591,2.015,.11,.012)],ivory,.068)
    seat_sweep('seat / continuous ergonomic cushion',cy,[(.31,1.128,.185,.016),(.25,1.133,.249,.037),
        (.00,1.105,.253,.048),(-.20,1.11,.249,.044),(-.31,1.21,.257,.043),
        (-.386,1.425,.267,.041),(-.458,1.669,.27,.040),(-.513,1.855,.19,.042),
        (-.526,1.951,.14,.029),(-.534,1.974,.08,.012)],cushion,.05)
    # Subtle seams and firm lateral bolsters visually describe the seating bucket.
    for side in (-1,1):
        tube('seat / piped seam',[(.27,cy+side*.234,1.181),(-.13,cy+side*.239,1.188),
             (-.282,cy+side*.249,1.27),(-.353,cy+side*.255,1.52),(-.445,cy+side*.24,1.713)],.004,gold,role='seat')
        cylinder('restraint / pivot',(-.29,cy+side*.246,.99),(-.29,cy+side*.323,.99),.064,aluminium,role='restraint')
        tube('restraint / articulated arm',[(-.29,cy+side*.287,.99),(-.16,cy+side*.287,1.205),
             (.13,cy+side*.245,1.33),(.31,cy+side*.206,1.325)],.03,carbon,role='restraint')
        tube('restraint / grab handle',[(.285,cy+side*.16,1.343),(.38,cy+side*.16,1.423),
             (.45,cy+side*.13,1.421),(.355,cy+side*.10,1.353)],.015,gold,role='restraint')
        cylinder('restraint / hydraulic cylinder',(-.34,cy+side*.285,.91),(-.07,cy+side*.285,1.16),.025,carbon,role='restraint')
        cylinder('restraint / hydraulic rod',(-.08,cy+side*.285,1.15),(.02,cy+side*.285,1.258),.012,steel,role='restraint')
    loft('restraint / individual lap cushion',[(.155,cy,.15,1.319,.021),(.18,cy,.223,1.321,.057),
        (.28,cy,.226,1.327,.068),(.38,cy,.21,1.333,.052),(.401,cy,.14,1.335,.019)],cushion,role='restraint',power=.7)
    box('restraint / status housing',(-.235,cy+.319,1.075),(.074,.052,.071),carbon,role='restraint',bevel=.009)
    cylinder('restraint / latched indicator',(-.19,cy+.32,1.076),(-.185,cy+.32,1.076),.010,lightpaint,role='restraint',sides=20)

# Articulated central drawbar and clevis, retained per-car attachment points.
box('coupler / front drawbar',(1.443,0,.704),(.57,.19,.115),aluminium,role='coupler',bevel=.025)
box('coupler / rear drawbar',(-1.444,0,.704),(.59,.23,.13),carbon,role='coupler',bevel=.018)
cylinder('coupler / spherical joint',(-1.7,0,.618),(-1.7,0,.79),.108,aluminium,role='coupler')
bolt('coupler / kingpin',(-1.7,0,.79),(0,0,1),.047,.027)
tube('coupler / service loom',[(-1.10,.22,.73),(-1.36,.255,.70),(-1.53,.26,.70),(-1.72,.18,.70)],.024,black,role='coupler')
for side in (-1,1):
    box('chassis / LSM magnet cassette',(.03,side*.36,.314),(2.15,.12,.15),carbon,bevel=.014)
    for x in (-.85,-.45,-.05,.35,.75):
        box('chassis / magnet segment',(x,side*.361,.232),(.28,.112,.018),aluminium,bevel=.006)

# Save the standard car object set; front aero body is unique to the lead car.
standard=list(parts.objects)
nose=loft('nose / flowing upper shell',[(.94,0,.86,.87,.12),(1.17,0,.92,.88,.20),
          (1.47,0,.86,.856,.224),(1.77,0,.71,.801,.204),(2.06,0,.46,.735,.152),
          (2.25,0,.22,.695,.102),(2.30,0,.065,.686,.05)],paint,role='nose',power=.74,sides=48)
loft('nose / graphite splitter',[(1.01,0,.91,.688,.025),(1.5,0,.9,.64,.034),
     (1.9,0,.65,.57,.036),(2.26,0,.27,.568,.024),(2.315,0,.055,.598,.011)],carbon,role='nose',sides=48)
for side in (-1,1):
    tube('nose / shoulder accent',[(1.015,side*.86,.992),(1.31,side*.88,1.034),
        (1.66,side*.76,.98),(1.99,side*.515,.877),(2.215,side*.27,.782)],.017,gold,role='nose')
    tube('nose / recessed lamp socket',[(1.68,side*.815,.84),(1.89,side*.667,.79),(2.04,side*.515,.751)],.035,black,role='nose')
    tube('nose / blade light',[(1.70,side*.852,.852),(1.89,side*.704,.802),(2.03,side*.559,.765)],.012,lamp,role='nose')
    for j in range(5):
        x=1.18+j*.068
        tube('nose / upper pressure vent',[(x,side*.40,1.084),(x+.03,side*.62,1.079)],.01,black,role='nose')
# Small swept clear wind lip below the rider sightline. Closed 6 mm shell.
verts=[]
for dz in (0,.006):
    for i in range(6):
        t=i/5
        for j in range(33):
            y=-.795+1.59*j/32
            verts.append((1.11-.25*t+.12*(y/.795)**2,y,1.035+.21*t+dz-.07*(abs(y)/.795)**2))
n=6*33
faces=[]
for layer in (0,1):
    for i in range(5):
        for j in range(32):
            a=layer*n+i*33+j
            f=(a,a+1,a+34,a+33)
            faces.append(f if layer==0 else tuple(reversed(f)))
bd=list(range(33))+[i*33+32 for i in range(1,6)]+list(range(196,164,-1))+[i*33 for i in range(4,0,-1)]
faces += [(a,b,b+n,a+n) for a,b in zip(bd,bd[1:]+bd[:1])]
mesh('nose / low clear aero lip',verts,faces,glass,role='nose',smooth=True)
tube('nose / aero lip lower gasket',[(1.23,-.795,.965),(1.14,-.4,1.02),(1.11,0,1.035),(1.14,.4,1.02),(1.23,.795,.965)],.012,carbon,role='nose')

# Six linked standard cars, fully editable by part and separable by assembly.
for index in range(1,P['cars']):
    collection=group(f'Car {index+1:02d} / standard')
    root=tag(bpy.data.objects.new(f'RT / Car {index+1:02d} / root',None),collection,'car_root')
    root.location.x=-index*P['car_pitch']
    root['car_index']=index
    for source in standard:
        if source.type=='EMPTY': continue
        obj=source.copy()
        collection.objects.link(obj)
        obj.parent=root
        obj.matrix_parent_inverse=Matrix.Identity(4)
        obj.name=source.name+f' / car {index+1:02d}'
    # A shallow front cowl ties the separate footwells together without enclosing knees.
    previous_root,car_root=car_root,root
    prior_parts=parts
    parts=collection
    loft(f'car {index+1:02d} / low front cowl',[(.88,0,.8,.85,.045),(1.15,0,.91,.827,.071),(1.35,0,.81,.801,.048)],paint,role='body',power=.7)
    parts=prior_parts
    car_root=previous_root

# A reference track at the exact current Exa section, using its shared geometry.
import runpy
geo=runpy.run_path(str(ROOT/'native/art/track_study_geometry.py'))
tp=json.loads((ROOT/'native/art/track_study_profile.json').read_text())
length=29.4; start=-24.2
for side in (-1,1):
    vv,ff,bands=geo['swept_tube'](tp,'straight',length,side*.7,0,.105,.02)
    obj=mesh('reference / running rail',[(x+start,y,z) for x,y,z in vv],ff,track_mat,track_group,'reference',True)
    obj.data.materials.append(steel)
    for face,band in zip(obj.data.polygons,bands): face.material_index=band
vv,ff,_=geo['swept_tube'](tp,'straight',length,0,-.8,.34,.024)
mesh('reference / spine',[(x+start,y,z) for x,y,z in vv],ff,track_mat,track_group,'reference',True)
for i in range(21):
    x=start+.7+i*1.4
    cylinder('reference / crosshead',(x,-.62,0),(x,.62,0),.065,track_mat,track_group,'reference')
    for side in (-1,1):
        vv,ff=geo['gusset_mesh'](tp,side)
        mesh('reference / welded gusset',[(xx+x,y,z) for xx,y,z in vv],ff,track_mat,track_group,'reference')
for x in (-23,-16,-9,-2,4):
    cylinder('reference / display support',(x,0,-1.93),(x,0,-1.10),.24,carbon,track_group,'reference')
    box('reference / base',(x,0,-1.94),(1.25,1.25,.12),carbon,track_group,'reference',.02)
box('studio / floor',(-8,0,-2.055),(200,200,.1),floor_mat,studio,'studio',0)

# Studio lighting and review cameras.
scene.world.use_nodes=True
background=next(n for n in scene.world.node_tree.nodes if n.type=='BACKGROUND')
background.inputs[0].default_value=(.48,.62,.75,1)
background.inputs[1].default_value=.45
for name,position,target,energy,size in [('key',(3,-8,11),(-3,0,0),4200,8),
    ('rim',(-7,5,9),(-4,0,1),5200,10),('front',(7,3,6),(0,0,1),2100,6),
    ('tail',(-20,-4,9),(-16,0,0),3600,9)]:
    data=bpy.data.lights.new('RT / '+name,'AREA')
    data.energy=energy; data.shape=next(i.identifier for i in data.bl_rna.properties['shape'].enum_items if i.identifier=='DISK'); data.size=size
    obj=tag(bpy.data.objects.new('RT / '+name,data),studio,'studio')
    obj.location=position
    obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()

CAMERAS={
    'hero':((7.8,-10.8,5.0),(-2.8,0,.55),52),
    'full-train':((11,-26,14),(-9.4,0,.25),43),
    'front':((7.8,-.02,2.4),(.05,0,.74),55),
    'side':((-.4,-8.4,2.5),(.15,0,.8),55),
    'wheel':((2.25,-3.45,1.12),(.70,-.70,.13),65),
    'clamp':((3.8,-2.0,.72),(.83,-.95,.14),66),
    'seating':((2.15,-3.7,3.0),(-.10,0,1.24),53),
    'rider':(tuple(P['rider_eye']),(18,P['rider_eye'][1],1.57),21),
    'rear':((-25,-5,3.2),(-19,0,.7),50),
}
for name,(position,target,lens) in CAMERAS.items():
    data=bpy.data.cameras.new('RT / Camera / '+name)
    data.lens=lens; data.clip_start=.025; data.clip_end=500
    obj=tag(bpy.data.objects.new('RT / Camera / '+name,data),studio,'camera')
    obj.location=position
    obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler()
scene.camera=bpy.data.objects['RT / Camera / hero']
try: scene.render.engine='CYCLES'
except TypeError as exc: raise RuntimeError('Cycles is required for the train preview') from exc
scene.cycles.device=next(i.identifier for i in scene.cycles.bl_rna.properties['device'].enum_items if i.identifier=='GPU')
scene.cycles.samples=16
scene.cycles.use_denoising=True
scene.render.threads_mode=next(i.identifier for i in scene.render.bl_rna.properties['threads_mode'].enum_items if i.identifier=='FIXED')
scene.render.threads=8
scene.render.resolution_x=1600; scene.render.resolution_y=1000; scene.render.resolution_percentage=100
scene.render.image_settings.file_format=next(i.identifier for i in scene.render.image_settings.bl_rna.properties['file_format'].enum_items if i.identifier=='PNG')
scene.view_settings.view_transform=old_scene.view_settings.view_transform
scene.view_settings.look=old_scene.view_settings.look
scene.view_settings.exposure=-.3
for area in bpy.context.screen.areas:
    if area.type=='VIEW_3D':
        area.spaces.active.shading.type=next(i.identifier for i in area.spaces.active.shading.bl_rna.properties['type'].enum_items if i.identifier=='SOLID')
        area.spaces.active.overlay.show_overlays=False
        area.spaces.active.region_3d.view_perspective=next(i.identifier for i in area.spaces.active.region_3d.bl_rna.properties['view_perspective'].enum_items if i.identifier=='CAMERA')
        area.spaces.active.clip_end=500
bpy.context.view_layer.update()
bpy.data.libraries.write(str(EXPORT/'Riftwake-Train.blend'), {scene}, compress=True)
print(json.dumps({'saved':str(EXPORT/'Riftwake-Train.blend'),'objects':len(scene.objects),'cars':P['cars'],
                  'profile':P['id'],'created_through':'Blender MCP','scene':scene.name}))
