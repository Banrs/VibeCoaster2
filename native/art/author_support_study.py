"""Build a native-generated support graph in Blender via MCP.

Native member axes and foundations drive the model. Terminal contacts receive
the Exa study's fabricated head; original terminal envelopes remain hidden and
editable for comparison. Fabrication fittings are proposed art geometry.
"""
import bpy
import bmesh
import json
import math
from mathutils import Vector

# TRACK_STUDY_GEOMETRY
# SUPPORT_DETAIL_GEOMETRY

DATA = json.loads(__SUPPORT_CONTEXT__)
PROFILE = json.loads(__TRACK_PROFILE__)
EXPORT_ROOT = "__EXPORT_ROOT__"
COARSE = __SUPPORT_COARSE__
DEFER_RENDER = __SUPPORT_DEFER_RENDER__


def render_still():
    if not DEFER_RENDER:
        bpy.ops.render.render(write_still=True)
kind = {0: "Camelback", 1: "Loop", 2: "Immelmann"}[DATA["regions"][0]["kind"]] if DATA["regions"] else "Layout"
name = DATA.get('reviewName',kind+" / terrain "+str(DATA["seed"]))
scene_name = DATA.get('scenePrefix','Support System / ')+name
scene = bpy.data.scenes.get(scene_name) or bpy.data.scenes.new(scene_name)
bpy.context.window.scene = scene
for obj in list(scene.objects):
    bpy.data.objects.remove(obj, do_unlink=True)
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
scene["source"] = "coaster_core axes and foundation footprints; Exa fabrication fittings are review art"
scene["track_alignment"] = "Exa art study positioned by spine-contact datum; runtime envelope migration remains separate"
scene["counts"] = json.dumps(DATA["counts"])
scene["native_data_json"] = json.dumps(DATA, separators=(",", ":"))


def material(name, color, metallic=0.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    m.diffuse_color = (*color, 1)
    shader = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Roughness"].default_value = .44
    shader.inputs["Metallic"].default_value = metallic
    return m


steel = material("SS Steel", (.24, .30, .32), .45)
footing = material("SS Concrete", (.52, .51, .47))
ground = material("SS Terrain", (.29, .34, .29))
rail = material("SS Rail", (.46, .54, .57), .7)
spine = material("SS Spine", (.78, .64, .34), .25)
fastener = material("SS Fasteners", (.35, .40, .42), .8)


def mesh_object(name, vertices, faces, mat, smooth=False, face_smoothing=None):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    scene.collection.objects.link(obj)
    mesh.materials.append(mat)
    for i, face in enumerate(mesh.polygons):
        face.use_smooth = face_smoothing[i] if face_smoothing is not None else smooth and len(face.vertices) == 4
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(mesh)
    bm.free()
    return obj


def member_geometry(a, b, r0, r1, sides=24):
    a, b = Vector(a), Vector(b)
    axis = (b-a).normalized()
    right = axis.cross(Vector((0, 0, 1)) if abs(axis.z) < .9 else Vector((0, 1, 0))).normalized()
    up = axis.cross(right)
    vertices = [tuple(p+r*(right*math.cos(2*math.pi*j/sides)+up*math.sin(2*math.pi*j/sides)))
                for p, r in ((a, r0), (b, r1)) for j in range(sides)]
    faces = [(j, (j+1)%sides, (j+1)%sides+sides, j+sides) for j in range(sides)]
    faces += [tuple(reversed(range(sides))), tuple(range(sides, 2*sides))]
    return vertices, faces


def detail_objects(label, parts, station):
    result = []
    for material_name, mat in (("steel", steel), ("fastener", fastener)):
        vertices, faces, smoothing = [], [], []
        for part in parts:
            if part["material"] != material_name:
                continue
            base = len(vertices)
            vertices += part["vertices"]
            faces += [tuple(base+k for k in face) for face in part["faces"]]
            flat = any(word in part["name"] for word in ("web", "cheek", "diaphragm", "load plate", "hex"))
            smoothing += [not flat and len(face) == 4 and ("flange" not in part["name"] or j%4 >= 2)
                          for j,face in enumerate(part["faces"])]
        if faces:
            obj = mesh_object(label+" / "+material_name, vertices, faces, mat, face_smoothing=smoothing)
            obj["track_distance_m"] = station
            obj["geometry_scope"] = "fabrication art; not adopted by native collision envelopes"
            result.append(obj)
    return result


fabrication = support_fabrication_plan(PROFILE,DATA)
fitted = fit_support_fabrication(fabrication)
connection_details = fabrication["details"]
all_members = [m for group in fabrication["members"] for m in group]
splice_count, foundation_count = 0, 0
for i, support in enumerate(DATA["supports"]):
    detail = connection_details[i]
    detail_objects("Track mount / "+str(i), [p for p in detail["parts"] if not COARSE or p["material"] == "steel"], support["distance"])
    # Separate editable members retain native attachment ownership and identity.
    for j, original in enumerate(support["members"]):
        member = fabrication["members"][i][j]
        a, b, r0, r1, concrete, contact = member
        bearing = fabrication["foundations"].get(tuple(a)) if not concrete else None
        if j in detail["replaced"]:
            verts, faces = member_geometry(*original[:4])
        elif not concrete:
            verts, faces = fitted['meshes'][(i,j)]
        else:
            verts, faces = member_geometry(a, b, r0, r1, 48 if concrete else 32)
        obj = mesh_object(("Footing" if concrete else "Spine contact" if contact else "Steel")+" / "+str(i)+" / "+str(j),
                          verts, faces, footing if concrete else steel, True)
        obj["track_distance_m"] = support["distance"]
        obj["native_member"] = json.dumps(original)
        if member != original or bearing is not None:
            obj["geometry_scope"] = "fabrication art; retained native axis and ground footprint"
        if j in detail["replaced"]:
            obj.name = "Native envelope / "+str(i)+" / "+str(j)
            obj.hide_render = True
            obj.hide_set(True)
            obj.display_type = 'WIRE'
            if contact:
                continue
        if COARSE:
            parts = []
        elif concrete:
            adjoining = [m for m in all_members if not m[4] and math.dist(m[0], b) < 1e-6]
            parts = footing_fixing_details(member, adjoining)
            foundation_count += 1
        else:
            parts = member_splice_details(member)
            splice_count += sum(part["name"] == "Pipe splice / flange" for part in parts)//2
        if parts:
            detail_objects("Foundation fixing" if concrete else "Pipe splices / "+str(i)+" / "+str(j), parts, support["distance"])
scene["fabrication_audit"] = json.dumps({"trackMounts":len(connection_details), "boltedPipeSplices":splice_count,
    "foundationFixings":foundation_count, "fittedMitres":fitted['mitres'], "copedTubeEnds":fitted['copedEnds'], "nativeEnvelopeAdopted":False})

# Draw the retained art profile on the native attachment datum. Rail size and
# centring are not silently changed to fit the older runtime track dimensions.
frames = []
offset = PROFILE["spine_depth"]+PROFILE["spine_radius"]-DATA["nativeSpineDepth"]-DATA["nativeSpineRadius"]
for position, right, up, station in DATA["track"]:
    r, u = Vector(right), Vector(up)
    frames.append((Vector(position)+u*offset, r, u))


def tube(name, local_y, local_z, radius, mat):
    sides = 36
    vertices = []
    wall = PROFILE["rail_wall"] if local_y else PROFILE["spine_wall"]
    for shell_radius in (radius, radius-wall):
        for p, r, u in frames:
            vertices += [tuple(p+r*(local_y+shell_radius*math.cos(2*math.pi*j/sides))+u*(local_z+shell_radius*math.sin(2*math.pi*j/sides))) for j in range(sides)]
    faces = []
    layer = len(frames)*sides
    for shell in (0, 1):
        for i in range(len(frames)-1):
            for j in range(sides):
                k = (j+1)%sides
                face = tuple(shell*layer+index for index in (i*sides+j,i*sides+k,(i+1)*sides+k,(i+1)*sides+j))
                faces.append(face if not shell else tuple(reversed(face)))
    for i in (0,len(frames)-1):
        for j in range(sides):
            k = (j+1)%sides
            faces.append((i*sides+j,layer+i*sides+j,layer+i*sides+k,i*sides+k))
    return mesh_object(name, vertices, faces, mat, True)


tube("Study rail / 105 mm radius / left", -PROFILE["gauge"]/2, 0, PROFILE["rail_radius"], rail)
tube("Study rail / 105 mm radius / right", PROFILE["gauge"]/2, 0, PROFILE["rail_radius"], rail)
tube("Study spine / attachment-aligned", 0, -PROFILE["spine_depth"], PROFILE["spine_radius"], spine)
tie_vertices, tie_faces, tie_smoothing = [], [], []
tie_frames = []
index, station = 1, math.ceil(DATA['track'][0][3]/PROFILE['tie_spacing'])*PROFILE['tie_spacing']
while station <= DATA["track"][-1][3]:
    while index+1 < len(frames) and DATA["track"][index][3] < station:
        index += 1
    s0, s1 = DATA["track"][index-1][3], DATA["track"][index][3]
    t = (station-s0)/(s1-s0)
    p = frames[index-1][0].lerp(frames[index][0],t)
    r = frames[index-1][1].lerp(frames[index][1],t).normalized()
    u = frames[index-1][2].lerp(frames[index][2],t).normalized()
    u = r.cross(u).normalized().cross(r).normalized()
    tie_frames.append((p,r,u))
    station += PROFILE["tie_spacing"]
for p, r, u in tie_frames:
    half = PROFILE["gauge"]*.5-PROFILE["crosshead_end_inset"]
    vertices, faces = member_geometry(p-r*half, p+r*half, PROFILE["crosshead_radius"], PROFILE["crosshead_radius"], 16)
    base = len(tie_vertices)
    tie_vertices += vertices
    tie_faces += [tuple(base+k for k in face) for face in faces]
    tie_smoothing += [len(face) == 4 for face in faces]
    # Use the actual Exa web geometry. The discarded preview put a short bar
    # inside the spine and left a gap from the crosshead down to that spine.
    frame = (tuple(p), tuple(r.cross(u).normalized()), tuple(r), tuple(u))
    for side in (-1, 1):
        vertices, faces = gusset_mesh(PROFILE, side)
        base = len(tie_vertices)
        tie_vertices += [transform(frame, vertex) for vertex in vertices]
        tie_faces += [tuple(base+k for k in face) for face in faces]
        tie_smoothing += [False]*len(faces)
mesh_object("Study ties / centred on rails", tie_vertices, tie_faces, spine, face_smoothing=tie_smoothing)

terrain = DATA["terrain"]
columns, rows = terrain["columns"], terrain["rows"]
faces = []
for y in range(rows-1):
    for x in range(columns-1):
        i = y*columns+x
        # The runtime surface uses the same 8 m grid and triangle diagonal.
        faces.extend(((i, i+1, i+columns+1), (i, i+columns+1, i+columns)))
mesh_object("Terrain / sampled native surface", terrain["points"], faces, ground)
# Continue the perimeter only outside the native review patch so ground views
# do not look through an open terrain edge. No footing or native vertex moves.
perimeter = list(range(columns))
perimeter += [y*columns+columns-1 for y in range(1,rows)]
perimeter += [(rows-1)*columns+x for x in range(columns-2,-1,-1)]
perimeter += [y*columns for y in range(rows-2,0,-1)]
patch_centre = (Vector(terrain['points'][0])+Vector(terrain['points'][-1]))*.5
apron = [tuple(terrain['points'][i]) for i in perimeter]
for i in perimeter:
    p = Vector(terrain['points'][i])
    radial = Vector((p.x-patch_centre.x,p.y-patch_centre.y,0)).normalized()
    apron.append(tuple(p+radial*4000))
n = len(perimeter)
apron_obj = mesh_object('Terrain / outside context apron',apron,
    [(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)],ground)
apron_obj['geometry_scope'] = 'Presentation outside sampled native terrain; not generation data'

lo, hi = Vector(DATA["bounds"][0]), Vector(DATA["bounds"][1])
centre = (lo+hi)*.5
extent = max(hi.x-lo.x, hi.y-lo.y, hi.z-lo.z)
camera_data = bpy.data.cameras.new("Support camera")
camera = bpy.data.objects.new("Support camera", camera_data)
scene.collection.objects.link(camera)
yaw = .31*DATA["seed"]
along, across = Vector((math.cos(yaw), math.sin(yaw), 0)), Vector((-math.sin(yaw), math.cos(yaw), 0))
camera.location = centre+across*(-extent*1.8)+along*(-extent*.35)+Vector((0, 0, extent*.5))
camera.rotation_euler = (centre-camera.location).to_track_quat("-Z", "Y").to_euler()
camera_data.type = "ORTHO"
rotation = camera.rotation_euler.to_matrix()
screen_x, screen_y = rotation @ Vector((1, 0, 0)), rotation @ Vector((0, 1, 0))
corners = [Vector((x, y, z)) for x in (lo.x, hi.x) for y in (lo.y, hi.y) for z in (lo.z, hi.z)]
review_points = [Vector(q[0]) for q in DATA['track']]
review_points += [Vector(p) for s in DATA['supports'] for m in s['members'] for p in m[:2]]
width = max(v.dot(screen_x) for v in corners)-min(v.dot(screen_x) for v in corners)
height = max(v.dot(screen_y) for v in corners)-min(v.dot(screen_y) for v in corners)
camera_data.ortho_scale = max(width, height*1800/1100)*1.10
camera_data.clip_end = max(10000., extent*10.)
scene.camera = camera
scene.world = bpy.data.worlds.new("Support study sky")
scene.world.use_nodes = True
background = next(n for n in scene.world.node_tree.nodes if n.type == 'BACKGROUND')
background.inputs[0].default_value = (.32, .51, .76, 1)
background.inputs[1].default_value = .7
light_data = bpy.data.lights.new("Support sunlight", "SUN")
light_data.energy = 3.0
light_data.angle = .15
light = bpy.data.objects.new("Support sunlight", light_data)
scene.collection.objects.link(light)
light.rotation_euler = (.45, -.5, -.6)
scene.render.engine = "CYCLES"
devices = bpy.context.preferences.addons['cycles'].preferences.devices
render_device = 'GPU' if any(d.use and d.type != 'CPU' for d in devices) else 'CPU'
scene.cycles.device = next(v.identifier for v in scene.cycles.bl_rna.properties['device'].enum_items if v.identifier == render_device)
scene.render.threads_mode = next(v.identifier for v in scene.render.bl_rna.properties['threads_mode'].enum_items if v.identifier == 'FIXED')
scene.render.threads = 4
scene.render.use_persistent_data = True
scene.cycles.samples = 8 if COARSE else 16
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1800, 1100
scene.render.resolution_percentage = 70
scene.view_settings.view_transform = "AgX"
scene.render.image_settings.file_format = "PNG"
scene.render.filepath = EXPORT_ROOT+"/"+kind.lower()+"-"+str(DATA["seed"])+".png"
for area in bpy.context.screen.areas:
    if area.type == "VIEW_3D":
        area.spaces.active.region_3d.view_perspective = "CAMERA"
bpy.ops.wm.save_as_mainfile(filepath=EXPORT_ROOT+"/Support-System.blend", compress=True)
render_still()
# Retained third-person views include a level broadside and a ground-level
# perspective. The latter matches the user's external Exa reference direction.
for suffix, shift in (("side", 0.), ("ground", -.35)):
    data = bpy.data.cameras.new("Support "+suffix+" camera")
    obj = bpy.data.objects.new("Support "+suffix+" camera", data)
    scene.collection.objects.link(obj)
    target = centre.copy()
    obj.location = centre+across*(-extent*1.45)+along*(extent*shift)
    if suffix == "ground":
        nearest = min(((p[0]-obj.location.x)**2+(p[1]-obj.location.y)**2,p[2]) for p in terrain["points"])
        obj.location.z = nearest[1]+1.8
        data.type = 'PERSP'
        data.lens = 42
    else:
        data.type = 'ORTHO'
        data.ortho_scale = max(hi.x-lo.x, hi.y-lo.y, (hi.z-lo.z)*1800/1100)*1.1
    data.clip_end = 10000
    obj.rotation_euler = (target-obj.location).to_track_quat('-Z','Y').to_euler()
    if suffix == 'ground':
        inverse = obj.rotation_euler.to_matrix().transposed()
        rays = [inverse@(p-obj.location) for p in review_points]
        required = max(max(abs(p.x),abs(p.y)*1800/1100)/-p.z for p in rays if p.z < -.01)
        data.lens = min(42.,data.sensor_width/(2*required*1.08))
    scene.camera = obj
    scene.render.filepath = EXPORT_ROOT+"/"+kind.lower()+"-"+str(DATA["seed"])+"-"+suffix+".png"
    render_still()
# A close view of a real generated joint, choosing an inverted attachment when
# available. The camera is retained in the editable scene for further review.
chosen = (min((s["frame"][2][2],i) for i,s in enumerate(DATA["supports"])) if kind != "Camelback"
          else max((s["attachment"][2],i) for i,s in enumerate(DATA["supports"])))[1]
detail = connection_details[chosen]
origin, forward, right, up = [Vector(detail[key]) for key in ("origin", "forward", "right", "up")]
close_data = bpy.data.cameras.new("Connection detail camera")
close_camera = bpy.data.objects.new("Connection detail camera", close_data)
scene.collection.objects.link(close_camera)
target = Vector(DATA["supports"][chosen]["attachment"])-up*.45
close_camera.location = origin-forward*3.5-right*4.2+up*.1
close_camera.rotation_euler = (target-close_camera.location).to_track_quat("-Z","Y").to_euler()
close_data.type, close_data.ortho_scale = 'ORTHO', max(4.8,detail["radius"]*11)
close_data.clip_start, close_data.clip_end = .02, 10000
scene.camera = close_camera
scene.render.filepath = EXPORT_ROOT+"/"+kind.lower()+"-"+str(DATA["seed"])+"-joint.png"
render_still()
other_data = bpy.data.cameras.new('Connection reverse camera')
other_camera = bpy.data.objects.new('Connection reverse camera',other_data)
scene.collection.objects.link(other_camera)
other_camera.location = origin+forward*3.5+right*4.2-up*1.5
other_camera.rotation_euler = (target-other_camera.location).to_track_quat('-Z','Y').to_euler()
other_data.type,other_data.ortho_scale = 'ORTHO',max(4.8,detail['radius']*11)
other_data.clip_start,other_data.clip_end = .02,10000
scene.camera = other_camera
scene.render.filepath = EXPORT_ROOT+'/'+kind.lower()+'-'+str(DATA['seed'])+'-joint-reverse.png'
render_still()
foundation_choice = max((m[3],i,j) for i,s in enumerate(DATA["supports"])
                        for j,m in enumerate(s["members"]) if m[4])
_, foundation_owner, foundation_index = foundation_choice
foundation_member = fabrication["members"][foundation_owner][foundation_index]
foundation_top = Vector(foundation_member[1])
base_data = bpy.data.cameras.new("Foundation detail camera")
base_camera = bpy.data.objects.new("Foundation detail camera", base_data)
scene.collection.objects.link(base_camera)
base_target = foundation_top+Vector((0,0,.3))
base_camera.location = foundation_top-across*6+along*4+Vector((0,0,4))
base_camera.rotation_euler = (base_target-base_camera.location).to_track_quat("-Z","Y").to_euler()
base_data.type, base_data.ortho_scale = 'ORTHO', max(5.5,foundation_member[2]*3.7)
base_data.clip_start, base_data.clip_end = .02,10000
scene.camera = base_camera
scene.render.filepath = EXPORT_ROOT+"/"+kind.lower()+"-"+str(DATA["seed"])+"-foundation.png"
render_still()
scene.camera = camera
node_positions = {}
for m in all_members:
    if not m[4] and not m[5]:
        for p in m[:2]:
            key = tuple(round(v,5) for v in p)
            node_positions.setdefault(key,[]).append(max(m[2],m[3]))
candidates = [(max(rs),-abs(p[2]-(lo.z+(hi.z-lo.z)*.52)),p)
              for p,rs in node_positions.items() if len(rs)>=4]
if candidates:
    target = Vector(max(candidates)[2])
    data = bpy.data.cameras.new('Structural joint camera')
    obj = bpy.data.objects.new('Structural joint camera',data)
    scene.collection.objects.link(obj)
    obj.location = target-along*5-across*8+Vector((0,0,3))
    obj.rotation_euler = (target-obj.location).to_track_quat('-Z','Y').to_euler()
    data.type, data.ortho_scale = 'ORTHO', 9.
    data.clip_start, data.clip_end = .02,10000
    scene.camera = obj
    scene.render.filepath = EXPORT_ROOT+'/'+kind.lower()+'-'+str(DATA['seed'])+'-node.png'
    render_still()
scene.camera = camera
scene.render.filepath = EXPORT_ROOT+"/"+kind.lower()+"-"+str(DATA["seed"])+".png"
bpy.ops.wm.save_as_mainfile(filepath=EXPORT_ROOT+"/Support-System.blend", compress=True)
print("SUPPORT_REVIEW_COMPLETE "+scene.render.filepath+" "+json.dumps(DATA["counts"]))
print("SUPPORT_FABRICATION_AUDIT "+scene["fabrication_audit"])
