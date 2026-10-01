"""Exa track assembly study. Execute exclusively through Blender MCP.

The runner injects the dimension profile and the pure geometry module. Every
rail, tie, wheel placeholder and mount uses the rail-midpoint datum in metres.
Only the dedicated Exa Track Study scene is replaced on a repeat run.
"""
import bpy
import bmesh
import json
import math
from mathutils import Matrix, Vector

# TRACK_STUDY_GEOMETRY

PROFILE = json.loads(__TRACK_PROFILE__)
EXPORT_ROOT = "__EXPORT_ROOT__"
checks = validate_profile(PROFILE)
scene = bpy.data.scenes.get("Exa Track Study") or bpy.data.scenes.new("Exa Track Study")
bpy.context.window.scene = scene
for obj in list(scene.objects):
    bpy.data.objects.remove(obj, do_unlink=True)
for collection in list(scene.collection.children):
    bpy.data.collections.remove(collection)
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
scene["profile_id"] = PROFILE["id"]
scene["status"] = "Exa reference study; dimensions are proposed; default.3 not modified"
scene["rail_gauge_metres"] = PROFILE["gauge"]
scene["support_mount_local"] = support_mount(PROFILE)


def material(name, spec):
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.use_nodes = True
    mat.diffuse_color = (*spec["colour"], 1)
    shader = next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    shader.inputs["Base Color"].default_value = (*spec["colour"], 1)
    shader.inputs["Metallic"].default_value = spec["metallic"]
    shader.inputs["Roughness"].default_value = spec["roughness"]
    return mat


materials = {name: material(name, spec) for name, spec in PROFILE["palette"].items()}
paint, steel = materials["TS_Paint"], materials["TS_RunningSteel"]
support, fastener = materials["TS_Support"], materials["TS_Fastener"]
wheelmat, annotation = materials["TS_Wheel"], materials["TS_Annotation"]
assets = []


def collection(name):
    result = bpy.data.collections.new(name)
    scene.collection.children.link(result)
    return result


def move_to(obj, group):
    for old in list(obj.users_collection):
        old.objects.unlink(obj)
    group.objects.link(obj)


def mesh_object(name, vertices, faces, mats, group, smooth=False, bands=None):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    group.objects.link(obj)
    for mat in mats:
        mesh.materials.append(mat)
    for i, polygon in enumerate(mesh.polygons):
        polygon.use_smooth = smooth
        if bands is not None:
            polygon.material_index = bands[i]
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(mesh)
    bm.free()
    return obj


def cylinder(name, a, b, radius, mat, group, sides=40, radius2=None):
    a, b = Vector(a), Vector(b)
    bpy.ops.mesh.primitive_cone_add(vertices=sides, radius1=radius,
        radius2=radius if radius2 is None else radius2, depth=(b-a).length, location=(a+b)*.5)
    obj = bpy.context.object
    obj.name = name
    obj.rotation_euler = (b-a).to_track_quat("Z", "Y").to_euler()
    obj.data.materials.append(mat)
    for poly in obj.data.polygons:
        poly.use_smooth = len(poly.vertices) == 4
    move_to(obj, group)
    return obj


def box(name, centre, size, mat, group, bevel=0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=centre)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = size
    obj.data.materials.append(mat)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        mod = obj.modifiers.new("Rounded fabrication edges", "BEVEL")
        mod.width, mod.segments = bevel, 3
        bpy.ops.object.modifier_apply(modifier=mod.name)
    move_to(obj, group)
    return obj


def ring_x(name, x, inner, outer, depth, z, mat, group, count=64):
    vertices = [(xx, r*math.cos(2*math.pi*j/count), z+r*math.sin(2*math.pi*j/count))
                for xx in (x-depth/2, x+depth/2) for r in (inner, outer) for j in range(count)]
    faces = []
    for j in range(count):
        k = (j+1) % count
        faces.extend([(j, k, k+count, j+count),
                      (j+2*count, j+3*count, k+3*count, k+2*count),
                      (j, j+2*count, k+2*count, k),
                      (j+count, k+count, k+3*count, j+3*count)])
    return mesh_object(name, vertices, faces, [mat], group)


def pose(frame):
    p, f, r, u = frame
    return Matrix(((f[0], r[0], u[0], p[0]), (f[1], r[1], u[1], p[1]),
                   (f[2], r[2], u[2], p[2]), (0, 0, 0, 1)))


def copies(parts, group, frame):
    result = []
    matrix = pose(frame)
    for source in parts:
        obj = source.copy()
        group.objects.link(obj)
        obj.hide_render = False
        obj.hide_set(False)
        obj.matrix_world = matrix @ source.matrix_world
        result.append(obj)
    return result


def export_asset(name, parts):
    # Export disposable merged copies; the saved Blender source remains editable.
    bpy.ops.object.select_all(action="DESELECT")
    clones = []
    for source in parts:
        obj = source.copy()
        obj.data = source.data.copy()
        scene.collection.objects.link(obj)
        obj.hide_set(False)
        obj.hide_render = False
        obj.select_set(True)
        clones.append(obj)
    bpy.context.view_layer.objects.active = clones[0]
    bpy.ops.object.join()
    obj = bpy.context.object
    obj.name = name
    scene.cursor.location = (0, 0, 0)
    bpy.ops.object.origin_set(type="ORIGIN_CURSOR")
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    tri = obj.modifiers.new("Export triangles", "TRIANGULATE")
    bpy.ops.object.modifier_apply(modifier=tri.name)
    coordinates = [v.co.copy() for v in obj.data.vertices]
    entry = {"name": name, "boundsMetres": [[min(v[i] for v in coordinates) for i in range(3)],
            [max(v[i] for v in coordinates) for i in range(3)]], "vertices": len(coordinates),
            "triangles": len(obj.data.polygons), "materials": [m.name for m in obj.data.materials],
            "purpose": "track study only"}
    assets.append(entry)
    # Match the verified FBX importer convention used by the retained kit.
    for v in obj.data.vertices:
        v.co.x = -v.co.x
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.reverse_faces(bm, faces=list(bm.faces))
    bm.normal_update()
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()
    bpy.ops.export_scene.fbx(filepath=EXPORT_ROOT+"/"+name+".fbx", use_selection=True,
        object_types={"MESH"}, global_scale=1, apply_unit_scale=True,
        apply_scale_options="FBX_SCALE_NONE", axis_forward="-Y", axis_up="Z",
        use_space_transform=False, bake_space_transform=False, mesh_smooth_type="FACE",
        use_tspace=False, add_leaf_bones=False, bake_anim=False, path_mode="STRIP")
    data = obj.data
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.meshes.remove(data)


kit = collection("00 / Editable component kit at rail datum")
half = PROFILE["gauge"]*.5-PROFILE["crosshead_end_inset"]
z = PROFILE["crosshead_height"]
tieparts = [cylinder("Crosshead / transverse tube", (0, -half, z), (0, half, z),
                     PROFILE["crosshead_radius"], paint, kit)]
for side in (-1, 1):
    vertices, faces = gusset_mesh(PROFILE, side)
    tieparts.append(mesh_object("Crosshead / longitudinal welded gusset", vertices, faces, [paint], kit))
export_asset("SM_ExaCrosshead", tieparts)

# The column ends flat at full diameter; separate webs meet the smaller spine.
radius, depth = PROFILE["spine_radius"], PROFILE["spine_depth"]
vertices, faces = support_head_mesh(PROFILE)
head = mesh_object("Support head / full-width capped column", vertices, faces, [support], kit)
for poly in list(head.data.polygons)[:48]:
    poly.use_smooth = True
headparts = [head]
mount = support_mount(PROFILE)
flange_top = mount[2]+PROFILE["support_flange_size"][2]
headparts.append(cylinder("Support head / upper mating flange", (0, 0, mount[2]),
    (0, 0, flange_top), PROFILE["support_flange_size"][0]/2, support, kit, 64))
headparts.append(cylinder("Support head / lower mating flange", (0, 0, mount[2]-.045),
    (0, 0, mount[2]+.001), PROFILE["support_flange_size"][0]/2, support, kit, 64))
for part_name, (vertices, faces) in support_bearing_parts(PROFILE):
    headparts.append(mesh_object(part_name, vertices, faces, [support], kit))
for j in range(PROFILE["support_bolt_count"]):
    angle = 2*math.pi*j/PROFILE["support_bolt_count"]
    x = PROFILE["support_bolt_circle_radius"]*math.cos(angle)
    y = PROFILE["support_bolt_circle_radius"]*math.sin(angle)
    headparts.append(cylinder("Support head / washer", (x, y, flange_top),
        (x, y, flange_top+.007), .033, fastener, kit, 32))
    headparts.append(cylinder("Support head / hex fastener", (x, y, flange_top+.007),
        (x, y, flange_top+.032), .024, fastener, kit, 6))
export_asset("SM_ExaSupportHead", headparts)

# Wheel barrels are removable fit placeholders. They are not a train design.
wheelparts = []
for wheel in wheel_envelopes(PROFILE):
    centre = Vector(wheel["centre"])
    axis = Vector((0, 0, 1) if wheel["role"] == "guide" else (0, 1, 0))
    halfwidth = wheel["width"]*.5
    wheelparts.append(cylinder(wheel["role"]+" wheel / fit placeholder",
        centre-axis*halfwidth, centre+axis*halfwidth, wheel["radius"], wheelmat, kit, 64))
    for side in (-1, 1):
        a = centre+axis*(side*(halfwidth+.001))
        wheelparts.append(cylinder("Wheel fit / hub", a, a+axis*(side*.009),
            wheel["radius"]*.53, steel, kit, 48))
export_asset("SM_ExaWheelFit", wheelparts)


def track(kind, length, group):
    parts = []
    for side in (-1, 1):
        vertices, faces, bands = swept_tube(PROFILE, kind, length,
            side*PROFILE["gauge"]*.5, 0, PROFILE["rail_radius"], PROFILE["rail_wall"])
        parts.append(mesh_object("Running rail / "+str(side), vertices, faces,
                                 [paint, steel], group, True, bands))
    vertices, faces, bands = swept_tube(PROFILE, kind, length, 0, -PROFILE["spine_depth"],
                                       PROFILE["spine_radius"], PROFILE["spine_wall"])
    parts.append(mesh_object("Exa heavy tubular spine", vertices, faces, [paint, steel], group, True, bands))
    stations = math.floor(length/PROFILE["tie_spacing"])
    for i in range(stations):
        s = (i+.5)*PROFILE["tie_spacing"]
        parts.extend(copies(tieparts, group, study_frame(kind, s)))
    return parts


studies = []
for kind, length, offset, name in (("straight", 11.2, (-5.6, -10, 2.8), "SM_ExaTrackStraight"),
        ("banked", 22.4, (-11.2, -2, 3.2), "SM_ExaTrackBanked"),
        ("inverted", 33.6, (-16.8, 10, 3.6), "SM_ExaTrackInverted")):
    group = collection("Study / "+kind)
    parts = track(kind, length, group)
    if kind == "straight":
        x = length/2
        for dx in (-.024, .024):
            parts.append(ring_x("Spine joint / flange", x+dx, radius-.003, radius+.065,
                                 .036, -depth, paint, group))
        for j in range(20):
            a = 2*math.pi*j/20
            y, zz = (radius+.038)*math.cos(a), -depth+(radius+.038)*math.sin(a)
            parts.append(cylinder("Spine joint / stud", (x-.067, y, zz), (x+.067, y, zz), .014, fastener, group, 6))
    export_asset(name, parts)
    supports = []
    if kind == "straight":
        for distance in (2.8, 8.4):
            supports.extend(copies(headparts, group, study_frame(kind, distance)))
            bottom = -offset[2]
            base_size = PROFILE["support_base_size"]
            supports.append(cylinder("Review stand / short support",
                (distance, 0, bottom+base_size[2]-.01), (distance, 0, mount[2]+.005),
                PROFILE["support_column_radius"], support, group, 48))
            supports.append(box("Review stand / base", (distance, 0, bottom+base_size[2]/2),
                base_size, support, group, .02))
    for obj in list(group.objects):
        obj.location += Vector(offset)
    studies.append({"kind": kind, "group": group, "length": length, "offset": offset, "asset": name})

section = collection("Study / cross-section and wheel fit")
track("straight", 1.4, section)
copies(headparts, section, study_frame("straight", .7))
copies(wheelparts, section, study_frame("straight", -.32))
for obj in list(section.objects):
    obj.location.z += 2

# Hide source kit, while keeping it editable and named in the .blend.
for obj in kit.objects:
    obj.hide_render = True
    obj.hide_set(True)

stage = collection("Review / studio")
box("Studio floor", (0, 0, -.15), (150, 150, .3), materials["TS_Floor"], stage, .01)
scene.world = bpy.data.worlds.new("Exa Study World")
scene.world.use_nodes = True
background = next(n for n in scene.world.node_tree.nodes if n.type == 'BACKGROUND')
background.inputs[0].default_value = (.32, .38, .44, 1)
background.inputs[1].default_value = .6
for name, position, energy, size in (("Key softbox", (-8, -14, 16), 4300, 12),
        ("Rim softbox", (10, 8, 16), 5500, 10), ("Fill", (-14, 12, 8), 3500, 10)):
    data = bpy.data.lights.new(name, "AREA")
    data.energy, data.shape, data.size = energy, "DISK", size
    obj = bpy.data.objects.new(name, data)
    stage.objects.link(obj)
    obj.location = position
    obj.rotation_euler = (Vector((0, 0, 2))-obj.location).to_track_quat("-Z", "Y").to_euler()
sun_data = bpy.data.lights.new("Soft sunlight", "SUN")
sun_data.energy, sun_data.angle = 1.5, math.radians(18)
sun = bpy.data.objects.new("Soft sunlight", sun_data)
stage.objects.link(sun)
sun.rotation_euler = (math.radians(28), math.radians(-23), math.radians(-35))
camera_data = bpy.data.cameras.new("Study camera")
camera = bpy.data.objects.new("Study camera", camera_data)
stage.objects.link(camera)
scene.camera = camera
scene.render.engine = "CYCLES"
devices = bpy.context.preferences.addons['cycles'].preferences.devices
render_device = 'GPU' if any(d.use and d.type != 'CPU' for d in devices) else 'CPU'
scene.cycles.device = next(v.identifier for v in scene.cycles.bl_rna.properties['device'].enum_items if v.identifier == render_device)
scene.render.threads_mode = next(v.identifier for v in scene.render.bl_rna.properties['threads_mode'].enum_items if v.identifier == 'FIXED')
scene.render.threads = 4
scene.cycles.samples = 16
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1800, 1120
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.view_settings.view_transform = "AgX"
scene.view_settings.exposure = .55


def visibility(kinds, show_section=False):
    for study in studies:
        for obj in study["group"].objects:
            obj.hide_render = study["kind"] not in kinds
    for obj in section.objects:
        obj.hide_render = not show_section


def camera_pose(position, target, scale, perspective=False):
    camera.location = position
    camera.rotation_euler = (Vector(target)-camera.location).to_track_quat("-Z", "Y").to_euler()
    camera.data.type = "PERSP" if perspective else "ORTHO"
    camera.data.ortho_scale = scale
    camera.data.lens = 28
    camera.data.clip_end = 300


# Save once before rendering; a rendering failure cannot lose the model.
visibility({"straight", "banked", "inverted"})
camera_pose((-31, -36, 29), (0, 2, 2), 47)
for area in bpy.context.screen.areas:
    if area.type == "VIEW_3D":
        area.spaces.active.region_3d.view_perspective = "CAMERA"
bpy.ops.wm.save_as_mainfile(filepath=EXPORT_ROOT+"/Exa-Track-Study.blend", compress=True)
print("ASSET_MANIFEST="+json.dumps({"createdThrough": "Blender MCP execute_blender_code",
    "blender": bpy.app.version_string, "profile": PROFILE, "checks": checks, "assets": assets,
    "studies": [{"kind": s["kind"], "lengthMetres": s["length"], "offsetMetres": s["offset"],
                 "asset": s["asset"]} for s in studies]}))
print("EXA_TRACK_STUDY_MODEL_SAVED")
