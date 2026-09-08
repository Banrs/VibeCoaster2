"""Build original VibeCoaster train art in Blender 4.5; execute inside Blender.

Only VCTrain_ datablocks are replaced. Does not clear other work, download assets,
change physics, launch Blender, render, or export the private project externally.
"""
import bpy
import json
import math
from mathutils import Vector

PREFIX = "VCTrain_"
# Root precreates these project-local directories and changes BUILD_ID/paths for
# a new preserved revision. MCP safe mode remains enabled: no raw filesystem I/O.
RUN = "20260908-train-v003"
SOURCE = "D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v003"
EXPORT = "D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v003"

# Replace this script's previous in-memory product only. Previous on-disk runs
# remain immutable; another artist's objects/materials/scenes are untouched.
for obj in list(bpy.data.objects):
    if obj.name.startswith(PREFIX):
        bpy.data.objects.remove(obj, do_unlink=True)
for scene in list(bpy.data.scenes):
    if scene.name.startswith(PREFIX):
        bpy.data.scenes.remove(scene)
for collection in list(bpy.data.collections):
    if collection.name.startswith(PREFIX):
        bpy.data.collections.remove(collection)
for pool in (bpy.data.meshes, bpy.data.curves, bpy.data.materials, bpy.data.cameras,
             bpy.data.lights, bpy.data.worlds):
    for block in list(pool):
        if block.name.startswith(PREFIX) and block.users == 0:
            pool.remove(block)

scene = bpy.data.scenes.new(PREFIX + "AssetReview")
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
root = bpy.data.collections.new(PREFIX + "Collection")
scene.collection.children.link(root)

def collection(label):
    c = bpy.data.collections.new(PREFIX + label)
    root.children.link(c)
    return c

runtime = collection("RuntimeParts_AboveRail")
review = collection("BogieReview_UNVALIDATED")
references = collection("ReferenceGuides")
exports = collection("AssembledExport")
review["clearance_status"] = "UNVALIDATED moving hardware below rail; NOT runtime approved"
review["export_separate"] = True


def linear(c):
    return c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4


def material(label, hexcolor, metal=0.0, rough=.4):
    m = bpy.data.materials.new(PREFIX + label)
    rgb = tuple(linear(int(hexcolor[i:i+2], 16) / 255) for i in (0, 2, 4))
    m.diffuse_color = (*rgb, 1)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Roughness"].default_value = rough
    return m

paint = material("Paint_WarmOrange", "EC6928", .32, .27)
paint_dark = material("Paint_BurntOrange", "AB431E", .25, .34)
padding = material("Padding_Graphite", "272D32", .03, .72)
rubber = material("Rubber_GripAndTyre", "121619", 0, .82)
metal = material("Metal_BrushedAluminium", "ABB4BB", .85, .32)
steel = material("Metal_DarkChassis", "353F48", .72, .4)
accent = material("Trim_Ivory", "EEE5D4", .18, .32)
reference_mat = material("Reference_RailBlue", "298CB8", .35, .4)


def mesh_obj(label, verts, faces, mat, dest=runtime, pivot=(0, 0, 0), smooth=True):
    pivot = Vector(pivot)
    mesh = bpy.data.meshes.new(PREFIX + label + "Mesh")
    mesh.from_pydata([Vector(v) - pivot for v in verts], [], faces)
    mesh.update()
    obj = bpy.data.objects.new(PREFIX + label, mesh)
    dest.objects.link(obj)
    obj.location = pivot
    obj.data.materials.append(mat)
    for poly in mesh.polygons:
        poly.use_smooth = smooth and len(poly.vertices) == 4
    obj["units"] = "metres"
    obj["visual_only"] = True
    return obj


def bevel(obj, width=.015, segments=3):
    mod = obj.modifiers.new("Edge_Radii", "BEVEL")
    mod.width = width
    mod.segments = segments
    mod.limit_method = "ANGLE"
    return obj


def box(label, centre, size, mat, dest=runtime, radius=.015):
    c = Vector(centre)
    hx, hy, hz = (s / 2 for s in size)
    verts = [c + Vector((x*hx, y*hy, z*hz))
             for z in (-1, 1) for y in (-1, 1) for x in (-1, 1)]
    faces = [(0,2,3,1), (4,5,7,6), (0,1,5,4), (2,6,7,3), (0,4,6,2), (1,3,7,5)]
    obj = mesh_obj(label, verts, faces, mat, dest, smooth=False)
    if radius:
        bevel(obj, min(radius, min(size) * .4), 4)
    return obj


def tube(label, points, radius, mat, dest=runtime, pivot=(0,0,0)):
    curve = bpy.data.curves.new(PREFIX + label + "Curve", "CURVE")
    curve.dimensions = "3D"
    curve.resolution_u = 12
    curve.bevel_depth = radius
    curve.bevel_resolution = 3
    curve.use_fill_caps = True
    spline = curve.splines.new("BEZIER")
    spline.bezier_points.add(len(points)-1)
    for bp, p in zip(spline.bezier_points, points):
        bp.co = Vector(p) - Vector(pivot)
        bp.handle_left_type = "AUTO"
        bp.handle_right_type = "AUTO"
    obj = bpy.data.objects.new(PREFIX + label, curve)
    dest.objects.link(obj)
    obj.location = pivot
    curve.materials.append(mat)
    obj["units"] = "metres"
    obj["visual_only"] = True
    return obj


def cylinder(label, centre, radius, length, mat, axis=(0,0,1), dest=runtime, sides=32):
    centre = Vector(centre)
    q = Vector(axis).normalized().to_track_quat("Z", "Y")
    verts = [centre + q @ Vector((radius*math.cos(2*math.pi*i/sides),
                                 radius*math.sin(2*math.pi*i/sides), z*length/2))
             for z in (-1,1) for i in range(sides)]
    faces = [tuple(reversed(range(sides))), tuple(range(sides,2*sides))]
    faces += [(i, (i+1)%sides, (i+1)%sides+sides, i+sides) for i in range(sides)]
    obj = mesh_obj(label, verts, faces, mat, dest, pivot=centre)
    obj["axle_axis_local_source"] = list(axis)
    return obj


def signed_power(v, p):
    return math.copysign(abs(v)**p, v)


def loft_z(label, rings, mat, dest=runtime, sides=64):
    """Each ring: centre X/Y, Z, X radius, Y radius, superellipse exponent."""
    verts = []
    for cx, cy, z, rx, ry, exponent in rings:
        for i in range(sides):
            a = 2*math.pi*i/sides
            verts.append((cx+rx*signed_power(math.cos(a), exponent),
                          cy+ry*signed_power(math.sin(a), exponent), z))
    faces = [tuple(reversed(range(sides)))]
    for r in range(len(rings)-1):
        for i in range(sides):
            a=r*sides+i; b=r*sides+(i+1)%sides
            faces.append((a,b,b+sides,a+sides))
    faces.append(tuple(range((len(rings)-1)*sides, len(rings)*sides)))
    return mesh_obj(label, verts, faces, mat, dest)

# A single continuous moulded shell has exact 2.55 x 1.70 m maximum footprint.
# Separate upper pods and seat shells supply the more complex silhouette.
hull = loft_z("Body_ContouredShell", [
    (0,0,.16,1.15,.71,.52), (0,0,.20,1.23,.81,.48),
    (0,0,.30,1.275,.85,.48), (0,0,.39,1.275,.85,.48),
    (0,0,.48,1.23,.81,.48), (0,0,.515,1.16,.74,.52)], paint)
hull["exact_footprint_m"] = [2.55,1.70]
box("Chassis_CentralDeck", (-.1,0,.24), (2.15,1.30,.13), steel, radius=.045)
box("Floor_AntiSlip", (.22,0,.532), (1.46,1.32,.036), rubber, radius=.014)
# Front cowl drops towards nose rather than becoming a solid wall at eye height.
loft_z("Nose_SculptedCowl", [
    (.88,0,.50,.32,.71,.55), (.87,0,.54,.34,.70,.55),
    (.83,0,.60,.31,.64,.62), (.78,0,.64,.24,.53,.72)], paint)
for side, label in ((-1,"Right"),(1,"Left")):
    y = side*.44
    # Low external side pods, kept well below the eye.
    loft_z(label+"_SidePod", [
        (-.20,side*.735,.47,.73,.080,.55),
        (-.24,side*.733,.58,.68,.082,.55),
        (-.38,side*.730,.70,.49,.080,.62),
        (-.50,side*.720,.735,.29,.064,.72)], paint_dark)
    tube(label+"_IvoryShoulderLine", [(-1.02,side*.795,.465),
         (-.45,side*.818,.48),(.30,side*.807,.48),(.97,side*.73,.50)], .012, accent)
    # Seat pan and back are shaped lofts, not scaled generic cubes.
    loft_z(label+"_SeatOuterPan", [
        (-.33,y,.53,.42,.32,.50),(-.33,y,.61,.43,.32,.50),
        (-.37,y,.685,.40,.31,.55)], paint_dark)
    loft_z(label+"_SeatCushion", [
        (-.28,y,.635,.34,.272,.56),(-.29,y,.69,.35,.279,.56),
        (-.34,y,.735,.31,.260,.64)], padding)
    loft_z(label+"_SeatBackShell", [
        (-.64,y,.65,.105,.315,.50),(-.70,y,.85,.110,.310,.55),
        (-.77,y,1.12,.105,.285,.66),(-.79,y,1.285,.090,.235,.73)], paint)
    loft_z(label+"_SeatBackPadding", [
        (-.565,y,.70,.035,.270,.55),(-.608,y,.86,.060,.265,.57),
        (-.672,y,1.09,.065,.240,.68),(-.700,y,1.235,.052,.202,.78)], padding)
    loft_z(label+"_Headrest", [
        (-.79,y,1.255,.063,.129,.67),(-.805,y,1.37,.077,.141,.67),
        (-.815,y,1.49,.062,.125,.75)], padding)
    # Hinge axis across car. Arms and pad are the closed art pose only.
    hinge = (-.36,side*.765,.765)
    cylinder(label+"_RestraintHinge", hinge, .061,.07,metal,axis=(0,1,0))
    cylinder(label+"_HingeCap", (-.36,side*.807,.765),.037,.012,steel,axis=(0,1,0))
    tube(label+"_LapBarArm", [hinge,(-.10,side*.740,.84),
         (.19,side*.640,.94),(.28,y,.94)], .028, steel, pivot=hinge)
    pad=box(label+"_LapPad", (.275,y,.965), (.22,.47,.12), padding, radius=.045)
    pad["pose"]="closed; animation/ride restraint interlock not implemented"
    tube(label+"_GrabHandle", [(.36,y-.14,.99),(.38,y-.14,1.055),
         (.38,y+.14,1.055),(.36,y+.14,.99)], .016, rubber)
    # Recessed tread plates and visible chassis fasteners add local scale cues.
    box(label+"_FootPlate", (.46,y,.561),(.38,.42,.022),metal,radius=.009)
    for j in range(5):
        box(label+"_TreadGroove%02d"%j, (.32+j*.07,y,.574),(.013,.34,.006),steel,radius=.002)
    for j,x in enumerate((-.95,-.55,.52,.92)):
        cylinder(label+"_ShellFastener%02d"%j,(x,side*.78,.425),.017,.012,
                 metal,axis=(0,side,0),sides=12)
# Short chassis rails stay above rail; nothing here impersonates a certified bogie.
for side in (-1,1):
    box("ChassisRail"+str(side),(-.08,side*.52,.135),(2.16,.09,.07),metal,radius=.014)
box("RearServicePanel",(-1.13,0,.37),(.05,.93,.17),steel,radius=.018)
for i in range(6):
    box("RearVent%02d"%i,(-1.158,-.34+i*.136,.37),(.006,.065,.085),rubber,radius=.004)

# Separate mechanically arranged running/side/up-stop wheel study. Contact
# surfaces meet the circular rail at z +/-0.085 or lateral +/-0.085.
# This collection is NEVER merged into the runtime-approved above-rail car.
for axle, x in enumerate((-.87,.87)):
    for side,label in ((-1,"Right"),(1,"Left")):
        tag=f"Bogie{axle}_{label}_"
        y=side*.65
        cylinder(tag+"RunningTyre",(x,y,.215),.13,.115,rubber,axis=(0,1,0),dest=review)
        cylinder(tag+"RunningHub",(x,y,.215),.073,.125,metal,axis=(0,1,0),dest=review)
        cylinder(tag+"UpstopTyre",(x,y,-.165),.08,.09,rubber,axis=(0,1,0),dest=review)
        cylinder(tag+"UpstopHub",(x,y,-.165),.040,.103,metal,axis=(0,1,0),dest=review)
        cylinder(tag+"SideGuideTyre",(x,side*.81,0),.075,.07,rubber,axis=(0,0,1),dest=review)
        cylinder(tag+"SideGuideHub",(x,side*.81,0),.036,.079,metal,axis=(0,0,1),dest=review)
        tube(tag+"CFrame",[(x,side*.73,.215),(x,side*.865,.215),
             (x,side*.92,-.02),(x,side*.865,-.165),(x,side*.72,-.165)],
             .030,steel,dest=review)
        cylinder(tag+"RunningAxle",(x,side*.73,.215),.023,.23,metal,
                 axis=(0,1,0),dest=review)
        cylinder(tag+"UpstopAxle",(x,side*.72,-.165),.017,.18,metal,
                 axis=(0,1,0),dest=review)
        box(tag+"GuideMount",(x,side*.81,-.09),(.095,.13,.042),steel,dest=review,radius=.013)
for side in (-1,1):
    cylinder("ReferenceRail"+str(side),(0,side*.65,0),.085,2.9,reference_mat,
             axis=(1,0,0),dest=references,sides=48)

# Empty witness markers carry the exact contract without adding visible geometry.
for label,pos in (("RailOrigin",(0,0,0)),("EyeDatum_1p2m",(0,0,1.2)),
                  ("ForwardWitness",(1,0,0)),("RiderRightWitness",(0,-1,0))):
    obj=bpy.data.objects.new(PREFIX+label,None)
    references.objects.link(obj)
    obj.location=pos
    obj.empty_display_type="ARROWS"
    obj.empty_display_size=.12


print("VC_TRAIN_STAGE_01_CONSTRUCTED="+json.dumps({"scene":scene.name,"runtime_objects":len(runtime.objects),"review_objects":len(review.objects),"source_scene_active":bpy.context.scene.name,"vertices":sum(len(o.data.vertices) for o in runtime.objects if o.type=="MESH")}))
