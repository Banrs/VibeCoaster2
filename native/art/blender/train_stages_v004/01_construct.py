"""Build original VibeCoaster train art in Blender 5.2; execute inside Blender.

Only VCTrain4_ datablocks are replaced. Does not clear other work, download assets,
change physics, launch Blender, render, or export the private project externally.
"""
import bpy
import bmesh
import json
import math
from mathutils import Vector

PREFIX = "VCTrain4_"
# Root precreates these project-local directories and changes BUILD_ID/paths for
# a new preserved revision. MCP safe mode remains enabled: no raw filesystem I/O.
RUN = "20260908-train-v004"
SOURCE = "D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v004"
EXPORT = "D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v004"

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
review["clearance_status"] = "No wheel geometry in v004 runtime; previous unvalidated bogie study remains separate"
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

paint = material("Shell_PearlTitanium", "D3DCE0", .60, .24)
paint_dark = material("Shell_DeepPetrol", "15353C", .60, .29)
padding = material("Padding_Graphite", "151C22", .02, .66)
rubber = material("Rubber_GripAndTyre", "121619", 0, .82)
metal = material("Metal_BrushedAluminium", "ABB4BB", .85, .32)
steel = material("Metal_DarkChassis", "353F48", .72, .4)
accent = material("Trim_IceCyan", "53C6D0", .42, .24)
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


# Original high-speed visual concept. Fixed above-rail envelope; no altered
# train physics, gauge, mass, speed, force telemetry or camera position.
carbon=material("StructuralCarbon", "10181D", .36, .36)
ceramic=material("Restraint_Ceramic", "AFBCC3", .48, .27)
glass=material("WindDeflector_ClearCyan", "A8DCE4", .0, .095)
bsdf=glass.node_tree.nodes.get("Principled BSDF")
bsdf.inputs["Transmission Weight"].default_value=.82
bsdf.inputs["IOR"].default_value=1.49
bsdf.inputs["Alpha"].default_value=.26
glass.diffuse_color=(*glass.diffuse_color[:3],.26)
if hasattr(glass,"surface_render_method"):
    glass.surface_render_method="DITHERED"
glass["UE_import_requirement"]="Explicit translucent/two-sided thin shield material review; FBX slot import alone does not verify optical behaviour"


def profile_x(label, sections, mat):
    verts=[]
    for x,w,bottom,top in sections:
        cy=min(.050,w*.15)
        cz=min(.028,(top-bottom)*.20)
        yz=[(-w+cy,bottom),(w-cy,bottom),(w,bottom+cz),(w,top-cz),
            (w-cy,top),(-w+cy,top),(-w,top-cz),(-w,bottom+cz)]
        verts.extend((x,y,z) for y,z in yz)
    faces=[tuple(reversed(range(8)))]
    for row in range(len(sections)-1):
        for i in range(8):
            a=row*8+i;b=row*8+(i+1)%8
            faces.append((a,b,b+8,a+8))
    faces.append(tuple(range((len(sections)-1)*8,len(sections)*8)))
    return mesh_obj(label,verts,faces,mat,smooth=False)

# Narrow leading tip and continuous hard shoulder. This has no full-width
# rubber bumper, oval inflated tub, or separate oversized bumper-car bonnet.
hull=profile_x("Monocoque_AeroWedge",[
    (-1.275,.55,.16,.34),(-1.10,.70,.11,.43),(-.70,.82,.10,.45),
    (-.35,.85,.10,.49),(.15,.85,.10,.53),(.50,.72,.105,.48),
    (.85,.53,.11,.39),(1.13,.32,.135,.27),(1.275,.20,.155,.205)],paint)
hull["exact_footprint_m"]=[2.55,1.70]
hull["design_intent"]="2030s compact high-speed wedge; visual concept, no CFD or structural certification"
profile_x("VentralMachinedChassis",[(-1.16,.51,.11,.18),(-.85,.61,.10,.19),
    (.55,.52,.10,.21),(1.10,.20,.135,.17)],steel)
# Twin tapering blades frame the open cabin rather than a bathtub wall.
for side,label in ((-1,"Right"),(1,"Left")):
    y=side*.45
    mesh_obj(label+"_StructuralSideBlade",[
        (-1.15,side*.63,.32),(-.84,side*.81,.36),(.20,side*.83,.47),(.87,side*.45,.35),
        (-1.13,side*.61,.55),(-.76,side*.78,.64),(.17,side*.78,.66),(.82,side*.43,.40)],
        [(0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0)],carbon,smooth=False)
    tube(label+"_AeroShoulderAccent",[(-1.10,side*.634,.552),(-.74,side*.780,.645),
        (.17,side*.781,.665),(.80,side*.438,.410)],.009,accent)
    # Machined seat supports are visibly load-path shaped, above the validated rail datum.
    for offset in (-.21,.21):
        tube(label+"_SeatSupport"+str(offset),[(-.82,y+offset,.29),(-.69,y+offset,.46),
             (-.36,y+offset,.56)],.030,metal)
    loft_z(label+"_SeatBucketShell",[
        (-.43,y,.43,.35,.294,.44),(-.39,y,.55,.39,.302,.45),
        (-.44,y,.645,.355,.290,.47)],paint_dark,sides=48)
    loft_z(label+"_SeatPanPadding",[
        (-.355,y,.585,.285,.251,.49),(-.365,y,.645,.302,.259,.50),
        (-.410,y,.690,.273,.241,.53)],padding,sides=48)
    # Narrow continuous spine and shoulder pods replace rectangular old seat backs.
    loft_z(label+"_SeatSpineMonocoque",[
        (-.72,y,.55,.075,.285,.44),(-.79,y,.82,.085,.263,.47),
        (-.89,y,1.10,.074,.241,.51),(-.93,y,1.34,.073,.202,.55),
        (-.95,y,1.49,.059,.164,.62)],paint_dark,sides=48)
    loft_z(label+"_SeatBackPadding",[
        (-.665,y,.70,.040,.230,.50),(-.704,y,.88,.048,.218,.51),
        (-.799,y,1.12,.046,.189,.58),(-.83,y,1.27,.038,.146,.63)],padding,sides=48)
    loft_z(label+"_IntegratedHeadPad",[
        (-.872,y,1.245,.035,.122,.52),(-.883,y,1.365,.043,.129,.54),
        (-.894,y,1.465,.033,.107,.62)],padding,sides=48)
    for ear in (-1,1):
        # Head/ear deflection cheeks sweep rearward, leaving the centre sightline clear.
        ey=y+ear*.171
        mesh_obj(label+"_HeadCheek"+str(ear),[
            (-.96,ey-ear*.022,1.13),(-.78,ey-ear*.012,1.18),(-.78,ey-ear*.004,1.42),(-.98,ey-ear*.016,1.51),
            (-.96,ey+ear*.020,1.13),(-.78,ey+ear*.017,1.18),(-.78,ey+ear*.020,1.42),(-.98,ey+ear*.022,1.51)],
            [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],paint,smooth=False)
    tube(label+"_SeatSpineTrim",[(-.845,y+side*.251,.86),(-.956,y+side*.20,1.31),
        (-.970,y+side*.135,1.47)],.008,accent)
    # Over-the-shoulder routing terminates at an open lap pad, rather than a
    # closed harness across the rider's chest. Its hinge and support remain visible.
    hinge=(-.91,y+side*.235,1.18)
    cylinder(label+"_UpperRestraintPivot",hinge,.044,.052,metal,axis=(0,1,0))
    tube(label+"_CarbonRestraintArm",[hinge,(-.68,y+side*.255,1.14),
        (-.25,y+side*.251,.95),(.07,y+side*.205,.855)],.025,carbon,pivot=hinge)
    lap=loft_z(label+"_SculptedLapPad",[(.06,y,.79,.16,.238,.46),
        (.055,y,.865,.173,.232,.50),(.04,y,.91,.136,.206,.60)],padding,sides=48)
    lap["pose"]="closed visual pose; no invented restraint controller"
    tube(label+"_LowerLapCrossbar",[(.075,y-side*.195,.832),(.11,y,.838),(.075,y+side*.205,.855)],.022,ceramic)
    for hand in (-1,1):
        tube(label+"_RecessedHandGrip"+str(hand),[(.135,y+hand*.13,.87),
            (.175,y+hand*.13,.945),(.095,y+hand*.16,.97)],.012,rubber)
    # Clear, raked per-seat wind deflector. Top reaches eye height at rider
    # positions but the centre camera lane remains open. No aerodynamic claim.
    shield_vertices=[]
    nx=6;ny=10
    for back in (0,1):
        for iz in range(nx+1):
            t=iz/nx
            for iy in range(ny+1):
                u=-1+2*iy/ny
                z=.56+.73*t-.045*u*u*t
                x=.92-.33*t+.080*u*u-.004*back
                yy=y+u*(.31-.050*t)
                shield_vertices.append((x,yy,z))
    ring=(nx+1)*(ny+1)
    faces=[]
    for iz in range(nx):
        for iy in range(ny):
            a=iz*(ny+1)+iy;b=a+1;c=a+ny+1;d=c+1
            faces.extend([(a,b,d,c),(a+ring,c+ring,d+ring,b+ring)])
    boundary=list(range(ny+1))+[iz*(ny+1)+ny for iz in range(1,nx+1)]+list(reversed(range(nx*(ny+1),nx*(ny+1)+ny)))+[iz*(ny+1) for iz in range(nx-1,0,-1)]
    for i,a in enumerate(boundary):
        b=boundary[(i+1)%len(boundary)]
        faces.append((a,a+ring,b+ring,b))
    shield=mesh_obj(label+"_ClearWindDeflector",shield_vertices,faces,glass)
    shield["optics_status"]="Original visual shield; not validated aerodynamic protection"
    for edge in (-1,1):
        tube(label+"_ShieldEdgeSpar"+str(edge),[(1.000,y+edge*.310,.56),
            (.835,y+edge*.285,.90),(.670,y+edge*.260,1.245)],.011,metal)
    tube(label+"_ShieldTopRim",[(.670,y-.260,1.245),(.590,y,1.290),
        (.670,y+.260,1.245)],.008,paint_dark)
    box(label+"_FootwellCarbon",(.28,y,.53),(.54,.43,.025),carbon,radius=.007)
    for k in range(4):
        box(label+"_TreadInlay%02d"%k,(.13+k*.09,y,.546),(.018,.33,.005),metal,radius=.001)
    # Functional-scale vents and service seams, with restrained accent use.
    for k in range(5):
        box(label+"_ShoulderCoolingSlot%02d"%k,(-.97+k*.075,side*.67,.47),(.038,.045,.025),steel,radius=.004)
    for k,x in enumerate((-.97,-.45,.13)):
        cylinder(label+"_FlushFastener%02d"%k,(x,side*.803,.39),.012,.008,metal,axis=(0,side,0),sides=12)
# Splitter and blade graphics sit inside the nose envelope, not ahead of it.
profile_x("NoseCentralCarbonBlade",[(.39,.093,.480,.498),(.80,.070,.375,.394),
    (1.25,.031,.191,.205)],carbon)
for side in (-1,1):
    tube("NoseSignature"+str(side),[(.53,side*.590,.463),(.85,side*.426,.385),
        (1.12,side*.240,.274)],.006,accent)
box("RearChassisServiceRail",(-1.16,0,.29),(.11,.92,.055),metal,radius=.009)

for label,pos in (("RailOrigin",(0,0,0)),("EyeDatum_1p2m",(0,0,1.2)),
                  ("ForwardWitness",(1,0,0)),("RiderRightWitness",(0,-1,0))):
    obj=bpy.data.objects.new(PREFIX+label,None)
    references.objects.link(obj)
    obj.location=pos
    obj.empty_display_type="ARROWS"
    obj.empty_display_size=.12
# Each authored solid is closed; normalise outward face orientation after
# mirrored left/right construction, without changing any old source datablock.
for obj in runtime.objects:
    if obj.type!="MESH": continue
    bm=bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()
print("VC_TRAIN4_STAGE_01_CONSTRUCTED="+json.dumps({"scene":scene.name,"runtime_objects":len(runtime.objects),
    "runtime_materials":sorted(set(m.name for o in runtime.objects for m in o.data.materials)),
    "source_scene_active":bpy.context.scene.name,"note":"Above-rail art only; paired wind deflectors require explicit UE optical material review"}))
