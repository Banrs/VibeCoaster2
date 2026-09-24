"""V3 ride kit, authored through Blender MCP in metres at the runtime pivots.

X is forward and Z is up. Every module stays inside its existing conservative
collision solid. The open roof, hollow track web and high-speed lead/passenger train are
deliberate sightline choices; no transparent windshield is needed.
"""
import bpy
import bmesh
import math
import json
from mathutils import Vector

EXPORT_ROOT = "__EXPORT_ROOT__"
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"
scene.unit_settings.scale_length = 1.0
scene.world.color = (0.12, 0.12, 0.12)


def material(name, colour, metallic=0.0, roughness=0.45):
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    mat.diffuse_color = (*colour, 1.0)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*colour, 1.0)
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    return mat


petrol = material("VC2_Petrol", (0.018, 0.105, 0.13), 0.62, 0.29)
pearl = material("VC2_Pearl", (0.57, 0.64, 0.65), 0.4, 0.27)
graphite = material("VC2_Graphite", (0.022, 0.031, 0.035), 0.45, 0.42)
copper = material("VC2_Copper", (0.83, 0.265, 0.045), 0.46, 0.34)
steel = material("VC2_Steel", (0.31, 0.38, 0.41), 0.78, 0.3)
padding = material("VC2_Padding", (0.016, 0.024, 0.029), 0.0, 0.82)
concrete = material("VC2_Concrete", (0.38, 0.395, 0.36), 0.0, 0.88)
light = material("VC2_Light", (0.59, 0.88, 0.93), 0.0, 0.29)
glass = material("VC2_Glass", (0.43, 0.69, 0.76), 0.0, 0.08)
glass.diffuse_color=(.43,.69,.76,.10)
glass.node_tree.nodes["Principled BSDF"].inputs["Alpha"].default_value=.10
glass.node_tree.nodes["Principled BSDF"].inputs["Transmission Weight"].default_value=.0
glass.surface_render_method="BLENDED"

# ASSET_MODULES


def finish(obj, name, mat, bevel=0.0):
    obj.name = name
    obj.data.materials.append(mat)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        mod = obj.modifiers.new("Machined edge", "BEVEL")
        mod.width = bevel
        mod.segments = 2
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
    return obj


def box(name, centre, size, mat, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=centre)
    obj = bpy.context.object
    obj.dimensions = size
    return finish(obj, name, mat, bevel)


def tube(name, a, b, radius, mat, sides=12, radius2=None):
    a, b = Vector(a), Vector(b)
    bpy.ops.mesh.primitive_cone_add(vertices=sides, radius1=radius,
                                   radius2=radius if radius2 is None else radius2,
                                   depth=(b-a).length, end_fill_type="NGON", location=(a+b)*0.5)
    obj = bpy.context.object
    obj.rotation_euler = (b-a).to_track_quat("Z", "Y").to_euler()
    finish(obj, name, mat)
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    return obj


def loft(name, sections, mat):
    # A chamfered, closed eight-point transverse shell; variable sections sculpt
    # the silhouette rather than stacking rectangular proxy boxes.
    vertices, faces = [], []
    for x, width, bottom, top in sections:
        height = top-bottom
        ring = [(-.8*width,bottom), (.8*width,bottom), (width,bottom+.24*height),
                (width,top-.23*height), (.72*width,top), (-.72*width,top),
                (-width,top-.23*height), (-width,bottom+.24*height)]
        vertices.extend((x,y,z) for y,z in ring)
    faces.append(tuple(reversed(range(8))))
    for ring in range(len(sections)-1):
        for j in range(8):
            faces.append((ring*8+j,ring*8+(j+1)%8,(ring+1)*8+(j+1)%8,(ring+1)*8+j))
    faces.append(tuple((len(sections)-1)*8+j for j in range(8)))
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name,mesh)
    scene.collection.objects.link(obj)
    obj.data.materials.append(mat)
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    return obj


def bent_tube(name, points, radius, mat):
    bpy.ops.object.select_all(action="DESELECT")
    curve = bpy.data.curves.new(name,"CURVE")
    curve.dimensions = "3D"
    curve.resolution_u = 2
    curve.bevel_depth = radius
    curve.bevel_resolution = 1
    curve.resolution_u = 6
    spline = curve.splines.new("BEZIER")
    spline.bezier_points.add(len(points)-1)
    for point, co in zip(spline.bezier_points,points):
        point.co=co
        point.handle_left_type="AUTO"
        point.handle_right_type="AUTO"
    obj=bpy.data.objects.new(name,curve)
    scene.collection.objects.link(obj)
    bpy.context.view_layer.objects.active=obj
    obj.select_set(True)
    bpy.ops.object.convert(target="MESH")
    obj=bpy.context.object
    obj.data.materials.append(mat)
    return obj


assets = []
asset_objects = []


def collect(name, start):
    parts=[obj for obj in scene.objects if obj.type=="MESH" and obj not in start]
    bpy.ops.object.select_all(action="DESELECT")
    for obj in parts:
        obj.select_set(True)
    bpy.context.view_layer.objects.active=parts[0]
    bpy.ops.object.join()
    obj=bpy.context.object
    obj.name=name
    scene.cursor.location=(0,0,0)
    bpy.ops.object.origin_set(type="ORIGIN_CURSOR")
    bpy.ops.object.transform_apply(location=False,rotation=True,scale=True)
    tri=obj.modifiers.new("Export triangles","TRIANGULATE")
    bpy.ops.object.modifier_apply(modifier=tri.name)
    coordinates=[vertex.co for vertex in obj.data.vertices]
    bounds=[[round(min(p[i] for p in coordinates),8) for i in range(3)],
            [round(max(p[i] for p in coordinates),8) for i in range(3)]]
    entry={"name":name,"boundsMetres":bounds,"triangles":len(obj.data.polygons),
           "vertices":len(coordinates),"materials":[mat.name for mat in obj.data.materials]}
    if name == "SM_TrainCoupler":
        entry["runtime"] = False  # Frozen train concept; runtime coupling is out of scope.
    assets.append(entry)
    asset_objects.append(obj)
    collection=bpy.data.collections.new(name)
    scene.collection.children.link(collection)
    for previous in list(obj.users_collection):
        previous.objects.unlink(obj)
    collection.objects.link(obj)
    obj["runtime_asset"]="/Game/Art/V3/"+name
    # FBX's Unreal handedness conversion reflects its source X even when
    # scene conversion is disabled. Reflect ONLY the temporary export mesh and
    # its winding, so the imported numerical XYZ match the canonical source.
    # The .blend source, measured manifest and all artist-facing pivots stay put.
    export_obj=obj.copy()
    export_obj.data=obj.data.copy()
    export_obj.name=name+"_UEExport"
    scene.collection.objects.link(export_obj)
    obj.select_set(False)
    export_obj.select_set(True)
    bpy.context.view_layer.objects.active=export_obj
    for vertex in export_obj.data.vertices:
        vertex.co.x=-vertex.co.x
    bm=bmesh.new()
    bm.from_mesh(export_obj.data)
    bmesh.ops.reverse_faces(bm,faces=list(bm.faces))
    bm.normal_update()
    bm.to_mesh(export_obj.data)
    bm.free()
    export_obj.data.update()
    bpy.ops.export_scene.fbx(filepath=EXPORT_ROOT+"/"+name+".fbx",use_selection=True,
        object_types={"MESH"},global_scale=1.0,apply_unit_scale=True,
        apply_scale_options="FBX_SCALE_NONE",axis_forward="-Y",axis_up="Z",
        use_space_transform=False,bake_space_transform=False,mesh_smooth_type="FACE",
        use_tspace=False,add_leaf_bones=False,bake_anim=False,path_mode="STRIP")
    export_data=export_obj.data
    bpy.data.objects.remove(export_obj,do_unlink=True)
    bpy.data.meshes.remove(export_data)
    obj.select_set(True)
    bpy.context.view_layer.objects.active=obj
    return obj


def panel(name, rows, mat, thickness=.035, smooth=True, depth_axis=(0,0,1)):
    # Swept aerodynamic panel, with closed inner skin. Rows carry x and a
    # transverse profile: explicit curves keep every silhouette intentional.
    vertices, faces = [], []
    count=len(rows[0])
    for depth in (0,-thickness):
        vertices.extend((x+depth*depth_axis[0],y+depth*depth_axis[1],z+depth*depth_axis[2]) for row in rows for x,y,z in row)
    layer=len(rows)*count
    for side in range(2):
        offset=side*layer
        for i in range(len(rows)-1):
            for j in range(count-1):
                face=(offset+i*count+j,offset+i*count+j+1,offset+(i+1)*count+j+1,offset+(i+1)*count+j)
                faces.append(face if side==0 else tuple(reversed(face)))
    rim=list(range(count))+[i*count+count-1 for i in range(1,len(rows))]
    rim+=list(range(layer-2,layer-count-1,-1))+[i*count for i in range(len(rows)-2,0,-1)]
    for i,j in zip(rim,rim[1:]+rim[:1]):
        faces.append((i,j,j+layer,i+layer))
    mesh=bpy.data.meshes.new(name)
    mesh.from_pydata(vertices,[],faces)
    mesh.update()
    obj=bpy.data.objects.new(name,mesh)
    scene.collection.objects.link(obj)
    obj.data.materials.append(mat)
    bm=bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm,faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    for face in mesh.polygons:
        face.use_smooth=smooth
    return obj


def smooth_sections(keys, steps=5):
    result=[]
    for i in range(len(keys)-1):
        a=keys[max(0,i-1)]
        b=keys[i]
        c=keys[i+1]
        d=keys[min(len(keys)-1,i+2)]
        for j in range(steps):
            t=j/steps
            values=[b[0]+(c[0]-b[0])*t]
            for k in range(1,len(b)):
                value=.5*((2*b[k])+(-a[k]+c[k])*t+(2*a[k]-5*b[k]+4*c[k]-d[k])*t*t+(-a[k]+3*b[k]-3*c[k]+d[k])*t*t*t)
                values.append(max(min(b[k],c[k]),min(max(b[k],c[k]),value)))
            result.append(values)
    result.append(keys[-1])
    return result


vehicles=author_train_models(box,tube,collect,{"shell":petrol,"ceramic":pearl,
    "carbon":graphite,"metal":steel,"accent":copper,"cushion":padding,"glass":glass,"light":light})
train=vehicles["lead"]
author_track_hardware(box,tube,collect,{"shell":petrol,"edge":pearl,"dark":graphite,"conductor":copper,"steel":steel})

start=set(scene.objects)
# Thin, swept canopy blades continue the car's split-nose language. The open
# central slot keeps sky visible; longitudinal module edges retain exact seams.
for side in (-1,1):
    rows=[]
    for y,z in ((1.35,.16),(1.65,.18),(2.40,.15),(3.50,.10),(4.65,.045),(5.65,-.015)):
        rows.append([(x,side*y,z-.018*(x/1.496)**2) for x in (-1.496,-1.10,0,1.10,1.496)])
    panel("Swept canopy blade",rows,pearl,.048)
    rows=[]
    for y,z in ((1.35,.13),(2.2,.09),(3.5,.03),(4.7,-.035),(5.65,-.095)):
        rows.append([(-1.48,side*y,z),(-1.40,side*y,z-.065)])
    panel("Canopy leading chine",rows,petrol,.015)
    rows=[]
    for y,z in ((1.35,.13),(2.2,.09),(3.5,.03),(4.7,-.035),(5.65,-.095)):
        rows.append([(1.40,side*y,z-.065),(1.48,side*y,z)])
    panel("Canopy trailing chine",rows,petrol,.015)
    box("Inner slot beam",(0,side*1.39,.005),(2.992,.08,.23),petrol,.017)
    box("Inner slot light",(0,side*1.438,-.090),(2.81,.016,.018),light,.006)
    for x in (-1.39,0,1.39):
        rows=[]
        for y,z in ((1.35,.09),(2.4,.035),(3.6,-.035),(4.8,-.085),(5.64,-.095)):
            rows.append([(x-.026,side*y,z),(x+.026,side*y,z-.06)])
        panel("Tapered canopy rib",rows,graphite,.015)
for x in (-1.43,1.43):
    box("Slot connector",(x,0,-.117),(.075,2.80,.126),petrol,.012)
collect("SM_StationRoofPanel",start)

start=set(scene.objects)
box("Foundation shoe",(0,0,.04),(.40,.40,.08),steel,.025)
column=loft("Swept structural blade",[(.08,.135,-.10,.10),(.60,.115,-.09,.085),
       (2.1,.095,-.058,.083),(3.7,.10,-.08,.06),(4.80,.155,-.135,.09),(5.12,.17,-.16,.10)],petrol)
column.rotation_euler.y=-math.pi*.5
box("Canopy bearing",(0,0,5.17),(.35,.35,.10),steel,.025)
for x in (-.135,.135):
    for y in (-.135,.135):
        tube("Anchor nut",(x,y,.078),(x,y,.112),.023,graphite,6)
rows=[]
for z,x in ((.28,-.084),(1,-.074),(2.3,-.061),(3.6,-.067),(4.7,-.085),(5.06,-.10)):
    rows.append([(x,-.031,z),(x-.004,.031,z)])
panel("Column copper inlay",rows,copper,.006,depth_axis=(1,0,0))
collect("SM_StationPost",start)


station_assets=author_station_models(box,tube,collect,{"ceramic":pearl,"shell":petrol,
    "dark":graphite,"metal":steel,"stone":concrete,"glass":glass,"light":light})

# Save source geometry at canonical pivots before moving copies for the review.
bpy.ops.object.select_all(action="DESELECT")
for obj in asset_objects:
    obj.select_set(True)
for obj in asset_objects:
    obj.hide_set(obj.name != "SM_StationRouteRoof")
bpy.ops.wm.save_as_mainfile(filepath=EXPORT_ROOT+"/VibeCoaster-V3.blend",compress=True)
print("ASSET_MANIFEST="+json.dumps({"createdThrough":"Blender MCP execute_blender_code",
    "blender":bpy.app.version_string,"units":"metres","assets":assets}))
