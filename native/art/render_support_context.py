"""Review exact support members together with the unchanged accepted track."""
import bpy
import bmesh
import math
import json
from mathutils import Vector

EXPORT_ROOT="__EXPORT_ROOT__"
CONTEXTS=json.loads(__SUPPORT_CONTEXT__)
scene=bpy.context.scene
for obj in scene.objects:
    obj.hide_render=True
scene.render.engine="BLENDER_EEVEE"
scene.render.resolution_x=1200
scene.render.resolution_y=850
scene.render.resolution_percentage=100
scene.render.image_settings.file_format="PNG"
scene.world.use_nodes=True
scene.world.node_tree.nodes["Background"].inputs[0].default_value=(.28,.34,.40,1)
scene.world.node_tree.nodes["Background"].inputs[1].default_value=.7
scene.view_settings.view_transform="AgX"

steel=bpy.data.materials.get("VC2_Structure") or bpy.data.materials.new("VC2_Structure")
steel.use_nodes=True
steel.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value=(.31,.36,.35,1)
steel.node_tree.nodes["Principled BSDF"].inputs["Metallic"].default_value=.25
steel.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value=.66
concrete=bpy.data.materials["VC2_Concrete"]
rail_material=bpy.data.materials["VC2_Petrol"]
soil=bpy.data.materials.get("Support review terrain") or bpy.data.materials.new("Support review terrain")
soil.use_nodes=True
soil.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value=(.26,.19,.11,1)
soil.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value=.95

def mesh(name,vertices,faces,material):
    data=bpy.data.meshes.new(name)
    data.from_pydata(vertices,[],faces)
    data.update()
    obj=bpy.data.objects.new(name,data)
    scene.collection.objects.link(obj)
    data.materials.append(material)
    bm=bmesh.new()
    bm.from_mesh(data)
    bmesh.ops.recalc_face_normals(bm,faces=bm.faces)
    bm.to_mesh(data)
    bm.free()
    for face in data.polygons:
        face.use_smooth=len(face.vertices)==4
    return obj

def member(member,origin):
    a,b=Vector(member["base"])-origin,Vector(member["top"])-origin
    axis=(b-a).normalized()
    u=axis.cross(Vector((0,0,1)) if abs(axis.z)<.9 else Vector((0,1,0))).normalized()
    v=axis.cross(u)
    vertices=[]
    for centre,radius in ((a,member["radiusBase"]),(b,member["radiusTop"])):
        vertices.extend(tuple(centre+(u*math.cos(j*math.tau/16)+v*math.sin(j*math.tau/16))*radius) for j in range(16))
    faces=[tuple(reversed(range(16))),tuple(range(16,32))]
    faces += [(j,(j+1)%16,(j+1)%16+16,j+16) for j in range(16)]
    return mesh("Canonical support member",vertices,faces,concrete if member["kind"]=="Footing" else steel)

def rail(points,origin,side,height,radius):
    vertices=[]
    for point in points:
        up,right=Vector(point["up"]),Vector(point["right"])
        centre=Vector(point["position"])-origin+right*side+up*height
        vertices.extend(tuple(centre+(right*math.cos(j*math.tau/12)+up*math.sin(j*math.tau/12))*radius) for j in range(12))
    faces=[]
    for i in range(len(points)-1):
        for j in range(12):
            faces.append((i*12+j,i*12+(j+1)%12,(i+1)*12+(j+1)%12,(i+1)*12+j))
    return mesh("Actual accepted track",vertices,faces,rail_material)

bpy.ops.object.light_add(type="SUN",location=(-40,-70,80))
sun=bpy.context.object
sun.data.energy=2.8
sun.data.angle=math.radians(8)
sun.rotation_euler=(math.radians(30),math.radians(-25),math.radians(-40))
bpy.ops.object.camera_add(location=(40,-60,40))
camera=bpy.context.object
camera.data.type="ORTHO"
scene.camera=camera

for context in CONTEXTS:
    origin=Vector(context["origin"])
    made=[]
    coordinates=[]
    for support in context["supports"]:
        for item in support["members"]:
            made.append(member(item,origin))
            coordinates += [Vector(item["base"])-origin,Vector(item["top"])-origin]
    for side,height,radius in ((-.65,0,.085),(.65,0,.085),(0,-.55,.16)):
        made.append(rail(context["track"],origin,side,height,radius))
    coordinates += [Vector(p["position"])-origin for p in context["track"]]
    low=Vector(tuple(min(p[i] for p in coordinates) for i in range(3)))
    high=Vector(tuple(max(p[i] for p in coordinates) for i in range(3)))
    centre=(low+high)*.5
    size=high-low
    span=max(size.x,size.y,size.z,20)
    along=Vector(context["track"][-1]["position"])-Vector(context["track"][0]["position"])
    along.z=0
    along.normalize()
    across=along.cross(Vector((0,0,1))).normalized()
    camera.location=centre+(across*.95-along*.65+Vector((0,0,.43)))*span
    camera.rotation_euler=(centre-camera.location).to_track_quat("-Z","Y").to_euler()
    inverse=camera.rotation_euler.to_quaternion().inverted()
    projected=[inverse @ (point-centre) for point in coordinates]
    extent_x=max(p.x for p in projected)-min(p.x for p in projected)
    extent_y=max(p.y for p in projected)-min(p.y for p in projected)
    camera.data.ortho_scale=max(extent_x,extent_y*scene.render.resolution_x/scene.render.resolution_y)*1.16
    terrain=context["terrain"]
    vertices=[tuple(Vector(p)-origin) for p in terrain["vertices"]]
    made.append(mesh("Exact accepted terrain",vertices,terrain["triangles"],soil))
    scene.render.filepath=EXPORT_ROOT+"/support-context-"+context["name"]+".png"
    bpy.ops.render.render(write_still=True)
    for obj in made:
        obj.hide_render=True
print("Rendered exact accepted track, support members and canonical terrain triangles.")
