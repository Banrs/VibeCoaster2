"""Render the actual canonical member fixtures supplied by the support author."""
import bpy
import math
from mathutils import Vector

EXPORT_ROOT = "__EXPORT_ROOT__"
SUPPORT_GEOMETRY = __SUPPORT_GEOMETRY__
scene=bpy.context.scene
for obj in scene.objects:
    obj.hide_render=True
scene.render.engine="BLENDER_EEVEE"
scene.render.resolution_x=700
scene.render.resolution_y=900
scene.render.resolution_percentage=100
scene.render.image_settings.file_format="PNG"
scene.world.use_nodes=True
scene.world.node_tree.nodes["Background"].inputs[0].default_value=(.24,.29,.34,1)
scene.world.node_tree.nodes["Background"].inputs[1].default_value=.65
scene.view_settings.view_transform="AgX"

mat=bpy.data.materials.get("VC2_Structure") or bpy.data.materials.new("VC2_Structure")
mat.use_nodes=True
mat.diffuse_color=(.25,.28,.26,1)
mat.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value=(.25,.28,.26,1)
mat.node_tree.nodes["Principled BSDF"].inputs["Metallic"].default_value=.25
mat.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value=.66
footing=bpy.data.materials["VC2_Concrete"]

bpy.ops.mesh.primitive_plane_add(size=1600,location=(0,0,-.005))
ground=bpy.context.object
ground.data.materials.append(footing)
bpy.ops.object.light_add(type="SUN",location=(10,-20,30))
sun=bpy.context.object
sun.data.energy=3
sun.rotation_euler=(math.radians(29),math.radians(-22),math.radians(-35))
bpy.ops.object.camera_add(location=(30,-45,20))
camera=bpy.context.object
camera.data.type="ORTHO"
scene.camera=camera

def member_mesh(member):
    a=Vector(member["base"])-Vector((200,0,0))
    b=Vector(member["top"])-Vector((200,0,0))
    bpy.ops.mesh.primitive_cone_add(vertices=8,radius1=member["radiusBase"],radius2=member["radiusTop"],
                                   depth=(b-a).length,location=(a+b)*.5)
    obj=bpy.context.object
    obj.rotation_euler=(b-a).to_track_quat("Z","Y").to_euler()
    obj.data.materials.append(footing if member["kind"]=="Footing" else mat)
    for face in obj.data.polygons:
        face.use_smooth=len(face.vertices)==4
    return obj

for fixture in SUPPORT_GEOMETRY:
    created=[]
    for member in fixture["members"]:
        created.append(member_mesh(member))
    height=fixture["height"]
    span=min(height*.55,42)
    for x,z,r in ((-.65,height,.085),(.65,height,.085),(0,height-.55,.16)):
        bpy.ops.mesh.primitive_cylinder_add(vertices=12,radius=r,depth=span,location=(x,0,z))
        obj=bpy.context.object
        obj.rotation_euler.x=math.pi*.5
        obj.data.materials.append(bpy.data.materials["VC2_Petrol"])
        created.append(obj)
    camera.location=(height*.66,-height*.85,height*.58)
    camera.rotation_euler=(Vector((0,0,height*.49))-camera.location).to_track_quat("-Z","Y").to_euler()
    camera.data.ortho_scale=height*1.18+4
    scene.render.filepath=EXPORT_ROOT+"/support-"+str(height)+"m.png"
    bpy.ops.render.render(write_still=True)
    for obj in created:
        obj.hide_render=True
print("Rendered canonical support fixtures without altering their member geometry.")
