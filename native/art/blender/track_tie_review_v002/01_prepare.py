import bpy
import bmesh
import math
from mathutils import Matrix, Vector
P="VCWebReview2_"
assert P+"Scene" not in bpy.data.scenes,"Preserve existing review"
scene=bpy.data.scenes.new(P+"Scene")
scene.render.engine="BLENDER_EEVEE"
scene.render.resolution_x=1280
scene.render.resolution_y=720
scene.render.resolution_percentage=100
scene.render.image_settings.file_format="PNG"
scene.render.film_transparent=False
scene.render.filepath="D:/Coding/Codex/Vibecoasterjs/native/art/review/20260908-tie-v002/track-web-oblique.png"
scene.world=bpy.data.worlds.new(P+"World")
scene.world.use_nodes=True
scene.world.node_tree.nodes.get("Background").inputs[0].default_value=(.075,.085,.1,1)
scene.world.node_tree.nodes.get("Background").inputs[1].default_value=.6
scene.view_settings.view_transform="AgX"
collection=bpy.data.collections.new(P+"Assembly")
scene.collection.children.link(collection)
source=bpy.data.objects["VCTie2_SM_TrackTieWeb"]
for index,x in enumerate((-3,0,3)):
    obj=bpy.data.objects.new(P+"Tie"+str(index),source.data)
    collection.objects.link(obj)
    obj.location=(x,0,0)
# Exactly the renderer's circular rails and spine, expressed at the tie pivot.
for name,y,z,radius in (("RailLeft",-.65,.19,.085),("RailRight",.65,.19,.085),("Spine",0,-.36,.16)):
    mesh=bpy.data.meshes.new(P+name+"Mesh")
    bm=bmesh.new()
    bmesh.ops.create_cone(bm,cap_ends=True,cap_tris=False,segments=32,radius1=radius,radius2=radius,depth=8)
    bm.transform(Matrix.Translation((0,y,z))@Matrix.Rotation(math.pi/2,4,"Y"))
    bm.to_mesh(mesh);bm.free();mesh.update()
    mesh.materials.append(bpy.data.materials["VC_ENVKIT_BlueSteel"])
    for face in mesh.polygons:face.use_smooth=len(face.vertices)==4
    obj=bpy.data.objects.new(P+name,mesh);collection.objects.link(obj)
for index,location in enumerate(((1,-3,-3),(-3,1,3),(4,4,-1))):
    data=bpy.data.lights.new(P+"Light"+str(index),"AREA");data.energy=(1000,1600,1100)[index];data.shape="DISK";data.size=5
    obj=bpy.data.objects.new(P+"Light"+str(index),data);collection.objects.link(obj);obj.location=location
    obj.rotation_euler=(Vector((0,0,-.1))-obj.location).to_track_quat("-Z","Y").to_euler()
data=bpy.data.cameras.new(P+"Camera");data.lens=48;data.clip_start=.05;data.clip_end=100
camera=bpy.data.objects.new(P+"Camera",data);collection.objects.link(camera);camera.location=(7.5,-6,-2.6)
camera.rotation_euler=(Vector((0,0,-.1))-camera.location).to_track_quat("-Z","Y").to_euler()
scene.camera=camera
print("VC_WEB_REVIEW_PREPARED: 3 ties at 3m spacing, exact .17m rails and .32m spine; local tie pivot")
