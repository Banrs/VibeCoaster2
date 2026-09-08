import bpy
import bmesh
import json
from mathutils import Vector
PREFIX="VCTie2_"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/track/20260908-tie-v002"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/track/20260908-tie-v002"
CORE_SPEC={'status': 'isolated-prototype', 'axes': 'X forward,Y rider-right,Z up; SI metres', 'pivot': 'rail midpoint', 'tiePivotZ': -0.19, 'tieSpacing': 3, 'hardwareMotionPad': 0.06, 'maximumCornerRadius': 0.7058716653772067, 'webs': [{'center': [0, -0.37, -0.31], 'forward': [1, 0, 0], 'right': [0, -0.9191450300180579, 0.39391929857916763], 'up': [0, -0.39391929857916763, -0.9191450300180579], 'half': [0.06, 0.30463092423455634, 0.04], 'corners': [[-0.06, -0.07424322805683325, -0.3932341987992777], [-0.06, -0.10575677194316668, -0.4667658012007223], [-0.06, -0.6342432280568333, -0.1532341987992777], [-0.06, -0.6657567719431667, -0.22676580120072232], [0.06, -0.07424322805683325, -0.3932341987992777], [0.06, -0.10575677194316668, -0.4667658012007223], [0.06, -0.6342432280568333, -0.1532341987992777], [0.06, -0.6657567719431667, -0.22676580120072232]]}, {'center': [0, 0.37, -0.31], 'forward': [1, 0, 0], 'right': [0, 0.9191450300180579, 0.39391929857916763], 'up': [0, -0.39391929857916763, 0.9191450300180579], 'half': [0.06, 0.30463092423455634, 0.04], 'corners': [[-0.06, 0.10575677194316668, -0.4667658012007223], [-0.06, 0.07424322805683325, -0.3932341987992777], [-0.06, 0.6657567719431667, -0.22676580120072232], [-0.06, 0.6342432280568333, -0.1532341987992777], [0.06, 0.10575677194316668, -0.4667658012007223], [0.06, 0.07424322805683325, -0.3932341987992777], [0.06, 0.6657567719431667, -0.22676580120072232], [0.06, 0.6342432280568333, -0.1532341987992777]]}]}
CORE_SPEC_SHA256='bfe14e36bdac6ffc152a4138936ceb4a73df0050318ebd9dc53a91a39ac9fbf1'

assert PREFIX+"Scene" not in bpy.data.scenes, "Preserve existing v002 art run"
scene=bpy.data.scenes.new(PREFIX+"Scene")
scene.unit_settings.system="METRIC"
scene.unit_settings.scale_length=1
parts=bpy.data.collections.new(PREFIX+"Parts")
scene.collection.children.link(parts)
exports=bpy.data.collections.new(PREFIX+"Exports")
scene.collection.children.link(exports)
source=bpy.data.collections["VC_ENVKIT_SM_TrackTie"]
source_root=bpy.data.objects["VC_ENVKIT_SM_TrackTie_ROOT"]
originals=[o for o in source.objects if o.type=="MESH"]
assert originals, "Existing baked tie source required"
for old in originals:
    assert len(old.modifiers)==0
    mesh=old.data.copy()
    mesh.name=PREFIX+old.name+"Mesh"
    matrix=source_root.matrix_world.inverted()@old.matrix_world
    mesh.transform(matrix)
    mesh.update()
    obj=bpy.data.objects.new(PREFIX+old.name,mesh)
    parts.objects.link(obj)
    obj["kind"]="original_tie"
    for vertex in mesh.vertices:
        assert abs(vertex.co.x)<=.07+1e-6 and abs(vertex.co.y)<=.825+1e-6 and abs(vertex.co.z)<=.08+1e-6

certificates=[]
# Compiled specification uses core Y rider-right. Reflect it into Blender -Y
# rider-right, then add +.19 Z to move from rail datum into the existing tie pivot.
def direction(p): return Vector((p[0],-p[1],p[2]))
def position(p): return direction(p)+Vector((0,0,.19))
for index,solid in enumerate(CORE_SPEC["webs"]):
    verts=[position(p) for p in solid["corners"]]
    # Compiled corner ordering is x outer, y middle, z inner.
    faces=[(0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3)]
    mesh=bpy.data.meshes.new(PREFIX+"Web"+str(index)+"Mesh")
    mesh.from_pydata(verts,[],faces)
    bm=bmesh.new();bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bmesh.ops.bevel(bm,geom=list(bm.edges),offset=.003,segments=2,affect="EDGES")
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bm.to_mesh(mesh);bm.free();mesh.update()
    mesh.materials.append(bpy.data.materials["VC_ENVKIT_BlueSteel"])
    obj=bpy.data.objects.new(PREFIX+"DiagonalWeb"+str(index),mesh)
    parts.objects.link(obj)
    obj["kind"]="certified_obb_web"
    obj["core_index"]=index
    centre=position(solid["center"])
    axes=[direction(solid[k]) for k in ("forward","right","up")]
    half=solid["half"]
    max_extent=[0,0,0]
    for vertex in mesh.vertices:
        relative=vertex.co-centre
        for axis in range(3):
            extent=abs(relative.dot(axes[axis]))
            max_extent[axis]=max(max_extent[axis],extent)
            assert extent<=half[axis]+1e-6, "Web escapes its own exact OBB: "+str(index)
    cert={"coreIndex":index,"part":obj.name,"vertexCount":len(mesh.vertices),
          "coreOBB":solid,"blenderTieLocalOBB":{"center":list(centre),"axes":[list(a) for a in axes],"half":half},
          "measuredHalfExtentsM":max_extent,"maximumOutsideM":max(0,max(a-b for a,b in zip(max_extent,half))),
          "capExtensionM":max(0,max_extent[1]-half[1]),"chamferM":.003,"checkedBeforeAssembly":True}
    certificates.append(cert)
# Independent artist certificate exists before any combined mesh is generated.
manifest={"schema":"vibecoaster-track-tie-art-2","build":"20260908-tie-v002",
 "sourceBlend":SOURCE+"/TrackTieWeb_v002.blend","fbx":EXPORT+"/SM_TrackTieWeb.fbx",
 "assetName":"SM_TrackTieWeb","sourceAxes":"+X forward,-Y rider-right,+Z up; metres",
 "pivot":"existing tie midpoint; rail datum is local Z +.19 m","railToTieLocalZ":.19,
 "originalTieEnvelopeM":[[-.07,-.825,-.08],[.07,.825,.08]],
 "coreSpecPath":"native/unreal/Saved/TrackWeb/20260908-1/track-web-spec.json",
 "coreSpecSha256":CORE_SPEC_SHA256,"webCertificates":certificates,
 "sourcePartCount":len(parts.objects),"runtimeIntegrated":False,
 "status":"local-part-containment-checked; independent full-circuit hardware proof still required"}
scene[PREFIX+"manifest"]=json.dumps(manifest)
print("VC_TIE2_PART_CERTIFICATES="+json.dumps(manifest))
