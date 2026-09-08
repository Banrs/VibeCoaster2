import bpy
import bmesh
import json
from mathutils import Vector
PREFIX="VCTie2_"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/track/20260908-tie-v002"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/track/20260908-tie-v002"

scene=bpy.data.scenes[PREFIX+"Scene"]
assert bpy.context.scene==scene
manifest=json.loads(scene[PREFIX+"manifest"])
assert len(manifest["webCertificates"])==2 and all(c["checkedBeforeAssembly"] for c in manifest["webCertificates"])
parts=bpy.data.collections[PREFIX+"Parts"]
vertices=[];faces=[];mats=[];face_meta=[]
for obj in parts.objects:
    assert obj.type=="MESH" and tuple(obj.scale)==(1,1,1)
    start=len(vertices)
    vertices.extend(tuple(v.co) for v in obj.data.vertices)
    mapping=[]
    for mat in obj.data.materials:
        if mat not in mats:mats.append(mat)
        mapping.append(mats.index(mat))
    for poly in obj.data.polygons:
        faces.append(tuple(start+i for i in poly.vertices))
        face_meta.append((mapping[poly.material_index],poly.use_smooth))
mesh=bpy.data.meshes.new(PREFIX+"AssembledMesh")
mesh.from_pydata(vertices,[],faces);mesh.update()
for mat in mats:mesh.materials.append(mat)
for poly,(index,smooth) in zip(mesh.polygons,face_meta):
    poly.material_index=index;poly.use_smooth=smooth
obj=bpy.data.objects.new(PREFIX+"SM_TrackTieWeb",mesh)
bpy.data.collections[PREFIX+"Exports"].objects.link(obj)
manifest["assembledObject"]=obj.name
manifest["boundsMinimumM"]=[min(v[a] for v in vertices) for a in range(3)]
manifest["boundsMaximumM"]=[max(v[a] for v in vertices) for a in range(3)]
manifest["vertexCount"]=len(vertices)
manifest["triangleCount"]=sum(len(p)-2 for p in faces)
manifest["materialSlots"]=[m.name for m in mats]
scene[PREFIX+"manifest"]=json.dumps(manifest)
print("VC_TIE2_ASSEMBLED="+json.dumps(manifest))
