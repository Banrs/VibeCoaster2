"""Prepare seven new UE-coordinate meshes; artist source objects are read-only.
Run this, activate_unreal_export_scene.py, then export_unreal_assets.py as three
separate guarded MCP calls. No scene switch or depsgraph evaluation occurs here.
"""
import bpy
import json
from mathutils import Matrix, Vector

OWNER="VC_UE_EXPORT_v001"
OUTPUT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/unreal/20260908-v001"
scene=bpy.data.scenes.get("VC_ENVKIT_ExportMasters")
if scene is None:
    scene=bpy.data.scenes.get("VC_ENVKIT_Export")
assert scene is not None, "Existing environment export scene is required"
assert OWNER not in bpy.data.collections, "Preserve previous prepared export; use a new revision"
destination=bpy.data.collections.new(OWNER)
scene.collection.children.link(destination)
keys=["REVIEW_CoordinateWitness","SM_TrainCar","SM_StationPlatformPanel",
      "SM_StationPlatformEndPanel_1m","SM_StationRoofPanel","SM_StationPost","SM_TrackTie"]
manifest={"schema":"vibecoaster-unreal-art-export-1","revision":"20260908-v001",
          "scene":scene.name,"conversion":{"position":"(x,-y,z)","units":"metres",
          "winding":"reverse every source face once","normal":"inverse-transpose source transform, then (nx,-ny,nz)",
          "reason":"Actual FBX-to-UE witness preserved source XYZ; explicit source Y reflection required"},
          "source_modified":False,"assets":[],"fbxOverrides":{}}

for key in keys:
    if key=="SM_TrainCar":
        original=bpy.data.objects["VCTrain_SM_TrainCar_Assembled"]
        originals=[original]
        # Assembled train is already expressed at rail origin, with identity transform.
        assert original.parent is None
        inverse_root=Matrix.Identity(4)
        source_name=original.name
    else:
        source_collection=bpy.data.collections["VC_ENVKIT_"+key]
        source_root=bpy.data.objects["VC_ENVKIT_"+key+"_ROOT"]
        originals=[obj for obj in source_collection.objects if obj.type=="MESH"]
        inverse_root=source_root.matrix_world.inverted()
        source_name=source_collection.name
    assert originals, "No source mesh parts for "+key
    vertices=[];source_positions=[];faces=[];face_data=[];materials=[];corner_normals=[]
    for obj in originals:
        assert obj.type=="MESH" and len(obj.modifiers)==0, "Only already-baked source meshes may be copied: "+obj.name
        mesh=obj.data
        transform=inverse_root@obj.matrix_world
        assert transform.to_3x3().determinant()>0, "Unexpected source negative scale: "+obj.name
        normal_transform=transform.to_3x3().inverted().transposed()
        offset=len(vertices)
        for vertex in mesh.vertices:
            p=transform@vertex.co
            source_positions.append(tuple(p))
            vertices.append((p.x,-p.y,p.z))
        slot_map=[]
        for material in mesh.materials:
            if material not in materials: materials.append(material)
            slot_map.append(materials.index(material))
        assert len(mesh.corner_normals)==len(mesh.loops), "Source corner normals unavailable"
        for face in mesh.polygons:
            # Reflection reverses handedness. Reversing the order restores
            # outward orientation while preserving each corner's material/normal.
            source_loops=list(reversed(face.loop_indices))
            faces.append(tuple(offset+mesh.loops[i].vertex_index for i in source_loops))
            face_data.append((slot_map[face.material_index],face.use_smooth))
            for index in source_loops:
                n=(normal_transform@mesh.corner_normals[index].vector).normalized()
                corner_normals.append((n.x,-n.y,n.z))
    name=OWNER+"_"+key
    assert name not in bpy.data.objects
    mesh=bpy.data.meshes.new(name+"Mesh")
    mesh.from_pydata(vertices,[],faces)
    mesh.update()
    for material in materials: mesh.materials.append(material)
    for face,(index,smooth) in zip(mesh.polygons,face_data):
        face.material_index=index
        face.use_smooth=smooth
    assert len(mesh.loops)==len(corner_normals)
    mesh.normals_split_custom_set(corner_normals)
    obj=bpy.data.objects.new(name,mesh)
    destination.objects.link(obj)
    obj["asset_owner"]=OWNER
    obj["source_asset"]=source_name
    obj["source_to_ue_reflection"]="x,-y,z; reversed winding and reflected split normals"
    obj["runtime_geometry_certificate"]="Art copy only; existing physics geometry unchanged"
    source_bounds=[[min(v[i] for v in source_positions) for i in range(3)],
                   [max(v[i] for v in source_positions) for i in range(3)]]
    converted_bounds=[[min(v[i] for v in vertices) for i in range(3)],
                      [max(v[i] for v in vertices) for i in range(3)]]
    expected=[[source_bounds[0][0],-source_bounds[1][1],source_bounds[0][2]],
              [source_bounds[1][0],-source_bounds[0][1],source_bounds[1][2]]]
    assert max(abs(converted_bounds[r][c]-expected[r][c]) for r in (0,1) for c in (0,1,2))<1e-6
    path=OUTPUT+"/"+key+".fbx"
    manifest["fbxOverrides"][key]=path
    manifest["assets"].append({"name":key,"source":source_name,"sourceParts":[o.name for o in originals],
        "convertedObject":name,"fbx":path,"sourceBoundsM":source_bounds,"convertedBoundsM":converted_bounds,
        "materialSlots":[m.name for m in materials],"vertices":len(vertices),
        "triangles":sum(len(f)-2 for f in faces),"splitNormalsCopied":len(corner_normals),
        "status":"prepared-not-exported"})
assert len(manifest["assets"])==7
scene[OWNER+"_manifest"]=json.dumps(manifest)
print("VC_UNREAL_PREPARED="+json.dumps(manifest))
