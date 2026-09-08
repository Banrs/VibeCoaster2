import bpy
import json
from mathutils import Matrix
PREFIX="VCTrain4_"
SCENE="VCTrain4_AssetReview"
SOURCE="D:/Coding/Codex/Vibecoasterjs/native/art/source/train/20260908-train-v004"
EXPORT="D:/Coding/Codex/Vibecoasterjs/native/art/exports/train/20260908-train-v004"
scene=bpy.data.scenes[SCENE]
destination=bpy.data.collections[PREFIX+"AssembledExport"]
parts=[m for m in bpy.data.meshes if m.name.startswith(PREFIX+"V004Snapshot_")]
assert len(parts)>=60 and all(m["runtime_part"] for m in parts), "Only v004 runtime parts may be assembled"

def world(mesh):
    f=list(mesh["world_matrix"])
    return Matrix((f[0:4],f[4:8],f[8:12],f[12:16]))

def bounds(meshes):
    vs=[world(m)@v.co for m in meshes for v in m.vertices]
    return [[min(v[i] for v in vs) for i in range(3)], [max(v[i] for v in vs) for i in range(3)]]

def assemble(name,meshes):
    assert name not in bpy.data.objects, "Preserve prior assembly rather than overwrite"
    vs=[];fs=[];polymats=[];mats=[]
    for mesh in meshes:
        start=len(vs)
        matrix=world(mesh)
        vs.extend(matrix@v.co for v in mesh.vertices)
        indices=[]
        for mat in mesh.materials:
            if mat not in mats: mats.append(mat)
            indices.append(mats.index(mat))
        for face in mesh.polygons:
            fs.append(tuple(start+i for i in face.vertices))
            polymats.append((indices[face.material_index],face.use_smooth))
    mesh=bpy.data.meshes.new(name+"Mesh")
    mesh.from_pydata(vs,[],fs)
    mesh.update()
    for mat in mats: mesh.materials.append(mat)
    for face,(index,smooth) in zip(mesh.polygons,polymats):
        face.material_index=index
        face.use_smooth=smooth
    obj=bpy.data.objects.new(name,mesh)
    destination.objects.link(obj)
    obj["units"]="metres"
    obj["visual_only"]=True
    return obj

runtime=[m for m in parts if m["runtime_part"]]
review=[m for m in parts if not m["runtime_part"]]
car_bounds=bounds(runtime)
lo,hi=car_bounds
assert lo[0]>=-1.275-1e-6 and hi[0]<=1.275+1e-6, car_bounds
assert lo[1]>=-.85-1e-6 and hi[1]<=.85+1e-6, car_bounds
assert lo[2]>=0 and hi[2]<=2.4, car_bounds
assert abs(hi[0]-lo[0]-2.55)<1e-6 and abs(hi[1]-lo[1]-1.70)<1e-6
car=assemble(PREFIX+"SM_TrainCar_Assembled",runtime)
assert len(review)==0, "No unvalidated moving hardware may enter v004"
car["clearance_status"]="Source above-rail bounds asserted; imported UE and POV review pending"
car["optics_status"]="WindDeflector_ClearCyan requires explicit UE optical material review"
manifest={"schema":"vibecoaster-train-art-1","build":"20260908-train-v004","blender":bpy.app.version_string,
 "source_blend":SOURCE+"/VibeCoaster_HighSpeedTrain.blend","exports_directory":EXPORT,
 "runtime_parts":len(runtime),"review_bogie_parts":len(review),"runtime_bounds_m":car_bounds,
 "review_bounds_m":None,"runtime_vertices":len(car.data.vertices),
 "runtime_triangles":sum(len(p.vertices)-2 for p in car.data.polygons),
 "runtime_material_slots":[m.name for m in car.data.materials],
 "source_axes":{"forward":"+X","rider_right":"-Y","up":"+Z","unit":"metre"},
 "eye_m":[0,0,1.2],"origin_m":[0,0,0],"rail_gauge_m":1.3,"car_spacing_m":3.4,
 "runtime_export":"train_car_runtime","design":"Aero Wedge 2030 original concept","review_only_export":None,"exports":{}}
scene["VCTrain4_manifest_json"]=json.dumps(manifest)
print("VC_TRAIN_STAGE_05_ASSEMBLED="+json.dumps(manifest))
