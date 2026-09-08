"""Export prepared UE-coordinate copies only. Run after separate activation call."""
import bpy
import json
OWNER="VC_UE_EXPORT_v001"
scene=bpy.context.scene
assert OWNER+"_manifest" in scene, "Run preparation and separate scene activation first"
manifest=json.loads(scene[OWNER+"_manifest"])
layer=bpy.context.view_layer
assert len(manifest["assets"])==7
for item in manifest["assets"]:
    obj=bpy.data.objects[item["convertedObject"]]
    assert obj.name in layer.objects
    assert obj.parent is None and tuple(obj.location)==(0,0,0) and tuple(obj.scale)==(1,1,1)
    for other in list(layer.objects): other.select_set(False)
    obj.select_set(True)
    layer.objects.active=obj
    print("VC_UNREAL_EXPORT_BEGIN="+item["name"])
    result=bpy.ops.export_scene.fbx(filepath=item["fbx"],use_selection=True,object_types={"MESH"},
        use_mesh_modifiers=False,apply_unit_scale=True,apply_scale_options="FBX_SCALE_UNITS",
        axis_forward="X",axis_up="Z",global_scale=1.0,add_leaf_bones=False,bake_anim=False,
        use_custom_props=True,path_mode="AUTO")
    assert "FINISHED" in result, "FBX export did not finish: "+item["name"]
    item["status"]="exported-pending-independent-UE-witness"
    scene[OWNER+"_manifest"]=json.dumps(manifest)
    print("VC_UNREAL_EXPORT_END="+item["name"])
print("VC_UNREAL_EXPORT_MANIFEST="+json.dumps(manifest))
