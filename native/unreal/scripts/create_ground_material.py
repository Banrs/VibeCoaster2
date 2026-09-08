"""Create a separate world-space ground material; preserve all existing assets.

Lengths in the shader are SI metres, converted from absolute UE centimetres.
Broad 90 m patches, 12 m soil variation and 1.8 m detail provide fixed visual
scale. Pixel-footprint filtering fades unresolved detail toward its mean.
This shader changes colour only: no displaced terrain or altered clearance.
"""
from pathlib import Path
import hashlib
import shutil
from datetime import datetime, timezone
import unreal

CODE = r"""
float2 metres = WorldPosition.xy * 0.01;
float3 samples = float3(0.5,0.5,0.5);
[unroll] for (int octave=0; octave<3; ++octave) {
    float wavelength = octave==0 ? 90.0 : (octave==1 ? 12.0 : 1.8);
    float2 q = metres / wavelength;
    float2 cell = floor(q);
    float2 f = frac(q); f=f*f*(3.0-2.0*f);
    float4 h=float4(dot(cell,float2(127.1,311.7)),
      dot(cell+float2(1,0),float2(127.1,311.7)),
      dot(cell+float2(0,1),float2(127.1,311.7)),
      dot(cell+float2(1,1),float2(127.1,311.7)));
    h=frac(sin(h)*43758.5453);
    float value=lerp(lerp(h.x,h.y,f.x),lerp(h.z,h.w,f.x),f.y);
    float footprint=max(length(ddx(q)),length(ddy(q)));
    samples[octave]=lerp(value,0.5,smoothstep(0.2,0.7,footprint));
}
float soil=smoothstep(0.40,0.69,samples.x*0.75+samples.y*0.25);
float3 colour=lerp(float3(0.035,0.052,0.023),float3(0.073,0.069,0.035),soil);
colour*=0.78+0.30*samples.y+0.22*samples.z;
float rock=smoothstep(0.06,0.24,1.0-saturate(abs(SurfaceNormal.z)));
return lerp(colour,float3(0.18,0.14,0.10)*(0.8+0.4*samples.y),rock);
"""
NAME="M_Ground_Organic"
ASSET="/Game/Materials/"+NAME
SIGNATURE=hashlib.sha256(CODE.encode()).hexdigest()

def require(value,message):
    if not value: raise RuntimeError(message)

def main():
    lib=unreal.EditorAssetLibrary
    edit=unreal.MaterialEditingLibrary
    if lib.does_asset_exist(ASSET):
        material=lib.load_asset(ASSET)
        require(isinstance(material,unreal.Material),"Ground asset has unexpected type")
        node=edit.get_material_property_input_node(material,unreal.MaterialProperty.MP_BASE_COLOR)
        require(isinstance(node,unreal.MaterialExpressionCustom), "Unexpected organic ground graph")
        existing=node.get_editor_property("code")
        if existing != CODE:
            require(hashlib.sha256(existing.encode()).hexdigest()=="082d0c35e6a8469e4a70d08d807f1107b587ceb6b4768a08c83b865944eb59bd",
                    "Existing organic ground differs from known first pass; preserving user edits")
            backup=Path(unreal.Paths.project_saved_dir())/"Presentation"/(datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S-%f")+"-ground-soften")
            backup.mkdir(parents=True,exist_ok=False)
            source=Path(unreal.Paths.project_content_dir())/"Materials"/(NAME+".uasset")
            shutil.copy2(source,backup/source.name)
            require(hashlib.sha256(source.read_bytes()).digest()==hashlib.sha256((backup/source.name).read_bytes()).digest(),"Ground backup mismatch")
            node.set_editor_property("code",CODE)
            errors=edit.recompile_material(material)
            require(not errors,"Ground shader compiler errors: "+str(errors))
            require(lib.save_loaded_asset(material,only_if_is_dirty=False),"Ground correction save failed")
        require(edit.get_material_property_input_node(material,unreal.MaterialProperty.MP_WORLD_POSITION_OFFSET) is None,
                "Ground shader must not displace validated terrain")
        errors=edit.recompile_material(material)
        require(not errors,"Ground shader compiler errors: "+str(errors))
        unreal.log("VIBECOASTER_GROUND_MATERIAL_PRESENT "+SIGNATURE)
        return
    material=unreal.AssetToolsHelpers.get_asset_tools().create_asset(NAME,"/Game/Materials",unreal.Material,unreal.MaterialFactoryNew())
    require(material,"Could not create organic ground")
    material.set_editor_property("two_sided",True)
    custom=edit.create_material_expression(material,unreal.MaterialExpressionCustom,-200,0)
    custom.set_editor_property("code",CODE)
    custom.set_editor_property("description","Fixed SI ground detail, pixel-footprint filtered")
    custom.set_editor_property("output_type",unreal.CustomMaterialOutputType.CMOT_FLOAT3)
    inputs=[]
    for name in ("WorldPosition","SurfaceNormal"):
        item=unreal.CustomInput();item.set_editor_property("input_name",name);inputs.append(item)
    custom.set_editor_property("inputs",inputs)
    position=edit.create_material_expression(material,unreal.MaterialExpressionWorldPosition,-650,0)
    normal=edit.create_material_expression(material,unreal.MaterialExpressionPixelNormalWS,-650,200)
    require(edit.connect_material_expressions(position,"",custom,"WorldPosition"),"Position connection failed")
    require(edit.connect_material_expressions(normal,"",custom,"SurfaceNormal"),"Normal connection failed")
    require(edit.connect_material_property(custom,"",unreal.MaterialProperty.MP_BASE_COLOR),"Base colour connection failed")
    rough=edit.create_material_expression(material,unreal.MaterialExpressionConstant,-200,300)
    rough.set_editor_property("r",0.94)
    require(edit.connect_material_property(rough,"",unreal.MaterialProperty.MP_ROUGHNESS),"Roughness connection failed")
    errors=edit.recompile_material(material)
    require(not errors,"Ground shader compiler errors: "+str(errors))
    require(lib.save_loaded_asset(material,only_if_is_dirty=False),"Organic ground save failed")
    unreal.log("VIBECOASTER_GROUND_MATERIAL_CREATED "+SIGNATURE)

if __name__=="__main__": main()
