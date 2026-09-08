import unreal,json
from pathlib import Path
s=unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
out={}
for path in ("/Game/Art/V071/REVIEW_CoordinateWitness","/Game/Art/V071/Import2/REVIEW_CoordinateWitness"):
 m=unreal.load_asset(path);s.set_allow_cpu_access(m,True);slots=m.get_editor_property("static_materials");parts=[]
 for i in range(m.get_num_sections(0)):
  vs,ts,ns,us,ta=unreal.ProceduralMeshLibrary.get_section_from_static_mesh(m,0,i);pts=[[v.x,v.y,v.z] for v in vs];idx=s.get_lod_material_slot(m,0,i)
  parts.append({"slot":str(slots[idx].get_editor_property("imported_material_slot_name")),"bounds":[[min(v[k] for v in pts) for k in range(3)],[max(v[k] for v in pts) for k in range(3)]]})
 out[path]=parts
Path(r"D:/Coding/Codex/Vibecoasterjs/native/art/review/20260907-3/witness-sections.json").write_text(json.dumps(out,indent=2))
