"""Build and verify the project's small, reproducible runtime assets."""
import unreal

MAP = '/Game/Maps/Ride'
MATERIAL = '/Game/Materials/VertexSurface'

if not unreal.EditorAssetLibrary.does_asset_exist(MAP):
    unreal.EditorLevelLibrary.new_level(MAP)
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError('Could not save the runtime map')

material = unreal.load_asset(MATERIAL)
if material is None:
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        'VertexSurface', '/Game/Materials', unreal.Material, unreal.MaterialFactoryNew())
if not isinstance(material, unreal.Material):
    raise RuntimeError('VertexSurface must be a Material asset')

library = unreal.MaterialEditingLibrary
expressions = library.get_material_expressions(material)
vertex = next((node for node in expressions
               if isinstance(node, unreal.MaterialExpressionVertexColor)), None)
if vertex is None:
    vertex = library.create_material_expression(
        material, unreal.MaterialExpressionVertexColor, -200, 0)
rough = library.get_material_property_input_node(material, unreal.MaterialProperty.MP_ROUGHNESS)
if not isinstance(rough, unreal.MaterialExpressionConstant):
    rough = library.create_material_expression(
        material, unreal.MaterialExpressionConstant, -200, 200)
rough.set_editor_property('r', 0.72)

# UE's first Vertex Color output is unnamed. "RGB" returns false and leaves
# Base Color disconnected; never ignore a material-connection result.
if not library.connect_material_property(vertex, '', unreal.MaterialProperty.MP_BASE_COLOR):
    raise RuntimeError('Could not connect vertex colour to Base Color')
if not library.connect_material_property(rough, '', unreal.MaterialProperty.MP_ROUGHNESS):
    raise RuntimeError('Could not connect surface roughness')
if library.get_material_property_input_node(material, unreal.MaterialProperty.MP_BASE_COLOR) != vertex:
    raise RuntimeError('Base Color does not use the expected vertex-colour node')
material.set_editor_property('two_sided', True)
library.recompile_material(material)
if not unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False):
    raise RuntimeError('Could not save the verified material')
unreal.log('VibeContent PASS: vertex colour connected, roughness 0.72, material saved')
