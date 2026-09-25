# Blender source and Unreal imports

The source kit is authored through the installed Blender MCP's
`execute_blender_code` tool. `run_blender_mcp.py` is a standard MCP stdio client;
it retains safe mode and disables telemetry. It does not bypass the MCP with a
raw socket or execute the model through a background Blender process.

`author_models.py` composes the asset modules, exports triangulated FBX files,
writes a source `.blend`, and prints measured bounds/material slots into
`exports/manifest.json`. The source and runtime pivots are deliberate: X forward,
Y right, Z up, with metres in Blender and centimetres after the Unreal import.
The importer verifies those units and measured bounds before saving meshes.

With Blender's local MCP add-on already running, use the Python environment
containing the MCP client package:

```powershell
python native/art/run_blender_mcp.py --server <installed-blender-mcp-executable>
```

For the optional station preview, also pass --script render_station.py and
--station-context <canonical-station-boxes.json>. The default command authors
the V3 asset kit.

The modelling script replaces its authoring scene. Use a dedicated Blender
instance/profile for the ride kit. Source assets are saved before review scripts
move copies or set up lights. Preview scripts render the real meshes; they do
not substitute concept images for model geometry.

Run `native/unreal/scripts/import_v3_art.py` through Unreal editor Python to
create the `/Game/Art/V3` meshes and materials. Its import receipt is
`native/unreal/Saved/V3ArtImport.json`. Terrain palette preparation belongs to
`native/unreal/scripts/create_content.py` and runs separately. No external
textures or paid asset generation services are required.

The train kit includes distinct lead and passenger cars, with seven two-seat
rows in the current default ride. The coupler concept remains marked
`runtime: false` and is not imported.

Train reference photographs inspected during authoring were the official
[Intamin Falcon's Flight train details](https://www.intamin.com/project/falcons-flight/),
[Intamin Formula Rossa photographs](https://www.intamin.com/project/formula-rosso/),
and [Ferrari World's Formula Rossa lead-car photograph](https://www.ferrariworldabudhabi.com/en/rides/formula-rossa).
Those photographs are references only; the kit contains original geometry and
no copied branding or image assets.
