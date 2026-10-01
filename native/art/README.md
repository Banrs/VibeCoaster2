# Art assets

The new [Riftwake train model](TRAIN_DESIGN.md) is an editable seven-car, fourteen-seat
Blender proposal with a train-only GLB export. The latest user correction puts
compact wheel clamps across the rails and removes the tall outer cages.
Earlier train designs were rejected and deleted; only [1.40 m gauge and maximum
2.40 m train width](DIMENSIONS.md) carried forward. The new model measures 2.35 m.
Development builds still display position markers; this art has not been adopted
into the runtime envelope or playable package.

The new [Exa track study](TRACK_STUDY.md) starts the replacement modeling work:
210 mm diameter hollow running rails, a heavy round spine, centred tubular crossheads,
gussets and 750 mm diameter capped columns with separate bearing webs. It has
editable Blender MCP source and an isolated
Unreal review map. The live ride has not yet adopted its proposed section.

The explicit `exa-1` native review now shares the same dimensions and complete
support fitting geometry with validation and Unreal. The selected default.3
save remains unchanged. `import_native_exa.py`, executed through Blender MCP,
imports exact portable meshes, preserving native triangle diagonals. The primary
model is `exports/support-system/Riftwake-Exa-Adapted-Review.blend`, with the complete
actual saved ride in scene `Riftwake / Exa generated ride`. Its source is
`out/support-adapted/final/regenerated-supports.vcdesign`. To reproduce it, run
`support_review OUTPUT SAVED_DESIGN --current-supports` with output
`out/support-adapted/ride/fixtures`, then execute the importer through Blender MCP
with `REVIEW_ROOT='out/support-adapted/ride'`, `CASE='ride-42'`,
`DATA_FILE='saved-layout'` and `SCENE_NAME='Riftwake / Exa generated ride'`.
`render_native_ride.py` creates cameras only: `REGION_INDEX=0..3` selects the
twisted drop, main camelback, loop and Immelmann; `'cliff'` selects the wall
brackets and `'tallest'` the high transition. Set `VIEW='ground'`, `'oblique'`,
`'joint'` or `'anchor'` (cliff only). Terrain sightlines include foundations,
and framing fits the real native bounds. Terrain and track come from the native export;
Blender adds review lighting and a backdrop outside the terrain boundary.

Secondary fixtures use `REVIEW_ROOT='out/support-2030'`,
`SCENE_PREFIX='Exa 2030 / '` and the fixture `CASE`. Tall loop/Immelmann fixtures use
`out/support-2030/tall` and `Exa 2030 Tall / `. Set
`SUPPORT_COLOR=(.52,.57,.60,1)` for the scene-local satin support finish.
Current captures, exports and verification logs are in `out/support-adapted` at repository
root. `Riftwake-Exa-2030.blend` retains the preceding design and historical review
scenes unloaded from the live session to reduce memory usage.
`Exa-All-Tall-Elements.blend` / `out/support-refinement` and the earlier
`Exa-Support-Revision.blend` / `out/exa-revision` are comparison material.
`render_native_support_review.py` supplies secondary fixture cameras;
`audit_native_support_joints.py` checks real mesh end surfaces for visible gaps.
`update_native_supports.py` can replace just the support meshes from another
complete native export, or `support_review --supports-only`. The latter includes all support fittings while retaining the verified rail meshes. It verifies the common track sample grid and requires
the existing terrain grid to contain the new one with identical heights; a
changed track or expanded terrain requires the full importer. This speeds
support iteration without substituting hand-built Blender geometry.
Both run through Blender MCP. `style_native_review.py` applies scene-local daylight materials without changing geometry. Render requests use eight threads and 16 samples.
See the
[integration record](../../docs/exa-runtime-integration.md).

The [adaptive support generator](../../docs/support-system.md) exports actual
native member graphs to `author_support_study.py` through Blender MCP. Its
camelback, loop and Immelmann review scenes use shared foundations and each
example's actual terrain surface; they are not scaled copies of a pillar asset.
`support_detail_geometry.py` adds oriented track mounts, pipe splices and seated
foundation fixings. Full member axes and ground footprints come from native
format 2 exports; proposed terminal diameters and pedestal tops are art changes.
The [support review](exports/support-system/review.html) provides overview,
joint and foundation views for each scene. Run `verify_support_details.py` against
the nine native review exports for focused mesh and attachment checks.
`verify_support_scene.py`, run through MCP, checks the actual scene buffers and
retained native metadata. The Blender file opens at the camelback joint; the
overview and foundation cameras are retained alongside it.

The remaining kit authors the existing station, ties, LSM and brake hardware.
`author_models.py` composes `station_models.py` and `track_hardware.py` through
`run_blender_mcp.py`. Use an installed Blender MCP server and a dedicated scene:

```powershell
python native/art/run_blender_mcp.py --server <installed-blender-mcp-executable>
```

It replaces the authoring scene, exports FBX and `manifest.json`, and saves
`Station-Hardware-V3.blend`. The previous combined train/station blend was deleted;
the station and hardware remain reproducible from these sources and their FBX.
The station preview requires `--script render_station.py` and
`--station-context <canonical-station-boxes.json>`.

`native/unreal/scripts/import_v3_art.py` imports this reduced manifest into Unreal
and verifies metre-to-centimetre conversion and mesh bounds. Terrain preparation
uses `create_content.py`. The frozen default.3 package is independent of these
source assets and retains its original appearance and dimensions.
