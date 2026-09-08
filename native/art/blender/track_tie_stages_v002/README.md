# Track tie v002: exact diagonal webs

Root executes the five `track_tie_stages_v002` scripts in numeric order through separate safe-mode MCP requests. Do not combine activation, mesh evaluation or exports into one request. Existing source/exports are preserved; stages deliberately refuse an existing named scene.

The source tie is copied unchanged and two 3 mm inward-chamfered prisms use the compiled corner specification from `native/unreal/Saved/TrackWeb/20260908-1/track-web-spec.json`. Each web is certified against its own exact OBB before assembly. Cap planes never extend beyond the two specified endpoints. Source pivot is the existing tie centre; rail midpoint is local Z +0.19 m. Source axes are +X forward, -Y rider-right, +Z up, metres.

Save the final printed `VC_TIE2_MANIFEST` JSON as `native/art/review/20260908-tie-v002/manifest.json`. Actual stage output is authoritative for bounds/counts. Root observed all five stages pass: 11 parts, 352 vertices, 660 triangles, bounds approximately [-.07,-.825,-.275584] to [.07,.825,.08] m.

Import only this new asset using `native/unreal/scripts/import_track_tie_v002.py` with `VIBECOASTER_TRACK_TIE_IMPORT_SPEC` pointing to `native/art/import/import-spec-track-tie-v002.json`. The fresh `/Game/Art/V072/TrackWeb1/SM_TrackTieWeb` destination preserves existing train/station assets. Original FBX + explicit UE yaw 90 retains the earlier independently witnessed source-to-UE basis. No auto collision or Nanite is generated. Import receipts verify per-part certificates, source hash, every imported triangle within one approved convex solid, exact bounds and opaque material assignments. Art containment does not replace full-circuit core hardware proof.

For one lightweight connectivity view run `track_tie_review_v002/01_prepare.py`, `02_activate.py`, then `03_render.py` separately. The image uses three ties at 3 m pitch and actual renderer rail/spine radii; it is a visual check, not performance or engineering certification. No source objects are modified.
