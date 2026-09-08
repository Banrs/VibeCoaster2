# Environment detail kit

`build_environment_kit.py` is a Blender 5.2-compatible script prepared for Blender MCP safe mode. It imports only `bpy`, `bmesh`, `math`, `mathutils`, and `json`. It does not launch Blender, read files, install add-ons, register handlers, contact external services, or change runtime source. The root agent controls execution in the shared Blender session.

Before running, create `native/art/source/environment/` and `native/art/exports/environment/` externally. Send the whole script through the guarded MCP `execute_blender_code` tool. It exports nine modular/reference assets plus one complete saved tower, each as GLB and FBX, saves a copy to `EnvironmentKit.blend`, and prints the JSON manifest between `VC_ENVKIT_MANIFEST_BEGIN` / `END` markers. Persist that manifest and source/output hashes externally; the script deliberately has no filesystem access outside Blender operators.

The script replaces only its own `asset_owner = VC_ENVKIT` datablocks on rerun. It preserves other objects, scenes, and materials, and restores the previously active unrelated scene. The `.blend` is a copy of the current shared Blender session, including other existing scene data; exports select only the named asset collection. Nothing is rendered automatically. Select `VC_ENVKIT_StationAndModules` or `VC_ENVKIT_AdaptiveTowerReview` and their named cameras for inspection.

## Assets

- `SM_StationPlatformPanel`: 3 m long, 3.85 m wide, 0.8 m deep; top-centre pivot, inward-facing edge at local -Y. Concrete slabs, narrow recessed joints, graphite fascia and a restrained ochre edge line. The 1 m end panel closes the default 82 m platform without stretching details.
- `SM_StationRoofPanel`: 3 by 11.3 by 0.36 m, centre-plane pivot. Weather skin, thin side fascia, transverse ribs, underside infill and inset diffuser strips all stay inside the existing canopy box. No extra eaves, stairs or equipment extend its envelope.
- `SM_StationPost`: 0.40 m square, 5.22 m high, base-centre pivot, separate recessed channels and cap/base detail. Use the full authored size; a differently sized post requires regeneration.
- `SM_TrackTie`: exactly the 0.14 by 1.65 by 0.16 m existing tie envelope, midpoint origin. It is an open flange/web section with inset fasteners. Runtime origin remains 0.19 m below the rail datum.
- `REVIEW_TrackRailToSpineWeb`: coral review plates at the rail datum, deliberately separate because they extend outside the existing tie envelope. Do not integrate until additional hardware solids and clearance are validated. It is not a manufacturer connection drawing.
- `SM_SupportJoint_R032_L032` and `SM_FootingCap_R180_H020`: reference detail inside a 0.32 m radius by 0.32 m member slice and 1.80 m radius by 0.20 m footing-cap slice. These are demonstrations of contained detail. Different persisted radii/lengths require regeneration and checks, not generic stretching. Place the cap slice within the footing rather than above it.
- `REVIEW_CoordinateWitness`: a true one-metre cube, +X ochre nose and -Y coral rider-right witness. Authoring axes are +X forward, +Z up, -Y rider-right. GLB uses the standard glTF Y-up conversion; FBX is configured X-forward/Z-up in metre authoring units. Actual UE scale and handedness must be verified before integration.
- `REVIEW_AdaptiveTower_Saved072`: all 92 members from canyon seed 42 support 72, each using the exact saved base/top/radii/kind/contact; assembly height from saved support base/top is 74.05508292796398 m. The assembly is only translated by its saved base. Members have individual base pivots, local +Z along the actual endpoint direction, unit scale, and inscribed 32-sided solids. Footings retain distinct actual elevations. This is an example of endpoint-driven geometry, never a replacement fixed pillar for other supports.

The script checks each ordinary asset against its declared convex box and checks circular reference radii before exporting. These art checks do not certify structural strength or replace the numerical core clearance gates. Canonical procedural rails, towers, terrain and train-path transforms remain authoritative. No ground textures or external assets are used.

`environment_support_reference.json` records the original accepted save path, SHA-256, generation version, exact member coordinates and support identity. Its COASTER5 byte count and FNV-1a checksum were verified during extraction. This was not a fresh simulation. The saved reference is also embedded literally in the script, so execution does not read private repository files through Blender MCP.

## Validation status

The authored script passes `blender_mcp.safe_mode.validate_code` from the installed pinned MCP package. Python syntax compilation passes. Blender execution, actual visual inspection, exported-mesh import round trip, and UE integration are still the root agent’s next steps; this note does not claim those have happened.

## Export-session stability revision

The renderer is selected from the actual registered RNA engine identifiers (Eevee Next, Eevee, then Workbench). Export masters remain linked to a dedicated scene throughout all exports; the script changes selection, rather than unlinking collections under the active dependency graph. The editable source is saved before export starts. This reduces avoidable graph mutation and preserves models if a native exporter fails; it does not establish the cause of the unrelated train-v001 native crash. Root must still execute and inspect the actual Blender 5.2 result.
