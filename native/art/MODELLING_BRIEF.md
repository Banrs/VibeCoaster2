# Coaster modelling brief

## Required higher-force restraint concept — 2026-09-10

The 0.8.3 numerical design now selects the higher historical F2291-23b restraint-dependent force profile. The train remains conventionally seated and individually contained. Model an appropriate backrest/headrest with maintained rider support and a padded upper-torso/over-shoulder restraint that minimizes forward motion. The numerical enhanced braking curve reaches -3.5 g for up to two seconds only with load-buildup onset below 15 g/s. Extended uplift reaches -2.8 g at 200 ms, tapering with duration, and requires a special restraint arrangement justified by ride analysis. An ordinary lap bar or a generic OTS silhouette does not prove that provision.

These requirements are pending modelling/design verification. They are explicitly flagged in every native force report. Existing train art, including the preserved lap-bar models, is not evidence that they are met. Keep the user's simple conventional train styling; show plausible padding, contact, restraint hinges and load paths without inventing qualified dimensions or components. Check front/middle/rear POV and the full moving/reach envelopes at the unchanged camera/force datum. Structural, fit/population, closure/interlock and restraint-load qualification remain separate from mesh inspection.

The selected curves, conditions and source editions are in [FORCE_GUIDELINES.md](../core/FORCE_GUIDELINES.md); the profile and requirement text have one runtime owner in [force_envelope.hpp](../core/include/coaster/force_envelope.hpp). Higher force capacity does not authorize consuming existing clearance reserves or shrinking the rider envelope.

## Preserved initial dimension assessment

2026-09-07. Read-only source assessment of native `0.7.0-folded.1`. Dimensions below are the current game's geometry contract, not manufacturer drawings or structural certification. Production source was not changed for this brief.

## First deliverable: a usable train POV

Replace the five cubes per car with an open, contoured two-seat car. Keep the existing camera/force datum and train spacing. The current seat-back cubes reach 1.45 m above rail and present broad rectangular faces to the following cars; model separate seat shells, slim headrests, an open centre gap, and believable restraint hardware. Review front, middle and rear views on an inversion, a crest and a bank transition. Do not raise the camera or alter ride speed to hide obstruction. Do not claim that a redesigned silhouette alone reproduces an authentic manufacturer seat.

Use an original modern steel-coaster visual language. Mechanical proportions must follow the actual rail and car contract; names, logos and proprietary engineering details are unnecessary.

## Coordinates and import contract

- Author in Blender metric units: 1 Blender unit = 1 metre. Apply object scale and rotation before export. Keep reusable mesh origins deliberately placed; never centre all objects on the whole scene.
- Author local +X forward, +Z up, -Y rider-right (a right-handed source frame). The imported UE asset must have +X forward, +Y rider-right, +Z up and metre dimensions represented in centimetres.
- The runtime core uses metres and reflects world Y exactly once at the UE boundary: `(x,y,z) -> (100x,-100y,100z)`. A static-art importer must perform its own single source-to-UE handedness conversion, not receive another manual world-coordinate reflection. Verify an asymmetric left/right witness asset and a 1 m cube before importing the collection; exporter settings are not yet established in this project.
- A car's root origin is the canonical rail midpoint at its own distance sample. Its eye reference is `(0,0,1.2 m)`; its local forward is the track tangent. In a Blender preview scene use this exact eye at the current 82 degree camera field of view, then verify the actual UE camera as authoritative.
- Rotating wheels: origin at axle centre, spin axis along the actual axle; separate bogie yaw/pitch pivot from wheel spin pivot. Restraints: origin on the real hinge axis, with a closed reference pose. Static prototypes can use separate named objects without inventing a skeletal physics rig.
- Variable straight support members: source pivot at base, local +Z toward top. Runtime endpoints and both radii must drive the final shape. Do not scale one fixed tower to every height.
- Station modules: local +X along track, +Z up; assembly origin is station track sample zero. Individual panels may use a corner origin for tiling, but record that offset explicitly.

Source: [CoordinateContract.h](../unreal/Source/VibeCoaster/Public/CoordinateContract.h), [CoasterMesh.h](../unreal/Source/VibeCoaster/Private/CoasterMesh.h).

## Exact train and track dimensions

| Item | Current contract in metres |
|---|---|
| Default train | 6 cars; 3.4 m centre spacing; car mass parameter 1500 kg each (9000 kg total model mass) |
| Car root samples | Train centre distance + 8.5, +5.1, +1.7, -1.7, -5.1, -8.5 m |
| Current visual car base | 2.55 long × 1.70 wide × 0.45 high; centre `(0,0,0.35)`; bottom 0.125, top 0.575 |
| Current seat backs | Two 0.25 long × 0.60 wide × 1.00 high blocks; forward -0.80, lateral ±0.43, up 0.95 |
| Current forward posts | Two 0.18 long × 0.12 wide × 0.65 high blocks; forward +0.65, lateral ±0.66, up 1.05 |
| Body length over six straight cars | 19.55 m; 0.85 m nominal gap between 2.55 m bodies |
| Rider eye/force sampling height | 1.20 m above rail centreline; default selected front/middle/rear car indices 0/2/5 |
| Rail centres | Lateral ±0.65 m at up 0; 1.30 m centre-to-centre gauge |
| Running rail cross-section | Circular radius 0.085 m (diameter 0.17 m) |
| Spine | Circular radius 0.16 m; centre at up -0.55 m; bottom -0.71 m |
| Current tie | 0.14 along track × 1.65 across × 0.16 high; centre at up -0.19 m; every 3 m |
| Track render sampling | 8 sides per tube ring, approximately 2 m longitudinal ring spacing; 80 m chunks |

Source: [coaster.hpp](../core/include/coaster/coaster.hpp) lines 15, 71-75; [VibeCoasterWorld.cpp](../unreal/Source/VibeCoaster/Private/VibeCoasterWorld.cpp) lines 321-348; [CoasterMesh.cpp](../unreal/Source/VibeCoaster/Private/CoasterMesh.cpp) lines 10-73.

The clearance body is deliberately larger than the current car art: longitudinal ±1.275 m, lateral ±1.5 m, and up 0 to `max(2.4, seatHeight + 0.6)` m, with separate continuous-motion reserves. These reserves are not extra space to consume with art. New seat shells and restraints should stay inside the unpadded body. An under-rail wheel assembly is **not** automatically covered by this envelope: under/side wheels below up 0 require an explicit moving-hardware envelope and independent clearance validation before runtime integration. The static rail/tie/spine envelopes are not a substitute for moving bogie clearance. Do not silently shrink wheel geometry until it becomes mechanically incoherent.

Source: [clearance.cpp](../core/src/clearance.cpp) lines 78 and 142-153.

## Asset modules and priorities

| Priority | Asset / suggested name | Authoring and integration requirement |
|---|---|---|
| 1 | `SM_TrainCarBody`, `SM_SeatShell`, `SM_Headrest` | Fit the 2.55 × 1.70 m car footprint. Root at rail datum. Preserve a readable forward view without hiding all neighbouring cars. Separate painted shell, dark seat padding and metal material slots. |
| 1 | `SM_RestraintClosed` | Separate hinged part; restrained rider space is within the existing body envelope. Make the closed pose first. Add animation only when controls/state actually exist. |
| 2 | `SM_Wheel`, `SM_BogieFrame` | Build a rail-cross-section preview with running, side-guide and up-stop contact. Wheel diameter is a design choice still requiring geometry review; it is not specified by current physics. Keep as a review asset until the moving clearance contract is extended. |
| 2 | `SM_TrackTie` | Replacement can occupy the exact current tie box at its midpoint origin. A realistic rail-to-spine web extends outside that box and therefore needs explicit hardware validation; author it separately for review. |
| 2 | Procedural rail/spine cross-section | Keep the canonical sampled path. Higher radial detail and better normal/material treatment can improve tubes without replacing the track with an authored fixed circuit. Do not use long rigid Blender rail pieces over changing curvature. |
| 3 | `SM_StationPlatformPanel`, `SM_StationRoofPanel`, `SM_StationPost` | Tile or deform only inside actual station boxes. Separate end caps from repeatable panels; avoid stretching stairs, doors, bolts or signage with the whole building. |
| 3 | `SM_SupportJoint`, `SM_FootingCap` | Detail must remain inside the persisted member solids, or receive an explicit new validated solid. Use recessed bolt/plate detail where possible. Full supporting shafts, tapers and footings remain endpoint/radius driven. |

## Station assembly

For the default six-car train, boarding interval is -18 to +64 m, length 82 m, midpoint +23 m. The generator computes these dimensions from train length, so modular art must read the saved station rather than assume this size forever.

| Part | Default dimensions and position relative to station rail datum |
|---|---|
| Two platforms | Each 82 × 3.85 × 0.80 m; centre x=23, lateral=±3.275, up=-0.40; top at rail level; inner edge 1.35 m from centreline |
| Canopy solid | 84 × 11.30 × 0.36 m; centre x=23, up=5.40; underside 5.22 m |
| Posts | 0.40 × 0.40 × 5.22 m; lateral ±4.80; centre up=2.61 |
| Post stations | Six pairs at x=-14, 0.8, 15.6, 30.4, 45.2, 60 m |
| Piers | 0.60 × 0.60 m footprint; variable vertical length from the actual footing top to platform bottom (up=-0.80) |
| Station footings | 1.80 × 1.80 m footprint; variable bottom/top enclosing actual terrain and its certified slope margin |

These are closed physical boxes used by station/structure validation. A thinner detailed shell may sit inside them while the conservative box remains the collision certificate. New stairs, outer railings, overhanging eaves or station equipment outside them require added physical geometry and validation. Do not render a platform crossing the rail corridor. Source: [station.cpp](../core/src/station.cpp) lines 107-141.

## Adaptive support assembly

The authoritative support is a graph of straight, closed, tapered circular members with saved `base`, `top`, `radiusBase`, `radiusTop`, kind and a single spine contact. The mesh currently uses an inscribed eight-sided cross-section. Rounded replacement surfaces may reach the canonical circle, but not exceed it. Every terrain footing is calculated separately; do not flatten all feet to one plane.

The attachment is at `track.position - track.up * 0.71 m`. Low upright single posts are tried first (height 3-22 m), then paired bents (3-90 m), then four-leg braced towers. Families are rejected if they intersect the complete train/track/station/terrain envelopes. Supports are attempted every 40 m. Towers can use lateral offsets of ±12/18/26/36/48 m; 2 m under-spine standoff is normal, with 6 and 10 m fallbacks for constrained roll/crossing geometry. These are intentional placements, not a licence to recentre a tower under the visible rail.

For tower height `h` metres: base half-width `clamp(1.8 + 0.035h, 2.1, 12)`; top half-width `clamp(0.7 + 0.003h, 0.8, 1.8)`; leg radii `0.32 + 0.0022h` at base and `0.19 + 0.00045h` at top; brace radius `0.10 + 0.00045h`; ring radius `0.13 + 0.0005h`; tiers `ceil(h/16)`. These proportions are geometry, not load engineering. Footing broad radius is `1.2 + 2.3 * legBase`; steep slopes may select narrower embedded rock piers. The persisted member data, not an independently repeated formula in Blender, remains authoritative.

Source: [supports.cpp](../core/src/supports.cpp) lines 8-111 and 165 onward; [support_mesh.hpp](../core/include/coaster/support_mesh.hpp).

## Source assets and UE integration feasibility

Keep editable sources in `native/art/blender/`, reproducible modelling scripts in `native/art/scripts/`, and exchange exports in `native/art/exports/`. These directories are proposed; this brief does not create models. Proposed cooked destinations are `/Game/Art/Train/`, `/Game/Art/Track/`, `/Game/Art/Station/`, `/Game/Art/Supports/`. Preserve material-slot names and an asset manifest with metre bounds, origin, axes and source-file hash.

Current runtime loads `/Engine/BasicShapes/Cube.Cube` for ties, all car parts and station boxes, and `/Engine/BasicShapes/Cylinder.Cylinder` only for legacy simple supports. Modern support solids and rail tubes are already procedural mesh buffers. There is **no existing external static-mesh import pipeline or separate car-part mesh loader**. `Cars` is one instanced-static-mesh component with five cube instances per car. A finished modular train needs separate instanced components per part mesh (or one assembled static car for the first unanimated version), updated transforms and staging/cancellation handling. Replacing the cube globally would deform every part incorrectly.

`create_content.py` currently creates materials and the level; `package.ps1` bootstraps and verifies those assets. Extend that existing preparation step with explicit local imports and required asset paths when models are ready. Load cooked meshes once on the game thread, retaining the existing value-only worker preparation. Keep portable Blender/exchange sources for macOS; Windows `.uasset` cooking alone is not a Mac delivery.

Current materials: `/Game/Materials/M_Rail`, `M_Ground_Plain`, `M_Structure`, `M_Footing`, `M_Train`. Preserve the user's plain-ground preference. High-value first art work is silhouette, scale, contact and unobstructed POV; no ground texture work is required.

Integration sources: [VibeCoasterWorld.cpp](../unreal/Source/VibeCoaster/Private/VibeCoasterWorld.cpp) lines 80-97, 244-299, 321-349; [create_content.py](../unreal/scripts/create_content.py); [package.ps1](../unreal/scripts/package.ps1).
