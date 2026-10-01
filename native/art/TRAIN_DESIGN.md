# Riftwake train model

Editable train art, 30 September 2026. Current profile: `riftwake-train-02`.
The user corrected the first wheel-clamp orientation and asked for less support.
The current carriers are compact C clamps in the transverse YZ plane, open
toward the rail, with 100 mm thickness along the direction of travel. The tall
longitudinal cages and their bridging members have been removed.

The design has seven articulated car assemblies, two seats per car, a low lead
nose, shallow clear aero lip, individual lap restraints, contoured seat shells,
ventilated wheel rims and a visible chassis. The finish is petrol enamel with
warm titanium accents and charcoal seating. The road, guide and upstop wheels
are separate named parts. This is an original art proposal awaiting visual review.

## Dimensions and files

| Property | Proposed model |
| --- | --- |
| Rail-centre gauge | 1.40 m |
| Overall width | 2.35 m, within the 2.40 m requirement |
| Cars / riders | 7 / 14 |
| Car centre spacing | 3.40 m |
| Overall length including rear drawbar | 24.523 m |
| Height above rail centres | 2.015 m |
| Running / upstop / guide wheel diameter | 520 / 300 / 280 mm |
| Track fit | Current Exa profile: 210 mm rails, centred crossheads |

- Editable source: `exports/train/Riftwake-Train.blend`, scene `Riftwake / Train design`.
- Portable train-only export: `exports/train/Riftwake-Train.glb`. Metres, glTF Y-up,
  seven independently transformable car nodes sharing two meshes.
- Parameters: `train_profile.json`.
- Authoring: `author_train.py`; checks: `verify_train_model.py`.
- Rendering: `render_train.py`; portable export: `export_train.py`.
- Nine rendered views and the local gallery: `../../out/train-model/review.html`.
- Verification: `../../out/train-model/geometry-checks.json` and `verification.json`.

Run the Python scripts through live Blender MCP with `runpy.run_path`. `ROOT`
can be supplied in `init_globals` to override the default repository path.
`render_train.py` accepts `VIEWS`, a list of camera names: `hero`, `full-train`,
`front`, `side`, `wheel`, `clamp`, `seating`, `rider`, `rear`. Keep Blender at
eight logical CPUs, BelowNormal priority, eight render threads and 16 samples.
The editable source retains per-part meshes and curves; the GLB uses disposable
merged copies and does not include the review rail, floor or lights.

## Evidence and scope

The actual evaluated train measures 2.3499999 m across a measured 1.4000000 m
gauge. All 495 unique evaluated component meshes are closed; Blender's coincident
curve-cap seams are welded at 0.1 micrometres for this closure check. There are
no triangle intersections with the straight reference rails, crossheads, gussets
or spine. Conservative longitudinal projection leaves 42 mm between the nearest
part and the crosshead/gusset solids. All 132 rays from both front-seat eye points
clear the model across yaw -25 to +25 degrees and pitch -10 to +15 degrees.

These checks cover the art and straight-track fit. A full occupied-train sweep,
station and launch-hardware fit, dynamic bogie articulation, and Unreal adoption
remain separate integration work. The current development game still uses
position markers. Default.3's save, selected package and Play shortcut are intact.
The existing support review Blender file is also preserved.

The design draws on Intamin's description of open lap-restraint seating,
machined chassis, ventilated wheels and aerodynamic bodywork on
[Falcons Flight](https://www.intamin.com/project/falcons-flight/).
The proportions, wheel-carrier layout and livery here are original proposals,
not published manufacturer measurements or an engineering assessment.
