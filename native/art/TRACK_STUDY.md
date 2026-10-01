# Exa track study

The first replacement track assembly, authored through Blender MCP on 28 September
2026. The user's direction is an Exa coaster informed by existing and concept
designs, with visibly substantial running rails. This is an editable proportions
study and an Unreal import kit; its dimensions have not been adopted by the live
default.3 ride.

## References and interpretation

The reference family is [Intamin's Falcon's Flight](https://www.intamin.com/project/falcons-flight/).
The [built track close-up](https://www.intamin.com/wp-content/uploads/2026/02/IAR_SoMe_Falcons100Procent_20263.jpg)
shows a large round spine, repeated tubular crossheads, longitudinal plate gussets
and substantial flanged support connections. The
[built camelback](https://www.intamin.com/wp-content/uploads/2026/01/QIC_LSMLaunch_FalconsFlight_9.jpg)
also demonstrates that the giant support superstructure is a separate design
problem from the track section.

The retained Intamin-watermarked concept animation stills in
`docs/references/plateau-with-train.png`, `camelback-original.png` and
`out/reference-videos/falcon-thirdperson/detail-*.jpg` informed the broad
proportions. Their provenance is recorded in
`docs/references/2026-09-redesign-reference-audit.md`. These are concept evidence,
not fabrication drawings. All dimensions below are proposed game-art dimensions,
not published measurements of Falcon's Flight.

## Current section

| Part | Dimension |
| --- | --- |
| Rail-centre gauge | 1.40 m |
| Running rails | **105 mm outside radius / 210 mm outside diameter** |
| Running-rail wall | **20 mm**, modeled as a hollow shell |
| Main spine | 340 mm radius / 680 mm diameter; 24 mm wall |
| Spine centre | 800 mm below the rail-centre datum |
| Crossheads | 130 mm diameter, spaced every 1.40 m |
| Crosshead axis | At the rail-centre height, with zero vertical offset |
| Paired longitudinal gussets | 24 mm plate |
| Support flange datum | 1.74 m below the rail-centre midpoint |
| Support columns | **750 mm diameter** |
| Support head | Full-width flat cap, 1.22 m saddle, 60 mm seat plate, 50 mm transverse webs and 55 mm longitudinal cheeks |
| Connection flanges | 1.05 m diameter; 65 mm upper / 45 mm lower; eight bolts on an 880 mm circle |
| Review bases | 1.25 m square, 180 mm thick |
| Provisional running wheels | 520 mm diameter |
| Train-width study envelope | 2.35 m within the retained 2.40 m maximum |

The user restored the 105 mm running-rail radius and identified the thin support
wrapper and contracted neck as the connection problem. The head now keeps the
column's full 750 mm diameter. The first full-width revision was also rejected:
its curved bearing stopped short of the spine sides, leaving a wedge-shaped gap.
The extended cope was also rejected: the column is wider than the spine, leaving
exposed edges beside it. The current revision ends the column flat, with separate
transverse webs and longitudinal cheeks connecting a seat plate to the pipe. There is no wrapped tube or
tapered neck. Round mating flanges join the full-width column sections.
This latest revision awaits user design approval.
Their end-on image also identified the crosshead below the rail centre. Its axis
now passes through both rail centres, with refitted ends, gussets and wheel positions.
The short display columns are review stands, not the finished
support system. Teal wheels are removable fit placeholders, not a train design.

## Files and regeneration

- `track_study_profile.json` is the dimension and material source.
- `track_study_geometry.py` shares the rail frames, hollow tube sweeps, gussets,
  wheel envelopes and support attachment datum with the verifier.
- `author_track_study.py` builds the editable assembly in a dedicated scene.
- `render_track_study.py` renders hero, joint close-up, section, banked, inverted, overview and
  rider views.
- `exports/track-study/Exa-Track-Study.blend` contains the editable components,
  an 11.2 m straight, a 22.4 m banked curve and a 33.6 m roll-to-inverted sample.
- Six FBX assets and their dimension/material manifest are beside the blend.
- `../unreal/scripts/import_track_study.py` imports those assets into
  `/Game/Art/TrackStudy` and creates `/Game/Art/TrackStudy/Maps/ExaTrackStudy`.
- `../unreal/scripts/capture_track_study.py` captures the saved Unreal review
  cameras. Its images and receipts go to `out/track-study`.

Run the MCP bridge with a Python environment containing the MCP client, while
Blender is running its loopback add-on. The bridge retains safe mode.
When `--server` is omitted it uses Codex's configured Blender stdio MCP and its
environment, so an updated app connection replaces the previous executable.

```powershell
python native/art/run_blender_mcp.py --server <blender-mcp-executable> --script author_track_study.py --output exports/track-study --prompt "Build the Exa track study"
python native/art/run_blender_mcp.py --server <blender-mcp-executable> --script render_track_study.py --output exports/track-study --view hero --prompt "Render the Exa study"
python native/art/verify_track_study.py
```

Run the Unreal importer in a separate editor process with
`-ExecutePythonScript="<repo>/native/unreal/scripts/import_track_study.py"`.
The importer checks axis/unit conversion and material slots, and changes only
its dedicated content folder and generated review actors. It preserves manual
additions in that map. Use the analogous capture script for Unreal screenshots.

## Checks and next integration

The focused verifier checks the actual rail buffers for a constant 1.40 m gauge,
105 mm outer radius and 20 mm wall, closed pipe shells, finite vertices and correctly handed frames through banking
and inversion. It checks the support datum and conservatively projects the
provisional wheel barrels against the crossheads/gussets. The current minimum
local gap is 42.0 mm; this does not include bogie arms or an occupied-train sweep.
The column head is checked for a closed mesh and its full diameter at both
perimeter rings. It ends flat below a seat plate. Two transverse webs and two
longitudinal cheek plates seat into the spine within its width; their caps, plate contact and pipe contact
are checked. This replaces the rejected coped column, which was wider than the
spine and left exposed edges when its top was extended around the pipe.
Six invalid profile fixtures check rejection of gauge, width, wheel collision,
wall thickness, wrong rail radius and off-centre crosshead errors. Results are
in `out/track-study/geometry-checks.json`.

The next runtime geometry change must adopt the profile in procedural track,
hardware, train clearance and station fit together. The mount datum provides the
starting interface for support generation; terrain and foundations must then
follow the real support feet and rider clearances. The separate
[adaptive support system](../../docs/support-system.md) now generates and validates
whole-element member graphs on the existing native geometry contract. The active save,
package selector and Play shortcuts retain their existing default.3 identities.
