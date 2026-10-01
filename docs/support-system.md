# Adaptive coaster supports

29 September 2026. This implements the requirement from **Redesign coaster and
supports**: supports adapt around the ride, including signature elements; the
track does not route around a repeated tower model.

**30 September:** the [Exa integration](exa-runtime-integration.md) now ports the
fittings into portable native geometry and validates their complete visible
envelopes. The explicit `exa-1` review uses those same meshes in Blender and
Unreal. The user rejected the first integrated review's topology and exposed
junctions, then found the all-tall-element lattice too dense and unattractive.
The current resource-adaptation review is in `out/support-adapted`: a shared
girder on two trestles for the twisted drop, shoulder-supported inversion crowns,
simpler cliff cantilevers, and a less dense camelback lattice. It replaces the
repetitive tall bents in `out/support-direct`. The earlier branching-arm proposal
in `out/support-forms` was rejected. The current review is not user-approved. Earlier Blender files and
`out/exa-revision` / `out/support-refinement` are comparison material. See the
integration record for research, current verification and model paths. The
older receipts below describe the historical Python art study.

## Reference interpretation

| Reference | Observation used in the generator |
| --- | --- |
| [Intamin's Falcon's Flight project](https://www.intamin.com/project/falcons-flight/) and its [built camelback photograph](https://www.intamin.com/wp-content/uploads/2026/01/QIC_LSMLaunch_FalconsFlight_9.jpg) | A large open span, inclined main legs and a triangulated structure distributing several track connections into a small number of foundation groups. Intamin describes the 165 m camelback as a roughly 2,500-tonne superstructure. |
| User's supplied Exa concept render, retained in `out/support-study/reference/user-exa-camelback-render.png` | Continuous inclined primary legs run from foundations to the crown. Outer chords follow beneath the track, connected by a few large triangular bays. This supersedes the rejected short-pier/elevated-bridge interpretation. |
| [Intamin's December 2024 track-closing photographs](https://www.linkedin.com/posts/intaminofficial_falconsflight-qiddiya-intamin-activity-7272989609804902400--P7p), retained as `out/support-study/reference/ff-closing-1.jpg` and `ff-closing-4.jpg` | The close external view shows a short, substantial flanged stub and a stiffened bracket under the spine. This informs the compact flanged saddle; exact hidden construction and dimensions are not recoverable from the photograph. |
| [SFOT Source's original TRR ground photographs](https://www.sfotsource.com/rides/tormenta), Immelmann DSC02913 / DSC02708 and loop DSC02301 | Direct inclined tubes and compact bearing connections. Local headers distribute neighbouring attachments without long fans to remote hubs. The user's requested reference method is external photos/renders, not POV footage. |

These are visual design observations, not surveyed member dimensions. The game
uses original tubular frames inspired by them. Member sizing below is a geometric
heuristic, not a manufacturer design or a finite-element stress calculation.

## Code and behaviour

- `native/core/src/support_assemblies.hpp` plans whole elements from sampled
  track frames and actual terrain clearance. Prominent upright crests select a
  camelback; substantial descending bank distinguishes a twisted drop. Inverted
  runs and entry/exit headings distinguish loops and Immelmanns.
  No seed number, named recipe element or fixed world axis selects a
  particular mesh.
- Camelbacks use two braced trestles below the high shoulders, joined by a deep
  crown truss. Main legs, outer track-following chords and transverse bracing
  meet at shared nodes in every bay. Primary leg bays are at most approximately
  48 m tall for Exa; the bottom braces join raised toe nodes above the plates.
  The Exa planner includes steep upright approaches: low world-Z in the up
  vector caused by pitch must not truncate the structure at the crest.
- `support_bridge.hpp` separates beam sections, rail mounts, girder nodes and
  terrain footings. Primary diameters depend on height; chord/web sizes and
  depth depend on the actual distance between trestles. Three continuous chords
  have closed transverse triangles and alternating side/underside diagonals.
  Large bracing bays stay at most 24 m long; each longitudinal chord is sampled
  at most 6 m along the real track curve. This prevents polygonal crown profiles
  without multiplying the main lattice bays. Each trestle has two main feet
  and one shared rear footing; backstays end at real mid-leg nodes.
- Exa inversions place a shallow spatial girder only around the upper crown,
  supported from its two shoulders. The loop opening no longer needs a central
  ground frame. The crown is found from the sampled elevation range, including
  a rolling Immelmann exit. Lower shoulders retain compact direct paired rakers,
  at no more than 48 m spacing. The validated direct-frame family remains the
  fallback where terrain or another track rules out a shared crown.
- `support_twisted.hpp` uses the shared girder across the complete banked hill.
  Its short rail saddle follows bank; the girder stays below the track and the
  foundation frame does not spin with it. Two main trestles and two low end seats
  replace a row of independent tall bents. Candidate depth, spread and rake are
  checked against actual terrain and the complete rider, track and fitting mesh.
- `support_cliff.hpp` fits Exa highland terrain during explicit generation or
  regeneration. It searches cliff front/width candidates around the typed cliff
  section (geometric fallback for fixtures), targeting an approximately 8 m
  horizontal centreline-to-wall gap on the steep descent. The frozen track and
  occupied train envelope are unchanged; a candidate must pass continuous swept
  terrain clearance before selection. Short reaches use a single substantial
  cantilever and rock socket; reaches over 12 m add one lower haunch and socket.
  The actual ride has six brackets and seven sockets, without ground bents.
  Each socket's entire rear cap must be buried, its front bearing cap exposed,
  and all visible steel checked against terrain and the train sweep.
- Ordinary tall Exa bents use tapered primaries, three-dimensional backstays and
  open bracing bays. Adjacent upper rollout mounts may share one complete bent.
  The branch has three distinct primary roots; a bank transition can move its
  upper root below the roll when a direct chord would cross the rider. Every
  candidate uses the same solid and swept-clearance checks. This removes whole
  duplicate towers, not their visible parts alone.
- Height and span change section sizes, bay count, truss depth and footing
  spread. Candidate footprint shifts and depth changes are tried against the
  actual terrain, station and continuous rider/hardware sweep. Foundations use
  terrain height ranges over their full footprint; shallow rakers get adequate
  pedestal height for the entire circular end of the steel member.
- The existing compact posts, wishbones and offset A-frames remain for ordinary
  track and as a validated fallback where an assembly cannot fit. If none fit,
  generation reports failure. It never silently alters the track or suppresses
  collision checks. This is a bounded candidate search, not a guarantee that
  every possible terrain/track combination can be supported.

`validateSupportLayout` checks one shared endpoint graph across explicit support
members. Each connected component must reach both a verified spine joint and a
real terrain footing or a validated rock socket. Crossings through a member's middle do not count as joints.
Each attachment retains exactly one tightly bounded own-spine contact; that
contact never exempts it from rider clearance. Disconnected, unanchored, buried,
oversized or malformed members are rejected. Cancellation preserves the previous
complete support graph.

The existing COASTER6 member representation stores the graph losslessly; member
kind 2 now means `RockAnchor` (old Steel=0 and Footing=1 remain stable). No
visual-only braces or duplicate foundations are added. Loading validates shared
connectivity, and Unreal renders these same explicit members with its existing
solid mesh builder. Existing saved designs retain their stored members on load;
support generation is not an implicit migration of a saved ride.

`regenerateSupports` is the explicit migration API. It fits the cliff and builds
supports into a copy, then
runs the same geometry, station, coarse/fine and spatial replay acceptance used
for a loaded ride. Only a fully accepted result replaces the caller's design;
cancellation or failure preserves the original. The accepted revision is rebuilt
before saving, so support edits cannot bypass the normal persistence checks.

## Review and checks

The native Exa revision uses directional continuity to select each joint's
through chord, with at least one of the largest incoming tubes in the host pair.
This prevents a large branch being incorrectly fitted into two smaller hosts.
Branch rings are fitted against the actual triangulated host
surfaces, including taper; a closed welded node can receives blind forks with
no opposing through pair. Ground foundations and wall sockets fit steel ends
to the actual oriented bearing plane, with matching plates and bolts.
Only the validated concrete socket body may enter the rock. Fitted end rings have 64 vertices.
`import_native_exa.py` preserves the native fan triangulation in Blender.

The primary review now uses the complete saved ride, as requested by the user.
Export `out/support-direct/final/regenerated-supports.vcdesign` with
`support_review out/support-direct/ride/fixtures SAVED_DESIGN --current-supports`.
Import with `REVIEW_ROOT='out/support-direct/ride'`, `CASE='ride-42'`,
`DATA_FILE='saved-layout'` and `SCENE_NAME='Riftwake / Exa generated ride'`.
The saved model is `native/art/exports/support-system/Riftwake-Exa-Direct-Review.blend`.
`render_native_ride.py` places external cameras using the actual exported
regions, support bounds and terrain. It does not author replacement geometry.
Select `REGION_INDEX=0..3`, `'cliff'` or `'tallest'`, and `VIEW='ground'`,
`'oblique'`, `'joint'` or `'anchor'` (cliff sockets only).

For secondary regression fixtures, set `REVIEW_ROOT='out/support-2030'`, `CASE`
to a fixture name and `SCENE_PREFIX='Exa 2030 / '`, then execute
`native/art/import_native_exa.py` through live Blender MCP. Execute
`render_native_support_review.py` with `VIEW` set to `ground`, `oblique`,
`joint`, `reverse`, `shoulder` or `foundation` to inspect external views.
Tall inversion fixtures use `out/support-2030/tall` and `Exa 2030 Tall / `.
`SUPPORT_COLOR=(.52,.57,.60,1)` selects the scene-local satin concept finish;
geometry and track materials remain identical. `support_review_terrain.py`
joins the actual exported terrain boundary to the external backdrop.
`audit_native_support_joints.py` probes endpoint vertices and edge midpoints
against neighbouring real mesh surfaces. It confirms potential BVH precision
failures with double-precision triangle distances and reports exposed ends
over 12 mm. This supplements full native clearance and visual inspection.

Use `support_review OUTPUT SAVED_DESIGN --current-supports --joints-only` to
export a saved graph's fitted tubes without regenerating supports or writing
the large track/fastener payload. It still validates the source save. The
optional streaming `extract_native_joint_meshes.py` handles existing full
fabrication exports. The commands and receipts below are historical context.

Build `support_review` through CMake, then run:

```powershell
native/build/support_review.exe out/support-study/data
native/build/support_review.exe out/support-study/data out/riftwake-completion-07.vcdesign
native/build/support_review.exe out/support-study/data out/riftwake-completion-07.vcdesign --save-review
python native/art/run_blender_mcp.py --script author_support_study.py --support-context out/support-study/data/camelback-0.json --output exports/support-system --prompt "Review the generated camelback support graph"
```

The runner uses Codex's configured Blender MCP by default, with `--server` as an
optional override. This review was completed using the newer `mcp-for-blender.exe`
connection. The separate downloaded Blender Lab add-on was not required.
Safe mode remains enabled. Format 2 exports include each exact attachment frame
and track samples at no more than 350 mm spacing, including every attachment
station. The runner transfers the complete JSON through bounded MCP calls into
a collection's plain metadata; it does not downsample, round coordinates or use
executable text blocks to transport model data.

`native/tools/support_examples.hpp` supplies open geometric fixtures, not complete
accepted rides. Nine cases vary element type, heading, height and terrain. An
additional test changes only terrain beneath an identical track. Tests verify
shared foundations, full member clearance, exact deterministic regeneration,
unchanged track frames, rejected detached/unanchored members and cancellation.
Current fabrication tests also cover two rotated cliff fixtures, fully buried
socket backs/exposed caps, rejected floating sockets, cancellation preserving
terrain, and a banked hill selecting the distinct twisted-drop assembly.
The existing support suite verifies actual accepted ride generation, canonical
mesh buffers and exact save/load member round trips.

The current flat camelback has 23 attachments without individual footings;
the loop has 10. These are measured example counts, not targets imposed on seeds.

The optional `--save-review` writes a separate `regenerated-supports.vcdesign`
after full native save validation and an independent reload. It never selects a
profile or overwrites its source save. Use an isolated Unreal profile to view it.

The [visual review](../native/art/exports/support-system/review.html) includes two
camelback examples, a vertical loop and an Immelmann, each with an overview,
ground and broadside views, track-mount close-up, foundation close-up and
structural-node detail. All cameras remain in the editable Blender scenes.

Blender scenes in `native/art/exports/support-system/Support-System.blend` are
created through MCP from the native export. Member axes, footing positions and
foundation ground footprints are retained. The retained 105 mm rail-radius /
1.40 m gauge Exa **art study** is
aligned by the spine-contact datum for visual context. Its larger track section
is not the legacy runtime clearance envelope. Adopting that section in track,
hardware, train clearance and station fit together remains a separate geometry
migration. The frozen default.3 save, package selector and launch identities are
unchanged.

The user rejected both the initial overall structures and the coped pillar head.
Those renders and the former assembly implementation are recorded in
`out/support-study/rejected-v1`. Numerical acceptance does not approve their form.

The revised joint uses a full-diameter 750 mm column, round mating flanges and
a 1.22 m long stiffened saddle. A 60 mm seat plate, two 50 mm transverse webs
and two 55 mm longitudinal cheeks distribute the visible connection along the
spine. Weld edges follow the actual curve and roll around each native attachment.
The standalone mount is 600 mm below the underside of the spine.
It avoids extrapolating a circular cope outside the narrower spine. The structure
preview also now uses the real Exa gussets; its former simplified tie bars left
a gap above the spine. Rails remain at 105 mm radius with centred crossheads.
This revised connection is a review model; it has not received user design
approval or replaced the native contact solid.

## Fabrication modelling pass

`native/art/support_detail_geometry.py` builds the actual mesh buffers used by
`author_support_study.py`. Each full structure now has:

- Track mounts oriented from the exact native attachment frame. The column is
  at least 750 mm in diameter, or wider when its incoming member requires it.
  The incoming stem continues as one surface into the mating flange, avoiding
  a seam between separately phased cylindrical meshes. The full-size head,
  seat plate, bearing webs and cheeks follow banking and inversion together.
  Inclined incoming members retain their native axes and meet a separate stub
  on a common mitre; the elbow reserve leaves space below the flange. This path
  is exercised by 93 heads on the regenerated retained ride, including an
  88-degree connection shown from both sides in a dedicated Blender scene.
- Bolted pipe splices on long, substantial members, with a nominal maximum
  12 m section length. Circular flange faces stay flat, with separate hex nuts
  and washers. These are fabrication details, not extra frame members.
- Horizontal bearing plates and anchors on the generated foundations. Inclined
  steel ends are cut to the plate plane. The pedestal top widens where necessary
  to contain the elliptical pipe end and anchor margin, while retaining the
  exact original base radius and terrain footprint. Anchor locations avoid the
  actual incoming steel. This fixes the open wedge beneath a perpendicular-cut
  pipe resting on a horizontal plate.
- Hollow track pipes and centred crossheads at 1.40 m intervals along the
  exported track distance. Gusset plates use flat face shading.

The standalone joint has a 600 mm mount drop. Full-structure mounts fit the
available generated contact length (540 mm drop in the current examples).
The art's wider terminal stems, plate hardware and enlarged pedestal tops are
not native collision members. Original terminal solids remain hidden in the
scene; each native member's original data and the complete export are retained
as metadata. Runtime clearance adoption remains open for the complete Exa
profile and these fittings together.

Run `python native/art/verify_support_details.py` for the nine examples and the
regenerated retained ride when its export is present.
`verify_support_junctions.py` independently checks fitted structural buffers and
compares the retained dense track samples against the rejected revision. The
support graph intentionally changes in this redesign; old support-graph equality
checks do not apply.
Checks cover closed finite mesh parts, full-width heads, spine seating,
upright/inverted frames, planar pipe bases, anchor space and unchanged foundation
ground footprints. These checks do not constitute rider-clearance adoption of
the added art geometry.

## Current redesign

The user rated the preceding support form -5/10. Its source, data and images are
preserved in `out/support-study/rejected-20260929`; numerical success did not
approve that appearance. The execution plan is `docs/support-redesign-plan.md`.

`fitted_support_members` uses the real shared graph to choose primary tube pairs,
build a common mitre ellipse, and fishmouth secondary branches against the
primary cylinders. Ring phases are matched by radial direction, preventing
twisted strips on sloping terrain. Pipe bases retain their planar bearing cut.
Each generated scene includes overview, broadside, ground, track mount,
foundation and structural-node cameras. Blender is restricted to four logical
CPUs and at most four render threads; live material rendering is off following
the user's CPU limit request. The configured GPU handles path tracing.

`out/support-study/junction-checks.json` checks closed fitted buffers, common
seams and unchanged track samples across all nine cases. Current native tests
and retained-save regeneration are recorded separately from the receipts below.

The current eight native checks passed in `tests-v3-final.log` plus the focused
`test-family-v3-repair.log`. The repair scopes an ordinary-bent shape assertion
to ordinary families; full sweep and graph checks remain universal. Fresh
acceptance, save and independent reload of the retained ride passed, with 287
attachments and 62 shared attachments (`retained-save-v3.log`). The canonical
default.3 hash remains unchanged. `fabrication-checks.json` covers all nine
fixtures plus that retained ride: 343,372 closed parts, 51 inverted mounts and
93 inclined heads. The native overview uses exact member solids and simplified
track context; it is distinct from the four full-profile fabrication scenes.

The final `scene-audit.json` checks five full-profile Blender scenes, including
the isolated inclined head: 99 mounts and seven cameras per scene. Their
embedded native exports match the source files by length and checksum. Sampled
rail radii remain within 0.14 mm of 105 mm in Blender's float32 mesh buffers.
`verification-v3.json` reconciles these checks, the native tests, retained-save
hashes and 31 rendered support views. The separate Exa component kit uses the
same saddle geometry and has refreshed external review renders.

## Historical verification receipts — rejected support revision

- `out/support-study/rejected-20260929/scene-audit-previous.json`: the previous four Blender scenes retained all
  native member metadata, full input-data checksums, 90 fitted mounts and three
  cameras each. Sampled rail radii differed from 105 mm by at most 0.016 mm in the
  float32 mesh buffers. The rejected models and renders are archived alongside it.
- `out/support-study/build-details.log` and `export-details.log`: rebuilt native
  review exporter and nine successful format 2 exports. No generator algorithm
  was changed in this fabrication pass; the runtime receipts below predate it.
- `out/support-study/mcp-details-*.log`: previous full-structure Blender authoring
  and overview/joint/foundation renders through MCP.
- `out/support-study/tests-v2-final.log`: eight focused native test cases passed,
  including the nine terrain/family examples and accepted save round trips.
- `out/support-study/native-build-v2-final.log` and `unreal-build-v2.log`:
  native and Unreal builds succeeded.
- `out/support-study/game-runtime-v2/result.json`: the same review save completed
  a 181.8-second front-seat traversal and pause/restart/cancellation checks, with
  2560×1440 captures. Keyboard interaction and visual design approval remain
  untested; no standalone package was produced or activated.
- `out/track-study/unreal-import.json`: the previous local art joint imported
  into the separate study map. The current saddle has not been imported there.

The first runtime review also passed numerically, but its shapes were rejected.
The `v2` receipts describe that historical revision, not the current generator.
