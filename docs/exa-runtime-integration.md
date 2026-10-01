# Exa runtime integration — 30 September 2026

Continue the Blender support work into native generation, clearance and Unreal.
The accepted art section is 1.40 m gauge, 105 mm rail radius, 680 mm spine
diameter, 800 mm spine depth and centred crossheads. Keep the canonical default.3
save and selected playable identity intact; migration produces a separate review.

1. Add an explicit, persisted section profile. Old saves keep their original
   geometry. The Exa profile supplies the same dimensions to generation,
   terrain/track/station clearance and rendering.
2. Port the shared support fittings and fitted junctions into portable native
   geometry. Check the complete visible envelope, including flanges and bases.
   Keep own-spine seating narrowly scoped; rider and other-track checks remain.
3. Build a separate regenerated review, verify save/load and unchanged track
   frames, and exercise varied terrain and inversions.
4. Render the same native parts in Unreal, inspect external whole-structure and
   joint views, and compare with the Blender study. Fix concrete differences.

The user's updated limit is eight logical CPUs (the PC reports 24), low
process priority and GPU rendering. Builds may use eight jobs. Do not switch
the active profile, shortcuts or package.

## Current resource adaptation: shared girders and compact mounts

The user asked to adapt the researched resources into a system that works and
looks good. `out/support-adapted` replaces the repetitive direct bents with an
original native implementation of their endpoint/connector/footing separation.
No GPL importer code or restricted Rollygon asset is embedded in the game.

`support_bridge.hpp` sizes primary beams from terrain clearance and sizes
girders from actual span. It keeps local saddles separate from the ground frame,
with complete three-chord bracing bays. The twisted drop uses two main trestles
and two low end seats, reducing its foundations from 25 to 10. The upper loop
and Immelmann use shallower shoulder-supported crown girders with clear centres;
lower shoulders retain direct rakers. The first render exposed coarse polygonal
chords and over-wide back feet: the second pass follows the actual curve at 6 m
intervals and shares each trestle's rear foundation. Main lattice bays stay 24 m
or less. Camelback mounts now keep a substantial constant-diameter neck.

The separate accepted/reloaded save is
`out/support-adapted/final/regenerated-supports.vcdesign`. Nine focused tests pass,
including varied size, yaw, terrain, bank, cancellation, and deliberate flange
and rider obstructions. The native mesh-end audit checks 348,544 probes with no
exposed ends over 12 mm. These checks do not substitute for visual review.

`support_review --supports-only` exports all support fittings without rewriting
the unchanged rail meshes. `update_native_supports.py` imports those exact parts
only after checking common track samples and existing terrain coverage. A fresh
scene still requires the full export. `style_native_review.py` changes only
review materials/lighting; geometry continues to come from native code.

## Previous direct-frame revision

The user subsequently rejected the long branching arms specifically on the
twisted drop and inversions. `out/support-forms/final` is a rejected visual
checkpoint, despite passing native acceptance. Work moved to `out/support-direct`:
direct bank-aligned receivers and inclined main members, removing remote branch
hubs and the loop-crown transfer fans. The cliff and camelback are not the arms
identified in that clarification. No new result is user-approved.

### Open-source/resource review requested after the branching rejection

- [CMDRSpirit/RollercoasterDesigner](https://github.com/CMDRSpirit/RollercoasterDesigner/tree/main/Scripts/Supports), MIT: read both `TrackSupportGenerator.cs` and `TrackSupportPart.cs`. It places prefabs at spline intervals, mirrors their angle with roll and stretches members toward world Y=0. Useful separation of placement/parts, but not terrain-aware and not a ready solution for Exa superstructures. No code copied.
- [bestdani/blender_nl2pro_supports](https://github.com/bestdani/blender_nl2pro_supports), GPL-3.0: inspected the importer and example XML. It preserves rail/free/beam/footer nodes, beam properties and prefab atomization. It is an authoring/interchange resource, not an automatic structural-layout solver. No runtime dependency or copied implementation introduced.
- [Rollygon B&M Support Generator](https://rollygon.com/tools/b-and-m-support-generator/): reviewed its creator documentation and connector/radius/combined-frame demonstration images. Endpoint-defined beams have separately adjustable connectors, flanges and footers; connector sizes respond to beam radius. The site calls its tools open sourced, but the product page asks users not to redistribute or resell the tool. Treat this as a free editable tool/reference, not a permissively licensed code library. No tool asset downloaded or redistributed.
- [Rollygon Premier Rides Support Generator](https://rollygon.com/tools/premier-rides-support-generator/): operates on authored/exported support curves; it does not claim to solve a new terrain-dependent layout automatically.
- [KexEdit](https://github.com/IndividualKex/KexEdit) and [Coaster Mixer](https://github.com/mle-gall/coasterMixer): inspected project descriptions and repository file inventories. They address track authoring/simulation; no applicable support-layout implementation was established. They are not replacements for this task's support planner.

Design inference from those resources: keep primary members direct, localize
track connectors at member endpoints, and size detailing with the actual tube.
That supports removing the invented long forks. It does not supply certified
sizes or a ready-made automatic design for this much larger coaster. The native
terrain, train-envelope and fabrication validation remain authoritative.

The user rejected the first wall/inversion/twisted-drop pass in `out/support-wall`:
the wall arms look too skinny and the inversion and twisted-drop structures look
wrong. They then clarified that the camelback's outer geometry is right but its
internal lattice must also change. The new source iteration is in
`out/support-forms`. The wall-pass acceptance and mesh receipts are historical
geometry checks, not approval of its appearance.

Guidance checked at the user's request:

| Source | Application to this model |
| --- | --- |
| [NoLimits 2: Supports Tab](https://nolimitscoaster.com/nolimits2/help/pages/supportstab.html) | Plan rail, beam and footing nodes explicitly, then join them. Its automatic bevel guidance reinforces checking actual end surfaces rather than overlapping primitive cylinders. |
| [NoLimits 2: Supports Panel](https://nolimitscoaster.com/nolimits2/help/pages/supportspanel.html) | Set real pipe diameters, match connector size to incoming members, orient footings, and treat flange/offset choices as separate from structural topology. It gives editing controls, not an engineering prescription for this coaster's dimensions. |
| [Planet Coaster 2 artists interviewed by Creative Bloq](https://www.creativebloq.com/3d/video-game-design/how-planet-coaster-2s-artists-made-hollywood-work-as-a-theme-park) | Frontier's artists discuss substantial proportions, shapes readable from a distance, and reducing clutter from small repeated details. Apply that to primary/secondary support hierarchy. This is direct art-process guidance, not a support sizing standard. |
| [Planet Coaster builder's own custom-support project](https://www.reddit.com/r/PlanetCoaster/comments/1m68ge9/custom_supportsso_tedious_yet_so_worth_it/) | The creator describes using a real ride as a reference, standard cylindrical members, flanges and scaled connectors. Use reference proportions and connected parts; their project is not a source of certified member sizes. |

The new native proposal uses fewer substantial inversion frames with fore/aft
backstays, locally branching frames along the banked drop, thicker wall arms
with stiffened bearing connections, and a regular alternating camelback lattice.
The camelback primary-leg and outer-chord positions remain fixed. Track profile,
canonical path and the smaller cliff gap remain part of the review.

## Preceding 2030 design direction

The user wants a less crowded, original support silhouette for this larger ride,
using existing and experimental design research as inspiration. The preceding
all-tall-element model was judged too dense and unattractive. The current concept
uses tapered primary pylons, larger open bays, two shoulder structures carrying
the loop crown, and shared spatial branches along tall rollouts. The loop and
Immelmann are generated from their actual frames and terrain, including fixtures
with 2.5 times the earlier inversion radius. The accepted rail/spine dimensions
and canonical track path remain fixed.

The user's latest direction is to use geometry from the code directly. The main
Blender model is now the **complete saved ride**, imported from
`out/support-2030/ride/fixtures/saved-layout-fabrication.json`, with its actual
native terrain grid and triangle diagonals. It is saved as
`native/art/exports/support-system/Riftwake-Exa-2030.blend`, active scene
`Riftwake / Exa 2030 generated ride`. `render_native_ride.py` only places cameras;
the importer adds review lighting and a backdrop beyond the terrain boundary.
No replacement track or support layout is authored in Blender.

The separate accepted review save has 257 attachments, 54 without individual
footings, 291 foundations and 2,142 explicit members. The full native export has
175,719 mesh parts, imported into Blender with 7,609,908 shared vertices.
Full-ride inspection caught a large branch fitted against two smaller hosts;
requiring the host pair to include a largest incoming member fixes that exposed
end in both Blender and Unreal. The final tube-end audit has 333,056 probes and
zero exposed ends above its 12 mm threshold. Nine focused native tests and the
Unreal build pass. Current image/source receipts are in
`out/support-2030/verification.json`; the review is `out/support-2030/review.html`.
Ten final external Unreal views in `runtime-final` load and render the same
accepted review save. This run did not perform a full runtime traversal. All
27,009 canonical knots and 15 operations are byte-identical, and the selected
default.3 package/profile and Play shortcut identities are unchanged.

Secondary regression studies include a tall loop with 182 explicit members and 25 foundations, versus
276 and 33 in the preceding all-tall-element iteration. The tall Immelmann has
289 and 29, versus 517 and 39. Counts include secondary steel and foundations;
they are not a claim about stress capacity or material tonnage. Attachments are
retained at no more than 40 m along an inversion. Native generation, clearance,
Blender imports and Unreal rendering use the same meshes.

### Research used, checked 30 September 2026

| Primary source | Design interpretation and limit |
| --- | --- |
| [Intamin, Mahuka / upgraded Hot Racer](https://www.intamin.com/2024/06/17/mahuka-opening/) | Intamin attributes fewer columns to its backbone track design. The principle is to share support across a span. Mahuka's single rail, scale and dimensions are not copied into this accepted Exa track. Its documented carbon-fibre lap bars concern vehicle weight, not replacement structural pylons. |
| [Intamin, Falcon's Flight](https://www.intamin.com/2026/01/15/six-flags-qiddiya-city-falcons-flight/) | Large elements must be composed as whole structures. Ground/reference imagery informs the hierarchy of primary legs, secondary bracing and compact track mounts; it does not supply dimensions for this larger ride. |
| [Cast Connex, O'Hare branching columns](https://www.castconnex.com/news/steel-castings-realizing-architectural-expression-inside-chicago-ohares-satellite-concourse-1) | Custom three-dimensional steel nodes consolidate complex connections and support branching forms with fewer columns. This suggests a cleaner branching silhouette. Building nodes are not evidence of coaster fatigue performance. Current geometry retains its tested welded fittings; it is not represented as a cast-node engineering design. |
| [TU Delft / Coimbra, optimized WAAM steel T-joint, 2025](https://resolver.tudelft.nl/uuid:a23a800b-0653-40c9-bb12-e83379b8046e) | A manufactured and experimentally assessed optimized joint demonstrates a credible research direction for complex steel connections. It supports considering localized geometry optimization for a 2030 concept, not claiming production-ready printed coaster supports. |
| [SSAB, structural design principles](https://www.ssab.com/en/support/how-to-design/general-design/structural-design-principles/20-questions) | Higher yield strength does not remove stiffness, buckling or welded-fatigue constraints. This revision simplifies the arrangement and changes section proportions; it does not apply an invented strength multiplier or assume thinner tubes solve the whole problem. |

`out/support-2030` contains the current exports, external renders and logs.
The primary/secondary proportions are an art and geometry proposal. No dynamic
structural analysis or manufacturer validation is claimed. Earlier verification
below remains historical until a current receipt explicitly supersedes it.

## Earlier support revision

The user rejected the integrated model's large single A and poorly fitted tube
ends. The next pass changes the native assemblies and the exact shared meshes:

- Two trestles support a deep crown truss; both side frames have complete
  transverse bracing, shorter leg bays and raised toe joints above the plates.
- Exa inversion headers continue between raker groups, with rounded knee bends.
- Steep upright camelback approaches remain part of the signature frame.
  World-Z of the track up vector also decreases with pitch; the old threshold
  mistakenly reduced the full ride's region to 174 m around a 247 m crest.
  The corrected region spans approximately 666 m to the lower shoulders.
- Joint hosts follow primary-member continuity. Fitted end rings meet the actual
  triangulated, tapered hosts. Closed node cans receive blind forks without an
  opposing pair; foundation tubes seat independently on horizontal plates.
- Blender imports the native triangle diagonals exactly. The mesh surface audit
  probes both tube-end vertices and edge midpoints and confirms potential
  single-precision BVH errors using double-precision triangle distances.
  It reports exposed ends over 12 mm; it does not calculate structural stresses.

The current editable model is
`native/art/exports/support-system/Exa-Support-Revision.blend` with five
`Exa Revision / ...` scenes. `out/exa-revision/review.html` collects ground,
opposite-side saddle, structural-joint, foundation and runtime views. The
separate full-ride save is in `out/exa-revision/complete-frame`; it preserves
the original path and operations. `out/exa-revision/verification.json` records
the current hashes, graph counts and checks.

The saved graph contains 296 attachments, 94 without individual foundations,
284 foundations and 2,583 explicit members. The corrected full camelback has
29 attachments sharing eight foundations. Fresh native acceptance, saving and
reload pass. All 27,009 canonical knots and 15 operations are byte-identical.
Nine fixture mesh audits plus the full ride check 941,568 tube-end/edge probes
with no exposed ends above the 12 mm threshold.

Nine focused native tests pass after this revision, including a new steep
camelback regression. Native and Unreal builds pass. Logs use the
`out/support-study/support-revision-complete-*` prefix.
Six final Unreal ground/joint views at 2560 by 1440 were inspected in
`out/exa-revision/runtime-captures-complete`. The gallery includes 20 current
Blender/Unreal images plus the preceding camelback comparison. This runtime
check loaded and rendered the complete accepted ride; it did not traverse it.

## Preceding integration evidence

- `track_profile.hpp` defines Legacy and Exa. The optional `TRACK_PROFILE`
  extension persists `exa-1`; absent means Legacy and unknown profiles fail.
- `support_fabrication.cpp` supplies closed fitted members, full-width heads,
  curved-spine saddles, bolted flanges and terrain-seated foundations. Candidate
  selection and final structure validation test those visible meshes against
  rider/track/station/terrain bounds. Only the intended saddle weld region can
  meet its own spine. Rejected candidates are re-sited, not exempted.
- `track_mesh.hpp` supplies 105 mm radius rails with 20 mm walls, a 340 mm radius
  spine with 24 mm walls, and centred 1.4 m crossheads. Unreal uses the same native
  fitting meshes and retains its old adapter for Legacy saves.
- Exa operation modules sit in crosshead bays; the station underpass roof is
  lowered to retain clearance beneath the deeper spine. Spatial refinement
  preserves the selected profile.
- `out/exa-runtime/regenerated-supports.vcdesign` passed complete acceptance,
  serialization and fresh reload. Its 27,009 canonical knots and 15 operations
  are byte-identical to the baseline. The graph has 287 attachments, 339
  foundations and 2,033 members. `verification.json` records source/review hashes.
- Nine focused native tests pass: support, support_family, support_system,
  support_fabrication, clearance, track_web, track_profile, structures and terrain.
  Coverage includes nine family/terrain cases, closed positive-volume solids,
  unit normals, exact rail radii/walls, flange obstruction, rider/track intrusion,
  cancellation and profile mismatch. An unknown persisted profile is rejected.
- Four `Native Exa / ...` Blender scenes import the exact native mesh exports.
  Ground views and upright/inverted joints, a sloped foundation and a structural
  junction have been inspected. The saved file is
  `native/art/exports/support-system/Native-Exa-Review.blend`.
- Unreal builds successfully. Six external ground/joint views at 2560 × 1440
  loaded, rendered and were inspected. The receipt and images are in
  `out/exa-runtime/runtime-captures-framed`; this was not a full runtime traversal.
  `out/exa-runtime/review.html` collects all 15 Blender and Unreal review images.

The selected default.3 save, shortcuts and package are unchanged. This is a
separate model/geometry review, not a replacement train or a structural analysis.
