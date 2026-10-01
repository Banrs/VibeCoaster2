# Active handoff — Riftwake default.3 baseline

Updated 1 October 2026. Default.3 is the working geometry baseline, rated
about 5/10; default.4 was rejected, rated about 3/10. Neither is a finished design.

## Online diagnosis handoff, 1 October 2026

The user requested cleaning and pushing the current local state because it is
buggy and the online version needs to understand the flagged issues. This is a
source/evidence handoff, not a design fix or user acceptance. Treat all items in
**Latest user requirements for the next task** below as open, including forces,
flow, banking, layout/reference fidelity, drives/brakes, FVD transitions,
clearance cost, rider obstruction and loading time. Numerical acceptance and
previous passing tests do not resolve those complaints.

Start with [the portable diagnosis guide](docs/ONLINE_DIAGNOSIS.md). It includes
hash-verified exact default.3 and separate adapted-support saves, selected current
Blender/Unreal views, and prior verification receipts in Git. The new train's
editable Blend and GLB are included; it is still an art proposal and is not in
the development runtime. Large local support/track Blender reviews, generated
renders, build output and playable profiles remain ignored and preserved locally.
Default.4 remains rejected. Active save, package selector and Play shortcut
identities are preserved.

## Current source and next edit

- **Latest train modelling, 30 September.** The user requested a new train, then
  corrected the wheel-clamp orientation and said there was too much support.
  `native/art/TRAIN_DESIGN.md` documents the new seven-car/fourteen-seat proposal,
  authored through Blender MCP. Current `riftwake-train-02` uses compact C clamps
  in the transverse YZ plane, open inboard, with 100 mm thickness along travel;
  the tall longitudinal cages are removed. The model has a low nose, clear aero
  lip, contoured seats, individual lap restraints, ventilated rims and couplers.
  Editable source: `native/art/exports/train/Riftwake-Train.blend`, scene
  `Riftwake / Train design`; portable train-only `Riftwake-Train.glb` is beside it.
  Width is 2.35 m on 1.40 m gauge. Actual mesh checks pass: 495 unique closed
  evaluated parts (curve cap seams welded for checking), no straight-track
  triangle intersections, 42 mm conservative crosshead/gusset sweep gap, and
  132 clear front-seat sight rays. Renders and receipts are in `out/train-model`.
  This is an art proposal, not user approval or full-ride/station/hardware
  validation. The development runtime still uses position markers. Canonical
  default.3 save, package selector, Play shortcut and prior support Blender source
  are unchanged. Blender is left on the train, solid viewport, eight CPUs,
  BelowNormal, eight render threads and 16 samples.

- **Latest revision: resource adaptation, 30 September.** The user asked to adapt
  the researched resources and use best judgement. `support_bridge.hpp` now
  separates height/span sizing, local rail mounts, three-chord girders, and
  terrain-fitted trestles. This is original native code inspired by the reviewed
  authoring patterns; no GPL implementation or restricted Rollygon asset is bundled.
  The twisted drop has two main trestles and low end seats (10 foundations,
  previously 25). Loop/Immelmann upper crowns are carried from the shoulders,
  leaving their centres open. Lower direct rakers remain, with a validated
  direct-frame fallback for obstructed sites. Camelback necks now have a constant
  substantial diameter. Cliff wall brackets and accepted track geometry remain.

  Two visual iterations were reviewed: the refinement curves chords along the
  native path at 6 m intervals, preserves large 24 m bracing bays, and consolidates
  rear feet. `out/support-adapted/final/regenerated-supports.vcdesign` passes full
  acceptance and reload. Counts: 261 attachments, 57 shared sites, 269 footings,
  7 rock anchors, 2,181 members. Nine focused tests pass (174.65 s), including new
  resized/rotated banked-hill terrain cases. The exact mesh audit checks 348,544
  probes, with zero exposed ends above 12 mm. Unreal builds and completes 13
  isolated external views in `out/support-adapted/runtime-verified` (not a full
  traversal). The first empty pre-created capture directory was rejected by the
  runtime freshness guard; the fresh-directory rerun completed successfully.

  Blender MCP saved `native/art/exports/support-system/Riftwake-Exa-Adapted-Review.blend`
  (335 MB), scene `Riftwake / Exa generated ride`, 754 objects, opening on the
  twisted-drop oblique ground view. Support-only native export/import verifies
  24,819 common track frames and unchanged terrain; rail meshes are retained from
  the full `out/support-direct` export. `style_native_review.py` changes review
  materials/lighting only. Fifteen Blender views and the identity/check receipt
  are under `out/support-adapted`; start with `review.html`.
  All 27,009 canonical knots, 15 operations, canonical-save/shortcut hashes and
  package/profile identity are unchanged. This is **not user approval of appearance**.
  All test/build/runtime processes have exited. Blender is idle, solid viewport,
  eight logical CPUs (affinity 255), BelowNormal, eight render threads/16 GPU samples.

- **Preceding direct supports, after the user rejected branching arms.**
  The user clarified that the rejected arms were on the twisted drop/inversions,
  not the cliff. They then explicitly requested open-source resources. Research
  and limitations are recorded in `docs/exa-runtime-integration.md`: the MIT
  Unity generator assumes flat ground; the GPL NoLimits importer preserves authored
  beam/node data but does not plan structures; Rollygon offers editable geometry
  nodes and useful connector examples, but its product page restricts tool
  redistribution. No external implementation was copied into the native core.

  Current source: `support_twisted.hpp` uses direct bank-aligned bents, terrain
  footings, height-dependent dimensions/spacing and backstays for tall frames.
  `support_assemblies.hpp` no longer builds the rejected crown transfer fans or
  neighbouring remote branch hubs. Rolling inversion exits use substantial
  fallback members matching their neighbours. Camelback lattice has fewer,
  larger alternating bays; the user permits changing its outer form too.
  `support_cliff.hpp` uses one cantilever/socket for short reaches and one extra
  lower haunch above 12 m; actual cliff has six brackets/seven rock sockets.
  Thickened wall bearings include fitted native stiffeners. Steel, terrain,
  station and train-envelope checks remain in place.

  Current review: `out/support-direct/final/regenerated-supports.vcdesign` passes
  full native acceptance and save/reload. Complete native meshes are under
  `out/support-direct/ride/fixtures`. There are 243 attachments, 30 shared sites,
  291 footings, 7 rock anchors and 1,694 members. Nine focused tests pass in
  `out/support-direct/tests.log`; Unreal build passes in `unreal-build.log`.
  219,008 native mesh-end probes found zero exposed ends above 12 mm.
  The isolated Unreal review completed all 13 external views in
  `out/support-direct/runtime-final`; this is not a full ride traversal.
  Identity receipt confirms all 27,009 canonical knots and 15 operations,
  the canonical save hash, Play shortcut hash and package/profile are unchanged.
  These are geometry checks, **not user approval of appearance**.

  Blender MCP saved `native/art/exports/support-system/Riftwake-Exa-Direct-Review.blend`
  (347 MB), scene `Riftwake / Exa generated ride`, opening on the twisted-drop
  oblique ground view. `update_native_supports.py` replaces exact native support
  meshes while checking 24,793 common track samples and terrain equality over
  the required grid. The previous slightly wider terrain grid remains as context.
  All accepted rail dimensions and canonical path must stay intact.
  The `out/support-forms` branching checkpoint is rejected; do not publish it.
  `Riftwake-Exa-2030.blend` retains older scenes and comparison material.
  This chat owns coaster code and Blender; Fusion remains with the other chat.
  Limit heavy processes to affinity 255 (eight logical CPUs), BelowNormal,
  eight build/render threads and 16 GPU samples; solid viewport between renders.
  Native tests and the isolated Unreal process have exited; Blender is idle.

- **Rejected 30 September wall and twisted-drop revision (numerical baseline):**
  the user requested fewer inversion supports, a distinct twisted-drop structure,
  cliff supports harnessed into the wall, and reduced clearance. They confirmed
  that clearance means both a smaller track-to-cliff gap and embedded wall anchors.
  `support_cliff.hpp` fits the Exa cliff to the frozen track during explicit
  generation/regeneration, then builds short spatial brackets with three verified
  rock sockets each. The occupied train envelope is still checked continuously.
  The actual review has six wall brackets / 18 sockets, with no tall ground columns
  below that drop. `support_twisted.hpp` adds a bank-following triangular transfer
  truss on two braced tripods; the main camelback graph is exactly unchanged.
  The loop has 126 members versus 162 previously; the Immelmann 132 versus 163.
  Inversion attachment spacing is capped at 50 m. The complete native ride is in
  `out/support-wall/ride`, sourced from the separate accepted/reloaded
  `out/support-wall/final/regenerated-supports.vcdesign`. It has 247 attachments,
  43 shared attachments, 274 ground foundations, 18 wall sockets and 1,897 members.
  Nine focused native tests and the Unreal build pass; 274,560 tube-end surface
  probes found zero exposed ends above 12 mm. All 27,009 canonical knots and 15
  operations remain identical. Current Blender scene: `Riftwake / Exa generated ride`;
  no final .blend or combined receipt was saved before the user rejected this form.
  Native exports, intermediate views and logs are in `out/support-wall`.
  Earlier review scenes remain in `Riftwake-Exa-2030.blend`;
  historical scenes were unloaded from the live session to reduce memory usage.
  Keep eight-CPU affinity, low priority, and the canonical default.3
  package/save/shortcut identities unchanged. This remains a visual proposal.

- **Preceding 30 September 2030 support concept:** the user
  found the all-tall-element revision too dense and unattractive, and clarified
  that "2030" means experimental materials/design research, not a 20–30% target.
  References are inspiration; this larger coaster needs original proportions.
  The next Exa iteration uses tapered primary pylons, larger open bracing bays,
  a loop crown shared between two shoulder frames, and spatial upper branches
  carrying adjacent tall rollout mounts. Inversion attachments remain at most
  40 m apart. Bank transitions try real clearance-checked branch roots below
  the roll. The accepted track dimensions/path remain fixed. Current exports,
  ground renders and validation logs are in `out/support-2030`. The user's latest
  direction is to use the complete ride geometry from code, not author substitute
  geometry. The primary Blender scene is now `Riftwake / Exa 2030 generated ride`
  in `native/art/exports/support-system/Riftwake-Exa-2030.blend`: the native
  triangles of the complete saved ride, including its actual terrain. The
  `Exa 2030 / ...` fixtures are secondary regression studies. The review save has
  257 attachments, 54 sharing frames, 291 foundations and 2,142 members. Nine
  focused native tests and the Unreal build pass. The final full-ride tube-end
  audit covers 333,056 probes with no exposed ends over 12 mm. Host selection now
  includes the largest incoming tube, fixing the gap discovered on the real ride.
  `out/support-2030/review.html` and `verification.json` collect the final ground
  and connection views with reproducible source identities.
  `docs/exa-runtime-integration.md` records the research and its limits. The
  preceding `Exa-All-Tall-Elements.blend` / `out/support-refinement` are comparison
  material, not the current design target. Keep eight-CPU affinity, low priority,
  and the canonical default.3 package/save/shortcut identities unchanged.

- **Earlier 30 September support revision:** the user rejected the large single
  A-frame and protruding/short tube ends. The Exa generator now uses two braced
  trestles and a deep crown truss, full transverse bracing, shorter main-leg bays
  and raised toe joints above the foundation plates. Loop/Immelmann headers are
  continuous between paired rakers, with rounded knees. The steep upright
  approaches stay in the camelback region: world-Z of the up vector also varies
  with pitch, and the old threshold incorrectly left the lower approaches as
  independent tall columns. A dedicated steep-hill regression covers this.
  Junctions select continuous primary chords, fit secondary rings against actual
  tapered/faceted host meshes, and use closed node cans at blind forks. Blender
  preserves the native triangle diagonals. The 105 mm rails and accepted track
  path remain fixed. Current model: `native/art/exports/support-system/Exa-Support-Revision.blend`;
  five `Exa Revision / ...` scenes, external ground and joint cameras. Current
  gallery and verification are in `out/exa-revision`. The same generator and
  fabrication code feed the separately regenerated Exa ride and Unreal.
  `out/exa-revision/complete-frame/regenerated-supports.vcdesign` passes fresh
  acceptance and reload: 296 attachments, 94 sharing a frame without individual
  footings, 284 foundations and 2,583 members. All 27,009 canonical knots and 15
  operations are byte-identical. Nine native tests pass, including the steep
  camelback case. Native and Unreal builds pass. Ten layout mesh audits cover
  941,568 end-ring/edge probes with no exposed ends over the 12 mm threshold.
  Six final external Unreal ground/joint captures were inspected in
  `out/exa-revision/runtime-captures-complete`; this was not a full runtime
  traversal. The gallery collects 20 current Blender/Unreal images. The
  canonical save hash, selected package/profile and Play shortcut are unchanged.
  `support_review --save-review` now validates regeneration before export; it
  must not rebuild members after acceptance and leave a stale fingerprint.
  Earlier studies and passing receipts are historical, not user design approval.

- **Preceding 30 September integration:** the explicit `exa-1` profile drives native
  track dimensions, terrain/track/station clearance, support placement and Unreal
  meshes. `support_fabrication.cpp` generates the actual flanges, saddle webs,
  fitted junctions and seated bases for both validation and rendering. Legacy
  saves retain their legacy profile; independent spatial refinement preserves
  the chosen profile. The Exa underpass roof clears the deeper spine and operation
  modules sit between the centred crossheads. This remains a visual proposal.
  The separate `out/exa-runtime/regenerated-supports.vcdesign` passes fresh
  acceptance and save/reload: 287 attachments, 339 foundations, 2,033 members.
  All 27,009 canonical knots and 15 operations are byte-identical to default.3.
  Nine focused tests pass (`out/support-study/exa-final-tests.log`).
  Native and Unreal builds pass. Six external Unreal ground/joint views were
  loaded, rendered and inspected; this was not a full runtime traversal.
  `out/exa-runtime/review.html` collects the Blender and Unreal review images.
  `native/art/exports/support-system/Native-Exa-Review.blend` contains four new
  native mesh scenes alongside the prior studies. Ground/joint/foundation/node
  renders and receipts are in `out/exa-runtime`. Use `import_native_exa.py`
  through Blender MCP to import exact native geometry.
  The canonical default.3 save SHA256 and selected package/profile are unchanged.

- New modeling work starts with the [Exa track study](native/art/TRACK_STUDY.md),
  authored through Blender MCP from built and concept Falcon's Flight references.
  The user's latest rail size is 105 mm radius: the study uses 105 mm radius / 210 mm
  diameter hollow rails, 20 mm walls and a 680 mm spine at the retained 1.40 m
  gauge. The crosshead axis is centred on the rail centres (zero vertical offset).
  Pillars were enlarged to 750 mm diameter with matching connection plates and bases.
  The user's clarification targets the thin wrapper and contracted support neck:
  the coped tube heads were also rejected. The 750 mm column was wider than the
  680 mm spine, so extrapolating a pipe cope created exposed edges. The current
  revision ends the column flat at full diameter and uses a 1.22 m long stiffened
  saddle, 60 mm seat plate, transverse webs, longitudinal cheeks and round mating
  flanges. Weld edges follow the actual curved spine. Intamin's track-closing
  close-ups inform this connection. It awaits user design review.
  Editable components, three track samples and a separate Unreal review
  map are available. The explicit Exa review now integrates its dimensions;
  default.3's stored dimensions and launch identity remain unchanged.
- The [adaptive support system](docs/support-system.md) plans whole camelbacks,
  loops and Immelmanns from track frames and each seed's terrain. The user rated
  the previous supports -5/10 and supplied an external Exa concept render.
  The preceding redesign used continuous ground-to-crown main legs, outer chords
  and large triangular bays. Its short inversion headers and single crown joint
  are superseded by the revision above. Reference supports from
  third-person renders/ground photographs, explicitly not POVs. Original TRR
  ground photos are retained in out/support-study/reference. Rejected source,
  data and renders are archived in out/support-study/rejected-20260929.
  Do not treat earlier passing tests as visual approval.
  Member/terrain/station/rider checks and global endpoint connectivity validate
  the same members used by persistence and Unreal. Ordinary bents remain as a
  fallback. This changes generation code; loading default.3 preserves its stored
  supports. Native support review exports and editable Blender MCP scenes are in
  out/support-study and native/art/exports/support-system. Shapes remain review
  proposals, not approved final art or a structural stress calculation.
- The 29 September redesign also fits structural tube junctions with common
  mitre ellipses and fishmouthed secondary members. Ring phase follows radial
  direction to avoid twisted strips on sloping terrain. Bolted pipe splices,
  full-width track heads and foundation fixings remain. Inclined pipe ends seat
  on horizontal plates; pedestal tops adapt inside the original ground footprint.
  `native/art/support_detail_geometry.py` supplies the meshes. Format 2 native
  exports carry exact attachment frames and denser track samples; MCP receives
  the complete data without coordinate rounding. The four Blender scenes have
  overview, ground, broadside, track mount, foundation and structural-joint
  cameras in the interactive support review, including the reverse side of
  each track mount. A dedicated retained-ride scene checks an 88-degree incoming
  member with a fitted elbow; the full retained ride has 93 inclined heads.
  The historical Python study remains editable. The new Exa review uses the
  native fabrication port and checks its full visible geometry. Native graph,
  track and terrain inputs remain in scene metadata.
- The user increased the CPU allowance on 30 September to eight logical CPUs.
  Keep Blender and build/test processes capped at eight and BelowNormal priority,
  low sample counts and solid viewport shading. Live Blender reports eight render
  threads. Builds may use eight jobs. The PC has 24 logical processors.
- User is AFK and explicitly authorized coordination with the wind-tunnel chat
  `01a0eda3-d4fc-7393-b93a-192907068c1b`. That chat owns Fusion; this chat uses
  Blender and the coaster repository. Work independently and message only for a
  real conflict or dependency, not routine status exchanges. Do not touch Fusion.
- Earlier rejected train models, studies, review scripts, captures, FBX and source
  Unreal meshes are deleted. The new train art proposal is described above.
  Station and runtime hardware remain. Development builds show car-position markers.
- Only **1.40 m rail-centre gauge** and **maximum 2.40 m overall train width**
  carry forward; **2.35 m is acceptable**. No previous train/bodywork/wheel/seating
  or track-section proportions are requirements. See native/art/DIMENSIONS.md.
  The explicit Exa profile applies these dimensions to hardware, rendering,
  clearance and station fit together. The saved default.3 envelope is unchanged.
- Development builds have train-follow orbit/pan/zoom and detached free flight.
  Scroll adjusts third-person distance, moves free view in 5 m steps, and changes
  rider FOV from 30 to 110 degrees (H resets to 82). Free flight is 150 m/s;
  Shift boosts it to 600 m/s and increases free-view scroll steps to 20 m.
  Launch with native/unreal/scripts/inspect_geometry.ps1; controls are in
  docs/camera-controls.md. The frozen Play package does not gain source changes.
- Generation and loading now share acceptance_replay.hpp for coarse/fine/spatial
  replay, cancellation, worker lifetime and final acceptance. Support collision
  paths share a stack-backed member view. Motion audits search the relevant frame
  interval instead of rescanning all frames for each section.
- Removed the arbitrary flat-or-five-degree propulsion rule, clifftop/braking
  duration gates, and old boost/coast pacing rejection rules. Pacing measurements
  and warnings remain. Removed frozen layout/order/trim-location/reference-force
  assertions and duplicate manual numerical probes. Independent force,
  continuity, energy, clearance, refinement and persistence checks remain.
- CMake supports BUILD_TESTING=OFF. Stale core/Unreal graph caches and abandoned
  connection-probe output were deleted; inspect current source through the code map.
- This is preparation for the next geometry edit. It does not repair the ride's
  shape or establish a loading-speed improvement.

## Playable identity

The user requested a shortcut update on 28 September. Both Play shortcuts now
launch the current UnrealEditor game module with -game -CoasterLoad and the
separate UserData-GeometryReview profile, initially copied from the verified
default.3 save. Third-person/free view and the train-art removal are available.
Recreate them with create_shortcuts.ps1 -GeometryReview -Desktop -UnrealRoot
D:/Games/Epic Games/UE_5.8.

dist/current.json retains native/unreal/Packaged/run-20260924-231703-433 and
UserData-Riftwake-V3. The package records source 9631e488; its executable and
save are unchanged. The hashes below identify that retained package and baseline.

- Executable SHA256: DBB65AF1F0F6E9B54D05E2DB83B48994237F4E845580B244B609650B42A29EF4.
- Save SHA256: D48ED31CF62621405EE43A859C3E6E1EA468E405B383217FC7FB3E84895A0CF7.
- Canonical save: out/riftwake-completion-07.vcdesign.
- Prior packaged front/rear traversals and restoration receipts remain in out/.

## Verification

The current redesign passed eight focused native cases: support, support_family,
support_system, terrain_profile, dimensions, clearance, track_web and structures.
`out/support-study/tests-v3-final.log` records seven passes and an obsolete
ordinary-bent shape assertion applied to an element header. The corrected
support-family check passed in `test-family-v3-repair.log`; all collision and
anchored-graph checks still apply to both families. The native build passed.

The fabrication verifier now covers the nine support/terrain fixtures and the
separately regenerated retained ride: 343,372 closed parts, 51 inverted mounts
and 93 inclined heads (`out/support-study/fabrication-checks.json`). Inclined
incoming members retain their axes and meet the full-width stub at a common
mitre. Its required length leaves room below the flange. Structural fitting,
spine seating, foundation seating and unchanged ground footprints are checked.
`junction-checks.json` also verifies unchanged dense track samples across the
nine fixtures; `out/track-study/geometry-checks.json` retains the 105 mm rails.

The final Blender scene audit checks five full-profile scenes and 99 mounts,
including the retained inclined head. Native payload checksums match the exact
source exports, and sampled rail radii remain within 0.14 mm of 105 mm in the
float32 meshes. `out/support-study/verification-v3.json` consolidates the receipts.
The support gallery contains 31 views, including the simplified native overview
of the retained ride. The separate Exa component kit shares the new saddle.

The separately regenerated default.3 review has 287 attachments, 62 sharing
foundations, 339 foundations and 1,955 native members. Full native acceptance,
save validation and independent reload passed (`retained-save-v3.log` and
`data/regenerated-supports-report.json`). The canonical save retains the hash
above. No profile, shortcut or package was switched.

The `v2` native/Unreal build and 181.8-second traversal receipts describe the
previous, rejected support revision. No new Unreal traversal or art import is
claimed for this redesign. The complete Exa art envelope and these fittings
still need joint adoption in runtime collision, hardware and station geometry.

The refactored native CLI accepts the exact default.3 save. Before/after JSON
reports agree in every field except elapsed timings; the 0-180 km/h launch is
1.3970000017 seconds. See out/refactor-comparison.json and the adjacent reports.
All 26 native CTest cases passed (16 component, 10 integration). Both native
and Unreal editor builds passed, and all seven Unreal automation contracts
passed. The final recipe-independent organic test rerun also passed
(out/refactor-organic-final.log). See out/refactor-tests.log and
out/refactor-automation/index.json.

The rendered camera check against the rebuilt core passed paused orbit/pan/zoom,
free flight, independence from train motion and return to rider POV
(out/refactor-camera-runtime/result.json).
The subsequent scroll/speed update passed the rebuilt CameraContract and
rendered rider-FOV/reset checks (out/scroll-camera-automation-final/index.json
and out/scroll-camera-runtime-final/result.json).
Physical keyboard/mouse interaction and a new full traversal were not tested.
No standalone package was produced or activated. Train-art deletion is recorded
in out/rejected-art-deletion.json; cache and obsolete-build cleanup in
out/refactor-cache-deletion.json and out/refactor-obsolete-deletion.json.

## Latest user requirements for the next task

Treat every item below as open on default.3. Prior numerical acceptance is not proof of ride quality. Diagnose against the actual native outputs and game views, then make the smallest coherent changes that solve the user's design intent.

- Forces and flow: unnatural G forces; random flat resets and other flat sections; pitch, yaw and roll that get stuck at arbitrary angles; poor banking; missing intermediate elements such as S curves and useful connecting elements, at appropriate rather than excessive frequency; weak overall flow and coherence. Increase intensity where it belongs and reduce it where it does not, using sound coaster design principles.
- Reference fidelity: opening section and initial twisted drop are weak; the clifftop is not faithful to Falcon's Flight; the 180-degree return turn is in the wrong location and inaccurate in geometry and speed; the vertical loop and Immelmann do not faithfully scale Tormenta Rampaging Run's dynamics, especially yaw; the return after the inversions is poor.
- Drives and brakes: LSM boosters are misplaced and fail to use the available acceleration corridor, leaving dragged-out deceleration before a boost and an extended wait before the next element after it. Trim brakes are illogically placed. Measure these gaps on the actual replay and fix the ride's pacing and hardware placement together.
- Authoring and clearance: FVD authoring is weak compared with NoLimits 2 FVD and OpenFVD++, especially transitions between FVD and spline. Ground and other-track clearance detection is overly conservative, harms authoring and adds loading cost. Investigate false positives and cost before changing margins; maintain real physical clearance.
- Art and diagnosis: the coaster/train model obstructs the rider view. Use the new development inspection cameras for geometry diagnosis alongside rider views; capture the next geometry changes in both.
- Performance and product direction: reduce loading time as much as possible while retaining fresh validation. The long-term vision is a realistic, quickly generated coaster inside an amusement park where other coasters can coexist.

The 0–180 km/h launch in about 1.4 seconds is intentional, inspired by the now-closed Do-Dodonpa. Do not challenge or silently weaken it while solving the other issues.

## Research and verification to do in the next task

For the next geometry task, watch Falcon's Flight POVs and inspect its layout and 180-degree turn, inspect Tormenta Rampaging Run's layout and loop/Immelmann, and study relevant record-breaking coasters. Use the retained local reference audit and seek primary footage/layout evidence. Research NoLimits 2 FVD, OpenFVD++ and direct authoring guides to decide where FVD or spline fits and how transitions should behave. Mark uncertain inferences as such; do not force a layout from a single animation or consumer G overlay.

Inspect front, middle and rear force histories, speed/grade/bank/heading traces, hardware intervals, clearance results and actual rider/third-person screenshots. Check the opening, cliff, inversions, turnaround and return in sequence. If an old validation rule or hard-coded layout assumption conflicts with the user's latest design direction, investigate and revise it on evidence rather than letting it dictate a visibly poor ride. Keep independent physics, clearance and saved-design validation.

## Known performance limit and cleanup state

Default.3's final 100-warm packaged benchmark missed the 3-second target: request-to-ready median 3.150476 s, empirical p99 3.253885 s; process-to-ready median 4.849519 s, empirical p99 4.978452 s. All 100 warm runs exceeded 3 seconds. These are run-specific empirical statistics, not cold-cache or population p99 guarantees. See out/startup-final-100-20260925/evidence.md. The next task should prioritize measured reductions without skipping validation.

## Archive and navigation

Rejected default.4 packages, profiles, captures, backups and stale graph snapshots are in D:\Coding\Codex\vibecoasterlegacy\rejected-default4-20260925; see archive-manifest.json and graphify-archive-manifest.json. Older distributions, checkpoint media, experiments, scratch probes, inactive profiles, V072 art and superseded docs are in the adjacent workspace-deflation-20260925 folder; see its manifests. The earlier long handoff is archived as HANDOFF-before-deflation.md. These are reference records, not current instructions. The restoration receipt still records the backup’s original out/pre-default3-restore path; its files are now under rejected-default4-20260925/outputs/pre-default3-restore-20260925-132413-956.

Start with README.md, this file and docs/CODE_MAP.md. The optional Graphify
caches were deleted because they no longer represented the source. Default.3
delivery history is in docs/checkpoints/10-riftwake-completion.md; rejected
default.4 history is in docs/checkpoints/11-riftwake-fvd-redesign.md. Historical
reports and archive receipts do not override the current requirements above.
