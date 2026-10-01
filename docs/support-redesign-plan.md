# Support redesign — 29 September 2026

Historical plan and receipts. The user rejected this pass again on 30 September;
the current source, eight-CPU allowance and subsequent modeling revision are
recorded in HANDOFF.md and docs/exa-runtime-integration.md.

The user accepts the track and rejects the current supports at -5/10. This
supersedes the previous fabrication pass's visual direction. Numerical checks
are necessary but do not approve the appearance.

## Locked input

Keep the track path, 105 mm rail radius, 1.40 m gauge, spine and crossheads fixed.
Keep the playable default.3 save, selector and shortcuts intact. Native support
generation and Blender support geometry may change. Rejected source, exports
and renders are preserved in `out/support-study/rejected-20260929`.

## Diagnosis and reference direction

- The user supplied an Intamin Exa camelback concept render and explicitly
  requested third-person/ground reference views, not POVs. It shows continuous
  main legs rising from foundations to the crown, outer chords following the
  hill and a few large triangular bays. The current short piers and elevated
  angular bridge miss that load-path hierarchy. Adapt the supplied topology to
  the locked track, with continuous inclined primary members.
- Loop/Immelmann clusters branch toward distant elevated hubs, creating
  conspicuous triangular spiders above the track. Replace these with compact
  curved headers carried by spaced inclined bents, responding to each track
  frame without a surrounding cage.
- Untrimmed cylinder ends show chopped wedges, exposed caps and intersections
  at structural nodes. Model fitted junctions with continuous main members and
  properly terminated secondary members. Hardware must follow those joints.
- The track mount needs a compact, finished welded bearing, with consistent
  plate thickness and member diameter. Preserve the full-width stem.
- The user's later connection check adds Intamin's external track-closing
  photographs: use a short flanged stub and a substantial stiffened saddle.
  The revised cheeks must meet the curved/rolling spine, including inversions.

Primary reference is the user's Exa concept render, retained at
`out/support-study/reference/user-exa-camelback-render.png`, together with the
[built Falcon's Flight camelback ground photo](https://www.intamin.com/wp-content/uploads/2026/01/QIC_LSMLaunch_FalconsFlight_9.jpg)
and [Intamin's project page](https://www.intamin.com/project/falcons-flight/).
Use external Tormenta renders/ground views for the inversions. Do not infer their
support topology from POV footage. References guide form and member hierarchy,
not exact manufacturer dimensions or load certification.

## Execution and review loop

1. [x] Inspect current source, renders and primary visual references; preserve
   the rejected revision and lock track inputs.
2. [x] Replace the signature support topology in native generation. Export and
   render coarse forms before spending time on fabrication hardware.
3. [x] Inspect camelback, loop and Immelmann from side and oblique views. Revise
   awkward proportions, unsupported-looking spans, excess poles and poor nodes.
4. [x] Fit the actual support junction geometry and compact track bearing.
   Inspect close-ups from multiple sides; correct gaps, caps and intersections.
5. [x] Repeat across the nine terrain/heading examples. Check graph connectivity,
   full native clearance, deterministic generation and unchanged track frames.
6. [x] Inspect a regenerated retained-track review, run affected native tests,
   save the final Blender scenes and document remaining integration boundaries.

Stop the visual loop when the primary form is coherent in all three families,
the visible joints are clean, and another pass finds no concrete defect to fix.
Do not call it perfect or imply user approval.

The user subsequently requested low CPU usage, then explicitly allowed four
threads. Blender is pinned to four logical CPUs at BelowNormal priority.
Render requests allow up to four threads, the configured GPU and 16 samples;
the MCP runtime currently clamps the saved render-thread setting to one.
Live material preview is disabled. Run substantive checks sequentially.

Completed review: eight focused native tests, all nine terrain fixtures and a
freshly accepted/saved/reloaded retained ride. Ten fabrication cases cover
343,372 closed parts and 93 inclined heads. The live Blender audit checks 99
mounts in five full-profile scenes. Both Blend files are saved, with the support
file focused on a generated spine saddle. The final crown close-up was also
traced to its actual mesh faces: the apparent end-cap shape is the rear main
leg's side wall; the adjoining main legs share their complete mitre ring.
See `out/support-study/verification-v3.json` for the consolidated receipts.
