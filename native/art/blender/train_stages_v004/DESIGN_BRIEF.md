# Aero Wedge 2030 — original train concept, v004

The new silhouette is a compact high-speed machine: a narrow leading wedge, continuous sharp shoulders, exposed dark chassis and two sculpted rider pods. The old inflated oval hull, broad bumper, separate rounded bonnet and upright rectangular seat backs are removed. Pearl titanium and deep petrol panels with thin cyan details separate structure from padding; the shape carries the design rather than decorative lights.

## Primary reference and design reasoning

Intamin states that Falcons Flight uses purpose-designed windshields and spoilers, ergonomic seats with carbon lap bars, an aluminium-machined chassis and ventilated wheel rims. It also says that the track was widened for that project. Those are meaningful responses to high speed; painting an ordinary car silver would not address them. [Intamin: Falcons Flight, 7 April 2026](https://www.intamin.com/project/falcons-flight/).

Intamin’s LSM product page shows front, side, top and rear train views and describes ergonomic seating with over-the-shoulder lap-bar restraint routing. This informs the visible upper pivots, swept side arms, individual lap pads and open chest area. [Intamin LSM trains](https://www.intamin.com/product/lsm-launch-coaster/), [official perspective drawing](https://www.intamin.com/wp-content/uploads/2021/02/intamin-lsm-roller-coaster-train-perspective-1200x848.png), [official side drawing](https://www.intamin.com/wp-content/uploads/2021/02/intamin-lsm-roller-coaster-train-side-1200x848.png).

This is an original visual concept, not a replica or a claim of tested aerodynamic protection at 300+ km/h. Manufacturer text and published reference links were consulted; no external private project material was sent and no public POV video is claimed watched. Wind-deflector thickness and shape are art dimensions, not validated glazing or CFD results. No source images are bundled as textures.

## Concrete geometry choices

- The complete car retains the exact 2.55 × 1.70 m footprint. Maximum width occurs at the shoulders; the forward tip narrows to 0.40 m and reaches x=+1.275 m. Its leading top is only 0.205 m above rail. The wedge increases toward the cabin instead of carrying a bulbous full-width front bumper.
- The visible chassis reaches up from z=0.10 m. Thin side blades and seat supports show a load-path silhouette without pretending to be a structural calculation.
- Two narrow bucket spines taper into integrated head pads and swept head/ear cheeks. They leave a wider central gap than the previous backs, with no crossbar at eye height.
- Each seat receives a clear raked deflector, approximately 0.56–1.29 m above rail, and slim edge spars. Top centres are x=0.59 m, in front of the unchanged eye. The two shields remain separated across the centre POV lane; there is no opaque full-width windscreen hiding the circuit.
- Visible upper restraint pivots, carbon side arms, lap pads and recessed grips replace the old horizontal furniture-like bar. The closed pose is static art; no restraint controller is invented.
- The shared car body already gives the lead car a distinct narrow nose. A separate giant lead nose would consume the existing inter-car space or require a revised envelope. v004 therefore uses one coherent module for all six cars; no core change is proposed solely for styling.
- Source axes stay +X forward, -Y rider-right, +Z up. Origin stays the canonical rail midpoint. Eye stays (0,0,1.2 m); six car centres retain 3.4 m spacing. No speed, mass, force, gauge or clearance limit changes accompany the art.

The previous unvalidated wheel study is preserved in its own version. No bogie, extra under-rail hardware, oversized spoiler or out-of-envelope nose enters this runtime candidate. A physically widened track or new large-wheel chassis would require explicit core work; it cannot be implied by the visual reference.

## Staged build and promotion

Use the proven serial MCP workflow: execute numbered files 01 through 10 as separate calls, allowing each to return. In particular, do not combine construction, scene activation and dependency evaluation into one call. All v004 objects use `VCTrain4_`; v003 scenes and source assets remain untouched.

Root precreates:

- `native/art/source/train/20260908-train-v004/`
- `native/art/exports/train/20260908-train-v004/`

Scene: `VCTrain4_AssetReview`. Assembled mesh: `VCTrain4_SM_TrainCar_Assembled`. Exports: `train_car_runtime.glb` and `.fbx`. Source copies: `VibeCoaster_HighSpeedTrain.blend` and `_Review.blend`. Stage 05 prints measured bounds, actual part/triangle counts and material slots. Stage 10 prints the final manifest. The scripts require the above-rail body bounds and exact footprint before exporting.

Use original FBX vertices and the independently established UE importer convention (scene/unit conversion, force-front-X and import yaw +90 degrees), with a new destination and independent witness. Do not reuse the rejected explicitly reflected copies. No v004 asset is promoted by these scripts.

**Optical material gate:** `VCTrain4_WindDeflector_ClearCyan` uses thin transparent/transmissive shading in Blender. FBX material-name success does not prove equivalent UE shading. Root must set/inspect an appropriate transparent or translucent material, validate its sorting and outward normals, and view actual ride POV before promotion. If the shield becomes an opaque plate, the import is unsuitable.

Required visual review is front/side/rear silhouette, physical-eye front/middle/rear views, and a banking/inversion view. Check the nose taper, open centre sightline, shield optics, seat support contact and intermediate-car spacing. Blender still renders are concept evidence; packaged moving POV remains the acceptance evidence.

Python syntax and installed MCP safe-mode validation were checked locally. The modelling agent has not executed Blender, MCP or UE for v004.
