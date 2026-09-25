# Force-authored redesign contract

The 25 September 2026 user rejection supersedes the completion-07 visual acceptance language in earlier checkpoints. That saved ride remains a rollback, not the accepted result of this redesign.

## Reference method

The [NoLimits 2 Force Vector Designer guide](https://nolimitscoaster.com/nolimits2/help/pages/forcevector.html) describes track creation from vertical force, lateral force and roll over time, with entry speed and simulated exit speed. [OpenFVD's force-section implementation](https://github.com/altlenny/openFVD/blob/master/core/secforced.cpp) inherits incoming force/roll state and integrates the resulting force vector, direction and energy. These sources inform the method; no source code is copied from OpenFVD.

For Riftwake, author force-critical ride features as saved normal-force, lateral-force and tangent-roll timelines. Integrate position and energy from the inherited physical entry. Preserve value and derivatives through chapter boundaries. An element name must not create an upright, level or zero-curvature reset. Keep independent source reconstruction and finite-train front/middle/rear force histories: a point-mass source passing alone is insufficient.

A vertical loop needs loaded lower arcs, a smaller-radius unloaded crown and one consistent displacement toward its exit side. Solve its complete frame and height from the force programme. Do not add alternating global yaw to make the crossing. An Immelmann reverses direction through the vertical half-loop and rolls upright; a loaded roll needs its support resolved into both rider axes to avoid unintended world-space lateral steering. Zero rider-lateral force is not equivalent to a planar path during a roll.

## Rejected baseline

Completion-07 contains nine FVD sources and 36 geometric spline programmes. Its main opening hill, cliff dive, downhill-launch pullout, protected camelback, wave, loop, Immelmann, return hill and final banked approach use FVD. The small early hills, climb transitions, clifftop, inversion spacer and signature are geometric. A programme count alone cannot establish coverage: constant-grade drive rails and hardware alignment may remain geometric, while loaded hills and banking transitions require force-led design.

The independent 60 Hz display-trace audit is `scratch/audit-redesign-trace.py`; its retained baseline result is `out/riftwake-completion-07-independent-audit.json`. Native acceptance remains at its existing higher rates. Before-clifftop time is 47.877 s; after-cliff-drop time to final stop is 96.643 s. The redesigned timing split must move active ride content earlier without adding straight coasting length.

## Verification required for the replacement

- Published category ceilings apply to actual generated vertical extent: loop 81.8388 m, Immelmann 99.6696 m, cliff drop 247.5 m. Measure and report the full element; do not rely on its nominal recipe height. Other element benchmarks and missing record categories are identified in the [reference audit](2026-09-redesign-reference-audit.md).
- Check inversion ascent peaks for every measured physical seat against the provisional Tormenta observations, about 4.344 g for the loop and 4.326 g for the Immelmann. Retain phase durations and existing project/F2291 limits. Keep camelback ascent, negative crown and recovery comparisons separate. The available reference records cannot prove exact pointwise force dominance for every feature.
- Inspect clifftop plan, height profile and third-person geometry for a slow, winding terrain-following sequence and an outward-bank move over the cliff edge. Do not substitute repeated standard airtime crests. Use both normal and lateral force to keep the intended path during the outward bank; inspect front/rear runtime views. A small numerical near-level percentage alone did not satisfy the previous user review.
- Examine closure pitch, curvature, banking and airtime transitions AFTER the inversions. No filler airtime belongs between Loop and Immelmann; compare their spacing with the appropriate Tormenta inversion pair. The user subsequently allowed a modestly longer interval for the taller inversions, so the reference timing is a proportional design guide, not a hard upper bound. A straight inclined interval can still be an unwanted flat piece between hills even if its pitch is nonzero. Preserve live curvature at joins and remove extended inactive intervals.
- Place LSM hardware from the real constant-grade start using its local physical alignment envelope. Keep per-car force, engagement ramps, full-train straddling and source-speed calibration. Do not make speed assignments or simply remove alignment validation.
- Remove redundant trim-before-boost placement. Retain useful off-design regulation, upstream sensing, rated brake force and actual deployment evidence.
- Complete native source/force/derivative/clearance/refinement checks, inspect terrain gaps by ordinary section, and verify an exact saved replacement in the standalone runtime before promotion.

## Native implementation checkpoint

Candidate11 implements the replacement and passes all native/reload checks. See [checkpoint11](../checkpoints/11-riftwake-fvd-redesign.md) for actual measurements, per-seat floors, conservative foundation-height cap, terrain/drive evidence and the remaining standalone review.
