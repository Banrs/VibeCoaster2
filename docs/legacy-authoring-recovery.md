# Legacy authoring recovery

Default 1 immediately before the rewrite is the reference baseline. Its historical numerical acceptance is not current design approval. Useful unbuilt edits made after Default 1 are reviewed individually.

## Preserved baseline

- Packaged source: `a532c8c8aeb02e20ce27a31372556bc6cc9cfccc`.
- Restored branch point: `a2149dfb0c91a0264ae70bd49ff74bba1073f930` (same native source tree).
- Rejected rewrite remains on `codex/fresh-rewrite` at `15781504dbd0624ce2d461aceefca3636608c864`.
- Original archived workspace and accepted save remain untouched.
- Exact Default 1 save SHA256: `83E298F0BCB60855DB17D47B4D9ABEBAB7794863FEC1C954CF71D8A1FAC3DF4A`.

The restored package completed an isolated 2560×1440 front-seat traversal. Geometry SHA1 was `96BC4D0C623F2816870510337F8FD556146735F5`; duration 197.858 s. Automatic pause/restart/load/pose/cancellation checks passed. Keyboard interaction and design approval are separate from that smoke test.

## Open design requirements

- Preserve the reference camelback's planar asymmetric silhouette, including a single uniform scale and correct travel direction.
- Downhill LSM must feed directly into its loaded ascent. Check actual speed in the pullout, not just the powered interval.
- Correct loop, wave and Immelmann shape and handedness; do not treat their old generated geometry as an approved reference.
- Inherit physical position/frame/force derivatives. Do not clip signatures and silently repair them with arbitrary connecting splines.
- Check physical signed angular velocity, acceleration and jerk, plus explicitly intended reversals. Continuity alone does not establish intentional flow.
- Build a compact clifftop sequence with terrain contact, edge exposure, recommitment and brief anticipation. Do not add shelf distance to satisfy duration thresholds.
- Remove long inactive return travel by changing upstream composition. Banked level coasting must remain visible in diagnostics.
- Use modest coherent terrain and the valley station location; avoid manufactured rail-following benches and excessive secondary hills.
- Verify finite-train forces, energy, clearance, source replay and refinement. Numerical passes are not manufacturer or whole-standard certification.
- Review actual front/rear rendered traversals, commit the reviewed implementation, and pass Windows/macOS CI.

## Current implementation direction

The launch pullout replaces the early neutral ramp with one monotone FVD force ramp. It matches the reference's first positive-load shoulder in pitch, speed, curvature and physical frame derivatives. The remaining source is preserved within a uniformly scaled reference family; energy calibration may change that scale. Reference overlays must therefore compare normalized shape as well as actual size and placement.

Signature entry APIs retain complete incoming ports. Additive twist semantics are explicit and serialized; historical replacement semantics remain readable. Open energy-calibration prefixes use a named mode, cannot count as completed rides, and cannot be saved as accepted designs. Full closed-circuit replay remains mandatory.

New clifftop/lip/return timing and banked level-coast observations are diagnostics. They do not impose an old minimum-duration rule that could encourage filler.

Primary references: [Intamin Falcon's Flight description](https://www.intamin.com/project/falcons-flight/) and [OpenFVD force-section source](https://raw.githubusercontent.com/altlenny/openFVD/master/core/secforced.cpp). The latter informs inherited force-authoring concepts; no GPL implementation was copied.
