# Generator rewrite: design evidence and authoring decisions

Researched 6 November 2026 against the current [handoff](../../HANDOFF.md),
[inversion study](inversion-shape-research.md), [pacing study](ride-pacing-research.md)
and [September reference audit](2026-09-redesign-reference-audit.md).
The user's rewrite request supersedes the earlier preference for surgical patches.
The retained default.3 is evidence, not proof that its ride feels right.

## Falcon's Flight: summit and return are separate acts

**Verified primary description.** [Intamin's project page](https://www.intamin.com/project/falcons-flight/)
describes a 55 m twisted opening drop, preliminary curves/hills, a 150 km/h
cliff launch, an outward-banked cliff-edge turn followed by more turns,
holding brakes, the cliff dive/tunnel, a final 250 km/h launch, a 165 m
camelback, then elongated high-speed turns and hills. Its published ride
duration is about 205 seconds. It emphasizes progressive transitions and
alternation between intensity and recovery.

**Fresh visual inspection.** The manufacturer's
[layout image](https://www.intamin.com/wp-content/uploads/2026/02/LayoutDrawing01-scaled-e1773412395729.jpg)
shows a compact winding shelf at the cliff summit and a separate raised
return at the opposite end of the long layout. The camelback, cliff and
raised turnaround are visibly distinct structures. This perspective image
does not establish radii, elevations, speed, exact signed turn angles or
ground clearance.

**Implementation judgment.** Author the summit as a shallow, terrain-following
sequence: rim reveal, opposing sweep, smaller alignment turn, brake approach.
Keep the major 180-degree return after the camelback. The earlier four-crest
summit proposal was rejected; sub-1 g samples alone do not justify inserting
airtime hills. Let scenery-facing recovery have a purpose, without making it
a long anonymous flat coast.

The [official POV linked by Intamin](https://www.youtube.com/watch?v=ZiwyxnHF5hU)
was identified again. Fresh playback was unavailable: web fetch was throttled
and the workspace proxy rejected YouTube access. Consequently this note does
**not** claim a new frame-by-frame POV review. The retained September audit's
rim/sweep/alignment observations and consumer-video timestamps remain
historical evidence. They cannot be synchronized with this official POV or
treated as surveyed motion data.

## Tormenta: preserve inversion identity, not artificial yaw

**Verified dimensions.** Both the [park's attraction page](https://www.sixflags.com/overtexas/attractions/tormenta-rampaging-run)
and [B&M's announcement](https://www.bolliger-mabillard.com/blog/coming-soon-tormenta-rampaging-run)
publish a 179 ft vertical loop and 218 ft Immelmann, with an 87 mph maximum
ride speed. The park specifies a three-second first-drop hold. These are
published ride figures, not measured element-entry speeds.

**Fresh visual inspection.** [B&M's current Tormenta gallery](https://www.bolliger-mabillard.com/coasters/tormenta-rampaging-run)
includes an off-ride layout photograph and a loop close-up. They show
recognizable elongated inversion arcs and low connecting valleys. The
Immelmann's half-loop and rollout read as one maneuver. Perspective photos
cannot prove a zero-yaw tolerance, a lateral-offset distance or a force trace.

**Implementation judgment.** Retain the project's specified rider-left loop
crossing and near-parallel exit, with one coherent lateral displacement.
Author the Immelmann as a near-planar half-loop with a controlled half-roll;
do not add a yaw weave merely to make rider-lateral force zero. A roll changes
how a fixed world-space support force projects into rider axes. Allowing that
real lateral component can preserve the intended centerline. The earlier
[inversion study](inversion-shape-research.md) explains the existing failure.

For the retained 1.5-times-category size requirement, the published comparison
figures yield **81.8388 m for the loop** and **99.6696 m for the Immelmann**.
Measure the generated labeled element, not the whole ride. Near these limits,
check the datum: a published height is not automatically identical to
crest-to-lowest-rail extent. The prior provisional 70 m turnaround screen is
not a verified world-record category. No primary Falcon's Flight turnaround
dimension was found.

The retained [official Tormenta POV](https://www.youtube.com/watch?v=LesJ9BGNk8w)
observations put the loop and following Immelmann roughly 6.5 seconds
apex-to-apex, with broad uncertainty. Playback was unavailable here too.
Use that historical cadence as a comparison, not an exact timing constraint
or evidence that the current geometry matches the ride.

## NoLimits and OpenFVD: a shared boundary between authoring modes

The [official NoLimits 2 FVD manual](https://www.nolimitscoaster.com/nolimits2/help/pages/forcevector.html)
describes force/roll graphs over time, starting from the selected track end,
with a supplied entry speed and a simulated, read-only exit speed. Constant
speed is an explicit option for powered coasters. It supports relative and
absolute roll. This is useful evidence for carrying state between elements
and distinguishing authored forces from resulting geometry.

The [original OpenFVD README](https://github.com/altlenny/openFVD) documents
force-based and geometry-based sections, transition graphs, roll-speed
smoothing, and spline/tangent-preserving exports. The
[author-hosted FVD++ guide](https://lucasbosch.de/nolimits-tools/FVD%2B%2B/older_versions/fvd%2B%2B_0.5_documentation.pdf)
was located; indexed section 3.5 confirms transition-graph authoring for both
modes, but the full PDF fetch failed. No claim here depends on unread manual
details. The [revival fork](https://github.com/H27CK/openFVD) is archived;
fork names should not be mistaken for a current supported release.

**Implementation judgment, not a claimed feature of those products:**

- Use FVD for intentional load histories and geometry/splines for layout,
  station, clearance-constrained approach and closure. Neither mode should
  silently flatten the inherited pose.
- Pass one boundary state: position, continuous frame, speed, curvature and
  roll rate. Convert force targets to curvature using that actual speed;
  match endpoint derivatives when entering a spline.
- Preserve a continuous orientation through straight segments and inflections.
  Do not reconstruct bank from an unstable Frenet normal, or align rider-up
  blindly to a support vector that reverses during negative-G airtime.
- One canonical path must feed replay, rail meshes, train poses, clearance and
  persistence. Resampling or rendering must not invent a second track.
- Hardware occupies authored intervals. Design acceleration and brake distance
  with the approaching feature, then measure the actual idle gap on replay.
  Preserve the user's intentional 0–180 km/h launch in about 1.4 seconds.

These are original architecture recommendations. No OpenFVD implementation
or third-party ride asset was copied into the native code.

## Physical basis and a small set of feel regressions

[Pendrill's loop paper](https://lucris.lub.lu.se/ws/files/7485137/jubilee.pdf)
explains why circular-loop force transitions are problematic and discusses
clothoid and force-shaped alternatives. [Pendrill and Eager's jerk study](https://doi.org/10.1088/1361-6552/aba732)
explains why direction and changing acceleration matter alongside peak G.
[Jönköping University's mechanics derivation](https://mechanics.ju.se/Kinetics/RollerCoaster.html)
distinguishes curvature acceleration from rider load and couples speed to
height through energy.

Use `a = (dv/dt) T + v² (dT/ds)` and project `(a - gravity) / g` into the
rider frame. Evaluate changes against elapsed time. On an unpowered segment,
speed follows energy plus losses; a hidden speed floor conceals an unreachable
hill. Existing finite-train replay remains important because the front,
middle and rear do not traverse each crest at the same whole-train position.

Keep a small representative seed corpus and four behavior groups:

| Regression | Experience it protects |
| --- | --- |
| Energy and drive placement | Real acceleration into the next feature, no unexplained propulsion or long dead interval after a boost. |
| Pacing and load contrast | Meaningful airtime duration, loaded recovery, moving summit longer than lip wait, active return after inversions. |
| Element and join continuity | Correct loop hand, no Immelmann yaw weave, no arbitrary flat reset or abrupt bank/force jump at FVD/spline joins. |
| Playback and persistence | Consistent replay across timestep/render schedules and save/reload, using the same underlying path. |

Prefer broad physical/behavioral assertions and actual replayed durations to
full-track snapshots or more tests of implementation details. Consumer
accelerometer extrema in the retained audit are provisional comparison data,
not exact targets or manufacturer limits. Numerical passes do not replace
rider and third-person inspection of the resulting design.

## Experience targets and C3 continuity: 6 November clarification

The user permits small force-target excursions for a better-feeling, plausible
2030s design. Treat this as an authoring choice within a reported simulation
envelope, not evidence that future riders have greater physiological tolerance.
[NASA's acceleration review](https://ntrs.nasa.gov/citations/19980228043) identifies
restraint, direction, onset and duration as interacting factors.
[NASA's current human-system requirements, section 6.5](https://www.nasa.gov/reference/6-0-natural-and-induced-environments-vol-2/)
also distinguish sustained/transient exposure, posture, restraint and rotational
loads. Its crew/equipment assumptions make its numerical limits unsuitable as
drop-in amusement-ride comfort limits.

**Rewrite policy:** remove the fixed percentage acceptance cap. Nominal
vertical, lateral, longitudinal and component-rate values are configurable
experience targets, not universal human limits. Exceeding one should produce
a style diagnostic with the element, seat, sign, time, duration and peak.
Do not multiply all targets by a larger universal number. Choose the force
history for each intended sensation, retaining both its intensity and recovery.
Keep the intentional launch.

Preserve the existing [duration/axis assessment](../acceleration-standard.md):
`acceleration.cpp` separately filters the complete seat histories and checks
signed duration curves, paired axes, reversals and negative-G history.
Those findings, including impacts requiring assessment, remain hard acceptance
failures. For example, the adopted normal positive-Z curve permits a larger
short pulse than a two-second pulse, while its negative-history rule further
reduces subsequent positive loading. The same peak can consequently have
different outcomes depending on its duration and preceding motion. Separate
raw-path checks must still reject nonfinite states, geometric discontinuities,
unreachable motion and unresolved numerical error; a filtered trace must not
conceal numerical spikes. High-frequency variation and excessive rates remain
visible experience diagnostics even when the scoped assessment passes.

The assessor is explicitly pinned to selected **F2291-25** provisions and its
documented seating case. The publisher now lists
[F2291-26b](https://store.astm.org/f2291-26b.html) as active; its numerical changes
were not assessed here. Preserve edition provenance rather than relabeling the
existing implementation as current or certified. This research supports no
blanket claim of human safety.

For **C3**, the relevant trajectory is the passenger's position over time.
The following is a chain-rule derivation, not a standard prescription. On a
regular moving span with centerline `r(s)` and speed `v = ds/dt`:

```text
acceleration = r'' v² + r' dv/dt
jerk         = r''' v³ + 3 r'' v dv/dt + r' d²v/dt²
```

Thus C3 spatial position plus C2 speed in time is sufficient for continuous
centerline jerk. Continuous geometry alone does not repair a discontinuous
drive/brake acceleration. For a fixed seat/head offset `h`, evaluate
`p(s) = r(s) + R(s) h`; the same formula applies to `p`. Continuous passenger
jerk requires the composed path to be C3. C3 centerline **and C3 orientation**
are sufficient, but a tangent-derived frame normally inherits one fewer
derivative than the centerline. A regular C4 centerline with C3 banking is a
sufficient construction for that tangent-aligned frame; C3 centerline alone
does not guarantee C3 motion at nonzero seat height.

Carry endpoint derivative data across FVD/spline boundaries in one parameter,
using the chain rule to convert time and arc-length derivatives. Do not erase
incoming curvature or roll derivatives at a join. For the rider-frame force
`f = Rᵀ(a − gravity)`, its rate includes both translational jerk and frame
rotation: `df/dt = Rᵀ jerk − angularVelocityBody × f`. Accordingly, the project's
force-component rate is not simply world-space jerk divided by g.

Implement this continuity in the shared geometry/orientation/drive model and
use the existing full-seat replay to measure the result. C3 removes jumps;
it does not bound their magnitude. No larger regression matrix is needed to
make that architectural improvement.

## Direct force integration for the new compiler

A fast common kernel can integrate the heartline state directly, without
nonlinear geometry fitting. With unit tangent `T`, orthonormal rider up/side
axes `U,L`, gravity vector `g`, gravity magnitude `g0`, and authored normal and
lateral specific loads `n,l` in g units, use:

```text
positionDot = v T
speedDot    = dot(g,T) + driveAcceleration - resistance
tangentDot  = (g - T dot(g,T) + g0 (n U + l L)) / v
omega       = cross(T,tangentDot) + rollRate T
```

Integrate the complete frame with `omega`, preserving orthonormality. Explicit
drive intervals supply propulsion; stalls are reported rather than cured with
a hidden minimum speed. C2 load, roll-rate and drive profiles provide smooth
state derivatives on regular moving spans. Finite-train replay then measures
the actual front/middle/rear loads rather than assuming the authored heartline
targets hold identically for every seat.

Geometry/spline mode supplies its curvature to that same state model and
derives the corresponding force, while FVD mode derives curvature from force
and current speed. At a mode switch, convert derivative data and preserve the
whole state. Plan corridors and feature order first; use an explicit Hermite
bridge with inherited endpoint derivatives for a constrained approach or
closure. Its coefficients require a fixed linear calculation, not a search over
the whole ride. This supplies predictable forward generation while keeping one
path for physics, meshes and persistence. These equations are our point-mass
authoring model; they are not a reproduction of a manufacturer's design system.
