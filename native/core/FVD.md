# Force-vector section authoring

Working 0.8.3 uses independent force-authored source sections joined by terrain-aware routing. Source construction is an open point-mass problem; complete circuits use the separate finite-train simulator. Neither a reconstructed source nor a small speed residual is ride acceptance.

## Intent, reference and acceptance

Reference telemetry and POVs inform each element's phases, proportions and pacing. They are not force targets to copy exactly or acceptance ceilings. Curvature acceleration is `v²/R`: changing speed by a factor `b` and local radius by `a` changes that contribution by `b²/a`. Gravity projected into the moving rider frame must then be added. This relation does not scale total rider g directly.

Choose load and phase duration together. Raising a positive peak with the same impulse duration can make a hill unnecessarily high or turn it beyond its intended pitch. Gravity similarity scales length by lambda and speed/time by sqrt(lambda), preserving dimensionless centerline force; explicitly modelled losses and finite rider offsets still require replay. The [phase atlas](../artifacts/flow-intent-v083-20260910/references/element-profiles/ATLAS.md) separates observed recordings, source intentions and generated outcomes.

The stronger phases now use approximately +5g and −1.5g as nominal design intentions, allowing physically produced rider excursions within the separate acceptance profile. They are neither whole-ride averages nor clipping thresholds. The supporting sequence distinguishes strong airtime, mild floating relief, a still-positive crest and stronger closing airtime. Positive valleys remain connected. Actual FVD dimensions, phase duration, energy loss and finite-rider forces determine whether the intended sequence works; increasing an acceptance limit does not increase source forces.

The selected signed-duration, onset, combined-load and reversal profile is assessed independently on all three riders at 960/1920 Hz. Its historical source and enhanced-restraint requirements are documented in [FORCE_GUIDELINES.md](FORCE_GUIDELINES.md). Source authoring bounds are not a standards certificate or a substitute for the complete physical restraint design.

## Coordinates and equations

SI coordinates use XY ground and Z up, gravity `G=(0,0,-g)`, `g=9.80665 m/s²`. State is centerline position `r`, speed `v`, distance `s`, forward `T`, up `U`, and integrated dissipated work per mass `W`. Public right is `R=cross(T,U)`, so +X forward/+Z up gives -Y right.

Normal and lateral controls are signed non-gravitational centerline specific forces divided by g. They are neither world acceleration nor elevated finite-train rider loads. Upright horizontal 1g/0g stays straight; 0g/0g is ballistic.

```text
A       = G + g*normalG*U + g*lateralG*R
loss    = rollingAcceleration + dragAccelerationCoefficient*v²
dr/dt   = v*T
ds/dt   = v
dv/dt   = dot(G,T) - loss
dW/dt   = loss*v
dT/dt   = (A - T*dot(A,T))/v
K       = (dT/dt)/v
omega   = cross(T,dT/dt) + rollRate*T
de/dt   = cross(omega,e), for each frame axis e
```

The generator passes `rollingAcceleration=g*rollingResistance` and `dragAccelerationCoefficient=0.5*airDensity*dragCdA/(cars*carMass)` to every active source. Defaults remain zero for analytic callers. There is no source propulsion. Audit `v²/2+g*r.z+W`; drag is not silently added to normal or lateral force.

`rollRate` is physical twist about the tangent, not an Euler bank derivative. The full frame remains defined through vertical track and inversion.

## Continuous controls, integration and independent replay

Tabulated lateral and roll channels use quintic smoothstep. Normal controls additionally support an explicit first derivative, with zero second derivative at each knot. Quintic Hermite interpolation therefore preserves C2 force without requiring the load to stop changing at every authored control. The default zero slopes reproduce the original interpolation. Hill knees use nonzero slopes; an observation time does not become a new zero-slope control.

One quaternion RK4 integrator in [fvd.cpp](src/fvd.cpp) handles every source family. Each control interval retains its boundaries and at least three steps. Integrated position, tangent, curvature and up become canonical knots; the existing septic centerline and quintic reference-frame fitting remain authoritative.

A separate midpoint replay on the fitted track runs at twice the authoring resolution, integrating its own distance and speed. It measures `specific=v²*K+(dv/dt)*T-G`, projects that onto the canonical normal/lateral axes, and measures tangent twist from `v*dot(U_s,R)`. It does not substitute requested forces or stored speeds. Default residual tolerances are 0.02g, 0.02rad/s, 0.02m and 0.02m/s. These are sampled reconstruction checks, not continuous clearance or rider acceptance bounds.

Finite state, orthonormal frames, ordered controls, step/sample budgets and cancellation are checked. Unreachable energy, low speed, failed shooting and failed reconstruction remain explicit failures. No coordinate warp, speed clamp or endpoint reset repairs them.

## Source families

| Source | Solved behavior and composition contract |
|---|---|
| Tall hill | Requested height and force phases determine an independently solved ascent and descent with explicit losses. Actual span, exit speed and pose are outputs. The supported family is 220–280m and 75–90m/s when feasible. |
| Airtime chain | Continuous pull-in, unloading, negative crest and recovery phases; descent closes the physical valley under rolling/drag losses. Connected hills share their real state. The requested port ramp fixes the final unload duration; the last valley hold is solved around it. An undersized final hill remains infeasible. |
| Full loop | Jointly solve the complete pitch revolution, actual apex energy/height, and physical separation at the crossing arms. Twist produces lateral displacement through force; exit position and yaw remain outputs. No reflected descent or lateral geometric warp. |
| Immelmann and pullout | One force/roll solve owns the true inverted apex, overlapping half-roll and lower upright valley. Position and heading remain free. The roll exit is an internal curved descending checkpoint, without a separate straight port or hold. |

The planar `designFvdPitch()` remains an independent half-loop API; production full loops use `designFvdLoop()` through the layout adapter. The governing pitch formulation follows [Nordmark and Essen, Eq. 8](https://arxiv.org/pdf/1007.1394).

Full-loop `crossingOffset` means separation of the actual intersecting arms in the source's vertical projection. Endpoint offset alone is insufficient. Conservative canonical body sweeps must still clear; the failed endpoint-offset experiment is preserved in the [loop study](../artifacts/flow-intent-v083-20260910/fvd-chain/full-loop-study/REPORT.md).

## Whole-route energy and terrain composition

Rigid sources retain their physical displacement, heading, frame and internal shape. Terrain may translate them; connectors must continue from their actual ports. A loop's net yaw is propagated into its following launch and hill rather than undone by an offset warp. The tall source and full loop are solved once and reused during a candidate's energy feedback.

The tall hill and airtime chains also retain their sampled source tangent, curvature and up during canonical compilation. A shared source/turn vertex owns both modules' clearance requirements before datum placement. Low upright supports use the actual foundation-to-cap post requirement, rather than imposing an unrelated minimum cap height above terrain.

Terrain transfer sizing integrates local height, rolling resistance, drag and bounded motor work before assessing signed vertical-plane load. The ordinary terminal descent has its own −1 to +3.5g intent; selected acceptance limits do not automatically become every element's target. Full three-dimensional banking, finite-train motion and rider offsets are evaluated separately.

At most eight feedback rebuilds share one measured-port update path. It compares the signature's inlet/outlet, each airtime apex/valley/exit, loop inlet/apex/exit, reversal inlet/exit and pullout exit. Ordinary turn dimensions use the maximum actual speed while any car occupies the turn. The maximum source-speed residual is 0.5m/s. An unreached point supplies no observation.

Physical rail lies inside the intended train-center work domain by half a train at each end. The initial departure retains upstream rail beneath the stopped train. Thus source inlet observations are taken after every car has cleared the previous actuator. Trim settings and upstream motor supply are corrected separately; added supply can also decrease. A passive affine transfer in `w=v²` accounts for mean train potential, rolling and aerodynamic loss. It is an energy-planning calculation, never a prescribed simulation trajectory.

A provisional route holds its family/site while energy converges. One optional reranking uses the converged source dimensions within the same rebuild budget and retains the measured provisional route if the alternative fails. The actual selected route and any rejected alternative remain in diagnostics.

Per-phase diagnostics compare source and delivered speeds and report each front/middle/rear rider when that rider passes the landmark. Display traces are 60Hz; acceptance uses full-rate simulation. The final track must independently pass geometry, terrain, structural/envelope checks, all force assessments, selected targets, exact-version persistence and 960/1920 convergence. Qualified reference evidence, packaged POV and performance review remain separate requirements.

## Verification

Build `coaster_core` and run the `fvd`, `layout_modules`, `force_envelope`, `passive_transfer`, `drive_profile`, `terrain_transfer`, `terrain_motion` and whole-circuit suites through CMake/CTest. FVD checks include analytic motion, independent energy/force reconstruction, actual step halving, C2 controls, reflection/similarity, physical crossing clearance, invalid inputs, cancellation and retained infeasible source cases. Current component evidence and limitations are in the [continuous hill study](../artifacts/flow-intent-v083-20260910/continuous-load-study/continuous-family/CONTINUOUS_HILL_REPORT.md) and [VALIDATION.md](../VALIDATION.md).
