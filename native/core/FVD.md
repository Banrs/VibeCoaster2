# Force-vector section authoring

The current four-corridor integration remains an uncommitted experiment under [architecture review](../GENERATOR_ARCHITECTURE.md). Its side-specific load and canyon placement choices below describe that experiment, not approved requirements for the general generator replacement.

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

One ordered list of actual source occurrences owns identity, geometry, port poses and speed intent. All FVD sources retain their sampled canonical tangents, curvature and rider frames through the same rigid placement path. The connected cliff ascent → summit → dive → fastest launch → giant camelback progression is fixed; the loop, Immelmann and supporting airtime occupy physically compatible opening and return positions. [Architecture and scope](../GENERATOR_ARCHITECTURE.md).

Planar closure changes connecting lengths. Initial source selection therefore resolves airtime intent against the actual closed lengths before freezing those sources. Independent RK4 transport covers the previously conflicting order. The first real terrain/train assessment then sizes the final footprint; subsequent measured feedback holds that footprint and source geometry fixed.

FVD point-model speed remains part of the source's authoring history. The circuit's phase-speed reference instead uses the selected train's mean potential and the existing passive loss model. This distinction matters at an apex occupied by a long train. Loop and Immelmann pulse selection also uses the shared finite-train simulator and unchanged force evaluator over the allowed inlet-speed interval; lowering a peak indiscriminately can worsen its duration exposure.

One constrained solve owns rigid-source translations, C3 connecting profiles, level motor domains, the bounded shared station datum, passive train energy and nonlocal crossing separation. It uses the same proposed placement for all these constraints. A crossing's feasible over/under order is solved jointly; there is no post-placement lift, route-specific baseline repair or timing warp.

Climb and dive retain their selected rise and certified active windows on every terrain. Placement queries the actual transformed terrain rather than reproducing a canyon formula or requiring named sources on its rim. Source and station height budgets remain hard constraints. Source-local and placed chord lengths may differ by roundoff; active-window validation retains the same1e-8m constraint tolerance and exact certified rise/window.

Ordinary turns use their own maximum occupied train speed, complete roll ramps and the existing duration-dependent force budget. Short turns solve an attainable peak at that speed. Terrain bending shares the resultant budget, with a separate signed negative-load bound. Connector onset reserves the existing post-airtime transition requirement before allocating the remaining rate to vertical bending. These are authoring constraints; actual rider forces remain independently assessed.

Hills2 is the dedicated crossover fixture: canonical branches must be at least1,000m apart along the circuit and genuinely transverse, with complete clearance acceptance. Other fixtures retain crest roles, seeded variation, terrain adaptation and the under35% straight-flat-share regression. No source or turn parameter depends on a named seed.

Ordinary boosters have a nominal0.8g rating within selected limits. Length uses the existing force/power/governor law, rolling and aerodynamic losses, complete-train occupancy, ramps and exit fades. Each motor receives its own measured inlet through the same feedback process. [Zamperla's published1.3g Lightning launch](https://www.zamperla.com/news/lightning-lsm-coaster-at-pne-vancouver) grounds the ordinary acceleration scale; that three-car installation does not qualify this game's high-speed power or train configuration.

Physical rail lies inside its intended train-centre work domain by half a train at each end. The departure retains rail under the stopped train. Passive source observations occur after all cars clear preceding hardware. The height solve can recover surplus energy before a motor; downstream code does not repair it with a new trim or instantaneous speed assignment.

At most eight complete-ride geometry builds share one measured feedback path. Source inlet/apex/valley/exit speeds, maximum occupied turn/link speeds and each motor inlet must agree within0.5m/s. Coupled inputs use one midpoint update after the first terrain assessment. An unreached point supplies no observation; a partial trace can correct a measured motor inlet within the same budget. Cancellation remains explicit.

Generation-only phase diagnostics distinguish finite-train `sourceSpeedMps` from FVD point-authoring `sourceSeconds` and centreline force history. Actual front/middle/rear forces and passage times are separately measured. Display traces are60Hz; acceptance uses full-rate simulation. Final geometry, terrain, vehicle/hardware envelopes, structures, selected targets, exact-version persistence and960/1920Hz convergence remain mandatory. Qualified reference evidence, packaged POV and performance review remain separate.

## Verification

Build `coaster_core` and run the `fvd`, `layout_modules`, `force_envelope`, `passive_transfer`, `drive_profile`, `terrain_baseline`, `terrain_transfer`, `terrain_motion` and whole-circuit suites through CMake/CTest. FVD checks include analytic motion, independent energy/force reconstruction, actual step halving, C2 controls, reflection/similarity, physical crossing clearance, invalid inputs, cancellation and retained infeasible source cases. The baseline suite independently checks continuous bounds, station/source ownership, crossings, numerical rank, endpoint termination and derivative composition. Current evidence and limitations are in [VALIDATION.md](../VALIDATION.md).
