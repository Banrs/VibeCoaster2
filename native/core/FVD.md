# Force-vector section authoring

`designFvdSection()` in [fvd.hpp](include/coaster/fvd.hpp) authors an **open point-mass section** from force and roll profiles, with explicit optional rolling and aerodynamic losses. Defaults retain gravity-only behavior. It is compiled into `coaster_core`; the `coaster_fvd` CMake interface target remains for compatibility. Active 0.8.1 source uses `designFvdAirtime()` for four airtime sections and `designFvdPitch()` for the full loop. The joint reversal/pullout uses `designFvdReversal()` and `designFvdPullout()`. `designFvdTallHill()` now solves a height-constrained signature with loss-aware ascent and descent; its generator integration is under review. This is hybrid circuit authoring, not an entire FVD circuit or an independently accepted `Design`.

## Coordinates, forces and equations

All quantities use SI. The core has XY ground and Z up, with gravity vector `G = (0, 0, -g)` and `g = 9.80665 m/s²`. State consists of centerline position `r`, speed `v`, distance `s`, unit forward `T` and unit up `U`. The public right axis is **`R = cross(T, U)`**, matching `TrackSample.right`; for forward +X and up +Z, right is -Y.

`normalG` and `lateralG` are signed **non-gravitational specific-force components at the track centerline**, divided by `g`. They are not total world acceleration, finite-train forces, or forces at an elevated rider heartline. Positive normal points along `U`; positive lateral points along `R`. An upright horizontal 1g/0g section cancels gravity and stays straight. A 0g/0g section is ballistic.

For controls `n = normalG`, `l = lateralG`, and `rho = rollRate`:

```text
A       = G + g*n*U + g*l*R
dr/dt   = v*T
ds/dt   = v
loss    = rollingAcceleration + dragAccelerationCoefficient*v²
dv/dt   = dot(G, T) - loss
dW/dt   = loss*v
dT/dt   = (A - T*dot(A, T)) / v
K       = (dT/dt) / v
omega   = cross(T, dT/dt) + rho*T
de/dt   = cross(omega, e), for each frame axis e
```

There is no propulsion. The caller supplies constant rolling acceleration in m/s² and quadratic drag coefficient in 1/m; neither coefficient is inferred or silently enabled. For the current train model these are `g*rollingResistance` and `0.5*airDensity*dragCdA/(cars*carMass)`. Tangential loss changes speed without being counted as normal or lateral force. The energy audit checks `v²/2 + g*r.z + W`, where W is integrated dissipated work per mass. With default-zero losses this reduces to the previous conserved mechanical energy.

`rollRate` is physical **twist about the instantaneous tangent**, in radians/second. It is not the derivative of an Euler bank angle. For example, a constant-bank horizontal circle has vertical angular velocity and zero tangent twist. A complete frame is needed through vertical track and inversions; a world-up-derived Euler frame would be singular there.

For the tabulated API, adjacent `FvdControl` values interpolate every channel with `S(u) = 10u³ - 15u⁴ + 6u⁵`, where `u` is normalized time between controls. Values and their first two time derivatives join continuously, with zero first/second derivatives at each control. Profiles are time-based only. The reversal adapter passes a continuous internal evaluator through the same integrator and independent assessment. There is no mutable global control state or second integrator, and no dense zero-jet table substitutes for its continuous profile.

## Integration, canonical fitting and replay

[fvd.cpp](src/fvd.cpp) implements classical RK4 for position, speed, distance, dissipated work and a quaternion increment. The scalar-first quaternion obeys `dq/dt = (0, omega) * q / 2`. Stage rotations use normalized quaternions, while the quaternion derivative uses the raw stage value; the final increment is normalized. The same rotation acts on forward and up, preserving an SO(3) frame. Final orthonormalization removes rounding drift.

Each control interval is divided into uniform steps no larger than the requested `step`, with at least three steps per interval. Control boundaries are retained. Integrated position, tangent, curvature and physical up become canonical `Knot` values with bank zero. The existing `Track::rebuild()` fits its septic position and quintic reference-frame polynomials and enforces its unchanged canonical domain.

A separate **midpoint forward replay on that fitted Track** runs at twice the authoring time resolution. It integrates its own distance and speed from gravity and the requested loss law; target forces and stored speeds are not substituted for measured forces. At replay samples:

```text
dv/dt          = dot(G, Track.T) - rollingAcceleration - dragAccelerationCoefficient*v²
specific       = v²*Track.K + (dv/dt)*Track.T - G
measuredNormal = dot(specific, Track.U) / g
measuredLateral= dot(specific, Track.R) / g
measuredTwist  = v * dot(Track.U_s, Track.R)
```

The assessment records maximum sampled normal/lateral/twist residuals and final distance/speed discrepancies. Defaults are 0.02g, 0.02 rad/s, 0.02m and 0.02m/s respectively. `integrated`, `canonicalBuilt`, and `assessment.performed/passed` distinguish the stages. A fit or residual failure remains a report error even when integration completed.

These are **sampled numerical diagnostics, not continuous bounds or an acceptance certificate**. They do not certify unsampled extrema, jerk, collision clearance or rider safety. `maxEnergyDrift` is diagnostic and is not itself a pass threshold.

## Bounds and cancellation

The API checks finite values, an orthonormal initial frame, strictly increasing controls beginning at zero, duration at most 60s, speeds in [0.5,250]m/s, position magnitude at most 100000m, curvature at most 0.15/m, forces within ±20g and twist within ±4pi rad/s. Explicit loss coefficients are bounded to [0,1]m/s² and [0,0.01]/m. These are numerical authoring bounds, not equipment ratings. The integration step is in [0.0001,0.05]s; the sample budget is at most 50000. Angular increments above 0.1 rad are rejected with a request for smaller steps. Canonical fitting can impose additional restrictions.

Low speed is rejected explicitly; it is never clamped into a plausible trajectory. Cancellation is polled during integration, knot preparation and replay. The existing bounded `Track::rebuild()` call has no cancellation callback. Always inspect cancellation and report errors, not just the presence of samples or a Track.

## Build, tests and evidence

From the repository root, with CMake and a C++20 toolchain available:

```powershell
cmake -S native -B native/build-fvd -G "Visual Studio 17 2022" -A x64
cmake --build native/build-fvd --config Release --target fvd_tests
ctest --test-dir native/build-fvd -C Release -R "^fvd$" --output-on-failure
```

The compatibility target inherits the core include path. The core uses `/W4 /fp:strict` on MSVC and `-Wall -Wextra -Wpedantic -ffp-contract=off` elsewhere. The optional Zig build and Unreal wrapper compile the same `fvd.cpp` source.

[fvd_tests.cpp](tests/fvd_tests.cpp) passed **4,232 checks** with MSVC 14.44 in the isolated [native test output directory](../unreal/Saved/NativeMSVC/20260907-082510-018/fvd-section/), then independently with MSVC 14.38 through CMake/CTest: [CTest summary](../unreal/Saved/CMakeIntegration/20260907-fvd/fvd-ctest.log), [complete test output](../unreal/Saved/CMakeIntegration/20260907-fvd/Testing/Temporary/LastTest.log). These local `Saved` paths are run evidence, not portable checked-in fixtures.

Tests cover analytic level 1g straight, constant-bank circular turn, ballistic 0g position/speed, orthonormality, actual step-halving convergence for varying force/roll profiles, invalid inputs, low-speed rejection, cancellation, and deliberate canonical residual rejection. They are analytic/synthetic software checks and make no real-ride reference claim.

The first incorrect circular-test roll expectation is recorded in [first-test-output.txt](../unreal/Saved/NativeMSVC/20260907-082510-018/fvd-section/first-test-output.txt). A subsequent genuine midpoint-integrator force-fit failure is retained in [midpoint-residual-failure.txt](../unreal/Saved/NativeMSVC/20260907-082510-018/fvd-section/midpoint-residual-failure.txt). Quaternion RK4 resolved that discrepancy without relaxing the default residual tolerances.

## Production airtime adapter

`designFvdAirtime()` begins at a horizontal 1g port, smoothly raises normal force, transitions into the crest force, and solves the constant-crest hold until the track is horizontal at its apex. Mirroring the force history in time returns gravity-only integration to its entry elevation, heading and speed. Route planning uses its actual span; geometry is never stretched to fit a corridor. Guard straights and smooth force transitions provide level ports.

Four sections use independently seeded push forces of 2.08 to 2.4g and crest forces of -0.22 to -0.08g. Initial source entry speed is 65m/s; bounded whole-route feedback updates it from actual finite-train entry speeds. Each source-section replay must pass. Requested point forces are authoring inputs, not promised final rider-force traces: joining, terrain composition, drag and rider offsets can change the delivered history.

Shared transition polynomials match sampled endpoint position/curvature jets and raw-up/bank derivatives, then undergo canonical resampling and compilation. The resulting endpoint jets are approximate; regression checks bound rider-force disturbance outside the join and require improvement with finer sampling. Legacy guarded inversion adapters limit transition borrowing to their straight guards. Joint reversal/pullout sources instead meet at their actual descending pose; their interiors must not be replaced by a transition polynomial. Indexed motor zones remain physical operations. Terrain composition is still under review; guard preservation alone does not establish correct final forces or suitable element placement.

The historical first adapter run passed 4,468 combined FVD checks with MSVC14.38 in `unreal/Saved/FvdIntegration/20260907-1/`, including independent energy, crest-force and level-port checks at three control settings. The first integrated flat42 circuit passed actual 960/1920Hz acceptance in `unreal/Saved/FoldedGenerator/20260907-3/`; packaged validation is separate.

## Force-designed pitch and inversion modules (0.8.1)

`designFvdPitch()` solves a gravity-coupled planar half-loop. Its initial speed follows requested height and ideal apex energy. A quintic normal-force ramp rises from 1 g to the selected positive intent, holds, then transitions to the requested apex normal force. A bounded damped two-duration solve matches height and a pi-radian pitch change. Failed shooting, non-monotone pitch, low speed or failed canonical replay remains a reported failure; coordinates and speeds are not snapped or scaled to close the element.

The pitch equations follow the gravity/normal-force formulation of [Nordmark and Essen, Eq.(8)](https://arxiv.org/pdf/1007.1394). Curvature changes with energy instead of scaling a prescribed symmetric half-loop against the safety ceiling. The current 3.5 g positive source intent is a design input, not a new force limit or an ASTM allowance.

The retained legacy `buildEnergyReversingModule()` overlaps the half-roll with the final pitch arc; a -1g inverted horizontal source endpoint gives zero pitch curvature before the roll finishes upright. Reversing that actual path produces the dive. Source pitch generally has nonzero forward displacement `D`: the Immelmann endpoint is `D-rollLength` forward and `height` upward, and the dive reverses that displacement. The caller must consume the actual exit pose. The legacy geometric `buildReversingModule()` remains available for its existing callers and tests.

`buildEnergyLoopModule()` uses a positive source crest load (currently 0.5 g), reflecting the descending pitch to complete a full revolution without a half-roll. Its endpoint advances `2*D+2*portLength`; a smooth lateral offset separates its arms. The ordinary loop's seeded 55-68 m height is distinct from the selected terrain-relative inversion record, which the higher Immelmann must satisfy in final replay.

## Joint reversal and descending pullout sources

`designFvdReversal()` takes entry speed, exit height/pitch, normal intent, ramp timing, roll overlap, lateral shaping and handedness. Entry is level at the origin along +X. A bounded damped shoot solves hold duration, unloading duration and total twist against exit height, downward pitch and upright-relative-to-horizon orientation. Horizontal position and heading remain free. Results include actual canonical entry/exit kinematics, continuous intent at source samples, geometric apex height, highest sampled point with `up.z < -0.5`, minimum speed and timings. Exit height is not apex or inverted-record height.

During unloading, F varies smoothly from positive pull-up intent to `-cos(exitPitch)`. With quintic smoothstep S over normalized roll time u, body controls are `N=F*cos(pi*S(u))`, `L=-F*sin(pi*S(u))+lateralPulse*16*u*u*(1-u)*(1-u)`, and `twist=totalTwist*dS/dt`. Handedness changes lateral/twist signs together. Terminal N/L/twist are `cos(exitPitch)/0/0` independently of solved total twist, producing a straight-compatible sloping port. No frame post-rotation, endpoint warp or straight rolling tail closes the result.

`designFvdPullout()` consumes an actual upright descending entry sample, lower target height, normal intent and ramp time. Unsupported entry curvature is rejected rather than zeroed. A bounded two-duration shoot solves a short constant-slope passage and unloading ramp to reach a lower level valley while preserving entry heading. Results expose the tabulated authoring profile, canonical source, endpoint kinematics and timings. No propulsion is supplied.

Both solves have finite iteration/duration bounds and poll cancellation during shooting. Unreachable energy, infeasible ports, unsupported controls, non-monotone geometry or failed replay remain errors. The numerical authoring domain is not a safety envelope. Check the complete source report as well as `assessment.passed`: source-shape checks can reject an otherwise reconstructed section.

The focused production suite passed **5,276 checks**, including legacy analytic/tabulated cases, descending roll, mechanical energy, body-force consistency, parallel mirrored calls, endpoint derivatives, refinement, an infeasible shallow pullout, invalid inputs and cancellation: [test output](../artifacts/generator-intent-v081/element-energy/production-fvd-pullout-test.log). The isolated [prototype report](../artifacts/generator-intent-v081/element-energy/COUPLED-FVD-PROTOTYPE.md) retains exploratory and failed variants. Neither establishes final finite-train force histories or full-route acceptance.

Composition must preserve actual displacement, heading, slope and differential geometry, place the complete source consistently with terrain, and author continuation from that port. Physical similarity scales length/height by lambda and speed/time by sqrt(lambda), preserving dimensionless source forces; arbitrary coordinate stretching does not. Whole-train losses, seat offsets and subsequent turns still require actual replay.

## Height-constrained signature and explicit losses

`designFvdTallHill()` takes height, entry speed, positive/crest normal intent, ramp time and explicit loss coefficients. Its bounded family covers 220–280 m and 75–90 m/s when feasible. It first rejects requests whose entry energy cannot reach the height even on an ideal vertical ascent with those losses. A bounded hold-duration solve reaches the requested apex; an independent two-duration descent solve closes height and pitch. Exit speed, horizontal extent and the full canonical shape are outputs. No position scaling, mirrored loss assumption or exit-speed reset is applied.

The source keeps forward travel and one apex, with level zero-curvature ports. Actual finite-train passage remains necessary: the source is a point mass, and rider offsets and the occupied train's potential energy affect forces. In the focused 230 m / 75 m/s component, the default train reached the inlet at 75.208 m/s and produced −0.196 to 3.443 g across all three riders. Four component cases passed unchanged force checks and 960/1920 Hz convergence. They are open-component results, not accepted complete rides.

The production FVD suite passes the existing 5,276 checks plus 53 public-API regressions for analytic rolling/drag speed, distance and dissipated work; default-zero behavior; independently closed hill ports; impossible energy; invalid input and cancellation. Evidence is in [the loss-aware integration report](../artifacts/generator-intent-v081/loss-aware-production-v1/REPORT.md). Existing airtime, pitch, reversal and pullout adapters still use default-zero losses until explicitly migrated.

## Whole-route energy feedback and its limits

The generator permits at most eight feedback rebuilds at the selected route family/site. It compares four actual airtime entry speeds, the full-loop apex speed and the reversal exit speed with their authoring intentions. The maximum speed residual must be at most 0.5 m/s; otherwise `AUTHORING_ENERGY` rejects the candidate. This replaces the previous single correction. The reversal exit target applies after the roll, and should not be mistaken for an independently measured inverted-apex speed.

Feedback adjusts required trim energy and upstream relaunch energy separately, then rebuilds and simulates the actual route. A planning-only passive-transfer estimate uses whole-train mean potential energy, rolling resistance and aerodynamic attenuation over bounded 2 m spatial steps. It estimates how much energy can reach the trim exit without propulsion or braking; exhausted energy is reported as unreachable. It neither supplies a prescribed simulation speed nor verifies seat forces, terrain or ride acceptance.

Small speed residuals do not prove that final rider-force histories match point-mass source profiles. Terrain datum/connection quality, entry/exit transitions and final per-element loads still require review. Every resulting circuit must pass the existing geometry/structure clearance, finite-train simulation, mandatory step-halving, target/reference, persistence and `Design::accepted()` gates described in [NUMERICS.md](NUMERICS.md). Section replay and energy convergence establish no real-ride benchmark, F2291 compliance or completed game foundation.
