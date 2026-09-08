# Force-vector section authoring

`designFvdSection()` in [fvd.hpp](include/coaster/fvd.hpp) authors an **open, gravity-only point-mass section** from force and roll profiles. It is compiled into `coaster_core`; the `coaster_fvd` CMake interface target remains for compatibility. Active 0.7 generation uses `designFvdAirtime()` to supply four seeded airtime source sections. This is hybrid circuit authoring, not an entire FVD circuit or an independently accepted `Design`.

## Coordinates, forces and equations

All quantities use SI. The core has XY ground and Z up, with gravity vector `G = (0, 0, -g)` and `g = 9.80665 m/s²`. State consists of centerline position `r`, speed `v`, distance `s`, unit forward `T` and unit up `U`. The public right axis is **`R = cross(T, U)`**, matching `TrackSample.right`; for forward +X and up +Z, right is -Y.

`normalG` and `lateralG` are signed **non-gravitational specific-force components at the track centerline**, divided by `g`. They are not total world acceleration, finite-train forces, or forces at an elevated rider heartline. Positive normal points along `U`; positive lateral points along `R`. An upright horizontal 1g/0g section cancels gravity and stays straight. A 0g/0g section is ballistic.

For controls `n = normalG`, `l = lateralG`, and `rho = rollRate`:

```text
A       = G + g*n*U + g*l*R
dr/dt   = v*T
ds/dt   = v
dv/dt   = dot(G, T)
dT/dt   = (A - T*dot(A, T)) / v
K       = (dT/dt) / v
omega   = cross(T, dT/dt) + rho*T
de/dt   = cross(omega, e), for each frame axis e
```

There is no longitudinal propulsion, drag or rolling-resistance term. Constant speed on a horizontal straight or horizontal banked circle follows naturally because gravity has zero tangent component; it is not an imposed constant-speed mode. With no losses, `v²/2 + g*r.z` is the conserved specific mechanical energy; the result reports its numerical drift.

`rollRate` is physical **twist about the instantaneous tangent**, in radians/second. It is not the derivative of an Euler bank angle. For example, a constant-bank horizontal circle has vertical angular velocity and zero tangent twist. A complete frame is needed through vertical track and inversions; a world-up-derived Euler frame would be singular there.

Adjacent `FvdControl` values interpolate every channel with `S(u) = 10u³ - 15u⁴ + 6u⁵`, where `u` is normalized time between controls. Values and their first two time derivatives join continuously, with zero first/second derivatives at each control. Profiles are time-based only.

## Integration, canonical fitting and replay

[fvd.cpp](src/fvd.cpp) implements classical RK4 for position, speed, distance and a quaternion increment. The scalar-first quaternion obeys `dq/dt = (0, omega) * q / 2`. Stage rotations use normalized quaternions, while the quaternion derivative uses the raw stage value; the final increment is normalized. The same rotation acts on forward and up, preserving an SO(3) frame. Final orthonormalization removes rounding drift.

Each control interval is divided into uniform steps no larger than the requested `step`, with at least three steps per interval. Control boundaries are retained. Integrated position, tangent, curvature and physical up become canonical `Knot` values with bank zero. The existing `Track::rebuild()` fits its septic position and quintic reference-frame polynomials and enforces its unchanged canonical domain.

A separate **midpoint forward replay on that fitted Track** runs at twice the authoring time resolution. It integrates its own distance and gravity-coupled speed; target forces and stored speeds are not substituted for measured forces. At replay samples:

```text
dv/dt          = dot(G, Track.T)
specific       = v²*Track.K + (dv/dt)*Track.T - G
measuredNormal = dot(specific, Track.U) / g
measuredLateral= dot(specific, Track.R) / g
measuredTwist  = v * dot(Track.U_s, Track.R)
```

The assessment records maximum sampled normal/lateral/twist residuals and final distance/speed discrepancies. Defaults are 0.02g, 0.02 rad/s, 0.02m and 0.02m/s respectively. `integrated`, `canonicalBuilt`, and `assessment.performed/passed` distinguish the stages. A fit or residual failure remains a report error even when integration completed.

These are **sampled numerical diagnostics, not continuous bounds or an acceptance certificate**. They do not certify unsampled extrema, jerk, collision clearance or rider safety. `maxEnergyDrift` is diagnostic and is not itself a pass threshold.

## Bounds and cancellation

The API checks finite values, an orthonormal initial frame, strictly increasing controls beginning at zero, duration at most 60s, speeds in [0.5,250]m/s, position magnitude at most 100000m, curvature at most 0.15/m, forces within ±20g and twist within ±4pi rad/s. The integration step is in [0.0001,0.05]s; the sample budget is at most 50000. Angular increments above 0.1 rad are rejected with a request for smaller steps. Canonical fitting can impose additional restrictions.

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

Four sections use independently seeded push forces of 2.08 to 2.4g and crest forces of -0.22 to -0.08g at a nominal 65m/s. Source-section replay must pass. The generator then samples, joins, terrain-shapes and refits the geometry. Requested point forces are authoring inputs, not promised final rider-force traces. Final finite-train forces, drives, losses, rider offsets and all acceptance gates are evaluated on the joined circuit.

The adapter run passed 4,468 combined FVD checks with MSVC14.38 in `unreal/Saved/FvdIntegration/20260907-1/`, including independent energy, crest-force and level-port checks at three control settings. The first integrated flat42 circuit passed actual 960/1920Hz acceptance in `unreal/Saved/FoldedGenerator/20260907-3/`; packaged validation is separate.

## Remaining full-circuit FVD work

Full FVD generation still needs circuit closure and transition matching; force/power-limited drives and brakes with losses; finite-train speed/load coupling; rider offsets and their angular acceleration terms; and review of force/roll rates after canonical fitting. Every resulting full circuit must pass the existing geometry/structure clearance, finite-train simulation, actual step-halving convergence, target/reference, persistence and `Design::accepted()` gates described in [NUMERICS.md](NUMERICS.md). Section replay cannot bypass those requirements.
