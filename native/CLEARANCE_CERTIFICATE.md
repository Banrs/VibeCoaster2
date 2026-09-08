# Canonical G3/C2 clearance certificate

Scope: `0.5.0-geometry.2`, COASTER5. This is a source/numerical certificate for canonical solids, not UE playback or triangle-mesh intersection verification. Measured release status is recorded separately in [validation](VALIDATION.md). The bounds below describe real arithmetic; the implementation adds coefficient-scaled floating arithmetic reserves and retains .012 m body / .004 m hardware margin. It is not a formal outward-rounded interval arithmetic library.

## Actual polynomial and frame domain

`buildClearanceSweep` rebuilds a canonical copy and compares total length, every span start/length, all eight position coefficients and all six raw-up/bank coefficients. A stale or nonfinite cache cannot establish coverage. No caller supplies sampled frames or a claimed angular rate to the opaque sweep constructor.

For each span, the degree-six P′, degree-five P″/W, and degree-four W′/bank′ power polynomials are converted to Bernstein controls. De Casteljau subdivision restricts their value hulls to a parameter interval. Derivative values remain derivatives with respect to the **original u**, so every rate is multiplied by that interval's original parameter width.

Choose the midpoint tangent direction A and midpoint raw-up direction B. The minimum projections of P′ controls onto A and W controls onto B give positive lower bounds qmin and wmin on their norms. Failure to establish a positive bound causes subdivision, then explicit rejection at the depth/budget limit. Let:

- M = maximum norm of P′ controls;
- K = maximum norm of P″ controls / qmin, bounding |T_u|;
- D = maximum norm of W′ controls; Wmax = maximum norm of W controls;
- h = interval parameter width / 2;
- c = |Tmid·Wmid| + (K Wmax + D) h, plus floating reserves;
- nmin = sqrt(wmin² − c²), required positive;
- Ω = K + (D + c K) / nmin + maximum absolute bank′ control.

The c bound encloses |T·W| throughout the interval, hence nmin encloses the length of W projected perpendicular to T. The unbanked frame's twist rate is at most (D + c K)/nmin; its bending is at most K. Adding actual bank′ therefore bounds the complete orthonormal frame angular speed. This uses the cached quintic frame itself, with no normalized-linear-up or sampled-rate premise.

Each certified node is partitioned uniformly until **M du ≤ .04 m and Ω du ≤ .08 rad**. Nodes whose projection cannot be bounded are subdivided further. There are at most one million cells, two million visited nodes and depth 30; work and member queries check cancellation. Each immutable frame records span, parameter endpoints, arc bound and angular bound for independent audit. The 16 m spatial index and existing conservative member broad phase remain.

## Continuous motion and member clearance

At any point of a cell, translation from its exact midpoint is at most .02 m and orientation change at most .04 rad. A local point of radius R moves at most .02 + .04 R (the chord rotation is no larger than R times angle).

The complete configured body has half-length1.275 m, half-width1.5 m, bottom−.8 m for terrain and top max(2.4, seatHeight+.6) ≤3.6 m. Its radius is below 4.2 m. The station body's slightly different layered boxes also fit that radius. Thus displacement <.188 m is covered by the unchanged **.20 m** body pad. Every canonical spine/rail/tie corner lies below .9 m radius, so displacement <.056 m is covered by the unchanged **.06 m** hardware pad.

Support tapered solids and footings use their maximum canonical radius. Train, rail, tie and spine checks retain their existing dimensions and contact rules. Only a verified support arm endpoint can make its own intended spine contact; this never exempts a train, tie, rail or station collision. Axis-expanded OBB tests can conservatively reject separated solids; acceptance is not obtained by relaxing those tests.

## Complete terrain footprint

Terrain uses analytic global gradient bounds L=0 (flat), .056 (hills), .28 (canyon). For body midpoint center C, terrain height obeys H(X) ≤ H(Cxy)+L|X−Cxy|. The resulting lower clearance function z−H(Cxy)−L|X−Cxy| is concave. Its minimum over the complete convex body occurs at a vertex, so eight algebraic corner evaluations certify the **interior footprint** as well. Only one terrain query is needed per body pose. Flat terrain uses the exact minimum body Z.

Subtract .20 sqrt(1+L²) to cover continuous body motion: the clearance function z−H is sqrt(1+L²)-Lipschitz in 3D. Compare that lower bound to the configured `minClearance`. The previous 2 m cross-section test and its extra 1.6 m reserve are retained separately; this change does not relax old terrain acceptance. The prepared sweep is reused for terrain and all support members.

## Permanent verification and practical limits

The permanent clearance suite exercises direct intermediate frame/body/hardware containment, rapid quintic frame/bank variation, singular projections, all cached polynomial coefficients, headroom and cancellation. Terrain tests cover full body/interior footprint and continuous motion, plus certified chord-domain rejection. Support/station suites cover explicit-member dimensions, geometry retention, malformed checksummed saves, own-contact exclusions and mandatory structures. From the repository root, run all suites with `native/tools/build.ps1 -Compiler native/.tools/zig-x86_64-windows-0.14.1/zig.exe -Test`.

Dense probes supplement the analytic argument; their count alone is not a proof. The local12 m self-track adjacency model is documented separately in `SELF_TRACK_ASSESSMENT.md`. UE tube tessellation is not covered by a formal rendered-triangle intersection proof. No UE compile, cook, playback, GPU or structural engineering certification is implied.
