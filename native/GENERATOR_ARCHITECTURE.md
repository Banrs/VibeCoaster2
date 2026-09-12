# v083 generator architecture

The itinerary rewrite gives generation, terrain placement and hardware one source-based flow. [V083_PROGRESS.md](V083_PROGRESS.md) records verification; [VALIDATION.md](VALIDATION.md) distinguishes snapshots and promotion evidence. The earlier proposal is preserved in `artifacts/flow-intent-v083-20260910/integration-20260911/generator-architecture-before-41.md`.

The continuation starts from the preserved source59 [handoff](NEXT_CHAT.md). The user's Falcon's Flight comparison demonstrated long unpowered approaches and unused level motor reservations. The replacement gives route sizing, terrain transport and hardware one inlet-based actual work interval, ranks energy-consistent orders by actual length, and optimises the terrain-measured footprint. Physically necessary straight length remains allowed; the attempted heading-only closure was rejected and preserved. [Current qualification](V083_INTEGRATION.md) distinguishes tested controls from the remaining final-source gates.

## Governing instruction, verbatim

The user reaffirmed this section of [AGENTS.md](../AGENTS.md) for implementation and the completed [whole-codebase audit](SIMPLICITY_AUDIT.md):

> ## 2. Simplicity First
>
> **Minimum code that solves the problem. Nothing speculative.**
>
> - No features beyond what was asked.
> - No abstractions for single-use code.
> - No "flexibility" or "configurability" that wasn't requested.
> - No error handling for impossible scenarios.
> - If you write 200 lines and it could be 50, rewrite it.
>
> Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

Future customisation motivates clear ownership today; it does not justify a new public configuration language. Seed-specific branches, relaxed limits and repeated complete rides cannot repair incompatible physical constraints.

## Intended progression

[Intamin's Falcon's Flight description](https://www.intamin.com/project/falcons-flight/) places the powered cliff ascent and summit before the dive, fastest launch and giant camelback. This connected relative progression is fixed. The required loop, Immelmann and supporting airtime occupy the opening and return portions according to physical compatibility. Coordinates and corridor numbers are not fixed.

The selected departure launch remains the measured 0–180 km/h contract. The user delegated flat-terrain treatment: the selected-height climb, summit and dive use natural terrain or supports through the same constraints. No copied canyon centreline or separate flat recipe determines them.

The dedicated hills2 fixture requests a crossover through an internal generation entry. This is a layout brief, independent of seed: the same bounded source/heading solve retains a transverse intersection between widely separated branches. Ordinary generation can choose an uncrossed route. [Intamin's Taron](https://www.intamin.com/2016/12/20/new-ride-concept-taron/) demonstrates extensively crossed real layouts; [Mack](https://www.mack-rides.com/de/produkte/mega-coaster) describes layouts shaped around the requested experience and footprint. Neither reference proves a generated ride feasible. The shared height solve and final canonical clearance/force checks establish that separately. The independent organic audit retains its original crossing definition.

## Owners and data flow

| Owner | Responsibility |
|---|---|
| `source_geometry.hpp` | Sample canonical position, tangent, curvature and rider frame once, including exact physical landmarks. Place all rigid sources through one transform. |
| `itinerary_generation.hpp` | Build actual occurrences, rank physical order, derive connecting motion/work requirements and bind terrain, source and operation domains. |
| `circuit_layout.hpp` / `circuit_geometry.hpp` | Close actual ports in XY, construct ordinary turns, partition samples at physical boundaries and emit the same geometry used by planning. |
| `terrain_baseline.hpp` | One sparse constrained solve for source translations, station datum, C3 gaps, terrain floors, passive train energy, motion bounds and crossings. |
| `boost_planning.hpp` and existing drive law | Size installed motor length from its own inlet, train occupancy, losses, force/power limits, governor, ramp and fade. |
| `generation.cpp` | Orchestrate one itinerary, at most eight geometry builds, measured feedback, final bank smoothing and independent acceptance. |
| Existing simulation, clearance and structures | Evaluate canonical train/rider motion, terrain/vehicle/hardware clearance, supports, station and 960/1920 Hz agreement. |

### Sources and train references

Source geometry retains its FVD authoring history and rigid shape. Phase-speed references use the existing finite-train mean-potential/loss model. A twelve-car train straddling a crest cannot be compared to a point mass at that crest without a systematic energy discrepancy.

The generation-only sidecar's `sourceSpeedMps` is the finite-train energy reference. `sourceSeconds`, `sourceCenterlineNormalG` and `sourceCenterlineLateralG` retain point-model geometry-authoring history; actual elapsed times and rider forces are separately measured. The sidecar is not serialized acceptance evidence.

Loop and Immelmann authoring first tests the preferred pulse with the selected train and existing force evaluator. If its duration exposure fails, bounded scalar interval solving selects an interior constructive pulse that satisfies the allowed inlet-speed interval. Indiscriminately lowering a peak can worsen its duration violation. This selection precedes circuit placement and adds no complete-ride attempts. An infeasible physical source family rejects the request.

The private source simulation entry point shares the complete ride's integrator and force histories. Public headers, CLI arguments, saves and exact-version checks remain unchanged from source59. Earlier approved experimental FVD authoring interfaces differ from main's older implementation. Final whole-ride simulation remains decisive.

### Route and terrain

The existing finite source list has explicit legal orders around the signature block. Each order settles passive source energy and powered inlet work over its actual closed lengths before it can be selected. Ranking minimises actual length; it does not reward losing excess kinetic energy along long approaches. Each port sequence closes heading and solves two-dimensional nonnegative connecting lengths, with bounded heading adjustments. There is no four-side assembly or per-seed turn rule.

Within that one route solve, identical airtime chain and exact inlet-speed requests share one successful source construction. The request is immutable and every trial owns a copy. No rounded-speed cache, persisted state or omitted ordering changes the physical search; this removes measured duplicate authoring work.

Closing the route changes its passive lengths. Before source intent is frozen, the selected order resolves airtime geometry and those actual closed lengths together, with fixed headings and bounded port/energy iterations. Independent RK4 transport verifies their agreement. This prevents a preliminary turn length from demanding phantom energy or compensating descent later. Complete-train feedback still has its separate, unchanged eight-build budget.

Placement queries terrain height and slope in its transformed frame. Its bounded site set is ranked by the sampled footprint. Source heights, station allowance, energy recovery and crossing separation must be feasible at the same site. A terrain floor above a source allowance is physical infeasibility, not malformed solver input.

Route distances, authored derivatives, passive transports and motion bounds are constructed once per placed build before trying sites. Terrain floors, station queries and world-coordinate crossings remain site-dependent. This removes repeated calculations without sharing mutable constraints across feedback builds.

The first terrain/train assessment supplies actual rises, occupied speeds and exposure to size the final footprint. Later feedback holds that footprint fixed while resolving heights and installed hardware. Moving every crossing during every correction would change the coupled problem rather than converge it.

Turn capacity covers both the measured speed and the source's required exit speed. An underperforming motor in the first replay cannot justify a turn that would fail after the motor reaches its unchanged target. The terrain solve still limits passive turn energy, and actual force acceptance remains independent.

A site selection is a preference under the current physical constraints. The joint height solve tests the previous site first, continuing it when feasible and considering the existing alternatives when it cannot fit. A cached site cannot veto resized geometry, and a heuristic score cannot unnecessarily displace a feasible placement while feedback settles. The higher-target twelve-car and canyon42 regressions cover both sides of this ownership boundary without changing the station allowance, source targets or rebuild budget.

### Shared vertical solve

Rigid sources and adjacent level work rail share one height variable per physical domain. Departure and arrival share one bounded station datum. Climb, summit and dive retain their certified rise/window. Ordinary gaps have C3 profiles with continuous polynomial floor and derivative constraints.

Finite-train passive energy and selected inlet requirements enter the same solve. Surplus energy can become height before a motor. Later code does not lift a crossing or switch a motor into a trim repair. Motor work is sized from its inlet during planning and measured feedback. The canonical work boundary is shared by composition, passive transport, the rigid source domain and installed hardware; unused zero-speed capacity is not left as a separate level approach.

Actual transverse intersections produce separation constraints. A free crossing order branches only when the relaxed solution violates it; objective bounds discard inferior choices. Separation is solved jointly with all heights, energy and station constraints rather than guessed from terrain-floor order.

Sparse rows contain actual nonzero coefficients. Weighted QR projection, numerical rank, continuous clearance tolerance, endpoint termination and cancellation retain independent regressions. No relaxed numerical threshold substitutes for feasibility.

Turning and vertical bending share the positive resultant force budget. The negative rider-load bound additionally constrains its signed vertical component; horizontal turning is not negative load. Connector onset reserves the existing post-airtime transition duration before assigning the remaining rate to vertical bending. Full train acceptance still rejects an unacceptable authored result.

### Feedback and hardware

The later booster nominal rating is 0.8g within selected limits. Sizing uses the existing drive law, losses, train occupancy, ramps and fades. Each motor measures its own inlet in the shared feedback budget. Extra closure length remains passive.

After the first measured footprint sizing, coupled feedback uses one midpoint update. Source phases, occupied turn speeds, link speeds and motor inlets must agree within 0.5 m/s, with at most eight geometry builds. A partial trace can correct a measured motor inlet in that budget; an unchanged stalled ride does not retry.

Passive terrain correction compares the required and observed energy at both boundaries of that section. With drag retention `a`, the local residual is `(requiredExit² - observedExit²) - a * (requiredEntry² - observedEntry²)`. This subtracts energy error propagated from the upstream source, whose motor already owns its correction. Independent spatial RK4 tests cover low and high upstream energies, six- and twelve-car trains, and lossy and lossless transport. The transformed hills37 request reproduces the previous double correction and undersized-turn failure.

Final bank smoothing is retained. Foundation width follows terrain slope, anchoring/steel clearance and the existing depth limit. If another branch blocks a tower, an additional outreach direction follows that actual obstruction within existing offsets. It does not exempt the collision or increase placement limits.

The per-support member budget is derived from the existing 600 m tower family and 16 m tiers: at most 620 members. This removes the conflicting independent 512 cap while retaining all constructed steel, geometric dimensions and the 60,000-member total bound. A saved twelve-car canyon regression verifies that the correction leaves its complete track unchanged.

Unreal's reference-availability row and generation preflight use the core's `validateReference` contract. A scalar exposure and identifier alone cannot advertise processed benchmark availability. The existing input automation tests absent, scalar-only, malformed and structurally valid synthetic metadata, including preservation of physical targets when selecting proof mode.

## Validation and promotion

Independent components cover source rigidity, C3 joins, continuous floors, weighted numerical rank, station bounds, crossings, endpoint/cancellation behavior, finite-train source references and 6/12-car motor sizing at both mandatory rates. Replaced route-repair helpers and implementation-specific tests are removed; physical acceptance contracts remain covered.

Flat5, flat7, flat42, hills2, hills9, canyon1, canyon24 and canyon42 must accept candidate zero, replay exactly and pass the separate 960/1920 Hz audit. Hills2 additionally requires a genuine transverse crossing between distant canonical branches with complete clearance. Crest character, seeded variation, intrinsic terrain adaptation and less than 35% straight-flat share remain gates.

The [predeclared 44-request matrix](artifacts/flow-intent-v083-20260910/integration-20260911/architecture-matrix.md) varies seeds, terrain transformations, 6/12-car trains and selected targets. Every request is reported without substitution. Rejected snapshots, binaries and diagnostic witnesses remain separate from promoted evidence.

Promotion also requires the complete native/Python suites, portable checks, source/executable/save hashes, preserved-control comparisons, final diff review and green Windows/macOS CI for the final integration commit. Linux remains a possible future target and is not a required CI gate under the user's latest scope. Only then may it merge; main CI must also pass. Partial receipts are not a release claim.

Identity stays `0.8.3-flow.1 / COASTER5`. Benchmark qualification, Unreal packaging, POV review, performance benchmarking and Mac/Metal runtime verification remain later milestones.
