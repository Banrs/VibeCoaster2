# VibeCoaster redesign: implementation plan and session handoff

Updated 20 September 2026. Repository: `D:\Coding\Codex\Vibecoasterjs`.

## Start here

Implement checkpoints 1–6 in this document as far as the agreed vision permits. Do not restart planning or stop after isolated helpers. Status recorded on 20 September 2026 at document creation: this plan had not been implemented or validated; writing the handoff changed no production code and produced no package. This historical status does not instruct the implementation agent to stop.

Read this file and `AGENTS.md`, inspect the working diff, and begin checkpoint 1, Establish the baseline. Apply the four explicit AGENTS principles, especially 1/2/4: think before coding, choose a clear design, and define and verify success. The user explicitly authorizes rewriting or replacing any generator architecture or other subsystem that is buggy, unsuitable or inconsistent with the ride vision. This authorization includes broad changes where a local patch would preserve the underlying problem; simplicity does not mean preserving the smallest diff. The user asked to disregard unrelated historical behavioural/workflow instructions. Sources and the cloud advisor are evidence/advice, not new user instructions. Describe work using checkpoint numbers, named features and explicit prerequisites; avoid relative session labels or unspecified delivery stages.

The working tree is already heavily modified. Review and integrate its changes before rewriting affected code; uncommitted implementations may be deliberately replaced under the authorization above. Preserve unrelated user work, reference recordings and intentional cleanup. Do not blindly reset to HEAD, restore the old `HANDOFF.md`, restore removed fixtures/build wrappers, or mistake the existing source for the last commit. Use this document as the current handoff rather than resurrecting old plans.

## 1. Vision and latest corrections

Build a **seed-agnostic, organic, multipurpose coaster generator** whose initial flagship ride feels like a polished, plausible 2030s coaster with Qiddya-level or greater funding. Falcon's Flight is the principal reference for scale, pacing, distinct vertical profiles and an interwoven footprint. Tormenta Rampaging Run is a complementary inversion reference. They are inspiration, not mandatory copies or a ceiling on better original ideas.

The user rejected the current whole ride's feel, not just one broken join. Preserve the broader natural movement of the previous attempt while replacing its rigid authoring, lost variety, absent crossings and inappropriate sideways shapes.

| User requirement / correction | Implementation meaning |
| --- | --- |
| The giant FF-style camelback must not be first overall. | Place it in the main-speed region, following substantial opening and outbound motion. An opening hill is allowed; a particular top hat is not mandatory. |
| Do not make the opening or earlier sections weak. | The record launch must lead into substantial, purposeful climbs, drops and turns. Treat speed and scale as goals for each functional section compared with FF, not just one global peak. Do not invent official records for arbitrary sections. |
| Record-beating opening launch is the minimum. | Keep the current working target of 0–180 km/h in 1.4 s. The user's hard intent is to beat the documented launch benchmark; do not silently weaken it or the existing force limits. |
| C3 transitions and better blending are the main flaw. | Preserve meaningful pitch, bank, curvature, force and derivative state across authoring boundaries. No element -> forced-neutral connector -> similar element. |
| Planar elements must remain planar. | The main camelback must not turn sideways. Do not add yaw to ordinary hills merely to disguise straight track. Intentional inversion rolls and graded turns remain 3D. |
| Remove the sideways hill after the vertical loop, before the booster. | Author a sideways S directly into the booster; no extra hill or flat waiting connector. |
| The final airtime hill's shape and elevation were bad. | Redesign the return from its energy and route. Do not manufacture an elevated bridge hill to solve a crossing. |
| Include variety, including ordinary transition/filler motion. | Use distinct hills, valleys, sweeps, changing banks, graded reversals and inversion phases. The user's explicit permission to adapt references supersedes the demand to copy every FF/TRR element. |
| No excessive frequency compared with FF. | Avoid both tightly repeated airtime/direction-change events and rapid pitch/roll oscillations inside them. Greater intensity should come from deliberate steepness, speed, scale, airtime and sustained load. |
| Crossovers must return. | Reserve meaningful over/under relationships in macro planning and verify full train/rider/support/ground clearance. A centreline intersection or a loop aperture passage alone is not proof. |
| At least one original, exciting signature. | Use one coherent original concept whose motion and placement adapt to each seed. Do not build a menu of unrelated fixed signatures or paste one immutable piece into every layout. |
| Support user customization through reusable authoring. | Build reusable motion intent and constraints in checkpoints 2–5; re-solve affected neighbourhoods after changes. Element systems, interfaces and editor architecture may be redesigned when they help deliver the requested generator and ride experience. |

"0% flat" means zero purposeless flat waiting between motions. Real station, launch and brake alignment still needs physical length. A valley's instantaneous horizontal tangent is necessary, and a plan-straight camelback is desirable. Do not confuse either with a dead connector. Do not use a two-kilometre coasting straight to reach an element or meet a length target.

Improved train/seat design can motivate comfort and restraint design choices; it is not evidence that unlimited force, jerk or oscillation is acceptable. Keep limits explicit and provisional, and verify the implemented vehicle model.

## 2. Targets and questions to settle with evidence

Current source defaults, verified for this handoff:

| Quantity | Current value |
| --- | --- |
| Height / maximum speed / inversion-height targets | 220 m / 290 km/h / 80 m |
| Launch metric | Time from rest to 50 m/s; target 1.4 s |
| Train | 6 cars, 1,500 kg per car, 3.4 m spacing, 1.2 m seat height |
| Provisional specific-force limits | Gz -1.5 to +5.5; absolute Gy 1.5; absolute Gx 4.5 |
| Existing vertical force-rate limit | 20 g/s (`maxJerkGps`, despite the ambiguous name) |
| Lateral / longitudinal force-rate limits | Currently NaN/unassessed unless explicitly configured |
| Minimum configured clearance | 2 m, applied through the real envelope checks |
| Native integration / convergence | 960 Hz / 1,920 Hz |
| Candidate budget | 8 default, bounded; maximum 64 |

Earlier conversation mentioned 300 km/h; the current source is 290 km/h. Do not lower the established performance to ease solving. Retain the current explicit request/setpoint semantics; do not silently change the speed dial or substitute a fixed seed. Whether 290 or 300 is used for a particular new baseline must be stated in its evidence.

The proposal for new generated rides is to assess all three force rates, initially using the existing 20 g/s magnitude for the other axes as a **design hypothesis**, not a published rider standard. Existing saved rides with unset optional limits must retain honest "unassessed" reporting. Do not claim all-axis jerk acceptance while leaving Gy/Gx unassessed.

Before choosing launch ramps, calculate and replay their feasibility. 50 m/s in 1.4 s needs about 3.64 g average net forward acceleration; eased onset needs a higher peak. The 1.4 s metric is the **first attainment of 180 km/h**, not necessarily the end of propulsion. If the opening's energy programme needs a faster exit, acceleration may continue beyond 180 km/h and fade at the solved launch-exit target. Distinguish this from incorrectly forcing both ramp-up and complete cutoff into 1.4 s. Include drag, rolling losses, train coverage, power/force limits and the seat-axis rates in the check. If the user's hard targets conflict, report the measured conflict and ask rather than secretly relaxing them.

Before implementation, answer these design questions in the local code/diagnostics:

1. What incoming state must this span inherit, and is a neutral port actually required by hardware?
2. Does its slope/heading/bank history express one intended motion, or does a bridge introduce a shelf, extra dip or roll flick?
3. Where does each launch's energy go? Does its powered exit feed substantial motion before entry to another powered section?
4. Does this primitive remain useful across entry speeds, scale, handedness and neighbouring motion?
5. Are the macro footprint and crossings planned before track is forced to close?
6. Which architecture best delivers the ride vision: extending, replacing or removing the existing kernel and its supporting systems? Compare correctness, expressive capability, maintainability and unnecessary complexity rather than assuming reuse is preferable.

## 3. Ride composition and seed variation

The following is an illustrative functional sequence, **not a fixed layout, mandatory top hat, frozen element order or set of neutral-ended modules**. Motion and roll profiles may cross every named region.

| Region | Functional motion | Required character |
| --- | --- | --- |
| Opening | Record acceleration into a substantial rise/crest or directional climb, descent and loaded sweep | Use the launch energy immediately. A smaller opening hill is compatible with the user's correction; the giant camelback belongs in the main-speed region following opening and outbound motion. No weak prelude. |
| Outbound | Smaller airtime, loaded valley, graded banking/turning and purposeful approach to power | Preserve ordinary sensations and naturally elevated crossing opportunities. Avoid identical repeated bumps and return-to-level ports. |
| High excursion | Inclined power into a high crest/turn, outward banking where intentional, then a steep main dive and broad pullout | Escalating scale and anticipation. Current terrain is flat, so this is supported track rather than a fabricated cliff. No long level summit. |
| Main speed / giant hill | Power sized for this phase directly into a large planar camelback, sustained crest and steep descent | Fixed plan heading through the principal hill. The approach, crest and descent are a composed vertical profile, not a hill at the end of a long coasting straight. |
| Reversal | Camelback descent -> continuous loaded valley -> Immelmann ascent and roll exit -> graded continuation | First rideable audit span. One intended valley, no flat pause or extra dip before the half-loop. Inversion height and incoming energy solved together. |
| Loop region | Broad purposeful approach and predominantly planar vertical loop | Retain recognisable inversion shape. Only deliberate entry/exit separation, not an arbitrary sideways hill disguised as a loop. |
| Loop exit / power | Sideways S flowing into booster | No unwanted sideways crest, no extra flat connector. Bank/pitch settle only as required for the entire powered train footprint; motor starts at the first usable alignment. |
| Original signature / return | Seed-adapted signature, broad pullout, complementary reversal, lower airtime and low turns | Exciting spatial interaction and distinct motion. Final hills spend energy instead of creating artificial crossing platforms. Maintain clearance even at maximum bank. |
| Finish | Progressive braking and station return | Only necessary alignment and stopping distance. No velocity snap, abrupt bank reset or decorative straight extension. |

For the flagship style, target roughly three meaningful crossings and require at least two genuine grade-separated route crossings as a proposed acceptance rule. This count is a concrete design decision to satisfy the user's missing-crossovers complaint, not a quoted FF statistic or a universal restriction on other coaster styles. Good candidates include naturally elevated camelback legs over low outbound/return routes and a smaller crest over a graded approach. Move the route/crossing rather than raising the whole final hill.

Seeded choices should alter macro arrangement, eligible motion ordering, handedness, corridor placement, crossings, profile timing and shape. A few compatible arrangements (for example folded outbound/return, overlapping lobes and an offset figure eight) are a starting proposal; replace that approach if another generator design produces better coherent variation. Choose shared or specialised solving methods according to the constraints they must satisfy. Meaningful variation is not mirroring, rigid transformation or +/-2% length jitter alone. Reserve closure, height ownership, station/drive corridors and signature interaction early. Keep deterministic RNG use and a bounded candidate search.

### Original signature proposal: Aperture Dive

Proposed experience: a loaded climbing turn releases into one broad rolling-airtime crest; as the nose falls and the bank develops toward the exit, an earlier part of the ride frames the view; a steep diagonal dive passes through the reserved opening and resolves in a large loaded pullout. The combination of release, changing sightline, roll/pitch coordination and spatial passage should be recognisably its own event, not a renamed ordinary hill.

An earlier loop's aperture is the leading placement proposal. It must be jointly solved with the loop plane, entry/exit geometry, support layout, energy and full swept envelopes. Do most heading change while positive load is available; do not demand a sharp horizontal turn during near-ballistic flight. Use one broad roll event, not a rapid double flick.

Adapt approach side, handedness, crest height/load, entry energy, roll excursion/timing, dive heading and exit corridor to the seed. The exact fly-through is a **proposal requiring feasibility evidence**, not a user-mandated immutable location. If it makes the multipurpose generator brittle, a coherent alternative spatial frame can preserve the same original motion and experience. Do not silently drop the signature or turn it into a generic filler hill; document why the adaptation remains distinct. No claim of worldwide novelty has been researched.

## 4. FVD/spline architecture and rewrite authority

Design the architecture around the requested ride experience and verified physical behaviour. The implementation is authorized to rewrite, replace or remove any unsuitable part of the generator, geometry/frame kernel, simulator, solver, element system, persistence, validation, telemetry, viewer or supporting code. A new simulator, general solver framework or element plugin system is permitted when it is the best way to deliver the vision. Existing files, APIs, module boundaries and algorithms are starting evidence, not mandatory foundations.

Explain substantial architectural choices with the concrete defect or design limitation they resolve, and verify the resulting behaviour. A reproducible bug is one reason to rewrite; an architecture that cannot express cohesive, seed-adapted motion is another. Do not require a defect in otherwise functioning code before replacing a design that conflicts with the vision. Prefer a coherent, maintainable implementation, remove superseded paths and avoid complexity that serves no requested capability. Routine architectural decisions within this authorization do not require additional permission.

1. **Continuous inherited state.** Carry position, physical orientation, speed and the derivatives needed for curvature/frame/control continuity. A named element is an intent label, not an instruction to set pitch/bank to zero or force to 1 g. Extract compatible state from the actual preceding span. A rigid coordinate transform is fine; nonlinear warping requires re-solving and replay.
2. **Independent control timelines.** Normal force, lateral force, roll and propulsion should have independently timed, derivative-aware controls. Allow force and roll phases to overlap. Use the smallest curve representation that supports the required incoming/outgoing derivatives and shape constraints; do not automatically zero all derivatives at every internal knot.
3. **FVD for motion, spline constraints for placement.** Integrate motion from force/roll intent; constrain planarity, target height, entry/exit corridors, inversion plane, crossings, drive alignment and station closure. Solve coupled neighbourhoods against actual entry energy. Assess the existing bounded shooting solver; extend or replace it according to the coupled problem's needs.
4. **Consistent drive semantics.** Authoring integration, work accounting, canonical replay and validation must agree with actual operations. Do not make an authoring-only acceleration curve that the finite-train ride does not execute. Redesign APIs, defaults and persisted representations when necessary, with explicit versioning/migration and clear reporting of changed behaviour. An independent verification model is allowed; define and investigate differences between models instead of hiding them.
5. **Intentional bank ownership.** Automatic force balancing may assist within declared freedom. It must not overwrite an outward bank, inversion roll or planar/upright hill. Geometry/bank edits after a solve invalidate its acceptance until final canonical replay is checked again.
6. **Fitting that preserves motion intent.** Evaluate `flow_bridge.hpp` as one possible constrained-fit implementation; replace or remove it if a different formulation better handles the ride. Avoid universal seam repair of independently completed neutral-ended pieces. Trim duplicated ease-out/ease-in portions and choose phase-compatible ports. If fixed endpoint derivatives mathematically force a shoulder, move the ports/modify the neighbouring motion. More polynomial degree alone does not fix contradictory boundary intent.
7. **Compile and validate the actual final route.** Recompute arc length and operation locations after geometry changes. Check local-fit versus whole-route compilation and save/load preservation. Finite-train replay may change the energy estimate; feed that back into the relevant local solve rather than weakening the force gates.

Reusable motion vocabulary: powered acceleration/deceleration; planar rise/crest/descent; banked directional change with controlled grade; inversion/reversal; and an S/bank reversal. A "twisted turn" is a deliberate combination of grade, heading and roll, not the default connector. Expose only controls that have an implemented solve.

### What C3 and jerk acceptance mean

- Check the final centreline and physical orientation frame, including module boundaries, motor/brake boundaries, inversion exits and station closure. State the parameter used; C3 in a polynomial parameter is not automatically C3 in arc length or time.
- Matching position/first/second/third derivatives at both endpoints supplies eight scalar constraints. A septic Hermite polynomial can satisfy those; an arbitrary quartic cannot. Quartic splines can be C3 with suitable knot structure. Neither fact establishes good interior shape.
- Evaluate unwanted extrema, reverse motion, rate notches, near-zero-rate dwell, overshoot and narrow roll events between endpoints. One continuous valley recovery should not become two pullouts with a hesitation between them.
- For arc-length centreline `r(s)`, time jerk is `r''' v^3 + 3 r'' v a + r' da/dt`. Smooth geometry cannot compensate for a stepped drive command. Include physical orientation derivatives and seat-offset rotational terms.
- Rider-axis force rate (g/s), inertial translational jerk (m/s^3) and angular jerk are different quantities. Keep their labels, units and acceptance separate. Avoid Euler-angle wrap and vertical-pitch singularities by validating quaternion/frame quantities.

## 5. Implementation checkpoints and verification

Checkpoints are reviewable results, not recurring permission gates. The user wants as much implementation as possible within this vision. Keep progressing on independent authorised work; ask when a real unresolved tradeoff would change that vision or a required constraint conflicts.

| Step | Deliverable | Verification before treating it as complete |
| --- | --- | --- |
| 1. Establish the baseline | Inventory working changes; fresh local native build; capture one current seed-42 report/trace; identify architectural defects and choose what to retain, replace or remove | Baseline build status is explicit. Preserve unrelated user work and intentional cleanup. Document the forced-flat/turning-hill failure and architectural constraints that prevent the desired flow. |
| 2. Shared motion foundation and first span | Inherited non-neutral FVD controls; consistent propulsion where required; trace extension; parameterized planar camelback/valley/Immelmann span | No extra dip/shelf; main hill heading fixed; intended valley shape; analytic frame/geometry continuity; finite-train forces and rates; several entry speeds/scales. New controls must be used by the real generator. |
| 3. Inversion continuation | Graded loop approach, recognisable loop, sideways S and immediately useful booster | No sideways airtime hill; powered full-train alignment; minimum necessary coasting; continuous roll/pitch/drive behaviour; loop and clearance preserved. |
| 4. Strong opening and high excursion | Record launch, substantial opening motion, ordinary outbound variety, inclined power and high dive | Measured 0–180 launch; useful energy spent in each region; onset/cutoff rates; slope/steepness; actual energy handoff from the high-excursion dive into the main-speed region. Giant camelback follows substantial opening and outbound motion. |
| 5. Original signature and full generator | Seed-adapted signature, multiple meaningful macro arrangements, crossings, energy-aware return and station closure | Full train/rider/structure/ground clearance; signature identity; required variety; C3 plus interior shape; deterministic and materially different seeds; no hidden fixed-seed fallback. |
| 6. Final validation and package | Complete accepted generated ride, evidence and updated packaged shortcut | Focused corpus and required tests pass; native 960/1920 Hz convergence; correct saved/reloaded geometry; final package smoke test and actual build identity; latest shortcut target verified. |

Crossing and signature corridors must be reserved during macro planning before the local pieces are solved. Step 5 validates the assembled interaction; it must not be the first time closure or footprint is considered.

### Telemetry and audit artifacts

The existing CLI `--trace` path writes a 60 Hz display trace and geometry every 5 m. Extend or replace the export/telemetry architecture to provide synchronized **roll, pitch, yaw, speed, Gx, Gy, Gz**, with front/middle/rear seat series and necessary derivative/rate diagnostics. Keep data definitions and provenance explicit across any replacement. Define Gx forward, Gy right/lateral, Gz up/normal; current stored seat array order is **vertical, lateral, longitudinal**, so map explicitly.

Provide a readable time-aligned plot/report with section boundaries, plus a macro layout view and a reproducible ride/saved design for the user's checkpoint audit. Do not infer peak jerk from a 60 Hz display plot: authoritative gates use analytic/native-resolution results and refinement. Rewrite plotting/viewer/UI components when that is needed to inspect or deliver the actual ride experience.

For FF frequency, compare event spacing and sustained motion against the supplied render/POV qualitatively. Exact FF force, jerk or angular-frequency equality cannot be claimed without comparable telemetry. Do not invent numerical telemetry from video.

### Compact acceptance matrix

1. **Continuity:** final canonical centreline and physical frame at every seam, including closure and operation edges; stable regular parameterization.
2. **Shape:** planar main camelback; one intended valley before Immelmann; no redundant flat shelf, extra downhill dip, forced-neutral bank pause or rapid roll reversal. Test interiors, not just endpoints.
3. **Motion:** actual train/seat forces, axis rates, angular behaviour and launch/brake dynamics within explicit limits; speed and scale retained through all substantial sections.
4. **Hardware and structure:** powered/braking alignment across the whole train; actual stopping without a snap; all crossings, rails, supports and ground clear full swept volumes.
5. **Variety and pacing:** signature plus distinct ordinary motions; meaningful crossings; no padded coasting straight or rapid substitute bumps. Do not erase variety to satisfy an arbitrary short duration.
6. **Generation:** reproducibility, bounded search/cancellation, meaningful seed/parameter variation, and clear rejection when incompatible; no seed-42 special case.
7. **Refinement and persistence:** same candidate under finer spatial compilation and 960/1920 Hz temporal replay; saved/reloaded geometry retains accepted behaviour.
8. **Delivery:** current package, actual seed-42 replay, reviewable evidence and verified shortcut, with remaining limitations stated accurately.

Review FVD, flow-bridge, frame/force, drive-profile, geometry, clearance and organic-generation coverage against the replacement architecture. Retain useful invariants, rewrite invalid tests and delete obsolete or redundant coverage. In particular, `organic_generation_tests.cpp` currently **requires the main hill to turn >25 degrees**; replace this with the intended planarity contract. Old fixed sequence, <6 km and <=200 s assumptions must be reviewed against the redesigned motion, not blindly preserved at the cost of variety. Correct buggy validation and physical assumptions as well as buggy generation. Enforce the valid force, continuity and clearance requirements through trustworthy checks; do not weaken acceptance merely to make a candidate pass.

Initial final corpus: seeds 1, 7, 42, 77, 314 and 2718 at the baseline, plus a few deliberate speed/height, inversion-scale and train variations. This is a small proposed corpus, not permission for a large Cartesian product after each edit. Record topology/motion choices as well as aggregate physical metrics. Run affected checks per change; full required validation once the integrated candidate is ready, repeating only for new changes/failures.

Delete genuinely redundant or obsolete tests made unnecessary by this work, as the user requested. Preserve distinct meaningful invariants and cancellation/data-integrity coverage. Remove code/imports/helpers and superseded subsystems made unnecessary by the redesign. Broad cleanup or refactoring is authorized when it corrects defects or removes architecture that conflicts with the vision; preserve unrelated user content.

## 6. Current implementation map and known defects

All observations below refer to the working tree inspected on 20 September, not just HEAD. Symbol names are safer than line numbers after edits.

| File / area | Observed state and required change |
| --- | --- |
| `native/core/src/generation.cpp` | Fixed `candidate()` sequence. `sweep()` uses a `sin(pi*u)^4` rise and zero endpoint pitch; `curveTo()` uses explicitly horizontal tangents. Replace the neutral-end assumptions with inherited motion and coupled spans. |
| Main hill in `candidate()` | `turning-record-hill` currently sweeps 55 degrees immediately after departure launch. Author a substantial opening and place the planar hero hill in the main-speed region following opening and outbound motion. |
| Inversion approach | Separate 600 m descending sweep, trim and neutral-start FVD Immelmann. Solve the approach/valley/ascent together; do not hide the force/pitch reset with a longer bridge. |
| Loop / return | Loop shape has large drift; loop-exit S exists, but the final route has a generic elevated crest and lacks intended interweaving. Re-solve shape/energy/placement rather than preserving labels. |
| `blendAuthoredJoin` / `flow_bridge.hpp` | Joins are patched after module construction. Reuse, replace or remove the fitter according to the redesigned authoring model; stop relying on it to repair incompatible motion intent. |
| `improveBanking()` | Global postpass can change intentionally authored bank/entry/exit motion. Scope its freedom and revalidate the final physical frame. |
| `native/core/include/coaster/fvd.hpp`, `native/core/src/fvd.cpp` | Existing quaternion/RK4 point-mass FVD, packed quintic controls, twist phases, bounded shooting, loss/work tracking and canonical diagnostic replay. No propulsion in generic FVD. Presets use horizontal 1 g ports. Assess whether extension or replacement best supports continuous inherited motion and coupled solving. |
| `track.cpp`, `frame.cpp`, `arc_length.hpp` | Existing canonical high-order centreline/frame and kinematics machinery. Audit its correctness and fit to the authoring requirements; rewrite any unsuitable representation or algorithm and verify derivatives/regularity. |
| `simulation.cpp`, `drive_profile.cpp`, `convergence.cpp` | Existing finite-train physics and drive/convergence acceptance. The implementation may be replaced. The authoritative ride replay must still assess real train/seat dynamics, energy, operations and convergence; point-mass authoring alone does not establish those results. |
| `native/core/src/main.cpp` | Existing `--trace`, `--plan`, `--json`, `--out` paths and explicit optional axis-rate flags. Redesign interfaces and exporters as needed; document the supported workflow. |
| `persistence.cpp` | COASTER5 stores authored geometry and operations. Redesign storage or compatibility handling if necessary; preserve user files and explicitly version, migrate or report unsupported formats. Do not silently reinterpret an old ride as a freshly generated layout. |
| `organic_generation_tests.cpp` | Useful full-route checks plus obsolete turning-hero/fixed-route assumptions. Rework coverage around the intended behaviour and actual architecture, without protecting obsolete implementation details. |

The source reports missing real intensity reference data honestly. `physics-proof` is suitable for engineering development without those recordings; it must not be presented as a successful `all-records` intensity comparison. Do not fabricate reference exposure data to obtain acceptance.

## 7. Workspace, build and delivery handoff

- Branch: `setpoints-and-elements`.
- HEAD at handoff: `b57b1b4` (`Calibrate coaster generation and convergence validation`). Other recorded commits include `a8a2bc2` (bank/friction) and `1dce397` (shortcut). The working diff contains substantial uncommitted edits beyond that HEAD.
- Current source/version label: `0.8.4-layout.1`; numerical save format COASTER5. The user does not accept its ride feel. A passing old test/package is not evidence the redesign is done.
- Many current edits/deletions belong to earlier work and repository cleanup: `README.md`, `ROADMAP.md`, CMake/workflow, geometry/frame/generation/persistence/tests, UE version files, shortcut, removed saved rides/fixtures/helpers and removed `build-and-play.ps1` / old `HANDOFF.md`. Inspect before writing. Do not re-add deleted legacy material.
- No existing workspace `build/` directory at handoff. Both `build/` and `build-cmake/` are ignored. A fresh `native/build-cmake` is consistent with README; verify the chosen cache is fresh before reusing it.
- User previously reported a computer crash during a test. Cause is not established. Start with one build/test worker and avoid concurrent heavyweight native/Unreal runs. The current package script hardcodes two compiler actions in two places; deliberately bound this if necessary rather than assuming it is already one-worker.

Verified tool locations (read/use the installed tools; do not rebuild from a legacy source checkout):

```text
VS environment: D:\Toolchains\VS2022\VC\Auxiliary\Build\vcvars64.bat
CMake / CTest: D:\Coding\Codex\vibecoasterlegacy\archive-2026-09-13\removed\native\build\tools\cmake\data\bin
Ninja: D:\Coding\Codex\vibecoasterlegacy\archive-2026-09-13\removed\native\build\tools\bin\ninja.exe
Python: C:\Users\danie\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe
Unreal: D:\Games\Epic Games\UE_5.8
```

The README provides supported core build/test commands. Configure Release with the available compiler/tool paths, then build with `--parallel 1`. List CTest tests and run relevant components first. Use final component/integration labels for whole validation; `--parallel 1 --output-on-failure --timeout 600 --stop-on-failure` is an appropriate conservative start. Package with the actual workspace `native/unreal/scripts/package.ps1` after the native design is accepted; review its parameters rather than using the deleted wrapper.

Existing CLI example after a fresh build (create the ignored output directory first; executable location depends on the CMake generator):

```text
coaster_cli generate --preset physics-proof --seed 42 --height 220 --speed-kmh 290 --inversion-height 80 --launch-seconds 1.4 --candidates 8 --step 0.0010416666666666667 --json native/test-output/seed42-report.json --trace native/test-output/seed42-trace.json --plan native/test-output/seed42-plan.json --out native/test-output/seed42.coaster
```

This example reflects today's flags, not proof of a newly solved ride or an instruction to leave axis-rate gates unassessed. Add the explicit axis-rate options when evaluating the proposed all-axis limits. Capture failure diagnostics without repeatedly rebuilding or repackaging.

`Play VibeCoaster.lnk` currently resolves to the following **existing** executable (existence verified; no new runtime smoke check in this handoff):

```text
D:\Coding\Codex\vibecoasterlegacy\layout-084-game\rebuild-20260919-134910\run-20260919-134930-692\Windows\VibeCoaster\Binaries\Win64\VibeCoaster.exe
```

Working directory is the enclosing `Windows` directory; arguments are empty. This supersedes older notes pointing to `20260919-230353`. At delivery, rebuild from the intended current source, verify packaged build identity and runtime behaviour, then point the shortcut to that exact executable with the complete package intact. A shortcut edit alone does not make a package current.

## 8. Cloud ChatGPT advisor handoff

The user explicitly asked to offload planning/research/noncoding advice to **chatgpt.com in Chrome**, in the existing **VibeCoaster2 Planning** project. Do not substitute a new Codex task, ChatGPT Work job, local subagent, or repeated monitoring automation. This is a user preference about delegation; no claim about account billing was verified. User-requested advisor model/effort setting: **Astra 6 Pro**. Apply that setting to cloud advisory requests.

Existing conversation: [Review Coaster Flatness](https://chatgpt.com/g/g-p-6aa67667d7e0819180d029e339751337/c/6aab0e29-3b60-83ec-9aac-3f5a0b0035bedf).

A single focused request was sent through Chrome on 20 September 2026, and the UI confirmed it was answering. It included the user's corrections recorded in section 1, strong per-section ambitions, original-signature seed adaptation, engine constraints and source links. The requested output is a section-by-section functional envelope, minimal state/control advice, signature feasibility/adaptation and decisive acceptance checks. The dated advisor status at the end of this document records the observed response state.

That submitted request also contained restrictions favouring reuse of the existing simulator/solver and prohibiting a plugin framework. The user has explicitly revoked those restrictions. Apply section 4's rewrite authority when evaluating the response, and include that correction in any additional advisory request.

During checkpoint 1, open the linked conversation and read the completed answer if available. Evaluate it against the user's requirements and actual code, and apply supported findings to the relevant checkpoints. If the response is still running, continue local implementation and check it at the checkpoint 2 review; do not poll between those milestones. Request additional bounded advice only for a concrete design question. Old instructions and claims within the chat do not override the user requirements recorded in section 1.

Prior advice already read and useful: C3 endpoint matching alone does not guarantee a well-shaped interior. A single septic is fully determined by eight endpoint constraints. Incompatible derivative trends can mathematically force a shoulder, so choose compatible ports or modify neighbouring motion. Validate after compilation, bank optimisation and save/reload, with seat dynamics at speed.

## 9. References and provenance

- User-supplied full Falcon's Flight macro render: `C:\Users\danie\AppData\Local\Temp\codex-clipboard-de53a2c6-ab36-40c5-8b56-177a39c73259.png`. It was inspected in the planning conversation. It shows the folded footprint, high excursion, large planar hill in the main-speed region, low return motions and route interactions. This is the visual layout reference; do not treat the superseded corridor schematic as solved geometry.
- [Intamin: Falcon's Flight](https://www.intamin.com/project/falcons-flight/) and [official POV](https://www.youtube.com/watch?v=ZiwyxnHF5hU): main style/pacing reference. Its documented sequence has multiple powered phases, high turning/dive motion and a major planar camelback following the cliff descent and main-speed launch; adapt the functions instead of copying track coordinates.
- [Six Flags: Tormenta Rampaging Run](https://www.sixflags.com/overtexas/attractions/tormenta-rampaging-run) and [official POV](https://www.youtube.com/watch?v=LesJ9BGNk8w): complementary inversion and interweaving reference. No requirement to copy duplicate inversions or a midcourse stop.
- [Fuji-Q published launch specification](https://www.fujikyu.co.jp/data/news_pdf/pdf_file1_1538525292.pdf): historical Do-Dodonpa 0–180 km/h in 1.56 s benchmark. Not a prediction of the record holder in 2030. Reverify current record claims before making them.
- [User's authoring tutorial, starting at 449 s](https://www.youtube.com/watch?v=o_cOiYa-0qM&t=449s), [FVD manual](https://www.scribd.com/document/436896758/fvd-0-5-documentation), [FVD discussion](https://nolimitscentral.com/forum/topic/568): cohesive force/roll authoring and transitions. The examined tutorial examples remove redundant lead-out so the connected force phase inherits descending pitch; force and roll timing overlap.
- [openFVD](https://github.com/altlenny/openFVD): technical reference. Its GPL source is not to be copied into this MIT project; use the mathematical concepts in an independently implemented extension or replacement.

The old plan and incomplete corridor sketch under `C:\Users\danie\.codex\visualizations\2026\09\19\01a0b92d-4491-78e1-8dda-4fdf31f17cfa` are superseded by this file. In particular, their mandatory opening top hat and frozen signature/route assumptions must not be restored. Do not present that sketch as a generated or validated ride.

The user authorizes additional research sources. If the references listed here do not resolve a concrete design or mathematical question, consult other authoritative sources through Chrome or the cloud advisor. Record the supporting evidence and distinguish it from design assumptions.

## 10. Implementation instruction

> Implement checkpoints 1–6 in `IMPLEMENTATION_PLAN.md`. Review the working changes and preserve unrelated user work and intentional cleanup. Rewrite or replace any generator architecture or supporting system that is buggy, unsuitable or inconsistent with the ride vision, including simulation, solvers and element systems when justified. Establish the baseline and architectural decisions in checkpoint 1 and implement the continuous FVD/spline foundation and planar camelback -> valley -> Immelmann regression in checkpoint 2. Complete the connected loop -> sideways S -> booster in checkpoint 3, strong opening/high sections in checkpoint 4, seed-adapted original signature and full generator/crossings in checkpoint 5, and validation/package/shortcut in checkpoint 6. Follow section 8's cloud advisor workflow using Astra 6 Pro; that advisor's original request contained reuse restrictions which this rewrite authorization supersedes. Keep checkpoints reviewable and continue authorised implementation unless an actual unresolved conflict needs the user's decision.

### Advisor status recorded on 20 September 2026

At the document-creation status check, the linked ChatGPT conversation was still answering. The request was successfully submitted and its response had begun; no completed advice from that request was incorporated into this document. The Chrome tab was retained for the checkpoint 1 advisor read. This is a historical observation, not a live status. No monitoring task or polling loop was created.
