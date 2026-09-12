# Native foundation status

**Source48 checkpoint, 12 September 2026:** the source cleanup, shared CMake numeric policy and indexed UE terrain pass complete local qualification: 30 native suites, 44 fixed requests, 12 saved replay/convergence cases, eight organic audits, portable checks and ten real UE contracts. [Qualification](V083_INTEGRATION.md) preserves precise results and failure controls. The user's Falcon's Flight comparison establishes a remaining route-flow issue: long unpowered approaches dominate quiet time. Connecting geometry, energy and actual motor-space ownership require architectural resolution before promotion. Angle is qualitative because its scale is offset/inaccurate. Main remains unmerged; final-head CI and main CI remain required.

**Current source47, 12 September 2026:** the universal itinerary and shared terrain/source solver are implemented and pushed as `4906de0`. All 30 native suites, Python and portable checks pass. All eight required rides and four additional regressions accept candidate zero with exact replay and independent convergence; all eight required organic audits pass. The full 44-request matrix and three-platform CI are running. [Qualification results, control comparisons and hashes](V083_INTEGRATION.md) distinguish this source from preserved failures. Final documentation, complete matrix, final-commit CI, gated merge and main CI remain required. See [progress](V083_PROGRESS.md) and [handoff](NEXT_CHAT.md). Earlier paragraphs below are historical; no new package is claimed.

**Current production rewrite, 2026-09-12:** follow the active [seven-step completion checklist](V083_PROGRESS.md) and [exact handoff](NEXT_CHAT.md). The latest isolated footprint snapshot31 accepts five of eight required rides, including canyon1 and flat7; hills2's saved distant crossing and organic checks pass independently. Its full suite is21/29. A subsequent sampling correction accepts hills9; the current build also addresses terrain-driven XY sizing and the existing133ms post-airtime transition rule. `itinerary-terrain-footprint-33` is running all eight requests. All failures, source snapshots and binaries remain preserved separately. Production source has not yet adopted these isolated changes. The44-request matrix and final local/CI qualification remain pending; no final integration push or merge has occurred.

**2026-09-11 architecture work:** the earlier common5/booster experiment passes its29 native suites and eight requested candidate-zero save/replay cases, but the user has rejected further fixture-driven route tuning. The [replacement direction](GENERATOR_ARCHITECTURE.md) and completed [whole-codebase simplicity audit](SIMPLICITY_AUDIT.md) govern this pass. Protocol repair is local commit `83f5f35`, with176 Python tests, eight real CLI tests and full160-metric replay of the eight preserved saves passing. Source placement now retains canonical derivatives for every FVD family; its component and full organic-generation suites pass. The general itinerary/route replacement is still under implementation and feasibility review. No push, final integration CI, merge or new package is claimed. Older status below remains historical.

## Active engineering — 0.8.3-flow.1 (not packaged)

For the user's requested chat transition, read [the exact resumption handoff](NEXT_CHAT.md). The unfinished source checkpoint is on `codex/flow-v083-checkpoint`; the shorter common-owner/propulsion results are retained proposals, not the current published executable.

The current work separates per-element design intent from acceptance. Real FF, TRR and Pantherian recordings and reviewed POVs anchor phase character; intended size, speed and load can deliberately exceed them through the physical velocity-squared/curvature relationship. Uniform enlargement or matching reference force magnitudes is not the objective. The [phase atlas](artifacts/flow-intent-v083-20260910/references/element-profiles/ATLAS.md) identifies repeated low-load shoulders despite already large, fast supporting elements.

Implemented numerical work includes the explicit historical higher restraint-dependent force profile, complete signed duration/onset/paired histories, loss-aware airtime/reversal/pullout sources, and one direct affine passive-energy calculation. Terrain source datums are being made local, and selected route sites no longer rerank every alternative during an energy correction. The benchmark-specific extra helix and obsolete terrain rise compensation were removed; measured strict-record acceptance remains required. The [force scope](core/FORCE_GUIDELINES.md) and [modelling requirements](art/MODELLING_BRIEF.md) distinguish this verified historical profile from current-edition or physical-restraint certification.

These changes are still being integrated. The coupled Immelmann replaces separate reversal/pullout solvers and removes the lateral/crest overlays; its actual rolling descent was inspected in the [v27 editor-game review](artifacts/flow-intent-v083-20260910/editor-v27-load-retained-flat42-front/REVIEW.md). That load-only run completes in 130.944 seconds; eight rendered captures were reviewed, with sparse perimeter routing and long boosters still visible. Ordinary save and packaged performance are not established by that run. The [nominal phase trial](artifacts/flow-intent-v083-20260910/nominal-load-v26/REPORT.md) measures +5.032/−1.762g with distinct mild, positive and stronger negative crest character; its independent geometric-crest/telemetry regression passes. These are nominal stronger-phase intentions, not whole-ride averages or clipped forces.

The [general angle-closure study](artifacts/flow-intent-v083-20260910/route-angle-study/REPORT.md) removes unnecessary recovery straights. Its later shared terrain/source-height prototype, with the nominal sources and physically derived turn ramps, accepts flat42 and hills9 at 126.979 and 126.854 seconds through the unchanged 960/1920 gates. The [actual geometry/trace comparison](artifacts/flow-intent-v083-20260910/flow-review-v28.png) still finds 2.97–3.58 seconds of near-1g coasting at the late booster. The common height solve replaces baseline repairs and post-placement translations; canyon domains, station arrival and actual crossing constraints are still under integration. Directly replacing bank smoothing by force alignment was independently rejected on canyon geometry; [the counterexample](artifacts/flow-intent-v083-20260910/fvd-chain/turn-phase-study/height-bank-owner/DECISION.md) justifies retaining that physical smoothing. Full integrated native/UE tests, source/route variety and a fresh package must pass before promotion. Failed experiments and their exact binaries remain under `artifacts/flow-intent-v083-20260910/`.

The published package and launcher remain v082 below. Authentic reference files are present; strict benchmark qualification remains unresolved. No v083 packaged, performance or whole-ride-feel result is claimed.

## Terrain routing and complexity reduction — 0.8.2-terrain.1

Active source jointly places the canyon climb/return against the actual terrain, preserves clear underpasses, gives the station a real straight arrival, sizes the terminal turn from measured speed, and accounts for passive energy when trimming the loop. Ordinary-turn feedback damping removes a demonstrated geometry/energy limit cycle without expanding the eight-rebuild budget. Two obsolete reversing adapters and the unused terrain-jet path were removed; AGENTS.md applies the user's complexity and realism requirements to the entire codebase and future commits.

The final six-request panel accepts all six on candidate zero, saves and independently replays every ride, and passes 960/1920 Hz convergence. Moving times are 140.027–172.440 s, with complete stops at 164.755–207.575 s. Canyon9 still has a local ascent height of about 91 m above terrain, and long approaches remain. This is useful terrain engineering, not completed macro-flow, reference calibration or structural validation. All 26 native suites passed across the full run and targeted fixture rerun; 178 Python tests were discovered (six CLI cases run through CTest), and all seven UE contracts passed. The 6f23a9e Windows package passed two fresh generated riding/save-load runs, selected-frame review, extracted startup and all 48 runtime-file hashes. Older packages remain preserved; see [delivery](DELIVERY.md). [Current evidence](artifacts/terrain-routing-v082-20260910/REPORT.md).

## Focused optimization checkpoint (2026-09-10)

The 0.8.1 generator stores compact placement parameters instead of copying sampled turns for every site, and unreachable old grammar branches are removed without changing seeded random draws. HUD parsing, rail trigonometry and plot grouping avoid repeated work. Three terrain cases have byte-identical plans, traces and saves before/after; observed peak process memory is 3–4 MB lower, with no substantial latency claim. All 26 CTest suites, 20 plot tests and seven UE contracts passed. Six actual editor-game HUD captures were inspected during a complete flat42 traversal. [Evidence and precise scope](artifacts/optimization-v081-20260910/REPORT.md).

This is a behavior-preserving source checkpoint; the published ae09f02 package remains unchanged. Terrain routing engineering is the next separate change: full climb/return placement and the overly delayed canyon terminal descent remain unresolved here.


## Engineering preview — 0.8.1-intent.1, source ae09f02

The numerical checkpoint contains a loss-aware force-authored tall signature, an energy-authored full loop, a joint-force Immelmann with a rolling descent and pullout, bounded finite-train energy feedback, terrain-aware element placement and composed crossing clearance. The canyon climb now unwinds through the trim approach instead of imposing a level reset at the motor/brake boundary. Conventional-train presentation, persisted operation hardware and uncapped native-display defaults have separate committed checkpoints. The current review covers whole-route terrain use, source preservation, physical supply and coherent element/transition forces; passing peak caps alone is insufficient.

The tall-hill/climb integration's targeted eleven-case panel accepts all eleven circuits, with exact saved replay and independent 960/1920 Hz convergence. Time to final braking is 137.708–147.896 s; this excludes the final station stop. The largest authoring speed residual is 0.465348 m/s against the unchanged 0.5 m/s gate. This is a targeted panel, not a population success rate. The source family is explicitly bounded and unsupported targets are reported without clamping. All 26 final local CTest suites passed in 427.64 s, including synthetic-intensity feasibility and the later core contracts. [Integration evidence](artifacts/generator-intent-v081/tall-hill-integration-v1/REPORT.md) preserves earlier failures and corrections.

A real editor-game traversal of production canyon0 completed in 171.013542 s, reaching final braking at 144.397043 s. Nine actual rendered captures were inspected. The source crest pacing and continuous climb improved, but long approaches, a sparse macro footprint and the very high powered turn/support scale still need engineering work. This load-only run verifies neither ordinary save nor keyboard input or packaged FPS. A separate terrain-led itinerary remains an artifact experiment, not the seeded production generator. [Production visual review](artifacts/generator-intent-v081/tall-hill-integration-v1/editor-canyon0-front/REVIEW.md), [experimental itinerary review](artifacts/generator-intent-v081/macro-editor-v8/REVIEW.md), and [earlier integration review](artifacts/generator-intent-v081/REVIEW.md) keep those scopes separate.

GitHub checkpoints are on `main`. The [numerical-source](https://github.com/Banrs/VibeCoaster2/actions/runs/34409644136), [display/launcher](https://github.com/Banrs/VibeCoaster2/actions/runs/34410362411), [hardware/train](https://github.com/Banrs/VibeCoaster2/actions/runs/34410760281), [exact-rate enforcement](https://github.com/Banrs/VibeCoaster2/actions/runs/34412505706), and [tall-hill/climb integration](https://github.com/Banrs/VibeCoaster2/actions/runs/34413788049) checkpoints all passed Windows, Linux and macOS CI. These jobs build/test the portable core and Python tooling; they do not build UE or establish Mac/Metal runtime support.

The current [Windows engineering package](DELIVERY.md) is **0.8.1-intent.1-ae09f02**. Seven UE contracts, actual build/cook/package and three complete packaged generation/traversal/save/load runs passed. Its extracted startup and all 48 runtime hashes were verified. The runtime visual review still finds excessive approaches and support heights, including a nearly level late return above a falling canyon shelf. Source and package verification do not close whole-ride feel or terrain coherence. The next planner change must solve source placement with actual approach/return headings and curvature, avoiding fixed-corridor filler; [the bounded terrain study](artifacts/generator-intent-v081/tall-hill-integration-v1/terrain-anchor-v3/REPORT.md) records useful component placement and rejected whole-circuit closures.

Older 0.8.0 evidence below remains preserved and must not be presented as 0.8.1 verification. ALL RECORDS still requires eligible authentic benchmark recordings; full current F2291 clauses and a Mac/Metal build remain unavailable.

## Released 0.8.0 checkpoint


Active source is **0.8.0-flow.1 / COASTER5**. This iteration blends module joins,
overlaps inversion pitch/roll, sizes transfers from energy, and feeds measured
entry speeds back into FVD authoring. The fixed 18-request proof panel accepts
18/18 with exact save replay and independent convergence; 15/18 reach terminal
braking within 180 seconds, with a maximum of 183.231 seconds. Synthetic intensity
feasibility and impossible-target regressions pass after correcting a helix
descent that invalidated crossing clearance. All 23 CTest suites have passed
across the full run and focused correction rerun; 178 Python tests were discovered
(six real-CLI cases execute separately in CTest). Windows packaging, terrain
tessellation and overview corrections passed engine and packaged checks. Current
iteration evidence is local under `artifacts/flow-v080-20260908/`.

Final Windows traversal timing covers 133,697 frames across three terrain/seat
runs at 1440p: pooled p95 5.021 ms, maximum 20.960 ms and seven frames above
16.667 ms. The extracted portable ZIP passes all 48 runtime hashes and startup.
See [delivery scope and controls](DELIVERY.md). This is measured headroom, not
an every-frame guarantee or a completed foundation.

FVD feedback is one bounded correction, not a converged whole-route force design:
the largest observed source/final entry-speed mismatch is 8.462 m/s. RFDB imports
and public POV observations are diagnostic; eligible benchmark curation remains
unfinished. No strict-record or completed-foundation claim is made. Older saves
require their preserved older application; the 0.8 generator is an exact-version
break. Historical release evidence below is not new 0.8 acceptance evidence.

The active repository now contains only the native C++/UE game. Legacy browser source, including uncommitted work, was verified and retired into ignored `artifacts/browser-retired-20260908/`; it is not an active build dependency. A reproduced two-writer save race is fixed by exclusive per-save temporary files.

## Preserved recovery and 0.7 evidence

Recovery delivery **0.7.3-review.5** adds a 60fps default and a portable Windows EXE bundle; numerical/save identity remains **0.7.2-pacing.2 / COASTER5**. No new performance benchmarks were run while Cities: Skylines 2 was open. [Current download and status](DELIVERY.md). The review.2 evidence below belongs to that previous package.

Application **0.7.3-review.2** has passed real Windows build/cook/package, five UE contracts and four final complete packaged runs. Numerical geometry and persistence remain **0.7.2-pacing.2 / COASTER5**, with existing p2 save compatibility verified. With Cities: Skylines 2 closed, three complete 1440p timing runs measured 152,697 traversal frames: pooled p95 **4.996ms**, maximum **12.862ms**, zero above 16.667 ms. This is focused Windows performance evidence, not completed foundation acceptance. [Review, exact scope and remaining limits](artifacts/review-v073-20260908/REPORT.md).

Previous verified p2 delivery: [connected track webbing and package](artifacts/track-web-v072p2-20260908/REPORT.md). Five UE contracts and build/cook/package passed. Its targeted canyon front-seat traversal/save/load, exact-save native replay and ten inspected packaged frames belong to p2. The [p1 pacing/train report](artifacts/pacing-aero-v072-20260908/REPORT.md) separately records three terrain/seat runs and sixteen inspected frames. The p2 hardware change preserved those three full numerical traces byte-for-byte. These scopes must not be combined into a new application acceptance claim.

The p1 correction introduced speed/energy-sized connecting crests, closer ground placement, locally adaptive supports and the original aerodynamic Blender 5.2.1 train; p2 added canonical connected track webbing. The rejected v071 car and inactive-transit evidence remains preserved. No ASTM compliance or completed foundation is claimed. Authentic I305/Pantherian comparison, applicable full force-standard clauses, actual Mac/Metal verification, continuous human POV/keyboard review and broader performance coverage remain outstanding.

The default terrain clearance request is now **2 m**, deliberately reduced from 4 m at the user's request after complete three-terrain acceptance. The unchanged full train body, reach envelope, extra terrain reserves and continuous certificates remain active; this is not a copied ASTM clearance value. Minimum centreline-to-ground height in the focused final examples is about 5.4–6.0 m, a separate definition. Force caps and mandatory960/1920 Hz acceptance are unchanged. [Standards research and diagnostic scope](core/FORCE_GUIDELINES.md) explain why public F2291 material does not justify a blanket peak-cap increase.

## Correction of the user-rejected layout

The user rejected the 0.6 packaged ride on 2026-09-07: ordinary seed42 still produced a perimeter square with too few elements, extensive flat recovery and a gentle canyon. The procedural ground pattern was also rejected. Numerical and runtime passes did not resolve those design failures. That correction shipped as **0.7.0-folded.1**, replacing the convex route with folded, crossing circuits, richer element coverage and steep canyon walls. Plain lit ground replaces the patterned material. The 0.6 package and evidence remain preserved. The correction has passed real Windows UHT, editor/game compilation, cooking and packaging; its own evidence is linked above. All-records still lacks authentic eligible I305/Pantherian force data and must stay explicitly unavailable.

## Preserved v0.5.0 numerical checkpoint

- All 12 C++ suites and 147 Python tool tests passed, together with portable Unreal coordinate/wrapper checks.

- The fixed 1,000-request matrix accepted 488/500 physics-proof rides (97.6%) and 0/500 all-record requests. Overall acceptance is 48.8%; the overall and strict 95% gates fail. No missing cases, infrastructure failures or timeouts occurred.

- Proof by terrain: flat 167/167, hills 167/167, canyon 154/166. All 12 proof rejections remain recorded; the lower canyon rate is not hidden by the aggregate.

- All 488 accepted saves independently replayed and passed full half-step auditing. Maximum normalized differences were 0.001518% for speed and 0.744337% for assessed force/rate/statistical fields, below the unchanged 1%/2% gates.

- Accepted native generation p95 was 6.602 seconds; replay p95 was 1.791 seconds, with two subprocess workers. This does not establish packaged-game performance.

- Thirty diagnostic samples and five population extremes were actually viewed. Route order varies, but perimeter routing repeats and slow inversion passages need POV assessment. Sustained intensity remains an explicit user ambition. These are not 30 packaged UE POV reviews.

[Validation](VALIDATION.md), [matrix audit](artifacts/matrix-v050-audit/AUDIT.md), [convergence audit](artifacts/convergence-v050-audit/AUDIT.md), and [diagnostic review](artifacts/delivery-v050-draft/VISUAL_REVIEW.md) contain exact evidence and limitations.

## Shared implementation and historical promotion

One G3 septic centerline and C2 quintic frame serve simulation, rendering and clearance. Five-point frame-jet fitting and analytic rider-offset forces remove the identified orientation discontinuities. Adjacent identical authored drive profiles are coalesced, and explicit persisted motor exit fades remove physical force cuts. The default solver is 960 Hz with mandatory 1920 Hz verification. Targets and provisional force limits were not relaxed.

Generation and load require complete traversal, braking/station return, geometry/structure/terrain validation and a passing half-step assessment. Failure or cancellation preserves the accepted design. COASTER5 accepts only the exact current geometry identity; old saves are not silently reinterpreted.

The historical 0.5.0-geometry.2 checkpoint CLI SHA256 is `c828cb51e35fb0232d60910f2874734abc97a98d135e2fb2830cd84e98c8e126`. [The frozen release](artifacts/release-v050-geometry2/source-manifest.json) and [promotion record](artifacts/pre-promotion-v050/promotion.json) bind the source and executable. At promotion, runtime/tools were exact copies; eight documented delivery-document overrides and one package-version metadata line were separate. Subsequent UE integration work does not replace that frozen checkpoint. All reference files and old release files were preserved.

An [accepted example coaster](artifacts/example-v050/ride.coaster) is available with [historical provenance and replay verification](artifacts/example-v050/provenance.json). It uses explicit physics-proof settings and is not an I305 intensity-record claim.

## Remaining completion gates

No eligible authentic I305 raw/processed benchmark set is available. Metadata peaks, manual overlay notes and synthetic tests cannot fill it. All-records must remain unavailable until suitable recordings and provenance/calibration/context are supplied and reviewed.

The earlier missing-engine/toolchain blocker is resolved. UE5.8.2 and VS2022 now build, cook and package the game on Windows. The preserved geometry.2 package passed three real engine contracts and six scripted capture/performance runs across seed42 flat, hills and canyon, including complete playback and accepted save/load. A 2560×1440 viewport at 100% TSR recorded 86,749 frames: pooled p95 4.880 ms, with two canyon hitches above 16.667 ms (maximum37.749ms). Physical display is1920×1080; this is viewport evidence, not a GPU render-target capture. See [actual integration evidence](artifacts/ue58-integration-20260907/REPORT.md). Screenshots of that historical package exposed repetitive perimeter routing, excessive low support outreach and weak ground scale; continuous human POV, full keyboard controls, macOS and strict-reference acceptance remain unverified.

## Preserved organic checkpoint

The user-rejected 0.6 package is preserved at `unreal/Packaged/run-20260907-104744-631/Windows/`. All 17 C++ suite outcomes for that preserved checkpoint passed (16 in its integrated run, then the corrected reference fixture separately), as did all three UE contracts. Six runs of that package passed complete playback, pause/restart and save/load. Three independent full-ride timing passes at a 2560×1440 viewport recorded 137,014 frames: pooled p95 4.810 ms, with four frames above 16.667 ms and a 22.670 ms maximum. Physical monitor is 1920×1080. Eleven screenshots of that package were inspected; this is not continuous human POV review. [The checkpoint report](artifacts/organic-v060-20260907/REPORT.md) records exact scope, identities and failures.

The subsequent 0.6-to-0.7 expansion retained the existing record hill and full pitch loop. That work added shape-variable true Immelmann/dive-loop modules, interior routing, explicit seeded terrain profiles with a 195–225 m canyon shelf, and compact low/intermediate support families. These change generation identity; old saves are not silently reinterpreted. The verified package and source snapshot remain available. All accepted new examples pass the same complete 960/1920 Hz finite-train, terrain, station and exact-support validation. Those early expansion results did not settle layout/presentation quality; later p1/p2 corrections are recorded above. The preserved optional lateral-rate assessment rejected canyon seed 3 when its measured half-step disagreement exceeds the unchanged 2% tolerance; default proof does not certify unassessed axes.

The FVD library now authors four production crest sections from smooth normal-force profiles, with independent point replay and seeded control inputs. The joined circuit is still a hybrid of geometric modules and FVD sections, followed by complete finite-train simulation and mandatory convergence; source force targets are not promised final rider forces. See [FVD scope and equations](core/FVD.md). Full force-profile-driven circuit closure remains future work.

Mac packaging preparation is in `unreal/MACOS.md`; actual macOS compilation, Metal runtime and performance require a Mac host. The authentic I305/Pantherian reference remains missing, so the strict-record gate stays unavailable. Never promote proof results as all-record acceptance.

## Preserved history

The previous active release is archived at `artifacts/release-v030-pacing2/`, and every overwritten active file was backed up before promotion. Frozen `artifacts/integration-experiment/` retains the v0.4 matrix and its 88 failed convergence cases. `generation-repair/`, `geometry-repair/` and the numerical audit preserve the diagnosis and correction evidence. Historical results are not current acceptance claims.
