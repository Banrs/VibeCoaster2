# Native foundation status

The active repository now contains only the native C++/UE game. Legacy browser source, including uncommitted work, was verified and retired into ignored `artifacts/browser-retired-20260908/`; it is not an active build dependency. A reproduced two-writer save race is fixed by exclusive per-save temporary files.

Recovery delivery **0.7.3-review.4** adds a 60fps default and a portable Windows EXE bundle; numerical/save identity remains **0.7.2-pacing.2 / COASTER5**. No new performance benchmarks were run while Cities: Skylines 2 was open. [Current download and status](DELIVERY.md). The review.2 evidence below belongs to that previous package.

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
