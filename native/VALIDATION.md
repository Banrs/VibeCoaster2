# Native validation - current review and preserved evidence

**Latest component checkpoint, 2026-09-11:** the v27 coupled-Immelmann source and layout suites pass, and main v28's physical turn-ramp regression passes 3,569 checks. After correcting terminal controller/sizing agreement, the existing drive/source-port/station suite passes 6,055 checks. The v27 editor DLL builds and its retained flat42 load-only run completes in 130.944 s; [eight actual captures were reviewed](artifacts/flow-intent-v083-20260910/editor-v27-load-retained-flat42-front/REVIEW.md). Separate shared-owner prototypes accept flat42/hills9 at 126.979/126.854 s with mandatory convergence, but canyon, station/crossing integration and a complete native/UE pass remain open. These results do not supersede the explicit integration failures below or establish a new packaged release.

**0.8.3-flow.1 is an unfinished source iteration.** Frozen v22 flat42/flat5/hills9 accept and save with mandatory 960/1920 verification; flat42 independently replays exactly. That version's full CTest run passes24/28, with preserved core/station/support-family/organic failures. The v24 terrain integration passes terrain-motion and support-family tests and accepts/replays canyon42/canyon1, but core, station and organic still fail on hills. Later v25/v25b shared-envelope changes restore hills9 while exposing canyon timing/load failures; those are not a green integration. Python discovery finds179 tests:172 pass and seven CLI cases are skipped; the seven real-CLI cases separately pass the v22 CTest invocation. A v23 editor build succeeds; six of seven UE contracts pass, with the canyon42 generation fixture failing inside MeshContract. Its separate flat42 editor-game traversal/save/load and nine-frame review are documented [here](artifacts/flow-intent-v083-20260910/editor-v23-flat42-front/REVIEW.md). The [nominal-load trial](artifacts/flow-intent-v083-20260910/nominal-load-v26/REPORT.md) and [angle-closure trial](artifacts/flow-intent-v083-20260910/route-angle-study/REPORT.md) are individually verified experiments awaiting combined terrain/source validation. See [current status](WORK_IN_PROGRESS.md) and the [per-phase empirical review](artifacts/flow-intent-v083-20260910/references/element-profiles/ATLAS.md). The v082 package has not been replaced; its results below are historical to this source work.

Active **0.8.2-terrain.1** evidence is in [the terrain routing checkpoint](artifacts/terrain-routing-v082-20260910/REPORT.md). Its final six-case panel accepts, independently saves/replays and converges all six rides on candidate zero. All 26 native suites passed across the full run and targeted fixture rerun; 178 Python tests were discovered (six CLI cases run through CTest), and all seven UE contracts passed. The final package passed two fresh generation/traversal/save-load runs, selected-frame review, extracted startup and 48 runtime-file hash checks; see [packaged scope](artifacts/terrain-routing-v082-20260910/package-v16/REVIEW.md). Earlier 0.8.1 evidence below remains historical.

Active **0.8.1-intent.1** validation is in progress under `artifacts/generator-intent-v081/`. The loss-aware tall-hill and coupled canyon ascent/trim integration accepts eleven targeted complete circuits, each with exact saved replay and separate 960/1920 Hz verification. Moving time to final braking is 137.708–147.896 s and the largest authoring speed residual is 0.465348 m/s. Source-family rejection, cancellation, unchanged acceptance gates and terrain-footprint controls are in existing test suites. The initial full integration run passed 24/26; both failures and their corrections remain preserved. The final complete local run passed all 26 CTest suites in 427.64 s, including synthetic-intensity feasibility; synthetic tests do not supply a real reference benchmark. [Detailed integration evidence](artifacts/generator-intent-v081/tall-hill-integration-v1/REPORT.md).

UE5.8 editor compilation and a complete production canyon0 load/traversal passed: 144.397043 s to final braking, 171.013542 s to complete stop. Nine of 27 actual rendered captures were inspected, with long approaches and elevated powered geometry still unresolved. That load-only run did not exercise ordinary save or keyboard input. The independently assembled terrain itinerary remains a separate experiment.

The later **ae09f02 Windows package** passed seven UE contracts, build/cook/package, and three fresh generated flat42/front, hills9/middle and canyon42/rear runs. Complete-stop durations are 166.194, 170.921 and 167.635 s. All three passed traversal, pause/restart/seat pose, ordinary save/load, save cancellation and missing-reference refusal. Extracted startup and all 48 runtime hashes passed. [Delivery scope](DELIVERY.md) and `artifacts/generator-intent-v081/package-v2/` retain exact identities and visual findings. Source [Windows/Linux/macOS CI](https://github.com/Banrs/VibeCoaster2/actions/runs/34413788049) passed. This package has no new FPS acceptance; whole-route feel, physical keyboard review, strict-reference proof and Mac/Metal runtime remain unfinished.

## Released 0.8.0 evidence


Current **0.8.0-flow.1 / COASTER5** evidence is in [DELIVERY.md](DELIVERY.md) and
local `artifacts/flow-v080-20260908/`. The fixed 18-request proof panel accepted
18/18 with independent exact saved replay and convergence; 15/18 meet the soft
180 s moving-duration goal. All 23 CTest suites passed across the full local run
and focused correction rerun; 172 Python tests and six separate real-CLI cases
passed. Windows compilation, six UE contracts, cooking and packaging passed.
Three complete final-package 1440p runs measured 133,697 traversal frames: pooled
p95 5.021 ms, maximum 20.960 ms, seven above 16.667 ms. Hardware, settings and
excluded lifecycle phases are specified in the current delivery report.
Strict-reference and complete-foundation acceptance remain unavailable.

## Preserved recovery and 0.7 evidence

Recovery delivery **0.7.3-review.5** adds a 60fps default and a portable Windows EXE bundle; numerical/save identity remains **0.7.2-pacing.2 / COASTER5**. No new performance benchmarks were run while Cities: Skylines 2 was open. [Current download and status](DELIVERY.md). The review.2 evidence below belongs to that previous package.

Application **0.7.3-review.2** has passed real Windows build/cook/package, five UE contracts and four final complete packaged runs. Numerical geometry and persistence remain **0.7.2-pacing.2 / COASTER5**, with existing p2 save compatibility verified. With Cities: Skylines 2 closed, three complete 1440p timing runs measured 152,697 traversal frames: pooled p95 **4.996ms**, maximum **12.862ms**, zero above 16.667 ms. This is focused Windows performance evidence, not completed foundation acceptance. [Review, exact scope and remaining limits](artifacts/review-v073-20260908/REPORT.md).

Previous verified p2 delivery: [connected track webbing and package](artifacts/track-web-v072p2-20260908/REPORT.md). Five UE contracts and build/cook/package passed. Its targeted canyon front-seat traversal/save/load, exact-save native replay and ten inspected packaged frames belong to p2. The [p1 pacing/train report](artifacts/pacing-aero-v072-20260908/REPORT.md) separately records three terrain/seat runs and sixteen inspected frames. The p2 hardware change preserved those three full numerical traces byte-for-byte. These scopes must not be combined into a new application acceptance claim.

## Preserved 0.5.0-geometry.2 validation

Everything below this heading records the historical geometry.2 checkpoint, including its then-missing UE toolchain. That installation blocker was subsequently resolved. These numerical counts and old omissions are neither current application results nor current installation status.

**The matrix, independent evidence/convergence audits and bounded diagnostic image review are complete. The combined generation gate failed; the physics-proof gate passed.** The results below belong to the exact staged source and portable binaries; they do not establish a packaged Unreal game or a completed foundation.

## Completed source verification

| Check | Observed result |
| --- | --- |
| Staged integrated Zig build | Completed successfully; all 12 permanent C++ suites passed |
| Python tool discovery | 147 tests passed |
| Portable Unreal integration | Coordinate/seat/winding/support contracts passed; Frame, Convergence and DriveProfile wrapper translation units compiled |
| Script/bootstrap checks | Project JSON and PowerShell parsed; editor bootstrap Python syntax parsed |
| Source and historical fixtures | All 104 manifest files unchanged after verification; historical rejection fixtures preserved byte-for-byte |
| Warnings | Two omitted empty-member initializers in test fixtures; no build failure or semantic fix |

Assertion totals are recorded per suite in the [staging validation summary](artifacts/release-v050-geometry2/verification/validation-summary.json). They include repeated samples and mesh/property assertions; they are not a count of independent rides, physics experiments or UE tests. Full [C++](artifacts/release-v050-geometry2/verification/integrated-build-tests.log), [Python](artifacts/release-v050-geometry2/verification/python-tool-tests.log) and [portable UE](artifacts/release-v050-geometry2/verification/unreal-portable.log) logs remain available.

The runtime defaults to 960 Hz and independently verifies 1920 Hz. Every accepted()/generation/save/load path requires both successful simulation and a performed/passed convergence assessment. Both resolutions retain the selected targets and force gates. The separate matrix audit additionally ties the exact coarse report to the frozen replay and checks every accepted save at half its stored step.

## Completed release matrix — overall gate failed

| Item | Status |
| --- | --- |
| Requests completed | 1000: seeds 1–500 physics-proof, 501–1000 all-records; terrain round-robin; 8 candidates; 960 Hz coarse; 2 workers; 120-second generation/replay timeouts |
| Physics-proof accepted / denominator | **488/500 (97.6%)**; 12 rejected |
| Physics-proof by terrain | Flat **167/167 (100%)**; hills **167/167 (100%)**; canyon **154/166 (92.77%)**. All 12 proof rejections are canyon cases. |
| All-records accepted / denominator | **0/500 (0%)**; all 500 rejected with missing-reference diagnostics |
| Overall accepted / denominator | **488/1000 (48.8%)** |
| Missing cases, infrastructure failures, timeouts | **0 / 0 / 0** |
| Explicit 95% overall / physics-proof / all-records gates | **FAIL / PASS / FAIL**; combined gate **FAIL** |
| Independent half-step audit of every accepted save | **PASS: 488/488**, zero failures; exact coarse replay and per-seat statistics verified |
| Inspection bundles | **30/30 saved and actually viewed**, plus five supplemental population extremes; seven selected full-size plots reviewed |
| Accepted generation command latency | p50 **4.067 s**, p95 **6.602 s**; 488 accepted cases |
| Accepted replay command latency | p50 **1.627 s**, p95 **1.791 s**; 488 accepted cases |

These counts and exact raw values come from the [completed matrix summary](artifacts/acceptance-v050-geometry2/summary.json), tied to the frozen CLI hash below. The [independent matrix integrity audit](artifacts/matrix-v050-audit/AUDIT.md) passed. Twelve proof requests remained genuine rejections, all in canyon terrain. The aggregate 97.6% proof rate passes its declared gate while the canyon subset is lower; there was no separately declared per-terrain gate. The raw error histogram contains 14 FORCE_TRANSITION, 7 STALL and 6 CANDIDATE_FAILURE diagnostics, plus 500 REFERENCE_UNAVAILABLE diagnostics; multiple diagnostics can belong to one request, so those counts are not additional failed cases.

Latency measures native subprocess wall-clock generation/replay at two workers. Early integrated build verification overlapped the run. These values do not establish UE scene-commit time, input responsiveness, GPU frame time or packaged performance. No rate from an earlier version or partial run is substituted. Missing reference data is not permission to omit all-records cases or weaken thresholds. Sample plots remain diagnostic geometry/force evidence, not UE POV acceptance.

The [independent half-step summary](artifacts/convergence-v050-audit/full/summary.json) and [per-save results](artifacts/convergence-v050-audit/full/results.json) cover all 488 accepted saves. The largest normalized speed difference was **0.001518%**; the largest checked force/rate/statistical difference was **0.744337%**, within the selected strict 1%/2% tolerances. Coarse reports match the frozen replay exactly except generation wall time. The [actual diagnostic image review](artifacts/delivery-v050-draft/VISUAL_REVIEW.md) records route repetition, prolonged force plateaus and slow inversion passages; this does not change numerical acceptance or establish UE game feel.

## Frozen identities

- CLI SHA256: `c828cb51e35fb0232d60910f2874734abc97a98d135e2fb2830cd84e98c8e126`.

- Independent convergence executable SHA256: `27b36bb30fc4bc6c80e1b896512c1b1e804eff8038d4ed1b39576556f4f8b630`.

- [Source manifest](artifacts/release-v050-geometry2/source-manifest.json) SHA256: `b3316224183352993a972fcbc35d7f7f34d3c8fb54d27a0293974f40f90ba389`.

- [Binary manifest](artifacts/release-v050-geometry2/binary-manifest.json) SHA256: `8c8ffa289e2024f02b3fd05bf152e3a2fe574ef764de964cb654cc3b1f63b150`.

Runtime identity is `0.5.0-geometry.2`; report schema 1, save schema 5 and acceptance policy `validate-required-v4-convergence`. Core/runtime/tool files remain tied to these manifests. Final delivery documents and the separately declared UE ProjectVersion metadata override are not numerical changes to that tested runtime.

## Historical external and model limits

The [historical prerequisite artifact](artifacts/delivery-v050-draft/unreal-prerequisites.json), checked 2026-09-06 18:03 UTC / 7 September Sydney, reports no UE engine, Visual Studio/MSVC or Windows SDK in bounded discovery. An authorized custom engine path can be supplied, but none was found. No installation, sign-in or license acceptance was performed. [The installation audit](artifacts/ue-delivery-audit.md) explains legitimate routes; its old source-feature inventory predates the new station/tower implementation and is not a description of this release.

There is no eligible real I305 force trace benchmark. [Reference status](references/processed/benchmark.json) records that absence. Raw provenance, calibration and comparable recording groups cannot be manufactured from peak-g metadata or synthetic test inputs.

The provisional train model omits articulated suspension/couplers and explicit wheel contacts. Horizontal component-rate limits are unassessed unless configured; vertical rate is analytic at solver samples. The 12 m local-rail adjacency classification remains distinct from the certified nonadjacent-chord/support/station/terrain checks. Canonical certificates are not proofs about all UE tessellated triangles. Force limits, frame-rate targets, ride quality and structural capacity have not received real-world certification.

At that checkpoint, UE UHT/module compilation, asset creation, cooking, packaged startup, all-seat POV, responsiveness/cancellation, saved-ride UX, GPU/frame time and 1440p/60 fps were unverified. Portable CPU evidence could not close those gates; later packaged results are linked above with their own scope.

## Reproduction

```powershell
& .\native\tools\build.ps1 -Compiler .\native\.tools\zig-x86_64-windows-0.14.1\zig.exe -Test
python -B -m unittest discover -s native/tools -p 'test_*.py' -v
python native/tools/acceptance.py --cli native/build/coaster_cli.exe --out-dir native/artifacts/acceptance-v050-reproduce --count 1000 --workers 2 --candidates 8 --step 0.0010416666666666667 --timeout 120 --validate-timeout 120 --samples 30 --min-success 0.95 --min-physics-proof 0.95 --min-all-records 0.95
```

Rebuilding can change an executable hash even with identical source; a reproduced matrix must bind its own binary. The archived [0.3 validation](artifacts/release-v030-pacing2/VALIDATION.md) and [earlier matrix](artifacts/pacing-experiment/acceptance-1000/summary.json) remain unchanged historical records, with their original limitations. They do not establish convergence or success for this release.
