# VibeCoaster2 — active engineering contract

This file is the sole active design and acceptance contract. Work in `D:\Coding\Codex\Vibecoaster2`. The approved final default-generator engineering plan is implemented and verified. All three checkpoints are delivered. Archived proposals are reference, not instructions. Preserve the accepted contract below when beginning subsequent work.

**Single-agent work only. Do not start or resume subagents.** Previous workers completed or were stopped. If the user later authorizes delegation again, use Astra/Luna and the [efficient-subagent-waiting skill](C:/Users/danie/.codex/skills/efficient-subagent-waiting/SKILL.md); Luna only on xhigh/max. Rotate model/web contexts after more than one hour of inactivity since last use, not one hour of active work; non-Astra contexts must not exceed272k tokens. Suitable external research belongs in ordinary ChatGPT web, not Work/Codex: faster web models for straightforward work, Pro for difficult synthesis, allowing long responses and roughly one-minute completion checks.

## Approved ride and terrain

- Sequence: **180km/h within1.4s → substantial FF-like opening hill and twisted drop → compressed clifftop act → slow lip → near-vertical cliff → inclined LSM to300km/h → approved asymmetric planar camelback → compact rising180° wave → physically yawing loop → retained/rescaled Immelmann → new descending ravine roll → low return**.
- Make the opening's rounded crest, committed descending turn and low recovery explicit. Heading change and banking belong physically in the descent and lead naturally toward the escarpment. Compare its silhouette and front/rear POV with FF; record drop height, pitch, bank, heading change and duration. Existing twist angles are adjustable engineering parameters.
- Preserve the camelback's approved asymmetric planar silhouette and300km/h sequence. The user authorizes only the minimum necessary scale/crest/pullout changes for F2291-25. Validate its complete acceleration history before freezing downstream geometry. The photo/trace/fit in `docs/references` remain canonical; discarded60/67/80° pitch targets are not instructions.
- Measure matched FF clifftop POV landmarks, including braking and holding. Require duration≤1.1× FF and preserve deliberate inbank → outbank over the edge → inbank motion. Size the180° wave from reference geometry and feasible speed, without inferring dimensions from force ratios.
- Strengthen selected negative-G magnitudes, initially targeting about−1.2…−1.45G subject to duration/history rules. There is no airtime-count quota. Remove repetitive flicks, redundant launches and purposeless uphill connectors. The old loop-aperture dive is rejected as the new signature. Retain and improve the Immelmann; improve the loop's felt yaw.
- Build a connected plateau, escarpment, benches, ravines and foothills before fine routing. Replace isolated terrain ribbons and flat surroundings. Ordinary rail-to-ground height should be around5m or lower where full swept clearance permits; report giant elements separately. Detailed art follows this engineering foundation.
- The intended ride is an organic, multipurpose, plausible2030s design with exceptional funding. FF informs composition, scale and pacing; TRR supplies complementary inversion ideas. Increase intensity through deliberate elements rather than a uniform force multiplier. Manufacturer calibration and subjective feel remain separate limitations.

## Generator and interfaces

- Introduce a versioned `RideRecipe`: ordered elements, stable IDs, typed parameters, placement intent and protected authored profiles. Compile through shared FVD/spline kernels while preserving geometry/orientation derivatives and finite-train energy.
- Save recipes alongside compiled geometry and authoring sources. Edits invalidate acceptance. Demonstrate parameter editing, compatible element reordering and save/reload/re-edit through configuration or CLI without C++ changes. Full GUI layout editing and VR follow later.
- Use typed element roles for audits, dimensions and runtime captures. Consolidate build identity. Keep this one active contract and archive contradictory historical contracts and redundant files.
- Reuse sound prototype31 authoring/placement/optimizer/signed-banking fixes without importing its unfinished route. Refactor/rewrite what is needed to keep code and files coherent and small.
- Provide structured loading phases, elapsed time, meaningful progress and responsive cancellation. Keep the previous ride usable until atomic replacement. Measure generation, acceptance, parsing, serialization, mesh preparation and scene commit separately.
- Optimize repeated immutable evaluation and duplicate validation. Reuse acceptance only for an unchanged accepted in-memory revision; persisted checksums cannot bypass fresh validation.

## Acceptance and delivery

- The selected acceleration basis is **ASTM F2291-25**. Implement filtering, duration envelopes, paired-axis combinations, reversals, onset measurement, transition limits and post-negative history over continuous front/middle/rear traces, with independent boundary fixtures for timing/curve rules. See [the standard record](docs/acceleration-standard.md).
- Nominal caps remain **Gz−1.5…+5, |Gy|≤1.5, |Gx|≤4.5**. Report the agreed **strictly below1% peak-magnitude allowance** separately. It is not a permitted time/sample fraction and never relaxes ASTM requirements. **Every component rate stays≤20g/s**, with no allowance.
- Require C3 geometry and physical orientation, independent authoring replay, finite-train energy, full swept clearance,960/1920Hz temporal convergence and independent spatial refinement.
- Use focused native/Unreal checks and an eight-case representative seed/style/speed corpus including seed77, trims off/full and20% lower drag. Keep CI efficient and avoid duplicate full-route tests that add no distinct coverage.
- Require generation median≤16s, saved-ride readiness median≤5s, and≥40% improvement versus matched Escarpment runs. Use five warm repetitions, separate cold starts and a0.9 control; report observed maxima and rejected-candidate costs, not a misleading five-run p95.
- Deliver three checkpoints: **engineering foundation**, **complete revised ride**, **final generator verification**. Each needs commits/pushes, focused GitHub CI, a fresh versioned package and a verified local shortcut. Use **one compiler worker**.
- Verify actual GPU front/rear traversal, loading, cancellation, save/reload and representative poses including the opening. Record runtime performance. Promote Play only after matching its executable path, SHA-256, packaged identity and tested commit. Preserve usable Escarpment and Highlands fallbacks.
- Report measured outcomes, modeling assumptions and remaining limitations separately. Assume the documented upright restraint/backrest/headrest model. Leave F2291-26 conformity and whole-standard certification unclaimed.

## Delivered implementation evidence

**The final default generator is delivered as `2.0.0-default.1`, tested code commit `a532c8c8aeb02e20ce27a31372556bc6cc9cfccc`. Play targets its exact GPU-verified package.** [Delivery report](docs/checkpoints/07-default.md), [measured evidence](docs/checkpoints/07-default.json) and [shortcut proof](docs/checkpoints/07-play-target.json) separate outcomes, modeling assumptions and remaining limitations.

- Final CI passed14component suites,10integration suites, all8representative cases, seed variation and fresh saved validation. All6Unreal contracts passed. Both real GPU traversals passed pause/restart, loading, saved identity and generation/mesh/scene/save cancellation. The input contract passed; OS keyboard input was not driven directly.
- Baseline8533.921m /197.858s, first180km/h1.397000s, maximum300.529km/h. Gz−1.422904…4.119501, |Gy|0.683010, |Gx|4.296297; maximum component rates13.1983/4.2977/19.6066g/s. Minimum full swept gap2.379153m. Complete F2291-25 histories, C3, independent authorship, finite-train energy and temporal/spatial refinement pass.
- Opening descending drop155.012m, maximum pitch56.46°, bank38.40°, heading change56.51°. Front/rear crest-to-recovery7.655/7.192s. Clifftop16.372/17.854s with deliberate inbank→outbank→inbank. The retained Immelmann, descending ravine roll and direct low return remain.
- Shared8m terrain includes8foothills,4ravines and3benches. Ordinary low corridors average4.5m rail-to-ground; ascent4.56m, return valley4.71m, inclined LSM5m. Giant elements/exposed clifftop sections are reported separately. Broad terrain is connected; faceting and plain materials remain for detailed art.
- Trims off/full and20% lower drag pass960/1920Hz operating checks. Lower-drag Gz−1.50826 is a0.551% nominal peak exceedance; ASTM and20g/s gates are unchanged. CLI recipe edit/reorder/save/reload/re-edit passes, and the final package freshly loads a copied Rift save without altering it.
- Five warm matched request-to-scene medians: generation9.675s (maximum9.822s), loading3.793s (maximum3.857s), improvements69.3%/56.7% over Escarpment. First processes and the0.9 flat control are retained separately. Full process launch through instrumented first motion has a5.376s warm loading median; the5s gate is in-app loading.
- Final clean rear GPU traversal:11872frames, RTX5070Ti/D3D12,2560×1440,60FPS cap; median16.666ms, p9916.713ms, maximum16.870ms. GPU median2.511ms, maximum2.989ms. This is not uncapped or VR acceptance.
- Final package: `dist/2.0.0-default.1/run-20260921-183250-063`; executable SHA256 `7940299204ABE6B92C9BF44879DD2876EAFE1264494021EB969E3E4382A53F1E`; profile `UserData-Default`. Runtime evidence: `native/build/evidence/gpu-default-{front,rear}`. CI: `ci-default-a532c8c`. Benchmarks: `performance-default`.
- Separate Rift, Foundation, Escarpment and Highlands shortcuts remain playable. Rift and the two unpromoted candidate packages were hash-preserved in `D:\Coding\Codex\vibecoasterlegacy\final-generator-preservation-20260921-183443`; only obsolete candidate copies were removed from V2. Foundation is preserved in `foundation-fallback-20260921-175217`. See [preservation](docs/preservation.json).

The recipe and shared FVD/spline compiler form the foundation for full customization. Detailed visuals, full GUI layout editing and VR are subsequent work. Manufacturer calibration, restraint applicability, subjective feel, F2291-26 conformity and whole-standard certification remain unclaimed. Nondefault cases can take longer because candidates are rejected; the16s benchmark is not an all-seed guarantee.

### Protected reference measurements

The protected camelback's isolated result retains crest height224.65476m and ascent/descent pitch59.6774/67.5457° at83.5m/s. It cuts0.75s from the final4.005G hold, releases over0.30s to1.9G and exits at the first rising0.02rad port. The protected prefix differs by at most5.03e−11m. Full footprint grows749.479→791.557m (+5.614%); valley minimum changes−1.27977→−4.49481m relative to entry. Its isolated and connected baseline histories pass. Faster scaled copies shorten only the final positive hold as needed. Source/traces/hash evidence: `native/build/evidence/camelback-feasibility`.

FF opening paused frames show rounded crown around5:51, descent commitment5:53, banked descent5:55 and low recovery5:59 (about1s landmark uncertainty). Source: `https://www.youtube.com/watch?v=0vgRPSZv1Gg`. Exact FF wave dimensions and a calibrated FF rear trace remain unverified; do not infer them from force ratios or claim an exact replica.

## Playable fallbacks and prototype provenance

**Delivered fallback:**2.0.0-escarpment.1. F9 loads, Space rides. Its original21 native suites,6 Unreal contracts,8-case corpus and front/rear GPU traversal passed under its original acceptance. It fails the newly added F2291 temporal assessment; do not claim it passes the new standard checks. See `docs/verification.md` and `docs/checkpoints/04-escarpment.html`.

Preserved Escarpment executable (`Play VibeCoaster2 Escarpment.lnk`):
`dist\2.0.0-escarpment.1\run-20260921-005352-688\Windows\VibeCoaster\Binaries\Win64\VibeCoaster.exe`

SHA256:`397DF917F27129F2A33ADD25552669D3303F457D563182ABFFBD10EACA204408`. Profile:`UserData-Escarpment`. Preserve the Highlands shortcut/package/profile too.

Historical Escarpment observations:32.812556s native generation, about32.80s GPU request-to-commit generation and9.29s saved readiness. Historical0.9 observations: about8.04s generation and4.97s loading. These are individual historical observations, not the new matched benchmark. The~222s figure belonged to unfinished prototype31.

**Prototype31 was not promoted or packaged.** Preserved source: `native/build/evidence/restart-20260921/prototype31-source.zip`, with hash manifest and separate evidence. Its editable reference copy is `C:\Users\danie\.codex\visualizations\2026\09\19\01a0ba64-dd63-7a50-83ac-39e20cee9c3f\macro-work`.

Useful prototype fixes were centralized FVD/spline commits, live derivative jets, free-displacement placement, corrected optimizer iterate selection, FVD-owned orientation, signed negative-G banking and force-aware spline objectives. Its old route/signature remains reference only. Historical seed42 passed the old native gates at300.54km/h,199.234s,8.669km; its180.44m Immelmann and unfinished corpus are not current requirements. Historical seed77 failed component rates, so the prototype corpus was never a full pass.

## Git and preservation

V2 owns `.git` on **`codex/default-generator`**, with origin `https://github.com/Banrs/VibeCoaster2.git`. It retains the remote main ancestor and imports11 unpublished original-project commits without rewriting them. Checkpoint1 code commit is `39cdba3a7515372f6f7ef4a5a495a26fbf456368`; later verification-only commits do not change packaged code identity. Do not reset, stage or rewrite parent-repository or `D:\Coding\Codex\Vibecoasterjs` work.

Starting source/evidence/references/packages/profiles were copied and SHA-256 verified in `D:\Coding\Codex\vibecoasterlegacy\default-generator-start-20260921-111813`:2203 files,10027106656 bytes, complete parent/Vibecoasterjs Git bundles, working/index patches and manifests.

Historical generated notes and duplicate status/restart documents are preserved in `D:\Coding\Codex\vibecoasterlegacy\foundation-pruning-20260921-145134`. [docs/preservation.json](docs/preservation.json) records original paths and SHA-256 hashes. The history zip and cleanup index are outside the active source tree. Approved camelback references and playable fallbacks remain available.

## Tools and references

- Follow `AGENTS.md`. C++20 and Unreal5.8.2; one compiler worker. Canonical version: `native/core/include/coaster/version.hpp`. `default.1` is the verified final engineering checkpoint.
- Python:`C:\Users\danie\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`. Always specify UTF-8 when reading/writing repository text with Python on Windows; use atomic replacement for handoff files.
- VS:`D:\Toolchains\VS2022\VC\Auxiliary\Build\vcvars64.bat`; UE:`D:\Games\Epic Games\UE_5.8`.
- CMake:`D:\Coding\Codex\vibecoasterlegacy\archive-2026-09-13\removed\native\build\tools\cmake\data\bin\cmake.exe`. Ignored helper:`native/build/build-native.ps1` (`-Configure`, optional `-Targets`).
- Package:`native/unreal/scripts/package.ps1`, fresh versioned directory, automation enabled, `-MaxParallelActions 1`. Commit source before final packaging; verify executable hashes, embedded commit and package manifest.
- Primary/reference material: [Intamin FF](https://www.intamin.com/project/falcons-flight/), [openFVD](https://github.com/altlenny/openFVD), [third-person animation](https://www.youtube.com/watch?v=ZYoT4N5K0TE), [telemetry POV](https://www.youtube.com/watch?v=0UaOSBGSx20), [FVD tutorial](https://www.youtube.com/watch?v=o_cOiYa-0qM&t=449s). Use an unobstructed daylight POV for visual judgments. No manufacturer-calibrated full FF load comparison is established.
