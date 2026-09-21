**VibeCoaster2 — final default-generator delivery**

`2.0.0-default.1`, code commit [`a532c8c`](https://github.com/Banrs/VibeCoaster2/commit/a532c8c8aeb02e20ce27a31372556bc6cc9cfccc), is delivered. Play targets the exact packaged executable tested on the GPU. Its SHA-256 is `7940299204ABE6B92C9BF44879DD2876EAFE1264494021EB969E3E4382A53F1E`. [Shortcut proof](07-play-target.json) and [machine-readable evidence](07-default.json) retain the paths, identities and measurements.

**Measured outcomes**

These timings run from a generate/load request to the accepted scene. Each comparison used one separately reported first process and five warm new-process repetitions per build, alternating their order on the same machine. No compiler, local native tests or second game ran concurrently. OS and driver caches were not flushed.

| Highlands, seed 42 | Escarpment warm median | Final warm median | Final warm maximum | Improvement |
|---|---:|---:|---:|---:|
| Generation | 31.476 s | **9.675 s** | 9.822 s | **69.3%** |
| Saved loading | 8.771 s | **3.793 s** | 3.857 s | **56.7%** |

The 16 s generation, 5 s in-app loading and 40% improvement gates pass. First-process request-to-scene observations were 31.440→9.706 s for generation and 8.796→3.761 s for loading. Whole-process launch through instrumented first motion is a separate measurement: the final warm medians were 11.249 s generation and 5.376 s loading, with maxima 11.400/5.468 s. Those figures include engine startup and verification work.

| Flat seed 42 control | 0.9 warm median | Final warm median | Final warm maximum |
|---|---:|---:|---:|
| Generation | **7.879 s** | 8.728 s | 8.796 s |
| Saved loading | 4.940 s | **3.547 s** | 3.657 s |

The 0.9 control generates faster. Its ride and validation scope differ; it is separate from the matched Escarpment acceptance comparison.

The final Highlands generation has an 8.965 s median native phase, 0.096 s mesh preparation and 0.137 s scene commit. Saved validation takes 3.080 s natively, including 0.437 s parsing; mesh preparation takes 0.094 s and scene commit 0.150 s. Acceptance phase wall intervals total a 2.462 s median during generation and 2.643 s during loading; generation also has a 0.804 s structure-preparation phase. Independent checks overlap, so per-phase figures describe wall intervals rather than isolated CPU costs. Accepted-revision serialization was measured separately at 0.161 s on the final CI runner and 0.119 s locally at the unchanged Rift serializer.

The ride retains the approved sequence: fast departure, rounded opening hill and descending twist, compressed clifftop, slow lip, near-vertical cliff, inclined LSM to 300 km/h, protected planar camelback, rising 180° wave, yawing loop, Immelmann, descending ravine roll and low return.

| Baseline measure | Result |
|---|---:|
| Circuit / complete stop | 8533.921 m / 197.858 s |
| First 180 km/h / maximum speed | 1.397000 s / 300.529 km/h |
| Opening descending drop / maximum pitch | 155.012 m / 56.46° |
| Opening bank / heading change | 38.40° / 56.51° |
| Crest-to-recovery, front / rear | 7.655 s / 7.192 s |
| Clifftop, front / rear | 16.372 s / 17.854 s |
| Gz range / maximum absolute Gy / Gx | −1.422904…4.119501 / 0.683010 / 4.296297 G |
| Maximum vertical / lateral / longitudinal rate | 13.1983 / 4.2977 / 19.6066 g/s |
| Minimum full swept ground clearance | 2.379153 m |

The opening expresses FF's rounded crown, committed descending turn and low recovery at a larger V2 scale. The [paused FF front reference](https://www.youtube.com/watch?v=0vgRPSZv1Gg) places crown-to-recovery at roughly 8 s and the matched clifftop at roughly 20 s, including braking/holding, with about 1 s landmark uncertainty. The clifftop satisfies ≤1.1× that reference and the stricter 20.5 s native cap. Inbank→outbank→inbank motion reaches about +45°/−30° of physical bank.

The protected camelback prefix is retained; the minimum documented pullout/hold changes satisfy its connected acceleration history. The [Rift record](06-rift.json) retains the detailed shape, operating-scenario and recipe demonstration evidence. Terrain now has connected plateau/escarpment forms, eight foothills, four ravines and three broad benches on the shared 8 m rendering/collision surface. Ordinary low corridors average about 4.5 m rail-to-ground; the terrain ascent averages 4.56 m, return valley 4.71 m and inclined LSM 5 m. Large elements and exposed cliff sections are reported separately.

The versioned recipe supports typed parameters, stable IDs, placement intent and protected authored profiles. CLI parameter editing, compatible return-element reordering, save/reload and re-edit all produced accepted rides without changing C++. Recipe/request edits invalidate acceptance; persisted checksums never replace validation. The final package also freshly validated a copied Rift save without changing the saved file.

[Final CI](https://github.com/Banrs/VibeCoaster2/actions/runs/35639066450) passed 14 component suites, 10 integration suites, all 8 seed/style/speed cases, seed variation and fresh saved-file validation. C3 geometry/orientation, independent authoring replay, finite-train energy, swept clearance and temporal/spatial refinement pass. Trims off/full and 20% lower drag pass at 960/1920 Hz. Lower drag reaches −1.50826 G, a 0.551% nominal peak exceedance within the agreed strictly-below 1% allowance; ASTM requirements and 20 g/s rates receive no allowance.

Both final-package GPU runs passed full traversal, pause/restart, saved loading, retained paused poses and generation/mesh/scene/save cancellation. The front run saved successfully. Maximum observed generation/mesh/scene cancellation latencies were 76.4/33.1/16.6 ms. The clean rear run produced 11872 frames at 2560×1440 on RTX 5070 Ti/D3D12: 60.0 mean FPS under a 60 FPS cap, 16.666 ms median, 16.713 ms p99 and 16.870 ms maximum. GPU time was 2.511 ms median and 2.989 ms maximum. Screenshot readback was disabled for that performance run.

Representative captures: [overview](07-overview.png), [opening descent](07-opening.png), [loading feedback](07-loading.png). The loading panel shows phase, elapsed time, measured scene progress or activity, and cancellation guidance while the previous ride stays usable.

**Modeling assumptions**

The [F2291-25 assessment](../acceleration-standard.md) uses the documented upright Class 4/5 restraint, lower-body containment, backrest and headrest case. It assesses filtering, duration envelopes, paired axes, reversals, 100 ms onset and post-negative history over complete front/middle/rear traces. The vehicle model uses six 1500 kg cars, 3.4 m spacing, 1.2 m seat height and the documented drag/rolling assumptions. Force and rate limits remain the separate agreed project envelope.

The saved-readiness gate uses in-app request-to-scene time, consistent with the original loading baseline. “First process” does not mean empty OS/driver caches. Performance is measured on this hardware and scene.

**Remaining limitations**

- Whole-process launch through instrumented first motion has a 5.376 s warm loading median. The 5 s in-app gate does not claim a sub-5-second complete application launch.
- Some nondefault requests reject candidates before acceptance. Seed 77, flowing 7 and the 310 km/h case accepted candidate 2; the last took 45.276 s on CI. The 16 s result is the default baseline benchmark, not an all-seed guarantee. Rejected-candidate costs are retained in the machine-readable record.
- Manufacturer calibration, restraint applicability, subjective ride feel, F2291-26 conformity and whole-standard certification remain unclaimed. Exact FF wave dimensions and a calibrated FF rear trace remain unavailable.
- Full GUI layout editing, VR and detailed art remain later work. Terrain faceting and plain materials are visible; this delivery establishes the generator/terrain engineering foundation.
- OS keyboard input was not driven directly. Input contracts and actual runtime actions passed. The 60 FPS result is neither an uncapped nor VR performance claim.

Rift, Foundation, Escarpment and Highlands remain separately playable. Historical contracts, unrelated work and fallbacks are preserved under `D:\Coding\Codex\vibecoasterlegacy`; [the preservation record](../preservation.json) identifies the verified manifests. The sibling Vibecoasterjs source and Git state were not reset. This continuation used one agent and one compiler worker.
