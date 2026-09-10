# Windows engineering preview: 0.8.2-terrain.1-6f23a9e

[Download the current Windows ZIP](https://github.com/Banrs/VibeCoaster2/releases/tag/native-v0.8.2-terrain.1-6f23a9e), extract it completely, then open **Play VibeCoaster.cmd**. The repository launcher points to this verified build. Unreal Editor and Blender are not required. Fresh profiles use native borderless resolution, 100% internal resolution, no FPS cap and VSync off; saved settings remain respected.

Choose **PHYSICS-PROOF**, **Canyon**, seed **9**, Enter to generate and Space to ride. F5 saves, F9 loads, 1/2/3 selects seats, M shows overview and R restarts. ALL RECORDS requires eligible authentic I305/Pantherian recordings.

Source is `6f23a9e`, application/save identity **0.8.2-terrain.1 / COASTER5**. Terrain transfers follow the seeded cliff more closely, crossing clearance preserves clear underpasses, the station has a straight arrival, and turn sizing and loop braking account for actual speed/energy. Unused authoring implementations were removed. Older exact-version saves require their preserved older app.

All 26 local native suites passed across the full run and targeted fixture rerun. Six targeted rides independently saved/replayed and passed 960/1920 Hz convergence, reaching final braking in 140.027-172.440 s. Seven Unreal contracts and actual Windows build/cook/package passed. Two fresh packaged runs passed generation, complete traversal, ordinary save/load, pause/restart and missing-reference refusal: flat42/rear stopped in 164.755 s; canyon9/front in 207.575 s. Seven rendered ride/overview frames and both extracted startup paths were inspected. This is selected-frame review, not continuous human POV/keyboard approval.

All 48 extracted runtime files match their manifest. The extracted game and root bootstrap opened successfully at native 1920x1080, with MaxFPS 0, VSync 0 and ScreenPercentage 100. The ZIP is 338,574,334 bytes; SHA256 `23277570ed9fae6b8eb7ec34a6de43b039eb7166bad890a6fc972751634b7fe8` matches GitHub's uploaded asset digest. [Verification and visual findings](artifacts/terrain-routing-v082-20260910/package-v16/REVIEW.md).

[Three-platform CI for the packaged numerical source](https://github.com/Banrs/VibeCoaster2/actions/runs/34450824684) builds and checks the portable core and Python tooling; it does not build the Unreal game on macOS.

This remains an engineering preview. The canyon layout still has long approaches and a restricted folded footprint, with a sampled 91 m ascent-height outlier, tall signature supports and dark cliff lighting. Authentic strict-record proof, complete F2291/structural assessment, current-package FPS measurement and Mac/Metal runtime verification remain unfinished. Historical performance below does not apply to this package.

## Preserved 0.8.1-intent.1-ae09f02 delivery

The following controls, verification and observations belong to the older package and its former repository launcher.

[Download the current Windows ZIP](https://github.com/Banrs/VibeCoaster2/releases/tag/native-v0.8.1-intent.1-ae09f02), extract it completely, then open **Play VibeCoaster.cmd**. The repository's `native/Play VibeCoaster.cmd` points to this build. Unreal Editor and Blender are not required. Fresh profiles use desktop-native borderless resolution, 100% internal resolution and uncapped rendering with VSync off; saved settings remain respected. The launcher forwards command-line options.

Choose **PHYSICS-PROOF**, terrain and seed, Enter to generate, then Space to ride. F5 saves; F9 loads. Tab toggles setup, 1/2/3 switches seats, M shows overview and R restarts. ALL RECORDS remains explicitly unavailable without eligible authentic I305/Pantherian recordings.

Application and save identity are **0.8.1-intent.1 / COASTER5**, built from `ae09f0284a7055dffe4b374cca9dcb16e1f3f812`. The filename's commit suffix distinguishes older internal bundles; no frozen release was overwritten. This package includes the loss-aware force-authored tall signature, continuous canyon climb/trim transition, bounded whole-route energy feedback, exact-rate acceptance enforcement, conventional open train and persisted operation hardware.

All 26 local CTest suites and [Windows/Linux/macOS core and Python CI](https://github.com/Banrs/VibeCoaster2/actions/runs/34413788049) passed. Eleven targeted circuits passed exact saved replay and independent 960/1920 Hz convergence, reaching final braking in 137.708–147.896 s. Seven UE contracts and Windows editor/game compilation, cooking and packaging passed with 195 source/config/art hashes unchanged.

Three fresh packaged generation runs completed traversal, pause/restart/seat-pose, ordinary save/load, save cancellation and missing-reference refusal:

| Case | Complete-stop duration |
|---|---:|
| Flat42, front seat | 166.194 s |
| Hills9, middle seat | 170.921 s |
| Canyon42, rear seat | 167.635 s |

All 48 extracted runtime files match their manifest. Extracted startup was actually viewed and closed normally at desktop-native 1920x1080, with MaxFPS0/VSync0/ScreenPercentage100. The ZIP is 338,570,135 bytes; SHA256 is `41ecfa8f278a47004f499535a761b0675fa7fa5513d3545c92e01f814ed21368`, also verified against GitHub's uploaded asset digest. Local evidence is under `artifacts/generator-intent-v081/package-v2/`.

This remains an engineering preview. Inspected packaged frames show improved crest/inversion pacing and visible hardware, but also long approaches, sparse macro layout, an elevated powered turn and a nearly level return over the canyon's falling terrain. The separate compact terrain itinerary is not in this generator. Continuous human POV/keyboard review, real drive/support engineering, broader seed/customization coverage and Mac/Metal runtime verification remain unfinished. No new FPS acceptance, authentic strict record or F2291 compliance is claimed. The scoped 0.8.0 performance results below do not apply to this package.

## Preserved 0.8.0-flow.1 delivery

The following evidence and launch settings belong to the older package.

[Download the Windows ZIP](https://github.com/Banrs/VibeCoaster2/releases/tag/native-v0.8.0-flow.1).
Extract it completely, then open **Play VibeCoaster.cmd** or **VibeCoaster.exe**.
The launcher uses a 1600×900 window capped at 60fps. Unreal Editor, Blender and
a compiler are not needed to play. Keep Engine and VibeCoaster beside the EXE.

Select **PHYSICS-PROOF**, **Canyon**, seed **42**, then Enter to generate and
Space to ride. Up/Down selects a setting; Left/Right changes it. Tab toggles
setup, 1/2/3 switches seats, M shows overview, R restarts, C compares, T toggles
telemetry, F5 saves, F9 loads, and Alt+F4 exits. If MSVC runtime DLLs are missing,
run the included `Engine/Extras/Redist/en-us/vc_redist.x64.exe`.

**ALL RECORDS remains unavailable without eligible authentic reference recordings.**
PHYSICS-PROOF retains the other selected targets and full numerical acceptance;
it is not an intensity-record claim. This version changes numerical/save identity
to **0.8.0-flow.1 / COASTER5**. Older saves require their preserved older app.

## What changed

Module joins now blend their position and orientation derivatives, Immelmann/dive
pitch and roll overlap, and transfers use entry energy and force limits to set
their length. A bounded correction feeds measured full-train entry speeds back
into FVD authoring. The return route has one broad flyover crest, removing a
redundant rise/valley sequence. Seeded dimensions, crossings, terrain placement,
the record hill and full pitch loop remain.

Banking fixes remove an incorrect angle branch and a rapid near-weightless bank
transition. An intensity-only descent now precedes crossing-clearance planning;
low supports adapt their attachment neck to the actual terrain. None of these
changes relax selected targets, force limits, terrain/train/support clearances,
or mandatory 960/1920 Hz verification.

The canyon's nearby cliff mesh uses 5 m sampling, reducing visible approximation
errors. The overview camera fits the whole circuit to the viewport. The HUD
distinguishes time to final braking from time to a complete stop. Existing train,
track and station assets are retained; this is not a new art pass.

## Verification

- The fixed six-seed, three-terrain panel accepted **18/18** proof rides. All 18
  independently reloaded with exact physical reports and passed saved convergence.
- **15/18** reach final braking within the soft 150–180 s target. The measured
  range is **171.701–183.231 s**; the three longer rides retain a visible warning.
  Full stopping takes longer and is reported separately.
- Maximum assessed force/rate convergence difference was **0.184592%**. Forces
  come from the accepted full-train replay, not a scaled visual effect.
- All 23 CTest entries (22 C++ suites and the six-case Python CLI suite) passed
  across the full local run and focused correction rerun. Python discovery
  separately executed 172 passing tests.
- Windows UHT, editor/game compilation, six UE contracts, cooking and packaging
  passed. Complete scripted terrain/seat runs check ride traversal, pause/restart,
  seat pose, save/reload, save cancellation and missing-reference refusal.
- The portable ZIP passed copy-hash and CRC checks. All 48 extracted runtime
  files matched their manifest; the extracted root EXE loaded the Ride map,
  confirmed its fresh-profile 60fps default and exited successfully.

With Cities: Skylines 2 closed, three complete loaded rides at **2560×1440**
measured **133,697 frames**, pooled wall p95 **5.021 ms**, p99 **5.259 ms** and
maximum **20.960 ms**. **Seven frames exceeded 16.667 ms**; this is substantial
60fps headroom, not an every-frame guarantee. Hardware was Ryzen 9 5900X and
RTX 5070 Ti, driver 610.88; UE Development, D3D12 SM5, ray tracing off, 100%
screen percentage, uncapped with VSync/sound/screenshots off. Startup, generation,
mesh commit and save/load are outside these traversal timings. No hitches were
trimmed. These Windows results do not establish other hardware or Mac performance.

The detailed local evidence is in `artifacts/flow-v080-20260908/`. Its failed
probes, earlier packages and all historical releases remain preserved. This small
panel is not a new 1,000-request population audit.

## Remaining limits

This is an unsigned development preview, **not a completed foundation**. The
generator still uses one folded route family. Some elevated connectors remain
quiet; scenery and the rider's view through the train need further review.
Inspected packaged frames and scripted traversals do not establish continuous
human POV or physical-keyboard acceptance.

FVD feedback is one bounded correction, not a converged whole-route force design:
the largest observed source/final entry-speed mismatch is **8.462 m/s**. Supplied
RFDB recordings are converted with provenance and separate viewer smoothing, but
remain unverified and ineligible for the strict benchmark. No shared-run POV-to-
telemetry alignment, reference median/spread, or ASTM compliance is claimed.
An actual Mac/Metal game build and runtime review still require a Mac engine host.

Source builds: [BUILDING.md](tools/BUILDING.md). Numerical scope:
[NUMERICS.md](core/NUMERICS.md). Mac preparation: [MACOS.md](unreal/MACOS.md).
Original project code remains MIT; [license scope](../LICENSE-SCOPE.md) separates
it from Epic's runtime and third-party data terms.
