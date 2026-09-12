# Windows flow preview: 0.8.0-flow.1

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
