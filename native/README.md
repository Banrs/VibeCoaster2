# VibeCoaster native game

Active source and the current Windows engineering preview use **0.8.2-terrain.1 / COASTER5**, built from `6f23a9e`. Terrain routing and code simplification are verified within the scope below; whole-ride flow and hardware feasibility remain under review. See [work in progress](WORK_IN_PROGRESS.md).

Fresh profiles use desktop-native borderless resolution, 100% internal resolution and uncapped rendering with VSync off. Saved player settings remain respected, and distribution launchers forward command-line options.


**Windows engineering preview: 0.8.2-terrain.1-6f23a9e.** [Download and play](https://github.com/Banrs/VibeCoaster2/releases/tag/native-v0.8.2-terrain.1-6f23a9e) · [Delivery status and controls](DELIVERY.md) · [Build from source](tools/BUILDING.md)

Extract the entire Windows ZIP, then open **Play VibeCoaster.cmd** or
**VibeCoaster.exe**. Unreal Editor, Blender and a compiler are not required to
play. Keep the Engine and VibeCoaster folders alongside the EXE.

Choose **PHYSICS-PROOF**, **Canyon**, seed **9**, press Enter to generate, then
Space to ride. Up/Down selects a setting; Left/Right changes it. Tab shows setup,
1/2/3 selects front/middle/rear while riding, M overview, R restart, F5 save,
F9 load, and Alt+F4 exit. Other seeds vary the dimensions and placement within the
current folded route family.

**ALL RECORDS is unavailable** until authentic eligible I305/Pantherian reference
recordings are supplied and curated. PHYSICS-PROOF tests the remaining selected
targets; it does not claim an intensity record or silently substitute a benchmark.

## Existing implementation

This is the single active C++/UE5 game. The obsolete TypeScript/browser
implementation was archived and removed at the user's request. Source geometry/save
identity is **0.8.2-terrain.1 / COASTER5**; older releases' saves require the preserved older app.

- Canonical G3 septic centreline and C2 orientation shared by dynamics, rendering,
  rider forces, clearance and saved geometry.
- Seeded folded circuits with crossings, a loss-aware force-authored tall signature,
  full pitch loop, joint-force reversal/pullout, banked turns and force-authored crests.
- Flat, hills and canyon landscapes; canyon shelf settings vary from 195–225 m.
  Actual local-ground height, terrain relief and station-relative height are
  distinct measurements.
- Finite-train dynamics with gravity, drag and explicit force/power-limited drives
  and brakes. Front/middle/rear forces come from simulation, not random values.
- Mandatory 960Hz simulation and 1920Hz verification before acceptance. Rejected
  or incomplete candidates do not become rideable or saveable.
- Adaptive explicit supports, connected track webbing, original modern train and
  texture-free terrain. FVD supplies bounded source sections; full circuit
  acceptance independently evaluates the joined finite-train dynamics.
- Generation/cancellation, pause/restart, three rider positions, overview,
  comparison, telemetry and transactional save/load.

## Current verification and limits

All 26 local CTest suites passed across the full run and targeted fixture rerun.
Six targeted circuits passed independent saved replay and 960/1920 Hz convergence,
reaching final braking in 140.027–172.440 s. Windows build/cook/package and seven
UE contracts passed. Two fresh packaged terrain/seat runs passed complete
traversal and save/load; extracted startup and all 48 runtime hashes were verified.
See [DELIVERY.md](DELIVERY.md) for scope. Frozen evidence, failed runs and older
binaries remain preserved. This package has no new performance acceptance claim.

The foundation is not complete: authentic reference data, actual Mac/Metal tests,
coherent terrain-directed layout/flow, drive/support engineering and continuous human
POV/keyboard review remain. No ASTM compliance, structural certification or
telemetry calibration is claimed.

[Numerics](core/NUMERICS.md) · [FVD scope](core/FVD.md) ·
[Force guideline scope](core/FORCE_GUIDELINES.md) · [Unreal setup](unreal/README.md) ·
[Mac preparation](unreal/MACOS.md) · [Editable art](art/README.md) ·
[Historical validation](VALIDATION.md)
