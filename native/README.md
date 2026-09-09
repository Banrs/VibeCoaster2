# VibeCoaster native game

Active development is **0.8.1-intent.1 / COASTER5**, undergoing generator and hardware review. The latest released Windows preview is still **0.8.0-flow.1**; use the download below for that verified package. See [work in progress](WORK_IN_PROGRESS.md) for the distinction.


Released Windows preview: **0.8.0-flow.1 / COASTER5**.

**Windows preview: 0.8.0-flow.1.** [Download and play](https://github.com/Banrs/VibeCoaster2/releases/tag/native-v0.8.0-flow.1) · [Delivery status and controls](DELIVERY.md) · [Build from source](tools/BUILDING.md)

Extract the entire Windows ZIP, then open **Play VibeCoaster.cmd** or
**VibeCoaster.exe**. Unreal Editor, Blender and a compiler are not required to
play. The launcher uses a 1600×900 window capped at 60fps. Keep the Engine and
VibeCoaster folders alongside the EXE.

Choose **PHYSICS-PROOF**, **Canyon**, seed **42**, press Enter to generate, then
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
identity is **0.8.0-flow.1 / COASTER5**; older saves require the preserved older app.

- Canonical G3 septic centreline and C2 orientation shared by dynamics, rendering,
  rider forces, clearance and saved geometry.
- Seeded folded circuits with crossings, a record hill, full pitch loop, true
  Immelmann/dive reversals, banked turns and force-authored crest sections.
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

The 0.8 panel accepted 18/18 proof rides with exact saved replay and independent
convergence; 15/18 reach final braking within the soft 180 s target. Windows
build/cook/package and six UE contracts passed. Complete scripted terrain/seat
runs exercise ride controls and save/reload. See [DELIVERY.md](DELIVERY.md) for
timing, source checks and their limits. Large frozen evidence, failed runs and
historical binaries remain local under ignored artifact folders.

The foundation is not complete: authentic reference data, actual Mac/Metal tests,
more convincing scenery and continuous human POV/keyboard review remain. No ASTM
compliance, structural certification or telemetry calibration is claimed.

[Numerics](core/NUMERICS.md) · [FVD scope](core/FVD.md) ·
[Force guideline scope](core/FORCE_GUIDELINES.md) · [Unreal setup](unreal/README.md) ·
[Mac preparation](unreal/MACOS.md) · [Editable art](art/README.md) ·
[Historical validation](VALIDATION.md)
