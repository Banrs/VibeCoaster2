# VibeCoaster native game

**Windows preview: 0.7.3-review.4.** [Download and play](https://github.com/Banrs/OpenVibeCoaster/releases/tag/native-v0.7.3-review.4) · [Delivery status and controls](DELIVERY.md) · [Build from source](tools/BUILDING.md)

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
implementation was archived and removed at the user's request. Geometry/save identity remains **0.7.2-pacing.2 / COASTER5**, so the
0.7.3 application updates retain compatible p2 saves.

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

The recovery check matched all 50 files of the previous package and all 211
snapshot source files against their saved hashes. Review.3 adds a 60fps default,
removes unused Android file-server settings and provides a player distribution.
No performance benchmark was rerun while Cities: Skylines 2 was open.

Previous review.2 evidence includes five UE contracts, four full packaged runs,
bit-exact simulation comparisons and three historical 1440p timing runs. Those
measurements are not new review.3 performance results. Large frozen evidence,
failed runs and historical binaries remain local under ignored artifact folders.

The foundation is not complete: authentic reference data, actual Mac/Metal tests,
more convincing scenery and continuous human POV/keyboard review remain. No ASTM
compliance, structural certification or telemetry calibration is claimed.

[Numerics](core/NUMERICS.md) · [FVD scope](core/FVD.md) ·
[Force guideline scope](core/FORCE_GUIDELINES.md) · [Unreal setup](unreal/README.md) ·
[Mac preparation](unreal/MACOS.md) · [Editable art](art/README.md) ·
[Historical validation](VALIDATION.md)
