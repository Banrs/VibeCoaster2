# Inspection (diagnostic plots only)

The plotter uses Python standard-library code and an installed matplotlib with the Agg backend. It reads diagnostic exports without modifying core geometry, Unreal assets or references.

## Usage

```powershell
python native/tools/inspect_rides.py <trace.json|traces_dir> [--out-dir native/artifacts/inspection] [--max-traces 30] [--accepted-only] [--preset NAME] [--summary-dir DIR] [--report-json FILE]
python native/tools/test_inspect_rides.py
```

Input: file or directory (recursive `*.json`, deterministic sorted order, first `--max-traces` rendered).
Coordinate convention (core, right-handed, Z UP): geometry rows `[s,x,y,z,upX,upY,upZ,elementEnum]`, x/y ground (`Terrain::height(x,y)`), z height, up ~`{0,0,1}`. Plots use plan(x,y) equal-aspect, elevation(s,z), 3D(x,y,z). Data never modified.
Summaries: optional evolving sidecars (`<stem>.summary.json`/`<stem>.report.json`, sibling `cli_json/`, `--summary-dir`, `--report-json`) read-only. Acceptance is never inferred from exit codes or filenames — only JSON fields. `--accepted-only` renders solely summaries with `accepted`+`completed` true, `cancelled` false, `errors` empty, plus `--preset` match when given. Otherwise labels are `accepted` (strict conjunction), `rejected` (explicit signals), else `unknown`.

Outputs under `--out-dir` (default `native/artifacts/inspection`): `plots/<ride>.png` (1/ride; compact header + small diagnostic footer), `contact_*.png` (dynamic grid 1x1..2x3, omitted for single-ride runs), `manifest.json` (`sourceSha256->plot`, `summaryPreset`, `invalidFiles`, `counts`).

Malformed/empty/truncated files appear in `invalidFiles`, never silently skipped. Missing input writes a manifest and exits 2.

Example after generating the ride in the root README: `python native/tools/inspect_rides.py native/artifacts/example-v050/trace.json --report-json native/artifacts/example-v050/report.json --out-dir native/artifacts/example-v050/inspection`

## Limitations

Plots support a diagnostic review of canonical route geometry and force profiles. Their summary labels do not independently authenticate a report or rerun convergence; use the release acceptance runner for that evidence. They do not establish rendered POV, game feel, frame rate or complete ride acceptance.
