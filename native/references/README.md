# Reference tooling (bounded)

Stdlib-only Python. No auth, no bypass.

## Layout
- `raw/` — verbatim bodies (`rideInfo-*.json`, `getRec-*.response.json`) + `.provenance.json` (URL, UTC, SHA-256, `probe_outcome`). Legacy files preserved. A transport failure preserves prior raw; HTTP responses, including denials, are preserved as the current response.
- `processed/rfdb_metadata_summary.json` — peaks are instantaneous only; `raw_trace_status` derived from probes (`observed-false`/`denied`/`non-trace`/`schema-invalid`/`verified-raw`/`not-probed`/`transport-failure`). `false` alone never verifies login need. Empty/whitespace bodies have the separate `empty-response` outcome.
- `processed/benchmark.json` — preserved 2026-09-06 result, predating the supplied uploads. Its `unavailable` status is not an inventory of today's raw files; a qualified multi-record median remains unestablished.
- `source_register.json`, `manifest_template.json` (needs `recording_id`, `calibration.calibration_id`), `examples/SYNTHETIC_*` (synthetic only).

## Use
Use Python 3.11 or newer.
```powershell
python native/tools/reference_fetch.py --refs-dir native/references
python native/tools/reference_forces.py analyze --csv YOUR.csv --manifest YOUR.manifest.json --out native/references/processed/YOUR.analysis.json
python native/tools/reference_forces.py aggregate --inputs native/references/processed/*.analysis.json --out native/references/processed/benchmark.json
python native/tools/test_reference_forces.py
python native/tools/test_reference_fetch.py
```
CSV `time_s,vertical_g,lateral_g,longitudinal_g` (rider with-g, +into seat). Manifest convention exactly `rider-vertical-specific-force-with-g`, `frame:rider`, `transform:none`, meaningful `ride/device/seat/source/configuration/recording_id/calibration_id`. Device axes rejected. `--out` aliasing any input (resolved/symlink/hardlink) refused; writes atomic.

## Metric
`S=max ∫max(a_v,0)` over 10 s via prefix-R exact integral; knots = samples + zero-crossings ±10 s + interior `r(t+10)=r(t)` roots. Strict `>`/`<` durations (plateaus 0). No resample/gravity change; smoothing verbatim. Stationarity only from `calibration.stationary_reference.interval_s`, else `unverified`.

## Limits
- The supplied Falcon's Flight 4804, Pantherian 6839 and Tormenta 6383 files contain full traces with usable signed force/duration/exposure observations. [Measured inventory](SPECIFICATIONS.md#supplied-force-observations) and `source_register.json:supplied_recordings` identify each file, source and result. There is one observation per ride; between-recording spread and calibrated measurement uncertainty are unavailable.
- Current code requires ≥3 independent recordings within identical `(ride,configuration,seat,device,calibration_id)`. This is the implemented conservative grouping, not a scientific requirement that validated devices or per-run calibrations must be identical. Explicitly reviewed compatible populations can retain seat/device/calibration as strata; supporting that policy requires a coordinated analyzer/export contract change, not invented shared IDs. No eligibility behavior changed in this documentation correction.
- Equivalent re-encodings count once; conflicting exports remain excluded pending review. Gaps, rotation, exclusion flags and insufficient duration still matter. Show a single usable observation with its missing metadata and unavailable spread instead of describing its data as absent; it cannot satisfy strict multi-record qualification.

## Calibration identity and conflicting exports

`recording_id` identifies one ride recording; `calibration_id` identifies an unchanged calibration session/profile. Recalibration creates a new identity. Do not reuse an ID merely to satisfy the current grouping or infer calibration from similar forces. A stable gravity neighborhood is useful consistency evidence, not proof that the device remained fixed throughout a run.

If a supplied stationary interval conflicts with the canonical 1g convention, or its metadata is invalid, the recording is ineligible. A ride-only trace with an explicitly known calibration may omit an in-trace stationary interval; it stays labeled `unverified` for that local check. Spike magnitude alone still does not exclude data. Conflicting exports of one recording are listed in `conflicts`/`conflicting_recording_ids` and contribute no benchmark value; input ordering cannot choose the preferred export.

## Raw payload recognition

The fetcher preserves unknown bodies without inventing their meaning. `verified-raw` recognizes only this tool's canonical `trace_version: 1` shape: a recording ID matching the requested ID, nonempty ride/device/seat/source/configuration metadata, known rider-axis with-g calibration and calibration ID, and at least two sample objects containing finite `time_s`, `vertical_g`, `lateral_g`, `longitudinal_g` values with strictly increasing times. Identity, axes and timing must all be present. The tests use synthetic instances of that schema, not captured RFDB traces. RFDB's gated native payload schema has not been established as matching it; unfamiliar native shapes remain unverified until a grounded adapter exists.

Structural recognition does not prove provenance, independence or benchmark eligibility. The native adapter below handles the supplied format without requiring the fetcher's canonical wire schema. Exposure can be measured from these full observations; metadata peaks alone cannot supply it.

## Native Ride Forces files

The grounded local adapter handles the observed seated, nominal 50 Hz `.forces`
JSON format separately from the fetcher's canonical schema:

```powershell
python native/tools/reference_native_forces.py --input "RFDB Data/recording.forces" --out-dir native/artifacts/reference-import/new-recording
```

It retains source hashes, crop indices, calibration vectors and unknown metadata.
The unsmoothed CSV uses `vertical=RFDB y`, `lateral=-RFDB x`,
`longitudinal=RFDB z`, with gravity retained. A separate CSV reproduces the RFDB viewer's
11-sample smoothing and original lateral sign; it is labelled for comparison and
cannot be mistaken for canonical analyzer input. See the adapter's linked
[recorder](https://rideforcesdb.com/record) and
[viewer implementation](https://rideforcesdb.com/js/index.js?v=4681fe0a) sources.

Conversion does not verify device stability, actual sample timing, seat identity
or calibration. `--seat` and `--rfdb-id` may supply observed metadata, never guesses.
The generated manifest retains those uncertainties; it does not confer strict
benchmark eligibility. Existing [signed calculations](../artifacts/flow-intent-v083-20260910/references/FINDINGS.md)
use the supplied observations without changing their manifests. User recordings
under `RFDB Data/` are ignored by Git and remain local.
