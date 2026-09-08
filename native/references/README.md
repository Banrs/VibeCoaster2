# Reference tooling (bounded)

Stdlib-only Python. No auth, no bypass.

## Layout
- `raw/` — verbatim bodies (`rideInfo-*.json`, `getRec-*.response.json`) + `.provenance.json` (URL, UTC, SHA-256, `probe_outcome`). Legacy files preserved. A transport failure preserves prior raw; HTTP responses, including denials, are preserved as the current response.
- `processed/rfdb_metadata_summary.json` — peaks are instantaneous only; `raw_trace_status` derived from probes (`observed-false`/`denied`/`non-trace`/`schema-invalid`/`verified-raw`/`not-probed`/`transport-failure`). `false` alone never verifies login need. Empty/whitespace bodies have the separate `empty-response` outcome.
- `processed/benchmark.json` — `unavailable` (no eligible real traces; no fake benchmark).
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
- An eligible reference median remains unavailable. Three user-supplied native recordings now support local diagnostics, but do not establish comparable independent Pantherian recordings or a verified calibration session.
- Median needs ≥3 independent `(recording_id,canonical-hash)` in same `(ride,configuration,seat,device,calibration_id)`; equivalent re-encodings deduped; conflicting data/calibration/quality for the same recording excluded until resolved; missing identity insufficient; smoothing/convention splits flagged. Gaps/rotation/excluded/<10 s ineligible; spikes flag only.


## Calibration identity and conflicting exports

`recording_id` identifies one ride recording. `calibration_id` identifies an unchanged calibration session/profile, so several independent recordings can share it when that calibration remained valid. Recalibration creates a new identity. Do not reuse an ID merely to reach three recordings. Distinct calibration sessions remain separate unless a future explicit review establishes comparability; this tool does not infer equivalence from similar force values.

If a supplied stationary interval conflicts with the canonical 1g convention, or its metadata is invalid, the recording is ineligible. A ride-only trace with an explicitly known calibration may omit an in-trace stationary interval; it stays labeled `unverified` for that local check. Spike magnitude alone still does not exclude data. Conflicting exports of one recording are listed in `conflicts`/`conflicting_recording_ids` and contribute no benchmark value; input ordering cannot choose the preferred export.

## Raw payload recognition

The fetcher preserves unknown bodies without inventing their meaning. `verified-raw` recognizes only this tool's canonical `trace_version: 1` shape: a recording ID matching the requested ID, nonempty ride/device/seat/source/configuration metadata, known rider-axis with-g calibration and calibration ID, and at least two sample objects containing finite `time_s`, `vertical_g`, `lateral_g`, `longitudinal_g` values with strictly increasing times. Identity, axes and timing must all be present. The tests use synthetic instances of that schema, not captured RFDB traces. RFDB's gated native payload schema has not been established as matching it; unfamiliar native shapes remain unverified until a grounded adapter exists.

Structural recognition does not prove measurement provenance, independent recordings or benchmark eligibility. Short/gapped data, contradictory calibration and excluded statistics still require the analyzer's checks. No source peaks are promoted to ten-second exposure, and the eligible benchmark remains unavailable.

## Native Ride Forces files

The grounded local adapter handles the observed seated, nominal 50 Hz `.forces`
JSON format separately from the fetcher's canonical schema:

```powershell
python native/tools/reference_native_forces.py --input "RFDB Data/recording.forces" --out-dir native/artifacts/reference-import/new-recording
```

It retains source hashes, crop indices, calibration vectors and unknown metadata.
The unsmoothed CSV maps RFDB `(y, x, z)` to native `(vertical, -lateral,
longitudinal)` with gravity retained. A separate CSV reproduces the RFDB viewer's
11-sample smoothing and original lateral sign; it is labelled for comparison and
cannot be mistaken for canonical analyzer input. See the adapter's linked
[recorder](https://rideforcesdb.com/record) and
[viewer implementation](https://rideforcesdb.com/js/index.js?v=4681fe0a) sources.

Conversion does not verify device stability, actual sample timing, seat identity
or calibration. `--seat` and `--rfdb-id` may supply observed metadata, never guesses.
The generated manifest remains ineligible until those requirements are reviewed.
User recordings under `RFDB Data/` are ignored by Git and remain local.
