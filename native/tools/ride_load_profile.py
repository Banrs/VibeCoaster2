#!/usr/bin/env python3
"""Unfiltered CLI presentation-trace diagnostics; never an acceptance/ASTM gate.

Explicit threshold syntax: vertical:gt:2, vertical:lt:-0.5, lateral:lt:-0.5.
Durations use piecewise-linear crossing times strictly inside observed intervals.
Onset is a least-squares slope of observed samples in a centred 100 ms window.
Only full windows are assessed; no filter, resampling or edge extrapolation occurs.
CLI presentation traces are nominally 60 Hz, NOT raw solver or authentic rider data.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
import hashlib
import json
import math
from pathlib import Path

AXES = ("vertical", "lateral", "longitudinal")
SEATS = ("front", "middle", "rear")
WINDOW_SECONDS = 0.1


def finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(label + " must be a finite number")
    return float(value)


def parse_threshold(text):
    try:
        axis, comparison, raw = text.split(":")
        value = float(raw)
    except (ValueError, TypeError) as error:
        raise ValueError("Threshold must be axis:gt|lt:finite_g, got " + repr(text)) from error
    if axis not in AXES or comparison not in ("gt", "lt") or not math.isfinite(value):
        raise ValueError("Invalid signed threshold: " + text)
    return axis, comparison, value


def validate_trace(trace):
    if not isinstance(trace, dict) or trace.get("seatOrder") != list(SEATS):
        raise ValueError("Expected explicit seatOrder front, middle, rear")
    rate = finite_number(trace.get("sampleRateHz"), "sampleRateHz")
    if rate < 20 or rate > 100000:
        raise ValueError("sampleRateHz must be between 20 and 100000 for a 100 ms fit")
    frames = trace.get("frames")
    if not isinstance(frames, list) or len(frames) < 3:
        raise ValueError("Expected at least three trace frames")
    times, forces = [], [[] for _ in SEATS]
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError("Invalid frame " + str(index))
        time = finite_number(frame.get("time"), "frame time")
        if time < 0 or (times and time <= times[-1]):
            raise ValueError("Trace times must be nonnegative and strictly increasing")
        samples = frame.get("seats")
        if not isinstance(samples, list) or len(samples) != 3:
            raise ValueError("Each frame must contain three seats")
        for seat, values in enumerate(samples):
            if not isinstance(values, list) or len(values) != 3:
                raise ValueError("Each seat must contain signed vertical, lateral, longitudinal g")
            forces[seat].append([finite_number(value, "rider force") for value in values])
        times.append(time)
    step = 1.0 / rate
    timing_tolerance = max(1e-9, step * 1e-5)
    for index, (left, right) in enumerate(zip(times, times[1:])):
        interval = right - left
        final = index == len(times) - 2
        if final:
            if interval > step + timing_tolerance:
                raise ValueError("Final interval exceeds the nominal step; refusing edge extrapolation")
        elif abs(interval - step) > timing_tolerance:
            raise ValueError("Nonuniform interior sampling/gap; resampling is not performed")
    if times[-1] - times[0] < WINDOW_SECONDS:
        raise ValueError("Trace is shorter than a complete 100 ms window")
    return times, forces, rate


def threshold_runs(times, values, comparison, threshold):
    """Strict threshold runs, joined across isolated equality points only.

A positive-duration plateau exactly at the threshold breaks a run. Trace-boundary
runs are marked truncated rather than extended beyond the observed timestamps.
"""
    if comparison not in ("gt", "lt") or not math.isfinite(threshold):
        raise ValueError("Invalid threshold comparison")
    passes = (lambda value: value > threshold) if comparison == "gt" else (lambda value: value < threshold)
    extreme = max if comparison == "gt" else min
    runs = []
    for index in range(len(times) - 1):
        start, end = times[index], times[index + 1]
        a, b = values[index], values[index + 1]
        inside_a, inside_b = passes(a), passes(b)
        if not inside_a and not inside_b:
            continue
        if inside_a != inside_b:
            crossing = start + (threshold - a) / (b - a) * (end - start)
            if inside_a:
                end = crossing
            else:
                start = crossing
        if end <= start:
            continue
        value = extreme(a, b)
        if runs and abs(start - runs[-1]["endTime"]) <= 1e-12:
            runs[-1]["endTime"] = end
            runs[-1]["extremeG"] = extreme(runs[-1]["extremeG"], value)
        else:
            runs.append({"startTime": start, "endTime": end, "extremeG": value})
    for run in runs:
        run["durationSeconds"] = run["endTime"] - run["startTime"]
        run["touchesTraceStart"] = abs(run["startTime"] - times[0]) <= 1e-12
        run["touchesTraceEnd"] = abs(run["endTime"] - times[-1]) <= 1e-12
    return runs


def centered_onset(times, values):
    """Return [observed centre time, slope g/s] for fully observed 100 ms windows."""
    half = WINDOW_SECONDS / 2
    # Tolerance only absorbs serialized timestamp roundoff at window membership;
    # the full-window edge eligibility test itself never extends the trace.
    membership_tolerance = 1e-8
    results = []
    for center in times:
        if center - half < times[0] or center + half > times[-1]:
            continue
        begin = bisect_left(times, center - half - membership_tolerance)
        end = bisect_right(times, center + half + membership_tolerance)
        if end - begin < 3:
            continue
        xs = [time - center for time in times[begin:end]]
        baseline = values[begin]
        ys = [value - baseline for value in values[begin:end]]
        mean_x, mean_y = math.fsum(xs)/len(xs), math.fsum(ys)/len(ys)
        denominator = math.fsum((x - mean_x)**2 for x in xs)
        if denominator <= 0:
            raise ValueError("Degenerate onset fit window")
        slope = math.fsum((x - mean_x)*(y - mean_y) for x, y in zip(xs, ys)) / denominator
        if not math.isfinite(slope):
            raise ValueError("Non-finite onset result")
        results.append([center, slope])
    if not results:
        raise ValueError("No observed sample has a complete 100 ms fit window")
    return results


def profile_trace(trace, thresholds, include_onset_series=False):
    times, forces, rate = validate_trace(trace)
    if not thresholds:
        raise ValueError("Supply at least one explicit threshold; there are no default limits")
    parsed = [parse_threshold(value) for value in thresholds]
    if len(set(parsed)) != len(parsed):
        raise ValueError("Duplicate threshold specification")
    result = {
        "schema": "vibecoaster-force-profile-1", "diagnosticOnly": True,
        "acceptanceAssessed": False, "standardComplianceAssessed": False,
        "sourceKind": "Unfiltered CLI presentation samples; not solver-rate raw data or real ride telemetry",
        "sampleRateHzNominal": rate, "frameCount": len(times), "startTime": times[0], "endTime": times[-1],
        "finalIntervalSeconds": times[-1]-times[-2],
        "thresholdMethod": "Strict signed exceedance with linear crossing interpolation inside observed intervals; isolated equality points merge, equality plateaus break runs",
        "onsetMethod": "Historical-method diagnostic: least-squares slope of observed signed samples within a centred 100 ms window; complete windows only; no filtering/resampling/extrapolation",
        "limitations": ["Nominal 60 Hz presentation samples cannot recover solver-rate peaks or short impacts",
                        "Raw-window slopes are not F2291 filtered onset or a compliance assessment",
                        "Explicit thresholds are descriptive bins, not calibrated rider limits",
                        "No force limits, acceptance state, reference evidence, geometry or saved ride is modified"],
        "seats": {},
    }
    for seat, name in enumerate(SEATS):
        axes = {axis: [row[index] for row in forces[seat]] for index, axis in enumerate(AXES)}
        onset = {}
        for axis, values in axes.items():
            series = centered_onset(times, values)
            low, high = min(series, key=lambda row: row[1]), max(series, key=lambda row: row[1])
            entry = {"minimumGps": low[1], "minimumAtTime": low[0], "maximumGps": high[1], "maximumAtTime": high[0],
                     "assessedCenters": len(series), "unassessedEdgeCenters": len(times)-len(series),
                     "firstAssessedTime": series[0][0], "lastAssessedTime": series[-1][0]}
            if include_onset_series:
                entry["seriesTimeSecondsSlopeGps"] = series
            onset[axis] = entry
        bins = []
        for axis, comparison, value in parsed:
            runs = threshold_runs(times, axes[axis], comparison, value)
            bins.append({"axis": axis, "comparison": comparison, "thresholdG": value, "runs": runs,
                         "totalSeconds": math.fsum(run["durationSeconds"] for run in runs),
                         "longestSeconds": max((run["durationSeconds"] for run in runs), default=0.0)})
        result["seats"][name] = {"signedExtremaG": {axis: {"minimum": min(values), "maximum": max(values)} for axis, values in axes.items()},
                                 "forceDurationRuns": bins, "onset100ms": onset}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--threshold", action="append", required=True, help="Repeat axis:gt|lt:g; use --threshold=vertical:lt:-0.5")
    parser.add_argument("--output", type=Path, required=True, help="Fresh output file; existing files are never overwritten")
    parser.add_argument("--include-onset-series", action="store_true")
    args = parser.parse_args(argv)
    try:
        source = args.trace.read_bytes()
        trace = json.loads(source)
        report = profile_trace(trace, args.threshold, args.include_onset_series)
        report["sourcePath"] = str(args.trace.resolve())
        report["sourceSha256"] = hashlib.sha256(source).hexdigest()
        # Exclusive creation preserves earlier successes and failures. Parent must exist.
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
            stream.write("\n")
    except (OSError, ValueError, OverflowError) as error:
        parser.exit(2, "Force-profile diagnostic rejected: " + str(error) + "\n")
    print("Diagnostic written: " + str(args.output.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
