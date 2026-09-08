"""CSV force importer / analyzer / aggregator for user-provided traces.

Input contract (never mutated; derived results are written separately):
  CSV columns (required, case-sensitive): time_s, vertical_g, lateral_g,
    longitudinal_g. Extra columns are ignored. Units: seconds and g.
    Convention: calibrated rider-axis SPECIFIC FORCE with-g, vertical
    positive INTO seat, ~1.0 at station. This tool NEVER resamples to
    uniform dt, NEVER subtracts gravity, NEVER normalises timing. Timing
    is integrated as-given with linear interpolation ONLY at window /
    threshold boundaries.

  Manifest (JSON dict): ride, device, seat, source, configuration,
    recording_id, calibration{status, convention, calibration_id,
    smoothing?, frame?, transform?, stationary_reference?, notes?},
    quality{device_rotated?, excluded_statistics?, notes?} (optional).
    calibration.status MUST be "known" and convention MUST be exactly
    "rider-vertical-specific-force-with-g". Device/world axes are rejected
    (no implicit transform; this tool supports no device->rider transform).
    smoothing metadata is retained verbatim. All identity/configuration
    values must be meaningful non-empty strings.

Metric:
  r(t) = max(a_v(t), 0) with a_v piecewise-linear; I(t0) = R(t0+10)-R(t0)
  where R is the exact prefix integral of r; S = max I. Interior maxima
  satisfy r(t0+10)=r(t0); candidates are interval endpoints from merged
  knots (samples, rectified zero-crossings, and those minus 10 s) plus
  every interior derivative-zero root per partition. No spike==record
  assertion.

Thresholds are STRICT (over means >, below means <); constant plateaus
at equality contribute 0.

Aggregation: median/spread ONLY over eligible independent recordings
  sharing (ride, configuration, seat, device, calibration_id) with n>=3.
  Independence key is (recording_id, canonical numeric hash), never raw
  bytes, so re-encoded duplicates cannot count. Missing identity stays
  insufficient/unverified. With no eligible traces, machine-readable
  {"status": "unavailable"}.

Stdlib only.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import datetime
import hashlib
import json
import math
import os
import pathlib
import statistics
import tempfile
from typing import Any, Dict, List, Tuple

WINDOW_S = 10.0
REQUIRED_COLUMNS = ["time_s", "vertical_g", "lateral_g", "longitudinal_g"]
REQUIRED_MANIFEST_KEYS = [
    "ride", "device", "seat", "source", "configuration",
    "recording_id", "calibration",
]
CANONICAL_CONVENTION = "rider-vertical-specific-force-with-g"
SUPPORTED_CONVENTIONS = {CANONICAL_CONVENTION}

SPIKE_VERT_POS = 6.0
SPIKE_VERT_NEG = -2.0
SPIKE_LAT_ABS = 3.0
SPIKE_LON_ABS = 4.0
DEFAULT_MAX_GAP_S = 0.5
MIN_GROUP_N = 3
_EPS = 1e-12


class ForceValidationError(ValueError):
    pass


class CalibrationError(ForceValidationError):
    pass


class OutputAliasError(ForceValidationError):
    pass


def utc_now_iso() -> str:
    return (
        datetime.datetime.now(datetime.timezone.utc)
        .isoformat(timespec="seconds")
        .replace("+00:00", "Z")
    )


def sha256_file(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_samples_sha256(
    times: List[float], vert: List[float], lat: List[float], lon: List[float]
) -> str:
    # Canonical numeric hash: parsed floats via repr, so "1.0"/"1.00"/"1.000"
    # (same recording re-exported) hash identically; raw bytes do not.
    h = hashlib.sha256()
    for t, v, a, o in zip(times, vert, lat, lon):
        h.update(f"{t!r},{v!r},{a!r},{o!r}\n".encode("utf-8"))
    return h.hexdigest()


def load_samples(csv_path: pathlib.Path) -> Tuple[List[float], List[float], List[float], List[float]]:
    try:
        with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                raise ForceValidationError("malformed: missing header row")
            missing = [c for c in REQUIRED_COLUMNS if c not in reader.fieldnames]
            if missing:
                raise ForceValidationError(f"malformed: missing columns {missing}")
            t: List[float] = []
            v: List[float] = []
            lat: List[float] = []
            lon: List[float] = []
            for i, row in enumerate(reader, start=2):
                try:
                    vals = [float(row[c]) for c in REQUIRED_COLUMNS]
                except (TypeError, ValueError):
                    raise ForceValidationError(
                        f"malformed: non-numeric value at CSV line {i}"
                    )
                if any(not math.isfinite(x) for x in vals):
                    raise ForceValidationError(
                        f"malformed: non-finite value at CSV line {i}"
                    )
                t.append(vals[0])
                v.append(vals[1])
                lat.append(vals[2])
                lon.append(vals[3])
    except ForceValidationError:
        raise
    except Exception as exc:
        raise ForceValidationError(f"malformed: cannot read CSV: {exc}")
    if len(t) < 2:
        raise ForceValidationError("malformed: need >=2 samples")
    return t, v, lat, lon


def validate_times(times: List[float], max_gap_s: float = DEFAULT_MAX_GAP_S) -> Dict[str, Any]:
    for i in range(1, len(times)):
        if not times[i] > times[i - 1]:
            raise ForceValidationError(
                f"time-reversal/non-monotonic at index {i}: "
                f"{times[i-1]} -> {times[i]} (strictly increasing required)"
            )
    dts = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    max_gap = max(dts)
    gaps = [
        {"index": i, "t0": times[i], "t1": times[i + 1], "dt": dts[i]}
        for i in range(len(dts))
        if dts[i] > max_gap_s
    ]
    s = sorted(dts)
    mid = len(s) // 2
    median_dt = (s[mid] + s[~mid]) / 2.0 if s else 0.0
    return {
        "n": len(times),
        "t_start": times[0],
        "t_end": times[-1],
        "duration_s": times[-1] - times[0],
        "dt_min": min(dts),
        "dt_max": max_gap,
        "dt_median": median_dt,
        "max_gap_s": max_gap,
        "max_gap_allowed_s": max_gap_s,
        "gaps": gaps,
        "gapped": bool(gaps),
    }


def _meaningful_str(x: Any) -> bool:
    return isinstance(x, str) and bool(x.strip())


def validate_manifest(m: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(m, dict):
        raise ForceValidationError("malformed manifest: must be a JSON object")
    missing = [k for k in REQUIRED_MANIFEST_KEYS if k not in m]
    if missing:
        raise ForceValidationError(f"malformed manifest: missing keys {missing}")
    for k in ("ride", "device", "seat", "source", "configuration", "recording_id"):
        if not _meaningful_str(m.get(k)):
            raise ForceValidationError(
                f"malformed manifest: '{k}' must be a meaningful non-empty string"
            )
    cal = m.get("calibration")
    if not isinstance(cal, dict) or cal.get("status") != "known":
        raise CalibrationError(
            "unknown calibration rejected: manifest.calibration.status must be "
            f"'known'; got {cal!r}. Unknown/non-rider calibration cannot qualify."
        )
    conv = cal.get("convention", "")
    if conv not in SUPPORTED_CONVENTIONS:
        low = str(conv).lower()
        if "device" in low or "world" in low:
            raise CalibrationError(
                f"device/world axes rejected as rider axes (got {conv!r}); "
                "no device->rider transform is supported by this tool."
            )
        raise CalibrationError(
            f"unknown calibration rejected: convention must be exactly "
            f"'{CANONICAL_CONVENTION}' (got {conv!r})."
        )
    frame = str(cal.get("frame", "rider")).lower()
    if frame not in ("rider", "rider-axes", "rider_axes"):
        raise CalibrationError(
            f"non-rider frame rejected (got {cal.get('frame')!r}); "
            "only rider axes supported, no transform available."
        )
    tr = cal.get("transform", None)
    if tr not in (None, "none", "identity", {"type": "identity"}):
        raise CalibrationError(
            f"explicit device->rider transform not supported (got {tr!r})."
        )
    if not _meaningful_str(cal.get("calibration_id")):
        raise ForceValidationError(
            "malformed manifest: calibration.calibration_id must be a "
            "meaningful non-empty string (calibration identity required)."
        )
    return m


def interp_linear(t: float, times: List[float], vals: List[float], seg: int) -> float:
    t0, t1 = times[seg], times[seg + 1]
    v0, v1 = vals[seg], vals[seg + 1]
    if t1 == t0:
        return v0
    f = (t - t0) / (t1 - t0)
    return v0 + f * (v1 - v0)


def _eval_av(t: float, times: List[float], vals: List[float]) -> float:
    i = bisect.bisect_right(times, t) - 1
    i = max(0, min(i, len(times) - 2))
    return interp_linear(t, times, vals, i)


def _eval_r(t: float, times: List[float], vals: List[float]) -> float:
    return max(_eval_av(t, times, vals), 0.0)


def _slope_r(t: float, times: List[float], vals: List[float]) -> float:
    i = bisect.bisect_right(times, t) - 1
    i = max(0, min(i, len(times) - 2))
    t0, t1 = times[i], times[i + 1]
    slope_a = (vals[i + 1] - vals[i]) / (t1 - t0) if t1 != t0 else 0.0
    av = interp_linear(t, times, vals, i)
    if av > _EPS:
        return slope_a
    if av < -_EPS:
        return 0.0
    # Exactly at zero: caller uses interval midpoints so this is a kink;
    # return average is wrong -> signal by returning slope of positive side
    # if approaching from above else 0. Fall back to 0 and let partition
    # endpoints (which include this kink) handle it.
    return 0.0


def _zero_crossings(times: List[float], vals: List[float]) -> List[float]:
    out: List[float] = []
    for i in range(len(times) - 1):
        a, b = vals[i], vals[i + 1]
        if (a < 0 < b) or (a > 0 > b):
            t0, t1 = times[i], times[i + 1]
            out.append(t0 - a * (t1 - t0) / (b - a))
    return out


def _rectified_knots(times: List[float], vals: List[float]) -> List[float]:
    pts = list(times) + _zero_crossings(times, vals)
    pts.sort()
    dedup: List[float] = []
    for p in pts:
        if not dedup or abs(p - dedup[-1]) > 1e-12:
            dedup.append(p)
    return dedup


def _prefix_R(knots: List[float], times: List[float], vals: List[float]):
    r = [_eval_r(k, times, vals) for k in knots]
    R = [0.0]
    for j in range(len(knots) - 1):
        dt = knots[j + 1] - knots[j]
        R.append(R[-1] + 0.5 * (r[j] + r[j + 1]) * dt)
    return r, R


def _eval_R(t: float, knots: List[float], r: List[float], R: List[float]) -> float:
    if t <= knots[0]:
        return R[0]
    if t >= knots[-1]:
        return R[-1]
    j = bisect.bisect_right(knots, t) - 1
    j = max(0, min(j, len(knots) - 2))
    dt = t - knots[j]
    seg = knots[j + 1] - knots[j]
    slope = (r[j + 1] - r[j]) / seg if seg else 0.0
    return R[j] + r[j] * dt + 0.5 * slope * dt * dt


def integral_rectified_window(
    t0: float, times: List[float], vert: List[float], window: float = WINDOW_S
) -> float:
    knots = _rectified_knots(times, vert)
    r, R = _prefix_R(knots, times, vert)
    return _eval_R(t0 + window, knots, r, R) - _eval_R(t0, knots, r, R)


def strongest_10s(times: List[float], vert: List[float], window: float = WINDOW_S) -> Dict[str, Any]:
    duration = times[-1] - times[0]
    if duration < window - _EPS:
        return {
            "status": "insufficient-duration",
            "window_s": window,
            "duration_s": duration,
            "S_g_s": None,
            "t0_s": None,
            "note": f"trace shorter than {window}s; S unavailable, not zero.",
        }
    knots = _rectified_knots(times, vert)
    r, R = _prefix_R(knots, times, vert)
    lo, hi = times[0], times[-1] - window

    def I(t0: float) -> float:
        return _eval_R(t0 + window, knots, r, R) - _eval_R(t0, knots, r, R)

    part = {lo, hi}
    for k in knots:
        if lo - _EPS <= k <= hi + _EPS:
            part.add(min(max(k, lo), hi))
        ks = k - window
        if lo - _EPS <= ks <= hi + _EPS:
            part.add(min(max(ks, lo), hi))
    P = sorted(part)
    cands: List[float] = list(P)
    for i in range(len(P) - 1):
        a, b = P[i], P[i + 1]
        if b - a < 1e-9:
            continue
        mid = 0.5 * (a + b)
        a0 = _slope_r(mid, times, vert)
        a1 = _slope_r(mid + window, times, vert)
        denom = a1 - a0
        if abs(denom) < 1e-15:
            continue
        r0 = _eval_r(a, times, vert)
        r1 = _eval_r(a + window, times, vert)
        t_star = a - (r1 - r0) / denom
        if a + 1e-9 < t_star < b - 1e-9:
            cands.append(t_star)
    best_t0, best_I = lo, I(lo)
    for t0 in cands[1:]:
        val = I(t0)
        if val > best_I + 1e-12:
            best_I, best_t0 = val, t0
    # Handle exactly-10s trace: lo==hi, single candidate.
    return {
        "status": "ok",
        "window_s": window,
        "duration_s": duration,
        "S_g_s": best_I,
        "t0_s": best_t0,
        "t1_s": best_t0 + window,
        "method": (
            "exact max of piecewise-linear max(a_v,0) integral via prefix R; "
            "merged knots (samples, zero-crossings, minus-window) plus "
            "interior roots of r(t0+W)-r(t0)=0 per partition; "
            "no resampling, no gravity renormalisation."
        ),
    }


def time_above(times: List[float], vals: List[float], thr: float, above: bool = True) -> float:
    """Strict durations: over means > thr, below means < thr.

    Constant plateaus exactly at thr contribute 0. Isolated equality at a
    single crossing has zero measure; endpoint equality still counts the
    open interval (full dt) since only one point is excluded.
    """
    total = 0.0
    for i in range(len(times) - 1):
        ta, tb = times[i], times[i + 1]
        va, vb = vals[i], vals[i + 1]
        dt = tb - ta
        if above:
            a_gt = va > thr
            b_gt = vb > thr
            a_eq = va == thr
            b_eq = vb == thr
            if a_gt and b_gt:
                total += dt
            elif (a_gt and b_eq) or (a_eq and b_gt):
                total += dt
            elif a_eq and b_eq:
                total += 0.0
            elif a_gt and vb < thr:
                total += dt * (va - thr) / (va - vb)
            elif va < thr and b_gt:
                total += dt * (vb - thr) / (vb - va)
        else:
            a_lt = va < thr
            b_lt = vb < thr
            a_eq = va == thr
            b_eq = vb == thr
            if a_lt and b_lt:
                total += dt
            elif (a_lt and b_eq) or (a_eq and b_lt):
                total += dt
            elif a_eq and b_eq:
                total += 0.0
            elif a_lt and vb > thr:
                total += dt * (thr - va) / (vb - va)
            elif va > thr and b_lt:
                total += dt * (thr - vb) / (va - vb)
    return total


def longest_threshold_bout(times, vals, threshold, above=True):
    """Longest qualifying interval; zero-duration equality points do not split a bout."""
    best=run=0.0;previous_end=None
    for ta,tb,va,vb in zip(times,times[1:],vals,vals[1:]):
        qa=(va>threshold) if above else (va<threshold)
        qb=(vb>threshold) if above else (vb<threshold)
        if not qa and not qb:
            run=0.0;previous_end=None;continue
        lo,hi=ta,tb
        if qa != qb:
            crossing=ta+(tb-ta)*(threshold-va)/(vb-va)
            if qa:hi=crossing
            else:lo=crossing
        if previous_end is None or abs(lo-previous_end)>1e-12:run=0.0
        run+=hi-lo;best=max(best,run);previous_end=hi
    return best


def rolling_mean_extrema(times: List[float], vals: List[float], window: float) -> Dict[str, Any]:
    """Exact signed piecewise-linear mean extrema over full windows only."""
    if times[-1] - times[0] < window:
        return {"status": "insufficient-duration", "min_g": None, "max_g": None}
    prefix = [0.0]
    for i in range(len(times)-1):
        prefix.append(prefix[-1] + (vals[i]+vals[i+1])*.5*(times[i+1]-times[i]))
    lo, hi = times[0], times[-1]-window
    points = sorted({lo, hi} | {x for t in times for x in (t, t-window) if lo <= x <= hi})
    candidates = list(points)
    def value(t):
        i = min(len(times)-2, max(0,bisect.bisect_right(times,t)-1))
        return interp_linear(t,times,vals,i)
    def slope(t):
        i = min(len(times)-2, max(0,bisect.bisect_right(times,t)-1))
        return (vals[i+1]-vals[i])/(times[i+1]-times[i])
    for a,b in zip(points,points[1:]):
        denom=slope((a+b)/2+window)-slope((a+b)/2)
        if abs(denom)>1e-15:
            root=a-(value(a+window)-value(a))/denom
            if a < root < b:candidates.append(root)
    means=[(_eval_R(t+window,times,vals,prefix)-_eval_R(t,times,vals,prefix))/window for t in candidates]
    return {"status":"ok", "min_g":min(means), "max_g":max(means),
            "method":"exact full-window signed piecewise-linear integral; interior roots included"}


def axis_summary(times: List[float], vals: List[float]) -> Dict[str, Any]:
    duration = times[-1] - times[0]
    integ = sum(
        0.5 * (vals[i] + vals[i + 1]) * (times[i + 1] - times[i])
        for i in range(len(times) - 1)
    )
    return {
        "min_g": min(vals),
        "max_g": max(vals),
        "time_weighted_mean_g": integ / duration if duration > 0 else None,
        "integral_g_s": integ,
        "max_abs_component_rate_g_s": max(abs(b-a)/(tb-ta) for a,b,ta,tb in zip(vals,vals[1:],times,times[1:])),
        "rate_assessment": "unassessed; finite-difference rider-axis rate depends on bandwidth and processing",
        "rolling_mean": {f"{w}s": rolling_mean_extrema(times,vals,w) for w in (1,10)},
    }


def _stationarity_info(
    times: List[float], vert: List[float], manifest: Dict[str, Any]
) -> Dict[str, Any]:
    cal = manifest.get("calibration", {})
    ref = cal.get("stationary_reference", None)
    if ref is None:
        return {
            "status": "unverified",
            "note": (
                "no stationary interval in calibration metadata; a single "
                "near-1g ride sample proves nothing about stationarity."
            ),
        }
    try:
        a, b = ref.get("interval_s", [None, None]) if isinstance(ref, dict) else (None, None)
        exp = ref.get("expected_g", 1.0) if isinstance(ref, dict) else 1.0
        a, b, exp = float(a), float(b), float(exp)
    except Exception:
        return {"status": "invalid-metadata",
                "note": "stationary_reference.interval_s must be [t0,t1] numbers."}
    if not all(math.isfinite(x) for x in (a, b, exp)) or abs(exp - 1.0) > 1e-9:
        return {"status": "invalid-metadata",
                "note": "stationary rider with-g reference must expect finite 1g."}
    if not (times[0] <= a < b <= times[-1]):
        return {"status": "invalid-metadata",
                "note": "stationary interval must lie inside the trace."}
    # Time-weighted mean over [a,b] via exact segment integration.
    num = 0.0
    den = 0.0
    for i in range(len(times) - 1):
        ta, tb = times[i], times[i + 1]
        lo, hi = max(ta, a), min(tb, b)
        if hi <= lo:
            continue
        va = interp_linear(lo, times, vert, i)
        vb = interp_linear(hi, times, vert, i)
        num += 0.5 * (va + vb) * (hi - lo)
        den += hi - lo
    mean = num / den if den else None
    ok = mean is not None and abs(mean - exp) <= 0.2
    return {"status": "ok" if ok else "mismatch", "interval_s": [a, b],
            "expected_g": exp, "mean_g": mean,
            "note": "grounded in calibration stationary interval."}


def analyze_recording(
    csv_path: pathlib.Path,
    manifest: Dict[str, Any],
    max_gap_s: float = DEFAULT_MAX_GAP_S,
) -> Dict[str, Any]:
    manifest = validate_manifest(manifest)
    times, vert, lat, lon = load_samples(csv_path)
    tinfo = validate_times(times, max_gap_s)

    spike = (
        max(vert) > SPIKE_VERT_POS
        or min(vert) < SPIKE_VERT_NEG
        or max(abs(x) for x in lat) > SPIKE_LAT_ABS
        or max(abs(x) for x in lon) > SPIKE_LON_ABS
    )
    q = manifest.get("quality", {}) if isinstance(manifest.get("quality"), dict) else {}
    device_rotated = bool(q.get("device_rotated", False))
    excluded = bool(q.get("excluded_statistics", False))
    stationarity = _stationarity_info(times, vert, manifest)

    s10 = strongest_10s(times, vert)
    durations = {
        "vertical_over_2g_s": time_above(times, vert, 2.0, True),
        "vertical_over_3g_s": time_above(times, vert, 3.0, True),
        "vertical_over_4g_s": time_above(times, vert, 4.0, True),
        "vertical_below_0g_s": time_above(times, vert, 0.0, False),
        "vertical_below_minus0_5g_s": time_above(times, vert, -0.5, False),
        "longest_vertical_below_0g_s": longest_threshold_bout(times,vert,0,False),
        "longest_vertical_over_2g_s": longest_threshold_bout(times,vert,2),
        "longest_vertical_over_3g_s": longest_threshold_bout(times,vert,3),
        "longest_vertical_over_4g_s": longest_threshold_bout(times,vert,4),
    }
    canon = canonical_samples_sha256(times, vert, lat, lon)
    result: Dict[str, Any] = {
        "input_csv": str(csv_path),
        "input_sha256": sha256_file(csv_path),
        "canonical_samples_sha256": canon,
        "recording_id": manifest.get("recording_id"),
        "calibration_id": manifest.get("calibration", {}).get("calibration_id"),
        "analyzed_utc": utc_now_iso(),
        "manifest": manifest,
        "smoothing_retained": manifest.get("calibration", {}).get("smoothing", None),
        "validation": tinfo,
        "flags": {
            "gapped": tinfo["gapped"],
            "spike_review": spike,
            "spike_note": (
                "magnitude-only heuristic; flagged for calibration/seat review, "
                "NOT auto-rejected." if spike else "no spike-range excursion"
            ),
            "stationarity": stationarity,
            "device_rotated": device_rotated,
            "excluded_statistics": excluded,
        },
        "strongest10s": s10,
        "durations_s": durations,
        "axes": {
            "vertical_g": axis_summary(times, vert),
            "lateral_g": axis_summary(times, lat),
            "longitudinal_g": axis_summary(times, lon),
        },
        "gravity_note": "Input assumed calibrated rider with-g (1g at station). No gravity subtracted.",
    }
    reasons: List[str] = []
    if stationarity["status"] not in ("ok", "unverified"):
        reasons.append("stationary calibration " + stationarity["status"])
    if tinfo["gapped"]:
        reasons.append(f"gapped: max dt {tinfo['max_gap_s']:.3f}s > {max_gap_s}s")
    if device_rotated:
        reasons.append("device_rotated per manifest/RFDB flag")
    if excluded:
        reasons.append("excluded_statistics per manifest/RFDB flag")
    if s10.get("status") != "ok":
        reasons.append(str(s10.get("status")))
    result["eligibility"] = {"eligible": not reasons, "reasons": reasons}
    return result


def group_key(analysis: Dict[str, Any]) -> Tuple[str, str, str, str, str]:
    m = analysis.get("manifest", {})
    cal = m.get("calibration", {}) if isinstance(m.get("calibration"), dict) else {}
    return (
        str(m.get("ride")), str(m.get("configuration")), str(m.get("seat")),
        str(m.get("device")), str(cal.get("calibration_id")),
    )


def _analysis_identity_ok(a: Dict[str, Any]) -> Tuple[bool, str]:
    m = a.get("manifest", {})
    try:
        validate_manifest(m)
    except ForceValidationError as exc:
        return False, str(exc)
    if a.get("canonical_samples_sha256") is None:
        return False, "missing canonical hash"
    # Derived analysis files may predate the current eligibility rules. Recheck
    # their evidence instead of letting a stale eligible=true bypass calibration.
    flags = a.get("flags", {})
    stationarity = flags.get("stationarity", {})
    status = stationarity.get("status")
    if status not in ("ok", "unverified"):
        return False, "stationary calibration " + str(status)
    ref = m["calibration"].get("stationary_reference")
    if ref is None:
        if status != "unverified":
            return False, "stationary calibration evidence lacks interval metadata"
    else:
        try:
            start, end = ref["interval_s"]
            expected = float(ref.get("expected_g", 1.0))
            mean = float(stationarity["mean_g"])
            numbers = (float(start), float(end), expected, mean,
                       float(stationarity["expected_g"]))
            valid = (status == "ok" and all(math.isfinite(v) for v in numbers)
                     and numbers[0] < numbers[1]
                     and abs(expected - 1.0) <= 1e-9
                     and abs(numbers[4] - 1.0) <= 1e-9
                     and abs(mean - 1.0) <= 0.2
                     and stationarity["interval_s"] == [start, end])
        except (KeyError, TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            return False, "stationary calibration evidence invalid or inconsistent"
    quality = m.get("quality", {})
    if (flags.get("gapped") or flags.get("device_rotated")
            or flags.get("excluded_statistics")
            or quality.get("device_rotated") or quality.get("excluded_statistics")):
        return False, "quality flags exclude this recording"
    return True, ""


def aggregate_recordings(analyses: List[Dict[str, Any]]) -> Dict[str, Any]:
    # Conflicting exports cannot choose their benchmark contribution by input
    # order. Exclude the recording until its canonical export is resolved.
    def signature(a):
        m = a.get("manifest", {})
        cal = m.get("calibration", {})
        quality = m.get("quality", {})
        semantic = {
            "samples": a.get("canonical_samples_sha256"),
            "group": group_key(a),
            "calibration": {key: cal.get(key) for key in
                ("status", "convention", "frame", "transform", "calibration_id",
                 "smoothing", "stationary_reference")},
            "quality": {key: quality.get(key, False) for key in
                ("device_rotated", "excluded_statistics")},
            "eligibility": a.get("eligibility"),
            "flags": {key: a.get("flags", {}).get(key) for key in
                ("stationarity", "gapped", "device_rotated", "excluded_statistics")},
            "strongest10s": a.get("strongest10s"),
        }
        return json.dumps(semantic, sort_keys=True, separators=(",", ":"))

    by_rid: Dict[str, List[Dict[str, Any]]] = {}
    no_identity: List[Dict[str, Any]] = []
    for a in analyses:
        rid = str(a.get("manifest", {}).get("recording_id", "") or a.get("recording_id", "") or "")
        if not rid.strip():
            no_identity.append(a)
        else:
            by_rid.setdefault(rid, []).append(a)
    indep: List[Dict[str, Any]] = []
    n_dup = 0
    divergent: List[str] = []
    conflicts: List[Dict[str, Any]] = []
    for rid, exports in sorted(by_rid.items()):
        n_dup += len(exports) - 1
        hashes = {a.get("canonical_samples_sha256") for a in exports}
        if len(hashes) > 1:
            divergent.append(rid)
        if len({signature(a) for a in exports}) > 1:
            conflicts.append({"recording_ids": [rid], "exports": len(exports),
                              "reason": "conflicting samples/calibration/quality; recording excluded"})
            continue
        indep.append(exports[0])  # All acceptance-relevant fields are equivalent.

    groups: Dict[Tuple[str, str, str, str, str], List[Dict[str, Any]]] = {}
    for a in indep:
        groups.setdefault(group_key(a), []).append(a)
    for key, members in groups.items():
        by_hash: Dict[Any, List[Dict[str, Any]]] = {}
        for a in members:
            by_hash.setdefault(a.get("canonical_samples_sha256"), []).append(a)
        kept = []
        for exports in by_hash.values():
            n_dup += len(exports) - 1
            if len({signature(a) for a in exports}) > 1:
                conflicts.append({"recording_ids": sorted(a["manifest"]["recording_id"] for a in exports),
                                  "exports": len(exports),
                                  "reason": "duplicate samples with conflicting metadata; recordings excluded"})
                continue
            kept.append(min(exports, key=lambda a: a["manifest"]["recording_id"]))
        groups[key] = sorted(kept, key=lambda a: a["manifest"]["recording_id"])
    # Legacy/no-identity analyses cannot prove independence: report separately.
    out_groups: List[Dict[str, Any]] = []
    for key in sorted(groups):
        ride, config, seat, device, cal_id = key
        members = groups[key]
        eligible: List[Dict[str, Any]] = []
        inelig_reasons: List[str] = []
        for m in members:
            ok_id, why = _analysis_identity_ok(m)
            if not ok_id:
                inelig_reasons.append(why)
                continue
            if not m.get("eligibility", {}).get("eligible"):
                inelig_reasons.extend(m.get("eligibility", {}).get("reasons", ["ineligible"]))
                continue
            eligible.append(m)
        s_vals = [
            m["strongest10s"]["S_g_s"] for m in eligible
            if m.get("strongest10s", {}).get("S_g_s") is not None
        ]
        smooths = sorted({json.dumps(m["manifest"].get("calibration", {}).get("smoothing"), sort_keys=True) for m in eligible})
        convs = sorted({str(m["manifest"].get("calibration", {}).get("convention")) for m in eligible})
        heterogeneous = len(smooths) > 1 or len(convs) > 1
        g: Dict[str, Any] = {
            "ride": ride,
            "configuration": config,
            "seat": seat,
            "device": device,
            "calibration_id": cal_id,
            "n_total": len(members),
            "n_independent": len(members),
            "n_duplicates_skipped": 0,  # per-group dup accounting below
            "n_eligible": len(eligible),
            "eligible_recording_ids": [m["manifest"]["recording_id"] for m in eligible],
            "heterogeneous_smoothing_convention": heterogeneous,
        }
        if not eligible:
            g.update({"status": "missing" if not members else "insufficient",
                      "median_S_g_s": None,
                      "note": "no eligible real traces in this group; "
                              + ("; ".join(inelig_reasons[:3]) if inelig_reasons else "")})
        elif len(eligible) < MIN_GROUP_N:
            g.update({"status": "insufficient",
                      "median_S_g_s": None,
                      "S_values_g_s": sorted(s_vals),
                      "note": f"n={len(eligible)} < {MIN_GROUP_N}; too thin for median."})
        elif heterogeneous:
            s_sorted = sorted(s_vals)
            g.update({
                "status": "heterogeneous-needs-split",
                "median_S_g_s": statistics.median(s_sorted),
                "min_S_g_s": min(s_sorted),
                "max_S_g_s": max(s_sorted),
                "S_values_g_s": s_sorted,
                "note": ("provisional median only; smoothing/convention differ "
                         "-> NOT equivalent, split before any benchmark claim."),
            })
        else:
            s_sorted = sorted(s_vals)
            qs = statistics.quantiles(s_sorted, n=4) if len(s_sorted) >= 4 else []
            g.update({
                "status": "ok",
                "median_S_g_s": statistics.median(s_sorted),
                "min_S_g_s": min(s_sorted),
                "max_S_g_s": max(s_sorted),
                "quartiles_g_s": qs,
                "S_values_g_s": s_sorted,
            })
        out_groups.append(g)
    if no_identity:
        out_groups.append({
            "ride": "(unidentified)", "configuration": "(unidentified)",
            "seat": "(unidentified)", "device": "(unidentified)",
            "calibration_id": "(missing)",
            "n_total": len(no_identity), "n_independent": 0,
            "n_duplicates_skipped": 0, "n_eligible": 0,
            "heterogeneous_smoothing_convention": False,
            "status": "insufficient",
            "median_S_g_s": None,
            "note": "missing recording_id/calibration identity; independence unverified.",
        })
    overall_ok = any(g["status"] == "ok" for g in out_groups)
    return {
        "tool": "reference_forces.aggregate",
        "aggregated_utc": utc_now_iso(),
        "window_s": WINDOW_S,
        "min_group_n": MIN_GROUP_N,
        "n_duplicates_skipped_total": n_dup,
        "divergent_reexports_same_id": divergent,
        "conflicts": sorted(conflicts, key=lambda c: c["recording_ids"]),
        "conflicting_recording_ids": sorted({rid for c in conflicts for rid in c["recording_ids"]}),
        "groups": out_groups,
        "overall": "available" if overall_ok else "unavailable",
    }


def unavailable_benchmark(reason: str) -> Dict[str, Any]:
    return {
        "tool": "reference_forces.benchmark",
        "status": "unavailable",
        "reason": reason,
        "window_s": WINDOW_S,
        "metric": "strongest10s integral of max(vertical_g,0) [g*s]",
        "median_S_g_s": None,
        "n_eligible": 0,
        "note": ("No eligible real traces. Peaks from RFDB metadata do NOT "
                 "substitute for S. Provide calibrated CSVs via importer."),
        "generated_utc": utc_now_iso(),
    }


def _load_manifest_arg(s: str) -> Dict[str, Any]:
    p = pathlib.Path(s)
    if p.exists():
        return json.loads(p.read_text(encoding="utf-8"))
    return json.loads(s)


def _normcase(p: pathlib.Path) -> str:
    try:
        return os.path.normcase(str(p.resolve()))
    except Exception:
        return os.path.normcase(str(pathlib.Path(p).absolute()))


def _same_file(a: pathlib.Path, b: pathlib.Path) -> bool:
    try:
        return os.path.samefile(str(a), str(b))
    except Exception:
        pass
    try:
        return _normcase(a) == _normcase(b)
    except Exception:
        return False


def ensure_out_not_input(out: pathlib.Path, inputs: List[pathlib.Path]) -> None:
    for inp in inputs:
        if _same_file(out, inp):
            raise OutputAliasError(
                f"refusing to overwrite input '{inp}' with --out '{out}' "
                "(resolved alias/symlink/hardlink; raw preserved)."
            )


def atomic_write_json(path: pathlib.Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=".tmp-", suffix=".json")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(json.dumps(obj, indent=2, sort_keys=True) + "\n")
        os.replace(tmp, path)
    finally:
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except Exception:
            pass


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("analyze", help="analyze one CSV + manifest")
    a.add_argument("--csv", required=True)
    a.add_argument("--manifest", required=True,
                   help="manifest JSON file or inline JSON string")
    a.add_argument("--out", required=True)
    a.add_argument("--max-gap-s", type=float, default=DEFAULT_MAX_GAP_S)
    g = sub.add_parser("aggregate", help="aggregate analysis JSONs into benchmark")
    g.add_argument("--inputs", nargs="+", required=True,
                   help="analysis JSON files from `analyze`")
    g.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    if args.cmd == "analyze":
        csv_p = pathlib.Path(args.csv)
        out_p = pathlib.Path(args.out)
        manifest_p = pathlib.Path(args.manifest)
        check: List[pathlib.Path] = [csv_p]
        if manifest_p.exists():
            check.append(manifest_p)
        ensure_out_not_input(out_p, check)
        m = _load_manifest_arg(args.manifest)
        res = analyze_recording(csv_p, m, args.max_gap_s)
        atomic_write_json(out_p, res)
        print(json.dumps({"S_g_s": res["strongest10s"].get("S_g_s"),
                          "eligible": res["eligibility"]}, indent=2))
    else:
        in_ps = [pathlib.Path(p) for p in args.inputs]
        out_p = pathlib.Path(args.out)
        ensure_out_not_input(out_p, in_ps)
        analyses = [json.loads(p.read_text(encoding="utf-8")) for p in in_ps]
        agg = aggregate_recordings(analyses)
        if agg["overall"] == "unavailable":
            bench = unavailable_benchmark(
                "no eligible group reached n>=3 homogeneous recordings")
            bench["aggregation"] = agg
            out = bench
        else:
            out = {"tool": "reference_forces.benchmark", "status": "available",
                   "aggregation": agg, "generated_utc": utc_now_iso()}
        atomic_write_json(out_p, out)
        print(json.dumps({"status": out["status"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
