#!/usr/bin/env python3
"""Acceptance orchestration for coaster_cli (stdlib only, callable ready tool).

Pinned core report contract (core/src/persistence.cpp:reportJson,
core/src/main.cpp, core/src/generation.cpp:evaluateTargets):
  schemaVersion==1, generatorVersion=="0.5.0-geometry.2", seed/terrain/preset echo the
  request, intensityRequired==(preset=="all-records"),
  accepted/completed/cancelled are strict booleans, errors==[] iff accepted.
  physics-proof is the ONLY intensity exemption; all-records with
  REFERENCE_UNAVAILABLE (I305 benchmark uncalibrated) is a valid rejection.
  generate exit 0==accepted, 2==rejected, 1==usage/IO.
  validate FILE --json PATH replays + revalidates; only a validate-confirmed
  save counts as accepted. Generation wall time and replay time are recorded
  separately so p95 stays comparable.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

TERRAINS = ("flat", "hills", "canyon")
PRESETS = ("physics-proof", "all-records")
DEFAULT_COUNT = 1000
DEFAULT_WORKERS = 2
DEFAULT_TIMEOUT = 120.0
DEFAULT_CANDIDATES = 8
DEFAULT_STEP = 1.0 / 960.0

CASE_JSON_VERSION = 3
MANIFEST_VERSION = 3
REPORT_SCHEMA_VERSION = 1
EXPECTED_GENERATOR_VERSION = "0.5.0-geometry.2"
VALIDATION_POLICY = "validate-required-v4-convergence"


def build_manifest(count=DEFAULT_COUNT, candidates=DEFAULT_CANDIDATES,
                   step=DEFAULT_STEP):
    """Predeclare `count` cases deterministically (half/half, round-robin)."""
    if count <= 0:
        raise ValueError("count must be > 0")
    half = count // 2
    cases = []
    for i in range(count):
        preset = "physics-proof" if i < half else "all-records"
        cases.append({
            "index": i,
            "id": f"case_{i:04d}",
            "seed": i + 1,
            "preset": preset,
            "terrain": TERRAINS[i % len(TERRAINS)],
            "candidates": int(candidates),
            "step": float(step),
        })
    return cases


# ---- file / hash helpers (stdlib only) ----

def sha256_file(path):
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def cli_identity_sha256(cli):
    """Hash the executable bytes. Accepts str path or [interp, script] (tests)."""
    if isinstance(cli, (list, tuple)):
        for part in cli:
            p = Path(str(part))
            if p.is_file() and not p.is_symlink():
                # Skip interpreters; hash the first real script/exe.
                if p.suffix == ".py" or p.stat().st_size > 0:
                    digest = sha256_file(p)
                    if digest and p.suffix == ".py":
                        return digest
        # Fall back: hash the joined command string (test-only path).
        return hashlib.sha256(" ".join(str(c) for c in cli).encode()).hexdigest()
    return sha256_file(str(cli))


def is_finite_number(x):
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        return False
    try:
        return math.isfinite(float(x))
    except (OverflowError, ValueError, TypeError):
        return False


def is_finite_nonneg(x):
    return is_finite_number(x) and x >= 0


def is_valid_file(path, min_size=1):
    try:
        p = Path(path)
        if p.is_symlink():
            return False
        if not p.is_file():
            return False
        return p.stat().st_size >= min_size
    except OSError:
        return False


def _resolved_path(path):
    resolved = Path(path).resolve()
    if os.name == "nt":
        name = str(resolved)
        prefix = chr(92)*2 + "?" + chr(92)
        if name.startswith(prefix):
            # Windows resolution may retain its extended prefix while another
            # worker creates an ancestor. Normalize only equivalent DOS/UNC
            # spellings, after resolution; never fall back to the input path.
            tail = name[len(prefix):]
            if tail[:4].upper() == "UNC" + chr(92):
                resolved = Path(chr(92)*2 + tail[4:])
            elif (len(tail) >= 3 and tail[0].upper() in "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                  and tail[1:3] == ":" + chr(92)):
                resolved = Path(tail)
            else:
                raise ValueError("unsupported resolved Windows device path")
    return resolved


def resolve_within(base, target):
    """Fail closed on resolution errors or any resolved escape through links."""
    try:
        b = _resolved_path(base)
        t = _resolved_path(target)
        return t if t == b or b in t.parents else None
    except (OSError, RuntimeError, ValueError):
        return None


def safe_remove(path, base):
    """Remove a verified descendant; never remove base or traverse an escape."""
    try:
        p = Path(path)
        resolved = resolve_within(base, p)
        if resolved is None or resolved == _resolved_path(base):
            return False
        if not p.exists() and not p.is_symlink():
            return True
        if p.is_symlink():
            p.unlink()
        elif getattr(p, "is_junction", lambda: False)():
            p.rmdir()  # Remove only this verified junction, not its contents.
        elif p.is_file():
            p.unlink()
        elif p.is_dir():
            shutil.rmtree(p)  # Python 3.8+ does not traverse nested Windows junctions.
        else:
            return False
        return True
    except (OSError, RuntimeError, ValueError):
        return False

def atomic_write_text(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp-%d" % os.getpid())
    try:
        tmp.write_text(text, encoding="utf-8")
        os.replace(str(tmp), str(path))
    finally:
        try:
            if tmp.exists():
                tmp.unlink()
        except OSError:
            pass


def atomic_write_json(path, obj):
    atomic_write_text(path, json.dumps(obj, indent=2))


def atomic_copy(src, dst):
    dst = Path(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_name(dst.name + ".tmp-%d" % os.getpid())
    try:
        shutil.copyfile(str(src), str(tmp))
        os.replace(str(tmp), str(dst))
        return True
    except OSError:
        try:
            if tmp.exists():
                tmp.unlink()
        except OSError:
            pass
        return False


# ---- strict report contract ----

def _error_codes_strict(errors):
    codes = []
    for e in errors:
        if not isinstance(e, dict):
            return None
        code = e.get("code")
        if not isinstance(code, str) or not code.strip():
            return None
        codes.append(code.strip())
    return sorted(set(codes))


def _convergence_check(obj, case):
    """Check the claimed two-resolution certificate; replay remains mandatory."""
    c = obj.get("convergence")
    if not isinstance(c, dict) or c.get("performed") is not True or c.get("passed") is not True:
        return "CONVERGENCE_NOT_VERIFIED"
    expected_step = case.get("step")
    if not is_finite_nonneg(expected_step) or expected_step <= 0:
        return "CONVERGENCE_STEP"
    for key, expected in (("coarseStep", expected_step), ("fineStep", expected_step / 2)):
        value = c.get(key)
        if not is_finite_nonneg(value) or not math.isclose(value, expected, rel_tol=1e-9, abs_tol=1e-12):
            return "CONVERGENCE_STEP"
    required = {"maxSpeed", "minVerticalG", "maxVerticalG", "maxLateralG", "maxLongitudinalG", "exposure10Seconds", "maxVerticalRateGps"}
    limits = obj.get("limits", {})
    if not isinstance(limits, dict):
        return "CONVERGENCE_LIMITS"
    for key in ("maxLateralRateGps", "maxLongitudinalRateGps"):
        limit = limits.get(key)
        if limit is not None and (not is_finite_nonneg(limit) or limit <= 0):
            return "CONVERGENCE_LIMITS"
    reported = obj.get("metrics"); seats = obj.get("seatStatistics")
    if not isinstance(reported, dict) or not isinstance(seats, list) or len(seats) != 3:
        return "CONVERGENCE_COARSE_BINDING"
    coarse_values = {name: reported.get("maxJerkGps" if name == "maxVerticalRateGps" else name) for name in required}
    for index, seat in enumerate(("front", "middle", "rear")):
        record = seats[index]
        if not isinstance(record, dict) or not isinstance(record.get("axes"), list) or len(record["axes"]) != 3:
            return "CONVERGENCE_COARSE_BINDING"
        coarse_values[seat + ".exposure10Seconds"] = record.get("exposure10Seconds")
        for axis_index, axis in enumerate(("vertical", "lateral", "longitudinal")):
            statistics = record["axes"][axis_index]
            if not isinstance(statistics, dict):
                return "CONVERGENCE_COARSE_BINDING"
            for name, value in statistics.items():
                coarse_values[seat + "." + axis + "." + name] = value
    for seat in ("front", "middle", "rear"):
        required.add(seat + ".exposure10Seconds")
        for axis in ("vertical", "lateral", "longitudinal"):
            prefix = seat + "." + axis + "."
            required.update(prefix + field for field in ("minG", "maxG", "meanG", "mean1sMin", "mean1sMax", "mean10sMin", "mean10sMax"))
            if axis == "vertical" or is_finite_nonneg(limits.get("max" + axis.title() + "RateGps")):
                required.add(prefix + "maxRateGps")
    rows = c.get("metrics")
    if not isinstance(rows, list) or len(rows) != len(required):
        return "CONVERGENCE_METRICS"
    seen = set(); speed_error = 0.; force_error = 0.
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get("name"), str) or row["name"] not in required or row["name"] in seen:
            return "CONVERGENCE_METRICS"
        seen.add(row["name"])
        values = [row.get(k) for k in ("coarse", "fine", "absoluteDifference", "tolerance")]
        if any(not is_finite_number(v) for v in values):
            return "CONVERGENCE_NONFINITE"
        coarse, fine, difference, tolerance = values
        bound = coarse_values.get(row["name"])
        if not is_finite_number(bound) or bound != coarse:
            return "CONVERGENCE_COARSE_BINDING"
        actual = abs(coarse - fine); scale = max(1., abs(fine)); fraction = .01 if row["name"] == "maxSpeed" else .02
        if not is_finite_number(actual):
            return "CONVERGENCE_NONFINITE"
        if not math.isclose(difference, actual, rel_tol=1e-9, abs_tol=1e-9) or not math.isclose(tolerance, fraction * scale, rel_tol=1e-9, abs_tol=1e-9):
            return "CONVERGENCE_ARITHMETIC"
        if not actual < fraction * scale:
            return "CONVERGENCE_TOLERANCE"
        if row["name"] == "maxSpeed": speed_error = actual / scale
        else: force_error = max(force_error, actual / scale)
    for key, actual in (("maxSpeedRelativeError", speed_error), ("maxForceRelativeError", force_error)):
        value = c.get(key)
        if not is_finite_nonneg(value) or not math.isclose(value, actual, rel_tol=1e-9, abs_tol=1e-9):
            return "CONVERGENCE_SUMMARY"
    return None


def strict_report_check(obj, case):
    """Pin current core contract. Returns (kind, codes, reason).

    kind: 'accepted-candidate' | 'rejected-candidate' | 'invalid'.
    """
    if not isinstance(obj, dict):
        return ("invalid", [], "NOT_AN_OBJECT")
    sv = obj.get("schemaVersion")
    if not isinstance(sv, int) or isinstance(sv, bool) or sv != REPORT_SCHEMA_VERSION:
        return ("invalid", [], "SCHEMA_MISMATCH")
    if obj.get("generatorVersion") != EXPECTED_GENERATOR_VERSION:
        return ("invalid", [], "GENERATOR_MISMATCH")
    seed = obj.get("seed")
    if not isinstance(seed, int) or isinstance(seed, bool) or seed != case.get("seed"):
        return ("invalid", [], "SEED_MISMATCH")
    if obj.get("terrain") != case.get("terrain"):
        return ("invalid", [], "TERRAIN_MISMATCH")
    if obj.get("preset") != case.get("preset"):
        return ("invalid", [], "PRESET_MISMATCH")
    inten = obj.get("intensityRequired")
    if not isinstance(inten, bool):
        return ("invalid", [], "INTENSITY_MISMATCH")
    expect_inten = (case.get("preset") == "all-records")
    if inten != expect_inten:
        return ("invalid", [], "INTENSITY_MISMATCH")
    accepted = obj.get("accepted")
    completed = obj.get("completed")
    cancelled = obj.get("cancelled")
    if not isinstance(accepted, bool) or not isinstance(completed, bool) \
            or not isinstance(cancelled, bool):
        return ("invalid", [], "CONTRADICTORY_TYPES")
    errors = obj.get("errors")
    if not isinstance(errors, list):
        return ("invalid", [], "ERRORS_MALFORMED")
    codes = _error_codes_strict(errors)
    if codes is None:
        return ("invalid", [], "ERRORS_MALFORMED")
    if accepted is True:
        if completed is not True or cancelled is not False or len(errors) != 0:
            return ("invalid", codes or ["CONTRADICTORY"], "CONTRADICTORY")
        convergence_error = _convergence_check(obj, case)
        if convergence_error:
            return ("invalid", [convergence_error], convergence_error)
        if expect_inten:
            targets = obj.get("targets")
            if not isinstance(targets, dict):
                return ("invalid", ["REFERENCE_UNAVAILABLE"], "REFERENCE_BINDING_MISSING")
            exposure = targets.get("referenceExposure")
            identity = targets.get("referenceId")
            if not is_finite_nonneg(exposure) or exposure <= 0 or not isinstance(identity, str) or not identity.strip():
                return ("invalid", ["REFERENCE_UNAVAILABLE"], "REFERENCE_BINDING_MISSING")
        return ("accepted-candidate", [], "strict accepted candidate")
    # accepted is False: valid rejection needs evidence of failure.
    if completed is True and cancelled is False and len(errors) == 0:
        return ("invalid", [], "CONTRADICTORY_EMPTY_REJECTION")
    return ("rejected-candidate", codes or ["REJECTED_NO_CODE"],
            "strict rejection")


def parse_report(obj, case=None):
    """Compat wrapper: strict when case given, else best-effort codes only."""
    if case is not None:
        kind, codes, _ = strict_report_check(obj, case)
        if kind == "accepted-candidate":
            return True, codes
        if kind == "rejected-candidate":
            return False, codes
        return None, codes
    # No case: order-independent code collection from strict `errors` shape only.
    if not isinstance(obj, dict):
        return None, []
    errors = obj.get("errors")
    if not isinstance(errors, list):
        return None, []
    codes = _error_codes_strict(errors)
    if codes is None:
        return None, []
    accepted = obj.get("accepted")
    if isinstance(accepted, bool):
        return accepted, codes
    return None, codes


def load_cli_json(path):
    try:
        p = Path(path)
        if p.is_symlink() or not p.is_file():
            return None
        if p.stat().st_size <= 0:
            return None
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def classify_result(returncode, elapsed, timed_out, cli_obj, cli_json_exists,
                    out_exists, case=None, out_size_ok=False,
                    trace_ok=False):
    """Strict generate-stage classification (no validate here).

    Categories: accepted-pending-validation | rejected | timeout | infrastructure.
    Callers must run `validate` before counting accepted. Kept name for compat.
    """
    if timed_out:
        return ("timeout", False, ["TIMEOUT"], "subprocess timeout")
    if returncode == 0:
        if cli_obj is None or not cli_json_exists:
            return ("infrastructure", False, ["NO_OUTPUT"],
                    "exit 0 but --json missing/unparseable/empty/dir")
        if case is None:
            return ("infrastructure", False, ["NO_CASE"],
                    "no case for strict match")
        kind, codes, reason = strict_report_check(cli_obj, case)
        if kind != "accepted-candidate":
            return ("infrastructure", False, codes or [reason],
                    f"exit 0 not strict-accepted: {reason}")
        if not out_exists or not out_size_ok:
            return ("infrastructure", False, ["NO_OUTPUT"],
                    "exit 0 strict-accepted but --out missing/empty/dir")
        if not trace_ok:
            return ("infrastructure", False, ["NO_TRACE"],
                    "exit 0 strict-accepted but --trace missing/empty")
        return ("accepted-pending-validation", False, [],
                "strict candidate awaits validate")
    if returncode == 2:
        if cli_obj is None or not cli_json_exists:
            return ("infrastructure", False, ["NO_OUTPUT"],
                    "exit 2 (rejected) but --json missing/unparseable")
        if case is None:
            return ("infrastructure", False, ["NO_CASE"],
                    "no case for strict match")
        kind, codes, reason = strict_report_check(cli_obj, case)
        if kind != "rejected-candidate":
            return ("infrastructure", False, codes or [reason],
                    f"exit 2 not strict-rejected: {reason}")
        return ("rejected", False, codes, "strict-reference rejection")
    if cli_obj is not None:
        _, codes = parse_report(cli_obj, case)
    else:
        codes = []
    if not codes:
        codes = ["CLI_EXIT_%s" % returncode]
    return ("infrastructure", False, codes, "cli exit %s" % returncode)


def percentile(values, pct):
    if not values:
        return 0.0
    s = sorted(values)
    if len(s) == 1:
        return float(s[0])
    k = (len(s) - 1) * (pct / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    frac = k - lo
    return float(s[lo] + (s[hi] - s[lo]) * frac)


def aggregate_results(records):
    """Denominator is always len(records); no gaming. Splits gen vs replay."""
    total = len(records)
    accepted = sum(1 for r in records if r.get("category") == "accepted")
    rejected = sum(1 for r in records if r.get("category") == "rejected")
    timeouts = sum(1 for r in records if r.get("category") == "timeout")
    infra = sum(1 for r in records if r.get("category") == "infrastructure")
    infra_total = infra + timeouts
    by_preset = {}
    for preset in PRESETS:
        sub = [r for r in records if r.get("preset") == preset]
        a = sum(1 for r in sub if r.get("category") == "accepted")
        by_preset[preset] = {
            "total": len(sub),
            "accepted": a,
            "rejected": sum(1 for r in sub if r.get("category") == "rejected"),
            "timeout": sum(1 for r in sub if r.get("category") == "timeout"),
            "infrastructure": sum(1 for r in sub if r.get("category") == "infrastructure"),
            "successRate": (a / len(sub)) if sub else 0.0,
        }
    hist: dict = {}
    for r in records:
        if r.get("category") == "rejected":
            for c in r.get("errorCodes", []):
                hist[c] = hist.get(c, 0) + 1
    gen_all = [float(r.get("elapsed", 0.0)) for r in records]
    gen_acc = [float(r.get("elapsed", 0.0)) for r in records
               if r.get("category") == "accepted"]
    replay_vals = [float(r.get("replayElapsed", 0.0)) for r in records
                   if r.get("replayReturncode") is not None]
    total_vals = [float(r.get("elapsed", 0.0)) + float(r.get("replayElapsed", 0.0) or 0.0)
                  for r in records]
    summary = {
        "total": total,
        "accepted": accepted,
        "rejected": rejected,
        "timeout": timeouts,
        "infrastructure": infra,
        "infrastructureIncludingTimeout": infra_total,
        "successRate": (accepted / total) if total else 0.0,
        "byPreset": by_preset,
        "errorCodeHistogram": dict(sorted(hist.items(), key=lambda kv: (-kv[1], kv[0]))),
        "latency": {
            "allAttempts": {
                "count": len(gen_all),
                "p50": percentile(gen_all, 50),
                "p95": percentile(gen_all, 95),
            },
            "acceptedOnly": {
                "count": len(gen_acc),
                "p50": percentile(gen_acc, 50),
                "p95": percentile(gen_acc, 95),
            },
            "replay": {
                "count": len(replay_vals),
                "p50": percentile(replay_vals, 50),
                "p95": percentile(replay_vals, 95),
            },
            "total": {
                "count": len(total_vals),
                "p50": percentile(total_vals, 50),
                "p95": percentile(total_vals, 95),
            },
        },
    }
    return summary


def select_samples(records, n):
    """Up to n accepted records in manifest order. Rejected never consume slots."""
    if n <= 0:
        return []
    ordered = sorted(records, key=lambda r: r.get("index", 0))
    out = [r for r in ordered if r.get("category") == "accepted"]
    return out[:n]


def _case_paths(out_dir, case_id):
    base = Path(out_dir)
    return {
        "result": base / "cases" / f"{case_id}.json",
        "cli_json": base / "cli_json" / f"{case_id}.json",
        "trace": base / "traces" / f"{case_id}.trace.json",
        "out": base / "outs" / f"{case_id}.coaster",
        "replay": base / "replays" / f"{case_id}.replay.json",
    }


def _base_cmd(cli):
    return list(cli) if isinstance(cli, (list, tuple)) else [str(cli)]


def run_one_case(cli, case, out_dir, timeout, validate_timeout=None):
    """Fresh isolated attempt; validate before accept; publish only fresh files."""
    out_dir = Path(out_dir).resolve()
    vtimeout = float(validate_timeout) if validate_timeout is not None else float(timeout)
    finals = _case_paths(out_dir, case["id"])
    attempts_base = out_dir / "attempts"
    if any(resolve_within(out_dir, path) is None for path in [attempts_base, *finals.values()]):
        raise ValueError("case output path escapes output directory")
    if not safe_remove(finals["result"], out_dir):
        raise OSError("cannot invalidate previous case result before retry")
    attempts_base.mkdir(parents=True, exist_ok=True)
    attempt_dir = Path(tempfile.mkdtemp(prefix=case["id"] + "_", dir=str(attempts_base)))
    a_gen = attempt_dir / "gen.json"
    a_trace = attempt_dir / "trace.json"
    a_out = attempt_dir / "out.coaster"
    a_replay = attempt_dir / "replay.json"
    base = _base_cmd(cli)
    current_sha = cli_identity_sha256(cli)

    def finish(rec, publish_map=None):
        # Clear every old evidence path, including traces/reports after a no-output
        # attempt. A publication failure keeps its isolated inputs for diagnosis.
        failures = []
        published = set()
        for key in ("cli_json", "trace", "out", "replay"):
            if not safe_remove(finals[key], out_dir):
                failures.append("cannot clear " + key)
        if not failures:
            for src, dst in (publish_map or {}).items():
                if resolve_within(out_dir, dst) is None or not is_valid_file(src):
                    failures.append("unsafe or missing evidence: " + str(dst))
                    continue
                if not atomic_copy(src, dst) or not is_valid_file(dst) or sha256_file(src) != sha256_file(dst):
                    failures.append("cannot publish " + str(dst))
                else:
                    published.add(Path(dst))
        if rec.get("category") == "accepted" and any(finals[k] not in published for k in ("cli_json", "trace", "out", "replay")):
            failures.append("accepted result lacks all four published evidence files")
        if failures:
            rec.update(category="infrastructure", accepted=False,
                       errorCodes=sorted(set(rec.get("errorCodes", []) + ["EVIDENCE_PUBLISH_FAILED"])),
                       note="Evidence publication failed; isolated attempt retained.",
                       publicationFailures=failures, attemptDirectory=str(attempt_dir))
            # No partial bundle should look like an accepted published sample.
            for key in ("cli_json", "trace", "out", "replay"):
                if not safe_remove(finals[key], out_dir):
                    failures.append("cleanup failed: " + key)
            published.clear()
        for key, field in (("cli_json", "genJsonSha256"), ("trace", "traceSha256"),
                           ("out", "outSha256"), ("replay", "replayJsonSha256")):
            rec[field] = sha256_file(finals[key]) if finals[key] in published else None
        rec["cliSha256"] = current_sha
        rec["generationTimeout"] = float(timeout)
        rec["validateTimeout"] = float(vtimeout)
        rec["reportSchemaVersion"] = REPORT_SCHEMA_VERSION
        rec["generatorVersion"] = EXPECTED_GENERATOR_VERSION
        rec["validationPolicy"] = VALIDATION_POLICY
        if resolve_within(out_dir, finals["result"]) is None:
            rec.update(category="infrastructure", accepted=False,
                       errorCodes=["EVIDENCE_PUBLISH_FAILED"], attemptDirectory=str(attempt_dir))
            return rec
        try:
            atomic_write_json(finals["result"], rec)
        except OSError as error:
            rec.update(category="infrastructure", accepted=False,
                       errorCodes=["RESULT_WRITE_FAILED"], note=str(error),
                       attemptDirectory=str(attempt_dir))
            return rec
        if not failures:
            safe_remove(attempt_dir, out_dir)
        return rec

    def base_rec():
        return {"version": CASE_JSON_VERSION, "index": case["index"],
                "id": case["id"], "seed": case["seed"], "preset": case["preset"],
                "terrain": case["terrain"], "candidates": case["candidates"],
                "step": case["step"]}

    gen_cmd = base + ["generate",
                      "--seed", str(case["seed"]),
                      "--terrain", case["terrain"],
                      "--preset", case["preset"],
                      "--candidates", str(case["candidates"]),
                      "--step", repr(float(case["step"])),
                      "--json", str(a_gen),
                      "--trace", str(a_trace),
                      "--out", str(a_out)]
    start = time.perf_counter()
    timed_out = False
    returncode = None
    try:
        proc = subprocess.run(gen_cmd, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, timeout=float(timeout))
        returncode = proc.returncode
    except subprocess.TimeoutExpired:
        timed_out = True
        returncode = 124
    except OSError as e:
        elapsed = time.perf_counter() - start
        rec = base_rec()
        rec.update({"category": "infrastructure", "accepted": False,
                    "errorCodes": ["SPAWN_FAILED"], "elapsed": elapsed,
                    "replayElapsed": 0.0, "returncode": 127,
                    "replayReturncode": None, "timeout": False,
                    "replayTimeout": False, "note": f"spawn failed: {e}"})
        return finish(rec)
    gen_elapsed = time.perf_counter() - start
    gen_obj = load_cli_json(a_gen) if a_gen.exists() else None
    gen_json_ok = is_valid_file(a_gen)
    out_ok = is_valid_file(a_out)
    trace_ok = is_valid_file(a_trace)
    stage, accepted_flag, codes, note = classify_result(
        returncode, gen_elapsed, timed_out, gen_obj, gen_json_ok,
        out_ok, case, out_ok, trace_ok)

    if stage == "timeout":
        rec = base_rec()
        rec.update({"category": "timeout", "accepted": False,
                    "errorCodes": ["TIMEOUT"], "elapsed": gen_elapsed,
                    "replayElapsed": 0.0, "returncode": returncode,
                    "replayReturncode": None, "timeout": True,
                    "replayTimeout": False, "note": note})
        pubs = {}
        if gen_json_ok:
            pubs[str(a_gen)] = str(finals["cli_json"])
        return finish(rec, pubs)

    if stage == "infrastructure":
        rec = base_rec()
        rec.update({"category": "infrastructure", "accepted": False,
                    "errorCodes": codes, "elapsed": gen_elapsed,
                    "replayElapsed": 0.0, "returncode": returncode,
                    "replayReturncode": None, "timeout": bool(timed_out),
                    "replayTimeout": False, "note": note})
        pubs = {}
        if gen_json_ok:
            pubs[str(a_gen)] = str(finals["cli_json"])
        if trace_ok:
            pubs[str(a_trace)] = str(finals["trace"])
        return finish(rec, pubs)

    if stage == "rejected":
        rec = base_rec()
        rec.update({"category": "rejected", "accepted": False,
                    "errorCodes": codes, "elapsed": gen_elapsed,
                    "replayElapsed": 0.0, "returncode": returncode,
                    "replayReturncode": None, "timeout": False,
                    "replayTimeout": False, "note": note})
        pubs = {}
        if gen_json_ok:
            pubs[str(a_gen)] = str(finals["cli_json"])
        if trace_ok:
            pubs[str(a_trace)] = str(finals["trace"])
        return finish(rec, pubs)

    # stage == accepted-pending-validation: independent replay, bounded timeout.
    vcmd = base + ["validate", str(a_out), "--json", str(a_replay)]
    vstart = time.perf_counter()
    v_timed_out = False
    v_returncode = None
    try:
        vproc = subprocess.run(vcmd, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, timeout=vtimeout)
        v_returncode = vproc.returncode
    except subprocess.TimeoutExpired:
        v_timed_out = True
        v_returncode = 124
    except OSError as e:
        replay_elapsed = time.perf_counter() - vstart
        rec = base_rec()
        rec.update({"category": "infrastructure", "accepted": False,
                    "errorCodes": ["REPLAY_SPAWN_FAILED"],
                    "elapsed": gen_elapsed, "replayElapsed": replay_elapsed,
                    "returncode": returncode, "replayReturncode": 127,
                    "timeout": False, "replayTimeout": False,
                    "note": f"replay spawn failed: {e}"})
        pubs = {}
        if gen_json_ok:
            pubs[str(a_gen)] = str(finals["cli_json"])
        return finish(rec, pubs)
    replay_elapsed = time.perf_counter() - vstart
    if v_timed_out:
        rec = base_rec()
        rec.update({"category": "infrastructure", "accepted": False,
                    "errorCodes": ["REPLAY_TIMEOUT"], "elapsed": gen_elapsed,
                    "replayElapsed": replay_elapsed, "returncode": returncode,
                    "replayReturncode": v_returncode, "timeout": False,
                    "replayTimeout": True, "note": "validate timeout"})
        pubs = {}
        if gen_json_ok:
            pubs[str(a_gen)] = str(finals["cli_json"])
        return finish(rec, pubs)
    replay_obj = load_cli_json(a_replay) if a_replay.exists() else None
    replay_ok = is_valid_file(a_replay)
    if v_returncode == 0 and replay_ok and replay_obj is not None:
        kind, rcodes, reason = strict_report_check(replay_obj, case)
        if kind == "accepted-candidate":
            rec = base_rec()
            rec.update({"category": "accepted", "accepted": True,
                        "errorCodes": [], "elapsed": gen_elapsed,
                        "replayElapsed": replay_elapsed, "returncode": returncode,
                        "replayReturncode": v_returncode, "timeout": False,
                        "replayTimeout": False, "note": "validate-confirmed"})
            pubs = {str(a_gen): str(finals["cli_json"]),
                    str(a_trace): str(finals["trace"]),
                    str(a_out): str(finals["out"]),
                    str(a_replay): str(finals["replay"])}
            return finish(rec, pubs)
        # Exit zero with a rejected/invalid report violates the replay contract.
        rec = base_rec()
        rec.update({"category": "infrastructure", "accepted": False,
                    "errorCodes": rcodes or [reason or "REPLAY_REJECTED"],
                    "elapsed": gen_elapsed, "replayElapsed": replay_elapsed,
                    "returncode": returncode, "replayReturncode": v_returncode,
                    "timeout": False, "replayTimeout": False,
                    "note": f"replay invalid: {reason}"})
        pubs = {}
        if gen_json_ok:
            pubs[str(a_gen)] = str(finals["cli_json"])
        if replay_ok:
            pubs[str(a_replay)] = str(finals["replay"])
        if trace_ok:
            pubs[str(a_trace)] = str(finals["trace"])
        return finish(rec, pubs)
    if v_returncode == 2 and replay_ok and replay_obj is not None:
        kind, rcodes, reason = strict_report_check(replay_obj, case)
        if kind == "rejected-candidate":
            rec = base_rec()
            rec.update({"category": "rejected", "accepted": False,
                        "errorCodes": rcodes, "elapsed": gen_elapsed,
                        "replayElapsed": replay_elapsed, "returncode": returncode,
                        "replayReturncode": v_returncode, "timeout": False,
                        "replayTimeout": False,
                        "note": "replay rejected saved design"})
            pubs = {}
            if gen_json_ok:
                pubs[str(a_gen)] = str(finals["cli_json"])
            pubs[str(a_replay)] = str(finals["replay"])
            if trace_ok:
                pubs[str(a_trace)] = str(finals["trace"])
            return finish(rec, pubs)
    rec = base_rec()
    rec.update({"category": "infrastructure", "accepted": False,
                "errorCodes": ["REPLAY_INVALID"], "elapsed": gen_elapsed,
                "replayElapsed": replay_elapsed, "returncode": returncode,
                "replayReturncode": v_returncode, "timeout": False,
                "replayTimeout": bool(v_timed_out),
                "note": f"replay exit {v_returncode} without valid confirm"})
    pubs = {}
    if gen_json_ok:
        pubs[str(a_gen)] = str(finals["cli_json"])
    if replay_ok:
        pubs[str(a_replay)] = str(finals["replay"])
    return finish(rec, pubs)


def _basic_record_ok(obj, case):
    if not isinstance(obj, dict) or obj.get("version") != CASE_JSON_VERSION:
        return False
    for key in ("id", "index", "seed", "preset", "terrain", "candidates", "step"):
        if type(obj.get(key)) is not type(case.get(key)) or obj.get(key) != case.get(key):
            return False
    for key in ("accepted", "timeout", "replayTimeout"):
        if type(obj.get(key)) is not bool:
            return False
    codes = obj.get("errorCodes")
    if not isinstance(codes, list) or any(not isinstance(c, str) or not c.strip() for c in codes):
        return False
    for key in ("elapsed", "replayElapsed", "generationTimeout", "validateTimeout"):
        if not is_finite_nonneg(obj.get(key)):
            return False
    if obj["generationTimeout"] <= 0 or obj["validateTimeout"] <= 0:
        return False
    rc, replay = obj.get("returncode"), obj.get("replayReturncode")
    if type(rc) is not int or (replay is not None and type(replay) is not int):
        return False
    cat = obj.get("category")
    if cat == "accepted":
        return obj["accepted"] and rc == 0 and replay == 0 and not obj["timeout"] and not obj["replayTimeout"] and codes == []
    if obj["accepted"] or not codes:
        return False
    if cat == "rejected":
        return not obj["timeout"] and not obj["replayTimeout"] and ((rc == 2 and replay is None) or (rc == 0 and replay == 2))
    if cat == "timeout":
        return obj["timeout"] and rc == 124 and replay is None
    return cat == "infrastructure"

def load_result_json(path, case):
    try:
        obj = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not _basic_record_ok(obj, case):
        return None
    return obj


def verify_resume_evidence(rec, case, out_dir, expected):
    """Resume only completed decisions with current, coherent hashed evidence."""
    if not _basic_record_ok(rec, case) or rec["category"] not in ("accepted", "rejected"):
        return False  # Retry infrastructure/timeout failures.
    if rec.get("cliSha256") != expected.get("cliSha256"):
        return False
    if rec.get("reportSchemaVersion") != REPORT_SCHEMA_VERSION or rec.get("generatorVersion") != EXPECTED_GENERATOR_VERSION or rec.get("validationPolicy") != VALIDATION_POLICY:
        return False
    if rec["generationTimeout"] != expected.get("timeout") or rec["validateTimeout"] != expected.get("validateTimeout"):
        return False
    finals = _case_paths(out_dir, case["id"])
    if any(resolve_within(out_dir, path) is None for path in finals.values()):
        return False
    def evidence(key, field):
        path = finals[key]
        return is_valid_file(path) and isinstance(rec.get(field), str) and sha256_file(path) == rec[field]
    if not evidence("cli_json", "genJsonSha256"):
        return False
    kind, codes, _ = strict_report_check(load_cli_json(finals["cli_json"]), case)
    if rec["returncode"] == 2:
        return kind == "rejected-candidate" and sorted(set(rec["errorCodes"])) == codes and not finals["out"].exists() and not finals["replay"].exists()
    if kind != "accepted-candidate" or not evidence("replay", "replayJsonSha256"):
        return False
    rkind, rcodes, _ = strict_report_check(load_cli_json(finals["replay"]), case)
    if rec["category"] == "rejected":
        return rkind == "rejected-candidate" and sorted(set(rec["errorCodes"])) == rcodes and not finals["out"].exists()
    return rkind == "accepted-candidate" and evidence("trace", "traceSha256") and evidence("out", "outSha256")

def write_report_csv(records, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["case_id", "seed", "preset", "terrain", "accepted",
                    "category", "errorCodes", "elapsed", "replayElapsed",
                    "returncode", "replayReturncode", "timeout"])
        for r in sorted(records, key=lambda r: r.get("index", 0)):
            w.writerow([r.get("id"), r.get("seed"), r.get("preset"),
                        r.get("terrain"), int(bool(r.get("accepted"))),
                        r.get("category"), ";".join(r.get("errorCodes", [])),
                        f"{float(r.get('elapsed', 0.0)):.3f}",
                        f"{float(r.get('replayElapsed', 0.0) or 0.0):.3f}",
                        r.get("returncode"), r.get("replayReturncode"),
                        int(bool(r.get("timeout")))])


def publish_samples(records, out_dir, requested):
    """Copy only validated saves; count actual copied files; clear stale."""
    out_dir = Path(out_dir).resolve()
    samples_dir = out_dir / "samples"
    empty = {"requested": requested, "eligible": 0, "selected": 0, "saved": 0,
             "ids": [], "failures": []}
    if requested <= 0:
        return empty
    if resolve_within(out_dir, samples_dir) is None:
        empty["failures"] = ["unsafe sample directory"]
        return empty
    samples_dir.mkdir(parents=True, exist_ok=True)
    for child in list(samples_dir.iterdir()):
        if not safe_remove(child, out_dir):
            empty["failures"].append("cannot clear " + str(child))
    if empty["failures"]:
        return empty
    eligible = []
    for r in sorted(records, key=lambda x: x.get("index", 0)):
        if r.get("category") != "accepted" or not r.get("accepted"):
            continue
        finals = _case_paths(out_dir, r["id"])
        if all(is_valid_file(finals[k]) for k in ("cli_json", "trace", "out", "replay")):
            eligible.append(r)
    selected = eligible[:requested]
    saved = []
    failures = []
    for r in selected:
        finals = _case_paths(out_dir, r["id"])
        ok = True
        made = []
        for key in ("cli_json", "trace", "out", "replay"):
            src = finals[key]
            dst = samples_dir / f"{r['id']}_{src.name}"
            if resolve_within(out_dir, dst) is None or not is_valid_file(src):
                ok = False
                break
            if not atomic_copy(src, dst) or not is_valid_file(dst):
                ok = False
                break
            made.append(dst.name)
        if ok:
            r["sampledFiles"] = made
            saved.append(r)
        else:
            failures.append(r["id"])
            for name in made:
                try:
                    (samples_dir / name).unlink()
                except OSError:
                    pass
    return {"requested": requested, "eligible": len(eligible),
            "selected": len(selected), "saved": len(saved),
            "ids": [r["id"] for r in saved], "failures": failures}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Run coaster_cli acceptance sweep.")
    p.add_argument("--cli", required=True, help="Path to coaster_cli executable")
    p.add_argument("--out-dir", required=True, help="Output directory")
    p.add_argument("--count", type=int, default=DEFAULT_COUNT,
                   help="Manifest size override for smoke tests (default 1000)")
    p.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                   help="Per-case generate timeout seconds (default 120)")
    p.add_argument("--validate-timeout", type=float, default=None,
                   help="Per-case validate timeout (default: same as --timeout)")
    p.add_argument("--candidates", type=int, default=DEFAULT_CANDIDATES)
    p.add_argument("--step", type=float, default=DEFAULT_STEP)
    p.add_argument("--samples", type=int, default=0,
                   help="Preserve up to N accepted traces/outs under samples/")
    p.add_argument("--resume", action="store_true",
                   help="Reuse existing per-case results only on exact manifest match")
    p.add_argument("--min-success", type=float, default=None)
    p.add_argument("--min-physics-proof", type=float, default=None)
    p.add_argument("--min-all-records", type=float, default=None)
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    cli = Path(args.cli)
    out_dir = Path(args.out_dir)
    if not cli.exists() or cli.is_dir():
        print(f"error: CLI not found: {cli}", file=sys.stderr)
        return 2
    if args.count <= 0 or args.workers <= 0 or not is_finite_nonneg(args.timeout) or args.timeout <= 0:
        print("error: --count/--workers/--timeout must be > 0", file=sys.stderr)
        return 2
    vtimeout = float(args.validate_timeout) if args.validate_timeout is not None else float(args.timeout)
    if not is_finite_nonneg(vtimeout) or vtimeout <= 0:
        print("error: --validate-timeout must be > 0", file=sys.stderr)
        return 2
    cli_hash = sha256_file(str(cli))
    if cli_hash is None:
        print(f"error: cannot hash CLI: {cli}", file=sys.stderr)
        return 2
    manifest = build_manifest(args.count, args.candidates, args.step)
    out_dir = out_dir.resolve()
    if any(resolve_within(out_dir, out_dir / child) is None for child in
           ("cases", "attempts", "outs", "cli_json", "traces", "replays", "samples", "manifest.json", "summary.json", "report.csv")):
        print("error: an output path escapes --out-dir", file=sys.stderr)
        return 2
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "cases").mkdir(parents=True, exist_ok=True)

    expected = {"cliSha256": cli_hash, "timeout": float(args.timeout),
                "validateTimeout": float(vtimeout),
                "reportSchemaVersion": REPORT_SCHEMA_VERSION,
                "generatorVersion": EXPECTED_GENERATOR_VERSION,
                "validationPolicy": VALIDATION_POLICY}
    manifest_doc = {"version": MANIFEST_VERSION, "count": args.count,
                    "candidates": args.candidates, "step": args.step,
                    "timeout": float(args.timeout),
                    "validateTimeout": float(vtimeout),
                    "cliSha256": cli_hash,
                    "reportSchemaVersion": REPORT_SCHEMA_VERSION,
                    "generatorVersion": EXPECTED_GENERATOR_VERSION,
                    "validationPolicy": VALIDATION_POLICY,
                    "cases": manifest}
    manifest_path = out_dir / "manifest.json"
    resumed = 0
    records_by_id = {}
    if args.resume and manifest_path.exists():
        try:
            old = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            old = None
        same = (isinstance(old, dict) and old.get("version") == MANIFEST_VERSION
                and old.get("count") == args.count
                and old.get("candidates") == args.candidates
                and old.get("step") == args.step
                and old.get("timeout") == float(args.timeout)
                and old.get("validateTimeout", float(args.timeout)) == float(vtimeout)
                and old.get("cliSha256") == cli_hash
                and old.get("reportSchemaVersion") == REPORT_SCHEMA_VERSION
                and old.get("generatorVersion") == EXPECTED_GENERATOR_VERSION
                and old.get("validationPolicy") == VALIDATION_POLICY
                and old.get("cases") == manifest)
        if same:
            for case in manifest:
                rp = _case_paths(out_dir, case["id"])["result"]
                if rp.exists():
                    rec = load_result_json(rp, case)
                    if rec is not None and verify_resume_evidence(rec, case, out_dir, expected):
                        records_by_id[case["id"]] = rec
                        resumed += 1
            print(f"resume: reused {resumed}/{len(manifest)} verified results")
        else:
            print("resume: manifest/config/executable differs; rerunning all",
                  file=sys.stderr)
    atomic_write_json(manifest_path, manifest_doc)

    pending = [c for c in manifest if c["id"] not in records_by_id]
    workers = max(1, min(args.workers, len(pending) if pending else 1))
    if pending:
        with ThreadPoolExecutor(max_workers=workers) as ex:
            futs = {ex.submit(run_one_case, str(cli), c, str(out_dir),
                              float(args.timeout), float(vtimeout)): c
                    for c in pending}
            done = 0
            for fut in as_completed(futs):
                case = futs[fut]
                try:
                    rec = fut.result()
                except Exception as e:  # never fabricate success
                    rec = {"version": CASE_JSON_VERSION, "index": case["index"],
                           "id": case["id"], "seed": case["seed"],
                           "preset": case["preset"], "terrain": case["terrain"],
                           "candidates": case["candidates"], "step": case["step"],
                           "category": "infrastructure", "accepted": False,
                           "errorCodes": ["HARNESS_EXCEPTION"],
                           "elapsed": 0.0, "replayElapsed": 0.0,
                           "returncode": -1, "replayReturncode": None,
                           "timeout": False, "replayTimeout": False,
                           "note": f"harness exception: {e}"}
                records_by_id[case["id"]] = rec
                done += 1
                if done % 25 == 0 or done == len(pending):
                    print(f"progress: {done}/{len(pending)} new "
                          f"(+{resumed} resumed)", flush=True)

    records = [records_by_id[c["id"]] for c in manifest
               if c["id"] in records_by_id]
    missing = [c["id"] for c in manifest if c["id"] not in records_by_id]
    summary = aggregate_results(records)
    summary["config"] = {"cli": str(cli), "cliSha256": cli_hash,
                         "outDir": str(out_dir),
                         "count": args.count, "workers": args.workers,
                         "timeout": float(args.timeout),
                         "validateTimeout": float(vtimeout),
                         "candidates": args.candidates,
                         "step": args.step, "resumed": resumed,
                         "missing": missing,
                         "reportSchemaVersion": REPORT_SCHEMA_VERSION,
                         "generatorVersion": EXPECTED_GENERATOR_VERSION,
                         "validationPolicy": VALIDATION_POLICY}

    sample_info = publish_samples(records, out_dir, args.samples)
    summary["samples"] = sample_info

    write_report_csv(records, out_dir / "report.csv")
    atomic_write_json(out_dir / "summary.json", summary)

    print(f"total={summary['total']} accepted={summary['accepted']} "
          f"rejected={summary['rejected']} infra={summary['infrastructure']} "
          f"timeout={summary['timeout']} successRate={summary['successRate']:.3f}")
    for preset, s in summary["byPreset"].items():
        print(f"  {preset}: {s['accepted']}/{s['total']} "
              f"rate={s['successRate']:.3f}")
    lat = summary["latency"]
    print(f"  gen p50/p95 all={lat['allAttempts']['p50']:.2f}s/"
          f"{lat['allAttempts']['p95']:.2f}s "
          f"accepted={lat['acceptedOnly']['p50']:.2f}s/"
          f"{lat['acceptedOnly']['p95']:.2f}s "
          f"replay={lat['replay']['p50']:.2f}s/{lat['replay']['p95']:.2f}s")
    print(f"  samples: {sample_info['saved']}/{args.samples} saved "
          f"(eligible {sample_info['eligible']})")
    if summary["errorCodeHistogram"]:
        top = list(summary["errorCodeHistogram"].items())[:10]
        print("  top rejections: " + ", ".join(f"{k}x{v}" for k, v in top))

    reasons = []
    if missing:
        reasons.append(f"{len(missing)} cases missing results")
    if summary["infrastructureIncludingTimeout"] > 0:
        reasons.append(f"{summary['infrastructureIncludingTimeout']} infrastructure/timeout failures")
    if sample_info["failures"]:
        reasons.append(f"{len(sample_info['failures'])} sample publication failures")
    if args.min_success is not None and summary["successRate"] < args.min_success:
        reasons.append(f"overall rate {summary['successRate']:.3f} < {args.min_success}")
    for key, flag in (("physics-proof", args.min_physics_proof),
                      ("all-records", args.min_all_records)):
        if flag is not None and summary["byPreset"][key]["successRate"] < flag:
            reasons.append(f"{key} rate {summary['byPreset'][key]['successRate']:.3f} < {flag}")
    summary["gates"] = {"passed": not reasons, "reasons": reasons}
    atomic_write_json(out_dir / "summary.json", summary)
    if reasons:
        print("GATES FAILED: " + "; ".join(reasons), file=sys.stderr)
        return 1
    print("GATES PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
