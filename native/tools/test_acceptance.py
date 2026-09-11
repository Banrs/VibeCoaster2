#!/usr/bin/env python3
"""Regression tests for acceptance.py (stdlib only, no real core build).

Covers audit A1-A4 + build safeguard. Failing-first: each audit reproduction
must fail on the old weak harness and pass here.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import acceptance as acc


def convergence_obj(step=1/960):
    names = ["maxSpeed", "minVerticalG", "maxVerticalG", "maxLateralG", "maxLongitudinalG", "exposure10Seconds", "maxVerticalRateGps"]
    for seat in ("front", "middle", "rear"):
        names.append(seat + ".exposure10Seconds")
        for axis in ("vertical", "lateral", "longitudinal"):
            for field in ("minG", "maxG", "meanG", "mean1sMin", "mean1sMax", "mean10sMin", "mean10sMax"):
                names.append(seat + "." + axis + "." + field)
            if axis == "vertical": names.append(seat + "." + axis + ".maxRateGps")
        for axis in ("vertical", "lateral", "longitudinal"):
            for field in ("minimumG", "maximumG", "minimumOnsetGps", "maximumOnsetGps"):
                names.append(seat + ".forceEnvelope." + axis + "." + field)
        for field, count in (("directional", 6), ("paired", 3), ("horizontalReversal", 2), ("durationExtent", 2)):
            names.extend(seat + ".forceEnvelope." + field + str(i) for i in range(count))
        names.extend(seat + ".forceEnvelope." + field for field in ("reducedPositive", "zeroToTwo", "enhancedLongitudinalOnset"))
    return {"performed": True, "passed": True, "coarseStep": step, "fineStep": step / 2, "maxSpeedRelativeError": 0., "maxForceRelativeError": 0.,
            "metrics": [{"name": n, "coarse": 0., "fine": 0., "absoluteDifference": 0., "tolerance": .01 if n == "maxSpeed" else .02} for n in names]}


def fixture_report(obj):
    """Synthetic protocol metadata; these fixtures do not establish real physics."""
    metrics = {}; seats = [{"exposure10Seconds": 0., "axes": [{}, {}, {}]} for _ in range(3)]
    for row in obj["convergence"]["metrics"]:
        parts = row["name"].split(".")
        if len(parts) == 1:
            metrics["maxJerkGps" if parts[0] == "maxVerticalRateGps" else parts[0]] = row["coarse"]
        elif parts[1] != "forceEnvelope":
            seat = seats[("front", "middle", "rear").index(parts[0])]
            if len(parts) == 2: seat[parts[1]] = row["coarse"]
            else: seat["axes"][("vertical", "lateral", "longitudinal").index(parts[1])][parts[2]] = row["coarse"]
    values = {row["name"]: row["coarse"] for row in obj["convergence"]["metrics"]}
    envelope = []
    for name in ("front", "middle", "rear"):
        prefix = name + ".forceEnvelope."
        seat = {"performed": True, "passed": True, "axes": [
            {field: values.get(prefix + axis + "." + field, 0.)
             for field in ("minimumG", "maximumG", "minimumOnsetGps", "maximumOnsetGps")}
            for axis in ("vertical", "lateral", "longitudinal")]}
        for field, report_field, count in (("directional", "directionalG", 6), ("paired", "pairedSquaredUtilization", 3),
                                          ("horizontalReversal", "horizontalReversalG", 2), ("durationExtent", "durationExtentSeconds", 2)):
            seat[report_field] = [{"utilization": values.get(prefix + field + str(i), 0.)} for i in range(count)]
        for field, report_field in (("reducedPositive", "reducedPositiveG"), ("zeroToTwo", "zeroToTwoSeconds"),
                                    ("enhancedLongitudinalOnset", "enhancedLongitudinalOnsetGps")):
            seat[report_field] = {"utilization": values.get(prefix + field, 0.)}
        envelope.append(seat)
    obj["forceEnvelope"] = {"sampleRateHz": 960, "seats": envelope}
    obj["metrics"] = metrics; obj["seatStatistics"] = seats
    return obj


def strict_obj(seed=1, terrain="flat", preset="physics-proof", accepted=True,
               codes=None, completed=True, cancelled=False,
               schema=1, gen=acc.EXPECTED_GENERATOR_VERSION, intensity=None):
    if intensity is None:
        intensity = (preset == "all-records")
    if codes is None:
        errors = [] if accepted else [{"code": "STRICT_REF", "message": "m",
                                       "distance": 0, "actual": 0, "limit": 0}]
    else:
        errors = [{"code": c, "message": "m", "distance": 0,
                   "actual": 0, "limit": 0} for c in codes]
    return fixture_report({"schemaVersion": schema, "generatorVersion": gen, "seed": seed,
            "terrain": terrain, "preset": preset,
            "intensityRequired": intensity, "accepted": accepted,
            "completed": completed, "cancelled": cancelled, "candidate": 0,
            "topology": "stadium-helix", "lengthMeters": 8000,
            "targets": {}, "metrics": {}, "convergence": convergence_obj(), "errors": errors, "warnings": []})


def case(seed=1, terrain="flat", preset="physics-proof", idx=0, cid="case_0000"):
    return {"index": idx, "id": cid, "seed": seed, "preset": preset,
            "terrain": terrain, "candidates": 2, "step": 1/960}


class ManifestTests(unittest.TestCase):
    def test_default_1000_balance(self):
        m = acc.build_manifest(1000, 8, 1 / 960)
        self.assertEqual(len(m), 1000)
        self.assertEqual(sum(1 for c in m if c["preset"] == "physics-proof"), 500)
        self.assertEqual(sum(1 for c in m if c["preset"] == "all-records"), 500)
        self.assertEqual(len({c["seed"] for c in m}), 1000)
        for t in acc.TERRAINS:
            self.assertIn(sum(1 for c in m if c["terrain"] == t), (333, 334))

    def test_deterministic(self):
        self.assertEqual(acc.build_manifest(12, 8, 1/960),
                         acc.build_manifest(12, 8, 1/960))

    def test_unsupported_acceptance_rates_are_rejected(self):
        for step in (1/100, 1/240, 1/1920, float("nan")):
            with self.subTest(step=step), self.assertRaises(ValueError):
                acc.build_manifest(2, 8, step)


class A1StrictContractTests(unittest.TestCase):
    def test_audit_weak_boolean_not_accepted(self):
        # Old harness counted this as accepted (A1 repro).
        obj = {"accepted": True, "completed": False, "cancelled": True,
               "errors": [{"code": "MISSING_I305"}]}
        kind, _, reason = acc.strict_report_check(obj, case())
        self.assertEqual(kind, "invalid")
        cat, _, codes, _ = acc.classify_result(
            0, 1.0, False, obj, True, True, case(), True, True)
        self.assertEqual(cat, "infrastructure")

    def test_strict_accepted_candidate(self):
        obj = strict_obj()
        kind, codes, _ = acc.strict_report_check(obj, case())
        self.assertEqual(kind, "accepted-candidate")
        cat, _, _, _ = acc.classify_result(
            0, 1.0, False, obj, True, True, case(), True, True)
        self.assertEqual(cat, "accepted-pending-validation")

    def test_contradictory_accepted_with_errors(self):
        obj = strict_obj(accepted=True, codes=["X"])
        kind, _, _ = acc.strict_report_check(obj, case())
        self.assertEqual(kind, "invalid")

    def test_contradictory_accepted_not_completed(self):
        obj = strict_obj(accepted=True, completed=False)
        self.assertEqual(acc.strict_report_check(obj, case())[0], "invalid")

    def test_contradictory_empty_rejection(self):
        obj = strict_obj(accepted=False, codes=[])
        obj["errors"] = []
        obj["completed"] = True
        obj["cancelled"] = False
        self.assertEqual(acc.strict_report_check(obj, case())[0], "invalid")

    def test_wrong_preset_seed_terrain(self):
        base = case(seed=1, terrain="flat", preset="physics-proof")
        self.assertEqual(acc.strict_report_check(
            strict_obj(seed=999), base)[0], "invalid")
        self.assertEqual(acc.strict_report_check(
            strict_obj(terrain="hills"), base)[0], "invalid")
        self.assertEqual(acc.strict_report_check(
            strict_obj(preset="all-records", intensity=True), base)[0], "invalid")

    def test_intensity_pin(self):
        # all-records must require intensity; physics-proof is the only exemption.
        pp = case(preset="physics-proof")
        ar = case(preset="all-records")
        self.assertEqual(acc.strict_report_check(
            strict_obj(preset="physics-proof", intensity=False), pp)[0],
            "accepted-candidate")
        self.assertEqual(acc.strict_report_check(
            strict_obj(preset="physics-proof", intensity=True), pp)[0], "invalid")
        self.assertEqual(acc.strict_report_check(
            strict_obj(preset="all-records", intensity=True,
                       accepted=False, codes=["REFERENCE_UNAVAILABLE"]), ar)[0],
            "rejected-candidate")
        self.assertEqual(acc.strict_report_check(
            strict_obj(preset="all-records", intensity=False,
                       accepted=False, codes=["REFERENCE_UNAVAILABLE"]), ar)[0],
            "invalid")

    def test_schema_generator_pin(self):
        self.assertEqual(acc.strict_report_check(
            strict_obj(schema=2), case())[0], "invalid")
        self.assertEqual(acc.strict_report_check(
            strict_obj(gen="9.9.9"), case())[0], "invalid")

    def test_malformed_errors(self):
        for bad in (None, {}, "x", [{"message": "no code"}],
                    [{"code": ""}], ["X"]):
            obj = strict_obj()
            obj["errors"] = bad
            obj["accepted"] = False
            self.assertEqual(acc.strict_report_check(obj, case())[0],
                             "invalid", bad)

    def test_missing_i305_stays_rejected(self):
        ar = case(preset="all-records")
        obj = strict_obj(preset="all-records", intensity=True, accepted=False,
                         codes=["REFERENCE_UNAVAILABLE"])
        kind, codes, _ = acc.strict_report_check(obj, ar)
        self.assertEqual(kind, "rejected-candidate")
        cat, ok, out_codes, _ = acc.classify_result(
            2, 1.0, False, obj, True, False, ar, False, True)
        self.assertEqual(cat, "rejected")
        self.assertIn("REFERENCE_UNAVAILABLE", out_codes)

    def test_exit0_with_valid_rejection_is_infra(self):
        obj = strict_obj(accepted=False, codes=["STRICT_REF"])
        cat, _, _, _ = acc.classify_result(
            0, 1.0, False, obj, True, False, case(), False, True)
        self.assertEqual(cat, "infrastructure")

    def test_exit2_with_accepted_is_infra(self):
        obj = strict_obj(accepted=True)
        cat, _, _, _ = acc.classify_result(
            2, 1.0, False, obj, True, False, case(), False, False)
        self.assertEqual(cat, "infrastructure")

    def test_directory_and_empty_are_infra(self):
        obj = strict_obj()
        cat, _, codes, _ = acc.classify_result(
            0, 1.0, False, None, False, False, case(), False, False)
        self.assertEqual(cat, "infrastructure")
        self.assertIn("NO_OUTPUT", codes)
        cat2, _, codes2, _ = acc.classify_result(
            0, 1.0, False, obj, True, True, case(), False, False)
        self.assertEqual(cat2, "infrastructure")
        self.assertIn("NO_OUTPUT", codes2)
        cat3, _, codes3, _ = acc.classify_result(
            0, 1.0, False, obj, True, True, case(), True, False)
        self.assertEqual(cat3, "infrastructure")
        self.assertIn("NO_TRACE", codes3)


def _cmd_paths(cmd):
    d = {}
    for i, a in enumerate(cmd):
        if a in ("--json", "--trace", "--out") and i + 1 < len(cmd):
            d[a] = cmd[i + 1]
    return d


def _write_fake_cli(path, validate_mode="ok"):
    path.write_text(f'''import json, sys
from pathlib import Path
sys.path.insert(0, {str(Path(__file__).resolve().parent)!r})
from test_acceptance import strict_obj, convergence_obj, fixture_report
a = sys.argv
def g(key): return a[a.index(key)+1]
validating = a[1] == "validate"
if validating:
    parts = Path(a[2]).read_text().split()
    seed, terrain, preset, step = int(parts[1]), parts[2], parts[3], float(parts[4])
    accepted = {validate_mode != "reject"!r}
    codes = [] if accepted else ["REPLAY_REJECTED"]
else:
    seed, terrain, preset, step = int(g("--seed")), g("--terrain"), g("--preset"), float(g("--step"))
    accepted = seed % 2 == 0 and preset == "physics-proof"
    codes = [] if accepted else ["REFERENCE_UNAVAILABLE" if preset == "all-records" else "STRICT_REF"]
    Path(g("--trace")).write_text("{{}}")
    if accepted:
        Path(g("--out")).write_text("COASTER %d %s %s %.17g payload" % (seed, terrain, preset, step))
report = strict_obj(seed=seed, terrain=terrain, preset=preset, accepted=accepted, codes=codes)
report["convergence"] = convergence_obj(step)
Path(g("--json")).write_text(json.dumps(fixture_report(report)))
sys.exit(0 if accepted else 2)
''', encoding="utf-8")


class A1ValidateTests(unittest.TestCase):
    def test_accepted_requires_separate_validate(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            fake = Path(td) / "fake.py"
            _write_fake_cli(fake, "ok")
            cli = [sys.executable, str(fake)]
            out = Path(td) / "out"
            rec = acc.run_one_case(cli, case(seed=2, cid="case_0001", idx=1),
                                   str(out), timeout=30)
            self.assertEqual(rec["category"], "accepted")
            self.assertEqual(rec["replayReturncode"], 0)
            self.assertGreaterEqual(rec["replayElapsed"], 0.0)
            self.assertNotEqual(rec["elapsed"], rec["replayElapsed"])
            # Evidence published from fresh attempt, not stale.
            self.assertTrue((out / "outs" / "case_0001.coaster").exists())
            self.assertTrue((out / "replays" / "case_0001.replay.json").exists())

    def test_failed_revalidation_counts_rejection(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            fake = Path(td) / "fake.py"
            _write_fake_cli(fake, "reject")
            cli = [sys.executable, str(fake)]
            out = Path(td) / "out"
            rec = acc.run_one_case(cli, case(seed=2, cid="case_0001", idx=1),
                                   str(out), timeout=30)
            self.assertEqual(rec["category"], "rejected")
            self.assertFalse(rec["accepted"])
            self.assertIn("REPLAY_REJECTED", rec["errorCodes"])
            self.assertIsNotNone(rec["replayElapsed"])
            # No validated save may linger as success.
            self.assertFalse((out / "outs" / "case_0001.coaster").exists())

    def test_rejected_needs_no_validate(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            fake = Path(td) / "fake.py"
            _write_fake_cli(fake, "ok")
            cli = [sys.executable, str(fake)]
            out = Path(td) / "out"
            rec = acc.run_one_case(cli, case(seed=1, cid="case_0000", idx=0),
                                   str(out), timeout=30)
            self.assertEqual(rec["category"], "rejected")
            self.assertIsNone(rec["replayReturncode"])


class A2StaleTests(unittest.TestCase):
    def test_stale_success_then_exit0_no_output_is_infra(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "out"
            c = case(seed=2, cid="case_0001", idx=1)
            finals = acc._case_paths(str(out), c["id"])
            for p in finals.values():
                p.parent.mkdir(parents=True, exist_ok=True)
            finals["cli_json"].write_text(json.dumps(strict_obj(seed=2)), encoding="utf-8")
            finals["out"].write_text("STALE", encoding="utf-8")
            finals["trace"].write_text("{}", encoding="utf-8")

            def side_effect(cmd, **kw):
                return subprocess.CompletedProcess(cmd, 0, b"", b"")
            with mock.patch.object(subprocess, "run", side_effect=side_effect):
                rec = acc.run_one_case("dummy", c, str(out), timeout=30)
            self.assertEqual(rec["category"], "infrastructure")
            self.assertFalse(rec["accepted"])
            self.assertFalse(finals["out"].exists(),
                             "stale .coaster must not survive a non-accept")

    def test_stale_success_then_rejected_clears_out(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "out"
            c = case(seed=1, cid="case_0000", idx=0)
            finals = acc._case_paths(str(out), c["id"])
            for p in finals.values():
                p.parent.mkdir(parents=True, exist_ok=True)
            finals["out"].write_text("STALE", encoding="utf-8")

            def side_effect(cmd, **kw):
                if len(cmd) > 1 and cmd[1] == "generate":
                    p = _cmd_paths(cmd)
                    Path(p["--json"]).write_text(
                        json.dumps(strict_obj(seed=1, accepted=False,
                                              codes=["STRICT_REF"])), encoding="utf-8")
                    Path(p["--trace"]).write_text("{}", encoding="utf-8")
                    return subprocess.CompletedProcess(cmd, 2, b"", b"")
                raise AssertionError("no validate for rejection")
            with mock.patch.object(subprocess, "run", side_effect=side_effect):
                rec = acc.run_one_case("dummy", c, str(out), timeout=30)
            self.assertEqual(rec["category"], "rejected")
            self.assertFalse(finals["out"].exists())

    def test_stale_success_then_timeout_clears_out(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "out"
            c = case(seed=1, cid="case_0000", idx=0)
            finals = acc._case_paths(str(out), c["id"])
            for p in finals.values():
                p.parent.mkdir(parents=True, exist_ok=True)
            finals["out"].write_text("STALE", encoding="utf-8")
            with mock.patch.object(
                    subprocess, "run",
                    side_effect=subprocess.TimeoutExpired("cli", 1)):
                rec = acc.run_one_case("dummy", c, str(out), timeout=1)
            self.assertEqual(rec["category"], "timeout")
            self.assertFalse(finals["out"].exists())

    def test_safe_remove_refuses_outside(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td1, tempfile.TemporaryDirectory() as td2:
            base = Path(td1)
            outside = Path(td2) / "keep.txt"
            outside.write_text("keep", encoding="utf-8")
            self.assertFalse(acc.safe_remove(str(outside), str(base)))
            self.assertTrue(outside.exists())
            self.assertIsNone(acc.resolve_within(str(base), str(outside)))


class A3ResumeTests(unittest.TestCase):
    def test_inconsistent_wrapper_rejected(self):
        c = case()
        bad = {"version": 2, "id": c["id"], "index": 0, "seed": 1,
               "preset": "physics-proof", "terrain": "flat", "candidates": 2,
               "step": 1/960, "category": "accepted", "accepted": False,
               "errorCodes": [], "elapsed": -99, "returncode": 2,
               "timeout": False}
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "r.json"
            p.write_text(json.dumps(bad), encoding="utf-8")
            self.assertIsNone(acc.load_result_json(str(p), c))

    def test_nan_negative_truncated_old_version(self):
        import tempfile
        c = case()
        for elapsed in (float("nan"), float("inf"), -1.0):
            rec = {"version": 2, "id": c["id"], "index": 0, "seed": 1,
                   "preset": "physics-proof", "terrain": "flat",
                   "candidates": 2, "step": 1/960, "category": "rejected",
                   "accepted": False, "errorCodes": ["X"], "elapsed": elapsed,
                   "returncode": 2, "timeout": False}
            with tempfile.TemporaryDirectory() as td:
                p = Path(td) / "r.json"
                p.write_text(json.dumps(rec), encoding="utf-8")
                self.assertIsNone(acc.load_result_json(str(p), c), elapsed)
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "r.json"
            p.write_text('{"version":2,"id":', encoding="utf-8")
            self.assertIsNone(acc.load_result_json(str(p), c))
            old = {"version": 1, "id": c["id"], "index": 0, "seed": 1,
                   "preset": "physics-proof", "terrain": "flat",
                   "candidates": 2, "step": 1/960, "category": "accepted",
                   "accepted": True, "errorCodes": [], "elapsed": 1.0,
                   "returncode": 0, "timeout": False}
            p.write_text(json.dumps(old), encoding="utf-8")
            self.assertIsNone(acc.load_result_json(str(p), c))

    def test_missing_evidence_and_hash_mismatch(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            fake = Path(td) / "fake.py"
            _write_fake_cli(fake, "ok")
            cli = [sys.executable, str(fake)]
            out = Path(td) / "out"
            rec = acc.run_one_case(cli, case(seed=2, cid="case_0001", idx=1),
                                   str(out), timeout=30)
            self.assertEqual(rec["category"], "accepted")
            exp = {"cliSha256": rec["cliSha256"], "timeout": rec["generationTimeout"],
                   "validateTimeout": rec["validateTimeout"],
                   "reportSchemaVersion": 1, "generatorVersion": acc.EXPECTED_GENERATOR_VERSION,
                   "validationPolicy": acc.VALIDATION_POLICY}
            c = case(seed=2, cid="case_0001", idx=1)
            self.assertTrue(acc.verify_resume_evidence(rec, c, str(out), exp))
            # Missing evidence.
            (out / "outs" / "case_0001.coaster").unlink()
            self.assertFalse(acc.verify_resume_evidence(rec, c, str(out), exp))

    def test_timeout_change_and_binary_replacement_force_rerun(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "out"
            out.mkdir()
            cli_file = Path(td) / "cli.exe"
            cli_file.write_bytes(b"v1")
            # Old manifest with different timeout and no sha binding.
            old_cases = acc.build_manifest(2, 2, 1/960)
            (out / "manifest.json").write_text(json.dumps(
                {"version": 2, "count": 2, "candidates": 2, "step": 0.01,
                 "timeout": 9999, "validateTimeout": 9999,
                 "cliSha256": "old", "reportSchemaVersion": 1,
                 "generatorVersion": "0.5.0-geometry.2",
                 "validationPolicy": acc.VALIDATION_POLICY,
                 "cases": old_cases}), encoding="utf-8")
            for cc in old_cases:
                r = {"version": 2, "id": cc["id"], "index": cc["index"],
                     "seed": cc["seed"], "preset": cc["preset"],
                     "terrain": cc["terrain"], "candidates": cc["candidates"],
                     "step": cc["step"], "category": "accepted",
                     "accepted": True, "errorCodes": [], "elapsed": 1.0,
                     "replayElapsed": 0.5, "returncode": 0,
                     "replayReturncode": 0, "timeout": False,
                     "replayTimeout": False, "note": "stale",
                     "genJsonSha256": None, "traceSha256": None,
                     "outSha256": None, "replayJsonSha256": None,
                     "cliSha256": "old", "timeout": 9999,
                     "validateTimeout": 9999, "reportSchemaVersion": 1,
                     "generatorVersion": "0.5.0-geometry.2",
                     "validationPolicy": acc.VALIDATION_POLICY}
                acc.atomic_write_json(acc._case_paths(str(out), cc["id"])["result"], r)
            calls = []

            def side_effect(cmd, **kw):
                calls.append(cmd)
                if len(cmd) > 1 and cmd[1] == "generate":
                    p = _cmd_paths(cmd)
                    seed = int(cmd[cmd.index("--seed") + 1])
                    Path(p["--json"]).write_text(json.dumps(
                        strict_obj(seed=seed, terrain=cmd[cmd.index("--terrain") + 1],
                                   preset=cmd[cmd.index("--preset") + 1],
                                   accepted=False, codes=["STRICT_REF"])),
                        encoding="utf-8")
                    Path(p["--trace"]).write_text("{}", encoding="utf-8")
                    return subprocess.CompletedProcess(cmd, 2, b"", b"")
                raise AssertionError("unexpected validate")
            with mock.patch.object(subprocess, "run", side_effect=side_effect):
                rc = acc.main(["--cli", str(cli_file), "--out-dir", str(out),
                               "--count", "2", "--timeout", "0.1",
                               "--candidates", "2", "--step", str(1/960),
                               "--resume", "--samples", "2"])
            self.assertTrue(calls, "stale results must be rerun, not reused")
            # Two strict rejections: gates pass (no infra) but nothing fabricated.
            self.assertEqual(rc, 0)
            # Resume must not have fabricated 2/2 accepted.
            summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["config"]["resumed"], 0)
            self.assertEqual(summary["accepted"], 0)
            self.assertEqual(summary["rejected"], 2)


class A4SamplesTests(unittest.TestCase):
    def test_absent_trace_saves_nothing(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "out"
            c = case(seed=2, cid="case_0001", idx=1)
            rec = {"index": 1, "id": "case_0001", "category": "accepted",
                   "accepted": True}
            finals = acc._case_paths(str(out), c["id"])
            for p in finals.values():
                p.parent.mkdir(parents=True, exist_ok=True)
            finals["cli_json"].write_text("{}", encoding="utf-8")
            finals["out"].write_text("x", encoding="utf-8")
            finals["replay"].write_text("{}", encoding="utf-8")
            # trace missing
            info = acc.publish_samples([rec], str(out), 2)
            self.assertEqual(info["saved"], 0)
            self.assertEqual(info["ids"], [])
            self.assertEqual(list((out / "samples").iterdir()), [])

    def test_copy_failure_counts_zero(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "out"
            c = case(seed=2, cid="case_0001", idx=1)
            rec = {"index": 1, "id": "case_0001", "category": "accepted",
                   "accepted": True}
            finals = acc._case_paths(str(out), c["id"])
            for p in finals.values():
                p.parent.mkdir(parents=True, exist_ok=True)
            for k in ("cli_json", "trace", "out", "replay"):
                finals[k].write_text("x", encoding="utf-8")
            with mock.patch.object(acc, "atomic_copy", return_value=False):
                info = acc.publish_samples([rec], str(out), 1)
            self.assertEqual(info["saved"], 0)
            self.assertEqual(info["failures"], ["case_0001"])

    def test_stale_samples_cleared(self):
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "out"
            samples = out / "samples"
            samples.mkdir(parents=True)
            (samples / "old.txt").write_text("stale", encoding="utf-8")
            info = acc.publish_samples([], str(out), 1)
            self.assertEqual(info["saved"], 0)
            self.assertFalse((samples / "old.txt").exists())


class AggregateTests(unittest.TestCase):
    def test_denominator_and_split_latency(self):
        def rec(i, cat, preset="physics-proof"):
            return {"index": i, "id": f"case_{i:04d}", "preset": preset,
                    "category": cat, "accepted": cat == "accepted",
                    "errorCodes": [] if cat == "accepted" else ["E"],
                    "elapsed": 1.0, "replayElapsed": 0.5 if cat == "accepted" else 0.0,
                    "replayReturncode": 0 if cat == "accepted" else None}
        recs = [rec(0, "accepted"), rec(1, "rejected"),
                rec(2, "infrastructure"), rec(3, "timeout")]
        s = acc.aggregate_results(recs)
        self.assertEqual(s["total"], 4)
        self.assertAlmostEqual(s["successRate"], 0.25)
        self.assertIn("replay", s["latency"])
        self.assertIn("total", s["latency"])
        self.assertEqual(s["latency"]["replay"]["count"], 1)


class BuildScriptTests(unittest.TestCase):
    PATH = Path(__file__).resolve().parent / "build.ps1"

    def test_clean_boundary_and_reparse(self):
        txt = self.PATH.read_text(encoding="utf-8")
        self.assertIn("ResolvedNative", txt)
        self.assertIn("ExpectedBuild", txt)
        self.assertIn("ReparsePoint", txt)
        self.assertIn("remove the link only", txt)
        self.assertIn("-LiteralPath", txt)
        self.assertNotIn("rm -rf", txt)

    def test_no_auto_download(self):
        txt = self.PATH.read_text(encoding="utf-8").lower()
        self.assertIn("never downloads", txt)



class AuditFollowupTests(unittest.TestCase):
    def _run(self, root, mode="ok"):
        fake = root / "fake.py"
        _write_fake_cli(fake, mode)
        c = case(seed=2)
        out = root / "out"
        return acc.run_one_case([sys.executable, str(fake)], c, out, 30), c, out

    def test_success_record_roundtrips_through_full_resume_gate(self):
        with tempfile.TemporaryDirectory() as td:
            rec, c, out = self._run(Path(td))
            self.assertIs(rec["timeout"], False)
            self.assertEqual(rec["generationTimeout"], 30)
            loaded = acc.load_result_json(acc._case_paths(out, c["id"])["result"], c)
            self.assertIsNotNone(loaded)
            expected = {"cliSha256": rec["cliSha256"], "timeout": 30, "validateTimeout": 30}
            self.assertTrue(acc.verify_resume_evidence(loaded, c, out, expected))

    def test_failed_publication_cannot_accept_and_retains_attempt(self):
        with tempfile.TemporaryDirectory() as td:
            with mock.patch.object(acc, "atomic_copy", return_value=False):
                rec, c, out = self._run(Path(td))
            self.assertEqual(rec["category"], "infrastructure")
            self.assertFalse(rec["accepted"])
            self.assertIn("EVIDENCE_PUBLISH_FAILED", rec["errorCodes"])
            retained = Path(rec["attemptDirectory"])
            self.assertTrue((retained / "gen.json").is_file())
            self.assertTrue((retained / "out.coaster").is_file())
            self.assertTrue((retained / "replay.json").is_file())
            self.assertEqual(acc.aggregate_results([rec])["accepted"], 0)

    def test_failed_result_write_cannot_leave_old_resumable_success(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            first, c, out = self._run(root)
            self.assertEqual(first["category"], "accepted")
            with mock.patch.object(acc, "atomic_write_json", side_effect=OSError("synthetic disk full")):
                second, _, _ = self._run(root)
            self.assertEqual(second["category"], "infrastructure")
            self.assertFalse(acc._case_paths(out, c["id"])["result"].exists())
            self.assertTrue(Path(second["attemptDirectory"]).is_dir())

    def test_stale_reports_and_trace_are_removed_on_no_output(self):
        with tempfile.TemporaryDirectory() as td:
            rec, c, out = self._run(Path(td))
            with mock.patch.object(subprocess, "run", return_value=subprocess.CompletedProcess([], 0)):
                second = acc.run_one_case("dummy", c, out, 30)
            self.assertEqual(second["category"], "infrastructure")
            for key in ("cli_json", "trace", "out", "replay"):
                self.assertFalse(acc._case_paths(out, c["id"])[key].exists(), key)

    def test_replay_rejection_keeps_evidence_and_resumes(self):
        with tempfile.TemporaryDirectory() as td:
            rec, c, out = self._run(Path(td), "reject")
            replay = acc._case_paths(out, c["id"])["replay"]
            self.assertTrue(replay.is_file())
            loaded = acc.load_result_json(acc._case_paths(out, c["id"])["result"], c)
            self.assertIsNotNone(loaded)
            self.assertTrue(acc.verify_resume_evidence(loaded, c, out,
                {"cliSha256": rec["cliSha256"], "timeout": 30, "validateTimeout": 30}))

    def test_same_configuration_main_run_reuses_actual_results(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); fake = root / "fake.py"; out = root / "out"
            _write_fake_cli(fake)
            args = ["--cli", str(fake), "--out-dir", str(out), "--count", "4",
                    "--workers", "1", "--timeout", "30", "--samples", "1"]
            with mock.patch.object(acc, "_base_cmd", return_value=[sys.executable, str(fake)]):
                self.assertEqual(acc.main(args), 0)
            with mock.patch.object(acc, "run_one_case", side_effect=AssertionError("resume should not execute")):
                self.assertEqual(acc.main(args + ["--resume"]), 0)
            summary = json.loads((out / "summary.json").read_text())
            self.assertEqual(summary["config"]["resumed"], 4)
            self.assertEqual(summary["samples"]["saved"], 1)

    def test_all_records_acceptance_requires_reference_binding(self):
        obj = strict_obj(preset="all-records")
        self.assertEqual(acc.strict_report_check(obj, case(preset="all-records"))[0], "invalid")
        obj["targets"] = {"referenceExposure": 10, "referenceId": "SYNTHETIC-reference"}
        self.assertEqual(acc.strict_report_check(obj, case(preset="all-records"))[0], "accepted-candidate")

    @unittest.skipUnless(os.name == "nt", "Windows extended path spelling")
    def test_resolved_extended_windows_paths_have_same_containment(self):
        with tempfile.TemporaryDirectory() as td:
            base = Path(td).resolve()
            target = base / "evidence" / "case.json"
            prefix = chr(92)*2 + "?" + chr(92)
            extended_base, extended_target = Path(prefix+str(base)), Path(prefix+str(target))
            for b, t in ((base, extended_target), (extended_base, target)):
                with mock.patch.object(Path, "resolve", side_effect=[b, t]):
                    self.assertEqual(acc.resolve_within(base, target), target)
            escaped = Path(prefix+str(base.parent / "outside" / "case.json"))
            with mock.patch.object(Path, "resolve", side_effect=[base, escaped]):
                self.assertIsNone(acc.resolve_within(base, escaped))

    @unittest.skipUnless(os.name == "nt", "Windows extended UNC spelling")
    def test_resolved_extended_unc_is_normalized_without_resolving_device_paths(self):
        slash = chr(92)
        base = Path(slash*2 + "server" + slash + "share" + slash + "out")
        target = base / "case.json"
        extended = Path(slash*2 + "?" + slash + "UNC" + slash + str(target)[2:])
        with mock.patch.object(Path, "resolve", side_effect=[base, extended]):
            self.assertEqual(acc.resolve_within(base, target), target)
        device = Path(slash*2 + "?" + slash + "GLOBALROOT" + slash + "Device")
        with mock.patch.object(Path, "resolve", side_effect=[base, device]):
            self.assertIsNone(acc.resolve_within(base, device))

    def test_ancestor_junction_cannot_escape_output_boundary(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); base = root / "base"; outside = root / "outside"
            base.mkdir(); outside.mkdir(); victim = outside / "keep.txt"; victim.write_text("keep")
            link = base / "junction"
            try:
                link.symlink_to(outside, target_is_directory=True)
            except OSError:
                if os.name != "nt": self.skipTest("directory links unavailable")
                env = dict(os.environ, COASTER_TEST_LINK=str(link), COASTER_TEST_TARGET=str(outside))
                proc = subprocess.run(["powershell", "-NoProfile", "-Command",
                    "New-Item -ItemType Junction -Path $env:COASTER_TEST_LINK -Value $env:COASTER_TEST_TARGET | Out-Null"],
                    env=env, capture_output=True)
                if proc.returncode: self.skipTest("junction creation unavailable")
            target = link / "keep.txt"
            # Both endpoints are deliberately inside this owned temporary fixture.
            self.assertIn(root.resolve(), target.resolve().parents)
            self.assertIsNone(acc.resolve_within(base, target))
            self.assertFalse(acc.safe_remove(target, base))
            self.assertEqual(victim.read_text(), "keep")

if __name__ == "__main__":
    unittest.main()
