"""SYNTHETIC TESTS ONLY - never measured reference data.

Failing-first regressions for verification-audit R1-R4/R6.
Stdlib unittest only.
"""
from __future__ import annotations

import csv
import itertools
import json
import os
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import reference_forces as rf

_counter = itertools.count(1)


def good_manifest(**over):
    n = next(_counter)
    m = {
        "ride": "SYNTHETIC-TestCoaster",
        "device": "SYNTHETIC-Phone",
        "seat": "SYNTHETIC-Row1",
        "source": "SYNTHETIC-generator",
        "configuration": "SYNTHETIC-config-A",
        "recording_id": f"SYNTHETIC-REC-{n:05d}",
        "calibration": {
            "status": "known",
            "convention": "rider-vertical-specific-force-with-g",
            "frame": "rider",
            "transform": "none",
            "calibration_id": "SYNTHETIC-CAL-A",
            "smoothing": "none",
        },
        "quality": {},
    }
    if "calibration" in over and isinstance(over["calibration"], dict):
        base = dict(m["calibration"])
        base.update(over.pop("calibration"))
        m["calibration"] = base
    m.update(over)
    return m


def write_csv(path: pathlib.Path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "vertical_g", "lateral_g", "longitudinal_g"])
        w.writerows(rows)


class TestExactStrongest10s(unittest.TestCase):
    """R1: direct function tests, independent of gap quality."""

    def test_triangle_interior_75(self):
        r = rf.strongest_10s([0.0, 10.0, 20.0], [0.0, 10.0, 0.0])
        self.assertEqual(r["status"], "ok")
        self.assertAlmostEqual(r["S_g_s"], 75.0, places=9)
        self.assertAlmostEqual(r["t0_s"], 5.0, places=9)

    def test_triangle_irregular_matches_exact(self):
        T = sorted([0.13] + [i * 0.4 for i in range(51)])
        V = [10.0 - abs(t - 10.0) for t in T]
        r = rf.strongest_10s(T, V)
        self.assertAlmostEqual(r["S_g_s"], 75.0, places=9)
        self.assertAlmostEqual(r["t0_s"], 5.0, places=6)

    def test_zero_crossing_clipped(self):
        # Ramps through zero; rectified integral must clip exactly.
        # Compare exact S against dense brute-force max (0.01 grid).
        times = [0.0, 6.0, 12.0, 18.0, 24.0]
        vals = [-2.0, 4.0, -1.0, 3.0, -2.0]
        exact = rf.strongest_10s(times, vals)
        best = 0.0
        t = times[0]
        while t <= times[-1] - 10.0 + 1e-9:
            best = max(best, rf.integral_rectified_window(t, times, vals))
            t += 0.01
        self.assertAlmostEqual(exact["S_g_s"], best, places=6)

    def test_offset_origin(self):
        r = rf.strongest_10s([100.0, 110.0, 120.0], [0.0, 10.0, 0.0])
        self.assertAlmostEqual(r["S_g_s"], 75.0, places=9)
        self.assertAlmostEqual(r["t0_s"], 105.0, places=9)

    def test_plateau_tie_earliest(self):
        times = [round(i * 0.25, 9) for i in range(61)]  # 0..15s
        vals = [2.0] * len(times)
        r = rf.strongest_10s(times, vals)
        self.assertAlmostEqual(r["S_g_s"], 20.0, places=9)
        self.assertAlmostEqual(r["t0_s"], times[0], places=9)

    def test_exactly_10s(self):
        r = rf.strongest_10s([7.0, 12.0, 17.0], [2.0, 2.0, 2.0])
        self.assertEqual(r["status"], "ok")
        self.assertAlmostEqual(r["S_g_s"], 20.0, places=9)

    def test_short_trace(self):
        r = rf.strongest_10s([0.0, 5.0], [1.0, 1.0])
        self.assertEqual(r["status"], "insufficient-duration")
        self.assertIsNone(r["S_g_s"])

    def test_constant_1g_irregular_is_10gs(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            times = [0.0, 0.13, 0.5, 1.7, 3.3, 7.9, 10.0, 12.5]
            write_csv(p, [(t, 1.0, 0.0, 0.0) for t in times])
            res = rf.analyze_recording(p, good_manifest())
            self.assertEqual(res["strongest10s"]["status"], "ok")
            self.assertAlmostEqual(res["strongest10s"]["S_g_s"], 10.0, places=9)

    def test_exact_10s_block_boundaries(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = []
            t = 0.0
            while t <= 20.0 + 1e-9:
                v = 2.0 if 5.0 <= t <= 15.0 else 1.0
                rows.append((round(t, 9), v, 0.0, 0.0))
                t += 0.5
            write_csv(p, rows)
            res = rf.analyze_recording(p, good_manifest())
            self.assertAlmostEqual(res["strongest10s"]["S_g_s"], 20.0, places=9)
            self.assertAlmostEqual(res["strongest10s"]["t0_s"], 5.0, places=9)

    def test_negative_clipping(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), -1.0, 0.0, 0.0) for i in range(44)]
            rows += [(round(11.0 + i * 0.25, 9), 2.0, 0.0, 0.0) for i in range(44)]
            write_csv(p, rows)
            res = rf.analyze_recording(p, good_manifest())
            self.assertAlmostEqual(res["strongest10s"]["S_g_s"], 20.0, places=6)

    def test_all_negative_gives_zero(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), -1.0, 0.0, 0.0) for i in range(49)]
            write_csv(p, rows)
            res = rf.analyze_recording(p, good_manifest())
            self.assertAlmostEqual(res["strongest10s"]["S_g_s"], 0.0, places=9)

    def test_spike_flags_but_not_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.1, 9), 1.0, 0.0, 0.0) for i in range(121)]
            rows[60] = (rows[60][0], 15.0, 0.0, 0.0)
            write_csv(p, rows)
            res = rf.analyze_recording(p, good_manifest())
            self.assertTrue(res["flags"]["spike_review"])
            self.assertTrue(res["eligibility"]["eligible"])

    def test_time_weighted_mean_no_resample(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            write_csv(p, [(0.0, 0.0, 0.0, 0.0), (9.0, 0.0, 0.0, 0.0),
                          (10.0, 10.0, 0.0, 0.0), (20.0, 0.0, 0.0, 0.0)])
            res = rf.analyze_recording(p, good_manifest())
            self.assertAlmostEqual(
                res["axes"]["vertical_g"]["time_weighted_mean_g"], 2.75, places=9)


class TestStrictThresholds(unittest.TestCase):
    """R6: strict over/below exclude full equal plateaus."""

    def test_plateaus_at_equality_zero(self):
        for thr, above in [(0.0, False), (-0.5, False), (2.0, True),
                           (3.0, True), (4.0, True)]:
            with self.subTest(thr=thr):
                self.assertAlmostEqual(
                    rf.time_above([0.0, 10.0], [thr, thr], thr, above), 0.0, places=12)

    def test_threshold_duration_uses_interpolation(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            write_csv(p, [(0.0, 0.0, 0.0, 0.0), (4.0, 4.0, 0.0, 0.0),
                          (14.0, 4.0, 0.0, 0.0)])
            res = rf.analyze_recording(p, good_manifest())
            self.assertAlmostEqual(res["durations_s"]["vertical_over_2g_s"], 12.0, places=9)
            # constant-at-threshold exports must be strict (tested via function)
            self.assertAlmostEqual(rf.time_above([0.0, 5.0], [2.0, 2.0], 2.0, True), 0.0)


class TestValidation(unittest.TestCase):
    def test_malformed_missing_column(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            with open(p, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["time_s", "vertical_g", "lateral_g"])
                w.writerow([0.0, 1.0, 0.0])
                w.writerow([1.0, 1.0, 0.0])
            with self.assertRaises(rf.ForceValidationError):
                rf.analyze_recording(p, good_manifest())

    def test_time_reversal_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            write_csv(p, [(0.0, 1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0),
                          (0.5, 1.0, 0.0, 0.0), (12.0, 1.0, 0.0, 0.0)])
            with self.assertRaises(rf.ForceValidationError):
                rf.analyze_recording(p, good_manifest())

    def test_gapped_flagged_and_ineligible(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            write_csv(p, [(0.0, 1.0, 0.0, 0.0), (0.1, 1.0, 0.0, 0.0),
                          (5.0, 1.0, 0.0, 0.0), (5.1, 1.0, 0.0, 0.0),
                          (16.0, 1.0, 0.0, 0.0)])
            res = rf.analyze_recording(p, good_manifest())
            self.assertTrue(res["flags"]["gapped"])
            self.assertFalse(res["eligibility"]["eligible"])

    def test_unknown_calibration_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), 1.0, 0.0, 0.0) for i in range(49)]
            write_csv(p, rows)
            bad = good_manifest()
            bad["calibration"] = dict(bad["calibration"], status="unknown")
            with self.assertRaises(rf.CalibrationError):
                rf.analyze_recording(p, bad)

    def test_device_axes_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), 1.0, 0.0, 0.0) for i in range(49)]
            write_csv(p, rows)
            for conv in ("device-axes-with-g", "world-axes-with-g", "not-with-g", "mystery"):
                bad = good_manifest(calibration={"convention": conv})
                with self.subTest(conv=conv):
                    with self.assertRaises(rf.CalibrationError):
                        rf.analyze_recording(p, bad)

    def test_transform_unsupported_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), 1.0, 0.0, 0.0) for i in range(49)]
            write_csv(p, rows)
            bad = good_manifest(calibration={"transform": {"type": "rotate", "R": [1]}})
            with self.assertRaises(rf.CalibrationError):
                rf.analyze_recording(p, bad)

    def test_empty_identity_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), 1.0, 0.0, 0.0) for i in range(49)]
            write_csv(p, rows)
            for key in ("ride", "configuration", "recording_id"):
                bad = good_manifest(**{key: "   "})
                with self.subTest(key=key):
                    with self.assertRaises(rf.ForceValidationError):
                        rf.analyze_recording(p, bad)
            bad = good_manifest(calibration={"calibration_id": "  "})
            with self.assertRaises(rf.ForceValidationError):
                rf.analyze_recording(p, bad)

    def test_stationarity_unverified_without_interval(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), 1.0, 0.0, 0.0) for i in range(49)]
            write_csv(p, rows)
            res = rf.analyze_recording(p, good_manifest())
            self.assertEqual(res["flags"]["stationarity"]["status"], "unverified")

    def test_stationarity_grounded_with_interval(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), 1.0, 0.0, 0.0) for i in range(49)]
            write_csv(p, rows)
            m = good_manifest(calibration={
                "stationary_reference": {"interval_s": [0.0, 2.0], "expected_g": 1.0}})
            res = rf.analyze_recording(p, m)
            self.assertEqual(res["flags"]["stationarity"]["status"], "ok")

    def test_raw_input_unchanged(self):
        with tempfile.TemporaryDirectory() as d:
            p = pathlib.Path(d) / "synth.csv"
            rows = [(round(i * 0.25, 9), 1.0, 0.0, 0.0) for i in range(49)]
            write_csv(p, rows)
            before = p.read_bytes()
            rf.analyze_recording(p, good_manifest())
            self.assertEqual(p.read_bytes(), before)


class TestOverwriteGuard(unittest.TestCase):
    """R2: CLI must never overwrite inputs, including alias paths."""

    def _fixtures(self, d: pathlib.Path):
        csv_p = d / "in.csv"
        man_p = d / "in.manifest.json"
        rows = [(round(i * 0.25, 9), 1.0, 0.0, 0.0) for i in range(49)]
        write_csv(csv_p, rows)
        man_p.write_text(json.dumps(good_manifest()), encoding="utf-8")
        return csv_p, man_p, rows

    def test_identical_csv_out_refused(self):
        with tempfile.TemporaryDirectory() as td:
            d = pathlib.Path(td)
            csv_p, man_p, _ = self._fixtures(d)
            before = csv_p.read_bytes()
            with self.assertRaises(rf.OutputAliasError):
                rf.main(["analyze", "--csv", str(csv_p), "--manifest", str(man_p),
                         "--out", str(csv_p)])
            self.assertEqual(csv_p.read_bytes(), before)

    def test_relative_alias_refused(self):
        with tempfile.TemporaryDirectory() as td:
            d = pathlib.Path(td)
            csv_p, man_p, _ = self._fixtures(d)
            alias = d / "." / "in.csv"
            before = csv_p.read_bytes()
            with self.assertRaises(rf.OutputAliasError):
                rf.main(["analyze", "--csv", str(csv_p), "--manifest", str(man_p),
                         "--out", str(alias)])
            self.assertEqual(csv_p.read_bytes(), before)

    def test_manifest_target_refused(self):
        with tempfile.TemporaryDirectory() as td:
            d = pathlib.Path(td)
            csv_p, man_p, _ = self._fixtures(d)
            before = man_p.read_bytes()
            with self.assertRaises(rf.OutputAliasError):
                rf.main(["analyze", "--csv", str(csv_p), "--manifest", str(man_p),
                         "--out", str(man_p)])
            self.assertEqual(man_p.read_bytes(), before)

    def test_aggregate_input_refused(self):
        with tempfile.TemporaryDirectory() as td:
            d = pathlib.Path(td)
            csv_p, man_p, _ = self._fixtures(d)
            a_out = d / "a.json"
            rf.main(["analyze", "--csv", str(csv_p), "--manifest", str(man_p),
                     "--out", str(a_out)])
            before = a_out.read_bytes()
            with self.assertRaises(rf.OutputAliasError):
                rf.main(["aggregate", "--inputs", str(a_out), "--out", str(a_out)])
            self.assertEqual(a_out.read_bytes(), before)

    def test_symlink_hardlink_alias_refused(self):
        with tempfile.TemporaryDirectory() as td:
            d = pathlib.Path(td)
            csv_p, man_p, _ = self._fixtures(d)
            link = d / "link.csv"
            made = False
            try:
                try:
                    os.symlink(str(csv_p), str(link))
                    made = True
                except Exception:
                    os.link(str(csv_p), str(link))
                    made = True
            except Exception:
                self.skipTest("links unavailable")
            if made:
                before = csv_p.read_bytes()
                with self.assertRaises(rf.OutputAliasError):
                    rf.main(["analyze", "--csv", str(csv_p), "--manifest", str(man_p),
                             "--out", str(link)])
                self.assertEqual(csv_p.read_bytes(), before)


class TestGrouping(unittest.TestCase):
    """R4: recording_id + canonical hash independence; split groups."""

    def _analysis(self, value, seat="SYNTHETIC-Row1", config="SYNTHETIC-config-A",
                  ride="SYNTHETIC-Ride", device="SYNTHETIC-Phone",
                  cal_id="SYNTHETIC-CAL-A", rec_id=None, smoothing="none"):
        d = tempfile.mkdtemp()
        safe = str(seat).replace(" ", "_").replace("/", "_")
        rid = rec_id or f"SYNTHETIC-REC-{next(_counter):05d}"
        p = pathlib.Path(d) / f"synth_{value}_{safe}_{rid}.csv"
        rows = [(round(i * 0.25, 9), float(value), 0.0, 0.0) for i in range(49)]
        write_csv(p, rows)
        m = good_manifest(ride=ride, seat=seat, configuration=config,
                          device=device, recording_id=rid,
                          calibration={"calibration_id": cal_id, "smoothing": smoothing})
        return rf.analyze_recording(p, m)

    def test_median_needs_three_homogeneous(self):
        trio = [self._analysis(v) for v in (1.0, 2.0, 3.0)]
        agg = rf.aggregate_recordings(trio)
        self.assertEqual(len(agg["groups"]), 1)
        g = agg["groups"][0]
        self.assertEqual(g["status"], "ok")
        self.assertAlmostEqual(g["median_S_g_s"], 20.0, places=9)

    def test_two_is_insufficient(self):
        pair = [self._analysis(v) for v in (1.0, 2.0)]
        agg = rf.aggregate_recordings(pair)
        self.assertEqual(agg["groups"][0]["status"], "insufficient")
        self.assertIsNone(agg["groups"][0]["median_S_g_s"])

    def test_different_configs_not_pooled(self):
        a = self._analysis(1.0, config="SYNTHETIC-config-A")
        b = self._analysis(1.0, config="SYNTHETIC-config-B")
        agg = rf.aggregate_recordings([a, b])
        self.assertEqual(len(agg["groups"]), 2)

    def test_seats_split_not_pooled(self):
        trio = [self._analysis(2.0 + i * 0.1, seat=f"SYNTHETIC-Row{i}") for i in (1, 2, 7)]
        agg = rf.aggregate_recordings(trio)
        self.assertEqual(len(agg["groups"]), 3)
        self.assertTrue(all(g["status"] == "insufficient" for g in agg["groups"]))

    def test_recalibrations_split(self):
        trio = [self._analysis(2.0, cal_id=f"SYNTHETIC-CAL-{i}") for i in ("A", "B", "C")]
        agg = rf.aggregate_recordings(trio)
        self.assertEqual(len(agg["groups"]), 3)

    def test_smoothing_divergence_flagged(self):
        trio = [self._analysis(2.0 + i * 0.1, smoothing=s) for i, s in
                enumerate(["none", "smooth5", "smooth9"])]
        # Same seat/device/cal id but different smoothing -> heterogeneous.
        agg = rf.aggregate_recordings(trio)
        self.assertEqual(agg["groups"][0]["status"], "heterogeneous-needs-split")

    def test_duplicate_reencodings_count_once(self):
        # Same recording_id, same numerics, different byte spellings.
        with tempfile.TemporaryDirectory() as td:
            d = pathlib.Path(td)
            times = [round(i * 0.25, 9) for i in range(49)]
            variants = ["1.0", "1.00", "1.000"]
            analyses = []
            for spell in variants:
                p = d / f"re_{spell.replace('.', '_')}.csv"
                with open(p, "w", newline="", encoding="utf-8") as f:
                    f.write("time_s,vertical_g,lateral_g,longitudinal_g\n")
                    for t in times:
                        f.write(f"{t},{spell},0.0,0.0\n")
                m = good_manifest(recording_id="SYNTHETIC-REC-DUP",
                                  calibration={"calibration_id": "SYNTHETIC-CAL-A"})
                analyses.append(rf.analyze_recording(p, m))
            hashes = {a["canonical_samples_sha256"] for a in analyses}
            self.assertEqual(len(hashes), 1)
            agg = rf.aggregate_recordings(analyses)
            total_indep = sum(g["n_independent"] for g in agg["groups"])
            self.assertLessEqual(total_indep, 1)

    def test_same_id_divergent_export_flagged(self):
        a = self._analysis(1.0, rec_id="SYNTHETIC-REC-SAME")
        b = self._analysis(2.0, rec_id="SYNTHETIC-REC-SAME")
        agg = rf.aggregate_recordings([a, b])
        self.assertIn("SYNTHETIC-REC-SAME", agg["divergent_reexports_same_id"])

    def test_missing_identity_insufficient(self):
        a = self._analysis(1.0)
        del a["manifest"]["recording_id"]
        a.pop("recording_id", None)
        agg = rf.aggregate_recordings([a])
        self.assertEqual(agg["overall"], "unavailable")

    def test_unavailable_when_nothing_eligible(self):
        bench = rf.unavailable_benchmark("SYNTHETIC: no eligible real traces")
        self.assertEqual(bench["status"], "unavailable")
        self.assertIsNone(bench["median_S_g_s"])
        agg = rf.aggregate_recordings([])
        self.assertEqual(agg["overall"], "unavailable")



class AuditCalibrationFollowupTests(unittest.TestCase):
    def _analysis(self, root, value, rid, stationary=None):
        path = root / (rid + "-" + str(value) + ".csv")
        write_csv(path, [(i*.25, value, 0., 0.) for i in range(49)])
        cal = {"calibration_id": "SYNTHETIC-stable-session"}
        if stationary is not None: cal["stationary_reference"] = stationary
        return rf.analyze_recording(path, good_manifest(recording_id=rid, calibration=cal))

    def test_stationarity_mismatch_and_invalid_metadata_are_ineligible(self):
        with tempfile.TemporaryDirectory() as td:
            root = pathlib.Path(td)
            for ref in ({"interval_s": [0,2], "expected_g": 1},
                        {"interval_s": [-1,2]}, {"interval_s": [0,2], "expected_g": 5}):
                res = self._analysis(root, 5., "SYNTHETIC-bad-cal", ref)
                self.assertFalse(res["eligibility"]["eligible"], ref)
            valid = self._analysis(root, 1., "SYNTHETIC-good-cal", {"interval_s": [0,2]})
            self.assertTrue(valid["eligibility"]["eligible"])
            # A ride-only trace may rely on explicit known calibration metadata.
            no_interval = self._analysis(root, 2., "SYNTHETIC-external-cal")
            self.assertTrue(no_interval["eligibility"]["eligible"])

    def test_cached_eligible_claim_cannot_override_failed_stationarity(self):
        with tempfile.TemporaryDirectory() as td:
            root = pathlib.Path(td)
            for stationary in ({"interval_s": [0,2], "expected_g": 1},
                               {"interval_s": [0,2], "expected_g": 5},
                               {"interval_s": [-1,2]}):
                inputs = [self._analysis(root, v, "SYNTHETIC-"+str(v), stationary)
                          for v in (5.,6.,7.)]
                # Model cached analysis produced before eligibility checked this flag.
                for a in inputs: a["eligibility"] = {"eligible": True, "reasons": []}
                self.assertEqual(rf.aggregate_recordings(inputs)["overall"], "unavailable")

    def test_conflicting_cached_stationarity_is_excluded_in_every_order(self):
        with tempfile.TemporaryDirectory() as td:
            import copy
            root = pathlib.Path(td)
            a = self._analysis(root, 1., "SYNTHETIC-A", {"interval_s": [0,2]})
            bad = copy.deepcopy(a)
            bad["flags"]["stationarity"].update(status="mismatch", mean_g=5.)
            others = [self._analysis(root, v, "SYNTHETIC-"+str(v)) for v in (2.,3.)]
            for inputs in ([a,bad]+others, [bad,a]+others):
                self.assertEqual(rf.aggregate_recordings(inputs)["overall"], "unavailable")

    def test_divergent_same_recording_is_excluded_in_every_input_order(self):
        with tempfile.TemporaryDirectory() as td:
            root = pathlib.Path(td)
            a = self._analysis(root, 1., "SYNTHETIC-A")
            b = self._analysis(root, 100., "SYNTHETIC-A")
            c = self._analysis(root, 2., "SYNTHETIC-B")
            d = self._analysis(root, 3., "SYNTHETIC-C")
            for inputs in ([a,b,c,d], [b,a,c,d], [d,c,b,a]):
                agg = rf.aggregate_recordings(inputs)
                self.assertEqual(agg["overall"], "unavailable")
                self.assertLessEqual(sum(g["n_eligible"] for g in agg["groups"]), 2)
                self.assertIn("SYNTHETIC-A", agg["divergent_reexports_same_id"])

    def test_same_samples_with_conflicting_quality_cannot_choose_good_first(self):
        with tempfile.TemporaryDirectory() as td:
            import copy
            root = pathlib.Path(td)
            a = self._analysis(root, 1., "SYNTHETIC-A")
            bad = copy.deepcopy(a)
            bad["manifest"]["quality"]["excluded_statistics"] = True
            bad["eligibility"] = {"eligible": False, "reasons": ["excluded"]}
            others = [self._analysis(root, v, "SYNTHETIC-"+str(v)) for v in (2.,3.)]
            for inputs in ([a,bad]+others, [bad,a]+others):
                self.assertEqual(rf.aggregate_recordings(inputs)["overall"], "unavailable")

if __name__ == "__main__":
    unittest.main()
