"""Focused analytic tests; synthetic signals are never ride-reference evidence."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ride_load_profile as profile


def trace_of(function, rate=100, duration=1.0):
    times = [i/rate for i in range(round(duration*rate)+1)]
    return {"sampleRateHz": rate, "seatOrder": ["front", "middle", "rear"],
            "frames": [{"time": time, "seats": [list(function(time, seat)) for seat in range(3)]} for time in times]}


class ForceProfileTests(unittest.TestCase):
    def test_constant_signal_zero_onset_and_full_duration(self):
        trace = trace_of(lambda time, seat: (1.0, 0.0, 0.0))
        result = profile.profile_trace(trace, ["vertical:gt:0.5", "vertical:gt:1"])
        for seat in result["seats"].values():
            active, equality = seat["forceDurationRuns"]
            self.assertEqual(active["totalSeconds"], 1.0)
            self.assertEqual(len(active["runs"]), 1)
            self.assertTrue(active["runs"][0]["touchesTraceStart"])
            self.assertTrue(active["runs"][0]["touchesTraceEnd"])
            self.assertEqual(equality["runs"], [])
            self.assertEqual(seat["onset100ms"]["vertical"]["maximumGps"], 0.0)
        self.assertFalse(result["acceptanceAssessed"])
        self.assertFalse(result["standardComplianceAssessed"])

    def test_signed_linear_ramps_exact_all_seats_and_axes(self):
        trace = trace_of(lambda time, seat: (1 + .7*time, -(seat+1)*time, 2*(seat+1)*time))
        result = profile.profile_trace(trace, ["lateral:lt:-0.25"], True)
        for index, name in enumerate(profile.SEATS):
            data = result["seats"][name]
            for axis, slope in (("vertical", .7), ("lateral", -(index+1)), ("longitudinal", 2*(index+1))):
                for center, actual in data["onset100ms"][axis]["seriesTimeSecondsSlopeGps"]:
                    self.assertAlmostEqual(actual, slope, places=11)
                    self.assertGreaterEqual(center-.05, 0)
                    self.assertLessEqual(center+.05, 1)
            self.assertAlmostEqual(data["forceDurationRuns"][0]["totalSeconds"], 1-.25/(index+1), places=12)

    def test_linear_crossings_are_interpolated_not_quantized(self):
        trace = trace_of(lambda time, seat: (time, 0, 0))
        result = profile.profile_trace(trace, ["vertical:gt:0.253", "vertical:lt:0.253"])
        above, below = result["seats"]["front"]["forceDurationRuns"]
        self.assertAlmostEqual(above["runs"][0]["startTime"], .253, places=12)
        self.assertAlmostEqual(above["totalSeconds"], .747, places=12)
        self.assertAlmostEqual(below["totalSeconds"], .253, places=12)

    def test_triangle_duration_and_equality_plateau_splits(self):
        times = [i/100 for i in range(101)]
        values = [1 - abs(time-.5)*2 for time in times]
        runs = profile.threshold_runs(times, values, "gt", .5)
        self.assertEqual(len(runs), 1)
        self.assertAlmostEqual(runs[0]["startTime"], .25)
        self.assertAlmostEqual(runs[0]["endTime"], .75)
        self.assertFalse(runs[0]["touchesTraceStart"])
        self.assertFalse(runs[0]["touchesTraceEnd"])
        plateau = profile.threshold_runs([0, .1, .2, .3], [2, 1, 1, 2], "gt", 1)
        self.assertEqual(len(plateau), 2)
        self.assertAlmostEqual(sum(run["durationSeconds"] for run in plateau), .2)

    def test_full_window_no_edge_extrapolation(self):
        trace = trace_of(lambda time, seat: (4*time, 0, 0), rate=100, duration=.2)
        result = profile.profile_trace(trace, ["vertical:gt:0"], True)
        series = result["seats"]["front"]["onset100ms"]["vertical"]["seriesTimeSecondsSlopeGps"]
        self.assertTrue(series)
        self.assertTrue(all(.05 <= row[0] <= .15 for row in series))
        self.assertTrue(all(abs(row[1]-4) < 1e-12 for row in series))
        self.assertGreater(result["seats"]["front"]["onset100ms"]["vertical"]["unassessedEdgeCenters"], 0)

    def test_quadratic_fit_uses_centered_window(self):
        trace = trace_of(lambda time, seat: (time*time, 0, 0))
        result = profile.profile_trace(trace, ["vertical:gt:0"], True)
        for time, slope in result["seats"]["front"]["onset100ms"]["vertical"]["seriesTimeSecondsSlopeGps"]:
            self.assertAlmostEqual(slope, 2*time, places=11)

    def test_short_terminal_interval_uses_actual_time(self):
        trace = trace_of(lambda time, seat: (3*time, 0, 0))
        trace["frames"][-1] = {"time": .997, "seats": [[3*.997, 0, 0] for _ in range(3)]}
        result = profile.profile_trace(trace, ["vertical:gt:0.75"], True)
        data = result["seats"]["front"]
        self.assertAlmostEqual(result["finalIntervalSeconds"], .007, places=12)
        self.assertAlmostEqual(data["forceDurationRuns"][0]["totalSeconds"], .997-.25, places=12)
        self.assertTrue(all(row[0]+.05 <= .997 for row in data["onset100ms"]["vertical"]["seriesTimeSecondsSlopeGps"]))

    def test_nonuniform_interior_gap_and_long_terminal_rejected(self):
        original = trace_of(lambda time, seat: (1, 0, 0))
        for index, offset in ((40, .001), (-1, .001)):
            trace = copy.deepcopy(original)
            trace["frames"][index]["time"] += offset
            with self.subTest(index=index), self.assertRaises(ValueError):
                profile.profile_trace(trace, ["vertical:gt:0"])

    def test_invalid_times_forces_and_seat_schema_rejected(self):
        original = trace_of(lambda time, seat: (1, 0, 0))
        mutations = [
            lambda t: t["frames"][3].update(time=math.nan),
            lambda t: t["frames"][3].update(time=math.inf),
            lambda t: t["frames"][3].update(time=.02),
            lambda t: t["frames"][0].update(time=-.01),
            lambda t: t["frames"][3]["seats"][2].__setitem__(1, math.nan),
            lambda t: t["frames"][3]["seats"][1].__setitem__(0, True),
            lambda t: t.update(seatOrder=["rear", "middle", "front"]),
            lambda t: t.update(sampleRateHz=0),
        ]
        for mutation in mutations:
            trace = copy.deepcopy(original)
            mutation(trace)
            with self.assertRaises(ValueError):
                profile.profile_trace(trace, ["vertical:gt:0"])

    def test_missing_invalid_duplicate_thresholds_rejected(self):
        trace = trace_of(lambda time, seat: (1, 0, 0))
        for thresholds in ([], ["vertical:gt:nan"], ["lateral:abs:0.5"], ["x:gt:1"], ["vertical:gt:1", "vertical:gt:1"]):
            with self.subTest(thresholds=thresholds), self.assertRaises(ValueError):
                profile.profile_trace(trace, thresholds)

    def test_trace_too_short_has_no_invented_window(self):
        trace = trace_of(lambda time, seat: (time, 0, 0), duration=.08)
        with self.assertRaises(ValueError):
            profile.profile_trace(trace, ["vertical:gt:0"])

    def test_cli_preserves_source_and_refuses_output_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            source, output = folder/"trace.json", folder/"profile.json"
            source.write_text(json.dumps(trace_of(lambda time, seat: (time, 0, 0))))
            original = source.read_bytes()
            self.assertEqual(profile.main([str(source), "--threshold", "vertical:gt:0.3", "--output", str(output)]), 0)
            self.assertEqual(source.read_bytes(), original)
            saved = output.read_bytes()
            with self.assertRaises(SystemExit) as caught:
                profile.main([str(source), "--threshold", "vertical:gt:0.3", "--output", str(output)])
            self.assertEqual(caught.exception.code, 2)
            self.assertEqual(output.read_bytes(), saved)


if __name__ == "__main__":
    unittest.main()
