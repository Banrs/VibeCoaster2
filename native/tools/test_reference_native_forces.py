"""Synthetic-only coverage of source semantics; no third-party trace fixtures."""
import copy
import csv
import hashlib
import json
import pathlib
import tempfile
import unittest

import reference_forces as rf
import reference_native_forces as native


def fixture():
    vec = lambda x, y, z: dict(zip("xyz", map(str, (x, y, z))))
    return {
        "name": "SYNTHETIC ride", "device": "SYNTHETIC sensor",
        "uuid": "11111111-2222-4333-8444-555555555555", "ridingPosition": "Seated",
        "sampleRate": 50, "startIndex": 5, "endIndex": 25,
        "gravityIndex": 0, "inclineIndex": 1, "smoothingFactor": 1,
        "xCalibrationVector": vec(-1, 0, 0), "yCalibrationVector": vec(0, -1, 0),
        "zCalibrationVector": vec(0, 0, -1),
        "accelerationSequence": [vec(0, -1, 0) for _ in range(31)],
        "attributes": [{"label": "Row", "value": "2"}, {"label": "Seat", "value": "8"}],
    }


class NativeForcesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = pathlib.Path(self.temp.name)

    def convert(self, data=None, **kwargs):
        source = self.root / "synthetic.forces"
        source.write_text(json.dumps(data or fixture()), encoding="utf-8")
        output = self.root / "result"
        manifest = native.convert(source, output, **kwargs)
        with (output / "canonical.csv").open(newline="") as stream:
            rows = [dict((k, float(v)) for k, v in row.items()) for row in csv.DictReader(stream)]
        return source, output, manifest, rows

    def test_stationary_with_gravity_is_one_and_never_eligible(self):
        source, output, m, rows = self.convert(rfdb_id=123)
        self.assertTrue(all(r["vertical_g"] == 1 for r in rows))
        self.assertEqual(m["seat"], "Row 2, Seat 8")
        self.assertEqual(m["provenance"]["original_sha256"], hashlib.sha256(source.read_bytes()).hexdigest())
        for name, digest in m["provenance"]["outputs_sha256"].items():
            self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), digest)
        self.assertFalse(m["benchmark_eligible"])
        with self.assertRaises(rf.ForceValidationError):
            rf.analyze_recording(output / "canonical.csv", m)

    def test_axis_basis_signs_and_inversion(self):
        data = fixture()
        # Source negative-identity rows: native right=-X, up=+Y, forward=+Z.
        cases = [(0, -1, 0), (0, -1, -2), (0, -1, 2), (-2, -1, 0), (2, -1, 0), (0, 1, 0)]
        for i, v in enumerate(cases, 5):
            data["accelerationSequence"][i] = dict(zip("xyz", v))
        _, _, _, rows = self.convert(data)
        expected = [(1, 0, 0), (1, 0, 2), (1, 0, -2), (1, -2, 0), (1, 2, 0), (-1, 0, 0)]
        self.assertEqual([tuple(r[k] for k in rf.REQUIRED_COLUMNS[1:]) for r in rows[:6]], expected)

    def test_native_specific_force_basis_matches_both_turn_signs(self):
        data = fixture()
        # Independent physical oracle: F=U*v + (T cross U)*l + T*f.
        up, right, forward = (0, 1, 0), (-1, 0, 0), (0, 0, 1)
        expected = [(1, 0, 0), (1, 0, 2), (1, 0, -2), (1, 2, 0), (1, -2, 0), (-1, 0, 0)]
        for i, (v, lateral, longitudinal) in enumerate(expected, 5):
            physical = [up[k]*v + right[k]*lateral + forward[k]*longitudinal for k in range(3)]
            data["accelerationSequence"][i] = dict(zip("xyz", [-f for f in physical]))
        _, output, _, rows = self.convert(data)
        self.assertEqual([tuple(r[k] for k in rf.REQUIRED_COLUMNS[1:]) for r in rows[:6]], expected)
        with (output / "rfdb-smoothed-comparison.csv").open(newline="") as stream:
            self.assertIn("rfdb_lateral_g", csv.DictReader(stream).fieldnames)
        with self.assertRaises(rf.ForceValidationError):
            rf.load_samples(output / "rfdb-smoothed-comparison.csv")

    def test_banked_incline_and_forward_acceleration_keep_gravity_projections(self):
        data = fixture()
        # A tilted native orthonormal frame, R=T cross U, in world XYZ.
        up, right, forward = (-.64, -.6, .48), (.48, -.8, -.36), (.6, 0, .8)
        for key, row in zip("xyz", (right, [-v for v in up], [-v for v in forward])):
            data[key + "CalibrationVector"] = dict(zip("xyz", row))
        # At rest F=worldUp; with forward acceleration F=worldUp+2*T.
        for i, physical in enumerate(((0, 0, 1), (1.2, 0, 2.6)), 5):
            data["accelerationSequence"][i] = dict(zip("xyz", [-v for v in physical]))
        _, _, _, rows = self.convert(data)
        for row, expected in zip(rows, ((.48, -.36, .8), (.48, -.36, 2.8))):
            for key, value in zip(rf.REQUIRED_COLUMNS[1:], expected):
                self.assertAlmostEqual(row[key], value, places=14)

    def test_smoothing_full_recording_before_crop_and_boundary_exclusion(self):
        data = fixture()
        data["accelerationSequence"][4]["y"] = -12  # Outside crop, inside first sample's filter.
        data["accelerationSequence"][25]["y"] = -100  # Integration boundary is preserved.
        _, output, m, rows = self.convert(data)
        with (output / "rfdb-smoothed-comparison.csv").open(newline="") as stream:
            smoothed = list(csv.DictReader(stream))
        self.assertEqual(rows[0]["vertical_g"], 1)
        self.assertEqual(float(smoothed[0]["vertical_g"]), 2)
        timing = m["provenance"]["timing"]
        self.assertEqual(timing["csv_indices_inclusive"], [5, 25])
        self.assertEqual(rows[-1]["time_s"], .4)
        self.assertEqual(rows[-1]["vertical_g"], 100)
        self.assertTrue(timing["rfdb_statistics_end_exclusive"])
        self.assertTrue(timing["end_boundary_sample_included"])

    def test_smoothing_edges_and_short_series(self):
        values = list(range(20))
        expected = [sum(values[max(0, i-5):i+6])/len(values[max(0, i-5):i+6]) for i in range(20)]
        self.assertEqual(native.rfdb_smooth(values), expected)
        self.assertEqual(native.rfdb_smooth([1, 3]), [2, 2])

    def test_end_at_length_does_not_invent_missing_boundary(self):
        data = fixture()
        data["endIndex"] = len(data["accelerationSequence"])
        _, _, m, rows = self.convert(data)
        timing = m["provenance"]["timing"]
        self.assertFalse(timing["end_boundary_sample_included"])
        self.assertEqual(timing["rfdb_boundary_duration_s"], .52)
        self.assertEqual(rows[-1]["time_s"], .5)

    def test_gravity_sequence_is_not_subtracted_and_unknown_seat_preserved(self):
        data = fixture()
        data["attributes"] = []
        data["gravitySequence"] = copy.deepcopy(data["accelerationSequence"])
        _, _, m, rows = self.convert(data)
        self.assertIsNone(m["seat"])
        self.assertTrue(m["provenance"]["gravity_sequence_present"])
        self.assertTrue(all(r["vertical_g"] == 1 for r in rows))

    def test_page_seat_is_separate_from_export_attributes(self):
        _, _, m, _ = self.convert(seat="Row 12, Left seat, Blue train")
        self.assertEqual(m["seat"], "Row 12, Left seat, Blue train")
        self.assertEqual(m["provenance"]["source_attributes"], fixture()["attributes"])
        self.assertIsNone(m["calibration"]["calibration_id"])

    def test_malformed_and_unsupported_data_fails_before_output(self):
        mutations = [lambda d: d.update(sampleRate=60), lambda d: d.update(sampleRate=True),
                     lambda d: d.update(ridingPosition="Flying"), lambda d: d.update(startIndex=-1),
                     lambda d: d.update(endIndex=32), lambda d: d.update(endIndex=5),
                     lambda d: d.update(gravityIndex=31), lambda d: d.update(gravitySequence=[]),
                     lambda d: d.update(uuid="not-an-id"),
                     lambda d: d["yCalibrationVector"].update(y=1),  # Wrong handedness.
                     lambda d: d["xCalibrationVector"].update(x=-2),
                     lambda d: d["accelerationSequence"][0].update(x="nan"),
                     lambda d: d["accelerationSequence"][0].update(x=True)]
        for mutate in mutations:
            with self.subTest(mutation=mutations.index(mutate)):
                data = fixture()
                mutate(data)
                with self.assertRaises(rf.ForceValidationError):
                    self.convert(data)
                self.assertFalse((self.root / "result").exists())

    def test_existing_output_and_source_alias_cannot_be_overwritten(self):
        source, output, _, _ = self.convert()
        before = {p.name: p.read_bytes() for p in output.iterdir()}
        with self.assertRaises(rf.OutputAliasError):
            native.convert(source, output)
        original = source.read_bytes()
        with self.assertRaises(rf.OutputAliasError):
            native.convert(source, source)
        self.assertEqual(source.read_bytes(), original)
        self.assertEqual({p.name: p.read_bytes() for p in output.iterdir()}, before)


if __name__ == "__main__":
    unittest.main()
