"""SYNTHETIC TESTS ONLY - fetcher evidence taxonomy offline (R5)."""
from __future__ import annotations

import io
import json
import pathlib
import sys
import tempfile
import unittest
import urllib.error

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import reference_fetch as rf


class FakeResp:
    def __init__(self, status, body):
        self.status = status
        self._body = body

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def read(self):
        return self._body


def patch_urlopen(fn):
    import urllib.request
    orig = urllib.request.urlopen
    urllib.request.urlopen = fn
    return orig


class TestFetchTaxonomy(unittest.TestCase):
    def test_sha256_stable(self):
        h = rf.sha256_hex(b"false")
        self.assertEqual(len(h), 64)
        self.assertEqual(h, rf.sha256_hex(b"false"))
        self.assertNotEqual(h, rf.sha256_hex(b"true"))

    def test_ride_table_has_four(self):
        self.assertEqual(
            {v["rideInfo_id"] for v in rf.RIDES.values()},
            {6839, 4718, 6383, 4804},
        )

    def test_http_error_body_preserved(self):
        import urllib.request
        body = b"raw denial body"
        def fake(req, timeout=None):
            raise urllib.error.HTTPError(req.full_url, 403, "Forbidden", {}, io.BytesIO(body))
        orig = patch_urlopen(fake)
        try:
            st, got, err = rf.single_get("https://example.invalid/x", 5)
        finally:
            urllib.request.urlopen = orig
        self.assertEqual(st, 403)
        self.assertEqual(got, body)
        self.assertIn("403", err)

    def test_login_html_not_verified_raw(self):
        cls = rf.classify_getrec(200, b"<html>Login required</html>")
        self.assertFalse(cls["raw_accessible_without_login"])
        self.assertIn("non-trace", cls["probe_outcome"] + cls["raw_status"])

    def test_false_is_observed_not_verified_login(self):
        for payload in (b"false", b" false\n"):
            cls = rf.classify_getrec(200, payload)
            self.assertEqual(cls["probe_outcome"], "observed-false")
            self.assertFalse(cls["raw_accessible_without_login"])
            self.assertNotIn("verified-raw", cls["raw_status"])
            self.assertIn("NOT", cls["raw_status"])

    def test_malformed_json_schema_invalid(self):
        cls = rf.classify_getrec(200, b"{not json")
        self.assertFalse(cls["raw_accessible_without_login"])
        self.assertIn("schema-invalid", cls["probe_outcome"] + cls["raw_status"])

    def test_transport_failure_taxonomy(self):
        cls = rf.classify_getrec("transport-error", b"")
        self.assertFalse(cls["raw_accessible_without_login"])
        self.assertIn("probe-failed", cls["probe_outcome"] + cls["raw_status"])

    def test_valid_trace_shape_verified(self):
        body = json.dumps(canonical_raw_fixture()).encode()
        cls = rf.classify_getrec(200, body)
        self.assertTrue(cls["raw_accessible_without_login"])
        self.assertEqual(cls["probe_outcome"], "verified-raw")

    def test_failed_refetch_preserves_prior(self):
        import urllib.request
        with tempfile.TemporaryDirectory() as td:
            raw = pathlib.Path(td) / "raw"
            raw.mkdir()
            prior = b'{"name":"X","location":"Y","num":0,"recordings":[]}'
            (raw / "rideInfo-1.json").write_bytes(prior)
            def fake(req, timeout=None):
                raise urllib.error.URLError("down")
            orig = patch_urlopen(fake)
            try:
                e = rf.fetch_rideinfo("pantherian", 1, raw, 5, "2026-09-06T00:00:00Z")
            finally:
                urllib.request.urlopen = orig
            self.assertEqual((raw / "rideInfo-1.json").read_bytes(), prior)
            self.assertEqual(e["probe_outcome"], "transport-failure")
            self.assertEqual(e["sha256"], rf.sha256_hex(prior))

    def test_summary_not_probed_vs_observed(self):
        with tempfile.TemporaryDirectory() as td:
            raw = pathlib.Path(td) / "raw"
            raw.mkdir()
            s = rf.build_summary([], [], raw, "2026-09-06T00:00:00Z")
            self.assertIn("not-probed", s["raw_trace_status"])
            probes = [{
                "probe_outcome": "observed-false", "recording_id": 1,
                "http_status": 200, "sha256": "x", "source_url": "u",
            }]
            s2 = rf.build_summary([], probes, raw, "2026-09-06T00:00:00Z")
            self.assertIn("observed-false", s2["raw_trace_status"])
            self.assertNotIn("verified", s2["raw_trace_status"])

    def test_summary_marks_peaks_not_exposure(self):
        with tempfile.TemporaryDirectory() as td:
            raw = pathlib.Path(td) / "raw"
            raw.mkdir()
            body = json.dumps({
                "name": "SYNTHETIC Ride", "location": "Nowhere", "num": 1,
                "recordings": [{"id": 1, "author": "x", "seat": "Row 1",
                                "quality": 10, "advanced": 0,
                                "statistics": {"maxy": 4.5, "miny": -1.0,
                                               "maxc": 4.6}}],
            }).encode()
            (raw / "rideInfo-1.json").write_bytes(body)
            ride_entries = [{
                "ride_slug": "pantherian", "rideInfo_id": 1,
                "source_url": "https://example.invalid/rideInfo?id=1",
                "retrieved_utc": "2026-09-06T00:00:00Z", "http_status": 200,
                "sha256": "abc", "bytes": len(body),
                "raw_file": "raw/rideInfo-1.json", "parsed_ok": True,
                "probe_outcome": "verified-metadata",
            }]
            orig = dict(rf.RIDES)
            rf.RIDES["pantherian"] = {"rideInfo_id": 1, "label": "SYNTHETIC"}
            try:
                s = rf.build_summary(ride_entries, [], raw, "2026-09-06T00:00:00Z")
            finally:
                rf.RIDES.clear()
                rf.RIDES.update(orig)
            txt = json.dumps(s)
            self.assertIn("INSTANTANEOUS", txt)
            self.assertIn("ten_second_exposure", txt)



def canonical_raw_fixture():
    # Synthetic canonical schema, not a captured RFDB payload.
    return {"trace_version": 1, "recording_id": 1, "ride": "SYNTHETIC-Ride",
            "device": "SYNTHETIC-Device", "seat": "SYNTHETIC-Seat",
            "source": "SYNTHETIC-fixture", "configuration": "SYNTHETIC-Config",
            "calibration": {"status": "known", "calibration_id": "SYNTHETIC-Cal",
                            "convention": "rider-vertical-specific-force-with-g",
                            "frame": "rider", "transform": "none"},
            "samples": [{"time_s": 0., "vertical_g": 1., "lateral_g": 0., "longitudinal_g": 0.},
                        {"time_s": .1, "vertical_g": 1., "lateral_g": 0., "longitudinal_g": 0.}]}

class RawSchemaFollowupTests(unittest.TestCase):
    def test_empty_body_is_not_an_observed_false(self):
        for body in (b"", b" \n"):
            result = rf.classify_getrec(200, body)
            self.assertEqual(result["probe_outcome"], "empty-response")
            self.assertFalse(result["raw_accessible_without_login"])

    def test_empty_arbitrary_uncalibrated_or_untimed_samples_are_not_raw(self):
        import copy
        variants = [{"samples": []}, {"samples": ["arbitrary"]},
                    {"trace_version": 1, "samples": [{"t": 0, "v": 1}]}]
        for key in ("calibration", "recording_id", "ride", "trace_version"):
            obj = canonical_raw_fixture(); del obj[key]; variants.append(obj)
        for change in ({"time_s": 0}, {"time_s": float("nan")},
                       {"vertical_g": float("inf")}, {"lateral_g": True}):
            obj = canonical_raw_fixture(); obj["samples"][1].update(change); variants.append(obj)
        obj = canonical_raw_fixture(); del obj["samples"][1]["longitudinal_g"]; variants.append(obj)
        obj = canonical_raw_fixture(); obj["calibration"]["status"] = "unknown"; variants.append(obj)
        for obj in variants:
            with self.subTest(obj=obj):
                self.assertFalse(rf.classify_getrec(200, json.dumps(obj).encode())["raw_accessible_without_login"])

    def test_probe_identity_must_match_the_recording_returned(self):
        from unittest import mock
        with tempfile.TemporaryDirectory() as td:
            raw = pathlib.Path(td)
            with mock.patch.object(rf, "single_get", return_value=(200, json.dumps(canonical_raw_fixture()).encode(), "")):
                result = rf.probe_getrec(999, raw, 1, "SYNTHETIC-time")
            self.assertFalse(result["raw_accessible_without_login"])
            self.assertEqual((raw / "getRec-999.response.json").read_bytes(), json.dumps(canonical_raw_fixture()).encode())

if __name__ == "__main__":
    unittest.main()
