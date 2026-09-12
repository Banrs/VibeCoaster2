#!/usr/bin/env python3
"""Synthetic-only unit tests for inspect_rides.py (tempdirs, no recorded data)."""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import inspect_rides as ir


def synth_trace(n_frames=180, radius=30.0, seed_phase=0.0):
    # Core convention: right-handed, x/y ground plane, Z UP.
    frames = []
    speed = 15.0
    for i in range(n_frames):
        t = i / 60.0
        d = speed * t
        frames.append({
            "time": t, "distance": d, "speed": speed,
            "seats": [
                [1.0 + 0.5 * math.sin(t * 2.0 + seed_phase + s),
                 0.1 * math.sin(t * 3.0 + s),
                 0.05 * math.cos(t * 1.5 + s)]
                for s in range(3)
            ],
        })
    n_geo = 12
    geo = []
    for i in range(n_geo):
        ang = 2.0 * math.pi * i / n_geo + seed_phase
        geo.append([i * 5.0, radius * math.cos(ang), radius * math.sin(ang),
                    10.0 + 2.0 * math.sin(ang),
                    0.0, 0.0, 1.0, i % 3])
    return {"sampleRateHz": 60, "seatOrder": ["front", "middle", "rear"],
            "frames": frames, "geometry": geo}


def synth_rect_trace():
    # Synthetic rectangle on the XY ground plane at elevated Z.
    # Ground x/y in {0..100}/{0..50}, height z in {50..65}: any y/z swap
    # in the plot mapping is numerically visible here.
    corners = [(0.0, 0.0, 50.0), (100.0, 0.0, 55.0),
               (100.0, 50.0, 60.0), (0.0, 50.0, 65.0), (0.0, 0.0, 50.0)]
    geo = [[i * 5.0, x, y, z, 0.0, 0.0, 1.0, 0] for i, (x, y, z) in enumerate(corners)]
    frames = [{"time": i / 60.0, "distance": 15.0 * i / 60.0, "speed": 15.0,
               "seats": [[1.0, 0.0, 0.0]] * 3} for i in range(10)]
    return {"sampleRateHz": 60, "seatOrder": ["front", "middle", "rear"],
            "frames": frames, "geometry": geo}


def strict_summary(**over):
    s = {"accepted": True, "completed": True, "cancelled": False, "errors": []}
    s.update(over)
    return s


def write_trace(path: Path, obj) -> Path:
    path.write_text(json.dumps(obj), encoding="utf-8")
    return path


class ValidateTests(unittest.TestCase):
    def test_valid_synthetic_closed_loop(self):
        tr = ir.validate_trace(synth_trace())
        self.assertEqual(len(tr["frames"]), 180)
        self.assertEqual(len(tr["geometry"]), 12)

    def test_empty_file_is_invalid(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "empty.trace.json"
            p.write_text("", encoding="utf-8")
            with self.assertRaises(Exception):
                ir.validate_trace(ir.load_json(p))

    def test_malformed_json_raises_on_load(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "bad.trace.json"
            p.write_text("{not json", encoding="utf-8")
            with self.assertRaises(ValueError):
                ir.load_json(p)

    def test_truncated_json_raises_on_load(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "trunc.trace.json"
            full = json.dumps(synth_trace(n_frames=10))
            p.write_text(full[: len(full) // 2], encoding="utf-8")
            with self.assertRaises(ValueError):
                ir.load_json(p)

    def test_missing_keys_rejected(self):
        with self.assertRaises(ir.TraceError):
            ir.validate_trace({"sampleRateHz": 60})
        bad = synth_trace()
        del bad["geometry"]
        with self.assertRaises(ir.TraceError):
            ir.validate_trace(bad)

    def test_wrong_rate_or_seats_rejected(self):
        bad = synth_trace()
        bad["sampleRateHz"] = 30
        with self.assertRaises(ir.TraceError):
            ir.validate_trace(bad)
        bad2 = synth_trace()
        bad2["seatOrder"] = ["front"]
        with self.assertRaises(ir.TraceError):
            ir.validate_trace(bad2)


class AxisMappingTests(unittest.TestCase):
    def test_geometry_series_z_up(self):
        g = ir.geometry_series(synth_rect_trace())
        self.assertEqual(g["x"], [0.0, 100.0, 100.0, 0.0, 0.0])
        self.assertEqual(g["y"], [0.0, 0.0, 50.0, 50.0, 0.0])
        self.assertEqual(g["z"], [50.0, 55.0, 60.0, 65.0, 50.0])

    def test_plotted_line_data_uses_plan_xy_elevation_sz(self):
        import matplotlib.pyplot as plt
        tr = ir.validate_trace(synth_rect_trace())
        fig, mapped, _ = ir.figure_for_ride(
            tr, Path("rect.trace.json"), "0" * 64, "unknown", None)
        try:
            plan_x, plan_y = mapped["plan"]
            elev_x, elev_y = mapped["elevation"]
            # Plan carries ground x/y (ranges 100/50), never height z (>=50).
            self.assertEqual(plan_x, [0.0, 100.0, 100.0, 0.0, 0.0])
            self.assertEqual(plan_y, [0.0, 0.0, 50.0, 50.0, 0.0])
            self.assertLess(min(plan_y), 50.0)
            # Elevation carries distance s vs height z (all >= 50).
            self.assertEqual(elev_x, [0.0, 5.0, 10.0, 15.0, 20.0])
            self.assertEqual(elev_y, [50.0, 55.0, 60.0, 65.0, 50.0])
            self.assertGreaterEqual(min(elev_y), 50.0)
            # 3D path keeps (x, y ground, z up) ordering.
            p3 = mapped["paths3d"][0]
            self.assertEqual(p3["x"], [0.0, 100.0, 100.0, 0.0, 0.0])
            self.assertEqual(p3["y"], [0.0, 0.0, 50.0, 50.0, 0.0])
            self.assertEqual(p3["z"], [50.0, 55.0, 60.0, 65.0, 50.0])
            # Axis labels state the convention explicitly.
            labels = [a.get_xlabel() + a.get_ylabel()
                      for a in fig.get_axes() if not a.name.startswith("3d")]
            self.assertTrue(any("ground" in t for t in labels))
        finally:
            plt.close(fig)


class StrictAcceptanceTests(unittest.TestCase):
    def test_conjunction_required(self):
        self.assertTrue(ir.is_summary_accepted(strict_summary()))
        self.assertFalse(ir.is_summary_accepted({"accepted": True}))
        self.assertFalse(ir.is_summary_accepted(strict_summary(completed=False)))
        self.assertFalse(ir.is_summary_accepted(strict_summary(cancelled=True)))
        self.assertFalse(ir.is_summary_accepted(
            strict_summary(errors=[{"code": "X"}])))
        self.assertFalse(ir.is_summary_accepted(None))

    def test_preset_matching(self):
        s = strict_summary(preset="physics-proof")
        self.assertTrue(ir.is_summary_accepted(s, "physics-proof"))
        self.assertFalse(ir.is_summary_accepted(s, "all-records"))
        self.assertTrue(ir.is_summary_accepted(s))  # no filter, no constraint

    def test_status_labels(self):
        self.assertEqual(ir.validation_status(strict_summary()), "accepted")
        self.assertEqual(ir.validation_status({"accepted": False}), "rejected")
        self.assertEqual(ir.validation_status(
            strict_summary(cancelled=True)), "rejected")
        self.assertEqual(ir.validation_status(None), "unknown")
        self.assertEqual(ir.validation_status({"accepted": True}), "unknown")

    def test_accepted_only_rejects_incomplete_and_preset_mismatch(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            src = td / "src"
            src.mkdir()
            write_trace(src / "ok.trace.json", synth_trace(n_frames=30))
            write_trace(src / "half.trace.json", synth_trace(n_frames=30))
            write_trace(src / "err.trace.json", synth_trace(n_frames=30))
            (src / "ok.summary.json").write_text(
                json.dumps(strict_summary(preset="physics-proof")), encoding="utf-8")
            (src / "half.summary.json").write_text(
                json.dumps({"accepted": True}), encoding="utf-8")
            (src / "err.summary.json").write_text(
                json.dumps(strict_summary(errors=[{"code": "X"}])), encoding="utf-8")
            out = td / "out"
            self.assertEqual(
                ir.main([str(src), "--out-dir", str(out), "--accepted-only",
                         "--preset", "physics-proof"]), 0)
            man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man["counts"]["rendered"], 1)
            self.assertTrue(man["rides"][0]["source"].endswith("ok.trace.json"))
            self.assertEqual(man["rides"][0]["validationStatus"], "accepted")
            self.assertEqual(man["rides"][0]["summaryPreset"], "physics-proof")
            self.assertEqual(man["counts"]["skippedNotAccepted"], 2)


class ContactGridTests(unittest.TestCase):
    def test_grid_has_no_empty_cells_up_to_4(self):
        self.assertEqual(ir.contact_grid(1), (1, 1))
        self.assertEqual(ir.contact_grid(2), (1, 2))
        self.assertEqual(ir.contact_grid(3), (1, 3))
        self.assertEqual(ir.contact_grid(4), (2, 2))
        self.assertEqual(ir.contact_grid(6), (2, 3))

    def test_single_ride_omits_contact_but_manifest_correct(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            tp = write_trace(td / "ride.trace.json", synth_trace(n_frames=60))
            out = td / "out"
            self.assertEqual(ir.main([str(tp), "--out-dir", str(out)]), 0)
            man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man["counts"]["rendered"], 1)
            self.assertEqual(man["contactSheets"], [])

    def test_two_rides_share_one_sheet(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            src = td / "src"
            src.mkdir()
            write_trace(src / "a.trace.json", synth_trace(n_frames=30))
            write_trace(src / "b.trace.json", synth_trace(n_frames=30))
            out = td / "out"
            self.assertEqual(ir.main([str(src), "--out-dir", str(out)]), 0)
            man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man["counts"]["rendered"], 2)
            self.assertEqual(len(man["contactSheets"]), 1)


class RenderSmokeTests(unittest.TestCase):
    def test_render_smoke_closed_loop(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            tp = write_trace(td / "ride.trace.json", synth_trace(n_frames=120))
            out = td / "out"
            rc = ir.main([str(tp), "--out-dir", str(out)])
            self.assertEqual(rc, 0)
            pngs = list((out / "plots").glob("*.png"))
            self.assertEqual(len(pngs), 1)
            self.assertGreater(pngs[0].stat().st_size, 0)
            man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man["counts"]["rendered"], 1)
            self.assertEqual(man["contactSheets"], [])  # single ride: omitted

    def test_manifest_hash_and_unknown_status(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            tp = write_trace(td / "a.trace.json", synth_trace(n_frames=60))
            out = td / "out"
            self.assertEqual(ir.main([str(tp), "--out-dir", str(out)]), 0)
            man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man["rides"][0]["validationStatus"], "unknown")
            self.assertEqual(man["rides"][0]["sourceSha256"], ir.sha256_file(tp))
            self.assertEqual(man["rides"][0]["plot"], f"plots/{man['rides'][0]['plot'].split('/')[-1]}")

    def test_invalid_listed_not_silent(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            write_trace(td / "good.trace.json", synth_trace(n_frames=60))
            (td / "bad.trace.json").write_text("{oops", encoding="utf-8")
            (td / "empty.trace.json").write_text("", encoding="utf-8")
            out = td / "out"
            self.assertEqual(ir.main([str(td), "--out-dir", str(out)]), 0)
            man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man["counts"]["rendered"], 1)
            self.assertEqual(man["contactSheets"], [])  # single ride: omitted
            self.assertEqual(man["counts"]["invalid"], 2)
            paths = [e["path"] for e in man["invalidFiles"]]
            self.assertTrue(any("bad.trace.json" in p for p in paths))
            self.assertTrue(any("empty.trace.json" in p for p in paths))

    def test_deterministic_selection(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            src = td / "src"
            src.mkdir()
            for name in ["e.trace.json", "a.trace.json", "c.trace.json",
                         "b.trace.json", "d.trace.json"]:
                write_trace(src / name, synth_trace(n_frames=30))
            out = td / "out"
            self.assertEqual(ir.main([str(src), "--out-dir", str(out), "--max-traces", "2"]), 0)
            man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
            rendered = [Path(r["source"]).name for r in man["rides"]]
            self.assertEqual(rendered, ["a.trace.json", "b.trace.json"])
            self.assertEqual(man["counts"]["skippedTruncated"], 3)
            # Repeat run is identical.
            out2 = td / "out2"
            self.assertEqual(ir.main([str(src), "--out-dir", str(out2), "--max-traces", "2"]), 0)
            man2 = json.loads((out2 / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual([r["source"] for r in man2["rides"]],
                             [r["source"] for r in man["rides"]])

    def test_accepted_only_requires_summary(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            src = td / "src"
            src.mkdir()
            write_trace(src / "ok.trace.json", synth_trace(n_frames=30))
            write_trace(src / "no.trace.json", synth_trace(n_frames=30))
            (src / "ok.summary.json").write_text(
                json.dumps(strict_summary()), encoding="utf-8")
            (src / "no.summary.json").write_text(
                json.dumps({"accepted": False, "completed": False,
                            "cancelled": False, "errors": [{"code": "X"}]}),
                encoding="utf-8")
            out = td / "out"
            self.assertEqual(ir.main([str(src), "--out-dir", str(out), "--accepted-only"]), 0)
            man = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man["counts"]["rendered"], 1)
            self.assertTrue(man["rides"][0]["source"].endswith("ok.trace.json"))
            self.assertEqual(man["rides"][0]["validationStatus"], "accepted")
            # No summary -> skipped under accepted-only.
            src2 = td / "src2"
            src2.mkdir()
            write_trace(src2 / "lone.trace.json", synth_trace(n_frames=30))
            out3 = td / "out3"
            self.assertEqual(ir.main([str(src2), "--out-dir", str(out3), "--accepted-only"]), 0)
            man3 = json.loads((out3 / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(man3["counts"]["rendered"], 0)
            self.assertEqual(man3["counts"]["skippedNotAccepted"], 1)


if __name__ == "__main__":
    unittest.main()
