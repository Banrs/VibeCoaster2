"""Real CLI argument and generated phase-plan protocol regressions.

Run: python test_cli_arguments.py /path/to/coaster_cli
Discovery runs can set COASTER_CLI; otherwise these executable tests are skipped.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import acceptance

CLI = os.environ.get("COASTER_CLI")
if __name__ == "__main__" and len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
    CLI = sys.argv.pop(1)


@unittest.skipUnless(CLI, "Pass a built coaster_cli executable or set COASTER_CLI")
class CliArguments(unittest.TestCase):
    def invoke(self, *args):
        # Either invalid setting stops generation even when testing the other.
        return subprocess.run(
            [str(Path(CLI).resolve()), "generate", "--candidates", "0",
             "--launch-seconds", "0", *args],
            capture_output=True, text=True, encoding="utf-8", timeout=15,
        )

    def invalid(self, *args):
        result = self.invoke(*args)
        self.assertEqual(result.returncode, 1, (args, result.stdout, result.stderr))
        self.assertEqual(result.stdout, "")
        self.assertTrue(result.stderr.strip())

    def parsed(self, *args):
        result = self.invoke(*args)
        self.assertEqual(result.returncode, 2, result.stderr)
        report = json.loads(result.stdout)
        self.assertFalse(report["accepted"])
        self.assertIn("REQUEST_RANGE", [item["code"] for item in report["errors"]])
        return report

    def test_seed_requires_unsigned_decimal_digits(self):
        for text in ("-1", "+1", "", " 42", "42 ", "42junk", "0x2a", "18446744073709551616"):
            with self.subTest(text=text):
                self.invalid("--seed", text)

    def test_seed_unsigned_boundaries(self):
        for text in ("0", "42", "00042", "18446744073709551615"):
            with self.subTest(text=text):
                self.assertEqual(self.parsed("--seed", text)["seed"], int(text))

    def test_numeric_suffixes_are_not_ignored(self):
        for option in ("--candidates", "--step", "--launch-seconds",
                       "--reference-exposure", "--lateral-rate-limit", "--longitudinal-rate-limit"):
            for text in ("1junk", "1.0junk", "1e2junk"):
                with self.subTest(option=option, text=text):
                    self.invalid(option, text)

    def test_numeric_overflow_is_usage_error(self):
        self.invalid("--candidates", "2147483648")
        self.invalid("--candidates", "-2147483649")
        for option in ("--step", "--launch-seconds", "--reference-exposure",
                       "--lateral-rate-limit", "--longitudinal-rate-limit"):
            with self.subTest(option=option):
                self.invalid(option, "1e10000")

    def test_valid_integer_and_exponent_syntax_is_preserved(self):
        for text in ("0", "1", "+1", "2147483647", "-2147483648"):
            with self.subTest(text=text):
                self.assertEqual(self.parsed("--candidates", text)["maxCandidates"], int(text))
        for text in ("0.001", "1e-3", "+1.0e-3", " 0.001"):
            with self.subTest(text=text):
                self.assertEqual(self.parsed("--step", text)["simulationStep"], 0.001)

    def test_other_valid_real_options_reach_validation(self):
        self.assertEqual(self.parsed("--launch-seconds", "1.4e0")["targets"]["launchSeconds"], 1.4)
        reference = self.parsed("--reference-exposure", "3.7e1", "--reference-id", "SYNTHETIC_ARGUMENT_TEST_ONLY")
        self.assertEqual(reference["targets"]["referenceExposure"], 37)
        for option, key in (("--lateral-rate-limit", "maxLateralRateGps"),
                            ("--longitudinal-rate-limit", "maxLongitudinalRateGps")):
            with self.subTest(option=option):
                self.assertEqual(self.parsed(option, "1e3")["limits"][key], 1000)

    def test_generated_phase_plan_is_valid_json(self):
        cli = Path(CLI).resolve()
        with tempfile.TemporaryDirectory(dir=cli.parent) as directory:
            plan_path = Path(directory) / "plan.json"
            result = subprocess.run(
                [str(cli), "generate", "--seed", "42", "--terrain", "flat",
                 "--preset", "physics-proof", "--candidates", "1", "--plan", str(plan_path)],
                cwd=directory, capture_output=True, text=True, encoding="utf-8", timeout=180,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            plan = json.loads(plan_path.read_text(encoding="utf-8"))
            phases = plan["airtimePhaseIntent"]
            self.assertTrue({"entry", "apex", "valley", "exit"}.issubset({p["phase"] for p in phases}))
            for phase in phases:
                self.assertLessEqual(abs(phase["speedResidualMps"]), .500001)
                self.assertEqual(len(phase["ridersAtPhase"]), 3)
                for rider in phase["ridersAtPhase"]:
                    self.assertEqual(len(rider["Gzyx"]), 3)
                    self.assertTrue(all(math.isfinite(v) for v in [rider["time"], rider["speedMps"], *rider["Gzyx"]]))

    def test_acceptance_tool_matches_current_cli_and_saved_replay(self):
        cli = Path(CLI).resolve()
        with tempfile.TemporaryDirectory(dir=cli.parent) as directory:
            for preset, expected in (("physics-proof", "accepted"), ("all-records", "rejected")):
                case = {"index": 0, "id": preset, "seed": 42, "terrain": "flat",
                        "preset": preset, "candidates": 1, "step": 1/960}
                record = acceptance.run_one_case(str(cli), case, directory, timeout=180)
                self.assertEqual(record["category"], expected, record)
                if expected == "accepted":
                    self.assertEqual(record["note"], "validate-confirmed")
                    self.assertEqual(record["replayReturncode"], 0)
                    report = json.loads((Path(directory) / "replays" / (preset + ".replay.json")).read_text())
                    self.assertTrue(any(".forceEnvelope." in r["name"] for r in report["convergence"]["metrics"]))
                else:
                    self.assertIn("REFERENCE_UNAVAILABLE", record["errorCodes"])


if __name__ == "__main__":
    unittest.main()
