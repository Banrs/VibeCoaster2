"""Run: python cli_arguments_test.py /path/to/coaster_cli"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

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

    def test_validate_rejects_generation_options_before_loading(self):
        with tempfile.TemporaryDirectory() as folder:
            for option, value in (("--seed", "99"), ("--candidates", "0"),
                                  ("--terrain", "flat"), ("--reference-file", "missing.json")):
                with self.subTest(option=option):
                    result = subprocess.run(
                        [str(Path(CLI).resolve()), "validate", str(Path(folder) / "missing.coaster"), option, value],
                        capture_output=True, text=True, encoding="utf-8", timeout=15,
                    )
                    self.assertEqual(result.returncode, 1, result.stderr)
                    self.assertIn("generate", result.stderr)
                    self.assertEqual(result.stdout, "")

    def test_only_flat_terrain_is_available(self):
        self.assertEqual(self.parsed("--terrain", "flat")["terrain"], "flat")
        with tempfile.TemporaryDirectory() as folder:
            for terrain in ("hills", "canyon"):
                with self.subTest(terrain=terrain):
                    self.invalid("--terrain", terrain, "--out", str(Path(folder) / "ride.coaster"),
                                 "--json", str(Path(folder) / "report.json"))
                    self.assertEqual(list(Path(folder).iterdir()), [])

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


if __name__ == "__main__":
    unittest.main()
