"""Pure-Python qualification protocol tests. All rides/processes are synthetic.

These test fail-closed evidence handling, never coaster physics or native code.
"""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
from qualification import run as q
from test_acceptance import strict_obj


SAVE = b"COASTER 5 synthetic protocol fixture, not a native ride\n"
SAVE_HASH = hashlib.sha256(SAVE).hexdigest()
SYNTHETIC_SOURCE = b"Synthetic source identity fixture\n"


def save_json(path, value):
    path.write_text(json.dumps(value), encoding="utf-8")


def synthetic_baseline():
    request = {"seed": 1, "terrain": "flat", "preset": "physics-proof", "intensityRequired": False,
               "maxCandidates": 1, "simulationStep": 1 / 960, "limits": {}, "train": {},
               "targets": {}, "terrainProfile": {}}
    cases = [{"id": f"case-{index:02d}", "kind": "matrix", "index": index,
              "request": copy.deepcopy(request), "saveSha256": SAVE_HASH} for index in range(44)]
    for name in q.PANEL:
        terrain = name.rstrip("0123456789")
        seed = int(name[len(terrain):])
        candidates = 1 if name in ("hills2", "hills9", "canyon1") else 8
        panel_request = {**copy.deepcopy(request), "seed": seed, "terrain": terrain,
                         "maxCandidates": 1 if name == "hills2" else candidates}
        cases.append({"id": name, "kind": "panel", "seed": seed, "terrain": terrain,
                      "candidates": candidates, "crossover": name == "hills2",
                      "request": panel_request, "saveSha256": SAVE_HASH})
    return {"format": 1, "generatorVersion": q.VERSION, "baselineCommit": q.BASELINE_COMMIT, "cases": cases,
            "drivers": {name: hashlib.sha256(SYNTHETIC_SOURCE).hexdigest()
                        for name in ("matrix.cpp", "fixture.cpp", "organic.cpp")}}


def synthetic_ride(folder, case, save_path):
    folder.mkdir(exist_ok=True)
    report = strict_obj(seed=case["request"]["seed"], terrain=case["request"]["terrain"])
    report.update(copy.deepcopy(case["request"]), runtimeVersion=q.VERSION)
    save_json(folder / "report.json", report)
    save_json(folder / "replay.json", {**report, "generationSeconds": 99})
    (folder / "ride.coaster").write_bytes(SAVE)
    (folder / "save-list.txt").write_text(save_path + "\n", encoding="utf-8")
    audit = {"passed": True, "loaded": True, "fineCompleted": True, "fineSimulationValid": True,
             "fineTargetErrors": [], "runtime": q.VERSION, "step": 1 / 960, "source": save_path,
             "seed": report["seed"], "terrain": report["terrain"], "coarseReport": report,
             "metrics": {row["name"]: {"coarse": row["coarse"], "fine": row["fine"], "normalizedError": 0.,
                                      "normalizationFloor": 1, "limit": .01 if row["name"] == "maxSpeed" else .02,
                                      "passed": True} for row in report["convergence"]["metrics"]}}
    save_json(folder / "audit.jsonl", audit)
    save_json(folder / "plan.json", {"authoringEnergy": {"converged": True, "corrections": 8,
              "maximumSpeedResidualMps": .5, "toleranceMps": .5}, "requestedCrossover": case.get("crossover", False)})
    for stage in ("generate", "replay", "audit"):
        (folder / (stage + ".log")).write_text("Synthetic process-double log\n", encoding="utf-8")
    if case["kind"] == "panel":
        save_json(folder / "trace.json", {"synthetic": True})
        (folder / "organic.log").write_text("PASS 123 independent saved-ride organic checks\n", encoding="utf-8")


class QualificationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.baseline = synthetic_baseline()
        self.baseline_path = self.root / "synthetic-baseline.json"
        save_json(self.baseline_path, self.baseline)
        self.baseline_patch = mock.patch.object(q, "BASELINE", self.baseline_path)
        self.baseline_patch.start()
        self.addCleanup(self.baseline_patch.stop)
        # A small labelled corpus exercises the same inventory contract without
        # manufacturing hundreds of copies of the real source for each test.
        self.source_patch = mock.patch.object(q, "REQUIRED_SOURCE", {
            "native/core/include/coaster/coaster.hpp", "native/core/src/generation.cpp",
            "native/CMakeLists.txt", "native/tools/convergence/audit.cpp", "native/tools/acceptance.py",
            "native/tools/qualification/run.py", "native/tools/qualification/baseline.json"})
        self.source_patch.start()
        self.addCleanup(self.source_patch.stop)
        self.artifacts = self.root / "downloaded"
        self.artifacts.mkdir()

    def artifacts_fixture(self):
        """Complete synthetic offline evidence with intentionally nonexistent CI paths."""
        for index in range(4):
            folder = self.artifacts / f"shard-{index}"
            folder.mkdir()
            sources = folder / "source"
            binaries = folder / "bin"
            binaries.mkdir()
            source_names = sorted(q.REQUIRED_SOURCE | {"native/tools/qualification/" + name for name in self.baseline["drivers"]})
            for name in source_names:
                path = sources / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(SYNTHETIC_SOURCE)
            save_json(sources / "native/tools/qualification/baseline.json", self.baseline)
            for name in q.BINARY_NAMES:
                (binaries / name).write_bytes(b"Synthetic binary identity fixture; never executable\n")
            manifest = {"commit": "a" * 40, "compiler": {"synthetic": "protocol-test-only", "version": "0.14.1",
                        "sha256": hashlib.sha256(b"Synthetic compiler fixture").hexdigest(), "flags": q.COMPILER_FLAGS},
                        "source": {name: q.digest(sources / name) for name in source_names},
                        "binaries": {name: q.digest(binaries / name) for name in q.BINARY_NAMES}}
            save_json(folder / "manifest.json", manifest)
            save_json(folder / "baseline.json", self.baseline)
            cases = q.shard_cases(self.baseline["cases"], index)
            original = q.original_path(f"Z:\\nonexistent-ci\\shard-{index}")
            shard = {"format": 1, "shard": index, "workers": 2, "caseIds": [case["id"] for case in cases],
                     "manifestSha256": q.digest(folder / "manifest.json"),
                     "baselineSha256": q.digest(folder / "baseline.json"),
                     "outputPath": str(original), "binaryPath": str(original / "bin")}
            save_json(folder / "shard.json", shard)
            save_json(folder / "shard-result.json", {"shard": index, "seconds": 25., "passed": True, "caseIds": shard["caseIds"]})
            for case in cases:
                case_folder = folder / case["id"]
                save_path = str(original / case["id"] / "ride.coaster")
                synthetic_ride(case_folder, case, save_path)
                record = {"id": case["id"], "kind": case["kind"], "savePath": save_path,
                          "saveSha256": SAVE_HASH, "passed": True, "failures": [],
                          "commands": [{"stage": stage, "argv": argv, "cwd": str(cwd), "timeoutSeconds": timeout,
                                        "exit": 0, "timedOut": False, "seconds": 1.5}
                                       for stage, argv, cwd, timeout in q.commands(case, original / "bin", original / case["id"])],
                          "files": {p.name: q.digest(p) for p in case_folder.iterdir()}}
                save_json(case_folder / "result.json", record)

    def rewrite_raw(self, name, transform):
        folder = self.artifacts / "shard-0" / "case-00"
        value = q.read(folder / name)
        transform(value)
        save_json(folder / name, value)
        record = q.read(folder / "result.json")
        record["files"][name] = q.digest(folder / name)
        save_json(folder / "result.json", record)

    def assert_failed(self, text):
        summary = q.verify(self.artifacts)
        self.assertFalse(summary["passed"])
        self.assertTrue(any(text in reason for reason in summary["failures"]), summary["failures"])

    def test_fixed_four_shards_cover_every_original_request_once(self):
        groups = [q.shard_cases(self.baseline["cases"], index) for index in range(4)]
        self.assertEqual([len(group) for group in groups], [13] * 4)
        self.assertEqual({case["id"] for group in groups for case in group}, {case["id"] for case in self.baseline["cases"]})
        self.assertEqual(sum(len(group) for group in groups), 52)
        for index, group in enumerate(groups):
            self.assertEqual(group, [case for position, case in enumerate(self.baseline["cases"]) if position % 4 == index])
        for invalid in (-1, 4, True):
            with self.assertRaises(ValueError):
                q.shard_cases(self.baseline["cases"], invalid)

    def test_original_commands_and_timeouts_include_private_crossover(self):
        cases = {case["id"]: case for case in self.baseline["cases"]}
        binaries, folder = Path("bin"), Path("output") / "case"
        self.assertEqual([c[3] for c in q.commands(cases["case-00"], binaries, folder)], [600, 120, 120])
        self.assertEqual(q.commands(cases["hills2"], binaries, folder)[0][1], [str(binaries / "crossover_fixture.exe"), "2"])
        self.assertEqual(q.commands(cases["hills2"], binaries, folder)[-1][1][-1], "require-crossing")
        for name in q.PANEL:
            command = q.commands(cases[name], binaries, folder)
            self.assertEqual([c[3] for c in command], [600] * 4)
            if name != "hills2":
                argv = command[0][1]
                self.assertEqual(argv[argv.index("--candidates") + 1], "1" if name in ("hills9", "canyon1") else "8")

    def test_complete_offline_evidence_never_runs_native_processes(self):
        self.artifacts_fixture()
        with mock.patch.object(q.subprocess, "run", side_effect=AssertionError("offline verify launched a process")):
            summary = q.verify(self.artifacts)
        self.assertTrue(summary["passed"], summary["failures"])
        self.assertEqual(len(summary["cases"]), 52)
        self.assertEqual(len(summary["shards"]), 4)
        self.assertEqual(summary["caseSeconds"], (44 * 3 + 8 * 4) * 1.5)

    def test_missing_and_duplicate_shards_are_rejected(self):
        self.artifacts_fixture()
        folder = self.artifacts / "shard-3"
        (folder / "shard.json").rename(folder / "absent-shard.json")
        self.assert_failed("expected four shard")
        (folder / "absent-shard.json").rename(folder / "shard.json")
        shard = q.read(folder / "shard.json")
        shard["shard"] = 0
        save_json(folder / "shard.json", shard)
        self.assert_failed("duplicate shard ID")

    def test_missing_and_duplicate_case_receipts_are_rejected(self):
        self.artifacts_fixture()
        folder = self.artifacts / "shard-0" / "case-04"
        result = q.read(folder / "result.json")
        (folder / "result.json").unlink()
        self.assert_failed("case-04")
        result["id"] = "case-00"
        save_json(folder / "result.json", result)
        self.assert_failed("duplicate case ID")

    def test_mixed_binary_source_and_compiler_identity_are_rejected(self):
        self.artifacts_fixture()
        folder = self.artifacts / "shard-3"
        original = q.read(folder / "manifest.json")
        for group, name in (("binaries", "coaster_cli.exe"), ("source", "native/tools/acceptance.py"), ("compiler", "synthetic")):
            with self.subTest(group=group):
                manifest = copy.deepcopy(original)
                changed = None
                if group == "compiler":
                    manifest[group][name] = "different compiler"
                else:
                    changed = folder / ("bin" if group == "binaries" else "source") / name
                    old_bytes = changed.read_bytes()
                    changed.write_bytes(b"Different synthetic build identity")
                    manifest[group][name] = q.digest(changed)
                save_json(folder / "manifest.json", manifest)
                shard = q.read(folder / "shard.json")
                shard["manifestSha256"] = q.digest(folder / "manifest.json")
                save_json(folder / "shard.json", shard)
                self.assert_failed("mixed source/compiler/binary")
                if changed:
                    changed.write_bytes(old_bytes)

    def test_raw_hash_and_exact_replay_cannot_be_replaced_by_success_receipt(self):
        self.artifacts_fixture()
        replay = self.artifacts / "shard-0" / "case-00" / "replay.json"
        value = q.read(replay)
        value["topology"] = "different synthetic replay"
        save_json(replay, value)
        self.assert_failed("raw evidence hash mismatch")
        self.rewrite_raw("replay.json", lambda value: None)
        self.assert_failed("exact replay mismatch")

    def test_all_shards_repeating_incomplete_source_or_compiler_still_fail(self):
        self.artifacts_fixture()
        originals = {folder: q.read(folder / "manifest.json") for folder in self.artifacts.iterdir()}
        for change, reason in (("source", "incomplete source identity"), ("compiler", "unpinned compiler identity"),
                               ("driver", "frozen qualification driver changed")):
            with self.subTest(change=change):
                for folder, original in originals.items():
                    manifest = copy.deepcopy(original)
                    if change == "source":
                        del manifest["source"]["native/core/src/generation.cpp"]
                    elif change == "compiler":
                        manifest["compiler"]["version"] = "different compiler"
                    else:
                        del manifest["source"]["native/tools/qualification/fixture.cpp"]
                    save_json(folder / "manifest.json", manifest)
                    shard = q.read(folder / "shard.json")
                    shard["manifestSha256"] = q.digest(folder / "manifest.json")
                    save_json(folder / "shard.json", shard)
                self.assert_failed(reason)

    def test_audit_source_and_metric_binding_are_verified(self):
        self.artifacts_fixture()
        self.rewrite_raw("audit.jsonl", lambda value: value.update(source="Z:\\wrong-save.coaster"))
        self.assert_failed("audit source binding mismatch")
        save_path = q.read(self.artifacts / "shard-0" / "case-00" / "result.json")["savePath"]
        self.rewrite_raw("audit.jsonl", lambda value: value.update(source=save_path))
        self.rewrite_raw("audit.jsonl", lambda value: value["metrics"]["maxSpeed"].update(fine=.001))
        self.assert_failed("audit metric binding")

    def test_source64_exact_save_and_authoring_energy_are_required(self):
        self.artifacts_fixture()
        folder = self.artifacts / "shard-0" / "case-00"
        (folder / "ride.coaster").write_bytes(SAVE + b"different save")
        record = q.read(folder / "result.json")
        record["saveSha256"] = record["files"]["ride.coaster"] = q.digest(folder / "ride.coaster")
        save_json(folder / "result.json", record)
        self.assert_failed("source64 exact-save parity")
        (folder / "ride.coaster").write_bytes(SAVE)
        record["saveSha256"] = record["files"]["ride.coaster"] = SAVE_HASH
        save_json(folder / "result.json", record)
        self.rewrite_raw("plan.json", lambda value: value["authoringEnergy"].update(corrections=9))
        self.assert_failed("authoring energy contract")

    def test_failed_shard_still_verifies_raw_case_failures(self):
        self.artifacts_fixture()
        folder = self.artifacts / "shard-0"
        receipt = q.read(folder / "shard-result.json")
        receipt["passed"] = False
        save_json(folder / "shard-result.json", receipt)
        self.rewrite_raw("plan.json", lambda value: value["authoringEnergy"].update(maximumSpeedResidualMps=.51))
        self.assert_failed("failed shard receipt")
        self.assert_failed("authoring energy contract")

    def test_timeout_retains_failed_attempt_without_retry_or_replay(self):
        case = self.baseline["cases"][0]
        binaries = self.root / "synthetic-bin"
        shard = {"outputPath": str(self.artifacts), "binaryPath": str(binaries)}
        with mock.patch.object(q.subprocess, "run", side_effect=subprocess.TimeoutExpired("synthetic-process", 600)) as process:
            result = q.run_case(case, self.artifacts, binaries, shard)
        self.assertEqual(process.call_count, 1)
        self.assertFalse(result["passed"])
        self.assertTrue(result["commands"][0]["timedOut"])
        self.assertIsNone(result["commands"][0]["exit"])
        self.assertTrue((self.artifacts / "case-00" / "generate.log").is_file())
        self.assertTrue((self.artifacts / "case-00" / "result.json").is_file())


if __name__ == "__main__":
    unittest.main()
