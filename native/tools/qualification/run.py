#!/usr/bin/env python3
"""CI-only native qualification; offline verification never launches a process.

The 44 original requests and eight panel requests are pinned in baseline.json.
Raw failed attempts are retained. Synthetic unit fixtures are not ride evidence.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import shutil
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from acceptance import is_finite_nonneg, strict_report_check

ROOT = Path(__file__).resolve().parents[3]
BASELINE = Path(__file__).with_name("baseline.json")
VERSION = "0.8.3-flow.1"
BASELINE_COMMIT = "d68acb66f77016aa9531cac58175e8c380e24f7b"
PANEL = ("flat5", "flat7", "flat42", "hills2", "hills9", "canyon1", "canyon24", "canyon42")
BINARY_NAMES = ("coaster_cli.exe", "coaster_convergence.exe", "coaster_matrix.exe",
                "crossover_fixture.exe", "replay_organic.exe")
REQUEST_FIELDS = {"seed", "terrain", "preset", "intensityRequired", "maxCandidates",
                  "simulationStep", "limits", "train", "targets", "terrainProfile"}
COMPILER_FLAGS = ["-std=c++20", "-O2", "-ffp-contract=off", "-Wall", "-Wextra"]
REQUIRED_SOURCE = {"native/CMakeLists.txt", "native/tools/convergence/audit.cpp", "native/tools/acceptance.py",
                   "native/tools/qualification/run.py", "native/tools/qualification/baseline.json"}
REQUIRED_SOURCE.update(
    path.relative_to(ROOT).as_posix()
    for tree in ("native/core/src", "native/core/include", "native/core/tests")
    for path in (ROOT / tree).rglob("*")
    if path.suffix in (".cpp", ".hpp", ".h")
    and not {"artifacts", "generated", "build", "fixtures"}.intersection(path.relative_to(ROOT).parts)
)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f"duplicate JSON key: {key}")
        result[key] = value
    return result


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"), object_pairs_hook=unique_object,
                      parse_constant=lambda value: require(False, f"nonfinite JSON: {value}"))


def write(path, value):
    # Exclusive creation protects prior failed evidence from reruns.
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def digest(path):
    require(Path(path).is_file() and not Path(path).is_symlink(), f"missing/linked file: {path}")
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def sha(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def load_baseline(path=None):
    baseline = read(BASELINE if path is None else path)
    require(baseline.get("format") == 1 and baseline.get("generatorVersion") == VERSION
            and baseline.get("baselineCommit") == BASELINE_COMMIT, "baseline version/commit mismatch")
    cases = baseline["cases"]
    require([case["id"] for case in cases] == [f"case-{i:02d}" for i in range(44)] + list(PANEL),
            "baseline must contain the ordered original 52 cases")
    for index, case in enumerate(cases):
        request = case["request"]
        require(set(request) == REQUEST_FIELDS and request["simulationStep"] == 1 / 960
                and sha(case["saveSha256"]), f"invalid pinned request: {case['id']}")
        if index < 44:
            require(case["kind"] == "matrix" and case["index"] == index, "matrix index mismatch")
        else:
            name = case["id"]
            terrain = name.rstrip("0123456789")
            require(case["kind"] == "panel" and case["terrain"] == terrain
                    and case["seed"] == int(name[len(terrain):])
                    and case["candidates"] == (1 if name in ("hills2", "hills9", "canyon1") else 8)
                    and request["maxCandidates"] == case["candidates"]
                    and case["crossover"] is (name == "hills2"), "panel command mismatch")
    return baseline


def shard_cases(cases, shard):
    require(type(shard) is int and 0 <= shard < 4, "shard must be 0..3")
    return cases[shard::4]


def relative_file(base, name):
    relative = PurePosixPath(name)
    require(not relative.is_absolute() and ".." not in relative.parts and "\\" not in name
            and ":" not in name, f"invalid manifest path: {name}")
    path = Path(base).joinpath(*relative.parts)
    require(path.resolve().is_relative_to(Path(base).resolve()), f"manifest path escapes: {name}")
    return path


def check_manifest(manifest, source, binaries, baseline):
    require(isinstance(manifest.get("commit"), str)
            and re.fullmatch(r"[0-9a-f]{40}", manifest["commit"]), "invalid source commit")
    compiler = manifest.get("compiler")
    require(isinstance(compiler, dict) and compiler.get("version") == "0.14.1"
            and sha(compiler.get("sha256")) and compiler.get("flags") == COMPILER_FLAGS,
            "incomplete/unpinned compiler identity")
    require(set(manifest["binaries"]) == set(BINARY_NAMES), "incomplete binary identity")
    require(REQUIRED_SOURCE <= set(manifest["source"]), "incomplete source identity")
    require(set(baseline["drivers"]) == {"matrix.cpp", "fixture.cpp", "organic.cpp"}, "incomplete frozen driver identity")
    for name, expected in baseline["drivers"].items():
        require(manifest["source"].get("native/tools/qualification/" + name) == expected,
                f"frozen qualification driver changed: {name}")
    for base, hashes in ((source, manifest["source"]), (binaries, manifest["binaries"])):
        for name, expected in hashes.items():
            require(sha(expected) and digest(relative_file(base, name)) == expected,
                    f"source/binary hash mismatch: {name}")
    require(read(Path(source) / "native/tools/qualification/baseline.json") == baseline,
            "compiled source baseline identity mismatch")


def commands(case, binaries, folder):
    """Keep the original driver argv, working directories and timeouts."""
    cli, audit = str(binaries / "coaster_cli.exe"), str(binaries / "coaster_convergence.exe")
    if case["kind"] == "matrix":
        return [("generate", [str(binaries / "coaster_matrix.exe"), str(case["index"]), str(folder)], folder.parent, 600),
                ("replay", [cli, "validate", str(folder / "ride.coaster"), "--json", str(folder / "replay.json")], folder.parent, 120),
                ("audit", [audit, str(folder / "save-list.txt"), str(folder / "audit.jsonl")], folder.parent, 120)]
    generate = [cli, "generate", "--seed", str(case["seed"]), "--terrain", case["terrain"],
                "--preset", "physics-proof", "--candidates", str(case["candidates"]),
                "--json", "report.json", "--plan", "plan.json", "--trace", "trace.json", "--out", "ride.coaster"]
    organic = [str(binaries / "replay_organic.exe"), "ride.coaster", "plan.json"]
    if case["crossover"]:
        generate = [str(binaries / "crossover_fixture.exe"), str(case["seed"])]
        organic.append("require-crossing")
    return [("generate", generate, folder, 600),
            ("replay", [cli, "validate", "ride.coaster", "--json", "replay.json"], folder, 600),
            ("audit", [audit, "save-list.txt", "audit.jsonl"], folder, 600),
            ("organic", organic, folder, 600)]


def physical(report):
    return {key: value for key, value in report.items() if key != "generationSeconds"}


def check_ride(folder, case, record):
    report, replay = read(folder / "report.json"), read(folder / "replay.json")
    request = case["request"]
    contract = {**request, "step": request["simulationStep"]}
    for value in (report, replay):
        kind, _, reason = strict_report_check(value, contract)
        require(kind == "accepted-candidate", f"strict report: {reason}")
        require({key: value.get(key) for key in REQUEST_FIELDS} == request, "changed original request")
        require(value.get("generatorVersion") == value.get("runtimeVersion") == VERSION and type(value.get("candidate")) is int
                and value["candidate"] == 0, "runtime/candidate mismatch")
        require(value["convergence"]["coarseStep"] == 1 / 960
                and value["convergence"]["fineStep"] == 1 / 1920
                and len(value["convergence"]["metrics"]) == 160, "960/1920 160-metric contract")
    require(physical(report) == physical(replay), "exact replay mismatch")
    save = folder / "ride.coaster"
    require(save.read_bytes().startswith(b"COASTER 5 "), "exact-version save format mismatch")
    require(digest(save) == case["saveSha256"] == record["saveSha256"], "source64 exact-save parity mismatch")
    save_list = (folder / "save-list.txt").read_text(encoding="utf-8").splitlines()
    require(save_list == [record["savePath"]], "audit save-list binding mismatch")
    lines = (folder / "audit.jsonl").read_text(encoding="utf-8").splitlines()
    require(len(lines) == 1, "audit must contain exactly one ride")
    audit = json.loads(lines[0], object_pairs_hook=unique_object)
    require(all(audit.get(key) is True for key in ("passed", "loaded", "fineCompleted", "fineSimulationValid"))
            and audit.get("fineTargetErrors") == [] and audit.get("runtime") == VERSION
            and audit.get("step") == 1 / 960, "independent audit failed")
    require(audit.get("source") == record["savePath"], "audit source binding mismatch")
    require(audit.get("seed") == request["seed"] and audit.get("terrain") == request["terrain"], "audit request mismatch")
    require(physical(audit["coarseReport"]) == physical(replay), "audit coarse report binding mismatch")
    expected = {row["name"]: row for row in replay["convergence"]["metrics"]}
    require(set(audit["metrics"]) == set(expected), "audit metric coverage mismatch")
    for name, metric in audit["metrics"].items():
        row = expected[name]
        require(metric.get("passed") is True and metric.get("coarse") == row["coarse"]
                and metric.get("fine") == row["fine"], f"audit metric binding: {name}")
        fraction = .01 if name == "maxSpeed" else .02
        error = abs(row["coarse"] - row["fine"]) / max(1., abs(row["fine"]))
        require(metric.get("normalizationFloor") == 1 and is_finite_nonneg(metric.get("normalizedError"))
                and math.isclose(metric["normalizedError"], error, rel_tol=1e-9, abs_tol=1e-9)
                and is_finite_nonneg(metric.get("limit"))
                and math.isclose(metric["limit"], fraction, rel_tol=1e-9, abs_tol=1e-9)
                and error < fraction, f"audit metric arithmetic: {name}")
    plan = read(folder / "plan.json")
    energy = plan["authoringEnergy"]
    require(energy.get("converged") is True and type(energy.get("corrections")) is int
            and 0 <= energy["corrections"] <= 8 and energy.get("toleranceMps") == .5
            and is_finite_nonneg(energy.get("maximumSpeedResidualMps"))
            and energy["maximumSpeedResidualMps"] <= .5, "authoring energy contract")
    if case["kind"] == "panel":
        require(plan.get("requestedCrossover") is case["crossover"], "requested crossover mismatch")
        require(re.search(r"^PASS \d+ independent saved-ride organic checks$",
                          (folder / "organic.log").read_text(encoding="utf-8"), re.MULTILINE), "organic checks missing")
    return {"saveSha256": digest(save), "corrections": energy["corrections"],
            "maximumSpeedResidualMps": energy["maximumSpeedResidualMps"]}


def original_path(value):
    return PureWindowsPath(value) if PureWindowsPath(value).drive else PurePosixPath(value)


def check_case(folder, case, record, shard):
    require(record.get("id") == case["id"] and record.get("kind") == case["kind"], "case identity mismatch")
    original_folder = original_path(shard["outputPath"]) / case["id"]
    expected = commands(case, original_path(shard["binaryPath"]), original_folder)
    receipts = record["commands"]
    require([r["stage"] for r in receipts] == [c[0] for c in expected], "missing/duplicate command receipts")
    for receipt, (stage, argv, cwd, timeout) in zip(receipts, expected):
        require(receipt["argv"] == argv and receipt["cwd"] == str(cwd)
                and receipt["timeoutSeconds"] == timeout, f"changed command: {stage}")
        require(type(receipt.get("exit")) is int and receipt["exit"] == 0
                and receipt.get("timedOut") is False and is_finite_nonneg(receipt.get("seconds")),
                f"failed command: {stage}")
    required = {"report.json", "replay.json", "ride.coaster", "save-list.txt", "audit.jsonl", "plan.json"}
    required.update(stage + ".log" for stage, *_ in expected)
    if case["kind"] == "panel":
        required.add("trace.json")
    files = {p.name for p in folder.iterdir() if p.is_file() and p.name != "result.json"}
    require(required <= files and set(record["files"]) == files, "missing/unbound raw evidence")
    for name, expected_hash in record["files"].items():
        require(sha(expected_hash) and digest(relative_file(folder, name)) == expected_hash, f"raw evidence hash mismatch: {name}")
    require(record["savePath"] == str(original_folder / "ride.coaster"), "recorded save path mismatch")
    result = check_ride(folder, case, record)
    result["seconds"] = {r["stage"]: r["seconds"] for r in receipts}
    return result


def run_case(case, output, binaries, shard):
    folder = output / case["id"]
    record = {"id": case["id"], "kind": case["kind"], "commands": [], "failures": [],
              "savePath": str(folder / "ride.coaster")}
    if case["kind"] == "panel":
        folder.mkdir()
    try:
        for stage, argv, cwd, timeout in commands(case, binaries, folder):
            if stage != "generate" and not (folder / "ride.coaster").is_file():
                break
            if stage == "audit":
                (folder / "save-list.txt").write_text(record["savePath"] + "\n", encoding="utf-8")
            # The unchanged matrix driver itself creates its case directory.
            log_path = output / (case["id"] + "-generate.log") if stage == "generate" else folder / (stage + ".log")
            receipt = {"stage": stage, "argv": argv, "cwd": str(cwd), "timeoutSeconds": timeout,
                       "exit": None, "timedOut": False}
            started = time.monotonic()
            try:
                with log_path.open("x", encoding="utf-8") as log:
                    receipt["exit"] = subprocess.run(argv, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                                                     timeout=timeout).returncode
            except subprocess.TimeoutExpired:
                receipt["timedOut"] = True
            finally:
                receipt["seconds"] = time.monotonic() - started
                record["commands"].append(receipt)
                folder.mkdir(exist_ok=True)
                if stage == "generate" and log_path.exists():
                    log_path.rename(folder / "generate.log")
        record["saveSha256"] = digest(folder / "ride.coaster") if (folder / "ride.coaster").is_file() else None
    except (OSError, ValueError) as error:
        record["failures"].append(str(error))
    folder.mkdir(exist_ok=True)
    record["files"] = {p.name: digest(p) for p in sorted(folder.iterdir()) if p.is_file()}
    try:
        check_case(folder, case, record, shard)
    except (OSError, ValueError, KeyError, TypeError) as error:
        record["failures"].append(str(error))
    record["passed"] = not record["failures"]
    write(folder / "result.json", record)
    print(json.dumps({"id": case["id"], "passed": record["passed"], "failures": record["failures"]}), flush=True)
    return record


def run_shard(bin_dir, output, shard_index):
    baseline = load_baseline()
    assigned = shard_cases(baseline["cases"], shard_index)
    bin_dir, output = bin_dir.resolve(), output.resolve()
    manifest = read(bin_dir / "manifest.json")
    check_manifest(manifest, ROOT, bin_dir, baseline)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, capture_output=True, text=True).stdout.strip()
    require(manifest["commit"] == commit, "build commit differs from current checkout")
    output.mkdir(parents=True, exist_ok=False)
    write(output / "manifest.json", manifest)
    write(output / "baseline.json", baseline)
    for base, hashes, target in ((ROOT, manifest["source"], output / "source"),
                                 (bin_dir, manifest["binaries"], output / "bin")):
        for name in hashes:
            destination = relative_file(target, name)
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(relative_file(base, name), destination)
    binaries = output / "bin"
    check_manifest(manifest, output / "source", binaries, baseline)
    shard = {"format": 1, "shard": shard_index, "workers": 2, "caseIds": [c["id"] for c in assigned],
             "manifestSha256": digest(output / "manifest.json"), "baselineSha256": digest(output / "baseline.json"),
             "outputPath": str(output), "binaryPath": str(binaries)}
    write(output / "shard.json", shard)
    started = time.monotonic()
    with ThreadPoolExecutor(max_workers=2) as pool:
        records = list(pool.map(lambda case: run_case(case, output, binaries, shard), assigned))
    # Also reject source or executable changes during qualification.
    check_manifest(manifest, ROOT, bin_dir, baseline)
    check_manifest(manifest, output / "source", binaries, baseline)
    summary = {"shard": shard_index, "seconds": time.monotonic() - started,
               "passed": all(r["passed"] for r in records), "caseIds": [r["id"] for r in records]}
    write(output / "shard-result.json", summary)
    return 0 if summary["passed"] else 1


def verify(input_dir):
    """Read retained evidence only; original CI paths need not exist here."""
    baseline = load_baseline()
    summary = {"format": 1, "passed": False, "expectedCases": 52, "cases": {}, "shards": [], "failures": []}
    identity = None
    seen_shards, seen_cases = set(), set()
    shard_files = sorted(input_dir.rglob("shard.json"))
    if len(shard_files) != 4:
        summary["failures"].append(f"expected four shard directories, found {len(shard_files)}")
    for shard_file in shard_files:
        folder = shard_file.parent
        try:
            shard = read(shard_file)
            index = shard["shard"]
            assigned = shard_cases(baseline["cases"], index)
            require(index not in seen_shards, "duplicate shard ID")
            seen_shards.add(index)
            require(shard.get("format") == 1 and shard.get("workers") == 2
                    and shard["caseIds"] == [c["id"] for c in assigned], "assigned shard coverage mismatch")
            require(digest(folder / "manifest.json") == shard["manifestSha256"]
                    and digest(folder / "baseline.json") == shard["baselineSha256"], "shard identity hash mismatch")
            require(load_baseline(folder / "baseline.json") == baseline, "baseline identity mismatch")
            manifest = read(folder / "manifest.json")
            check_manifest(manifest, folder / "source", folder / "bin", baseline)
            require(identity is None or identity == manifest, "mixed source/compiler/binary identity")
            identity = manifest
            require({p.name for p in folder.iterdir() if p.is_dir()} == {"source", "bin", *shard["caseIds"]},
                    "missing/unexpected case directory")
            receipt = read(folder / "shard-result.json")
            require(receipt.get("shard") == index and receipt.get("caseIds") == shard["caseIds"]
                    and is_finite_nonneg(receipt.get("seconds")), "incomplete shard receipt")
            summary["shards"].append(receipt)
            if receipt.get("passed") is not True:
                summary["failures"].append(f"{folder.name}: failed shard receipt")
            for case in assigned:
                case_id = case["id"]
                try:
                    record = read(folder / case_id / "result.json")
                    require(record.get("id") not in seen_cases, "duplicate case ID")
                    seen_cases.add(record.get("id"))
                    result = check_case(folder / case_id, case, record, shard)
                    require(record.get("passed") is True and record.get("failures") == [], "failed case receipt")
                    summary["cases"][case_id] = result
                except (OSError, ValueError, KeyError, TypeError) as error:
                    summary["failures"].append(f"{case_id}: {error}")
        except (OSError, ValueError, KeyError, TypeError) as error:
            summary["failures"].append(f"{folder.name}: {error}")
    expected_ids = {case["id"] for case in baseline["cases"]}
    if seen_shards != set(range(4)) or set(summary["cases"]) != expected_ids:
        summary["failures"].append("incomplete verified 52-case/four-shard coverage")
    summary["identity"] = identity
    summary["passed"] = not summary["failures"]
    summary["caseSeconds"] = sum(sum(case["seconds"].values()) for case in summary["cases"].values())
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    run = sub.add_parser("run")
    run.add_argument("--bin-dir", type=Path, required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--shard", type=int, choices=range(4), required=True)
    check = sub.add_parser("verify")
    check.add_argument("--input", type=Path, required=True)
    args = parser.parse_args()
    if args.action == "run":
        return run_shard(args.bin_dir, args.output, args.shard)
    summary = verify(args.input)
    write(args.input / "qualification-summary.json", summary)
    print(json.dumps({key: summary[key] for key in ("passed", "expectedCases", "failures", "caseSeconds")}, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
