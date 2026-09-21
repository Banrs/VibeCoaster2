"""Compact deterministic acceptance corpus; keeps failures and full evidence.

Run with Python 3: redesign_corpus.py COASTER_CLI OUTPUT_DIRECTORY
The output directory must be fresh. This is a prototype physics check, not a
certification of vehicle, restraint, launch, aerodynamic or structural design.
The native organic-generation test supplies the matched 20-percent lower-drag
simulation because the CLI intentionally exposes ride style and recipe inputs,
not an unreviewed vehicle-dynamics override.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

CASES = [
    ("baseline-42", ["--seed", "42", "--terrain", "highlands", "--trims", "auto"]),
    ("seed-77", ["--seed", "77", "--terrain", "highlands", "--trims", "auto"]),
    ("trims-full-42", ["--seed", "42", "--terrain", "flat", "--trims", "auto"]),
    ("trims-off-42", ["--seed", "42", "--terrain", "flat", "--trims", "off"]),
    ("seed-5", ["--seed", "5", "--terrain", "highlands", "--return-style", "airtime"]),
    ("flowing-7", ["--seed", "7", "--terrain", "flat", "--return-style", "flowing",
                   "--airtime", ".85", "--signature-roll", "35"]),
    ("higher-314", ["--seed", "314", "--terrain", "highlands", "--speed-kmh", "310",
                    "--airtime", "1.1", "--signature-roll", "55"]),
    ("lower-42", ["--seed", "42", "--terrain", "flat", "--speed-kmh", "290",
                  "--height", "210", "--airtime", "1.15"]),
]


def main():
    cli, output = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
    output.mkdir(parents=True, exist_ok=False)
    summary = {"utc": datetime.now(timezone.utc).isoformat(), "executable": str(cli),
               "executableSha256": hashlib.sha256(cli.read_bytes()).hexdigest(), "cases": []}
    for name, parameters in CASES:
        base = output / name
        args = [str(cli), "generate", "--preset", "physics-proof", *parameters,
                "--json", str(base.with_suffix(".json")), "--trace", str(base) + "-trace.json",
                "--plan", str(base) + "-plan.json"]
        if name in ("baseline-42", "trims-full-42", "flowing-7"):
            args += ["--out", str(base.with_suffix(".coaster"))]
        with base.with_suffix(".stdout.log").open("w", encoding="utf-8") as stdout, base.with_suffix(".stderr.log").open("w", encoding="utf-8") as stderr:
            run = subprocess.run(args, stdout=stdout, stderr=stderr, timeout=300)
        report = json.loads(base.with_suffix(".json").read_text(encoding="utf-8"))
        row = {"name": name, "arguments": parameters, "exitCode": run.returncode,
               "accepted": report["accepted"] and run.returncode == 0,
               "candidate": report["candidate"], "topology": report["topology"],
               "lengthMeters": report["lengthMeters"], "metrics": report["metrics"],
               "seatStatistics": report["seatStatistics"], "motionAudit": report["motionAudit"],
               "errors": report["errors"], "warnings": report["warnings"]}
        summary["cases"].append(row)
        print(json.dumps({"name": name, "accepted": row["accepted"], "candidate": row["candidate"], "errors": sorted({e["code"] for e in row["errors"]})}), flush=True)
        (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    source = output / "baseline-42.coaster"
    if source.exists():
        result = subprocess.run([str(cli), "validate", str(source), "--json", str(output / "roundtrip.json")],
                                capture_output=True, text=True, timeout=180)
        (output / "roundtrip.log").write_text(result.stderr, encoding="utf-8")
        summary["roundtripPassed"] = result.returncode == 0
        summary["canonicalSha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
    summary["passed"] = all(c["accepted"] for c in summary["cases"]) and summary.get("roundtripPassed", False)
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
