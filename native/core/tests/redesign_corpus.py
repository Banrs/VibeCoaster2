"""Compact deterministic acceptance corpus; keeps failures and full evidence.

Run with Python 3: redesign_corpus.py COASTER_CLI OUTPUT_DIRECTORY
The output directory must be fresh. This is a prototype physics check, not a
certification of vehicle, restraint, launch, aerodynamic or structural design.
The native organic-generation test supplies the matched 20-percent lower-drag
simulation because the CLI intentionally exposes ride style and recipe inputs,
not an unreviewed vehicle-dynamics override.
"""
import hashlib
import argparse
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

CASES = [
    ("baseline-42", ["--seed", "42", "--terrain", "highlands", "--trims", "auto"]),
    ("seed-77", ["--seed", "77", "--terrain", "highlands", "--trims", "auto"]),
    ("flat-42", ["--seed", "42", "--terrain", "flat", "--trims", "auto"]),
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
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cli",type=Path)
    parser.add_argument("output",type=Path)
    parser.add_argument("--candidates",type=int,default=8,choices=range(1,65))
    options=parser.parse_args()
    cli, output = options.cli.resolve(), options.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    summary = {"utc": datetime.now(timezone.utc).isoformat(), "executable": str(cli),
               "executableSha256": hashlib.sha256(cli.read_bytes()).hexdigest(), "cases": []}
    for name, parameters in CASES:
        base = output / name
        args = [str(cli), "generate", "--preset", "physics-proof", *parameters,"--candidates",str(options.candidates),
                "--json", str(base.with_suffix(".json")), "--trace", str(base) + "-trace.json",
                "--plan", str(base) + "-plan.json"]
        if name in ("baseline-42", "flat-42", "flowing-7"):
            args += ["--out", str(base.with_suffix(".coaster"))]
        with base.with_suffix(".stdout.log").open("w", encoding="utf-8") as stdout, base.with_suffix(".stderr.log").open("w", encoding="utf-8") as stderr:
            run = subprocess.run(args, stdout=stdout, stderr=stderr, timeout=300)
        report = json.loads(base.with_suffix(".json").read_text(encoding="utf-8"))
        geometry=json.loads(Path(str(base)+"-trace.json").read_text(encoding="utf-8"))["geometry"]
        row = {"name": name, "arguments": parameters, "exitCode": run.returncode,
               "accepted": report["accepted"] and run.returncode == 0,
               "candidate": report["candidate"], "topology": report["topology"],
               "generationSeconds":report["generationSeconds"],"timingsSeconds":report["timingsSeconds"],
               "geometrySha256":hashlib.sha256(json.dumps(geometry,separators=(",",":")).encode()).hexdigest(),
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
    by_name={case["name"]:case for case in summary["cases"]}
    summary["seedVariationPassed"]=by_name["baseline-42"]["geometrySha256"]!=by_name["seed-77"]["geometrySha256"]
    summary["passed"] = all(c["accepted"] for c in summary["cases"]) and summary.get("roundtripPassed", False) and summary["seedVariationPassed"]
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
