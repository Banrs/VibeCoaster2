"""Compare Development executables from launch through accepted, moving playback."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import queue
import statistics
import subprocess
import threading
import time


def run(executable, output, terrain, seed):
    output.mkdir()
    events_dir = output / "events"
    log_path = output / "engine.log"
    args = [str(executable), "-windowed", "-ResX=2560", "-ResY=1440", "-ForceRes",
            "-RenderOffscreen", "-nosplash", "-unattended", "-NoVSync",
            "-ExecCmds=t.MaxFPS 60", f"-UserDir={output / 'profile'}",
            f"-CoasterVerify={events_dir}", "-CoasterVerifyMode=PhysicsProof",
            "-CoasterVerifyBenchmark", f"-CoasterVerifySeed={seed}",
            f"-CoasterVerifyTerrain={terrain}", f"-abslog={log_path}", "-stdout", "-FullStdOutLogOutput"]
    startup = None
    if hasattr(subprocess, "STARTUPINFO"):
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = 0
    start = time.perf_counter()
    process = subprocess.Popen(args, cwd=executable.parent, startupinfo=startup,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, encoding="utf-8", errors="replace")
    incoming = queue.Queue()

    def read_output():
        for line in process.stdout:
            if "CoasterVerify: " in line:
                incoming.put((time.perf_counter(), line.split("CoasterVerify: ", 1)[1]))

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    events = {}
    try:
        while time.perf_counter() - start < 180:
            try:
                observed, line = incoming.get(timeout=.05)
            except queue.Empty:
                if process.poll() is not None and not reader.is_alive():
                    raise RuntimeError(f"Game exited before motion: {log_path}")
                continue
            event = json.loads(line)
            events[event["event"]] = event
            if event["event"] == "first-motion":
                elapsed = observed - start
                break
        else:
            raise TimeoutError(f"Launch-to-motion timed out: {log_path}")
        if process.wait(timeout=15) != 0:
            raise RuntimeError(f"Game reported a benchmark failure: {log_path}")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        reader.join(timeout=5)
        process.stdout.close()
    result = json.loads((events_dir / "result.json").read_text(encoding="utf-8-sig"))
    report = json.loads((events_dir / "accepted-report.json").read_text(encoding="utf-8-sig"))
    if (result["status"] != "launch-benchmark-passed" or not report["accepted"]
            or report["candidate"] != 0 or not report["convergence"]["passed"]):
        raise RuntimeError(f"Benchmark did not preserve candidate-zero acceptance: {output}")
    requested = events["generation-requested"]["wall_seconds"]
    committed = events["accepted-commit"]["wall_seconds"]
    moving = events["first-motion"]["wall_seconds"]
    stages = {}
    log = log_path.read_text(encoding="utf-8-sig", errors="replace")
    for name in ("meshPreparationSeconds", "sceneCommitSeconds"):
        match = re.search(rf"{name}=([0-9.]+)", log)
        if match:
            stages[name] = float(match[1])
    return {"launch_to_motion_seconds": elapsed, "launch_to_request_seconds": elapsed - moving + requested,
            "request_to_scene_seconds": committed - requested, "scene_to_motion_seconds": moving - committed,
            "geometry_sha1": events["accepted-commit"]["geometry_sha1"],
            "version": events["begin"]["geometry_version"], **stages}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.runs < 1:
        parser.error("--runs must be positive")
    root = args.output.resolve()
    root.mkdir()  # Retain previous evidence; never reuse an output directory.
    executables = {name: getattr(args, name).resolve(strict=True) for name in ("before", "after")}
    metadata = {"executables": {name: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                               for name, path in executables.items()},
                "timing": "Process launch to moving-playback event received on stdout; no human input delays.",
                "cache": "First launch reported separately; OS and driver caches are not forcibly cleared.",
                "scope": "Real 2560x1440 rendering; full traversal and screenshots are separate verification."}
    (root / "metadata.json").write_text(json.dumps(metadata, indent=2))
    rows = []
    for repetition in range(args.runs + 1):
        for terrain in ("flat",):
            order = ("before", "after") if repetition % 2 == 0 else ("after", "before")
            for variant in order:
                row = {"variant": variant, "terrain": terrain, "seed": args.seed, "repetition": repetition,
                       **run(executables[variant], root / f"{variant}-{terrain}-{repetition}", terrain, args.seed)}
                rows.append(row)
                with (root / "runs.jsonl").open("a") as stream:
                    stream.write(json.dumps(row) + "\n")
                print(json.dumps(row), flush=True)
    summary = []
    for terrain in ("flat",):
        values = {v: [r["launch_to_motion_seconds"] for r in rows if r["terrain"] == terrain
                      and r["variant"] == v and r["repetition"] > 0] for v in executables}
        medians = {v: statistics.median(times) for v, times in values.items()}
        summary.append({"terrain": terrain, "median_seconds": medians,
                        "speedup": medians["before"] / medians["after"],
                        "observed_max_seconds": {v: max(times) for v, times in values.items()}})
    (root / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
