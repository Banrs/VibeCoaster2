"""Compare Development executables from request through accepted moving playback.

The benchmark records one cold process and a configurable warm sample set. It
reports medians and maxima for the warm samples; it deliberately does not
invent a percentile from five observations. Native progress callbacks and
Unreal mesh/scene timing records are retained separately so generation and
saved-ride loading can be diagnosed without collapsing them into one number.
"""

import argparse
import hashlib
import json
from pathlib import Path
import queue
import re
import statistics
import subprocess
import threading
import time


def _number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_progress(line):
    """Parse the machine-readable CoasterProgress log line emitted by Unreal."""
    phase = re.search(r"phase=(\d+)", line)
    phase_key = re.search(r"phaseKey=([^\s]+)", line)
    phase_name = re.search(r"phaseName=(.*?)\s+candidate=", line)
    candidate = re.search(r"candidate=(-?\d+)", line)
    completed = re.search(r"completed=([^\s]+)", line)
    total = re.search(r"total=([^\s]+)", line)
    elapsed = re.search(r"elapsedSeconds=([^\s]+)", line)
    request = re.search(r"request=(\d+)", line)
    if not (phase and phase_name and candidate and completed and total and elapsed and request):
        return None
    return {
        "request": int(request[1]),
        "phase": int(phase[1]),
        "phase_key": phase_key[1] if phase_key else phase_name[1],
        "phase_name": phase_name[1],
        "candidate": int(candidate[1]),
        "completed": _number(completed[1]),
        "total": _number(total[1]),
        "elapsed_seconds": _number(elapsed[1]),
    }


def parse_timing(line):
    """Parse the machine-readable CoasterTiming log line emitted by Unreal."""
    request = re.search(r"request=(\d+)", line)
    stage = re.search(r"stage=([^\s]+)", line)
    elapsed = re.search(r"elapsedSeconds=([^\s]+)", line)
    total = re.search(r"totalSeconds=([^\s]+)", line)
    candidate = re.search(r"candidate=(-?\d+)", line)
    if not (request and stage and elapsed):
        return None
    result = {"request": int(request[1]), "stage": stage[1],
              "elapsed_seconds": _number(elapsed[1])}
    if total:
        result["total_seconds"] = _number(total[1])
    if candidate:
        result["candidate"] = int(candidate[1])
    return result


def native_phase_timings(progress, native_seconds):
    """Close callback phase intervals using the native-complete wall time."""
    if not progress:
        return {}
    phases = {}
    current = None
    started = None
    for update in progress:
        elapsed = update.get("elapsed_seconds")
        if elapsed is None:
            continue
        name = update.get("phase_key", update["phase_name"])
        if name in {"meshPreparation", "sceneCommit", "complete"}:
            continue  # These stages have separate wall-clock measurements.
        if current is None:
            current, started = name, elapsed
        elif name != current:
            phases[current] = phases.get(current, 0.0) + max(0.0, elapsed - started)
            current, started = name, elapsed
    if current is not None and started is not None and native_seconds is not None:
        phases[current] = phases.get(current, 0.0) + max(0.0, native_seconds - started)
    return phases


def run(executable, output, terrain, seed, timeout, mode="generate", load_profile=None):
    output.mkdir(parents=True)
    events_dir = output / "events"
    log_path = output / "engine.log"
    profile = Path(load_profile).resolve() if load_profile else (output / "profile").resolve()
    args = [str(executable), "-windowed", "-ResX=2560", "-ResY=1440", "-ForceRes",
            "-RenderOffscreen", "-nosplash", "-unattended", "-NoVSync",
            "-ExecCmds=t.MaxFPS 60", f"-UserDir={profile}",
            f"-CoasterVerify={events_dir.resolve()}", "-CoasterVerifyMode=PhysicsProof",
            "-CoasterVerifyBenchmark", f"-CoasterVerifySeed={seed}",
            f"-CoasterVerifyTerrain={terrain}", f"-abslog={log_path.resolve()}", "-stdout", "-FullStdOutLogOutput"]
    if mode == "load":
        args.append("-CoasterVerifyLoad")
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
            observed = time.perf_counter()
            if "CoasterVerify: " in line:
                incoming.put(("verify", observed, line.split("CoasterVerify: ", 1)[1]))
            elif "CoasterProgress " in line:
                incoming.put(("progress", observed, line.split("CoasterProgress ", 1)[1]))
            elif "CoasterTiming " in line:
                incoming.put(("timing", observed, line.split("CoasterTiming ", 1)[1]))

    reader = threading.Thread(target=read_output, daemon=True)
    reader.start()
    events = {}
    progress = []
    timings = []
    first_motion_observed = None
    try:
        while time.perf_counter() - start < timeout:
            try:
                kind, observed, line = incoming.get(timeout=.05)
            except queue.Empty:
                if process.poll() is not None and not reader.is_alive():
                    raise RuntimeError(f"Game exited before accepted motion: {log_path}")
                continue
            if kind == "verify":
                event = json.loads(line)
                events[event["event"]] = event
                if event["event"] == "first-motion":
                    first_motion_observed = observed
                    break
            elif kind == "progress":
                update = parse_progress(line)
                if update:
                    progress.append(update)
            else:
                timing = parse_timing(line)
                if timing:
                    timings.append(timing)
        else:
            raise TimeoutError(f"Launch-to-motion timed out after {timeout:.0f}s: {log_path}")
        try:
            if process.wait(timeout=90) != 0:
                raise RuntimeError(f"Game reported a benchmark failure: {log_path}")
        except subprocess.TimeoutExpired as error:
            raise TimeoutError(f"Game did not exit after accepted motion: {log_path}") from error
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        reader.join(timeout=5)
        process.stdout.close()
    if first_motion_observed is None:
        raise RuntimeError(f"Benchmark did not observe motion: {output}")
    result = json.loads((events_dir / "result.json").read_text(encoding="utf-8-sig"))
    report = json.loads((events_dir / "accepted-report.json").read_text(encoding="utf-8-sig"))
    candidate = report.get("candidate")
    if (result.get("status") != "launch-benchmark-passed" or not report.get("accepted")
            or not isinstance(candidate, int) or candidate < 0
            or not report.get("convergence", {}).get("passed")):
        raise RuntimeError(f"Benchmark did not produce a valid accepted candidate: {output}")
    requested_event = events.get("generation-requested") or events.get("cross-process-load-requested")
    committed_event = events.get("accepted-commit")
    moving_event = events.get("first-motion")
    if not (requested_event and committed_event and moving_event):
        raise RuntimeError(f"Benchmark event stream lacks request/commit/motion markers: {output}")
    native_seconds = next((x["elapsed_seconds"] for x in reversed(timings)
                           if x["stage"] == "native-complete"), None)
    stage_seconds = {x["stage"]: x["elapsed_seconds"] for x in timings
                     if x["stage"] in {"mesh-preparation", "scene-commit", "ready"}}
    phase_seconds = native_phase_timings(progress, native_seconds)
    parsing_values = [value for name, value in phase_seconds.items()
                      if name == "parsing" or "saved ride" in name.lower() or "revalidat" in name.lower()]
    parsing_seconds = sum(parsing_values) if parsing_values else None
    wall_motion = first_motion_observed - start
    return {
        "launch_to_motion_seconds": wall_motion,
        "launch_to_request_seconds": requested_event["wall_seconds"],
        "request_to_scene_seconds": committed_event["wall_seconds"] - requested_event["wall_seconds"],
        "scene_to_motion_seconds": moving_event["wall_seconds"] - committed_event["wall_seconds"],
        "geometry_sha1": committed_event["geometry_sha1"],
        "version": events.get("begin", {}).get("geometry_version"),
        "build_commit":events.get("begin",{}).get("build_commit"),
        "mode": mode,
        "terrain": terrain,
        "candidate": candidate,
        "native_seconds": native_seconds,
        "native_phase_seconds": phase_seconds,
        "parsing_revalidation_seconds": parsing_seconds,
        "mesh_preparation_seconds": stage_seconds.get("mesh-preparation"),
        "scene_commit_seconds": stage_seconds.get("scene-commit"),
        "ready_seconds": stage_seconds.get("ready"),
    }


def samples(rows, variant, sample_kind):
    return [row for row in rows if row["variant"] == variant and row["sample"] == sample_kind]


def summarize(rows, variants):
    summary = {}
    for variant in variants:
        cold = samples(rows, variant, "cold")
        warm = samples(rows, variant, "warm")
        warm_values = [row["launch_to_motion_seconds"] for row in warm]
        cold_values = [row["launch_to_motion_seconds"] for row in cold]
        native_values = [r["native_seconds"] for r in warm if r["native_seconds"] is not None]
        parsing_values = [r["parsing_revalidation_seconds"] for r in warm if r["parsing_revalidation_seconds"] is not None]
        mesh_values = [r["mesh_preparation_seconds"] for r in warm if r["mesh_preparation_seconds"] is not None]
        commit_values = [r["scene_commit_seconds"] for r in warm if r["scene_commit_seconds"] is not None]
        ready_values = [r["ready_seconds"] for r in warm if r["ready_seconds"] is not None]
        request_values=[r["request_to_scene_seconds"] for r in warm]
        summary[variant] = {
            "cold_seconds": cold_values[0] if len(cold_values) == 1 else None,
            "warm_sample_count": len(warm_values),
            "warm_median_seconds": statistics.median(warm_values) if warm_values else None,
            "warm_max_seconds": max(warm_values) if warm_values else None,
            "warm_native_median_seconds": statistics.median(native_values) if native_values else None,
            "warm_parsing_revalidation_median_seconds": statistics.median(parsing_values) if parsing_values else None,
            "warm_mesh_median_seconds": statistics.median(mesh_values) if mesh_values else None,
            "warm_scene_commit_median_seconds": statistics.median(commit_values) if commit_values else None,
            "warm_ready_median_seconds": statistics.median(ready_values) if ready_values else None,
            "warm_request_to_scene_median_seconds":statistics.median(request_values) if request_values else None,
            "warm_request_to_scene_max_seconds":max(request_values) if request_values else None,
        }
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", required=True, type=Path)
    parser.add_argument("--after", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--warm-runs", "--runs", dest="warm_runs", type=int, default=5,
                        help="Warm repetitions per executable after its separate cold run (default: 5).")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--terrain", choices=("highlands", "flat"), default="highlands")
    parser.add_argument("--mode", choices=("generate", "load"), default="generate")
    parser.add_argument("--before-load-profile", type=Path)
    parser.add_argument("--after-load-profile", type=Path)
    parser.add_argument("--timeout", type=float, default=1500,
                        help="Per-run wall timeout; the native verifier has its own 1200s safety limit.")
    args = parser.parse_args()
    if args.warm_runs < 1:
        parser.error("--warm-runs must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.mode == "load" and (args.before_load_profile is None or args.after_load_profile is None):
        parser.error("--mode load requires --before-load-profile and --after-load-profile")
    root = args.output.resolve()
    root.mkdir(parents=True)
    executables = {name: getattr(args, name).resolve(strict=True) for name in ("before", "after")}
    load_profiles = {"before": args.before_load_profile, "after": args.after_load_profile}
    metadata = {
        "executables": {name: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                         for name, path in executables.items()},
        "mode": args.mode,
        "terrain": args.terrain,
        "seed": args.seed,
        "timing": "Process launch through accepted moving-playback event; native callback, mesh preparation, scene commit and ready timings are retained separately.",
        "samples": "One cold sample and exactly warm_runs warm samples per executable; warm summaries report median and maximum only. No p95 is inferred from five samples.",
        "cache": "Each repetition uses a new process. The first is reported separately; OS and driver caches are not forcibly cleared, so cold means first process in this experiment, not an empty machine cache.",
        "scope": "Real 2560x1440 rendering; full traversal and screenshots are separate verification.",
    }
    (root / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    rows = []
    variants = tuple(executables)
    for repetition in range(args.warm_runs + 1):
        sample = "cold" if repetition == 0 else "warm"
        order = ("before", "after") if repetition % 2 == 0 else ("after", "before")
        for variant in order:
            row = {"variant": variant, "sample": sample, "terrain": args.terrain,
                   "seed": args.seed, "repetition": repetition,
                   **run(executables[variant], root / f"{variant}-{args.mode}-{args.terrain}-{repetition}",
                         args.terrain, args.seed, args.timeout, args.mode,
                         load_profiles[variant])}
            rows.append(row)
            with (root / "runs.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(row) + "\n")
            print(json.dumps(row), flush=True)
    summary = {"terrain": args.terrain, "mode": args.mode,
               "warm_runs": args.warm_runs, "variants": summarize(rows, variants)}
    (root / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
