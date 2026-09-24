"""Measure packaged game scene readiness and accepted moving playback.

The first process and configurable later processes are reported separately.
A nearest-rank p99 appears only with at least 100 valid warm samples. Native
progress and Unreal mesh/scene timings remain available for diagnosis.
"""

import argparse
import hashlib
import math
import json
from pathlib import Path
import queue
import re
import shutil
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
        args.extend(("-CoasterVerifyLoad", "-CoasterLoad"))
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
    ready_observed = None
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
                    if timing["stage"] == "ready":
                        ready_observed = observed
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
    requested_event = (events.get("generation-requested") or events.get("cross-process-load-requested")
                       or events.get("shortcut-startup-load-requested"))
    committed_event = events.get("accepted-commit")
    moving_event = events.get("first-motion")
    if not (requested_event and committed_event and moving_event and ready_observed):
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
    ready_seconds = stage_seconds.get("ready")
    if ready_seconds is None or ready_seconds < 0 or ready_observed > first_motion_observed:
        raise RuntimeError(f"Missing or invalid engine scene-ready timing: {output}")
    return {
        "process_to_motion_seconds": wall_motion,
        "process_to_ready_seconds": ready_observed - start,
        "request_to_ready_seconds": ready_seconds,
        "ready_to_motion_seconds": first_motion_observed - ready_observed,
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
    }


def nearest_rank_p99(values):
    """Only report a p99 order statistic when the experiment has 100+ samples."""
    if len(values) < 100:
        return None
    return sorted(values)[math.ceil(.99 * len(values)) - 1]


def summarize(rows, variants):
    summary = {}
    for variant in variants:
        cold = [r for r in rows if r["variant"] == variant and r["sample"] == "first"]
        warm = [r for r in rows if r["variant"] == variant and r["sample"] == "warm"]
        metrics = ("process_to_ready_seconds", "request_to_ready_seconds",
                   "process_to_motion_seconds", "native_seconds",
                   "parsing_revalidation_seconds", "mesh_preparation_seconds",
                   "scene_commit_seconds")
        values = {name: [r[name] for r in warm if r.get(name) is not None] for name in metrics}
        result = {"first_process": cold[0] if len(cold) == 1 else None,
                  "warm_sample_count": len(warm)}
        for name, group in values.items():
            result[name] = {"count": len(group),
                            "median": statistics.median(group) if group else None,
                            "max": max(group) if group else None,
                            "p99_nearest_rank": nearest_rank_p99(group)}
        summary[variant] = result
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, help="Benchmark one packaged game.")
    parser.add_argument("--before", type=Path, help="Baseline executable for comparison.")
    parser.add_argument("--after", type=Path, help="Candidate executable for comparison.")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--warm-runs", "--runs", dest="warm_runs", type=int, default=5,
                        help="Warm repetitions per executable after its first-process run (default: 5).")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--terrain", choices=("highlands", "flat"), default="highlands")
    parser.add_argument("--mode", choices=("generate", "load"), default="generate")
    parser.add_argument("--load-design", type=Path, help="Accepted .vcdesign copied into fresh isolated load profiles.")
    parser.add_argument("--load-profile", type=Path)
    parser.add_argument("--before-load-profile", type=Path)
    parser.add_argument("--after-load-profile", type=Path)
    parser.add_argument("--timeout", type=float, default=1500,
                        help="Per-run wall timeout; the native verifier has its own 1200s safety limit.")
    args = parser.parse_args()
    if args.warm_runs < 1:
        parser.error("--warm-runs must be positive")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.executable:
        if args.before or args.after or args.before_load_profile or args.after_load_profile:
            parser.error("--executable cannot be combined with --before/--after options")
        executables = {"package": args.executable.resolve(strict=True)}
        load_profiles = {"package": args.load_profile}
    else:
        if not args.before or not args.after or args.load_profile:
            parser.error("Supply --executable or both --before and --after")
        executables = {name: getattr(args, name).resolve(strict=True) for name in ("before", "after")}
        load_profiles = {"before": args.before_load_profile, "after": args.after_load_profile}
    if args.load_design and any(profile is not None for profile in load_profiles.values()):
        parser.error("--load-design cannot be combined with an existing load profile")
    if args.mode != "load" and args.load_design:
        parser.error("--load-design requires --mode load")
    if args.mode == "load" and not args.load_design and any(profile is None for profile in load_profiles.values()):
        parser.error("--mode load requires --load-design or a marked profile for every executable")
    for profile in load_profiles.values():
        if profile and not (profile / "coaster-verification-profile.txt").is_file():
            parser.error(f"Not a marked verification profile: {profile}")
    source_design = args.load_design.resolve(strict=True) if args.load_design else None
    if source_design and not source_design.is_file():
        parser.error("--load-design must name a file")
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    if source_design:
        for variant in executables:
            profile = root / f"{variant}-profile"
            target = profile / "Saved" / "VibeCoaster2" / "Designs" / "Accepted.vcdesign"
            target.parent.mkdir(parents=True)
            shutil.copy2(source_design, target)
            (profile / "coaster-verification-profile.txt").write_text(
                "VibeCoaster isolated runtime verification profile v1\n", encoding="utf-8")
            load_profiles[variant] = profile
    metadata = {
        "executables": {name: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                         for name, path in executables.items()},
        "load_design": {"path": str(source_design), "sha256": hashlib.sha256(source_design.read_bytes()).hexdigest()}
                       if source_design else None,
        "mode": args.mode,
        "terrain": args.terrain,
        "seed": args.seed,
        "timing": "Process-to-ready is local monotonic time from before Popen through observed engine scene-ready log. Request-to-ready is the engine monotonic request-to-commit duration. Process-to-motion ends at the verification event after three moving game ticks; this proves usable playback, not a presented frame.",
        "samples": "One first-process sample and exactly warm_runs subsequent samples per executable. Nearest-rank p99 is reported only with at least 100 valid warm samples; it is an empirical order statistic, not an SLA confidence bound.",
        "cache": "Each repetition uses a new process. The first is reported separately; OS and driver caches are not cleared, so it is not a cold-disk measurement.",
        "scope": "Real 2560x1440 rendering; full traversal and screenshots are separate verification.",
    }
    (root / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    rows = []
    variants = tuple(executables)
    for repetition in range(args.warm_runs + 1):
        sample = "first" if repetition == 0 else "warm"
        order = variants if repetition % 2 == 0 else tuple(reversed(variants))
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
