"""Export a self-contained, offline ride review from actual CLI output.

python native/tools/ride_review.py out/ride --out out/ride-review.html
expects ride.json, ride-trace.json and optionally ride-plan.json. The viewer
interpolates the exported display samples; it does not replace native validation.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def finite(values):
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in values)


def compact(value):
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("Non-finite value in review data")
        return round(value, 6)
    if isinstance(value, list):
        return [compact(x) for x in value]
    if isinstance(value, dict):
        return {k: compact(v) for k, v in value.items()}
    return value


def export(base, destination, fps=30):
    report_path = Path(str(base) + ".json")
    trace_path = Path(str(base) + "-trace.json")
    plan_path = Path(str(base) + "-plan.json")
    report, trace = read(report_path), read(trace_path)
    plan = (read(plan_path) if plan_path.exists() else None) or {}
    geometry, source_frames = trace.get("geometry", []), trace.get("frames", [])
    if len(geometry) < 2 or len(source_frames) < 2:
        raise ValueError("Review requires sampled geometry and at least two replay frames")
    if any(len(row) < 8 or not finite(row[:8]) for row in geometry):
        raise ValueError("Geometry contains an invalid sample")
    if any(b[0] <= a[0] for a, b in zip(geometry, geometry[1:])):
        raise ValueError("Geometry distances must increase")
    if abs(report["lengthMeters"] - geometry[-1][0]) > 3:
        raise ValueError("Report and trace have different track lengths")
    if trace.get("generatorVersion", report["generatorVersion"]) != report["generatorVersion"]:
        raise ValueError("Report and trace have different generator versions")
    frames, next_time, previous_time = [], -1, -1
    for index, frame in enumerate(source_frames):
        time = frame["time"]
        if not finite([time, frame["distance"], frame["speed"]]) or time <= previous_time:
            raise ValueError("Replay frames must have finite, increasing times")
        previous_time = time
        if time < next_time and index != len(source_frames) - 1:
            continue
        poses = [pose["forward"] + pose["up"] for pose in frame["dynamics"]]
        forces = frame["seats"]
        if len(poses) != 3 or len(forces) != 3 or any(len(p) != 6 or not finite(p) for p in poses) or any(len(f) != 3 or not finite(f) for f in forces):
            raise ValueError("Replay requires three finite seat force/frame samples")
        row = [time, frame["distance"], frame["speed"], *forces, *poses]
        positions = [pose.get("position") for pose in frame["dynamics"]]
        if all(p is not None and len(p) == 3 and finite(p) for p in positions):
            row.extend(positions)
        frames.append(row)
        next_time = time + 1 / fps - 1e-6
    sections = [{k: section[k] for k in ("name", "role", "start", "end")}
                for section in trace.get("sections", report.get("motionAudit", {}).get("sections", []))]
    anchors = (plan.get("composition") or {}).get("terrainAnchors", [])
    # Only an explicitly exported terrain mesh is drawn as terrain. The ordinary
    # report's analytic parameters are not approximated with invented landforms.
    terrain = trace.get("terrainMesh")
    if terrain is None and trace.get("terrainGrid"):
        grid = trace["terrainGrid"]
        columns, rows = grid["columns"], grid["rows"]
        if columns < 2 or rows < 2 or len(grid["heights"]) != columns * rows or not finite(grid["heights"]):
            raise ValueError("Invalid exported terrain grid")
        stride = max(1, math.ceil(max(columns, rows) / 50))
        xs = sorted(set(range(0, columns, stride)) | {columns - 1})
        ys = sorted(set(range(0, rows, stride)) | {rows - 1})
        vertices = [[grid["origin"][0] + x * grid["step"][0],
                     grid["origin"][1] + y * grid["step"][1], grid["heights"][y * columns + x]]
                    for y in ys for x in xs]
        triangles = []
        for y in range(len(ys) - 1):
            for x in range(len(xs) - 1):
                i = y * len(xs) + x
                triangles.extend(([i, i + 1, i + len(xs)], [i + 1, i + len(xs) + 1, i + len(xs)]))
        terrain = {"vertices": vertices, "triangles": triangles,
                   "representation": f"Exported terrain overview reduced every {stride} samples for display"}
    if terrain is not None:
        if not isinstance(terrain, dict) or not terrain.get("vertices") or not terrain.get("triangles"):
            raise ValueError("terrainMesh requires vertices and triangles")
        if any(len(p) != 3 or not finite(p) for p in terrain["vertices"]):
            raise ValueError("Invalid exported terrain vertex")
        if any(len(t) != 3 or any(not isinstance(i, int) or i < 0 or i >= len(terrain["vertices"]) for i in t) for t in terrain["triangles"]):
            raise ValueError("Invalid exported terrain triangle")
    data = compact({
        "title": trace.get("title", "Follow the feeling."),
        "seed": str(report["seed"]), "version": report["generatorVersion"],
        "accepted": bool(report["accepted"]), "length": report["lengthMeters"],
        "sourceRate": trace.get("sampleRateHz"), "reviewRate": fps,
        "nativeRate": trace.get("acceptanceSampleRateHz"), "train": report["train"],
        "gauge": report.get("trackSection", {}).get("gaugeMeters", 1.4),
        "geometry": geometry, "frames": frames, "sections": sections,
        "anchors": anchors, "terrain": terrain,
        "landmarks": trace.get("landmarks", []),
        "operations": trace.get("operations", []),
        "supportMembers": trace.get("supportMembers", []),
        "stationBoxes": trace.get("stationBoxes", []),
        "metrics": {k: report["metrics"].get(k) for k in ("maxSpeed", "maxGroundHeight", "launchTo180", "duration")},
        "errors": [{"code": e["code"], "message": e["message"]} for e in report.get("errors", [])],
        "sources": [{"name": p.name, "sha256": hashlib.sha256(p.read_bytes()).hexdigest()} for p in (report_path, trace_path)],
    })
    payload = json.dumps(data, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
    payload = payload.replace("<", "\\u003c").replace("\u2028", "\\u2028").replace("\u2029", "\\u2029")
    template = Path(__file__).with_name("ride_review.html").read_text(encoding="utf-8")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(template.replace("__RIDE_DATA__", payload), encoding="utf-8")
    return len(frames), len(geometry)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("base", type=Path, help="Prefix of the CLI report and -trace.json files")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=30, choices=(15, 30, 60))
    args = parser.parse_args()
    try:
        frames, points = export(args.base, args.out, args.fps)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"Cannot create ride review: {error}\n")
    print(f"{args.out}: {frames} replay frames, {points} actual track samples; opens offline")
