#!/usr/bin/env python3
"""Diagnostic-only ride trace plots (stdlib + matplotlib Agg).

Reads trace JSON files matching the canonical contract (unmodified):

  {sampleRateHz: 60, seatOrder: [front, middle, rear],
   frames: [{time, distance, speed, seats: [[vertical, lateral, longitudinal], ...]}],
   geometry: [[s, x, y, z, upX, upY, upZ, elementEnum], ...]}
  geometry canonical every 5 m (s spacing ~5 m; deviations noted, not repaired).

Optional evolving JSON summaries alongside traces are read if available
but never edited. Without a summary proving acceptance, plots are labelled
validation=unknown. --accepted-only renders only summary-accepted traces.

Coordinate convention (core, right-handed, Z UP): trace geometry rows are
[s, x, y, z, upX, upY, upZ, elementEnum] with x/y the ground plane
(Terrain::height(x, y)) and z height; up is ~{0,0,1}. Plots use
plan(x, y) equal-aspect, elevation(distance s, z), 3D(x, y, z).

Outputs per ride: 1 PNG (plan, elevation, 3D by element, speed vs time,
all-seat vertical G, lateral/longitudinal traces+peaks) with a compact
DIAGNOSTIC-ONLY footer. Plus contact sheets (up to 6 thumbnails/page,
dynamic grid, omitted for single-ride runs) rendered from existing PNGs,
and manifest.json (source sha256 -> plots, invalid files, counts).

These plots show recorded numbers only. They prove nothing about track
visual quality, rendered POV, frame rate, or acceptance.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (enables 3d projection)

SAMPLE_RATE_HZ = 60
SEAT_ORDER = ["front", "middle", "rear"]
GEOMETRY_STEP_M = 5.0
GEOMETRY_STEP_TOL_M = 0.75
DEFAULT_MAX_TRACES = 30
CONTACT_PER_PAGE = 6

DIAGNOSTIC_LABEL = "DIAGNOSTIC ONLY — numbers as-recorded; no visual-quality / POV / fps / acceptance claim."
TOOL_NAME = "inspect_rides"


class TraceError(ValueError):
    pass


def utc_now_iso() -> str:
    return (
        datetime.datetime.now(datetime.timezone.utc)
        .isoformat(timespec="seconds")
        .replace("+00:00", "Z")
    )


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def validate_trace(obj) -> dict:
    """Validate trace contract. Returns normalized dict or raises TraceError."""
    if not isinstance(obj, dict):
        raise TraceError("top-level JSON must be an object")
    sr = obj.get("sampleRateHz")
    if not _is_num(sr) or float(sr) != float(SAMPLE_RATE_HZ):
        raise TraceError(f"sampleRateHz must be {SAMPLE_RATE_HZ}, got {sr!r}")
    if list(obj.get("seatOrder", None) or []) != SEAT_ORDER:
        raise TraceError(f"seatOrder must be {SEAT_ORDER}, got {obj.get('seatOrder')!r}")
    frames = obj.get("frames")
    if not isinstance(frames, list) or not frames:
        raise TraceError("frames must be a non-empty list")
    for i, fr in enumerate(frames):
        if not isinstance(fr, dict):
            raise TraceError(f"frames[{i}] must be an object")
        for k in ("time", "distance", "speed"):
            if not _is_num(fr.get(k)):
                raise TraceError(f"frames[{i}].{k} must be a number")
        seats = fr.get("seats")
        if not isinstance(seats, list) or len(seats) != 3:
            raise TraceError(f"frames[{i}].seats must be a list of 3 seats")
        for s, seat in enumerate(seats):
            if not isinstance(seat, list) or len(seat) != 3:
                raise TraceError(f"frames[{i}].seats[{s}] must be [vertical,lateral,longitudinal]")
            for a, v in enumerate(seat):
                if not _is_num(v):
                    raise TraceError(f"frames[{i}].seats[{s}][{a}] must be a number")
    geo = obj.get("geometry")
    if not isinstance(geo, list) or not geo:
        raise TraceError("geometry must be a non-empty list")
    for i, row in enumerate(geo):
        if not isinstance(row, list) or len(row) != 8:
            raise TraceError(f"geometry[{i}] must be [s,x,y,z,upX,upY,upZ,elementEnum]")
        for c in range(7):
            if not _is_num(row[c]):
                raise TraceError(f"geometry[{i}][{c}] must be a number")
        if not isinstance(row[7], int) or isinstance(row[7], bool):
            raise TraceError(f"geometry[{i}][7] elementEnum must be an int")
    # Canonical spacing check: warn via returned notes, do not repair/fail.
    notes = []
    try:
        ss = [float(r[0]) for r in geo]
        for i in range(1, len(ss)):
            step = ss[i] - ss[i - 1]
            if abs(step - GEOMETRY_STEP_M) > GEOMETRY_STEP_TOL_M:
                notes.append(
                    f"geometry step s[{i-1}]->s[{i}] = {step:.3f} m (expected ~5 m)"
                )
                break
    except Exception:
        pass
    return {"sampleRateHz": 60, "seatOrder": list(SEAT_ORDER),
            "frames": frames, "geometry": geo, "spacingNotes": notes}


def parse_summary(obj) -> bool | None:
    """Tolerantly detect accepted/rejected in evolving summary JSON."""
    if not isinstance(obj, dict):
        return None
    for key in ("accepted", "valid", "success", "ok", "passed"):
        if isinstance(obj.get(key), bool):
            return obj[key]
    status = obj.get("status")
    if isinstance(status, str):
        s = status.strip().lower()
        if s in ("accepted", "ok", "valid", "pass", "passed"):
            return True
        if s in ("rejected", "invalid", "failed", "fail", "error"):
            return False
    for key in ("report", "validation", "simulation", "result", "data", "summary"):
        sub = obj.get(key)
        if isinstance(sub, dict):
            r = parse_summary(sub)
            if r is not None:
                return r
    return None


def summary_preset(obj) -> str | None:
    """Return summary preset string if present (else None)."""
    if isinstance(obj, dict):
        p = obj.get("preset")
        if isinstance(p, str) and p:
            return p
    return None


def is_summary_accepted(obj, preset: str | None = None) -> bool:
    """Strict acceptance: accepted and completed true, cancelled false,
    errors empty, plus preset match when a preset filter is given.

    Never inferred from exit codes or filenames: only these JSON fields.
    Missing fields fail closed (not accepted).
    """
    if not isinstance(obj, dict):
        return False
    if obj.get("accepted") is not True:
        return False
    if obj.get("completed") is not True:
        return False
    if obj.get("cancelled") is not False:
        return False
    if not isinstance(obj.get("errors"), list) or len(obj["errors"]) != 0:
        return False
    if preset is not None and obj.get("preset") != preset:
        return False
    return True


def validation_status(obj) -> str:
    """Display label: accepted only on the strict conjunction; rejected on
    explicit rejection signals; otherwise unknown."""
    if not isinstance(obj, dict):
        return "unknown"
    if is_summary_accepted(obj):
        return "accepted"
    if (obj.get("accepted") is False or obj.get("cancelled") is True
            or obj.get("completed") is False
            or (isinstance(obj.get("errors"), list) and len(obj["errors"]) > 0)):
        return "rejected"
    return "unknown"


def load_json(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def find_summary_for_trace(trace_path: Path, summary_dir: Path | None,
                           report_json: Path | None):
    """Locate optional sidecar summary (read-only). Returns (obj, path|None)."""
    candidates: list[Path] = []
    if report_json is not None:
        candidates.append(Path(report_json))
    stem = trace_path.name
    # Strip known trace suffixes to derive case stem.
    base = stem
    for suffix in (".trace.json", ".json"):
        if base.endswith(suffix):
            base = base[: -len(suffix)]
            break
    same = trace_path.parent
    candidates += [
        same / (base + ".summary.json"),
        same / (base + ".report.json"),
        same / (base + ".json"),
        same / (stem + ".summary.json"),
    ]
    # Sibling cli_json layout from acceptance harness (read-only).
    for parent in (trace_path.parent, trace_path.parent.parent):
        candidates.append(parent / "cli_json" / (base + ".json"))
    if summary_dir is not None:
        sd = Path(summary_dir)
        candidates += [sd / (base + ".json"), sd / (base + ".summary.json"),
                       sd / trace_path.name]
    for c in candidates:
        try:
            if c.is_file():
                return load_json(c), c
        except (OSError, ValueError):
            continue
    return None, None


def collect_trace_files(input_path: Path) -> list[Path]:
    """Deterministically sorted trace candidates (recursive for dirs).

    Sidecar summaries (read-only inputs, never plots) are excluded so they
    are not misreported as malformed traces.
    """
    p = Path(input_path)
    if p.is_file():
        return [p]
    if not p.exists():
        return []
    files = []
    for q in p.rglob("*.json"):
        if not q.is_file():
            continue
        name = q.name.lower()
        if name in ("manifest.json", "summary.json", "report.json", "report.csv"):
            continue
        if name.endswith((".summary.json", ".report.json")):
            continue
        if "cli_json" in {part.lower() for part in q.parts}:
            continue
        files.append(q)
    return sorted(files, key=lambda q: q.as_posix())


def select_traces(paths: list[Path], max_traces: int) -> tuple[list[Path], list[Path]]:
    """First max_traces in sorted order; rest are truncated (deterministic)."""
    if max_traces <= 0:
        return [], list(paths)
    return list(paths[:max_traces]), list(paths[max_traces:])


def _peaks(vals: list[float]) -> tuple[float, float]:
    if not vals:
        return (float("nan"), float("nan"))
    return (min(vals), max(vals))


def geometry_series(trace: dict) -> dict:
    """Split canonical geometry rows [s,x,y,z,...] with x/y ground, z up."""
    geo = trace["geometry"]
    return {
        "s": [float(r[0]) for r in geo],
        "x": [float(r[1]) for r in geo],
        "y": [float(r[2]) for r in geo],
        "z": [float(r[3]) for r in geo],
        "elem": [int(r[7]) for r in geo],
    }


def figure_for_ride(trace: dict, source_path: Path, source_hash: str,
                    status: str, summary_path: Path | None):
    """Build the diagnostic figure. Returns (fig, mapped) where mapped holds
    the numeric series actually attached to the plan/elevation/3D artists,
    so tests can assert the axis mapping (plan=x/y, elevation=s/z)."""
    frames = trace["frames"]
    times = [float(fr["time"]) for fr in frames]
    speeds = [float(fr["speed"]) for fr in frames]
    seats_v = [[float(fr["seats"][s][0]) for fr in frames] for s in range(3)]
    seats_lat = [[float(fr["seats"][s][1]) for fr in frames] for s in range(3)]
    seats_lon = [[float(fr["seats"][s][2]) for fr in frames] for s in range(3)]
    g = geometry_series(trace)
    gx, gy, gz, gs, gelem = g["x"], g["y"], g["z"], g["s"], g["elem"]

    duration = times[-1] - times[0] if len(times) > 1 else 0.0
    length = gs[-1] - gs[0] if len(gs) > 1 else 0.0
    mapped: dict = {}

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(
        f"{source_path.name}  |  sha:{source_hash[:12]}  |  "
        f"n={len(frames)} dur={duration:.1f}s len={length:.0f}m  |  "
        f"validation={status} (diagnostic only)",
        fontsize=10, y=0.97,
    )
    grid = fig.add_gridspec(3, 2, hspace=0.55, wspace=0.25,
                            left=0.06, right=0.97, top=0.90, bottom=0.09)

    # 1. Plan view: ground plane x vs y, equal aspect. Core is Z-up.
    ax = fig.add_subplot(grid[0, 0])
    (plan_line,) = ax.plot(gx, gy, linewidth=1.0)
    ax.scatter([gx[0]], [gy[0]], s=24, label="start")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m, ground)")
    ax.set_ylabel("y (m, ground)")
    ax.set_title("Plan view (x vs y, equal aspect)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    mapped["plan"] = (list(plan_line.get_xdata()), list(plan_line.get_ydata()))

    # 2. Elevation vs distance: height is z.
    ax = fig.add_subplot(grid[0, 1])
    (elev_line,) = ax.plot(gs, gz, linewidth=1.0)
    ax.set_xlabel("distance s (m)")
    ax.set_ylabel("height z (m, up)")
    ax.set_title("Elevation vs distance (z up)")
    ax.grid(True, alpha=0.3)
    mapped["elevation"] = (list(elev_line.get_xdata()), list(elev_line.get_ydata()))

    # 3. 3D canonical path (x, y ground, z up), colored by element.
    ax = fig.add_subplot(grid[1, 0], projection="3d")
    coordinates = {}
    for e, x, y, z in zip(gelem, gx, gy, gz):
        if e not in coordinates:
            coordinates[e] = ([], [], [])
        xs, ys, zs = coordinates[e]
        xs.append(x)
        ys.append(y)
        zs.append(z)
    paths3d = []
    for k, e in enumerate(sorted(coordinates)):
        xs, ys, zs = coordinates[e]
        ax.scatter(xs, ys, zs, s=6, color=f"C{k % 10}", label=f"elem {e}")
        paths3d.append({"element": e, "x": xs, "y": ys, "z": zs})
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m, up)")
    ax.set_title("Canonical path 3D (x,y ground, z up)")
    ax.legend(fontsize=6, loc="best")
    mapped["paths3d"] = paths3d

    # 4. Speed vs time (unmodified).
    ax = fig.add_subplot(grid[1, 1])
    ax.plot(times, speeds, linewidth=1.0)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed (m/s)")
    ax.set_title("Speed vs time")
    ax.grid(True, alpha=0.3)

    # 5. All-seat vertical G vs time, shared scale.
    ax = fig.add_subplot(grid[2, 0])
    for s, name in enumerate(SEAT_ORDER):
        ax.plot(times, seats_v[s], linewidth=0.9, label=name)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("vertical (g)")
    ax.set_title("Seat vertical G vs time (shared scale)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 6. Lateral/longitudinal traces + peaks.
    ax = fig.add_subplot(grid[2, 1])
    peak_lines = []
    for s, name in enumerate(SEAT_ORDER):
        ax.plot(times, seats_lat[s], linewidth=0.8, linestyle="-",
                label=f"{name} lat")
        ax.plot(times, seats_lon[s], linewidth=0.8, linestyle="--",
                label=f"{name} lon")
        lo, hi = _peaks(seats_lat[s])
        lo2, hi2 = _peaks(seats_lon[s])
        peak_lines.append(f"{name}: lat[{lo:.2f},{hi:.2f}] lon[{lo2:.2f},{hi2:.2f}] g")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("acceleration (g)")
    ax.set_title("Lateral (solid) / longitudinal (dashed) vs time")
    ax.legend(fontsize=6, loc="best")
    ax.grid(True, alpha=0.3)
    ax.text(0.01, 0.01, "peaks (g):\n" + "\n".join(peak_lines),
            transform=ax.transAxes, fontsize=6, va="bottom", ha="left",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    foot = (f"diagnostic only — as-recorded numbers; no visual/POV/fps/acceptance claim.  "
            f"source: {source_path.name}  summary: {summary_path.name if summary_path else 'none'}")
    if trace.get("spacingNotes"):
        foot += f"  |  {trace['spacingNotes'][0]}"
    fig.text(0.06, 0.015, foot, fontsize=7)
    stats = {
        "frames": len(frames),
        "durationS": duration,
        "lengthM": length,
        "speedMin": min(speeds) if speeds else None,
        "speedMax": max(speeds) if speeds else None,
        "verticalPeaks": {n: {"min": _peaks(seats_v[s])[0], "max": _peaks(seats_v[s])[1]}
                          for s, n in enumerate(SEAT_ORDER)},
    }
    return fig, mapped, stats


def render_ride_png(trace: dict, source_path: Path, source_hash: str,
                    status: str, summary_path: Path | None,
                    out_png: Path) -> dict:
    """Render one diagnostic PNG from unmodified trace data."""
    fig, _, stats = figure_for_ride(trace, source_path, source_hash,
                                    status, summary_path)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    return stats


def contact_grid(n: int) -> tuple[int, int]:
    """Dynamic thumbnail grid with no empty cells for n<=4."""
    if n <= 1:
        return (1, 1)
    if n <= 3:
        return (1, n)
    if n == 4:
        return (2, 2)
    return (2, 3)


def render_contact_sheet(pngs: list[Path], out_path: Path, title: str) -> None:
    """Render thumbnails of existing PNGs (no re-plotting)."""
    n = len(pngs)
    if n == 0:
        return
    rows, cols = contact_grid(n)
    fig, axes = plt.subplots(rows, cols,
                             figsize=(5.5 * cols, 4.2 * rows + 0.9),
                             squeeze=False)
    fig.suptitle(title, fontsize=11, y=0.97)
    fig.text(0.5, 0.01, "Diagnostic only — thumbnails of as-recorded plots.",
             ha="center", fontsize=8)
    flat = list(axes.flat)
    for i, ax in enumerate(flat):
        ax.axis("off")
        if i < n:
            try:
                img = mpimg.imread(str(pngs[i]))
                ax.imshow(img)
                ax.set_title(pngs[i].name, fontsize=9)
            except (OSError, ValueError):
                ax.set_title(f"{pngs[i].name}\n(unreadable thumbnail)", fontsize=9)
                ax.text(0.5, 0.5, "unreadable", ha="center", va="center")
    fig.tight_layout(rect=(0, 0.03, 1, 0.93))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def default_out_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "artifacts" / "inspection"


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Diagnostic-only plots for canonical ride traces (Agg).")
    p.add_argument("input", help="Trace JSON file or directory (recursive).")
    p.add_argument("--out-dir", default=str(default_out_dir()),
                   help="Output directory (default native/artifacts/inspection).")
    p.add_argument("--max-traces", type=int, default=DEFAULT_MAX_TRACES,
                   help="Max traces rendered in deterministic sorted order (default 30).")
    p.add_argument("--accepted-only", action="store_true",
                   help="Render only traces whose sidecar summary strictly validates "
                        "(accepted+completed true, cancelled false, errors empty).")
    p.add_argument("--preset", default=None,
                   help="With --accepted-only, additionally require summary preset match.")
    p.add_argument("--summary-dir", default=None,
                   help="Optional directory of sidecar summary JSONs (read-only).")
    p.add_argument("--report-json", default=None,
                   help="Optional single summary JSON (read-only, single-trace runs).")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    in_path = Path(args.input)
    out_dir = Path(args.out_dir)
    summary_dir = Path(args.summary_dir) if args.summary_dir else None
    report_json = Path(args.report_json) if args.report_json else None
    if args.max_traces <= 0:
        print("error: --max-traces must be > 0", file=sys.stderr)
        return 2
    if not in_path.exists():
        print(f"error: input not found: {in_path}", file=sys.stderr)
        out_dir.mkdir(parents=True, exist_ok=True)
        manifest = {
            "tool": TOOL_NAME, "generatedAt": utc_now_iso(),
            "diagnosticOnly": DIAGNOSTIC_LABEL,
            "input": str(in_path),
            "validationMode": "accepted-only" if args.accepted_only else "unknown-unless-summary",
            "counts": {"discovered": 0, "selected": 0, "rendered": 0,
                       "skippedNotAccepted": 0, "skippedTruncated": 0, "invalid": 1},
            "rides": [],
            "invalidFiles": [{"path": str(in_path), "reason": "input path not found"}],
            "truncated": [],
            "contactSheets": [],
            "limitations": ("Diagnostic plots of as-recorded numbers only; "
                            "no visual-quality, POV, frame-rate, or acceptance claim."),
        }
        (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        return 2
    if report_json is not None and not report_json.exists():
        print(f"error: --report-json not found: {report_json}", file=sys.stderr)
        return 2

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "plots").mkdir(parents=True, exist_ok=True)

    discovered = collect_trace_files(in_path)
    selected, truncated = select_traces(discovered, args.max_traces)

    rides = []
    invalid_files = []
    skipped_not_accepted = []
    for tp in selected:
        try:
            raw = load_json(tp)
        except (OSError, ValueError) as e:
            invalid_files.append({"path": tp.as_posix(), "reason": f"unparseable JSON: {e}"})
            continue
        try:
            trace = validate_trace(raw)
        except TraceError as e:
            # Sidecar summaries that slip through name filters are not traces.
            if (isinstance(raw, dict) and "frames" not in raw
                    and "geometry" not in raw and parse_summary(raw) is not None):
                continue
            invalid_files.append({"path": tp.as_posix(), "reason": str(e)})
            continue
        try:
            h = sha256_file(tp)
        except OSError as e:
            invalid_files.append({"path": tp.as_posix(), "reason": f"hash failed: {e}"})
            continue
        summary_obj, summary_path = find_summary_for_trace(tp, summary_dir, report_json)
        # Acceptance is NEVER inferred from exit codes or filenames: only the
        # sidecar JSON fields (accepted/completed/cancelled/errors).
        status = validation_status(summary_obj)
        if args.accepted_only and not is_summary_accepted(summary_obj, args.preset):
            if args.preset is not None and is_summary_accepted(summary_obj):
                reason = f"preset mismatch (required {args.preset!r})"
            else:
                reason = ("summary did not strictly validate "
                          "(need accepted+completed true, cancelled false, errors [])")
            skipped_not_accepted.append(
                {"path": tp.as_posix(), "validationStatus": status, "reason": reason})
            continue
        out_png = out_dir / "plots" / (tp.stem.replace(".trace", "").replace(".json", "") + ".png")
        # Avoid collisions from identical stems in different dirs.
        if any(r.get("plot") == out_png.name for r in rides):
            out_png = out_dir / "plots" / (h[:12] + "_" + out_png.name)
        try:
            stats = render_ride_png(trace, tp, h, status, summary_path, out_png)
        except Exception as e:  # never silently skip
            invalid_files.append({"path": tp.as_posix(), "reason": f"render failed: {e}"})
            continue
        rides.append({
            "source": tp.as_posix(), "sourceSha256": h,
            "plot": f"plots/{out_png.name}", "validationStatus": status,
            "summaryPath": summary_path.as_posix() if summary_path else None,
            "summaryPreset": summary_preset(summary_obj),
            "stats": stats,
            "spacingNotes": trace.get("spacingNotes", []),
        })

    rides.sort(key=lambda r: r["source"])
    plot_paths = [out_dir / r["plot"] for r in rides]
    # Single-ride runs omit the contact sheet (a 1x1 re-embed adds nothing);
    # the manifest still records the decision honestly.
    contacts = []
    if len(plot_paths) > 1:
        for i in range(0, len(plot_paths), CONTACT_PER_PAGE):
            chunk = plot_paths[i:i + CONTACT_PER_PAGE]
            page = i // CONTACT_PER_PAGE + 1
            cp = out_dir / f"contact_{page:02d}.png"
            render_contact_sheet(chunk, cp, f"Inspection contact sheet {page} ({len(chunk)} rides)")
            contacts.append(cp.name)

    manifest = {
        "tool": TOOL_NAME,
        "generatedAt": utc_now_iso(),
        "diagnosticOnly": DIAGNOSTIC_LABEL,
        "input": str(in_path),
        "outDir": str(out_dir),
        "validationMode": "accepted-only" if args.accepted_only else "unknown-unless-summary",
        "presetFilter": args.preset,
        "limits": {"maxTraces": args.max_traces, "contactPerPage": CONTACT_PER_PAGE},
        "counts": {
            "discovered": len(discovered),
            "selected": len(selected),
            "rendered": len(rides),
            "skippedNotAccepted": len(skipped_not_accepted),
            "skippedTruncated": len(truncated),
            "invalid": len(invalid_files),
        },
        "rides": rides,
        "skippedNotAccepted": skipped_not_accepted,
        "invalidFiles": sorted(invalid_files, key=lambda d: d["path"]),
        "truncated": [t.as_posix() for t in truncated],
        "contactSheets": contacts,
        "limitations": ("Diagnostic plots of as-recorded numbers only; "
                        "no visual-quality, POV, frame-rate, or acceptance claim."),
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"discovered={len(discovered)} rendered={len(rides)} "
          f"invalid={len(invalid_files)} skippedNotAccepted={len(skipped_not_accepted)} "
          f"truncated={len(truncated)} contacts={len(contacts)}")
    for inv in sorted(invalid_files, key=lambda d: d["path"]):
        print(f"  invalid: {inv['path']}: {inv['reason']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
