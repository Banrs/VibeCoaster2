"""Convert user-supplied Ride Forces JSON without asserting measurement eligibility.

Verified source: https://rideforcesdb.com/record and
https://rideforcesdb.com/js/index.js?v=4681fe0a (2026-09-08).
The supported schema is seated, 50 Hz, including-gravity device acceleration.
RFDB uses calibration-row dot products, not a right-handed rotation repair.
Its viewer smooths the full projected sequence before selecting the ride.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import pathlib
import uuid

import reference_forces as rf

AXES = ("y", "x", "z")  # Canonical CSV: vertical, lateral, longitudinal.
VIEWER_SOURCE = "https://rideforcesdb.com/js/index.js?v=4681fe0a"


def number(value):
    if isinstance(value, bool):
        raise rf.ForceValidationError("Boolean is not a force or sample rate")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise rf.ForceValidationError("Expected a finite number") from None
    if not math.isfinite(result):
        raise rf.ForceValidationError("Expected a finite number")
    return result


def vector(value):
    if not isinstance(value, dict) or not all(k in value for k in "xyz"):
        raise rf.ForceValidationError("Expected an xyz vector")
    return tuple(number(value[k]) for k in "xyz")


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def rfdb_smooth(values):
    """RFDB's centered 11-sample mean, with shortened full-recording edges."""
    total = sum(values[:5])
    count = min(5, len(values))
    result = []
    for i in range(len(values)):
        if i + 5 < len(values):
            total += values[i + 5]
            count += 1
        if i >= 6:
            total -= values[i - 6]
            count -= 1
        result.append(total / count)
    return result


def csv_bytes(times, axes, columns=rf.REQUIRED_COLUMNS):
    stream = io.StringIO(newline="")
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(columns)
    writer.writerows(zip(times, *axes))
    return stream.getvalue().encode("utf-8")


def convert(source, output, rfdb_id=None, seat=None):
    source, output = pathlib.Path(source), pathlib.Path(output)
    if output.exists():
        raise rf.OutputAliasError("Output directory must be new; previous results are preserved")
    if rfdb_id is not None and (isinstance(rfdb_id, bool) or not isinstance(rfdb_id, int) or rfdb_id <= 0):
        raise rf.ForceValidationError("RFDB ID must be a positive integer")
    if seat is not None and (not isinstance(seat, str) or not seat.strip()):
        raise rf.ForceValidationError("Seat override must describe the observed source seat")
    raw = source.read_bytes()
    try:
        data = json.loads(raw.decode("utf-8-sig"))
    except (ValueError, UnicodeError) as exc:
        raise rf.ForceValidationError("Invalid .forces JSON") from exc
    if not isinstance(data, dict):
        raise rf.ForceValidationError("Expected a .forces object")
    for key in ("name", "device"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise rf.ForceValidationError("Missing recording " + key)
    try:
        identity = str(uuid.UUID(data.get("uuid", "")))
    except (ValueError, AttributeError, TypeError):
        raise rf.ForceValidationError("Missing or invalid recording UUID") from None
    if number(data.get("sampleRate")) != 50 or data.get("ridingPosition") != "Seated":
        raise rf.ForceValidationError("Only verified seated 50 Hz .forces recordings are supported")
    sequence = data.get("accelerationSequence")
    if not isinstance(sequence, list) or len(sequence) < 2:
        raise rf.ForceValidationError("At least two acceleration samples are required")
    samples = [vector(v) for v in sequence]
    n = len(samples)
    for key in ("startIndex", "endIndex", "gravityIndex", "inclineIndex"):
        value = data.get(key)
        upper = n if key == "endIndex" else n - 1
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= upper:
            raise rf.ForceValidationError("Invalid " + key)
    start, end = data["startIndex"], data["endIndex"]
    if end <= start:
        raise rf.ForceValidationError("Selected interval must have positive duration")
    basis = {key: vector(data.get(key + "CalibrationVector")) for key in "xyz"}
    # Published vectors are rounded to 3 decimals. Check, but never change, them.
    x, y, z = (basis[key] for key in "xyz")
    cross = (z[1]*y[2]-z[2]*y[1], z[2]*y[0]-z[0]*y[2], z[0]*y[1]-z[1]*y[0])
    if (any(abs(dot(v, v) - 1) > .005 for v in basis.values())
            or any(abs(dot(a, b)) > .005 for a, b in ((x, y), (y, z), (z, x)))
            or any(abs(a - b) > .005 for a, b in zip(x, cross))):
        raise rf.ForceValidationError("Unsupported calibration basis; expected approximately x = z cross y")
    gravity = data.get("gravitySequence")
    if gravity is not None:
        if not isinstance(gravity, list) or len(gravity) != n:
            raise rf.ForceValidationError("Gravity sequence must match acceleration sample count")
        for value in gravity:
            vector(value)  # Retained in original; never subtracted from forces.
    projected = [[dot(basis[key], sample) for sample in samples] for key in AXES]
    smoothed = [rfdb_smooth(axis) for axis in projected]
    # The existing end sample anchors continuous integration at the crop boundary.
    # RFDB's discrete statistics still exclude that sample. Never extend a trace.
    stop = min(end + 1, n)
    times = [(i - start) / 50 for i in range(start, stop)]
    if len(times) < 2:
        raise rf.ForceValidationError("Selected interval needs two observed samples")
    # Native R=T cross U; source X=R, Y=-U, Z=-T act on felt force -F.
    # Consequently native specific-force components are (RFDB y, -RFDB x, RFDB z).
    canonical = csv_bytes(times, [projected[0][start:stop],
                                 [-v for v in projected[1][start:stop]], projected[2][start:stop]])
    comparison = csv_bytes(times, [axis[start:stop] for axis in smoothed],
                           ["time_s", "vertical_g", "rfdb_lateral_g", "longitudinal_g"])
    attributes = data.get("attributes", [])
    if not isinstance(attributes, list):
        raise rf.ForceValidationError("Attributes must be a list")
    seat_parts = [str(a["label"]) + " " + str(a["value"]) for a in attributes
                  if isinstance(a, dict) and a.get("label") in ("Row", "Seat") and a.get("value")]
    source_url = f"https://rideforcesdb.com/?id={rfdb_id}" if rfdb_id else None
    manifest = {
        "ride": data["name"], "device": data["device"],
        "seat": seat.strip() if seat is not None else (", ".join(seat_parts) or None),
        "source": source_url or "User-provided .forces: " + source.name,
        "configuration": None, "recording_id": "rideforces:" + identity,
        "calibration": {
            "status": "unverified", "convention": rf.CANONICAL_CONVENTION,
            "frame": "rider", "transform": "none", "calibration_id": None,
            "smoothing": "none",
            "notes": "CSV already projected using source rows; calibration session and device stability need review.",
        },
        "quality": {"device_rotated": None, "excluded_statistics": None},
        "benchmark_eligible": False,
        "provenance": {
            "adapter": "rideforces-seated-50hz-v2", "original_file": str(source.resolve()),
            "original_sha256": hashlib.sha256(raw).hexdigest(), "original_uuid": data["uuid"],
            "rfdb_id": rfdb_id, "source_url": source_url,
            "source_attributes": attributes, "seat_override": seat,
            "source_date": data.get("date"), "date_epoch": "2001-01-01T00:00:00Z",
            "calibration_vectors": {key: data[key + "CalibrationVector"] for key in "xyz"},
            "gravity_index": data["gravityIndex"], "incline_index": data["inclineIndex"],
            "gravity_sequence_present": gravity is not None,
            "axis_mapping": "vertical=dot(y,acceleration); lateral=-dot(x,acceleration); longitudinal=dot(z,acceleration)",
            "axis_sign_basis": "Native R=T cross U; RFDB felt-force rows X=R,Y=-U,Z=-T act on -specificAcceleration. RFDB comparison retains its original lateral sign.",
            "gravity_subtracted": False, "basis_repaired": False,
            "timing": {
                "sample_rate_hz": 50, "basis": "nominal sample index; actual gaps/jitter unverified",
                "source_sample_count": n, "rfdb_statistics_indices": [start, end],
                "rfdb_statistics_end_exclusive": True, "rfdb_boundary_duration_s": (end-start)/50,
                "csv_indices_inclusive": [start, stop-1], "csv_duration_s": times[-1],
                "end_boundary_sample_included": end < n,
            },
            "source_smoothing_factor": data.get("smoothingFactor"),
            "comparison": {
                "file": "rfdb-smoothed-comparison.csv", "purpose": "RFDB display comparison; not benchmark input",
                "method": "Full-sequence centered 11-sample mean; shortened edges, then crop",
                "source": VIEWER_SOURCE,
                "vertical_min_g": min(smoothed[0][start:end]),
                "vertical_max_g": max(smoothed[0][start:end]),
                "source_stored_min_g": data.get("minForce"), "source_stored_max_g": data.get("maxForce"),
            },
            "outputs_sha256": {"canonical.csv": hashlib.sha256(canonical).hexdigest(),
                               "rfdb-smoothed-comparison.csv": hashlib.sha256(comparison).hexdigest()},
        },
    }
    encoded = (json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")
    output.mkdir(parents=True)  # Exclusive new result directory; no previous evidence is replaced.
    for name, content in (("canonical.csv", canonical), ("rfdb-smoothed-comparison.csv", comparison),
                          ("manifest.json", encoded)):
        with (output / name).open("xb") as stream:
            stream.write(content)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=pathlib.Path)
    parser.add_argument("--out-dir", required=True, type=pathlib.Path)
    parser.add_argument("--rfdb-id", type=int)
    parser.add_argument("--seat", help="Seat observed on the source page; retained separately from original attributes")
    args = parser.parse_args(argv)
    try:
        result = convert(args.input, args.out_dir, args.rfdb_id, args.seat)
    except (OSError, ValueError) as exc:
        parser.exit(2, f"Import failed: {exc}\n")
    print(json.dumps({"status": "converted-unverified", "benchmark_eligible": False,
                      "recording_id": result["recording_id"], "output": str(args.out_dir)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
