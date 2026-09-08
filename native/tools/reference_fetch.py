"""Reproducible public RFDB metadata fetcher (read-only, no auth, no bypass).

- Public endpoint:  https://rideforcesdb.com/rideInfo?id=<rideId>
- Gated probe:      https://rideforcesdb.com/getRec?id=<recId> (ONE GET per
  probed id only to preserve the observed status/body. No retries, no
  session spoofing, no account creation, no bypass.)

Raw bodies are stored verbatim (including HTTPError bodies) plus a
.provenance.json sidecar (source URL, retrieval UTC, SHA-256, status).
Probe outcomes distinguish: not-probed, transport-failure, denied /
unavailable (HTTP non-200), observed-false, non-trace-response (e.g. login
HTML, not a valid JSON trace), schema-invalid/unrecognized, verified-raw
(explicitly recognized trace shape only). Login requirement is NEVER
inferred from an arbitrary failure; observations are preserved. A failed
refetch never overwrites a prior raw body nor attaches a null hash to it.
Derived summaries never claim peaks imply ten-second exposure.

Stdlib only.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import math
import pathlib
import urllib.error
import urllib.request
from typing import Any, Dict, List, Tuple

TOOL_VERSION = "1.1.0"
USER_AGENT = "Vibecoasterjs-reference-tool/1.1 (read-only public-metadata probe)"

RIDES: Dict[str, Dict[str, Any]] = {
    "pantherian": {"rideInfo_id": 6839, "label": "Pantherian (ex-Intimidator 305)"},
    "do-dodonpa": {"rideInfo_id": 4718, "label": "Do-Dodonpa"},
    "tormenta": {"rideInfo_id": 6383, "label": "Tormenta Rampaging Run"},
    "falcons-flight": {"rideInfo_id": 4804, "label": "Falcon's Flight"},
}

PROBE_REC_IDS = (6839, 2096)

RIDEINFO_URL = "https://rideforcesdb.com/rideInfo?id={rid}"
GETREC_URL = "https://rideforcesdb.com/getRec?id={rid}"


def utc_now_iso() -> str:
    return (
        datetime.datetime.now(datetime.timezone.utc)
        .isoformat(timespec="seconds")
        .replace("+00:00", "Z")
    )


def sha256_hex(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: pathlib.Path) -> str | None:
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
        return h.hexdigest()
    except Exception:
        return None


def single_get(url: str, timeout: float) -> Tuple[Any, bytes, str]:
    """One HTTP GET, no retries. Preserves HTTPError status+body.

    Returns (status, body, error_str) where status is int on HTTP response
    (including 4xx/5xx via HTTPError) or "transport-error" when no HTTP
    response was received. Body is preserved in both cases (b"" if none).
    """
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, resp.read(), ""
    except urllib.error.HTTPError as exc:
        try:
            body = exc.read() or b""
        except Exception:
            body = b""
        return exc.code, body, f"HTTPError: {exc.code} {exc.reason}"
    except Exception as exc:
        return "transport-error", b"", f"{type(exc).__name__}: {exc}"


def write_bytes(path: pathlib.Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def write_json(path: pathlib.Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _looks_like_rideinfo(obj: Any) -> bool:
    return (
        isinstance(obj, dict)
        and isinstance(obj.get("name"), str)
        and isinstance(obj.get("recordings"), list)
    )


def _looks_like_trace(obj: Any, recording_id: Any = None) -> bool:
    """Recognize only the documented canonical rider trace schema.

    RFDB's inaccessible native payload schema is not assumed to match this.
    Unknown shapes stay unverified. Structural validation is not calibration
    provenance review or a strongest-10s benchmark eligibility decision.
    """
    if not isinstance(obj, dict) or type(obj.get("trace_version")) is not int or obj["trace_version"] != 1:
        return False
    rid = obj.get("recording_id")
    if not ((type(rid) is int and rid > 0) or (isinstance(rid, str) and rid.strip())):
        return False
    if recording_id is not None and str(rid) != str(recording_id):
        return False
    for key in ("ride", "device", "seat", "source", "configuration"):
        if not isinstance(obj.get(key), str) or not obj[key].strip():
            return False
    cal = obj.get("calibration")
    if not isinstance(cal, dict) or cal.get("status") != "known" or cal.get("convention") != "rider-vertical-specific-force-with-g":
        return False
    if cal.get("frame") != "rider" or cal.get("transform") not in (None, "none", "identity", {"type": "identity"}):
        return False
    if not isinstance(cal.get("calibration_id"), str) or not cal["calibration_id"].strip():
        return False
    samples = obj.get("samples")
    if not isinstance(samples, list) or len(samples) < 2:
        return False
    previous = None
    for row in samples:
        if not isinstance(row, dict):
            return False
        values = [row.get(k) for k in ("time_s", "vertical_g", "lateral_g", "longitudinal_g")]
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            return False
        timestamp = values[0]
        if previous is not None and timestamp <= previous:
            return False
        previous = timestamp
    return True

def _looks_like_html(body: bytes) -> bool:
    head = body[:2048].lstrip().lower()
    return head.startswith(b"<html") or head.startswith(b"<!doctype html") or b"<html" in head


def fetch_rideinfo(
    slug: str, rid: int, raw_dir: pathlib.Path, timeout: float, retrieved_utc: str
) -> Dict[str, Any]:
    url = RIDEINFO_URL.format(rid=rid)
    status, body, error = single_get(url, timeout)
    raw_name = f"rideInfo-{rid}.json"
    prov_name = f"rideInfo-{rid}.provenance.json"
    raw_path = raw_dir / raw_name
    prior_hash = sha256_file(raw_path) if raw_path.exists() else None
    entry: Dict[str, Any] = {
        "ride_slug": slug,
        "rideInfo_id": rid,
        "source_url": url,
        "retrieved_utc": retrieved_utc,
        "http_status": status,
        "fetch_error": error,
        "raw_file": f"raw/{raw_name}",
        "provenance_file": f"raw/{prov_name}",
        "access": "public-metadata",
    }
    if status == "transport-error":
        entry.update({
            "probe_outcome": "transport-failure",
            "sha256": prior_hash,
            "bytes": raw_path.stat().st_size if raw_path.exists() else 0,
            "raw_preserved": raw_path.exists(),
            "parsed_ok": False,
            "note": "no HTTP response; prior raw (if any) retained, not overwritten.",
        })
    else:
        # Preserve HTTP body verbatim, including denial bodies.
        write_bytes(raw_path, body)
        entry.update({
            "sha256": sha256_hex(body),
            "bytes": len(body),
            "raw_preserved": False,
        })
        if status != 200:
            entry.update({
                "probe_outcome": f"denied/unavailable (HTTP {status})",
                "parsed_ok": False,
            })
        else:
            try:
                parsed = json.loads(body.decode("utf-8"))
            except Exception as exc:
                entry.update({
                    "probe_outcome": "schema-invalid (200 with non-JSON body)",
                    "parsed_ok": False,
                    "parse_error": f"{type(exc).__name__}: {exc}",
                })
            else:
                if _looks_like_rideinfo(parsed):
                    entry.update({
                        "probe_outcome": "verified-metadata",
                        "parsed_ok": True,
                        "ride_name": parsed.get("name"),
                        "location": parsed.get("location"),
                        "num_recordings": parsed.get("num"),
                        "recording_ids": [r.get("id") for r in parsed.get("recordings", [])],
                    })
                else:
                    entry.update({
                        "probe_outcome": "schema-invalid (unrecognized rideInfo shape)",
                        "parsed_ok": False,
                    })
    write_json(raw_dir / prov_name, entry)
    return entry


def classify_getrec(status: Any, body: bytes, recording_id: Any = None) -> Dict[str, Any]:
    if status == "transport-error":
        return {
            "probe_outcome": "probe-failed-transport",
            "raw_accessible_without_login": False,
            "raw_status": "probe-failed (transport; login requirement not verified)",
        }
    if status != 200:
        return {
            "probe_outcome": f"denied/unavailable (HTTP {status})",
            "raw_accessible_without_login": False,
            "raw_status": f"denied/unavailable (HTTP {status}; body preserved, login requirement not inferred)",
        }
    text = body.decode("utf-8", errors="replace").strip() if body else ""
    if text == "":
        return {
            "probe_outcome": "empty-response",
            "raw_accessible_without_login": False,
            "raw_status": "empty-response (no trace or literal false was returned)",
        }
    if text == "false":
        return {
            "probe_outcome": "observed-false",
            "raw_accessible_without_login": False,
            "raw_status": (
                "observed-false (unauthenticated probe returned literal "
                "'false'; login requirement reported by site docs but NOT "
                "verified by this tool from this response alone)"
            ),
        }
    try:
        obj = json.loads(body.decode("utf-8"))
    except Exception:
        if _looks_like_html(body) or "Login" in text[:2000]:
            return {
                "probe_outcome": "non-trace-response",
                "raw_accessible_without_login": False,
                "raw_status": "non-trace-response (e.g. login HTML, not a valid JSON trace)",
            }
        return {
            "probe_outcome": "schema-invalid (non-JSON 200 body)",
            "raw_accessible_without_login": False,
            "raw_status": "schema-invalid (200 body is not JSON trace)",
        }
    if obj is False:
        return {
            "probe_outcome": "observed-false",
            "raw_accessible_without_login": False,
            "raw_status": (
                "observed-false (unauthenticated probe returned literal "
                "'false'; login requirement reported by site docs but NOT "
                "verified by this tool from this response alone)"
            ),
        }
    if _looks_like_trace(obj, recording_id):
        return {
            "probe_outcome": "verified-raw",
            "raw_accessible_without_login": True,
            "raw_status": "verified-raw (canonical identity, calibrated rider axes and finite increasing samples; provenance/benchmark eligibility not established)",
        }
    return {
        "probe_outcome": "schema-invalid (unrecognized JSON, not a trace)",
        "raw_accessible_without_login": False,
        "raw_status": "schema-invalid (JSON but not a recognized trace shape)",
    }


def probe_getrec(
    rec_id: int, raw_dir: pathlib.Path, timeout: float, retrieved_utc: str
) -> Dict[str, Any]:
    url = GETREC_URL.format(rid=rec_id)
    status, body, error = single_get(url, timeout)
    raw_name = f"getRec-{rec_id}.response.json"
    prov_name = f"getRec-{rec_id}.provenance.json"
    raw_path = raw_dir / raw_name
    cls = classify_getrec(status, body, rec_id)
    entry: Dict[str, Any] = {
        "recording_id": rec_id,
        "source_url": url,
        "retrieved_utc": retrieved_utc,
        "http_status": status,
        "fetch_error": error,
        "raw_file": f"raw/{raw_name}",
        "provenance_file": f"raw/{prov_name}",
        "note": (
            "Single probe only; no retry, no bypass. Provide your own "
            "calibrated CSV via reference_forces importer; never treat "
            "peaks as traces."
        ),
    }
    entry.update(cls)
    if status == "transport-error":
        prior = sha256_file(raw_path) if raw_path.exists() else None
        entry.update({
            "sha256": prior,
            "bytes": raw_path.stat().st_size if raw_path.exists() else 0,
            "raw_preserved": raw_path.exists(),
        })
    else:
        write_bytes(raw_path, body)
        entry.update({"sha256": sha256_hex(body), "bytes": len(body), "raw_preserved": False})
    write_json(raw_dir / prov_name, entry)
    return entry


def build_summary(
    ride_entries: List[Dict[str, Any]],
    rec_entries: List[Dict[str, Any]],
    raw_dir: pathlib.Path,
    retrieved_utc: str,
) -> Dict[str, Any]:
    rides_out: List[Dict[str, Any]] = []
    for e in ride_entries:
        item: Dict[str, Any] = {
            "ride_slug": e["ride_slug"],
            "rideInfo_id": e["rideInfo_id"],
            "label": RIDES[e["ride_slug"]]["label"],
            "source_url": e["source_url"],
            "retrieved_utc": e["retrieved_utc"],
            "sha256": e.get("sha256"),
            "raw_file": e.get("raw_file"),
            "http_status": e.get("http_status"),
            "fetch_error": e.get("fetch_error", ""),
            "probe_outcome": e.get("probe_outcome", "unknown"),
        }
        if e.get("parsed_ok"):
            raw_path = raw_dir / f"rideInfo-{e['rideInfo_id']}.json"
            try:
                parsed = json.loads(raw_path.read_text(encoding="utf-8"))
            except Exception:
                parsed = {}
            recs = parsed.get("recordings", [])
            peaks = []
            for r in recs:
                st = r.get("statistics", {}) or {}
                peaks.append({
                    "recording_id": r.get("id"),
                    "author": r.get("author"),
                    "seat": r.get("seat"),
                    "quality": r.get("quality"),
                    "advanced": r.get("advanced"),
                    "maxy": st.get("maxy"),
                    "miny": st.get("miny"),
                    "maxc": st.get("maxc"),
                })
            item.update({
                "ride_name": parsed.get("name"),
                "location": parsed.get("location"),
                "num_recordings": parsed.get("num"),
                "recording_peaks": peaks,
                "peaks_disclaimer": (
                    "maxy/miny/maxc are INSTANTANEOUS peaks from RFDB "
                    "metadata. They do NOT yield a ten-second exposure "
                    "integral S. S requires a calibrated time trace."
                ),
            })
        else:
            item["status"] = "metadata-unavailable"
            item["parse_error"] = e.get("parse_error", "")
        rides_out.append(item)
    if not rec_entries:
        raw_status = "not-probed (no /getRec probe in this run)"
    else:
        outcomes = [r.get("probe_outcome", "?") for r in rec_entries]
        if all(o == "observed-false" for o in outcomes):
            raw_status = (
                "observed-false on unauthenticated probes (literal 'false'); "
                "site docs report login-gated traces but this tool does NOT "
                "verify that requirement from 'false' alone."
            )
        elif any(o == "verified-raw" for o in outcomes):
            raw_status = "verified-raw present (see per-probe outcomes)"
        else:
            raw_status = "no verified raw; per-probe outcomes: " + "; ".join(sorted(set(outcomes)))
    return {
        "tool": "reference_fetch",
        "tool_version": TOOL_VERSION,
        "retrieved_utc": retrieved_utc,
        "rides": rides_out,
        "raw_trace_probes": rec_entries,
        "raw_trace_status": raw_status,
        "ten_second_exposure": "unavailable-from-metadata-alone",
    }


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    here = pathlib.Path(__file__).resolve()
    default_ref = here.parent.parent / "references"
    ap.add_argument("--refs-dir", default=str(default_ref))
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--no-probe-raw", action="store_true",
                    help="skip /getRec probes (metadata only)")
    args = ap.parse_args(argv)

    refs_dir = pathlib.Path(args.refs_dir)
    raw_dir = refs_dir / "raw"
    proc_dir = refs_dir / "processed"
    raw_dir.mkdir(parents=True, exist_ok=True)
    proc_dir.mkdir(parents=True, exist_ok=True)

    retrieved_utc = utc_now_iso()
    ride_entries = [
        fetch_rideinfo(slug, info["rideInfo_id"], raw_dir, args.timeout, retrieved_utc)
        for slug, info in RIDES.items()
    ]
    rec_entries: List[Dict[str, Any]] = []
    if not args.no_probe_raw:
        rec_entries = [
            probe_getrec(rid, raw_dir, args.timeout, retrieved_utc)
            for rid in PROBE_REC_IDS
        ]
    summary = build_summary(ride_entries, rec_entries, raw_dir, retrieved_utc)
    write_json(proc_dir / "rfdb_metadata_summary.json", summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
