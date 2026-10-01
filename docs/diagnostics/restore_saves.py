"""Restore hash-verified diagnosis saves without replacing existing files."""
import gzip
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[2]
snapshot = root / "docs/diagnostics/current"
manifest = json.loads((snapshot / "manifest.json").read_text(encoding="utf-8"))
output = root / "out/online-diagnosis"
output.mkdir(parents=True, exist_ok=True)
for entry in manifest["artifacts"]:
    if not entry["path"].endswith(".vcdesign.gz"):
        continue
    compressed = (snapshot / entry["path"]).read_bytes()
    assert hashlib.sha256(compressed).hexdigest() == entry["sha256"], entry["path"]
    data = gzip.decompress(compressed)
    assert hashlib.sha256(data).hexdigest() == entry["uncompressedSha256"], entry["path"]
    destination = output / entry["path"][:-3]
    if destination.exists():
        if destination.read_bytes() != data:
            raise SystemExit(f"Refusing to overwrite a different save: {destination}")
    else:
        with destination.open("xb") as stream:
            stream.write(data)
    print(f"Verified {destination}: {entry['uncompressedSha256']}")
