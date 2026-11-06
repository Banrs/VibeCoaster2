# V3 generator rewrite

The latest user direction, 6 November 2026, is to rewrite the generator
architecture rather than keep patching it, research coaster design, and add a
few strong ride-feel regressions. Forward progress takes priority over expanding
a defensive test matrix. The user also permits changing inherited constraints
for understandable architecture and small, evidence-supported force excursions
for better feel. Future hardware is not an assumption of greater human tolerance.

Starting point: `main` at `5dab7c0`, already the newest remote branch. Work is on
`codex/coaster-rewrite` for PR review. The earlier handoff is preserved at
[docs/checkpoints/default3-handoff.md](docs/checkpoints/default3-handoff.md).

## Design direction

- Route compilation owns geometry, frames and hardware. Finite-train energy
  fitting remains a physical solve, not permission to change force intent after
  independent acceptance rejects a ride.
- Source-owned frames replace the global banking/restoration cycle. Boundaries
  inherit their complete geometry and frame state.
- Freeze the route before independent physics, structure and clearance checks.
- Measure experience using elapsed replay time: airtime, contrasting load beats,
  inactive gaps and roll episodes.
- Development launches explicitly generate current source. Historical saves
  retain their identity and receive fresh validation when explicitly loaded.

## Product requirements

Keep the deliberate 0–180 km/h launch in about 1.4 seconds. Improve opening
flow, banking and connections, cliff pacing, drive placement and the return.
Use [manufacturer and authoring research](docs/references/generator-rewrite-research.md)
for Falcon's Flight and Tormenta Rampaging Run, distinguishing verified evidence
from inferred geometry. Consumer video overlays are not engineering force data.

Retain art assets, cameras, dimensional contracts, independent finite-train
physics, clearance and atomic saves. The new train art remains a proposal.
This cloud workspace has no Unreal installation; native tests cannot establish
Unreal runtime, rider-view or packaged-build readiness.

## Baseline comparison

Fresh Linux Release generation from unmodified `5dab7c0`, seed 42, highlands:
accepted in 28.736 seconds; 8,594.98 m; 181.818 seconds ride duration; 1.397 seconds
to 180 km/h. Clifftop active time was 26.247 seconds and longest level coast was
5.767 seconds. Local comparison artifacts are in ignored `out/rewrite/before*.json`.
These measurements describe the old generator, not acceptance of the new ride.

Current verification and limitations belong in the rewrite PR and verification
record. Do not reuse historical receipts as proof for changed geometry.

Final native results and limitations: [rewrite verification](docs/diagnostics/rewrite/README.md).
