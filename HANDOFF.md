# Faultline and the forward generator

The latest user direction, 6 November 2026, is to rewrite the generator
architecture rather than keep patching it, research coaster design, and add a
few strong ride-feel regressions. Forward progress takes priority over expanding
a defensive test matrix. The user also permits changing inherited constraints
for understandable architecture and evidence-supported force excursions
for better feel. The subsequent instruction explicitly rejects a modest refactor
and slower generation: replace the construction architecture and drastically
reduce generation time. The latest instruction grants creative freedom and rejects
keeping the old ride behind the new compiler. There is no blanket 5% acceptance
cutoff. Future hardware is not an assumption of greater human tolerance.

Starting point: `main` at `5dab7c0`, already the newest remote branch. Work is on
`codex/coaster-rewrite` for PR review. The earlier handoff is preserved at
[docs/checkpoints/default3-handoff.md](docs/checkpoints/default3-handoff.md).

## Design direction

- One forward `RouteDraft` owns geometry, frames and hardware. Persistent
  finite-train energy advances only across newly authored track; a short local
  forecast supplies the next FVD entry. Never rebuild the prefix to fit entry
  speeds or move previous sections to repair station closure.
- Curve compilation uses source error to choose canonical spans. Exact endpoint
  geometry and analytic frame derivatives preserve C4 centerline/C3 orientation.
  Spatial resolution follows approximation error instead of a dense fixed grid.
- Source-owned frames replace the global banking/restoration cycle. Boundaries
  inherit their complete geometry and frame state. Free spline frames balance
  actual load direction with smooth, held attitudes; deliberate overbanks
  select the positive-load branch without flipping unrelated airtime upside down.
- Freeze the route before independent physics, structure and clearance checks.
- Measure experience using elapsed replay time: airtime, contrasting load beats,
  inactive gaps and roll episodes.
- Development launches explicitly generate current source. Historical saves
  retain their identity and receive fresh validation when explicitly loaded.

## Product requirements

The 3.1 default is **Faultline**: a low ravine opening, one cliff reveal, an
original force-authored floating hill, an overbanked wall turn, and a
float/trench/countersteer finale aimed toward the station. The recipe grammar
allows optional and reordered acts. Legacy named reference profiles and saves
retain their recorded parameters; a new default does not require their catalog.
The [creative design record](docs/references/creative-redesign.md) distinguishes
authoring intent, abandoned ideas and measured results.

Keep the deliberate 0–180 km/h launch in about 1.4 seconds. Improve opening
flow, banking and connections, cliff pacing, drive placement and the return.
Use [manufacturer and authoring research](docs/references/generator-rewrite-research.md)
for Falcon's Flight and Tormenta Rampaging Run, distinguishing verified evidence
from inferred geometry. Consumer video overlays are not engineering force data.

Retain art assets, cameras, dimensional contracts, independent finite-train
physics, clearance and atomic saves. The new train art remains a proposal.
This cloud workspace has no Unreal installation; native tests cannot establish
Unreal runtime or packaged-build readiness. Use the offline browser review
for generated geometry, terrain, structures and front/middle/rear replay. It
shows sampled native output, not the Unreal scene. CLI code and atomic scene
export live under `native/cli`; the zero-dependency viewer is in `native/tools`.

## Baseline comparison

Fresh Linux Release generation from unmodified `5dab7c0`, seed 42, highlands:
accepted in 28.736 seconds; 8,594.98 m; 181.818 seconds ride duration; 1.397 seconds
to 180 km/h. Clifftop active time was 26.247 seconds and longest level coast was
5.767 seconds. Local comparison artifacts are in ignored `out/rewrite/before*.json`.
These measurements describe the old generator, not acceptance of the new ride.

Current verification and limitations belong in the rewrite PR and verification
record. Do not reuse historical receipts as proof for changed geometry.

The earlier 3.0 forward checkpoint accepted seeds 42, 7 and 99 in 7.53–7.75 seconds
on this Linux Release build, with 1,345 canonical knots for seed 42. All 20
component suites, targeted persistence/cancellation checks and fresh-load feel
checks passed at that checkpoint. Its lateral peak was 1.621 g against its 1.5 g
preference; independent duration/history assessment passed. Those receipts do
not establish acceptance of the changed 3.1 composition.

Faultline 3.1 accepts seeds 42, 7 and 99 in 3.01–3.17 seconds. Seed 42 is
5.746 km / 113.49 seconds, with 609 canonical knots and a 1.397-second launch.
All 20 component suites and three fresh save/load/feel paths pass. The actual
offline browser review includes terrain, supports, station and seat dynamics.

Current results and limitations: [Faultline verification](docs/diagnostics/faultline/README.md).
The [3.0 checkpoint](docs/diagnostics/rewrite/README.md) remains historical.
