# Runtime verification

The fresh Unreal 5.8.2 module is a development preview. Its nominal source,
physics and terrain/track checks do not yet constitute the complete release
acceptance pipeline. The UI deliberately labels full validation as pending.

The authoring overview shows a depth-tested orange FVD trace and blue spatial
spline trace. These screen-width lines are an inspection overlay and are hidden
in the front/rear views. Physical rails retain their actual mesh dimensions.

## Reproducing development checks

Build `VibeCoasterEditor` with `-MaxParallelActions=1`. Prepare the owned map and
verified vertex-colour material using `scripts/make-content.py` through Unreal's
Python commandlet. Asset preparation with `-nullrhi` is not GPU verification.

Run the project in a real window with `-game`, a separate `-UserDir`, and:

- `-VibeLoad=<absolute saved design>` selects the source fixture.
- `-VibeVerify=<new absolute output directory>` records JSONL events and images.
- `-VibeCycles=N` repeats saved loading; the supported range is 1–2000.
- `-VibeView=0`, `1` or `2` selects overview, front or rear inspection view.
- `-VibeRideVerify` traverses the complete ride from front and rear, recording
  rendered-frame counts and fifteen captured points per pass.
- `-VibeQuit` exits after completion; failure uses a nonzero process status.

Freeze each benchmark fixture and record its hash, executable/module identity,
asset identity, commit, display configuration and background workload. Keep
process startup, initial UI readiness, requests, native validation, mesh work,
material readiness and GPU completion as separate measurements.

## What the GPU marker establishes

The marker follows complete game-thread material maps, verification of the
same material's complete map on the render thread, actual back-buffer frames
from the game window, and a GPU fence. It does not use elapsed sleeps as a
substitute for readiness. A material-preparation timeout rejects the request.
The previous scene remains visible during CPU/material preparation and is
retained until the replacement is acknowledged by the GPU.

Early preview fences were invalid performance evidence: they allowed fallback
materials. Visual inspection caught this. The asset script also used an invalid
named vertex-colour output; it now checks every graph connection and saves the
verified graph. The runtime now releases its retained material reference before
UObject teardown, after removing the render callback and draining render work.

## Current evidence and limits

The earlier complete traversal in `development-traversal-events.jsonl` records
both 191.94-second passes and roughly 45,000 rendered frames each, followed by
normal completion. It predates the latest terrain and inspection-camera edits;
it is evidence of the development traversal mechanism, not final release proof.

Recent nominal-only preview loads are around 2.0–2.2 seconds, with a clean exit.
The first request after rebuilding the material required 5.77 seconds; that
shader-compilation tail is retained in the local run record. These few samples
are not a p99 claim. Full operating/refinement activation checks, complete scene
clearance, cancellation flows, sufficient cold/warm samples and packaged
executable/Play identity remain required before release acceptance.
