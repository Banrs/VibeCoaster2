# Runtime verification

The Unreal 5.8.2 preview runs fresh source, nominal and operating dynamics,
separate spatial/temporal refinement, terrain/track and shared support/station
clearance checks before replacement. Physical motor coverage and release
verification remain open.

The overview uses a depth-tested orange FVD trace and blue spatial-spline trace.
These screen-width inspection lines are hidden in front/rear views; physical
rail dimensions remain unchanged.

## Reproduction

On Windows, `scripts/verify-runtime.ps1 -Mode Loads -Cycles 3` runs the existing
editor build with a new isolated output/profile directory. `-Mode Ride` verifies
both complete traversals; `-Mode Flow` checks generation/save/load/cancellation and
rejection. `-Executable` with `-Packaged` selects an already-built game package.
The script records hashes, source state, startup, completion-marker consistency,
process exit and required events. OS file/driver caches are explicitly uncontrolled.


Build `VibeCoasterEditor` with `-MaxParallelActions=1`. Prepare the owned map and
vertex-colour material with `scripts/make-content.py` through Unreal's Python
commandlet. Asset preparation with `-nullrhi` is not GPU verification.

Run a real window with `-game`, a separate `-UserDir`, and:

- `-VibeLoad=<absolute saved design>` selects the frozen source fixture.
- `-VibeVerify=<new absolute output directory>` records JSONL events and images.
- `-VibeCycles=N` repeats saved loading; the range is 1–2000.
- `-VibeView=0`, `1` or `2` selects overview, front or rear inspection view.
- `-VibeRideVerify` traverses the complete ride from front and rear, recording
  frame counts and fifteen captures per pass.
- `-VibeFlowVerify` exercises seed-77/intense generation, save/reload, restored
  authoring controls, cancellation during CPU work/upload/GPU handoff, and
  corrupt-load rejection. It checks that the prior ride stays visible and plays
  and renders forward. It invokes the same handlers as UI actions; this is
  automated runtime coverage, not manual mouse testing.
- `-VibeAutoLoad` optionally starts loading without enabling verification mode.
- `-VibeQuit` exits after completion; failures use a nonzero process status.

Freeze the fixture and record its hash, executable/module and asset identities,
commit, display configuration and background workload. Keep startup, initial UI
readiness, requests, native checks, mesh work, material readiness and GPU completion
separate. Working-tree preview identities must not be presented as packaged release
identities.

## Readiness and retention

The marker follows complete game-thread material maps, verification of the same
map on the render thread, actual back-buffer frames from the game window and a GPU
fence. It never substitutes elapsed sleeps for readiness. Material-preparation and
GPU-acknowledgement timeouts reject the request and restore the previous scene.

The previous ride remains visible during CPU/material preparation and is retained
until GPU acknowledgement. Cancellation at CPU, upload and post-visibility-commit
stages is exercised. A corrupted saved file also leaves the previous ride playing.
A late native save-cancellation test verifies that the existing file bytes survive.

Early preview fences allowed fallback materials and are excluded from final
performance evidence. Visual inspection also caught an invalid vertex-colour graph
connection. The asset script now checks connections and the runtime waits for the
actual shader map. Its retained native material reference is released after removing
the render callback and draining render work, before UObject teardown.

## Current evidence

The newest `scene-*-events.jsonl`, matching identities and summaries cover the
shared checked/rendered structures, vehicle parts and closed-station continuity.
Both complete traversals and the generation/save/load/cancel/rejection flow passed
with process exit 0. Every requested load includes those fresh scene checks.
The latest three loads were 2.434, 2.585 and 2.501 s; initial UI startup was 9.66 s.
These small development samples do not establish p99.

The following retained files describe the preceding development checkpoint:


`validated-traversal-events.jsonl` records both 191.94-second passes on the fixed
ravine route with the current terrain and cameras, about 45,000 rendered frames per
view, and normal completion. Its matching identity file records the fixture and
module hashes. These remain development preview results.

`validated-flow-events.jsonl` records generation, save/reload, control restoration,
all three cancellation stages and corrupt-load rejection. The process exits 0.
`runtime-retained-ride.png` shows the retained seed-77/intense ride after rejection.

Three current fully validated preview loads measured 2.387, 2.532 and 2.547 seconds
through GPU readiness. An earlier first request after material rebuilding took
5.77 seconds; that shader-compilation tail remains in the local record. These few
samples are not a p99 claim. Physical hardware coverage, sufficient process-cold/warm
samples, packaged builds and exact Play identity remain required for release.