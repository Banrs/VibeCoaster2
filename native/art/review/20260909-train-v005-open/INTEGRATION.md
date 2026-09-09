# Conventional train integration checkpoint — 2026-09-10

The original `REVIEW.md` records the asset worker's handoff before Unreal import. Its Conventional1 command subsequently failed and is retained as history. The active runtime uses **Conventional2/SM_TrainCar**, with the same original FBX geometry and nine opaque materials.

Import succeeded using `import-spec-v072-trainv005-open-v2.json` and full Unreal Editor startup with `native/unreal/scripts/import_art_and_quit.py`. Full startup supplies the required StaticMeshEditorSubsystem; the earlier commandlet invocation did not. The local receipt `native/unreal/Saved/ArtImport/v072_20260909_trainv005_open_v2/receipt.json` records `import-validated-not-integrated`, including the independent coordinate witness, section/material and bounds checks. Integration was verified separately afterward. The failed Conventional1 receipt and assets remain locally preserved and are not runtime dependencies.

The runtime source in this checkpoint matches the five UE C++ source/header hashes in `native/artifacts/generator-intent-v081/package-v1/source-after.json`. That intermediate package passed editor/game compilation, cooking and all seven UE contracts. Its flat/front and hills/middle runs completed traversal, pause/restart and save/load checks. The canyon/rear run was interrupted and is not counted as a pass. Static captured frames showed operation stators, brake fins and station equipment following the persisted physical zones; this does not establish mechanical equipment feasibility or continuous POV quality.

The numerical generator has since changed. Final-source packaging and representative gameplay verification remain pending; the published application remains 0.8.0-flow.1. The later 100% internal-resolution defaults have separate editor-profile evidence and were not present in that intermediate package. No new performance or completed-foundation claim is made.

The default car envelope is unchanged. Configurable car spacing/seat height are not yet reflected by this fixed artwork, and validated moving bogies/reaction hardware remain unfinished.
