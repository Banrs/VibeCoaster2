# Serial MCP train crash isolation

Run each numbered file as its own `execute_blender_code` request through the actual MCP, safe mode enabled. Stop at the first failed/crashed request; do not concatenate stages. Python local state is not shared: persistent snapshots and the manifest live in namespaced Blender datablocks. The authoring agent has not launched Blender or executed these files.

Root precreates `native/art/source/train/20260908-train-v003/` and `native/art/exports/train/20260908-train-v003/`. Keep v001/v002 files and crash logs unchanged.

1. `01_construct.py`: original source construction only. No window scene change, explicit dependency-graph update, snapshot, export or save. Returns source counts.
2. `02_activate_scene.py`: changes the real window to the dedicated train scene only, then returns to the event loop.
3. `03_evaluate_layer.py`: checks current context, updates its layer and gets the dependency graph. No geometry mutation.
4. `04_snapshot.py`: copies evaluated source geometry into persistent independent meshes; no source IDs are replaced/freed. Each snapshot carries source name, world transform, pivot and runtime/review flag. If a partial snapshot stage survives a Python error, do not rerun it over those IDs; inspect first.
5. `05_assemble.py`: reads snapshots, asserts footprint/body bounds, builds the two assembled mesh objects and prints/stores the manifest. No forced dependency update or exporter call.
6. `06_save_source.py`: saves the local source `.blend` copy only.
7. `07` and `08`: runtime GLB then FBX, separate calls.
8. `09` and `10`: explicitly unvalidated bogie GLB then FBX, separate calls.
9. `11_review_visibility.py`: hides export duplicates/reference/bogie objects only after export, in a separate call.
10. `12_save_review.py`: saves final review visibility and prints the final manifest.

All 12 stages pass Python syntax and the installed `blender_mcp.safe_mode.validate_code` validator. This is stage isolation, not evidence that Blender execution succeeds.

## What the current crash establishes

Blender 5.2.1 crash log has `BKE_base_eval_flags -> BKE_object_eval_eval_base_flags`, reading address `0x24`, in dependency-graph evaluation. This narrows the failure to object/base flags in view-layer evaluation. It does not identify a bad polygon, prove exporter fault or prove a fix. The earlier 4.5.13 stack lacked symbols and crashed after both runtime exports; 5.2.1 reached no source/export output. Neither record proves that snapshot mutation was the root cause.

The official Blender object-evaluation implementation computes object base flags using view-layer base data, which matches the stack's subsystem. The official Context API says the evaluated dependency graph belongs to the current scene/view layer. Thus stage 02 and stage 03 intentionally separate scene activation from evaluation before any mesh-copy call. Source references: [Blender object evaluation](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/object_update.cc), [Blender Context API](https://docs.blender.org/api/4.5/bpy.types.Context.html). These are subsystem references, not a confirmed matching bug report.
