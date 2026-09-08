# Texture-free terrain relief palette proposal

Read-only presentation review actually viewed all ten final p2 gameplay PNGs, including full-size inverted POV 17. The current plain tan ground leaves shelf and wall difficult to distinguish; screenshots 03 and 05 show the cliff largely as a dark slab, while low-pass frames 11, 12 and 13 lack local scale. The transparent windshield and connected webbing read correctly. Tall tower density is visible at inversion 17; altering towers would require new canonical support work, not an art-only thinning.

## Bounded change

Create a separate `/Game/Materials/M_Ground_Relief` through `native/unreal/scripts/create_terrain_palette_v073.py`. Preserve all old assets. Keep the existing flat shelf color (linear RGB .120, .105, .085). Blend toward restrained cool-grey rock (.073, .081, .092) as slope grows from 20 to 65 degrees. This is a continuous slope palette, with no spatial texture, noise, stripes, emissive, displacement, normal map or new geometry. It cannot supply missing close optical-flow objects or repair distant geometry faceting.

PixelNormalWS reads the rendered canonical terrain normal; the reflected core Y axis leaves world Z unchanged. Absolute normal Z preserves two-sided material behavior. Existing distant canyon normals fade to upward outside the dense halo, so that region will smoothly return to shelf color. Do not claim a geology classification or modify normal/terrain fields to obtain the colors.

## Exact graph and readback

Five expressions: PixelNormalWS -> one-input Custom float3 -> BaseColor, plus constant Roughness .95, Metallic 0 and Specular .20. Opaque DefaultLit, two-sided as before. Validate exact code SHA, five-expression count, named input and its connected class, scalar values and absent extra material properties before saving. Existing assets are only validated; differing graphs are refused and preserved. Each invocation writes a new receipt under `Saved/TerrainPalette/<UTC timestamp>-v073/`; no runtime promotion occurs in this script. Python syntax compilation passed locally; actual UE execution and visual review remain root-owned and pending.

## Actual screenshot review targets

1. Canyon 42 front, original p2 frames 03/05: slope palette should distinguish the wall from the shelf without introducing patchy detail or hiding rail/support silhouettes.
2. Canyon low-pass 11/12/13: shelf should remain the same restrained untextured ground; no shimmer, striping or fake surface motion.
3. Canyon overview and inverted 17/20: inspect the dense/coarse terrain transition for a new color halo, severe dark cliff silhouettes or seams. Reject a distracting result rather than increasing contrast repeatedly.
4. Flat terrain representative POV: because normal Z is one, shelf color should be unchanged. Hills may receive subtle coloring only on sufficiently steep slopes.

The parent also received the paused-car transform/MarkRenderStateDirty finding; runtime optimization is owned by another worker. No game, editor or Blender process was launched by this reviewer.
