# Bounded self-track assessment

The runtime checks nonadjacent central chords against an unchanged 6 m minimum. Chord endpoints are sampled at nominal spacing at most2 m; the 16 m hash grid tests neighboring cells. Pairs whose starting distances differ by less than 12 nominal metres, including the closed seam, are classified as the same local rail and excluded. This is a central-clearance model, not a complete train-versus-every-triangle collision solver.

## Closed sampling gap for checked pairs

It is insufficient to assume that 2 m of the quadrature distance coordinate is exactly 2 m of true arc. The new `chord_validation` helper instead takes the actual canonical parameters returned by `Track::locate` for each endpoint. It integrates the shared certified cell bound M du over their interval, using partial cells at the ends. One monotone cursor visits every cell once: O(cells+chords), with cancellation. No difference of large cumulative integrals is needed. Summation/interpolation receives a conservative floating reserve.

The runtime rejects `TRACK_SAMPLING_DOMAIN` when that upper true-arc bound exceeds 2.1 m. It does not change the 6 m threshold. Track rebuild already bounds continuous geometric curvature by .2 per metre using derivative Bernstein controls and positive speed projection. For a unit-speed curve of arc length L, the maximum displacement from its straight endpoint chord is at most κ L²/8; therefore each checked chord deviates by at most .11025 m.

A complete body lies within 4.2 m of its canonical origin, and every other rail/spine/tie corner within .9 m of its origin. For checked pairs,6 −4.2 −.9 −2(.11025) = .6795 m of conservative separation remains. The argument covers different car positions because every car origin traverses the same canonical curve. It does not require approximate arc inversion to be physically exact.

The grid is also conservative under this chord bound: a chord's length is at most 2.1 m, so a pair closer than 6 m has midpoint separation at most 8.1 m. Such midpoints must lie in the same or adjacent 16 m cells. No distant candidate pair is lost through that broad phase.

## Explicit local adjacency scope

The 12 m wrap-distance exclusion remains an interpretation of the same nearby piece of rail. Local rail, wheel and chassis contact cannot be judged by blindly applying a spherical body-versus-hardware separation test to its own rail. The current code does not explicitly label contact surfaces or test every excluded pair of hardware/body solids. Accordingly this work certifies the nonadjacent pairs that the model tests; it does **not** claim a complete continuous other-track intersection proof across that exclusion.

No unsafe accepted self-track counterexample was reproduced during this bounded review. The remaining limitation is the explicit local-adjacency model and missing contact classification, rather than an observed bypass being described as established fact. Support members and station parts receive the complete shared sweep and do not inherit this 12 m exclusion. A future refinement should classify intended wheel/rail contact and use actual canonical intervals for any narrower local exemption; it should preserve the current nonadjacent clearance threshold.

## Permanent verification

The terrain/chord suite compares actual subchords against the integrated upper arc bound, rejects deliberately coarsened3 m chords with the domain diagnostic and checks cancellation. These tests support the interval argument; they do not substitute for a release acceptance matrix. No earlier working-snapshot measurements are presented as this release's performance evidence.

No UE playback, visual acceptance, rendered-triangle proof or structural engineering certification is implied.
