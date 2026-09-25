# Scoped acceleration assessment — F2291-25

This is a numerical assessment of selected acceleration provisions of **ASTM F2291-25**, not whole-standard certification or a biomechanical safety finding. The selected seating case is upright, Class 4/5, with lower-body containment, backrest and headrest. Upper-torso braking, prone and extended negative-Z allowances are not selected. Seat/restraint construction and applicability still require the ride analysis.

## Provenance and numerical limits

The source is the [edition 25 document reproduction](https://studylib.net/doc/27870841/f2291-25). Figures 6–8 and 20 were visually checked in the reproduced original pages; HTML prose alone omits their plotted limits. Figures 6/7 are printed page 13, Figure 8 page 14, and Figure 20 page 20. No edition 26 values or historical project tolerance models are used.

The following coordinates are `(duration seconds, magnitude g)`; lines between coordinates are linear. The sign is applied separately.

| Axis and sign | Coordinates |
|---|---|
| +X | (.2,6), (1,6), (2,4), (4,4), (5,3), (11.8,3), (12,2.5) |
| −X, base case | (.2,2), (.5,1.5) |
| ±Y | (.2,3), (1,3), (2,2) |
| +Z, normal | (.2,6), (1,6), (2,4), (4,4), (5,3), (11.8,3), (12,2) |
| −Z, base case | (.2,2), (.5,1.5), (4,1.5), (7,1.1) |
| +Z, reduced after negative history | (.2,5), (1.5,5), (2,4), (2.5,2) |

The normal tails are horizontal. The +Z tail includes a “max. 40 s” annotation; exposure beyond 90 s is outside the stated treatment in §7.1.4.6. The implementation's conservative response to ambiguous +Z tail applicability is explained below.

## Implementation conventions

These are explicit numerical choices, not additional quotations or invented standard clauses:

- `AccelerationSeries` contains uniformly spaced **full simulation-rate** seat-fixed specific accelerations, including gravity. X/Y/Z map to longitudinal/lateral/vertical. Positive Z is eyes down. The producer owns both the time origin and terminal sampling convention. A sample is a node: N nodes span `(N−1) × stepSeconds`; the assessor never silently adds a terminal hold.
- The production entry point applies a causal fourth-order, single-pass 5 Hz Butterworth filter as two biquads. Bilinear prewarping uses `tan(pi × 5 × step)`, with Q values `1/(2 cos(pi/8))` and `1/(2 cos(3pi/8))`. Causality is an implementation interpretation of single pass. Each biquad starts in equilibrium at the first channel value; no startup samples are removed and no future samples or end padding are invented. This documents the state-initialization choice permitted by X2.4.
- Onset is the discrete least-squares slope through a centered 100 ms window. Window endpoints are interpolated when they fall between samples; ordinary samples and those endpoints have equal fitting weight. At 960/1920 Hz, the endpoints fall on real samples. Only complete windows are reported. Rolling sums avoid a separate regression over every window.
- Duration events are connected superlevel sets of piecewise-linear filtered data. Slice spacing is at most 0.1 g, with curve vertex magnitudes and the observed maximum also sampled. Entry/exit times use linear interpolation. Any interval below a slice separates its events; there is no relief-gap merge or hysteresis. Values beyond the 200 ms intercept have a dedicated cap-crossing assessment, so redundant slices above that cap are unnecessary to establish failure/review.
- Paired-axis assessment uses the signed quadrant radii at 200 ms. Intersections with an ellipse solve the quadratic along each interpolated segment, splitting at sign changes. Utilization is normalized radial magnitude, with 1 on the boundary. Sub-200 ms combined excursions are excluded by §7.1.5.1.
- X/Y reversal events must each be sustained. Timing is between opposite-sign peaks, using the last point of a preceding flat peak and the first point of the following flat peak. At exactly 200 ms the implementation conservatively applies the 50% signed peak limits because the two sentences of §7.1.6.1 leave equality ambiguous. Short intervening sign events do not become sustained reversals.
- Negative-Z history activates only after a strictly longer-than-three-second negative interval. The six-second reduction begins at the following upward zero crossing. A zero plateau delays that crossing. Overlapping reduction windows form their union. As a conservative applicability interpretation, the +Z ellipse radius also becomes 5 during those windows.
- For §7.1.7.2, qualifying history requires at least 200 ms at Z≤0. The first subsequent upward crossing of +2 must be at least 133 ms after the latest upward zero crossing. Brief returns to Z≤0 retain the qualifying history and update that departure; reaching +2 consumes it. This brief-return policy is explicit because Figure 20 does not illustrate that case. No +2 plateau is required.
- Comparison tolerances are numerical roundoff only: `1e-10 s` for duration equality and `1e-12 × max(1, limit)` for acceleration comparisons. The user's 5% project force/rate allowance is never applied here.

The raw-input and processed-input entry points are separate. `assessProcessedAccelerationF2291_25` supports independent boundary fixtures or already processed records; production calls `assessAccelerationF2291_25`. Cancellation leaves `performed=false`, `passed=false`, `cancelled=true`. Valid completed work sets `performed=true`; `passed` requires no failure or review-required diagnostics. Diagnostic ordering is deterministic by clause, rule, axis, sign, seat and interval.

## Scope boundaries and review diagnostics

Short single-axis excursions above the 200 ms intercept produce `impact-review-required`. They are not sustained-limit violations or accepted impacts. Appendix X11 describes a separate raw-data 10 Hz filtering and delta-V/average-acceleration procedure; this module does **not** implement that procedure or invent impact acceptance thresholds. Passing this module therefore makes no assertion that all impact hazards have been evaluated.

`positive-tail-review-required` conservatively marks a continuous +Z exposure above 1 g lasting more than 40 s. This is a scope guard, not an extrapolated numerical standard curve; stationary 1 g is exempt. `duration-over-90s-review-required` marks other continuous signed dynamic exposures exceeding 90 s. Review cases prevent this scoped assessment from passing until the scope/applicability is resolved.

The project's nominal Gz −1.5…+5, |Gy| 1.5, |Gx| 4.5, and component-rate 20 g/s checks remain separate. A uniform user-approved allowance of up to 5% applies to those project limits, with every exceedance of nominal reported; the F2291 curves, history, combined-axis and reversal checks are not relaxed. Filtered 100 ms onset is reported without treating it as that project rate limit. Filter startup, rider population, physical restraint suitability, angular accelerations, impacts, full ride analysis, measurement validation and the rest of F2291 are outside a blanket compliance claim.

## Verification

`native/core/tests/acceleration_tests.cpp` uses independent piecewise-linear and constant fixtures through the processed-data entry point, plus analytic ramp and frequency-response checks through production filtering. It covers signed curve vertices/interpolation, strict small exceedances, interpolated entry/exit times, genuine short relief gaps, sustained/impact boundaries, reversal timing, signed ellipses, history activation/expiry, zero-to-two timing, baseline preservation, arbitrary-rate onset windows, invalid input, cancellation and deterministic evidence.

The standalone MSVC C++20 `/O2 /W4 /fp:strict` run passed **235 checks**, with one compiler worker and no warnings. These include every reduced positive-Z vertex/interpolation, 91 s of filtered stationary equilibrium, and a 180 s independent onset-slope oracle. Production generation, fresh saved-file validation and numerical refinement now assess complete front/middle/rear traces at960/1920Hz. The archived Rift checkpoint evidence at D:/Coding/Codex/vibecoasterlegacy/workspace-deflation-20260925/old-docs/checkpoints/06-rift.json records the successful native corpus, trim/drag variants and packaged GPU delivery; the standalone fixtures alone do not establish ride acceptance or certification.
