# Product vision

The eventual game is a coaster-design and refinement sandbox in the spirit of NoLimits 2, with fast automatic generation as a starting point and assistant. Several coasters can share one environment. There is no park-management simulation. Walking is limited to an optional, skippable queue-to-seat boarding sequence.

The current task is the basic generator. It does not implement the eventual editor, multiple-coaster project workflow, scenery tools or boarding interaction.

## Current generator direction

- Approximately 1.33 times FF scale and 1.2 times its speed: 300 km/h or slightly above.
- Approximate per-element/phase force baselines: FF +15%, TRR inversions +8 1/3%, other comparable elements +10%. Negative forces use increased magnitude. Preserve all-seat comparisons and avoid whole-ride peak substitutions.
- Use genuine FVD force and physical twist transitions, inherited boundary derivatives, plausible element shapes and a compact terrain-integrated composition.
- Keep the supplied asymmetric planar camelback visually faithful with one uniform scale. Do not spend repeated iterations chasing small image-fit differences.
- Allow up to 5% over the user's project force/rate and pacing limits while retaining the independent F2291 assessment unchanged. Do not choose rounding or precision to conceal failures.
- Use configured train/drag settings. The lower-drag stress cases are not acceptance requirements.
- Continue solo; no further agent delegation for this recovery.
