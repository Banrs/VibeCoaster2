# Product vision

The eventual game is a coaster-design and refinement sandbox in the spirit of NoLimits 2, with fast automatic generation as a starting point and assistant. Several coasters can share one environment. There is no park-management simulation. Walking is limited to an optional, skippable queue-to-seat boarding sequence.

The current task is the basic generator. It does not implement the eventual editor, multiple-coaster project workflow, scenery tools or boarding interaction.

## Current generator direction

- Approximately 1.33 times FF scale and 1.2 times its speed: 300 km/h or slightly above. Preserve the extreme 0-180 km/h in 1.4 s launch, explicitly reconfirmed on 24 September.
- Target approximately 180 seconds with a richer opening, 25-30 seconds of active clifftop turns, a short brake/lip approach and a substantial high-speed return. Use Falcon's Flight and Tormenta Rampaging Run as references, with an original Riftwake ravine signature.
- Prioritize believable front/middle/rear seat force histories and progressive roll transitions. Reference force readings are approximate context; do not raise jerk or distort an element to achieve historical uplift percentages.
- Use genuine FVD force and physical twist transitions, inherited boundary derivatives, plausible element shapes and a compact terrain-integrated composition.
- Keep the supplied asymmetric planar camelback visually faithful with one uniform scale. Do not spend repeated iterations chasing small image-fit differences.
- Allow up to 5% over the user's project force/rate and pacing limits while retaining the independent F2291 assessment unchanged. Do not choose rounding or precision to conceal failures.
- Use configured train/drag settings. The lower-drag stress cases are not acceptance requirements.
- Keep authored inversion and signature shape/yaw intact through layout composition. Lower average height above local terrain, preserve swept clearance, and keep support sightlines open.
- Target p99 three-second loading, measuring process startup separately from saved-design validation. Use asynchronous work and report measured limits honestly.
- Delegate the hardest authoring, physics and Blender model work to Astra; use Sol/Luna for bounded implementation and research. Keep verification focused and commit meaningful GitHub checkpoints without unnecessary CI.
