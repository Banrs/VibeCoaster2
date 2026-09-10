# Frozen v16 final regression panel

Six physics-proof requests; hills9 is constrained to one candidate, other cases allow four. At most two panel CLI processes ran simultaneously. Frozen CLI SHA256 `b50b167b10f05018463de50c817a1a5c502123423ee7529f85cb48b16e12fef0`. Build identity is retained in `identity.json`. No isolated performance claim is made.

All six requests accepted candidate0, completed, passed 960/1920 Hz convergence and successfully saved after independent geometry/replay checks. No source/test edits or shared rebuilds were performed by this task.

| Case | Length m | Moving to final brake s | Complete stop s |
|---|---:|---:|---:|
| flat42 | 7690.652 | 140.027 | 164.755 |
| hills9 | 7896.611 | 143.993 | 168.716 |
| canyon0 | 9640.330 | 168.840 | 204.953 |
| canyon1 | 8834.421 | 156.302 | 189.371 |
| canyon9 | 10210.588 | 172.440 | 207.575 |
| canyon42 | 9174.464 | 162.285 | 197.316 |

## Sampled terrain and terminal-speed checks

Terrain metrics independently evaluate the exact analytic seeded landscape at exported 5 m geometry samples. Heights are centerline above local ground, not support height or a continuous-extremum certificate. Terminal-speed comparisons use display-rate telemetry over the full any-car occupancy interval; full organic tests separately use their complete simulation frames.

| Case | Ascent max AGL m | Terminal max AGL m | Terminal measured minus hint m/s |
|---|---:|---:|---:|
| flat42 | nan | 14.285 | -0.005666 |
| hills9 | nan | 25.832 | -0.015500 |
| canyon0 | 18.050 | 52.785 | 0.242711 |
| canyon1 | 28.185 | 68.942 | 0.003438 |
| canyon9 | 91.237 | 44.273 | -0.268764 |
| canyon42 | 29.470 | 46.501 | 0.013631 |

Canyon1 meets the new60 m ascent fixture. Hills9 verifies the restored first-candidate contract under an explicit one-candidate budget. All displayed terminal-speed residuals are within0.5 m/s. These results establish this fixed native panel; they do not replace packaged POV review, broader seed coverage or whole-product completion.
