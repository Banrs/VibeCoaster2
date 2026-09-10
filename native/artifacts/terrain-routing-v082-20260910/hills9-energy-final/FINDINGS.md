# Hills 9 first-candidate convergence repair

Frozen v15 CLI, physics-proof seed 9 hills, maxCandidates=1 reproduces a completed but rejected first candidate: AUTHORING_ENERGY residual 3.88309179922 m/s after the unchanged eight corrections.

Artifact-only instrumentation attributes the limit cycle to ordinary turn geometry feedback. Turn 0 alternates measured maxima 70.4225 and 67.3545 m/s; turn 2 alternates 37.0553 and 40.1528 m/s. Their source speed hints resize the geometry and therefore its terrain relationship. The loop apex follows this alternating geometry (23.4242 / 16.1169 m/s). The terminal turn already converges to 44.8417 m/s; it is not the cause.

The observed alternating points estimate a fixed-point slope near -1.67. At 0.75 relaxation the update slope is approximately 0.25 + 0.75*(-1.67) = -1.00, sustaining the cycle. At 0.5 relaxation it is approximately -0.34. This is a local diagnosis, not a proof for all seeds.

One production literal changes: ordinary-turn speed relaxation 0.75 -> 0.5. Terminal turn gain remains 1.0; every other authoring update, the eight-rebuild budget, acceptance tolerance and physical/convergence gates remain unchanged.

Exact line:

    for(size_t side=0;side<measuredTurn.size();++side)if(ports.ordinaryTurn[side])feedback.turnSpeed[side]+=(side==3?1.:.5)*(measuredTurn[side]-feedback.turnSpeed[side]);

The isolated full generation now accepts candidate 0 within maxCandidates=1. Final residual 0.359496406079 m/s at iteration 8; full 960/1920 verification passes. `damped-report.json`, `damped-plan.json`, and `damped-run.log` preserve that result. The unchanged first-candidate station regression and organic candidate-zero assertion are appropriate regressions; root must still run the broader affected panel after integrating the one-line delta.

No production source was edited by this investigation.
