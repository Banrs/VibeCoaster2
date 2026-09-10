# Optimized runtime HUD smoke review

The current editor-game binary loaded the accepted converged flat seed 42 save and passed the existing load-only verification harness. Result: automated-smoke-passed, complete ride time 166.193750 s, final distance 7790.677335 m. Pause, restart, paused seat/overview pose, cross-process load, missing-reference refusal and pre-commit save cancellation passed. The copied save remained unchanged. Invocation and module/save hashes are in invocation.json; machine results and capture hashes remain in capture/result.json and capture/events.jsonl.

Six actual 2560 x 1440 captures were opened and inspected:

- capture/00-missing-reference.png: setup and two-line missing-reference refusal are legible; bottom amber status remains inside its panel. The refusal itself uses the generic multiline helper, while the bottom status uses the optimized pre-parsed path.
- capture/01-overview.png and capture/02-station.png: two amber accepted-status lines, active seed row and four telemetry lines have consistent spacing and fit within the background panel.
- capture/04-pov-1.png and capture/18-pov-15.png: status and telemetry remain legible at the tall crest and heavily banked low turn, without overlap or clipping against the sky/terrain.
- capture/22-terminal.png: the one-line save-cancellation status is legible; the panel and following seed/telemetry rows correctly contract by one line relative to the accepted two-line status.

No HUD regression was observed in these six captures. This is a focused current editor-game review, not a visual comparison to a separately captured before binary. Telemetry-off toggling, accepted setup display and keyboard input were not exercised. The load-only harness does not verify ordinary save success. No packaged performance, complete ride-feel, whole-layout or terrain-coherence acceptance is claimed. The overview still exposes the known sparse macro footprint.

All 26 portable CTest suites also passed with two workers in 406.46 s; see ../ctest.log. CTest and the editor ran concurrently during part of this verification, so elapsed times are validation observations rather than benchmark comparisons. The earlier package and historical evidence were not modified.
