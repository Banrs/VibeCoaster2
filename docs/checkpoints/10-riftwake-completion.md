# Riftwake path, terrain and V3 delivery

Source/package checkpoint: `9631e488f93e71a1fa2056550264b7d1a31967ed` (`[skip ci]`). This closes the geometry and V3 integration reopened after the user rejected checkpoint09. Startup results are recorded separately below; a numerical acceptance is not a claim of user design approval.

## Requested geometry

The loop now shifts18m toward its exit side without reversing lateral direction and exits nearly parallel to its entry. The Immelmann reverses through its half-loop and rolls upright in its entry plane, with no added horizontal weave. Its physical force law includes the modest real lateral force needed during the roll. Neither camera offsets nor telemetry clamps produce this result. See [inversion research](../references/inversion-shape-research.md).

A180m,6m-high rollover gives those two elements physical spacing. The Immelmann recovery finishes20m above its own entry, allowing the short signature approach to bridge above the loop. The opposing45-degree signature wings remain. Explicit custom-yaw source requests keep their compatible nonplanar authoring branch.

The clifftop has six compact connected arcs, deeper hills and sustained banks. Complete position and frame derivatives pass through the joins. Level pitch at a curved crest and upright bank at a directional reversal are intentional; the prior flat transition plateaus are removed.

| Measurement | User-rejected resume06 | Delivered completion07 |
|---|---:|---:|
| Full ride |181.96s|181.82s|
| Active cliff shelf |27.05s|26.25s|
| Front-seat cliff lip interval |5.75s|2.69s|
| Actual whole-train cliff braking |3.18s|2.65s|
| Slow coast after braking to centre departure |2.62s|0.45s|
| Longest nearly-level, slowly changing shelf interval |1.53s|0.65s|
| Spatial mean rail-to-ground distance |47.41m|39.76m|
| Track within10m of terrain |30.6%|40.0%|

Braking/work and near-level diagnostics use the60Hz display trace; acceptance uses the native960Hz simulation, half-step convergence and independent spatial replay. Near-level means absolute pitch<3 degrees and absolute pitch rate<0.015rad/s, excluding the intentional curved crests.

Terrain is shaped around actual valleys and wing crests, with the signature terrace shifted away from the lower loop. LSM and pullout section means are4.5–5.7m; the new rollover averages7.4m, rising-bank spans9.5–10.0m and return hills10.1m. Signature means are17.8/15.5/14.7/11.5m. The deliberate upper approach bridge averages20.8m. The ≤5m spatial share improves15.9%→20.5%, still below the shorter historical route's~26%; tall signature elements remain deliberately elevated.

## Native evidence

`out/riftwake-completion-07.vcdesign`: seed42/candidate0,181.817708s, seven two-seat modules/fourteen riders,0–180km/h in1.397s. SHA256 `d48ed31cf62621405ee43a859c3e6e1ea468e405b383217fc7fb3e84895a0cf7`.

Errors are empty. Front/middle/rear acceleration histories, source replay, physical C3 joins, terrain/track/support/station clearances,960→1920Hz convergence and independent half-spacing reconstruction/replay pass. Swept-ground clearance is+2.56160m; spatial force relative error is5.04e-9. No force, clearance or terrain-domain gates were relaxed. Nearby seed43 accepts on candidate1 at181.5625s with the same full checks.

Evidence: `out/riftwake-completion-07-{report,trace,plan,recipe}.json`, `out/riftwake-completion-07-geometry-summary.json`, `out/riftwake-completion-07-geometry-evidence.md`, `out/riftwake-completion-nearby-43-*`, and `scratch/terrain-fit-completion/final-clearance-audit.md`. Earlier completion01–06 are rejected diagnostic history.

## Train, station and actual runtime

The actual Blender MCP kit includes a higher continuous bonnet, sculpted bodywork, connected seats/restraints and bogies, distinct lead/passenger modules, rail web, LSM and brake hardware. The seven physical rows provide14 seats; explicit legacy six-row saves stay12. Rigid four-car grouping remains explicitly deferred and the parked coupler prototype is not used at runtime.

The80-part canonical station includes covered queue/merge, seven correctly aligned holding lanes/gates, dispatch, unload, separate exit, stairs, lifts and an underpass. Independent review corrected stair/roof interference, the upper stair landing and the two lower accessible floor seams. The final floor vertices meet the decks at−4.2m. Native station tests cover seven-row and legacy six-row fixtures.

Fresh real-RHI V3 import validates21 runtime meshes and nine materials. Final evidence is `out/completion-v3-floorfix-import-receipt.json` and its matching engine log; the actual MCP export is `out/completion-v3-floorfix-blender-mcp.log`. V2 is retained for the user's old running editor-game.

The UE5.8.2 Development standalone builds, cooks and packages from the committed source. Packaged CoordinateContract, MeshContract and StationArtContract all pass. ImportedArtContract remains editor-only; current mesh bounds/materials were checked against the fresh import receipt and source contract instead.

Both actual2560×1440 packaged front and rear traversals pass: load, full traversal, pause/restart, paused camera poses and generation/mesh/scene/save cancellation. The geometry SHA1 is identical in both: `00BA13C0A10848CA46C9F477E7E5A4F80E4CB83F`; the accepted save remains unchanged. The assistant inspected the actual station queue/unload, higher bonnet and train formation, clifftop transitions, inversions, bridge and low signature captures. No material fallback or rail/terrain clipping was seen. Raw reports retain human design approval as pending and keyboard input as untested; automated actions exercised the runtime methods.

Runtime evidence: `out/completion-runtime-front/result.json` (62captures), `out/completion-runtime-rear/result.json` (63captures), and `out/completion-game-contracts-engine.log`.

## Startup

The standalone retains full fresh validation of loaded files. Refined simulation and refined spatial validation run concurrently and both finish before acceptance. The benchmark reports process-launch and request-to-ready separately, and checks three actual moving ticks after readiness. These markers are not proof of a presented frame.

The10-warm pilot after correcting the Windows LF profile marker measured request-to-ready median3.167s/max3.337s and process-to-ready median4.882s/max5.071s. Native validation median2.954s, mesh preparation0.098s and scene commit0.102s. Both3s targets are missed. The completed100-warm series gives request-to-ready median3.150476s, empirical nearest-rank p993.253885s and max3.300493s; process-to-ready median4.849519s, p994.978452s and max5.009111s. All100warm runs exceed3s on both measures. The separately recorded first process measured3.166820s request-to-ready and4.859317s process-to-ready. All101runs passed and matched the final save, geometry identity, duration, length,14riders and80station parts.

Final evidence is `out/startup-final-100-20260925/{summary.json,runs.jsonl,metadata.json,evidence.md}`. The benchmark ran on an AMD Ryzen9 5900X with Windows and real2560x1440 rendering. Native validation median2.936474s/p993.045395s remains the main request-stage cost; the median additional process-startup cost is1.700472s. These are empirical results on this machine, not a population/SLA guarantee. OS/driver caches are not cleared; the first process is reported separately and is not labeled a cold-cache result. The user's old game remains running.

## Playable delivery and rollback

`dist/current.json` now points to `native/unreal/Packaged/run-20260924-231703-433/package-manifest.json` and the fresh `UserData-Riftwake-V3` profile. Both root and desktop shortcuts target that verified standalone and load the accepted ride. The already-running old editor-game is not replaced in place; relaunch through the updated shortcut to use this version.

`out/completion-activation.json` records identities. Previous launch metadata, root/desktop shortcuts and accepted save are copied to `out/completion-activation-backup/`; the original `UserData-Development` save is unchanged. Earlier binaries and V2 assets remain preserved. No running user game was terminated.
