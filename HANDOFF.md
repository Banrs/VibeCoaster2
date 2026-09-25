# Active handoff — Riftwake default.3 baseline

Updated 26 September 2026. This section records the user's latest direction for the next task and supersedes conflicting plans, instructions, completion claims, or forced design behaviour in earlier plans and checkpoint records. The user rates default.3 about 5/10 and default.4 about 3/10. Default.3 is the working baseline, not the finished design. This cleanup task records the issues; it does not research or implement the next redesign.

## Playable and source state

- Default.3 is active. The default.4 redesign was rejected and rolled back. The user identified constant roll and a late LSM in section 1 and an unfaithful clifftop, then requested the default.3 return.
- Branch: codex/legacy-authoring. Rollback commit 6a9a496 restores the complete native tree to default.3 baseline 6a68f85. The verified packaged game's recorded build source is 9631e488; the difference from the restored native tree is a benchmark helper's marker-file write, not compiled runtime C++ or Unreal config.
- dist/current.json and both root/desktop Play shortcuts select native/unreal/Packaged/run-20260924-231703-433 and UserData-Riftwake-V3 with -CoasterLoad. Package executable SHA256: DBB65AF1F0F6E9B54D05E2DB83B48994237F4E845580B244B609650B42A29EF4. Accepted save SHA256: D48ED31CF62621405EE43A859C3E6E1EA468E405B383217FC7FB3E84895A0CF7. Manifest, executable, save, shortcut targets and arguments were independently rechecked against out/default3-restoration.json.
- Default.3 front/rear packaged traversals previously passed automated load, pause, restart and full playback with the same package/save identity. The current cleanup did not launch the game or modify its package/profile. Manual keyboard testing and user styling approval are not established.
- Rejected default.4 packages, profiles, captures and backups were moved to D:\Coding\Codex\vibecoasterlegacy\rejected-default4-20260925. Its Git commit b92efe0 and tracked checkpoint 11 remain historical references. Do not treat their old acceptance receipts as current or user-approved.

## Latest user requirements for the next task

Treat every item below as open on default.3. Prior numerical acceptance is not proof of ride quality. Diagnose against the actual native outputs and game views, then make the smallest coherent changes that solve the user's design intent.

- Forces and flow: unnatural G forces; random flat resets and other flat sections; pitch, yaw and roll that get stuck at arbitrary angles; poor banking; missing intermediate elements such as S curves and useful connecting elements, at appropriate rather than excessive frequency; weak overall flow and coherence. Increase intensity where it belongs and reduce it where it does not, using sound coaster design principles.
- Reference fidelity: opening section and initial twisted drop are weak; the clifftop is not faithful to Falcon's Flight; the 180-degree return turn is in the wrong location and inaccurate in geometry and speed; the vertical loop and Immelmann do not faithfully scale Tormenta Rampaging Run's dynamics, especially yaw; the return after the inversions is poor.
- Drives and brakes: LSM boosters are misplaced and fail to use the available acceleration corridor, leaving dragged-out deceleration before a boost and an extended wait before the next element after it. Trim brakes are illogically placed. Measure these gaps on the actual replay and fix the ride's pacing and hardware placement together.
- Authoring and clearance: FVD authoring is weak compared with NoLimits 2 FVD and OpenFVD++, especially transitions between FVD and spline. Ground and other-track clearance detection is overly conservative, harms authoring and adds loading cost. Investigate false positives and cost before changing margins; maintain real physical clearance.
- Art and diagnosis: the coaster/train model obstructs the rider view. Provide a usable third-person camera and screenshots for geometry diagnosis, alongside rider views.
- Performance and product direction: reduce loading time as much as possible while retaining fresh validation. The long-term vision is a realistic, quickly generated coaster inside an amusement park where other coasters can coexist.

The 0–180 km/h launch in about 1.4 seconds is intentional, inspired by the now-closed Do-Dodonpa. Do not challenge or silently weaken it while solving the other issues.

## Research and verification to do in the next task

The user explicitly requested a handoff here, not research in this cleanup task. The next task should properly watch Falcon's Flight POVs and inspect its layout and 180-degree turn, inspect Tormenta Rampaging Run's layout and loop/Immelmann, and study relevant record-breaking coasters. Use the retained local reference audit and seek primary footage/layout evidence. Research NoLimits 2 FVD, OpenFVD++ and direct authoring guides to decide where FVD or spline fits and how transitions should behave. Mark uncertain inferences as such; do not force a layout from a single animation or consumer G overlay.

Inspect front, middle and rear force histories, speed/grade/bank/heading traces, hardware intervals, clearance results and actual rider/third-person screenshots. Check the opening, cliff, inversions, turnaround and return in sequence. If an old validation rule or hard-coded layout assumption conflicts with the user's latest design direction, investigate and revise it on evidence rather than letting it dictate a visibly poor ride. Keep independent physics, clearance and saved-design validation.

## Known performance limit and cleanup state

Default.3's final 100-warm packaged benchmark missed the 3-second target: request-to-ready median 3.150476 s, empirical p99 3.253885 s; process-to-ready median 4.849519 s, empirical p99 4.978452 s. All 100 warm runs exceeded 3 seconds. These are run-specific empirical statistics, not cold-cache or population p99 guarantees. See out/startup-final-100-20260925/evidence.md. The next task should prioritize measured reductions without skipping validation.

This cleanup used ten bounded Luna audits. The stale native CLI was clean-built from restored default.3 source; its SHA256 is 8404A86B59E2863C776C6F5724BBDCB86D3081050CEF105BC3ACAC928F230949. Post-archive exact-save validation also exited 0 with generator/runtime 2.0.0-default.3, accepted/completed true, candidate 0. The full component suite and Unreal package were not rerun for this documentation/workspace cleanup. An older CTest log predates the rebuild and records a motion_spline failure; a future source-changing task should run relevant tests. The active V3 art importer was renamed to import_v3_art.py with all tracked callers updated; Python, PowerShell and shell syntax checks passed.

## Archive and navigation

Rejected default.4 packages, profiles, captures, backups and stale graph snapshots are in D:\Coding\Codex\vibecoasterlegacy\rejected-default4-20260925; see archive-manifest.json and graphify-archive-manifest.json. Older experiments, scratch probes, snapshots, inactive profiles and superseded docs are in the adjacent workspace-deflation-20260925 folder; see its manifests. The earlier long handoff is archived as HANDOFF-before-deflation.md. These are reference records, not current instructions.

Start with README.md, this file and docs/CODE_MAP.md. The refreshed native/core Graphify index remains in graphify-out, and a separate Unreal Source graph is under native/unreal/Source/VibeCoaster/graphify-out. Use source-qualified explain queries from the code map instead of broad node dumps; confirm cross-module calls in source. For default.3 delivery details use docs/checkpoints/10-riftwake-completion.md; for rejected default.4 history use docs/checkpoints/11-riftwake-fvd-redesign.md. The active package, V3 profile, development profile and shortcuts remain in the workspace.