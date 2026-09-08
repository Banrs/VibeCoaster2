# Opt-in packaged runtime smoke verification

The Development-only harness drives the **same controller/world generation, playback, save and load methods** as the game. Without `-CoasterVerify`, ordinary gameplay is unchanged. This is a small initial packaged smoke run, not complete product acceptance or a replacement for actual POV review.

Launch the real Development executable with a **nonexistent absolute evidence directory** and a separate isolated user profile. The explicit `PhysicsProof` mode is mandatory; the intensity reference is not invented. For example, replace the executable path with the fresh package printed by the packaging script:

```powershell
& 'D:\Builds\fresh-package\Windows\VibeCoaster.exe' `
  -windowed -ResX=2560 -ResY=1440 -ForceRes -csvGpuStats `
  '-CoasterVerify=D:\Builds\verification\flat42-front-capture' `
  '-UserDir=D:\Builds\verification\flat42-front-profile' `
  -CoasterVerifyMode=PhysicsProof -CoasterVerifySeed=42 `
  -CoasterVerifyTerrain=flat -CoasterVerifySeat=0
```

Terrain options are `flat`, `hills`, `canyon`; seats are `0` front, `1` physical middle car, `2` rear. The exact viewport must become 2560×1440. Do not use NullRHI, a fixed timestep or time scaling. The output directory must not already exist; use unique names for every attempt and retain failures. `-UserDir` itself redirects Unreal's saved-data root. A generation run refuses an existing `Saved/Designs/Accepted.vcdesign` instead of overwriting it.

When no reference is configured, the run first verifies that the ordinary controller refuses the initial all-record request. It then explicitly requests physics-proof, waits for actual core acceptance and visible chunk commit, checks pause/restart, and traverses one complete accepted circuit at normal playback speed. Up to eleven viewport PNGs cover the initial state, overview, station, the highest terrain-relative point, intervals across the circuit, inverted-apex passage and terminal station. The harness saves through the normal asynchronous API, loads/revalidates, and compares canonical geometry identity and saved-file bytes. A rejected candidate or failed operation yields a nonzero process result; the harness never marks a ride accepted itself.

To test a **new process loading the prior isolated save**, use a fresh evidence directory, the same previously marked verification profile and `-CoasterVerifyLoad`. This branch calls the normal load path and performs the same complete traversal without resaving the source slot. It refuses an unmarked profile.

For a separate performance recording, use another fresh evidence/profile pair and add `-CoasterVerifyNoScreenshots`. That avoids screenshot readback/compression contaminating its measurements. After the stationary warmup, normal playback emits `wall-frames.csv`; when the build enables CSVProfiler it also captures the engine's real CSV. `-csvGpuStats` requests available GPU counters, but missing counters are not inferred. Screenshot runs also retain wall-frame data and must be identified as capture-overhead runs.

Evidence includes `environment.json`, ordered `events.jsonl`, a final `result.json`, screenshot PNGs with dimensions and SHA1 byte hashes, canonical geometry identity, saved-file hash, and actual playback/wall-time samples. Screenshot events distinguish requested trace time from the observation after readback. Hashes identify bytes/geometry; they do not certify image quality. An absent final result or unfinished capture means the run is incomplete even if an earlier log line succeeded.

An `automated-smoke-passed` result covers only this scripted path. **Human visual review remains pending** until the PNGs are actually viewed; full-motion comfort and ride quality require POV playback review. Physical keyboard/focus behavior, all cancellation phases, rapid replacement, corrupt-save handling, shutdown during work, representative seed coverage, strict-record acceptance and measured sustained 1440p/60fps remain separate checks. Raw game-loop wall times do not establish GPU timing or a 60fps guarantee. Read the actual engine CSV columns and retain the hardware/settings context when reporting performance.
