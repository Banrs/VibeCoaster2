# Geometry inspection cameras

The current development game provides rider, train-follow and free cameras.
Fresh generation uses the isolated `UserData-GeneratorDevelopment` profile.
The frozen standalone package remains recorded in the local `dist/current.json`.

From the repository root, launch the built editor game with:

```powershell
./native/unreal/scripts/inspect_geometry.ps1
```

This generates a new ride with the current generator every launch. Use `-Seed 123`
to select a repeatable layout. F5 saves the accepted development ride; F9 explicitly
reloads it. A failed generation reports its failure and never loads an older save.

To compare the retained ride, pass `-Baseline`. That mode loads the separate
`UserData-GeometryReview` save and bootstraps it from the verified frozen profile
only when absent. Existing review saves are preserved. It requires the local
package selector and baseline, which are not included in a clean source checkout.

On Windows, recreate the Play shortcut for fresh generation with:

```powershell
./native/unreal/scripts/create_shortcuts.ps1 -Development -UnrealRoot 'D:/Games/Epic Games/UE_5.8'
```

Pass the Unreal root as a quoted path when it contains spaces. Use `-GeometryReview`
instead of `-Development` for the historical review save. Checked-in shortcuts
are machine-specific; creating a shortcut does not build the game module.

Direct game startup accepts `-CoasterGenerate [-CoasterSeed=42]` or `-CoasterLoad`.
Supplying both actions reports an input error. Starting without either opens the
setup menu. Runtime logs identify both the generator build and loaded geometry
version so comparisons cannot mistake a historical save for new generation.

Hide setup with Tab and close comparison with C before using inspection controls.

| Control | Action |
| --- | --- |
| V | Third-person train-follow view; press again for rider POV |
| F | Free view at the current camera position; press again for rider POV |
| Right mouse drag | Orbit in third person; look in free view |
| Middle mouse drag | Pan the third-person focus |
| Mouse wheel | Third-person distance; free-view forward/back zoom; first-person FOV (30-110 degrees) |
| W / S, A / D | Fly forward/back, left/right at 150 m/s |
| Q / E | Fly down/up in world space |
| Shift | Fly at 600 m/s; free-view scroll steps increase from 5 m to 20 m |
| H | Recenter inspection views; reset first-person FOV to 82 degrees |
| 1 / 2 / 3 | Return directly to front/middle/rear rider POV |
| M | Whole-circuit overview; press again for rider POV |
| Space / R | Pause/resume / restart playback in any view |

Third person follows the train centre while keeping a level horizon through
inversions. Free view stays independent of the moving train. Both work with
playback paused. Free view has no collision, allowing inspection through terrain
and structures. Camera movement is suspended while setup/comparison is open.

Small blocks mark each simulated car position while train art is absent. They
are diagnostic markers, not the next train or a physical clearance envelope.
