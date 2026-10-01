# Geometry inspection cameras

The repository and desktop Play shortcuts now launch the current development
game with these controls, using the separate UserData-GeometryReview profile.
The frozen standalone package remains recorded in dist/current.json.

From the repository root, launch the built editor game with:

```powershell
./native/unreal/scripts/inspect_geometry.ps1
```

This creates a separate `UserData-GeometryReview` profile from the verified
baseline on first use. Later launches retain that review profile's save.

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
