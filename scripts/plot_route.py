"""Visual evidence from native outputs; these plots are not GPU verification."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent.parent
rows = list(csv.DictReader((root / "out/track.csv").open()))
xyz = np.array([[float(row[k]) for k in ("x", "y", "z")] for row in rows])
s = np.array([float(row["s"]) for row in rows])
forces = list(csv.DictReader((root / "out/forces.csv").open()))
times = np.interp(s, [float(row["s"]) for row in forces], [float(row["t"]) for row in forces])

fig = plt.figure(figsize=(13, 10))
plan = fig.add_subplot(221)
plan.plot(xyz[:, 0], xyz[:, 1], color="#aeb9c9", linewidth=1)
points = plan.scatter(xyz[:, 0], xyz[:, 1], c=times, s=3, cmap="turbo")
plan.scatter(*xyz[0, :2], marker="s", s=55, color="black", label="Station")
plan.axhline(0, color="#99633c", alpha=.5, linestyle="--", label="Steep escarpment edge")
plan.set(title="Plan — time along the ride", xlabel="East (m)", ylabel="North (m)", aspect="equal")
plan.legend(fontsize=8)
fig.colorbar(points, ax=plan, label="Elapsed seconds", shrink=.75)

height = fig.add_subplot(222)
height.plot(times, xyz[:, 2], color="#075cdf", linewidth=1.5)
height.set(title="Actual authored height profile", xlabel="Elapsed seconds", ylabel="Elevation (m)")
terminal_index = next(i for i, row in enumerate(rows) if row["role"] == "terminal-overpass")
height.axvline(times[terminal_index], color="#a93636", linestyle="--", label="Terminal braking begins (lip time separate)")
height.legend(fontsize=8)
height.grid(alpha=.2)

view = fig.add_subplot(223, projection="3d")
view.plot(*xyz.T, color="#075cdf", linewidth=1)
view.set(title="Oblique geometry view", xlabel="East", ylabel="North", zlabel="Height")
view.set_box_aspect((np.ptp(xyz[:, 0]), np.ptp(xyz[:, 1]), 600))
view.view_init(elev=24, azim=-55)

axis = fig.add_subplot(224)
t = [float(row["t"]) for row in forces]
for key, label in [("frontZ", "Front"), ("middleZ", "Middle"), ("rearZ", "Rear")]:
    axis.plot(t, [float(row[key]) for row in forces], linewidth=.7, label=label)
axis.axhline(-1.5, color="#a93636", linestyle="--", linewidth=.8)
axis.axhline(5, color="#a93636", linestyle="--", linewidth=.8)
axis.set(title="Seat vertical forces (display samples)", xlabel="Seconds", ylabel="g")
axis.legend(fontsize=8)
axis.grid(alpha=.2)
fig.tight_layout()
fig.savefig(root / "out/route-review.png", dpi=135)

ref = np.array(json.loads((root / "docs/references/camelback-fvd-fit.json").read_text())["points"])
indices = [i for i, row in enumerate(rows) if row["role"] == "protected-camelback"]
part = xyz[indices]
direction = part[1, :2] - part[0, :2]
direction /= np.linalg.norm(direction)
x = (part[:, :2] - part[0, :2]) @ direction
z = part[:, 2] - part[0, 2]
fig, axis = plt.subplots(figsize=(10, 4))
axis.plot(ref[:, 0], ref[:, 1], color="#7a8795", label="Approved fit", linewidth=2)
axis.plot(x, z, color="#075cdf", label="Current candidate", linewidth=1.6)
axis.set(xlabel="Distance in plane (m)", ylabel="Height above entry (m)", title="Protected camelback silhouette", aspect="equal")
axis.legend()
axis.grid(alpha=.2)
fig.tight_layout()
fig.savefig(root / "out/camelback-review.png", dpi=130)
