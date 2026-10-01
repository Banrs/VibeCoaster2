# Retained design dimensions

User direction, 28 September 2026:

- Track gauge: **1.40 m between rail centres**.
- Overall train width: **at most 2.40 m**; **2.35 m is acceptable**.

All previous train designs and track-section studies are rejected. No bodywork,
wheel dimensions, seating layout, restraint, windscreen, row spacing, rail diameter,
backbone profile, or overhang constraint carries forward.

These are requirements for the next design. The frozen default.3 package and its
native validation envelopes still have their original dimensions. Adopt the new
dimensions in rendering, hardware, clearance and station fit together in the next
geometry edit. Development train markers show positions only, not a proposed train.

The subsequent [Exa track study](TRACK_STUDY.md) follows the user's direction to
use existing and concept Exa coaster references and sufficiently large rail radii.
The user's latest dimension is **105 mm outside rail radius (210 mm diameter)**. The study
uses **20 mm walls** and a **680 mm diameter spine**. These are recorded in
`track_study_profile.json`, not measured Falcon's Flight dimensions
or an adoption into the default.3 validation envelope. The crosshead axis passes
through the rail centres, with zero vertical offset.
The user also requested thicker pillars; the study now uses **750 mm diameter
columns**, with larger connection plates and bases.
Their clarification identifies the thin wrapper and contracted neck as the
connection problem. The cut-tube heads were also rejected. The current revision
retains the full column diameter beneath a flat load plate; separate 45 mm bearing
webs meet the narrower spine, with no wrapped tube or tapered neck. It remains
a proposal for visual review.
