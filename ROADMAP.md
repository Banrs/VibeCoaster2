# VibeCoaster direction

The product goals are ride specification using real coaster elements, layout
editing and VR riding. The generator redesign is scoped in `IMPLEMENTATION_PLAN.md`;
full layout editing and VR delivery are outside its six checkpoints.

## Principles

- Treat target dials as setpoints with stated tolerances and report achieved values.
- Use 10 km/h speed steps and 5 m height steps.
- Build from real elements: airtime hills, camelbacks, wave turns, overbanks,
  hammerheads, Immelmanns, dive loops, vertical loops, zero-g rolls, launches and brakes.
- Derive propulsion from design speed and use boosters to restore a defined fraction
  of that speed.
- Improve forces and pacing through layout changes, avoiding long stretches with
  no change in rider sensation.
- Rewrite or replace generator architecture and supporting systems that are buggy
  or conflict with the ride vision. Existing simulators, solvers, element systems
  and APIs are replaceable; verify the resulting ride behaviour.

## Customisation

| Defined scope | Player controls |
| --- | --- |
| Implemented in 0.8.4-layout.1 | Seed, mode, height, speed, inversion height, launch time, candidate budget |
| Request-UI expansion; no implementation milestone assigned | Inversion count and type, coaster style, terrain |
| Layout-editor expansion; no implementation milestone assigned | Element order, per-element parameters, saved-layout editing |

Element customisation will require a named, ordered element list instead of a fixed
sequence in `candidate()`.

## Sensation frequency

Use Falcon's Flight as a pacing reference, without claiming numerical equivalence
until a comparable per-sensation target is established. The intended direction is
more airtime while holding banked and inverted time steady or slightly lower.
