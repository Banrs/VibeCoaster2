# VibeCoaster direction

The end game is a hybrid of fully authored NoLimits 2, a Planet Coaster 2 style builder, and a VR riding
game: the player sets the ride up from real coaster vocabulary and the generator delivers exactly that.

## Principles

1. **Targets are setpoints, not floors.** A dial the player sets is a value the generator hits within a
   stated tolerance, not a minimum it may exceed by a random margin. Today every target reads `>=` and the
   design speed is `target + rng(7..11) m/s`; that noise is a defect, not variety.
2. **Dials move in the units the player thinks in.** Speed in 10 km/h steps, heights in 5 m steps. The ride
   report states the achieved value and its error against the setpoint.
3. **Real elements only.** Every authored module is a real coaster element (airtime hill, camelback, wave
   turn, overbanked turn, hammerhead, Immelmann, dive loop, vertical loop, zero-g roll, launch, brake run).
   No invented composites: an element that climbs turning one way and descends turning back the other, with
   a hump in the middle, is not a coaster element and must not be authored.
4. **Propulsion is normalised.** One launch acceleration for the ride, derived from the design speed, at
   real LSM magnitudes (about 1.5-2.5 g, not 4 g). Boosters restore a defined fraction of design speed
   rather than a hardcoded value.
5. **Forces and pacing come from the layout**, not from letting the ride run long. Dead track is a defect:
   if the rider feels nothing on any axis for seconds at a time, the element sequence is wrong.

## Customisation roadmap

| Stage | What the player controls |
|---|---|
| now | seed, mode, height, speed, inversion height, launch time, candidate budget — as precise setpoints |
| next | inversion count and type; coaster style (steel hyper, launched, dive, wing...); terrain choice |
| later | element order and selection from the real-element vocabulary; per-element parameters; saved layouts |

Customisation implies the element vocabulary becomes data (a named, ordered, parameterised list) rather than
a hardcoded sequence in `candidate()`. Work done before that point should move toward it, not away.
