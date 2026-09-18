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

## Sensation frequency — the open question

The ride should match Falcon's Flight's *frequency* per sensation (airtime, banking, inversion), not its
length. Measured on the current build with `tools/sensation_mix.py`, over the core ride only (launch and
closing brake excluded, 186.7 s of a 199 s ride):

| sensation | share |
|---|---|
| banked over 25 deg | 36.9% |
| strong positive, over 2.5 G | 16.0% |
| airtime, under 0.4 G | 14.4% |
| steep, over 20 deg | 14.1% |
| heavily banked, over 60 deg | 13.1% |
| floater, 0.4 to 0.8 G | 10.5% |
| inverted | 3.2% |

Shares overlap deliberately: a banked turn pulling 3 G is both banked and a strong positive, and forcing one
label per instant would hide the overlap that makes an element read the way it does.

**This cannot currently be matched numerically.** There is no per-sensation telemetry for Falcon's Flight;
what is published is speed, height, length, duration and the three LSM stagings. Matching its frequency needs
either a stated target share per sensation, or a proxy built from its published element list. Until one
exists, the working direction is the one given: hold banked and inverted time at or slightly below where it
is, and raise airtime.

The layout's banked time is concentrated in four `banked-camelback-turn` corners of about 500 m each, roughly
a fifth of the circuit. Airtime is concentrated in the record hill crest and the FVD trio. The long recovery
corridors between them are where airtime can be added without lengthening the ride or adding inversions.
