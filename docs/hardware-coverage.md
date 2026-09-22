# Modeled drive coverage and demand

Every assessed finite-train step now assigns drive work to an authored equipment
zone and to reaction points actually inside that zone. The plan contains the
initial cable launch, ascent and downhill linear motors, lip and terminal brakes,
and the three overspeed-control zones. Coast elements cannot acquire motor
capacity from a saved reference speed. Brake-only zones cannot supply positive
power, and commands with no engaged reaction point fail validation.

The six 1,500 kg cars share tangential force across engaged reactions. The
explicit model assumes motor/brake reactions 0.30 m below each car's track frame
and one initial launch catch 1.9 m behind the rear car at height 0.05 m. The
reaction-point velocity Jacobian converts those forces to generalized train work;
physical speed is never reset. These are force-model assumptions. Detailed
actuator solids, contact mechanics, coupler loads, thermal duty and manufacturer
capacity selection remain outside this preview's geometric equipment model.
The existing continuous scene proof covers the rendered train, occupant volumes,
supports, station, terrain and other track; it does not certify unmodeled machinery.

`vibe_audit` reports each zone's extent, peak total and per-reaction force, peak
mechanical power, supplied/absorbed work, minimum engaged reactions and virtual-work
residual for every operating and convergence scenario. No force/power rating is
invented or treated as manufacturer approval. At the default nominal recipe, the
initial launch requires about 394 kN and 17.64 MW peak mechanical power; the
inclined boost requires about 177 kN and 13.70 MW. These are required demands,
not electrical input ratings or evidence that equipment is commercially available.

Regressions reject uncovered drive and powered braking, check finite entry
engagement, and verify force allocation/power on a known curved reaction frame.
All nine representative recipes pass the complete fresh validation with coverage
checks in every assessed dynamics scenario. Existing magnitude, history, rate,
geometry and clearance thresholds are unchanged.

Primary mechanism references:

- [Intamin LSM Launch Coaster](https://www.intamin.com/product/lsm-launch-coaster/)
  describes speeds above 200 km/h and inclined or vertical lift launches.
- [S&S Air Launch Coaster](https://www.s-s.com/ride-pages/air-launch-coaster)
  describes a compressed-air launch mechanism relevant to the dedicated initial
  launch. It does not establish this design's requested speed/time capability.

Neither reference provides ratings or approval for this design. The retained
F2291-25 acceleration assessment remains scoped, not whole-standard certification.
