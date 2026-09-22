# Hardware coverage work

Status: implementation and verification remain open. The current independent
finite-train model accounts for generalized drive work, but its drive authority
still needs explicit spatial hardware coverage and force/power reporting.

The intended split is a dedicated high-acceleration initial launch, powered
ascent and downhill LSM boost, speed-control brake zones and terminal brakes.
No extra launch is needed for the park return. Every positive or negative drive
command must be attributed to available hardware at the actual train positions;
source reference speeds must never reset physical energy.

Primary references consulted:

- [Intamin LSM Launch Coaster](https://www.intamin.com/product/lsm-launch-coaster/)
  describes speeds above 200 km/h and inclined or vertical lift launches. This
  supports the mechanism choice for the ascent and downhill boost, but supplies
  no force/power rating for this particular design.
- [S&S Air Launch Coaster](https://www.s-s.com/ride-pages/air-launch-coaster)
  describes its compressed-air launch mechanism. It is a relevant reference for
  the initial high-acceleration launch, not evidence that a manufacturer has
  approved this requested speed/time combination.

Any selected force, power, reaction-point and equipment-clearance values must
be explicit model assumptions, sized and checked against the simulated demand.
The geometric scene proof and retained F2291-25 scope are not hardware or
whole-standard certification.