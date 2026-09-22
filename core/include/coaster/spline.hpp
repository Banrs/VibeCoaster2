#pragma once
#include "coaster/motion.hpp"
namespace coaster {
struct SplineIntent {
    double length{300}, headingChange{}, height{}, exitHeight{}, entryPitch{}, exitPitch{};
    double bankBias{}, rollTurns{};
};
Program terrainSpline(const State &, const SplineIntent &, const Cancel &cancel = {});
// Route to a fixed site point using length and total heading change. The
// inherited entry frame and zero-curvature exit remain explicit spline ports.
Program splineTo(const State &, Vec3 target, double rollTurns, const Cancel &cancel = {});
Shot shootSpline(const Program &, double spatialStep, const Cancel &cancel = {});
std::vector<Jet> replaySpline(const Program &, double spatialStep, const Cancel &cancel = {});
void validateSpline(const Program &);
} // namespace coaster
