#pragma once
#include "coaster/motion.hpp"
namespace coaster {
struct SplineIntent {
    double length{300}, headingChange{}, height{}, exitHeight{}, entryPitch{}, exitPitch{};
    double bankBias{}, rollTurns{};
};
Program terrainSpline(const State &, const SplineIntent &, const Cancel &cancel = {});
Shot shootSpline(const Program &, double spatialStep, const Cancel &cancel = {});
std::vector<Jet> replaySpline(const Program &, double spatialStep, const Cancel &cancel = {});
void validateSpline(const Program &);
} // namespace coaster
