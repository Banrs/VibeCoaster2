#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Preliminary authoring needs motion only; this result cannot certify a ride.
struct MotionResult { std::vector<Frame> frames; bool completed{},cancelled{}; };
bool validDriveParameters(const Operation&);
// Conservative spatial candidates in original operation order. Exact interval,
// lap and ramp checks still run in the integrator.
class DriveIndex {
    double cellWidth{64};
    std::vector<std::vector<size_t>> cells;
public:
    DriveIndex(const std::vector<Operation>&,double trackLength);
    const std::vector<size_t>& at(double wrappedDistance) const;
};
MotionResult simulateMotion(const Track&,const std::vector<Operation>&,const TrainConfig&,double step,Cancel);
void verifyConvergenceWith(Design&,const std::function<SimulationResult()>& fineReplay,Cancel);
// Independent spatial work reads only the frozen geometry/operations/structures.
// Compare its result after the primary seat replay is available.
struct SpatialReplay {SpatialAssessment assessment;SimulationResult simulation;ValidationReport report;};
SpatialReplay replaySpatialRefinement(const Design&,Cancel);
void verifySpatialRefinementWith(Design&,const std::function<SpatialReplay()>&,Cancel);
}
