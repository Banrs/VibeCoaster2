#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Preliminary authoring needs motion only; this result cannot certify a ride.
struct MotionResult { std::vector<Frame> frames; bool completed{},cancelled{}; };
bool validDriveParameters(const Operation&);
MotionResult simulateMotion(const Track&,const std::vector<Operation>&,const TrainConfig&,double step,Cancel);
void verifyConvergenceWith(Design&,const std::function<SimulationResult()>& fineReplay,Cancel);
}
