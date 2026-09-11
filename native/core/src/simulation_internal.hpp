#pragma once
#include "coaster/coaster.hpp"
namespace coaster::detail {
// Internal open-source replay uses the complete ride's force integration with
// an explicit inlet speed and no installed operations. Track ports are level.
SimulationResult simulateSource(const Track&,const TrainConfig&,double inletSpeed,double step,Cancel cancel={});
ValidationReport validateSimulationForces(const SimulationResult&,const Limits&);
}
