#pragma once
#include "coaster/coaster.hpp"

namespace coaster::detail {
// Internal layout brief used by the dedicated crossover fixture. Public
// generation keeps its existing request/API and does not mandate crossings.
Design generateRide(const GenerationRequest& request,bool requireCrossing,Cancel cancel={},
    std::function<void(int,const std::string&)> progress={});
}
