#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Author physical roll on one ride-time programme, then compile it into the
// canonical frame. The signature owns a broad outward release at its crest.
std::vector<double> continuousRollTarget(const std::vector<double>& upright,const std::vector<double>& reference,size_t crest,double bank);
void authorBanking(Design&,const std::vector<Frame>&);
}
