#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Resolve one physical revolution for an authored roll phrase.
std::vector<double> continuousRollTarget(const std::vector<double> &upright,
                                         const std::vector<double> &reference, size_t crest,
                                         double bank);
// Compile only unowned spline intervals. FVD and aligned hardware frames are
// immutable boundary conditions; independent replay assesses the final result.
void authorBanking(Design &, const std::vector<Frame> &);
} // namespace coaster
