#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Presentation-only export of the actual geometry, replay and authored scene.
void writeRideTrace(const Design &, const std::string &path);
} // namespace coaster
