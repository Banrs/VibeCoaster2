#pragma once
#include "authoring.hpp"

namespace coaster {
// Append a planar C3 turn/straight/turn approach to an exact station port.
// Radius follows entry energy; closure never moves already authored geometry.
void closeRouteApproach(MotionBuilder &, Vec3 endPosition, double endHeading, const char *name,
                        Cancel = {});
} // namespace coaster
