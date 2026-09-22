#pragma once
#include "coaster/math.hpp"
#include <vector>

namespace coaster {
inline constexpr unsigned terrainRevision = 2;
// Fixed site: 5 m cells in the ride area, progressively coarser toward the
// distant horizon. Geometry and clearance use the same A-C triangle diagonal.
const std::vector<double> &terrainXCoordinates();
const std::vector<double> &terrainYCoordinates();
double siteElevation(double x, double y, double plateau = 210);
double ground(double x, double y, double plateau = 210);
Vec3 terrainShadingNormal(double x, double y, double plateau = 210);
double terrainUpperBound(Vec3 minimum, Vec3 maximum, double plateau);
} // namespace coaster
