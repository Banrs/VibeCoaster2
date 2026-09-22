#pragma once
#include "coaster/scene.hpp"

namespace coaster {
enum class VehicleMaterial { Shell, Seat, Metal };
struct VehiclePart {
    ClearanceObstacle shape;
    VehicleMaterial material;
    bool cylinder{};
};
struct VehicleGeometry {
    std::vector<VehiclePart> parts;
    // Includes every rendered part and four conservative seated occupant
    // volumes. These are modelling assumptions, not restraint certification.
    std::vector<ClearanceObstacle> occupied;
};
// Matches Unreal's MakeFromXZ basis: local Y is up cross forward. The force
// reporting lateral basis uses the opposite sign; do not reuse it here.
inline Vec3 vehicleVector(const Frame &frame, Vec3 local) {
    return frame.t * local.x - frame.r * local.y + frame.u * local.z;
}
const VehicleGeometry &vehicleGeometry();
} // namespace coaster