#pragma once
#include "coaster/track.hpp"

namespace coaster {
inline constexpr std::array<double, 6> carOffsets{-8.5, -5.1, -1.7, 1.7, 5.1, 8.5};
inline constexpr double trainMass = 6 * 1500;
enum class ActuatorKind { CableLaunch, LinearMotor, Brake };
struct ActuatorZone {
    std::string id;
    ActuatorKind kind;
    double begin{}, end{};
};
struct HardwarePlan {
    std::vector<ActuatorZone> zones;
    std::vector<int> elementZone;
};
// Equipment extents are derived from authored element roles, independently of
// instantaneous demand. Coast elements cannot gain drive authority from a save.
HardwarePlan planHardware(const Track &);
struct DriveDelivery {
    int zone{-1};
    unsigned engaged{};
    double force{}, forcePerReaction{}, power{}, generalizedForce{}, residual{};
};
// Fin reaction height -0.3 m; rear launch catch at x=-1.9, z=0.05 m.
// Uses virtual work at actual offset car positions, without resetting speed.
DriveDelivery deliverDrive(const HardwarePlan &, std::size_t element, double s, double speed,
                           const std::array<Frame, 6> &cars, double generalizedForce);
struct HardwareDemand {
    double peakForce{}, peakReactionForce{}, peakPower{}, positiveWork{}, absorbedWork{},
        maximumWorkResidual{};
    unsigned minimumEngaged{};
};
} // namespace coaster
