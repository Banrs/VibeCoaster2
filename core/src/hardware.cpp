#include "coaster/hardware.hpp"

namespace coaster {
HardwarePlan planHardware(const Track &track) {
    if (track.source.size() != track.elementEnds.size())
        throw std::runtime_error("Hardware requires complete source extents");
    HardwarePlan result;
    result.elementZone.assign(track.source.size(), -1);
    for (std::size_t i = 0; i < track.source.size(); ++i) {
        const auto &p = track.source[i];
        double begin = i ? track.elementEnds[i - 1] : 0, end = track.elementEnds[i];
        ActuatorKind kind;
        if (p.role == Role::Launch) {
            if (i != 0)
                throw std::runtime_error("Only the initial launch may use a cable catch");
            kind = ActuatorKind::CableLaunch;
            begin -= 10.4;
        } else if (p.role == Role::Ascent || p.role == Role::DownhillLaunch) {
            kind = ActuatorKind::LinearMotor;
        } else if (p.role == Role::Lip || isTerminal(p.role) || p.id == "plateau-weave" ||
                   p.id == "valley-carve" || p.role == Role::Camelback) {
            kind = ActuatorKind::Brake;
            if (p.id == "plateau-weave")
                begin = std::max(begin, end - 150);
            if (p.role == Role::Camelback)
                end = begin + .72 * (end - begin);
        } else {
            continue;
        }
        result.elementZone[i] = int(result.zones.size());
        result.zones.push_back({p.id, kind, begin, end});
    }
    return result;
}
DriveDelivery deliverDrive(const HardwarePlan &plan, std::size_t element, double s, double speed,
                           const std::array<Frame, 6> &cars, double requested) {
    if (!std::isfinite(requested) || !std::isfinite(s) || !std::isfinite(speed) || speed < 0)
        throw std::runtime_error("Nonfinite hardware demand");
    DriveDelivery result;
    if (std::abs(requested) <= 1e-7)
        return result;
    if (element >= plan.elementZone.size() || plan.elementZone[element] < 0)
        throw std::runtime_error("Drive command has no authored equipment zone");
    result.zone = plan.elementZone[element];
    const auto &zone = plan.zones.at(std::size_t(result.zone));
    if (zone.kind == ActuatorKind::Brake && requested > 0)
        throw std::runtime_error("A brake-only zone requested positive drive");
    if (zone.kind == ActuatorKind::CableLaunch && requested < 0)
        throw std::runtime_error("The launch catch requested reverse drive");
    double jacobianSum = 0;
    for (std::size_t i = 0; i < cars.size(); ++i) {
        const bool catchDrive = zone.kind == ActuatorKind::CableLaunch;
        if (catchDrive && i != 0)
            continue;
        const double x = catchDrive ? -1.9 : 0, height = catchDrive ? .05 : -.3;
        const double position = s + carOffsets[i] + x;
        if (position < zone.begin || position > zone.end)
            continue;
        const auto &f = cars[i];
        const double jacobian = dot(f.t, f.t + f.k * x + f.upS * height);
        if (!std::isfinite(jacobian) || jacobian <= .5)
            throw std::runtime_error("Invalid hardware reaction-point motion");
        jacobianSum += jacobian;
        ++result.engaged;
    }
    if (!result.engaged)
        throw std::runtime_error("Drive requested with no reaction point inside its equipment zone");
    // Equal physical force at each engaged reaction. The Jacobian converts
    // tangential force to generalized train force, preserving power exactly.
    result.forcePerReaction = requested / jacobianSum;
    result.force = result.forcePerReaction * result.engaged;
    result.generalizedForce = result.forcePerReaction * jacobianSum;
    result.power = result.generalizedForce * speed;
    result.residual = std::abs(result.generalizedForce - requested);
    return result;
}
} // namespace coaster
