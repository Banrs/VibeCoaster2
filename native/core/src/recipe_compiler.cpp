#include "route_plan.hpp"
#include "authoring.hpp"
#include "banking.hpp"
#include "simulation_internal.hpp"
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace coaster {
double departureRamp(double acceleration, const Limits &limits) {
    return std::isfinite(limits.maxLongitudinalRateGps)
               ? std::max(.08,
                          1.875 * acceleration / (gravity * limits.maxLongitudinalRateGps) * 1.02)
               : .43;
}
double plannedDepartureSeconds(double motorAcceleration, const TrainConfig &train,
                               const Limits &limits) {
    constexpr double dt = .0005;
    double speed = 0, time = 0;
    const double drag = .5 * train.airDensity * train.dragCdA / (train.cars * train.carMass),
                 ramp = departureRamp(motorAcceleration, limits);
    auto acceleration = [&](double v, double t) {
        const double external = motorAcceleration * smooth(t / ramp) - drag * v * v,
                     friction = gravity * train.rollingResistance;
        return v > 0 ? external - friction : std::max(0., external - friction);
    };
    while (time < 10) {
        const double a = acceleration(speed, time), mid = std::max(0., speed + a * dt * .5),
                     next = std::max(0., speed + acceleration(mid, time + dt * .5) * dt);
        if (next >= 50)
            return time + dt * (50 - speed) / (next - speed);
        speed = next;
        time += dt;
    }
    return std::numeric_limits<double>::infinity();
}

Design compileRoute(const GenerationRequest &request, Cancel cancel, Progress progress) {
    std::vector<RecipePort> ports;
    auto route = composeRoute(request, ports, cancel, progress);
    if (progress)
        progress(
            {WorkPhase::Motion, 0, 0, 0, "Replaying the finished route once for frame authoring"});
    const auto motion = simulateMotion(route.track, route.operations, request.train,
                                       request.simulationStep, cancel);
    if (motion.cancelled || !motion.completed || !motion.report.valid()) {
        route.simulation.cancelled = motion.cancelled;
        route.simulation.report = motion.report;
        throw RecipeCompileFailure("Forward-authored circuit did not complete its motion replay",
                                   std::move(route), ports);
    }
    double portError = 0;
    for (const auto &port : ports)
        portError =
            std::max(portError, std::abs(replayValueAt(motion.frames, port.distance) - port.speed));
    if (progress)
        progress(
            {WorkPhase::Authoring, 0, 0, 0, "Compiling C3 rider frames between source boundaries"});
    authorBanking(route, motion.frames);
    std::ostringstream diagnostics;
    diagnostics << std::setprecision(12)
                << "{\"schemaVersion\":6,\"generationOnly\":true,\"architecture\":\"forward-draft-"
                   "incremental-energy\","
                << "\"constructionPasses\":1,\"maximumPortSpeedError\":" << portError
                << ",\"canonicalKnots\":" << route.track.knots.size()
                << ",\"composition\":" << route.planningDiagnostics << '}';
    route.planningDiagnostics = diagnostics.str();
    return route;
}
} // namespace coaster
