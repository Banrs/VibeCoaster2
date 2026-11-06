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

RecipeBootstrapResult bootstrapRecipeEnergy(const RecipeCompileFailure &failure,
                                            RecipeEnergyEstimate &feedback, Cancel cancel) {
    RecipeBootstrapResult result;
    auto cancelled = [&] {
        if (cancel && cancel()) {
            result.cancelled = true;
            result.report.fail("CANCELLED", "Provisional energy bootstrap cancelled");
            return true;
        }
        return false;
    };
    if (cancelled())
        return result;
    const auto &partial = failure.partial;
    const auto &train = partial.request.train;
    if (partial.track.closed || partial.track.knots.size() < 4 || !failure.pendingPort ||
        !std::isfinite(failure.pendingPort->speed) || failure.pendingPort->speed <= 0 ||
        std::abs(failure.pendingPort->distance - partial.track.length) > 1e-5) {
        result.report.fail("ENERGY_BOOTSTRAP_DOMAIN",
                           "Energy bootstrap requires an open prefix ending exactly at a typed "
                           "pending source port");
        return result;
    }
    const double half = (train.cars - 1) * train.spacing * .5;
    if (!std::isfinite(half) || half < 0 || half > 160) {
        result.report.fail("ENERGY_BOOTSTRAP_DOMAIN",
                           "Invalid finite-train coverage for energy bootstrap");
        return result;
    }
    // Open replay reserves half a train plus 1 m at its far boundary. Add 2 m
    // beyond that so the pending centre position is actually sampled. This is
    // a separate hypothesis used only to seed source energy, never route art.
    result.continuationMeters = half + 3;
    try {
        Track probe = partial.track;
        probe.closed = false;
        probe.authoredFrame = false;
        const auto &last = partial.track.knots.back();
        if (std::hypot(last.tangent.x, last.tangent.y) < .17)
            throw std::runtime_error(
                "Provisional angular continuation is not defined near a vertical tangent");
        const detail::MotionJet jet{last.position, last.tangent, last.curvature, last.third,
                                    last.fourth};
        const auto angles = detail::directionAngles(jet);
        const double length = result.continuationMeters;
        auto continued = [&](detail::AngleJet a) {
            return detail::AngleJet{a.value + a.first * length + a.second * length * length * .5 +
                                        a.third * length * length * length / 6,
                                    a.first + a.second * length + a.third * length * length * .5,
                                    a.second + a.third * length, a.third};
        };
        const auto program = polynomialMotion(
            jet, detail::anglePolynomial(angles[0], continued(angles[0]), length),
            detail::anglePolynomial(angles[1], continued(angles[1]), length), length);
        const auto first = program.direction(0);
        if (norm(first.tangent - jet.tangent) + norm(first.curvature - jet.curvature) +
                norm(first.third - jet.third) + norm(first.fourth - jet.fourth) >
            1e-7)
            throw std::runtime_error(
                "Provisional continuation did not inherit the complete spatial jet");
        const size_t count = size_t(std::ceil(length / .3));
        for (size_t i = 1; i <= count; ++i) {
            if (cancelled())
                return result;
            const double at = length * i / count;
            auto q = program.direction(at);
            q.position = jet.position + program.displacement(at);
            if (!finite(q.position) || std::abs(q.tangent.z) > .985 || norm(q.curvature) > .15)
                throw std::runtime_error(
                    "Provisional continuation left its short, nonvertical curvature domain");
            const auto up = unit(Vec3{0, 0, 1} - q.tangent * q.tangent.z);
            probe.knots.push_back(
                {q.position, q.tangent, q.curvature, up, 0, Element::Return, q.third, q.fourth});
        }
        probe.rebuild();
        if (cancelled())
            return result;
        auto replay =
            simulateMotion(probe, partial.operations, train, partial.request.simulationStep, cancel,
                           MotionReplayMode::StationEnergyPrefix);
        if (replay.cancelled) {
            result.cancelled = true;
            result.report = std::move(replay.report);
            return result;
        }
        if (!replay.prefixReachedEnd || !replay.report.valid() || replay.frames.empty() ||
            replay.frames.back().distance < failure.pendingPort->distance) {
            result.report = std::move(replay.report);
            result.report.fail(
                "ENERGY_BOOTSTRAP_REPLAY",
                "Finite-train provisional replay did not reach the pending centre position");
            return result;
        }
        auto corrected = feedback;
        auto observe = [&](const RecipePort &port) {
            if (port.distance < replay.frames.front().distance ||
                port.distance > replay.frames.back().distance || !std::isfinite(port.speed) ||
                port.speed <= 0)
                return;
            const double actual = replayValueAt(replay.frames, port.distance);
            if (!std::isfinite(actual) || actual <= 0)
                throw std::runtime_error(
                    "Provisional replay returned an invalid reached-port speed");
            result.observations.push_back(
                {port, actual, port.distance + half > partial.track.length});
            const double delta = actual - port.speed;
            result.maximumSpeedCorrection =
                std::max(result.maximumSpeedCorrection, std::abs(delta));
            if (std::abs(delta) >= .01)
                corrected.energyCorrection[port.id] += actual * actual - port.speed * port.speed;
        };
        for (const auto &port : failure.reachedPorts)
            if (port.id != failure.pendingPort->id)
                observe(port);
        observe(*failure.pendingPort);
        if (cancelled())
            return result;
        if (result.maximumSpeedCorrection < .05) {
            result.report.fail(
                "ENERGY_BOOTSTRAP_NO_PROGRESS",
                "Provisional reached-port correction is below 0.05 m/s; the failed source requires "
                "a different authored intent or a better resolved model");
            return result;
        }
        feedback = std::move(corrected);
        result.corrected = true;
        result.report.warnings.push_back(
            "Pending boundary speed is provisional: front cars used a short inherited-jet "
            "continuation. Real-source calibration and complete validation remain mandatory.");
    } catch (const std::exception &error) {
        if (cancelled())
            return result;
        result.report.fail("ENERGY_BOOTSTRAP", error.what());
    }
    return result;
}

namespace {
enum class PlacementStep { Converged, Advanced, Unavailable };
static PlacementStep solveClosureStep(const Design &d, RoutePlacement &placement,
                                      const std::optional<RecipePort> &pending, Cancel cancel,
                                      double &residual) {
    std::string cliffId, signatureId, approachId;
    for (const auto &e : d.request.recipe.elements) {
        if (e.role == RideRole::CliffApproach)
            cliffId = e.id;
        if (e.role == RideRole::Signature)
            signatureId = e.id;
        if (e.role == RideRole::Return && e.anchor == TerrainAnchor::Approach &&
            std::holds_alternative<SweepParameters>(e.parameters))
            approachId = e.id;
    }
    if (cliffId.empty() || signatureId.empty() || approachId.empty())
        return PlacementStep::Unavailable;
    auto findSource = [&](const std::string &id) {
        return std::find_if(d.forcePrograms.begin(), d.forcePrograms.end(),
                            [&](const auto &p) { return p.name == id; });
    };
    const auto approach = findSource(approachId);
    const auto signature =
        std::find_if(d.splinePrograms.begin(), d.splinePrograms.end(),
                     [&](const auto &p) { return p.name == signatureId + "-approach"; });
    const auto setup = std::find_if(d.splinePrograms.begin(), d.splinePrograms.end(),
                                    [&](const auto &p) { return p.name == cliffId + "-recovery"; });
    if (setup == d.splinePrograms.end() || signature == d.splinePrograms.end())
        return PlacementStep::Unavailable;
    double distance = 0, speed = 0;
    if (pending && pending->id == approachId) {
        distance = pending->distance;
        speed = pending->speed;
    } else if (approach != d.forcePrograms.end()) {
        distance = d.track.spans.at(approach->firstKnot).start;
        speed = approach->program.speed;
    } else
        return PlacementStep::Unavailable;
    const auto q = sampleKinematics(d.track, distance);
    if (std::abs(q.sample.tangent.z) > 1e-6 || norm(q.sample.up - Vec3{0, 0, 1}) > 1e-6 ||
        norm(q.sample.curvature) > 1e-6)
        return PlacementStep::Unavailable;
    const double hand = signature->hand;
    auto unmirror = [&](Vec3 p) {
        p.y *= hand;
        return p;
    };
    const Vec3 point = unmirror(q.sample.position),
               cp = unmirror(d.track.knots[setup->firstKnot].position),
               sp = unmirror(d.track.knots[signature->firstKnot].position);
    const auto tangent = unmirror(q.sample.tangent);
    const double heading = std::atan2(tangent.y, tangent.x);
    if (std::abs(heading) < .35 || std::abs(heading) > 2.5 || speed < 15 || speed > 80)
        return PlacementStep::Unavailable;
    const auto &train = d.request.train;
    const double rolling = gravity * train.rollingResistance,
                 drag = .5 * train.airDensity * train.dragCdA / (train.cars * train.carMass);
    using State = std::array<double, 4>; // x, y, heading, speed
    auto shortArc = [&](double yaw) {
        const double bank = std::copysign(std::acos(1 / recipeApproachNormalG), yaw),
                     sign = std::copysign(1., yaw);
        auto integrate = [&](double hold) {
            State state{0, 0, yaw, speed};
            const std::array<double, 5> durations{.4, recipeApproachBankRampSeconds, hold,
                                                  recipeApproachBankRampSeconds, .4};
            auto add = [](State a, State b, double step) {
                for (int i = 0; i < 4; ++i)
                    a[i] += b[i] * step;
                return a;
            };
            for (int phase = 0; phase < 5; ++phase) {
                const double duration = durations[phase];
                const int count = std::max(1, int(std::ceil(duration / .04)));
                const double dt = duration / count;
                auto rate = [&](const State &x, double u) {
                    const double smooth = u * u * u * u * (35 + u * (-84 + u * (70 - 20 * u)));
                    const double phi = bank * (phase == 1   ? smooth
                                               : phase == 3 ? 1 - smooth
                                               : phase == 2 ? 1
                                                            : 0);
                    return State{x[3] * std::cos(x[2]), x[3] * std::sin(x[2]),
                                 -gravity * std::tan(phi) / x[3], -rolling - drag * x[3] * x[3]};
                };
                for (int i = 0; i < count; ++i) {
                    const auto a = rate(state, double(i) / count),
                               b = rate(add(state, a, dt * .5), (i + .5) / count),
                               c = rate(add(state, b, dt * .5), (i + .5) / count),
                               e = rate(add(state, c, dt), (i + 1.) / count);
                    for (int j = 0; j < 4; ++j)
                        state[j] += dt * (a[j] + 2 * b[j] + 2 * c[j] + e[j]) / 6;
                }
            }
            return state;
        };
        double lo = .001, hi = 8;
        if (sign * integrate(lo)[2] < 0 || sign * integrate(hi)[2] > 0)
            return Vec3{NAN, NAN, 0};
        for (int i = 0; i < 24; ++i) {
            if (cancel && cancel())
                return Vec3{NAN, NAN, 0};
            const double mid = (lo + hi) * .5;
            if (sign * integrate(mid)[2] > 0)
                lo = mid;
            else
                hi = mid;
        }
        const auto end = integrate((lo + hi) * .5);
        return Vec3{end[0], end[1], 0};
    };
    const auto arc = shortArc(heading),
               derivative = (shortArc(heading + .001) - shortArc(heading - .001)) / .002;
    if (!finite(arc) || !finite(derivative))
        return PlacementStep::Unavailable;
    const auto &brakes = std::get<OperationParameters>(d.request.recipe.elements.back().parameters);
    const Vec3 target{-brakes.lengthMeters - (train.cars - 1) * train.spacing - 3, 0, point.z};
    const Vec3 error = point + arc - target,
               jc = Vec3{-(point - cp).y, (point - cp).x, 0} + derivative,
               js = Vec3{-(point - sp).y, (point - sp).x, 0} + derivative;
    residual = norm(error);
    if (residual < .25)
        return PlacementStep::Converged;
    const double determinant = jc.x * js.y - js.x * jc.y;
    if (std::abs(determinant) < 1)
        return PlacementStep::Unavailable;
    const double dc = std::clamp((-error.x * js.y + error.y * js.x) / determinant, -.2, .2),
                 ds = std::clamp((-error.y * jc.x + error.x * jc.y) / determinant, -.2, .2);
    const double prior = placement.climbHeading;
    const double nextCliff = std::clamp(prior + dc, -pi, pi),
                 nextSignature = std::clamp(placement.signatureHeading + ds, -pi, pi);
    if (std::abs(nextCliff - placement.climbHeading) +
            std::abs(nextSignature - placement.signatureHeading) <
        1e-7)
        return PlacementStep::Unavailable;
    placement.climbHeading = nextCliff;
    placement.signatureHeading = nextSignature;
    return PlacementStep::Advanced;
}

struct EnergyFit {
    double maximumPortError{};
    double peakSpeedError{};
};
EnergyFit updateEnergyPlan(const Design &route, const std::vector<RecipePort> &ports,
                           const std::vector<Frame> &frames, RouteEnergy &energy) {
    EnergyFit fit;
    for (const auto &port : ports) {
        if (port.distance > frames.back().distance)
            continue;
        const double actual = replayValueAt(frames, port.distance);
        fit.maximumPortError = std::max(fit.maximumPortError, std::abs(actual - port.speed));
        if (std::abs(actual - port.speed) >= .01)
            energy.portSquaredSpeedCorrection[port.id] += actual * actual - port.speed * port.speed;
    }
    double begin = INFINITY, end = 0, peak = 0;
    for (const auto &section : route.sections)
        if (section.role == RideRole::DownhillLaunch) {
            begin = std::min(begin, section.start);
            end = std::max(end, section.end);
        }
    for (const auto &landmark : route.landmarks)
        if (landmark.kind == LandmarkKind::CamelbackCrest)
            end = std::max(end, landmark.distance);
    for (const auto &frame : frames)
        if (frame.distance >= begin && frame.distance <= end)
            peak = std::max(peak, frame.speed);
    if (peak > 0) {
        fit.peakSpeedError = route.request.targets.speed + .15 - peak;
        if (std::abs(fit.peakSpeedError) >= .01)
            energy.downhillSpeedCorrection += fit.peakSpeedError;
    }
    return fit;
}
} // namespace

Design compileRoute(const GenerationRequest &request, Cancel cancel, Progress progress) {
    // Only this compiler owns mutation. Independent acceptance sees one final
    // route, never a partially banked route or a repaired force history.
    RouteEnergy energy;
    RoutePlacement placement;
    std::vector<RecipePort> ports;
    Design route;
    route.request = request;
    unsigned energyPasses = 0, bootstrapPasses = 0, closurePasses = 0;
    const auto phase = [&](WorkPhase value, const std::string &detail) {
        if (progress)
            progress({value, 0, 0, 0, detail});
    };
    auto checkpoint = [&]() {
        if (cancel && cancel()) {
            route.simulation.cancelled = true;
            throw RecipeCompileFailure("Route compilation cancelled", std::move(route));
        }
    };
    auto prefix = [&](const std::string &detail) {
        for (;;) {
            checkpoint();
            phase(WorkPhase::Authoring, detail);
            try {
                return composeRoute(request, energy, placement, ports, cancel,
                                    RecipeCompileMode::EnergyCalibrationPrefix);
            } catch (RecipeCompileFailure &failed) {
                if (!failed.pendingPort || bootstrapPasses == 4 ||
                    failed.partial.simulation.cancelled)
                    throw;
                ++bootstrapPasses;
                RecipeEnergyEstimate bootstrap;
                bootstrap.energyCorrection = energy.portSquaredSpeedCorrection;
                phase(WorkPhase::Motion, "Estimating reached source energy, bootstrap " +
                                             std::to_string(bootstrapPasses));
                const auto estimate = bootstrapRecipeEnergy(failed, bootstrap, cancel);
                energy.portSquaredSpeedCorrection = std::move(bootstrap.energyCorrection);
                if (!estimate.corrected) {
                    failed.partial.report.errors.insert(failed.partial.report.errors.end(),
                                                        estimate.report.errors.begin(),
                                                        estimate.report.errors.end());
                    failed.partial.simulation.cancelled = estimate.cancelled;
                    throw;
                }
            }
        }
    };
    // Solve the coupled physical state. Placement changes turn length and loss;
    // energy changes force-authored dimensions. One bounded loop owns both.
    // It never changes force intent or accepts a partial route.
    bool converged = false;
    double closureResidual = INFINITY;
    const unsigned solveBudget = unsigned(request.maxClosureIterations) + 4;
    for (; energyPasses < solveBudget; ++energyPasses) {
        route = prefix("Composing physical route, solve pass " + std::to_string(energyPasses + 1));
        phase(WorkPhase::Motion,
              "Replaying finite-train energy, solve pass " + std::to_string(energyPasses + 1));
        const auto motion =
            simulateMotion(route.track, route.operations, request.train, request.simulationStep,
                           cancel, MotionReplayMode::StationEnergyPrefix);
        if (motion.cancelled || !motion.prefixReachedEnd || !motion.report.valid()) {
            route.simulation.cancelled = motion.cancelled;
            route.simulation.report = motion.report;
            throw RecipeCompileFailure(
                "Finite-train energy solve did not traverse the authored route", std::move(route));
        }
        const auto fit = updateEnergyPlan(route, ports, motion.frames, energy);
        phase(WorkPhase::Authoring, "Solving station approach placement");
        const auto approach = std::find_if(route.request.recipe.elements.begin(),
                                           route.request.recipe.elements.end(), [](const auto &e) {
                                               return e.role == RideRole::Return &&
                                                      e.anchor == TerrainAnchor::Approach;
                                           });
        const auto port = approach == route.request.recipe.elements.end()
                              ? ports.end()
                              : std::find_if(ports.begin(), ports.end(),
                                             [&](const auto &p) { return p.id == approach->id; });
        if (port == ports.end())
            throw RecipeCompileFailure("Route plan has no terminal approach port",
                                       std::move(route));
        const auto step = solveClosureStep(route, placement, *port, cancel, closureResidual);
        if (step == PlacementStep::Unavailable)
            throw RecipeCompileFailure("Terminal placement model left its physical approach domain",
                                       std::move(route));
        if (step == PlacementStep::Advanced &&
            ++closurePasses > unsigned(request.maxClosureIterations))
            break;
        if (step == PlacementStep::Converged && fit.maximumPortError < .20 &&
            std::abs(fit.peakSpeedError) < .20) {
            converged = true;
            ++energyPasses;
            break;
        }
    }
    if (!converged) {
        route.report.fail("ROUTE_SOLVE",
                          "Bounded coupled energy and station placement solve did not converge", 0,
                          closureResidual, .25);
        throw RecipeCompileFailure("Route plan did not converge", std::move(route));
    }
    // One real closed-circuit solve. There is no geometry fallback or candidate
    // acceptance loop, and no rejected force profile can mutate the plan.
    phase(WorkPhase::Authoring, "Closing the planned route at the station");
    route =
        composeRoute(request, energy, placement, ports, cancel, RecipeCompileMode::ClosedCircuit);
    phase(WorkPhase::Motion, "Replaying the complete circuit and checking source entry speeds");
    auto motion = simulateMotion(route.track, route.operations, request.train,
                                 request.simulationStep, cancel, MotionReplayMode::TrackDomain);
    if (motion.cancelled || !motion.completed || !motion.report.valid()) {
        route.simulation.cancelled = motion.cancelled;
        route.simulation.report = motion.report;
        throw RecipeCompileFailure("Compiled circuit did not complete its motion replay",
                                   std::move(route));
    }
    double portError = 0;
    for (const auto &port : ports)
        portError =
            std::max(portError, std::abs(replayValueAt(motion.frames, port.distance) - port.speed));
    if (portError > .25) {
        route.report.fail("SOURCE_SPEED",
                          "Complete finite-train motion disagrees with source entry speed", 0,
                          portError, .25);
        throw RecipeCompileFailure("Closed route changed calibrated energy", std::move(route));
    }
    phase(WorkPhase::Authoring, "Compiling physical frames between fixed source boundaries");
    authorBanking(route, motion.frames);
    // Trim placement is explicit operational intent. Automatic sampling of
    // arbitrary sections used to scatter fins into the ride; nominal generation
    // now emits only the recipe's declared brake and propulsion corridors.
    std::ostringstream diagnostics;
    diagnostics << std::setprecision(12)
                << "{\"schemaVersion\":5,\"generationOnly\":true,\"architecture\":\"plan-compose-"
                   "energy-closure-frame\","
                << "\"energyPasses\":" << energyPasses << ",\"bootstrapPasses\":" << bootstrapPasses
                << ",\"closurePasses\":" << closurePasses << ",\"closurePlacementResidualMeters\":";
    if (std::isfinite(closureResidual))
        diagnostics << closureResidual;
    else
        diagnostics << "null";
    diagnostics << ",\"maximumPortSpeedError\":" << portError
                << ",\"composition\":" << route.planningDiagnostics << '}';
    route.planningDiagnostics = diagnostics.str();
    return route;
}

} // namespace coaster
