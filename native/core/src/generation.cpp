#include "coaster/coaster.hpp"
#include "coaster/clearance.hpp"
#include "recipe_compiler.hpp"
#include "authoring.hpp"
#include "progress_internal.hpp"
#include "acceptance_replay.hpp"
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace coaster {
namespace {
// Hardware corridors are a property of the final route. Verification must not
// straighten or unbank the route after the compiler has committed its sources.
void validatePropulsionCorridors(Design &design) {
    const double margin = (design.request.train.cars - 1) * design.request.train.spacing + 1.5;
    for (const auto &operation : design.operations) {
        if (operation.kind != DriveKind::Launch && operation.kind != DriveKind::Boost)
            continue;
        const auto first = design.track.sample(operation.start - margin);
        const double heading = std::atan2(first.tangent.y, first.tangent.x);
        auto aligned = [&](double distance) {
            const auto k = sampleKinematics(design.track, distance);
            const auto &p = k.sample;
            const Vec3 upright = unit(Vec3{0, 0, 1} - p.tangent * p.tangent.z);
            return std::hypot(p.tangent.x, p.tangent.y) > 1e-3 &&
                   std::abs(cross(p.tangent, p.curvature).z) < 1e-5 && dot(p.up, upright) > .9998 &&
                   std::abs(dot(k.upS, p.right)) < .001 &&
                   std::abs(std::remainder(std::atan2(p.tangent.y, p.tangent.x) - heading,
                                           2 * pi)) <= .001;
        };
        for (double distance = operation.start - margin; distance <= operation.end + margin;
             distance += .125) {
            if (!aligned(distance)) {
                design.report.fail("PROPULSION_ALIGNMENT",
                                   "The final route leaves the powered train's alignment corridor",
                                   distance);
                return;
            }
        }
        if (!aligned(operation.end + margin)) {
            design.report.fail("PROPULSION_ALIGNMENT",
                               "The final route leaves the powered train's exit corridor",
                               operation.end + margin);
            return;
        }
        for (const auto &span : design.track.spans) {
            if (span.start < operation.start - margin || span.start > operation.end + margin)
                continue;
            if (!aligned(span.start)) {
                design.report.fail("PROPULSION_ALIGNMENT",
                                   "A route boundary leaves the powered train's alignment corridor",
                                   span.start);
                return;
            }
        }
    }
}
} // namespace

double movingRideSeconds(const Design &d) {
    for (const auto &op : d.operations)
        if (op.kind == DriveKind::Station)
            return replayValueAt(d.simulation.frames, op.start, true);
    throw std::runtime_error("Moving duration requires the terminal station brake");
}

void evaluateTargets(Design &d, const ClearanceSweep *prepared) {
    if (d.supports.empty())
        d.report.fail("SUPPORT_LAYOUT", "Design contains no connected supports");
    auto &m = d.simulation.metrics;
    const auto &req = d.request;
    double low = 1e9, high = -1e9, station = d.track.knots.front().position.z;
    for (double s = 0; s < d.track.length; s += 1) {
        auto p = d.track.sample(s);
        double gh = p.position.z - req.terrain.height(p.position.x, p.position.y);
        low = std::min(low, p.position.z);
        high = std::max(high, p.position.z);
        m.maxGroundHeight = std::max(m.maxGroundHeight, gh);
        if (p.element == Element::Inversion && p.up.z < -.5)
            m.inversionGroundHeight = std::max(m.inversionGroundHeight, gh);
    }
    m.heightAboveStation = high - station;
    m.verticalRelief = high - low;
    m.minGroundClearance = prepared ? minimumSweptGroundClearance(*prepared, req.terrain)
                                    : minimumSweptGroundClearance(d.track, req.terrain, req.train);
    auto min = [&](const char *code, double value, double goal) {
        if (!std::isfinite(value) || value + 1e-6 < goal)
            d.report.fail(code, "Measured result misses requested target", 0, value, goal);
    };
    min("HEIGHT_TARGET", m.maxGroundHeight, req.targets.height);
    min("INVERSION_TARGET", m.inversionGroundHeight, req.targets.inversionHeight);
    auto dynamic = validateSimulationTargets(d.simulation, req.targets, req.limits);
    d.report.errors.insert(d.report.errors.end(), dynamic.errors.begin(), dynamic.errors.end());
    d.report.warnings.insert(d.report.warnings.end(), dynamic.warnings.begin(),
                             dynamic.warnings.end());
    d.report.warnings.push_back(
        "Force limits are provisional game assumptions; reference calibration is not complete.");
}
ValidationReport validateRequest(const GenerationRequest &req) {
    if (req.trackProfile != TrackProfile::Legacy && req.trackProfile != TrackProfile::Exa) {
        ValidationReport r;
        r.fail("TRACK_PROFILE", "Unknown track section profile");
        return r;
    }
    if (!req.recipe.elements.empty()) {
        std::string error;
        if (!validateRecipe(req.recipe, error)) {
            ValidationReport report;
            report.fail("RECIPE_CONFIG", error);
            return report;
        }
    }
    ValidationReport r;
    if (!req.terrain.valid()) {
        r.fail("TERRAIN_PROFILE", "Terrain profile is invalid");
        return r;
    }
    const auto &t = req.targets;
    const auto &l = req.limits;
    for (double value :
         {t.height, t.speed, t.inversionHeight, t.launchSeconds, l.minVerticalG, l.maxVerticalG,
          l.maxLateralG, l.maxLongitudinalG, l.maxJerkGps, l.minClearance, req.simulationStep})
        if (!std::isfinite(value)) {
            r.fail("REQUEST_RANGE", "Request contains a nonfinite value");
            return r;
        }
    if (req.maxClosureIterations < 1 || req.maxClosureIterations > 64 || t.height < 0 ||
        t.height > 350 || t.speed < 1 || t.speed > 110 || t.inversionHeight < 0 ||
        t.inversionHeight > 140 || t.launchSeconds < .8 || t.launchSeconds > 10 ||
        l.maxVerticalG <= l.minVerticalG || l.maxLateralG <= 0 || l.maxLongitudinalG <= 0 ||
        l.maxJerkGps <= 0 || l.minClearance < 0 || req.simulationStep < 1. / 2000 ||
        req.simulationStep > 1. / 30 || !req.terrain.valid())
        r.fail("REQUEST_RANGE", "Request is outside the prototype's supported domain");
    if (std::isinf(t.referenceExposure) ||
        (std::isfinite(t.referenceExposure) && t.referenceExposure <= 0) ||
        (!t.referenceId.empty() && !std::isfinite(t.referenceExposure)) ||
        (std::isfinite(t.referenceExposure) && t.referenceId.empty()))
        r.fail("REFERENCE_CONFIG",
               "Configured reference needs a finite positive exposure and a nonempty ID");
    auto reference = validateReference(t);
    r.errors.insert(r.errors.end(), reference.errors.begin(), reference.errors.end());
    for (double rate : {l.maxLateralRateGps, l.maxLongitudinalRateGps})
        if (std::isinf(rate) || (std::isfinite(rate) && rate <= 0))
            r.fail("AXIS_RATE_CONFIG",
                   "Optional component rate gates must be finite positive values, or unset");
    if (!std::isfinite(req.style.airtime) || req.style.airtime < .75 || req.style.airtime > 1.2 ||
        !std::isfinite(req.style.signatureRollDegrees) || req.style.signatureRollDegrees < 30 ||
        req.style.signatureRollDegrees > 60 || req.style.returnStyle < -1 ||
        req.style.returnStyle > 1)
        r.fail("STYLE_CONFIG", "Airtime strength must be 0.75..1.2, signature roll 30..60 degrees, "
                               "and return style automatic/flowing/twin-airtime");
    const auto &train = req.train;
    for (double value : {train.carMass, train.spacing, train.seatHeight, train.dragCdA,
                         train.rollingResistance, train.airDensity})
        if (!std::isfinite(value)) {
            r.fail("TRAIN_CONFIG", "Train contains nonfinite settings");
            return r;
        }
    if (train.cars < 1 || train.cars > 16 || train.spacing <= 0 || train.spacing > 20 ||
        train.carMass <= 0 || train.seatHeight < 0 || train.seatHeight > 3 || train.dragCdA < 0 ||
        train.rollingResistance < 0 || train.airDensity < 0)
        r.fail("TRAIN_CONFIG", "Train is outside the supported model domain");
    return r;
}
Design generate(const GenerationRequest &request, Cancel cancellation, Progress progress) {
    WorkRecorder work(progress);
    Design design;
    design.request = request;
    design.report = validateRequest(request);

    // The compiler and concurrent independent validators share cancellation,
    // but the caller is never required to provide a thread-safe callback.
    std::mutex cancellationMutex;
    Cancel cancel;
    if (cancellation)
        cancel = [&] {
            std::lock_guard lock(cancellationMutex);
            return cancellation();
        };
    auto cancelled = [&] {
        if (!cancel || !cancel())
            return false;
        design.simulation.cancelled = true;
        if (std::none_of(design.report.errors.begin(), design.report.errors.end(),
                         [](const Finding &finding) { return finding.code == "CANCELLED"; }))
            design.report.fail("CANCELLED", "Generation cancelled");
        return true;
    };
    auto finish = [&]() -> Design {
        design.timings = work.snapshot();
        if (design.planningDiagnostics.empty())
            design.planningDiagnostics = "{\"generationOnly\":true}";
        if (design.simulation.completed && !design.simulation.frames.empty() &&
            std::any_of(
                design.operations.begin(), design.operations.end(),
                [](const Operation &operation) { return operation.kind == DriveKind::Station; })) {
            std::ostringstream timing;
            timing << std::setprecision(12)
                   << ",\"movingDurationSeconds\":" << movingRideSeconds(design)
                   << ",\"movingDurationDefinition\":\"Powered departure to train center reaching "
                      "terminal Station operation start; excludes terminal stopping\"";
            design.planningDiagnostics.insert(design.planningDiagnostics.size() - 1, timing.str());
        }
        work.message(0, design.accepted() ? "Route accepted" : "Route rejected");
        for (const auto *report : {&design.report, &design.simulation.report})
            for (const auto &finding : report->errors) {
                std::ostringstream detail;
                detail << std::setprecision(12) << finding.code << ": " << finding.message
                       << " | distanceMeters=" << finding.distance << " actual=" << finding.actual
                       << " limit=" << finding.limit;
                work.message(0, detail.str());
            }
        return std::move(design);
    };
    if (!design.report.valid() || cancelled())
        return finish();

    try {
        const double fastest = plannedDepartureSeconds(
            request.limits.maxLongitudinalG * gravity - .02, request.train, request.limits);
        if (fastest > request.targets.launchSeconds) {
            design.report.fail("LAUNCH_FEASIBILITY",
                               "Requested departure exceeds the force-limited station launch", 0,
                               fastest, request.targets.launchSeconds);
            return finish();
        }
        if (request.targets.requireIntensity && std::isfinite(request.targets.referenceExposure) &&
            request.targets.referenceExposure > 10 * std::max(0., request.limits.maxVerticalG)) {
            design.report.fail(
                "INTENSITY_FEASIBILITY",
                "Reference exposure exceeds ten seconds at the selected vertical force ceiling", 0,
                request.targets.referenceExposure, 10 * std::max(0., request.limits.maxVerticalG));
            return finish();
        }
        work.enter(WorkPhase::Authoring, 0,
                   "Planning the route, energy and hardware as one composition");
        design = compileRoute(request, cancel, [&](const WorkProgress &stage) {
            work.enter(stage.phase, stage.candidate, stage.detail, stage.completedWork,
                       stage.totalWork);
        });
        if (cancelled() || !design.report.valid())
            return finish();
        if (!design.track.closed || design.track.spans.empty() || design.track.knots.size() < 2) {
            design.report.fail("ROUTE_INCOMPLETE",
                               "The route compiler did not produce a complete circuit");
            return finish();
        }
        validatePropulsionCorridors(design);
        if (!design.report.valid())
            return finish();

        // From here onward geometry, physical frames and operations are final.
        // Terrain fitting and structures consume that route; acceptance only
        // measures it. A rejection cannot silently trigger a different design.
        fitCliffTerrain(design, cancel);
        AcceptanceReplay replay(design, cancel);
        work.enter(WorkPhase::Structures, 0,
                   "Constructing station and supports while replaying the final route");
        std::vector<Finding> constructionFailures;
        try {
            design.station =
                buildStation(design.track, design.request.terrain, request.train, cancel);
        } catch (const std::exception &error) {
            constructionFailures.push_back({"STATION_LAYOUT", error.what()});
        }
        try {
            buildSupportLayout(design, cancel);
        } catch (const std::exception &error) {
            constructionFailures.push_back({"SUPPORT_LAYOUT", error.what()});
        }
        if (cancelled()) {
            replay.join();
            return finish();
        }
        design.inversionDimensions =
            measureInversionDimensions(design.track, design.sections, cancel);
        replay.startSpatial();
        work.enter(WorkPhase::Geometry, 0,
                   "Checking the occupied train envelope and constructed route");
        auto sweep = buildClearanceSweepVerified(design.track, request.train, cancel);
        sweep.prepareGround(design.request.terrain, cancel);
        design.report = validateGeometry(design.track, design.request.terrain, request.limits,
                                         request.train, design.supports, sweep, cancel);
        design.report.errors.insert(design.report.errors.end(), constructionFailures.begin(),
                                    constructionFailures.end());
        const auto structures = validateDesignStructures(design, sweep, cancel);
        design.report.errors.insert(design.report.errors.end(), structures.errors.begin(),
                                    structures.errors.end());
        replay.finish(sweep, &work);
        cancelled();
    } catch (RecipeCompileFailure &failure) {
        design = std::move(failure.partial);
        if (!cancelled())
            design.report.fail("AUTHORING", failure.what());
    } catch (const std::exception &error) {
        if (!cancelled())
            design.report.fail("GENERATION_FAILURE", error.what());
    }
    return finish();
}
} // namespace coaster
