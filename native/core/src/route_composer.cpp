#include "route_plan.hpp"
#include "launch_camelback.hpp"
#include "authoring_energy.hpp"
#include "route_closure.hpp"
#include <iomanip>
#include <sstream>

namespace coaster {
namespace {
using detail::AngleJet;
using detail::MotionJet;
Vec3 rotated(Vec3 p, double heading) {
    return {p.x * std::cos(heading) - p.y * std::sin(heading),
            p.x * std::sin(heading) + p.y * std::cos(heading), p.z};
}
MotionJet rotated(MotionJet p, double heading) {
    p.position = rotated(p.position, heading);
    p.tangent = rotated(p.tangent, heading);
    p.curvature = rotated(p.curvature, heading);
    p.third = rotated(p.third, heading);
    p.fourth = rotated(p.fourth, heading);
    return p;
}
double seeded(uint64_t seed, double low, double high) {
    return low + (high - low) * double(seed >> 11) * 0x1.0p-53;
}
double accelerationFor(const GenerationRequest &req) {
    // Size the actuator from the requested departure, not a preferred G ceiling.
    double low = 0, high = std::max(req.limits.maxLongitudinalG * gravity - .02,
                                    100. / req.targets.launchSeconds);
    for (int i = 0; i < 32; ++i) {
        const double mid = (low + high) * .5;
        if (plannedDepartureSeconds(mid, req.train, req.limits) > req.targets.launchSeconds - .003)
            low = mid;
        else
            high = mid;
    }
    return high;
}
struct CompiledElement {
    size_t moduleBegin{}, moduleEnd{};
    RideRole role;
    std::string id;
};

class RouteDraft {
    RoutePlan plan;
    Design d;
    const GenerationRequest &req;
    AuthoringEnergy energy;
    Progress progress;
    Cancel cancel;
    std::vector<RecipePort> &ports;
    MotionBuilder motion;
    MotionJet &cursor;
    double datum, halfTrain, hand;
    std::vector<CompiledElement> compiled;
    std::vector<std::pair<std::string, size_t>> sourcePorts;
    std::vector<std::pair<LandmarkKind, size_t>> landmarks;
    std::vector<double> sourceSpeeds;
    size_t brakeBegin = 0;
    Vec3 plateauArrival{}, cliffTop{}, cliffFoot{}, waveBase{}, loopBase{}, immelExit{},
        returnRavine{};
    double cliffYaw = 0, brakeCapacity = 7, brakeAlignment = 0;
    Vec3 openingRecovery{};
    std::array<Vec3, 2> openingCrests{}, openingValleys{};
    struct SourceOptions {
        double begin{};
        bool worldCoordinates{}, recordPort{true};
    };
    static Design initialize(const RoutePlan &plan) {
        Design design;
        design.request = plan.request;
        design.track.profile = plan.request.trackProfile;
        design.track.closed = false;
        (void)trackSection(design.track.profile);
        design.topology = "route/" + plan.request.recipe.name;
        return design;
    }
    double heading() const {
        return std::atan2(cursor.tangent.y, cursor.tangent.x);
    }

    Vec3 planningTangent(double distance) const {
        const double end = motion.length();
        if (distance <= end)
            return motion.sampleGeometry(distance).tangent;
        // Only the unbuilt front half of the train is forecast. The retained
        // energy cursor stops before it; emitted geometry always replaces it.
        const double h = distance - end;
        return unit(cursor.tangent + cursor.curvature * h + cursor.third * (h * h * .5) +
                    cursor.fourth * (h * h * h / 6));
    }
    void refreshEnergy() {
        refreshOperations(false);
        const double knownEnd = motion.length() - halfTrain - 1;
        if (knownEnd > energy.distance())
            energy.advanceTo(
                knownEnd, [this](double s) { return planningTangent(s); }, d.operations, cancel);
        if (motion.length() > energy.distance())
            motion.nominalSpeed = energy.forecastTo(
                motion.length(), [this](double s) { return planningTangent(s); }, d.operations,
                cancel);
    }
    double speedFor(const std::string &id) {
        if (progress)
            progress({WorkPhase::Motion, 0, 0, 0, "Advancing source energy for " + id});
        refreshEnergy();
        if (progress)
            progress({WorkPhase::Authoring, 0, 0, 0,
                      "Source " + id + " entry speed " + std::to_string(motion.nominalSpeed)});
        if (!(motion.nominalSpeed > .5))
            throw std::runtime_error("Source entry has no physical forward energy: " + id);
        return motion.nominalSpeed;
    }

    void refreshOperations(bool station) {
        d.operations.clear();
        const size_t last = d.track.knots.size() - 1;
        auto at = [&](size_t index) {
            return index >= d.track.spans.size() ? d.track.length : d.track.spans[index].start;
        };
        for (const auto &motor : motion.motors)
            if (motor.begin < motor.end && motor.end <= last) {
                const double ramp = motor.kind == DriveKind::Launch
                                        ? departureRamp(motor.acceleration, req.limits)
                                        : .65;
                // Brake contact is local to each car. Neighboring cars may already
                // enter/leave the curved approach while the jaws remain on straight
                // rail; only powered alignment needs the full train-length margin.
                const double entryMargin = motor.kind == DriveKind::Boost ? 2 * halfTrain + 3 : 3;
                const double exitMargin = motor.kind == DriveKind::Brake ? 3 : 2 * halfTrain + 3;
                d.operations.push_back({at(motor.begin) + entryMargin, at(motor.end) - exitMargin,
                                        motor.kind, motor.speed,
                                        req.train.carMass * motor.acceleration,
                                        req.train.carMass * motor.acceleration * 110, ramp});
            }
        if (station)
            d.operations.push_back({at(brakeBegin) + 2 * halfTrain + 3, 80, DriveKind::Station, 0,
                                    req.train.carMass * brakeCapacity,
                                    req.train.carMass * brakeCapacity * 100, .5,
                                    std::min(6., brakeCapacity * 6 / 7), 1.5});
        for (auto &operation : d.operations)
            operation.exitFadeMeters = std::max(1., operation.targetSpeed * operation.rampSeconds);
    }
    void finalize() {
        d.track.rebuild();
        refreshOperations(true);
        d.sections.clear();
        d.landmarks.clear();
        ports.clear();
        const size_t last = d.track.knots.size() - 1;
        auto at = [&](size_t index) {
            return index >= d.track.spans.size() ? d.track.length : d.track.spans[index].start;
        };
        for (const auto &element : compiled)
            for (size_t i = element.moduleBegin; i < element.moduleEnd; ++i) {
                const auto &m = motion.modules[i];
                if (m.begin < last && m.end > m.begin)
                    d.sections.push_back({m.name, at(m.begin), at(std::min(m.end, last)),
                                          m.reversals, m.planar, element.role, element.id});
            }
        for (const auto &[kind, knot] : landmarks)
            if (knot <= last)
                d.landmarks.push_back({kind, at(knot)});
        d.landmarks.push_back({LandmarkKind::BrakeEntry, at(brakeBegin)});
        for (size_t i = 0; i < sourcePorts.size(); ++i)
            if (sourcePorts[i].second <= last)
                ports.push_back({sourcePorts[i].first, at(sourcePorts[i].second), sourceSpeeds[i]});
    }

    template <class Intent> void lossFor(Intent &intent) const {
        intent.rollingAcceleration = gravity * req.train.rollingResistance;
        intent.dragAccelerationCoefficient = motion.drag;
    }

    MotionBuilder::Range freeDirection(AngleJet endPitch, AngleJet endYaw, double length,
                                       Element kind, const std::string &name, double bank = 0.) {
        const auto incoming = detail::directionAngles(cursor);
        endYaw.value = incoming[1].value + std::remainder(endYaw.value - incoming[1].value, 2 * pi);
        const auto hint =
            polynomialMotion(cursor, detail::anglePolynomial(incoming[0], endPitch, length),
                             detail::anglePolynomial(incoming[1], endYaw, length), length);
        return motion.programme(hint, kind, name.c_str(), bank);
    }

    MotionBuilder::Range heightSweep(MotionJet end, double length, Element kind,
                                     const std::string &name, double bank = 0) {
        const auto start = cursor;
        const auto first = detail::directionAngles(start), last = detail::directionAngles(end);
        const double half = length * .5;
        const auto yaw = detail::anglePolynomial(first[1], last[1], length);
        const auto middleYaw = detail::angleAt(yaw, half, length);
        auto shaped = [&](double pitch) {
            const AngleJet middle{pitch, 0, 0, 0};
            auto a = polynomialMotion(start, detail::anglePolynomial(first[0], middle, half),
                                      detail::anglePolynomial(first[1], middleYaw, half), half);
            auto b = polynomialMotion(a.end, detail::anglePolynomial(middle, last[0], half),
                                      detail::anglePolynomial(middleYaw, last[1], half), half);
            return std::pair{a, b};
        };
        double low = -.9, high = .9;
        if (shaped(low).second.end.position.z > end.position.z ||
            shaped(high).second.end.position.z < end.position.z)
            throw std::runtime_error(name + ": requested shelf height exceeds its pitch corridor");
        for (int i = 0; i < 45; ++i) {
            const double mid = (low + high) * .5;
            if (shaped(mid).second.end.position.z < end.position.z)
                low = mid;
            else
                high = mid;
        }
        const auto programmes = shaped((low + high) * .5);
        const auto begin = motion.programme(programmes.first, kind, (name + "-entry").c_str());
        const auto finish = motion.programme(programmes.second, kind, name.c_str(), bank);
        return {begin.first, finish.second};
    }

    MotionBuilder::Range terrainCurve(MotionJet end, double length, Element kind,
                                      const std::string &name, double bank = 0.) {
        const auto first = detail::directionAngles(cursor), last = detail::directionAngles(end);
        const double yaw = first[1].value + std::remainder(last[1].value - first[1].value, 2 * pi);
        const AngleJet endYaw{yaw, last[1].first, last[1].second, last[1].third};
        const auto yawPolynomial = detail::anglePolynomial(first[1], endYaw, length);
        const double target = end.position.z, sign = target >= cursor.position.z ? 1. : -1.;
        std::optional<std::pair<MotionProgram, MotionProgram>> selected;
        // Unequal phases let inherited crest curvature release over a physical
        // distance without adding a shoulder to a shallow terrain-following act.
        for (double fraction : {.5, .375, .625, .25, .75}) {
            const double firstLength = length * fraction, secondLength = length - firstLength;
            const auto midYaw = detail::angleAt(yawPolynomial, firstLength, length);
            auto shaped = [&](double peak) {
                const AngleJet middle{peak, 0, 0, 0};
                auto a = polynomialMotion(
                    cursor, detail::anglePolynomial(first[0], middle, firstLength),
                    detail::anglePolynomial(first[1], midYaw, firstLength), firstLength);
                auto b = polynomialMotion(
                    a.end, detail::anglePolynomial(middle, last[0], secondLength),
                    detail::anglePolynomial(midYaw, endYaw, secondLength), secondLength);
                return std::pair{a, b};
            };
            double low = sign > 0 ? std::max({0., first[0].value, last[0].value}) : -.9;
            double high = sign > 0 ? .9 : std::min({0., first[0].value, last[0].value});
            if (shaped(low).second.end.position.z > target ||
                shaped(high).second.end.position.z < target)
                continue;
            for (int i = 0; i < 45; ++i) {
                const double mid = (low + high) * .5;
                if (shaped(mid).second.end.position.z < target)
                    low = mid;
                else
                    high = mid;
            }
            auto pair = shaped((low + high) * .5);
            bool ordered = true;
            for (const auto &[program, trend] : std::array<std::pair<MotionProgram, double>, 2>{
                     {{pair.first, sign}, {pair.second, -sign}}})
                for (int i = 0; i <= 128; ++i) {
                    const auto pitch =
                        detail::directionAngles(program.direction(program.length * i / 128.))[0];
                    if (trend * pitch.first < -1e-10 || sign * pitch.value < -1e-10)
                        ordered = false;
                }
            if (!ordered)
                continue;
            if (std::abs(pair.second.end.position.z - target) > 1e-8)
                throw std::runtime_error(name + ": fixed-length height did not converge");
            selected = std::move(pair);
            break;
        }
        if (!selected)
            throw std::runtime_error(
                name + ": inherited pitch cannot form one fixed-length ordered crest or trough");
        const size_t moduleBegin = motion.modules.size();
        const auto a = motion.programme(selected->first, kind, (name + "-entry").c_str());
        const auto b = motion.programme(selected->second, kind, name.c_str(), bank);
        const bool planar =
            motion.modules[moduleBegin].planar && motion.modules[moduleBegin + 1].planar;
        motion.modules.resize(moduleBegin);
        motion.modules.push_back({a.first, b.second, name, 0, planar});
        return MotionBuilder::Range{a.first, b.second};
    }

    void source(const FvdHillResult &result, Element kind, const RecipeElement &element,
                SourceOptions options) {
        if (progress)
            progress({WorkPhase::Authoring, 0, 0, 0, "Emitting source " + element.id});
        const double begin = options.begin;
        if (!result.section.report.valid() || !result.section.assessment.passed)
            throw std::runtime_error(
                element.id + ": " +
                (result.section.report.errors.empty()
                     ? "force authoring residual"
                     : result.section.report.errors.front().message) +
                "; entrySpeed=" + std::to_string(result.authoring.speed) +
                ", incomingPitch=" + std::to_string(std::asin(cursor.tangent.z)) +
                ", height=" + std::to_string(cursor.position.z));
        const double yaw = options.worldCoordinates ? 0 : heading();
        const auto first = rotated(detail::jet(sampleKinematics(result.section.track, begin)), yaw);
        if (norm(first.tangent - cursor.tangent) + norm(first.curvature - cursor.curvature) +
                norm(first.third - cursor.third) + norm(first.fourth - cursor.fourth) >
            1e-7) {
            throw std::runtime_error(element.id + ": source must inherit the live geometry jet");
        }
        const Vec3 origin = cursor.position - first.position;
        const size_t firstKnot = d.track.knots.size() - 1;
        const auto range = motion.force(result.section, result.authoring, kind, element.id.c_str(),
                                        origin, yaw, begin);
        if (options.recordPort)
            sourcePorts.push_back({element.id, firstKnot});
        const auto &samples = result.section.samples;
        const auto at =
            std::lower_bound(samples.begin(), samples.end(), begin,
                             [](const FvdSample &a, double s) { return a.distance < s; });
        double sourceSpeed = result.authoring.speed;
        if (at != samples.end()) {
            if (at == samples.begin())
                sourceSpeed = at->speed;
            else {
                const auto &before = *(at - 1);
                sourceSpeed =
                    std::lerp(before.speed, at->speed,
                              (begin - before.distance) / (at->distance - before.distance));
            }
        }
        if (options.recordPort)
            sourceSpeeds.push_back(sourceSpeed);
        motion.nominalSpeed = samples.back().speed;
        size_t apex = range.first;
        for (size_t k = range.first + 1; k <= range.second; ++k)
            if (d.track.knots[k].position.z > d.track.knots[apex].position.z)
                apex = k;
        if (element.role == RideRole::Opening) {
            landmarks.push_back({LandmarkKind::OpeningCrest, apex});
            landmarks.push_back({LandmarkKind::OpeningRecovery, range.second});
        }
        if (element.role == RideRole::Camelback)
            landmarks.push_back({LandmarkKind::CamelbackCrest, apex});
        if (element.role == RideRole::Wave)
            landmarks.push_back({LandmarkKind::WaveCrest, apex});
        if (element.role == RideRole::Loop)
            landmarks.push_back({LandmarkKind::LoopCrest, apex});
        if (element.role == RideRole::Immelmann)
            landmarks.push_back({LandmarkKind::ImmelmannCrest, apex});
        // Split only at actual extrema so section intent describes the source,
        // while every boundary keeps its complete live geometric derivatives.
        size_t start = range.first, turning = range.first;
        int previous = 0, index = 0;
        for (size_t k = range.first; k <= range.second; ++k) {
            const double dz = d.track.knots[k].tangent.z;
            const int sign = dz > .008 ? 1 : dz < -.008 ? -1 : 0;
            if (previous > 0 ? d.track.knots[k].position.z > d.track.knots[turning].position.z
                             : d.track.knots[k].position.z < d.track.knots[turning].position.z)
                turning = k;
            if (sign && previous && sign != previous && turning > start) {
                motion.modules.push_back(
                    {start, turning, element.id + "-" + std::to_string(index++), 0,
                     std::holds_alternative<CamelbackParameters>(element.parameters)});
                start = turning;
                turning = k;
            }
            if (sign)
                previous = sign;
        }
        if (start < range.second)
            motion.modules.push_back(
                {start, range.second, element.id + "-" + std::to_string(index), 0,
                 std::holds_alternative<CamelbackParameters>(element.parameters)});
    }

    void source(const FvdHillResult &result, Element kind, const RecipeElement &element) {
        source(result, kind, element, SourceOptions{});
    }
    void groundLandscape(Vec3 crest, double yaw, double summit) {
        const Vec3 plateauCenter = crest + rotated(Vec3{-80, 150, 0}, yaw);
        if (req.terrain.kind != TerrainKind::Highlands)
            return;
        auto &t = d.request.terrain;
        t.ridge = {};
        t.ramps.clear();
        t.knolls.clear();
        t.foothills.clear();
        t.ravines.clear();
        const double openingGrade = (openingValleys.back().z - openingRecovery.z) /
                                    std::hypot(openingValleys.back().x - openingRecovery.x,
                                               openingValleys.back().y - openingRecovery.y);
        t.ramps.push_back({openingRecovery.x, openingRecovery.y, openingValleys.back().x,
                           openingValleys.back().y, openingRecovery.z - datum,
                           openingValleys.back().z - datum, openingGrade, openingGrade, 140});
        t.foothills.push_back(
            {openingCrests[0].x, openingCrests[0].y, std::max(0., openingCrests[0].z - 17.5), 190});
        t.foothills.push_back(
            {openingCrests[1].x, openingCrests[1].y, std::max(0., openingCrests[1].z - 13.5), 170});
        const auto center = plateauCenter;
        t.centerX = center.x;
        t.centerY = center.y;
        t.heightMeters = std::max(0., summit - 14.5);
        t.backSlope = TerrainSlope{
            crest.x, crest.y, t.heightMeters, .65 * std::cos(yaw), .65 * std::sin(yaw), 60};
        t.radiusX = 700;
        t.radiusY = 620;
        t.plateau = .72;
        t.bend = .06;
        t.cliffX = crest.x + 500 * std::cos(yaw);
        t.cliffY = crest.y + 500 * std::sin(yaw);
        t.cliffHeading = yaw;
        t.cliffWidth = 24;
        t.cliffCurvature = .00022;
        // The connected shelf and broad foothills are semantic landforms, not
        // a dense copy of rail samples. Later ports determine their final front.
    }

    void authorStation(const RecipeElement &element) {

        const auto &p = std::get<OperationParameters>(element.parameters);
        if (p.lengthMeters > halfTrain + 27)
            throw std::runtime_error(
                "Station length must leave the stopped train within the departure stators");
        motion.line(p.lengthMeters, Element::Station, element.id.c_str());
    }

    void authorDeparture(const RecipeElement &element) {

        const auto &p = std::get<OperationParameters>(element.parameters);
        const double target = p.targetSpeedKmh > 0 ? p.targetSpeedKmh / 3.6 : 64;
        motion.drive(p.lengthMeters, DriveKind::Launch, target,
                     p.accelerationMps2 > 0 ? p.accelerationMps2 : accelerationFor(req),
                     element.id.c_str());
    }

    void authorOpening(const RecipeElement &element) {
        const uint64_t random = elementSeed(req.seed, element.id);
        const double variation = seeded(elementSeed(req.seed, element.id), .985, 1.015);

        const auto &p = std::get<HillParameters>(element.parameters);
        FvdHillRequest intent;
        intent.entrySpeed = speedFor(element.id);
        intent.height = p.riseMeters * p.profileScale * variation;
        intent.airtimeG = p.negativeG * req.style.airtime;
        intent.positiveG = p.pulloutG;
        intent.twistAngle = p.twistDegrees * seeded(random, .9, 1.1) * pi / 180;
        intent.rampSeconds = p.releaseSeconds;
        intent.exitRampSeconds = p.recoverySeconds;
        intent.exitHeight = 6;
        intent.exitPitch = .08;
        lossFor(intent);
        source(designFvdHill(intent, cancel), Element::Hill, element);
        openingRecovery = cursor.position;
        // Broad low hills follow the first twisted drop. They carry the
        // complete live geometry into deliberate rising and falling curves.
        for (int n = 0; n < 2; ++n) {
            const double base = datum + (n == 0 ? 6 : 3), rise = n == 0 ? 35 : 20, yaw = heading();
            const double crest = motion.crestCurvature(rise, 180, .05);
            auto top = detail::planarJet({cursor.position.x, cursor.position.y, base + rise},
                                         yaw + (n == 0 ? -25 : 30) * pi / 180, 0, crest);
            terrainCurve(top, 190, Element::Airtime,
                         element.id + (n == 0 ? "-river-hop-crest" : "-rising-bank-crest"),
                         (n == 0 ? -15 : 20) * pi / 180);
            openingCrests[n] = cursor.position;
            auto low = detail::planarJet({cursor.position.x, cursor.position.y, base},
                                         heading() + (n == 0 ? -20 : 15) * pi / 180, 0);
            terrainCurve(low, 190, Element::Airtime,
                         element.id + (n == 0 ? "-river-hop-valley" : "-rising-bank-valley"));
            openingValleys[n] = cursor.position;
        }
    }

    void authorCliffApproach(const RecipeElement &element) {

        const auto &p = std::get<CliffParameters>(element.parameters);
        const double summit = datum + p.summitHeightMeters;
        // The ascent starts with one deliberate bearing change; all following
        // source geometry inherits this live direction.
        const double layoutHeading = -12 * pi / 180;
        freeDirection({20 * pi / 180, 0, 0, 0}, {heading() + layoutHeading, 0, 0, 0},
                      std::max(120., motion.nominalSpeed * 2.0), Element::Turn,
                      element.id + "-recovery");
        motion.driveGraded(summit - 55, 32 * pi / 180, 58, 16, (element.id + "-ascent").c_str());
        motion.pitchToHeight(summit, {0, motion.crestCurvature(55, 180, -.5), 0, 0}, Element::Hill,
                             (element.id + "-arrival").c_str());
        plateauArrival = cursor.position;
        const double arrivalYaw = heading();
        groundLandscape(plateauArrival, arrivalYaw, summit);
        landmarks.push_back({LandmarkKind::PlateauArrival, d.track.knots.size() - 1});
        // The powered ascent is a viaduct over the valley into the mesa's
        // rear slope. An independent raised ribbon here would extend through
        // the cliff face and bury the lower launch corridor.
        // An active shelf journey: outward reveal, inland counterturn,
        // second edge crest and two low turns into the dive approach.
        // Each direction programme owns its pitch/yaw jets. No overlap
        // operation may resculpt these curves after they are authored.
        const double shelfHeading = heading(), lengthScale = p.approachLengthMeters / 1000.;
        for (size_t n = 0; n < plan.shelf.size(); ++n) {
            const auto &point = plan.shelf[n];
            auto end = detail::directionJet({0, point.pitchCurvature / lengthScale, 0, 0},
                                            {shelfHeading + point.headingDegrees * pi / 180,
                                             point.yawCurvature / lengthScale, 0, 0});
            end.position = {cursor.position.x, cursor.position.y, summit + point.heightOffset};
            const double outward = (n == 1 || n == 3) ? p.outwardBankDegrees * pi / 180 : 0;
            heightSweep(end, point.lengthMeters * lengthScale, Element::Turn,
                        element.id + "-shelf-" + std::to_string(n + 1), outward);
            if (req.terrain.kind == TerrainKind::Highlands && n % 2 == 0) {
                // Three shallow dry gullies cross the natural low points;
                // the mesa remains connected beneath the intervening hills.
                auto &t = d.request.terrain;
                const Vec3 side{-std::sin(heading()), std::cos(heading()), 0};
                const auto a = cursor.position - side * 130, b = cursor.position + side * 130;
                const double depth = std::max(0., t.heightMeters - cursor.position.z + 7);
                t.ravines.push_back({a.x, a.y, b.x, b.y, depth, depth, 150, 150});
            }
        }
    }

    void authorCliffLip(const RecipeElement &element) {

        const auto &p = std::get<OperationParameters>(element.parameters);
        const double target = p.targetSpeedKmh / 3.6;
        const double acceleration = p.accelerationMps2 > 0 ? p.accelerationMps2 : 7;
        const double length =
            std::max(p.lengthMeters, (motion.nominalSpeed * motion.nominalSpeed - target * target) /
                                             (2 * acceleration) +
                                         motion.nominalSpeed * .5 + 6);
        motion.drive(length, DriveKind::Brake, target, acceleration, element.id.c_str());
    }

    void authorCliffDrop(const RecipeElement &element) {

        const auto &p = std::get<CliffParameters>(element.parameters);
        FvdDiveRequest intent;
        cliffTop = cursor.position;
        cliffYaw = heading();
        intent.entrySpeed = speedFor(element.id);
        landmarks.push_back({LandmarkKind::CliffDeparture, d.track.knots.size() - 1});
        const auto boost = std::find_if(
            req.recipe.elements.begin(), req.recipe.elements.end(),
            [](const RecipeElement &item) { return item.role == RideRole::DownhillLaunch; });
        const auto &boostParameters = std::get<OperationParameters>(boost->parameters);
        intent.drop = cliffTop.z - (datum + 57);
        intent.maximumPitch = p.dropDegrees * pi / 180;
        intent.exitPitch = boostParameters.gradeDegrees * pi / 180;
        intent.crestRampSeconds = 2.2;
        intent.crestG = -.9;
        intent.pulloutG = 4.3;
        intent.pulloutRampSeconds = 1.5;
        intent.exitRampSeconds = 1.2;
        lossFor(intent);
        if (progress)
            progress({WorkPhase::Authoring, 0, 0, 0,
                      "Solving dive: drop " + std::to_string(intent.drop)});
        const auto dive = designFvdDive(intent, cancel);
        source(dive, Element::Hill, element);
        cliffFoot = cursor.position;
        if (req.terrain.kind == TerrainKind::Highlands) {
            auto &t = d.request.terrain;
            t.cliffX = cliffTop.x + std::cos(cliffYaw) * 7;
            t.cliffY = cliffTop.y + std::sin(cliffYaw) * 7;
            t.cliffHeading = cliffYaw;
            // Decorative foothills belong beside the resolved dive corridor;
            // fixed offsets from the climb crest can bury a redesigned drop.
            const Vec3 side{-std::sin(cliffYaw), std::cos(cliffYaw), 0};
            for (double offset : {-350., 350.}) {
                const auto center = cliffFoot + side * offset;
                t.foothills.push_back({center.x, center.y, 70, 250});
            }
        }
    }

    void authorDownhillLaunch(const RecipeElement &element) {

        const auto &p = std::get<OperationParameters>(element.parameters);
        const double target = (p.targetSpeedKmh > 0 ? p.targetSpeedKmh / 3.6 : req.targets.speed);
        const double acceleration = p.accelerationMps2 > 0 ? p.accelerationMps2 : 16;
        const double net = acceleration - gravity * cursor.tangent.z -
                           gravity * req.train.rollingResistance - motion.drag * target * target;
        const double length =
            p.lengthMeters > 0
                ? p.lengthMeters
                : std::max(120., (target * target - motion.nominalSpeed * motion.nominalSpeed) /
                                         (2 * net) +
                                     target * .85 + 4 * halfTrain + 6);
        motion.drive(length, DriveKind::Boost, target, acceleration, element.id.c_str());
        landmarks.push_back({LandmarkKind::DownhillLaunchExit, d.track.knots.size() - 1});
        if (cursor.position.z < datum)
            throw std::runtime_error("Inclined launch exhausted the authored cliff-foot relief");
        if (req.terrain.kind == TerrainKind::Highlands) {
            auto &t = d.request.terrain;
            t.ramps.push_back({cliffFoot.x, cliffFoot.y, cursor.position.x, cursor.position.y,
                               cliffFoot.z - datum, cursor.position.z - datum,
                               std::tan(p.gradeDegrees * pi / 180), 0, 140});
            const auto shoulder =
                (cliffFoot + cursor.position) * .5 + rotated(Vec3{0, -180, 0}, heading());
            t.foothills.push_back({shoulder.x, shoulder.y, 45, 280});
        }
    }

    void authorCamelback(const RecipeElement &element) {

        auto p = std::get<CamelbackParameters>(element.parameters);
        auto entry = element;
        entry.id = element.id + "-pullout";
        entry.role = RideRole::Unspecified;
        auto paired =
            designLaunchCamelback(speedFor(entry.id), std::asin(cursor.tangent.z), p,
                                  gravity * req.train.rollingResistance, motion.drag, cancel);
        const size_t pulloutBegin = d.track.knots.size() - 1;
        source(paired.pullout, Element::Hill, entry, {});
        if (req.terrain.kind == TerrainKind::Highlands) {
            const auto low = std::min_element(
                d.track.knots.begin() + pulloutBegin, d.track.knots.end(),
                [](const Knot &a, const Knot &b) { return a.position.z < b.position.z; });
            // The cliff-foot bench continues through the real valley, so
            // ordinary launch recovery stays close to connected ground.
            auto &bench = d.request.terrain.ramps.back();
            bench.x1 = low->position.x;
            bench.y1 = low->position.y;
            bench.h1 = std::max(0., low->position.z - datum);
            bench.grade1 = 0;
        }
        // Retain the reference from its loaded ascent shoulder. The pullout
        // shares that source's pitch, curvature, force and energy jets.
        // Both pieces share one physical entry and the same evolving energy.
        source({std::move(paired.camelback.authoring), std::move(paired.camelback.section)},
               Element::Hill, element,
               {.begin = paired.retainedBeginDistance, .recordPort = false});
    }

    void authorWave(const RecipeElement &element) {

        const auto &p = std::get<TurnParameters>(element.parameters);
        FvdWaveRequest intent;
        waveBase = cursor.position;
        intent.entrySpeed = speedFor(element.id);
        intent.entryPitch = std::asin(cursor.tangent.z);
        intent.entry = motion.fvdEntry(intent.entrySpeed);
        const auto upright = unit(Vec3{0, 0, 1} - cursor.tangent * cursor.tangent.z);
        intent.entryNormalG =
            dot(cursor.curvature * (intent.entrySpeed * intent.entrySpeed) + Vec3{0, 0, gravity},
                upright) /
            gravity;
        intent.entryHoldSeconds = 1.8;
        intent.height = p.riseMeters;
        intent.exitHeight = 10;
        intent.turnAngle = p.headingDegrees * pi / 180;
        intent.bankAngle = p.bankDegrees * pi / 180;
        intent.exitNormalG = p.exitNormalG;
        lossFor(intent);
        source(designFvdWave(intent, cancel), Element::Turn, element, {.worldCoordinates = true});
    }

    void authorLoop(const RecipeElement &element) {

        const auto &p = std::get<InversionParameters>(element.parameters);
        FvdLoopRequest intent;
        loopBase = cursor.position;
        intent.entrySpeed = speedFor(element.id);
        intent.height = p.referenceRiseMeters * std::pow(intent.entrySpeed / 65, 2);
        intent.yawAngle = p.yawDegrees * pi / 180;
        intent.crestG = p.crestG;
        intent.exitPitch = p.exitPitchDegrees * pi / 180;
        intent.exitNormalG = 1.5;
        intent.normalG = 3.8;
        intent.exitPositiveG = 3.8;
        // The default loop keeps parallel arms and a signed 18 m crossing.
        // Explicit yaw recipes retain their prescribed plane-turn family.
        intent.crossingOffset = std::abs(p.yawDegrees) < 1e-9 ? 18 : 0;
        intent.ascentReleaseSeconds = intent.crossingOffset != 0 ? 0 : 1.2;
        intent.entry = motion.fvdEntry(intent.entrySpeed);
        lossFor(intent);
        source(designFvdLoop(intent, cancel), Element::Inversion, element,
               {.worldCoordinates = true});
        // A low rollover spaces the following half-loop without steering
        // either inversion sideways or inserting a level straight.
        const Vec3 valley = cursor.position;
        auto crest = detail::planarJet({cursor.position.x, cursor.position.y, valley.z + 6},
                                       heading(), 0, -.0025);
        terrainCurve(crest, 90, Element::Hill, element.id + "-ground-rollover-crest");
        auto recovery = detail::planarJet({cursor.position.x, cursor.position.y, valley.z},
                                          heading(), 0, .0014);
        terrainCurve(recovery, 90, Element::Hill, element.id + "-ground-rollover-valley");
        if (req.terrain.kind == TerrainKind::Highlands)
            d.request.terrain.ramps.push_back({valley.x, valley.y, cursor.position.x,
                                               cursor.position.y, std::max(0., valley.z - datum),
                                               std::max(0., cursor.position.z - datum), 0, 0, 100});
    }

    void authorImmelmann(const RecipeElement &element) {

        const auto &p = std::get<InversionParameters>(element.parameters);
        FvdImmelmannRequest intent;
        intent.entrySpeed = speedFor(element.id);
        intent.height = p.referenceRiseMeters * std::pow(intent.entrySpeed / 53, 2);
        intent.exitHeight = std::abs(p.yawDegrees) < 1e-9 ? 20 : 10;
        intent.exitPitch = p.exitPitchDegrees * pi / 180;
        intent.normalG = 3.8;
        intent.exitPositiveG = 3;
        intent.crestG = p.crestG;
        intent.rollExitG = .6;
        intent.rollReleaseFraction = 0;
        intent.planarRoll = std::abs(p.yawDegrees) < 1e-9;
        intent.ascentReleaseSeconds = intent.planarRoll ? 0 : 1.2;
        intent.rampSeconds = 1.0;
        intent.exitRampSeconds = 1.2;
        intent.rollOverlapFraction = intent.planarRoll ? .35 : 0;
        intent.yawAngle = p.yawDegrees * pi / 180;
        intent.exitNormalG = std::cos(intent.exitPitch);
        intent.hand = -1;
        lossFor(intent);
        // The preceding rollover has a prescribed vertical plane. Resolve
        // its upright physical frame analytically before the FVD handoff;
        // its spline placeholders are not valid source-frame derivatives.
        const auto angles = detail::directionAngles(cursor);
        if (std::abs(angles[1].first) + std::abs(angles[1].second) + std::abs(angles[1].third) >
            1e-10)
            throw std::runtime_error("Immelmann rollover must retain its vertical entry plane");
        auto port = d.track.knots.back();
        const auto pitch = angles[0];
        port.up = unit(Vec3{0, 0, 1} - cursor.tangent * cursor.tangent.z);
        port.upFirst = cursor.tangent * (-pitch.first);
        port.upSecond = cursor.curvature * (-pitch.first) - cursor.tangent * pitch.second;
        port.upThird = cursor.third * (-pitch.first) - cursor.curvature * (2 * pitch.second) -
                       cursor.tangent * pitch.third;
        intent.entry = makeFvdEntry(port, intent.entrySpeed, gravity * req.train.rollingResistance,
                                    motion.drag);
        const auto inversion = designFvdImmelmann(intent, cancel);
        source({inversion.authoring, inversion.section}, Element::Inversion, element,
               {.worldCoordinates = true});
        immelExit = cursor.position;
    }

    void authorSignature(const RecipeElement &element) {

        const auto &p = std::get<SweepParameters>(element.parameters);
        // Aim the descending signature across the valley toward a staging
        // corridor west of the station. Its average bearing includes half of
        // the authored wing sweep; the final hill then retains room to turn
        // into the eastbound brake run without circling behind the station.
        const auto &brakes = std::get<OperationParameters>(req.recipe.elements.back().parameters);
        const double returnRadius =
            motion.nominalSpeed * motion.nominalSpeed / (gravity * std::sqrt(3.2 * 3.2 - 1));
        const Vec3 staging{-brakes.lengthMeters - 2 * halfTrain - 3 - 3 * returnRadius,
                           -2 * returnRadius, datum};
        const double bearing =
            std::atan2(staging.y - cursor.position.y, staging.x - cursor.position.x);
        const double desiredHeading = bearing - .5 * p.headingDegrees * pi / 180;
        const double turn = std::remainder(desiredHeading - heading(), 2 * pi);
        // A placement bearing must bring its physical turn radius with it.
        // The septic yaw transition peaks at 2.1875 times its mean rate.
        const double turnLength = 2.1875 * std::abs(turn) * motion.nominalSpeed *
                                  motion.nominalSpeed / (gravity * std::tan(65 * pi / 180));
        freeDirection({0, 0, 0, 0}, {heading() + turn, 0, 0, 0},
                      std::max({110., motion.nominalSpeed * 2.5, turnLength}), Element::Turn,
                      element.id + "-approach");
        const Vec3 signatureEntry = cursor.position;
        const double signatureYaw = heading(),
                     floor = std::max(datum + 2, cursor.position.z + p.riseMeters),
                     drop = cursor.position.z - floor, scale = p.lengthMeters / 540;
        // Riftwake: an outward floating wing, a falling transfer across
        // the ravine, an opposing wing and a low carving dive. The two
        // crests share one descending silhouette and authored bearing.
        Vec3 transfer;
        std::array<Vec3, 2> signatureCrests;
        for (int wing = 0; wing < 2; ++wing) {
            const double rise = (wing == 0 ? 25 : 20) * scale, base = cursor.position.z;
            const double crestYaw =
                signatureYaw + p.headingDegrees * pi / 180 * (wing == 0 ? .18 : .68);
            auto top = detail::planarJet(
                {cursor.position.x, cursor.position.y, base + rise}, crestYaw, 0,
                motion.crestCurvature(rise, 160 * scale, p.negativeG * req.style.airtime));
            const double bank = (wing == 0 ? 1 : -1) * p.rollDegrees *
                                (req.style.signatureRollDegrees / 45) * pi / 180;
            terrainCurve(top, 160 * scale, Element::Airtime,
                         element.id + (wing == 0 ? "-outward-wing" : "-counter-wing"), bank);
            signatureCrests[wing] = cursor.position;
            if (wing == 0)
                landmarks.push_back({LandmarkKind::SignatureRelease, d.track.knots.size() - 1});
            const double lowHeight = wing == 0 ? floor + drop * .48 : floor;
            const double lowYaw = signatureYaw + p.headingDegrees * pi / 180 * (wing == 0 ? .5 : 1);
            auto low = detail::planarJet({cursor.position.x, cursor.position.y, lowHeight}, lowYaw,
                                         wing == 0 ? 0 : p.exitPitchDegrees * pi / 180);
            terrainCurve(low, (wing == 0 ? 200 : 220) * scale, Element::Turn,
                         element.id + (wing == 0 ? "-ravine-transfer" : "-low-carve"));
            if (wing == 0)
                transfer = cursor.position;
        }
        returnRavine = cursor.position;
        if (req.terrain.kind == TerrainKind::Highlands) {
            auto &t = d.request.terrain;
            // A descending terrace connects the approach and the two valley
            // ports. A shallow stream cut leaves the hills themselves intact.
            auto terrace = [&](Vec3 a, Vec3 b, double gap0, double gap1, double width) {
                const double h0 = std::max(0., a.z - gap0), h1 = std::max(0., b.z - gap1);
                const double grade = (h1 - h0) / std::hypot(b.x - a.x, b.y - a.y);
                t.ramps.push_back({a.x, a.y, b.x, b.y, h0, h1, grade, grade, width});
            };
            // The upper approach bridges the lower inversion basin. Its
            // shared landform must remain below the lower loop/rollover.
            const double basin = std::max(0., loopBase.z - datum);
            terrace(immelExit, signatureEntry, immelExit.z - basin, signatureEntry.z - basin, 140);
            const auto wingSide =
                unit(Vec3{-(transfer - signatureEntry).y, (transfer - signatureEntry).x, 0}) *
                (p.headingDegrees < 0 ? 50. : -50.);
            terrace(signatureEntry + wingSide, transfer + wingSide, 5, 5, 80);
            terrace(transfer, cursor.position, 5, 4.5, 180);
            for (const auto crest : signatureCrests)
                t.foothills.push_back({crest.x, crest.y, std::max(6., crest.z - 16), 120});
            t.ravines.push_back(
                {signatureEntry.x, signatureEntry.y, transfer.x, transfer.y, 3, 3, 70, 80});
            t.ravines.push_back(
                {transfer.x, transfer.y, cursor.position.x, cursor.position.y, 3, 2, 80, 70});
        }
    }

    void authorReturn(const RecipeElement &element) {
        const double turnVariation =
            seeded(elementSeed(elementSeed(req.seed, element.id), "heading"), -.06, .06);

        std::visit(
            [&](const auto &p) {
                using P = std::decay_t<decltype(p)>;
                if constexpr (std::is_same_v<P, HillParameters>) {
                    FvdHillRequest intent;
                    intent.entrySpeed = speedFor(element.id);
                    intent.height = p.riseMeters * p.profileScale;
                    intent.positiveG = p.pulloutG;
                    intent.airtimeG = p.negativeG * req.style.airtime;
                    intent.rampSeconds = p.releaseSeconds;
                    intent.exitRampSeconds = p.recoverySeconds;
                    intent.twistAngle =
                        (p.twistDegrees + (req.style.returnStyle ? 25. : 0)) * pi / 180;
                    const size_t next = size_t(&element - req.recipe.elements.data()) + 1;
                    const bool terminal =
                        next < req.recipe.elements.size() &&
                        req.recipe.elements[next].anchor == TerrainAnchor::Approach;
                    intent.exitHeight = terminal ? datum - cursor.position.z : 0;
                    intent.exitPitch = terminal ? 0 : -.12;
                    intent.exitNormalG = terminal ? 1 : 0;
                    lossFor(intent);
                    intent.entry = motion.fvdEntry(intent.entrySpeed);
                    const size_t hillBegin = d.track.knots.size() - 1;
                    source(designFvdHill(intent, cancel), Element::Airtime, element,
                           {.worldCoordinates = true});
                    if (terminal && req.terrain.kind == TerrainKind::Highlands) {
                        const auto crest =
                            std::max_element(d.track.knots.begin() + hillBegin, d.track.knots.end(),
                                             [](const Knot &a, const Knot &b) {
                                                 return a.position.z < b.position.z;
                                             });
                        d.request.terrain.foothills.push_back({crest->position.x, crest->position.y,
                                                               std::max(0., crest->position.z - 18),
                                                               160});
                    }
                } else if constexpr (std::is_same_v<P, TurnParameters>) {
                    const double angle = p.headingDegrees * pi / 180 + turnVariation;
                    const double length = std::max(
                        p.lengthMeters,
                        1.875 * std::abs(angle) * motion.nominalSpeed * motion.nominalSpeed /
                            (gravity * std::tan(std::abs(p.bankDegrees) * pi / 180)));
                    auto end = detail::planarJet(
                        {cursor.position.x, cursor.position.y, cursor.position.z + p.riseMeters},
                        heading() + angle, 0,
                        (p.exitNormalG - 1) * gravity /
                            (motion.nominalSpeed * motion.nominalSpeed));
                    auto program = solveHeightMotion(cursor, end, length,
                                                     {motion.nominalSpeed,
                                                      gravity * req.train.rollingResistance,
                                                      motion.drag, 0, 4.4, -.8},
                                                     cancel);
                    motion.programme(program, Element::Turn, element.id.c_str());
                    motion.modules.back().reversals = 1;
                } else if constexpr (std::is_same_v<P, SweepParameters>) {
                    if (element.anchor != TerrainAnchor::Approach) {
                        auto end = detail::planarJet({cursor.position.x, cursor.position.y,
                                                      cursor.position.z + p.riseMeters},
                                                     heading() + p.headingDegrees * pi / 180,
                                                     p.exitPitchDegrees * pi / 180);
                        motion.programme(
                            solveHeightMotion(cursor, end, p.lengthMeters,
                                              {motion.nominalSpeed,
                                               gravity * req.train.rollingResistance, motion.drag,
                                               p.rollDegrees * pi / 180, 4.4, -.8},
                                              cancel),
                            Element::Turn, element.id.c_str(), p.rollDegrees * pi / 180);
                        motion.modules.back().reversals = 1;
                        return;
                    }
                    if (req.terrain.kind == TerrainKind::Highlands)
                        d.request.terrain.ravines.push_back({returnRavine.x, returnRavine.y,
                                                             cursor.position.x, cursor.position.y,
                                                             3, 2, 70, 90});
                    // A reordered final hill may still be descending while
                    // pulling out. Finish that same curvature release before
                    // asking a placed level corridor to reach the station.
                    const auto jets = detail::directionAngles(cursor);
                    if (std::abs(jets[0].value) > 1e-5) {
                        auto released = [](AngleJet q, double length) {
                            return q.value + q.first * length * .5 +
                                   q.second * length * length * .1 +
                                   q.third * length * length * length / 120;
                        };
                        double length = std::max(40., motion.nominalSpeed * 2);
                        double lo = 1, hi = 2;
                        while (hi < 400 && released(jets[0], lo) * released(jets[0], hi) > 0) {
                            lo = hi;
                            hi *= 1.25;
                        }
                        if (hi < 400) {
                            for (int k = 0; k < 48; ++k) {
                                const double mid = (lo + hi) * .5;
                                if (released(jets[0], lo) * released(jets[0], mid) > 0)
                                    lo = mid;
                                else
                                    hi = mid;
                            }
                            length = (lo + hi) * .5;
                        }
                        const auto program = polynomialMotion(
                            cursor, detail::anglePolynomial(jets[0], {0, 0, 0, 0}, length),
                            detail::anglePolynomial(jets[1], {released(jets[1], length), 0, 0, 0},
                                                    length),
                            length);
                        for (int k = 1; k < 100; ++k) {
                            const double pitch =
                                std::asin(program.direction(length * k / 100).tangent.z);
                            if (pitch * jets[0].value < -1e-10 ||
                                std::abs(pitch) > std::abs(jets[0].value) + 1e-7)
                                throw std::runtime_error("The incoming grade cannot release "
                                                         "monotonically into the low return");
                        }
                        motion.programme(program, Element::Turn,
                                         (element.id + "-grade-release").c_str());
                    }
                    // The valley already places this port outside the plateau.
                    // Reach the station approach directly instead of detouring
                    // outward to a second, redundant landscape waypoint.
                    const auto &brakes =
                        std::get<OperationParameters>(req.recipe.elements.back().parameters);
                    brakeAlignment = 2 * halfTrain + 3;
                    const Vec3 approach{-brakes.lengthMeters - brakeAlignment, 0, datum};
                    if (req.terrain.kind == TerrainKind::Highlands)
                        d.request.terrain.ravines.push_back({cursor.position.x, cursor.position.y,
                                                             approach.x, approach.y, 16, 8, 100,
                                                             150});
                    const size_t begin = d.track.knots.size() - 1;
                    motion.nominalSpeed = speedFor(element.id);
                    closeRouteApproach(motion, approach, 0, element.id.c_str(), cancel);
                    for (size_t k = begin; k < d.track.knots.size(); ++k)
                        if (d.track.knots[k].position.z < datum - .05)
                            throw std::runtime_error("Low return corridor left its basin floor; no "
                                                     "clearance lift is inserted");
                } else
                    throw std::runtime_error("Unsupported return parameter family");
            },
            element.parameters);
    }

    void authorBrakes(const RecipeElement &element) {

        const auto &p = std::get<OperationParameters>(element.parameters);
        brakeCapacity = -p.accelerationMps2;
        const double alignment = brakeAlignment > 0 ? brakeAlignment : 2 * halfTrain + 3;
        motion.line(alignment, Element::Brake, "brake-alignment");
        brakeBegin = d.track.knots.size() - 1;
        motion.line(p.lengthMeters, Element::Brake, element.id.c_str());
    }

    void authorElement(const RecipeElement &element) {
        using Author = void (RouteDraft::*)(const RecipeElement &);
        static constexpr std::pair<RideRole, Author> authors[]{
            {RideRole::Station, &RouteDraft::authorStation},
            {RideRole::Departure, &RouteDraft::authorDeparture},
            {RideRole::Opening, &RouteDraft::authorOpening},
            {RideRole::CliffApproach, &RouteDraft::authorCliffApproach},
            {RideRole::CliffLip, &RouteDraft::authorCliffLip},
            {RideRole::CliffDrop, &RouteDraft::authorCliffDrop},
            {RideRole::DownhillLaunch, &RouteDraft::authorDownhillLaunch},
            {RideRole::Camelback, &RouteDraft::authorCamelback},
            {RideRole::Wave, &RouteDraft::authorWave},
            {RideRole::Loop, &RouteDraft::authorLoop},
            {RideRole::Immelmann, &RouteDraft::authorImmelmann},
            {RideRole::Signature, &RouteDraft::authorSignature},
            {RideRole::Return, &RouteDraft::authorReturn},
            {RideRole::Brakes, &RouteDraft::authorBrakes},
        };
        const auto found =
            std::find_if(std::begin(authors), std::end(authors),
                         [&](const auto &item) { return item.first == element.role; });
        if (found == std::end(authors))
            throw std::invalid_argument("Unspecified recipe role");
        (this->*found->second)(element);
    }

  public:
    RouteDraft(const GenerationRequest &input, std::vector<RecipePort> &reached, Cancel stop,
               Progress callback)
        : plan(input), d(initialize(plan)), req(d.request), energy(req.train),
          progress(std::move(callback)), cancel(std::move(stop)), ports(reached), motion(d, cancel),
          cursor(motion.cursor), datum(motion.datum),
          halfTrain((req.train.cars - 1) * req.train.spacing * .5), hand(plan.hand) {
        ports.clear();
    }
    Design compose() && {
        for (const auto &element : req.recipe.elements) {
            if (cancel && cancel())
                throw std::runtime_error("CANCELLED");
            const size_t moduleBegin = motion.modules.size();
            if (progress)
                progress({WorkPhase::Authoring, 0, 0, 0, "Authoring " + element.id});
            try {
                authorElement(element);
                if (element.role != RideRole::Brakes)
                    refreshEnergy();
            } catch (const std::exception &failure) {
                d.track.closed = false;
                d.report.fail("AUTHORING", element.id + ": " + failure.what());
                throw RecipeCompileFailure(element.id + ": " + failure.what(), std::move(d), ports);
            }
            compiled.push_back({moduleBegin, motion.modules.size(), element.role, element.id});
        }
        if ((norm(cursor.position - d.track.knots.front().position) > 1e-7 ||
             norm(cursor.tangent - d.track.knots.front().tangent) > 1e-7 ||
             norm(cursor.curvature) > 1e-7 || norm(cursor.third) > 1e-7 ||
             norm(cursor.fourth) > 1e-7)) {
            std::ostringstream message;
            message << std::setprecision(17)
                    << "Recipe station closure failed its complete endpoint jet: position="
                    << cursor.position.x << ',' << cursor.position.y << ',' << cursor.position.z
                    << " tangent=" << cursor.tangent.x << ',' << cursor.tangent.y << ','
                    << cursor.tangent.z << " curvature=" << norm(cursor.curvature)
                    << " third=" << norm(cursor.third) << " fourth=" << norm(cursor.fourth);
            throw std::runtime_error(message.str());
        }
        d.track.knots.back() = d.track.knots.front();
        d.track.closed = true;
        for (auto &k : d.track.knots) {
            k.position.y *= hand;
            k.tangent.y *= hand;
            k.curvature.y *= hand;
            k.third.y *= hand;
            k.fourth.y *= hand;
            k.up.y *= hand;
            k.upFirst.y *= hand;
            k.upSecond.y *= hand;
            k.upThird.y *= hand;
            k.bank *= hand;
        }
        auto &terrain = d.request.terrain;
        terrain.centerY *= hand;
        terrain.bend *= hand;
        terrain.cliffY *= hand;
        terrain.cliffHeading *= hand;
        if (terrain.backSlope) {
            terrain.backSlope->y *= hand;
            terrain.backSlope->gradeY *= hand;
        }
        for (auto &ramp : terrain.ramps) {
            ramp.y0 *= hand;
            ramp.y1 *= hand;
        }
        for (auto &knoll : terrain.knolls)
            knoll.y *= hand;
        for (auto &ravine : terrain.ravines) {
            ravine.y0 *= hand;
            ravine.y1 *= hand;
        }
        for (auto &foothill : terrain.foothills)
            foothill.y *= hand;
        for (auto &program : d.forcePrograms)
            program.hand = hand;
        for (auto &program : d.splinePrograms)
            program.hand = hand;
        if (!terrain.valid())
            throw std::runtime_error("Resolved semantic terrain is outside its bounded domain");
        finalize();
        d.topology = "route/" + d.request.recipe.name;
        std::ostringstream diagnostics;
        diagnostics << std::setprecision(12)
                    << "{\"schemaVersion\":4,\"generationOnly\":true,\"recipeVersion\":"
                    << req.recipe.version << ",\"seed\":" << req.seed << ",\"hand\":" << hand
                    << ",\"ports\":[";
        for (size_t i = 0; i < ports.size(); ++i) {
            if (i)
                diagnostics << ',';
            diagnostics << "{\"id\":" << std::quoted(ports[i].id)
                        << ",\"distance\":" << ports[i].distance
                        << ",\"plannedSpeed\":" << ports[i].speed << '}';
        }
        diagnostics << "],\"terrainAnchors\":[";
        bool comma = false;
        for (const auto &[name, position] :
             std::array<std::pair<const char *, Vec3>, 5>{{{"plateau", plateauArrival},
                                                           {"cliff-foot", cliffFoot},
                                                           {"wave-bench", waveBase},
                                                           {"loop-basin", loopBase},
                                                           {"immelmann-shoulder", immelExit}}}) {
            if (comma)
                diagnostics << ',';
            comma = true;
            diagnostics << "{\"name\":" << std::quoted(name) << ",\"position\":[" << position.x
                        << ',' << position.y * hand << ',' << position.z << "]}";
        }
        diagnostics << "]}";
        d.planningDiagnostics = diagnostics.str();
        return std::move(d);
    }
};
} // namespace
Design composeRoute(const GenerationRequest &input, std::vector<RecipePort> &ports, Cancel cancel,
                    Progress progress) {
    return RouteDraft(input, ports, std::move(cancel), std::move(progress)).compose();
}
} // namespace coaster
