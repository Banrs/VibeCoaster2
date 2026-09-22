#include "coaster/recipe.hpp"
#include "coaster/elements.hpp"
#include "coaster/journey.hpp"
#include "coaster/simulation.hpp"
#include <map>

#include <charconv>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace coaster {
namespace {
double parseNumber(const std::string &value) {
    double x{};
    const auto [end, ec] = std::from_chars(value.data(), value.data() + value.size(), x);
    if (ec != std::errc{} || end != value.data() + value.size() || !std::isfinite(x))
        throw std::runtime_error("Invalid recipe number");
    return x;
}
void range(double x, double lo, double hi, const char *field) {
    if (!std::isfinite(x) || x < lo || x > hi)
        throw std::runtime_error(std::string("Unsupported recipe value: ") + field);
}
} // namespace
void validateRecipe(const Recipe &r) {
    if (r.version != 1)
        throw std::runtime_error("Unsupported recipe schema");
    if (r.style != "balanced" && r.style != "flow" && r.style != "intense")
        throw std::runtime_error("Unsupported ride style");
    range(r.plateau, 200, 220, "plateau");
    range(r.openingHeight, 55, 95, "openingHeight");
    range(r.camelbackHeight, 200, 240, "camelbackHeight");
    range(r.loopHeight, 100, 150, "loopHeight");
    range(r.immelmannHeight, 75, 120, "immelmannHeight");
    range(r.topSpeedKph, 285, 310, "topSpeedKph");
    range(r.activeSeconds, 180, 180, "activeSeconds");
    range(r.terminalSeconds, 5, 10, "terminalSeconds");
}
Recipe readRecipe(const std::filesystem::path &path) {
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot open recipe");
    Recipe r;
    std::string line;
    std::set<std::string> seen;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.empty() || line[0] == '#')
            continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos)
            throw std::runtime_error("Recipe needs key=value");
        const auto key = line.substr(0, eq), value = line.substr(eq + 1);
        if (!seen.insert(key).second)
            throw std::runtime_error("Duplicate recipe field");
        if (key == "style")
            r.style = value;
        else if (key == "version") {
            const double n = parseNumber(value);
            if (n != 1)
                throw std::runtime_error("Unsupported recipe version");
            r.version = 1;
        } else if (key == "seed") {
            const double n = parseNumber(value);
            range(n, 0, 4294967295., "seed");
            if (std::floor(n) != n)
                throw std::runtime_error("Seed must be integer");
            r.seed = static_cast<unsigned>(n);
        } else {
            double *field = nullptr;
            if (key == "plateau")
                field = &r.plateau;
            else if (key == "openingHeight")
                field = &r.openingHeight;
            else if (key == "camelbackHeight")
                field = &r.camelbackHeight;
            else if (key == "loopHeight")
                field = &r.loopHeight;
            else if (key == "immelmannHeight")
                field = &r.immelmannHeight;
            else if (key == "topSpeedKph")
                field = &r.topSpeedKph;
            else if (key == "activeSeconds")
                field = &r.activeSeconds;
            else if (key == "terminalSeconds")
                field = &r.terminalSeconds;
            else
                throw std::runtime_error("Unknown recipe field: " + key);
            *field = parseNumber(value);
        }
    }
    if (!seen.contains("version"))
        throw std::runtime_error("Missing recipe version");
    validateRecipe(r);
    return r;
}
void writeRecipe(const Recipe &r, const std::filesystem::path &path) {
    validateRecipe(r);
    std::ofstream f(path);
    if (!f)
        throw std::runtime_error("Cannot write recipe");
    f << std::setprecision(17) << "version=" << r.version << "\nseed=" << r.seed << "\nstyle=" << r.style
      << "\nplateau=" << r.plateau << "\nopeningHeight=" << r.openingHeight
      << "\ncamelbackHeight=" << r.camelbackHeight << "\nloopHeight=" << r.loopHeight
      << "\nimmelmannHeight=" << r.immelmannHeight << "\ntopSpeedKph=" << r.topSpeedKph
      << "\nactiveSeconds=" << r.activeSeconds << "\nterminalSeconds=" << r.terminalSeconds << '\n';
    if (!f)
        throw std::runtime_error("Recipe write failed");
}
void validateAuthoring(const Design &design, const Cancel &cancel) {
    validateRecipe(design.recipe);
    std::set<std::string> ids;
    for (const auto &p : design.track.source) {
        poll(cancel);
        if (p.id.empty() || !ids.insert(p.id).second)
            throw std::runtime_error("Ride authoring IDs are empty or duplicated");
    }
    struct SourceRole {
        const char *id;
        Role role;
        bool spline;
    };
    const std::array<SourceRole, 20> story{{{"launch-180", Role::Launch, false},
                                            {"opening", Role::Opening, false},
                                            {"ascent-lsm", Role::Ascent, true},
                                            {"plateau-sweep", Role::Clifftop, true},
                                            {"plateau-weave", Role::Clifftop, true},
                                            {"edge", Role::Edge, false},
                                            {"ridge-turn", Role::Clifftop, true},
                                            {"ridge-sweep", Role::Clifftop, true},
                                            {"lip-brake", Role::Lip, false},
                                            {"cliff", Role::Cliff, false},
                                            {"lsm-300", Role::DownhillLaunch, false},
                                            {"camelback", Role::Camelback, false},
                                            {"wave", Role::Wave, false},
                                            {"loop", Role::Loop, false},
                                            {"valley-carve", Role::Journey, true},
                                            {"immelmann", Role::Immelmann, false},
                                            {"ravine-roll", Role::Ravine, true},
                                            {"ravine-exit", Role::Airtime, false},
                                            {"terminal-entry", Role::TerminalOverpass, true},
                                            {"terminal", Role::Terminal, false}}};
    auto at = design.track.source.begin();
    for (const auto &required : story) {
        at = std::find_if(at, design.track.source.end(),
                          [&](const Program &p) { return p.id == required.id; });
        if (at == design.track.source.end() || at->role != required.role ||
            (!at->geometry.empty()) != required.spline)
            throw std::runtime_error(std::string("Ride authoring sequence/type mismatch: ") + required.id);
        ++at;
    }
    const auto &first = design.track.source.front();
    if (first.id != "launch-180" || design.track.source.back().id != "terminal" ||
        std::abs(first.initial.v) > 1e-9 || std::abs(first.duration() - 1.4) > 1e-9 ||
        std::abs(shoot(first, .005, cancel).end.v - 50) > .001)
        throw std::runtime_error("Ride authoring launch/terminal contract mismatch");
    double brakingSeconds = 0, sourceSeconds = 0;
    for (const auto &p : design.track.source) {
        sourceSeconds += p.duration();
        if (isTerminal(p.role))
            brakingSeconds += p.duration();
        if (p.role == Role::Lip && (p.duration() < 1 || p.duration() > 5))
            throw std::runtime_error("Ride authoring lip braking is not brief");
    }
    if (sourceSeconds > 230 || std::abs(brakingSeconds - design.recipe.terminalSeconds) > .001)
        throw std::runtime_error("Ride authoring braking duration mismatch");
    struct Required {
        const char *id;
        Role role;
        double height;
    };
    const std::array<Required, 4> required{{{"opening", Role::Opening, design.recipe.openingHeight},
                                            {"camelback", Role::Camelback, design.recipe.camelbackHeight},
                                            {"loop", Role::Loop, design.recipe.loopHeight},
                                            {"immelmann", Role::Immelmann, design.recipe.immelmannHeight}}};
    for (const auto &target : required) {
        const auto found = std::find_if(design.track.source.begin(), design.track.source.end(),
                                        [&](const Program &p) { return p.id == target.id; });
        if (found == design.track.source.end() || found->role != target.role || !found->geometry.empty())
            throw std::runtime_error(std::string("Ride authoring source is missing or has the wrong type: ") +
                                     target.id);
        const auto shot = shoot(*found, .005, cancel);
        const double actual = shot.maximumHeight - found->initial.p.z;
        if (!std::isfinite(actual) || std::abs(actual - target.height) > .03)
            throw std::runtime_error(std::string("Ride authoring height mismatch: ") + target.id + " is " +
                                     std::to_string(actual) + " m; recipe requests " +
                                     std::to_string(target.height) + " m");
    }
    const auto ascent = std::find_if(design.track.source.begin(), design.track.source.end(),
                                     [](const Program &p) { return p.id == "ascent-lsm"; });
    if (ascent == design.track.source.end() || ascent->role != Role::Ascent || ascent->geometry.empty() ||
        std::abs(shoot(*ascent, .005, cancel).end.p.z - ascent->initial.p.z - design.recipe.plateau) > .03)
        throw std::runtime_error("Ride authoring ascent does not reach the requested plateau");
}
Program launch(const State &state, double targetSpeed, double seconds) {
    Program p;
    p.initial = state;
    constexpr double ramp = .25, ease = .015;
    auto controls = [&](double peak) {
        const double j = peak / (ramp - ease), a = .5 * ease * j, b = seconds - ramp;
        p.controls = {{0, 1, 0, 0, 0},
                      {ease, 1, 0, 0, a},
                      {ramp - ease, 1, 0, 0, peak - a},
                      {ramp, 1, 0, 0, peak},
                      {b, 1, 0, 0, peak},
                      {b + ease, 1, 0, 0, peak - a},
                      {seconds - ease, 1, 0, 0, a},
                      {seconds, 1, 0, 0, 0}};
        p.controls[1].first[3] = p.controls[2].first[3] = j;
        p.controls[5].first[3] = p.controls[6].first[3] = -j;
    };
    double lo = 0, hi = 60;
    for (int i = 0; i < 38; ++i) {
        const double peak = (lo + hi) / 2;
        controls(peak);
        if (shoot(p, .005).end.v < targetSpeed)
            lo = peak;
        else
            hi = peak;
    }
    controls((lo + hi) / 2);
    return p;
}
static Design author(const Recipe &recipe, const std::map<std::string, double> &speeds, double boostTarget,
                     double timingCorrection, JourneyHint &journeyHint, const Cancel &cancel) {
    validateRecipe(recipe);
    auto mix = [](unsigned x) {
        x ^= x >> 16;
        x *= 0x7feb352dU;
        x ^= x >> 15;
        x *= 0x846ca68bU;
        x ^= x >> 16;
        return double(x) / 4294967295.;
    };
    const double variation = mix(recipe.seed) - mix(42), style = recipe.style == "flow"      ? -1
                                                                 : recipe.style == "intense" ? 1
                                                                                             : 0;
    double plateauLeg = 420;
    for (int fit = 0; fit < 6; ++fit) {
        Design d;
        d.recipe = recipe;
        std::vector<Program> source;
        State state;
        state.p = {0, 300, 4};
        state.v = 0;
        auto reached = [&](const Program &p, const char *id) {
            auto q = shoot(p, .01, cancel).end;
            if (const auto found = speeds.find(id); found != speeds.end())
                q.v = found->second;
            return q;
        };
        auto append = [&](Program p, const char *id, const char *label, Role role) {
            poll(cancel);
            p.id = id;
            p.label = label;
            p.role = role;
            state = reached(p, id);
            source.push_back(std::move(p));
        };
        append(launch(state, 50, 1.4), "launch-180", "180 km/h launch", Role::Launch);
        HillShape opening;
        opening.height = recipe.openingHeight;
        opening.bankDegrees = -25 + 2 * variation + 2 * style - .8 * std::max(0., 300 - recipe.topSpeedKph);
        opening.positive = 3.6 + .1 * style;
        opening.negative = -1.15 + .009 * std::max(0., recipe.openingHeight - 75);
        opening.descentNegative = -1.20 + .009 * std::max(0., recipe.openingHeight - 75);
        append(hill(state, opening, cancel), "opening", "Rounded hill and twisted descending turn",
               Role::Opening);
        append(ascent(state, recipe.plateau, 704, 50, cancel), "ascent-lsm",
               "Powered ascent of the escarpment", Role::Ascent);
        append(terrainSpline(state, {450, pi}, cancel), "plateau-sweep", "Grounded plateau sweep",
               Role::Clifftop);
        append(terrainSpline(state, {plateauLeg, rad(-60)}, cancel), "plateau-weave",
               "Clifftop direction change", Role::Clifftop);
        append(edgeAct(state, rad(-65), -1.03, cancel), "edge", "Inbank, outward exposure, inbank",
               Role::Edge);

        // Pin the authored cliff approach to the fixed site's escarpment. This is
        // a rigid placement of the ride, never a terrain deformation under rails.
        const Vec3 edgeChord = state.p - source.back().initial.p;
        const double rotation = pi - std::atan2(edgeChord.y, edgeChord.x);
        const Vec3 pivot = state.p, target{0, 20, recipe.plateau + 4};
        auto placed = [&](Vec3 point) { return rotate(point - pivot, {0, 0, 1}, rotation) + target; };
        for (auto &p : source) {
            p.initial.p = placed(p.initial.p);
            p.initial.t = rotate(p.initial.t, {0, 0, 1}, rotation);
            p.initial.u = rotate(p.initial.u, {0, 0, 1}, rotation);
            for (auto &k : p.geometry)
                k.value[1] += rotation;
        }
        state.p = target;
        state.t = rotate(state.t, {0, 0, 1}, rotation);
        state.u = rotate(state.u, {0, 0, 1}, rotation);
        const auto &climb = *std::find_if(source.begin(), source.end(),
                                          [](const Program &p) { return p.id == "ascent-lsm"; });
        const double shoulderOffset = (climb.initial.p.y + shoot(climb, .02, cancel).end.p.y) * .5;
        if (std::abs(shoulderOffset) > .01) {
            plateauLeg -= shoulderOffset / .8;
            continue;
        }
        const State edgeExit = state;
        Program ridgeTurn, ridgeSweep, approach;
        double ridgeLength = 145;
        for (int iteration = 0; iteration < 4; ++iteration) {
            ridgeTurn = terrainSpline(edgeExit,
                                      {ridgeLength, pi / 2 - std::atan2(edgeExit.t.y, edgeExit.t.x)}, cancel);
            ridgeSweep = terrainSpline(reached(ridgeTurn, "ridge-turn"), {360, -pi}, cancel);
            approach = brake(reached(ridgeSweep, "ridge-sweep"), 10, 3.8);
            const double error = shoot(approach, .01, cancel).end.p.y - 15;
            ridgeLength -= error / .8;
            if (std::abs(error) < .01)
                break;
        }
        append(ridgeTurn, "ridge-turn", "Grounded turn away from exposed edge", Role::Clifftop);
        append(ridgeSweep, "ridge-sweep", "Clifftop sweep toward the dive", Role::Clifftop);
        append(approach, "lip-brake", "Brief controlled cliff approach", Role::Lip);
        const State lip = state;
        double drop = recipe.plateau - 20;
        Program cliff, boost;
        for (int i = 0; i < 5; ++i) {
            cliff = dive(lip, drop, rad(-18), cancel);
            boost = inclinedBoost(reached(cliff, "cliff"), boostTarget, cancel);
            const double error = shoot(boost, .01, cancel).end.p.z - 4;
            drop += error;
            if (std::abs(error) < .001)
                break;
        }
        append(cliff, "cliff", "Near-vertical escarpment dive", Role::Cliff);
        append(boost, "lsm-300", "Inclined downhill LSM and low recovery", Role::DownhillLaunch);
        HillShape camel;
        camel.height = recipe.camelbackHeight;
        camel.positive = 2.934128338;
        camel.exitPositive = 3.85 + .025 * std::max(0., 300 - recipe.topSpeedKph);
        camel.negative = -.89598697;
        camel.descentNegative = -1.25;
        camel.riseRamp = 2.4251346;
        camel.releaseRamp = 2.9076106;
        camel.recoveryRamp = 3.2860954;
        camel.exitRelease = 1.;
        camel.crownRelief = .25;
        camel.crownHold = .4;
        camel.crownFraction = .49 + .003 * std::clamp(300 - recipe.topSpeedKph, 0., 10.);
        append(hill(state, camel, cancel), "camelback", "Protected asymmetric planar camelback",
               Role::Camelback);
        append(wave(state, 65, cancel), "wave", "Compact rising 180-degree wave", Role::Wave);
        append(loop(state, recipe.loopHeight, rad(15), cancel), "loop", "Physically yawing loop", Role::Loop);
        append(terrainSpline(state, {400, rad(60)}, cancel), "valley-carve",
               "Low valley carve toward the ravine", Role::Journey);
        append(immelmann(state, recipe.immelmannHeight, 35, cancel,
                         4 - .02 * std::max(0., recipe.topSpeedKph - 300)),
               "immelmann", "Immelmann into the ravine shoulder", Role::Immelmann);
        append(splineTo(state, {790, -660, -8}, 1, cancel), "ravine-roll",
               "Descending full roll through the ravine", Role::Ravine);
        HillShape finale;
        finale.height = 40;
        finale.exitHeight = 4 - state.p.z;
        finale.positive = 3.1 + .1 * style;
        finale.exitPositive = 3.2 + .1 * style;
        finale.negative = -1.08;
        finale.descentNegative = -1.12;
        finale.riseRamp = .9;
        finale.releaseRamp = 1.1;
        finale.recoveryRamp = 1.2;
        finale.exitRelease = .9;
        finale.bankDegrees = 18;
        append(hill(state, finale, cancel), "ravine-exit", "Twisting airtime rise out of the ravine",
               Role::Airtime);
        const State station = source.front().initial;
        double active = 0;
        for (const auto &p : source)
            if (p.role != Role::Lip)
                active += p.duration();
        const double remaining = recipe.activeSeconds - active - timingCorrection;
        double approachSpeed =
            coastingExitSpeed(state.v, remaining, .004 * gravity, .5 * 1.225 * 3 / (6 * 1500));
        approachSpeed = std::sqrt(std::max(4., approachSpeed * approachSpeed - 2 * gravity * 10.5));
        if (const auto it = speeds.find("park-crest"); it != speeds.end())
            approachSpeed = it->second;
        const double speedReduction = std::clamp((300 - recipe.topSpeedKph) / 10, 0., 1.);
        const TerminalShape terminalShape{7.2 + 2.3 * speedReduction, 2 + .5 * speedReduction,
                                          30 + 10 * speedReduction};
        const double stopReference = speeds.contains("terminal-entry") ? speeds.at("terminal-entry") : -1;
        const auto planned =
            planBrakes(approachSpeed, recipe.terminalSeconds, cancel, stopReference, terminalShape);
        const Vec3 brakeStart =
            station.p - station.t * planned.horizontalLength + Vec3{0, 0, planned.entryHeight};
        std::string routeRejections;
        int forceRejects = 0, clearRejects = 0;
        auto routeFilter = [&](const Program &candidate) {
            State approach = shoot(candidate, .01, cancel).end;
            approach.v = approachSpeed;
            auto braking =
                finishBrakes(approach, recipe.terminalSeconds, cancel, stopReference, terminalShape);
            braking.entry.id = "terminal-entry";
            braking.stop.id = "terminal";
            // Include the real continuation so front/rear cars never see an
            // artificial cut at a curved endpoint during candidate evaluation.
            const auto local = compile({candidate, braking.entry, braking.stop}, .03, cancel);
            const auto evaluated = simulate(local, {}, 1. / 240, cancel);
            const bool nominal = std::all_of(evaluated.envelope.begin(), evaluated.envelope.end(),
                                             [](const auto &e) { return e.nominalPassed; });
            if (!evaluated.failures.empty() || !nominal) {
                ++forceRejects;
                if (forceRejects < 4)
                    routeRejections +=
                        " force=" +
                        (evaluated.failures.empty() ? std::string("Nominal authoring target exceeded")
                                                    : evaluated.failures.front()) +
                        " Y=" + std::to_string(evaluated.maximum[0].y) +
                        " rate=" + std::to_string(evaluated.rate[0].y);
                return false;
            }
            auto trial = source;
            trial.push_back(candidate);
            trial.push_back(braking.entry);
            trial.push_back(braking.stop);
            const auto all = compile(std::move(trial), .03, cancel);
            const auto clearance = assessClearance(all, recipe.plateau, 1, cancel);
            const bool clear = clearance.terrainHits == 0 && clearance.trackHits == 0;
            if (!clear) {
                ++clearRejects;
                if (clearRejects < 4)
                    routeRejections +=
                        " clearance=" + std::to_string(clearance.trackHits) + "/" +
                        std::to_string(clearance.terrainHits) + " closed=" + std::to_string(all.closed) +
                        " end-gap=" + std::to_string(norm(all.at(0).p - all.at(all.length).p)) + " pair=" +
                        (clearance.firstTrackS >= 0
                             ? all.source[all.at(clearance.firstTrackS).element].id + "/" +
                                   all.source[all.at(clearance.secondTrackS).element].id
                             : "terrain") +
                        " at=" + std::to_string(clearance.firstTrackS) + "/" +
                        std::to_string(clearance.secondTrackS) +
                        (clearance.firstTrackS >= 0
                             ? " elevations=" + std::to_string(all.at(clearance.firstTrackS).p.z) + "/" +
                                   std::to_string(all.at(clearance.secondTrackS).p.z)
                             : "");
            }
            return clear;
        };
        JourneyParts journey;
        try {
            journey =
                closeJourney(state, brakeStart, station.t, remaining, cancel, routeFilter, &journeyHint);
        } catch (const Cancelled &) {
            throw;
        } catch (const std::runtime_error &e) {
            throw std::runtime_error(std::string(e.what()) +
                                     " force-rejects=" + std::to_string(forceRejects) +
                                     " clearance-rejects=" + std::to_string(clearRejects) + routeRejections);
        }
        for (auto &section : journey.active) {
            section.initial = state;
            section.geometricDuration = replaySpline(section, .5, cancel).back().time;
            const auto id = section.id, label = section.label;
            const auto role = section.role;
            append(std::move(section), id.c_str(), label.c_str(), role);
        }
        const auto braking =
            finishBrakes(state, recipe.terminalSeconds, cancel, stopReference, terminalShape);
        append(braking.entry, "terminal-entry", "Braking descent over the valley crossing",
               Role::TerminalOverpass);
        append(braking.stop, "terminal", "Terminal brake run into the station", Role::Terminal);
        d.track = compile(std::move(source), .02, cancel);
        d.notes.push_back("Complete circuit candidate; activation requires independent acceptance");
        return d;
    }
    throw std::runtime_error("Site shoulder placement did not converge");
}

Design generate(const Recipe &recipe, const Cancel &cancel) {
    // FVD entry velocities are authoring reference values. Calibrate them to
    // the separately integrated finite train, without changing geometry limits.
    JourneyHint journeyHint;
    std::map<std::string, double> speeds;
    double motorTarget = recipe.topSpeedKph / 3.6, timingCorrection = 0;
    for (int iteration = 0; iteration < 12; ++iteration) {
        auto d = author(recipe, speeds, motorTarget, timingCorrection, journeyHint, cancel);
        auto physical = std::make_shared<Simulation>(simulate(d.track, {}, 1. / 240, cancel, false));
        if (!physical->completed)
            throw std::runtime_error("Finite train cannot complete authored route");
        double maximum = 0, boostError = physical->maxSpeed - recipe.topSpeedKph / 3.6;
        for (std::size_t i = 1; i < d.track.source.size(); ++i) {
            const double actual = physical->entrySpeeds[i];
            if (!std::isfinite(actual))
                throw std::runtime_error("Missing finite train source crossing");
            maximum = std::max(maximum, std::abs(actual - d.track.source[i].initial.v));
            speeds[d.track.source[i - 1].id] = actual;
        }
        motorTarget -= boostError;
        timingCorrection += physical->active - recipe.activeSeconds;
        if (maximum < .015 && std::abs(boostError) < .003 &&
            std::abs(physical->active - recipe.activeSeconds) < .01) {
            for (const auto &q : physical->playback)
                d.track.operationSpeed.push_back({q.s, q.speed});
            d.baseline = std::make_shared<Simulation>(simulate(d.track, {}, 1. / 960, cancel));
            validateAuthoring(d, cancel);
            return d;
        }
        if (iteration == 11)
            throw std::runtime_error("Finite train source-speed calibration did not converge: " +
                                     std::to_string(maximum));
    }
    throw std::runtime_error("Unreachable generation state");
}
} // namespace coaster
