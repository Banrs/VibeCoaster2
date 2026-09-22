#include "coaster/simulation.hpp"
#include <limits>

#include <unordered_map>

namespace coaster {
namespace {
constexpr std::array<double, 6> carOffsets{-8.5, -5.1, -1.7, 1.7, 5.1, 8.5};
constexpr std::array<double, 3> seats{8.5, 0, -8.5};
constexpr double seatHeight = 1.2;
struct Dynamics {
    double a{}, potentialSlope{}, drive{}, loss{}, metric{1}, metricS{}, potential{};
};
Dynamics acceleration(const Track &track, double s, double v, const Scenario &scenario,
                      std::array<std::size_t, 6> &hints) {
    Dynamics d;
    d.metric = 0;
    for (std::size_t i = 0; i < carOffsets.size(); ++i) {
        const auto f = track.at(s + carOffsets[i], hints[i]);
        const Vec3 first = f.t + f.upS * .5, second = f.k + f.upSS * .5;
        d.metric += dot(first, first) / 6;
        d.metricS += 2 * dot(first, second) / 6;
        d.potentialSlope += gravity * first.z / 6;
        d.potential += gravity * (f.p.z + .5 * f.u.z) / 6;
    }
    const auto center = track.at(s);
    const auto &p = track.source[center.element];
    // A distributed drive command acts on the train, with finite train mass
    // and offset energy. Spatial motor coverage is a separate hardware audit.
    d.drive = center.drive / scenario.massScale;
    d.loss = (p.rolling * std::tanh(v / .2) + p.drag * scenario.dragScale * v * v) / scenario.massScale;
    if (p.role == Role::Ascent || p.role == Role::DownhillLaunch || p.role == Role::Lip) {
        const double begin = center.element ? track.elementEnds[center.element - 1] : 0,
                     end = track.elementEnds[center.element];
        const double blend = smooth((s - begin) / std::max(1., p.initial.v * .7)) *
                             smooth((end - s) / std::max(1., center.sourceSpeed * .7));
        d.drive += blend * std::clamp(2 * (center.sourceSpeed - v), -4., 4.);
        if (p.role == Role::Lip)
            d.drive = std::min(0., d.drive);
    }
    if (!track.operationSpeed.empty() &&
        (p.id == "valley-carve" || p.id == "plateau-weave" || p.role == Role::Camelback)) {
        const double begin = center.element ? track.elementEnds[center.element - 1] : 0,
                     end = track.elementEnds[center.element];
        const double zoneEnd = p.role == Role::Camelback ? begin + (end - begin) * .72 : end;
        const double zoneBegin = p.id == "plateau-weave" ? std::max(begin, end - 150) : begin;
        const double blend = smooth((s - zoneBegin) / 35) * smooth((zoneEnd - s) / 35);
        auto hi = std::lower_bound(track.operationSpeed.begin(), track.operationSpeed.end(), s,
                                   [](const auto &a, double x) { return a[0] < x; });
        double target = hi == track.operationSpeed.end() ? track.operationSpeed.back()[1] : (*hi)[1];
        if (hi != track.operationSpeed.begin() && hi != track.operationSpeed.end()) {
            const auto &lo = *(hi - 1);
            target = std::lerp(lo[1], (*hi)[1], (s - lo[0]) / ((*hi)[0] - lo[0]));
        }
        // Optional early trims use a tighter target band. Mandatory overspeed
        // protection remains available in full mode.
        const double overspeed = std::max(0., v - target - (scenario.trims ? .1 : .3));
        d.drive -= blend * std::min(3., 2 * overspeed);
    }
    if (isTerminal(p.role)) {
        const double begin = center.element ? track.elementEnds[center.element - 1] : 0;
        const double enable = smooth((s - begin) / std::max(1., p.initial.v * .7));
        d.drive = std::min(0., d.drive + enable * 2 * (center.sourceSpeed - v));
        const double remaining = std::max(1e-8, track.length - s),
                     positionDeceleration = -.8 * v * v / remaining;
        const double controlled =
            d.loss + d.potentialSlope + .5 * d.metricS * v * v + d.metric * positionDeceleration;
        d.drive = std::lerp(d.drive, std::min(0., controlled), smooth((3 - v) / 2));
    }
    d.a = (d.drive - d.loss - d.potentialSlope - .5 * d.metricS * v * v) / d.metric;
    return d;
}
void include(Vec3 x, Vec3 &lo, Vec3 &hi) {
    lo = {std::min(lo.x, x.x), std::min(lo.y, x.y), std::min(lo.z, x.z)};
    hi = {std::max(hi.x, x.x), std::max(hi.y, x.y), std::max(hi.z, x.z)};
}
} // namespace
Simulation simulate(const Track &track, const Scenario &scenario, double dt, const Cancel &cancel,
                    bool assess) {
    if (!(dt > 0 && dt <= .01) || scenario.dragScale <= 0 || scenario.massScale <= 0)
        throw std::runtime_error("Invalid simulation configuration");
    Simulation result;
    result.dt = dt;
    result.assessed = assess;
    result.entrySpeeds.assign(track.source.size(), std::numeric_limits<double>::quiet_NaN());
    result.entrySpeeds[0] = track.source.front().initial.v;
    for (auto &m : result.minimum)
        m = {1e9, 1e9, 1e9};
    for (auto &m : result.maximum)
        m = {-1e9, -1e9, -1e9};
    std::array<std::array<std::vector<double>, 3>, 3> traces;
    for (auto &seat : traces)
        for (auto &axis : seat)
            axis.reserve(std::size_t(210 / dt));
    double s = 0, v = track.source.front().initial.v, t = 0, previousS = 0, previousV = v, initialEnergy = 0,
           workNet = 0;
    std::size_t nextEntry = 1;
    std::array<std::size_t, 6> hints{};
    std::array<std::size_t, 3> seatHints{};
    std::array<Vec3, 3> previous{};
    // Time-commanded launch avoids the singular inverse t(s) at standstill.
    const auto &first = track.source.front();
    for (std::size_t index = 0; index < std::size_t(260 / dt); ++index) {
        poll(cancel);
        while (nextEntry < track.source.size() && s >= track.elementEnds[nextEntry - 1]) {
            const double fraction =
                (track.elementEnds[nextEntry - 1] - previousS) / std::max(1e-12, s - previousS);
            result.entrySpeeds[nextEntry] = std::sqrt(
                std::max(0., std::lerp(previousV * previousV, v * v, std::clamp(fraction, 0., 1.))));
            ++nextEntry;
        }
        Sample q;
        q.time = t;
        q.s = s;
        q.speed = v;
        auto dynamics = acceleration(track, s, v, scenario, hints);
        if (t <= first.duration() && first.role == Role::Launch) {
            dynamics.drive = controlAt(first, t).drive / scenario.massScale;
            dynamics.a =
                (dynamics.drive - dynamics.loss - dynamics.potentialSlope - .5 * dynamics.metricS * v * v) /
                dynamics.metric;
        }
        const double energy = .5 * dynamics.metric * v * v + dynamics.potential;
        if (index == 0)
            initialEnergy = energy;
        else
            result.energyResidual =
                std::max(result.energyResidual, std::abs(energy - initialEnergy - workNet));
        result.maxSpeed = std::max(result.maxSpeed, v);
        if (!result.launchTime && v >= 50 - 1e-5)
            result.launchTime = t;
        if (assess)
            for (std::size_t i = 0; i < seats.size(); ++i) {
                const auto f = track.at(s + seats[i], seatHints[i]);
                const Vec3 firstDerivative = f.t + f.upS * seatHeight,
                           secondDerivative = f.k + f.upSS * seatHeight;
                const Vec3 a =
                    secondDerivative * (v * v) + firstDerivative * dynamics.a + Vec3{0, 0, gravity};
                q.force[i] = {dot(a, f.t) / gravity, dot(a, f.r) / gravity, dot(a, f.u) / gravity};
                include(q.force[i], result.minimum[i], result.maximum[i]);
                traces[i][0].push_back(q.force[i].x);
                traces[i][1].push_back(q.force[i].y);
                traces[i][2].push_back(q.force[i].z);
                if (index) {
                    const Vec3 rate = (q.force[i] - previous[i]) / dt;
                    Vec3 unused{1e9, 1e9, 1e9};
                    include({std::abs(rate.x), std::abs(rate.y), std::abs(rate.z)}, unused, result.rate[i]);
                }
                previous[i] = q.force[i];
            }
        if (index % std::max(std::size_t(1), std::size_t(std::llround(1 / (60 * dt)))) == 0)
            result.playback.push_back(q);
        if (s >= track.length ||
            (track.source.back().role == Role::Terminal && track.length - s < 1e-5 && v < 1e-5)) {
            result.completed = true;
            result.duration = t;
            break;
        }
        const auto role = track.source[track.at(s).element].role;
        if (isTerminal(role))
            result.terminal += dt;
        else if (role != Role::Lip)
            result.active += dt;
        if (role == Role::Clifftop || role == Role::Edge)
            result.clifftopActive += dt;
        if (role == Role::Lip)
            result.lip += dt;
        // Midpoint independent energy evolution on the canonical geometry.
        const double vm = std::max(0., v + dynamics.a * dt / 2), sm = s + v * dt / 2;
        auto middle = acceleration(track, sm, vm, scenario, hints);
        if (t + dt / 2 <= first.duration() && first.role == Role::Launch) {
            middle.drive = controlAt(first, t + dt / 2).drive / scenario.massScale;
            middle.a = (middle.drive - middle.loss - middle.potentialSlope - .5 * middle.metricS * vm * vm) /
                       middle.metric;
        }
        workNet += (middle.drive - middle.loss) * vm * dt;
        previousS = s;
        previousV = v;
        s += vm * dt;
        v += middle.a * dt;
        t += dt;
        if ((v < -.01 && track.source[track.at(s).element].role != Role::Terminal) || !std::isfinite(v) ||
            v > 150) {
            result.failures.push_back("Train stalled or left supported speed domain");
            break;
        }
        v = std::max(0., v);
    }
    if (!result.completed)
        result.failures.push_back("Train did not finish");
    if (!assess)
        return result;
    for (std::size_t i = 0; i < seats.size(); ++i) {
        result.acceleration[i] =
            assessAccelerationF2291_25({traces[i][0], traces[i][1], traces[i][2], dt, 0, i}, cancel);
        if (!result.acceleration[i].passed)
            result.failures.push_back("F2291-25 scoped acceleration assessment failed at seat " +
                                      std::to_string(i));
        const auto lo = result.minimum[i], hi = result.maximum[i], rate = result.rate[i];
        if (lo.x < -4.5 || hi.x > 4.5 || lo.y < -1.5 || hi.y > 1.5 || lo.z < -1.5 || hi.z > 5)
            result.failures.push_back("Nominal force envelope exceeded at seat " + std::to_string(i));
        if (std::max({rate.x, rate.y, rate.z}) > 20)
            result.failures.push_back("Component rate exceeded at seat " + std::to_string(i));
    }
    return result;
}
} // namespace coaster
