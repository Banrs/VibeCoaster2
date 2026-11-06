#include "coaster/ride_feel.hpp"
#include "coaster/angular_motion.hpp"
#include <iomanip>
#include <sstream>

namespace coaster {
namespace {
constexpr double rollThreshold = 3 * pi / 180, quietRotation = 3 * pi / 180;
struct Sample {
    SeatForces force;
    double roll{}, angularSpeed{};
};
struct Episode {
    double quiet{}, rolling{}, beatSeconds{};
    int beat{}, previousBeat{};
};
double mix(double a, double b, double u) {
    return std::lerp(a, b, u);
}
void crossing(std::vector<double> &cuts, double a, double b, double threshold) {
    if (a == b)
        return;
    const double u = (threshold - a) / (b - a);
    if (u > 0 && u < 1)
        cuts.push_back(u);
}
Sample sample(const Design &d, const Frame &f, int seat) {
    const auto k =
        sampleKinematics(d.track, f.distance + seatDistanceOffset(d.request.train, seat));
    const auto angular = signedAngularMotion(k, f.speed, f.acceleration, f.accelerationRate);
    return {f.seats[seat], angular.velocity[0],
            std::hypot(angular.velocity[0], angular.velocity[1], angular.velocity[2])};
}
bool valid(const Frame &f) {
    if (!std::isfinite(f.time) || !std::isfinite(f.distance) || !std::isfinite(f.speed) ||
        f.speed < 0 || !std::isfinite(f.acceleration) || !std::isfinite(f.accelerationRate))
        return false;
    for (const auto &s : f.seats)
        if (!std::isfinite(s.vertical) || !std::isfinite(s.lateral) ||
            !std::isfinite(s.longitudinal))
            return false;
    return true;
}
} // namespace
RideFeel measureRideFeel(const Design &d) {
    RideFeel result;
    const auto &frames = d.simulation.frames;
    if (d.track.spans.empty() || frames.size() < 2) {
        result.error = "Ride feel requires canonical geometry and a replay";
        return result;
    }
    for (size_t i = 0; i < frames.size(); ++i)
        if (!valid(frames[i]) || (i && (frames[i].time <= frames[i - 1].time ||
                                        frames[i].distance < frames[i - 1].distance))) {
            result.error = "Ride feel requires finite forward samples with increasing elapsed time";
            return result;
        }
    const double halfTrain = (d.request.train.cars - 1) * d.request.train.spacing * .5;
    // Terminal stopping/positioning is intentionally outside the experience
    // interval. Do not let a long station dwell dilute rolling duty or pacing.
    double finish = d.track.length;
    for (const auto &op : d.operations)
        if (op.kind == DriveKind::Station)
            finish = std::min(finish, op.start);
    std::vector<std::pair<double, double>> hardware;
    for (const auto &op : d.operations)
        if (op.kind != DriveKind::Station)
            hardware.push_back({op.start - halfTrain, op.end + halfTrain});
    std::array<Episode, 3> episodes{};
    try {
        std::array<Sample, 3> before;
        for (int seat = 0; seat < 3; ++seat)
            before[seat] = sample(d, frames.front(), seat);
        for (size_t i = 1; i < frames.size(); ++i) {
            const auto &a = frames[i - 1];
            const auto &b = frames[i];
            if (a.distance >= finish)
                break;
            std::array<Sample, 3> after;
            for (int seat = 0; seat < 3; ++seat)
                after[seat] = sample(d, b, seat);
            std::vector<double> cuts{0, 1};
            crossing(cuts, a.distance, b.distance, finish);
            crossing(cuts, a.speed, b.speed, 5);
            for (const auto &[start, end] : hardware) {
                crossing(cuts, a.distance, b.distance, start);
                crossing(cuts, a.distance, b.distance, end);
            }
            for (int seat = 0; seat < 3; ++seat) {
                const auto &x = before[seat];
                const auto &y = after[seat];
                for (double value : {0., .25, .8, 1.2, 2.})
                    crossing(cuts, x.force.vertical, y.force.vertical, value);
                for (double value : {-.5, -.2, .2, .5})
                    crossing(cuts, x.force.lateral, y.force.lateral, value);
                for (double value : {-.2, .2})
                    crossing(cuts, x.force.longitudinal, y.force.longitudinal, value);
                for (double value : {-rollThreshold, 0., rollThreshold})
                    crossing(cuts, x.roll, y.roll, value);
                crossing(cuts, x.angularSpeed, y.angularSpeed, quietRotation);
            }
            std::sort(cuts.begin(), cuts.end());
            cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
            for (size_t j = 1; j < cuts.size(); ++j) {
                const double lo = cuts[j - 1], hi = cuts[j], u = (lo + hi) * .5;
                const double distance = mix(a.distance, b.distance, u),
                             dt = (b.time - a.time) * (hi - lo);
                if (distance >= finish)
                    continue;
                if (mix(a.speed, b.speed, u) < 5) {
                    for (auto &e : episodes)
                        e = {};
                    continue;
                }
                result.movingSeconds += dt;
                const bool powered =
                    std::any_of(hardware.begin(), hardware.end(), [&](const auto &range) {
                        return distance >= range.first && distance <= range.second;
                    });
                if (!powered)
                    result.freeRunningSeconds += dt;
                for (int seat = 0; seat < 3; ++seat) {
                    auto &out = result.seats[seat];
                    auto &e = episodes[seat];
                    const auto &x = before[seat];
                    const auto &y = after[seat];
                    const double g = mix(x.force.vertical, y.force.vertical, u),
                                 lateral = mix(x.force.lateral, y.force.lateral, u);
                    const double longitudinal = mix(x.force.longitudinal, y.force.longitudinal, u);
                    if (g < .25)
                        out.airtimeSeconds += dt;
                    if (g < 0)
                        out.negativeGSeconds += dt;
                    if (g > 2)
                        out.loadedSeconds += dt;
                    if (std::abs(lateral) > .5)
                        out.lateralExposureSeconds += dt;
                    const int beat = g < .25 ? -1 : g > 2 ? 1 : 0;
                    if (beat != e.beat) {
                        e.beat = beat;
                        e.beatSeconds = 0;
                    }
                    e.beatSeconds += dt;
                    if (beat && e.beatSeconds >= .25 && beat != e.previousBeat) {
                        ++out.forceBeats;
                        e.previousBeat = beat;
                    }
                    // Hardware corridors are a deliberate part of the ride.
                    // Their constant attitude is not an unpowered dead gap.
                    const bool quiet = !powered && g > .8 && g < 1.2 && std::abs(lateral) < .2 &&
                                       std::abs(longitudinal) < .2 &&
                                       mix(x.angularSpeed, y.angularSpeed, u) < quietRotation;
                    e.quiet = quiet ? e.quiet + dt : 0;
                    out.longestQuietSeconds = std::max(out.longestQuietSeconds, e.quiet);
                    const bool rolling =
                        !powered && std::abs(mix(x.roll, y.roll, u)) > rollThreshold;
                    if (rolling)
                        out.rollingSeconds += dt;
                    e.rolling = rolling ? e.rolling + dt : 0;
                    out.longestRollingSeconds = std::max(out.longestRollingSeconds, e.rolling);
                    if (!powered)
                        out.totalRollRadians += dt *
                                                (std::abs(mix(x.roll, y.roll, lo)) +
                                                 std::abs(mix(x.roll, y.roll, hi))) *
                                                .5;
                }
            }
            before = after;
        }
    } catch (const std::exception &e) {
        result.error = e.what();
        return result;
    }
    result.performed = true;
    return result;
}
std::string rideFeelJson(const RideFeel &feel) {
    std::ostringstream out;
    out << std::setprecision(12);
    out << "{\"performed\":" << (feel.performed ? "true" : "false")
        << ",\"error\":" << std::quoted(feel.error)
        << ",\"method\":\"Elapsed replay seconds with interpolated threshold crossings; below 0.25 "
           "g airtime, above 2 g loading; body roll above 3 deg/s. Station stopping and speeds "
           "below 5 m/s excluded; physical hardware contact excluded from quiet/rolling "
           "intervals.\""
        << ",\"movingSeconds\":" << feel.movingSeconds
        << ",\"freeRunningSeconds\":" << feel.freeRunningSeconds << ",\"seats\":[";
    for (size_t i = 0; i < feel.seats.size(); ++i) {
        if (i)
            out << ',';
        const auto &seat = feel.seats[i];
        out << "{\"seat\":"
            << std::quoted(i == 0   ? "front"
                           : i == 1 ? "middle"
                                    : "rear")
            << ",\"airtimeSeconds\":" << seat.airtimeSeconds
            << ",\"negativeGSeconds\":" << seat.negativeGSeconds
            << ",\"loadedSeconds\":" << seat.loadedSeconds
            << ",\"lateralExposureSeconds\":" << seat.lateralExposureSeconds
            << ",\"longestQuietSeconds\":" << seat.longestQuietSeconds
            << ",\"forceBeats\":" << seat.forceBeats
            << ",\"rollingSeconds\":" << seat.rollingSeconds
            << ",\"longestRollingSeconds\":" << seat.longestRollingSeconds
            << ",\"rollingFraction\":"
            << (feel.freeRunningSeconds > 0 ? seat.rollingSeconds / feel.freeRunningSeconds : 0)
            << ",\"totalRollRadians\":" << seat.totalRollRadians << '}';
    }
    out << "]}";
    return out.str();
}
} // namespace coaster
