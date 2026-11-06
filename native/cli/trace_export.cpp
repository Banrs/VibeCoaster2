#include "trace_export.hpp"
#include <fstream>
#include <filesystem>
#include <chrono>
#include <atomic>
#include <locale>
#include <sstream>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif
#include <iomanip>
#include <stdexcept>

namespace coaster {
namespace {
std::string jsonString(std::string_view text) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << '"';
    for (unsigned char c : text) {
        switch (c) {
        case '"':
            out << "\\\"";
            break;
        case '\\':
            out << "\\\\";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            if (c < 32)
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
            else
                out << c;
        }
    }
    out << '"';
    return out.str();
}
} // namespace
void writeRideTrace(const Design &d, const std::string &path) {
    if (d.track.knots.empty() || d.track.spans.empty() || !std::isfinite(d.track.length) ||
        d.track.length <= 0)
        throw std::runtime_error("Trace export requires canonical track geometry");
    const std::filesystem::path destination{std::u8string(path.begin(), path.end())};
    // Directory creation is exclusive on both platforms. Unlike opening a
    // timestamp-named file, concurrent exporters cannot truncate one another's
    // staging data. Keeping it beside the destination preserves atomic rename.
    static std::atomic<unsigned long long> sequence{};
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    std::filesystem::path staging;
    for (int attempt = 0; attempt < 64; ++attempt) {
        auto candidate = destination;
        candidate += ".tmp-" + std::to_string(stamp) + "-" + std::to_string(sequence.fetch_add(1));
        std::error_code error;
        if (std::filesystem::create_directory(candidate, error)) {
            staging = std::move(candidate);
            break;
        }
        if (error && error != std::errc::file_exists)
            throw std::runtime_error("Cannot create trace staging directory: " + error.message());
    }
    if (staging.empty())
        throw std::runtime_error("Cannot allocate trace staging directory");
    struct Cleanup {
        std::filesystem::path path;
        ~Cleanup() {
            std::error_code error;
            std::filesystem::remove_all(path, error);
        }
    } cleanup{staging};
    const auto temporary = staging / "trace.json";
    std::ofstream f(temporary, std::ios::binary);
    if (!f)
        throw std::runtime_error("Cannot write trace");
    f.imbue(std::locale::classic());
    f << std::setprecision(12)
      << "{\"sampleRateHz\":60,\"acceptanceSampleRateHz\":" << 1 / d.request.simulationStep
      << ",\"displayTraceOnly\":true,\"orientationReference\":\"train-centre; each dynamics entry "
         "carries its own seat "
         "frame\",\"seatOrder\":[\"front\",\"middle\",\"rear\"],\"seatForceOrder\":[\"Gz_up\",\"Gy_"
         "right\",\"Gx_forward\"],\"angleUnits\":\"radians\",\"frames\":[";
    auto vector = [&](Vec3 q) { f << '[' << q.x << ',' << q.y << ',' << q.z << ']'; };
    std::array<std::array<double, 3>, 4> previous{};
    bool comma = false;
    auto orientation = [](const TrackSample &q, std::array<double, 3> &prior) {
        // Choose the nearest equivalent Euler branch; frame vectors remain
        // authoritative at vertical pitch where Euler yaw is indeterminate.
        double yaw = std::atan2(q.tangent.y, q.tangent.x),
               pitch = std::asin(std::clamp(q.tangent.z, -1., 1.));
        std::array<double, 3> angles{};
        double best = INFINITY;
        for (int branch = 0; branch < 2; ++branch) {
            double y = yaw + branch * pi, p = branch ? pi - pitch : pitch;
            Vec3 up{-std::cos(y) * std::sin(p), -std::sin(y) * std::sin(p), std::cos(p)};
            double roll = std::atan2(dot(q.up, cross(q.tangent, up)), dot(q.up, up));
            std::array<double, 3> candidate{roll, p, y};
            double score = 0;
            for (int i = 0; i < 3; ++i) {
                candidate[i] = prior[i] + std::remainder(candidate[i] - prior[i], 2 * pi);
                score += std::pow(candidate[i] - prior[i], 2);
            }
            if (score < best) {
                angles = candidate;
                best = score;
            }
        }
        prior = angles;
        return angles;
    };
    for (const auto &x : d.simulation.frames) {
        const auto q = d.track.sample(x.distance);
        const auto angles = orientation(q, previous[0]);
        if (comma)
            f << ',';
        comma = true;
        f << "{\"time\":" << x.time << ",\"distance\":" << x.distance << ",\"speed\":" << x.speed
          << ",\"roll\":" << angles[0] << ",\"pitch\":" << angles[1] << ",\"yaw\":" << angles[2]
          << ",\"acceleration\":" << x.acceleration
          << ",\"accelerationRate\":" << x.accelerationRate
          << ",\"driveWorkPerMass\":" << x.driveWorkPerMass
          << ",\"brakeWorkPerMass\":" << x.brakeWorkPerMass
          << ",\"lossWorkPerMass\":" << x.lossWorkPerMass
          << ",\"energyResidual\":" << x.energyResidual << ",\"forward\":";
        vector(q.tangent);
        f << ",\"up\":";
        vector(q.up);
        f << ",\"position\":";
        vector(q.position);
        f << ",\"seats\":[";
        for (int i = 0; i < 3; ++i) {
            if (i)
                f << ',';
            auto a = x.seats[i];
            f << '[' << a.vertical << ',' << a.lateral << ',' << a.longitudinal << ']';
        }
        f << "],\"dynamics\":[";
        for (int i = 0; i < 3; ++i) {
            if (i)
                f << ',';
            const double seatDistance = x.distance + seatDistanceOffset(d.request.train, i);
            const auto a = measureSeatDynamics(d.track, seatDistance, x.speed, x.acceleration,
                                               x.accelerationRate, d.request.train.seatHeight);
            const auto pose = d.track.sample(seatDistance);
            const auto seatAngles = orientation(pose, previous[i + 1]);
            f << "{\"roll\":" << seatAngles[0] << ",\"pitch\":" << seatAngles[1]
              << ",\"yaw\":" << seatAngles[2] << ",\"forward\":";
            vector(pose.tangent);
            f << ",\"up\":";
            vector(pose.up);
            f << ",\"position\":";
            vector(pose.position + pose.up * d.request.train.seatHeight);
            f << ",\"forceRateGps\":[" << a.rate.vertical << ',' << a.rate.lateral << ','
              << a.rate.longitudinal << "],\"inertialJerkMps3\":";
            vector(a.inertialJerk);
            f << ",\"angularVelocityRadps\":";
            vector(a.angularVelocity);
            f << ",\"angularAccelerationRadps2\":";
            vector(a.angularAcceleration);
            f << ",\"angularJerkRadps3\":";
            vector(a.angularJerk);
            f << '}';
        }
        f << "]}";
    }
    f << "],\"geometry\":[";
    comma = false;
    for (double s = 0; s < d.track.length; s += 2) {
        auto p = d.track.sample(s);
        if (comma)
            f << ',';
        comma = true;
        f << '[' << s << ',' << p.position.x << ',' << p.position.y << ',' << p.position.z << ','
          << p.up.x << ',' << p.up.y << ',' << p.up.z << ',' << int(p.element) << ']';
    }
    f << "],\"title\":" << jsonString(d.request.recipe.name)
      << ",\"generatorVersion\":" << jsonString(d.generationVersion)
      << ",\"accepted\":" << (d.accepted() ? "true" : "false")
      << ",\"lengthMeters\":" << d.track.length << ",\"train\":{\"cars\":" << d.request.train.cars
      << ",\"spacing\":" << d.request.train.spacing
      << ",\"seatHeight\":" << d.request.train.seatHeight
      << ",\"gauge\":" << trackSection(d.track.profile).gauge << '}';
    f << ",\"sections\":[";
    comma = false;
    for (const auto &section : d.sections) {
        if (comma)
            f << ',';
        comma = true;
        f << "{\"id\":" << jsonString(section.recipeId) << ",\"name\":" << jsonString(section.name)
          << ",\"role\":" << jsonString(roleName(section.role)) << ",\"start\":" << section.start
          << ",\"end\":" << section.end << '}';
    }
    f << "],\"operations\":[";
    comma = false;
    static constexpr const char *driveNames[]{"launch", "boost", "brake", "station", "trim"};
    for (const auto &op : d.operations) {
        if (comma)
            f << ',';
        comma = true;
        const int kind = int(op.kind);
        if (kind < 0 || kind >= int(std::size(driveNames)))
            throw std::runtime_error("Trace export encountered an unknown drive kind");
        f << "{\"kind\":" << jsonString(driveNames[kind]) << ",\"start\":" << op.start
          << ",\"end\":" << op.end << ",\"targetSpeed\":" << op.targetSpeed << '}';
    }
    f << "],\"landmarks\":[";
    comma = false;
    for (const auto &mark : d.landmarks) {
        if (comma)
            f << ',';
        comma = true;
        f << "{\"name\":" << jsonString(landmarkName(mark.kind))
          << ",\"distance\":" << mark.distance << ",\"position\":";
        vector(d.track.sample(mark.distance).position);
        f << '}';
    }
    f << "],\"supportMembers\":[";
    comma = false;
    for (const auto &support : d.supports)
        for (const auto &member : support.members) {
            if (comma)
                f << ',';
            comma = true;
            f << '[' << member.base.x << ',' << member.base.y << ',' << member.base.z << ','
              << member.top.x << ',' << member.top.y << ',' << member.top.z << ','
              << member.radiusBase << ',' << member.radiusTop << ']';
        }
    f << "],\"stationBoxes\":[";
    comma = false;
    for (const auto &box : d.station.boxes) {
        if (comma)
            f << ',';
        comma = true;
        f << "{\"center\":";
        vector(box.center);
        f << ",\"forward\":";
        vector(box.forward);
        f << ",\"right\":";
        vector(box.right);
        f << ",\"up\":";
        vector(box.up);
        f << ",\"half\":";
        vector(box.half);
        f << ",\"role\":" << int(box.role) << '}';
    }
    // An overview of the actual authored landform, not a second terrain model.
    // The browser labels this display sampling; acceptance evaluates the full model.
    double minX = INFINITY, minY = INFINITY, maxX = -INFINITY, maxY = -INFINITY;
    for (const auto &knot : d.track.knots) {
        minX = std::min(minX, knot.position.x);
        maxX = std::max(maxX, knot.position.x);
        minY = std::min(minY, knot.position.y);
        maxY = std::max(maxY, knot.position.y);
    }
    minX -= 120;
    minY -= 120;
    maxX += 120;
    maxY += 120;
    constexpr int columns = 121, rows = 121;
    const double dx = (maxX - minX) / (columns - 1), dy = (maxY - minY) / (rows - 1);
    f << "],\"terrainGrid\":{\"origin\":[" << minX << ',' << minY << "],\"step\":[" << dx << ','
      << dy << "],\"columns\":" << columns << ",\"rows\":" << rows
      << ",\"representation\":\"Sampled overview of the authored terrain; validation evaluates the "
         "full "
         "terrain model\",\"heights\":[";
    comma = false;
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < columns; ++x) {
            if (comma)
                f << ',';
            comma = true;
            f << d.request.terrain.height(minX + x * dx, minY + y * dy);
        }
    f << "]}}";
    f.close();
    if (!f)
        throw std::runtime_error("Cannot finish trace export");
    // A failed diagnostic export must not leave a plausible partial scene or
    // destroy the previous complete preview.
#if defined(_WIN32)
    if (!MoveFileExW(temporary.c_str(), destination.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw std::runtime_error("Cannot replace trace export");
#else
    std::filesystem::rename(temporary, destination);
#endif
}
} // namespace coaster
