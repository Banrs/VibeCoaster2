#include "coaster/ride_feel.hpp"
#include <iostream>
#include <stdexcept>

using namespace coaster;
namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
void near(double actual, double expected, const char *message) {
    require(std::abs(actual - expected) < 1e-8, message);
}

// Prescribed measurements isolate the experience integrator from the physics
// engine. The separate generated-ride case below consumes real seat loads.
Design fixture(double twist = 0) {
    Design d;
    d.request.train.cars = 1;
    d.track.closed = false;
    d.track.authoredGeometry = d.track.authoredFrame = true;
    for (int i = 0; i <= 40; ++i) {
        const double x = 5. * i, b = .4 + twist * x;
        const Vec3 up{0, -std::sin(b), std::cos(b)}, right{0, -std::cos(b), -std::sin(b)};
        d.track.knots.push_back({{x, 0, 20},
                                 {1, 0, 0},
                                 {},
                                 up,
                                 0,
                                 Element::Return,
                                 {},
                                 {},
                                 right * twist,
                                 up * (-twist * twist),
                                 right * (-twist * twist * twist)});
    }
    d.track.rebuild();
    return d;
}
Frame measured(double time, double normal) {
    Frame f;
    f.time = time;
    f.distance = 10 * time;
    f.speed = 10;
    for (auto &seat : f.seats)
        seat = {normal, 0, 0};
    return f;
}
void elapsedTimeAndBeats() {
    auto d = fixture();
    for (auto [time, g] : {std::pair{0., 1.},
                           {2., -.5},
                           {5., -.5},
                           {7., 3.},
                           {10., 3.},
                           {12., -.5},
                           {14., -.5},
                           {16., 1.}})
        d.simulation.frames.push_back(measured(time, g));
    const auto original = measureRideFeel(d);
    require(original.performed, "Valid irregular replay was not measured");
    for (const auto &s : original.seats) {
        near(s.airtimeSeconds, 55. / 7,
             "Airtime must include threshold crossing fractions, in seconds");
        near(s.loadedSeconds, 29. / 7,
             "High load duration must use elapsed time, not sample counts");
        require(s.forceBeats == 3, "Sustained low/high/low force episodes must remain three beats");
    }
    auto dense = d;
    dense.simulation.frames.clear();
    for (size_t i = 1; i < d.simulation.frames.size(); ++i) {
        const auto &a = d.simulation.frames[i - 1];
        const auto &b = d.simulation.frames[i];
        for (int j = 0; j < 7; ++j) {
            const double u = j / 7.;
            dense.simulation.frames.push_back(
                measured(std::lerp(a.time, b.time, u),
                         std::lerp(a.seats[0].vertical, b.seats[0].vertical, u)));
        }
    }
    dense.simulation.frames.push_back(d.simulation.frames.back());
    const auto refined = measureRideFeel(dense);
    for (int i = 0; i < 3; ++i) {
        near(refined.seats[i].airtimeSeconds, original.seats[i].airtimeSeconds,
             "Resampling must not invent airtime");
        require(refined.seats[i].forceBeats == original.seats[i].forceBeats,
                "Resampling must not change the ride's force beats");
    }
    dense.simulation.frames[2].time = dense.simulation.frames[1].time;
    require(!measureRideFeel(dense).performed,
            "Malformed elapsed time must not produce a plausible feel report");
}
void physicalRollAndHardware() {
    auto still = fixture(), rolling = fixture(.01);
    for (int i = 0; i <= 32; ++i) {
        still.simulation.frames.push_back(measured(i * .5, 1));
        rolling.simulation.frames.push_back(measured(i * .5, 1));
    }
    auto a = measureRideFeel(still), b = measureRideFeel(rolling);
    near(a.seats[0].longestQuietSeconds, 16, "A held bank must not count as ongoing rotation");
    near(b.seats[0].rollingSeconds, 16,
         "Continuous physical roll must remain visible in the report");
    near(b.seats[0].longestRollingSeconds, 16, "A nonstop roll must not be hidden by sampling");
    near(b.seats[0].totalRollRadians, 1.6, "Roll must integrate actual body angular velocity");
    still.operations = {{40, 80, DriveKind::Boost}, {140, 0, DriveKind::Station}};
    a = measureRideFeel(still);
    near(a.movingSeconds, 14, "Terminal stopping must not dilute experience metrics");
    near(a.freeRunningSeconds, 10, "Only the actual hardware corridor is excluded");
    near(a.seats[0].longestQuietSeconds, 6,
         "Hardware separates quiet intervals instead of hiding adjacent coasting");
}
void rideCharacter(const Design &d) {
    require(d.accepted(), "Ride feel regression requires a fully accepted physical ride");
    const auto feel = measureRideFeel(d);
    std::cout << rideFeelJson(feel) << '\n';
    require(feel.performed && feel.freeRunningSeconds > 10,
            "The generated ride must have a real unpowered course");
    require(d.simulation.metrics.launchTo180 >= 1.3 && d.simulation.metrics.launchTo180 <= 1.4,
            "Keep the deliberate 0-180 km/h launch character");
    // Broad product budgets catch a dead or constantly rotating ride without
    // freezing a particular layout. Native force/rate gates remain separate.
    for (const auto &seat : feel.seats) {
        require(seat.airtimeSeconds >= .75 && seat.loadedSeconds >= 2 && seat.forceBeats >= 4,
                "Every seating position needs sustained airtime and contrasting force beats");
        require(seat.longestQuietSeconds <= 8,
                "An unpowered neutral interval became a prolonged dead gap");
        require(seat.rollingSeconds / feel.freeRunningSeconds <= .65 &&
                    seat.longestRollingSeconds <= 12,
                "Banking became constant roll instead of distinct transitions");
    }
    require(d.simulation.metrics.maxEnergyResidual < .5,
            "Ride feel cannot be bought with unaccounted energy");
}
} // namespace
int main(int argc, char **argv) {
    try {
        if (argc == 1) {
            elapsedTimeAndBeats();
            physicalRollAndHardware();
        } else if (argc == 3 && std::string(argv[1]) == "--design") {
            Design d;
            std::string error;
            if (!loadDesign(argv[2], d, error))
                throw std::runtime_error("Reload failed: " + error);
            rideCharacter(d);
        } else if ((argc == 2 || argc == 3) && std::string(argv[1]) == "--generated") {
            GenerationRequest request;
            request.terrain.kind = TerrainKind::Highlands;
            if (argc == 3)
                request.seed = std::stoull(argv[2]);
            const auto d = generate(request);
            if (!d.accepted())
                for (const auto *report : {&d.report, &d.simulation.report})
                    for (const auto &error : report->errors)
                        std::cerr << error.code << ": " << error.message << '\n';
            rideCharacter(d);
        } else
            throw std::runtime_error("Usage: ride_feel_tests [--design FILE | --generated [SEED]]");
        std::cout << "PASS ride-feel regressions\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
