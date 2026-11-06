#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Experience diagnostics, separate from the independent force/clearance gates.
// All durations use elapsed replay time, with linear threshold crossings. They
// are deliberately not counts of display samples or a claim of rider comfort.
struct SeatFeel {
    double airtimeSeconds{}; // Normal load below 0.25 g.
    double negativeGSeconds{};
    double loadedSeconds{};          // Normal load above 2 g.
    double lateralExposureSeconds{}; // Magnitude above 0.5 g.
    double longestQuietSeconds{};    // Unpowered, nearly 1 g, little rotation.
    double rollingSeconds{};         // Physical body roll above 3 degrees/s.
    double longestRollingSeconds{};
    double totalRollRadians{}; // Absolute physical body roll, not Euler bank.
    unsigned forceBeats{};     // Alternating low/high episodes lasting >=0.25 s.
};
struct RideFeel {
    bool performed{};
    double movingSeconds{}, freeRunningSeconds{};
    std::array<SeatFeel, 3> seats; // Front, middle, rear; same as force replay.
    std::string error;
};
RideFeel measureRideFeel(const Design &);
std::string rideFeelJson(const RideFeel &);
} // namespace coaster
