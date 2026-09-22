#pragma once
#include "coaster/recipe.hpp"
#include "coaster/acceleration.hpp"
#include "coaster/hardware.hpp"

namespace coaster {
struct Scenario {
    double dragScale{1}, massScale{1};
    bool trims{true};
};
struct Sample {
    double time{}, s{}, speed{};
    std::array<Vec3, 3> force;
};
struct ForceEnvelope {
    bool nominalPassed{}, peakAllowancePassed{};
    double maximumExcessPercent{};
};
ForceEnvelope assessForceEnvelope(Vec3 minimum, Vec3 maximum);
struct Simulation {
    bool completed{}, assessed{};
    double dt{}, duration{}, active{}, terminal{}, clifftopActive{}, lip{}, maxSpeed{}, launchTime{},
        energyResidual{};
    std::array<Vec3, 3> minimum, maximum, rate;
    std::array<AccelerationAssessment, 3> acceleration;
    std::array<ForceEnvelope, 3> envelope;
    HardwarePlan hardware;
    std::vector<HardwareDemand> demand;
    std::vector<Sample> playback;
    std::vector<double> entrySpeeds;
    std::vector<std::string> failures;
};
Simulation simulate(const Track &, const Scenario &scenario = {}, double dt = 1. / 960,
                    const Cancel &cancel = {}, bool assess = true);
struct Clearance {
    bool continuous{};
    double minimumGround{1e9}, minimumNonlocal{1e9}, ordinaryMaxHeight{}, ordinaryMeanHeight{};
    double firstTrackS{-1}, secondTrackS{-1};
    std::size_t terrainHits{}, trackHits{}, samples{};
};
Clearance assessClearance(const Track &, double plateau, double spacing = .5, const Cancel &cancel = {});
} // namespace coaster
