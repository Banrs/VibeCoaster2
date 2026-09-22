#pragma once
#include "coaster/recipe.hpp"
#include "coaster/acceleration.hpp"

namespace coaster {
struct Scenario {double dragScale{1},massScale{1};bool trims{true};};
struct Sample {double time{},s{},speed{};std::array<Vec3,3> force;};
struct Simulation {
    bool completed{};
    double dt{},duration{},active{},terminal{},clifftopActive{},lip{},maxSpeed{},launchTime{},energyResidual{};
    std::array<Vec3,3> minimum,maximum,rate;
    std::array<AccelerationAssessment,3> acceleration;
    std::vector<Sample> playback;
    std::vector<std::string> failures;
};
Simulation simulate(const Track&,const Scenario& scenario={},double dt=1./960,const Cancel& cancel={});
struct Clearance {
    double minimumGround{1e9},minimumNonlocal{1e9},ordinaryMaxHeight{},ordinaryMeanHeight{};
    std::size_t terrainHits{},trackHits{},samples{};
};
Clearance assessClearance(const Track&,double plateau,double spacing=.5,const Cancel& cancel={});
}
