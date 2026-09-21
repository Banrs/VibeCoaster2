#pragma once
#include "coaster/math.hpp"
#include <array>
#include <string>
#include <vector>

namespace coaster {
struct FvdControl {
    double time{},normalG{1},lateralG{},rollRate{};
    double drive{};
    std::array<double,4> first{},second{}; // normal, lateral, twist, drive
};
struct FvdTwistPhase {double begin{},end{},angle{};};
struct FvdRequest {
    Vec3 position{0,0,50},forward{1,0,0},up{0,0,1};
    double speed{20},step{.005};
    std::vector<FvdControl> controls{{0,1,0,0},{1,1,0,0}};
    std::vector<FvdTwistPhase> twists;
    bool gravityReferencedRoll{}; // Bank relative to projected gravity; undefined at vertical tangents.
    std::vector<std::pair<double,std::string>> chapters;
    double rollingAcceleration{},dragAccelerationCoefficient{};
    size_t maxSamples{20000};
    double forceToleranceG{.02},rollToleranceRadPerSecond{.02};
    double replayDistanceTolerance{.02},replaySpeedTolerance{.02};
};

// Source functions remain editable data. Distances map the retained FVD source
// onto the final canonical knots; the loop's deliberate lane shift is explicit.
struct ForceAuthoring {
    std::string name;
    FvdRequest program;
    Vec3 origin;
    double heading{},hand{1},laneShift{},laneLength{};
    size_t firstKnot{};
    std::vector<double> sourceDistances;
};
struct SplineAuthoring {
    std::string name;
    size_t firstKnot{},lastKnot{};
    Vec3 origin;
    double length{},entrySpeed{},rollingAcceleration{},dragCoefficient{},hand{1},requestedRoll{};
    std::array<double,32> pitch{},heading{};
};
struct AuthorshipAssessment {
    bool performed{},passed{};
    double maximumSourcePositionError{},maximumNormalResidualG{},maximumLateralResidualG{},maximumRollResidualRadians{};
};
}
