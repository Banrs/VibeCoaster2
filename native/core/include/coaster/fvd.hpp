#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// SI, Z-up; right=cross(forward,up). Centreline specific force is 1g upright
// and 0g ballistic. Roll is tangent twist in rad/s. Controls use quintic
// smoothstep interpolation and strictly increasing time starting at zero.
struct FvdControl {
    double time{},normalG{1},lateralG{},rollRate{};
};
// Optional physical twist about the moving tangent, with smooth onset/exit.
struct FvdTwistPhase {double begin{},end{},angle{};};
struct FvdRequest {
    Vec3 position{0,0,50},forward{1,0,0},up{0,0,1};
    double speed{20},step{.005};
    std::vector<FvdControl> controls{{0,1,0,0},{1,1,0,0}};
    std::vector<FvdTwistPhase> twists;
    double rollingAcceleration{},dragAccelerationCoefficient{};
    size_t maxSamples{20000};
    double forceToleranceG{.02},rollToleranceRadPerSecond{.02};
    double replayDistanceTolerance{.02},replaySpeedTolerance{.02};
};
struct FvdSample {
    double time{},distance{},speed{};
    Vec3 position,forward,up,curvature;
    double dissipatedWorkPerMass{};
};
struct FvdAssessment {
    bool performed{},passed{};
    size_t evaluations{};
    double maxNormalResidualG{},maxLateralResidualG{},maxRollResidualRadPerSecond{};
    double endDistanceError{},endSpeedError{},maxEnergyDrift{};
};
struct FvdResult {
    bool integrated{},canonicalBuilt{},cancelled{};
    std::vector<FvdSample> samples;
    Track track;
    FvdAssessment assessment;
    ValidationReport report;
};
// Point-mass authoring with optional rolling/drag losses; no propulsion or rider offsets.
// Sampled canonical replay is a diagnostic; ride validation is separate.
// Cancellation is polled during integration/replay, but not Track::rebuild.
FvdResult designFvdSection(const FvdRequest&,Cancel cancel={});
// Symmetric planar airtime hill with horizontal 1g ports. Apex pitch is solved
// by shooting; mirrored force history restores entry height and speed.
struct FvdAirtimeRequest {
    double speed{65},pushG{2.2},crestG{-.15};
    double guardSeconds{.1},pushRampSeconds{.8},pushHoldSeconds{.4},crestRampSeconds{1.2};
};
struct FvdAirtimeResult {
    FvdRequest authoring;
    FvdResult section;
    double span{},height{};
};
FvdAirtimeResult designFvdAirtime(const FvdAirtimeRequest&,Cancel cancel={});

// Immelmann: half-loop, descending roll and upright valley.
struct FvdImmelmannRequest {
    double entrySpeed{53},height{95},exitHeight{10};
    double normalG{4.8},crestG{.6},rollExitG{.3},rampSeconds{1.2},rollOverlapFraction{.35};
    int hand{1};double step{.0025},rollingAcceleration{},dragAccelerationCoefficient{};
};
struct FvdImmelmannResult {
    FvdRequest authoring;FvdResult section;
    FvdSample apex,rollExit,exit;
};
FvdImmelmannResult designFvdImmelmann(const FvdImmelmannRequest&,Cancel cancel={});

}
