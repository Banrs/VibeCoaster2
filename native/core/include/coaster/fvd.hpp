#pragma once
#include "coaster/coaster.hpp"
#include <optional>

namespace coaster {
// SI, Z-up; right=cross(forward,up). Centreline specific force is 1g upright
// and 0g ballistic. Roll is tangent twist in rad/s; drive is signed actuator
// force / mass in m/s^2. Quintic Hermite controls share value, rate and second
// derivative. A named chapter is metadata and never resets the physical state.
struct FvdKnot {double time{},value{},first{},second{};};
using FvdChannels=std::array<std::vector<FvdKnot>,4>;
// Resample four independent timelines onto their union, preserving all jets.
std::vector<FvdControl> combineFvdChannels(const FvdChannels&);
FvdControl sampleFvdControl(const std::vector<FvdControl>&,double time);
// Optional physical twist about the moving tangent, with smooth onset/exit.
struct FvdSample {
    double time{},distance{},speed{};
    Vec3 position,forward,up,curvature;
    double dissipatedWorkPerMass{};
    double drivenWorkPerMass{};
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
// Point-mass authoring with actuator work and rolling/drag losses; no rider offsets.
// Sampled canonical replay is a diagnostic; ride validation is separate.
// Cancellation is polled during integration/replay, but not Track::rebuild.
FvdResult designFvdSection(const FvdRequest&,Cancel cancel={});
// An inherited physical port is independent of an element's shape knobs.
// Spatial jets are analytic arc-length derivatives; controls use time derivatives.
struct FvdDriveJet {double value{},first{},second{};};
struct FvdEntry {Knot jet;double speed{};FvdControl initial;};
FvdEntry makeFvdEntry(const Knot&,double speed,double rollingAcceleration,
    double dragAccelerationCoefficient,FvdDriveJet drive={});
struct FvdHillRequest {
    std::optional<FvdEntry> entry;
    double entrySpeed{64},height{105},exitHeight{5},exitPitch{-.1};
    double positiveG{4.0},airtimeG{-.4},rampSeconds{1.15},twistAngle{};
    double exitPositiveG{},exitRampSeconds{},ascentReleaseSeconds{}; // Zero inherits the ascent; otherwise an asymmetric recovery.
    double crestLoadChangeG{}; // Smooth normal-load change across the crest; zero preserves constant airtime.
    double exitNormalG{},exitReleaseSeconds{.8}; // Optional final force release and upright bank after the recovery.
    double rollingAcceleration{},dragAccelerationCoefficient{};
};
struct FvdHillResult { FvdRequest authoring;FvdResult section; };
// Three phase durations solve crest height, exit height and live exit pitch.
// The end remains positively loaded; adjoining motion inherits its full jet.
FvdHillResult designFvdHill(const FvdHillRequest&,Cancel cancel={});
struct FvdCamelbackRequest {
    FvdHillRequest hill;
    double tailCutSeconds{.75},releaseSeconds{.30},exitNormalG{1.9},minimumExitPitch{.02};
};
struct FvdCamelbackResult {
    FvdRequest authoring;FvdResult section;
    double protectedEndTime{},protectedEndDistance{},originalEndTime{};
};
// Solve the supplied protected hill, retain its force timeline except for a
// shortened final constant-load hold, then release to a freely placed rising
// port. The complete circuit must still assess subsequent acceleration history.
FvdCamelbackResult designFvdCamelback(const FvdCamelbackRequest&,Cancel cancel={});
struct FvdDiveRequest {
    double entrySpeed{9},drop{135},exitPitch{-18*pi/180},maximumPitch{88*pi/180};
    double crestG{-.5},pulloutG{3.0},crestRampSeconds{1.0},pulloutRampSeconds{1.7},exitRampSeconds{1.2};
    double rollingAcceleration{},dragAccelerationCoefficient{};
};
// Slow crest, steep fall and loaded recovery are one continuous force programme.
// The final straight grade is derived from the force solution for graded LSM.
FvdHillResult designFvdDive(const FvdDiveRequest&,Cancel cancel={});
struct FvdWaveRequest {
    std::optional<FvdEntry> entry; // Actual world-space port; absent retains the legacy isolated fixture.
    double entrySpeed{70},entryPitch{-15*pi/180},height{75},exitHeight{-11},turnAngle{pi};
    double bankAngle{70*pi/180},exitNormalG{3.9};
    double rollingAcceleration{},dragAccelerationCoefficient{};
    double entryNormalG{},entryHoldSeconds{}; // Zero load inherits cos(entryPitch); coordinated bank can begin during the low-load hold.
};
// Loaded rise, banked elevated reversal and descending recovery share a live
// force timeline. The exit is level and positively curved, ready for a loop.
FvdHillResult designFvdWave(const FvdWaveRequest&,Cancel cancel={});
struct FvdLoopRequest {
    std::optional<FvdEntry> entry; // Actual world-space port; absent retains the legacy isolated fixture.
    double entrySpeed{65},height{145},normalG{3.8},crestG{.9},yawAngle{15*pi/180};
    double ascentReleaseSeconds{}; // Optional prescribed unload followed by a solved crown hold.
    double exitPositiveG{}; // Zero inherits normalG; otherwise prescribe the descending recovery load.
    double exitPitch{},exitNormalG{1.5},rollingAcceleration{},dragAccelerationCoefficient{};
    double crossingOffset{}; // Nonzero: signed low-arm gap in metres, positive entry-frame left; parallel exit replaces yawAngle.
};
// Upright entry, explicit unloaded inverted crest and real lateral/yaw motion.
// Apex height and the final frame are solved; final XYZ placement remains free.
FvdHillResult designFvdLoop(const FvdLoopRequest&,Cancel cancel={});
enum class TerrainAct {Clifftop,RavineRoll};
struct FvdTerrainActRequest {
    TerrainAct kind{TerrainAct::Clifftop};
    double entrySpeed{40},entryPitch{},pitchRateS{},pitchSecondS{},pitchThirdS{};
    double heightChange{-10},headingChange{130*pi/180},airtimeG{-1.25},outwardBank{30*pi/180},durationScale{4./3},exitPitch{};
    double rollingAcceleration{},dragAccelerationCoefficient{};
};
// Inherits a planar entry's complete pitch jet, then coordinates rider force
// and bank through the inbank/outbank act. The exit is upright and level.
FvdHillResult designFvdTerrainAct(const FvdTerrainActRequest&,Cancel cancel={});
struct FvdApproachRequest {
    FvdEntry entry;
    Vec3 endPosition;
    double endHeading{},normalG{3.2},bankRampSeconds{1.2};
    double rollingAcceleration{},dragAccelerationCoefficient{};
};
// Level, coordinated banked turn with bounded straight entry/exit durations.
// The physical port and final position/heading are solved without a spline repair.
FvdHillResult designFvdApproach(const FvdApproachRequest&,Cancel cancel={});

// Immelmann: half-loop, descending roll and upright valley.
struct FvdImmelmannRequest {
    std::optional<FvdEntry> entry; // Actual world-space port; absent retains the legacy isolated fixture.
    double entrySpeed{53},height{95},exitHeight{10};
    double exitPitch{},exitNormalG{1};
    double normalG{4.8},crestG{.6},rollExitG{.3},rampSeconds{1.2},rollOverlapFraction{.35};
    double yawAngle{}; // Coordinated yaw of the ascending half-loop plane.
    double rollReleaseFraction{}; // Retain the crown load before releasing through the descending roll.
    double exitRampSeconds{}; // Zero inherits the ascent ramp; otherwise prescribe the recovery transition.
    double ascentReleaseSeconds{}; // Zero solves the unload; otherwise solve a crown hold after this release.
    double exitPositiveG{}; // Zero inherits normalG; descending valley can carry a different peak.
    int hand{1};double step{.0025},rollingAcceleration{},dragAccelerationCoefficient{};
    bool planarRoll{}; // Resolve the half-roll load in both rider axes to preserve the entry vertical plane.
};
struct FvdImmelmannResult {
    FvdRequest authoring;FvdResult section;
    FvdSample apex,rollExit,exit;
};
FvdImmelmannResult designFvdImmelmann(const FvdImmelmannRequest&,Cancel cancel={});

}
