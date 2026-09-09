#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Experimental, open-section authoring only. SI, Z-up, right=cross(forward,up).
// Forces are non-gravitational specific forces at the TRACK CENTERLINE: an
// upright level straight is normalG=1, lateralG=0; zero/zero is ballistic.
// Roll is the physical twist about forward in rad/s, not an Euler bank rate.
// Each adjacent control pair uses a quintic smoothstep (zero first/second
// derivatives at controls). Time must start at zero and increase strictly.
struct FvdControl {
    double time{},normalG{1},lateralG{},rollRate{};
};
struct FvdRequest {
    Vec3 position{0,0,50},forward{1,0,0},up{0,0,1};
    double speed{20},step{.005};
    std::vector<FvdControl> controls{{0,1,0,0},{1,1,0,0}};
    size_t maxSamples{20000};
    double forceToleranceG{.02},rollToleranceRadPerSecond{.02};
    double replayDistanceTolerance{.02},replaySpeedTolerance{.02};
    double rollingAcceleration{},dragAccelerationCoefficient{}; // SI: m/s^2 and 1/m. Default zero preserves gravity-only behavior.
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
    double endDistanceError{},endSpeedError{};
    double maxEnergyDrift{}; // Residual of mechanical energy plus dissipated work per mass (J/kg).
};
struct FvdResult {
    bool integrated{},canonicalBuilt{},cancelled{};
    std::vector<FvdSample> samples;
    Track track;
    FvdAssessment assessment;
    ValidationReport report;
};
// Point-mass coupling: dv/dt = gravityVector dot forward - rollingAcceleration
// - dragAccelerationCoefficient*speed^2. Defaults are gravity-only. No propulsion,
// finite train or rider offsets are modelled. Actual
// operations/finite-train simulation, clearance, convergence, targets and all
// full Design acceptance checks remain mandatory outside this module.
// Bounded domain: <=60s, speed [.5,250]m/s, |position|<=100000m,
// curvature <=.15/m, <=50000 samples, |force|<=20g, |roll|<=4pi rad/s.
// Low speed, excessive angular step, invalid input, fit failure and sampled
// replay residual failure are reported, never repaired by clamping speed.
// Canonical fitting retains exact integrated knot tangent/curvature/up. The
// sampled replay is a numerical diagnostic, not an interval certificate or
// a successful ride. Dissipated work per mass is integrated for the energy audit.
// Cancellation is polled during integration and replay;
// the bounded existing Track::rebuild itself has no cancellation callback.
FvdResult designFvdSection(const FvdRequest&,Cancel cancel={});
// Production authoring adapter: a symmetric, planar force-controlled airtime
// hill with horizontal 1g ports. Shooting closes pitch at the apex; mirroring
// the force history returns to the entry height/speed without spatial scaling.
// It remains a point-mass source shape, subject to all full-ride acceptance.
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
// Planar upward half-loop. A bounded duration shoot closes height and pi pitch
// using an energy-coupled normal-force profile, not a symmetric geometric arc.
// The resulting forward displacement is generally nonzero. Gravity-only source
// assumptions and full-ride validation requirements are identical to FVD above.
struct FvdPitchRequest {
    double height{95},apexSpeed{24},normalG{3.5},pushRampSeconds{1.2};
    double apexNormalG{-1}; // -1 closes a flat inverted port; positive supports a full-loop crest.
};
struct FvdPitchResult {
    FvdRequest authoring;
    FvdResult section;
    double forwardDisplacement{},height{},holdSeconds{},exitRampSeconds{};
};
FvdPitchResult designFvdPitch(const FvdPitchRequest&,Cancel cancel={});

// Joint body-force/twist authoring of a reversing ascent and rolling descent.
// Entry is at the origin, level along +X. The exit height/pitch are prescribed;
// horizontal displacement and heading are integrated, never post-warped.
// Forces are gravity-only point-source intent, not finite-train acceptance.
struct FvdReversalRequest {
    double entrySpeed{48.9107707974},exitHeight{50},exitPitch{-40*pi/180};
    double normalG{3.5},pushRampSeconds{1.2},rollStartFraction{.25},lateralPulseG{.15};
    int hand{1};
    double step{.0025};
};
struct FvdReversalResult {
    FvdResult section;
    TrackKinematics entry,exit;
    std::vector<FvdControl> intent; // Continuous controls evaluated at source samples.
    double geometricApexHeight{},highestInvertedHeight{},minSpeed{};
    double holdSeconds{},unloadSeconds{},totalTwist{},duration{};
};
FvdReversalResult designFvdReversal(const FvdReversalRequest&,Cancel cancel={});

// Continue an upright, straight-compatible descending source port into a level
// lower valley. The actual entry heading is preserved; no horizontal reset is
// inserted. The source entry sample must have negligible curvature.
struct FvdPulloutRequest {
    FvdSample entry;
    double targetHeight{},normalG{3.2},rampSeconds{1.2},step{.0025};
};
struct FvdPulloutResult {
    FvdRequest authoring;
    FvdResult section;
    TrackKinematics entry,exit;
    double slopeHoldSeconds{},unloadSeconds{},duration{};
};
FvdPulloutResult designFvdPullout(const FvdPulloutRequest&,Cancel cancel={});

// Height-constrained single hill with level, zero-curvature ports. The ascent
// and descent are solved separately with the requested explicit point losses;
// positions are never scaled and exit speed is an output, not reset to entry.
// The bounded family covers 220..280 m and 75..90 m/s when physically feasible.
// Full finite-train/rider/clearance/operations validation remains mandatory.
struct FvdTallHillRequest {
    double height{240},speed{80},normalG{3.35},crestG{-.025},rampSeconds{1.2};
    double rollingAcceleration{},dragAccelerationCoefficient{};
};
struct FvdTallHillResult {
    FvdRequest authoring;
    FvdResult section;
    FvdSample exit;
    double span{},height{};
};
FvdTallHillResult designFvdTallHill(const FvdTallHillRequest&,Cancel cancel={});
}
