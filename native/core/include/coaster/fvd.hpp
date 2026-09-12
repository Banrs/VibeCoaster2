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
// Point-mass speed coupling includes optional rolling/quadratic losses.
// Zero defaults preserve the 0.8.0 gravity-only airtime authoring. No propulsion,
// finite train or rider offsets are modelled. Actual
// operations/finite-train simulation, clearance, convergence, targets and all
// full Design acceptance checks remain mandatory outside this module.
// Bounded domain: <=60s, speed [.5,250]m/s, |position|<=100000m,
// curvature <=.15/m, <=50000 samples, |force|<=20g, |roll|<=4pi rad/s.
// Low speed, excessive angular step, invalid input, fit failure and sampled
// replay residual failure are reported, never repaired by clamping speed.
// Canonical fitting retains exact integrated knot tangent/curvature/up. The
// sampled replay is a numerical diagnostic, not an interval certificate or
// a successful ride. Cancellation is polled during integration and replay;
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

// Retained complete Immelmann: half-loop, descending roll and upright valley.
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
