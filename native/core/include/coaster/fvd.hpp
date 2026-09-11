#pragma once
#include "coaster/coaster.hpp"

namespace coaster {
// Experimental, open-section authoring only. SI, Z-up, right=cross(forward,up).
// Forces are non-gravitational specific forces at the TRACK CENTERLINE: an
// upright level straight is normalG=1, lateralG=0; zero/zero is ballistic.
// Roll is the physical twist about forward in rad/s, not an Euler bank rate.
// Adjacent controls use monotone quintics with zero second derivatives.
// Normal-force slopes are explicit; zero slopes retain quintic smoothstep.
// Lateral/roll slopes remain zero. Time starts at zero and increases strictly.
struct FvdControl {
    double time{},normalG{1},lateralG{},rollRate{},normalRateGps{};
};
// Physical twist integrated about the moving tangent. A quintic angle change
// gives zero twist rate and onset at both ends; angle is not an Euler bank.
struct FvdTwistPhase {double begin{},end{},angle{};};
struct FvdRequest {
    Vec3 position{0,0,50},forward{1,0,0},up{0,0,1};
    double speed{20},step{.005};
    std::vector<FvdControl> controls{{0,1,0,0},{1,1,0,0}};
    std::vector<FvdTwistPhase> twists; // Optional nonoverlapping phases; controls must then have zero rollRate.
    size_t maxSamples{20000};
    double forceToleranceG{.02},rollToleranceRadPerSecond{.02};
    double replayDistanceTolerance{.02},replaySpeedTolerance{.02};
    double rollingAcceleration{},dragAccelerationCoefficient{}; // SI: m/s^2 and 1/m. Default zero preserves gravity-only behavior.
};
struct FvdSample {
    double time{},distance{},speed{};
    Vec3 position,forward,up,curvature;
    double dissipatedWorkPerMass{}; // Work since this open section's inlet; add prior-section work when composing an energy audit.
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
// One to three force-controlled hills integrated as a single 3D source.
// Only the two outer ports are horizontal 1g. Every internal valley retains
// the next hill's pushG, nonzero curvature and continuous energy. Continuously
// changing load passes through an unloading knee and the crest; only explicit
// peak controls hold load. Apex pitch and descent height/pitch are shot in
// time, never spatially warped. The complete force history requires assessment.
// Explicit point losses carry through every apex and valley; exit speeds are
// integrated outputs. Zero loss coefficients retain the gravity-only case.
// Point-mass source intent still requires all full-ride acceptance checks.
struct FvdAirtimeHill {
    double pushG{2.2},crestG{-.15};
    double pushHoldSeconds{.4},crestRampSeconds{1.2},crestPulseSeconds{1.6}; // Crest phase spans zero crossings for negative crestG.
    double bankRadians{}; // Nominal crest-bank intent; the actual apex bank is an integrated output.
};
struct FvdAirtimeRequest {
    // Entry and final exit use explicit load ramps. Valley closure may solve
    // a positive-load hold, but cannot shorten these ramps to force a fit.
    double speed{65},guardSeconds{.1},portRampSeconds{.8},unloadG{1};
    std::vector<FvdAirtimeHill> hills{{}};
    double rollingAcceleration{},dragAccelerationCoefficient{};
};
struct FvdAirtimeSpan {
    FvdSample entry,apex,exit; // Distances and frames in the complete source.
};
struct FvdAirtimeResult {
    FvdRequest authoring;
    FvdResult section;
    std::vector<FvdAirtimeSpan> hills;
    double span{},height{}; // span is horizontal path arc length; use the actual last sample for the exit pose.
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

// Continuous full loop with explicit losses. Entry speed and phase durations
// close the actual apex height/speed and a level upright exit. Two physical
// twist phases separate the actual crossing arms; exit X/Y and heading
// remain integrated outputs. Zero body-lateral intent does not mean zero
// world-lateral acceleration: the normal force acts through the banked frame.
// The straight guards have actual path length portLength, including losses.
struct FvdLoopRequest {
    double height{60},apexSpeed{20},normalG{3.5},apexNormalG{.5},pushRampSeconds{1.2};
    double crossingOffset{18},portLength{8},step{.0025}; // Signed Y separation where the low arms share actual X/Z.
    double rollingAcceleration{},dragAccelerationCoefficient{};
    size_t maxSamples{20000};
};
struct FvdLoopResult {
    FvdRequest authoring;
    FvdResult section;
    FvdSample loopEntry,apex,loopExit; // Actual force-phase boundaries, with the guards outside them.
    FvdSample crossingEntry,crossingExit; // Integrated observations at solved crossing times, not force-control resets.
};
FvdLoopResult designFvdLoop(const FvdLoopRequest&,Cancel cancel={});

// One loss-aware Immelmann source owns the half-loop, overlapping half-roll
// and descending pullout. Time, force and physical twist are solved together
// for the true inverted apex and final upright valley. The rolling-descent
// checkpoint is curved; no straight descending port or extra hold is inserted.
// Actual XY, exit heading, speeds and work remain integrated outputs. These
// point forces still require complete finite-train/rider acceptance.

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

// Height-constrained single hill with level, zero-curvature ports. The ascent
// and descent are solved separately with the requested explicit point losses;
// positions are never scaled and exit speed is an output, not reset to entry.
// The bounded family covers 220..280 m and 75..90 m/s when physically feasible.
// A continuous unloading/recovery history brackets an explicit crest phase.
// Its positive knee has no held-load interval. Peak inputs are authoring intent;
// their durations and combined rider histories still require assessment.
// Full finite-train/rider/clearance/operations validation remains mandatory.
struct FvdTallHillRequest {
    double height{240},speed{80},normalG{3.35},crestG{-.025},rampSeconds{1.2};
    double rollingAcceleration{},dragAccelerationCoefficient{};
    double unloadG{1},crestPulseSeconds{1.6};
};
struct FvdTallHillResult {
    FvdRequest authoring;
    FvdResult section;
    FvdSample exit;
    double span{},height{};
};
FvdTallHillResult designFvdTallHill(const FvdTallHillRequest&,Cancel cancel={});
}
