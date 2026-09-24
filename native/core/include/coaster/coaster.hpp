#pragma once
#include "coaster/program.hpp"
#include "coaster/terrain.hpp"
#include "coaster/version.hpp"
#include "coaster/progress.hpp"
#include "coaster/recipe.hpp"
#include "coaster/acceleration.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace coaster {
constexpr double spineDepth=.55,spineRadius=.16,supportRadius=.18;
constexpr const char* generatorVersion=COASTER_GENERATOR_VERSION;
inline bool supportedGeneratorVersion(const std::string& version){return version==generatorVersion||version=="2.0.0-default.2"||version=="2.0.0-default.1"||version=="2.0.0-rift.1"||version=="2.0.0-foundation.1"||version=="2.0.0-escarpment.1"||version=="2.0.0-highlands.1"||version=="2.0.0-motion.1";}

enum class Element { Station, Launch, Hill, Turn, Inversion, Airtime, Brake, Return };
struct AuthoredPoint { Vec3 position; double bank{}; Element element{Element::Return}; Vec3 upHint{}; };
struct Knot {
    Vec3 position,tangent,curvature,up;double bank{};Element element{Element::Return};
    Vec3 third,fourth,upFirst,upSecond,upThird;
};
// Deterministic caches rebuilt from the COASTER6 canonical knots.
struct Span {
    std::array<Vec3,10> c{};std::array<Vec3,8> referenceUp{};std::array<double,8> bank{};
    double start{},length{};
    std::array<double,18> arcPolynomial{};bool polynomialArc{}; // Rebuilt, never trusted from disk.
};
struct TrackSample { Vec3 position,tangent,curvature,up,right; Element element; };
struct TrackLocation { size_t span{}; double parameter{}; };
struct TrackKinematics { TrackSample sample; Vec3 upS,upSS,curvatureS,upSSS,curvatureSS; };
struct Track {
    std::vector<Knot> knots;
    std::vector<Span> spans;
    double length{};
    bool closed{true};
    bool authoredGeometry{},authoredFrame{};
    void rebuild();
    TrackLocation locate(double distance) const;
    TrackLocation locate(double distance,size_t& spanHint) const;
    TrackSample sample(double distance) const;
    Vec3 position(double distance,size_t& spanHint) const;
    Vec3 tangent(double distance) const;
    Vec3 tangent(double distance,size_t& spanHint) const;
    TrackSample sampleSpan(size_t span,double parameter) const;
    std::array<double,8> bankPolynomial(size_t span) const;
    double distanceAtSpan(size_t span,double parameter) const;
};
void rebuildFramePolynomials(Track&);
void captureCanonicalDerivatives(Track&);
TrackKinematics sampleSpanKinematics(const Track&,size_t span,double parameter);
TrackKinematics sampleKinematics(const Track&,double distance);
Track compile(const std::vector<AuthoredPoint>& points,bool closed=true);

struct TrainConfig {
    bool operator==(const TrainConfig&) const=default;
    int cars{7}; double carMass{1500},spacing{3.4},seatHeight{1.2};
    // Provisional drag and rolling resistance, tuned against generated rides.
    double dragCdA{3.0},rollingResistance{0.004},airDensity{1.225};
};
inline int riderCapacity(const TrainConfig& train){return 2*train.cars;}
// Modelled two-abreast restrained adult seating. F2291-25 6.6.3 uses
// published 95th-percentile anthropometry plus extended limb reach; 3.1.13
// separately defines the clearance outside reach. See docs/clearance-model.md.
constexpr double limbExtension=.0762,patronSeparation=.0762;
constexpr double patronHalfWidth=.43+1.012+limbExtension+patronSeparation;
constexpr double trainHalfLength=1.275,trainEnvelopeBottom=-spineDepth-spineRadius;
inline double patronTopHeight(const TrainConfig& train){return std::max(1.51,train.seatHeight-.45+1.469+limbExtension+patronSeparation);}
inline double occupiedRadius(const TrainConfig& train){return norm(Vec3{trainHalfLength,patronHalfWidth,std::max(-trainEnvelopeBottom,patronTopHeight(train))});}
inline double seatDistanceOffset(const TrainConfig& train,int seat){int car=seat==0?0:seat==1?(train.cars-1)/2:train.cars-1;return ((train.cars-1)*.5-car)*train.spacing;}
enum class DriveKind { Launch, Boost, Brake, Station, Trim };
struct Operation {
    double start{},end{}; DriveKind kind{DriveKind::Launch};
    double targetSpeed{},maxForce{},maxPower{},rampSeconds{0.08};
    double stopDeceleration{1.8},stopOffset{.025};
    double exitFadeMeters{1}; // Explicit quintic spatial fade before the physical exit.
    double trimPeakSpeed{},trimSensorLead{}; // Eddy-current characteristic and upstream detector; Trim only.
};
// Merge exact adjacent, identical nonwrapping drive runs in authored order.
void coalesceDriveProfiles(std::vector<Operation>&);
struct Limits {
    // User allowance applies to the game envelope, never to F2291 assessment.
    static constexpr double allowanceFactor=1.05;
    // Provisional game envelope, NOT a calibrated or certified rider standard.
    double minVerticalG{-1.5},maxVerticalG{5.0},maxLateralG{1.5},maxLongitudinalG{4.5};
    double maxJerkGps{20},minClearance{0}; // Optional free gap outside the swept envelope.
    double maxLateralRateGps{20},maxLongitudinalRateGps{20};
    bool operator==(const Limits& other) const {
        const auto same=[](double a,double b){return a==b||(std::isnan(a)&&std::isnan(b));};
        return minVerticalG==other.minVerticalG&&maxVerticalG==other.maxVerticalG&&maxLateralG==other.maxLateralG&&maxLongitudinalG==other.maxLongitudinalG&&maxJerkGps==other.maxJerkGps&&minClearance==other.minClearance&&same(maxLateralRateGps,other.maxLateralRateGps)&&same(maxLongitudinalRateGps,other.maxLongitudinalRateGps);
    }
};
struct ReferenceRecording {
    bool operator==(const ReferenceRecording&) const=default;
    std::string recordingId,rawSha256,canonicalSha256,analysisSha256,source,notes;
    double sampleRateHz{},sampleRateMinHz{},sampleRateMaxHz{},exposure{};
};
struct ReferenceBenchmark {
    bool operator==(const ReferenceBenchmark&) const=default;
    bool processed{false},hasQuartiles{false};
    std::string method,groupId,ride,configuration,seat,device,calibrationId;
    double median{},minimum{},maximum{},q1{},q3{};
    std::vector<ReferenceRecording> recordings;
};
struct Targets {
    // SI units; launchSeconds measures acceleration from rest to 50 m/s.
    double height{220},speed{300/3.6},inversionHeight{80},launchSeconds{1.4};
    bool requireIntensity{false}; // Optional calibrated reference comparison.
    double referenceExposure{std::numeric_limits<double>::quiet_NaN()};
    std::string referenceId;
    ReferenceBenchmark reference;
    bool operator==(const Targets& other) const {
        return height==other.height&&speed==other.speed&&inversionHeight==other.inversionHeight&&launchSeconds==other.launchSeconds&&requireIntensity==other.requireIntensity&&(referenceExposure==other.referenceExposure||(std::isnan(referenceExposure)&&std::isnan(other.referenceExposure)))&&referenceId==other.referenceId&&reference==other.reference;
    }
};
struct RideStyle {
    bool operator==(const RideStyle&) const=default;
    double airtime{1},signatureRollDegrees{45};
    int returnStyle{-1}; // -1 seed choice, 0 flowing crest, 1 twin airtime
    bool automaticTrims{true};
};
struct GenerationRequest {
    bool operator==(const GenerationRequest&) const=default;
    uint64_t seed{42}; Terrain terrain; Targets targets; Limits limits; TrainConfig train;
    int maxCandidates{8}; double simulationStep{1./960};
    RideStyle style;
    RideRecipe recipe; // Empty selects the current default; accepted new rides retain the resolved recipe.
};
struct Finding { std::string code,message; double distance{},actual{},limit{}; };
struct ValidationReport {
    std::vector<Finding> errors;
    std::vector<std::string> warnings;
    bool valid() const {return errors.empty();}
    void fail(std::string code,std::string message,double s=0,double actual=0,double limit=0){errors.push_back({code,message,s,actual,limit});}
};
struct SeatForces { double vertical{},lateral{},longitudinal{}; };
SeatForces measureSeatForces(const Track&,double distance,double speed,double tangentialAcceleration,double seatHeight);
struct SeatDynamics {
    SeatForces force,rate;
    Vec3 inertialJerk,angularVelocity,angularAcceleration,angularJerk;
};
SeatDynamics measureSeatDynamics(const Track&,double distance,double speed,double acceleration,double accelerationRate,double seatHeight);
struct Frame {
    double time{},distance{},speed{};std::array<SeatForces,3> seats;
    double acceleration{},accelerationRate{},driveWorkPerMass{},brakeWorkPerMass{},lossWorkPerMass{},energyResidual{};
};
struct ReplayMotion {double distance{},speed{},acceleration{},jerk{};};
ReplayMotion interpolateMotion(const Frame&,const Frame&,double time);
struct AxisStatistics {
    double minG{},maxG{},meanG{},maxRateGps{};
    double mean1sMin{std::numeric_limits<double>::quiet_NaN()},mean1sMax{std::numeric_limits<double>::quiet_NaN()};
    double mean10sMin{std::numeric_limits<double>::quiet_NaN()},mean10sMax{std::numeric_limits<double>::quiet_NaN()};
};
struct SeatStatistics {
    std::array<AxisStatistics,3> axes; // vertical, lateral, longitudinal
    double exposure10Seconds{},airtimeBelowZeroSeconds{},positiveAbove2Seconds{},positiveAbove3Seconds{},positiveAbove4Seconds{};
    double longestAirtimeSeconds{},longestAbove2Seconds{},longestAbove3Seconds{},longestAbove4Seconds{};
    double maxInertialJerk{},maxAngularVelocity{},maxAngularAcceleration{},maxAngularJerk{};
    double maxAngularJerkDistance{},maxLateralRateDistance{};
};
AxisStatistics summarizeAxis(const std::vector<double>& values,double step);
struct Metrics {
    double maxSpeed{},heightAboveStation{},maxGroundHeight{},verticalRelief{},inversionGroundHeight{};
    double launchTo180{std::numeric_limits<double>::infinity()},exposure10Seconds{};
    double minVerticalG{1},maxVerticalG{1},maxLateralG{},maxLongitudinalG{},maxJerkGps{};
    double maxJerkDistance{};
    std::array<SeatStatistics,3> seats;
    double duration{},minGroundClearance{std::numeric_limits<double>::infinity()};
    double driveWorkPerMass{},brakeWorkPerMass{},lossWorkPerMass{},maxEnergyResidual{};
    double peakDrivePowerWatts{},peakBrakePowerWatts{};
};
struct TrimObservation {
    size_t operation{};
    double sensorTime{NAN},sensedSpeed{NAN},deployment{},energyJoules{},peakPowerWatts{};
};
struct SimulationResult {
    std::vector<TrimObservation> trims;
    std::vector<Frame> frames; Metrics metrics; ValidationReport report;
    std::array<AccelerationAssessment,3> acceleration;
    bool completed{},cancelled{};
};
using Cancel=std::function<bool()>;
struct Support;
struct Design;
struct ClearanceFrame {TrackSample sample;double distance{};size_t span{};double parameterBegin{},parameterEnd{},arcLengthBound{},angularVariationBound{};};
class ClearanceSweep {
public:
    ClearanceSweep(const ClearanceSweep&)=default;
    ClearanceSweep(ClearanceSweep&&)=default;
    const std::vector<ClearanceFrame>& frames() const{return samples;}
    double padding() const{return pad;}
    double bodyRadius() const{return radius;}
    double trainTop() const{return top;}
    void prepareGround(const Terrain&,Cancel cancel={});
    const std::vector<double>* groundBounds(const Terrain& terrain) const{return sampledTerrain&&*sampledTerrain==terrain?&groundLowerBounds:nullptr;}
private:
    ClearanceSweep()=default;
    struct Key {int x,y,z;bool operator==(const Key&) const=default;};
    struct Hash {size_t operator()(Key k)const{return uint64_t(k.x)*73856093ull^uint64_t(k.y)*19349663ull^uint64_t(k.z)*83492791ull;}};
    std::vector<ClearanceFrame> samples;
    std::unordered_map<Key,std::vector<size_t>,Hash> cells;
    double top{},length{},pad{},radius{};
    std::optional<Terrain> sampledTerrain;
    std::vector<double> groundLowerBounds;
    friend ClearanceSweep buildClearanceSweep(const Track&,const TrainConfig&,Cancel);
    friend ClearanceSweep buildClearanceSweepVerified(const Track&,const TrainConfig&,Cancel);
    friend int supportCollision(const Support&,const ClearanceSweep&,Cancel);
};
// Canonical-u interval coverage, independent of approximate arc inversion.
// Input must satisfy the checked polynomial and normalized orientation domain.
ClearanceSweep buildClearanceSweep(const Track&,const TrainConfig&,Cancel cancel={});
int supportCollision(const Support&,const ClearanceSweep&,Cancel cancel={});
enum class SupportMemberKind { Steel, Footing };
// Straight closed tapered circular solids in canonical SI coordinates. Empty
// Support.members preserves the historical column/arm at supportRadius exactly.
struct SupportMember {
    Vec3 base,top; double radiusBase{},radiusTop{};
    SupportMemberKind kind{SupportMemberKind::Steel}; bool spineContact{false};
};
struct Support {Vec3 base,top,attachment; bool hasAttachment{false}; double trackDistance{}; std::vector<SupportMember> members;};
constexpr size_t maxSupportMembers=512,maxTotalSupportMembers=60000;
ValidationReport validateSupportMembers(const Support&,const Terrain&,Cancel cancel={});

enum class StationRole {
    Platform, Canopy, Post, Pier, Footing, // Persisted roles 0..4 stay stable.
    QueueDeck, RouteRoof, MergeDeck, HoldingLane, BoardingGate,
    DispatchCabin, UnloadDeck, ExitWalkway, Lift, Stair, Underpass, QueueRail
};
struct StationBox {
    Vec3 center, forward, right, up, half;
    StationRole role{StationRole::Platform};
};
// Separating projections of the actual flat-capped tapered member. A false
// result is inconclusive; callers retain the conservative detailed tests.
bool memberSeparatedFromBox(const SupportMember&,const StationBox&,double padding=0);
struct StationGeometry {
    bool enabled{};
    double boardingBegin{-18}, boardingEnd{64};
    std::vector<StationBox> boxes;
};
bool stationBoxesOverlap(const StationBox&, const StationBox&);
StationGeometry buildStation(const Track&, const Terrain&, const TrainConfig&, Cancel cancel={});
ValidationReport validateStationDefinition(const StationGeometry&, Cancel cancel={});
ValidationReport validateStation(const Track&, const Terrain&, const TrainConfig&, const StationGeometry&, Cancel cancel={});
std::string stationPayload(const StationGeometry&, Cancel cancel={});
bool parseStationPayload(const std::string&, StationGeometry&, std::string& error, Cancel cancel={});

bool supportStationCollision(const Support&,const StationGeometry&,Cancel cancel={});
ValidationReport validateDesignStructures(const Design&,Cancel cancel={});
struct InversionDimensions {
    double startDistance{},endDistance{},pathLength{};
    bool wrapsSeam{},horizontalAxisFallback{};
    Vec3 horizontalForward{},horizontalRight{};
    double verticalMinimum{},verticalMaximum{},verticalExtent{},forwardExtent{},lateralExtent{};
    RideRole role{RideRole::Unspecified};std::string recipeId;
};
// Bounds of labeled canonical inversion spans, not terrain-relative apex height.
// Extrema are evaluated at endpoints and stationary points of the septic.
std::vector<InversionDimensions> measureInversionDimensions(const Track&,Cancel cancel={});
std::pair<double,double> polynomialBounds(const std::array<double,8>& coefficients);
struct ConvergenceMetric {
    std::string name; double coarse{},fine{},absoluteDifference{},tolerance{};
};
struct ConvergenceAssessment {
    bool performed{},passed{}; double coarseStep{},fineStep{},maxSpeedRelativeError{},maxForceRelativeError{};
    std::vector<ConvergenceMetric> metrics;
};
struct RideSection {
    std::string name;
    double start{},end{};
    int heightReversals{};
    bool planar{};
    RideRole role{RideRole::Unspecified};
    std::string recipeId;
};
std::vector<InversionDimensions> measureInversionDimensions(const Track&,const std::vector<RideSection>&,Cancel cancel={});
enum class LandmarkKind {OpeningCrest,OpeningRecovery,PlateauArrival,CliffDeparture,DownhillLaunchExit,CamelbackCrest,WaveCrest,LoopCrest,ImmelmannCrest,SignatureRelease,BrakeEntry};
inline const char* landmarkName(LandmarkKind kind){
    static constexpr const char* names[]{"opening-crest","opening-recovery","plateau-arrival","cliff-departure","downhill-launch-exit","camelback-crest","wave-crest","loop-crest","immelmann-crest","signature-release","brake-entry"};
    const auto i=static_cast<size_t>(kind);return i<std::size(names)?names[i]:"invalid";
}
struct RideLandmark {LandmarkKind kind;double distance{};};
struct SectionAssessment {
    double beginTime{},endTime{},minimumSpeed{},maximumSpeed{},entrySpeed{},exitSpeed{};
    double minimumHeight{},maximumHeight{},maximumPitch{},headingChange{};
    double minimumSignedPitch{},maximumBank{},signedHeadingChange{};
    double meanRailGroundHeight{},minimumRailGroundHeight{},maximumRailGroundHeight{};
    double driveWorkPerMass{},brakeWorkPerMass{},lossWorkPerMass{},energyResidual{};
    double passiveExitSpeedUpperBound{},maximumActuatorAcceleration{},maximumNetAcceleration{};
    int heightReversals{},pitchExtrema{};
};
struct Crossing {
    double firstDistance{},secondDistance{},heightSeparation{},angle{};
    Vec3 position;
};
struct MotionAssessment {
    bool performed{},passed{};
    std::array<double,4> positionJoinError{},orientationJoinError{};
    double flatCoastSeconds{},longestFlatCoastSeconds{};
    // Composition diagnostics only; missing semantic boundaries remain unknown.
    double clifftopActiveSeconds{NAN},clifftopBrakingSeconds{NAN},returnSeconds{NAN};
    double levelCoastSeconds{},longestLevelCoastSeconds{};
    double longestLevelCoastStartDistance{NAN},longestLevelCoastEndDistance{NAN};
    double returnLevelCoastSeconds{},longestReturnLevelCoastSeconds{};
    std::vector<SectionAssessment> sections;
    std::vector<Crossing> crossings;
};
struct SpatialAssessment {
    bool performed{},passed{};
    double maximumPositionError{},maximumOrientationError{};
    ConvergenceAssessment replay;
};
struct Design {
    GenerationRequest request; Track track; std::vector<Operation> operations; std::vector<Support> supports;
    StationGeometry station;
    SimulationResult simulation; ValidationReport report;
    std::string topology; int candidate{};
    std::string generationVersion{generatorVersion};
    std::string planningDiagnostics; // Derived generation-only sidecar.
    std::vector<InversionDimensions> inversionDimensions; // Recomputed canonical bounds, not persisted.
    ConvergenceAssessment convergence; // Recomputed; persisted telemetry is never trusted.
    std::vector<RideSection> sections;
    std::vector<RideLandmark> landmarks;
    std::vector<ForceAuthoring> forcePrograms;
    std::vector<SplineAuthoring> splinePrograms;
    AuthorshipAssessment authorship;
    MotionAssessment motion;
    SpatialAssessment spatial;
    WorkTimings timings;
    bool checksPassed() const {return report.valid()&&simulation.completed&&simulation.report.valid()&&!simulation.cancelled&&convergence.performed&&convergence.passed&&motion.passed&&spatial.passed&&authorship.passed&&std::all_of(simulation.acceleration.begin(),simulation.acceleration.end(),[](const AccelerationAssessment& a){return a.performed&&a.passed&&!a.cancelled;});}
    bool accepted() const {return checksPassed()&&acceptedPayload_&&request==acceptedRequest_;}
private:
    GenerationRequest acceptedRequest_;
    std::shared_ptr<const std::string> acceptedPayload_;
    std::shared_ptr<const std::string> acceptedCache_;
    friend void freezeAcceptedRevision(Design&);
    friend bool saveDesign(const Design&,const std::string&,std::string&,Cancel,Progress);
};
void assessAuthorship(Design&,Cancel cancel={});
std::string authorshipPayload(const Design&);
bool parseAuthorshipPayload(const std::string&,Design&,std::string& error);
void assessMotion(Design&,Cancel cancel={});
void verifySpatialRefinement(Design&,Cancel cancel={});
double minimumSweptGroundClearance(const Track&,const Terrain&,const TrainConfig&,Cancel cancel={});
std::string motionReportJson(const Design&);
SimulationResult simulate(const Track&,const std::vector<Operation>&,const TrainConfig&,double step=1./960,Cancel cancel={});
ValidationReport validateGeometry(const Track&,const Terrain&,const Limits&,const TrainConfig&,const std::vector<Support>&,Cancel cancel={});
ValidationReport validateRequest(const GenerationRequest&);
void evaluateTargets(Design&,const ClearanceSweep* prepared=nullptr);
ValidationReport validateSimulationTargets(const SimulationResult&,const Targets&,const Limits&);
ValidationReport compareSimulationConvergence(const SimulationResult&,const SimulationResult&,const Limits&,ConvergenceAssessment&);
void verifyConvergence(Design&,Cancel cancel={});
void buildSupportLayout(Design&,Cancel cancel={});
Design generate(const GenerationRequest&,Cancel cancel={},Progress progress={});
// Completed ride: powered departure until the train centre reaches final braking.
// Derived from replay and the terminal Station operation, including after load.
double movingRideSeconds(const Design&);
bool saveDesign(const Design&,const std::string& path,std::string& error,Cancel cancel={},Progress progress={});
bool loadDesign(const std::string& path,Design&,std::string& error,Cancel cancel={},Progress progress={});
// Inspection retains rejected, non-rideable data and diagnostics for tooling.
// Normal loadDesign remains atomic and never replaces its output on failure.
Design inspectDesign(const std::string& path,std::string& error,Cancel cancel={},Progress progress={});
std::string reportJson(const Design&);
ValidationReport validateReference(const Targets&);
std::string referenceStatus(const Targets&);
std::string serializeReference(const ReferenceBenchmark&);
bool parseReference(const std::string&,Targets&,std::string& error);
bool loadReference(const std::string& path,Targets&,std::string& error);
std::string referenceReportJson(const Targets&);
std::vector<std::string> referenceLines(const Targets&);
struct RideSummary { bool available{}; uint64_t seed{}; std::string terrain,referenceId,referenceState; Metrics metrics; };
struct ComparisonHistory {
    RideSummary current,previous;
    bool commit(const Design&);
};
std::vector<std::string> comparisonLines(const Design&,const RideSummary* previous=nullptr);

double forceExposure(const std::vector<double>& values,double step,double window=10);
}
