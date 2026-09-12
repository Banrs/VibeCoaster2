#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>
#include <unordered_map>

namespace coaster {
constexpr double pi=3.14159265358979323846, gravity=9.80665;
constexpr double spineDepth=.55,spineRadius=.16,supportRadius=.18;
constexpr const char* generatorVersion="0.8.0-immelmann.1";
struct Vec3 {
    double x{},y{},z{};
    Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator*(double a) const { return {x*a,y*a,z*a}; }
    Vec3 operator/(double a) const { return *this*(1/a); }
};
inline double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double norm(Vec3 a){return std::sqrt(dot(a,a));}
inline Vec3 unit(Vec3 a){double n=norm(a);return n>1e-12?a/n:Vec3{};}
inline bool finite(Vec3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
inline Vec3 rotate(Vec3 v,Vec3 axis,double angle){return v*std::cos(angle)+cross(axis,v)*std::sin(angle)+axis*(dot(axis,v)*(1-std::cos(angle)));}
inline double smooth(double u){u=std::clamp(u,0.,1.);return u*u*u*(10+u*(-15+6*u));}

enum class TerrainKind { Flat, Hills, Canyon };
struct Terrain {
    TerrainKind kind{TerrainKind::Flat};
    // Explicit SI landscape profile. Defaults retain the original analytic fixtures.
    double verticalScale{1},horizontalScale{1},offsetX{},offsetY{},headingRadians{};
    double cliffHeight{},cliffWidth{600};
    static Terrain seeded(TerrainKind,uint64_t seed);
    bool isDefaultProfile() const;
    bool valid() const;
    double slopeBound() const; // Global bound on norm(gradient(height)).
    double localSlopeBound(double x,double y,double radius) const; // Complete world-XY disk, including its interior.
    double height(double x,double y) const;
    std::string name() const;
};
enum class Element { Station, Launch, Hill, Turn, Inversion, Airtime, Brake, Return };
struct AuthoredPoint { Vec3 position; double bank{}; Element element{Element::Return}; Vec3 upHint{}; };
struct Knot { Vec3 position,tangent,curvature,up; double bank{}; Element element{Element::Return}; };
// Deterministic caches rebuilt from the COASTER5 canonical knots.
struct Span { std::array<Vec3,8> c{}; std::array<Vec3,6> referenceUp{}; std::array<double,6> bank{}; double start{},length{}; };
struct TrackSample { Vec3 position,tangent,curvature,up,right; Element element; };
struct TrackLocation { size_t span{}; double parameter{}; };
struct TrackKinematics { TrackSample sample; Vec3 upS,upSS,curvatureS; };
struct Track {
    std::vector<Knot> knots;
    std::vector<Span> spans;
    double length{};
    bool closed{true};
    void rebuild();
    TrackLocation locate(double distance) const;
    TrackSample sample(double distance) const;
    TrackSample sampleSpan(size_t span,double parameter) const;
    std::array<double,6> bankPolynomial(size_t span) const;
    double distanceAtSpan(size_t span,double parameter) const;
};
void rebuildFramePolynomials(Track&);
TrackKinematics sampleSpanKinematics(const Track&,size_t span,double parameter);
TrackKinematics sampleKinematics(const Track&,double distance);
Track compile(const std::vector<AuthoredPoint>& points,bool closed=true);

struct TrainConfig {
    int cars{6}; double carMass{1500},spacing{3.4},seatHeight{1.2};
    double dragCdA{2.4},rollingResistance{0.002},airDensity{1.225};
};
inline double seatDistanceOffset(const TrainConfig& train,int seat){int car=seat==0?0:seat==1?(train.cars-1)/2:train.cars-1;return ((train.cars-1)*.5-car)*train.spacing;}
enum class DriveKind { Launch, Boost, Brake, Station };
struct Operation {
    double start{},end{}; DriveKind kind{DriveKind::Launch};
    double targetSpeed{},maxForce{},maxPower{},rampSeconds{0.08};
    double stopDeceleration{1.8},stopOffset{.025};
    double exitFadeMeters{1}; // Explicit quintic spatial fade before the physical exit.
};
// Merge exact adjacent, identical nonwrapping drive runs in authored order.
void coalesceDriveProfiles(std::vector<Operation>&);
struct Limits {
    // Provisional game envelope, NOT a calibrated or certified rider standard.
    double minVerticalG{-1.5},maxVerticalG{5.5},maxLateralG{1.5},maxLongitudinalG{4.5};
    double maxJerkGps{20},minClearance{2};
    double maxLateralRateGps{std::numeric_limits<double>::quiet_NaN()},maxLongitudinalRateGps{std::numeric_limits<double>::quiet_NaN()};
};
struct ReferenceRecording {
    std::string recordingId,rawSha256,canonicalSha256,analysisSha256,source,notes;
    double sampleRateHz{},sampleRateMinHz{},sampleRateMaxHz{},exposure{};
};
struct ReferenceBenchmark {
    bool processed{false},hasQuartiles{false};
    std::string method,groupId,ride,configuration,seat,device,calibrationId;
    double median{},minimum{},maximum{},q1{},q3{};
    std::vector<ReferenceRecording> recordings;
};
struct Targets {
    double height{220},speed{75},inversionHeight{80},launchSeconds{1.4};
    bool requireIntensity{true};
    double referenceExposure{std::numeric_limits<double>::quiet_NaN()};
    std::string referenceId;
    ReferenceBenchmark reference;
};
struct GenerationRequest {
    uint64_t seed{42}; Terrain terrain; Targets targets; Limits limits; TrainConfig train;
    int maxCandidates{8}; double simulationStep{1./960};
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
struct Frame {double time{},distance{},speed{};std::array<SeatForces,3> seats;};
struct AxisStatistics {
    double minG{},maxG{},meanG{},maxRateGps{};
    double mean1sMin{std::numeric_limits<double>::quiet_NaN()},mean1sMax{std::numeric_limits<double>::quiet_NaN()};
    double mean10sMin{std::numeric_limits<double>::quiet_NaN()},mean10sMax{std::numeric_limits<double>::quiet_NaN()};
};
struct SeatStatistics {
    std::array<AxisStatistics,3> axes; // vertical, lateral, longitudinal
    double exposure10Seconds{},airtimeBelowZeroSeconds{},positiveAbove2Seconds{},positiveAbove3Seconds{},positiveAbove4Seconds{};
    double longestAirtimeSeconds{},longestAbove2Seconds{},longestAbove3Seconds{},longestAbove4Seconds{};
};
AxisStatistics summarizeAxis(const std::vector<double>& values,double step);
struct Metrics {
    double maxSpeed{},heightAboveStation{},maxGroundHeight{},verticalRelief{},inversionGroundHeight{};
    double launchTo180{std::numeric_limits<double>::infinity()},exposure10Seconds{};
    double minVerticalG{1},maxVerticalG{1},maxLateralG{},maxLongitudinalG{},maxJerkGps{};
    double maxJerkDistance{};
    std::array<SeatStatistics,3> seats;
    double duration{},minGroundClearance{std::numeric_limits<double>::infinity()};
};
struct SimulationResult {
    std::vector<Frame> frames; Metrics metrics; ValidationReport report;
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
    double padding() const{return .20;}
    double trainTop() const{return top;}
private:
    ClearanceSweep()=default;
    struct Key {int x,y,z;bool operator==(const Key&) const=default;};
    struct Hash {size_t operator()(Key k)const{return uint64_t(k.x)*73856093ull^uint64_t(k.y)*19349663ull^uint64_t(k.z)*83492791ull;}};
    std::vector<ClearanceFrame> samples;
    std::unordered_map<Key,std::vector<size_t>,Hash> cells;
    double top{},length{};
    friend ClearanceSweep buildClearanceSweep(const Track&,const TrainConfig&,Cancel);
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

enum class StationRole { Platform, Canopy, Post, Pier, Footing };
struct StationBox {
    Vec3 center, forward, right, up, half;
    StationRole role{StationRole::Platform};
};
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
struct Design {
    GenerationRequest request; Track track; std::vector<Operation> operations; std::vector<Support> supports;
    StationGeometry station;
    SimulationResult simulation; ValidationReport report;
    std::string topology; int candidate{};
    std::string generationVersion{generatorVersion};
    std::string planningDiagnostics; // Derived generation-only sidecar.
    std::vector<InversionDimensions> inversionDimensions; // Recomputed canonical bounds, not persisted.
    ConvergenceAssessment convergence; // Recomputed; persisted telemetry is never trusted.
    bool accepted() const {return report.valid()&&simulation.completed&&simulation.report.valid()&&!simulation.cancelled&&convergence.performed&&convergence.passed;}
};
SimulationResult simulate(const Track&,const std::vector<Operation>&,const TrainConfig&,double step=1./960,Cancel cancel={});
ValidationReport validateGeometry(const Track&,const Terrain&,const Limits&,const TrainConfig&,const std::vector<Support>&,Cancel cancel={});
ValidationReport validateRequest(const GenerationRequest&);
void evaluateTargets(Design&);
ValidationReport validateSimulationTargets(const SimulationResult&,const Targets&,const Limits&);
ValidationReport compareSimulationConvergence(const SimulationResult&,const SimulationResult&,const Limits&,ConvergenceAssessment&);
void verifyConvergence(Design&,Cancel cancel={});
void buildSupportLayout(Design&,Cancel cancel={});
Design generate(const GenerationRequest&,Cancel cancel={},std::function<void(int,const std::string&)> progress={});
// Completed ride: powered departure until the train centre reaches final braking.
// Derived from replay and the terminal Station operation, including after load.
double movingRideSeconds(const Design&);
bool saveDesign(const Design&,const std::string& path,std::string& error,Cancel cancel={});
bool loadDesign(const std::string& path,Design&,std::string& error,Cancel cancel={});
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
