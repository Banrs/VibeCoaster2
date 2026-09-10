#include "coaster/coaster.hpp"
#include "coaster/layout_modules.hpp"
#include "coaster/fvd.hpp"
#include "baseline_jets.hpp"
#include "flow_bridge.hpp"
#include "connector_profile.hpp"
#include "terrain_module_placement.hpp"
#include "passive_transfer.hpp"
#include "terrain_transfer.hpp"
#include "bank_target.hpp"
#include "turn_bank_profile.hpp"
#include "turn_shape.hpp"
#include <numeric>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace coaster {
struct SourceFamilyUnsupported : std::runtime_error {using std::runtime_error::runtime_error;};
struct Random {
    uint64_t state;
    uint64_t next(){uint64_t z=(state+=0x9e3779b97f4a7c15ull);z=(z^(z>>30))*0xbf58476d1ce4e5b9ull;z=(z^(z>>27))*0x94d049bb133111ebull;return z^(z>>31);}
    double range(double a,double b){return a+(b-a)*double(next()>>11)*0x1.0p-53;}
};


// Flat departure sizing is a planning estimate only. The canonical finite-train
// simulator independently measures the first actual crossing of 50 m/s.
static double plannedLaunchTime(double motorAcceleration,const TrainConfig& train){
    constexpr double dt=.0005;double speed=0,time=0,drag=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
    auto acceleration=[&](double v,double t){double external=motorAcceleration*smooth(t/.08)-drag*v*v,friction=gravity*train.rollingResistance;return v>0?external-friction:std::max(0.,external-friction);};
    while(time<10){double a=acceleration(speed,time),mid=std::max(0.,speed+a*dt*.5),next=std::max(0.,speed+acceleration(mid,time+dt*.5)*dt);if(next>=50)return time+dt*(50-speed)/(next-speed);speed=next;time+=dt;}return std::numeric_limits<double>::infinity();
}
static double sizedLaunchAcceleration(const GenerationRequest& req,double seedPreference){
    double upper=req.limits.maxLongitudinalG*gravity-.02,lower=0;
    for(int i=0;i<32;++i){double mid=(lower+upper)*.5;if(plannedLaunchTime(mid,req.train)>req.targets.launchSeconds-.003)lower=mid;else upper=mid;}
    return std::min(req.limits.maxLongitudinalG*gravity-.02,std::max(seedPreference,upper));
}
using detail::TurnShape;
using detail::makeTurn;
struct RoutePlan {
    bool stationChecked{},stationFeasible{},stationSelectionEligible{};double stationBayMinimum{},stationBayMaximum{},stationRequiredDatum{};
    int shape{},placement{};double heading{},length{},score{},relief{},deviation{},grade{},stationGrade{},groundMinimum{},groundMaximum{},valleyFraction{},valleyDistance{};int valleyCrossings{};
    Vec3 origin;std::vector<double> angles,lengths;
};
static double stationBankFactor(double remaining,const TrainConfig& train){double upright=std::max(40.,(train.cars-1)*train.spacing*.5+18);return smooth((remaining-upright)/100);}
static double layoutWarp(double u,double shape){return u+shape*std::sin(2*pi*u)/(2*pi);}
static double layoutSmooth(double u){u=std::clamp(u,0.,1.);return u*u*u*u*(35+u*(-84+u*(70-20*u)));}
static Vec3 inFrame(Vec3 p,double h){return {p.x*std::cos(h)-p.y*std::sin(h),p.x*std::sin(h)+p.y*std::cos(h),p.z};}
static std::vector<RoutePlan> planRoutes(const GenerationRequest& req,int sides,const std::vector<double>& minimum,const std::array<double,4>& radii,const std::array<double,4>& ramps,double terminalReturnMinimum,int holdCorner,const std::array<detail::TerrainModuleFootprint,4>& footprints,Random& rng,Cancel cancel){
    std::vector<RoutePlan> feasible;
    const bool dramaticTerrain=req.terrain.kind==TerrainKind::Canyon&&!req.terrain.isDefaultProfile();
    double startingHeading=rng.range(-pi,pi);Vec3 stationAnchor{rng.range(-200,200),rng.range(-160,160),0};
    for(int shape=0;shape<48;++shape){
        Random shapeRng{rng.next()};
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        RoutePlan p;p.shape=shape;p.angles.resize(sides);p.lengths=minimum;
        // A signed four-leg folded grammar. Paired unequal angles keep the
        // endpoint heading exact while the diagonals cross inside the layout.
        const double alpha=shapeRng.range(.71,.79)*pi,beta=shapeRng.range(.71,.79)*pi;
        p.angles={alpha,-beta,-alpha,beta};
        std::array<TurnShape,4> turns;
        std::vector<Vec3> directions;Vec3 sum{};double h=0;
        for(int side=0;side<sides;++side){directions.push_back({std::cos(h),std::sin(h),0});
            const double extra=side==holdCorner?std::copysign(2*pi,p.angles[side]):0;
            turns[side]=makeTurn(p.angles[side]+extra,radii[side],ramps[side]);sum=sum+inFrame(turns[side].points.back(),h);h+=p.angles[side];}
        // Exact closure leaves two free corridor lengths. Solve their small
        // feasible polygon instead of fixing a randomly padded first leg and
        // charging the resulting slack to the next element's recovery.
        auto bounds=minimum;
        bounds[3]+=std::max(0.,terminalReturnMinimum-turns[3].length);
        Vec3 x=directions[2],y=directions[3];double determinant=cross(x,y).z;
        if(std::abs(determinant)<.2)continue;
        auto closeFor=[&](double first,double second){Vec3 rhs=(sum+directions[0]*first+directions[1]*second)*(-1);return Vec3{cross(rhs,y).z/determinant,cross(x,rhs).z/determinant,0};};
        Vec3 origin=closeFor(0,0),firstSlope=closeFor(1,0)-origin,secondSlope=closeFor(0,1)-origin;
        const std::array<Vec3,4> coefficients{{{1,0,0},{0,1,0},{firstSlope.x,secondSlope.x,origin.x},{firstSlope.y,secondSlope.y,origin.y}}};
        struct Boundary{double x,y,value;};std::array<Boundary,8> boundaries;
        for(int side=0;side<sides;++side){const auto c=coefficients[side];boundaries[2*side]={c.x,c.y,bounds[side]-c.z};boundaries[2*side+1]={-c.x,-c.y,c.z-2800};}
        double shortest=INFINITY;
        for(size_t i=0;i<boundaries.size();++i)for(size_t j=i+1;j<boundaries.size();++j){
            const auto a=boundaries[i],b=boundaries[j];const double det=a.x*b.y-a.y*b.x;
            if(std::abs(det)<1e-10)continue;
            const double first=(a.value*b.y-a.y*b.value)/det,second=(a.x*b.value-a.value*b.x)/det;
            bool valid=true;for(const auto& bound:boundaries)if(bound.x*first+bound.y*second<bound.value-1e-7){valid=false;break;}
            if(!valid)continue;
            std::vector<double> lengths(sides);double total=0;
            for(int side=0;side<sides;++side){const auto c=coefficients[side];lengths[side]=c.x*first+c.y*second+c.z;total+=lengths[side];}
            if(total<shortest){shortest=total;p.lengths=std::move(lengths);}
        }
        if(!std::isfinite(shortest))continue;
        bool valid=true;p.length=0;
        for(int side=0;side<sides;++side){if(p.lengths[side]<bounds[side]-1e-7||p.lengths[side]>2800+1e-7)valid=false;p.length+=p.lengths[side]+turns[side].length;}
        // Major hills/inversion add approximately 400--650 m of 3-D rail to
        // this horizontal budget. Do not build a huge fallback corridor.
        // The exposure hold is an extra closed helix lap, not another long
        // perimeter corridor. Keep its real track length in all reports and
        // resource/clearance checks, but do not charge the same footprint twice.
        const double repeatedHelixLength=holdCorner>=0?2*pi*radii[holdCorner]:0;
        if(!valid||p.length<4700||p.length-repeatedHelixLength>9800)continue;
        std::vector<Vec3> corridor;Vec3 cursor{};h=0;
        std::array<detail::TerrainModuleFootprint,4> placedFootprints;
        constexpr std::array<int,4> footprintCorridor{1,2,1,3};
        for(int side=0;side<sides;++side){
            for(size_t index=0;index<footprints.size();++index)if(footprintCorridor[index]==side){
                const auto& source=footprints[index];auto& placed=placedFootprints[index];
                placed.entrance=cursor+inFrame(source.entrance,h);
                for(const auto& point:source.samples)placed.samples.push_back(cursor+inFrame(point,h));
            }
            int n=int(std::ceil(p.lengths[side]/25));Vec3 forward{std::cos(h),std::sin(h),0};
            for(int i=0;i<=n;++i)corridor.push_back(cursor+forward*(p.lengths[side]*i/n));cursor=cursor+forward*p.lengths[side];
            for(size_t i=14;i<turns[side].points.size();i+=14)corridor.push_back(cursor+inFrame(turns[side].points[i],h));
            cursor=cursor+inFrame(turns[side].points.back(),h);h+=p.angles[side];
        }
        if(norm(cursor)>1e-6)continue;
        for(int placement=0;placement<36;++placement){
            auto q=p;q.placement=placement;
            // Shape, orientation and anchored station placement are jointly
            // ranked from actual terrain; the landscape is never modified.
            q.heading=startingHeading+(placement/3)*pi/6;
            q.origin=stationAnchor+inFrame({0,double(placement%3-1)*180,0},q.heading);
            if(dramaticTerrain){
                // Place the departure signature along the actual canyon floor.
                // Keep the same bounded36 sites: three longitudinal positions,
                // both travel directions and six nearby tangent orientations.
                const double stationAlong=stationAnchor.x+600*(placement%3-1);
                const double stationAcross=120*std::sin(stationAlong/850)+stationAnchor.y*.5;
                q.origin=Vec3{req.terrain.offsetX,req.terrain.offsetY,0}+inFrame(Vec3{stationAlong,stationAcross,0}*req.terrain.horizontalScale,req.terrain.headingRadians);
                q.heading=req.terrain.headingRadians+(placement/18)*pi+std::atan((120./850)*std::cos(stationAlong/850))+
                    (placement/3%6-2.5)*5*pi/180;
            }
            double station=req.terrain.height(q.origin.x,q.origin.y),lo=1e9,hi=-1e9,totalDeviation=0,slope2=0,valleyNear=0,totalValleyDistance=0;Vec3 previous{};double previousGround=0,previousValley=0;bool first=true;
            for(const auto& local:corridor){
                Vec3 point=q.origin+inFrame(local,q.heading);double ground=req.terrain.height(point.x,point.y);lo=std::min(lo,ground);hi=std::max(hi,ground);totalDeviation+=std::abs(ground-station);
                Vec3 terrainPoint=inFrame({(point.x-req.terrain.offsetX)/req.terrain.horizontalScale,(point.y-req.terrain.offsetY)/req.terrain.horizontalScale,0},-req.terrain.headingRadians);
                double valley=terrainPoint.y-120*std::sin(terrainPoint.x/850);totalValleyDistance+=std::abs(valley);valleyNear+=std::abs(valley)<210?1:0;if(!first&&valley*previousValley<0)++q.valleyCrossings;previousValley=valley;
                if(!first){double span=norm(point-previous);if(span>1)slope2+=std::pow((ground-previousGround)/span,2);}first=false;previous=point;previousGround=ground;
            }
            q.groundMinimum=lo;q.groundMaximum=hi;q.valleyFraction=valleyNear/corridor.size();q.valleyDistance=totalValleyDistance/corridor.size();
            q.relief=hi-lo;q.deviation=totalDeviation/corridor.size();q.grade=std::sqrt(slope2/corridor.size());
            Vec3 stationEnd=q.origin+inFrame({80,0,0},q.heading);q.stationGrade=std::abs(req.terrain.height(stationEnd.x,stationEnd.y)-station)/80;
            if(req.terrain.kind==TerrainKind::Canyon&&!dramaticTerrain&&q.valleyFraction<.12)continue;
            q.score=dramaticTerrain?2*std::abs(q.relief-req.terrain.cliffHeight)+.1*q.deviation+200*q.grade+350*q.stationGrade+q.length/65+100*(1-q.valleyFraction):1.5*q.relief+.5*q.deviation+200*q.grade+150*q.stationGrade+q.length/65+(req.terrain.kind==TerrainKind::Canyon?100*(1-q.valleyFraction):0);
            // Rank every rigid force-designed interior, including the complete
            // tail chain. A low rim entry can still leave its exit on a tower
            // above the canyon floor, so assess both ports and the full footprint.
            std::array<detail::TerrainModulePlacement,4> modulePlacement;
            for(size_t index=0;index<placedFootprints.size();++index){
                const auto& footprint=placedFootprints[index];
                auto& assessment=modulePlacement[index];
                assessment=detail::assessTerrainModulePlacement(req.terrain,q.origin,q.heading,footprint,req.limits.minClearance+4.5);
                const auto& exit=footprint.samples.back();const Vec3 worldExit=q.origin+inFrame(exit,q.heading);
                const double exitGroundHeight=assessment.minimumDatum+exit.z-req.terrain.height(worldExit.x,worldExit.y);
                q.score+=assessment.score+2*std::max(0.,exitGroundHeight-24.);
            }
            // A full-span S7 launch transfer has |z''| <= 7.514*rise/L^2.
            // Charge missing transition room against the 3.5 g authoring load,
            // not merely the geometric ability to climb a cliff. This estimate
            // ranks placements; actual grades, drives and rider forces still gate.
            const Vec3 transfer=placedFootprints[2].entrance-placedFootprints[0].samples.back();
            const double rise=modulePlacement[2].minimumDatum+placedFootprints[2].entrance.z-
                modulePlacement[0].minimumDatum-placedFootprints[0].samples.back().z;
            const double normalAllowance=std::max(.1,std::min(3.5,req.limits.maxVerticalG)-1.);
            const double requiredTransfer=65*std::sqrt(7.514*std::abs(rise)/(gravity*normalAllowance));
            q.score+=2*std::max(0.,requiredTransfer-std::hypot(transfer.x,transfer.y));
            feasible.push_back(std::move(q));
        }
    }
    std::stable_sort(feasible.begin(),feasible.end(),[](const RoutePlan& a,const RoutePlan& b){return a.score<b.score;});return feasible;
}
struct AuthoringFeedback {
    std::array<double,4> airtimeSpeed{65,60,60,60};
    std::array<double,4> turnSpeed{65,65,65,65};
    double reversalEnergyCorrection{},loopEnergyCorrection{},loopSupplyEnergyCorrection{},relaunchEnergyCorrection{},loopApproachSpeed{65};
    int planShape{-1},planPlacement{-1};
};
struct CandidatePorts {
    std::array<double,4> airtimeEntry{},turnBegin{},turnEnd{};
    std::array<bool,4> ordinaryTurn{};
    double signatureEntry{},signatureExit{},signatureEntrySpeedHint{},signatureExitSpeedHint{};
    double reversalExit{},reversalSpeedHint{},reversalEntry{},reversalBrakeStart{},relaunchEnd{},reversalEntrySpeedHint{},loopApex{},loopBrakeStart{},loopEntry{},loopSupplyEnd{},loopSupplyTarget{},loopSpeedHint{};
    int planShape{-1},planPlacement{-1};
};
static double replayValueAt(const std::vector<Frame>& frames,double distance,bool time=false){
    if(frames.empty())throw std::runtime_error("Authoring feedback requires an actual replay");
    auto right=std::lower_bound(frames.begin(),frames.end(),distance,[](const Frame& f,double s){return f.distance<s;});
    if(right==frames.begin())return time?right->time:right->speed;
    if(right==frames.end())return time?frames.back().time:frames.back().speed;
    const auto& left=*(right-1);double u=(distance-left.distance)/(right->distance-left.distance);
    return time?left.time+u*(right->time-left.time):left.speed+u*(right->speed-left.speed);
}
double movingRideSeconds(const Design& d){
    for(const auto& op:d.operations)if(op.kind==DriveKind::Station)return replayValueAt(d.simulation.frames,op.start,true);
    throw std::runtime_error("Moving duration requires the terminal station brake");
}
static Design candidate(const GenerationRequest& req,int attempt,Cancel cancel,const AuthoringFeedback& feedback,CandidatePorts& ports,std::unique_ptr<const FvdTallHillResult>& preparedSignature){
    const int geometryAttempt=attempt/2,placementVariant=attempt%2;
    const bool dramaticTerrain=req.terrain.kind==TerrainKind::Canyon&&!req.terrain.isDefaultProfile();
    Design d;d.request=req;d.candidate=attempt;Random rng{req.seed};rng.next();constexpr int sides=4;
    rng.next(); // Preserve seeded route choices from the original grammar.
    bool intensityDesign=req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure);double exposureGoal=intensityDesign?req.targets.referenceExposure*1.1:0;
    double designNormalG=rng.range(3.2,3.6);if(intensityDesign)designNormalG=std::min(req.limits.maxVerticalG-.5,std::max(designNormalG,exposureGoal/10+.35+geometryAttempt*.10));
    designNormalG=std::max(1.2,designNormalG);double ramp=160+geometryAttempt*15;
    double elevation=req.targets.height+rng.range(8,30),loopHeight=rng.range(55,68);
    rng.range(1020,1120); // Preserve unrelated seeded route choices after replacing geometric hill width.
    double loopDrift=rng.range(330,380)+geometryAttempt*10;
    rng.range(7,11); // Source physics now selects supply; retain the route RNG sequence.
    double launchAcceleration=sizedLaunchAcceleration(req,rng.range(38,43));
    if(!preparedSignature){
        if(elevation<220||elevation>280||req.targets.speed>90){
            std::ostringstream reason;reason<<"Tall signature source family supports selected heights 220..280 m and source speeds 75..90 m/s; selected seeded height "
                <<elevation<<" m, requested minimum speed "<<req.targets.speed<<" m/s. Choose supported targets; broader source families are not authored.";
            throw SourceFamilyUnsupported(reason.str());
        }
        FvdTallHillRequest signatureRequest;signatureRequest.height=elevation;
        signatureRequest.rollingAcceleration=gravity*req.train.rollingResistance;
        signatureRequest.dragAccelerationCoefficient=.5*req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass);
        FvdTallHillResult signature;bool signatureFound=false;
        for(double sourceSpeed=std::max(75.,req.targets.speed);sourceSpeed<=90;sourceSpeed+=.25){
            signatureRequest.speed=sourceSpeed;auto trial=designFvdTallHill(signatureRequest,cancel);
            if(trial.section.cancelled)throw std::runtime_error("CANCELLED");
            if(!trial.section.report.valid()||!trial.section.assessment.passed)continue;
            double minimumSpeed=INFINITY;for(const auto& sample:trial.section.samples)minimumSpeed=std::min(minimumSpeed,sample.speed);
            if(minimumSpeed<25)continue;
            signature=std::move(trial);signatureFound=true;break;
        }
        if(!signatureFound)throw std::runtime_error("Tall signature has no bounded source meeting requested speed and 25 m/s traversal intent");
        preparedSignature=std::make_unique<const FvdTallHillResult>(std::move(signature));
    }
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    const auto& signature=*preparedSignature;
    if(std::abs(signature.height-elevation)>1e-4)throw std::runtime_error("Prepared signature height differs from seeded source request");
    const double topSpeed=signature.authoring.speed+.5; // Physical motor margin; actual source agreement is checked.
    const double hillWidth=signature.span;
    rng.range(17,23); // Retain the independent route random sequence.
    // Source variations use a separate stream from route selection.
    Random detail{req.seed^0x8d2f41b79a5c630eull};
    double reversalHeight=std::max(84.,req.targets.inversionHeight+detail.range(7,14));
    detail.range(56,68);detail.range(190,240);
    const double reversalLane=detail.range(95,125);detail.range(72,96);
    detail.range(-.12,.08);detail.range(-.3,.3);
    detail.range(23,26); // Preserve unrelated seeded source choices during this replacement.
    const int reversalHand=(detail.next()&1)?1:-1;
    const double reversalLengthPreference=detail.range(930,1010)/970;
    // Specify the load history independently of the acceptance ceiling.
    // Gravity-coupled FVD solves curvature and true displacement; the final
    // finite-train replay supplies losses and rider-offset forces.
    const double reversalScale=std::sqrt(reversalHeight/88.);
    FvdReversalRequest reversalRequest;
    reversalRequest.entrySpeed=48.9107707974*reversalScale;
    reversalRequest.exitHeight=50*reversalScale*reversalScale;
    reversalRequest.pushRampSeconds=1.2*reversalScale;
    reversalRequest.hand=reversalHand;
    auto reversalSource=designFvdReversal(reversalRequest,cancel);
    if(!reversalSource.section.assessment.passed||!reversalSource.section.report.valid())
        throw std::runtime_error(reversalSource.section.cancelled?"CANCELLED":"Joint force/roll reversal source failed");
    FvdPulloutRequest pulloutRequest;pulloutRequest.entry=reversalSource.section.samples.back();
    pulloutRequest.targetHeight=10*reversalScale*reversalScale;pulloutRequest.normalG=3.5;pulloutRequest.rampSeconds=1.2*reversalScale;
    auto pulloutSource=designFvdPullout(pulloutRequest,cancel);
    if(!pulloutSource.section.assessment.passed||!pulloutSource.section.report.valid())
        throw std::runtime_error(pulloutSource.section.cancelled?"CANCELLED":"Descending reversal pullout failed");
    std::vector<AuthoredPoint> reversalPoints;
    auto appendSource=[&](const FvdResult& source){
        const int samples=int(std::ceil(source.track.length/1.5));
        for(int i=reversalPoints.empty()?0:1;i<=samples;++i){auto p=source.track.sample(source.track.length*i/samples);
            reversalPoints.push_back({p.position,0,Element::Inversion,p.up});}
    };
    appendSource(reversalSource.section);const size_t reversalSourceEnd=reversalPoints.size()-1;
    appendSource(pulloutSource.section);const size_t pulloutSourceEnd=reversalPoints.size()-1;
    const auto valley=pulloutSource.section.samples.back();
    double valleyHeading=std::atan2(valley.forward.y,valley.forward.x);if(valleyHeading<0)valleyHeading+=2*pi;
    const double valleySpeed=std::sqrt(valley.speed*valley.speed+2*gravity*valley.position.z);
    const double valleyRadius=valleySpeed*valleySpeed/(gravity*std::sqrt(designNormalG*designNormalG-1));
    const double valleyPeakBank=std::atan(valleySpeed*valleySpeed/(gravity*valleyRadius));
    const auto valleyTurn=makeTurn(-valleyHeading,valleyRadius,valleySpeed*1.5,valleyPeakBank);
    for(size_t i=1;i<valleyTurn.points.size();++i){
        double s=valleyTurn.length*i/(valleyTurn.points.size()-1);
        Vec3 point=valley.position+inFrame(valleyTurn.points[i],valleyHeading);
        point.z=valley.position.z*(1-layoutSmooth(s/valleyTurn.length));
        reversalPoints.push_back({point,
            valleyTurn.bankAt(s),Element::Turn,{0,0,1}});
    }
    const Vec3 reversalIslandExit=reversalPoints.back().position;
    EnergyLoopModuleRequest loopRequest;loopRequest.height=loopHeight;loopRequest.lateralOffset=18;
    auto loopShape=buildEnergyLoopModule(loopRequest,cancel);
    if(!loopShape.canonicalBuilt||!loopShape.report.valid())throw std::runtime_error(loopShape.cancelled?"CANCELLED":"Force-designed vertical loop failed");
    loopDrift=loopShape.exit.position.x;
    // S7 lateral transfers have |y''| <= 7.514*offset/L^2. Size their
    // horizontal extent from real entry energy and the selected normal-load
    // envelope instead of reserving fixed long elevated and recovery lanes.
    const double reversalEntrySpeed=std::sqrt(reversalRequest.entrySpeed*reversalRequest.entrySpeed+feedback.reversalEnergyCorrection);
    if(!std::isfinite(reversalEntrySpeed)||reversalEntrySpeed<=0)throw std::runtime_error("Measured reversal energy cannot produce a positive entry-speed target");
    const double lateralAcceleration=gravity*std::sqrt(designNormalG*designNormalG-1);
    auto transferLength=[&](double offset,double speed){return speed*std::sqrt(7.514*offset/lateralAcceleration)*reversalLengthPreference;};
    const double brakeDistance=std::max(0.,(65*65-reversalEntrySpeed*reversalEntrySpeed)/(2*3.5))+65*.5+(req.train.cars-1)*req.train.spacing;
    const double reversalLead=std::max(transferLength(reversalLane,65),brakeDistance);
    // The broad return crest flies over the low entry footprint; size the
    // lateral transfer for its load rather than reserving a longer detour.
    // The complete three-dimensional sweep still decides crossing clearance.
    const double reversalReturnOffset=reversalLane+reversalIslandExit.y;
    const double reversalReturnLength=transferLength(std::abs(reversalReturnOffset),valleySpeed);
    const double reversalNetLength=reversalLead+reversalIslandExit.x+reversalReturnLength;
    detail.range(-.05,.05); // Preserve unrelated seeded sources after replacing sin^4 shaping.
    const double loopProfileShape=detail.range(-.04,.04);
    detail.range(1.6,1.85); // Retain the independent shape random sequence.
    std::vector<double> turnRiseScale(sides),turnRiseShape(sides);for(int side=0;side<sides;++side){turnRiseScale[side]=detail.range(.8,1.15);turnRiseShape[side]=detail.range(-.08,.08);}
    std::vector<FvdAirtimeResult> forceHills;
    for(int hill=0;hill<4;++hill){FvdAirtimeRequest force;force.speed=feedback.airtimeSpeed[hill];force.pushG=detail.range(2.08,2.4);force.crestG=detail.range(-.22,-.08);
        auto authored=designFvdAirtime(force,cancel);
        if(!authored.section.integrated||!authored.section.canonicalBuilt||!authored.section.report.valid()||!authored.section.assessment.passed){
            if(authored.section.cancelled)throw std::runtime_error("CANCELLED");
            throw std::runtime_error("FVD hill "+std::to_string(hill)+" at "+std::to_string(force.speed)+" m/s: "+(authored.section.report.errors.empty()?"canonical replay failed":authored.section.report.errors.front().message));
        }
        forceHills.push_back(std::move(authored));}
    const double forceReturnSpan=forceHills[1].span+forceHills[2].span+forceHills[3].span;
    std::vector<double> minimum{200+hillWidth,380+loopDrift+400+forceHills[0].span,
        reversalNetLength,400+forceReturnSpan};
    int holdCorner=intensityDesign&&exposureGoal>18?1:-1;
    std::array<detail::TerrainModuleFootprint,4> footprints;
    auto& loopFootprint=footprints[0];loopFootprint.entrance={380,0,0};
    for(int i=0;i<=32;++i)loopFootprint.samples.push_back({380.*i/32,0,0});
    for(size_t i=0;i<loopShape.points.size();i+=8)loopFootprint.samples.push_back(loopFootprint.entrance+loopShape.points[i].position);
    loopFootprint.samples.push_back(loopFootprint.entrance+loopShape.exit.position);
    auto& reversalFootprint=footprints[1];reversalFootprint.entrance={reversalLead,reversalLane,0};
    for(int i=0;i<=32;++i){double u=i/32.;reversalFootprint.samples.push_back({reversalLead*u,reversalLane*layoutSmooth(u),0});}
    for(size_t i=0;i<reversalPoints.size();i+=8)reversalFootprint.samples.push_back(reversalFootprint.entrance+reversalPoints[i].position);
    reversalFootprint.samples.push_back(reversalFootprint.entrance+reversalIslandExit);
    // Use the same source curves and endpoint displacement as forceHill below.
    // The post-loop launch removes the loop's lateral offset over 400 m; the
    // tail launch ends 400 m into corridor 3 before its three contiguous hills.
    footprints[2].entrance=loopFootprint.samples.back()+Vec3{400,-loopShape.exit.position.y,0};
    footprints[3].entrance={400,0,0};
    for(size_t index:{size_t(2),size_t(3)}){
        auto& footprint=footprints[index];Vec3 cursor=footprint.entrance;
        const int first=index==2?0:1,last=index==2?1:4;
        for(int hill=first;hill<last;++hill){const auto& source=forceHills[hill].section;
            for(int sample=0;sample<=64;++sample)
                footprint.samples.push_back(cursor+source.track.sample(source.track.length*sample/64).position);
            cursor=cursor+source.samples.back().position;footprint.samples.push_back(cursor);
        }
    }
    // Ordinary turn dimensions follow their own measured energy. The first
    // pass uses the previous conservative planning speed; subsequent passes
    // retain the same seeded shape/site while solving exact closure again.
    std::array<double,4> turnRadii{},turnRamps{};
    for(int side=0;side<sides;++side){
        const double speed=feedback.turnSpeed[side];
        turnRadii[side]=speed*speed/(gravity*std::sqrt(designNormalG*designNormalG-1))+(intensityDesign?0:geometryAttempt*12);
        turnRamps[side]=ramp*speed/65;
    }
    // Keep real terminal stopping room rather than a generic recovery pad.
    // The gravity-only source exit speed is conservative about rolling/drag
    // loss; downhill brake force and the actual stop are separately assessed.
    const double tailExitSpeed=forceHills.back().section.samples.back().speed;
    const double trainLength=(req.train.cars-1)*req.train.spacing;
    const double terminalReturnMinimum=tailExitSpeed*tailExitSpeed/(2*2.4)+2*trainLength+67;
    auto plans=planRoutes(req,sides,minimum,turnRadii,turnRamps,terminalReturnMinimum,holdCorner,footprints,rng,cancel);if(plans.empty())throw std::runtime_error("No terrain route fits exact closure and footprint budget");
    // Visit terrain-ranked plans in order. Exact raw departure/return boundary
    // and bay must fit the local budget before committing to a station site.
    // The candidate list is bounded by 48 shapes x 36 placements; rejected
    // station sites do not spend a geometry/physics attempt or alter its limits.
    int firstFeasiblePlacement=-1,transferRejectedSites=0;
    std::array<int,4> rejectedTransferPlacements{};
    for(auto& plan:plans){
    if(std::find(rejectedTransferPlacements.begin(),rejectedTransferPlacements.begin()+transferRejectedSites,plan.placement)!=rejectedTransferPlacements.begin()+transferRejectedSites)continue;
    if(feedback.planShape>=0&&(plan.shape!=feedback.planShape||plan.placement!=feedback.planPlacement))continue;
    if(placementVariant&&feedback.planShape<0&&plan.placement==firstFeasiblePlacement)continue;
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    double heading=plan.heading;Vec3 origin=plan.origin,cursor=origin;
    // Reconstruct only visited plans; placement ranking retains compact parameters
    // instead of copying the same sampled turn geometry for all 36 sites.
    std::array<TurnShape,4> turns;
    for(int side=0;side<sides;++side){const double extra=side==holdCorner?std::copysign(2*pi,plan.angles[side]):0;
        turns[side]=makeTurn(plan.angles[side]+extra,turnRadii[side],turnRamps[side]);}
    const std::string orderName="H-I-A";
    d.topology="folded-bow-tie/H-I-IM-valley-A4";
    size_t forceHoldEnd=0;std::vector<AuthoredPoint> raw;struct Pending{size_t begin,end;DriveKind kind;double speed;double acceleration{3.5};};std::vector<Pending> pending;
    struct ModuleRun{size_t begin,end;int corridor;std::string identity;};std::vector<ModuleRun> modules;
    auto append=[&](Vec3 p,double bank,Element e,Vec3 up=Vec3{0,0,1}){if((raw.size()&1023)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");if(raw.empty()||norm(raw.back().position-p)>1e-6)raw.push_back({p,bank,e,up});};
    auto piece=[&](double samplingLength,const std::function<Vec3(double)>& fn,Element element){
        Vec3 forward{std::cos(heading),std::sin(heading),0},left{-std::sin(heading),std::cos(heading),0},base=cursor;int n=int(std::ceil(samplingLength/1.8));size_t start=raw.empty()?0:raw.size()-1;
        for(int i=0;i<=n;++i){double u=double(i)/n;Vec3 p=fn(u),dv=fn(std::min(1.,u+1e-5))-fn(std::max(0.,u-1e-5));Vec3 tangent=unit(forward*dv.x+left*dv.y+Vec3{0,0,dv.z});Vec3 up=unit(cross(left*(-1),tangent));append(base+forward*p.x+left*p.y+Vec3{0,0,p.z},0,element,up);}
        Vec3 end=fn(1);cursor=base+forward*end.x+left*end.y+Vec3{0,0,end.z};return std::pair{start,raw.size()-1};
    };
    auto line=[&](double length,Element e){return piece(length,[=](double u){return Vec3{length*u,0,0};},e);};
    auto drive=[&](double length,DriveKind kind,double speed){auto range=line(length,kind==DriveKind::Brake?Element::Brake:Element::Launch);pending.push_back({range.first,range.second,kind,speed});return range;};
    auto record=[&](std::pair<size_t,size_t> range,int side,const std::string& identity){modules.push_back({range.first,range.second,side,identity});};
    struct SignatureKnot {size_t index;double sourceDistance;Knot source;};
    std::vector<SignatureKnot> signatureKnots;
    std::array<size_t,4> forceEntryIndices{};size_t reversalExitIndex=0,reversalEntryIndex=0,reversalBrakeStartIndex=0,relaunchEndIndex=0,loopApexIndex=0,loopBrakeStartIndex=0,loopEntryIndex=0;
    auto forceHill=[&](int index,int side){const auto& authored=forceHills[index];const auto& track=authored.section.track;
        Vec3 base=cursor;size_t start=raw.empty()?0:raw.size()-1;
        forceEntryIndices[index]=start;
        const int samples=int(std::ceil(track.length/1.5));
        for(int i=0;i<=samples;++i){auto point=track.sample(track.length*i/samples);append(base+inFrame(point.position,heading),0,Element::Airtime,inFrame(point.up,heading));}
        cursor=base+inFrame(authored.section.samples.back().position,heading);record({start,raw.size()-1},side,"fvd-airtime");
        return authored.span;
    };
    for(int side=0;side<sides;++side){
        double used=0,straight=plan.lengths[side];
        if(side==0){record(line(20,Element::Station),side,"station");record(drive(180,DriveKind::Launch,topSpeed),side,"departure-launch");used=200;}
        if(side==0){
            const Vec3 base=cursor;const size_t first=raw.size()-1;
            const int samples=int(std::ceil(signature.section.track.length/1.5));
            for(int i=0;i<=samples;++i){
                const double s=signature.section.track.length*i/samples;auto point=signature.section.track.sample(s);
                const Vec3 position=base+inFrame(point.position,heading),up=inFrame(point.up,heading);
                append(position,0,Element::Hill,up);
                signatureKnots.push_back({raw.size()-1,s,{position,inFrame(point.tangent,heading),inFrame(point.curvature,heading),up,0,Element::Hill}});
            }
            cursor=base+inFrame(signature.exit.position,heading);
            record({first,raw.size()-1},side,"record-hill");used+=signature.span;
        }else if(side==1){
            double brakeLength=380;
            const double trimTarget=std::sqrt(loopShape.idealEntrySpeed*loopShape.idealEntrySpeed+feedback.loopEnergyCorrection);
            const auto trim=drive(brakeLength,DriveKind::Brake,trimTarget);loopBrakeStartIndex=trim.first;loopEntryIndex=trim.second;
            const double effective=brakeLength-(req.train.cars-1)*req.train.spacing-.5*(feedback.loopApproachSpeed+trimTarget);
            if(effective<=0)throw std::runtime_error("Train and brake fades leave no effective loop trim length");
            pending.back().acceleration=std::max(3.5,(feedback.loopApproachSpeed*feedback.loopApproachSpeed-trimTarget*trimTarget)/(2*effective));
            record(trim,side,"inversion-entry-brake");used+=brakeLength;
            const double offset=loopShape.exit.position.y;Vec3 base=cursor;size_t start=raw.size()-1;
            for(const auto& point:loopShape.points)append(base+inFrame(point.position,heading),point.bank,point.element,inFrame(point.upHint,heading));
            loopApexIndex=size_t(std::max_element(raw.begin()+start,raw.end(),[](const auto& a,const auto& b){return a.position.z<b.position.z;})-raw.begin());
            cursor=base+inFrame(loopShape.exit.position,heading);record({start,raw.size()-1},side,"record-inversion");used+=loopDrift;
            // One identifiable relaunch supplies the next passive section's
            // energy. Its gentle lateral correction restores the loop offset;
            // no hidden corner boost completes an undersized launch later.
            auto launch=piece(400,[=](double u){return Vec3{400*u,-offset*layoutSmooth(u),0};},Element::Launch);
            const double target=std::sqrt(65*65+feedback.relaunchEnergyCorrection);
            if(!std::isfinite(target)||target<=0||target>100)throw std::runtime_error("Relaunch energy left the supported drive domain");
            // Reserve real track for the train and finite entry/exit fades.
            // Size force capacity as well as target speed; a larger target
            // alone cannot make an undersized motor supply missing work.
            const double effectiveLength=400-target-(req.train.cars-1)*req.train.spacing*1.5;
            if(effectiveLength<=0)throw std::runtime_error("Train and motor fades leave no effective relaunch length");
            const double drag=.5*req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass)*target*target+gravity*req.train.rollingResistance;
            const double entryEstimate=std::max(1.,loopShape.idealEntrySpeed-2);
            const double acceleration=std::min(req.limits.maxLongitudinalG*gravity-.1,std::max(4.5,(target*target-entryEstimate*entryEstimate)/(2*effectiveLength)+drag));
            relaunchEndIndex=launch.second;pending.push_back({launch.first,launch.second,DriveKind::Boost,target,acceleration});record(launch,side,"post-loop-launch");used+=400;
            used+=forceHill(0,side);
        }else if(side==2){
            // Lead inward before reversing so the returning ground-level lane
            // is laterally separated from both the lead and the elevated lane.
            // The speed here is an energy-based estimate only: the same real
            // finite-train brake and complete-circuit checks decide eligibility.
            double entrySpeed=reversalEntrySpeed;
            auto lead=piece(reversalLead+reversalLane,[=](double u){return Vec3{reversalLead*u,reversalLane*layoutSmooth(u),0};},Element::Turn);
            reversalBrakeStartIndex=lead.first;pending.push_back({lead.first,lead.second,DriveKind::Brake,entrySpeed});record(lead,side,"immelmann-inward-entry-brake");
            reversalEntryIndex=raw.size()-1;
            const Vec3 islandBase=cursor;size_t segmentStart=raw.size()-1;
            for(size_t i=1;i<reversalPoints.size();++i){const auto& point=reversalPoints[i];
                append(islandBase+inFrame(point.position,heading),point.bank,point.element,inFrame(point.upHint,heading));
                if(i==reversalSourceEnd){record({segmentStart,raw.size()-1},side,"high-immelmann");reversalExitIndex=raw.size()-1;segmentStart=raw.size()-1;}
                if(i==pulloutSourceEnd){record({segmentStart,raw.size()-1},side,"descending-pullout");segmentStart=raw.size()-1;}
            }
            record({segmentStart,raw.size()-1},side,"reversal-valley-turn");
            cursor=islandBase+inFrame(reversalIslandExit,heading);
            const double returnLength=reversalReturnLength;
            record(piece(returnLength+std::abs(reversalReturnOffset),[=](double u){return Vec3{returnLength*u,-reversalReturnOffset*layoutSmooth(u),0};},Element::Turn),side,"interior-low-return");
            used+=reversalNetLength;
        }else{
            record(drive(400,DriveKind::Boost,60),side,"airtime-entry-launch");used+=400;
            for(int h=1;h<=3;++h)used+=forceHill(h,side);
        }
        if(used>straight+1e-6)throw std::runtime_error("Module plan exceeded its actual source-port budget");
        // Source guards already carry matching G3 jets. A positive remainder
        // is a solved positional transfer, not a mandatory recovery/bump.
        const double recoveryLength=std::max(0.,straight-used);
        if(recoveryLength>1e-6)record(line(recoveryLength,Element::Return),side,"corridor-recovery");
        const auto& turn=turns[side];Vec3 base=cursor;size_t turnStart=raw.size()-1;double dz=side==holdCorner?55:0;
        const double turnSpeed=feedback.turnSpeed[side];
        const double bankedRise=std::min(26.,.65*gravity*turn.length*turn.length/(4*pi*pi*turnSpeed*turnSpeed))*turnRiseScale[side];
        for(size_t i=1;i<turn.points.size();++i){double along=turn.length*i/(turn.points.size()-1),curvature=smooth(std::min(along,turn.length-along)/turn.ramp)/turn.radius;cursor=base+inFrame(turn.points[i],heading);cursor.z=base.z+dz*smooth(along/turn.length)+bankedRise*std::pow(std::sin(pi*layoutWarp(along/turn.length,turnRiseShape[side])),4);append(cursor,-std::copysign(std::atan(turnSpeed*turnSpeed*curvature/gravity),turn.angle),Element::Turn);}
        modules.push_back({turnStart,raw.size()-1,side,side==holdCorner?"sustained-helix":"banked-camelback-turn"});if(side==holdCorner)forceHoldEnd=raw.size()-1;
        heading+=plan.angles[side];
    }
    if(std::hypot(cursor.x-origin.x,cursor.y-origin.y)>.001)throw std::runtime_error("Solved route failed canonical XY closure");
    size_t unique=raw.size()-1;std::vector<double> distance(raw.size());for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    if(forceHoldEnd){
        // Keep the reversing pair and its flyover on the same raised datum.
        // Settle the extra helix elevation on the final return before joins
        // and crossing lifts are planned; a later warp would erase clearance.
        size_t returnBegin=0;
        for(const auto& run:modules)if(run.corridor==sides-1&&run.identity=="fvd-airtime")returnBegin=run.end;
        const double begin=distance[returnBegin],remaining=distance.back()-begin;
        for(size_t i=returnBegin;i<raw.size();++i)raw[i].position.z-=55*layoutSmooth((distance[i]-begin)/remaining);
        for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    }
    // Full connecting crests occupy existing transit envelopes. Dimensions
    // follow the gravity energy budget and v^2 curvature, not random telemetry.
    // All existing operation indices survive and finite-train replay remains
    // authoritative after terrain composition and force-aligned banking.
    struct PacingCrest {size_t begin,end;double height,speed;int count;};std::vector<PacingCrest> pacingCrests;
    Random pacing{req.seed^0x5fb6d99a781ec341ull};
    auto pace=[&](size_t first,size_t last,double speed,int count){
        const double total=distance[last]-distance[first],length=total/count;
        const double height=detail::connectorCrestHeight(length,speed);
        const double warp=pacing.range(-.04,.04);
        for(size_t i=first;i<=last;++i){if((i&255)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");double cell=(distance[i]-distance[first])*count/total;double u=i==last?1:cell-std::floor(cell);double rise=height*detail::connectorProfileJet(u,warp)[0];raw[i].position.z+=rise;raw[i].upHint={0,0,1};}
        pacingCrests.push_back({first,last,height,speed,count});
    };
    for(size_t m=0;m<modules.size();++m){const auto& run=modules[m];
        if(run.identity=="inversion-entry-brake")pace(run.begin,run.end,60,1);
        if(run.identity=="inversion-recovery"&&m+1<modules.size()&&modules[m+1].identity=="airtime-entry-boost")pace(run.begin,modules[m+1].end,52,1);
        if(run.identity=="immelmann-inward-entry-brake")pace(run.begin,run.end,55,1);
        // A single broad crest flies over the low Immelmann entry footprint;
        // two crests put their shared valley directly at this crossing.
        if(run.identity=="interior-low-return")pace(run.begin,run.end,46,1);
    }
    for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    // Locate actual authored diagonal crossings after loop offsets and inward
    // routing. Their separation is solved after terrain datums are known.
    // An XY crossing never authorizes acceptance (full sweep/support tests do).
    size_t branchBegin[4]={raw.size(),raw.size(),raw.size(),raw.size()},branchEnd[4]={};
    for(const auto& run:modules){branchBegin[run.corridor]=std::min(branchBegin[run.corridor],run.begin);branchEnd[run.corridor]=std::max(branchEnd[run.corridor],run.end);}
    // Two worst-case supported 4.2m body envelopes plus a planning reserve.
    // This replaces the arbitrary 42m pedestal; the unchanged six-metre
    // certified chord model and every support/body sweep still gate acceptance.
    constexpr double crossingSeparation=2*4.2+.6;
    struct Crossing{size_t under,over;double u,v;};std::vector<Crossing> crossings;
    for(size_t i=branchBegin[1];i<branchEnd[1];++i){Vec3 a=raw[i].position,b=raw[i+1].position-a;
        for(size_t j=branchBegin[3];j<branchEnd[3];++j){Vec3 c=raw[j].position,e=raw[j+1].position-c;double det=cross(b,e).z;if(std::abs(det)<1e-8)continue;
            double u=cross(c-a,e).z/det,v=cross(c-a,b).z/det;if(u<0||u>1||v<0||v>1)continue;
            double along=distance[j]+v*(distance[j+1]-distance[j]);
            if(along-distance[branchBegin[3]]<300||distance[branchEnd[3]]-along<300)continue;
            crossings.push_back({i,j,u,v});
        }
    }
    if(crossings.empty())throw std::runtime_error("Folded route has no separated diagonal crossing");
    // A flyover translates the complete tail element chain. A varying lift
    // through its interiors would replace the requested FVD force histories.
    size_t tailBegin=branchEnd[3],tailEnd=branchBegin[3];
    for(const auto& run:modules)if(run.corridor==3&&run.identity=="fvd-airtime"){
        tailBegin=std::min(tailBegin,run.begin);tailEnd=std::max(tailEnd,run.end);}
    for(size_t i=0;i<raw.size();++i)raw[i].bank*=stationBankFactor(distance.back()-distance[i],req.train);
    if(norm(raw.back().position-raw.front().position)>.001)throw std::runtime_error("Solved route failed canonical height closure");raw.back().position=raw.front().position;

    // Fix the station to its local ground, then solve a smooth baseline away
    // from the boarding/launch boundary. A distant canyon rim cannot lift it.
    Vec3 stationForward{std::cos(plan.heading),std::sin(plan.heading),0},stationRight=cross(stationForward,Vec3{0,0,1});
    double trainHalf=(req.train.cars-1)*req.train.spacing*.5,stationBegin=-std::max(18.,trainHalf+8.),stationEnd=std::max(64.,2*trainHalf+38.);
    double stationGroundMin=1e9,stationGroundMax=-1e9;
    for(double x=stationBegin-2;x<=stationEnd+4;x+=2)for(double y=-6;y<=6;y+=2){Vec3 p=origin+stationForward*x+stationRight*y;double ground=req.terrain.height(p.x,p.y);stationGroundMin=std::min(stationGroundMin,ground);stationGroundMax=std::max(stationGroundMax,ground);}
    double stationDatum=stationGroundMax+req.limits.minClearance+4;
    const int controls=int(std::ceil(distance.back()/50));const double spacing=distance.back()/controls;
    std::vector<double> base(controls),target(controls),lower(controls,-1e9),weight(controls),required(unique),absoluteFloor(unique);std::vector<bool> fixed(controls);
    size_t tallest=0,inverted=0;double maxHill=-1e9,maxLoop=-1e9;
    for(size_t i=0;i<unique;++i){if(raw[i].element==Element::Hill&&raw[i].position.z>maxHill){maxHill=raw[i].position.z;tallest=i;}if(raw[i].element==Element::Inversion&&raw[i].position.z>maxLoop){maxLoop=raw[i].position.z;inverted=i;}}
    for(size_t i=0;i<unique;++i){
        if((i&255)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
        Vec3 tangent=unit(raw[(i+1)%unique].position-raw[(i+unique-1)%unique].position),up=unit(raw[i].upHint-tangent*dot(raw[i].upHint,tangent));up=rotate(up,tangent,raw[i].bank);Vec3 right=unit(cross(tangent,up));
        double floor=-1e9;const double terrainEnvelopeMargin=2*req.terrain.localSlopeBound(raw[i].position.x,raw[i].position.y,8);
        for(double side:{-1.5,1.5})for(double height:{-.8,2.4}){Vec3 corner=raw[i].position+right*side+up*height;floor=std::max(floor,req.terrain.height(corner.x,corner.y)+req.limits.minClearance+1.6+.3+terrainEnvelopeMargin-corner.z);}
        double ground=req.terrain.height(raw[i].position.x,raw[i].position.y);
        if(raw[i].element==Element::Turn){
            // Later force-aligned banking can rotate every authored rider
            // corner. Bound all such orientations, not only the initial bank.
            const double cornerRadius=std::hypot(1.5,2.4),slope=req.terrain.localSlopeBound(raw[i].position.x,raw[i].position.y,cornerRadius);
            floor=std::max(floor,ground+req.limits.minClearance+1.6+cornerRadius*std::hypot(1.,slope)+.3-raw[i].position.z);
        }
        if(i==tallest)floor=std::max(floor,ground+req.targets.height+4-raw[i].position.z);
        if(i==inverted)floor=std::max(floor,ground+req.targets.inversionHeight+4-raw[i].position.z);
        required[i]=floor;absoluteFloor[i]=floor+raw[i].position.z;int k=int(std::llround(distance[i]/spacing))%controls;lower[k]=std::max(lower[k],floor);target[k]+=ground+req.limits.minClearance+4;++weight[k];
    }
    // The first signature shares the departure datum. Preserve its source
    // shape instead of bending its lower ascent to follow a distant rim.
    // A site needing a higher datum must still fit the local station budget.
    int signatureFirst=0,signatureLast=0;
    for(const auto& run:modules)if(run.identity=="record-hill"&&run.corridor==0){
        signatureFirst=std::max(0,int(std::floor(distance[run.begin]/spacing)));
        signatureLast=std::min(controls-1,int(std::ceil(distance[run.end]/spacing)));
        for(int k=signatureFirst;k<=signatureLast;++k)stationDatum=std::max(stationDatum,lower[k]+.3);
    }
    const double fixedDeparture=std::ceil((200+trainHalf+12)/spacing)*spacing,fixedReturn=100;
    auto fixedStationBoundary=[&](double s){return s<=fixedDeparture||distance.back()-s<=fixedReturn;};
    // The same local flat datum must clear both the boarding bay and the
    // actual fixed launch/return boundary. Bound local platform height at 16 m
    // rather than inheriting the highest terrain elsewhere on the circuit.
    for(int k=0;k<controls;++k)if(fixedStationBoundary(k*spacing))stationDatum=std::max(stationDatum,lower[k]+.3);
    // The terminal approach is flattened to this absolute station height.
    // Its authored rise cannot count towards clearance after that replacement.
    for(size_t i=0;i<unique;++i)if(distance.back()-distance[i]<=fixedReturn)
        stationDatum=std::max(stationDatum,absoluteFloor[i]+.3);
    plan.stationChecked=true;plan.stationBayMinimum=stationGroundMin;plan.stationBayMaximum=stationGroundMax;plan.stationRequiredDatum=stationDatum;
    plan.stationFeasible=stationDatum-stationGroundMin<=16;
    if(!plan.stationFeasible)continue;
    for(int k=0;k<controls;++k){target[k]=weight[k]?target[k]/weight[k]:stationDatum;base[k]=std::max(target[k],lower[k]);fixed[k]=fixedStationBoundary(k*spacing);if(fixed[k]){if(lower[k]>stationDatum+1e-8)throw std::runtime_error("Local station datum cannot clear its fixed launch/boarding boundary");base[k]=stationDatum;}}
    for(int k=signatureFirst;k<=signatureLast;++k){base[k]=stationDatum;fixed[k]=true;}
    // Preserve genuine inversion pitch/roll geometry by translating complete
    // modules rigidly above their terrain envelope. Fixed constant control
    // plateaus extend two cells beyond their ports; the shared C3 baseline
    // joins those plateaus through the surrounding lead/recovery geometry.
    {
        std::vector<std::pair<size_t,size_t>> rigidRegions;size_t pairBegin=0,loopBegin=0;
        for(const auto& run:modules){if(run.identity=="inversion-entry-brake")loopBegin=run.begin;
            if(run.identity=="record-inversion")rigidRegions.push_back({loopBegin,run.end});
            if(run.identity=="immelmann-inward-entry-brake")pairBegin=run.begin;
            if(run.identity=="reversal-valley-turn")rigidRegions.push_back({pairBegin,run.end});
            if(run.identity=="fvd-airtime")rigidRegions.push_back({run.begin,run.end});}
        // Adjacent force sections share a datum rather than imposing
        // conflicting plateaus on their common boundary controls.
        std::sort(rigidRegions.begin(),rigidRegions.end());
        std::vector<std::pair<size_t,size_t>> merged;
        for(auto region:rigidRegions){if(!merged.empty()&&distance[region.first]-distance[merged.back().second]<5*spacing)merged.back().second=std::max(merged.back().second,region.second);else merged.push_back(region);}
        rigidRegions=std::move(merged);
        for(const auto& region:rigidRegions){int first=std::max(0,int(std::floor(distance[region.first]/spacing))-2),last=std::min(controls-1,int(std::ceil(distance[region.second]/spacing))+2);double datum=-INFINITY;
            for(int k=first;k<=last;++k)datum=std::max(datum,std::max(base[k],lower[k]));
            for(int k=first;k<=last;++k){if(fixed[k]&&std::abs(base[k]-datum)>1e-8)throw std::runtime_error("Rigid inversion terrain envelope conflicts with station datum");base[k]=datum;fixed[k]=true;}}
    }
    auto wrap=[&](int k){return (k%controls+controls)%controls;};
    // Minimize squared second and third spatial differences together. The
    // third-difference term spreads curvature changes over roughly two 50 m
    // control cells, while terrain lower bounds and the station datum remain
    // hard constraints. This is shape planning, not a force-limit exemption.
    std::vector<Vec3> baselineJets(controls);
    auto solveBaseline=[&]{for(int iteration=0;iteration<1500;++iteration){if((iteration&31)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");double maximumChange=0;for(int k=0;k<controls;++k)if(!fixed[k]){double next=std::max(lower[k],(64*(base[wrap(k-1)]+base[wrap(k+1)])-25*(base[wrap(k-2)]+base[wrap(k+2)])+4*(base[wrap(k-3)]+base[wrap(k+3)])+.001*target[k])/86.001);maximumChange=std::max(maximumChange,std::abs(next-base[k]));base[k]=next;}if(maximumChange<1e-6)break;}baselineJets=detail::minimumSnapJets(base,fixed,cancel);};

    auto baseline=[&](double s){double cell=s/spacing;int k=int(cell)%controls,n=wrap(k+1);double u=cell-std::floor(cell);
        double m0=baselineJets[k].x,m1=baselineJets[n].x,a0=baselineJets[k].y,a1=baselineJets[n].y;
        auto third=[&](int j){return baselineJets[j].z;};
        double c0=base[k],c1=m0,c2=a0*.5,c3=third(k)/6;
        double p=base[n]-c0-c1-c2-c3,v=m1-c1-2*c2-3*c3,a=a1-2*c2-6*c3,j=third(n)-6*c3;
        double c4=35*p-15*v+2.5*a-j/6,c5=-84*p+39*v-7*a+j*.5,c6=70*p-34*v+6.5*a-j*.5,c7=-20*p+10*v-2*a+j/6;
        return c0+u*(c1+u*(c2+u*(c3+u*(c4+u*(c5+u*(c6+u*c7))))));};
    // Transfer ports are fixed heights in the subsequent terrain fit. The
    // generic centimetre refinement tolerance cannot leave their conservative
    // floor a millimetre above the solved baseline and then reject the port.
    std::vector<bool> transferPort(unique);
    for(const auto& run:modules){
        if(run.identity=="post-loop-launch"||run.identity=="airtime-entry-launch"){transferPort[run.begin]=true;transferPort[run.end]=true;}
        if(run.identity=="record-hill")transferPort[run.end]=true;
        if(run.identity=="inversion-entry-brake")transferPort[run.begin]=true;
    }
    transferPort[tailEnd]=true;
    for(int refinement=0;refinement<3;++refinement){solveBaseline();bool raised=false;for(size_t i=0;i<unique;++i){double deficit=required[i]-baseline(distance[i]);if(deficit>(transferPort[i]?1e-9:.01)){int k=int(std::llround(distance[i]/spacing))%controls;if(fixed[k])throw std::runtime_error("Local station boundary violates terrain envelope");lower[k]=std::max(lower[k],base[k]+deficit*1.5);raised=true;}}if(!raised)break;}
    solveBaseline();for(size_t i=0;i<unique;++i)raw[i].position.z+=baseline(distance[i]);
    // Terrain datums can differ across a crossing. Solve the separation after
    // that composition, translating every tail FVD interior by one constant.
    // The adjacent physical launch/terminal transfers are fitted below.
    auto crossingWeight=[&](size_t i){
        if(i<tailBegin)return layoutSmooth((distance[i]-distance[branchBegin[3]])/(distance[tailBegin]-distance[branchBegin[3]]));
        if(i>tailEnd)return 1-layoutSmooth((distance[i]-distance[tailEnd])/(distance[branchEnd[3]]-distance[tailEnd]));
        return 1.;
    };
    double crossingLift=0;
    for(const auto& crossing:crossings){
        const auto i=crossing.under,j=crossing.over;const double u=crossing.u,v=crossing.v;
        const double under=raw[i].position.z*(1-u)+raw[i+1].position.z*u;
        const double over=raw[j].position.z*(1-v)+raw[j+1].position.z*v;
        const double crossingInfluence=crossingWeight(j)*(1-v)+crossingWeight(j+1)*v;
        if(crossingInfluence<=1e-9)throw std::runtime_error("Crossing conflicts with the fixed return boundary");
        crossingLift=std::max(crossingLift,(under+crossingSeparation-over)/crossingInfluence);
    }
    for(size_t i=branchBegin[3];i<=branchEnd[3];++i)raw[i].position.z+=crossingLift*crossingWeight(i);
    // Give purposeful transfers their full grade-change domain instead of
    // compressing terrain relief between neighbouring rigid guard plateaus.
    auto heightTransfer=[&](size_t first,size_t last,bool terminal=false,detail::TerrainPortContract contract=detail::TerrainPortContract::LevelAuthored){
        std::vector<double> horizontal(last-first+1),floor(last-first+1);
        for(size_t i=first;i<=last;++i){
            if(i>first){const Vec3 delta=raw[i].position-raw[i-1].position;horizontal[i-first]=horizontal[i-first-1]+std::hypot(delta.x,delta.y);}
            floor[i-first]=absoluteFloor[i];
        }
        // These are non-inverting terrain transfers, paced around 65 m/s.
        // The radius screen uses the configured upright force budget; actual
        // train speed, banking and rider offsets still pass full validation.
        const double forceBudget=std::min(req.limits.maxVerticalG-1,1-req.limits.minVerticalG);
        // Terminal braking changes speed throughout the descent; a constant
        // 65 m/s force estimate cannot reject that profile before simulation.
        const double curvatureBudget=terminal?.2:forceBudget*gravity/(65*65);
        detail::TerrainTransfer transfer;
        // Dedicated launch and terminal modules retain their authored level
        // ports. The hill-to-cliff ascent explicitly inherits the preserved
        // incoming terrain grade; resetting that substantial climb made a
        // false crest. Small baseline mismatches at other level ports remain
        // the local connector's responsibility, not a universal free-port fit.
        try{
            if(contract==detail::TerrainPortContract::PreserveIncoming)
                transfer=detail::fitJetTerrainTransfer(horizontal,floor,raw[first].position.z,raw[last].position.z,detail::terrainPortHeightJet(raw,first,true),{},2.,curvatureBudget,cancel);
            else transfer=detail::fitTerrainTransfer(horizontal,floor,raw[first].position.z,raw[last].position.z,2.,curvatureBudget,cancel);
        }
        catch(const detail::TerrainTransferInfeasible&){
            // Terrain fitting is station-site feasibility, before physics or
            // energy feedback. A pinned feedback site must remain exact.
            if(feedback.planShape>=0)throw;
            rejectedTransferPlacements[transferRejectedSites++]=plan.placement;
            if(transferRejectedSites==int(rejectedTransferPlacements.size()))throw;
            return false;
        }
        for(size_t i=first;i<=last;++i)raw[i].position.z=transfer.height(horizontal[i-first]);
        return true;
    };
    bool transfersFit=true;
    for(const auto& run:modules)if(run.identity=="post-loop-launch"||run.identity=="airtime-entry-launch")
        if(!heightTransfer(run.begin,run.end)){transfersFit=false;break;}
    if(!transfersFit)continue;
    size_t ascentBegin=0,ascentFinish=0;
    for(const auto& run:modules){if(run.corridor==0&&run.identity=="record-hill")ascentBegin=run.end;if(run.identity=="inversion-entry-brake")ascentFinish=run.begin;}
    if(req.terrain.cliffHeight>0&&ascentFinish>ascentBegin&&raw[ascentFinish].position.z-raw[ascentBegin].position.z>12){
        // The preserved force-designed source has a level zero-curvature exit. Use
        // that authored contract instead of estimating it from one-sided knots.
        // The operation boundary is not a geometric level-port requirement.
        // Finish the climb at the actual inversion source inlet, using the
        // existing straight trim approach for the corner's pitch unwind.
        if(!heightTransfer(ascentBegin,loopEntryIndex))continue;
        pending.push_back({ascentBegin,ascentFinish,DriveKind::Boost,65});
        // This is one powered ascent, not three independent reset segments.
        // Its entrance join can use the actual available transition length.
        auto first=std::find_if(modules.begin(),modules.end(),[&](const ModuleRun& run){return run.begin==ascentBegin;});
        auto last=std::find_if(first,modules.end(),[&](const ModuleRun& run){return run.begin==ascentFinish;});
        auto next=modules.erase(first,last);modules.insert(next,{ascentBegin,ascentFinish,0,"terrain-ascent-launch"});
    }
    const size_t levelReturn=size_t(std::lower_bound(distance.begin(),distance.end(),distance.back()-100)-distance.begin());
    if(raw[tailEnd].position.z-stationDatum>12){
        for(size_t i=levelReturn;i<raw.size();++i)raw[i].position.z=stationDatum;
        if(!heightTransfer(tailEnd,levelReturn,true))continue;
    }
    // Defer the first fully terrain-feasible site, not a station bay whose
    // connecting transfers cannot fit. This keeps the alternate geometry
    // attempt on a genuinely different placement after bounded site retries.
    if(placementVariant&&feedback.planShape<0&&firstFeasiblePlacement<0){firstFeasiblePlacement=plan.placement;continue;}
    plan.stationSelectionEligible=true;
    // Terrain composition is complete before local connections are fitted.
    for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    // Match motion locally at each boundary. A single polynomial across a
    // whole recovery erased its authored crest and amplified endpoint jets
    // into an unintended hill. Keep the connector interior and its energy
    // intent; drive zones retain their indices through compilation.
    auto connector=[](const std::string& name){return name=="recovery"||name=="corridor-recovery"||name=="inversion-recovery"||name=="airtime-entry-boost";};
    std::vector<std::pair<size_t,size_t>> flowJoins;
    size_t flowBridgeBegin=0,flowBridgeEnd=0;
    for(size_t m=0;m+1<modules.size();++m){
        const auto& left=modules[m];const auto& right=modules[m+1];
        if(left.identity=="station"||left.identity=="departure-launch")continue;
        if(left.identity=="terrain-ascent-launch"&&right.identity=="inversion-entry-brake")continue;
        // Airtime sections already have level zero-curvature guards. Keep
        // their matching source ports; refitting them can amplify tiny jets.
        if(left.identity=="fvd-airtime"||right.identity=="fvd-airtime"||left.identity=="record-hill"||right.identity=="record-hill")continue;
        // Joint FVD sources already meet at their actual pose and zero-load
        // derivative ports. Do not replace their interiors with a join warp.
        if(right.identity=="high-immelmann"||right.identity=="descending-pullout"||right.identity=="reversal-valley-turn")continue;
        // Recovery and its boost can share one continuous authored crest.
        if(connector(left.identity)&&connector(right.identity))continue;
        auto forceInversion=[](const std::string& identity){return identity=="record-inversion"||identity=="high-immelmann"||identity=="dive-loop"||identity=="fvd-airtime";};
        // FVD pitch/roll interiors are already force-designed. Borrow only
        // their straight guards, never overwrite the force ramp itself.
        const double begin=distance[left.end]-std::min(forceInversion(left.identity)?6.:65.,.25*(distance[left.end]-distance[left.begin]));
        const double end=distance[right.begin]+std::min(forceInversion(right.identity)?6.:65.,.25*(distance[right.end]-distance[right.begin]));
        // Terminal braking and the level station approach retain their own
        // physical profiles; this pass composes the moving ride's elements.
        size_t first=size_t(std::lower_bound(distance.begin(),distance.end(),begin)-distance.begin());
        if(first>=tailEnd)continue;
        size_t last=size_t(std::lower_bound(distance.begin(),distance.end(),end)-distance.begin());
        try{detail::blendAuthoredJoin(raw,first,last,cancel);}catch(const std::exception& e){throw std::runtime_error("Join "+left.identity+" to "+right.identity+": "+e.what());}flowJoins.push_back({first,last});
        if(left.identity=="interior-low-return"){flowBridgeBegin=first;flowBridgeEnd=last;}
        // The whole curved part of a drive participates in force-aligned
        // banking, including shared endpoints. Its motor remains an operation.
        if(raw[first].element==Element::Turn||raw[last].element==Element::Turn)
            for(size_t i=first;i<=last;++i)if(raw[i].element==Element::Launch||raw[i].element==Element::Brake||raw[i].element==Element::Return)raw[i].element=Element::Turn;
    }
    if(!flowBridgeEnd)throw std::runtime_error("Folded flow family has no returning-S raccord");
    for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    raw.back()=raw.front();try{d.track=compile(raw);}catch(const std::exception& e){throw std::runtime_error("Composed circuit: "+std::string(e.what()));}
    // Keep exact canonical source tangent/curvature/up after the surrounding
    // authored polyline compiles. Terrain may rigidly translate this source.
    const Vec3 signatureShift=raw[signatureKnots.front().index].position-signatureKnots.front().source.position;
    for(const auto& point:signatureKnots){
        if(norm(raw[point.index].position-point.source.position-signatureShift)>1e-5)
            throw std::runtime_error("Tall signature source was warped by terrain or a connector");
        auto& knot=d.track.knots[point.index];knot.tangent=point.source.tangent;knot.curvature=point.source.curvature;knot.up=point.source.up;knot.bank=0;
    }
    d.track.rebuild();
    auto at=[&](size_t index){return index>=d.track.spans.size()?d.track.length:d.track.spans[index].start;};
    double signaturePositionResidual=0,signatureCurvatureResidual=0;
    const double signatureBegin=at(signatureKnots.front().index),signatureEnd=at(signatureKnots.back().index);
    for(const auto& point:signatureKnots){
        auto q=d.track.sample(signatureBegin+point.sourceDistance);
        signaturePositionResidual=std::max(signaturePositionResidual,norm(q.position-point.source.position-signatureShift));
        signatureCurvatureResidual=std::max(signatureCurvatureResidual,norm(q.curvature-point.source.curvature));
    }
    std::array<double,2> signaturePortCurvatureS{},signaturePortUpSS{},signatureSourceCurvatureS{},signatureSourceUpSS{};
    for(int port=0;port<2;++port){
        auto actual=sampleKinematics(d.track,port?signatureEnd:signatureBegin);
        auto source=sampleKinematics(signature.section.track,port?signature.section.track.length:0);
        signaturePortCurvatureS[port]=norm(actual.curvatureS);signaturePortUpSS[port]=norm(actual.upSS);
        signatureSourceCurvatureS[port]=norm(source.curvatureS);signatureSourceUpSS[port]=norm(source.upSS);
    }
    ports.signatureEntry=signatureBegin;ports.signatureExit=signatureEnd;
    ports.signatureEntrySpeedHint=signature.authoring.speed;ports.signatureExitSpeedHint=signature.exit.speed;
    for(size_t i=0;i<forceEntryIndices.size();++i)ports.airtimeEntry[i]=at(forceEntryIndices[i]);
    ports.reversalExit=at(reversalExitIndex);ports.reversalSpeedHint=reversalSource.section.samples.back().speed;ports.reversalEntry=at(reversalEntryIndex);ports.reversalEntrySpeedHint=reversalEntrySpeed;
    ports.reversalBrakeStart=at(reversalBrakeStartIndex);ports.relaunchEnd=at(relaunchEndIndex);
    ports.loopEntry=at(loopEntryIndex);ports.loopBrakeStart=at(loopBrakeStartIndex);ports.loopApex=at(loopApexIndex);ports.loopSpeedHint=loopShape.idealApexSpeed;
    ports.planShape=plan.shape;ports.planPlacement=plan.placement;
    ports.ordinaryTurn.fill(false);
    for(const auto& run:modules)if(run.identity=="banked-camelback-turn"&&run.corridor<3){
        ports.ordinaryTurn[run.corridor]=true;ports.turnBegin[run.corridor]=at(run.begin);ports.turnEnd[run.corridor]=at(run.end);
    }
    const bool terrainDriveControl=req.terrain.kind!=TerrainKind::Flat&&!req.terrain.isDefaultProfile();
    struct TerrainDriveSizing{std::string module;double nominalSpeed,targetSpeed,actualRise,rawRise;};std::vector<TerrainDriveSizing> terrainDriveSizing;
    // Supply feedback belongs to the nearest existing motor before the trim,
    // including a terrain ascent when present. A brake cannot add missing work.
    size_t loopSupply=pending.size();
    for(size_t j=0;j<pending.size();++j)if((pending[j].kind==DriveKind::Launch||pending[j].kind==DriveKind::Boost)&&pending[j].end<=loopBrakeStartIndex&&
        (loopSupply==pending.size()||pending[j].end>pending[loopSupply].end))loopSupply=j;
    if(loopSupply==pending.size())throw std::runtime_error("Loop energy planning has no preceding powered section");
    ports.loopSupplyEnd=at(pending[loopSupply].end);
    for(size_t pendingIndex=0;pendingIndex<pending.size();++pendingIndex){const auto p=pending[pendingIndex];
        bool hard=p.kind==DriveKind::Launch;double acc=hard?launchAcceleration:p.acceleration,targetSpeed=p.speed;
        if(terrainDriveControl){
            for(size_t index=0;index+1<modules.size();++index)if(modules[index].begin==p.begin&&modules[index].end==p.end){
                const auto& next=modules[index+1];double rawRise=next.identity=="record-hill"?elevation:next.identity=="record-inversion"?loopHeight:next.identity=="high-immelmann"?reversalSource.geometricApexHeight:0;
                if(rawRise>0){
                    const double start=at(next.begin),end=at(next.end),entryHeight=d.track.sample(start).position.z;double crestHeight=entryHeight;
                    for(double location=start;location<end;location+=2)crestHeight=std::max(crestHeight,d.track.sample(location).position.z);
                    crestHeight=std::max(crestHeight,d.track.sample(end).position.z);
                    const double actualRise=crestHeight-entryHeight;
                    // Preserve the nominal kinetic-energy margin over the real
                    // canonical climb. This is only a controller target; motor
                    // force/power, losses and finite train dynamics remain real.
                    targetSpeed=std::sqrt(std::max(20*20.,p.speed*p.speed+2*gravity*(actualRise-rawRise)));
                    terrainDriveSizing.push_back({next.identity,p.speed,targetSpeed,actualRise,rawRise});
                }
                break;
            }
        }
        if(pendingIndex==loopSupply){
            targetSpeed=std::sqrt(targetSpeed*targetSpeed+feedback.loopSupplyEnergyCorrection);
            if(!std::isfinite(targetSpeed)||targetSpeed<=0)throw std::runtime_error("Loop supply target left its finite energy domain");
            if(feedback.loopSupplyEnergyCorrection>0){
                const double usable=at(p.end)-at(p.begin)-2*trainHalf-targetSpeed*(hard?.16:1.);
                if(usable<=0)throw std::runtime_error("Loop supply has no length after finite train and drive ramps");
                const double extraDrag=.5*req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass)*feedback.loopSupplyEnergyCorrection;
                acc=std::min(req.limits.maxLongitudinalG*gravity-(hard?.02:.1),acc+feedback.loopSupplyEnergyCorrection/(2*usable)+extraDrag);
            }
            ports.loopSupplyTarget=targetSpeed;
        }
        if(p.kind==DriveKind::Boost||p.kind==DriveKind::Brake){
            double opposingGrade=0;for(double s=at(p.begin);s<at(p.end);s+=2)opposingGrade=std::max(opposingGrade,(p.kind==DriveKind::Brake?-1:1)*d.track.sample(s).tangent.z);
            acc=std::min(req.limits.maxLongitudinalG*gravity-.1,acc+gravity*opposingGrade);
        }
        d.operations.push_back({at(p.begin),at(p.end),p.kind,targetSpeed,req.train.carMass*acc,req.train.carMass*acc*100,hard?.08:.5});

    }
    // Terminal hardware starts after the last authored airtime section, with
    // room for the rear car to clear it before the front car reaches a brake.
    // Size stopping work from the available return, never put brake hardware
    // inside a thrill element merely to obtain a fixed stopping distance.
    const double brakingStart=at(tailEnd)+2*trainHalf+2;
    const double stoppingRoom=d.track.length-brakingStart-2*trainHalf-65;
    if(stoppingRoom<=0)throw std::runtime_error("Final element leaves no physical terminal brake run");
    const double stopDeceleration=std::max(2.4,65*65/(2*stoppingRoom));
    if(stopDeceleration>std::min(20.,req.limits.maxLongitudinalG*gravity-.1))
        throw std::runtime_error("Final element leaves insufficient bounded stopping work");
    for(auto& op:d.operations)if(op.kind==DriveKind::Boost)op.end=std::min(op.end,brakingStart);
    d.operations.erase(std::remove_if(d.operations.begin(),d.operations.end(),[](const Operation& op){return op.end<=op.start;}),d.operations.end());
    // The final corridor can climb a real canyon shelf. Brakes cannot supply
    // that potential energy. An explicit bounded transfer drive holds only
    // the low approach speed when needed; it ends before the level station
    // boundary, leaving the existing independently simulated station stop.
    double terminalLow=d.track.knots.front().position.z;
    for(double s=brakingStart;s<d.track.length;s+=5)terminalLow=std::min(terminalLow,d.track.sample(s).position.z);
    const double terminalRise=d.track.knots.front().position.z-terminalLow;
    if(terminalRise>12) d.operations.push_back({brakingStart,d.track.length-100,DriveKind::Boost,24,req.train.carMass*3.5,req.train.carMass*350,.5});
    // A real descent can exert more than the former 4 m/s^2 brake rating.
    // Size the hardware against the canonical downhill grade plus the same
    // requested stopping deceleration; the controller still applies bounded
    // force/power through its ramp and the complete train is resimulated.
    double terminalDownhill=0;
    for(double s=brakingStart;s<d.track.length;s+=1)terminalDownhill=std::max(terminalDownhill,-d.track.sample(s).tangent.z);
    const double terminalBrakeAcceleration=std::min(req.limits.maxLongitudinalG*gravity-.1,std::max(4.,stopDeceleration+gravity*terminalDownhill+1));
    d.operations.push_back({brakingStart,80,DriveKind::Station,0,req.train.carMass*terminalBrakeAcceleration,req.train.carMass*terminalBrakeAcceleration*100,.5,2.4,.2});
    for(auto& operation:d.operations)operation.exitFadeMeters=std::max(1.,operation.targetSpeed*operation.rampSeconds);
    coalesceDriveProfiles(d.operations);
    d.topology+="/joint-fvd-immelmann-valley";
    if(!flowJoins.empty())d.topology+="/continuous-module-joins";
    if(!pacingCrests.empty())d.topology+="/paced-connectors";
    std::ostringstream diagnostic;diagnostic<<std::setprecision(12)<<"{\"schemaVersion\":1,\"generationOnly\":true,\"seed\":"<<req.seed<<",\"candidate\":"<<attempt<<",\"order\":\""<<orderName<<"\",\"stationAnchoring\":{\"geometryScale\":"<<geometryAttempt<<",\"placementVariant\":"<<placementVariant<<",\"deferredFirstPlacement\":"<<firstFeasiblePlacement<<",\"selectedTerrainRank\":"<<(&plan-plans.data())<<",\"checkedPlanCount\":"<<(&plan-plans.data()+1)<<",\"transferRejectedSites\":"<<transferRejectedSites<<",\"heightBudget\":16,\"datum\":"<<stationDatum<<",\"groundMinimum\":"<<stationGroundMin<<",\"groundMaximum\":"<<stationGroundMax<<",\"datumAboveLowestGround\":"<<stationDatum-stationGroundMin<<",\"datumAboveHighestGround\":"<<stationDatum-stationGroundMax<<",\"fixedDepartureMeters\":"<<fixedDeparture<<",\"fixedReturnMeters\":"<<fixedReturn<<",\"controlSpacing\":"<<spacing<<"},\"launchPlanning\":{\"requestedSeconds\":"<<req.targets.launchSeconds<<",\"motorAcceleration\":"<<launchAcceleration<<",\"predictedSeconds\":"<<plannedLaunchTime(launchAcceleration,req.train)<<"},\"intensityPlanning\":{\"configured\":"<<(intensityDesign?"true":"false")<<",\"exposureGoal\":"<<exposureGoal<<",\"normalLoadHint\":"<<designNormalG<<",\"extraHelixCorner\":"<<holdCorner<<"},\"sides\":"<<sides<<",\"plannedHorizontalLength\":"<<plan.length<<",\"canonicalLength\":"<<d.track.length<<",\"selected\":{\"shape\":"<<plan.shape<<",\"placement\":"<<plan.placement<<",\"score\":"<<plan.score<<",\"terrainRelief\":"<<plan.relief<<",\"terrainStationDeviation\":"<<plan.deviation<<",\"terrainGradeRms\":"<<plan.grade<<",\"stationGrade\":"<<plan.stationGrade<<",\"groundMinimum\":"<<plan.groundMinimum<<",\"groundMaximum\":"<<plan.groundMaximum<<",\"valleyFraction\":"<<plan.valleyFraction<<",\"meanValleyDistance\":"<<plan.valleyDistance<<",\"valleyCrossings\":"<<plan.valleyCrossings<<"},\"scoreWeights\":{\"terrainRelief\":"<<(dramaticTerrain?0:1.5)<<",\"terrainReliefTarget\":"<<(dramaticTerrain?req.terrain.cliffHeight:0)<<",\"terrainReliefTargetDeviation\":"<<(dramaticTerrain?2:0)<<",\"terrainStationDeviation\":"<<(dramaticTerrain?.1:.5)<<",\"terrainGradeRms\":200,\"stationGrade\":"<<(dramaticTerrain?350:150)<<",\"horizontalLength\":0.015384615384615385,\"canyonOutsideValleyFraction\":"<<(req.terrain.kind==TerrainKind::Canyon?100:0)<<",\"valleyCoordinateUsesTerrainProfile\":"<<"true"<<"},\"corridors\":[";
    for(int i=0;i<sides;++i){if(i)diagnostic<<',';diagnostic<<"{\"length\":"<<plan.lengths[i]<<",\"turnAngle\":"<<plan.angles[i]<<",\"turnSpeedMps\":"<<feedback.turnSpeed[i]<<",\"turnRadiusMeters\":"<<turns[i].radius<<",\"turnRampMeters\":"<<turns[i].ramp<<'}';}diagnostic<<"],\"rankedPlans\":[";
    for(size_t i=0;i<plans.size();++i){if(i)diagnostic<<',';auto& p=plans[i];diagnostic<<"{\"shape\":"<<p.shape<<",\"placement\":"<<p.placement<<",\"score\":"<<p.score<<",\"relief\":"<<p.relief<<",\"deviation\":"<<p.deviation<<",\"grade\":"<<p.grade<<",\"stationGrade\":"<<p.stationGrade<<",\"length\":"<<p.length<<",\"groundMinimum\":"<<p.groundMinimum<<",\"groundMaximum\":"<<p.groundMaximum<<",\"valleyFraction\":"<<p.valleyFraction<<",\"meanValleyDistance\":"<<p.valleyDistance<<",\"valleyCrossings\":"<<p.valleyCrossings<<",\"stationBudgetChecked\":"<<(p.stationChecked?"true":"false")<<",\"stationBudgetFeasible\":"<<(p.stationFeasible?"true":"false")<<",\"stationSelectionEligible\":"<<(p.stationSelectionEligible?"true":"false");if(p.stationChecked)diagnostic<<",\"stationBayMinimum\":"<<p.stationBayMinimum<<",\"stationBayMaximum\":"<<p.stationBayMaximum<<",\"stationRequiredDatum\":"<<p.stationRequiredDatum;diagnostic<<'}';}diagnostic<<"],\"modules\":[";
    for(size_t i=0;i<modules.size();++i){if(i)diagnostic<<',';auto& m=modules[i];diagnostic<<"{\"identity\":\""<<m.identity<<"\",\"corridor\":"<<m.corridor<<",\"start\":"<<at(m.begin)<<",\"end\":"<<at(m.end)<<'}';}diagnostic<<"]}";d.planningDiagnostics=diagnostic.str();
    d.planningDiagnostics.pop_back();std::ostringstream expansion;expansion<<std::setprecision(12)<<",\"layoutExpansion\":{\"reversingPair\":"<<"true"<<",\"organicRecoveryCount\":0"<<",\"loopProfileShape\":"<<loopProfileShape;
    expansion<<",\"tallHill\":{\"height\":"<<signature.height<<",\"span\":"<<signature.span<<",\"arc\":"<<signature.section.track.length<<",\"sourceInlet\":"<<signature.authoring.speed<<",\"sourceExit\":"<<signature.exit.speed<<",\"rollingAcceleration\":"<<signature.authoring.rollingAcceleration<<",\"dragCoefficient\":"<<signature.authoring.dragAccelerationCoefficient<<",\"maxSourceReplayNormalResidual\":"<<signature.section.assessment.maxNormalResidualG<<",\"positionResidual\":"<<signaturePositionResidual<<",\"curvatureResidual\":"<<signatureCurvatureResidual<<",\"portCurvatureS\":["<<signaturePortCurvatureS[0]<<','<<signaturePortCurvatureS[1]<<"],\"portUpSS\":["<<signaturePortUpSS[0]<<','<<signaturePortUpSS[1]<<"],\"sourcePortCurvatureS\":["<<signatureSourceCurvatureS[0]<<','<<signatureSourceCurvatureS[1]<<"],\"sourcePortUpSS\":["<<signatureSourceUpSS[0]<<','<<signatureSourceUpSS[1]<<"]}";
    expansion<<",\"sourceEntrySpeedMps\":"<<reversalRequest.entrySpeed<<",\"sourceInvertedHeightMeters\":"<<reversalSource.highestInvertedHeight<<",\"sourceApexHeightMeters\":"<<reversalSource.geometricApexHeight<<",\"sourceExitHeightMeters\":"<<reversalSource.exit.sample.position.z<<",\"sourceExitSpeedMps\":"<<reversalSource.section.samples.back().speed<<",\"sourceExitPitchRadians\":"<<reversalRequest.exitPitch<<",\"pulloutDurationSeconds\":"<<pulloutSource.duration<<",\"valleyTurnLengthMeters\":"<<valleyTurn.length<<",\"netForwardLength\":"<<reversalNetLength<<",\"entryEnergyCorrection\":"<<feedback.reversalEnergyCorrection<<",\"entrySpeedHint\":"<<reversalEntrySpeed<<",\"leadHorizontalLength\":"<<reversalLead<<",\"returnHorizontalLength\":"<<reversalReturnLength;
    expansion<<",\"returnFlowBridge\":{\"start\":"<<at(flowBridgeBegin)<<",\"end\":"<<at(flowBridgeEnd)<<"},\"flowJoins\":[";
    for(size_t i=0;i<flowJoins.size();++i){if(i)expansion<<',';expansion<<"{\"start\":"<<at(flowJoins[i].first)<<",\"end\":"<<at(flowJoins[i].second)<<'}';}
    expansion<<"],\"forceDesignedAirtime\":[";
    for(size_t i=0;i<forceHills.size();++i){if(i)expansion<<',';const auto& hill=forceHills[i];expansion<<"{\"span\":"<<hill.span<<",\"height\":"<<hill.height<<",\"sourceSpeedMps\":"<<hill.authoring.speed<<",\"sourceReplayPassed\":"<<(hill.section.assessment.passed?"true":"false")<<",\"maxSourceNormalResidualG\":"<<hill.section.assessment.maxNormalResidualG<<'}';}
    expansion<<"],\"connectorPacing\":[";
    for(size_t i=0;i<pacingCrests.size();++i){if(i)expansion<<',';const auto& crest=pacingCrests[i];expansion<<"{\"start\":"<<at(crest.begin)<<",\"end\":"<<at(crest.end)<<",\"height\":"<<crest.height<<",\"designSpeed\":"<<crest.speed<<",\"count\":"<<crest.count<<'}';}
    expansion<<"],\"terrainDriveSizing\":[";
    for(size_t index=0;index<terrainDriveSizing.size();++index){if(index)expansion<<',';const auto& sizing=terrainDriveSizing[index];expansion<<"{\"module\":\""<<sizing.module<<"\",\"nominalSpeed\":"<<sizing.nominalSpeed<<",\"targetSpeed\":"<<sizing.targetSpeed<<",\"actualRise\":"<<sizing.actualRise<<",\"rawRise\":"<<sizing.rawRise<<'}';}
    expansion<<"]}}";d.planningDiagnostics+=expansion.str();return d;
    }
    throw std::runtime_error(placementVariant?"No second distinct feasible station placement fits the actual boundary and footprint budget":"No terrain-ranked placement fits the actual station departure/return boundary and 16 m bay-height budget");
}
static void placeSupports(Design& d,Cancel cancel){buildSupportLayout(d,cancel);}
static void improveBanking(Design& d){
    const auto& frames=d.simulation.frames;if(frames.empty())return;
    const size_t count=d.track.spans.size();std::vector<double> authored(count),target(count),along(count),left(count),right(count),speed(count);
    for(size_t i=0;i<count;++i){const auto& k=d.track.knots[i];authored[i]=target[i]=k.bank;along[i]=d.track.spans[i].start;
        speed[i]=replayValueAt(frames,along[i]);if(k.element!=Element::Turn)continue;
        Vec3 required=k.curvature*(speed[i]*speed[i])+Vec3{0,0,gravity},side=cross(k.tangent,k.up);double normal=dot(required,k.up),lateral=dot(required,side);
        target[i]=detail::forceAxisBank(normal,lateral,k.bank);
    }
    double edge=0;
    for(size_t i=0;i<count;++i){if(d.track.knots[i].element!=Element::Turn)edge=along[i]+d.track.spans[i].length;left[i]=std::max(0.,along[i]-edge);}
    edge=d.track.length;for(size_t i=count;i-->0;){if(d.track.knots[i].element!=Element::Turn)edge=along[i];right[i]=std::max(0.,edge-along[i]);}
    // A 1.2-second authoring window on either side allows finite roll response
    // through low-load S crests. This is not an ASTM limit; full-train replay
    // still assesses the resulting canonical frame and rider-offset forces.
    for(size_t i=0;i<count;++i){auto& k=d.track.knots[i];if(k.element!=Element::Turn)continue;double weighted=0,total=0,radius=std::max(1.,speed[i]*1.2);
        size_t first=i;while(first&&along[i]-along[first-1]<radius)--first;
        for(size_t j=first;j<count&&along[j]-along[i]<radius;++j){double u=(along[j]-along[i])/radius,w=std::pow(std::max(0.,1-u*u),4)*d.track.spans[j].length;weighted+=w*target[j];total+=w;}
        double fade=layoutSmooth(std::min(left[i],right[i])/radius);double bank=authored[i]+fade*(weighted/total-authored[i]);
        k.bank=bank*stationBankFactor(d.track.length-along[i],d.request.train);
    }
    d.track.knots.back()=d.track.knots.front();d.track.rebuild();
    for(const auto& operation:d.operations)if(operation.kind==DriveKind::Station&&operation.end<operation.start){
        for(size_t first=0;first<count;){
            if(d.track.knots[first].element!=Element::Turn){++first;continue;}
            size_t last=first+1;while(last<count&&d.track.knots[last].element==Element::Turn)++last;
            const double end=last<count?d.track.spans[last].start:d.track.length;
            if(end>operation.start)detail::fitTurnBankProfile(d.track,first,last,speed);
            first=last;
        }
    }
}

void evaluateTargets(Design& d){
    if(d.supports.empty())d.report.fail("SUPPORT_LAYOUT","Design contains no connected supports");
    auto& m=d.simulation.metrics;const auto& req=d.request;double low=1e9,high=-1e9,station=d.track.knots.front().position.z;
    for(double s=0;s<d.track.length;s+=1){auto p=d.track.sample(s);double gh=p.position.z-req.terrain.height(p.position.x,p.position.y);low=std::min(low,p.position.z);high=std::max(high,p.position.z);m.maxGroundHeight=std::max(m.maxGroundHeight,gh);m.minGroundClearance=std::min(m.minGroundClearance,gh);
        if(p.element==Element::Inversion&&p.up.z<-.5)m.inversionGroundHeight=std::max(m.inversionGroundHeight,gh);
    }m.heightAboveStation=high-station;m.verticalRelief=high-low;
    auto min=[&](const char* code,double value,double goal){if(!std::isfinite(value)||value+1e-6<goal)d.report.fail(code,"Measured result misses requested target",0,value,goal);};
    min("HEIGHT_TARGET",m.maxGroundHeight,req.targets.height);min("INVERSION_TARGET",m.inversionGroundHeight,req.targets.inversionHeight);
    auto dynamic=validateSimulationTargets(d.simulation,req.targets,req.limits);
    d.report.errors.insert(d.report.errors.end(),dynamic.errors.begin(),dynamic.errors.end());
    d.report.warnings.push_back("Force limits are provisional game assumptions; reference calibration is not complete.");
    if(d.simulation.completed&&movingRideSeconds(d)>180)d.report.warnings.push_back("Moving ride exceeds the 180-second pacing goal; physical acceptance is unchanged.");
}
ValidationReport validateRequest(const GenerationRequest& req){
    ValidationReport r;if(!req.terrain.valid()){r.fail("TERRAIN_PROFILE","Terrain profile is outside its supported domain");return r;}const auto& t=req.targets;const auto& l=req.limits;
    for(double value:{t.height,t.speed,t.inversionHeight,t.launchSeconds,l.minVerticalG,l.maxVerticalG,l.maxLateralG,l.maxLongitudinalG,l.maxJerkGps,l.minClearance,req.simulationStep})if(!std::isfinite(value)){r.fail("REQUEST_RANGE","Request contains a nonfinite value");return r;}
    if(req.maxCandidates<1||req.maxCandidates>64||t.height<0||t.height>350||t.speed<1||t.speed>110||t.inversionHeight<0||t.inversionHeight>140||t.launchSeconds<.8||t.launchSeconds>10||l.maxVerticalG<=l.minVerticalG||l.maxLateralG<=0||l.maxLongitudinalG<=0||l.maxJerkGps<=0||l.minClearance<0||req.simulationStep!=1./960||int(req.terrain.kind)<0||int(req.terrain.kind)>2)r.fail("REQUEST_RANGE","Request is outside the prototype's supported domain");
    if(std::isinf(t.referenceExposure)||(std::isfinite(t.referenceExposure)&&t.referenceExposure<=0)||(!t.referenceId.empty()&&!std::isfinite(t.referenceExposure))||(std::isfinite(t.referenceExposure)&&t.referenceId.empty()))r.fail("REFERENCE_CONFIG","Configured reference needs a finite positive exposure and a nonempty ID");
    auto reference=validateReference(t);r.errors.insert(r.errors.end(),reference.errors.begin(),reference.errors.end());
    for(double rate:{l.maxLateralRateGps,l.maxLongitudinalRateGps})if(std::isinf(rate)||(std::isfinite(rate)&&rate<=0))r.fail("AXIS_RATE_CONFIG","Optional component rate gates must be finite positive values, or unset");
    const auto& train=req.train;
    for(double value:{train.carMass,train.spacing,train.seatHeight,train.dragCdA,train.rollingResistance,train.airDensity})if(!std::isfinite(value)){r.fail("TRAIN_CONFIG","Train contains nonfinite settings");return r;}
    if(train.cars<1||train.cars>16||train.spacing<=0||train.spacing>20||train.carMass<=0||train.seatHeight<0||train.seatHeight>3||train.dragCdA<0||train.rollingResistance<0||train.airDensity<0)r.fail("TRAIN_CONFIG","Train is outside the supported model domain");
    return r;
}
Design generate(const GenerationRequest& input,Cancel cancel,std::function<void(int,const std::string&)> progress){
    GenerationRequest req=input;if(req.terrain.isDefaultProfile())req.terrain=Terrain::seeded(req.terrain.kind,req.seed);
    Design last;last.request=req;
    last.report=validateRequest(req);if(!last.report.valid())return last;
    double fastestDeparture=plannedLaunchTime(req.limits.maxLongitudinalG*gravity-.02,req.train);
    if(fastestDeparture>req.targets.launchSeconds){last.report.fail("LAUNCH_FEASIBILITY","Requested departure is below the force-limited flat-station prototype bound",0,fastestDeparture,req.targets.launchSeconds);return last;}
    if(req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure)&&req.targets.referenceExposure*1.1>10*std::max(0.,req.limits.maxVerticalG)){last.report.fail("INTENSITY_FEASIBILITY","Requested exposure exceeds ten seconds at the selected vertical force ceiling",0,req.targets.referenceExposure*1.1,10*std::max(0.,req.limits.maxVerticalG));return last;}
    std::vector<std::string> history;Design bestIntensity,bestPacing;bool haveBestIntensity=false,haveBestPacing=false;double bestDuration=INFINITY;
    auto remember=[&](const Design* d,int index,const char* failure="CANDIDATE_FAILURE"){
        std::ostringstream h;h<<std::setprecision(12)<<"{\"candidate\":"<<index<<",\"constructed\":"<<(d?"true":"false");
        if(d){h<<",\"completed\":"<<(d->simulation.completed?"true":"false")<<",\"exposure10Seconds\":"<<d->simulation.metrics.exposure10Seconds<<",\"launchSeconds\":";if(std::isfinite(d->simulation.metrics.launchTo180))h<<d->simulation.metrics.launchTo180;else h<<"null";if(d->simulation.completed)h<<",\"movingDurationSeconds\":"<<movingRideSeconds(*d);h<<",\"errors\":[";bool comma=false;for(const auto* report:{&d->report,&d->simulation.report})for(const auto& f:report->errors){if(comma)h<<',';comma=true;h<<'"'<<f.code<<'"';}h<<']';}
        else h<<",\"errors\":[\""<<failure<<"\"]";h<<'}';history.push_back(h.str());
    };
    auto withHistory=[&](Design d,const char* selection){
        if(d.planningDiagnostics.empty())d.planningDiagnostics="{\"generationOnly\":true}";
        d.planningDiagnostics.pop_back();
        if(d.simulation.completed){std::ostringstream pacing;pacing<<std::setprecision(12)<<",\"movingDurationSeconds\":"<<movingRideSeconds(d)<<",\"movingDurationGoalMaximumSeconds\":180,\"movingDurationDefinition\":\"Powered departure to train center reaching terminal Station operation start; excludes terminal stopping\"";d.planningDiagnostics+=pacing.str();}
        d.planningDiagnostics+=",\"searchSelection\":\""+std::string(selection)+"\",\"candidateHistory\":[";
        for(size_t i=0;i<history.size();++i){if(i)d.planningDiagnostics+=',';d.planningDiagnostics+=history[i];}d.planningDiagnostics+="]}";return d;
    };
    for(int i=0;i<req.maxCandidates;++i){
        if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}
        if(progress)progress(i,"Solving terrain corridor and circuit");
        try{
            AuthoringFeedback feedback;CandidatePorts ports;
            std::unique_ptr<const FvdTallHillResult> preparedSignature;
            Design d=candidate(req,i,cancel,feedback,ports,preparedSignature);d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);
            if(d.simulation.cancelled){d.report.fail("CANCELLED","Generation cancelled");return d;}
            bool feedbackConverged=false;double energyResidual=INFINITY;int energyIterations=0;std::ostringstream energyHistory;energyHistory<<std::setprecision(12);
            // Hold the selected route family/site while solving actual entry
            // energy. Re-ranking after every correction could oscillate
            // between different terrain itineraries instead of converging.
            feedback.planShape=ports.planShape;feedback.planPlacement=ports.planPlacement;
            const int provisionalShape=ports.planShape,provisionalPlacement=ports.planPlacement;
            bool routeReselected=false;int routeReselectionIteration=-1;
            Design convergedProvisional;CandidatePorts convergedPorts;double provisionalResidual=INFINITY;
            bool haveConvergedProvisional=false,retainedProvisional=false;int attemptedShape=-1,attemptedPlacement=-1;
            double failedAlternateResidual=INFINITY;
            for(int correction=0;correction<=8;++correction){
                if(!d.simulation.completed){
                    // An optional replacement must not bootstrap past an
                    // already-converged route; preserve its actual failure.
                    if(haveConvergedProvisional){
                        energyResidual=INFINITY;
                        energyHistory<<",{\"iteration\":"<<correction<<",\"completed\":false,\"maximumResidualMps\":null}";break;
                    }
                    // A reached Immelmann entry can diagnose an energy-starved
                    // climb even when its exit and later hills were never ridden.
                    // Bootstrap only genuine stalls there, within the same total
                    // rebuild budget; missing telemetry is never extrapolated.
                    energyResidual=INFINITY;
                    const auto& frames=d.simulation.frames;
                    auto reached=[&](double at){return !frames.empty()&&at>=frames.front().distance&&at<=frames.back().distance;};
                    const bool stalled=!d.simulation.report.errors.empty()&&std::all_of(d.simulation.report.errors.begin(),d.simulation.report.errors.end(),[](const Finding& f){return f.code=="STALL";});
                    if(!stalled||!reached(ports.reversalEntry)||!reached(ports.reversalBrakeStart)||
                       !reached(ports.relaunchEnd)||reached(ports.reversalExit))break;
                    const double brakeStartSpeed=replayValueAt(frames,ports.reversalBrakeStart);
                    const auto passive=detail::estimatePassiveTransfer(d.track,req.train,ports.reversalBrakeStart,ports.reversalEntry,brakeStartSpeed,cancel);
                    if(!passive.reached)break;
                    const double deficit=ports.reversalEntrySpeedHint*ports.reversalEntrySpeedHint-passive.speed*passive.speed;
                    if(!(deficit>0))break;
                    if(correction)energyHistory<<',';
                    energyHistory<<"{\"iteration\":"<<correction<<",\"bootstrap\":\"reached-immelmann-entry\",\"maximumResidualMps\":null,\"reachedDistanceMeters\":"<<frames.back().distance<<",\"reversalEntryMps\":"<<replayValueAt(frames,ports.reversalEntry)<<",\"upstreamEnergyDeficitM2ps2\":"<<deficit<<'}';
                    if(correction==8)break;
                    for(size_t h=0;h<feedback.airtimeSpeed.size();++h)if(reached(ports.airtimeEntry[h]))
                        feedback.airtimeSpeed[h]+=.75*(replayValueAt(frames,ports.airtimeEntry[h])-feedback.airtimeSpeed[h]);
                    const double drag=req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass);
                    feedback.relaunchEnergyCorrection+=.75*deficit*std::exp(drag*(ports.reversalEntry-ports.relaunchEnd));
                    d=candidate(req,i,cancel,feedback,ports,preparedSignature);d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);++energyIterations;
                    if(d.simulation.cancelled){d.report.fail("CANCELLED","Generation cancelled");return d;}
                    continue;
                }
                std::array<double,4> measured{},measuredTurn{};
                const double signatureEntrySpeed=replayValueAt(d.simulation.frames,ports.signatureEntry);
                const double signatureExitSpeed=replayValueAt(d.simulation.frames,ports.signatureExit);
                energyResidual=std::max(std::abs(signatureEntrySpeed-ports.signatureEntrySpeedHint),
                    std::abs(signatureExitSpeed-ports.signatureExitSpeedHint));
                // Use the maximum speed while any car occupies the turn, not
                // an average weakened by its crest. This bounds the source
                // lateral demand; final rider loads remain independently gated.
                const double halfTrain=(req.train.cars-1)*req.train.spacing*.5;
                for(size_t side=0;side<measuredTurn.size();++side)if(ports.ordinaryTurn[side]){
                    for(const auto& frame:d.simulation.frames)if(frame.distance>=ports.turnBegin[side]-halfTrain&&frame.distance<=ports.turnEnd[side]+halfTrain)
                        measuredTurn[side]=std::max(measuredTurn[side],frame.speed);
                    if(!(measuredTurn[side]>0))throw std::runtime_error("Ordinary turn has no measured speed context");
                    energyResidual=std::max(energyResidual,std::abs(measuredTurn[side]-feedback.turnSpeed[side]));
                }
                for(size_t h=0;h<measured.size();++h){measured[h]=replayValueAt(d.simulation.frames,ports.airtimeEntry[h]);energyResidual=std::max(energyResidual,std::abs(measured[h]-feedback.airtimeSpeed[h]));}
                const double exitSpeed=replayValueAt(d.simulation.frames,ports.reversalExit),loopSpeed=replayValueAt(d.simulation.frames,ports.loopApex);
                energyResidual=std::max({energyResidual,std::abs(exitSpeed-ports.reversalSpeedHint),std::abs(loopSpeed-ports.loopSpeedHint)});
                const double brakeStartSpeed=replayValueAt(d.simulation.frames,ports.reversalBrakeStart);
                const auto passive=detail::estimatePassiveTransfer(d.track,req.train,ports.reversalBrakeStart,ports.reversalEntry,brakeStartSpeed,cancel);
                const double measuredEntry=replayValueAt(d.simulation.frames,ports.reversalEntry);
                const double exitEnergyError=ports.reversalSpeedHint*ports.reversalSpeedHint-exitSpeed*exitSpeed;
                if(!passive.reached)throw std::runtime_error("Completed brake transfer has no passive energy solution");
                const double drag=req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass);
                const double entryEnergyError=exitEnergyError*std::exp(drag*(ports.reversalExit-ports.reversalEntry));
                const double neededEntryEnergy=measuredEntry*measuredEntry+entryEnergyError;
                const double upstreamDeficit=std::max(0.,neededEntryEnergy-passive.speed*passive.speed);
                const double loopApproachSpeed=replayValueAt(d.simulation.frames,ports.loopBrakeStart);
                const auto loopPassive=detail::estimatePassiveTransfer(d.track,req.train,ports.loopBrakeStart,ports.loopApex,loopApproachSpeed,cancel);
                if(!loopPassive.reached)throw std::runtime_error("Completed loop has no passive energy solution");
                const double loopDeficit=std::max(0.,ports.loopSpeedHint*ports.loopSpeedHint-loopPassive.speed*loopPassive.speed);
                const bool upstreamStarved=upstreamDeficit>0;
                if(correction)energyHistory<<',';
                energyHistory<<"{\"iteration\":"<<correction<<",\"maximumResidualMps\":"<<energyResidual<<",\"signatureEntryMps\":"<<signatureEntrySpeed<<",\"signatureExitMps\":"<<signatureExitSpeed<<",\"brakeStartSpeedMps\":"<<brakeStartSpeed<<",\"brakeTargetMps\":"<<ports.reversalEntrySpeedHint<<",\"reversalEntryMps\":"<<replayValueAt(d.simulation.frames,ports.reversalEntry)<<",\"reversalExitMps\":"<<exitSpeed<<",\"loopApexMps\":"<<loopSpeed<<",\"loopPassiveApexMps\":"<<loopPassive.speed<<",\"loopSupplyTargetMps\":"<<ports.loopSupplyTarget<<",\"loopSupplyEndMeters\":"<<ports.loopSupplyEnd<<",\"loopUpstreamDeficitM2ps2\":"<<loopDeficit<<",\"upstreamStarved\":"<<(upstreamStarved?"true":"false")<<'}';
                // A provisional route supplies actual source energy first. Rank once
                // with those converged dimensions, then keep the final route
                // fixed. Both stages share this same eight-rebuild budget.
                const bool reselectRoute=energyResidual<=.5&&!routeReselected;
                if(energyResidual<=.5&&(routeReselected||correction==8)){feedbackConverged=true;break;}
                if(correction==8)break;
                if(reselectRoute){
                    // Re-ranking is optional. Keep the measured converged route
                    // until its replacement earns the same energy tolerance.
                    convergedProvisional=std::move(d);convergedPorts=ports;provisionalResidual=energyResidual;haveConvergedProvisional=true;
                }
                for(size_t h=0;h<measured.size();++h)feedback.airtimeSpeed[h]+=.75*(measured[h]-feedback.airtimeSpeed[h]);
                for(size_t side=0;side<measuredTurn.size();++side)if(ports.ordinaryTurn[side])feedback.turnSpeed[side]+=.75*(measuredTurn[side]-feedback.turnSpeed[side]);
                // Correct required trim energy and available launch energy
                // independently. The passive corridor attenuates added v^2
                // by exp(-rho*CdA*distance/mass); account for that work loss.
                feedback.reversalEnergyCorrection+=.75*entryEnergyError;
                feedback.relaunchEnergyCorrection+=.75*upstreamDeficit*std::exp(drag*(ports.reversalEntry-ports.relaunchEnd));
                feedback.loopApproachSpeed=loopApproachSpeed;
                feedback.loopEnergyCorrection+=.75*(ports.loopSpeedHint*ports.loopSpeedHint-loopSpeed*loopSpeed)*std::exp(drag*(ports.loopApex-ports.loopEntry));
                feedback.loopSupplyEnergyCorrection+=.75*loopDeficit*std::exp(drag*(ports.loopApex-ports.loopSupplyEnd));
                if(reselectRoute){feedback.planShape=-1;feedback.planPlacement=-1;routeReselected=true;routeReselectionIteration=correction+1;}
                ++energyIterations;
                try{d=candidate(req,i,cancel,feedback,ports,preparedSignature);}
                catch(const std::runtime_error& error){
                    if(std::string(error.what())=="CANCELLED"||(cancel&&cancel())){
                        Design cancelled;cancelled.request=req;cancelled.candidate=i;cancelled.simulation.cancelled=true;
                        cancelled.report.fail("CANCELLED","Generation cancelled");return cancelled;
                    }
                    if(!haveConvergedProvisional)throw;
                    energyResidual=INFINITY;
                    energyHistory<<",{\"iteration\":"<<correction+1<<",\"constructionError\":\"";
                    for(unsigned char c:std::string(error.what())){
                        if(c=='"'||c=='\\')energyHistory<<'\\'<<c;
                        else if(c<32)energyHistory<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
                        else energyHistory<<c;
                    }
                    energyHistory<<"\",\"maximumResidualMps\":null}";break;
                }
                if(routeReselected){attemptedShape=ports.planShape;attemptedPlacement=ports.planPlacement;}
                d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);
                if(reselectRoute){feedback.planShape=ports.planShape;feedback.planPlacement=ports.planPlacement;}
                if(d.simulation.cancelled){d.report.fail("CANCELLED","Generation cancelled");return d;}
            }
            if(cancel&&cancel()){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            if(!feedbackConverged&&haveConvergedProvisional){
                failedAlternateResidual=energyResidual;d=std::move(convergedProvisional);ports=convergedPorts;
                energyResidual=provisionalResidual;feedbackConverged=true;retainedProvisional=true;
            }
            if(!d.planningDiagnostics.empty()){
                d.planningDiagnostics.pop_back();std::ostringstream energy;energy<<std::setprecision(12)<<",\"authoringEnergy\":{\"corrections\":"<<energyIterations<<",\"converged\":"<<(feedbackConverged?"true":"false")<<",\"maximumSpeedResidualMps\":";
                if(std::isfinite(energyResidual))energy<<energyResidual;else energy<<"null";
                energy<<",\"routeSelection\":{\"reselected\":"<<(routeReselected?"true":"false")<<",\"iteration\":"<<routeReselectionIteration<<",\"provisionalShape\":"<<provisionalShape<<",\"provisionalPlacement\":"<<provisionalPlacement<<",\"finalShape\":"<<ports.planShape<<",\"finalPlacement\":"<<ports.planPlacement<<",\"attemptedShape\":"<<attemptedShape<<",\"attemptedPlacement\":"<<attemptedPlacement<<",\"retainedConvergedProvisional\":"<<(retainedProvisional?"true":"false")<<",\"failedAlternateResidualMps\":";
                if(retainedProvisional&&std::isfinite(failedAlternateResidual))energy<<failedAlternateResidual;else energy<<"null";
                energy<<"},\"toleranceMps\":0.5,\"loopApexTargetMps\":"<<ports.loopSpeedHint<<",\"reversalExitTargetMps\":"<<ports.reversalSpeedHint<<",\"iterations\":["<<energyHistory.str()<<"]}}";d.planningDiagnostics+=energy.str();
            }
            if(d.simulation.completed){improveBanking(d);d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);}
            d.station=buildStation(d.track,req.terrain,req.train,cancel);
            placeSupports(d,cancel);
            d.inversionDimensions=measureInversionDimensions(d.track,cancel);
            if(progress)progress(i,"Checking measured targets and clearance");
            d.report=validateGeometry(d.track,req.terrain,req.limits,req.train,d.supports,cancel);
            auto structures=validateDesignStructures(d,cancel);d.report.errors.insert(d.report.errors.end(),structures.errors.begin(),structures.errors.end());
            evaluateTargets(d);
            if(d.simulation.completed){
                const double entry=replayValueAt(d.simulation.frames,ports.signatureEntry),exit=replayValueAt(d.simulation.frames,ports.signatureExit);
                const double entryResidual=std::abs(entry-ports.signatureEntrySpeedHint),exitResidual=std::abs(exit-ports.signatureExitSpeedHint);
                if(std::max(entryResidual,exitResidual)>.5)d.report.fail("AUTHORING_SIGNATURE",
                    "Actual finite-train signature inlet/exit differs from its force-source energy intent",ports.signatureEntry,std::max(entryResidual,exitResidual),.5);
                d.planningDiagnostics.pop_back();std::ostringstream intent;intent<<std::setprecision(12)
                    <<",\"signatureIntent\":{\"sourceInletMps\":"<<ports.signatureEntrySpeedHint<<",\"actualInletMps\":"<<entry
                    <<",\"sourceExitMps\":"<<ports.signatureExitSpeedHint<<",\"actualExitMps\":"<<exit
                    <<",\"inletResidualMps\":"<<entryResidual<<",\"exitResidualMps\":"<<exitResidual
                    <<",\"toleranceMps\":0.5,\"minimumSourceTraversalMps\":25,\"preparedSourceReused\":true}}";
                d.planningDiagnostics+=intent.str();
            }
            if(d.simulation.completed&&!feedbackConverged)d.report.fail("AUTHORING_ENERGY","Joined ride did not converge to its element entry/apex energy intent",0,energyResidual,.5);
            if(d.report.valid()&&d.simulation.completed&&d.simulation.report.valid()&&!d.simulation.cancelled){
                if(progress)progress(i,"Checking independent half-step simulation");
                verifyConvergence(d,cancel);
                if(d.simulation.cancelled)return d;
            }
            remember(&d,i);if(d.accepted()){
                double duration=movingRideSeconds(d);
                if(duration<=180)return withHistory(std::move(d),"accepted-within-moving-duration-goal");
                if(duration<bestDuration){bestDuration=duration;bestPacing=std::move(d);haveBestPacing=true;}
                continue;
            }
            bool intensityOnly=d.simulation.completed&&d.simulation.report.valid()&&d.report.errors.size()==1&&d.report.errors.front().code=="INTENSITY_TARGET";
            if(intensityOnly&&(!haveBestIntensity||d.simulation.metrics.exposure10Seconds>bestIntensity.simulation.metrics.exposure10Seconds)){bestIntensity=d;haveBestIntensity=true;}last=std::move(d);
            // Missing reference data cannot be repaired by searching other seeds.
            bool onlyMissing=last.simulation.completed&&last.simulation.report.valid()&&last.report.errors.size()==1&&last.report.errors.front().code=="REFERENCE_UNAVAILABLE";
            if(onlyMissing)return withHistory(std::move(last),"reference-unavailable");
        }catch(const SourceFamilyUnsupported& e){
            remember(nullptr,i,"SOURCE_FAMILY");last.report.fail("SOURCE_FAMILY",e.what());
            return withHistory(std::move(last),"unsupported-source-family");
        }catch(const std::exception& e){if(std::string(e.what())=="CANCELLED"||(cancel&&cancel())){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}remember(nullptr,i);last.report.fail("CANDIDATE_FAILURE",e.what());if(progress)progress(i,std::string("Candidate rejected: ")+e.what());}
    }if(haveBestPacing)return withHistory(std::move(bestPacing),"best-accepted-moving-duration-shortfall");if(haveBestIntensity)return withHistory(std::move(bestIntensity),"best-physically-valid-intensity-shortfall");return withHistory(std::move(last),"last-constructed-rejection");
}
}
