#include "coaster/coaster.hpp"
#include "coaster/fvd.hpp"
#include "baseline_jets.hpp"
#include "flow_bridge.hpp"
#include "connector_profile.hpp"
#include "simulation_internal.hpp"
#include <future>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace coaster {
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
struct TurnShape {double angle{},ramp{},length{};std::vector<Vec3> points;};
static TurnShape makeTurn(double angle,double radius,double ramp){
    const double sign=angle<0?-1.:1.;double magnitude=std::abs(angle);
    TurnShape t;t.angle=angle;t.ramp=std::min(ramp,.85*radius*magnitude);t.length=radius*magnitude+t.ramp;
    int n=int(std::ceil(t.length/1.8));double ds=t.length/n;Vec3 cursor{};t.points.push_back(cursor);
    auto integral=[](double u){return u*u*u*u*(2.5+u*(-3+u));};
    auto theta=[&](double s){if(s<t.ramp)return t.ramp/radius*integral(s/t.ramp);if(s>t.length-t.ramp)return magnitude-t.ramp/radius*integral((t.length-s)/t.ramp);return (s-t.ramp*.5)/radius;};
    for(int i=1;i<=n;++i){double h=sign*theta((i-.5)*ds);cursor.x+=ds*std::cos(h);cursor.y+=ds*std::sin(h);t.points.push_back(cursor);}return t;
}
struct RoutePlan {
    bool stationChecked{},stationFeasible{},stationSelectionEligible{};double stationBayMinimum{},stationBayMaximum{},stationRequiredDatum{};
    int shape{},placement{};double heading{},length{},score{};
    Vec3 origin;std::vector<double> angles,lengths;std::vector<TurnShape> turns;
};
static double stationBankFactor(double remaining,const TrainConfig& train){double upright=std::max(40.,(train.cars-1)*train.spacing*.5+18);return smooth((remaining-upright)/100);}
static double layoutWarp(double u,double shape){return u+shape*std::sin(2*pi*u)/(2*pi);}
static double layoutSmooth(double u){u=std::clamp(u,0.,1.);return u*u*u*u*(35+u*(-84+u*(70-20*u)));}
static Vec3 inFrame(Vec3 p,double h){return {p.x*std::cos(h)-p.y*std::sin(h),p.x*std::sin(h)+p.y*std::cos(h),p.z};}
static std::vector<RoutePlan> planRoutes(int sides,const std::vector<double>& minimum,double radius,double ramp,int holdCorner,bool reversingPair,Random& rng,Cancel cancel){
    std::vector<RoutePlan> feasible;
    double startingHeading=rng.range(-pi,pi);Vec3 stationAnchor{rng.range(-200,200),rng.range(-160,160),0};
    for(int shape=0;shape<48;++shape){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        RoutePlan p;p.shape=shape;p.angles.resize(sides);p.lengths=minimum;
        // A signed four-leg folded grammar. Paired unequal angles keep the
        // endpoint heading exact while the diagonals cross inside the layout.
        const double alpha=rng.range(.71,.79)*pi,beta=rng.range(.71,.79)*pi;
        p.angles={alpha,-beta,-alpha,beta};
        std::vector<Vec3> directions;Vec3 sum{};double h=0;
        for(int side=0;side<sides;++side){directions.push_back({std::cos(h),std::sin(h),0});
            const double extra=side==holdCorner?std::copysign(2*pi,p.angles[side]):0;
            p.turns.push_back(makeTurn(p.angles[side]+extra,radius,ramp));sum=sum+inFrame(p.turns.back().points.back(),h);h+=p.angles[side];}
        p.lengths[0]+=rng.range(25,100);
        Vec3 x=directions[2],y=directions[3];double determinant=cross(x,y).z;
        if(std::abs(determinant)<.2)continue;
        auto closeFor=[&](double diagonal){Vec3 rhs=(sum+directions[0]*p.lengths[0]+directions[1]*diagonal)*(-1);return std::pair{cross(rhs,y).z/determinant,cross(x,rhs).z/determinant};};
        auto intercept=closeFor(0),unit=closeFor(1);
        double low=minimum[1],high=2800;
        auto constrain=[&](double interceptValue,double slope,double minimumValue){if(std::abs(slope)<1e-10){if(interceptValue<minimumValue||interceptValue>2800)high=-1;return;}double a=(minimumValue-interceptValue)/slope,b=(2800-interceptValue)/slope;low=std::max(low,std::min(a,b));high=std::min(high,std::max(a,b));};
        constrain(intercept.first,unit.first-intercept.first,minimum[2]);
        constrain(intercept.second,unit.second-intercept.second,minimum[3]);
        if(high<low)continue;p.lengths[1]=low+rng.range(0,std::min(90.,high-low));
        auto solved=closeFor(p.lengths[1]);p.lengths[2]=solved.first;p.lengths[3]=solved.second;
        bool valid=true;p.length=0;
        for(int side=0;side<sides;++side){if(p.lengths[side]<minimum[side]||p.lengths[side]>2800)valid=false;p.length+=p.lengths[side]+p.turns[side].length;}
        // Major hills/inversion add approximately 400--650 m of 3-D rail to
        // this horizontal budget. Do not build a huge fallback corridor.
        // The exposure hold is an extra closed helix lap, not another long
        // perimeter corridor. Keep its real track length in all reports and
        // resource/clearance checks, but do not charge the same footprint twice.
        const double repeatedHelixLength=reversingPair&&holdCorner>=0?2*pi*radius:0;
        if(!valid||p.length<4700||p.length-repeatedHelixLength>9800)continue;
        Vec3 cursor{};h=0;
        for(int side=0;side<sides;++side){
            Vec3 forward{std::cos(h),std::sin(h),0};
            cursor=cursor+forward*p.lengths[side];
            cursor=cursor+inFrame(p.turns[side].points.back(),h);h+=p.angles[side];
        }
        if(norm(cursor)>1e-6)continue;
        for(int placement=0;placement<36;++placement){
            auto q=p;q.placement=placement;
            q.heading=startingHeading+(placement/3)*pi/6;
            q.origin=stationAnchor+inFrame({0,double(placement%3-1)*180,0},q.heading);
            q.score=q.length/65;
            feasible.push_back(std::move(q));
        }
    }
    std::stable_sort(feasible.begin(),feasible.end(),[](const RoutePlan& a,const RoutePlan& b){return a.score<b.score;});return feasible;
}
struct AuthoringFeedback {
    std::array<double,4> airtimeSpeed{65,65,65,65};
    double reversalEnergyCorrection{};
};
struct CandidatePorts {
    std::array<double,4> airtimeEntry{};
    double reversalExit{},reversalSpeedHint{};
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
static Design candidate(const GenerationRequest& req,int attempt,Cancel cancel,const AuthoringFeedback& feedback,CandidatePorts& ports){
    const int geometryAttempt=attempt/2,placementVariant=attempt%2;
    Design d;d.request=req;d.candidate=attempt;Random rng{req.seed};rng.next();int sides=4;bool helix=false;
    rng.next();int order=0;
    bool intensityDesign=req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure);double exposureGoal=intensityDesign?req.targets.referenceExposure*1.1:0;
    double designNormalG=rng.range(3.2,3.6);if(intensityDesign)designNormalG=std::min(req.limits.maxVerticalG-.5,std::max(designNormalG,exposureGoal/10+.35+geometryAttempt*.10));
    designNormalG=std::max(1.2,designNormalG);double radius=65*65/(gravity*std::sqrt(designNormalG*designNormalG-1))+(intensityDesign?0:geometryAttempt*12),ramp=160+geometryAttempt*15;
    double elevation=req.targets.height+rng.range(8,30),loopHeight=req.targets.inversionHeight+rng.range(8,18);
    double hillWidth=rng.range(1020,1120)+geometryAttempt*20,loopDrift=rng.range(330,380)+geometryAttempt*10;
    double topSpeed=std::max(req.targets.speed+rng.range(7,11),std::sqrt(2*gravity*elevation+45*45)),launchAcceleration=sizedLaunchAcceleration(req,rng.range(38,43));
    int hillSide=0,loopSide=1;
    std::vector<int> module(sides,2);module[hillSide]=0;module[loopSide]=1;
    if(sides==6){module[2]=3;module[5]=3;}
    rng.range(17,23); // Retain the independent route random sequence.
    // A separate stream adds variation without consuming the legacy route
    // random sequence. The paired reversal has an explicit net displacement;
    // only six-corridor plans reserve its inward three-lane footprint.
    Random detail{req.seed^0x8d2f41b79a5c630eull};
    const bool reversingPair=true;
    double reversalHeight=std::max(84.,req.targets.inversionHeight+detail.range(7,14));
    detail.range(82,96);detail.range(190,240);
    const double reversalLane=detail.range(95,125);detail.range(155,190);
    detail.range(-.12,.08);detail.range(-.3,.3);detail.range(23,26);
    const int reversalHand=(detail.next()&1)?1:-1;
    const double reversalLengthPreference=detail.range(930,1010)/970;
    const double scale=std::sqrt(reversalHeight/88.);
    FvdImmelmannRequest inversion;
    inversion.entrySpeed=std::sqrt(53*53*scale*scale+feedback.reversalEnergyCorrection);
    inversion.height=95*scale*scale;inversion.exitHeight=10*scale*scale;
    inversion.rampSeconds=1.2*scale;inversion.hand=reversalHand;
    inversion.rollingAcceleration=gravity*req.train.rollingResistance;
    inversion.dragAccelerationCoefficient=.5*req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass);
    const auto immelmann=designFvdImmelmann(inversion,cancel);
    if(!immelmann.section.report.valid()||!immelmann.section.assessment.passed)
        throw std::runtime_error(immelmann.section.cancelled?"CANCELLED":"Immelmann source failed its physical replay");
    reversalHeight=inversion.height;
    const double reversalEntrySpeed=inversion.entrySpeed;
    const double lateralAcceleration=gravity*std::sqrt(designNormalG*designNormalG-1);
    auto transferLength=[&](double offset,double speed){return speed*std::sqrt(7.514*std::abs(offset)/lateralAcceleration)*reversalLengthPreference;};
    const double brakeDistance=std::max(0.,(65*65-reversalEntrySpeed*reversalEntrySpeed)/(2*3.5))+65*.5+(req.train.cars-1)*req.train.spacing;
    const double reversalLead=std::max(transferLength(reversalLane,65),brakeDistance);
    // The complete retained element exits low. Turn inward on that level
    // instead of attaching the old elevated dive loop to an incompatible port.
    const double exitHeading=std::atan2(immelmann.exit.forward.y,immelmann.exit.forward.x);
    const double returnAngle=exitHeading>0?-exitHeading:-2*pi-exitHeading;
    const double returnRadius=immelmann.exit.speed*immelmann.exit.speed/lateralAcceleration;
    const auto reversalTurn=makeTurn(returnAngle,returnRadius,ramp);
    const Vec3 reversalEnd=Vec3{reversalLead,reversalLane,0}+immelmann.exit.position+inFrame(reversalTurn.points.back(),exitHeading);
    const double reversalReturnLength=transferLength(reversalEnd.y,immelmann.exit.speed);
    const double reversalNetLength=reversalEnd.x+reversalReturnLength;
    const double hillProfileShape=detail.range(-.05,.05),loopProfileShape=detail.range(-.04,.04);
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
    if(reversingPair)module[2]=4;
    std::vector<double> minimum(sides);
    for(int side=0;side<sides;++side){
        double initial=side==0?200:0;
        if(module[side]==0)minimum[side]=initial+(side?500:0)+hillWidth+80+60;
        else if(module[side]==1)minimum[side]=initial+(side?380:300)+loopDrift+220+180+forceHills[0].span+60;
        else if(module[side]==2)minimum[side]=100+forceReturnSpan+70;
        else if(module[side]==4)minimum[side]=reversalNetLength+70;
        else minimum[side]=130;
    }
    int holdCorner=intensityDesign&&exposureGoal>18&&sides>2?(hillSide+1)%(sides-1):-1;
    auto plans=planRoutes(sides,minimum,radius,ramp,holdCorner,reversingPair,rng,cancel);if(plans.empty())throw std::runtime_error("No route fits exact closure and footprint budget");
    // Visit ranked plans in order. Exact raw departure/return boundary
    // and bay must fit the local budget before committing to a station site.
    // The candidate list is bounded by 48 shapes x 36 placements; rejected
    // station sites do not spend a geometry/physics attempt or alter its limits.
    int firstFeasiblePlacement=-1;
    for(auto& plan:plans){
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    double heading=plan.heading;Vec3 origin=plan.origin,cursor=origin;
    const std::string orderName=order==0?"H-I-A":order==1?"H-A-I":sides==2?"I-A-H":"I-H-A";
    d.topology="folded-bow-tie/H-I-IM-A4";
    int organicRecoveryCount=0;
    size_t forceHoldEnd=0;std::vector<AuthoredPoint> raw;struct Pending{size_t begin,end;DriveKind kind;double speed;};std::vector<Pending> pending;
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
    std::array<size_t,4> forceEntryIndices{};size_t reversalExitIndex=0;
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
        if(side==0){record(line(20,Element::Station),side,"station");record(drive(180,DriveKind::Launch,module[side]==1?62:topSpeed),side,"departure-launch");used=200;}
        if(module[side]==0){
            if(side){record(drive(500,DriveKind::Boost,topSpeed),side,"record-hill-relaunch");used+=500;}
            record(piece(hillWidth,[=](double u){return Vec3{hillWidth*u,0,elevation*std::pow(std::sin(pi*layoutWarp(u,hillProfileShape)),4)};},Element::Hill),side,"record-hill");used+=hillWidth;
            record(drive(80,DriveKind::Boost,65),side,"recovery");used+=80;
        }else if(module[side]==1){
            double brakeLength=side?380:300;record(drive(brakeLength,DriveKind::Brake,46+geometryAttempt*.3),side,"inversion-entry-brake");used+=brakeLength;
            double amplitude=(loopDrift+std::sqrt(4*pi*pi*loopHeight*25))/(2*pi),offset=36;
            record(piece(900,[=](double u){return Vec3{loopDrift*u+amplitude*std::sin(2*pi*u),offset*smooth(u),loopHeight*std::pow(std::sin(pi*layoutWarp(u,loopProfileShape)),4)};},Element::Inversion),side,"record-inversion");used+=loopDrift;
            record(piece(220,[=](double u){return Vec3{220*u,-offset*smooth(u),0};},Element::Return),side,"inversion-recovery");used+=220;
            record(drive(180,DriveKind::Boost,65),side,"airtime-entry-boost");used+=180;
            used+=forceHill(0,side);
        }else if(module[side]==4){
            // Lead inward before reversing so the returning ground-level lane
            // is laterally separated from both the lead and the elevated lane.
            // The speed here is an energy-based estimate only: the same real
            // finite-train brake and complete-circuit checks decide eligibility.
            double entrySpeed=reversalEntrySpeed;
            auto lead=piece(reversalLead+reversalLane,[=](double u){return Vec3{reversalLead*u,reversalLane*layoutSmooth(u),0};},Element::Turn);
            pending.push_back({lead.first,lead.second,DriveKind::Brake,entrySpeed});record(lead,side,"immelmann-inward-entry-brake");
            const Vec3 base=cursor;const double incomingHeading=heading;
            const size_t inversionStart=raw.size()-1;
            const auto& source=immelmann.section.track;
            const int samples=int(std::ceil(source.length/1.2));
            for(int i=0;i<=samples;++i){const auto q=source.sample(source.length*i/samples);
                append(base+inFrame(q.position,heading),0,Element::Inversion,inFrame(q.up,heading));}
            cursor=base+inFrame(immelmann.exit.position,heading);heading+=exitHeading;
            record({inversionStart,raw.size()-1},side,"high-immelmann");reversalExitIndex=raw.size()-1;
            const Vec3 turnBase=cursor;const size_t turnStart=raw.size()-1;
            for(size_t i=1;i<reversalTurn.points.size();++i){
                const double along=reversalTurn.length*i/(reversalTurn.points.size()-1);
                const double curvature=smooth(std::min(along,reversalTurn.length-along)/reversalTurn.ramp)/returnRadius;
                cursor=turnBase+inFrame(reversalTurn.points[i],heading);
                append(cursor,std::atan(immelmann.exit.speed*immelmann.exit.speed*curvature/gravity),Element::Turn);
            }
            heading=incomingHeading;record({turnStart,raw.size()-1},side,"immelmann-level-return");
            record(piece(reversalReturnLength+std::abs(reversalEnd.y),[=](double u){return Vec3{
                reversalReturnLength*u,-reversalEnd.y*layoutSmooth(u),-inversion.exitHeight*layoutSmooth(u)};
                },Element::Turn),side,"interior-low-return");
            used+=reversalNetLength;
        }else if(module[side]==2){
            record(drive(100,DriveKind::Boost,65),side,"airtime-entry-boost");used+=100;
            for(int h=1;h<=3;++h)used+=forceHill(h,side);
        }
        if(used>straight-40)throw std::runtime_error("Module plan exceeded reserved corridor");
        const double recoveryLength=straight-used;
        if(recoveryLength>340&&module[side]!=4){
            const double inward=std::min(90.,1.3*gravity*recoveryLength*recoveryLength/(40*65*65));
            auto recovery=piece(recoveryLength+2*inward,[=](double u){return Vec3{recoveryLength*u,inward*std::pow(std::sin(pi*u),4),0};},Element::Turn);
            pending.push_back({recovery.first,recovery.second,DriveKind::Boost,65});record(recovery,side,"organic-inward-recovery");++organicRecoveryCount;
        }else record(drive(recoveryLength,DriveKind::Boost,65),side,"corridor-recovery");
        const auto& turn=plan.turns[side];Vec3 base=cursor;size_t turnStart=raw.size()-1;double dz=helix?(side==0?55:-55):(side==holdCorner?55:0);
        const double bankedRise=std::min(26.,.65*gravity*turn.length*turn.length/(4*pi*pi*65*65))*turnRiseScale[side];
        for(size_t i=1;i<turn.points.size();++i){double along=turn.length*i/(turn.points.size()-1),curvature=smooth(std::min(along,turn.length-along)/turn.ramp)/radius;cursor=base+inFrame(turn.points[i],heading);cursor.z=base.z+dz*smooth(along/turn.length)+bankedRise*std::pow(std::sin(pi*layoutWarp(along/turn.length,turnRiseShape[side])),4);append(cursor,-std::copysign(std::atan(65*65*curvature/gravity),turn.angle),Element::Turn);}
        modules.push_back({turnStart,raw.size()-1,side,helix||side==holdCorner?"sustained-helix":"banked-camelback-turn"});if(side==holdCorner)forceHoldEnd=raw.size()-1;
        if(side+1<sides)pending.push_back({turnStart,raw.size()-1,DriveKind::Boost,65});heading+=plan.angles[side];
    }
    if(std::hypot(cursor.x-origin.x,cursor.y-origin.y)>.001)throw std::runtime_error("Solved route failed canonical XY closure");
    size_t unique=raw.size()-1;std::vector<double> distance(raw.size());for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    if(forceHoldEnd){
        // Keep the reversing pair and its flyover on the same raised datum.
        // Settle the extra helix elevation on the final return before joins
        // and crossing lifts are planned; a later warp would erase clearance.
        auto returning=std::find_if(modules.begin(),modules.end(),[&](const ModuleRun& run){return run.corridor==sides-1;});
        const double begin=distance[returning->begin],remaining=distance.back()-begin;
        for(size_t i=returning->begin;i<raw.size();++i)raw[i].position.z-=55*layoutSmooth((distance[i]-begin)/remaining);
        for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    }
    // Full connecting crests occupy existing transit envelopes. Dimensions
    // follow the gravity energy budget and v^2 curvature, not random telemetry.
    // All existing operation indices survive and finite-train replay remains
    // authoritative after ground placement and force-aligned banking.
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
        if(run.identity=="organic-inward-recovery"&&run.corridor==1)pace(run.begin,run.end,65,1);
        // The complete Immelmann returns over this entry at its low exit datum.
        // Keep it level while retaining the other crests' seeded variation.
        if(run.identity=="immelmann-inward-entry-brake"){pacing.next();continue;}
        // A single broad crest flies over the low Immelmann entry footprint;
        // two crests put their shared valley directly at this crossing.
        if(run.identity=="interior-low-return")pace(run.begin,run.end,46,1);
    }
    for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    // Join the neighbouring physical profiles through their connector, rather
    // than returning every port to zero curvature and roll. Keep a quarter
    // of each major profile as the maximum transition extent, so its interior
    // remains authored. Drive zones keep their indices through compilation.
    auto connector=[](const std::string& name){return name=="recovery"||name=="corridor-recovery"||name=="inversion-recovery"||name=="airtime-entry-boost";};
    std::vector<std::pair<size_t,size_t>> flowJoins;
    size_t flowBridgeBegin=0,flowBridgeEnd=0;
    for(size_t m=0;m+1<modules.size();++m){
        const auto& left=modules[m];if(connector(left.identity)||left.identity=="station"||left.identity=="departure-launch")continue;
        size_t n=m+1;while(n<modules.size()&&connector(modules[n].identity))++n;if(n==modules.size())continue;
        const auto& right=modules[n];
        if(left.identity=="high-immelmann"||right.identity=="high-immelmann")continue;
        const double begin=distance[left.end]-std::min(110.,.25*(distance[left.end]-distance[left.begin]));
        const double end=distance[right.begin]+std::min(110.,.25*(distance[right.end]-distance[right.begin]));
        // Terminal braking and the level station approach retain their own
        // physical profiles; this pass composes the moving ride's elements.
        if(end>distance.back()-(65*65/(2*2.4)+60))continue;
        size_t first=size_t(std::lower_bound(distance.begin(),distance.end(),begin)-distance.begin());
        size_t last=size_t(std::lower_bound(distance.begin(),distance.end(),end)-distance.begin());
        detail::blendAuthoredJoin(raw,first,last,cancel);flowJoins.push_back({first,last});
        if(left.identity=="interior-low-return"&&right.identity=="banked-camelback-turn"){flowBridgeBegin=first;flowBridgeEnd=last;}
        // The whole curved part of a drive participates in force-aligned
        // banking, including shared endpoints. Its motor remains an operation.
        if(raw[first].element==Element::Turn||raw[last].element==Element::Turn)
            for(size_t i=first;i<=last;++i)if(raw[i].element==Element::Launch||raw[i].element==Element::Brake||raw[i].element==Element::Return)raw[i].element=Element::Turn;
    }
    if(!flowBridgeEnd)throw std::runtime_error("Folded flow family has no returning-S raccord");
    for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    // Locate actual authored diagonal crossings after loop offsets and inward
    // routing. Raise the return diagonal with a broad C3 envelope; an XY
    // crossing alone never authorizes acceptance (full sweep/support tests do).
    size_t branchBegin[4]={raw.size(),raw.size(),raw.size(),raw.size()},branchEnd[4]={};
    for(const auto& run:modules){branchBegin[run.corridor]=std::min(branchBegin[run.corridor],run.begin);branchEnd[run.corridor]=std::max(branchEnd[run.corridor],run.end);}
    struct Crossing{double along,lift;};std::vector<Crossing> crossings;
    for(size_t i=branchBegin[1];i<branchEnd[1];++i){Vec3 a=raw[i].position,b=raw[i+1].position-a;
        for(size_t j=branchBegin[3];j<branchEnd[3];++j){Vec3 c=raw[j].position,e=raw[j+1].position-c;double det=cross(b,e).z;if(std::abs(det)<1e-8)continue;
            double u=cross(c-a,e).z/det,v=cross(c-a,b).z/det;if(u<0||u>1||v<0||v>1)continue;
            double along=distance[j]+v*(distance[j+1]-distance[j]);double lift=a.z+b.z*u+42-(c.z+e.z*v);
            if(along-distance[branchBegin[3]]<300||distance[branchEnd[3]]-along<300)continue;
            crossings.push_back({along,std::max(0.,lift)});
        }
    }
    if(crossings.empty())throw std::runtime_error("Folded route has no separated diagonal crossing");
    // Move the complete airtime sequence onto one flyover datum. A sloping
    // envelope through its crests can erase a descent or turn two hills into one.
    size_t flyoverBegin=branchBegin[3],crestBegin=branchEnd[3],crestEnd=branchBegin[3];
    for(const auto& run:modules){
        if(run.corridor==2&&run.identity=="banked-camelback-turn")flyoverBegin=run.begin;
        if(run.corridor==3&&run.identity=="fvd-airtime"){crestBegin=std::min(crestBegin,run.begin);crestEnd=std::max(crestEnd,run.end);}
    }
    double plateauBegin=distance[crestBegin],plateauEnd=distance[crestEnd],flyoverLift=0;
    for(const auto& crossing:crossings){plateauBegin=std::min(plateauBegin,crossing.along);plateauEnd=std::max(plateauEnd,crossing.along);flyoverLift=std::max(flyoverLift,crossing.lift);}
    for(size_t i=flyoverBegin;i<=branchEnd[3];++i){
        const double s=distance[i],rise=layoutSmooth((s-distance[flyoverBegin])/(plateauBegin-distance[flyoverBegin]));
        const double fall=layoutSmooth((distance[branchEnd[3]]-s)/(distance[branchEnd[3]]-plateauEnd));
        raw[i].position.z+=flyoverLift*std::min(rise,fall);
    }
    for(size_t i=0;i<raw.size();++i)raw[i].bank*=stationBankFactor(distance.back()-distance[i],req.train);
    if(norm(raw.back().position-raw.front().position)>.001)throw std::runtime_error("Solved route failed canonical height closure");raw.back().position=raw.front().position;

    // Fit the rider envelope above the flat ground while fixing the station boundary.
    const double trainHalf=(req.train.cars-1)*req.train.spacing*.5;
    const double stationGroundMin=0,stationGroundMax=0;
    double stationDatum=req.limits.minClearance+4;
    const int controls=int(std::ceil(distance.back()/50));const double spacing=distance.back()/controls;
    std::vector<double> base(controls),target(controls),lower(controls,-1e9),weight(controls),required(unique);std::vector<bool> fixed(controls);
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
        required[i]=floor;int k=int(std::llround(distance[i]/spacing))%controls;lower[k]=std::max(lower[k],floor);target[k]+=ground+req.limits.minClearance+4;++weight[k];
    }
    const double fixedDeparture=std::ceil((200+trainHalf+12)/spacing)*spacing,fixedReturn=100;
    auto fixedStationBoundary=[&](double s){return s<=fixedDeparture||distance.back()-s<=fixedReturn;};
    // The same local flat datum must clear both the boarding bay and the
    // actual fixed launch/return boundary. Bound local platform height at 16 m
    // rather than inheriting the highest terrain elsewhere on the circuit.
    for(int k=0;k<controls;++k)if(fixedStationBoundary(k*spacing))stationDatum=std::max(stationDatum,lower[k]+.3);
    plan.stationChecked=true;plan.stationBayMinimum=stationGroundMin;plan.stationBayMaximum=stationGroundMax;plan.stationRequiredDatum=stationDatum;
    plan.stationFeasible=stationDatum-stationGroundMin<=16;
    if(!plan.stationFeasible)continue;
    // A failed ride deserves one different station placement at the same
    // geometry scale before larger radii/ramps spend the footprint budget.
    // Placement identifiers jointly encode heading and anchored site; merely
    // selecting another shape at the same site is not this second alternative.
    if(placementVariant){
        if(firstFeasiblePlacement<0){firstFeasiblePlacement=plan.placement;continue;}
        if(plan.placement==firstFeasiblePlacement)continue;
    }
    plan.stationSelectionEligible=true;
    for(int k=0;k<controls;++k){target[k]=weight[k]?target[k]/weight[k]:stationDatum;base[k]=std::max(target[k],lower[k]);fixed[k]=fixedStationBoundary(k*spacing);if(fixed[k]){if(lower[k]>stationDatum+1e-8)throw std::runtime_error("Local station datum cannot clear its fixed launch/boarding boundary");base[k]=stationDatum;}}
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
    for(int refinement=0;refinement<3;++refinement){solveBaseline();bool raised=false;for(size_t i=0;i<unique;++i){double deficit=required[i]-baseline(distance[i]);if(deficit>.01){int k=int(std::llround(distance[i]/spacing))%controls;if(fixed[k])throw std::runtime_error("Local station boundary violates terrain envelope");lower[k]=std::max(lower[k],base[k]+deficit*1.5);raised=true;}}if(!raised)break;}
    solveBaseline();for(size_t i=0;i<unique;++i)raw[i].position.z+=baseline(distance[i]);raw.back()=raw.front();d.track=compile(raw);
    auto at=[&](size_t index){return index>=d.track.spans.size()?d.track.length:d.track.spans[index].start;};
    for(size_t i=0;i<forceEntryIndices.size();++i)ports.airtimeEntry[i]=at(forceEntryIndices[i]);
    ports.reversalExit=at(reversalExitIndex);ports.reversalSpeedHint=immelmann.exit.speed;
    for(auto p:pending){
        bool hard=p.kind==DriveKind::Launch;double acc=hard?launchAcceleration:3.5;
        d.operations.push_back({at(p.begin),at(p.end),p.kind,p.speed,req.train.carMass*acc,req.train.carMass*acc*100,hard?.08:.5});
    }
    double brakingStart=d.track.length-(65*65/(2*2.4)+60);
    for(auto& op:d.operations)if(op.kind==DriveKind::Boost)op.end=std::min(op.end,brakingStart);
    d.operations.erase(std::remove_if(d.operations.begin(),d.operations.end(),[](const Operation& op){return op.end<=op.start;}),d.operations.end());
    // A real descent can exert more than the former 4 m/s^2 brake rating.
    // Size the hardware against the canonical downhill grade plus the same
    // requested stopping deceleration; the controller still applies bounded
    // force/power through its ramp and the complete train is resimulated.
    double terminalDownhill=0;
    for(double s=brakingStart;s<d.track.length;s+=1)terminalDownhill=std::max(terminalDownhill,-d.track.sample(s).tangent.z);
    const double terminalBrakeAcceleration=std::max(4.,2.4+gravity*terminalDownhill+1);
    d.operations.push_back({brakingStart,80,DriveKind::Station,0,req.train.carMass*terminalBrakeAcceleration,req.train.carMass*terminalBrakeAcceleration*100,.5,2.4,.2});
    for(auto& operation:d.operations)operation.exitFadeMeters=std::max(1.,operation.targetSpeed*operation.rampSeconds);
    coalesceDriveProfiles(d.operations);
    if(reversingPair)d.topology+="/immelmann-low-return";
    if(!flowJoins.empty())d.topology+="/continuous-module-joins";
    if(!pacingCrests.empty())d.topology+="/paced-connectors";
    if(organicRecoveryCount)d.topology+="/organic-recovery";
    std::ostringstream diagnostic;diagnostic<<std::setprecision(12)
        <<"{\"schemaVersion\":1,\"generationOnly\":true,\"seed\":"<<req.seed<<",\"candidate\":"<<attempt
        <<",\"order\":\""<<orderName<<"\",\"stationAnchoring\":{\"geometryScale\":"<<geometryAttempt
        <<",\"placementVariant\":"<<placementVariant<<",\"selectedPlanRank\":"<<(&plan-plans.data())
        <<",\"heightBudget\":16,\"datum\":"<<stationDatum<<",\"fixedDepartureMeters\":"<<fixedDeparture
        <<",\"fixedReturnMeters\":"<<fixedReturn<<",\"controlSpacing\":"<<spacing
        <<"},\"launchPlanning\":{\"requestedSeconds\":"<<req.targets.launchSeconds<<",\"motorAcceleration\":"<<launchAcceleration
        <<",\"predictedSeconds\":"<<plannedLaunchTime(launchAcceleration,req.train)
        <<"},\"intensityPlanning\":{\"configured\":"<<(intensityDesign?"true":"false")<<",\"exposureGoal\":"<<exposureGoal
        <<",\"normalLoadHint\":"<<designNormalG<<",\"extraHelixCorner\":"<<holdCorner
        <<"},\"sides\":"<<sides<<",\"plannedHorizontalLength\":"<<plan.length<<",\"canonicalLength\":"<<d.track.length
        <<",\"selected\":{\"shape\":"<<plan.shape<<",\"placement\":"<<plan.placement<<",\"score\":"<<plan.score<<"},\"corridors\":[";
    for(int i=0;i<sides;++i){if(i)diagnostic<<',';diagnostic<<"{\"length\":"<<plan.lengths[i]<<",\"turnAngle\":"<<plan.angles[i]<<'}';}diagnostic<<"],\"rankedPlans\":[";
    for(size_t i=0;i<plans.size();++i){if(i)diagnostic<<',';const auto& p=plans[i];diagnostic<<"{\"shape\":"<<p.shape<<",\"placement\":"<<p.placement<<",\"length\":"<<p.length<<",\"score\":"<<p.score<<",\"stationBudgetChecked\":"<<(p.stationChecked?"true":"false")<<",\"stationBudgetFeasible\":"<<(p.stationFeasible?"true":"false")<<'}';}diagnostic<<"],\"modules\":[";
    for(size_t i=0;i<modules.size();++i){if(i)diagnostic<<',';auto& m=modules[i];diagnostic<<"{\"identity\":\""<<m.identity<<"\",\"corridor\":"<<m.corridor<<",\"start\":"<<at(m.begin)<<",\"end\":"<<at(m.end)<<'}';}diagnostic<<"]}";d.planningDiagnostics=diagnostic.str();
    d.planningDiagnostics.pop_back();std::ostringstream expansion;expansion<<std::setprecision(12)<<",\"layoutExpansion\":{\"reversingPair\":"<<(reversingPair?"true":"false")<<",\"organicRecoveryCount\":"<<organicRecoveryCount<<",\"hillProfileShape\":"<<hillProfileShape<<",\"loopProfileShape\":"<<loopProfileShape;
    if(reversingPair)expansion<<",\"height\":"<<reversalHeight<<",\"entrySpeedHint\":"<<reversalEntrySpeed
        <<",\"exitSpeedHint\":"<<immelmann.exit.speed<<",\"innerLane\":"<<reversalLane
        <<",\"netForwardLength\":"<<reversalNetLength<<",\"entryEnergyCorrection\":"<<feedback.reversalEnergyCorrection;
    expansion<<",\"returnFlowBridge\":{\"start\":"<<at(flowBridgeBegin)<<",\"end\":"<<at(flowBridgeEnd)<<"},\"flowJoins\":[";
    for(size_t i=0;i<flowJoins.size();++i){if(i)expansion<<',';expansion<<"{\"start\":"<<at(flowJoins[i].first)<<",\"end\":"<<at(flowJoins[i].second)<<'}';}
    expansion<<"],\"forceDesignedAirtime\":[";
    for(size_t i=0;i<forceHills.size();++i){if(i)expansion<<',';const auto& hill=forceHills[i];expansion<<"{\"span\":"<<hill.span<<",\"height\":"<<hill.height<<",\"sourceSpeedMps\":"<<hill.authoring.speed<<",\"sourceReplayPassed\":"<<(hill.section.assessment.passed?"true":"false")<<",\"maxSourceNormalResidualG\":"<<hill.section.assessment.maxNormalResidualG<<'}';}
    expansion<<"],\"connectorPacing\":[";
    for(size_t i=0;i<pacingCrests.size();++i){if(i)expansion<<',';const auto& crest=pacingCrests[i];expansion<<"{\"start\":"<<at(crest.begin)<<",\"end\":"<<at(crest.end)<<",\"height\":"<<crest.height<<",\"designSpeed\":"<<crest.speed<<",\"count\":"<<crest.count<<'}';}
    expansion<<"]}}";d.planningDiagnostics+=expansion.str();return d;
    }
    throw std::runtime_error(placementVariant?"No second distinct feasible station placement fits the actual boundary and footprint budget":"No station placement fits the actual station departure/return boundary and 16 m bay-height budget");
}
static void placeSupports(Design& d,Cancel cancel){buildSupportLayout(d,cancel);}
static void improveBanking(Design& d,const std::vector<Frame>& frames){
    if(frames.empty())return;
    const size_t count=d.track.spans.size();std::vector<double> authored(count),target(count),along(count),left(count),right(count),speed(count);
    for(size_t i=0;i<count;++i){const auto& k=d.track.knots[i];authored[i]=target[i]=k.bank;along[i]=d.track.spans[i].start;
        speed[i]=replayValueAt(frames,along[i]);if(k.element!=Element::Turn)continue;
        Vec3 required=k.curvature*(speed[i]*speed[i])+Vec3{0,0,gravity},side=cross(k.tangent,k.up);double normal=dot(required,k.up),lateral=dot(required,side);
        // Select within the bounded turn-bank interval before smoothing. A
        // force-vector winding cannot carry the bank target into another lap.
        double angle=k.bank;if(std::hypot(normal,lateral)>1e-5)angle=std::atan2(lateral,normal);
        target[i]=std::clamp(angle,-1.5,1.5);
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
    ValidationReport r;if(!req.terrain.valid()){r.fail("TERRAIN_PROFILE","Only flat ground is available in this build");return r;}const auto& t=req.targets;const auto& l=req.limits;
    for(double value:{t.height,t.speed,t.inversionHeight,t.launchSeconds,l.minVerticalG,l.maxVerticalG,l.maxLateralG,l.maxLongitudinalG,l.maxJerkGps,l.minClearance,req.simulationStep})if(!std::isfinite(value)){r.fail("REQUEST_RANGE","Request contains a nonfinite value");return r;}
    if(req.maxCandidates<1||req.maxCandidates>64||t.height<0||t.height>350||t.speed<1||t.speed>110||t.inversionHeight<0||t.inversionHeight>140||t.launchSeconds<.8||t.launchSeconds>10||l.maxVerticalG<=l.minVerticalG||l.maxLateralG<=0||l.maxLongitudinalG<=0||l.maxJerkGps<=0||l.minClearance<0||req.simulationStep<1./2000||req.simulationStep>1./30||!req.terrain.valid())r.fail("REQUEST_RANGE","Request is outside the prototype's supported domain");
    if(std::isinf(t.referenceExposure)||(std::isfinite(t.referenceExposure)&&t.referenceExposure<=0)||(!t.referenceId.empty()&&!std::isfinite(t.referenceExposure))||(std::isfinite(t.referenceExposure)&&t.referenceId.empty()))r.fail("REFERENCE_CONFIG","Configured reference needs a finite positive exposure and a nonempty ID");
    auto reference=validateReference(t);r.errors.insert(r.errors.end(),reference.errors.begin(),reference.errors.end());
    for(double rate:{l.maxLateralRateGps,l.maxLongitudinalRateGps})if(std::isinf(rate)||(std::isfinite(rate)&&rate<=0))r.fail("AXIS_RATE_CONFIG","Optional component rate gates must be finite positive values, or unset");
    const auto& train=req.train;
    for(double value:{train.carMass,train.spacing,train.seatHeight,train.dragCdA,train.rollingResistance,train.airDensity})if(!std::isfinite(value)){r.fail("TRAIN_CONFIG","Train contains nonfinite settings");return r;}
    if(train.cars<1||train.cars>16||train.spacing<=0||train.spacing>20||train.carMass<=0||train.seatHeight<0||train.seatHeight>3||train.dragCdA<0||train.rollingResistance<0||train.airDensity<0)r.fail("TRAIN_CONFIG","Train is outside the supported model domain");
    return r;
}
Design generate(const GenerationRequest& input,Cancel cancel,std::function<void(int,const std::string&)> progress){
    std::mutex cancellationMutex;
    const Cancel requestedCancel=std::move(cancel);
    if(requestedCancel)cancel=[&]{std::lock_guard lock(cancellationMutex);return requestedCancel();};
    GenerationRequest req=input;
    Design last;last.request=req;
    last.report=validateRequest(req);if(!last.report.valid())return last;
    double fastestDeparture=plannedLaunchTime(req.limits.maxLongitudinalG*gravity-.02,req.train);
    if(fastestDeparture>req.targets.launchSeconds){last.report.fail("LAUNCH_FEASIBILITY","Requested departure is below the force-limited flat-station prototype bound",0,fastestDeparture,req.targets.launchSeconds);return last;}
    if(req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure)&&req.targets.referenceExposure*1.1>10*std::max(0.,req.limits.maxVerticalG)){last.report.fail("INTENSITY_FEASIBILITY","Requested exposure exceeds ten seconds at the selected vertical force ceiling",0,req.targets.referenceExposure*1.1,10*std::max(0.,req.limits.maxVerticalG));return last;}
    std::vector<std::string> history;Design bestIntensity,bestPacing;bool haveBestIntensity=false,haveBestPacing=false;double bestDuration=INFINITY;
    auto remember=[&](const Design* d,int index){
        if(progress&&d){
            progress(index,d->accepted()?"Candidate accepted":"Candidate rejected");
            for(const auto* report:{&d->report,&d->simulation.report}){
                for(const auto& f:report->errors){
                    std::ostringstream line;line<<std::setprecision(17)<<f.code<<": "<<f.message
                        <<" | distanceMeters="<<f.distance<<" actual="<<f.actual<<" limit="<<f.limit;
                    progress(index,line.str());
                }
                for(const auto& warning:report->warnings)progress(index,"Warning: "+warning);
            }
        }
        std::ostringstream h;h<<std::setprecision(12)<<"{\"candidate\":"<<index<<",\"constructed\":"<<(d?"true":"false");
        if(d){h<<",\"completed\":"<<(d->simulation.completed?"true":"false")<<",\"exposure10Seconds\":"<<d->simulation.metrics.exposure10Seconds<<",\"launchSeconds\":";if(std::isfinite(d->simulation.metrics.launchTo180))h<<d->simulation.metrics.launchTo180;else h<<"null";if(d->simulation.completed)h<<",\"movingDurationSeconds\":"<<movingRideSeconds(*d);h<<",\"errors\":[";bool comma=false;for(const auto* report:{&d->report,&d->simulation.report})for(const auto& f:report->errors){if(comma)h<<',';comma=true;h<<'"'<<f.code<<'"';}h<<']';}
        else h<<",\"errors\":[\"CANDIDATE_FAILURE\"]";h<<'}';history.push_back(h.str());
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
        if(progress)progress(i,"Solving route corridors and circuit");
        try{
            AuthoringFeedback feedback;CandidatePorts ports;
            Design d=candidate(req,i,cancel,feedback,ports);
            if(progress)progress(i,"Simulating initial geometry");
            auto motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel);
            if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            if(motion.completed){
                // One bounded correction uses measured finite-train entry energy.
                // Rebuild the entire route and operation zones;
                // never stretch an FVD curve or substitute a prescribed speed.
                for(size_t h=0;h<feedback.airtimeSpeed.size();++h)feedback.airtimeSpeed[h]=replayValueAt(motion.frames,ports.airtimeEntry[h]);
                double exitSpeed=replayValueAt(motion.frames,ports.reversalExit);
                feedback.reversalEnergyCorrection=ports.reversalSpeedHint*ports.reversalSpeedHint-exitSpeed*exitSpeed;
                d=candidate(req,i,cancel,feedback,ports);
                if(progress)progress(i,"Simulating energy-corrected geometry");
                motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel);
                if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            }
            if(motion.completed)improveBanking(d,motion.frames);
            // These replays read frozen track/drive data. Structure construction
            // writes separate members; all workers join before d can be retired.
            std::atomic<bool> stopFine{false};
            auto coarse=std::async(std::launch::async,[&]{return simulate(d.track,d.operations,req.train,req.simulationStep,cancel);});
            std::future<SimulationResult> fine;
            if(motion.completed)fine=std::async(std::launch::async,[&]{return simulate(d.track,d.operations,req.train,req.simulationStep*.5,[&]{return stopFine.load()||(cancel&&cancel());});});
            if(progress)progress(i,"Replaying final geometry while constructing station and supports");
            d.station=buildStation(d.track,req.terrain,req.train,cancel);
            placeSupports(d,cancel);
            d.inversionDimensions=measureInversionDimensions(d.track,cancel);
            if(progress)progress(i,"Checking measured targets and clearance");
            d.report=validateGeometry(d.track,req.terrain,req.limits,req.train,d.supports,cancel);
            auto structures=validateDesignStructures(d,cancel);d.report.errors.insert(d.report.errors.end(),structures.errors.begin(),structures.errors.end());
            d.simulation=coarse.get();
            evaluateTargets(d);
            if(d.report.valid()&&d.simulation.completed&&d.simulation.report.valid()&&!d.simulation.cancelled){
                if(progress)progress(i,"Checking independent half-step simulation");
                if(fine.valid())verifyConvergenceWith(d,[&]{return fine.get();},cancel);
                else verifyConvergence(d,cancel);
            }
            if(fine.valid()){stopFine.store(true);fine.wait();}
            if(d.simulation.cancelled)return d;
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
        }catch(const std::exception& e){if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}remember(nullptr,i);last.report.fail("CANDIDATE_FAILURE",e.what());if(progress)progress(i,std::string("Candidate rejected: ")+e.what());}
    }if(haveBestPacing)return withHistory(std::move(bestPacing),"best-accepted-moving-duration-shortfall");if(haveBestIntensity)return withHistory(std::move(bestIntensity),"best-physically-valid-intensity-shortfall");return withHistory(std::move(last),"last-constructed-rejection");
}
}
