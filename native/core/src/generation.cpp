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
// The launch is compressed air, Do-Dodonpa class, and the dialled 0-180 km/h time
// is that record (1.56 s) times the player's multiplier. Solve for the smallest
// acceleration that meets it, so the ride never buys margin it was not asked for.
static double sizedLaunchAcceleration(const GenerationRequest& req){
    double upper=req.limits.maxLongitudinalG*gravity-.02,lower=0;
    for(int i=0;i<32;++i){double mid=(lower+upper)*.5;if(plannedLaunchTime(mid,req.train)>req.targets.launchSeconds-.003)lower=mid;else upper=mid;}
    return std::min(req.limits.maxLongitudinalG*gravity-.02,upper);
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
static double smoothIntegral(double u){if(u<=0)return 0.;if(u>=1)return u-.5;return u*u*u*u*(2.5+u*(-3+u));}
static Vec3 inFrame(Vec3 p,double h){return {p.x*std::cos(h)-p.y*std::sin(h),p.x*std::sin(h)+p.y*std::cos(h),p.z};}
static bool linearPropulsionGeometry(const Track& track,double s,double margin=1,double planMargin=1){
    const auto k=sampleKinematics(track,s);const auto& p=k.sample;
    const Vec3 upright=unit(Vec3{0,0,1}-p.tangent*p.tangent.z);
    // Near-constant grade, negligible plan curvature and an upright frame.
    return std::hypot(p.tangent.x,p.tangent.y)>.5&&std::abs(cross(p.tangent,p.curvature).z)<planMargin*1e-5&&std::abs(p.curvature.z)<margin*1e-4&&dot(p.up,upright)>.9998&&norm(k.upS)<margin*.001;
}

static std::vector<RoutePlan> planRoutes(int sides,const std::vector<double>& minimum,double radius,double ramp,int holdCorner,bool reversingPair,double stationApproach,Random& rng,Cancel cancel,bool crestGuide=false){
    std::vector<RoutePlan> feasible;
    double startingHeading=rng.range(-pi,pi);Vec3 stationAnchor{rng.range(-200,200),rng.range(-160,160),0};
    for(int shape=0;shape<48;++shape){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        RoutePlan p;p.shape=shape;p.angles.resize(sides);p.lengths=minimum;
        // A signed four-leg folded grammar. Paired unequal angles keep the
        // endpoint heading exact while the diagonals cross inside the layout.
        const double alpha=rng.range(.71,.79)*pi,beta=rng.range(.71,.79)*pi;
        p.angles={alpha,-beta,-alpha,beta};
        std::vector<Vec3> directions;Vec3 sum{stationApproach,0,0};double h=0;
        for(int side=0;side<sides;++side){directions.push_back({std::cos(h),std::sin(h),0});
            const double extra=side==holdCorner?std::copysign(2*pi,p.angles[side]):0;
            p.turns.push_back(makeTurn(p.angles[side]+extra,radius,ramp));sum=sum+inFrame(p.turns.back().points.back(),h);h+=p.angles[side];}
        // Draw always: preserves shape RNG sequence when crestGuide==false.
        const double formerAllowance=rng.range(25,100);if(crestGuide)p.lengths[0]+=formerAllowance;
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
        // An extra closed helix lap increases rail length without increasing footprint.
        const double repeatedHelixLength=reversingPair&&holdCorner>=0?2*pi*radius:0;
        p.length+=stationApproach;
        if(!valid||p.length<4700||p.length-repeatedHelixLength>9800)continue;
        Vec3 cursor{};h=0;
        for(int side=0;side<sides;++side){
            Vec3 forward{std::cos(h),std::sin(h),0};
            cursor=cursor+forward*p.lengths[side];
            cursor=cursor+inFrame(p.turns[side].points.back(),h);h+=p.angles[side];
        }
        cursor=cursor+Vec3{stationApproach,0,0};
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
    std::array<double,4> airtimeSpeed{52,65,65,65};
    double flyoverLift{INFINITY},reversalEnergyCorrection{},recoveryEntrySpeed{42},terminalEntrySpeed{50},helixEntrySpeed{45},postLoopEntrySpeed{42},loopExitSpeed{46};
    // Measured shortfall of the first pass against the speed dial: drag and the
    // motor fade cost the launch its last fraction, so the graded pass asks for
    // that much more and lands on the dial instead of near it.
    double speedCorrection{};
};
struct CandidatePorts {
    std::array<double,4> airtimeEntry{};
    double flyoverLift{},reversalExit{},reversalSpeedHint{},recoveryEntry{},terminalEntry{},helixEntry{},postLoopEntry{},loopExit{};
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
static Design candidate(const GenerationRequest& req,int attempt,Cancel cancel,const AuthoringFeedback& feedback,CandidatePorts& ports,bool graded=true){
    const int geometryAttempt=attempt/2,placementVariant=attempt%2;
    Design d;d.request=req;d.candidate=attempt;Random rng{req.seed};rng.next();int sides=4;bool helix=false;
    rng.next();int order=0;
    bool intensityDesign=req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure);double exposureGoal=intensityDesign?req.targets.referenceExposure*1.1:0;
    double designNormalG=rng.range(3.2,3.6);if(intensityDesign)designNormalG=std::min(req.limits.maxVerticalG-.5,std::max(designNormalG,exposureGoal/10+.35+geometryAttempt*.10));
    designNormalG=std::max(1.2,designNormalG);double radius=65*65/(gravity*std::sqrt(designNormalG*designNormalG-1))+(intensityDesign?0:geometryAttempt*12),ramp=160+geometryAttempt*15;
    double elevation=req.targets.height+rng.range(8,30),loopHeight=req.targets.inversionHeight+rng.range(8,18);
    double hillWidth=rng.range(1020,1120)+geometryAttempt*20,loopDrift=rng.range(330,380)+geometryAttempt*10;
    // Section boosters keep their authored power; only their target speed is
    // tied to the ride's design speed, so a faster dial feeds the elements that
    // follow them proportionally instead of a fixed 65 m/s.
    const double sectionSpeed=std::max(62.,req.targets.speed*.78);
    // The loop is entered through a trim brake whose target was a flat 46 m/s
    // regardless of the dial. That left the apex at 15 m/s: over the loop's own
    // radius the rider gets v^2/(gR)-1 there, so they HUNG in the restraints at
    // -0.2 g instead of being pressed into the seat, which is what a real
    // vertical loop does. Clearing a stall was never the test. Tied to the dial
    // at .6 of it, the apex carries positive force and the loop bottom stays
    // well inside its ceiling.
    const double loopEntrySpeed=std::max(46.,req.targets.speed*.6);
    // The speed dial is a setpoint, not a floor: the launch is the ride's fastest
    // point, so it targets the dialled speed exactly. A hill too tall to crest at
    // that speed raises it -- reported as the dial's error, never randomised.
    double topSpeed=std::max(req.targets.speed+(graded?feedback.speedCorrection:0),std::sqrt(2*gravity*elevation+45*45)),launchAcceleration=sizedLaunchAcceleration(req);
    int hillSide=0,loopSide=1;
    std::vector<int> module(sides,2);module[hillSide]=0;module[loopSide]=1;
    if(sides==6){module[2]=3;module[5]=3;}
    rng.range(17,23); // Retain the independent route random sequence.
    // Independent detail stream preserves the route random sequence.
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
    std::vector<FvdAirtimeResult> forceHills;std::array<double,4> forceFlank{};
    for(int hill=0;hill<4;++hill){FvdAirtimeRequest force;force.speed=feedback.airtimeSpeed[hill];force.pushG=detail.range(2.08,2.4);force.crestG=detail.range(-.22,-.08);
        auto authored=designFvdAirtime(force,cancel);
        if(!authored.section.integrated||!authored.section.canonicalBuilt||!authored.section.report.valid()||!authored.section.assessment.passed){
            if(authored.section.cancelled)throw std::runtime_error("CANCELLED");
            throw std::runtime_error("FVD hill "+std::to_string(hill)+" at "+std::to_string(force.speed)+" m/s: "+(authored.section.report.errors.empty()?"canonical replay failed":authored.section.report.errors.front().message));
        }
        const auto core=std::find_if(authored.section.samples.begin(),authored.section.samples.end(),[](const FvdSample& q){return dot(q.curvature*(q.speed*q.speed)+Vec3{0,0,gravity},q.up)<=0;});
        if(core==authored.section.samples.end())throw std::runtime_error("FVD hill has no negative-force body");
        // Keep the entire negative-force body and four authored derivative guards.
        forceFlank[hill]=std::min(authored.section.track.length/3,core->distance-8);
        forceHills.push_back(std::move(authored));}
    const double forceReturnSpan=forceHills[1].span+forceHills[2].span+forceHills[3].span;
    if(reversingPair)module[2]=4;
    std::vector<double> minimum(sides);
    for(int side=0;side<sides;++side){
        double initial=side==0?200:0;
        if(module[side]==0)minimum[side]=initial+(side?500:0)+hillWidth;
        else if(module[side]==1)minimum[side]=initial+(side?380:300)+loopDrift+220+180+forceHills[0].span+360;
        else if(module[side]==2)minimum[side]=forceReturnSpan+70;
        else if(module[side]==4)minimum[side]=reversalNetLength+150;
        else minimum[side]=130;
    }
    int holdCorner=intensityDesign&&exposureGoal>18&&sides>2?(hillSide+1)%(sides-1):-1;
    const double terminalDeceleration=6;
    // The existing station adds 30 m after the rear clears the seam. Reserve
    // the stopping energy, the motor ramp/controller response and train entry.
    const double stationApproach=feedback.terminalEntrySpeed*feedback.terminalEntrySpeed/(2*terminalDeceleration)+feedback.terminalEntrySpeed*.25+(req.train.cars-1)*req.train.spacing*.5-30;
    // Recover only the previous connector reservation for its visible crest
    // guide. This small closure solve does not author or simulate another ride.
    auto guideMinimum=minimum;guideMinimum[0]+=140;guideMinimum[1]-=300;guideMinimum[2]-=80;guideMinimum[3]+=100;
    auto guideRandom=rng;
    const auto guidePlans=planRoutes(sides,guideMinimum,radius,ramp,holdCorner,reversingPair,0,guideRandom,cancel,true);
    const double guideRecovery=guidePlans.empty()?60:guidePlans.front().lengths[1]-(guideMinimum[1]-60);
    // Reassign the connector reservations to two substantial ride sections.
    // The held helix has its own third section; the final return only coasts.
    const double postLoopReservation=holdCorner>=0?100:220;
    minimum[1]=guideMinimum[1]-60+guideRecovery+postLoopReservation+(holdCorner>=0?160:0);
    minimum[2]=reversalNetLength+220;
    auto plans=planRoutes(sides,minimum,radius,ramp,holdCorner,reversingPair,stationApproach,rng,cancel);if(plans.empty())throw std::runtime_error("No route fits exact closure and footprint budget");
    // Station fit filters ranked plans before geometry attempts are counted.
    int firstFeasiblePlacement=-1;
    for(auto& plan:plans){
    if(cancel&&cancel())throw std::runtime_error("CANCELLED");
    double heading=plan.heading;Vec3 origin=plan.origin,cursor=origin;
    const std::string orderName=order==0?"H-I-A":order==1?"H-A-I":sides==2?"I-A-H":"I-H-A";
    d.topology="folded-bow-tie/H-I-IM-A4";

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
        }else if(module[side]==1){
            double brakeLength=side?380:300;record(drive(brakeLength,DriveKind::Brake,loopEntrySpeed+geometryAttempt*.3),side,"inversion-entry-brake");used+=brakeLength;
            double amplitude=(loopDrift+std::sqrt(4*pi*pi*loopHeight*25))/(2*pi),offset=36;
            record(piece(900,[=](double u){return Vec3{loopDrift*u+amplitude*std::sin(2*pi*u),offset*smooth(u),loopHeight*std::pow(std::sin(pi*layoutWarp(u,loopProfileShape)),4)};},Element::Inversion),side,"record-inversion");used+=loopDrift;
            record(piece(220,[=](double u){return Vec3{220*u,-offset*smooth(u),0};},Element::Return),side,"inversion-recovery");used+=220;
            record(line(180,Element::Return),side,"airtime-entry-boost");used+=180;
            record(drive(postLoopReservation,DriveKind::Boost,sectionSpeed),side,"post-loop-section-boost");used+=postLoopReservation;
            used+=forceHill(0,side);
        }else if(module[side]==4){
            // The inward lead separates the low return from the incoming lane.
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
            for(int h=1;h<=3;++h)used+=forceHill(h,side);
        }
        if(used>straight+1e-6)throw std::runtime_error("Module plan exceeded reserved corridor");
        const double recoveryLength=straight-used;
        if(side==1&&holdCorner>=0){record(line(recoveryLength-160,Element::Return),side,"corridor-recovery");record(drive(160,DriveKind::Boost,sectionSpeed),side,"helix-section-boost");}
        else if(recoveryLength>1e-6)record(side==2?drive(recoveryLength,DriveKind::Boost,sectionSpeed):line(recoveryLength,Element::Return),side,"corridor-recovery");
        const auto& turn=plan.turns[side];Vec3 base=cursor;size_t turnStart=raw.size()-1;double dz=helix?(side==0?55:-55):(side==holdCorner?55:0);
        // The terminal turn keeps no crest of its own. Tried as a 12 m wave turn, it
        // spent the train's last speed on height right before the brakes, and the
        // turn was then taken at 24 m/s -- deader than the flat track it replaced.
        // The return's speed hills carry that stretch instead, and the turn keeps pace.
        const double bankedRise=graded&&side==sides-1?0:std::min(26.,.65*gravity*turn.length*turn.length/(4*pi*pi*65*65))*turnRiseScale[side];
        for(size_t i=1;i<turn.points.size();++i){double along=turn.length*i/(turn.points.size()-1),curvature=smooth(std::min(along,turn.length-along)/turn.ramp)/radius;cursor=base+inFrame(turn.points[i],heading);cursor.z=base.z+dz*smooth(along/turn.length)+bankedRise*std::pow(std::sin(pi*layoutWarp(along/turn.length,turnRiseShape[side])),4);append(cursor,-std::copysign(std::atan(65*65*curvature/gravity),turn.angle),Element::Turn);}
        modules.push_back({turnStart,raw.size()-1,side,helix||side==holdCorner?"sustained-helix":"banked-camelback-turn"});if(side==holdCorner)forceHoldEnd=raw.size()-1;
        heading+=plan.angles[side];
    }
    const size_t terminalTurnEnd=raw.size()-1;
    record(line(stationApproach,Element::Brake),sides-1,"station-approach");
    if(std::hypot(cursor.x-origin.x,cursor.y-origin.y)>.001)throw std::runtime_error("Solved route failed canonical XY closure");
    size_t unique=raw.size()-1;std::vector<double> distance(raw.size());for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    // Connecting crests fit the transit envelopes using gravity and curvature sizing.
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
        // The complete Immelmann returns over this entry at its low exit datum.
        // Keep it level while retaining the other crests' seeded variation.
        if(run.identity=="immelmann-inward-entry-brake"){pacing.next();continue;}
        // A single broad crest flies over the low Immelmann entry footprint;
        // two crests put their shared valley directly at this crossing.
        if(run.identity=="interior-low-return")pace(run.begin,run.end,46,1);
    }
    const auto ungraded=raw;
    if(graded){
        // A shared asymmetric crest spends its existing height across adjacent
        // transit. The maximum stays fixed; the inversion ports never move.
        auto carryCrest=[&](size_t first,size_t peak,size_t last,double crestWidth,double peakShift){
            const double length=distance[last]-distance[first],atPeak=distance[peak]-distance[first]+peakShift;
            const double crestStart=atPeak-crestWidth*.5;
            const double shoulder=std::min(70.,crestStart*.25),endShoulder=std::min(70.,(length-atPeak-crestWidth*.5)*.25);
            const double riseSpan=atPeak-shoulder*.5,fallSpan=length-atPeak-endShoulder*.5;
            const double begin=raw[first].position.z,end=raw[last].position.z,maximum=raw[peak].position.z;
            auto profile=[&](double s,double up,double down){return begin+up*shoulder*smoothIntegral(s/shoulder)-(up+down)*crestWidth*smoothIntegral((s-crestStart)/crestWidth)+down*endShoulder*smoothIntegral((s-length+endShoulder)/endShoulder);};
            double low=0,high=4*(maximum-std::min(begin,end))/std::min(riseSpan,fallSpan);
            for(int iteration=0;iteration<40;++iteration){
                const double up=(low+high)*.5,down=(begin+up*riseSpan-end)/fallSpan;
                double left=0,right=1;for(int j=0;j<40;++j){const double u=(left+right)*.5;if(smooth(u)<up/(up+down))left=u;else right=u;}
                const double top=profile(crestStart+crestWidth*(left+right)*.5,up,down);if(top>maximum)high=up;else low=up;
            }
            const double up=(low+high)*.5,down=(begin+up*riseSpan-end)/fallSpan;
            for(size_t i=first;i<=last;++i){raw[i].position.z=profile(distance[i]-distance[first],up,down);raw[i].upHint={0,0,1};}
        };
        for(size_t m=1;m<modules.size();++m){const auto& run=modules[m];
            if(run.identity=="immelmann-inward-entry-brake"){
                const auto& turn=modules[m-1];size_t peak=turn.begin;for(size_t i=turn.begin;i<=turn.end;++i)if(raw[i].position.z>raw[peak].position.z)peak=i;
                // Spend the camelback crest midway along the approach: both flanks
                // then keep a visible grade, and the banked turn holds one grade, so
                // force-aligned banking follows no crest inside it. The held helix spends
                // its whole turn climbing to the Immelmann datum, so only the entry brake
                // is left to carry the crest: size it by that brake and start its rounding
                // where the banking ends, so the descent keeps a grade of its own.
                const bool climbing=turn.identity=="sustained-helix";
                const double crestWidth=climbing?.3*(distance[run.end]-distance[run.begin])
                    :.3*(distance[turn.end]-distance[turn.begin]);
                const double crestShift=climbing?distance[run.begin]+crestWidth*.5-distance[peak]
                    :(distance[run.end]-distance[turn.begin])*.5-(distance[peak]-distance[turn.begin]);
                carryCrest(turn.begin,peak,run.end,crestWidth,crestShift);
            }
            if(run.identity=="interior-low-return"){
                const auto& turn=modules[m-1];size_t peak=run.begin;for(size_t i=run.begin;i<=run.end;++i)if(raw[i].position.z>raw[peak].position.z)peak=i;
                carryCrest(turn.begin,peak,run.end,.3*(distance[run.end]-distance[run.begin]),0);
            }
        }
    }
    for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);
    // Join the neighbouring physical profiles through their connector, rather
    // than returning every port to zero curvature and roll. Keep a quarter
    // of each major profile as the maximum transition extent, so its interior
    // remains authored. Drive zones keep their indices through compilation.
    auto connector=[](const std::string& name){return name=="recovery"||name=="corridor-recovery"||name=="inversion-recovery"||name=="airtime-entry-boost"||name=="post-loop-section-boost"||name=="helix-section-boost";};
    std::vector<std::pair<size_t,size_t>> flowJoins;
    size_t flowBridgeBegin=0,flowBridgeEnd=0;
    for(size_t m=0;m+1<modules.size();++m){
        const auto& left=modules[m];if(connector(left.identity)||left.identity=="station"||left.identity=="departure-launch")continue;
        size_t n=m+1;while(n<modules.size()&&connector(modules[n].identity))++n;if(n==modules.size())continue;
        const auto& right=modules[n];
        if(left.identity=="high-immelmann"||right.identity=="high-immelmann")continue;
        if(graded&&(right.identity=="immelmann-inward-entry-brake"||left.identity=="immelmann-level-return"&&right.identity=="interior-low-return"))continue;
        const bool airtimeValley=graded&&left.identity=="fvd-airtime"&&right.identity=="fvd-airtime";
        const bool airtimeEntry=graded&&left.identity=="banked-camelback-turn"&&right.identity=="fvd-airtime";
        // The camelback eases out level at its own end and the entry brake crest
        // eases in from level, so their shared valley reproduces both roundings
        // with a pause between them. Bridge from the same turn crest, out to where
        // the brake crest climbs straightest, so that valley turns in one pulse.
        const bool brakeEntry=graded&&left.identity=="banked-camelback-turn"&&right.identity=="inversion-entry-brake";
        const double leftExtent=airtimeValley?forceFlank[size_t(std::find(forceEntryIndices.begin(),forceEntryIndices.end(),left.begin)-forceEntryIndices.begin())]:std::min(110.,.25*(distance[left.end]-distance[left.begin]));
        // The turn's level ease-out duplicates the FVD entry's ease-in; bridge
        // from the turn's crest over the entry flank so the valley turns in one pulse.
        size_t leftCrest=left.end;
        if(airtimeEntry||brakeEntry)for(size_t i=left.begin;i<=left.end;++i)if(raw[i].position.z>raw[leftCrest].position.z)leftCrest=i;
        size_t entry=right.begin;
        if(brakeEntry){size_t top=right.begin;for(size_t i=right.begin;i<=right.end;++i)if(raw[i].position.z>raw[top].position.z)top=i;
            for(size_t i=right.begin;i<top;++i)if((raw[i+1].position.z-raw[i].position.z)/(distance[i+1]-distance[i])>(raw[entry+1].position.z-raw[entry].position.z)/(distance[entry+1]-distance[entry]))entry=i;}
        const double rightExtent=airtimeValley||airtimeEntry?forceFlank[size_t(std::find(forceEntryIndices.begin(),forceEntryIndices.end(),right.begin)-forceEntryIndices.begin())]:brakeEntry?distance[entry]-distance[right.begin]:std::min(110.,.25*(distance[right.end]-distance[right.begin]));
        auto join=[&](double begin,double end){
            if(end>distance[terminalTurnEnd])return;
            size_t first=size_t(std::lower_bound(distance.begin(),distance.end(),begin)-distance.begin());
            size_t last=size_t(std::lower_bound(distance.begin(),distance.end(),end)-distance.begin());
            detail::blendAuthoredJoin(raw,first,last,cancel,graded);flowJoins.push_back({first,last});
            if(left.identity=="interior-low-return"&&right.identity=="banked-camelback-turn"){flowBridgeBegin=first;flowBridgeEnd=last;}
            if(raw[first].element==Element::Turn||raw[last].element==Element::Turn)
                for(size_t i=first;i<=last;++i)if(raw[i].element==Element::Launch||raw[i].element==Element::Brake||raw[i].element==Element::Return)raw[i].element=Element::Turn;
        };
        if(n>m+1){
            const double connectorLength=distance[right.begin]-distance[left.end];
            const bool preserveCrest=left.identity=="record-inversion"||(left.identity=="fvd-airtime"&&left.corridor==1);
            std::vector<AuthoredPoint> crestSource,coastGuide;
            double crestHeight=0;size_t crestEnd=right.begin;
            const auto section=std::find_if(modules.begin()+m+1,modules.begin()+n,[](const ModuleRun& run){return run.identity=="post-loop-section-boost"||run.identity=="helix-section-boost";});
            if(section!=modules.begin()+n)crestEnd=section->begin;
            if(preserveCrest){
                crestSource=raw;auto guide=raw;
                // A coasting crest guide reads the authored turn rise; the graded
                // turn is restored under the guide after its crest is formed.
                if(left.identity=="fvd-airtime"&&section==modules.begin()+n)for(size_t j=right.begin;j<=right.end;++j)guide[j].position.z=ungraded[j].position.z;
                // The old broad join's endpoint jets define the donor crest.
                // Its height must use its former reservation, not the longer
                // connector created by the new straight stopping approach.
                size_t removed=0;
                if(section!=modules.begin()+n){
                    const Vec3 shift=raw[section->end].position-raw[section->begin].position;
                    removed=section->end-section->begin;
                    guide.erase(guide.begin()+section->begin+1,guide.begin()+section->end+1);
                    for(size_t j=section->begin+1;j<guide.size();++j)guide[j].position=guide[j].position-shift;
                }
                if(left.identity=="fvd-airtime"){
                    const Vec3 delta=guide[right.begin-removed].position-guide[left.end].position;
                    const double factor=guideRecovery/std::hypot(delta.x,delta.y);
                    for(size_t j=left.end;j<guide.size();++j){const double u=j>=crestEnd?1:(distance[j]-distance[left.end])/(distance[crestEnd]-distance[left.end]);guide[j].position=guide[j].position+Vec3{delta.x,delta.y,0}*((factor-1)*u);}
                }
                const size_t first=size_t(std::lower_bound(distance.begin(),distance.end(),distance[left.end]-leftExtent)-distance.begin());
                const size_t last=size_t(std::lower_bound(distance.begin(),distance.end(),distance[right.begin]+rightExtent)-distance.begin());
                detail::blendAuthoredJoin(guide,first,last-removed,cancel,graded);
                for(size_t j=left.end;j<=crestEnd;++j)crestHeight=std::max(crestHeight,guide[j].position.z-raw[left.end].position.z);
                if(left.identity=="fvd-airtime"&&!removed)coastGuide=std::move(guide);
            }
            const double endJoin=preserveCrest?std::min(35.,connectorLength*.2):0;
            join(distance[left.end]-leftExtent,distance[left.end]+endJoin);
            join(distance[right.begin]-endJoin,distance[right.begin]+rightExtent);
            if(preserveCrest){
                const size_t first=size_t(std::lower_bound(distance.begin(),distance.end(),distance[left.end]-leftExtent)-distance.begin());
                const size_t last=size_t(std::lower_bound(distance.begin(),distance.end(),distance[right.begin]+rightExtent)-distance.begin());
                if(!coastGuide.empty()){
                    for(size_t j=right.begin;j<=last;++j)coastGuide[j].position.z+=crestSource[j].position.z-ungraded[j].position.z;
                    const Vec3 shift=raw[last].position-coastGuide[last].position;
                    std::vector<double> along(last-first+1);for(size_t j=first+1;j<=last;++j)along[j-first]=along[j-first-1]+norm(coastGuide[j].position-coastGuide[j-1].position);
                    for(size_t j=first;j<=last;++j){raw[j].position=coastGuide[j].position+shift*layoutSmooth(along[j-first]/along.back());raw[j].upHint=coastGuide[j].upHint;raw[j].bank=coastGuide[j].bank;}
                    continue;
                }
                for(size_t j=first;j<=last;++j){raw[j].position.z=crestSource[j].position.z;raw[j].upHint=crestSource[j].upHint;}
                if(holdCorner>=0||graded&&section!=modules.begin()+n){
                    // Reuse the existing descending flank for each section.
                    // Keep the donor's ascending shoulder and crest width; the
                    // extra reservation extends only its powered descent.
                    std::vector<double> planar(right.begin-left.end+1);
                    for(size_t j=left.end+1;j<=right.begin;++j){const Vec3 delta=raw[j].position-raw[j-1].position;planar[j-left.end]=planar[j-left.end-1]+std::hypot(delta.x,delta.y);}
                    const bool heldEntry=left.identity=="fvd-airtime";
                    const double original=planar[crestEnd-left.end],length=planar.back(),shoulder=original*(heldEntry?.16:.125),endShoulder=heldEntry?50:shoulder;
                    const double crest=heldEntry?std::clamp(crestHeight*4,original*.2,original*.3):original*.3,start=(original-crest)*.5;
                    const double ratio=(start+crest*.5-shoulder*.5)/(length-start-crest*.5-endShoulder*.5);
                    auto profile=[&](double s){return shoulder*smoothIntegral(s/shoulder)-(1+ratio)*crest*smoothIntegral((s-start)/crest)+ratio*endShoulder*smoothIntegral((s-length+endShoulder)/endShoulder);};
                    double low=0,high=1;for(int j=0;j<40;++j){double u=(low+high)*.5;if(smooth(u)<1/(1+ratio))low=u;else high=u;}
                    const double peak=start+crest*(low+high)*.5;
                    if(heldEntry){
                        const double requiredRise=std::tan(5.05*pi/180)*profile(peak)/ratio;
                        if(requiredRise>crestHeight+std::max(5.,crestHeight*.1))throw std::runtime_error("Held section descent would raise its donor crest beyond the bounded preservation allowance");
                        crestHeight=std::max(crestHeight,requiredRise);
                    }
                    const double crestSlope=crestHeight/profile(peak),base=crestSource[left.end].position.z;
                    for(size_t j=left.end;j<=right.begin;++j){raw[j].position.z=base+crestSlope*profile(planar[j-left.end]);raw[j].upHint={0,0,1};}
                    for(auto& motor:pending)if(motor.kind==DriveKind::Boost&&motor.begin==section->begin){
                        motor.begin=left.end+size_t(std::lower_bound(planar.begin(),planar.end(),start+crest)-planar.begin());
                        motor.end=left.end+size_t(std::lower_bound(planar.begin(),planar.end(),length-endShoulder)-planar.begin());
                    }
                    continue;
                }
                const double length=distance[crestEnd]-distance[left.end];
                const double shoulder=length*(left.identity=="fvd-airtime"?.16:.125);
                const double crest=left.identity=="record-inversion"?length*.3:std::clamp(crestHeight*4,length*.2,length*.3);
                auto profile=[&](double s){return shoulder*smoothIntegral(s/shoulder)-2*crest*smoothIntegral((s-(length-crest)*.5)/crest)+shoulder*smoothIntegral((s-length+shoulder)/shoulder);};
                const double slope=crestHeight/profile(length*.5),base=crestSource[left.end].position.z;
                for(size_t j=left.end;j<=right.begin;++j){raw[j].position.z=base+slope*profile(std::min(length,distance[j]-distance[left.end]));raw[j].upHint={0,0,1};}
            }
        }else join(std::min(distance[left.end]-leftExtent,distance[leftCrest]),distance[right.begin]+rightExtent);
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
    size_t flyoverBegin=branchBegin[3],crestBegin=branchEnd[3],crestEnd=branchBegin[3];double flyoverRiseEnd=distance[branchBegin[3]];
    for(const auto& run:modules){
        if(run.corridor==2&&run.identity=="corridor-recovery"){flyoverBegin=size_t(std::lower_bound(distance.begin(),distance.end(),distance[run.begin]-110)-distance.begin());flyoverRiseEnd=distance[run.end]+110;}
        if(run.corridor==3&&run.identity=="fvd-airtime"){crestBegin=std::min(crestBegin,run.begin);crestEnd=std::max(crestEnd,run.end);}
    }
    double plateauBegin=distance[crestBegin],plateauEnd=distance[crestEnd],flyoverLift=0;
    for(const auto& crossing:crossings){plateauBegin=std::min(plateauBegin,crossing.along);plateauEnd=std::max(plateauEnd,crossing.along);flyoverLift=std::max(flyoverLift,crossing.lift);}
    ports.flyoverLift=flyoverLift;
    // Keep the calibrated shared datum when a fairer valley or a graded donor
    // changes the crossing below it. Full track/train/structure gates still
    // certify this actual geometry; the 42 m layout allowance is not a limit.
    if(graded)flyoverLift=std::min(flyoverLift,feedback.flyoverLift);
    const double riseSpan=flyoverRiseEnd-distance[flyoverBegin];
    double riseFinish=0;
    if(flyoverLift/(riseSpan-std::min(120.,riseSpan*.25))<std::tan(6.5*pi/180)){
        for(const auto& run:modules)if(run.corridor==2&&(run.identity=="banked-camelback-turn"||run.identity=="sustained-helix"))flyoverBegin=run.begin;
        flyoverRiseEnd=distance[branchBegin[3]];
        // Ease the rise out across the turn's valley bridge into the airtime
        // sequence rather than inside it, so both change pitch together.
        if(graded)for(const auto& bridge:flowJoins)if(bridge.first<branchBegin[3]&&branchBegin[3]<bridge.second){flyoverRiseEnd=distance[bridge.second];riseFinish=distance[bridge.second]-distance[bridge.first];}
    }else if(graded)for(const auto& run:modules)if(run.corridor==2&&run.identity=="banked-camelback-turn"){
        // Ease the booster grade out across the turn's crest rounding, between
        // its steepest authored rise and its crest, so the climb's pitch falls
        // monotonically into that crest. The constant grade ends where it did.
        size_t peak=run.begin,steepest=run.begin;
        for(size_t i=run.begin;i<=run.end;++i)if(raw[i].position.z>raw[peak].position.z)peak=i;
        for(size_t i=run.begin;i<peak;++i)if((raw[i+1].position.z-raw[i].position.z)/(distance[i+1]-distance[i])>(raw[steepest+1].position.z-raw[steepest].position.z)/(distance[steepest+1]-distance[steepest]))steepest=i;
        const double end=(distance[steepest]+distance[peak])*.5;
        riseFinish=end-flyoverRiseEnd+std::min(120.,riseSpan*.25);flyoverRiseEnd=end;
    }
    auto gradedTransition=[](double position,double span,double end=0,double begin=0){
        const double reach=std::min(120.,span*.25),transition=begin>0?begin:reach,finish=end>0?end:reach;
        return (transition*smoothIntegral(position/transition)-finish*smoothIntegral((position-span+finish)/finish))/(span-(transition+finish)*.5);
    };
    // This unpowered return uses the existing height release continuously;
    // there is no motor that requires a concentrated, steep grade here.
    const double fallEnd=graded?distance.back()-std::max(28.,(req.train.cars-1)*req.train.spacing*.5+18):distance[terminalTurnEnd];
    // The last airtime exit flank is the hand-over: the return takes its grade up
    // across that whole flank and the exit rise is spent out of the height the flank
    // already gives up, so the pull-out descends onto the return instead of
    // climbing over a level shelf. The held ride releases 55 m through the same
    // return, a steeper grade than its flank carries, and keeps the plateau start.
    const double handover=.75;
    // The speed-hill run sits clear of both ends of the return: it starts once the
    // last airtime hill has handed over to the descent and stops before the turn.
    const double hopBegin=distance[crestEnd]+40,hopEnd=distance[terminalTurnEnd]-50;
    // Rounded to the nearest whole hop rather than floored: over an 800 m return
    // the floor dropped the count to three, stretching the wavelength to 267 m.
    // A sin^2 hop of height h and wavelength L crests at radius L^2/(2*h*pi^2),
    // so that is a 600 m crest, and at the 50 m/s the train is doing there it
    // lifts only 0.4 g off the rider - a float, not airtime. At 200 m the crest
    // radius is 338 m and the same hop gives the 0.75 g that actually leaves the
    // seat. Height is the wrong lever here: the radius goes as the square of the
    // wavelength and only linearly with height.
    const double hopCount=std::max(1.,std::round((hopEnd-hopBegin)/190)),hopHeight=hopEnd-hopBegin>400?6.:0.;
    const double exitBegin=distance[crestEnd]-forceFlank.back();
    const size_t exitFirst=size_t(std::lower_bound(distance.begin(),distance.end(),exitBegin)-distance.begin());
    const double exitHeight=raw[exitFirst].position.z,exitDrop=exitHeight-raw[crestEnd].position.z;
    const double fallFrom=graded&&!forceHoldEnd?exitBegin:plateauEnd;
    const double fallSpan=fallEnd-fallFrom;
    const double formerFall=distance[terminalTurnEnd]-plateauEnd;
    const double terminalExitRise=graded?std::min(7.,(flyoverLift+(forceHoldEnd?55:0))*(fallSpan-formerFall)/(formerFall-std::min(120.,formerFall*.25))):0;
    for(size_t i=flyoverBegin;i<=branchEnd[3];++i){
        const double s=distance[i],rise=gradedTransition(s-distance[flyoverBegin],flyoverRiseEnd-distance[flyoverBegin],riseFinish);
        // Round into the level bay inside the straight brake, sized by that
        // straight rather than by the whole return it terminates.
        const double fall=1-gradedTransition(s-fallFrom,fallSpan,std::min(120.,(fallEnd-distance[terminalTurnEnd])*.25),fallFrom<plateauEnd?forceFlank.back():0);
        const double spend=fallFrom<plateauEnd?(s<exitBegin?0:smoothIntegral(std::clamp((exitHeight-raw[i].position.z)/exitDrop,0.,1.)/handover)*handover/(1-handover*.5)):layoutSmooth((s-exitBegin)/(plateauEnd-exitBegin));
        // Speed hills on the way home. The return was the ride's longest straight
        // -- a kilometre of level track where the rider felt nothing at all -- so
        // the descent crosses it in hops instead of one ramp. Both the hop and its
        // envelope are sin^2, so height, grade and curvature all reach the ends at
        // zero and the hand-over from the last hill is untouched.
        double hop=0;
        if(graded&&s>hopBegin&&s<hopEnd){
            const double along=(s-hopBegin)/(hopEnd-hopBegin);
            // The envelope holds full height across the run and only eases at its
            // ends: a sin^2 envelope reached a seventh of its height by the first
            // crest and left 350 m of the return doing nothing. layoutSmooth ends
            // with zero slope and zero curvature, so the hops still start silently.
            const double envelope=layoutSmooth(std::min(along,1-along)/.06),phase=along*hopCount;
            hop=hopHeight*envelope*std::pow(std::sin(pi*phase),2);
            // The first hop is an outward-banked hill: its crest leans away from the
            // turn ahead, so the rider is thrown to the outside over the top instead
            // of simply floating. The lean lives only on the crest and returns to
            // level with it. Authored on a non-Turn knot, so force-aligned banking
            // leaves it alone.
            if(phase<1)raw[i].bank=std::copysign(26*pi/180,-plan.angles[sides-1])*std::pow(std::sin(pi*phase),2)*envelope;
            // The last hop is a double-down: one crest, then a drop that pauses and
            // drops again. Pitch only -- no roll anywhere in it.
            if(phase>hopCount-1)hop-=hopHeight*.45*envelope*std::pow(std::sin(pi*(phase-hopCount+1)),2)*smooth(phase-hopCount+1);
        }
        raw[i].position.z+=flyoverLift*std::min(rise,fall)-(forceHoldEnd?55*(1-fall):0)+terminalExitRise*spend*fall+hop;
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
    const double fixedDeparture=std::ceil((200+trainHalf+12)/spacing)*spacing,fixedReturn=stationApproach+50;
    auto fixedStationBoundary=[&](double s){return s<=fixedDeparture||distance.back()-s<=fixedReturn;};
    // The same local flat datum must clear both the boarding bay and the
    // actual fixed launch/return boundary. Bound local platform height at 16 m
    // rather than inheriting the highest terrain elsewhere on the circuit.
    for(int k=0;k<controls;++k)if(fixedStationBoundary(k*spacing))stationDatum=std::max(stationDatum,lower[k]+.3);
    plan.stationChecked=true;plan.stationBayMinimum=stationGroundMin;plan.stationBayMaximum=stationGroundMax;plan.stationRequiredDatum=stationDatum;
    plan.stationFeasible=stationDatum-stationGroundMin<=16;
    if(!plan.stationFeasible)continue;
    // Try a different anchored station placement before increasing geometry scale.
    if(placementVariant){
        if(firstFeasiblePlacement<0){firstFeasiblePlacement=plan.placement;continue;}
        if(plan.placement==firstFeasiblePlacement)continue;
    }
    plan.stationSelectionEligible=true;
    for(int k=0;k<controls;++k){target[k]=weight[k]?target[k]/weight[k]:stationDatum;base[k]=std::max(target[k],lower[k]);fixed[k]=fixedStationBoundary(k*spacing);if(fixed[k]){if(lower[k]>stationDatum+1e-8)throw std::runtime_error("Local station datum cannot clear its fixed launch/boarding boundary");base[k]=stationDatum;}}
    auto wrap=[&](int k){return (k%controls+controls)%controls;};
    // Minimize squared second/third spatial differences with fixed station height
    // and ground-clearance lower bounds.
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
    solveBaseline();
    std::vector<double> fittedBaseline(raw.size());for(size_t i=0;i<raw.size();++i)fittedBaseline[i]=baseline(distance[i]);
    // Keep an authored level motor core genuinely level after ground fitting.
    // C3 shoulders outside the core retain the neighbouring height fields.
    for(const auto& motor:pending)if(motor.kind==DriveKind::Launch||motor.kind==DriveKind::Boost){
        size_t begin=motor.begin;
        auto level=[&](size_t end){if(distance[end]-distance[begin]<70)return;
            double height=-1e9;for(size_t j=begin;j<=end;++j)height=std::max(height,fittedBaseline[j]);
            const double start=distance[begin],finish=distance[end];
            const size_t first=size_t(std::lower_bound(distance.begin(),distance.end(),start-50)-distance.begin());
            const size_t last=size_t(std::lower_bound(distance.begin(),distance.end(),finish+50)-distance.begin());
            for(size_t j=first;j<=std::min(last,unique-1);++j){if(fixedStationBoundary(distance[j]))continue;const double weight=layoutSmooth((distance[j]-start+50)/50)*layoutSmooth((finish+50-distance[j])/50);fittedBaseline[j]+=weight*(height-fittedBaseline[j]);}
        };
        for(size_t j=motor.begin+1;j<=motor.end;++j)if(std::abs(raw[j].position.z-raw[j-1].position.z)>1e-7){level(j-1);begin=j;}
        level(motor.end);
    }
    for(size_t i=0;i<unique;++i)raw[i].position.z+=fittedBaseline[i];raw.back()=raw.front();d.track=compile(raw);
    auto at=[&](size_t index){return index>=d.track.spans.size()?d.track.length:d.track.spans[index].start;};
    for(size_t i=0;i<forceEntryIndices.size();++i)ports.airtimeEntry[i]=at(forceEntryIndices[i]);
    ports.reversalExit=at(reversalExitIndex);ports.reversalSpeedHint=immelmann.exit.speed;ports.terminalEntry=at(terminalTurnEnd);
    for(auto p:pending){
        bool hard=p.kind==DriveKind::Launch;double acc=hard?launchAcceleration:3.5;
        d.operations.push_back({at(p.begin),at(p.end),p.kind,p.speed,req.train.carMass*acc,req.train.carMass*acc*100,hard?.08:.5});
    }
    // A motor occupies a straight, upright corridor with essentially constant
    // grade. Reserve the transition ends for geometry and the whole train.
    coalesceDriveProfiles(d.operations);
    std::vector<Operation> linearDrives;
    for(auto op:d.operations){
        if(op.kind!=DriveKind::Launch&&op.kind!=DriveKind::Boost){linearDrives.push_back(op);continue;}
        const size_t before=linearDrives.size();double begin=-1;
        auto finish=[&](double end){if(begin<0)return;
            Operation segment=op;segment.start=std::max(op.start,begin+2*trainHalf);segment.end=std::min(op.end,end-2*trainHalf);
            if(segment.end-segment.start>op.targetSpeed*op.rampSeconds+2*trainHalf)linearDrives.push_back(segment);
            begin=-1;
        };
        // An operation acts per car: when the first car enters, the rear is
        // a full train length behind; when the rear leaves, the front is ahead.
        const double searchEnd=op.end+2*trainHalf;
        for(double s=op.start-2*trainHalf;s<searchEnd;s+=.125){if(linearPropulsionGeometry(d.track,s,.9,.35)&&linearPropulsionGeometry(d.track,std::min(s+.125,searchEnd),.9,.35)){if(begin<0)begin=s;}else finish(s);}
        finish(searchEnd);
        if(linearDrives.size()!=before+1)throw std::runtime_error("A section booster must occupy one contiguous physical corridor");
    }
    d.operations=std::move(linearDrives);
    // Each booster carries one complete section. Backwards coasting energy
    // includes the whole train's elevation and the existing drag/rolling model.
    auto trainHeight=[&](double s){double z=0;for(int car=0;car<req.train.cars;++car)z+=d.track.sample(s+trainHalf-car*req.train.spacing).position.z/req.train.cars;return z;};
    const double drag=.5*req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass);
    auto coastEntry=[&](double start,double finish,double speed){double squared=speed*speed;const int count=int(std::ceil((finish-start)/2));double right=finish;
        for(int j=1;j<=count;++j){const double left=finish-(finish-start)*j/count,ds=right-left,force=gravity*((trainHeight(right)-trainHeight(left))/ds+req.train.rollingResistance);
            if(drag>0){const double growth=std::exp(2*drag*ds);squared=squared*growth+force/drag*(growth-1);}else squared+=2*force*ds;right=left;}
        return squared;
    };
    const auto inversionEntry=std::find_if(modules.begin(),modules.end(),[](const ModuleRun& run){return run.identity=="high-immelmann";});
    const auto loop=std::find_if(modules.begin(),modules.end(),[](const ModuleRun& run){return run.identity=="record-inversion";});ports.loopExit=at(loop->end);
    for(const auto& run:modules){
        const bool postLoop=run.identity=="post-loop-section-boost",heldHelix=run.identity=="helix-section-boost",returning=run.corridor==2&&run.identity=="corridor-recovery";
        if(!postLoop&&!heldHelix&&!returning)continue;
        auto motor=std::find_if(d.operations.begin(),d.operations.end(),[&](const Operation& op){return op.kind==DriveKind::Boost&&((postLoop||heldHelix)?op.end>=at(run.begin):op.start>=at(run.begin))&&op.end<=at(run.end);});
        if(motor==d.operations.end())throw std::runtime_error("Section has no contiguous linear booster");
        const double goal=returning?at(branchBegin[3]):postLoop&&(graded||forceHoldEnd)?ports.airtimeEntry[0]:at(inversionEntry->begin);
        const double goalSpeed=returning?65:postLoop&&forceHoldEnd?60:graded&&postLoop?std::sqrt(coastEntry(ports.airtimeEntry[0],at(inversionEntry->begin),reversalEntrySpeed)):reversalEntrySpeed;
        const double entry=motor->start-trainHalf,exit=motor->end+trainHalf;
        const double entrySpeed=returning?feedback.recoveryEntrySpeed:heldHelix?feedback.helixEntrySpeed:graded?std::sqrt(std::max(0.,(feedback.loopExitSpeed*feedback.loopExitSpeed-coastEntry(ports.loopExit,entry,0))*std::exp(-2*drag*(entry-ports.loopExit)))):feedback.postLoopEntrySpeed;
        if(returning)ports.recoveryEntry=entry;else if(heldHelix)ports.helixEntry=entry;else ports.postLoopEntry=entry;
        auto sectionEntry=[&](double start){return std::sqrt(std::max(0.,coastEntry(start,goal,goalSpeed)));};
        motor->targetSpeed=sectionEntry(exit);
        const double usefulLength=motor->end-motor->start-motor->targetSpeed*motor->rampSeconds;
        const double requiredEntrySpeed=sectionEntry(entry);
        const double acceleration=std::clamp((requiredEntrySpeed*requiredEntrySpeed-entrySpeed*entrySpeed)/(2*usefulLength),3.5,std::min(20.,req.limits.maxLongitudinalG*gravity-.02));
        motor->maxForce=req.train.carMass*acceleration;motor->maxPower=motor->maxForce*100;
    }
    const double brakingStart=at(terminalTurnEnd)+2*trainHalf;
    const double terminalBrakeAcceleration=terminalDeceleration+1;
    d.operations.push_back({brakingStart,80,DriveKind::Station,0,req.train.carMass*terminalBrakeAcceleration,req.train.carMass*terminalBrakeAcceleration*100,.5,terminalDeceleration,1.5});
    for(auto& operation:d.operations)operation.exitFadeMeters=std::max(1.,operation.targetSpeed*operation.rampSeconds);
    coalesceDriveProfiles(d.operations);
    if(reversingPair)d.topology+="/immelmann-low-return";
    if(!flowJoins.empty())d.topology+="/continuous-module-joins";
    if(!pacingCrests.empty())d.topology+="/paced-connectors";
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
    d.planningDiagnostics.pop_back();std::ostringstream expansion;expansion<<std::setprecision(12)<<",\"layoutExpansion\":{\"reversingPair\":"<<(reversingPair?"true":"false")<<",\"hillProfileShape\":"<<hillProfileShape<<",\"loopProfileShape\":"<<loopProfileShape;
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
    // Smooth banking over a 1.2-second travel window on each side.
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
            Design d=candidate(req,i,cancel,feedback,ports,false);
            if(progress)progress(i,"Simulating initial geometry");
            auto motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel);
            if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            if(motion.completed){
                // Rebuild geometry and drive zones once using measured train entry energy.
                // Keep the first FVD source shape; section entry speed is certified by the full replay.
                for(size_t h=1;h<feedback.airtimeSpeed.size();++h)feedback.airtimeSpeed[h]=replayValueAt(motion.frames,ports.airtimeEntry[h]);
                feedback.postLoopEntrySpeed=replayValueAt(motion.frames,ports.postLoopEntry);
                feedback.loopExitSpeed=replayValueAt(motion.frames,ports.loopExit);
                feedback.flyoverLift=ports.flyoverLift;
                feedback.speedCorrection=req.targets.speed-std::max_element(motion.frames.begin(),motion.frames.end(),
                    [](const Frame& a,const Frame& b){return a.speed<b.speed;})->speed;
                feedback.recoveryEntrySpeed=replayValueAt(motion.frames,ports.recoveryEntry);
                feedback.terminalEntrySpeed=replayValueAt(motion.frames,ports.terminalEntry);
                if(ports.helixEntry)feedback.helixEntrySpeed=replayValueAt(motion.frames,ports.helixEntry);
                double exitSpeed=replayValueAt(motion.frames,ports.reversalExit);
                feedback.reversalEnergyCorrection=ports.reversalSpeedHint*ports.reversalSpeedHint-exitSpeed*exitSpeed;
                d=candidate(req,i,cancel,feedback,ports);
                if(progress)progress(i,"Simulating energy-corrected geometry");
                motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel);
                if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            }
            if(motion.completed)improveBanking(d,motion.frames);
            const double trainSpan=(req.train.cars-1)*req.train.spacing;
            for(const auto& operation:d.operations)if(operation.kind==DriveKind::Launch||operation.kind==DriveKind::Boost){
                const auto first=d.track.sample(operation.start-trainSpan);
                const double heading=std::atan2(first.tangent.y,first.tangent.x);
                double minimumPitch=std::asin(first.tangent.z),maximumPitch=minimumPitch;
                auto checkMotorGeometry=[&](double s){
                    const auto p=d.track.sample(s);minimumPitch=std::min(minimumPitch,std::asin(p.tangent.z));maximumPitch=std::max(maximumPitch,std::asin(p.tangent.z));
                    if(!linearPropulsionGeometry(d.track,s)||std::abs(std::remainder(std::atan2(p.tangent.y,p.tangent.x)-heading,2*pi))>.001||maximumPitch-minimumPitch>.005)
                        throw std::runtime_error("Final banked geometry entered a whole-train propulsion corridor at "+std::to_string(s));
                };
                for(double s=operation.start-trainSpan;s<operation.end+trainSpan;s+=.125)checkMotorGeometry(s);
                checkMotorGeometry(operation.end+trainSpan);
                for(const auto& span:d.track.spans)if(span.start>=operation.start-trainSpan&&span.start<=operation.end+trainSpan)checkMotorGeometry(span.start);
                const bool level=std::max(std::abs(minimumPitch),std::abs(maximumPitch))<=.05*pi/180;
                // About five degrees is the visual criterion. Datum fitting
                // may perturb the authored grade by a tenth of a degree.
                const bool visibleIncline=minimumPitch>=4.9*pi/180||maximumPitch<=-4.9*pi/180;
                if(!level&&!visibleIncline)throw std::runtime_error("Section booster is neither level nor visibly inclined: "+std::to_string(operation.start)+" pitch "+std::to_string(minimumPitch*180/pi)+" to "+std::to_string(maximumPitch*180/pi));
            }
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
                // The straight stopping approach moves the Station boundary.
                // A fully validated ride inside the requested physical-stop
                // budget does not need another pacing-search candidate.
                if(d.simulation.metrics.duration<=200)return withHistory(std::move(d),"accepted-within-final-stop-budget");
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
