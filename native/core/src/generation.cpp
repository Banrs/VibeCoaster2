#include "coaster/coaster.hpp"
#include "coaster/fvd.hpp"
#include "flow_bridge.hpp"
#include "simulation_internal.hpp"
#include <future>
#include <mutex>
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
// Find the smallest acceleration that meets the requested 0-180 km/h time.
static double sizedLaunchAcceleration(const GenerationRequest& req){
    double upper=req.limits.maxLongitudinalG*gravity-.02,lower=0;
    for(int i=0;i<32;++i){double mid=(lower+upper)*.5;if(plannedLaunchTime(mid,req.train)>req.targets.launchSeconds-.003)lower=mid;else upper=mid;}
    return std::min(req.limits.maxLongitudinalG*gravity-.02,upper);
}
static double stationBankFactor(double remaining,const TrainConfig& train){double upright=std::max(40.,(train.cars-1)*train.spacing*.5+18);return smooth((remaining-upright)/100);}
static double layoutSmooth(double u){u=std::clamp(u,0.,1.);return u*u*u*u*(35+u*(-84+u*(70-20*u)));}
static Vec3 inFrame(Vec3 p,double h){return {p.x*std::cos(h)-p.y*std::sin(h),p.x*std::sin(h)+p.y*std::cos(h),p.z};}
// Only the powered car must align with the motor: 1.275 m half-body plus
// registration margin. Other cars can already occupy the adjacent transition.
static constexpr double motorAlignmentMargin=1.5;
static bool linearPropulsionGeometry(const Track& track,double s,double margin=1,double planMargin=1){
    const auto k=sampleKinematics(track,s);const auto& p=k.sample;
    const Vec3 upright=unit(Vec3{0,0,1}-p.tangent*p.tangent.z);
    // Near-constant grade, negligible plan curvature and an upright frame.
    return std::hypot(p.tangent.x,p.tangent.y)>.5&&std::abs(cross(p.tangent,p.curvature).z)<planMargin*1e-5&&std::abs(p.curvature.z)<margin*1e-4&&dot(p.up,upright)>.9998&&norm(k.upS)<margin*.001;
}

struct AuthoringFeedback {double reversalEnergyCorrection{},speedCorrection{};};
struct CandidatePorts {double reversalExit{},reversalSpeedHint{};};
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
static Design candidate(const GenerationRequest& req,int attempt,Cancel cancel,const AuthoringFeedback& feedback,CandidatePorts& ports,bool corrected=true){
    Design d;d.request=req;d.candidate=attempt;Random rng{req.seed};
    const double scale=1+attempt*.035,variation=rng.range(.98,1.02);
    const double datum=req.limits.minClearance+5,elevation=req.targets.height+10;
    const double hillLength=(920+std::max(0.,elevation-230)*3)*scale*variation;
    const double loopHeight=req.targets.inversionHeight+10,loopDrift=350*scale;
    const double topSpeed=std::max(req.targets.speed+(corrected?feedback.speedCorrection:0),std::sqrt(2*gravity*elevation+40*40));
    const double launchAcceleration=sizedLaunchAcceleration(req),halfTrain=(req.train.cars-1)*req.train.spacing*.5;
    std::vector<AuthoredPoint> raw;
    struct Module {size_t begin,end;std::string name;};std::vector<Module> modules;
    struct Motor {size_t begin,end;DriveKind kind;double speed,acceleration;};std::vector<Motor> motors;
    Vec3 cursor{0,0,datum};double heading=0,curvature=0;
    auto append=[&](Vec3 p,Element e,Vec3 up=Vec3{0,0,1}){if((raw.size()&255)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");if(raw.empty()||norm(p-raw.back().position)>1e-7)raw.push_back({p,0,e,up});};
    append(cursor,Element::Station);
    // Heading and grade belong to the route. Named elements are intervals on
    // that route, not independent straight blocks with neutral end ports.
    auto sweep=[&](double length,double angle,double endCurvature,double rise,double endHeight,Element e,const char* name){
        const size_t begin=raw.size()-1;const double startZ=cursor.z,startHeading=heading;
        const auto yaw=detail::flowFramePolynomial(heading,curvature,0.,0.,heading+angle,endCurvature,0.,0.,length);
        const int count=int(std::ceil(length/1.2));const double ds=length/count;
        for(int i=1;i<=count;++i){double u=double(i)/count,a=detail::flowValue(yaw,(i-.5)/count);
            cursor.x+=ds*std::cos(a);cursor.y+=ds*std::sin(a);
            cursor.z=startZ+endHeight*layoutSmooth(u)+rise*std::pow(std::sin(pi*u),4);
            append(cursor,e);
        }
        heading=startHeading+angle;curvature=endCurvature;modules.push_back({begin,raw.size()-1,name});
        return std::pair{begin,raw.size()-1};
    };
    auto drive=[&](double length,DriveKind kind,double speed,double acceleration,const char* name){
        auto range=sweep(length,0,0,0,0,kind==DriveKind::Brake?Element::Brake:Element::Launch,name);
        motors.push_back({range.first,range.second,kind,speed,acceleration});return range;
    };
    auto curveTo=[&](Vec3 end,double endHeading,double endCurvature,double length,double crest,const char* name){
        const size_t begin=raw.size()-1;TrackKinematics a{},b{};
        a.sample.position=cursor;a.sample.tangent={std::cos(heading),std::sin(heading),0};a.sample.curvature={-std::sin(heading)*curvature,std::cos(heading)*curvature,0};a.curvatureS=a.sample.tangent*(-curvature*curvature);
        b.sample.position=end;b.sample.tangent={std::cos(endHeading),std::sin(endHeading),0};b.sample.curvature={-std::sin(endHeading)*endCurvature,std::cos(endHeading)*endCurvature,0};b.curvatureS=b.sample.tangent*(-endCurvature*endCurvature);
        const auto polynomial=detail::flowBridgePolynomial(a,b,length);const int count=int(std::ceil(length/1.2));
        for(int i=1;i<=count;++i){const double u=double(i)/count;auto p=detail::flowValue(polynomial,u);p.z+=crest*std::pow(std::sin(pi*u),4);append(p,Element::Turn);}
        cursor=end;heading=endHeading;curvature=endCurvature;modules.push_back({begin,raw.size()-1,name});
    };
    sweep(20,0,0,0,0,Element::Station,"station");
    drive(180,DriveKind::Launch,topSpeed,launchAcceleration,"departure-launch");
    sweep(hillLength,55*pi/180,.001,elevation-9,18,Element::Hill,"turning-record-hill");
    double heldRise=0;
    if(req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure)&&req.targets.referenceExposure*1.1>25){
        // An explicit sustained-force request gets one climbing spiral. Its
        // elevation separates the repeated heading; the following sweep spends
        // that height continuously on the way into the inversion.
        heldRise=30;const double radius=100*scale,ramp=130,length=2*pi*radius+ramp;
        const size_t begin=raw.size()-1;const double startHeading=heading,startHeight=cursor.z;
        auto integral=[](double u){return std::pow(u,5)*(7+u*(-14+u*(10-2.5*u)));};
        auto yaw=[&](double s){if(s<ramp)return ramp/radius*integral(s/ramp);if(s>length-ramp)return 2*pi-ramp/radius*integral((length-s)/ramp);return (s-ramp*.5)/radius;};
        const int count=int(std::ceil(length/1.2));const double step=length/count;
        for(int i=1;i<=count;++i){const double h=startHeading+yaw((i-.5)*step);cursor.x+=step*std::cos(h);cursor.y+=step*std::sin(h);cursor.z=startHeight+heldRise*layoutSmooth(double(i)/count);append(cursor,Element::Turn);}
        heading+=2*pi;curvature=0;modules.push_back({begin,raw.size()-1,"climbing-force-spiral"});
    }
    const auto descent=sweep(600*scale,40*pi/180,0,0,-18-heldRise,Element::Turn,"descending-sweep");

    FvdImmelmannRequest inversion;const double inversionScale=std::sqrt((req.targets.inversionHeight+14)/95.);
    inversion.height=95*inversionScale*inversionScale;inversion.exitHeight=10*inversionScale*inversionScale;
    inversion.entrySpeed=std::sqrt(53*53*inversionScale*inversionScale+std::min(0.,feedback.reversalEnergyCorrection));
    inversion.rampSeconds=1.2*inversionScale;inversion.rollOverlapFraction=.5;inversion.hand=-1;
    inversion.rollingAcceleration=gravity*req.train.rollingResistance;
    inversion.dragAccelerationCoefficient=.5*req.train.airDensity*req.train.dragCdA/(req.train.cars*req.train.carMass);
    const auto immelmann=designFvdImmelmann(inversion,cancel);
    if(!immelmann.section.report.valid()||!immelmann.section.assessment.passed)throw std::runtime_error("Immelmann source failed its physical replay");
    const size_t trimBegin=descent.second-size_t(210/1.2);
    motors.push_back({trimBegin,descent.second,DriveKind::Brake,inversion.entrySpeed,5});
    const auto inversionBegin=raw.size()-1;const Vec3 inversionBase=cursor;
    const int inversionSamples=int(std::ceil(immelmann.section.track.length/1.2));
    for(int i=1;i<=inversionSamples;++i){auto q=immelmann.section.track.sample(immelmann.section.track.length*i/inversionSamples);append(inversionBase+inFrame(q.position,heading),Element::Inversion,inFrame(q.up,heading));}
    cursor=raw.back().position;heading+=std::atan2(immelmann.exit.forward.y,immelmann.exit.forward.x);curvature=0;
    modules.push_back({inversionBegin,raw.size()-1,"high-immelmann"});const size_t reversalEnd=raw.size()-1;
    sweep(540*scale,-70*pi/180,0,12,-inversion.exitHeight,Element::Turn,"diving-reversal");
    const double loopEntrySpeed=std::max(46.,req.targets.speed*.63);
    drive(120,DriveKind::Boost,loopEntrySpeed,8,"loop-entry-boost");
    const size_t loopBegin=raw.size()-1;const Vec3 loopBase=cursor;
    const double amplitude=(loopDrift+std::sqrt(4*pi*pi*loopHeight*25))/(2*pi),lane=36;
    auto loopPoint=[&](double u){const double x=loopDrift*u+amplitude*std::sin(2*pi*u);return Vec3{x,lane*(layoutSmooth(u)-(u>.5?layoutSmooth((x-loopDrift+120)/220):0)),loopHeight*std::pow(std::sin(pi*u),4)};};
    for(int i=1;i<=750;++i){double u=double(i)/750;Vec3 p=loopPoint(u),t=unit(loopPoint(std::min(1.,u+1e-5))-loopPoint(std::max(0.,u-1e-5)));append(loopBase+inFrame(p,heading),Element::Inversion,inFrame(unit(cross(Vec3{0,-1,0},t)),heading));}
    cursor=raw.back().position;modules.push_back({loopBegin,raw.size()-1,"record-inversion"});
    const size_t sBegin=raw.size()-1;const Vec3 sBase=cursor;
    for(int i=1;i<=84;++i){double u=double(i)/84;append(sBase+inFrame({100*u,-lane*(layoutSmooth((120+100*u)/220)-layoutSmooth(120./220)),0},heading),Element::Turn);}
    cursor=raw.back().position;modules.push_back({sBegin,raw.size()-1,"loop-exit-s"});
    drive(240,DriveKind::Boost,65,7,"section-boost");
    // The finish sweeps around the inside of the outbound route. Its one
    // broad airtime crest is carried through the turn, with no elevated shelf.
    const double approach=210;
    const size_t finishBegin=raw.size()-1;
    const Vec3 returnShoulder{-430*scale,220*scale,datum};
    curveTo(returnShoulder,-pi/2,.003,1.39*norm(returnShoulder-cursor),0,"airtime-return-sweep");
    curveTo({-approach,0,datum},0,0,350*scale,0,"low-return-turn");
    double finishLength=0;for(size_t i=finishBegin+1;i<raw.size();++i)finishLength+=norm(raw[i].position-raw[i-1].position);
    double alongFinish=0;Vec3 previous=raw[finishBegin].position;
    for(size_t i=finishBegin+1;i<raw.size();++i){const auto p=raw[i].position;alongFinish+=norm(p-previous);previous=p;const double wave=std::sin(pi*alongFinish/finishLength);raw[i].position.z+=35*std::pow(wave,4);raw[i].bank=35*pi/180*std::pow(wave,6);raw[i].element=Element::Airtime;}
    const size_t brakeBegin=raw.size()-1;
    sweep(approach,0,0,0,0,Element::Brake,"station-approach");
    raw.back()=raw.front();

    std::vector<double> distance(raw.size());
    auto measure=[&]{for(size_t i=1;i<raw.size();++i)distance[i]=distance[i-1]+norm(raw[i].position-raw[i-1].position);};measure();
    // Carry position, pitch, curvature and orientation through shared windows.
    // A hardware straight has exactly one boundary; there is no settling line.
    std::vector<std::pair<size_t,size_t>> joins;
    for(size_t m=1;m+1<modules.size();++m){const auto& left=modules[m];const auto& right=modules[m+1];
        if(left.name=="departure-launch"||left.name=="section-boost"||right.name=="section-boost"||left.name=="loop-entry-boost"||right.name=="loop-entry-boost"||right.name=="station-approach"||left.name=="record-inversion"||left.name=="loop-exit-s")continue;
        const double extent=std::min({120.,(distance[left.end]-distance[left.begin])*.24,(distance[right.end]-distance[right.begin])*.24});
        const size_t first=size_t(std::lower_bound(distance.begin(),distance.end(),distance[left.end]-extent)-distance.begin()),last=size_t(std::lower_bound(distance.begin(),distance.end(),distance[right.begin]+extent)-distance.begin());
        if(first>=4&&last+5<raw.size()){try{detail::blendAuthoredJoin(raw,first,last,cancel);}catch(const std::exception& e){throw std::runtime_error(left.name+" -> "+right.name+": "+e.what());}joins.push_back({first,last});}
    }
    measure();d.track=compile(raw);
    auto at=[&](size_t i){return i>=d.track.spans.size()?d.track.length:d.track.spans[i].start;};
    for(const auto& motor:motors){const bool powered=motor.kind!=DriveKind::Brake;const double ramp=motor.kind==DriveKind::Launch?.08:.5;
        d.operations.push_back({at(motor.begin)+(powered?3:0),at(motor.end)-(powered?3:0),motor.kind,motor.speed,req.train.carMass*motor.acceleration,req.train.carMass*motor.acceleration*100,ramp});}
    d.operations.push_back({at(brakeBegin)+2*halfTrain,80,DriveKind::Station,0,req.train.carMass*7,req.train.carMass*700,.5,6,1.5});
    for(auto& op:d.operations)op.exitFadeMeters=std::max(1.,op.targetSpeed*op.rampSeconds);
    ports.reversalExit=at(reversalEnd);ports.reversalSpeedHint=immelmann.exit.speed;
    d.topology="sweeping-flight/turning-summit/immelmann-reversal/loop-S/airtime-return";
    std::ostringstream diagnostic;diagnostic<<std::setprecision(12)<<"{\"schemaVersion\":2,\"generationOnly\":true,\"seed\":"<<req.seed<<",\"candidate\":"<<attempt<<",\"canonicalLength\":"<<d.track.length<<",\"modules\":[";
    for(size_t i=0;i<modules.size();++i){if(i)diagnostic<<',';const auto& m=modules[i];diagnostic<<"{\"identity\":\""<<m.name<<"\",\"start\":"<<at(m.begin)<<",\"end\":"<<at(m.end)<<'}';}
    diagnostic<<"],\"flowJoins\":[";for(size_t i=0;i<joins.size();++i){if(i)diagnostic<<',';diagnostic<<"{\"start\":"<<at(joins[i].first)<<",\"end\":"<<at(joins[i].second)<<'}';}diagnostic<<"]}";d.planningDiagnostics=diagnostic.str();return d;
}

static void improveBanking(Design& d,const std::vector<Frame>& frames){
    if(frames.empty())return;
    const size_t count=d.track.spans.size();
    std::vector<double> target(count),along(count),speed(count),freedom(count);
    for(size_t i=0;i<count;++i){const auto& k=d.track.knots[i];along[i]=d.track.spans[i].start;speed[i]=replayValueAt(frames,along[i]);
        // The inversion's upright entry/exit participates in the same bank
        // field as its neighbours. Preserve its authored inverted/vertical body.
        freedom[i]=layoutSmooth((k.up.z-.25)/.65)*layoutSmooth((.8-std::abs(k.tangent.z))/.4);
        const Vec3 required=k.curvature*(speed[i]*speed[i])+Vec3{0,0,gravity};
        const double normal=dot(required,k.up),lateral=dot(required,cross(k.tangent,k.up)),load=norm(required);
        double angle=std::atan2(lateral,std::max(normal,.65*gravity));
        // A small residual lateral load gives gentle banking on broad curves.
        // Stronger turns retain the bank needed by the unchanged force envelope.
        angle=std::copysign(std::max(0.,std::abs(angle)-std::atan2(.45*gravity,load)),angle);
        const double concession=std::asin(std::clamp(d.request.limits.maxLateralG*gravity*.75/std::max(load,1e-6),0.,1.));
        const double ceiling=std::max(65*pi/180,std::abs(std::atan2(lateral,normal))-concession);
        angle=std::clamp(angle,-ceiling,ceiling);
        if(k.element==Element::Airtime){const double relief=layoutSmooth((2-load/gravity)/.75),weight=std::clamp(std::abs(k.bank)/(35*pi/180),0.,1.)*relief;angle=angle*(1-weight)+k.bank*relief;}
        target[i]=freedom[i]*angle+(1-freedom[i])*k.bank;
    }
    for(size_t i=0;i<count;++i){auto& k=d.track.knots[i];double weighted=0,total=0,radius=std::max(1.,speed[i]*1.5);
        size_t first=i;while(first&&along[i]-along[first-1]<radius)--first;
        for(size_t j=first;j<count&&along[j]-along[i]<radius;++j){const double u=(along[j]-along[i])/radius,w=std::pow(std::max(0.,1-u*u),4)*d.track.spans[j].length;weighted+=w*target[j];total+=w;}
        double fade=layoutSmooth(along[i]/60)*stationBankFactor(d.track.length-along[i],d.request.train);
        for(const auto& op:d.operations)if(op.kind==DriveKind::Launch||op.kind==DriveKind::Boost){
            const double gap=along[i]<op.start?op.start-along[i]:along[i]>op.end?along[i]-op.end:0;
            fade*=layoutSmooth((gap-motorAlignmentMargin-2)/radius);
        }
        for(const auto& op:d.operations)if(op.kind==DriveKind::Station)
            fade*=layoutSmooth((op.start-(d.request.train.cars-1)*d.request.train.spacing-along[i])/radius);
        k.bank=fade*(freedom[i]*weighted/total+(1-freedom[i])*k.bank);
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
        if(progress)progress(i,"Designing continuous route and shared transitions");
        try{
            AuthoringFeedback feedback;CandidatePorts ports;
            Design d=candidate(req,i,cancel,feedback,ports,false);
            if(progress)progress(i,"Simulating initial geometry");
            auto motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel);
            if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            if(motion.completed){
                feedback.speedCorrection=req.targets.speed-std::max_element(motion.frames.begin(),motion.frames.end(),
                    [](const Frame& a,const Frame& b){return a.speed<b.speed;})->speed;
                double exitSpeed=replayValueAt(motion.frames,ports.reversalExit);
                feedback.reversalEnergyCorrection=ports.reversalSpeedHint*ports.reversalSpeedHint-exitSpeed*exitSpeed;
                d=candidate(req,i,cancel,feedback,ports);
                if(progress)progress(i,"Simulating energy-corrected geometry");
                motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel);
                if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            }
            if(motion.completed)improveBanking(d,motion.frames);
            for(const auto& operation:d.operations)if(operation.kind==DriveKind::Launch||operation.kind==DriveKind::Boost){
                const auto first=d.track.sample(operation.start-motorAlignmentMargin);
                const double heading=std::atan2(first.tangent.y,first.tangent.x);
                double minimumPitch=std::asin(first.tangent.z),maximumPitch=minimumPitch;
                auto checkMotorGeometry=[&](double s){
                    const auto p=d.track.sample(s);minimumPitch=std::min(minimumPitch,std::asin(p.tangent.z));maximumPitch=std::max(maximumPitch,std::asin(p.tangent.z));
                    if(!linearPropulsionGeometry(d.track,s)||std::abs(std::remainder(std::atan2(p.tangent.y,p.tangent.x)-heading,2*pi))>.001||maximumPitch-minimumPitch>.005)
                        throw std::runtime_error("Final banked geometry entered a powered car's alignment corridor at "+std::to_string(s));
                };
                for(double s=operation.start-motorAlignmentMargin;s<operation.end+motorAlignmentMargin;s+=.125)checkMotorGeometry(s);
                checkMotorGeometry(operation.end+motorAlignmentMargin);
                for(const auto& span:d.track.spans)if(span.start>=operation.start-motorAlignmentMargin&&span.start<=operation.end+motorAlignmentMargin)checkMotorGeometry(span.start);
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
            buildSupportLayout(d,cancel);
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
