#include "recipe_compiler.hpp"
#include "authoring.hpp"
#include "launch_camelback.hpp"
#include "simulation_internal.hpp"
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace coaster {
namespace {
using detail::MotionJet;
using detail::AngleJet;
Vec3 rotated(Vec3 p,double heading){return {p.x*std::cos(heading)-p.y*std::sin(heading),p.x*std::sin(heading)+p.y*std::cos(heading),p.z};}
MotionJet rotated(MotionJet p,double heading){p.position=rotated(p.position,heading);p.tangent=rotated(p.tangent,heading);p.curvature=rotated(p.curvature,heading);p.third=rotated(p.third,heading);p.fourth=rotated(p.fourth,heading);return p;}
double seeded(uint64_t seed,double low,double high){return low+(high-low)*double(seed>>11)*0x1.0p-53;}
double accelerationFor(const GenerationRequest& req){
    double low=0,high=req.limits.maxLongitudinalG*gravity-.02;
    for(int i=0;i<32;++i){const double mid=(low+high)*.5;if(plannedDepartureSeconds(mid,req.train,req.limits)>req.targets.launchSeconds-.003)low=mid;else high=mid;}
    return high;
}
struct CompiledElement {size_t moduleBegin{},moduleEnd{};RideRole role;std::string id;};
}

double departureRamp(double acceleration,const Limits& limits){
    return std::isfinite(limits.maxLongitudinalRateGps)?std::max(.08,1.875*acceleration/(gravity*limits.maxLongitudinalRateGps)*1.02):.43;
}
double plannedDepartureSeconds(double motorAcceleration,const TrainConfig& train,const Limits& limits){
    constexpr double dt=.0005;double speed=0,time=0;
    const double drag=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass),ramp=departureRamp(motorAcceleration,limits);
    auto acceleration=[&](double v,double t){const double external=motorAcceleration*smooth(t/ramp)-drag*v*v,friction=gravity*train.rollingResistance;return v>0?external-friction:std::max(0.,external-friction);};
    while(time<10){const double a=acceleration(speed,time),mid=std::max(0.,speed+a*dt*.5),next=std::max(0.,speed+acceleration(mid,time+dt*.5)*dt);if(next>=50)return time+dt*(50-speed)/(next-speed);speed=next;time+=dt;}
    return std::numeric_limits<double>::infinity();
}

Design compileRecipe(const GenerationRequest& input,int attempt,const RecipeFeedback& feedback,std::vector<RecipePort>& ports,Cancel cancel,RecipeCompileMode mode){
    if(mode!=RecipeCompileMode::ClosedCircuit&&mode!=RecipeCompileMode::EnergyCalibrationPrefix)throw std::invalid_argument("Unknown recipe compile mode" );
    const bool energyPrefix=mode==RecipeCompileMode::EnergyCalibrationPrefix;
    ports.clear();
    Design d;d.request=input;d.candidate=attempt;
    if(d.request.recipe.elements.empty())d.request.recipe=defaultRideRecipe();
    if(d.request.style.returnStyle<0)d.request.style.returnStyle=int(elementSeed(input.seed,"return-style")&1);
    d.topology="recipe/"+d.request.recipe.name;
    std::string error;if(!validateRecipe(d.request.recipe,error))throw std::runtime_error(error);
    const auto& req=d.request;MotionBuilder motion(d,cancel);auto& cursor=motion.cursor;
    const double datum=motion.datum,halfTrain=(req.train.cars-1)*req.train.spacing*.5;
    const double hand=(elementSeed(req.seed,"landscape-hand")&1)?1.:-1.;
    std::vector<CompiledElement> compiled;std::vector<std::pair<std::string,size_t>> sourcePorts;
    std::vector<std::pair<LandmarkKind,size_t>> landmarks;
    std::vector<double> sourceSpeeds;size_t brakeBegin=0;
    Vec3 plateauArrival{},plateauCenter{},cliffTop{},cliffFoot{},waveBase{},loopBase{},immelExit{},outbankBay{},camelbackBase{},returnRavine{};
    double cliffYaw=0,brakeCapacity=7,brakeAlignment=0;
    auto heading=[&]{return std::atan2(cursor.tangent.y,cursor.tangent.x);};
    struct PendingPort {std::string id;size_t knot{};double speed{};};
    std::optional<PendingPort> pendingPort;
    auto speedFor=[&](const std::string& id){
        const auto found=feedback.energyCorrection.find(id);
        const double speed=std::sqrt(std::max(25.,motion.nominalSpeed*motion.nominalSpeed+(found==feedback.energyCorrection.end()?0:found->second)));
        pendingPort=PendingPort{id,d.track.knots.size()-1,speed};return speed;
    };
    // Only fully authored motor ranges and reached source ports are materialized.
    // The same definitions serve successful prefixes and diagnostic failures.
    auto finalizeKnownPrefix=[&](bool station){
        d.operations.clear();d.sections.clear();d.landmarks.clear();ports.clear();
        d.track.rebuild();const size_t last=d.track.knots.size()-1;
        auto at=[&](size_t index){return index>=d.track.spans.size()?d.track.length:d.track.spans[index].start;};
        for(const auto& motor:motion.motors)if(motor.begin<motor.end&&motor.end<=last){
            const double ramp=motor.kind==DriveKind::Launch?departureRamp(motor.acceleration,req.limits):.65;
            d.operations.push_back({at(motor.begin)+(motor.kind==DriveKind::Launch?3:2*halfTrain+3),at(motor.end)-2*halfTrain-3,motor.kind,motor.speed,req.train.carMass*motor.acceleration,req.train.carMass*motor.acceleration*110,ramp});
        }
        if(station)d.operations.push_back({at(brakeBegin)+2*halfTrain+3,80,DriveKind::Station,0,req.train.carMass*brakeCapacity,req.train.carMass*brakeCapacity*100,.5,std::min(6.,brakeCapacity*6/7),1.5});
        for(auto& operation:d.operations)operation.exitFadeMeters=std::max(1.,operation.targetSpeed*operation.rampSeconds);
        for(const auto& element:compiled)for(size_t i=element.moduleBegin;i<element.moduleEnd;++i){const auto& m=motion.modules[i];
            if(m.begin<last&&m.end>m.begin)d.sections.push_back({m.name,at(m.begin),at(std::min(m.end,last)),m.reversals,m.planar,element.role,element.id});
        }
        for(const auto& [kind,knot]:landmarks)if(knot<=last)d.landmarks.push_back({kind,at(knot)});
        if(station)d.landmarks.push_back({LandmarkKind::BrakeEntry,at(brakeBegin)});
        for(size_t i=0;i<sourcePorts.size();++i)if(sourcePorts[i].second<=last)
            ports.push_back({sourcePorts[i].first,at(sourcePorts[i].second),sourceSpeeds[i]});
    };
    auto lossFor=[&](auto& intent){intent.rollingAcceleration=gravity*req.train.rollingResistance;intent.dragAccelerationCoefficient=motion.drag;};
    auto freeDirection=[&](AngleJet endPitch,AngleJet endYaw,double length,Element kind,const std::string& name,double bank=0.){
        const auto incoming=detail::directionAngles(cursor);
        endYaw.value=incoming[1].value+std::remainder(endYaw.value-incoming[1].value,2*pi);
        const auto hint=polynomialMotion(cursor,detail::anglePolynomial(incoming[0],endPitch,length),detail::anglePolynomial(incoming[1],endYaw,length),length);
        return motion.programme(hint,kind,name.c_str(),bank);
    };
    // Two broad septic pitch phases share one ordered crest or trough. Their
    // total length and height are fixed, and the complete endpoint jets remain
    // inherited. Exact polynomials avoid concentrating a curvature release in
    // the endpoint-clustered B-spline controls. Horizontal placement stays free.
    auto terrainCurve=[&](MotionJet end,double length,Element kind,const std::string& name,double bank=0.){
        const auto first=detail::directionAngles(cursor),last=detail::directionAngles(end);
        const double yaw=first[1].value+std::remainder(last[1].value-first[1].value,2*pi);
        const AngleJet endYaw{yaw,last[1].first,last[1].second,last[1].third};
        const auto yawPolynomial=detail::anglePolynomial(first[1],endYaw,length);
        const double target=end.position.z,sign=target>=cursor.position.z?1.:-1.;
        std::optional<std::pair<MotionProgram,MotionProgram>> selected;
        // Unequal phases let inherited crest curvature release over a physical
        // distance without adding a shoulder to a shallow terrain-following act.
        for(double fraction:{.5,.375,.625,.25,.75}){
            const double firstLength=length*fraction,secondLength=length-firstLength;
            const auto midYaw=detail::angleAt(yawPolynomial,firstLength,length);
            auto shaped=[&](double peak){
                const AngleJet middle{peak,0,0,0};
                auto a=polynomialMotion(cursor,detail::anglePolynomial(first[0],middle,firstLength),
                    detail::anglePolynomial(first[1],midYaw,firstLength),firstLength);
                auto b=polynomialMotion(a.end,detail::anglePolynomial(middle,last[0],secondLength),
                    detail::anglePolynomial(midYaw,endYaw,secondLength),secondLength);
                return std::pair{a,b};
            };
            double low=sign>0?std::max({0.,first[0].value,last[0].value}):-.9;
            double high=sign>0?.9:std::min({0.,first[0].value,last[0].value});
            if(shaped(low).second.end.position.z>target||shaped(high).second.end.position.z<target)continue;
            for(int i=0;i<45;++i){const double mid=(low+high)*.5;if(shaped(mid).second.end.position.z<target)low=mid;else high=mid;}
            auto pair=shaped((low+high)*.5);
            bool ordered=true;
            for(const auto& [program,trend]:std::array<std::pair<MotionProgram,double>,2>{{{pair.first,sign},{pair.second,-sign}}})
                for(int i=0;i<=128;++i){
                    const auto pitch=detail::directionAngles(program.direction(program.length*i/128.))[0];
                    if(trend*pitch.first< -1e-10||sign*pitch.value< -1e-10)ordered=false;
                }
            if(!ordered)continue;
            if(std::abs(pair.second.end.position.z-target)>1e-8)throw std::runtime_error(name+": fixed-length height did not converge");
            selected=std::move(pair);break;
        }
        if(!selected)throw std::runtime_error(name+": inherited pitch cannot form one fixed-length ordered crest or trough");
        const size_t moduleBegin=motion.modules.size();
        const auto a=motion.programme(selected->first,kind,(name+"-entry").c_str());
        const auto b=motion.programme(selected->second,kind,name.c_str(),bank);
        const bool planar=motion.modules[moduleBegin].planar&&motion.modules[moduleBegin+1].planar;
        motion.modules.resize(moduleBegin);motion.modules.push_back({a.first,b.second,name,0,planar});
        return MotionBuilder::Range{a.first,b.second};
    };
    struct SourceOptions {
        double begin{},blend{};
        bool worldCoordinates{},allowConnector{false},calibrate{true};
    };
    auto source=[&](const FvdHillResult& result,Element kind,const RecipeElement& element,SourceOptions options=SourceOptions{}){
        const double begin=options.begin;
        if(!result.section.report.valid()||!result.section.assessment.passed)
            throw std::runtime_error(element.id+": "+(result.section.report.errors.empty()?"force authoring residual":result.section.report.errors.front().message)+"; entrySpeed="+std::to_string(result.authoring.speed)+", incomingPitch="+std::to_string(std::asin(cursor.tangent.z))+", height="+std::to_string(cursor.position.z));
        const double yaw=options.worldCoordinates?0:heading();const auto first=rotated(detail::jet(sampleKinematics(result.section.track,begin)),yaw);
        if(norm(first.tangent-cursor.tangent)+norm(first.curvature-cursor.curvature)+norm(first.third-cursor.third)+norm(first.fourth-cursor.fourth)>1e-7){
            if(!options.allowConnector)throw std::runtime_error(element.id+": inherited FVD source must meet the live geometry without a repair spline");
            const auto desired=detail::directionAngles(first);
            const double length=options.blend>0?options.blend:std::max(40.,motion.nominalSpeed*2.1);
            freeDirection(desired[0],desired[1],length,kind,element.id+"-entry");
        }
        const Vec3 origin=cursor.position-first.position;
        const size_t firstKnot=d.track.knots.size()-1;
        const auto range=motion.force(result.section,result.authoring,kind,element.id.c_str(),origin,yaw,begin);
        if(options.calibrate)sourcePorts.push_back({element.id,firstKnot});
        const auto& samples=result.section.samples;
        const auto at=std::lower_bound(samples.begin(),samples.end(),begin,[](const FvdSample& a,double s){return a.distance<s;});
        double sourceSpeed=result.authoring.speed;if(at!=samples.end()){if(at==samples.begin())sourceSpeed=at->speed;else {const auto& before=*(at-1);sourceSpeed=std::lerp(before.speed,at->speed,(begin-before.distance)/(at->distance-before.distance));}}
        if(options.calibrate)sourceSpeeds.push_back(sourceSpeed);motion.nominalSpeed=samples.back().speed;
        pendingPort.reset();
        size_t apex=range.first;for(size_t k=range.first+1;k<=range.second;++k)if(d.track.knots[k].position.z>d.track.knots[apex].position.z)apex=k;
        if(element.role==RideRole::Opening){landmarks.push_back({LandmarkKind::OpeningCrest,apex});landmarks.push_back({LandmarkKind::OpeningRecovery,range.second});}
        if(element.role==RideRole::Camelback)landmarks.push_back({LandmarkKind::CamelbackCrest,apex});
        if(element.role==RideRole::Wave)landmarks.push_back({LandmarkKind::WaveCrest,apex});
        if(element.role==RideRole::Loop)landmarks.push_back({LandmarkKind::LoopCrest,apex});
        if(element.role==RideRole::Immelmann)landmarks.push_back({LandmarkKind::ImmelmannCrest,apex});
        // Split only at actual extrema so section intent describes the source,
        // while every boundary keeps its complete live geometric derivatives.
        size_t start=range.first,turning=range.first;int previous=0,index=0;
        for(size_t k=range.first;k<range.second;++k){const double dz=d.track.knots[k].tangent.z;const int sign=dz>.008?1:dz<-.008?-1:0;
            if(previous>0?d.track.knots[k].position.z>d.track.knots[turning].position.z:d.track.knots[k].position.z<d.track.knots[turning].position.z)turning=k;
            if(sign&&previous&&sign!=previous&&turning>start){motion.modules.push_back({start,turning,element.id+"-"+std::to_string(index++),0,std::holds_alternative<CamelbackParameters>(element.parameters)});start=turning;turning=k;}if(sign)previous=sign;}
        if(start<range.second)motion.modules.push_back({start,range.second,element.id+"-"+std::to_string(index),0,std::holds_alternative<CamelbackParameters>(element.parameters)});
    };
    auto groundLandscape=[&](Vec3 crest,double yaw,double summit){
        plateauCenter=crest+rotated(Vec3{-80,150,0},yaw);
        if(req.terrain.kind!=TerrainKind::Highlands)return;
        auto& t=d.request.terrain;t.ridge={};t.ramps.clear();t.knolls.clear();t.foothills.clear();t.ravines.clear();
        const auto center=plateauCenter;
        t.centerX=center.x;t.centerY=center.y;t.heightMeters=std::max(0.,summit-14.5);
        t.backSlope=TerrainSlope{crest.x,crest.y,t.heightMeters,.65*std::cos(yaw),.65*std::sin(yaw),190};
        t.radiusX=700;t.radiusY=620;t.plateau=.72;t.bend=.06;
        t.cliffX=crest.x+500*std::cos(yaw);t.cliffY=crest.y+500*std::sin(yaw);t.cliffHeading=yaw;t.cliffWidth=24;t.cliffCurvature=.00022;
        // The connected shelf and broad foothills are semantic landforms, not
        // a dense copy of rail samples. Later ports determine their final front.
        for(const auto local:std::array<Vec3,3>{{{-160,-220,70},{260,-260,95},{-360,-80,38}}}){
            const auto p=crest+rotated(local,yaw);t.foothills.push_back({p.x,p.y,local.z,340});}
    };
    for(const auto& element:req.recipe.elements){
        if(energyPrefix&&element.role==RideRole::Return&&element.anchor==TerrainAnchor::Approach)break;
        if(energyPrefix&&element.role==RideRole::Brakes)break;
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        const size_t moduleBegin=motion.modules.size();
        pendingPort.reset();
        const uint64_t random=elementSeed(req.seed,element.id);
        const double variation=seeded(random,.985,1.015),turnVariation=seeded(elementSeed(random,"heading"),-.06,.06);
        try{switch(element.role){
        case RideRole::Station:{
            const auto& p=std::get<OperationParameters>(element.parameters);
            if(p.lengthMeters>halfTrain+27)throw std::runtime_error("Station length must leave the stopped train within the departure stators");
            motion.line(p.lengthMeters,Element::Station,element.id.c_str());break;
        }
        case RideRole::Departure:{
            const auto& p=std::get<OperationParameters>(element.parameters);
            const double target=p.targetSpeedKmh>0?p.targetSpeedKmh/3.6:64;
            motion.drive(p.lengthMeters,DriveKind::Launch,target,p.accelerationMps2>0?p.accelerationMps2:accelerationFor(req),element.id.c_str());break;
        }
        case RideRole::Opening:{
            const auto& p=std::get<HillParameters>(element.parameters);FvdHillRequest intent;
            intent.entrySpeed=speedFor(element.id);intent.height=p.riseMeters*p.profileScale*variation;
            intent.airtimeG=p.negativeG*req.style.airtime;intent.positiveG=p.pulloutG;
            intent.twistAngle=p.twistDegrees*seeded(random,.9,1.1)*pi/180;intent.rampSeconds=p.releaseSeconds;intent.exitRampSeconds=p.recoverySeconds;
            intent.exitHeight=6;intent.exitPitch=.08;
            lossFor(intent);source(designFvdHill(intent,cancel),Element::Hill,element);
            // Broad low hills follow the first twisted drop. They carry the
            // complete live geometry into deliberate rising and falling curves.
            for(int n=0;n<2;++n){
                const double base=datum+(n==0?6:3),rise=n==0?35:20,yaw=heading();
                const double crest=motion.crestCurvature(rise,180,.05);
                auto top=detail::planarJet({cursor.position.x,cursor.position.y,base+rise},yaw+(n==0?-25:30)*pi/180,0,crest);
                terrainCurve(top,190,Element::Airtime,element.id+(n==0?"-river-hop-crest":"-rising-bank-crest"),(n==0?-15:20)*pi/180);
                auto low=detail::planarJet({cursor.position.x,cursor.position.y,base},heading()+(n==0?-20:15)*pi/180,0);
                terrainCurve(low,190,Element::Airtime,element.id+(n==0?"-river-hop-valley":"-rising-bank-valley"));
            }
            break;
        }
        case RideRole::CliffApproach:{
            const auto& p=std::get<CliffParameters>(element.parameters);const double summit=datum+p.summitHeightMeters;
            // Layout correction belongs to the climb setup. Every subsequent shelf
            // turn and signature keeps the shape and yaw requested by its author.
            const double layoutHeading=.45+(feedback.compactReturn?feedback.cliffHeadingCorrection:attempt==1?-4*pi/180:attempt==2?4*pi/180:0);
            freeDirection({20*pi/180,0,0,0},{heading()+layoutHeading,0,0,0},std::max(120.,motion.nominalSpeed*3.0),Element::Turn,element.id+"-recovery");
            const auto ascentStart=cursor;
            motion.driveGraded(summit-55,32*pi/180,58,16,(element.id+"-ascent").c_str());
            const auto ascentEnd=cursor;
            motion.pitchToHeight(summit,{0,motion.crestCurvature(55,180,-.5),0,0},Element::Hill,(element.id+"-arrival").c_str());
            plateauArrival=cursor.position;const double arrivalYaw=heading();groundLandscape(plateauArrival,arrivalYaw,summit);
            landmarks.push_back({LandmarkKind::PlateauArrival,d.track.knots.size()-1});
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;t.ramps.push_back({ascentStart.position.x,ascentStart.position.y,ascentEnd.position.x,ascentEnd.position.y,std::max(0.,ascentStart.position.z-datum),std::max(0.,ascentEnd.position.z-datum),std::tan(20*pi/180),std::tan(32*pi/180),190});}
            // An active shelf journey: outward reveal, inland counterturn,
            // second edge crest and two low turns into the dive approach.
            // Each direction programme owns its pitch/yaw jets. No overlap
            // operation may resculpt these curves after they are authored.
            const double shelfHeading=heading(),lengthScale=p.approachLengthMeters/1000.;
            const std::array<double,6> yaw{35,90,155,205,155,p.approachHeadingDegrees};
            const std::array<double,6> level{-8,-3,-8,-3,-8,-10};
            const std::array<double,6> lengths{115,185,185,175,175,165};
            for(size_t n=0;n<yaw.size();++n){
                auto end=detail::planarJet({cursor.position.x,cursor.position.y,summit+level[n]},shelfHeading+yaw[n]*pi/180,0);
                const double outward=(n==1||n==3)?p.outwardBankDegrees*pi/180:0;
                terrainCurve(end,lengths[n]*lengthScale,Element::Turn,element.id+"-shelf-"+std::to_string(n+1),outward);
            }
            outbankBay=plateauArrival+rotated(Vec3{180,70,0},arrivalYaw);
            break;
        }
        case RideRole::CliffLip:{
            const auto& p=std::get<OperationParameters>(element.parameters);const double target=p.targetSpeedKmh/3.6;
            const double acceleration=p.accelerationMps2>0?p.accelerationMps2:7;
            const double length=std::max(p.lengthMeters,(motion.nominalSpeed*motion.nominalSpeed-target*target)/(2*acceleration)+motion.nominalSpeed*.5+4*halfTrain+6);
            motion.drive(length,DriveKind::Brake,target,acceleration,element.id.c_str());break;
        }
        case RideRole::CliffDrop:{
            const auto& p=std::get<CliffParameters>(element.parameters);FvdDiveRequest intent;
            cliffTop=cursor.position;cliffYaw=heading();intent.entrySpeed=speedFor(element.id);
            landmarks.push_back({LandmarkKind::CliffDeparture,d.track.knots.size()-1});
            const auto boost=std::find_if(req.recipe.elements.begin(),req.recipe.elements.end(),[](const RecipeElement& item){return item.role==RideRole::DownhillLaunch;});
            const auto& boostParameters=std::get<OperationParameters>(boost->parameters);
            intent.drop=cliffTop.z-(datum+65);intent.maximumPitch=p.dropDegrees*pi/180;intent.exitPitch=boostParameters.gradeDegrees*pi/180;
            intent.crestRampSeconds=2.6;intent.crestG=-.55;intent.pulloutG=4.3;intent.pulloutRampSeconds=1.5;intent.exitRampSeconds=1.2;lossFor(intent);
            source(designFvdDive(intent,cancel),Element::Hill,element);cliffFoot=cursor.position;
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;t.cliffX=cliffTop.x+std::cos(cliffYaw)*7;t.cliffY=cliffTop.y+std::sin(cliffYaw)*7;t.cliffHeading=cliffYaw;
            }
            break;
        }
        case RideRole::DownhillLaunch:{
            const auto& p=std::get<OperationParameters>(element.parameters);const double target=(p.targetSpeedKmh>0?p.targetSpeedKmh/3.6:req.targets.speed+.15)+feedback.peakSpeedCorrection;
            const double acceleration=p.accelerationMps2>0?p.accelerationMps2:16;
            const double net=acceleration-gravity*cursor.tangent.z-gravity*req.train.rollingResistance-motion.drag*target*target;
            const double length=p.lengthMeters>0?p.lengthMeters:std::max(120.,(target*target-motion.nominalSpeed*motion.nominalSpeed)/(2*net)+target*.85+4*halfTrain+6);
            motion.drive(length,DriveKind::Boost,target,acceleration,element.id.c_str());
            landmarks.push_back({LandmarkKind::DownhillLaunchExit,d.track.knots.size()-1});
            if(cursor.position.z<datum)throw std::runtime_error("Inclined launch exhausted the authored cliff-foot relief");
            camelbackBase=cursor.position;
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;
                t.ramps.push_back({cliffFoot.x,cliffFoot.y,cursor.position.x,cursor.position.y,cliffFoot.z-32,cursor.position.z-32,std::tan(p.gradeDegrees*pi/180),0,140});
                const auto shoulder=(cliffFoot+cursor.position)*.5+rotated(Vec3{0,-400,0},heading());
                t.foothills.push_back({shoulder.x,shoulder.y,45,280});
            }
            break;
        }
        case RideRole::Camelback:{
            auto p=std::get<CamelbackParameters>(element.parameters);
            p.tailCutSeconds+=feedback.camelbackTailCutSeconds;
            auto entry=element;entry.id=element.id+"-pullout";entry.role=RideRole::Unspecified;
            auto paired=designLaunchCamelback(speedFor(entry.id),std::asin(cursor.tangent.z),p,
                gravity*req.train.rollingResistance,motion.drag,cancel);
            source(paired.pullout,Element::Hill,entry,{.allowConnector=false});
            // Retain the reference from its loaded ascent shoulder. The pullout
            // shares that source's pitch, curvature, force and energy jets.
            // One calibration variable owns the whole connected element.
            source({std::move(paired.camelback.authoring),std::move(paired.camelback.section)},
                Element::Hill,element,{.begin=paired.retainedBeginDistance,.allowConnector=false,.calibrate=false});
            break;
        }
        case RideRole::Wave:{
            const auto& p=std::get<TurnParameters>(element.parameters);FvdWaveRequest intent;
            waveBase=cursor.position;intent.entrySpeed=speedFor(element.id);intent.entryPitch=std::asin(cursor.tangent.z);
            intent.entry=motion.fvdEntry(intent.entrySpeed);
            const auto upright=unit(Vec3{0,0,1}-cursor.tangent*cursor.tangent.z);
            intent.entryNormalG=dot(cursor.curvature*(intent.entrySpeed*intent.entrySpeed)+Vec3{0,0,gravity},upright)/gravity;intent.entryHoldSeconds=1.8;
            intent.height=p.riseMeters;intent.exitHeight=10;intent.turnAngle=p.headingDegrees*pi/180;intent.bankAngle=p.bankDegrees*pi/180;intent.exitNormalG=p.exitNormalG;
            lossFor(intent);source(designFvdWave(intent,cancel),Element::Turn,element,{.worldCoordinates=true,.allowConnector=false});break;
        }
        case RideRole::Loop:{
            const auto& p=std::get<InversionParameters>(element.parameters);FvdLoopRequest intent;
            loopBase=cursor.position;intent.entrySpeed=speedFor(element.id);intent.height=p.referenceRiseMeters*std::pow(intent.entrySpeed/65,2);intent.yawAngle=p.yawDegrees*pi/180;intent.crestG=p.crestG;intent.exitPitch=p.exitPitchDegrees*pi/180;intent.exitNormalG=1.5;intent.normalG=3.8;intent.exitPositiveG=3.8;intent.ascentReleaseSeconds=1.2;
            intent.entry=motion.fvdEntry(intent.entrySpeed);
            lossFor(intent);source(designFvdLoop(intent,cancel),Element::Inversion,element,{.worldCoordinates=true,.allowConnector=false});break;
        }
        case RideRole::Immelmann:{
            const auto& p=std::get<InversionParameters>(element.parameters);FvdImmelmannRequest intent;
            intent.entrySpeed=speedFor(element.id);intent.height=p.referenceRiseMeters*std::pow(intent.entrySpeed/53,2);intent.exitHeight=10;intent.exitPitch=p.exitPitchDegrees*pi/180;
            intent.normalG=3.8;intent.exitPositiveG=3;intent.crestG=p.crestG;intent.rollExitG=.6;intent.rollReleaseFraction=0;intent.ascentReleaseSeconds=1.2;intent.rampSeconds=1.0;intent.exitRampSeconds=1.2;intent.rollOverlapFraction=0;intent.yawAngle=p.yawDegrees*pi/180;intent.exitNormalG=std::cos(intent.exitPitch);intent.hand=-1;lossFor(intent);
            intent.entry=motion.fvdEntry(intent.entrySpeed);
            const auto inversion=designFvdImmelmann(intent,cancel);
            source({inversion.authoring,inversion.section},Element::Inversion,element,{.worldCoordinates=true,.allowConnector=false});immelExit=cursor.position;break;
        }
        case RideRole::Signature:{
            const auto& p=std::get<SweepParameters>(element.parameters);
            // The flexible approach steers the complete signature as one piece;
            // it never changes either wing's authored forces or bank sequence.
            freeDirection({0,0,0,0},{heading()-.45+feedback.approachHeadingCorrection,0,0,0},
                std::max(110.,motion.nominalSpeed*2.5),Element::Turn,element.id+"-approach");
            const double signatureYaw=heading(),floor=std::max(datum+2,cursor.position.z+p.riseMeters),drop=cursor.position.z-floor,scale=p.lengthMeters/540;
            // Riftwake: an outward floating wing, a falling transfer across
            // the ravine, an opposing wing and a low carving dive. The two
            // crests share one descending silhouette and authored bearing.
            Vec3 transfer;
            for(int wing=0;wing<2;++wing){
                const double rise=(wing==0?25:20)*scale,base=cursor.position.z;
                const double crestYaw=signatureYaw+p.headingDegrees*pi/180*(wing==0?.18:.68);
                auto top=detail::planarJet({cursor.position.x,cursor.position.y,base+rise},crestYaw,0,
                    motion.crestCurvature(rise,160*scale,std::max(-1.1,p.negativeG*req.style.airtime)*feedback.signatureAirtimeScale));
                const double bank=(wing==0?1:-1)*p.rollDegrees*(req.style.signatureRollDegrees/45)*pi/180;
                terrainCurve(top,160*scale,Element::Airtime,element.id+(wing==0?"-outward-wing":"-counter-wing"),bank);
                if(wing==0)landmarks.push_back({LandmarkKind::SignatureRelease,d.track.knots.size()-1});
                const double lowHeight=wing==0?floor+drop*.48:floor;
                const double lowYaw=signatureYaw+p.headingDegrees*pi/180*(wing==0?.5:1);
                auto low=detail::planarJet({cursor.position.x,cursor.position.y,lowHeight},lowYaw,wing==0?0:p.exitPitchDegrees*pi/180);
                terrainCurve(low,(wing==0?200:220)*scale,Element::Turn,element.id+(wing==0?"-ravine-transfer":"-low-carve"));
                if(wing==0)transfer=cursor.position;
            }
            returnRavine=cursor.position;
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;const auto mid=(immelExit+cursor.position)*.5;
                t.foothills.push_back({mid.x+110,mid.y+90,10,380});
                t.foothills.push_back({mid.x-280,mid.y-180,35,520});
                t.ravines.push_back({immelExit.x,immelExit.y,transfer.x,transfer.y,60,55,110,140});
                t.ravines.push_back({transfer.x,transfer.y,cursor.position.x,cursor.position.y,55,50,140,140});
            }
            break;
        }
        case RideRole::Return:{
            std::visit([&](const auto& p){using P=std::decay_t<decltype(p)>;
                if constexpr(std::is_same_v<P,HillParameters>){
                    FvdHillRequest intent;intent.entrySpeed=speedFor(element.id);intent.height=p.riseMeters*p.profileScale;
                    intent.positiveG=p.pulloutG;intent.airtimeG=std::max(-1.32,p.negativeG*req.style.airtime);intent.rampSeconds=p.releaseSeconds;intent.exitRampSeconds=p.recoverySeconds;
                    intent.twistAngle=(p.twistDegrees+(req.style.returnStyle?25.:0))*pi/180;
                    const size_t next=size_t(&element-req.recipe.elements.data())+1;
                    const bool terminal=next<req.recipe.elements.size()&&req.recipe.elements[next].anchor==TerrainAnchor::Approach;
                    intent.exitHeight=terminal?datum-cursor.position.z:0;
                    intent.exitPitch=terminal?0:-.12;intent.exitNormalG=terminal?1:0;
                    lossFor(intent);intent.entry=motion.fvdEntry(intent.entrySpeed);
                    source(designFvdHill(intent,cancel),Element::Airtime,element,{.worldCoordinates=true,.allowConnector=false});
                }else if constexpr(std::is_same_v<P,TurnParameters>){
                    const double angle=p.headingDegrees*pi/180+turnVariation;
                    const double length=std::max(p.lengthMeters,1.875*std::abs(angle)*motion.nominalSpeed*motion.nominalSpeed/(gravity*std::tan(std::abs(p.bankDegrees)*pi/180)));
                    auto end=detail::planarJet({cursor.position.x,cursor.position.y,cursor.position.z+p.riseMeters},heading()+angle,0,(p.exitNormalG-1)*gravity/(motion.nominalSpeed*motion.nominalSpeed));
                    auto program=solveHeightMotion(cursor,end,length,{motion.nominalSpeed,gravity*req.train.rollingResistance,motion.drag,0,4.4,-.8},cancel);
                    motion.programme(program,Element::Turn,element.id.c_str());motion.modules.back().reversals=1;
                }else if constexpr(std::is_same_v<P,SweepParameters>){
                    if(element.anchor!=TerrainAnchor::Approach){
                        auto end=detail::planarJet({cursor.position.x,cursor.position.y,cursor.position.z+p.riseMeters},heading()+p.headingDegrees*pi/180,p.exitPitchDegrees*pi/180);
                        motion.programme(solveHeightMotion(cursor,end,p.lengthMeters,{motion.nominalSpeed,gravity*req.train.rollingResistance,motion.drag,p.rollDegrees*pi/180,4.4,-.8},cancel),Element::Turn,element.id.c_str(),p.rollDegrees*pi/180);
                        motion.modules.back().reversals=1;return;
                    }
                    if(req.terrain.kind==TerrainKind::Highlands)d.request.terrain.ravines.push_back({returnRavine.x,returnRavine.y,cursor.position.x,cursor.position.y,60,50,160,190});
                    // A reordered final hill may still be descending while
                    // pulling out. Finish that same curvature release before
                    // asking a placed level corridor to reach the station.
                    const auto jets=detail::directionAngles(cursor);
                    if(std::abs(jets[0].value)>1e-5){
                        auto released=[](AngleJet q,double length){return q.value+q.first*length*.5+q.second*length*length*.1+q.third*length*length*length/120;};
                        double length=std::max(40.,motion.nominalSpeed*2);
                        double lo=1,hi=2;
                        while(hi<400&&released(jets[0],lo)*released(jets[0],hi)>0){lo=hi;hi*=1.25;}
                        if(hi<400){
                            for(int k=0;k<48;++k){const double mid=(lo+hi)*.5;if(released(jets[0],lo)*released(jets[0],mid)>0)lo=mid;else hi=mid;}
                            length=(lo+hi)*.5;
                        }
                        const auto program=polynomialMotion(cursor,detail::anglePolynomial(jets[0],{0,0,0,0},length),detail::anglePolynomial(jets[1],{released(jets[1],length),0,0,0},length),length);
                        for(int k=1;k<100;++k){const double pitch=std::asin(program.direction(length*k/100).tangent.z);
                            if(pitch*jets[0].value< -1e-10||std::abs(pitch)>std::abs(jets[0].value)+1e-7)throw std::runtime_error("The incoming grade cannot release monotonically into the low return");}
                        motion.programme(program,Element::Turn,(element.id+"-grade-release").c_str());
                    }
                    // The valley already places this port outside the plateau.
                    // Reach the station approach directly instead of detouring
                    // outward to a second, redundant landscape waypoint.
                    const auto& brakes=std::get<OperationParameters>(req.recipe.elements.back().parameters);
                    brakeAlignment=2*halfTrain+3;
                    const Vec3 approach{-brakes.lengthMeters-brakeAlignment,0,datum};
                    if(req.terrain.kind==TerrainKind::Highlands)d.request.terrain.ravines.push_back({cursor.position.x,cursor.position.y,approach.x,approach.y,65,45,260,290});
                    const size_t begin=d.track.knots.size()-1;
                    FvdApproachRequest intent;intent.entry=motion.fvdEntry(speedFor(element.id));
                    intent.endPosition=approach;intent.endHeading=0;intent.normalG=4;lossFor(intent);
                    source(designFvdApproach(intent,cancel),Element::Turn,element,{.worldCoordinates=true,.allowConnector=false});
                    for(size_t k=begin;k<d.track.knots.size();++k)if(d.track.knots[k].position.z<datum-.05)throw std::runtime_error("Low return corridor left its basin floor; no clearance lift is inserted");
                }else throw std::runtime_error("Unsupported return parameter family");
            },element.parameters);break;
        }
        case RideRole::Brakes:{
            const auto& p=std::get<OperationParameters>(element.parameters);
            brakeCapacity=-p.accelerationMps2;
            const double alignment=brakeAlignment>0?brakeAlignment:2*halfTrain+3;
            motion.line(alignment,Element::Brake,"brake-alignment");
            brakeBegin=d.track.knots.size()-1;motion.line(p.lengthMeters,Element::Brake,element.id.c_str());break;
        }
        default:throw std::runtime_error("Unspecified recipe role");
        }}catch(const std::exception& failure){
            // Keep only the known geometry up to the requested source port.
            // A partly appended failed element must never enter the energy probe.
            if(pendingPort&&pendingPort->knot+1<d.track.knots.size())d.track.knots.resize(pendingPort->knot+1);
            d.track.closed=false;
            compiled.push_back({moduleBegin,motion.modules.size(),element.role,element.id});
            std::optional<RecipePort> pending;
            if(d.track.knots.size()>=4){
                const size_t last=d.track.knots.size()-1;
                std::erase_if(d.forcePrograms,[&](const auto& item){return item.firstKnot>=last;});
                for(auto& item:d.forcePrograms)if(item.sourceDistances.size()>last-item.firstKnot+1)item.sourceDistances.resize(last-item.firstKnot+1);
                std::erase_if(d.splinePrograms,[&](const auto& item){return item.lastKnot>last;});
                try{
                    finalizeKnownPrefix(false);
                    if(pendingPort)pending=RecipePort{pendingPort->id,d.track.length,pendingPort->speed};
                }catch(const std::exception& recovery){
                    ports.clear();
                    d.report.fail("AUTHORING_PARTIAL_RECOVERY",std::string("Could not finalize the failed prefix: ")+recovery.what());
                }
            }
            d.report.fail("AUTHORING_PARTIAL","Unaccepted authored prefix; any continuation used for energy bootstrapping is provisional");
            std::ostringstream diagnostic;diagnostic<<std::setprecision(12)<<"{\"schemaVersion\":4,\"generationOnly\":true,\"partial\":true,\"failedElement\":"<<std::quoted(element.id);
            diagnostic<<",\"layoutCorrectionRadians\":["<<feedback.cliffHeadingCorrection<<','<<feedback.approachHeadingCorrection<<']';
            if(pending)diagnostic<<",\"pendingPort\":{\"id\":"<<std::quoted(pending->id)<<",\"distance\":"<<pending->distance<<",\"plannedSpeed\":"<<pending->speed<<'}';
            diagnostic<<'}';d.planningDiagnostics=diagnostic.str();
            throw RecipeCompileFailure(element.id+": "+failure.what(),std::move(d),ports,std::move(pending));
        }
        compiled.push_back({moduleBegin,motion.modules.size(),element.role,element.id});
    }
    if(!energyPrefix&&(norm(cursor.position-d.track.knots.front().position)>1e-7||norm(cursor.tangent-d.track.knots.front().tangent)>1e-7||norm(cursor.curvature)>1e-7||norm(cursor.third)>1e-7||norm(cursor.fourth)>1e-7)){
        std::ostringstream message;message<<std::setprecision(17)<<"Recipe station closure failed its complete endpoint jet: position="
            <<cursor.position.x<<','<<cursor.position.y<<','<<cursor.position.z<<" tangent="<<cursor.tangent.x<<','<<cursor.tangent.y<<','<<cursor.tangent.z
            <<" curvature="<<norm(cursor.curvature)<<" third="<<norm(cursor.third)<<" fourth="<<norm(cursor.fourth);
        throw std::runtime_error(message.str());
    }
    if(!energyPrefix)d.track.knots.back()=d.track.knots.front();
    d.track.closed=!energyPrefix;
    for(auto& k:d.track.knots){k.position.y*=hand;k.tangent.y*=hand;k.curvature.y*=hand;k.third.y*=hand;k.fourth.y*=hand;k.up.y*=hand;k.upFirst.y*=hand;k.upSecond.y*=hand;k.upThird.y*=hand;k.bank*=hand;}
    auto& terrain=d.request.terrain;terrain.centerY*=hand;terrain.bend*=hand;terrain.cliffY*=hand;terrain.cliffHeading*=hand;
    if(terrain.backSlope){terrain.backSlope->y*=hand;terrain.backSlope->gradeY*=hand;}
    for(auto& ramp:terrain.ramps){ramp.y0*=hand;ramp.y1*=hand;}for(auto& knoll:terrain.knolls)knoll.y*=hand;for(auto& ravine:terrain.ravines){ravine.y0*=hand;ravine.y1*=hand;}
    for(auto& foothill:terrain.foothills)foothill.y*=hand;
    for(auto& program:d.forcePrograms)program.hand=hand;for(auto& program:d.splinePrograms)program.hand=hand;
    if(!terrain.valid())throw std::runtime_error("Resolved semantic terrain is outside its bounded domain");
    finalizeKnownPrefix(!energyPrefix);
    d.topology="recipe/"+d.request.recipe.name;std::ostringstream diagnostics;diagnostics<<std::setprecision(12)<<"{\"schemaVersion\":4,\"generationOnly\":true,\"recipeVersion\":"<<req.recipe.version<<",\"seed\":"<<req.seed<<",\"candidate\":"<<attempt<<",\"hand\":"<<hand<<",\"layoutCorrectionRadians\":["<<feedback.cliffHeadingCorrection<<','<<feedback.approachHeadingCorrection<<"],\"ports\":[";
    for(size_t i=0;i<ports.size();++i){if(i)diagnostics<<',';diagnostics<<"{\"id\":"<<std::quoted(ports[i].id)<<",\"distance\":"<<ports[i].distance<<",\"plannedSpeed\":"<<ports[i].speed<<'}';}
    diagnostics<<"],\"forceCorrections\":{\"camelbackTailCutSeconds\":"<<feedback.camelbackTailCutSeconds<<",\"signatureAirtimeScale\":"<<feedback.signatureAirtimeScale<<"},\"terrainAnchors\":[";bool comma=false;
    for(const auto& [name,position]:std::array<std::pair<const char*,Vec3>,5>{{{"plateau",plateauArrival},{"cliff-foot",cliffFoot},{"wave-bench",waveBase},{"loop-basin",loopBase},{"immelmann-shoulder",immelExit}}}){
        if(comma)diagnostics<<',';comma=true;diagnostics<<"{\"name\":"<<std::quoted(name)<<",\"position\":["<<position.x<<','<<position.y*hand<<','<<position.z<<"]}";}
    diagnostics<<"]}";d.planningDiagnostics=diagnostics.str();
    if(energyPrefix)d.report.fail("ENERGY_CALIBRATION_PREFIX","Open authoring prefix is for energy calibration only; closed-circuit validation is required" );
    return d;
}
RecipeBootstrapResult bootstrapRecipeEnergy(const RecipeCompileFailure& failure,RecipeFeedback& feedback,Cancel cancel){
    RecipeBootstrapResult result;
    auto cancelled=[&]{if(cancel&&cancel()){result.cancelled=true;result.report.fail("CANCELLED","Provisional energy bootstrap cancelled");return true;}return false;};
    if(cancelled())return result;
    const auto& partial=failure.partial;const auto& train=partial.request.train;
    if(partial.track.closed||partial.track.knots.size()<4||!failure.pendingPort||
        !std::isfinite(failure.pendingPort->speed)||failure.pendingPort->speed<=0||
        std::abs(failure.pendingPort->distance-partial.track.length)>1e-5){
        result.report.fail("ENERGY_BOOTSTRAP_DOMAIN","Energy bootstrap requires an open prefix ending exactly at a typed pending source port");return result;
    }
    const double half=(train.cars-1)*train.spacing*.5;
    if(!std::isfinite(half)||half<0||half>160){result.report.fail("ENERGY_BOOTSTRAP_DOMAIN","Invalid finite-train coverage for energy bootstrap");return result;}
    // Open replay reserves half a train plus 1 m at its far boundary. Add 2 m
    // beyond that so the pending centre position is actually sampled. This is
    // a separate hypothesis used only to seed source energy, never route art.
    result.continuationMeters=half+3;
    try{
        Track probe=partial.track;probe.closed=false;probe.authoredFrame=false;
        const auto& last=partial.track.knots.back();
        if(std::hypot(last.tangent.x,last.tangent.y)<.17)throw std::runtime_error("Provisional angular continuation is not defined near a vertical tangent");
        const detail::MotionJet jet{last.position,last.tangent,last.curvature,last.third,last.fourth};
        const auto angles=detail::directionAngles(jet);const double length=result.continuationMeters;
        auto continued=[&](detail::AngleJet a){return detail::AngleJet{
            a.value+a.first*length+a.second*length*length*.5+a.third*length*length*length/6,
            a.first+a.second*length+a.third*length*length*.5,a.second+a.third*length,a.third};};
        const auto program=polynomialMotion(jet,detail::anglePolynomial(angles[0],continued(angles[0]),length),
            detail::anglePolynomial(angles[1],continued(angles[1]),length),length);
        const auto first=program.direction(0);
        if(norm(first.tangent-jet.tangent)+norm(first.curvature-jet.curvature)+norm(first.third-jet.third)+norm(first.fourth-jet.fourth)>1e-7)
            throw std::runtime_error("Provisional continuation did not inherit the complete spatial jet");
        const size_t count=size_t(std::ceil(length/.3));
        for(size_t i=1;i<=count;++i){
            if(cancelled())return result;
            const double at=length*i/count;auto q=program.direction(at);q.position=jet.position+program.displacement(at);
            if(!finite(q.position)||std::abs(q.tangent.z)>.985||norm(q.curvature)>.15)
                throw std::runtime_error("Provisional continuation left its short, nonvertical curvature domain");
            const auto up=unit(Vec3{0,0,1}-q.tangent*q.tangent.z);
            probe.knots.push_back({q.position,q.tangent,q.curvature,up,0,Element::Return,q.third,q.fourth});
        }
        probe.rebuild();
        if(cancelled())return result;
        auto replay=simulateMotion(probe,partial.operations,train,partial.request.simulationStep,cancel,MotionReplayMode::StationEnergyPrefix);
        if(replay.cancelled){result.cancelled=true;result.report=std::move(replay.report);return result;}
        if(!replay.prefixReachedEnd||!replay.report.valid()||replay.frames.empty()||replay.frames.back().distance<failure.pendingPort->distance){
            result.report=std::move(replay.report);result.report.fail("ENERGY_BOOTSTRAP_REPLAY","Finite-train provisional replay did not reach the pending centre position");return result;
        }
        auto corrected=feedback;
        auto observe=[&](const RecipePort& port){
            if(port.distance<replay.frames.front().distance||port.distance>replay.frames.back().distance||!std::isfinite(port.speed)||port.speed<=0)return;
            const double actual=replayValueAt(replay.frames,port.distance);
            if(!std::isfinite(actual)||actual<=0)throw std::runtime_error("Provisional replay returned an invalid reached-port speed");
            result.observations.push_back({port,actual,port.distance+half>partial.track.length});
            const double delta=actual-port.speed;result.maximumSpeedCorrection=std::max(result.maximumSpeedCorrection,std::abs(delta));
            if(std::abs(delta)>=.01)corrected.energyCorrection[port.id]+=actual*actual-port.speed*port.speed;
        };
        for(const auto& port:failure.reachedPorts)if(port.id!=failure.pendingPort->id)observe(port);
        observe(*failure.pendingPort);
        if(cancelled())return result;
        if(result.maximumSpeedCorrection<.05){result.report.fail("ENERGY_BOOTSTRAP_NO_PROGRESS","Provisional reached-port correction is below 0.05 m/s; the failed source requires a different authored intent or a better resolved model");return result;}
        feedback=std::move(corrected);result.corrected=true;
        result.report.warnings.push_back("Pending boundary speed is provisional: front cars used a short inherited-jet continuation. Real-source calibration and complete validation remain mandatory.");
    }catch(const std::exception& error){
        if(cancelled())return result;
        result.report.fail("ENERGY_BOOTSTRAP",error.what());
    }
    return result;
}

}
