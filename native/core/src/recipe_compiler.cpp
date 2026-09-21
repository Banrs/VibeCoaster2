#include "recipe_compiler.hpp"
#include "authoring.hpp"
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

Design compileRecipe(const GenerationRequest& input,int attempt,const RecipeFeedback& feedback,std::vector<RecipePort>& ports,Cancel cancel){
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
    Vec3 plateauArrival{},plateauCenter{},cliffTop{},cliffFoot{},waveBase{},loopBase{},immelExit{};
    double cliffYaw=0;
    auto heading=[&]{return std::atan2(cursor.tangent.y,cursor.tangent.x);};
    auto speedFor=[&](const std::string& id){const auto found=feedback.energyCorrection.find(id);return std::sqrt(std::max(25.,motion.nominalSpeed*motion.nominalSpeed+(found==feedback.energyCorrection.end()?0:found->second)));};
    auto lossFor=[&](auto& intent){intent.rollingAcceleration=gravity*req.train.rollingResistance;intent.dragAccelerationCoefficient=motion.drag;};
    auto terrainAct=[&](const RecipeElement& element,TerrainAct kind,double height,double yaw,double bank,double negative,double scale,double exitPitch=0){
        const auto pitch=detail::directionAngles(cursor)[0];FvdTerrainActRequest intent;
        intent.kind=kind;intent.entrySpeed=speedFor(element.id);intent.entryPitch=pitch.value;intent.pitchRateS=pitch.first;intent.pitchSecondS=pitch.second;intent.pitchThirdS=pitch.third;
        intent.heightChange=height;intent.headingChange=yaw;intent.outwardBank=bank;intent.airtimeG=negative;intent.durationScale=scale;intent.exitPitch=exitPitch;lossFor(intent);return designFvdTerrainAct(intent,cancel);
    };
    auto freeDirection=[&](AngleJet endPitch,AngleJet endYaw,double length,Element kind,const std::string& name,double bank=0.){
        const auto incoming=detail::directionAngles(cursor);
        endYaw.value=incoming[1].value+std::remainder(endYaw.value-incoming[1].value,2*pi);
        const auto hint=polynomialMotion(cursor,detail::anglePolynomial(incoming[0],endPitch,length),detail::anglePolynomial(incoming[1],endYaw,length),length);
        return motion.programme(hint,kind,name.c_str(),bank);
    };
    auto source=[&](const FvdHillResult& result,Element kind,const RecipeElement& element,double begin=0.,double blend=0.){
        if(!result.section.report.valid()||!result.section.assessment.passed)
            throw std::runtime_error(element.id+": "+(result.section.report.errors.empty()?"force authoring residual":result.section.report.errors.front().message)+"; entrySpeed="+std::to_string(result.authoring.speed)+", incomingPitch="+std::to_string(std::asin(cursor.tangent.z))+", height="+std::to_string(cursor.position.z));
        const double yaw=heading();const auto first=rotated(detail::jet(sampleKinematics(result.section.track,begin)),yaw);
        if(norm(first.tangent-cursor.tangent)+norm(first.curvature-cursor.curvature)+norm(first.third-cursor.third)+norm(first.fourth-cursor.fourth)>1e-7){
            const auto desired=detail::directionAngles(first);
            const double length=blend>0?blend:std::max(40.,motion.nominalSpeed*2.1);
            freeDirection(desired[0],desired[1],length,kind,element.id+"-entry");
        }
        const Vec3 origin=cursor.position-first.position;
        const size_t firstKnot=d.track.knots.size()-1;
        const auto range=motion.force(result.section,result.authoring,kind,element.id.c_str(),origin,yaw,begin);
        sourcePorts.push_back({element.id,firstKnot});
        const auto& samples=result.section.samples;
        const auto at=std::lower_bound(samples.begin(),samples.end(),begin,[](const FvdSample& a,double s){return a.distance<s;});
        double sourceSpeed=result.authoring.speed;if(at!=samples.end()){if(at==samples.begin())sourceSpeed=at->speed;else {const auto& before=*(at-1);sourceSpeed=std::lerp(before.speed,at->speed,(begin-before.distance)/(at->distance-before.distance));}}
        sourceSpeeds.push_back(sourceSpeed);motion.nominalSpeed=samples.back().speed;
        size_t apex=range.first;for(size_t k=range.first+1;k<=range.second;++k)if(d.track.knots[k].position.z>d.track.knots[apex].position.z)apex=k;
        if(element.role==RideRole::Opening){landmarks.push_back({LandmarkKind::OpeningCrest,apex});landmarks.push_back({LandmarkKind::OpeningRecovery,range.second});}
        if(element.role==RideRole::Camelback)landmarks.push_back({LandmarkKind::CamelbackCrest,apex});
        if(element.role==RideRole::Wave)landmarks.push_back({LandmarkKind::WaveCrest,apex});
        if(element.role==RideRole::Loop)landmarks.push_back({LandmarkKind::LoopCrest,apex});
        if(element.role==RideRole::Immelmann)landmarks.push_back({LandmarkKind::ImmelmannCrest,apex});
        if(element.role==RideRole::Signature){
            const auto control=std::min_element(result.authoring.controls.begin(),result.authoring.controls.end(),[](const auto& a,const auto& b){return a.normalG<b.normalG;});
            const auto sample=std::lower_bound(samples.begin(),samples.end(),control->time,[](const FvdSample& a,double time){return a.time<time;});
            const auto& distances=d.forcePrograms.back().sourceDistances;
            landmarks.push_back({LandmarkKind::SignatureRelease,range.first+size_t(std::lower_bound(distances.begin(),distances.end(),sample->distance)-distances.begin())});
        }
        // Split only at actual extrema so section intent describes the source,
        // while every boundary keeps its complete live geometric derivatives.
        size_t start=range.first,turning=range.first;int previous=0,index=0;
        for(size_t k=range.first;k<range.second;++k){const double dz=d.track.knots[k].tangent.z;const int sign=dz>.008?1:dz<-.008?-1:0;
            if(previous>0?d.track.knots[k].position.z>d.track.knots[turning].position.z:d.track.knots[k].position.z<d.track.knots[turning].position.z)turning=k;
            if(sign&&previous&&sign!=previous&&turning>start){motion.modules.push_back({start,turning,element.id+"-"+std::to_string(index++),0,element.role==RideRole::Camelback});start=turning;turning=k;}if(sign)previous=sign;}
        if(start<range.second)motion.modules.push_back({start,range.second,element.id+"-"+std::to_string(index),0,element.role==RideRole::Camelback});
    };
    auto groundLandscape=[&](Vec3 crest,double yaw,double summit){
        plateauCenter=crest+rotated(Vec3{120,-130,0},yaw);
        if(req.terrain.kind!=TerrainKind::Highlands)return;
        auto& t=d.request.terrain;t.ridge={};t.ramps.clear();t.knolls.clear();t.foothills.clear();t.ravines.clear();
        const auto center=plateauCenter;
        t.centerX=center.x;t.centerY=center.y;t.heightMeters=std::max(0.,summit-14.5);
        t.backSlope=TerrainSlope{crest.x,crest.y,t.heightMeters,.65*std::cos(yaw),.65*std::sin(yaw)};
        t.radiusX=520;t.radiusY=440;t.plateau=.65;t.bend=.10;
        t.cliffX=crest.x+500*std::cos(yaw);t.cliffY=crest.y+500*std::sin(yaw);t.cliffHeading=yaw;t.cliffWidth=24;t.cliffCurvature=.00022;
        // The connected shelf and broad foothills are semantic landforms, not
        // a dense copy of rail samples. Later ports determine their final front.
        for(const auto local:std::array<Vec3,3>{{{-160,-220,70},{260,-260,95},{-360,-80,38}}}){
            const auto p=crest+rotated(local,yaw);t.foothills.push_back({p.x,p.y,local.z,340});}
    };
    for(const auto& element:req.recipe.elements){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        const size_t moduleBegin=motion.modules.size();
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
            intent.exitHeight=6;intent.exitPitch=.08;lossFor(intent);source(designFvdHill(intent,cancel),Element::Hill,element);break;
        }
        case RideRole::CliffApproach:{
            const auto& p=std::get<CliffParameters>(element.parameters);const double summit=datum+p.summitHeightMeters;
            freeDirection({20*pi/180,0,0,0},{heading(),0,0,0},std::max(80.,motion.nominalSpeed*2.0),Element::Turn,element.id+"-recovery");
            const auto ascentStart=cursor;
            motion.driveGraded(summit-55,32*pi/180,58,16,(element.id+"-ascent").c_str());
            const auto ascentEnd=cursor;
            motion.pitchToHeight(summit,{0,motion.crestCurvature(55,180,-.5),0,0},Element::Hill,(element.id+"-arrival").c_str());
            plateauArrival=cursor.position;const double arrivalYaw=heading();groundLandscape(plateauArrival,arrivalYaw,summit);
            landmarks.push_back({LandmarkKind::PlateauArrival,d.track.knots.size()-1});
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;t.ramps.push_back({ascentStart.position.x,ascentStart.position.y,ascentEnd.position.x,ascentEnd.position.y,std::max(0.,ascentStart.position.z-datum),std::max(0.,ascentEnd.position.z-datum),std::tan(20*pi/180),std::tan(32*pi/180),190});}
            const auto act=terrainAct(element,TerrainAct::Clifftop,-10,-65*pi/180+turnVariation+.25*std::sin(attempt*2.399963229728653),p.outwardBankDegrees*pi/180,std::max(-1.24,-.99*req.style.airtime),p.approachLengthMeters/300);
            source(act,Element::Turn,element);break;
        }
        case RideRole::CliffLip:{
            const auto& p=std::get<OperationParameters>(element.parameters);const double target=p.targetSpeedKmh/3.6;
            const double acceleration=p.accelerationMps2>0?p.accelerationMps2:7;
            const double length=std::max(p.lengthMeters,(motion.nominalSpeed*motion.nominalSpeed-target*target)/(2*acceleration)+motion.nominalSpeed*.8+4*halfTrain+6);
            motion.drive(length,DriveKind::Brake,target,acceleration,element.id.c_str());break;
        }
        case RideRole::CliffDrop:{
            const auto& p=std::get<CliffParameters>(element.parameters);FvdDiveRequest intent;
            cliffTop=cursor.position;cliffYaw=heading();intent.entrySpeed=speedFor(element.id);
            landmarks.push_back({LandmarkKind::CliffDeparture,d.track.knots.size()-1});
            const auto boost=std::find_if(req.recipe.elements.begin(),req.recipe.elements.end(),[](const RecipeElement& item){return item.role==RideRole::DownhillLaunch;});
            const auto& boostParameters=std::get<OperationParameters>(boost->parameters);
            intent.drop=cliffTop.z-(datum+85);intent.maximumPitch=p.dropDegrees*pi/180;intent.exitPitch=boostParameters.gradeDegrees*pi/180;
            intent.crestRampSeconds=2.6;intent.crestG=-.55;intent.pulloutG=3.8;intent.pulloutRampSeconds=1.5;intent.exitRampSeconds=1.2;lossFor(intent);
            source(designFvdDive(intent,cancel),Element::Hill,element);cliffFoot=cursor.position;
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;t.cliffX=cliffTop.x+std::cos(cliffYaw)*7;t.cliffY=cliffTop.y+std::sin(cliffYaw)*7;t.cliffHeading=cliffYaw;}
            break;
        }
        case RideRole::DownhillLaunch:{
            const auto& p=std::get<OperationParameters>(element.parameters);const double target=(p.targetSpeedKmh>0?p.targetSpeedKmh/3.6:req.targets.speed+.15)+feedback.peakSpeedCorrection;
            const double acceleration=p.accelerationMps2>0?p.accelerationMps2:16;
            const double net=acceleration-gravity*cursor.tangent.z-gravity*req.train.rollingResistance-motion.drag*target*target;
            const double length=p.lengthMeters>0?p.lengthMeters:std::max(120.,(target*target-motion.nominalSpeed*motion.nominalSpeed)/(2*net)+target*.85+4*halfTrain+6);
            motion.drive(length,DriveKind::Boost,target,acceleration,element.id.c_str());
            landmarks.push_back({LandmarkKind::DownhillLaunchExit,d.track.knots.size()-1});
            freeDirection({0,0,0,0},{heading(),0,0,0},target*1.4,Element::Hill,element.id+"-valley");
            if(cursor.position.z<datum)throw std::runtime_error("Inclined launch exhausted the authored cliff-foot relief");
            break;
        }
        case RideRole::Camelback:{
            const auto& p=std::get<CamelbackParameters>(element.parameters);FvdCamelbackRequest protectedHill;auto& intent=protectedHill.hill;
            intent.entrySpeed=speedFor(element.id);const double scale=intent.entrySpeed/83.5,timeScale=scale*std::sqrt(p.profileScale);
            intent.height=224.654746724*p.profileScale*scale*scale;intent.exitHeight=0;intent.exitPitch=.12;
            intent.positiveG=2.934128338;intent.exitPositiveG=4.005031584;intent.airtimeG=-.895986970;
            intent.rampSeconds=2.425134587*timeScale;intent.ascentReleaseSeconds=2.907610573*timeScale;intent.exitRampSeconds=3.286095368*timeScale;intent.crestLoadChangeG=-.060263335;
            protectedHill.tailCutSeconds=p.tailCutSeconds*timeScale;protectedHill.releaseSeconds=p.releaseSeconds*timeScale;protectedHill.exitNormalG=p.exitNormalG;protectedHill.minimumExitPitch=p.minimumExitPitchDegrees*pi/180;
            lossFor(intent);auto protectedResult=designFvdCamelback(protectedHill,cancel);
            source({std::move(protectedResult.authoring),std::move(protectedResult.section)},Element::Hill,element);break;
        }
        case RideRole::Wave:{
            const auto& p=std::get<TurnParameters>(element.parameters);FvdWaveRequest intent;
            waveBase=cursor.position;intent.entrySpeed=speedFor(element.id);intent.entryPitch=std::asin(cursor.tangent.z);
            const auto upright=unit(Vec3{0,0,1}-cursor.tangent*cursor.tangent.z);
            intent.entryNormalG=dot(cursor.curvature*(intent.entrySpeed*intent.entrySpeed)+Vec3{0,0,gravity},upright)/gravity;intent.entryHoldSeconds=1.8;
            intent.height=p.riseMeters;intent.exitHeight=10;intent.turnAngle=p.headingDegrees*pi/180;intent.bankAngle=p.bankDegrees*pi/180;intent.exitNormalG=p.exitNormalG;
            lossFor(intent);source(designFvdWave(intent,cancel),Element::Turn,element);break;
        }
        case RideRole::Loop:{
            const auto& p=std::get<InversionParameters>(element.parameters);FvdLoopRequest intent;
            loopBase=cursor.position;intent.entrySpeed=speedFor(element.id);intent.height=p.referenceRiseMeters*std::pow(intent.entrySpeed/65,2);intent.yawAngle=p.yawDegrees*pi/180;intent.crestG=p.crestG;intent.exitPitch=p.exitPitchDegrees*pi/180;intent.exitNormalG=1.5;
            lossFor(intent);const auto loop=designFvdLoop(intent,cancel);double begin=0;
            if(loop.section.report.valid())while(begin<loop.section.track.length*.35&&loop.section.track.sample(begin).tangent.z<std::sin(p.entryPitchDegrees*pi/180))begin+=.5;
            source(loop,Element::Inversion,element,begin);break;
        }
        case RideRole::Immelmann:{
            const auto& p=std::get<InversionParameters>(element.parameters);FvdImmelmannRequest intent;
            intent.entrySpeed=speedFor(element.id);intent.height=p.referenceRiseMeters*std::pow(intent.entrySpeed/53,2);intent.exitHeight=std::clamp(datum+70-cursor.position.z,-160.,35.);intent.exitPitch=p.exitPitchDegrees*pi/180;
            intent.normalG=3.8;intent.crestG=p.crestG;intent.rollExitG=.25;intent.rampSeconds=1.4;intent.rollOverlapFraction=.45;intent.exitNormalG=std::cos(intent.exitPitch);intent.hand=-1;lossFor(intent);
            const auto inversion=designFvdImmelmann(intent,cancel);FvdHillResult result{inversion.authoring,inversion.section};double begin=0;
            if(result.section.report.valid())while(begin<result.section.track.length*.35&&result.section.track.sample(begin).tangent.z<std::sin(p.entryPitchDegrees*pi/180))begin+=.5;
            source(result,Element::Inversion,element,begin);immelExit=cursor.position;break;
        }
        case RideRole::Signature:{
            const auto& p=std::get<SweepParameters>(element.parameters);const double bank=p.rollDegrees*(req.style.signatureRollDegrees/45)*pi/180;
            // A descending outward roll over the ravine shoulder. It spends
            // existing elevation and ends in a positively loaded low carve.
            const auto act=terrainAct(element,TerrainAct::RavineRoll,std::max(p.riseMeters,datum+6-cursor.position.z),p.headingDegrees*pi/180,bank,std::max(-1.32,p.negativeG*req.style.airtime),p.lengthMeters/260,p.exitPitchDegrees*pi/180);
            source(act,Element::Turn,element);
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;const auto mid=(immelExit+cursor.position)*.5;t.foothills.push_back({mid.x+110,mid.y+90,std::max(0.,immelExit.z-12),240});t.ravines.push_back({immelExit.x,immelExit.y,cursor.position.x,cursor.position.y,60,20,100,80});}
            break;
        }
        case RideRole::Return:{
            std::visit([&](const auto& p){using P=std::decay_t<decltype(p)>;
                if constexpr(std::is_same_v<P,HillParameters>){
                    FvdHillRequest intent;intent.entrySpeed=speedFor(element.id);intent.height=p.riseMeters*p.profileScale;
                    intent.positiveG=p.pulloutG;intent.airtimeG=std::max(-1.32,p.negativeG*req.style.airtime);intent.rampSeconds=p.releaseSeconds;intent.exitRampSeconds=p.recoverySeconds;
                    intent.twistAngle=(p.twistDegrees+(req.style.returnStyle?25.:0))*pi/180;intent.exitHeight=0;intent.exitPitch=-.12;lossFor(intent);
                    const auto hill=designFvdHill(intent,cancel);double begin=0;
                    source(hill,Element::Airtime,element,begin);
                }else if constexpr(std::is_same_v<P,TurnParameters>){
                    const double angle=p.headingDegrees*pi/180+turnVariation;
                    const double length=std::max(p.lengthMeters,1.875*std::abs(angle)*motion.nominalSpeed*motion.nominalSpeed/(gravity*std::tan(std::abs(p.bankDegrees)*pi/180)));
                    auto end=detail::planarJet({cursor.position.x,cursor.position.y,cursor.position.z+p.riseMeters},heading()+angle,0,(p.exitNormalG-1)*gravity/(motion.nominalSpeed*motion.nominalSpeed));
                    auto program=solveHeightMotion(cursor,end,length,{motion.nominalSpeed,gravity*req.train.rollingResistance,motion.drag,0,4.4,-.8});
                    motion.programme(program,Element::Turn,element.id.c_str());motion.modules.back().reversals=1;
                }else if constexpr(std::is_same_v<P,SweepParameters>){
                    if(element.anchor!=TerrainAnchor::Approach){
                        auto end=detail::planarJet({cursor.position.x,cursor.position.y,cursor.position.z+p.riseMeters},heading()+p.headingDegrees*pi/180,p.exitPitchDegrees*pi/180);
                        motion.programme(solveHeightMotion(cursor,end,p.lengthMeters,{motion.nominalSpeed,gravity*req.train.rollingResistance,motion.drag,p.rollDegrees*pi/180,4.4,-.8}),Element::Turn,element.id.c_str(),p.rollDegrees*pi/180);
                        motion.modules.back().reversals=1;return;
                    }
                    // Resolve the return outside the broad plateau footprint,
                    // before constructing its two smooth low corridors.
                    const double side=cursor.position.y>=plateauCenter.y?1.:-1.;
                    const Vec3 gate{plateauCenter.x-640,side*std::max(450.,side*plateauCenter.y+584),datum+p.riseMeters};
                    const Vec3 approach{-450,side*250,datum+p.riseMeters};
                    const auto incoming=unit(Vec3{gate.x-cursor.position.x,gate.y-cursor.position.y,0}),outgoing=unit(Vec3{approach.x-gate.x,approach.y-gate.y,0});
                    const auto direction=unit(incoming+outgoing);const double yaw=std::atan2(direction.y,direction.x)+p.headingDegrees*pi/180;
                    auto gatePose=detail::directionJet({0,0,0,0},{yaw,std::copysign(1./600,cross(incoming,outgoing).z),0,0});gatePose.position=gate;
                    const size_t begin=d.track.knots.size()-1;
                    motion.curve(gatePose,std::max(p.lengthMeters,norm(gate-cursor.position)*1.12),Element::Turn,(element.id+"-foothill").c_str(),p.rollDegrees*pi/180);
                    motion.curve(detail::planarJet(approach,-side*pi/2,p.exitPitchDegrees*pi/180),std::max(p.lengthMeters,norm(approach-cursor.position)*1.12),Element::Turn,(element.id+"-station-approach").c_str());
                    for(size_t k=begin;k<d.track.knots.size();++k)if(d.track.knots[k].position.z<datum-.05)throw std::runtime_error("Low return corridor left its basin floor; no clearance lift is inserted");
                }else throw std::runtime_error("Unsupported return parameter family");
            },element.parameters);break;
        }
        case RideRole::Brakes:{
            const auto& p=std::get<OperationParameters>(element.parameters);
            const double alignment=std::max(24.,motion.nominalSpeed);
            const Vec3 destination{-p.lengthMeters-alignment,0,datum};
            motion.curve(detail::planarJet(destination,0,0),norm(destination-cursor.position)*1.25,Element::Turn,"station-return");
            motion.line(alignment,Element::Brake,"brake-alignment");
            brakeBegin=d.track.knots.size()-1;motion.line(p.lengthMeters,Element::Brake,element.id.c_str());break;
        }
        default:throw std::runtime_error("Unspecified recipe role");
        }}catch(const std::exception& failure){
            // Preserve the actual authored prefix for diagnosis. It is open,
            // unsimulated and cannot be accepted, saved, or committed to Play.
            d.track.closed=false;if(d.track.knots.size()>=4)d.track.rebuild();
            auto at=[&](size_t knot){return knot>=d.track.spans.size()?d.track.length:d.track.spans[knot].start;};
            compiled.push_back({moduleBegin,motion.modules.size(),element.role,element.id});
            for(const auto& done:compiled)for(size_t j=done.moduleBegin;j<done.moduleEnd;++j){const auto& m=motion.modules[j];d.sections.push_back({m.name,at(m.begin),at(m.end),m.reversals,m.planar,done.role,done.id});}
            std::ostringstream diagnostic;diagnostic<<"{\"schemaVersion\":4,\"generationOnly\":true,\"partial\":true,\"failedElement\":"<<std::quoted(element.id)<<'}';d.planningDiagnostics=diagnostic.str();
            throw RecipeCompileFailure(element.id+": "+failure.what(),std::move(d));
        }
        compiled.push_back({moduleBegin,motion.modules.size(),element.role,element.id});
    }
    if(norm(cursor.position-d.track.knots.front().position)>1e-7||norm(cursor.tangent-d.track.knots.front().tangent)>1e-7||norm(cursor.curvature)>1e-7||norm(cursor.third)>1e-7||norm(cursor.fourth)>1e-7)
        throw std::runtime_error("Recipe station closure failed its complete endpoint jet");
    d.track.knots.back()=d.track.knots.front();
    for(auto& k:d.track.knots){k.position.y*=hand;k.tangent.y*=hand;k.curvature.y*=hand;k.third.y*=hand;k.fourth.y*=hand;k.up.y*=hand;k.upFirst.y*=hand;k.upSecond.y*=hand;k.upThird.y*=hand;k.bank*=hand;}
    auto& terrain=d.request.terrain;terrain.centerY*=hand;terrain.bend*=hand;terrain.cliffY*=hand;terrain.cliffHeading*=hand;
    if(terrain.backSlope){terrain.backSlope->y*=hand;terrain.backSlope->gradeY*=hand;}
    for(auto& ramp:terrain.ramps){ramp.y0*=hand;ramp.y1*=hand;}for(auto& knoll:terrain.knolls)knoll.y*=hand;for(auto& ravine:terrain.ravines){ravine.y0*=hand;ravine.y1*=hand;}
    for(auto& foothill:terrain.foothills)foothill.y*=hand;
    for(auto& program:d.forcePrograms)program.hand=hand;for(auto& program:d.splinePrograms)program.hand=hand;
    if(!terrain.valid())throw std::runtime_error("Resolved semantic terrain is outside its bounded domain");
    d.track.rebuild();auto at=[&](size_t i){return i>=d.track.spans.size()?d.track.length:d.track.spans[i].start;};
    for(const auto& motor:motion.motors){const double ramp=motor.kind==DriveKind::Launch?departureRamp(motor.acceleration,req.limits):.65;
        d.operations.push_back({at(motor.begin)+(motor.kind==DriveKind::Launch?3:2*halfTrain+3),at(motor.end)-2*halfTrain-3,motor.kind,motor.speed,req.train.carMass*motor.acceleration,req.train.carMass*motor.acceleration*110,ramp});}
    d.operations.push_back({at(brakeBegin)+2*halfTrain+3,80,DriveKind::Station,0,req.train.carMass*7,req.train.carMass*700,.5,6,1.5});
    for(auto& operation:d.operations)operation.exitFadeMeters=std::max(1.,operation.targetSpeed*operation.rampSeconds);
    for(const auto& element:compiled)for(size_t i=element.moduleBegin;i<element.moduleEnd;++i){const auto& m=motion.modules[i];d.sections.push_back({m.name,at(m.begin),at(m.end),m.reversals,m.planar,element.role,element.id});}
    landmarks.push_back({LandmarkKind::BrakeEntry,brakeBegin});for(const auto& [kind,knot]:landmarks)d.landmarks.push_back({kind,at(knot)});
    ports.clear();for(size_t i=0;i<sourcePorts.size();++i)ports.push_back({sourcePorts[i].first,at(sourcePorts[i].second),sourceSpeeds[i]});
    d.topology="recipe/"+d.request.recipe.name;std::ostringstream diagnostics;diagnostics<<std::setprecision(12)<<"{\"schemaVersion\":4,\"generationOnly\":true,\"recipeVersion\":"<<req.recipe.version<<",\"seed\":"<<req.seed<<",\"candidate\":"<<attempt<<",\"hand\":"<<hand<<",\"ports\":[";
    for(size_t i=0;i<ports.size();++i){if(i)diagnostics<<',';diagnostics<<"{\"id\":"<<std::quoted(ports[i].id)<<",\"distance\":"<<ports[i].distance<<",\"plannedSpeed\":"<<ports[i].speed<<'}';}
    diagnostics<<"],\"terrainAnchors\":[";bool comma=false;
    for(const auto& [name,position]:std::array<std::pair<const char*,Vec3>,5>{{{"plateau",plateauArrival},{"cliff-foot",cliffFoot},{"wave-bench",waveBase},{"loop-basin",loopBase},{"immelmann-shoulder",immelExit}}}){
        if(comma)diagnostics<<',';comma=true;diagnostics<<"{\"name\":"<<std::quoted(name)<<",\"position\":["<<position.x<<','<<position.y*hand<<','<<position.z<<"]}";}
    diagnostics<<"]}";d.planningDiagnostics=diagnostics.str();return d;
}
}
