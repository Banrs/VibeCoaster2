#include "recipe_compiler.hpp"
#include "coaster/operation_hardware.hpp"
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
    double cliffYaw=0,brakeCapacity=7,brakeAlignment=0;std::optional<size_t> cliffLowRavine;
    Vec3 openingRecovery{};std::array<Vec3,2> openingCrests{},openingValleys{};
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
            const double ramp=motor.rampSeconds>0?motor.rampSeconds:motor.kind==DriveKind::Launch?departureRamp(motor.acceleration,req.limits):.65;
            // Brake contact is local to each car. Neighboring cars may already
            // enter/leave the curved approach while the jaws remain on straight
            // rail. Powered contact reserves only the local car/stator span.
            const double entryMargin=motor.kind==DriveKind::Boost?poweredAlignmentMargin():3;
            const double exitMargin=motor.kind==DriveKind::Boost?poweredAlignmentMargin():motor.kind==DriveKind::Brake?3:2*halfTrain+3;
            d.operations.push_back({at(motor.begin)+entryMargin,at(motor.end)-exitMargin,motor.kind,motor.speed,req.train.carMass*motor.acceleration,req.train.carMass*motor.acceleration*110,ramp});
            d.operations.back().exitFadeMeters=motor.exitFadeMeters;
        }
        if(station)d.operations.push_back({at(brakeBegin)+2*halfTrain+3,80,DriveKind::Station,0,req.train.carMass*brakeCapacity,req.train.carMass*brakeCapacity*100,.5,std::min(6.,brakeCapacity*6/7),1.5});
        for(auto& operation:d.operations)if(operation.exitFadeMeters<=0)operation.exitFadeMeters=std::max(1.,operation.targetSpeed*operation.rampSeconds);
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
    auto groundLandscape=[&](Vec3 crest,double yaw,double summit,double windingHand){
        plateauCenter=crest+rotated(Vec3{0,160*windingHand,0},yaw);
        if(req.terrain.kind!=TerrainKind::Highlands)return;
        auto& t=d.request.terrain;t.ridge={};t.ramps.clear();t.knolls.clear();t.foothills.clear();t.ravines.clear();
        const double openingGrade=(openingValleys.back().z-openingRecovery.z)/std::hypot(openingValleys.back().x-openingRecovery.x,openingValleys.back().y-openingRecovery.y);
        t.ramps.push_back({openingRecovery.x,openingRecovery.y,openingValleys.back().x,openingValleys.back().y,
            openingRecovery.z-datum,openingValleys.back().z-datum,openingGrade,openingGrade,140});
        t.foothills.push_back({openingCrests[0].x,openingCrests[0].y,std::max(0.,openingCrests[0].z-17.5),190});
        t.foothills.push_back({openingCrests[1].x,openingCrests[1].y,std::max(0.,openingCrests[1].z-13.5),170});
        const auto center=plateauCenter;
        t.centerX=center.x;t.centerY=center.y;t.heightMeters=std::max(0.,summit-2.5);
        t.backSlope=TerrainSlope{crest.x,crest.y,t.heightMeters,.65*std::cos(yaw),.65*std::sin(yaw),60};
        t.radiusX=420;t.radiusY=380;t.plateau=.72;t.bend=.06;
        t.cliffX=crest.x+500*std::cos(yaw);t.cliffY=crest.y+500*std::sin(yaw);t.cliffHeading=yaw;t.cliffWidth=24;t.cliffCurvature=.00022;
        // The connected shelf and broad foothills are semantic landforms, not
        // a dense copy of rail samples. Later ports determine their final front.
        for(auto local:std::array<Vec3,2>{{{-160,-220,70},{260,-260,95}}}){
            local.y*=windingHand;const auto p=crest+rotated(local,yaw);t.foothills.push_back({p.x,p.y,local.z,340});}
    };
    for(const auto& element:req.recipe.elements){
        if(energyPrefix&&element.role==RideRole::Return&&element.anchor==TerrainAnchor::Approach)break;
        if(energyPrefix&&element.role==RideRole::Brakes)break;
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        size_t moduleBegin=motion.modules.size();RideRole compiledRole=element.role;std::string compiledId=element.id;
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
            openingRecovery=cursor.position;
            for(int n=0;n<2;++n){
                auto child=element;child.id=element.id+(n==0?"-river-hop":"-rising-bank");child.role=RideRole::Unspecified;
                FvdHillRequest hill;hill.entry=motion.fvdEntry(speedFor(child.id));hill.height=n==0?45:35;hill.exitHeight=0;hill.exitPitch=.02;
                hill.positiveG=2.2;hill.airtimeG=-.4;hill.rampSeconds=.65;hill.exitRampSeconds=.8;hill.exitPositiveG=2;lossFor(hill);
                const auto result=designFvdHill(hill,cancel);source(result,Element::Airtime,child,{.worldCoordinates=true});
                openingCrests[n]=std::max_element(result.section.samples.begin(),result.section.samples.end(),[](const FvdSample& a,const FvdSample& b){return a.position.z<b.position.z;})->position;
                openingValleys[n]=cursor.position;
            }
            break;
        }
        case RideRole::CliffApproach:{
            const auto& p=std::get<CliffParameters>(element.parameters);const double summit=datum+p.summitHeightMeters;
            const double layoutHeading=recipeClimbSetupHeading+(feedback.compactReturn?feedback.cliffHeadingCorrection:attempt==1?-4*pi/180:attempt==2?4*pi/180:0);
            auto setupElement=element;setupElement.id=element.id+"-recovery";
            FvdClimbSetupRequest setupIntent;setupIntent.entry=motion.fvdEntry(speedFor(setupElement.id));setupIntent.headingChange=layoutHeading;lossFor(setupIntent);
            const auto setup=designFvdClimbSetup(setupIntent,cancel);source(setup,Element::Turn,setupElement,{.worldCoordinates=true});
            const auto ascentStart=cursor;
            const std::string gradeId=element.id+"-ascent-grade-blend";
            const auto gradeCorrection=feedback.energyCorrection.find(gradeId);
            const double gradeSpeed=std::sqrt(42*42+(gradeCorrection==feedback.energyCorrection.end()?0:gradeCorrection->second));
            motion.driveGraded(summit-55,32*pi/180,45.2,16,(element.id+"-ascent").c_str(),gradeSpeed);
            sourcePorts.push_back({gradeId,d.forcePrograms.back().firstKnot});sourceSpeeds.push_back(gradeSpeed);
            const auto ascentEnd=cursor;
            auto arrivalElement=element;arrivalElement.id=element.id+"-arrival";
            FvdPlateauArrivalRequest arrivalIntent;arrivalIntent.entry=motion.fvdEntry(speedFor(arrivalElement.id));arrivalIntent.rise=summit-cursor.position.z;lossFor(arrivalIntent);
            const auto arrival=designFvdPlateauArrival(arrivalIntent,cancel);source(arrival,Element::Hill,arrivalElement,{.worldCoordinates=true});
            plateauArrival=cursor.position;const double arrivalYaw=heading();const double windingHand=p.outwardBankDegrees<0?-1.:1.;groundLandscape(plateauArrival,arrivalYaw,summit,windingHand);
            landmarks.push_back({LandmarkKind::PlateauArrival,d.track.knots.size()-1});
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;
                // A single broad rising spur supports the loaded climb setup.
                // Sparse terrain anchors describe its changing grade; the mesh
                // and the complete banked train retain independent clearance.
                t.ridge.width=200;t.ridge.curvature=.001;
                for(int piece=0;piece<=8;++piece){const double at=setup.section.samples.back().time*piece/8;
                    const auto q=std::lower_bound(setup.section.samples.begin(),setup.section.samples.end(),at,[](const FvdSample& sample,double time){return sample.time<time;});
                    const auto& point=q==setup.section.samples.end()?setup.section.samples.back():*q;
                    const double horizontal=point.forward.x*point.forward.x+point.forward.y*point.forward.y;
                    t.ridge.points.push_back({point.position.x,point.position.y,std::max(0.,point.position.z-3.5),point.forward.z*point.forward.x/horizontal,point.forward.z*point.forward.y/horizontal});
                }
                t.ridge.spineCount=t.ridge.points.size();
                t.ramps.push_back({ascentStart.position.x,ascentStart.position.y,ascentEnd.position.x,ascentEnd.position.y,std::max(0.,ascentStart.position.z-2.25),std::max(0.,ascentEnd.position.z-2.25),std::tan(20*pi/180),std::tan(32*pi/180),190});
                // Three broad connected benches follow the coasting grade
                // release. A ramp ending at the motor left a visible notch.
                Vec3 previous=ascentEnd.position;double gap=2.25;
                for(int piece=1;piece<=3;++piece){const double at=arrival.section.samples.back().time*piece/3;
                    const auto q=std::lower_bound(arrival.section.samples.begin(),arrival.section.samples.end(),at,[](const FvdSample& sample,double time){return sample.time<time;});
                    const Vec3 end=q==arrival.section.samples.end()?arrival.section.samples.back().position:q->position;
                    const double h0=previous.z-gap,h1=end.z-3,grade=(h1-h0)/std::hypot(end.x-previous.x,end.y-previous.y);
                    t.ramps.push_back({previous.x,previous.y,end.x,end.y,h0,h1,grade,grade,140});previous=end;gap=3;
                }
            }
            auto windingElement=element;windingElement.id=element.id+"-clifftop";
            FvdWindingCliffRequest windingIntent;windingIntent.entry=motion.fvdEntry(speedFor(windingElement.id));windingIntent.headingChange=p.approachHeadingDegrees*pi/180+feedback.cliffDepartureHeadingCorrection;
            windingIntent.outwardBank=p.outwardBankDegrees*pi/180;windingIntent.durationScale=p.approachLengthMeters/580;lossFor(windingIntent);
            const auto winding=designFvdWindingCliff(windingIntent,cancel);source(winding,Element::Turn,windingElement,{.worldCoordinates=true});
            auto atTime=[&](double seconds)->const FvdSample&{const double time=seconds*windingIntent.durationScale;
                const auto q=std::lower_bound(winding.section.samples.begin(),winding.section.samples.end(),time,[](const FvdSample& sample,double at){return sample.time<at;});return q==winding.section.samples.end()?winding.section.samples.back():*q;};
            const auto edge=atTime(8);const Vec3 forward=unit(Vec3{edge.forward.x,edge.forward.y,0}),right{forward.y,-forward.x,0};
            outbankBay=edge.position+right*(17*windingHand);
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;const Vec3 first=atTime(13).position,low=atTime(18).position;
                const double a=std::max(0.,t.heightMeters-first.z+3.7),b=std::max(0.,t.heightMeters-low.z+3.7),c=std::max(0.,t.heightMeters-cursor.position.z+2.75);
                t.ravines.push_back({first.x,first.y,low.x,low.y,a,b,170,170});
                cliffLowRavine=t.ravines.size();t.ravines.push_back({low.x,low.y,cursor.position.x,cursor.position.y,b,c,170,170});
                // One connected side canyon exposes the outward-banked rim;
                // the later low traverse stays on the inland mesa.
                const double bayAngle=160*pi/180;
                const Vec3 mouth=outbankBay+(forward*std::cos(bayAngle)+right*(windingHand*std::sin(bayAngle)))*450;
                t.ravines.push_back({outbankBay.x,outbankBay.y,mouth.x,mouth.y,t.heightMeters,t.heightMeters,25,35});
            }
            break;
        }
        case RideRole::CliffLip:{
            const auto& p=std::get<OperationParameters>(element.parameters);const double target=p.targetSpeedKmh/3.6;
            const double acceleration=p.accelerationMps2>0?p.accelerationMps2:7;
            const double length=std::max(p.lengthMeters,(motion.nominalSpeed*motion.nominalSpeed-target*target)/(2*acceleration)+motion.nominalSpeed*.5+6);
            motion.drive(length,DriveKind::Brake,target,acceleration,element.id.c_str());break;
        }
        case RideRole::CliffDrop:{
            const auto& p=std::get<CliffParameters>(element.parameters);FvdDiveRequest intent;
            cliffTop=cursor.position;cliffYaw=heading();intent.entrySpeed=speedFor(element.id);
            if(cliffLowRavine&&req.terrain.kind==TerrainKind::Highlands){auto& low=d.request.terrain.ravines[*cliffLowRavine];low.x1=cliffTop.x;low.y1=cliffTop.y;low.depth1=std::max(0.,d.request.terrain.heightMeters-cliffTop.z+2.75);}
            landmarks.push_back({LandmarkKind::CliffDeparture,d.track.knots.size()-1});
            const auto boost=std::find_if(req.recipe.elements.begin(),req.recipe.elements.end(),[](const RecipeElement& item){return item.role==RideRole::DownhillLaunch;});
            const auto& boostParameters=std::get<OperationParameters>(boost->parameters);
            intent.drop=cliffTop.z-(datum+57);intent.maximumPitch=p.dropDegrees*pi/180;intent.exitPitch=boostParameters.gradeDegrees*pi/180;
            intent.crestRampSeconds=2.0;intent.crestG=-.9;intent.pulloutG=4.5;intent.pulloutRampSeconds=1.5;intent.exitRampSeconds=1.1;lossFor(intent);
            source(designFvdDive(intent,cancel),Element::Hill,element);cliffFoot=cursor.position;
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;t.cliffX=cliffTop.x+std::cos(cliffYaw)*7;t.cliffY=cliffTop.y+std::sin(cliffYaw)*7;t.cliffHeading=cliffYaw;
            }
            break;
        }
        case RideRole::DownhillLaunch:{
            const auto& p=std::get<OperationParameters>(element.parameters);const double target=(p.targetSpeedKmh>0?p.targetSpeedKmh/3.6:req.targets.speed+.15)+feedback.peakSpeedCorrection;
            const double acceleration=p.accelerationMps2>0?p.accelerationMps2:16;
            const double net=acceleration-gravity*cursor.tangent.z-gravity*req.train.rollingResistance-motion.drag*target*target;
            const double length=p.lengthMeters>0?p.lengthMeters:std::max(120.,(target*target-motion.nominalSpeed*motion.nominalSpeed)/(2*net)+target*.85+2*poweredAlignmentMargin());
            motion.drive(length,DriveKind::Boost,target,acceleration,element.id.c_str());
            landmarks.push_back({LandmarkKind::DownhillLaunchExit,d.track.knots.size()-1});
            if(cursor.position.z<datum)throw std::runtime_error("Inclined launch exhausted the authored cliff-foot relief");
            camelbackBase=cursor.position;
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;
                t.ramps.push_back({cliffFoot.x,cliffFoot.y,cursor.position.x,cursor.position.y,cliffFoot.z-2.25,cursor.position.z-2.25,std::tan(p.gradeDegrees*pi/180),0,140});
                const auto shoulder=(cliffFoot+cursor.position)*.5+rotated(Vec3{0,-180,0},heading());
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
            const size_t pulloutBegin=d.track.knots.size()-1;
            source(paired.pullout,Element::Hill,entry,{.allowConnector=false});
            if(req.terrain.kind==TerrainKind::Highlands){
                const auto low=std::min_element(d.track.knots.begin()+pulloutBegin,d.track.knots.end(),
                    [](const Knot& a,const Knot& b){return a.position.z<b.position.z;});
                // The cliff-foot bench continues through the real valley, so
                // ordinary launch recovery stays close to connected ground.
                auto& bench=d.request.terrain.ramps.back();bench.x1=low->position.x;bench.y1=low->position.y;
                bench.h1=std::max(0.,low->position.z-2.25);bench.grade1=0;
            }
            // Retain the reference from its loaded ascent shoulder. The pullout
            // shares that source's pitch, curvature, force and energy jets.
            // One calibration variable owns the whole connected element.
            source({std::move(paired.camelback.authoring),std::move(paired.camelback.section)},
                Element::Hill,element,{.begin=paired.retainedBeginDistance,.allowConnector=false,.calibrate=false});
            break;
        }
        case RideRole::Wave:{
            const auto& p=std::get<TurnParameters>(element.parameters);waveBase=cursor.position;const double speed=speedFor(element.id);
            if(std::abs(p.headingDegrees-180)<1e-9&&std::abs(p.bankDegrees-84)<1e-9&&std::abs(p.riseMeters-70)<1e-9&&std::abs(p.exitNormalG-3)<1e-9){
                source(designFvdCompactWave(motion.fvdEntry(speed),gravity*req.train.rollingResistance,motion.drag,cancel),Element::Turn,element,{.worldCoordinates=true});
            }else{
                FvdWaveRequest intent;intent.entrySpeed=speed;intent.entryPitch=std::asin(cursor.tangent.z);intent.entry=motion.fvdEntry(speed);
                const auto upright=unit(Vec3{0,0,1}-cursor.tangent*cursor.tangent.z);
                intent.entryNormalG=dot(cursor.curvature*(speed*speed)+Vec3{0,0,gravity},upright)/gravity;intent.entryHoldSeconds=1.8;
                intent.height=p.riseMeters;intent.exitHeight=10;intent.turnAngle=p.headingDegrees*pi/180;intent.bankAngle=p.bankDegrees*pi/180;intent.exitNormalG=p.exitNormalG;
                lossFor(intent);source(designFvdWave(intent,cancel),Element::Turn,element,{.worldCoordinates=true});
            }
            break;
        }
        case RideRole::Loop:{
            const auto& p=std::get<InversionParameters>(element.parameters);
            auto braking=element;braking.id=element.id+"-energy-brake";braking.role=RideRole::Unspecified;
            FvdBrakedPitchRequest brakeIntent;brakeIntent.entry=motion.fvdEntry(speedFor(braking.id));brakeIntent.heightChange=20;brakeIntent.exitPitch=.18;brakeIntent.targetSpeed=recipeLoopEntrySpeed;lossFor(brakeIntent);
            const auto brake=designFvdBrakedPitch(brakeIntent,cancel);const size_t brakeStart=d.track.knots.size()-1;
            source(brake,Element::Brake,braking,{.worldCoordinates=true});
            const double brakeRamp=.43*brake.authoring.controls.back().time;
            double peakBrake=0;for(const auto& control:brake.authoring.controls)peakBrake=std::max(peakBrake,-control.drive);
            const auto fadeAt=std::lower_bound(brake.section.samples.begin(),brake.section.samples.end(),brake.authoring.controls.back().time-brakeRamp,
                [](const FvdSample& q,double time){return q.time<time;});
            const double fade=brake.section.samples.back().distance-fadeAt->distance;
            const auto correction=feedback.brakeAccelerationCorrection.find(braking.id);
            const double hardwareBrake=std::clamp(peakBrake+(correction==feedback.brakeAccelerationCorrection.end()?0:correction->second),.2*gravity,.8*gravity);
            motion.motors.push_back({brakeStart,d.track.knots.size()-1,DriveKind::Brake,brakeIntent.targetSpeed,hardwareBrake,brakeRamp,fade});
            compiled.push_back({moduleBegin,motion.modules.size(),RideRole::Unspecified,braking.id});moduleBegin=motion.modules.size();
            FvdLoopRequest intent;loopBase=cursor.position;intent.entrySpeed=brakeIntent.targetSpeed;
            intent.height=std::min(81.8,p.referenceRiseMeters);intent.yawAngle=p.yawDegrees*pi/180;intent.crestG=p.crestG;intent.exitPitch=p.exitPitchDegrees*pi/180;
            intent.exitNormalG=2;intent.rampSeconds=.8;intent.normalG=4.75;intent.exitPositiveG=4.65;intent.crossingOffset=std::abs(p.yawDegrees)<1e-9?8:0;intent.ascentReleaseSeconds=intent.crossingOffset!=0?0:1.2;
            intent.entry=motion.fvdEntry(intent.entrySpeed);lossFor(intent);source(designFvdLoop(intent,cancel),Element::Inversion,element,{.worldCoordinates=true});
            compiled.push_back({moduleBegin,motion.modules.size(),RideRole::Loop,element.id});moduleBegin=motion.modules.size();
            // A short upright force-axis connection keeps the positively
            // loaded path without adding a roll pulse or filler airtime crest.
            auto connection=element;connection.id=element.id+"-bank-connection";connection.role=RideRole::Unspecified;
            source(designFvdInversionLink(motion.fvdEntry(speedFor(connection.id)),gravity*req.train.rollingResistance,motion.drag,cancel),Element::Turn,connection,{.worldCoordinates=true});
            compiledRole=RideRole::Unspecified;compiledId=connection.id;
            break;
        }
        case RideRole::Immelmann:{
            const auto& p=std::get<InversionParameters>(element.parameters);FvdImmelmannRequest intent;
            intent.entrySpeed=speedFor(element.id);intent.height=std::clamp(p.referenceRiseMeters,65.,99.6);intent.exitHeight=std::abs(p.yawDegrees)<1e-9?20:10;intent.exitPitch=p.exitPitchDegrees*pi/180;
            intent.normalG=4.9;intent.exitPositiveG=3;intent.crestG=p.crestG;intent.rollExitG=.6;intent.rollReleaseFraction=0;intent.planarRoll=std::abs(p.yawDegrees)<1e-9;intent.ascentReleaseSeconds=intent.planarRoll?0:1.2;intent.rampSeconds=.8;intent.exitRampSeconds=1.2;intent.rollOverlapFraction=intent.planarRoll?.35:0;intent.yawAngle=p.yawDegrees*pi/180;intent.exitNormalG=1;intent.hand=-1;lossFor(intent);
            intent.entry=motion.fvdEntry(intent.entrySpeed);const auto inversion=designFvdImmelmann(intent,cancel);
            source({inversion.authoring,inversion.section},Element::Inversion,element,{.worldCoordinates=true});immelExit=cursor.position;break;
        }
        case RideRole::Signature:{
            const auto& p=std::get<SweepParameters>(element.parameters);
            const Vec3 signatureEntry=cursor.position;
            FvdSignatureRequest intent;intent.entry=motion.fvdEntry(speedFor(element.id));intent.exitHeight=std::max(datum+2,cursor.position.z+p.riseMeters);
            intent.exitPitch=p.exitPitchDegrees*pi/180;intent.bank=p.rollDegrees*(req.style.signatureRollDegrees/45)*pi/180;
            intent.headingChange=p.headingDegrees*pi/180;intent.durationScale=p.lengthMeters/554;lossFor(intent);
            const auto wings=designFvdSignature(intent,cancel);source(wings,Element::Airtime,element,{.worldCoordinates=true});returnRavine=cursor.position;
            auto atTime=[&](double time)->const FvdSample&{const auto q=std::lower_bound(wings.section.samples.begin(),wings.section.samples.end(),time*intent.durationScale,[](const FvdSample& sample,double at){return sample.time<at;});return q==wings.section.samples.end()?wings.section.samples.back():*q;};
            const Vec3 transfer=atTime(6.6).position;
            const auto released=std::max_element(d.track.knots.begin()+d.forcePrograms.back().firstKnot,d.track.knots.end(),[](const Knot& a,const Knot& b){return a.position.z<b.position.z;});
            landmarks.push_back({LandmarkKind::SignatureRelease,size_t(released-d.track.knots.begin())});
            if(req.terrain.kind==TerrainKind::Highlands){auto& t=d.request.terrain;
                auto terrace=[&](Vec3 a,Vec3 b,double gap0,double gap1,double width){const double h0=std::max(0.,a.z-gap0),h1=std::max(0.,b.z-gap1),grade=(h1-h0)/std::hypot(b.x-a.x,b.y-a.y);
                    t.ramps.push_back({a.x,a.y,b.x,b.y,h0,h1,grade,grade,width});};
                // Two connected broad terraces support the ravine lows, while
                // leaving the opposing wings visibly above the stream valley.
                const double basin=std::max(0.,loopBase.z-datum);
                terrace(signatureEntry,transfer,std::max(5.,signatureEntry.z-basin),5,150);
                terrace(transfer,cursor.position,5,3.5,180);
                for(double time:{5.4,8.9}){const auto crest=atTime(time).position;t.foothills.push_back({crest.x,crest.y,std::max(0.,crest.z-17),120});}
                t.ravines.push_back({signatureEntry.x,signatureEntry.y,transfer.x,transfer.y,2,2,70,80});
                t.ravines.push_back({transfer.x,transfer.y,cursor.position.x,cursor.position.y,2,1,80,70});
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
                    const size_t hillBegin=d.track.knots.size()-1;
                    source(designFvdHill(intent,cancel),Element::Airtime,element,{.worldCoordinates=true,.allowConnector=false});
                    if(terminal&&req.terrain.kind==TerrainKind::Highlands){
                        const auto crest=std::max_element(d.track.knots.begin()+hillBegin,d.track.knots.end(),[](const Knot& a,const Knot& b){return a.position.z<b.position.z;});
                        d.request.terrain.foothills.push_back({crest->position.x,crest->position.y,std::max(0.,crest->position.z-18),160});
                    }
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
                    if(req.terrain.kind==TerrainKind::Highlands)d.request.terrain.ravines.push_back({returnRavine.x,returnRavine.y,cursor.position.x,cursor.position.y,3,2,70,90});
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
                    if(req.terrain.kind==TerrainKind::Highlands)d.request.terrain.ravines.push_back({cursor.position.x,cursor.position.y,approach.x,approach.y,16,8,100,150});
                    const size_t begin=d.track.knots.size()-1;
                    FvdApproachRequest intent;intent.entry=motion.fvdEntry(speedFor(element.id));
                    intent.endPosition=approach;intent.endHeading=0;intent.normalG=recipeApproachNormalG;intent.bankRampSeconds=recipeApproachBankRampSeconds;lossFor(intent);
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
            compiled.push_back({moduleBegin,motion.modules.size(),compiledRole,compiledId});
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
            diagnostic<<",\"layoutCorrectionRadians\":["<<feedback.cliffHeadingCorrection<<','<<feedback.cliffDepartureHeadingCorrection<<']';
            if(pending)diagnostic<<",\"pendingPort\":{\"id\":"<<std::quoted(pending->id)<<",\"distance\":"<<pending->distance<<",\"plannedSpeed\":"<<pending->speed<<'}';
            diagnostic<<'}';d.planningDiagnostics=diagnostic.str();
            throw RecipeCompileFailure(element.id+": "+failure.what(),std::move(d),ports,std::move(pending));
        }
        compiled.push_back({moduleBegin,motion.modules.size(),compiledRole,compiledId});
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
    for(auto& point:terrain.ridge.points){point.y*=hand;point.gy*=hand;}
    for(auto& program:d.forcePrograms)program.hand=hand;for(auto& program:d.splinePrograms)program.hand=hand;
    if(!terrain.valid())throw std::runtime_error("Resolved semantic terrain is outside its bounded domain");
    finalizeKnownPrefix(!energyPrefix);
    d.topology="recipe/"+d.request.recipe.name;std::ostringstream diagnostics;diagnostics<<std::setprecision(12)<<"{\"schemaVersion\":4,\"generationOnly\":true,\"recipeVersion\":"<<req.recipe.version<<",\"seed\":"<<req.seed<<",\"candidate\":"<<attempt<<",\"hand\":"<<hand<<",\"layoutCorrectionRadians\":["<<feedback.cliffHeadingCorrection<<','<<feedback.cliffDepartureHeadingCorrection<<"],\"ports\":[";
    for(size_t i=0;i<ports.size();++i){if(i)diagnostics<<',';diagnostics<<"{\"id\":"<<std::quoted(ports[i].id)<<",\"distance\":"<<ports[i].distance<<",\"plannedSpeed\":"<<ports[i].speed<<'}';}
    diagnostics<<"],\"forceCorrections\":{\"camelbackTailCutSeconds\":"<<feedback.camelbackTailCutSeconds<<"},\"terrainAnchors\":[";bool comma=false;
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
