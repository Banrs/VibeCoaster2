#include "coaster/coaster.hpp"
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
using namespace coaster;
namespace {
int checks=0;
void check(bool value,const char* message){++checks;if(!value)throw std::runtime_error(message);}
void checkInversions(const Design& d){
    check(d.inversionDimensions.size()==2,"The opening loop and later reversal remain real inversions");
    int fullLoops=0,reversals=0;
    for(const auto& region:d.inversionDimensions){
        bool ascending=false,inverted=false;double apex=0;
        const auto first=d.track.sample(region.startDistance),last=d.track.sample(region.endDistance);
        const Vec3 forward=unit(Vec3{first.tangent.x,first.tangent.y,0});
        const double initialPitch=std::atan2(first.tangent.z,dot(first.tangent,forward));double pitch=initialPitch;
        for(double s=region.startDistance;s<=region.endDistance;s+=.25){const auto q=d.track.sample(s);
            ascending|=q.tangent.z>.98;
            pitch+=std::remainder(std::atan2(q.tangent.z,dot(q.tangent,forward))-pitch,2*pi);
            if(q.up.z<-.5){inverted=true;apex=std::max(apex,q.position.z);}}
        check(ascending&&inverted&&apex>=d.request.targets.inversionHeight,"Each inversion reaches upright ascent, genuine inversion and its requested height");
        // A laterally separated loop need not contain an exactly vertical descent.
        // Its accumulated tangent rotation, unlike a single pitch sample,
        // distinguishes a full loop from a half-loop heading reversal.
        if(std::abs(pitch-initialPitch)>1.5*pi)++fullLoops;
        else{++reversals;check(dot(unit(Vec3{first.tangent.x,first.tangent.y,0}),unit(Vec3{last.tangent.x,last.tangent.y,0}))<-.8,"The Immelmann reverses the physical travel heading");}
    }
    check(fullLoops==1&&reversals==1,"Loop and high Immelmann have distinct measured pitch topologies");
}
void checkPropulsionCorridors(const Design& design){
    const double carAlignmentMargin=(design.request.train.cars-1)*design.request.train.spacing+1.5;
    const double trainSpan=(design.request.train.cars-1)*design.request.train.spacing;
    for(const auto& operation:design.operations){
        if(operation.kind!=DriveKind::Launch&&operation.kind!=DriveKind::Boost)continue;
        const auto first=design.track.sample(operation.start-carAlignmentMargin);
        const double heading=std::atan2(first.tangent.y,first.tangent.x);
        auto observe=[&](double distance){
            const auto k=sampleKinematics(design.track,distance);const auto& p=k.sample;
            const Vec3 upright=unit(Vec3{0,0,1}-p.tangent*p.tangent.z);
            check(std::abs(std::remainder(std::atan2(p.tangent.y,p.tangent.x)-heading,2*pi))<.001,"Every motor retains one fixed plan heading throughout the powered car footprint");
            check(std::abs(cross(p.tangent,p.curvature).z)<1e-5,"Every powered car remains in its aligned vertical plane");
            check(dot(p.up,upright)>.9998&&std::abs(dot(k.upS,p.right))<.001,"Each powered car remains upright and aligned with its motor");
        };
        for(double distance=operation.start-carAlignmentMargin;distance<operation.end+carAlignmentMargin;distance+=.125)observe(distance);
        observe(operation.end+carAlignmentMargin);
        for(const auto& span:design.track.spans)if(span.start>=operation.start-carAlignmentMargin&&span.start<=operation.end+carAlignmentMargin)observe(span.start);

        double actualWorkSeconds=0;
        for(size_t i=1;i<design.simulation.frames.size();++i){const auto& f=design.simulation.frames[i];const auto& before=design.simulation.frames[i-1];
            if(f.distance-trainSpan*.5<=operation.start||f.distance+trainSpan*.5>=operation.end||f.speed>=operation.targetSpeed-.1)continue;
            const double acceleration=(f.speed-before.speed)/(f.time-before.time);
            double grade=0;for(int car=0;car<design.request.train.cars;++car)grade+=design.track.sample(f.distance+trainSpan*.5-car*design.request.train.spacing).tangent.z/design.request.train.cars;
            const double resistance=gravity*design.request.train.rollingResistance+.5*design.request.train.airDensity*design.request.train.dragCdA*f.speed*f.speed/(design.request.train.cars*design.request.train.carMass);
            if(acceleration+gravity*grade+resistance>.1)actualWorkSeconds+=f.time-before.time;
        }
        check(actualWorkSeconds>.15,"Each section booster performs measured work below target speed");
    }

}
void checkTerminalBrake(const Design& design){
    const auto station=std::find_if(design.operations.begin(),design.operations.end(),[](const Operation& op){return op.kind==DriveKind::Station;});
    check(station!=design.operations.end(),"The terminal brake remains an explicit physical operation");
    const double trainSpan=(design.request.train.cars-1)*design.request.train.spacing;
    const double turnExit=station->start-trainSpan,finish=design.track.length+trainSpan*.5+30;
    const auto brakeEntry=design.track.sample(turnExit);
    const double heading=std::atan2(brakeEntry.tangent.y,brakeEntry.tangent.x);
    for(double distance=turnExit;distance<finish+trainSpan*.5;distance+=.25){const auto p=design.track.sample(distance);
        const Vec3 upright=unit(Vec3{0,0,1}-p.tangent*p.tangent.z);
        check(std::abs(std::remainder(std::atan2(p.tangent.y,p.tangent.x)-heading,2*pi))<.001&&std::abs(cross(p.tangent,p.curvature).z)<1e-5&&dot(p.up,upright)>.9998,"The full braking train is upright and plan-straight after clearing the final bank; a vertical grade is allowed");
    }
    for(double distance=design.track.length+design.station.boardingBegin;distance<=finish+trainSpan*.5;distance+=.25){const auto p=design.track.sample(distance);
        check(std::abs(p.tangent.z)<.001&&p.up.z>.9999,"The actual station bay and whole-train stopping continuation remain level across the seam");
    }
    auto timeAt=[&](double distance){auto right=std::lower_bound(design.simulation.frames.begin(),design.simulation.frames.end(),distance,[](const Frame& f,double s){return f.distance<s;});const auto& left=*(right-1);double u=(distance-left.distance)/(right->distance-left.distance);return left.time+u*(right->time-left.time);};
    const double entryTime=timeAt(turnExit);
    auto entry=std::lower_bound(design.simulation.frames.begin(),design.simulation.frames.end(),turnExit,[](const Frame& f,double s){return f.distance<s;});
    check(entry->speed>0&&entry->brakeWorkPerMass==design.simulation.frames[size_t(entry-design.simulation.frames.begin())-1].brakeWorkPerMass,"The return reaches its straight braking corridor under its remaining passive energy");
    check(design.simulation.frames.back().speed==0&&std::abs(design.simulation.frames.back().distance-finish)<.25,"Physical braking reaches actual zero speed at the unchanged station reference without a snap");
    std::cout<<"terminalBrakeSeconds="<<design.simulation.frames.back().time-entryTime<<'\n';
    for(size_t i=1;i<design.simulation.frames.size();++i){const auto& a=design.simulation.frames[i-1];const auto& b=design.simulation.frames[i];
        if(a.distance>=station->start&&a.speed>3&&b.speed>3)check(b.driveWorkPerMass==a.driveWorkPerMass,"Terminal brakes cannot accelerate the returning train; positioning tyres act only near rest");}
}
void checkC3Transitions(const Design& design){
    double largestUpJump=0,largestTangentJump=0;
    for(size_t i=0;i<design.track.spans.size();++i){
        const auto a=sampleSpanKinematics(design.track,i,1),b=sampleSpanKinematics(design.track,(i+1)%design.track.spans.size(),0);
        check(norm(a.sample.position-b.sample.position)<1e-8&&norm(a.sample.tangent-b.sample.tangent)<1e-9,"Every generated position/tangent join closes, including the station seam");
        check(norm(a.sample.curvature-b.sample.curvature)<1e-9&&norm(a.curvatureS-b.curvatureS)<1e-9,"Every generated centreline join agrees through the third arc derivative");
        check(norm(a.sample.up-b.sample.up)<1e-9&&norm(a.upS-b.upS)<1e-9&&norm(a.upSS-b.upSS)<1e-9,"Every generated roll/up join agrees through second order");
        largestUpJump=std::max(largestUpJump,norm(a.upSSS-b.upSSS));largestTangentJump=std::max(largestTangentJump,norm(a.curvatureSS-b.curvatureSS));
    }
    check(largestUpJump<1e-9&&largestTangentJump<1e-9,"The complete physical rider frame is C3 at every knot, module transition and closed seam");
    std::cout<<"maximumUpThirdJump="<<largestUpJump<<" maximumTangentThirdJump="<<largestTangentJump<<'\n';
}
Design generateChecked(uint64_t seed,TerrainKind terrain=TerrainKind::Flat){
    GenerationRequest request;request.seed=seed;request.terrain.kind=terrain;request.targets.requireIntensity=false;
    auto design=generate(request);
    if(!design.accepted())for(const auto* report:{&design.report,&design.simulation.report})for(const auto& error:report->errors)std::cerr<<"seed "<<seed<<' '<<error.code<<": "<<error.message<<'\n';
    check(design.accepted(),"Entire generated circuit passes unmodified geometry, train forces, target and convergence gates");
    check(design.simulation.completed&&design.simulation.frames.back().speed==0,"The complete compact composition reaches its physical stop without requiring filler duration");
    check(design.convergence.coarseStep==1./960&&design.convergence.fineStep==1./1920,"Full ride uses required 960/1920 Hz simulation and verification");
    check(design.simulation.metrics.maxGroundHeight>=request.targets.height&&std::abs(design.simulation.metrics.maxSpeed-request.targets.speed)<=request.targets.speed*.01,"Record hill clears its height and the ride lands on the dialled speed setpoint");
    std::cout<<"seed="<<seed<<" accepted=true candidate="<<design.candidate<<" length="<<design.track.length<<" topology="<<design.topology<<'\n';
    return design;
}
bool same(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
}
struct SectionSpan { double start{},end{}; };
SectionSpan sectionSpan(const Design& d,RideRole role,const char* recipeId){
    SectionSpan result{INFINITY,-INFINITY};
    for(const auto& s:d.sections)if(s.role==role&&s.recipeId==recipeId){result.start=std::min(result.start,s.start);result.end=std::max(result.end,s.end);}
    check(result.start<result.end,"Required typed recipe element has a measured span");return result;
}
void checkRecipeLayout(const Design& d){
    double previousEnd=0;
    for(const auto& element:d.request.recipe.elements){
        const auto span=sectionSpan(d,element.role,element.id.c_str());
        check(std::abs(span.start-previousEnd)<1e-7,"Compiled elements follow their authored recipe without gaps");
        previousEnd=span.end;
    }
    check(std::abs(previousEnd-d.track.length)<1e-7,"Recipe covers the full circuit");
    for(const auto& s:d.sections)check(s.role!=RideRole::Unspecified&&!s.recipeId.empty(),"Every generated section carries its recipe role and stable ID");
}
void checkComposition(const Design& d){
    checkRecipeLayout(d);
    check(d.motion.passed&&d.spatial.passed,"Motion continuity and independent spatial refinement pass");
    check(d.simulation.metrics.maxEnergyResidual<.5&&d.simulation.metrics.peakDrivePowerWatts>0,"Propulsion work closes the finite-train energy balance");
    const auto& frames=d.simulation.frames;const auto& stop=frames.back(),&before=frames[frames.size()-2];
    const double remaining=stop.time-before.time;
    check(stop.speed==0&&std::abs(stop.acceleration)<1e-9&&stop.accelerationRate==0&&before.speed<.001,
        "Terminal positioning tyres reach actual rest with zero acceleration and jerk");
    check(std::abs(before.acceleration+3*before.speed/remaining)<.002&&std::abs(before.accelerationRate-6*before.speed/(remaining*remaining))<.15,
        "The last display sample follows the cubic speed / quadratic acceleration capture law to its sub-step stopping event");
    for(const auto& seat:d.simulation.metrics.seats)for(int axis=0;axis<3;++axis){const double nominal=axis==0?d.request.limits.maxJerkGps:axis==1?d.request.limits.maxLateralRateGps:d.request.limits.maxLongitudinalRateGps;
        check(seat.axes[axis].maxRateGps<=nominal*Limits::allowanceFactor,"Every seat-axis force rate meets the declared project envelope");}
    double previous=frames.front().distance;
    for(size_t i=1;i<frames.size();++i)for(int sample=0;sample<16;++sample){const double time=frames[i-1].time+(frames[i].time-frames[i-1].time)*sample/16;
        const auto motion=interpolateMotion(frames[i-1],frames[i],time);
        check(motion.distance+1e-8>=previous&&motion.speed>=-1e-7,"Continuous replay cannot reverse or overshoot its physical stop");previous=motion.distance;
        for(int seat=0;seat<3;++seat){const auto measured=measureSeatDynamics(d.track,motion.distance+seatDistanceOffset(d.request.train,seat),motion.speed,motion.acceleration,motion.jerk,d.request.train.seatHeight);
            const bool valid=measured.force.vertical>=d.request.limits.minVerticalG*Limits::allowanceFactor&&measured.force.vertical<=d.request.limits.maxVerticalG*Limits::allowanceFactor&&std::abs(measured.force.lateral)<=d.request.limits.maxLateralG*Limits::allowanceFactor&&std::abs(measured.force.longitudinal)<=d.request.limits.maxLongitudinalG*Limits::allowanceFactor&&std::abs(measured.rate.vertical)<=d.request.limits.maxJerkGps*Limits::allowanceFactor&&std::abs(measured.rate.lateral)<=d.request.limits.maxLateralRateGps*Limits::allowanceFactor&&std::abs(measured.rate.longitudinal)<=d.request.limits.maxLongitudinalRateGps*Limits::allowanceFactor;
            if(!valid)std::cerr<<"replay time="<<time<<" seat="<<seat<<" G="<<measured.force.vertical<<','<<measured.force.lateral<<','<<measured.force.longitudinal<<" rates="<<measured.rate.vertical<<','<<measured.rate.lateral<<','<<measured.rate.longitudinal<<'\n';
            check(valid,"Continuous rendered motion and synchronized HUD loads retain the physical force/rate limits");
        }
    }
}
void checkTrimOperatingCases(const Design& d){
    for(int mode=0;mode<2;++mode){auto operations=d.operations;auto train=d.request.train;
        if(mode==0)operations.erase(std::remove_if(operations.begin(),operations.end(),[](const Operation& op){return op.kind==DriveKind::Trim;}),operations.end());
        if(mode==1)for(auto& op:operations)if(op.kind==DriveKind::Trim)op.targetSpeed=0;
        const auto run=simulate(d.track,operations,train),fine=simulate(d.track,operations,train,1./1920);ConvergenceAssessment assessment;
        const auto convergence=compareSimulationConvergence(run,fine,d.request.limits,assessment),limits=validateSimulationTargets(run,d.request.targets,d.request.limits);
        std::cout<<"trimMode="<<mode<<" completed="<<run.completed<<" Gz="<<run.metrics.minVerticalG<<":"<<run.metrics.maxVerticalG<<" Gy="<<run.metrics.maxLateralG<<'\n';
        for(const auto& error:limits.errors)std::cerr<<"trimMode="<<mode<<' '<<error.code<<" at "<<error.distance<<" actual="<<error.actual<<" limit="<<error.limit<<'\n';
        if(!limits.valid()){
            for(int seat=0;seat<3;++seat){const Frame* low=&run.frames.front(),*high=low;
                for(const auto& f:run.frames){if(f.seats[seat].vertical<low->seats[seat].vertical)low=&f;if(f.seats[seat].vertical>high->seats[seat].vertical)high=&f;}
                std::cerr<<"seat="<<seat<<" minG="<<low->seats[seat].vertical<<" at "<<low->distance<<" maxG="<<high->seats[seat].vertical<<" at "<<high->distance<<'\n';}
            for(const auto& trim:run.trims)std::cerr<<"trim="<<trim.operation<<" speed="<<trim.sensedSpeed<<" deployment="<<trim.deployment<<'\n';
        }
        check(run.completed&&limits.valid()&&validateSimulationTargets(fine,d.request.targets,d.request.limits).valid()&&convergence.valid(),"Trims off and fully deployed pass the configured-drag envelope at both time resolutions");
    }
}
void checkAcceptedRevisionPersistence(const Design& d){
    auto recipeEdited=d;
    auto hill=std::find_if(recipeEdited.request.recipe.elements.begin(),recipeEdited.request.recipe.elements.end(),
        [](const RecipeElement& element){return std::holds_alternative<HillParameters>(element.parameters);});
    check(hill!=recipeEdited.request.recipe.elements.end(),"Persistence fixture contains an editable hill");
    std::get<HillParameters>(hill->parameters).riseMeters+=1;
    check(!recipeEdited.accepted(),"Editing a recipe parameter invalidates the accepted in-memory revision");
    auto targetEdited=d;targetEdited.request.targets.height+=1;
    check(!targetEdited.accepted(),"Editing a generation target invalidates the accepted in-memory revision");

    const auto folder=std::filesystem::temp_directory_path()/(
        "vibecoaster-organic-persistence-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    check(std::filesystem::create_directory(folder),"Unique organic persistence directory created");
    const auto path=folder/"baseline.coaster";std::string error;int initialCallbacks=0;
    check(saveDesign(d,path.string(),error,{},[&](const WorkProgress&){++initialCallbacks;}),"Accepted organic baseline saves");
    check(initialCallbacks>0,"Initial accepted save reports its work phases");
    int unchangedCallbacks=0;bool unchangedForce=false;
    check(saveDesign(d,path.string(),error,{},[&](const WorkProgress& progress){++unchangedCallbacks;unchangedForce|=progress.phase==WorkPhase::Forces;}),"Unchanged accepted revision saves atomically");
    check(unchangedCallbacks>0&&!unchangedForce,"Unchanged accepted revision skips force-phase callbacks");
    auto spanEdited=d;spanEdited.sections.front().end+=1;int editedCallbacks=0;bool editedForce=false;
    check(!saveDesign(spanEdited,path.string(),error,{},[&](const WorkProgress& progress){++editedCallbacks;editedForce|=progress.phase==WorkPhase::Forces;}),"Edited canonical section span is refused without revalidation");
    check(error.find("REVALIDATION_REQUIRED")!=std::string::npos&&editedCallbacks>0&&!editedForce,"Edited canonical span reports the revalidation requirement before force work");
    Design reloaded;check(loadDesign(path.string(),reloaded,error)&&reloaded.accepted(),"Saved baseline reloads and passes independent validation");
    std::error_code ignored;std::filesystem::remove(path,ignored);std::filesystem::remove(folder,ignored);
}
int main(int argc,char** argv){try{
    check(Targets{}.speed==300/3.6,"Default baseline is 300 km/h");
    const bool baselineOnly=argc==3&&std::string(argv[1])=="--baseline-only";
    if(argc!=1&&!baselineOnly)throw std::runtime_error("Usage: organic_generation_tests [--baseline-only ACCEPTED_FILE]");
    Design first;if(baselineOnly){std::string error;check(loadDesign(argv[2],first,error),("Baseline reload: "+error).c_str());}else first=generateChecked(42);
    checkInversions(first);checkPropulsionCorridors(first);checkTerminalBrake(first);checkComposition(first);checkC3Transitions(first);checkTrimOperatingCases(first);
    check(first.simulation.metrics.minVerticalG<0,"The complete ride includes actual measured airtime");
    if(baselineOnly){checkAcceptedRevisionPersistence(first);std::cout<<"PASS "<<checks<<" checkpoint baseline, operating scenarios and persistence checks\n";return 0;}
    const auto repeated=generateChecked(42);bool identical=first.track.knots.size()==repeated.track.knots.size();
    if(identical)for(size_t i=0;i<first.track.knots.size();++i){const auto& a=first.track.knots[i];const auto& b=repeated.track.knots[i];identical&=same(a.position,b.position)&&same(a.tangent,b.tangent)&&same(a.curvature,b.curvature)&&same(a.up,b.up)&&a.bank==b.bank&&a.element==b.element;}
    first.timings=repeated.timings; // Wall-clock measurements are intentionally nondeterministic.
    check(identical&&reportJson(first)==reportJson(repeated),"Seed reproduces every canonical knot and independently measured telemetry");
    GenerationRequest request;auto cancelled=generate(request,[]{return true;});check(cancelled.simulation.cancelled&&!cancelled.accepted(),"Cancelled generation cannot be accepted");
    auto tampered=first;tampered.sections.front().end+=1;tampered.report={};assessMotion(tampered);check(!tampered.accepted(),"Invalid motion intent cannot bypass acceptance");
    auto relabeled=first;relabeled.sections.front().role=RideRole::Loop;relabeled.report={};assessMotion(relabeled);
    check(std::any_of(relabeled.report.errors.begin(),relabeled.report.errors.end(),[](const Finding& f){return f.code=="MOTION_RECIPE_MAP";}),"Rechecks reject a compiled role that disagrees with its saved recipe");
    checkAcceptedRevisionPersistence(first);
    std::cout<<"PASS "<<checks<<" complete-route physics, C3 motion, energy, typed composition, clearance and determinism checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL after "<<checks<<": "<<e.what()<<'\n';return 1;}}
