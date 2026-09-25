#include "coaster/coaster.hpp"
#include "coaster/clearance.hpp"
#include "coaster/operation_hardware.hpp"
#include "recipe_compiler.hpp"
#include "authoring.hpp"
#include "banking.hpp"
#include "trim_layout.hpp"
#include "simulation_internal.hpp"
#include "progress_internal.hpp"
#include "acceptance_internal.hpp"
#include <future>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace coaster {
// Recompose upstream bearings toward a short physical station approach.
// This approximation only chooses the next recipe trial; the real FVD source,
// complete train replay and every acceptance check still run independently.
static bool improveReturnLayout(const Design& d,RecipeFeedback& feedback,const std::optional<RecipePort>& pending,Cancel cancel){
    std::string cliffId,approachId;
    for(const auto& e:d.request.recipe.elements){
        if(e.role==RideRole::CliffApproach)cliffId=e.id;
        if(e.role==RideRole::Return&&e.anchor==TerrainAnchor::Approach&&std::holds_alternative<SweepParameters>(e.parameters))approachId=e.id;
    }
    if(cliffId.empty()||approachId.empty())return false;
    auto findSource=[&](const std::string& id){return std::find_if(d.forcePrograms.begin(),d.forcePrograms.end(),[&](const auto& p){return p.name==id;});};
    const auto approach=findSource(approachId);
    const auto winding=findSource(cliffId+"-clifftop"),setup=findSource(cliffId+"-recovery");
    if(setup==d.forcePrograms.end()||winding==d.forcePrograms.end())return false;
    double distance=0,speed=0;
    if(pending&&pending->id==approachId){distance=pending->distance;speed=pending->speed;}
    else if(approach!=d.forcePrograms.end()){distance=d.track.spans.at(approach->firstKnot).start;speed=approach->program.speed;}
    else return false;
    const auto q=sampleKinematics(d.track,distance);
    if(std::abs(q.sample.tangent.z)>1e-6||norm(q.sample.up-Vec3{0,0,1})>1e-6||norm(q.sample.curvature)>1e-6)return false;
    const double hand=winding->hand;
    auto unmirror=[&](Vec3 p){p.y*=hand;return p;};
    // The winding family's departure bearing is changed in its last turn.
    // Use that turn's centre as the local planning pivot; the signature's
    // force-authored heading is retained because it has limited authority.
    const size_t departurePivot=winding->firstKnot+size_t(.82*(winding->sourceDistances.size()-1));
    const Vec3 point=unmirror(q.sample.position),cp=unmirror(d.track.knots[setup->firstKnot].position),sp=unmirror(d.track.knots[departurePivot].position);
    const auto tangent=unmirror(q.sample.tangent);const double heading=std::atan2(tangent.y,tangent.x);
    if(std::abs(heading)<.35||std::abs(heading)>2.5||speed<15||speed>80)return false;
    const auto& train=d.request.train;
    const double rolling=gravity*train.rollingResistance,drag=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
    using State=std::array<double,4>; // x, y, heading, speed
    auto shortArc=[&](double yaw){
        const double bank=std::copysign(std::acos(1/recipeApproachNormalG),yaw),sign=std::copysign(1.,yaw);
        auto integrate=[&](double hold){
            State state{0,0,yaw,speed};
            const std::array<double,5> durations{.4,recipeApproachBankRampSeconds,hold,recipeApproachBankRampSeconds,.4};
            auto add=[](State a,State b,double step){for(int i=0;i<4;++i)a[i]+=b[i]*step;return a;};
            for(int phase=0;phase<5;++phase){const double duration=durations[phase];const int count=std::max(1,int(std::ceil(duration/.04)));const double dt=duration/count;
                auto rate=[&](const State& x,double u){const double smooth=u*u*u*u*(35+u*(-84+u*(70-20*u)));
                    const double phi=bank*(phase==1?smooth:phase==3?1-smooth:phase==2?1:0);
                    return State{x[3]*std::cos(x[2]),x[3]*std::sin(x[2]),-gravity*std::tan(phi)/x[3],-rolling-drag*x[3]*x[3]};};
                for(int i=0;i<count;++i){const auto a=rate(state,double(i)/count),b=rate(add(state,a,dt*.5),(i+.5)/count),c=rate(add(state,b,dt*.5),(i+.5)/count),e=rate(add(state,c,dt),(i+1.)/count);
                    for(int j=0;j<4;++j)state[j]+=dt*(a[j]+2*b[j]+2*c[j]+e[j])/6;}
            }
            return state;
        };
        double lo=.001,hi=8;
        if(sign*integrate(lo)[2]<0||sign*integrate(hi)[2]>0)return Vec3{NAN,NAN,0};
        for(int i=0;i<24;++i){if(cancel&&cancel())return Vec3{NAN,NAN,0};const double mid=(lo+hi)*.5;if(sign*integrate(mid)[2]>0)lo=mid;else hi=mid;}
        const auto end=integrate((lo+hi)*.5);return Vec3{end[0],end[1],0};
    };
    const auto arc=shortArc(heading),derivative=(shortArc(heading+.001)-shortArc(heading-.001))/.002;
    if(!finite(arc)||!finite(derivative))return false;
    const auto& brakes=std::get<OperationParameters>(d.request.recipe.elements.back().parameters);
    const Vec3 target{-brakes.lengthMeters-(train.cars-1)*train.spacing-3,0,point.z};
    const Vec3 error=point+arc-target,jc=Vec3{-(point-cp).y,(point-cp).x,0}+derivative,js=Vec3{-(point-sp).y,(point-sp).x,0}+derivative;
    const double determinant=jc.x*js.y-js.x*jc.y;
    if(std::abs(determinant)<1)return false;
    const double dc=std::clamp((-error.x*js.y+error.y*js.x)/determinant,-.1,.1),ds=std::clamp((-error.y*jc.x+error.x*jc.y)/determinant,-.1,.1);
    const double prior=feedback.compactReturn?feedback.cliffHeadingCorrection:d.candidate==1?-4*pi/180:d.candidate==2?4*pi/180:0;
    const double nextCliff=std::clamp(prior+dc,std::max(-.45,-1.5-recipeClimbSetupHeading),std::min(.45,.5-recipeClimbSetupHeading)),nextDeparture=std::clamp(feedback.cliffDepartureHeadingCorrection+ds,-.45,.45);
    if(feedback.compactReturn&&std::abs(nextCliff-feedback.cliffHeadingCorrection)+std::abs(nextDeparture-feedback.cliffDepartureHeadingCorrection)<1e-7)return false;
    feedback.cliffHeadingCorrection=nextCliff;feedback.cliffDepartureHeadingCorrection=nextDeparture;feedback.compactReturn=true;
    return true;
}


// Coarse, bounded source changes respond to rejected seat loads. They retain
// the protected hill body and are followed by a fresh complete-circuit replay.
static bool improveForceIntent(const Design& d,RecipeFeedback& feedback){
    bool changed=false;
    for(const auto& section:d.sections){
        if(section.role==RideRole::Camelback){
            const bool longRecovery=std::any_of(d.report.errors.begin(),d.report.errors.end(),[&](const Finding& f){
                return f.code=="F2291_25_negative-history-duration"&&f.distance>=section.start&&f.distance<=section.end;
            });
            if(!longRecovery)continue;
            const auto element=std::find_if(d.request.recipe.elements.begin(),d.request.recipe.elements.end(),[&](const auto& e){return e.id==section.recipeId;});
            if(element==d.request.recipe.elements.end())continue;
            const double cut=std::get<CamelbackParameters>(element->parameters).tailCutSeconds;
            const double next=std::min({.2,.99-cut,feedback.camelbackTailCutSeconds+.1});
            if(next>feedback.camelbackTailCutSeconds){feedback.camelbackTailCutSeconds=next;changed=true;}
        }

    }
    return changed;
}

double movingRideSeconds(const Design& d){
    for(const auto& op:d.operations)if(op.kind==DriveKind::Station)return replayValueAt(d.simulation.frames,op.start,true);
    throw std::runtime_error("Moving duration requires the terminal station brake");
}

void evaluateTargets(Design& d,const ClearanceSweep* prepared){
    if(d.supports.empty())d.report.fail("SUPPORT_LAYOUT","Design contains no connected supports");
    auto& m=d.simulation.metrics;const auto& req=d.request;double low=1e9,high=-1e9,station=d.track.knots.front().position.z;
    for(double s=0;s<d.track.length;s+=1){auto p=d.track.sample(s);double gh=p.position.z-req.terrain.height(p.position.x,p.position.y);low=std::min(low,p.position.z);high=std::max(high,p.position.z);m.maxGroundHeight=std::max(m.maxGroundHeight,gh);
        if(p.element==Element::Inversion&&p.up.z<-.5)m.inversionGroundHeight=std::max(m.inversionGroundHeight,gh);
    }m.heightAboveStation=high-station;m.verticalRelief=high-low;
    m.minGroundClearance=prepared?minimumSweptGroundClearance(*prepared,req.terrain):minimumSweptGroundClearance(d.track,req.terrain,req.train);
    auto min=[&](const char* code,double value,double goal){if(!std::isfinite(value)||value+1e-6<goal)d.report.fail(code,"Measured result misses requested target",0,value,goal);};
    min("HEIGHT_TARGET",m.maxGroundHeight,req.targets.height);min("INVERSION_TARGET",m.inversionGroundHeight,req.targets.inversionHeight);
    auto dynamic=validateSimulationTargets(d.simulation,req.targets,req.limits);
    d.report.errors.insert(d.report.errors.end(),dynamic.errors.begin(),dynamic.errors.end());
    d.report.warnings.insert(d.report.warnings.end(),dynamic.warnings.begin(),dynamic.warnings.end());
    d.report.warnings.push_back("Force limits are provisional game assumptions; reference calibration is not complete.");

}
ValidationReport validateRequest(const GenerationRequest& req){
    if(!req.recipe.elements.empty()){
        std::string error;if(!validateRecipe(req.recipe,error)){ValidationReport report;report.fail("RECIPE_CONFIG",error);return report;}
    }
    ValidationReport r;if(!req.terrain.valid()){r.fail("TERRAIN_PROFILE","Terrain profile is invalid");return r;}const auto& t=req.targets;const auto& l=req.limits;
    for(double value:{t.height,t.speed,t.inversionHeight,t.launchSeconds,l.minVerticalG,l.maxVerticalG,l.maxLateralG,l.maxLongitudinalG,l.maxJerkGps,l.minClearance,req.simulationStep})if(!std::isfinite(value)){r.fail("REQUEST_RANGE","Request contains a nonfinite value");return r;}
    if(req.maxCandidates<1||req.maxCandidates>64||t.height<0||t.height>350||t.speed<1||t.speed>110||t.inversionHeight<0||t.inversionHeight>140||t.launchSeconds<.8||t.launchSeconds>10||l.maxVerticalG<=l.minVerticalG||l.maxLateralG<=0||l.maxLongitudinalG<=0||l.maxJerkGps<=0||l.minClearance<0||req.simulationStep<1./2000||req.simulationStep>1./30||!req.terrain.valid())r.fail("REQUEST_RANGE","Request is outside the prototype's supported domain");
    if(std::isinf(t.referenceExposure)||(std::isfinite(t.referenceExposure)&&t.referenceExposure<=0)||(!t.referenceId.empty()&&!std::isfinite(t.referenceExposure))||(std::isfinite(t.referenceExposure)&&t.referenceId.empty()))r.fail("REFERENCE_CONFIG","Configured reference needs a finite positive exposure and a nonempty ID");
    auto reference=validateReference(t);r.errors.insert(r.errors.end(),reference.errors.begin(),reference.errors.end());
    for(double rate:{l.maxLateralRateGps,l.maxLongitudinalRateGps})if(std::isinf(rate)||(std::isfinite(rate)&&rate<=0))r.fail("AXIS_RATE_CONFIG","Optional component rate gates must be finite positive values, or unset");
    if(!std::isfinite(req.style.airtime)||req.style.airtime<.75||req.style.airtime>1.2||!std::isfinite(req.style.signatureRollDegrees)||req.style.signatureRollDegrees<30||req.style.signatureRollDegrees>60||req.style.returnStyle< -1||req.style.returnStyle>1)r.fail("STYLE_CONFIG","Airtime strength must be 0.75..1.2, signature roll 30..60 degrees, and return style automatic/flowing/twin-airtime");
    const auto& train=req.train;
    for(double value:{train.carMass,train.spacing,train.seatHeight,train.dragCdA,train.rollingResistance,train.airDensity})if(!std::isfinite(value)){r.fail("TRAIN_CONFIG","Train contains nonfinite settings");return r;}
    if(train.cars<1||train.cars>16||train.spacing<=0||train.spacing>20||train.carMass<=0||train.seatHeight<0||train.seatHeight>3||train.dragCdA<0||train.rollingResistance<0||train.airDensity<0)r.fail("TRAIN_CONFIG","Train is outside the supported model domain");
    return r;
}
Design generate(const GenerationRequest& input,Cancel cancel,Progress callback){
    WorkRecorder work(callback);
    std::function<void(int,const std::string&)> progress;
    if(callback)progress=[&](int candidate,const std::string& message){work.message(candidate,message);};
    std::mutex cancellationMutex;
    const Cancel requestedCancel=std::move(cancel);
    if(requestedCancel)cancel=[&]{std::lock_guard lock(cancellationMutex);return requestedCancel();};
    GenerationRequest req=input;
    Design last;last.request=req;
    last.report=validateRequest(req);if(!last.report.valid())return last;
    double fastestDeparture=plannedDepartureSeconds(req.limits.maxLongitudinalG*gravity-.02,req.train,req.limits);
    if(fastestDeparture>req.targets.launchSeconds){last.report.fail("LAUNCH_FEASIBILITY","Requested departure is below the force-limited flat-station prototype bound",0,fastestDeparture,req.targets.launchSeconds);return last;}
    if(req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure)&&req.targets.referenceExposure>10*std::max(0.,req.limits.maxVerticalG)){last.report.fail("INTENSITY_FEASIBILITY","Requested reference exposure exceeds ten seconds at the selected vertical force ceiling",0,req.targets.referenceExposure,10*std::max(0.,req.limits.maxVerticalG));return last;}
    std::vector<std::string> history;Design bestIntensity;bool haveBestIntensity=false;double previousCandidateElapsed=0;
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
        double elapsed=0;for(double phase:work.snapshot().seconds)elapsed+=phase;
        std::ostringstream h;h<<std::setprecision(12)<<"{\"candidate\":"<<index<<",\"constructed\":"<<(d?"true":"false")<<",\"elapsedSeconds\":"<<elapsed<<",\"candidateSeconds\":"<<elapsed-previousCandidateElapsed;previousCandidateElapsed=elapsed;
        if(d){h<<",\"completed\":"<<(d->simulation.completed?"true":"false")<<",\"exposure10Seconds\":"<<d->simulation.metrics.exposure10Seconds<<",\"launchSeconds\":";if(std::isfinite(d->simulation.metrics.launchTo180))h<<d->simulation.metrics.launchTo180;else h<<"null";if(d->simulation.completed)h<<",\"movingDurationSeconds\":"<<movingRideSeconds(*d);h<<",\"errors\":[";bool comma=false;for(const auto* report:{&d->report,&d->simulation.report})for(const auto& f:report->errors){if(comma)h<<',';comma=true;h<<'"'<<f.code<<'"';}h<<']';}
        else h<<",\"errors\":[\"CANDIDATE_FAILURE\"]";h<<'}';history.push_back(h.str());
    };
    auto withHistory=[&](Design d,const char* selection){
        d.timings=work.snapshot();
        if(d.planningDiagnostics.empty())d.planningDiagnostics="{\"generationOnly\":true}";
        d.planningDiagnostics.pop_back();
        if(d.simulation.completed){std::ostringstream pacing;pacing<<std::setprecision(12)<<",\"movingDurationSeconds\":"<<movingRideSeconds(d)<<",\"movingDurationDefinition\":\"Powered departure to train center reaching terminal Station operation start; excludes terminal stopping\"";d.planningDiagnostics+=pacing.str();}
        d.planningDiagnostics+=",\"searchSelection\":\""+std::string(selection)+"\",\"candidateHistory\":[";
        for(size_t i=0;i<history.size();++i){if(i)d.planningDiagnostics+=',';d.planningDiagnostics+=history[i];}d.planningDiagnostics+="]}";return d;
    };
    RecipeFeedback calibratedEnergy;
    for(int i=0;i<req.maxCandidates;++i){
        if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}
        work.enter(WorkPhase::Authoring,i,"Designing continuous route and shared transitions");
        try{
            RecipeFeedback feedback=calibratedEnergy;std::vector<RecipePort> ports;
            unsigned bootstrapAttempts=0;std::vector<std::string> bootstrapHistory;
            auto attachBootstrap=[&](Design& design){
                if(bootstrapHistory.empty())return;
                if(design.planningDiagnostics.empty())design.planningDiagnostics="{\"generationOnly\":true}";
                design.planningDiagnostics.pop_back();design.planningDiagnostics+=",\"provisionalEnergyBootstrap\":[";
                for(size_t n=0;n<bootstrapHistory.size();++n){if(n)design.planningDiagnostics+=',';design.planningDiagnostics+=bootstrapHistory[n];}
                design.planningDiagnostics+="]}";
            };
            auto recordBootstrap=[&](const RecipeCompileFailure& failure,const RecipeBootstrapResult& result,const char* status){
                std::ostringstream record;record<<std::setprecision(12)<<"{\"attempt\":"<<bootstrapAttempts<<",\"status\":"<<std::quoted(status)
                    <<",\"sourceFailure\":"<<std::quoted(failure.what())<<",\"continuationMeters\":"<<result.continuationMeters
                    <<",\"maximumSpeedCorrection\":"<<result.maximumSpeedCorrection<<",\"boundarySpeedIsProvisional\":true,\"ports\":[";
                for(size_t n=0;n<result.observations.size();++n){const auto& value=result.observations[n];if(n)record<<',';
                    record<<"{\"id\":"<<std::quoted(value.port.id)<<",\"distance\":"<<value.port.distance<<",\"plannedSpeed\":"<<value.port.speed
                        <<",\"estimatedSpeed\":"<<value.estimatedSpeed<<",\"provisional\":"<<(value.provisional?"true":"false")<<'}';}
                record<<"]}";bootstrapHistory.push_back(record.str());
            };
            auto compileCalibratedPrefix=[&](){
                for(;;){
                    try{auto prefix=compileRecipe(req,i,feedback,ports,cancel,RecipeCompileMode::EnergyCalibrationPrefix);attachBootstrap(prefix);return prefix;}
                    catch(RecipeCompileFailure& failure){
                        if(failure.partial.simulation.cancelled||(cancel&&cancel())){
                            failure.partial.simulation.cancelled=true;failure.partial.report.fail("CANCELLED","Generation cancelled during source construction");attachBootstrap(failure.partial);throw;
                        }
                        if(!failure.pendingPort){attachBootstrap(failure.partial);throw;}
                        if(bootstrapAttempts>=4){
                            RecipeBootstrapResult limit;recordBootstrap(failure,limit,"retry-limit");
                            failure.partial.report.fail("ENERGY_BOOTSTRAP_LIMIT","Four provisional energy corrections did not make the failed source constructible; no partial ride was accepted");attachBootstrap(failure.partial);throw;
                        }
                        ++bootstrapAttempts;work.enter(WorkPhase::Motion,i,"Estimating reached source energy with a provisional short continuation");
                        const auto bootstrap=bootstrapRecipeEnergy(failure,feedback,cancel);
                        recordBootstrap(failure,bootstrap,bootstrap.cancelled?"cancelled":bootstrap.corrected?"corrected":"no-progress-or-replay-rejected");
                        failure.partial.report.errors.insert(failure.partial.report.errors.end(),bootstrap.report.errors.begin(),bootstrap.report.errors.end());
                        failure.partial.report.warnings.insert(failure.partial.report.warnings.end(),bootstrap.report.warnings.begin(),bootstrap.report.warnings.end());
                        if(bootstrap.cancelled)failure.partial.simulation.cancelled=true;
                        if(!bootstrap.corrected){attachBootstrap(failure.partial);throw;}
                        work.enter(WorkPhase::Authoring,i,"Retrying the real source from provisional energy; full actual-source calibration remains required");
                    }
                }
            };
            // Calibrate upstream finite-train energy before solving the station
            // closure. This open prefix is never a complete or accepted ride.
            Design d=compileCalibratedPrefix();
            work.enter(WorkPhase::Motion,i,"Simulating initial geometry");
            auto motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel,MotionReplayMode::StationEnergyPrefix);
            if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            for(int refinement=0;!motion.frames.empty()&&refinement<6;++refinement){
                double maximumCorrection=0,boostPeak=0;
                // The downhill train still gains gravitational energy after
                // the last stator. Calibrate the actual valley peak through
                // the camelback ascent, not only the hardware interval.
                double launchStart=std::numeric_limits<double>::infinity(),pulloutEnd=0;
                for(const auto& section:d.sections)if(section.role==RideRole::DownhillLaunch){
                    launchStart=std::min(launchStart,section.start);pulloutEnd=std::max(pulloutEnd,section.end);
                }
                for(const auto& landmark:d.landmarks)if(landmark.kind==LandmarkKind::CamelbackCrest)
                    pulloutEnd=std::max(pulloutEnd,landmark.distance);
                for(const auto& frame:motion.frames)if(frame.distance>=launchStart&&frame.distance<=pulloutEnd)
                    boostPeak=std::max(boostPeak,frame.speed);
                if(boostPeak>0){const double change=req.targets.speed+.15-boostPeak;maximumCorrection=std::abs(change);if(std::abs(change)>=.01)feedback.peakSpeedCorrection+=change;}
                for(const auto& port:ports)if(port.distance<=motion.frames.back().distance){
                    const double actual=replayValueAt(motion.frames,port.distance);
                    // Do not re-author unchanged upstream elements for
                    // sub-centimetre-per-second replay roundoff. Full final
                    // train/force acceptance remains independent of this fit.
                    const auto loop=std::find_if(d.request.recipe.elements.begin(),d.request.recipe.elements.end(),[&](const RecipeElement& element){return element.role==RideRole::Loop&&element.id==port.id;});
                    if(loop!=d.request.recipe.elements.end()){
                        const std::string brakeId=port.id+"-energy-brake";double brakeBegin=INFINITY,brakeEnd=-INFINITY;
                        for(const auto& section:d.sections)if(section.recipeId==brakeId){brakeBegin=std::min(brakeBegin,section.start);brakeEnd=std::max(brakeEnd,section.end);}
                        if(std::abs(actual-recipeLoopEntrySpeed)>=.01&&brakeEnd>brakeBegin)
                            feedback.brakeAccelerationCorrection[brakeId]+=(actual*actual-recipeLoopEntrySpeed*recipeLoopEntrySpeed)/std::max(20.,brakeEnd-brakeBegin);
                    }else if(std::abs(actual-port.speed)>=.01)feedback.energyCorrection[port.id]+=actual*actual-port.speed*port.speed;
                    maximumCorrection=std::max(maximumCorrection,std::abs(actual-port.speed));
                }
                if(maximumCorrection<.20)break;
                work.enter(WorkPhase::Authoring,i,"Correcting reached source ports from finite-train energy");
                d=compileCalibratedPrefix();
                work.enter(WorkPhase::Motion,i,"Simulating energy-corrected geometry");
                motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel,MotionReplayMode::StationEnergyPrefix);
                if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            }
            if(!motion.prefixReachedEnd||!motion.report.valid()){
                d.simulation.frames=std::move(motion.frames);d.simulation.cancelled=motion.cancelled;
                d.simulation.report=std::move(motion.report);
                d.report.fail("ENERGY_PREFIX_INCOMPLETE","Finite-train energy calibration did not reach the open prefix end; closed-circuit authoring and acceptance were not attempted");
                remember(&d,i);last=std::move(d);continue;
            }
            // Nearby layout candidates may reuse this energy estimate; every
            // candidate still replays its own prefix and the complete circuit.
            calibratedEnergy=feedback;
            work.enter(WorkPhase::Authoring,i,"Placing the complete circuit from calibrated upstream energy");
            d=compileRecipe(req,i,feedback,ports,cancel,RecipeCompileMode::ClosedCircuit);
            attachBootstrap(d);
            work.enter(WorkPhase::Motion,i,"Replaying the complete circuit before final frame and force validation");
            motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel,MotionReplayMode::TrackDomain);
            if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            double portError=0;std::ostringstream portEvidence;portEvidence<<std::setprecision(12)<<",\"portReplay\":[";bool portComma=false;
            for(const auto& port:ports)if(!motion.frames.empty()&&port.distance<=motion.frames.back().distance){
                const double actual=replayValueAt(motion.frames,port.distance);portError=std::max(portError,std::abs(actual-port.speed));
                if(portComma)portEvidence<<',';portComma=true;portEvidence<<"{\"id\":"<<std::quoted(port.id)<<",\"plannedSpeed\":"<<port.speed<<",\"actualSpeed\":"<<actual<<'}';
            }
            portEvidence<<"],\"maximumPortSpeedError\":"<<portError;d.planningDiagnostics.pop_back();d.planningDiagnostics+=portEvidence.str()+"}";
            if(!motion.completed||portError>.25){
                d.simulation.completed=motion.completed;d.simulation.cancelled=motion.cancelled;
                d.simulation.frames=std::move(motion.frames);
                if(portError>.25)d.report.fail("SOURCE_SPEED","Finite-train replay does not yet match the authored source ports",0,portError,.25);
                if(!d.simulation.completed)d.report.fail("MOTION_INCOMPLETE","Preliminary finite-train replay did not complete; final frame/force acceptance was not attempted");
                remember(&d,i);last=std::move(d);continue;
            }
            authorBanking(d,motion.frames);planTrimBrakes(d,motion.frames);
            const double alignmentMargin=poweredAlignmentMargin();
            for(const auto& operation:d.operations)if(operation.kind==DriveKind::Launch||operation.kind==DriveKind::Boost){
                const auto first=d.track.sample(operation.start-alignmentMargin);
                const double heading=std::atan2(first.tangent.y,first.tangent.x);
                double minimumPitch=std::asin(first.tangent.z),maximumPitch=minimumPitch;
                auto checkMotorGeometry=[&](double s){
                    const auto p=d.track.sample(s);minimumPitch=std::min(minimumPitch,std::asin(p.tangent.z));maximumPitch=std::max(maximumPitch,std::asin(p.tangent.z));
                    if(!propulsionGeometry(d.track,s)||std::abs(std::remainder(std::atan2(p.tangent.y,p.tangent.x)-heading,2*pi))>.001)
                        throw std::runtime_error("Final banked geometry entered a powered car's alignment corridor at "+std::to_string(s));
                };
                for(double s=operation.start-alignmentMargin;s<operation.end+alignmentMargin;s+=.125)checkMotorGeometry(s);
                checkMotorGeometry(operation.end+alignmentMargin);
                for(const auto& span:d.track.spans)if(span.start>=operation.start-alignmentMargin&&span.start<=operation.end+alignmentMargin)checkMotorGeometry(span.start);
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
            work.enter(WorkPhase::Structures,i,"Replaying final geometry while constructing station and supports");
            std::vector<Finding> constructionFailures;
            try{d.station=buildStation(d.track,d.request.terrain,req.train,cancel);}
            catch(const std::exception& e){constructionFailures.push_back({"STATION_LAYOUT",e.what()});}
            try{buildSupportLayout(d,cancel);}
            catch(const std::exception& e){constructionFailures.push_back({"SUPPORT_LAYOUT",e.what()});}
            d.inversionDimensions=measureInversionDimensions(d.track,d.sections,cancel);
            auto spatial=std::async(std::launch::async,[&]{return replaySpatialRefinement(d,[&]{return stopFine.load()||(cancel&&cancel());});});
            work.enter(WorkPhase::Geometry,i,"Checking measured targets and clearance");
            auto sweep=buildClearanceSweepVerified(d.track,req.train,cancel);sweep.prepareGround(d.request.terrain,cancel);
            d.report=validateGeometry(d.track,d.request.terrain,req.limits,req.train,d.supports,sweep,cancel);
            d.report.errors.insert(d.report.errors.end(),constructionFailures.begin(),constructionFailures.end());
            auto structures=validateDesignStructures(d,sweep,cancel);d.report.errors.insert(d.report.errors.end(),structures.errors.begin(),structures.errors.end());
            d.simulation=coarse.get();
            work.enter(WorkPhase::Forces,i,"Checking measured seat loads and ride targets");
            evaluateTargets(d,&sweep);
            work.enter(WorkPhase::Authorship,i,"Checking editable geometry and continuous motion");
            assessAuthorship(d,cancel);assessMotion(d,cancel);
            // Actual canonical rail extrema enforce the category ceilings. The
            // separately labeled brake/link remain visible connector context.
            struct RailBounds {double low{INFINITY},high{-INFINITY},start{};RideRole role{};};
            std::unordered_map<std::string,RailBounds> roleBounds;double lowest=INFINITY,highest=-INFINITY;size_t sectionIndex=0;
            for(size_t spanIndex=0;spanIndex<d.track.spans.size();++spanIndex){
                if((spanIndex&127)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
                const auto& span=d.track.spans[spanIndex];std::array<double,10> z;for(size_t n=0;n<z.size();++n)z[n]=span.c[n].z;
                const auto [low,high]=canonicalPolynomialBounds(z);lowest=std::min(lowest,low);highest=std::max(highest,high);
                while(sectionIndex+1<d.sections.size()&&span.start>=d.sections[sectionIndex].end-1e-8)++sectionIndex;
                const auto& section=d.sections[sectionIndex];auto [at,inserted]=roleBounds.try_emplace(section.recipeId);
                auto& bounds=at->second;if(inserted){bounds.start=section.start;bounds.role=section.role;}
                bounds.low=std::min(bounds.low,low);bounds.high=std::max(bounds.high,high);
            }
            const auto dimensions=validateReferenceDimensions(d,cancel);
            d.report.errors.insert(d.report.errors.end(),dimensions.errors.begin(),dimensions.errors.end());
            const auto reference=validateInversionReferenceForces(d,d.simulation);
            d.report.errors.insert(d.report.errors.end(),reference.errors.begin(),reference.errors.end());

            std::ostringstream measured;measured<<std::setprecision(17)<<",\"canonicalSizeScreens\":{\"railVerticalEnvelopeMeters\":"<<highest-lowest<<",\"railEnvelopeProjectLimitMeters\":292.5";
            double footingGround=INFINITY,footingBottom=INFINITY;
            for(const auto& support:d.supports)for(const auto& member:support.members)if(member.kind==SupportMemberKind::Footing){
                footingGround=std::min(footingGround,d.request.terrain.height(member.base.x,member.base.y));footingBottom=std::min(footingBottom,member.base.z);}
            if(std::isfinite(footingGround))measured<<",\"maxRailAboveLowestFootingGroundMeters\":"<<highest-footingGround<<",\"maxRailAboveLowestFootingBottomMeters\":"<<highest-footingBottom;
            measured<<",\"datumConvention\":\"Footing ground is terrain at each footing axis; footing bottom includes its actual authored embedment. Rail envelope is a separate project screen.\",\"elements\":[";
            std::vector<std::string> sizeIds;for(const auto& [id,bounds]:roleBounds)sizeIds.push_back(id);std::sort(sizeIds.begin(),sizeIds.end());bool comma=false;
            for(const auto& id:sizeIds){const auto& bounds=roleBounds.at(id);if(comma)measured<<',';comma=true;
                measured<<"{\"id\":"<<std::quoted(id)<<",\"role\":"<<std::quoted(roleName(bounds.role))<<",\"minimumRailZ\":"<<bounds.low<<",\"maximumRailZ\":"<<bounds.high<<",\"verticalExtentMeters\":"<<bounds.high-bounds.low<<'}';}
            measured<<"]},\"loopBrakeEnergy\":[";comma=false;
            auto brakeWorkAt=[&](double distance){const auto& frames=d.simulation.frames;auto right=std::lower_bound(frames.begin(),frames.end(),distance,[](const Frame& frame,double at){return frame.distance<at;});
                if(right==frames.begin())return right->brakeWorkPerMass;if(right==frames.end())return frames.back().brakeWorkPerMass;
                const auto& left=*(right-1);return std::lerp(left.brakeWorkPerMass,right->brakeWorkPerMass,(distance-left.distance)/(right->distance-left.distance));};
            for(const auto& body:d.inversionDimensions)if(body.role==RideRole::Loop&&!d.simulation.frames.empty()&&d.simulation.frames.back().distance>=body.startDistance){
                const std::string brakeId=body.recipeId+"-energy-brake";const auto retained=std::find_if(d.forcePrograms.begin(),d.forcePrograms.end(),[&](const ForceAuthoring& item){return item.name==brakeId;});
                if(retained==d.forcePrograms.end())continue;const auto authored=designFvdSection(retained->program,cancel);
                if(!authored.assessment.passed){d.report.fail("BRAKE_SOURCE_REPLAY","Energy-management source failed its independent work replay");continue;}
                double begin=INFINITY,end=-INFINITY;for(const auto& section:d.sections)if(section.recipeId==brakeId){begin=std::min(begin,section.start);end=std::max(end,section.end);}
                const auto hardware=std::find_if(d.operations.begin(),d.operations.end(),[&](const Operation& op){return op.kind==DriveKind::Brake&&op.start>=begin&&op.end<=end;});
                if(hardware==d.operations.end()){d.report.fail("BRAKE_HARDWARE","Energy-management source has no real per-car Brake operation");continue;}
                const double actual=replayValueAt(d.simulation.frames,body.startDistance),front=hardware->start-seatDistanceOffset(req.train,0),rear=hardware->end-seatDistanceOffset(req.train,2);
                const auto loopSource=std::find_if(d.forcePrograms.begin(),d.forcePrograms.end(),[&](const ForceAuthoring& item){return item.name==body.recipeId;});
                if(comma)measured<<',';comma=true;measured<<"{\"id\":"<<std::quoted(brakeId)<<",\"fixedTargetMps\":"<<recipeLoopEntrySpeed<<",\"actualLoopEntryMps\":"<<actual<<",\"loopSourceEntryMps\":"<<(loopSource==d.forcePrograms.end()?0:loopSource->program.speed)
                    <<",\"brakeSourceTerminalMps\":"<<authored.samples.back().speed<<",\"sourceRemovedWorkJkg\":"<<-authored.samples.back().drivenWorkPerMass
                    <<",\"actualBrakeWorkFrontEntryToRearExitJkg\":"<<brakeWorkAt(rear)-brakeWorkAt(front)<<",\"hardwarePeakDecelerationMps2\":"<<hardware->maxForce/req.train.carMass<<",\"hardwareRampSeconds\":"<<hardware->rampSeconds<<",\"hardwareExitFadeMeters\":"<<hardware->exitFadeMeters<<'}';
                if(std::abs(actual-recipeLoopEntrySpeed)>.20)d.report.fail("LOOP_ENTRY_ENERGY","Actual finite-train loop entry missed the fixed brake energy target; source speed fitting cannot satisfy this check",body.startDistance,actual,recipeLoopEntrySpeed);
            }
            measured<<']';if(d.planningDiagnostics.empty())d.planningDiagnostics="{}";d.planningDiagnostics.pop_back();d.planningDiagnostics+=measured.str()+"}";

            if(d.report.valid()&&d.simulation.completed&&d.simulation.report.valid()&&!d.simulation.cancelled){
                work.enter(WorkPhase::Refinement,i,"Checking independent half-step simulation");
                if(fine.valid())verifyConvergenceWith(d,[&]{return fine.get();},cancel);
                else verifyConvergence(d,cancel);
            }
            if(fine.valid()){stopFine.store(true);fine.wait();}
            if(d.report.valid()&&d.convergence.passed)verifySpatialRefinementWith(d,[&]{return spatial.get();},cancel);
            if(spatial.valid()){stopFine.store(true);spatial.wait();}
            if(d.simulation.cancelled)return d;
            if(d.checksPassed())freezeAcceptedRevision(d);
            remember(&d,i);if(d.accepted()){
                return withHistory(std::move(d),"first-physically-accepted-composition");
            }
            const bool forceChanged=improveForceIntent(d,calibratedEnergy);
            const bool layoutChanged=std::any_of(d.report.errors.begin(),d.report.errors.end(),[](const Finding& f){return f.code=="WAITING_TRACK";})&&
                improveReturnLayout(d,calibratedEnergy,{},cancel);
            if(cancel&&cancel()){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled during return planning");return d;}
            bool intensityOnly=d.simulation.completed&&d.simulation.report.valid()&&d.report.errors.size()==1&&d.report.errors.front().code=="INTENSITY_TARGET";
            if(intensityOnly&&(!haveBestIntensity||d.simulation.metrics.exposure10Seconds>bestIntensity.simulation.metrics.exposure10Seconds)){bestIntensity=d;haveBestIntensity=true;}last=std::move(d);
            // Missing reference data cannot be repaired by searching other seeds.
            bool onlyMissing=last.simulation.completed&&last.simulation.report.valid()&&last.report.errors.size()==1&&last.report.errors.front().code=="REFERENCE_UNAVAILABLE";
            if(onlyMissing)return withHistory(std::move(last),"reference-unavailable");
            // Preserve the usual best-intensity selection when stopping a
            // search whose next attempt would compile the same geometry.
            if(feedback.compactReturn&&!layoutChanged&&!forceChanged)break;
        }catch(RecipeCompileFailure& e){last=std::move(e.partial);if(last.simulation.cancelled||(cancel&&cancel())){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}last.report.fail("AUTHORING",e.what());const bool layoutChanged=improveReturnLayout(last,calibratedEnergy,e.pendingPort,cancel);if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled during return planning");return last;}remember(&last,i);
            // Later candidates change placement, not the failed source's force
            // family. Repeating a converged energy bootstrap cannot repair it.
            const bool exhaustedEnergy=std::any_of(last.report.errors.begin(),last.report.errors.end(),[](const Finding& f){return f.code=="ENERGY_BOOTSTRAP_NO_PROGRESS";});
            if(exhaustedEnergy&&!layoutChanged)return withHistory(std::move(last),"unconstructible-authored-force-intent");
        }
        catch(const std::exception& e){if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}remember(nullptr,i);last.report.fail("CANDIDATE_FAILURE",e.what());if(progress)progress(i,std::string("Candidate rejected: ")+e.what());}
    }if(haveBestIntensity)return withHistory(std::move(bestIntensity),"best-physically-valid-intensity-shortfall");return withHistory(std::move(last),"last-constructed-rejection");
}
}
