#include "coaster/coaster.hpp"
#include "coaster/clearance.hpp"
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
static double motorAlignmentMargin(const TrainConfig& train){return (train.cars-1)*train.spacing+1.5;}
static bool propulsionGeometry(const Track& track,double s){
    const auto k=sampleKinematics(track,s);const auto& p=k.sample;
    const Vec3 upright=unit(Vec3{0,0,1}-p.tangent*p.tangent.z);
    return std::hypot(p.tangent.x,p.tangent.y)>1e-3&&std::abs(cross(p.tangent,p.curvature).z)<1e-5&&dot(p.up,upright)>.9998&&std::abs(dot(k.upS,p.right))<.001;
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
    for(int i=0;i<req.maxCandidates;++i){
        if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}
        work.enter(WorkPhase::Authoring,i,"Designing continuous route and shared transitions");
        try{
            RecipeFeedback feedback;std::vector<RecipePort> ports;
            Design d=compileRecipe(req,i,feedback,ports,cancel);
            work.enter(WorkPhase::Motion,i,"Simulating initial geometry");
            auto motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel);
            if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            for(int refinement=0;!motion.frames.empty()&&refinement<4;++refinement){
                double maximumCorrection=0,boostPeak=0;
                for(const auto& section:d.sections)if(section.role==RideRole::DownhillLaunch)
                    for(const auto& frame:motion.frames)if(frame.distance>=section.start&&frame.distance<=section.end)boostPeak=std::max(boostPeak,frame.speed);
                if(boostPeak>0){const double change=req.targets.speed+.15-boostPeak;maximumCorrection=std::abs(change);if(std::abs(change)>=.01)feedback.peakSpeedCorrection+=change;}
                for(const auto& port:ports)if(port.distance<=motion.frames.back().distance){
                    const double actual=replayValueAt(motion.frames,port.distance);
                    // Do not re-author unchanged upstream elements for
                    // sub-centimetre-per-second replay roundoff. Full final
                    // train/force acceptance remains independent of this fit.
                    if(std::abs(actual-port.speed)>=.01)feedback.energyCorrection[port.id]+=actual*actual-port.speed*port.speed;
                    maximumCorrection=std::max(maximumCorrection,std::abs(actual-port.speed));
                }
                if(maximumCorrection<.20)break;
                work.enter(WorkPhase::Authoring,i,"Correcting reached source ports from finite-train energy");
                d=compileRecipe(req,i,feedback,ports,cancel);
                work.enter(WorkPhase::Motion,i,"Simulating energy-corrected geometry");
                motion=simulateMotion(d.track,d.operations,req.train,req.simulationStep,cancel);
                if(motion.cancelled){d.simulation.cancelled=true;d.report.fail("CANCELLED","Generation cancelled");return d;}
            }
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
            const double alignmentMargin=motorAlignmentMargin(req.train);
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
            bool intensityOnly=d.simulation.completed&&d.simulation.report.valid()&&d.report.errors.size()==1&&d.report.errors.front().code=="INTENSITY_TARGET";
            if(intensityOnly&&(!haveBestIntensity||d.simulation.metrics.exposure10Seconds>bestIntensity.simulation.metrics.exposure10Seconds)){bestIntensity=d;haveBestIntensity=true;}last=std::move(d);
            // Missing reference data cannot be repaired by searching other seeds.
            bool onlyMissing=last.simulation.completed&&last.simulation.report.valid()&&last.report.errors.size()==1&&last.report.errors.front().code=="REFERENCE_UNAVAILABLE";
            if(onlyMissing)return withHistory(std::move(last),"reference-unavailable");
        }catch(RecipeCompileFailure& e){last=std::move(e.partial);if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}last.report.fail("AUTHORING",e.what());remember(&last,i);}
        catch(const std::exception& e){if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}remember(nullptr,i);last.report.fail("CANDIDATE_FAILURE",e.what());if(progress)progress(i,std::string("Candidate rejected: ")+e.what());}
    }if(haveBestIntensity)return withHistory(std::move(bestIntensity),"best-physically-valid-intensity-shortfall");return withHistory(std::move(last),"last-constructed-rejection");
}
}
