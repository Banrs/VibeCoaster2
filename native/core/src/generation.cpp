#include "coaster/coaster.hpp"
#include "generation_internal.hpp"
#include "itinerary_generation.hpp"
#include "bank_target.hpp"
#include <sstream>
#include <iomanip>

namespace coaster {
constexpr double departureRampSeconds=.16;
static double departureTarget(double acceleration){return std::max(60.,50+acceleration/4);}
// Flat departure sizing is a planning estimate only. The canonical finite-train
// simulator independently measures the first actual crossing of 50 m/s.
static double plannedLaunchTime(double motorAcceleration,const TrainConfig& train){
    constexpr double dt=.0005;double speed=0,time=0,drag=.5*train.airDensity*train.dragCdA/(train.cars*train.carMass);
    Operation motor{0,1,DriveKind::Launch,departureTarget(motorAcceleration),train.carMass*motorAcceleration,train.carMass*motorAcceleration*100,departureRampSeconds};
    auto acceleration=[&](double v,double t){double external=detail::appliedDriveForce(motor,train.carMass,v,t,0,motor.exitFadeMeters,0)/train.carMass-drag*v*v,friction=gravity*train.rollingResistance;return v>0?external-friction:std::max(0.,external-friction);};
    while(time<10){double a=acceleration(speed,time),mid=std::max(0.,speed+a*dt*.5),next=std::max(0.,speed+acceleration(mid,time+dt*.5)*dt);if(next>=50)return time+dt*(50-speed)/(next-speed);speed=next;time+=dt;}return std::numeric_limits<double>::infinity();
}
static double sizedLaunchAcceleration(const GenerationRequest& req,double seedPreference){
    double upper=req.limits.maxLongitudinalG*gravity-.02,lower=0;
    for(int i=0;i<32;++i){double mid=(lower+upper)*.5;if(plannedLaunchTime(mid,req.train)>req.targets.launchSeconds-.003)lower=mid;else upper=mid;}
    return std::min(req.limits.maxLongitudinalG*gravity-.02,std::max(seedPreference,upper));
}
static double stationBankFactor(double remaining,const TrainConfig& train){double upright=std::max(40.,(train.cars-1)*train.spacing*.5+18);return smooth((remaining-upright)/100);}
static double layoutSmooth(double u){u=std::clamp(u,0.,1.);return u*u*u*u*(35+u*(-84+u*(70-20*u)));}
static double replayValueAt(const std::vector<Frame>& frames,double distance,bool time=false){
    if(frames.empty()||distance<frames.front().distance||distance>frames.back().distance)
        throw std::runtime_error("Authoring feedback at "+std::to_string(distance)+" m is outside actual replay "+
            (frames.empty()?std::string("(empty)"):std::to_string(frames.front().distance)+".."+std::to_string(frames.back().distance)+" m"));
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
static void improveBanking(Design& d){
    const auto& frames=d.simulation.frames;if(frames.empty())return;
    const size_t count=d.track.spans.size();std::vector<double> authored(count),target(count),along(count),left(count),right(count),speed(count);
    for(size_t i=0;i<count;++i){const auto& k=d.track.knots[i];authored[i]=target[i]=k.bank;along[i]=d.track.spans[i].start;
        if(k.element!=Element::Turn)continue;speed[i]=replayValueAt(frames,along[i]);
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
    d.report.warnings.push_back("Higher historical restraint-dependent force profile selected; train/restraint provisions and reference calibration remain unverified.");
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
Design detail::generateRide(const GenerationRequest& input,bool requireCrossing,Cancel cancel,std::function<void(int,const std::string&)> progress){
    GenerationRequest req=input;if(req.terrain.isDefaultProfile())req.terrain=Terrain::seeded(req.terrain.kind,req.seed);
    Design last;last.request=req;
    last.report=validateRequest(req);if(!last.report.valid())return last;
    double fastestDeparture=plannedLaunchTime(req.limits.maxLongitudinalG*gravity-.02,req.train);
    if(fastestDeparture>req.targets.launchSeconds){last.report.fail("LAUNCH_FEASIBILITY","Requested departure is below the force-limited flat-station prototype bound",0,fastestDeparture,req.targets.launchSeconds);return last;}
    if(req.targets.requireIntensity&&std::isfinite(req.targets.referenceExposure)&&req.targets.referenceExposure*1.1>10*std::max(0.,req.limits.maxVerticalG)){last.report.fail("INTENSITY_FEASIBILITY","Requested exposure exceeds ten seconds at the selected vertical force ceiling",0,req.targets.referenceExposure*1.1,10*std::max(0.,req.limits.maxVerticalG));return last;}
    std::vector<std::string> history;
    auto remember=[&](const Design* d,int index,const char* failure="CANDIDATE_FAILURE"){
        std::ostringstream h;h<<std::setprecision(12)<<"{\"candidate\":"<<index<<",\"constructed\":"<<(d?"true":"false");
        if(d){h<<",\"completed\":"<<(d->simulation.completed?"true":"false")<<",\"exposure10Seconds\":"<<d->simulation.metrics.exposure10Seconds<<",\"launchSeconds\":";if(std::isfinite(d->simulation.metrics.launchTo180))h<<d->simulation.metrics.launchTo180;else h<<"null";if(d->simulation.completed)h<<",\"movingDurationSeconds\":"<<movingRideSeconds(*d);h<<",\"errors\":[";bool comma=false;for(const auto* report:{&d->report,&d->simulation.report})for(const auto& f:report->errors){if(comma)h<<',';comma=true;h<<'"'<<f.code<<'"';}h<<']';}
        else h<<",\"errors\":[\""<<failure<<"\"]";h<<'}';history.push_back(h.str());
    };
    auto withHistory=[&](Design d,const char* selection){
        if(d.planningDiagnostics.empty())d.planningDiagnostics="{\"generationOnly\":true}";
        d.planningDiagnostics.pop_back();
        d.planningDiagnostics+=std::string(",\"requestedCrossover\":")+(requireCrossing?"true":"false");
        if(d.simulation.completed){std::ostringstream pacing;pacing<<std::setprecision(12)<<",\"movingDurationSeconds\":"<<movingRideSeconds(d)<<",\"movingDurationGoalMaximumSeconds\":180,\"movingDurationDefinition\":\"Powered departure to train center reaching terminal Station operation start; excludes terminal stopping\"";d.planningDiagnostics+=pacing.str();}
        d.planningDiagnostics+=",\"searchSelection\":\""+std::string(selection)+"\",\"candidateHistory\":[";
        for(size_t i=0;i<history.size();++i){if(i)d.planningDiagnostics+=',';d.planningDiagnostics+=history[i];}d.planningDiagnostics+="]}";return d;
    };
    // Routing solves one bounded set of physical orders. Repeating the same
    // seeded route cannot repair a rejected physical constraint.
    {constexpr int i=0;
        if(cancel&&cancel()){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}
        if(progress)progress(i,"Solving physical itinerary and circuit");
        Design d;
        try{
            detail::RideFeedback feedback;
            const double acceleration=sizedLaunchAcceleration(req,38.9);
            const auto departure=detail::rideMotor(req,DriveKind::Launch,departureTarget(acceleration),acceleration,departureRampSeconds);
            auto route=detail::routeRide(req,detail::buildRideSources(req,feedback,departure,cancel),feedback,cancel,requireCrossing);
            detail::RideBuild build;double energyResidual=INFINITY;bool feedbackConverged=false;int energyIterations=0;
            std::ostringstream energyHistory;energyHistory<<std::setprecision(12);
            // One budget covers source geometry, actual turn speed and every
            // motor's own inlet. A rebuild never resets the itinerary's budget.
            for(int iteration=0;iteration<8;++iteration){
                try{build=detail::buildRide(req,i,departure,route,feedback,cancel);}
                catch(const detail::TerrainTransferInfeasible& error){
                    if(d.track.spans.empty())throw;
                    d.report.fail("CANDIDATE_FAILURE",error.what());break;
                }
                d=std::move(build.design);d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);
                ++energyIterations;
                if(d.simulation.cancelled){d.report.fail("CANCELLED","Generation cancelled");return d;}
                const auto& frames=d.simulation.frames;
                if(frames.empty())break;
                const auto distance=detail::circuitDistances(d.track);const double half=(req.train.cars-1)*req.train.spacing*.5;
                energyResidual=0;auto next=feedback;bool motorNeedsSizing=false;
                if(iteration)energyHistory<<',';
                energyHistory<<"{\"iteration\":"<<iteration+1<<",\"site\":"<<feedback.site<<",\"sources\":[";
                for(size_t occurrence=0;occurrence<build.route.order.size();++occurrence){
                    const auto id=build.route.order[occurrence];const auto& source=build.route.sources[id];
                    const auto interval=build.geometry.sources[occurrence];const double begin=distance[interval.first],end=distance[interval.last];
                    const auto reached=[&](double at){return at>=frames.front().distance&&at<=frames.back().distance;};
                    if(occurrence)energyHistory<<',';
                    energyHistory<<"{\"name\":\""<<detail::rideName(source.role)<<"\",\"begin\":"<<begin<<",\"end\":"<<end;
                    if(reached(begin)&&reached(end)){
                        const double entry=replayValueAt(frames,begin),exit=replayValueAt(frames,end);
                        if(!source.airtime())next.sourceEntry[id]=entry;
                        energyHistory<<",\"sourceEntryMps\":"<<source.entrySpeed<<",\"actualEntryMps\":"<<entry<<",\"sourceExitMps\":"<<source.exitSpeed<<",\"actualExitMps\":"<<exit;
                        if(source.forceSource()){
                            energyResidual=std::max(energyResidual,std::abs(entry-source.entrySpeed));
                            if(source.airtime())next.passiveEnergyCorrection[id]+=source.entrySpeed*source.entrySpeed-entry*entry;
                            for(const auto& phase:source.phases)
                                energyResidual=std::max(energyResidual,std::abs(replayValueAt(frames,begin+phase.distance)-phase.speed));
                        }else if(id!=0)energyResidual=std::max(energyResidual,std::abs(entry-feedback.sourceEntry[id]));
                    }else if(source.forceSource())energyResidual=INFINITY;
                    const auto& link=build.geometry.links[occurrence];const double turnBegin=distance[link.turn.first],turnEnd=distance[detail::ridePassiveEnd(build.route,build.geometry,occurrence)];
                    if(reached(turnBegin)&&reached(turnEnd)){
                        next.linkRise[id]=d.track.sample(turnEnd).position.z-d.track.sample(turnBegin).position.z;
                        double speed=0,turnSpeed=0,minimumSpeed=INFINITY;
                        for(const auto& frame:frames)if(frame.distance>=turnBegin-half&&frame.distance<=turnEnd+half){
                            speed=std::max(speed,frame.speed);minimumSpeed=std::min(minimumSpeed,frame.speed);
                            if(frame.distance<=distance[link.turn.last]+half)turnSpeed=std::max(turnSpeed,frame.speed);
                        }
                        energyResidual=std::max(energyResidual,std::abs(turnSpeed-feedback.turnSpeed[id]));next.turnSpeed[id]=turnSpeed;
                        energyResidual=std::max(energyResidual,std::abs(speed-feedback.linkSpeed[id]));next.linkSpeed[id]=speed;
                        energyHistory<<",\"turnSpeedMps\":"<<turnSpeed<<",\"turnSpeedResidualMps\":"<<turnSpeed-feedback.turnSpeed[id]
                            <<",\"linkSpeedMps\":"<<speed<<",\"linkSpeedResidualMps\":"<<speed-feedback.linkSpeed[id];
                        next.linkMinimumSpeed[id]=minimumSpeed;
                        next.linkDuration[id]=replayValueAt(frames,distance[link.turn.last],true)-replayValueAt(frames,turnBegin,true);
                    }
                    const double inlet=build.motorInlet[id];
                    if(inlet>=0&&reached(inlet)){
                        const double speed=replayValueAt(frames,inlet);
                        energyResidual=std::max(energyResidual,std::abs(speed-feedback.motorEntry[id]));next.motorEntry[id]=speed;
                        motorNeedsSizing|=std::abs(speed-feedback.motorEntry[id])>.5;
                        energyHistory<<",\"motorInletMps\":"<<speed<<",\"motorInletResidualMps\":"<<speed-feedback.motorEntry[id];
                    }
                    energyHistory<<'}';
                }
                energyHistory<<"]}";
                // The terrain assessment sizes the final footprint from its
                // actual connecting rises and train exposure. Subsequent
                // feedback resolves heights and the shared actual work boundary
                // within that footprint, rather than moving crossings on every pass.
                if(iteration==0){
                    next.measured=true;route=detail::routeRide(req,route.sources,next,cancel,requireCrossing);
                    feedback=std::move(next);continue;
                }
                if(d.simulation.completed&&d.simulation.report.valid()&&energyResidual<=.5){feedbackConverged=true;break;}
                // These inputs jointly change geometry and therefore each
                // other's next observation. Apply one damped fixed-point step,
                // starting from the actual planning inputs rather than zero.
                for(auto [before,after]:{std::pair{&feedback.sourceEntry,&next.sourceEntry},
                    {&feedback.turnSpeed,&next.turnSpeed},{&feedback.linkSpeed,&next.linkSpeed},
                    {&feedback.linkMinimumSpeed,&next.linkMinimumSpeed},{&feedback.linkDuration,&next.linkDuration},
                    {&feedback.linkRise,&next.linkRise},{&feedback.motorEntry,&next.motorEntry},
                    {&feedback.passiveEnergyCorrection,&next.passiveEnergyCorrection}})
                    for(size_t j=0;j<before->size();++j)(*after)[j]=((*before)[j]+(*after)[j])*.5;
                next.measured=true;feedback=std::move(next);
                // A partial trace can already measure the inlet that explains
                // a downstream stall. Use that physical sizing correction in
                // this same budget; an unchanged incomplete ride cannot retry.
                if(!d.simulation.completed&&!motorNeedsSizing)break;
            }
            if(d.simulation.completed){improveBanking(d);d.simulation=simulate(d.track,d.operations,req.train,req.simulationStep,cancel);}
            std::ostringstream diagnostic,airtime;bool firstAirtime=true;airtime<<std::setprecision(12);
            diagnostic<<std::setprecision(12)<<"{\"generationOnly\":true,\"architecture\":\"source-itinerary\",\"site\":"<<feedback.site<<",\"itinerary\":[";
            const auto distance=detail::circuitDistances(d.track);
            for(size_t occurrence=0;occurrence<build.route.order.size();++occurrence){
                if(occurrence)diagnostic<<',';const auto id=build.route.order[occurrence];const auto& interval=build.geometry.sources[occurrence];
                const auto& source=build.route.sources[id];double height=0;for(const auto& point:source.geometry.points)height=std::max(height,point.frame.position.z);
                diagnostic<<"{\"identity\":\""<<detail::rideName(source.role)<<"\",\"start\":"<<distance[interval.first]<<",\"end\":"<<distance[interval.last]
                    <<",\"sourceEntryMps\":"<<source.entrySpeed<<",\"sourceExitMps\":"<<source.exitSpeed<<",\"sourceHeightMeters\":"<<height<<",\"phases\":[";
                const double beginTime=d.simulation.completed&&source.airtime()?replayValueAt(d.simulation.frames,distance[interval.first],true):0;
                for(size_t point=0;point<source.phases.size();++point){if(point)diagnostic<<',';const auto& phase=source.phases[point];const double at=distance[interval.first]+phase.distance;
                    diagnostic<<"{\"distance\":"<<at<<",\"sourceSpeedMps\":"<<phase.speed<<'}';
                    if(d.simulation.completed){const double speed=replayValueAt(d.simulation.frames,at);
                        energyResidual=std::max(energyResidual,std::abs(speed-phase.speed));
                        if(source.airtime()){
                            if(!firstAirtime)airtime<<',';firstAirtime=false;
                            airtime<<"{\"chain\":"<<(id==7?1:0)<<",\"phase\":\""<<(point==0?"entry":point+1==source.phases.size()?"exit":point%2?"apex":"valley")
                                <<"\",\"hill\":"<<(point+1==source.phases.size()?int((source.phases.size()-3)/2):point?int((point-1)/2):0)
                                <<",\"distance\":"<<at<<",\"sourceSeconds\":"<<phase.pointTime<<",\"actualSeconds\":"<<replayValueAt(d.simulation.frames,at,true)-beginTime
                                <<",\"sourceSpeedMps\":"<<phase.speed<<",\"actualSpeedMps\":"<<speed<<",\"speedResidualMps\":"<<speed-phase.speed
                                <<",\"sourceCenterlineNormalG\":"<<phase.normalG<<",\"sourceCenterlineLateralG\":"<<phase.lateralG<<",\"ridersAtPhase\":[";
                            for(int seat=0;seat<3;++seat){if(seat)airtime<<',';const double center=at-seatDistanceOffset(req.train,seat);
                                const auto right=std::lower_bound(d.simulation.frames.begin(),d.simulation.frames.end(),center,[](const Frame& frame,double s){return frame.distance<s;});
                                const auto& left=*(right-1);const double u=(center-left.distance)/(right->distance-left.distance);
                                const auto& a=left.seats[seat];const auto& b=right->seats[seat];
                                airtime<<"{\"time\":"<<left.time+u*(right->time-left.time)<<",\"speedMps\":"<<left.speed+u*(right->speed-left.speed)
                                    <<",\"Gzyx\":["<<a.vertical+u*(b.vertical-a.vertical)<<','<<a.lateral+u*(b.lateral-a.lateral)<<','<<a.longitudinal+u*(b.longitudinal-a.longitudinal)<<"]}";
                            }airtime<<"]}";
                        }
                    }
                }diagnostic<<"]}";
            }
            diagnostic<<"],\"airtimePhaseIntent\":["<<airtime.str()<<"],\"connectingProfiles\":[";
            for(size_t occurrence=0;occurrence<build.route.order.size();++occurrence){if(occurrence)diagnostic<<',';const auto& link=build.geometry.links[occurrence];
                diagnostic<<"{\"start\":"<<distance[link.turn.first]<<",\"end\":"<<distance[detail::ridePassiveEnd(build.route,build.geometry,occurrence)]
                    <<",\"sourceSpeedMps\":"<<feedback.linkSpeed[build.route.order[occurrence]]<<",\"turnEnd\":"<<distance[link.turn.last]
                    <<",\"turnSpeedMps\":"<<feedback.turnSpeed[build.route.order[occurrence]]<<'}';}
            feedbackConverged=feedbackConverged&&energyResidual<=.5;
            diagnostic<<"],\"authoringEnergy\":{\"corrections\":"<<energyIterations<<",\"converged\":"<<(feedbackConverged?"true":"false")<<",\"maximumSpeedResidualMps\":";
            if(std::isfinite(energyResidual))diagnostic<<energyResidual;else diagnostic<<"null";
            diagnostic<<",\"toleranceMps\":0.5,\"iterations\":["<<energyHistory.str()<<"]}}";d.planningDiagnostics=diagnostic.str();
            d.station=buildStation(d.track,req.terrain,req.train,cancel);buildSupportLayout(d,cancel);
            d.inversionDimensions=measureInversionDimensions(d.track,cancel);
            if(progress)progress(i,"Checking measured targets and clearance");
            const auto geometryReport=validateGeometry(d.track,req.terrain,req.limits,req.train,d.supports,cancel);
            d.report.errors.insert(d.report.errors.end(),geometryReport.errors.begin(),geometryReport.errors.end());
            d.report.warnings.insert(d.report.warnings.end(),geometryReport.warnings.begin(),geometryReport.warnings.end());
            auto structures=validateDesignStructures(d,cancel);d.report.errors.insert(d.report.errors.end(),structures.errors.begin(),structures.errors.end());
            evaluateTargets(d);
            if(d.simulation.completed&&!feedbackConverged)d.report.fail("AUTHORING_ENERGY","Joined ride did not converge to its source and motor energy intent",0,energyResidual,.5);
            if(d.report.valid()&&d.simulation.completed&&d.simulation.report.valid()&&!d.simulation.cancelled){
                if(progress)progress(i,"Checking independent half-step simulation");
                verifyConvergence(d,cancel);
                if(d.simulation.cancelled)return d;
            }
            remember(&d,i);if(d.accepted()){
                const bool withinGoal=movingRideSeconds(d)<=180;
                return withHistory(std::move(d),withinGoal?"accepted-within-moving-duration-goal":"best-accepted-moving-duration-shortfall");
            }
            bool intensityOnly=d.simulation.completed&&d.simulation.report.valid()&&d.report.errors.size()==1&&d.report.errors.front().code=="INTENSITY_TARGET";
            if(intensityOnly)return withHistory(std::move(d),"best-physically-valid-intensity-shortfall");
            last=std::move(d);
            // Missing reference data cannot be repaired by searching other seeds.
            bool onlyMissing=last.simulation.completed&&last.simulation.report.valid()&&last.report.errors.size()==1&&last.report.errors.front().code=="REFERENCE_UNAVAILABLE";
            if(onlyMissing)return withHistory(std::move(last),"reference-unavailable");
        }catch(const std::exception& e){
            if(std::string(e.what()).rfind("SOURCE_FAMILY:",0)==0){remember(nullptr,i,"SOURCE_FAMILY");last.report.fail("SOURCE_FAMILY",e.what());return withHistory(std::move(last),"unsupported-source-family");}
            if(std::string(e.what())=="CANCELLED"||(cancel&&cancel())){last.simulation.cancelled=true;last.report.fail("CANCELLED","Generation cancelled");return last;}
            if(d.track.spans.empty()){remember(nullptr,i);last.report.fail("CANDIDATE_FAILURE",e.what());}
            else{
                // Preserve the last actual geometry/trace when later refinement
                // or structure placement fails. It remains explicitly rejected.
                d.report.fail("CANDIDATE_FAILURE",std::string("Refinement/validation failed; showing the last constructed circuit: ")+e.what());
                remember(&d,i);last=std::move(d);
            }
            if(progress)progress(i,std::string("Candidate rejected: ")+e.what());
        }
    }return withHistory(std::move(last),"last-constructed-rejection");
}
Design generate(const GenerationRequest& input,Cancel cancel,std::function<void(int,const std::string&)> progress){
    return detail::generateRide(input,false,cancel,std::move(progress));
}
}
