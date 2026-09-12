#pragma once
#include "coaster/fvd.hpp"
#include "coaster/force_envelope.hpp"
#include "coaster/layout_modules.hpp"
#include "circuit_geometry.hpp"
#include "boost_planning.hpp"
#include "terrain_motion.hpp"
#include "terrain_baseline.hpp"
#include "terrain_envelope.hpp"
#include "force_envelope_parameters.hpp"
#include "simulation_internal.hpp"
#include <map>
#include <optional>

namespace coaster::detail {
enum class RideRole {Departure,Loop,Immelmann,Climb,Dive,TallHill,Airtime,ClosingAirtime};
inline const char* rideName(RideRole role){
    switch(role){
        case RideRole::Departure:return "departure-launch";
        case RideRole::Loop:return "record-inversion";
        case RideRole::Immelmann:return "high-immelmann";
        case RideRole::Climb:return "cliff-ascent";
        case RideRole::Dive:return "cliff-dive";
        case RideRole::TallHill:return "record-hill";
        case RideRole::Airtime:return "fvd-airtime";
        case RideRole::ClosingAirtime:return "closing-airtime";
    }
    throw std::invalid_argument("Unknown ride source");
}
struct RideRandom {
    uint64_t state;
    uint64_t next(){uint64_t z=(state+=0x9e3779b97f4a7c15ull);z=(z^(z>>30))*0xbf58476d1ce4e5b9ull;z=(z^(z>>27))*0x94d049bb133111ebull;return z^(z>>31);}
    double range(double a,double b){return a+(b-a)*double(next()>>11)*0x1.0p-53;}
};
// Speed is the selected train's energy reference. Point-time and force retain
// the FVD geometry's authoring history, rather than impersonating train motion.
struct RidePhase {double distance,speed,pointTime,normalG,lateralG;};
struct RideSource {
    RideRole role;SourceGeometry geometry;
    double entrySpeed{},exitSpeed{};
    std::vector<RidePhase> phases;
    bool airtime() const{return role==RideRole::Airtime||role==RideRole::ClosingAirtime;}
    bool forceSource() const{return role==RideRole::Loop||role==RideRole::Immelmann||role==RideRole::TallHill||airtime();}
};
struct RideFeedback {
    std::vector<double> sourceEntry,turnSpeed,linkSpeed,linkMinimumSpeed,linkDuration,linkRise,motorEntry,passiveEnergyCorrection;
    std::vector<size_t> order;std::vector<double> headings;
    int site{-1};
    bool measured{};
};
inline double rideClimbAcceleration(const GenerationRequest& request){
    return std::min(request.limits.maxLongitudinalG*gravity-.1,3.5+gravity*2/std::sqrt(5.));
}
inline RideSource buildRideAirtime(const GenerationRequest& request,int chain,double speed,Cancel cancel={});
inline RidePhase ridePhase(const FvdSample& sample){
    const Vec3 force=sample.curvature*(sample.speed*sample.speed)+Vec3{0,0,gravity};
    return {sample.distance,sample.speed,sample.time,dot(force,sample.up)/gravity,dot(force,cross(sample.forward,sample.up))/gravity};
}
inline RideSource forceRideSource(RideRole role,const Track& track,const std::vector<FvdSample>& samples,
    std::optional<Element> element,std::vector<FvdSample> phases,const TrainConfig& train,Cancel cancel={}){
    phases.front().distance=0;phases.back().distance=track.length;
    std::vector<double> landmarks;for(const auto& phase:phases)landmarks.push_back(phase.distance);
    RideSource result{role,SourceGeometry(track,element,std::move(landmarks)),samples.front().speed,samples.back().speed,{}};
    for(const auto& phase:phases){auto reference=ridePhase(phase);
        const auto energy=estimatePassiveTransfer(track,train,0,phase.distance,result.entrySpeed,cancel,.25);
        if(!energy.reached)throw TerrainTransferInfeasible("Finite train cannot traverse its authored source");
        reference.speed=energy.speed;result.phases.push_back(reference);
    }
    return result;
}
inline bool sourceTrainForces(const RideSource& source,const GenerationRequest& request,Cancel cancel){
    Track track;track.closed=false;const auto& first=source.geometry.points.front().frame;const auto& last=source.geometry.points.back().frame;
    const double padding=(request.train.cars-1)*request.train.spacing*.5+source.entrySpeed;
    track.knots.push_back({first.position-first.tangent*padding,first.tangent,{},first.up,0,first.element});
    for(const auto& p:source.geometry.points){const auto& f=p.frame;track.knots.push_back({f.position,f.tangent,f.curvature,f.up,0,f.element});}
    track.knots.push_back({last.position+last.tangent*padding,last.tangent,{},last.up,0,last.element});track.rebuild();
    const auto approach=makePassiveTransfer(track,request.train,0,track.spans.front().length,cancel,.25);
    // Qualify the actual allowed source-inlet interval. Straight padding gives
    // every car the full event and initializes/settles the causal force filter.
    for(double offset:{-.5,0.,.5}){
        const auto inlet=approach.inverse(source.entrySpeed+offset);if(!inlet.reached)return false;
        const auto simulation=simulateSource(track,request.train,inlet.speed,1./960,cancel);
        if(simulation.cancelled)throw std::runtime_error("CANCELLED");
        if(!simulation.completed||!simulation.report.valid()||!validateSimulationForces(simulation,request.limits).valid())return false;
    }
    return true;
}
inline RideSource selectRideInversion(const GenerationRequest& request,double preferred,double maximum,
    const std::function<RideSource(double)>& build,Cancel cancel){
    auto initial=build(preferred);if(sourceTrainForces(initial,request,cancel))return initial;
    if(maximum<=preferred)throw TerrainTransferInfeasible("Inversion pulse has no remaining positive-load range");
    // A lower held peak can increase its duration violation. Solve the
    // constructive pulse branch above the preferred load: its upper boundary
    // is source geometry, and its lower accepted boundary is full-train force.
    // This occurs once before circuit placement, with no extra ride rebuilds.
    double geometry=preferred,ceiling=maximum;
    std::optional<RideSource> upper;
    try{upper=build(ceiling);geometry=ceiling;}catch(const TerrainTransferInfeasible&){}
    if(!upper)for(int iteration=0;iteration<16;++iteration){const double middle=(geometry+ceiling)*.5;
        try{auto candidate=build(middle);geometry=middle;upper=std::move(candidate);}
        catch(const TerrainTransferInfeasible&){ceiling=middle;}
    }
    if(!upper||!sourceTrainForces(*upper,request,cancel))throw TerrainTransferInfeasible("Inversion source has no train-qualified pulse in its physical family");
    double lower=preferred,accepted=geometry;
    for(int iteration=0;iteration<12;++iteration){const double middle=(lower+accepted)*.5;auto candidate=build(middle);
        if(sourceTrainForces(candidate,request,cancel))accepted=middle;else lower=middle;
    }
    // Interior of the feasible physical interval, rather than a source resting
    // on a force or geometry boundary after numerical root finding.
    auto selected=build((accepted+geometry)*.5);
    if(!sourceTrainForces(selected,request,cancel))throw TerrainTransferInfeasible("Inversion pulse left its connected feasible interval");
    return selected;
}
inline Track transferSource(const TerrainTransfer& profile,Element element){
    Track track;track.closed=false;const int count=std::max(4,int(std::ceil(profile.length/1.5)));
    for(int i=0;i<=count;++i){const double distance=profile.length*i/count;const auto jet=terrainVerticalJet(profile,distance);
        const Vec3 first{1,0,jet[1]},tangent=unit(first),second{0,0,jet[2]};
        const auto curvature=(second-tangent*dot(tangent,second))/dot(first,first);
        track.knots.push_back({{distance,0,jet[0]},tangent,curvature,unit(Vec3{-jet[1],0,1}),0,element});
    }
    track.rebuild();return track;
}
// Sources own their physical shapes and energy histories. Routing reads these
// same occurrences; its order does not change a loop into a geometric proxy.
inline std::vector<RideSource> buildRideSources(const GenerationRequest& request,RideFeedback& feedback,
    const Operation& departure,Cancel cancel={}){
    RideRandom random{request.seed^0x8d2f41b79a5c630eull};
    const double rolling=gravity*request.train.rollingResistance;
    const double drag=.5*request.train.airDensity*request.train.dragCdA/(request.train.cars*request.train.carMass);
    const double height=request.targets.height+random.range(8,30),loopHeight=random.range(68,78);
    const double immelmannHeight=std::max(84.,request.targets.inversionHeight+random.range(7,14));
    const int hand=(random.next()&1)?1:-1;
    if(height<220||height>280||request.targets.speed>90)
        throw std::runtime_error("SOURCE_FAMILY: tall hill requires 220..280 m and 75..90 m/s source intent");
    FvdTallHillRequest tallRequest;tallRequest.height=height;tallRequest.normalG=5;tallRequest.crestG=-1.45;tallRequest.crestPulseSeconds=2.2;
    tallRequest.rollingAcceleration=rolling;tallRequest.dragAccelerationCoefficient=drag;
    std::optional<FvdTallHillResult> selectedTall;
    for(double speed=std::max(75.,request.targets.speed);speed<=90;speed+=.25){
        tallRequest.speed=speed;auto source=designFvdTallHill(tallRequest,cancel);
        if(source.section.cancelled)throw std::runtime_error("CANCELLED");
        if(!source.section.report.valid()||!source.section.assessment.passed)continue;
        double minimum=INFINITY;for(const auto& sample:source.section.samples)minimum=std::min(minimum,sample.speed);
        if(minimum>=25){selectedTall=std::move(source);break;}
    }
    if(!selectedTall)throw std::runtime_error("SOURCE_FAMILY: no tall source retains the 25 m/s traversal intent");
    const auto& tall=*selectedTall;
    EnergyLoopModuleRequest loopRequest;loopRequest.height=loopHeight;loopRequest.apexSpeed=26;loopRequest.normalG=4.1;loopRequest.crossingOffset=18;
    loopRequest.rollingAcceleration=rolling;loopRequest.dragAccelerationCoefficient=drag;
    const auto loop=selectRideInversion(request,loopRequest.normalG,std::min(6.,request.limits.maxVerticalG),[&](double normal){
        loopRequest.normalG=normal;const auto source=buildEnergyLoopModule(loopRequest,cancel);
        if(source.cancelled)throw std::runtime_error("CANCELLED");
        if(!source.report.valid()||!source.assessment.passed)throw TerrainTransferInfeasible("Loop source failed its physical assessment");
        return forceRideSource(RideRole::Loop,source.track,source.samples,{},
            {source.samples.front(),source.apex,source.samples.back()},request.train,cancel);
    },cancel);
    const double scale=std::sqrt(immelmannHeight/88.);FvdImmelmannRequest immelmannRequest;
    immelmannRequest.entrySpeed=53*scale;immelmannRequest.height=95*scale*scale;immelmannRequest.exitHeight=10*scale*scale;
    immelmannRequest.rampSeconds=1.2*scale;immelmannRequest.hand=hand;immelmannRequest.rollingAcceleration=rolling;immelmannRequest.dragAccelerationCoefficient=drag;
    const auto immelmann=selectRideInversion(request,immelmannRequest.normalG,std::min(5.5,request.limits.maxVerticalG),[&](double normal){
        immelmannRequest.normalG=normal;const auto source=designFvdImmelmann(immelmannRequest,cancel);
        if(source.section.cancelled)throw std::runtime_error("CANCELLED");
        if(!source.section.report.valid()||!source.section.assessment.passed)throw TerrainTransferInfeasible("Immelmann source failed its physical assessment");
        return forceRideSource(RideRole::Immelmann,source.section.track,source.section.samples,Element::Inversion,
            {source.section.samples.front(),source.apex,source.rollExit,source.exit},request.train,cancel);
    },cancel);
    const double half=(request.train.cars-1)*request.train.spacing*.5;
    const double departureLength=half+30+plannedBoostLength(0,departure,request.train)+half;
    auto departureTrack=transferSource({0,0,departureLength},Element::Launch);
    for(auto& knot:departureTrack.knots)if(knot.position.x<20)knot.element=Element::Station;
    std::vector<RideSource> result;
    result.push_back({RideRole::Departure,SourceGeometry(departureTrack,{}, {20}),0,departure.targetSpeed,{}});
    result.push_back(loop);result.push_back(immelmann);
    // The selected record scale supplies the cliff rise on every terrain. Its
    // supporting datum remains a terrain variable; no canyon formula is copied.
    const double rise=request.targets.height,climbTarget=150/3.6;
    const double climbEntry=climbTarget;
    const double climbCapacity=rideClimbAcceleration(request);
    const TerrainMotion climbMotion{climbEntry,rolling,drag,0,2,climbTarget,climbCapacity};
    const double climbLength=minimumTerrainMotionLength(rise,2,climbMotion,cancel);
    const auto climbTrack=transferSource({0,rise,climbLength},Element::Launch);
    result.push_back({RideRole::Climb,SourceGeometry(climbTrack),climbEntry,climbTarget,{}});
    const double diveEntry=climbTarget;
    const TerrainMotion diveMotion{diveEntry,rolling,drag,-1,3.5,0,0};
    const double diveLength=minimumTerrainMotionLength(-rise,2,diveMotion,cancel);
    const TerrainTransfer dive{0,-rise,diveLength};const auto diveTrack=transferSource(dive,Element::Return);
    result.push_back({RideRole::Dive,SourceGeometry(diveTrack),diveEntry,assessTerrainMotion(dive,diveMotion,cancel).exitSpeedBound,{}});
    result.push_back(forceRideSource(RideRole::TallHill,tall.section.track,tall.section.samples,Element::Hill,
        {tall.section.samples.front(),tall.section.samples.back()},request.train,cancel));
    for(int chain=0;chain<2;++chain){
        result.push_back(buildRideAirtime(request,chain,chain?62:65,cancel));
    }
    for(const auto& source:result){feedback.sourceEntry.push_back(source.entrySpeed);feedback.turnSpeed.push_back(source.exitSpeed);feedback.linkSpeed.push_back(source.exitSpeed);}
    feedback.linkRise.resize(result.size());feedback.linkMinimumSpeed.resize(result.size());feedback.linkDuration.resize(result.size());feedback.motorEntry.resize(result.size());feedback.passiveEnergyCorrection.resize(result.size());
    return result;
}
inline RideSource buildRideAirtime(const GenerationRequest& request,int chain,double speed,Cancel cancel){
    RideRandom random{request.seed^uint64_t(chain+1)*0x8d2f41b79a5c630eull};
    const double bankHand=(random.next()&1)?1.:-1.;
    FvdAirtimeRequest airtime;airtime.speed=speed;airtime.hills.clear();
    airtime.rollingAcceleration=gravity*request.train.rollingResistance;
    airtime.dragAccelerationCoefficient=.5*request.train.airDensity*request.train.dragCdA/(request.train.cars*request.train.carMass);
    const double scale=speed/65;airtime.portRampSeconds=.6*scale;
    for(int i=0;i<(chain?3:1);++i){FvdAirtimeHill hill;
        hill.pushG=random.range(4.8,5)-.15*i;hill.crestG=(chain?std::array<double,3>{-.35,.15,-1}[i]:-1.25)+random.range(-.05,.05);
        hill.pushHoldSeconds=random.range(.20,.22)*scale+(chain&&i==2?airtime.portRampSeconds*.5:0);
        hill.crestRampSeconds=random.range(.80,.82)*scale;hill.crestPulseSeconds=1.6*scale;
        if(chain)hill.bankRadians=bankHand*(i==2?-1.:1.)*random.range(20,28)*pi/180;
        airtime.hills.push_back(hill);
    }
    const auto source=designFvdAirtime(airtime,cancel);
    if(source.section.cancelled)throw std::runtime_error("CANCELLED");
    if(!source.section.report.valid()||!source.section.assessment.passed)throw TerrainTransferInfeasible("Airtime source at "+std::to_string(speed)+" m/s failed its physical assessment"+
        (source.section.report.errors.empty()?std::string{}:": "+source.section.report.errors.front().message));
    std::vector<FvdSample> phases{source.section.samples.front()};for(const auto& hill:source.hills){phases.push_back(hill.apex);phases.push_back(hill.exit);}phases.push_back(source.section.samples.back());
    return forceRideSource(chain?RideRole::ClosingAirtime:RideRole::Airtime,source.section.track,source.section.samples,Element::Airtime,std::move(phases),request.train,cancel);
}

inline Operation rideMotor(const GenerationRequest& request,DriveKind kind,double target,double acceleration,double ramp=.5){
    Operation motor{0,1,kind,target,request.train.carMass*acceleration,request.train.carMass*acceleration*100,ramp};
    motor.exitFadeMeters=std::max(1.,target*ramp);return motor;
}
inline Operation sourceMotor(const GenerationRequest& request,const RideSource& source){
    const double acceleration=source.role==RideRole::Climb?rideClimbAcceleration(request):
        std::min(.8*gravity,request.limits.maxLongitudinalG*gravity-.1);
    return rideMotor(request,DriveKind::Boost,source.entrySpeed+.5,acceleration);
}
inline CircuitElement ridePort(const RideSource& source,double speed,double normalG){
    // Size for both delivered and required source energy. A motor still being
    // corrected cannot justify a tighter turn that fails once it meets intent.
    speed=std::max(speed,source.exitSpeed);
    const auto& end=source.geometry.points.back();const double bank=std::acos(1/normalG);
    const double ramp=speed*std::max(1.875*bank/(80*pi/180),std::sqrt((10*std::sqrt(3.)/3)*bank/(150*pi/180)));
    return {end.frame.position,std::atan2(end.frame.tangent.y,end.frame.tangent.x),end.distance,
        speed*speed/(gravity*std::tan(bank)),ramp,bank};
}
inline double rideCoast(const GenerationRequest& request,double speed,double length,double rise=0){
    const auto step=pathEnergyStep(length,rise,-gravity*request.train.rollingResistance,
        .5*request.train.airDensity*request.train.dragCdA/(request.train.cars*request.train.carMass));
    const double energy=speed*speed*step.attenuation+step.offsetSpeedSquared;
    if(energy<=0)throw TerrainTransferInfeasible("Connecting route exhausts its incoming energy");
    return std::sqrt(energy);
}
inline double rideInletHeight(const GenerationRequest& request,const RideSource& source){
    return source.role==RideRole::Dive?request.targets.height:0;
}
inline double rideLinkRise(const GenerationRequest& request,const RideSource& source,const RideSource& next){
    double rise=rideInletHeight(request,next)-rideInletHeight(request,source)-source.geometry.points.back().frame.position.z;
    if(source.role!=RideRole::Climb&&next.role!=RideRole::Departure&&!next.airtime()&&next.role!=RideRole::Dive)
        rise=std::max(rise,(source.exitSpeed*source.exitSpeed-next.entrySpeed*next.entrySpeed)/(2*gravity));
    return rise;
}
struct RideRoute {
    std::vector<RideSource> sources;
    std::vector<size_t> order;
    CircuitLayout layout;
};
// Only source energy is estimated here. Full-train feedback uses the complete
// circuit, including terrain and each motor's own inlet, within one budget.
inline RideRoute routeRide(const GenerationRequest& request,std::vector<RideSource> sources,RideFeedback& feedback,Cancel cancel={},bool requireCrossing=false){
    // Legal orders often request the same physical source. Author each exact
    // chain/speed once for this request; every order still owns its own copy.
    std::array<std::map<double,RideSource>,2> airtimeSources;
    const auto airtimeSource=[&](bool closing,double speed)->const RideSource&{
        auto& variants=airtimeSources[closing];
        const auto found=variants.find(speed);
        if(found!=variants.end())return found->second;
        auto source=buildRideAirtime(request,closing,speed,cancel);
        return variants.emplace(speed,std::move(source)).first->second;
    };
    RideRandom random{request.seed};const double normalG=random.range(3.8,4.2),hand=(random.next()&1)?1.:-1.;
    std::vector<double> initial(sources.size()),weights(sources.size());double total=0;
    for(auto& weight:weights){weight=random.range(.75,1.25);total+=weight;}
    for(size_t i=1;i<initial.size();++i)initial[i]=initial[i-1]+hand*2*pi*weights[i-1]/total;
    const double trainLength=(request.train.cars-1)*request.train.spacing;
    auto propose=[&](const std::vector<size_t>& order,const RideRoute* prior=nullptr){
        RideRoute route;route.sources=prior?prior->sources:sources;route.order=order;
        const auto& headings=prior?prior->layout.headings:feedback.headings.empty()?initial:feedback.headings;
        std::vector<CircuitElement> ports;
        for(size_t i=0;i<order.size();++i){const auto id=order[i];
            if(!prior&&feedback.order.empty()&&route.sources[id].airtime()){
                const auto& previous=route.sources[order[i-1]];const auto port=ports.back();
                const double angle=std::remainder(headings[i]-headings[i-1]-port.exitHeading,2*pi);
                const auto turn=circuitTurn(port,angle);
                const double speed=rideCoast(request,previous.exitSpeed,turn.length+trainLength,rideLinkRise(request,previous,route.sources[id]));
                route.sources[id]=airtimeSource(id==7,speed);
            }
            const double allowed=std::min(request.limits.maxVerticalG,historicalForceLimit(ForceAxis::Vertical,true,feedback.linkDuration[id]));
            auto port=ridePort(route.sources[id],feedback.order.empty()?route.sources[id].exitSpeed:feedback.turnSpeed[id],std::min(normalG,allowed));
            const auto next=order[(i+1)%order.size()];
            port.minimumSpeed=feedback.measured?feedback.linkMinimumSpeed[id]:std::min(route.sources[id].exitSpeed,next?route.sources[next].entrySpeed:route.sources[id].exitSpeed);
            ports.push_back(port);
        }
        std::vector<double> recovery(order.size()),rise(order.size());
        for(size_t i=0;i<order.size();++i){const auto id=order[i],next=order[(i+1)%order.size()];
            if(route.sources[id].role==RideRole::Climb)continue;
            rise[i]=feedback.measured?feedback.linkRise[id]:rideLinkRise(request,route.sources[id],route.sources[next]);
            const double speed=route.sources[id].exitSpeed;
            const TerrainMotion motion{speed,gravity*request.train.rollingResistance,.5*request.train.airDensity*request.train.dragCdA/(request.train.cars*request.train.carMass),-1,3.5,0,0};
            recovery[i]=minimumTerrainMotionLength(rise[i],2,motion,cancel);
        }
        const auto minimum=[&](size_t i,double turnLength)->CircuitLinkLengths{const auto id=order[i],next=order[(i+1)%order.size()];
            const auto& source=route.sources[id];const auto& destination=route.sources[next];
            const double entry=feedback.order.empty()?(prior?prior->layout.workEntrySpeeds[i]:
                rideCoast(request,source.exitSpeed,turnLength+recovery[i],rise[i])):feedback.motorEntry[next];
            if(next==0)return {std::pow(feedback.measured?feedback.motorEntry[0]:source.exitSpeed,2)/(2*6.)+2*trainLength+100,recovery[i],entry};
            if(destination.airtime()||destination.role==RideRole::Dive)return {0,std::max(trainLength,recovery[i])};
            return {plannedBoostLength(entry,sourceMotor(request,destination),request.train),recovery[i],entry};
        };
        std::vector<CircuitOccurrence> occurrences;
        if(requireCrossing)for(auto id:order)occurrences.push_back({route.sources[id].geometry,rideInletHeight(request,route.sources[id])});
        const auto admissible=[&](const CircuitLayout& layout){return !requireCrossing||circuitHasCrossing(occurrences,layout);};
        route.layout=!prior?solveCircuitLayout(ports,headings,minimum,cancel,requireCrossing?std::function<bool(const CircuitLayout&)>(admissible):nullptr):
            closeCircuit(ports,headings,minimum,nullptr,cancel);
        if(std::isfinite(route.layout.length)&&!admissible(route.layout))route.layout.length=INFINITY;
        return route;
    };
    RideRoute best;
    if(!feedback.order.empty())best=propose(feedback.order);
    else{
        std::array<size_t,4> movable{1,2,6,7};
        do{for(size_t split=1;split<movable.size();++split){
            std::vector<size_t> order{0};order.insert(order.end(),movable.begin(),movable.begin()+split);
            order.insert(order.end(),{3,4,5});order.insert(order.end(),movable.begin()+split,movable.end());
            try{auto trial=propose(order);
                if(!std::isfinite(trial.layout.length))continue;
                // Order selection consumes a complete source/connection plan.
                // Closure can change both airtime supply and motor inlet work;
                // an order outside a source's domain is never selected first
                // and repaired afterwards. This does not build complete rides.
                for(int iteration=0;;++iteration){
                    auto updated=trial;bool consistent=true;
                    for(size_t i=0;i<trial.order.size();++i){const auto id=trial.order[i],next=trial.order[(i+1)%trial.order.size()];
                        const auto& destination=trial.sources[next];
                        if(next==0||destination.role==RideRole::Dive)continue;
                        const double passive=trial.layout.turns[i].length+trial.layout.straights[i]-(destination.airtime()?0:trial.layout.workLengths[i]);
                        const double supplied=rideCoast(request,trial.sources[id].exitSpeed,passive,rideLinkRise(request,trial.sources[id],destination));
                        const double intended=destination.airtime()?destination.entrySpeed:trial.layout.workEntrySpeeds[i];
                        if(std::abs(supplied-intended)<=.5)continue;
                        consistent=false;
                        if(destination.airtime())updated.sources[next]=airtimeSource(destination.role==RideRole::ClosingAirtime,supplied);
                        else updated.layout.workEntrySpeeds[i]=supplied;
                    }
                    if(consistent)break;
                    if(iteration==7)throw TerrainTransferInfeasible("Initial source and closed-route energy intent did not agree");
                    trial=propose(updated.order,&updated);
                    if(!std::isfinite(trial.layout.length))throw TerrainTransferInfeasible("Energy-consistent source ports cannot close");
                }
                if(trial.layout.length<best.layout.length)best=std::move(trial);
            }catch(const TerrainTransferInfeasible&){/* An incompatible ordering is infeasible; the candidate still uses the same bounded set. */}
        }}while(std::next_permutation(movable.begin(),movable.end()));
    }
    if(!std::isfinite(best.layout.length))throw TerrainTransferInfeasible("No physical itinerary closes its source ports");
    if(feedback.order.empty())for(size_t i=0;i<best.order.size();++i){const auto id=best.order[i],next=best.order[(i+1)%best.order.size()];
        feedback.turnSpeed[id]=feedback.linkSpeed[id]=best.sources[id].exitSpeed;
        feedback.motorEntry[next]=best.layout.workEntrySpeeds[i];
        feedback.linkMinimumSpeed[id]=best.layout.minimumSpeeds[i];
        feedback.linkDuration[id]=best.layout.turns[i].length/best.layout.minimumSpeeds[i];
        feedback.linkRise[id]=rideLinkRise(request,best.sources[id],best.sources[next]);
    }
    feedback.order=best.order;feedback.headings=best.layout.headings;
    for(size_t i=0;i<best.sources.size();++i)if(best.sources[i].airtime())feedback.sourceEntry[i]=best.sources[i].entrySpeed;
    return best;
}

inline std::vector<double> circuitDistances(const Track& track){
    std::vector<double> distance;for(const auto& span:track.spans)distance.push_back(span.start);distance.push_back(track.length);return distance;
}
inline size_t ridePassiveEnd(const RideRoute& route,const CircuitGeometry& geometry,size_t occurrence){
    const auto& next=route.sources[route.order[(occurrence+1)%route.order.size()]];
    const auto& link=geometry.links[occurrence];
    return next.airtime()||next.role==RideRole::Dive?link.straight.last:link.workFirst;
}
inline CircuitGeometry placeRide(const GenerationRequest& request,const RideRoute& route,RideFeedback& feedback,Cancel cancel={}){
    std::vector<CircuitOccurrence> occurrences;
    for(auto id:route.order)occurrences.push_back({route.sources[id].geometry,rideInletHeight(request,route.sources[id])});
    auto local=composeCircuit(occurrences,route.layout,{},0,cancel);
    // Distances, authored vertical derivatives and connecting motion bounds
    // belong to the composed route. A site's horizontal rigid transform cannot
    // change them; only terrain floors, station datum and world crossings vary.
    const auto distance=circuitDistances(local.track);const size_t count=local.track.knots.size();
    const size_t stationBegin=size_t(std::lower_bound(distance.begin(),distance.end(),local.track.length-100)-distance.begin());
    const double half=(request.train.cars-1)*request.train.spacing*.5;
    std::vector<double> height(count);std::vector<BaselineJet> authoredJets(count);
    for(size_t i=0;i<count;++i){const auto& knot=local.track.knots[i];
        height[i]=knot.position.z;
        const auto kinematics=sampleSpanKinematics(local.track,std::min(i,local.track.spans.size()-1),i+1==count?1:0);
        authoredJets[i]={height[i],knot.tangent.z,knot.curvature.z,kinematics.curvatureS.z};
    }
    std::vector<BaselineTransport> transports;
    std::vector<BaselineMotionBounds> motion;
    for(size_t i=0;i<route.order.size();++i){if(route.sources[route.order[i]].role==RideRole::Climb)continue;
        const auto next=route.order[(i+1)%route.order.size()];const auto& destination=route.sources[next];
        const auto first=local.sources[i].last;auto last=i+1<route.order.size()?local.sources[i+1].first:stationBegin;
        if(next&&!destination.airtime()&&destination.role!=RideRole::Dive)last=local.links[i].workFirst;
        // Selected passive source intent is fixed before placement. This
        // shared solve supplies its inlet energy; a first trace does not
        // replace the source and change the entire route's footprint.
        double minimum=0,maximum=INFINITY;
        if(destination.airtime())
            minimum=maximum=std::sqrt(destination.entrySpeed*destination.entrySpeed+feedback.passiveEnergyCorrection[next]);
        // Recover surplus incoming kinetic energy as height before the
        // motor. The same solve owns that height and its reachable inlet;
        // a later rebuild does not switch this rail into a trim brake.
        if((destination.forceSource()&&!destination.airtime())||destination.role==RideRole::Climb)maximum=destination.entrySpeed;
        transports.push_back({first,next?last:local.links[i].workFirst,route.sources[route.order[i]].exitSpeed,minimum,maximum,
            local.links[i].turn.last,route.sources[route.order[i]].exitSpeed});
        const auto id=route.order[i];const double grade=2.;
        const auto& turn=route.layout.turns[i];double planarRate=0;
        if(turn.length>0)for(int part=0;part<64;++part){const double a=double(part)/64,b=double(part+1)/64,u=std::clamp(.5,a,b);
            const double bank=turn.peakBank*smooth(b),qPrime=30*u*u*(1-u)*(1-u);
            planarRate=std::max(planarRate,std::tan(bank)/std::cos(bank)*turn.peakBank*qPrime*feedback.turnSpeed[id]/turn.ramp);}
        // A connecting profile can follow sustained nonpositive load.
        // Its authoring rate must also allow the existing 0-to-2 g
        // transition duration, before allocating the planar contribution.
        const double onset=std::min(request.limits.maxJerkGps,2/zeroToTwoMinimumSeconds);
        if(planarRate>=onset)throw TerrainTransferInfeasible("Authored turn leaves no vertical profile onset budget");
        const double minimumSpeed=route.layout.minimumSpeeds[i];
        const double turnDuration=std::max(feedback.linkDuration[id],(2*turn.ramp+turn.radius*std::abs(turn.angle))/minimumSpeed);
        const auto turnEnd=local.links[i].turn.last;
        const double recoveryDuration=(distance[last]-distance[turnEnd])/minimumSpeed;
        // One resultant load budget owns turning and vertical recovery.
        // Geometry coefficients are sampled authoring estimates; the QP
        // bounds its complete C3 derivatives continuously, and canonical
        // finite-train simulation remains the independent acceptance gate.
        for(size_t span=first;span<last;++span){double horizontalMin=1,horizontalMax=0,metricRate=0,planar=0,seatRotation=0;
            // A later descent's peak speed does not resize the earlier
            // planar turn. Each phase uses its own finite-train exposure.
            const double speed=span<turnEnd?feedback.turnSpeed[id]:feedback.linkSpeed[id];
            const double verticalRate=onset-(span<turnEnd?planarRate:0);
            const double duration=span<turnEnd?turnDuration:recoveryDuration;
            const double positive=std::min(request.limits.maxVerticalG,historicalForceLimit(ForceAxis::Vertical,true,duration));
            for(double parameter:{0.,.5,1.}){const auto k=sampleSpanKinematics(local.track,span,parameter);const auto t=k.sample.tangent,c=k.sample.curvature;
                const double h=std::hypot(t.x,t.y);horizontalMin=std::min(horizontalMin,h);horizontalMax=std::max(horizontalMax,h);
                metricRate=std::max(metricRate,std::abs((t.x*c.x+t.y*c.y)/(h*h)));
                planar=std::max(planar,speed*speed*std::abs(cross(t,c).z)/(gravity*h*h*h));
                seatRotation=std::max(seatRotation,request.train.seatHeight*speed*speed*dot(k.upS,k.upS)/gravity);
            }
            if(planar>=positive)throw TerrainTransferInfeasible("Planar curve exhausts the complete connecting-profile force budget");
            const double negative=std::min(-request.limits.minVerticalG,historicalForceLimit(ForceAxis::Vertical,false,duration))-seatRotation;
            if(negative<=0)throw TerrainTransferInfeasible("Frame rotation exhausts the negative rider-force budget");
            const double verticalCapacity=std::sqrt(positive*positive-planar*planar);
            const double upper=(verticalCapacity-1)*gravity/(speed*speed);
            // The positive resultant budget bounds both signs of its
            // vertical component. Negative rider load additionally bounds
            // that component; horizontal turning is not negative load.
            const double lower=(-std::min(negative,verticalCapacity)-1/std::hypot(1.,grade/horizontalMin))*gravity/(speed*speed);
            // q_horizontal=(z_ss-(h_s/h)*z_s)/h^2. Reserving the full
            // bounded metric term keeps these rows in authored arc units.
            motion.push_back({span,span+1,grade,lower*horizontalMin*horizontalMin+metricRate*grade,
                upper*std::pow(upper>=0?horizontalMin:horizontalMax,2)-metricRate*grade,
                verticalRate*gravity/(speed*speed*speed)});
        }
    }
    struct Site {Vec3 origin;double heading,score;int id;};std::vector<Site> sites;
    const auto& terrain=request.terrain;
    // Query the landscape in its own transformed frame. No terrain-family
    // centreline, rim coordinates or particular seed appears in placement.
    for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)for(int orientation=0;orientation<4;++orientation){
        const int id=int(sites.size());const Vec3 origin=Vec3{terrain.offsetX,terrain.offsetY,0}+sourceYaw({x*600.*terrain.horizontalScale,y*600.*terrain.horizontalScale,0},terrain.headingRadians);
        const double heading=terrain.headingRadians+orientation*pi/2;double score=0;
        for(size_t i=0;i<local.track.knots.size();i+=16){const auto& knot=local.track.knots[i];const auto p=origin+sourceYaw(knot.position,heading);
            score+=std::abs(terrain.height(p.x,p.y)+request.limits.minClearance+4-p.z);}
        sites.push_back({origin,heading,score,id});
    }
    // Continue a feasible placement while the coupled inputs settle. Its ID
    // is a preference, never a constraint on changed geometry or motion.
    std::stable_sort(sites.begin(),sites.end(),[&](const Site& a,const Site& b){
        return std::pair{a.id!=feedback.site,a.score}<std::pair{b.id!=feedback.site,b.score};
    });
    std::string failure;
    for(const auto& site:sites){
        if(cancel&&cancel())throw std::runtime_error("CANCELLED");
        auto result=local;auto& track=result.track;
        for(auto& knot:track.knots){knot.position=site.origin+sourceYaw(knot.position,site.heading);
            knot.tangent=sourceYaw(knot.tangent,site.heading);knot.curvature=sourceYaw(knot.curvature,site.heading);knot.up=sourceYaw(knot.up,site.heading);}
        std::vector<double> floor(count),target(count);
        for(size_t i=0;i<count;++i){const auto& knot=track.knots[i];const auto up=rotate(knot.up,knot.tangent,knot.bank);
            const TrackSample frame{knot.position,knot.tangent,knot.curvature,up,cross(knot.tangent,up),knot.element};
            floor[i]=terrainEnvelopeDatum(terrain,request.limits,frame,knot.element==Element::Turn);
            target[i]=terrain.height(knot.position.x,knot.position.y)+request.limits.minClearance+4;
        }
        for(size_t occurrence=0;occurrence<route.order.size();++occurrence){const auto role=route.sources[route.order[occurrence]].role;
            if(role!=RideRole::TallHill&&role!=RideRole::Immelmann)continue;
            const auto interval=result.sources[occurrence];size_t apex=interval.first;double maximum=-INFINITY;
            for(size_t i=interval.first;i<=interval.last;++i)if((role==RideRole::TallHill||track.knots[i].up.z<-.5)&&height[i]>maximum){maximum=height[i];apex=i;}
            const auto p=track.knots[apex].position;floor[apex]=std::max(floor[apex],terrain.height(p.x,p.y)+(role==RideRole::TallHill?request.targets.height:request.targets.inversionHeight)+4-height[apex]);
        }
        double stationGround=INFINITY;
        for(double along=-half-30;along<=half+50;along+=2)for(double side:{-8.,8.}){
            const auto p=site.origin+sourceYaw({along,side,0},site.heading);stationGround=std::min(stationGround,terrain.height(p.x,p.y));}
        std::vector<BaselineAnchor> anchors;
        for(size_t i=0;i<route.order.size();++i){const auto id=route.order[i];auto interval=result.sources[i];
            // Level work rail and its following source have the same rigid
            // height owner. Separate ports plus equality rows would duplicate
            // this datum and make an otherwise simple constraint dependent.
            if(i&&!route.sources[id].airtime()&&route.sources[id].role!=RideRole::Dive)
                interval.first=result.links[i-1].workFirst;
            if(route.sources[id].role==RideRole::Climb){interval.last=result.sources[i+1].last;++i;}
            const bool station=id==0;anchors.push_back({interval.first,interval.last,*std::max_element(floor.begin()+interval.first,floor.begin()+interval.last+1),false,station?stationGround+16:INFINITY,station});
        }
        anchors.push_back({stationBegin,count-1,*std::max_element(floor.begin()+stationBegin,floor.end()),false,stationGround+16,true});
        std::vector<int> owner(count,-1);for(size_t a=0;a<anchors.size();++a)for(size_t i=anchors[a].first;i<=anchors[a].last;++i)owner[i]=int(a);
        std::vector<BaselineCrossing> crossings;
        for(size_t i=0;i+1<count;++i){if((i&63)==0&&cancel&&cancel())throw std::runtime_error("CANCELLED");
            const auto a=track.knots[i].position,b=track.knots[i+1].position-a;
            for(size_t j=i+2;j+1<count;++j){const double arc=distance[j]-distance[i];
                if(std::min(arc,track.length-arc)<=std::max(4*half,18.))continue;
                if(owner[i]>=0&&owner[i]==owner[i+1]&&owner[i]==owner[j]&&owner[i]==owner[j+1])continue;
                const auto c=track.knots[j].position,e=track.knots[j+1].position-c;const double determinant=cross(b,e).z;
                if(std::abs(determinant)<1e-8)continue;const double u=cross(c-a,e).z/determinant,v=cross(c-a,b).z/determinant;
                if(u<0||u>1||v<0||v>1)continue;
                crossings.push_back({i,j,u,v,9,true});
            }
        }
        try{
            const auto solved=solveTerrainBaseline(distance,floor,target,height,anchors,motion,crossings,cancel,authoredJets,transports,request.train);
            for(const auto& source:solved.sources){const auto first=size_t(std::lower_bound(distance.begin(),distance.end(),source.begin)-distance.begin());
                for(size_t i=first;i<count&&distance[i]<=source.end;++i)track.knots[i].position.z+=source.height;}
            for(const auto& gap:solved.gaps){const auto first=size_t(std::upper_bound(distance.begin(),distance.end(),gap.begin)-distance.begin());
                for(size_t i=first;i<count&&distance[i]<gap.end;++i)track.knots[i]=addBaselineJet(track.knots[i],gap.profile.jet(distance[i]-gap.begin));}
            track.knots.back()=track.knots.front();track.rebuild();feedback.site=site.id;return result;
        }catch(const TerrainTransferInfeasible& e){failure=e.what();}
    }
    throw TerrainTransferInfeasible("No site admits the shared source, station and crossing heights: "+failure);
}
struct RideBuild {
    Design design;RideRoute route;CircuitGeometry geometry;
    std::vector<double> motorInlet;
};
inline RideBuild buildRide(const GenerationRequest& request,int candidate,Operation departure,const RideRoute& route,RideFeedback& feedback,Cancel cancel={}){
    RideBuild result;result.route=route;
    // Hold source positions after footprint sizing, but make the actual work
    // boundary follow its own inlet. Composition, terrain transport and hardware
    // use this same boundary; no discarded capacity estimate becomes level rail.
    for(size_t i=0;i<result.route.order.size();++i){const auto next=result.route.order[(i+1)%result.route.order.size()];
        const auto& destination=result.route.sources[next];
        if(next==0||destination.airtime()||destination.role==RideRole::Dive)continue;
        const double length=plannedBoostLength(feedback.motorEntry[next],sourceMotor(request,destination),request.train);
        if(length>result.route.layout.straights[i]+1e-7)throw TerrainTransferInfeasible("Actual motor work cannot fit the connecting route");
        result.route.layout.workLengths[i]=length;result.route.layout.workEntrySpeeds[i]=feedback.motorEntry[next];
    }
    result.geometry=placeRide(request,result.route,feedback,cancel);
    auto& design=result.design;design.request=request;design.candidate=candidate;design.track=result.geometry.track;design.topology="source-itinerary";
    const auto distance=circuitDistances(design.track);const double half=(request.train.cars-1)*request.train.spacing*.5;
    result.motorInlet.resize(result.route.sources.size(),-1);
    departure.start=0;departure.end=distance[result.geometry.sources.front().last]-half;design.operations.push_back(departure);
    for(size_t i=0;i<result.route.order.size();++i){const auto id=result.route.order[i],next=result.route.order[(i+1)%result.route.order.size()];
        const auto& source=result.route.sources[id];const auto& destination=result.route.sources[next];const auto& link=result.geometry.links[i];
        const double begin=distance[link.workFirst],end=distance[link.straight.last];
        if(next==0){
            result.motorInlet[0]=begin;
            const double start=begin+2*half+2,room=design.track.length-start-2*half-65;
            if(room<=0)throw TerrainTransferInfeasible("Return lacks full-train stopping room");
            const double deceleration=std::max(6.,source.exitSpeed*source.exitSpeed/(2*room));
            double downhill=0;for(double s=start;s<design.track.length;s+=2)downhill=std::max(downhill,-design.track.sample(s).tangent.z);
            auto motor=rideMotor(request,DriveKind::Station,0,std::min(request.limits.maxLongitudinalG*gravity-.1,deceleration+gravity*downhill+1));
            motor.start=start;motor.end=80;motor.stopDeceleration=deceleration;motor.stopOffset=.2;design.operations.push_back(motor);continue;
        }
        if(destination.airtime()||destination.role==RideRole::Dive)continue;
        const bool climb=destination.role==RideRole::Climb;
        auto motor=sourceMotor(request,destination);
        const double inlet=begin;
        const auto coast=estimatePassiveTransfer(design.track,request.train,distance[result.geometry.sources[i].last],inlet,source.exitSpeed,cancel);
        if(!coast.reached)throw TerrainTransferInfeasible("Source cannot reach its following motor");
        motor.start=inlet+half;motor.end=(climb?distance[result.geometry.sources[i+1].last]:end)-half;
        if(motor.end<=motor.start)throw TerrainTransferInfeasible("Motor lacks complete-train hardware room");
        result.motorInlet[next]=inlet;design.operations.push_back(motor);
    }
    coalesceDriveProfiles(design.operations);return result;
}
}
